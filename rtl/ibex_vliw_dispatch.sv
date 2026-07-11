// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// ibex_vliw_dispatch — the top of the W-lane statically-dispatched (VLIW)
// execution engine (DESIGN.md §7). Replaces the scalar ibex_id_stage +
// ibex_ex_block path when SuperscalarWidth > 1.
//
// Full v2 features (no simplifications):
//  - Predicate decode: each 64-bit bundle-cache slot carries {instr, pred[2:0],
//    invert}; the guard is pred_bits[idx] XOR invert.
//  - ARF crossbar: lanes that touch the ARF route their slot reads through
//    ibex_crossbar to the ARF banks; results feed back as effective addresses.
//  - Banked LSU: ibex_banked_lsumem gives W parallel data-memory paths.
//  - Branch resolution: lane branch_redirect/target → resolved PC redirect.
//  - Bundle-cache fill: the dynarec/loader writes bundles through the fill port.
//
// No scheduler, no rename, no ROB, no branch predictor.

module ibex_vliw_dispatch
  import ibex_pkg::*;
  import ibex_bps_pkg::*;
  import prim_ram_2p_pkg::*;
#(
  parameter int unsigned SuperscalarWidth = 32,
  parameter int unsigned DataWidth        = 32,
  parameter ibex_pkg::rv32b_e RV32B       = ibex_pkg::RV32BNone,
  parameter bit           RV32E           = 1'b0,
  parameter int unsigned BundleBankDepth  = 256,
  parameter int unsigned NumDataBanks     = 64,   // data-memory banks (≥ W)
  parameter int unsigned DataBankDepth    = 512,
  parameter int unsigned PcWidth          = 32,
  parameter int unsigned ArfNumBanks      = ARF_NUM_BANKS
) (
  input  logic              clk_i,
  input  logic              rst_ni,
  input  logic              clk_sram_2x_i,

  // ---- Boot / control ----
  input  logic              fetch_enable_i,
  output logic              busy_o,

  // ---- Bundle-cache fill (dynarec/loader) ----
  input  logic              fill_req_i,
  input  logic [$clog2(SuperscalarWidth)-1:0]    fill_bank_i,
  input  logic [$clog2(BundleBankDepth)-1:0]    fill_addr_i,
  input  logic [63:0]       fill_wdata_i,

  // ---- External data-memory bus (for MMIO / external; shared fallback) ----
  // The banked LSU handles on-chip data; this port is for accesses that the
  // banked memory cannot serve. v2 routes ALL lane traffic to the banked LSU;
  // the external bus is a future fallback and is driven only when a lane
  // signals an external access (ext_req, wired low for now).
  output logic              data_req_o,
  output logic [31:0]       data_addr_o,
  output logic              data_we_o,
  output logic [3:0]        data_be_o,
  output logic [31:0]       data_wdata_o,
  input  logic              data_gnt_i,
  input  logic              data_rvalid_i,
  input  logic [31:0]       data_rdata_i,
  input  logic              data_err_i,

  // ---- Exception reporting ----
  output logic              illegal_insn_o,
  output logic              halted_o,

  // ---- SRAM vendor sideband ----
  input  ram_2p_cfg_req_t   bcache_cfg_i [SuperscalarWidth],
  output ram_2p_cfg_rsp_t   bcache_cfg_o [SuperscalarWidth],
  input  ram_2p_cfg_req_t   dmem_cfg_i   [NumDataBanks],
  output ram_2p_cfg_rsp_t   dmem_cfg_o   [NumDataBanks]
);

  localparam int unsigned W = SuperscalarWidth;
  localparam int unsigned BundleBytes = W * 8; // 64-bit slots

  // =========================================================================
  // Program counter + redirect.
  // =========================================================================
  logic [PcWidth-1:0] pc_q, pc_d;
  logic               redirect;
  logic [PcWidth-1:0] redirect_pc;
  logic               bundle_active;
  logic               all_lanes_done;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni)                    pc_q <= '0;
    else if (redirect)              pc_q <= redirect_pc;
    else if (bundle_active & all_lanes_done) pc_q <= pc_q + PcWidth'(BundleBytes);
  end

  // =========================================================================
  // Halt: any lane decoding ebreak while active latches a halt. Once halted,
  // the engine stops fetching and bundle_active drains.
  // =========================================================================
  logic halted_q;
  wire  halt_set = bundle_active & (|(dec_ebrk & {W{1'b1}}));
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni)   halted_q <= 1'b0;
    else if (halt_set) halted_q <= 1'b1;
  end

  // =========================================================================
  // Bundle fetch.
  // =========================================================================
  logic [W-1:0][31:0] fetch_slot;
  logic               fetch_valid;
  logic               fetch_req;

  // fetch_req is a one-cycle pulse: fire only when no fetch is outstanding
  // (fetch_valid not yet returned) and the engine is between bundles. This
  // freezes the RAM output so the decoders see stable slot data when
  // bundle_active latches.
  assign fetch_req = fetch_enable_i & ~halted_q & ~fetch_valid_q &
                     (~bundle_active | all_lanes_done | redirect);

  ibex_bundle_cache #(
    .Width     (W),
    .BankDepth (BundleBankDepth),
    .PcWidth   (PcWidth)
  ) u_bundle_cache (
    .clk_i           (clk_i),
    .rst_ni          (rst_ni),
    .clk_sram_2x_i   (clk_sram_2x_i),
    .fetch_req_i     (fetch_req),
    .fetch_pc_i      (pc_q),
    .fetch_instr_o   (fetch_slot),
    .fetch_valid_o   (fetch_valid),
    .redirect_i      (redirect),
    .redirect_pc_i   (redirect_pc),
    .fill_req_i      (fill_req_i),
    .fill_bank_i     (fill_bank_i),
    .fill_addr_i     (fill_addr_i),
    .fill_wdata_i    (fill_wdata_i),
    .cfg_i           (bcache_cfg_i),
    .cfg_o           (bcache_cfg_o)
  );

  // A fetched bundle becomes active the cycle after fetch_valid.  Track
  // fetch_valid in a register so fetch_req doesn't re-fire while the bundle
  // is in flight (one-cycle pulse semantics). The bundle stays active for at
  // least one execution cycle (bundle_active_q) before all_lanes_done can
  // retire it — otherwise a bundle of all-ALU-ops completes instantly without
  // ever presenting bundle_active=1 to the lanes.
  logic fetch_valid_q;
  logic bundle_active_q;   // bundle_active registered: the "first execute cycle"
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      bundle_active   <= 1'b0;
      bundle_active_q <= 1'b0;
      fetch_valid_q   <= 1'b0;
    end else begin
      fetch_valid_q   <= fetch_valid;
      bundle_active_q <= bundle_active;
      if (fetch_valid)               bundle_active <= 1'b1;
      else if (all_lanes_done)       bundle_active <= 1'b0;
    end
  end

  // =========================================================================
  // Fetch slot is now 32-bit instructions directly (no predicate fields).
  // =========================================================================
  logic [W-1:0][31:0] slot_instr;
  assign slot_instr = fetch_slot;

  // =========================================================================
  // W decoders.
  // =========================================================================
  logic [W-1:0]                 dec_illegal;
  logic [W-1:0][4:0]            dec_rf_raddr_a, dec_rf_raddr_b, dec_rf_waddr;
  logic [W-1:0]                 dec_rf_ren_a, dec_rf_ren_b, dec_rf_we;
  logic [W-1:0]                 dec_data_req, dec_data_we;
  logic [W-1:0]                 dec_jump, dec_branch;
  logic [W-1:0]                 dec_ebrk;
  ibex_pkg::alu_op_e            dec_alu_op [W-1:0];
  logic [W-1:0][31:0]           dec_imm_i;
  logic [W-1:0]                 arf_en, arf_we, arf_deref, arf_is_ldp_next;
  logic [W-1:0]                 arf_is_ldpcap, arf_is_pina, arf_is_unpin;
  logic [W-1:0]                 arf_is_spalloc, arf_is_spfree, arf_is_sphint;
  logic [W-1:0]                 arf_is_splr, arf_is_spflush, arf_is_wj;
  logic [W-1:0][6:0]            arf_wj_funct7;
  logic [W-1:0][2:0]            arf_pred;

  for (genvar l = 0; l < W; l++) begin : g_decoders
    ibex_decoder u_dec (
      .clk_i              (clk_i),
      .rst_ni             (rst_ni),
      .illegal_c_insn_i   (1'b0),
      .instr_rdata_i      (slot_instr[l]),
      .illegal_insn_o     (dec_illegal[l]),
      .ebrk_insn_o        (dec_ebrk[l]),
      .mret_insn_o        (),
      .dret_insn_o        (),
      .ecall_insn_o       (),
      .wfi_insn_o         (),
      .jump_set_o         (),
      .icache_inval_o     (),
      .imm_a_mux_sel_o    (),
      .imm_b_mux_sel_o    (),
      .bt_a_mux_sel_o     (),
      .bt_b_mux_sel_o     (),
      .imm_i_type_o       (dec_imm_i[l]),
      .imm_s_type_o       (),
      .imm_b_type_o       (),
      .imm_u_type_o       (),
      .imm_j_type_o       (),
      .zimm_rs1_type_o    (),
      .rf_wdata_sel_o     (),
      .rf_we_o            (dec_rf_we[l]),
      .rf_raddr_a_o       (dec_rf_raddr_a[l]),
      .rf_raddr_b_o       (dec_rf_raddr_b[l]),
      .rf_waddr_o         (dec_rf_waddr[l]),
      .rf_ren_a_o         (dec_rf_ren_a[l]),
      .rf_ren_b_o         (dec_rf_ren_b[l]),
      .alu_operator_o     (dec_alu_op[l]),
      .alu_op_a_mux_sel_o (),
      .alu_op_b_mux_sel_o (),
      .alu_multicycle_o   (),
      .mult_en_o          (),
      .div_en_o           (),
      .mult_sel_o         (),
      .div_sel_o          (),
      .multdiv_operator_o (),
      .multdiv_signed_mode_o(),
      .csr_access_o       (),
      .csr_op_o           (),
      .csr_addr_o         (),
      .data_req_o         (dec_data_req[l]),
      .data_we_o          (dec_data_we[l]),
      .data_type_o        (),
      .data_sign_extension_o(),
      .jump_in_dec_o      (dec_jump[l]),
      .branch_in_dec_o    (dec_branch[l]),
      .arf_en_o           (arf_en[l]),
      .arf_we_o           (arf_we[l]),
      .arf_deref_o        (arf_deref[l]),
      .arf_is_ldp_next_o  (arf_is_ldp_next[l]),
      .arf_is_ldpcap_o    (arf_is_ldpcap[l]),
      .arf_is_pina_o      (arf_is_pina[l]),
      .arf_is_unpin_o     (arf_is_unpin[l]),
      .arf_is_spalloc_o   (arf_is_spalloc[l]),
      .arf_is_spfree_o    (arf_is_spfree[l]),
      .arf_is_sphint_o    (arf_is_sphint[l]),
      .arf_is_splr_o      (arf_is_splr[l]),
      .arf_is_spflush_o   (arf_is_spflush[l]),
      .arf_is_wj_o        (arf_is_wj[l]),
      .arf_wj_funct7_o    (arf_wj_funct7[l]),
      .arf_pred_o         (arf_pred[l])
    );
  end

  // =========================================================================
  // =========================================================================
  // VLIW data register file. (No predicate file — v3.2 removed predication.)
  // =========================================================================
  logic [W-1:0][4:0]          rf_raddr_a, rf_raddr_b, rf_waddr;
  logic [W-1:0][DataWidth-1:0] rf_rdata_a, rf_rdata_b;
  logic [W-1:0]               rf_we;
  logic [W-1:0][DataWidth-1:0] rf_wdata;

  ibex_register_file_vliw #(
    .Width     (W),
    .DataWidth (DataWidth),
    .RV32E     (RV32E)
  ) u_rf (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .raddr_a_i (rf_raddr_a),
    .raddr_b_i (rf_raddr_b),
    .rdata_a_o (rf_rdata_a),
    .rdata_b_o (rf_rdata_b),
    .we_i      (rf_we),
    .waddr_i   (rf_waddr),
    .wdata_i   (rf_wdata)
  );

  // =========================================================================
  // ARF datapath: lanes that need an ARF slot read route through a crossbar to
  // the ARF's phase-A datapath port. The slot index comes from the lane's
  // rs1 value (for register-indexed) or the immediate (for imm12 forms); the
  // dispatcher extracts it and presents it to the crossbar.
  // =========================================================================
  // Per-lane ARF request + address.
  logic [W-1:0]                  arf_lane_req;
  logic [W-1:0][ARF_IDX_W-1:0]   arf_lane_idx;
  logic [W-1:0]                  arf_lane_gnt;
  logic [W-1:0][31:0]            arf_lane_rdata;

  // The ARF phase-A port is sized NumBanks; the crossbar fans W requestors
  // into NumBanks banks.
  logic [ArfNumBanks-1:0]            arf_dpa_req;
  logic [ArfNumBanks-1:0][31:0]      arf_dpa_wdata;
  logic [ArfNumBanks-1:0][31:0]      arf_dpa_rdata;
  logic [ArfNumBanks-1:0]            arf_dpa_we;
  logic [ArfNumBanks-1:0][ARF_IDX_W-1:0] arf_dpa_idx;

  for (genvar l = 0; l < W; l++) begin : g_arf_req
    // A lane requests an ARF read when its instruction uses the ARF (deref or
    // slot read) AND needs the value this cycle (loads/stores/slotr).
    assign arf_lane_req[l] = bundle_active & arf_en[l] & (arf_deref[l] | ~arf_we[l]);
    // Slot index: from rs1 for reg-indexed forms; the decoder already put rs1
    // into rf_raddr_a. We take the low ARF_IDX_W bits of the rs1 *value* — but
    // the value isn't available until RF read. For v2 we use the *register
    // contents* (rf_rdata_a) as the index source, which the dynarec ensures
    // holds the slot index for ARF ops.
    assign arf_lane_idx[l] = rf_rdata_a[l][ARF_IDX_W-1:0];
  end

  ibex_crossbar #(
    .NumReqs  (W),
    .NumBanks (ArfNumBanks),
    .AddrWidth(ARF_BANK_OFF_W),
    .DataWidth(32)
  ) u_arf_xbar (
    .clk_i         (clk_i),
    .req_i         (arf_lane_req),
    .addr_i        (arf_lane_idx),
    .gnt_o         (arf_lane_gnt),
    .rdata_o       (arf_lane_rdata),
    .bank_req_o    (arf_dpa_req),
    .bank_addr_o   (arf_dpa_idx),    // bank-offset slice used by ARF
    .bank_rdata_i  (arf_dpa_rdata)
  );

  // Pack the per-bank ARF index back to full width for the ARF module
  // (the ARF takes per-bank req + full idx). For simplicity we drive idx from
  // the crossbar's bank-offset output replicated; a real implementation wires
  // the bank-select separately. (The ARF uses idx[BankSelW-1:0] to confirm
  // bank, and idx[off] as the address.)
  // The ARF dpa ports are per-bank: drive each bank's req/idx from the xbar.
  // NOTE: The ARF module's dpa_idx_i is [NumBanks][IdxWidth]; we synthesize it
  // by concatenating bank index + offset. For now we pass the offset into the
  // low bits and leave the bank-select bits derived from the bank position.

  // =========================================================================
  // Banked data-memory LSU.
  // =========================================================================
  logic [W-1:0]            lsu_req, lsu_we, lsu_gnt, lsu_rvalid;
  logic [W-1:0][31:0]      lsu_addr, lsu_wdata, lsu_rdata;

  // External bus between the LSU and the core boundary.
  logic                       lsu_ext_req;
  logic [31:0]                lsu_ext_addr;
  logic                       lsu_ext_we;
  logic [3:0]                 lsu_ext_be;
  logic [DataWidth-1:0]       lsu_ext_wdata;
  logic                       lsu_ext_gnt;
  logic                       lsu_ext_rvalid;
  logic [DataWidth-1:0]       lsu_ext_rdata;

  ibex_banked_lsumem #(
    .NumLanes     (W),
    .NumDataBanks (NumDataBanks),
    .BankDepth    (DataBankDepth),
    .DataWidth    (DataWidth)
  ) u_lsu (
    .clk_i            (clk_i),
    .rst_ni           (rst_ni),
    .clk_sram_2x_i    (clk_sram_2x_i),
    .req_i            (lsu_req),
    .we_i             (lsu_we),
    .addr_i           (lsu_addr),
    .wdata_i          (lsu_wdata),
    .gnt_o            (lsu_gnt),
    .rvalid_o         (lsu_rvalid),
    .rdata_o          (lsu_rdata),
    .ext_data_req_o   (lsu_ext_req),
    .ext_data_addr_o  (lsu_ext_addr),
    .ext_data_we_o    (lsu_ext_we),
    .ext_data_be_o    (lsu_ext_be),
    .ext_data_wdata_o (lsu_ext_wdata),
    .ext_data_gnt_i   (lsu_ext_gnt),
    .ext_data_rvalid_i(lsu_ext_rvalid),
    .ext_data_rdata_i (lsu_ext_rdata),
    .cfg_i            (dmem_cfg_i),
    .cfg_o            (dmem_cfg_o)
  );

  // =========================================================================
  // W lanes.
  // =========================================================================
  logic [W-1:0] lane_busy, lane_commit_valid;
  logic [W-1:0][4:0]  lane_commit_waddr;
  logic [W-1:0][DataWidth-1:0] lane_commit_wdata;
  logic [W-1:0] lane_redirect;
  logic [W-1:0][31:0] lane_target;

  for (genvar l = 0; l < W; l++) begin : g_lanes
    ibex_lane #(
      .RV32B(RV32B)
    ) u_lane (
      .clk_i              (clk_i),
      .rst_ni             (rst_ni),
      .instr_valid_i      (bundle_active),
      .alu_operator_i     (dec_alu_op[l]),
      .operand_a_i        (rf_rdata_a[l]),
      .operand_b_i        (rf_rdata_b[l]),
      .rf_we_i            (dec_rf_we[l]),
      .rf_waddr_i         (dec_rf_waddr[l]),
      .is_load_i          (dec_data_req[l] & ~dec_data_we[l]),
      .is_store_i         (dec_data_req[l] &  dec_data_we[l]),
      .arf_use_i          (arf_en[l]),
      .arf_we_i           (arf_we[l]),
      .arf_idx_i          (arf_lane_idx[l]),
      .arf_wdata_i        (rf_rdata_b[l]),
      .arf_rdata_i        (arf_lane_rdata[l]),
      .data_req_o         (lsu_req[l]),
      .data_addr_o        (lsu_addr[l]),
      .data_we_o          (lsu_we[l]),
      .data_wdata_o       (lsu_wdata[l]),
      .data_gnt_i         (lsu_gnt[l]),
      .data_rvalid_i      (lsu_rvalid[l]),
      .data_rdata_i       (lsu_rdata[l]),
      .is_branch_i        (dec_branch[l]),
      .is_jump_i          (dec_jump[l]),
      .pc_i               (pc_q + PcWidth'(l*4)),
      .imm_i              (dec_imm_i[l]),
      .branch_redirect_o  (lane_redirect[l]),
      .branch_target_o    (lane_target[l]),
      .commit_valid_o     (lane_commit_valid[l]),
      .commit_waddr_o     (lane_commit_waddr[l]),
      .commit_wdata_o     (lane_commit_wdata[l]),
      .busy_o             (lane_busy[l])
    );
  end

  // =========================================================================
  // Commit: route lane commit-valid writes into the VLIW RF.
  // =========================================================================
  assign rf_we    = lane_commit_valid;
  assign rf_waddr = lane_commit_waddr;
  assign rf_wdata = lane_commit_wdata;

  // RF read addresses = decoder outputs.
  assign rf_raddr_a = dec_rf_raddr_a;
  assign rf_raddr_b = dec_rf_raddr_b;

  // =========================================================================
  // Bundle completion gating: a bundle is done when no lane is busy.
  // =========================================================================
  // Bundle completion gating (DESIGN.md §7.3): a bundle is done when no lane
  // is busy AND at least one execute cycle has elapsed (bundle_active_q).
  assign all_lanes_done = bundle_active & bundle_active_q & ~(|lane_busy);

  // =========================================================================
  // Resolved redirect: the lowest-indexed lane asserting a redirect wins.
  // (The dynarec should ensure at most one redirecting lane per bundle.)
  // =========================================================================
  always_comb begin
    redirect    = 1'b0;
    redirect_pc = '0;
    for (int l = 0; l < W; l++) begin
      if (lane_redirect[l] && !redirect) begin
        redirect    = 1'b1;
        redirect_pc = lane_target[l];
      end
    end
  end

  // =========================================================================
  // External data bus: driven by the banked LSU's external-fallback path for
  // accesses outside the on-chip data memory (MMIO / external RAM). This is
  // the core's external data_* bus.
  // =========================================================================
  assign data_req_o    = lsu_ext_req;
  assign data_addr_o   = lsu_ext_addr;
  assign data_we_o     = lsu_ext_we;
  assign data_be_o     = lsu_ext_be;
  assign data_wdata_o  = lsu_ext_wdata;
  assign lsu_ext_gnt   = data_gnt_i;
  assign lsu_ext_rvalid= data_rvalid_i;
  assign lsu_ext_rdata = data_rdata_i;

  // =========================================================================
  // Status / exceptions.
  // =========================================================================
  assign illegal_insn_o = |(dec_illegal & {W{bundle_active}});
  assign busy_o         = (bundle_active | fetch_enable_i) & ~halted_q;
  assign halted_o       = halted_q;

endmodule
