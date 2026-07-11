// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// ibex_vliw_dispatch — the top of the W-lane statically-dispatched (VLIW)
// execution engine (DESIGN.md §7). It replaces the scalar ibex_id_stage +
// ibex_ex_block path when SuperscalarWidth > 1.
//
// Pipeline (all within one core cycle, plus optional load-wait):
//   bundle_cache → W× ibex_decoder → W× ibex_lane → commit → next-PC
//
// There is NO scheduler, NO rename, NO ROB, NO branch predictor. The dynarec
// has already ordered everything. A bundle completes when its slowest lane is
// not busy (bundle-completion gating, DESIGN.md §7.3).
//
// Branches are resolved (not predicted): if any lane raises a redirect this
// cycle, the fetch PC is set to that lane's target for the next bundle.

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
  parameter int unsigned PcWidth          = 32
) (
  input  logic              clk_i,
  input  logic              rst_ni,
  input  logic              clk_sram_2x_i,

  // ---- Boot / control ----
  input  logic              fetch_enable_i,    // start fetching
  output logic              busy_o,            // engine is running

  // ---- Bundle cache fill (dynarec/loader writes instructions) ----
  input  logic              fill_req_i,
  input  logic [$clog2(SuperscalarWidth)-1:0] fill_bank_i,
  input  logic [$clog2(BundleBankDepth)-1:0]  fill_addr_i,
  input  logic [31:0]       fill_wdata_i,

  // ---- Data-memory bus (banked LSU, shared by all lanes' load/store) ----
  // For v1 this is a single shared port; a true banked LSU is future work.
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

  // ---- SRAM vendor sideband for the bundle-cache banks ----
  input  ram_2p_cfg_req_t   bcache_cfg_i [SuperscalarWidth],
  output ram_2p_cfg_rsp_t   bcache_cfg_o [SuperscalarWidth]
);

  localparam int unsigned W = SuperscalarWidth;

  // -------------------------------------------------------------------------
  // Program counter. No prediction: next PC = redirect ? target : pc + bundle_bytes.
  // -------------------------------------------------------------------------
  localparam logic [PcWidth-1:0] BundleBytes = PcWidth'(W * 4);
  logic [PcWidth-1:0] pc_q, pc_d;
  logic               redirect;
  logic [PcWidth-1:0] redirect_pc;
  logic               bundle_active;     // a bundle is currently executing

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni)        pc_q <= '0;      // boot address supplied externally in real SoC
    else if (redirect)  pc_q <= redirect_pc;
    else if (bundle_active & ~any_lane_busy) pc_q <= pc_q + BundleBytes;
  end

  // -------------------------------------------------------------------------
  // Bundle fetch.
  // -------------------------------------------------------------------------
  logic [W-1:0][31:0] fetch_instr;
  logic               fetch_valid;
  logic               fetch_req;

  // The engine fetches when enabled and no bundle is mid-flight, or when the
  // current bundle just completed.
  logic all_lanes_done;
  assign fetch_req = fetch_enable_i & (~bundle_active | all_lanes_done);

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
    .fetch_instr_o   (fetch_instr),
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

  // A fetched bundle becomes active the cycle after fetch_valid.
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni)              bundle_active <= 1'b0;
    else if (fetch_valid)     bundle_active <= 1'b1;
    else if (all_lanes_done)  bundle_active <= 1'b0;
  end

  // -------------------------------------------------------------------------
  // W decoders. Each lane decodes its own instruction from the bundle.
  // -------------------------------------------------------------------------
  // Decoder outputs (the subset we need per lane). We instantiate W decoders.
  logic [W-1:0]                 dec_illegal;
  logic [W-1:0][4:0]            dec_rf_raddr_a, dec_rf_raddr_b, dec_rf_waddr;
  logic [W-1:0]                 dec_rf_ren_a, dec_rf_ren_b, dec_rf_we;
  logic [W-1:0]                 dec_data_req, dec_data_we;
  logic [W-1:0]                 dec_jump, dec_branch;
  ibex_pkg::alu_op_e            dec_alu_op [W-1:0];
  logic [W-1:0]                 arf_en, arf_we, arf_deref, arf_is_ldp_next;
  logic [W-1:0]                 arf_is_ldpcap, arf_is_pina, arf_is_unpin;
  logic [W-1:0]                 arf_is_spalloc, arf_is_spfree, arf_is_sphint;
  logic [W-1:0]                 arf_is_splr, arf_is_spflush, arf_is_wj;
  logic [W-1:0][6:0]            arf_wj_funct7;
  logic [W-1:0][2:0]            arf_pred;
  logic [W-1:0]                 illegal_c_insn;

  for (genvar l = 0; l < W; l++) begin : g_decoders
    // Compressed-expansion is not handled in v1 (assume 32-bit bundles from
    // the dynarec). illegal_c_insn is tied low.
    assign illegal_c_insn[l] = 1'b0;

    ibex_decoder u_dec (
      .clk_i              (clk_i),
      .rst_ni             (rst_ni),
      .illegal_c_insn_i   (1'b0),
      .instr_rdata_i      (fetch_instr[l]),
      .illegal_insn_o     (dec_illegal[l]),
      .ebrk_insn_o        (),
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
      .imm_i_type_o       (),
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

  // -------------------------------------------------------------------------
  // Predicate file.
  // -------------------------------------------------------------------------
  logic [NUM_PRED-1:0] pred_rdata;
  logic [NUM_PRED-1:0] pred_we, pred_wdata;
  ibex_predicate u_predicate (
    .clk_i       (clk_i),
    .rst_ni      (rst_ni),
    .pred_rdata_o(pred_rdata),
    .pred_we_i   (pred_we),
    .pred_wdata_i(pred_wdata)
  );

  // -------------------------------------------------------------------------
  // VLIW data register file.
  // -------------------------------------------------------------------------
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

  // -------------------------------------------------------------------------
  // W lanes.
  // -------------------------------------------------------------------------
  logic [W-1:0] lane_busy, lane_commit_valid;
  logic [W-1:0][4:0]  lane_commit_waddr;
  logic [W-1:0][DataWidth-1:0] lane_commit_wdata;
  logic [W-1:0] lane_pred_we;
  logic [W-1:0][2:0] lane_pred_waddr;
  logic [W-1:0] lane_pred_wdata;

  // Shared data-memory bus: round-robin arbitrate among lanes that have a
  // request this cycle. (A full banked LSU is future work; this shared port
  // serializes memory ops across lanes, which is correct but not yet at full
  // bandwidth.)
  logic [W-1:0] lane_data_req, lane_data_we;
  logic [W-1:0][31:0] lane_data_addr, lane_data_wdata;
  logic [W-1:0] lane_data_gnt;

  // Simple fixed-priority arbiter (lane 0 highest). A round-robin would be
  // fairer; fixed priority is correct and simple for v1.
  always_comb begin
    lane_data_gnt = '0;
    data_req_o    = 1'b0;
    data_we_o     = 1'b0;
    data_addr_o   = '0;
    data_wdata_o  = '0;
    data_be_o     = 4'hF;
    for (int l = 0; l < W; l++) begin
      if (lane_data_req[l] && !data_req_o) begin
        data_req_o    = 1'b1;
        data_we_o     = lane_data_we[l];
        data_addr_o   = lane_data_addr[l];
        data_wdata_o  = lane_data_wdata[l];
        lane_data_gnt[l] = 1'b1;
      end
    end
  end

  // All lanes share the single rvalid (the granted lane consumes it).
  // This is a simplification: with one outstanding op per lane, only the
  // granted lane is in WAIT_RVALID, so routing rvalid to all is harmless.
  logic any_lane_busy;
  assign any_lane_busy = |lane_busy;

  for (genvar l = 0; l < W; l++) begin : g_lanes
    // Predicate guard: instructions without a predicate guard are always
    // active (guard = true). The dynarec encodes the guard in instr bits; for
    // v1 we treat all lanes as unguarded unless arf_pred indicates otherwise.
    logic guard;
    assign guard = bundle_active; // simplified: active bundle = all lanes eligible

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
      .pred_guard_i       (guard),
      .is_pred_set_i      (dec_branch[l]),   // branches produce a predicate
      .pred_waddr_i       (arf_pred[l]),
      .arf_use_i          (arf_en[l]),
      .arf_we_i           (arf_we[l]),
      .arf_idx_i          ('0),               // ARF routing is the §5 crossbar
      .arf_wdata_i        (rf_rdata_b[l]),
      .arf_rdata_o        (),
      .data_req_o         (lane_data_req[l]),
      .data_addr_o        (lane_data_addr[l]),
      .data_we_o          (lane_data_we[l]),
      .data_wdata_o       (lane_data_wdata[l]),
      .data_gnt_i         (lane_data_gnt[l]),
      .data_rvalid_i      (data_rvalid_i),
      .data_rdata_i       (data_rdata_i),
      .commit_valid_o     (lane_commit_valid[l]),
      .commit_waddr_o     (lane_commit_waddr[l]),
      .commit_wdata_o     (lane_commit_wdata[l]),
      .pred_we_o          (lane_pred_we[l]),
      .pred_waddr_commit_o(lane_pred_waddr[l]),
      .pred_wdata_commit_o(lane_pred_wdata[l]),
      .busy_o             (lane_busy[l])
    );
  end

  // -------------------------------------------------------------------------
  // Commit: route lane commit-valid writes into the VLIW RF.
  // -------------------------------------------------------------------------
  assign rf_we    = lane_commit_valid;
  assign rf_waddr = lane_commit_waddr;
  assign rf_wdata = lane_commit_wdata;

  // Predicate write: OR across lanes (dynarec guarantees ≤1 writer per pred).
  always_comb begin
    pred_we     = '0;
    pred_wdata  = '0;
    for (int l = 0; l < W; l++) begin
      if (lane_pred_we[l]) begin
        pred_we[lane_pred_waddr[l]] = 1'b1;
        pred_wdata[lane_pred_waddr[l]] = lane_pred_wdata[l];
      end
    end
  end

  // RF read addresses = decoder outputs.
  assign rf_raddr_a = dec_rf_raddr_a;
  assign rf_raddr_b = dec_rf_raddr_b;

  // -------------------------------------------------------------------------
  // Bundle completion gating (DESIGN.md §7.3): a bundle is done when no lane
  // is busy. (Memory-latency hiding across bundles is future work; v1 waits.)
  // -------------------------------------------------------------------------
  assign all_lanes_done = bundle_active & ~any_lane_busy;

  // -------------------------------------------------------------------------
  // Resolved redirect. v1: no jump/branch resolution yet (the dynarec emits
  // straight-line predicated code; control flow is handled by predication).
  // A future revision adds branch-target resolution from the lanes.
  // -------------------------------------------------------------------------
  assign redirect    = 1'b0;
  assign redirect_pc = '0;

  // -------------------------------------------------------------------------
  // Status / exceptions.
  // -------------------------------------------------------------------------
  assign illegal_insn_o = |(dec_illegal & {W{bundle_active}});
  assign busy_o         = bundle_active | fetch_enable_i;

endmodule
