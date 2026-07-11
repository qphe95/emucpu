// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// ibex_banked_lsumem — the banked data memory + load/store unit for the W-lane
// VLIW core (DESIGN.md §7.2), with an external-fallback path for addresses
// outside the on-chip data memory (MMIO, external RAM).
//
// Each lane's request is classified by address:
//   - On-chip  ([OnChipBase, OnChipBase+OnChipSize)) → banked memory path.
//   - External (anywhere else)                       → external bus path.
//
// Banked path: NumDataBanks prim_ram_2p banks (port A read / port B write),
// a read crossbar + per-bank write arbiter. W loads/stores proceed in parallel
// when they hit distinct banks.
//
// External path: a fixed-priority arbiter picks one external requestor per
// cycle and drives the core's external data_* bus. One outstanding external
// transaction at a time (matches the single external port). The response
// (rvalid/rdata) is routed back to the requesting lane via a pending-lane
// register.
//
// Per-lane gnt/rvalid: a lane is granted when either path grants it; rvalid
// comes from whichever path the lane's outstanding request is on.

module ibex_banked_lsumem
  import prim_ram_2p_pkg::*;
#(
  parameter int unsigned NumLanes     = 32,
  parameter int unsigned NumDataBanks = 64,
  parameter int unsigned BankDepth    = 512,   // words per bank
  parameter int unsigned DataWidth    = 32,
  // On-chip data-memory window. Addresses in [OnChipBase, OnChipBase+OnChipSize)
  // go to the banked memory; everything else goes to the external bus.
  parameter bit [31:0]     OnChipBase = 32'h0000_0000,
  parameter int unsigned   OnChipSize = 32'h0008_0000  // 512 KB default
) (
  input  logic              clk_i,
  input  logic              rst_ni,
  input  logic              clk_sram_2x_i,

  // ---- Per-lane load/store request ----
  input  logic [NumLanes-1:0]            req_i,
  input  logic [NumLanes-1:0]            we_i,
  input  logic [NumLanes-1:0][31:0]      addr_i,
  input  logic [NumLanes-1:0][DataWidth-1:0] wdata_i,
  output logic [NumLanes-1:0]            gnt_o,
  output logic [NumLanes-1:0]            rvalid_o,
  output logic [NumLanes-1:0][DataWidth-1:0] rdata_o,

  // ---- External data bus (for MMIO / external memory) ----
  output logic              ext_data_req_o,
  output logic [31:0]       ext_data_addr_o,
  output logic              ext_data_we_o,
  output logic [3:0]        ext_data_be_o,
  output logic [DataWidth-1:0] ext_data_wdata_o,
  input  logic              ext_data_gnt_i,
  input  logic              ext_data_rvalid_i,
  input  logic [DataWidth-1:0] ext_data_rdata_i,

  // ---- SRAM vendor sideband, per bank ----
  input  ram_2p_cfg_req_t   cfg_i [NumDataBanks],
  output ram_2p_cfg_rsp_t   cfg_o [NumDataBanks]
);

  localparam int unsigned BankSelW = $clog2(NumDataBanks);
  localparam int unsigned OffW     = $clog2(BankDepth);
  // Upper bound of the on-chip window (exclusive). Precomputed so the address
  // decode is a pair of plain comparisons without an additive expression in
  // the comparison (which trips lint and could wrap).
  localparam bit [31:0] OnChipLimit = OnChipBase + OnChipSize[31:0];

  // -------------------------------------------------------------------------
  // Address decode: classify each lane's request as on-chip or external.
  // -------------------------------------------------------------------------
  logic [NumLanes-1:0] onchip, external;
  for (genvar l = 0; l < NumLanes; l++) begin : g_decode
    // OnChipBase is a parameter; the lower-bound comparison is redundant only
    // in the default (base=0) case, but correct for any nonzero base, so we
    // keep it and silence the parameter-dependent lint.
    /* verilator lint_off UNSIGNED */
    assign onchip[l]  = req_i[l] &
                        (addr_i[l] >= OnChipBase) &
                        (addr_i[l] <  OnChipLimit);
    /* verilator lint_on UNSIGNED */
    assign external[l] = req_i[l] & ~onchip[l];
  end

  // =========================================================================
  // ON-CHIP PATH (banked memory)
  // =========================================================================
  logic [NumLanes-1:0]            load_req, store_req;
  logic [NumLanes-1:0][BankSelW-1:0] req_bank;
  logic [NumLanes-1:0][OffW-1:0]   req_off;

  for (genvar l = 0; l < NumLanes; l++) begin : g_split
    assign load_req[l]  = onchip[l] & ~we_i[l];
    assign store_req[l] = onchip[l] &  we_i[l];
    assign req_bank[l]  = addr_i[l][BankSelW+1 : 2];
    assign req_off[l]   = addr_i[l][OffW+BankSelW+1 : BankSelW+2];
  end

  // Read crossbar (loads).
  logic [NumDataBanks-1:0]                  rd_bank_req;
  logic [NumDataBanks-1:0][OffW-1:0]        rd_bank_addr;
  logic [NumDataBanks-1:0][DataWidth-1:0]   rd_bank_rdata;
  logic [NumLanes-1:0]                      oc_load_gnt;
  logic [NumLanes-1:0][DataWidth-1:0]       oc_load_rdata;

  logic [NumLanes-1:0][OffW+BankSelW-1:0] load_addr_packed;
  for (genvar l = 0; l < NumLanes; l++) begin : g_pack
    assign load_addr_packed[l] = {req_off[l], req_bank[l]};
  end

  ibex_crossbar #(
    .NumReqs  (NumLanes),
    .NumBanks (NumDataBanks),
    .AddrWidth(OffW),
    .DataWidth(DataWidth)
  ) u_rd_xbar (
    .clk_i         (clk_i),
    .req_i         (load_req),
    .addr_i        (load_addr_packed),
    .gnt_o         (oc_load_gnt),
    .rdata_o       (oc_load_rdata),
    .bank_req_o    (rd_bank_req),
    .bank_addr_o   (rd_bank_addr),
    .bank_rdata_i  (rd_bank_rdata)
  );

  // Write routing (stores): fixed-priority per bank.
  logic [NumDataBanks-1:0]            wr_bank_req;
  logic [NumDataBanks-1:0][OffW-1:0]  wr_bank_addr;
  logic [NumDataBanks-1:0][DataWidth-1:0] wr_bank_wdata;
  logic [NumLanes-1:0]                oc_store_gnt;

  always_comb begin
    logic [NumDataBanks-1:0] taken;
    taken         = '0;
    wr_bank_req   = '0;
    wr_bank_addr  = '0;
    wr_bank_wdata = '0;
    oc_store_gnt  = '0;
    for (int l = 0; l < NumLanes; l++) begin
      if (store_req[l] && !taken[req_bank[l]]) begin
        taken[req_bank[l]]        = 1'b1;
        wr_bank_req[req_bank[l]]  = 1'b1;
        wr_bank_addr[req_bank[l]] = req_off[l];
        wr_bank_wdata[req_bank[l]] = wdata_i[l];
        oc_store_gnt[l]           = 1'b1;
      end
    end
  end

  // Bank instantiation.
  for (genvar b = 0; b < NumDataBanks; b++) begin : g_banks
    prim_ram_2p #(
      .Width(DataWidth),
      .Depth(BankDepth),
      .DataBitsPerMask(DataWidth)
    ) u_bank (
      .clk_a_i (clk_sram_2x_i),
      .clk_b_i (clk_sram_2x_i),
      .a_req_i  (rd_bank_req[b]),
      .a_write_i(1'b0),
      .a_addr_i (rd_bank_addr[b]),
      .a_wdata_i('0),
      .a_wmask_i({DataWidth{1'b1}}),
      .a_rdata_o(rd_bank_rdata[b]),
      .b_req_i  (wr_bank_req[b]),
      .b_write_i(1'b1),
      .b_addr_i (wr_bank_addr[b]),
      .b_wdata_i(wr_bank_wdata[b]),
      .b_wmask_i({DataWidth{1'b1}}),
      .b_rdata_o(),
      .cfg_i   (cfg_i[b]),
      .cfg_o   (cfg_o[b])
    );
  end

  // On-chip per-lane gnt/rvalid.
  // Loads: gnt = crossbar grant; rvalid coincides (1-cycle read latency).
  // Stores: gnt = write-arbiter grant; rvalid = gnt (fire-and-forget).
  logic [NumLanes-1:0]            oc_gnt, oc_rvalid;
  logic [NumLanes-1:0][DataWidth-1:0] oc_rdata;
  for (genvar l = 0; l < NumLanes; l++) begin : g_oc_resp
    assign oc_gnt[l]    = oc_load_gnt[l] | oc_store_gnt[l];
    assign oc_rvalid[l] = oc_load_gnt[l] | oc_store_gnt[l];
    assign oc_rdata[l]  = oc_load_rdata[l];
  end

  // =========================================================================
  // EXTERNAL PATH (MMIO / external memory)
  // =========================================================================
  // One outstanding external transaction. A fixed-priority arbiter selects the
  // lowest-indexed lane with an external request when the bus is idle. The
  // pending lane index is held until rvalid returns; then rvalid/rdata route
  // back to that lane.
  // =========================================================================
  logic                          ext_busy;
  logic [$clog2(NumLanes)-1:0]   ext_pending_lane_q;
  logic                          ext_pending_is_store_q;
  logic                          ext_selected;
  logic [$clog2(NumLanes)-1:0]   ext_sel_lane;
  logic [NumLanes-1:0]           ext_gnt, ext_rvalid;
  logic [NumLanes-1:0][DataWidth-1:0] ext_rdata;

  // Select the lowest-indexed external requestor when not busy.
  always_comb begin
    ext_selected = 1'b0;
    ext_sel_lane = '0;
    for (int l = 0; l < NumLanes; l++) begin
      if (external[l] && !ext_selected) begin
        ext_selected = 1'b1;
        ext_sel_lane = l[$clog2(NumLanes)-1:0];
      end
    end
  end

  // Drive the external bus.
  assign ext_data_req_o   = ext_selected & ~ext_busy;
  assign ext_data_addr_o  = addr_i[ext_sel_lane];
  assign ext_data_we_o    = we_i[ext_sel_lane];
  assign ext_data_be_o    = 4'hF;
  assign ext_data_wdata_o = wdata_i[ext_sel_lane];

  // External FSM: IDLE (accept) -> WAIT_RVALID (for loads).
  typedef enum logic { EXT_IDLE, EXT_WAIT } ext_state_e;
  ext_state_e ext_state_q, ext_state_d;

  always_comb begin
    ext_state_d = ext_state_q;
    unique case (ext_state_q)
      EXT_IDLE: begin
        if (ext_data_req_o && ext_data_gnt_i) begin
          // For stores, the transaction completes at grant. For loads, wait.
          ext_state_d = ext_data_we_o ? EXT_IDLE : EXT_WAIT;
        end
      end
      EXT_WAIT: begin
        if (ext_data_rvalid_i) ext_state_d = EXT_IDLE;
      end
      default: ext_state_d = EXT_IDLE;
    endcase
  end

  assign ext_busy = (ext_state_q != EXT_IDLE);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ext_state_q           <= EXT_IDLE;
      ext_pending_lane_q    <= '0;
      ext_pending_is_store_q<= 1'b0;
    end else begin
      ext_state_q <= ext_state_d;
      if (ext_data_req_o && ext_data_gnt_i) begin
        ext_pending_lane_q     <= ext_sel_lane;
        ext_pending_is_store_q <= ext_data_we_o;
      end
    end
  end

  // Route external grant + response to lanes.
  for (genvar l = 0; l < NumLanes; l++) begin : g_ext_resp
    // Grant: this lane is the selected external requestor and the bus grants.
    assign ext_gnt[l] = ext_data_req_o & ext_data_gnt_i &
                        (ext_sel_lane == l[$clog2(NumLanes)-1:0]);
    // rvalid: stores complete at grant; loads complete at rvalid.
    assign ext_rvalid[l] = ((ext_pending_is_store_q & ext_gnt[l]) |
                           (~ext_pending_is_store_q & ext_data_rvalid_i &
                            (ext_pending_lane_q == l[$clog2(NumLanes)-1:0])));
    assign ext_rdata[l]  = ext_data_rdata_i;
  end

  // =========================================================================
  // Merge on-chip and external responses per lane.
  // =========================================================================
  for (genvar l = 0; l < NumLanes; l++) begin : g_merge
    assign gnt_o[l]    = oc_gnt[l]   | ext_gnt[l];
    assign rvalid_o[l] = oc_rvalid[l] | ext_rvalid[l];
    assign rdata_o[l]  = onchip[l] ? oc_rdata[l] : ext_rdata[l];
  end

endmodule
