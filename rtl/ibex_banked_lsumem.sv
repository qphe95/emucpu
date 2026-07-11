// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// ibex_banked_lsumem — the banked data memory + load/store unit for the W-lane
// VLIW core (DESIGN.md §7.2). Each lane has its own request port; the banked
// memory has NumDataBanks prim_ram_2p banks. A crossbar routes each lane's
// request to the bank selected by the low address bits, granting at most one
// lane per bank per cycle (fixed-priority on conflict).
//
// Banks are dual-port (prim_ram_2p): port A = read (load), port B = write
// (store), both clocked by clk_sram_2x_i. So each bank can do 1 read AND 1
// write per cycle — a read-modify-write from the same lane is supported by
// presenting both ports to the crossbar winner.
//
// This is on-chip data SRAM (not the ARF). Loads/stores that miss this memory
// (MMIO / external) are not handled here — they go through the core's external
// data bus, which a future revision can add as a fallback path. For now, all
// data traffic targets the banked on-chip memory.

module ibex_banked_lsumem
  import prim_ram_2p_pkg::*;
#(
  parameter int unsigned NumLanes     = 32,
  parameter int unsigned NumDataBanks = 64,
  parameter int unsigned BankDepth    = 512,   // words per bank
  parameter int unsigned DataWidth    = 32
) (
  input  logic              clk_i,
  input  logic              rst_ni,
  input  logic              clk_sram_2x_i,

  // ---- Per-lane load/store request ----
  input  logic [NumLanes-1:0]            req_i,     // 1=this lane wants the LSU
  input  logic [NumLanes-1:0]            we_i,      // 1=store, 0=load
  input  logic [NumLanes-1:0][31:0]      addr_i,    // byte address
  input  logic [NumLanes-1:0][DataWidth-1:0] wdata_i,
  output logic [NumLanes-1:0]            gnt_o,     // 1=request accepted this cycle
  output logic [NumLanes-1:0]            rvalid_o,  // 1=read data valid this cycle
  output logic [NumLanes-1:0][DataWidth-1:0] rdata_o,

  // ---- SRAM vendor sideband (c6edaa40), per bank ----
  input  ram_2p_cfg_req_t   cfg_i [NumDataBanks],
  output ram_2p_cfg_rsp_t   cfg_o [NumDataBanks]
);

  localparam int unsigned BankSelW = $clog2(NumDataBanks);
  localparam int unsigned OffW     = $clog2(BankDepth);

  // -------------------------------------------------------------------------
  // Split each lane's request into read (load) and write (store) populations.
  // The crossbar handles reads; writes are routed by a parallel fixed-priority
  // arbiter per bank (a bank can do 1 read + 1 write simultaneously via its
  // two ports, so read and write don't contend with each other).
  // -------------------------------------------------------------------------
  logic [NumLanes-1:0]            load_req, store_req;
  logic [NumLanes-1:0][BankSelW-1:0] req_bank;
  logic [NumLanes-1:0][OffW-1:0]   req_off;

  for (genvar l = 0; l < NumLanes; l++) begin : g_split
    assign load_req[l]  = req_i[l] & ~we_i[l];
    assign store_req[l] = req_i[l] &  we_i[l];
    // Word-aligned banked layout: bank = addr[BankSelW+1 : 2], offset = addr[OffW+BankSelW+1 : BankSelW+2]
    assign req_bank[l]  = addr_i[l][BankSelW+1 : 2];
    assign req_off[l]   = addr_i[l][OffW+BankSelW+1 : BankSelW+2];
  end

  // -------------------------------------------------------------------------
  // Read crossbar (loads).
  // -------------------------------------------------------------------------
  logic [NumDataBanks-1:0]                  rd_bank_req;
  logic [NumDataBanks-1:0][OffW-1:0]        rd_bank_addr;
  logic [NumDataBanks-1:0][DataWidth-1:0]   rd_bank_rdata;
  logic [NumLanes-1:0]                      load_gnt;
  logic [NumLanes-1:0][DataWidth-1:0]       load_rdata;

  // Pack addresses for the crossbar: {offset, bank} so the crossbar can slice.
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
    .gnt_o         (load_gnt),
    .rdata_o       (load_rdata),
    .bank_req_o    (rd_bank_req),
    .bank_addr_o   (rd_bank_addr),
    .bank_rdata_i  (rd_bank_rdata)
  );

  // -------------------------------------------------------------------------
  // Write routing (stores): fixed-priority per bank. A bank accepts at most one
  // store per cycle on its port B.
  // -------------------------------------------------------------------------
  logic [NumDataBanks-1:0]            wr_bank_req;
  logic [NumDataBanks-1:0][OffW-1:0]  wr_bank_addr;
  logic [NumDataBanks-1:0][DataWidth-1:0] wr_bank_wdata;
  logic [NumLanes-1:0]                store_gnt;

  always_comb begin
    logic [NumDataBanks-1:0] taken;
    taken         = '0;
    wr_bank_req   = '0;
    wr_bank_addr  = '0;
    wr_bank_wdata = '0;
    store_gnt     = '0;
    for (int l = 0; l < NumLanes; l++) begin
      if (store_req[l] && !taken[req_bank[l]]) begin
        taken[req_bank[l]]      = 1'b1;
        wr_bank_req[req_bank[l]] = 1'b1;
        wr_bank_addr[req_bank[l]] = req_off[l];
        wr_bank_wdata[req_bank[l]] = wdata_i[l];
        store_gnt[l]             = 1'b1;
      end
    end
  end

  // -------------------------------------------------------------------------
  // Bank instantiation: NumDataBanks prim_ram_2p.
  // -------------------------------------------------------------------------
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

  // -------------------------------------------------------------------------
  // Grant + response to lanes. Loads: gnt when the crossbar grants, rvalid the
  // next cycle (prim_ram_2p registers read data in the request cycle, so
  // rvalid coincides with gnt for the on-chip memory — 1-cycle latency).
  // Stores: gnt when the write arbiter grants; no rvalid (fire-and-forget).
  // -------------------------------------------------------------------------
  for (genvar l = 0; l < NumLanes; l++) begin : g_resp
    assign gnt_o[l]    = load_gnt[l] | store_gnt[l];
    assign rvalid_o[l] = load_gnt[l];           // 1-cycle read latency
    assign rdata_o[l]  = load_rdata[l];
  end

  // rst_ni reserved for a future outstanding-transaction tracking variant.
  logic unused_rst;
  assign unused_rst = rst_ni;

endmodule
