// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// ibex_bundle_cache — the W-bank, dual-port instruction memory that fetches
// one W-instruction bundle per cycle (DESIGN.md §7.2.1).
//
// Each of the W banks is a prim_ram_2p:
//   - port A (fetch): 1 read/cycle, supplies instruction i of the current
//     bundle to lane i.
//   - port B (fill): used by the dynarec/loader to write new bundles and to
//     read out code for deopt/inspection.
//
// Bundle layout: instruction i of a bundle lives at bank (i % W). A fetch at
// bundle-PC b reads one word from each bank. Because the bundle is the atomic
// fetch unit there are no cross-bundle bank conflicts.
//
// No branch prediction (DESIGN.md §3): the next bundle PC is always resolved
// — either current+1 (sequential) or a resolved branch target supplied via
// redirect_i. There is never a wrong-path fetch to squash.
//
// Both RAM ports run from clk_sram_2x_i; the fetch read is registered and
// available the same core cycle the bundle executes (fetch latency = 1 cycle,
// no fetch stalls).

module ibex_bundle_cache
  import ibex_bps_pkg::*;
  import prim_ram_2p_pkg::*;
#(
  parameter int unsigned Width      = 32,   // lanes / banks
  parameter int unsigned BankDepth  = 256,  // instructions per bank
  parameter int unsigned PcWidth    = 32
) (
  input  logic              clk_i,
  input  logic              rst_ni,
  input  logic              clk_sram_2x_i,

  // ---- Fetch interface: one bundle (W instructions) per cycle ----
  input  logic              fetch_req_i,     // advance / fetch this cycle
  input  logic [PcWidth-1:0] fetch_pc_i,     // bundle PC (byte addr; bundle = W*4 B)
  output logic [Width-1:0][31:0] fetch_instr_o, // the W instructions
  output logic              fetch_valid_o,   // fetch_instr_o holds a valid bundle
  input  logic              redirect_i,      // resolved redirect this cycle
  input  logic [PcWidth-1:0] redirect_pc_i,  // target bundle PC

  // ---- Fill interface (dynarec/loader): write one word into bank k ----
  input  logic              fill_req_i,
  input  logic [$clog2(Width)-1:0] fill_bank_i,
  input  logic [$clog2(BankDepth)-1:0] fill_addr_i,
  input  logic [31:0]       fill_wdata_i,

  // ---- SRAM vendor sideband (c6edaa40 generic types), per bank ----
  input  ram_2p_cfg_req_t   cfg_i [Width],
  output ram_2p_cfg_rsp_t   cfg_o [Width]
);

  // -------------------------------------------------------------------------
  // Bank instantiation: W prim_ram_2p banks.
  // -------------------------------------------------------------------------
  localparam int unsigned AddrW = $clog2(BankDepth);
  // Each bank's word index for the current bundle PC:
  //   bundle_pc / (W*4)  → which bundle
  //   instruction i is at bank i, word index = (bundle_pc / (W*4))
  // All banks share the same word index for a given bundle; bank i supplies
  // instruction i.
  logic [AddrW-1:0] bundle_word_idx;
  assign bundle_word_idx = fetch_pc_i[2 +: AddrW]; // [2+AddrW-1 : 2]

  for (genvar b = 0; b < Width; b++) begin : g_banks
    logic        a_req, b_req, b_we;
    logic [AddrW-1:0] a_addr, b_addr;
    logic [31:0] a_wdata, b_wdata, a_rdata, b_rdata;
    logic [31:0] a_wmask, b_wmask;

    // Fetch port: every bank reads the same word index each cycle on req.
    assign a_req   = fetch_req_i;
    assign a_addr  = bundle_word_idx;
    assign a_wdata = '0;
    assign a_wmask = {32{1'b1}};
    assign fetch_instr_o[b] = a_rdata;

    // Fill port: only the addressed bank receives the write.
    logic fill_bank_sel;
    assign fill_bank_sel = (fill_bank_i == $clog2(Width)'(b));
    assign b_req  = fill_req_i & fill_bank_sel;
    assign b_we   = 1'b1;
    assign b_addr = fill_addr_i;
    assign b_wdata = fill_wdata_i;
    assign b_wmask = {32{1'b1}};

    prim_ram_2p #(
      .Width(32),
      .Depth(BankDepth),
      .DataBitsPerMask(32)
    ) u_bank (
      .clk_a_i (clk_sram_2x_i),
      .clk_b_i (clk_sram_2x_i),
      .a_req_i (a_req),
      .a_write_i(1'b0),
      .a_addr_i (a_addr),
      .a_wdata_i(a_wdata),
      .a_wmask_i(a_wmask),
      .a_rdata_o(a_rdata),
      .b_req_i (b_req),
      .b_write_i(b_we),
      .b_addr_i (b_addr),
      .b_wdata_i(b_wdata),
      .b_wmask_i(b_wmask),
      .b_rdata_o(b_rdata),
      .cfg_i  (cfg_i[b]),
      .cfg_o  (cfg_o[b])
    );
  end

  // Fetch valid: a read requested last cycle is valid this cycle, unless a
  // redirect supersedes it. (prim_ram_2p registers read data in the same
  // cycle the request is asserted.)
  logic fetch_req_q;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) fetch_req_q <= 1'b0;
    else         fetch_req_q <= fetch_req_i & ~redirect_i;
  end
  assign fetch_valid_o = fetch_req_q;

endmodule
