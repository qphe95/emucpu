// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// ibex_bundle_cache — the W-bank, dual-port instruction memory that fetches
// one W-instruction bundle per cycle (DESIGN.md §7.2.1).
//
// Each of the W banks is a prim_ram_2p holding 32-bit instruction words.
// Bundle layout: instruction i of a bundle lives at bank (i % W). A fetch at
// bundle-PC b reads one word from each bank. Because the bundle is the atomic
// fetch unit there are no cross-bundle bank conflicts.
//
// Port A (fetch): 1 read/cycle → supplies instruction i to lane i.
// Port B (fill):  dynarec/loader writes new bundles; also read-out for deopt.
//
// No branch prediction (DESIGN.md §3): the next bundle PC is always resolved
// — either current+bundle_bytes (sequential) or a resolved redirect target.
// Both ports run from clk_sram_2x_i.

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
  input  logic              fetch_req_i,
  input  logic [PcWidth-1:0] fetch_pc_i,        // bundle PC (byte addr)
  output logic [Width-1:0][31:0] fetch_instr_o, // the W instructions
  output logic              fetch_valid_o,
  input  logic              redirect_i,
  input  logic [PcWidth-1:0] redirect_pc_i,

  // ---- Fill interface: write one word into bank k ----
  input  logic              fill_req_i,
  input  logic [$clog2(Width)-1:0]    fill_bank_i,
  input  logic [$clog2(BankDepth)-1:0] fill_addr_i,
  input  logic [31:0]       fill_wdata_i,

  // ---- SRAM vendor sideband, per bank ----
  input  ram_2p_cfg_req_t   cfg_i [Width],
  output ram_2p_cfg_rsp_t   cfg_o [Width]
);

  localparam int unsigned AddrW = $clog2(BankDepth);

  // Word index for the current bundle: bundle_pc / (Width*4) → which bundle.
  // All banks share the same word index for a given bundle.
  logic [AddrW-1:0] bundle_word_idx;
  assign bundle_word_idx = fetch_pc_i[2 +: AddrW]; // [2+AddrW-1 : 2] (32-bit words)

  for (genvar b = 0; b < Width; b++) begin : g_banks
    logic        a_req, b_req, b_we;
    logic [AddrW-1:0] a_addr, b_addr;
    logic [31:0] a_wdata, b_wdata, a_rdata, b_rdata;
    logic [31:0] a_wmask, b_wmask;

    assign a_req   = fetch_req_i;
    assign a_addr  = bundle_word_idx;
    assign a_wdata = '0;
    assign a_wmask = {32{1'b1}};
    // Latch the fetched instruction so it is stable when bundle_active asserts
    // (prim_ram_2p's read is registered; a_rdata is valid one cycle after a_req;
    // latching on fetch_req_q keeps it stable for the whole execute window).
    logic [31:0] a_rdata_q;
    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni)          a_rdata_q <= '0;
      else if (fetch_req_q) a_rdata_q <= a_rdata;
    end
    assign fetch_instr_o[b] = a_rdata_q;

    logic fill_bank_sel;
    assign fill_bank_sel = (fill_bank_i == $clog2(Width)'(b));
    assign b_req   = fill_req_i & fill_bank_sel;
    assign b_we    = 1'b1;
    assign b_addr  = fill_addr_i;
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

  // Fetch valid: registered read; cleared on redirect.
  logic fetch_req_q;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni)   fetch_req_q <= 1'b0;
    else           fetch_req_q <= fetch_req_i & ~redirect_i;
  end
  assign fetch_valid_o = fetch_req_q;

endmodule
