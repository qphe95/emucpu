// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// ibex_addr_regfile — the banked, double-clocked address register file (ARF)
// described in DESIGN.md §4. Holds 128K 32-bit *addresses* (never data) across
// 64 banks, so a W=32 bundle can read one slot per lane per cycle with no
// conflicts. Both ports of every bank run from clk_sram_2x_i (2x core clock);
// phase A is the datapath port, phase B is the management port.
//
// The free-pool metadata (valid bits + free list) lives in a small flip-flop
// directory for the ARF_MANAGED region only — the bulk address array holds
// the actual addresses. See DESIGN.md §5.2/§5.3.

module ibex_addr_regfile
  import ibex_bps_pkg::*;
  import prim_ram_2p_pkg::*;
#(
  // Whole feature can be compiled out by the caller by not instantiating this
  // module; these params exist so the build can shrink the ARF for bring-up.
  parameter int unsigned NumBanks   = ARF_NUM_BANKS,
  parameter int unsigned BankDepth  = ARF_BANK_DEPTH,
  parameter int unsigned IdxWidth   = ARF_IDX_W,
  parameter int unsigned BankSelW   = ARF_BANK_SEL_W,
  parameter int unsigned BankOffW   = ARF_BANK_OFF_W,
  parameter int unsigned ManagedCnt = 256  // directory size for free-pool mgmt
) (
  input  logic                          clk_core_i,    // core clock (for FF metadata)
  input  logic                          rst_core_ni,
  input  logic                          clk_sram_2x_i, // 2x clock for the RAM ports

  // ---- Phase A: datapath port (one read/write per lane per core cycle) ----
  // Index is the full slot index; the low BankSelW bits select the bank.
  input  logic [NumBanks-1:0]           dpa_req_i,
  input  logic [NumBanks-1:0]           dpa_we_i,    // 1=write, 0=read
  input  logic [NumBanks-1:0][IdxWidth-1:0] dpa_idx_i,
  input  logic [NumBanks-1:0][31:0]     dpa_wdata_i,
  output logic [NumBanks-1:0][31:0]     dpa_rdata_o,

  // ---- Phase B: management port (spill engine / dynarec bulk ops) ----
  input  logic                          mgmt_req_i,
  input  logic                          mgmt_we_i,
  input  logic [IdxWidth-1:0]           mgmt_idx_i,
  input  logic [31:0]                   mgmt_wdata_i,
  output logic [31:0]                   mgmt_rdata_o,

  // ---- Free-pool allocation interface (DESIGN.md §5.2) ----
  // spalloc: pop a free slot; spfree: push a slot back.
  input  logic                          spalloc_req_i,
  output logic                          spalloc_gnt_o,
  output logic [IdxWidth-1:0]          spalloc_idx_o,
  input  logic                          spfree_req_i,
  input  logic [IdxWidth-1:0]          spfree_idx_i,

  // ---- SRAM vendor sideband (c6edaa40 generic types), per bank ----
  input  ram_2p_cfg_req_t               cfg_i [NumBanks],
  output ram_2p_cfg_rsp_t               cfg_o [NumBanks]
);

  // -------------------------------------------------------------------------
  // Per-bank RAM instantiation.
  // Each bank is a prim_ram_2p: port A = datapath (1 read OR write/core cycle),
  // port B = management. Both clocked by clk_sram_2x_i. The generic model is
  // a behavioral array; Pango PDS infers Titan2 true-dual-port BSRAM from it
  // (DESIGN.md §13).
  // -------------------------------------------------------------------------
  // Per-bank wires for the management-port read-data reduction (lifted out of
  // the generate so a single mgmt_rdata_o is driven once).
  logic [31:0] mgmt_bank_rdata   [NumBanks];
  logic        mgmt_bank_sel_arr [NumBanks];

  always_comb begin
    mgmt_rdata_o = '0;
    for (int i = 0; i < NumBanks; i++) begin
      if (mgmt_req_i & mgmt_bank_sel_arr[i]) mgmt_rdata_o = mgmt_bank_rdata[i];
    end
  end

  for (genvar b = 0; b < NumBanks; b++) begin : g_banks
    // Slice the per-lane request for this bank. In the W=32 instance the
    // datapath port array is sized NumBanks (>= W) so lane->bank routing is
    // a direct index; the crossbar in ibex_core chooses the mapping.
    logic        a_req, a_we;
    logic [BankOffW-1:0] a_addr;
    logic [31:0] a_wdata;
    logic [31:0] a_rdata;
    logic        b_req, b_we;
    logic [BankOffW-1:0] b_addr;
    logic [31:0] b_wdata;
    logic [31:0] b_rdata;

    // Datapath lane for this bank (one per bank; the lane crossbar feeds these).
    assign a_req   = dpa_req_i[b];
    assign a_we    = dpa_we_i[b];
    assign a_addr  = dpa_idx_i[b][BankOffW-1+BankSelW : BankSelW];
    assign a_wdata = dpa_wdata_i[b];
    assign dpa_rdata_o[b] = a_rdata;

    // Management port: only the bank matching mgmt_idx_i's bank-select bits
    // receives the request; others see req=0.
    logic mgmt_bank_sel;
    assign mgmt_bank_sel = (mgmt_idx_i[BankSelW-1:0] == BankSelW'(b));
    assign b_req   = mgmt_req_i & mgmt_bank_sel;
    assign b_we    = mgmt_we_i;
    assign b_addr  = mgmt_idx_i[BankOffW-1+BankSelW : BankSelW];
    assign b_wdata = mgmt_wdata_i;
    // Per-bank mgmt read data; reduced outside the generate to avoid a
    // multi-driver on mgmt_rdata_o.
    assign mgmt_bank_rdata[b] = b_rdata;
    assign mgmt_bank_sel_arr[b] = mgmt_bank_sel;

    prim_ram_2p #(
      .Width(32),
      .Depth(BankDepth),
      .DataBitsPerMask(32)   // word-granularity writes; ARF stores whole words
    ) u_bank (
      .clk_a_i (clk_sram_2x_i),
      .clk_b_i (clk_sram_2x_i),
      .a_req_i (a_req),
      .a_write_i(a_we),
      .a_addr_i (a_addr),
      .a_wdata_i(a_wdata),
      .a_wmask_i({32{1'b1}}),
      .a_rdata_o(a_rdata),
      .b_req_i (b_req),
      .b_write_i(b_we),
      .b_addr_i (b_addr),
      .b_wdata_i(b_wdata),
      .b_wmask_i({32{1'b1}}),
      .b_rdata_o(b_rdata),
      .cfg_i  (cfg_i[b]),
      .cfg_o  (cfg_o[b])
    );
  end

  // -------------------------------------------------------------------------
  // Free-pool directory (small FF-based free list over the managed region).
  // DESIGN.md §5.2: spalloc returns a free slot, spfree returns one. The list
  // is seeded at reset from ARF_FREEPOOL_BASE. This is deliberately simple —
  // a head-pointer ring over a pre-allocated slot-index array — because the
  // dynarec, not the hardware, drives allocation policy.
  // -------------------------------------------------------------------------
  localparam int unsigned FreeListW = IdxWidth;
  logic [FreeListW-1:0] free_list_q [ManagedCnt];
  logic [$clog2(ManagedCnt)-1:0] free_head_q, free_tail_q;
  logic [$clog2(ManagedCnt+1)-1:0] free_count_q;
  logic free_not_full, free_not_empty;

  assign free_not_empty = (free_count_q != 0);
  assign free_not_full  = (free_count_q != ManagedCnt);
  assign spalloc_gnt_o  = spalloc_req_i & free_not_empty;

  // combinational pop
  assign spalloc_idx_o = free_list_q[free_head_q];

  always_ff @(posedge clk_core_i or negedge rst_core_ni) begin
    if (!rst_core_ni) begin
      for (int i = 0; i < ManagedCnt; i++) begin
        // seed: ManagedCnt contiguous free slots starting at the free pool base
        free_list_q[i] <= ARF_FREEPOOL_BASE + IdxWidth'(i);
      end
      free_head_q  <= '0;
      free_tail_q  <= '0;
      free_count_q <= ManagedCnt;
    end else begin
      // pop (spalloc granted)
      if (spalloc_gnt_o) begin
        free_head_q <= free_head_q + 1;
        free_count_q <= free_count_q - 1;
      end
      // push (spfree), only if not full and not simultaneously popping same slot
      if (spfree_req_i & free_not_full) begin
        if (!(spalloc_gnt_o && (free_count_q == 1))) begin
          free_list_q[free_tail_q] <= spfree_idx_i;
          free_tail_q  <= free_tail_q + 1;
          free_count_q <= spalloc_gnt_o ? free_count_q : (free_count_q + 1);
        end
      end
    end
  end

endmodule
