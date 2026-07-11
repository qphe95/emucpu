// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// ibex_bps_pkg — types and parameters for the BPS-V extension layer
// (banked address register file, custom ARF instructions, predicate state,
// hot-address detector, memory-event waits). See DESIGN.md.
//
// This package is self-contained and only depends on standard SystemVerilog
// types, so the BPS feature can be compiled out entirely by leaving
// `AddrRegFile` at its default of 0 in ibex_core.

package ibex_bps_pkg;

  // -------------------------------------------------------------------------
  // Opcodes claimed from the RISC-V reserved custom space (DESIGN.md §6.1).
  // Mirrors the entries added to ibex_pkg::opcode_e; replicated here as
  // raw constants so the decoder can test instr[6:0] without a cast.
  // -------------------------------------------------------------------------
  localparam logic [6:0] OPCODE_CUSTOM_0 = 7'h0b; // ARF datapath ops
  localparam logic [6:0] OPCODE_CUSTOM_1 = 7'h2b; // ARF management / spill / wait-jump

  // -------------------------------------------------------------------------
  // ARF geometry (DESIGN.md §4.1, §13). Sized for the PG2T390H instance.
  // -------------------------------------------------------------------------
  // Total ARF depth: 128K entries = 2^17 (fits 24% of the part's BRAM).
  parameter int unsigned ARF_DEPTH      = 17'd131072;
  // Number of banks: 64 = 2*W (W=32). Keeps the bank constraint satisfiable
  // by RA with headroom (DESIGN.md §4.4, DYNAREC.md §B.4).
  parameter int unsigned ARF_NUM_BANKS  = 64;
  // Entries per bank = ARF_DEPTH / ARF_NUM_BANKS = 2048.
  localparam int unsigned ARF_BANK_DEPTH = ARF_DEPTH / ARF_NUM_BANKS;

  // Index width for the full ARF space.
  localparam int unsigned ARF_IDX_W = 17; // log2(131072)
  // Bank-select bits: low bits of the index → interleaved banking so that
  // consecutive slots land in distinct banks (walk i → bank i, for i < 64).
  localparam int unsigned ARF_BANK_SEL_W = 6; // log2(64)
  localparam int unsigned ARF_BANK_OFF_W = ARF_IDX_W - ARF_BANK_SEL_W; // 11

  // Slot-index sentinel returned by spalloc when the free pool is empty.
  localparam logic [ARF_IDX_W-1:0] ARF_SLOT_INVALID = '1;

  // -------------------------------------------------------------------------
  // Logical regions of the ARF index space (DESIGN.md §4.2). Interleaved
  // banking means these are *logical* ranges; a range spans all banks.
  // -------------------------------------------------------------------------
  localparam logic [ARF_IDX_W-1:0] ARF_EXPLICIT_BASE  = 17'h00000; // 1K
  localparam int unsigned           ARF_EXPLICIT_SIZE  = 1024;
  localparam logic [ARF_IDX_W-1:0] ARF_WORKINGSET_BASE = 17'h00400; // ~7.75K
  localparam logic [ARF_IDX_W-1:0] ARF_FREEPOOL_BASE   = 17'h02000; // ~122K
  localparam logic [ARF_IDX_W-1:0] ARF_META_BASE       = 17'h1FFF0; // 16 slots

  // -------------------------------------------------------------------------
  // funct3 sub-operation selectors for the custom opcodes (DESIGN.md §6.2).
  // -------------------------------------------------------------------------
  typedef enum logic [2:0] {
    ARF_F3_SLOTR    = 3'b000, // also SLOTW (distinguished by funct7)
    ARF_F3_PINA     = 3'b001,
    ARF_F3_UNPIN    = 3'b010,
    ARF_F3_LDP      = 3'b011,
    ARF_F3_STP      = 3'b100,
    ARF_F3_LDPI     = 3'b101,
    ARF_F3_STPI     = 3'b110,
    ARF_F3_LDP_NEXT = 3'b111
  } arf_custom0_f3_e;

  typedef enum logic [2:0] {
    ARF_F3_SPHINT   = 3'b000,
    ARF_F3_SPLR     = 3'b001,
    ARF_F3_SPFREE   = 3'b010,
    ARF_F3_SPALLOC  = 3'b011,
    ARF_F3_LDPCAP   = 3'b100,
    ARF_F3_SPFLUSH  = 3'b111,
    // Memory-event wait/jump family (DESIGN.md §3.4). Uses a funct7 prefix
    // to distinguish the condition; funct3 groups them.
    ARF_F3_WJ       = 3'b101
  } arf_custom1_f3_e;

  // funct7 values for the wait/jump family (opcode CUSTOM-1, funct3 = ARF_F3_WJ).
  localparam logic [6:0] ARF_WJ_EQ = 7'b0000000; // wait-jump-if-equal
  localparam logic [6:0] ARF_WJ_NE = 7'b0000001; // wait-jump-if-not-equal
  localparam logic [6:0] ARF_WJ_SET = 7'b0000010; // wait-jump-on-bit-set

  // funct7 distinguishes SLOTR (0) from SLOTW (1) under CUSTOM-0/ARF_F3_SLOTR.
  localparam logic [6:0] ARF_F7_SLOTR = 7'b0000000;
  localparam logic [6:0] ARF_F7_SLOTW = 7'b0000001;

  // -------------------------------------------------------------------------
  // Predicate file (DESIGN.md §7.2). 8 1-bit predicate registers guard lanes.
  // -------------------------------------------------------------------------
  localparam int unsigned NUM_PRED = 8;

  // -------------------------------------------------------------------------
  // Hot-address detector (DESIGN.md §8). Snoops the data bus.
  // -------------------------------------------------------------------------
  parameter int unsigned HOT_TABLE_ENTRIES = 64;   // counter table size
  parameter int unsigned HOT_CNT_WIDTH     = 4;    // saturating counter width
  parameter int unsigned HOT_THRESH       = 8;     // promotion threshold
  parameter int unsigned HOT_RECQ_DEPTH   = 8;     // recommendation FIFO depth

endpackage
