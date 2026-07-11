// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// ibex_predicate — the 8-bit predicate register file (DESIGN.md §7.2).
// Predicate registers guard lane execution: a lane whose predicate bit is
// false commits nothing. Written by compare ops (cmp.eq/...), read as lane
// guards and by guarded mov.pt/mov.pf.
//
// Writes happen at the end of a bundle (commit); reads happen at the start of
// the next bundle (guard evaluation). Within a bundle, at most one write per
// predicate register (enforced by the dynarec) — so there is no write-port
// conflict. If two lanes in the same bundle write the same predicate (a
// dynarec bug), the lowest-indexed lane wins.

module ibex_predicate
  import ibex_bps_pkg::*;
#(
  parameter int unsigned NumPred = NUM_PRED // 8
) (
  input  logic              clk_i,
  input  logic              rst_ni,

  // Predicate reads: all NumPred bits broadcast to every lane each cycle.
  output logic [NumPred-1:0] pred_rdata_o,

  // Predicate write (commit-time): one write port. The dynarec guarantees at
  // most one writer per predicate per bundle; we take the lowest lane that
  // asserts a write to a given predicate.
  input  logic [NumPred-1:0] pred_we_i,   // one bit per predicate: 1=write this pred
  input  logic [NumPred-1:0] pred_wdata_i // the values to write (one bit per pred)
);

  logic [NumPred-1:0] pred_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // pred[0] is the unconditional predicate: reset to 1 so unguarded
      // instructions (pred index 0, no invert) execute by default.
      pred_q[0] <= 1'b1;
      for (int p = 1; p < NumPred; p++) pred_q[p] <= 1'b0;
    end else begin
      for (int p = 0; p < NumPred; p++) begin
        if (pred_we_i[p]) pred_q[p] <= pred_wdata_i[p];
      end
    end
  end

  assign pred_rdata_o = pred_q;

endmodule
