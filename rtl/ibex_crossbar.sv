// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// ibex_crossbar — a generic W-requestor × N-bank single-cycle read crossbar
// (DESIGN.md §4.4, §7.2). Each requestor presents an address whose low bits
// select a bank; the crossbar grants at most one requestor per bank per cycle
// (fixed priority on conflict), routes the granted requestor's address to that
// bank, and routes the bank's read data back to the requestor.
//
// This is a pure read crossbar (used by the ARF datapath port and the banked
// LSU read path). Writes are handled separately by the callers because the ARF
// and data-mem write paths differ in policy (free-list vs. plain).
//
// Bank selection: bank = addr[BankSelW-1:0]. BankSelW = $clog2(NumBanks).
// If a requestor targets a bank that another higher-priority requestor also
// targets, it is not granted this cycle and must retry (the caller sees
// gnt_o[r]==0). The dynarec is expected to lay out accesses so that conflicts
// are rare (DESIGN.md §4.4).

module ibex_crossbar #(
  parameter int unsigned NumReqs   = 32,  // number of requestors (lanes)
  parameter int unsigned NumBanks  = 64,  // number of banks
  parameter int unsigned AddrWidth = 17,  // bank-offset address width
  parameter int unsigned DataWidth = 32
) (
  input  logic                          clk_i,

  // Requestor side: each requestor presents req + bank-offset address.
  input  logic [NumReqs-1:0]            req_i,
  input  logic [NumReqs-1:0][AddrWidth+($clog2(NumBanks))-1:0] addr_i,
  output logic [NumReqs-1:0]            gnt_o,
  output logic [NumReqs-1:0][DataWidth-1:0] rdata_o,

  // Bank side: per-bank address out + rdata in. The crossbar drives the
  // bank's read address only for the granted requestor; other banks see
  // req=0 (the bank stays idle).
  output logic [NumBanks-1:0]                  bank_req_o,
  output logic [NumBanks-1:0][AddrWidth-1:0]   bank_addr_o,
  input  logic [NumBanks-1:0][DataWidth-1:0]   bank_rdata_i
);

  localparam int unsigned BankSelW = $clog2(NumBanks);

  // -------------------------------------------------------------------------
  // Arbitration: for each bank, pick the highest-priority requestor that
  // targets it. Fixed priority (lowest requestor index wins). Output a per-
  // requestor grant and a per-bank selected requestor (or 'no-one').
  // -------------------------------------------------------------------------
  logic [BankSelW-1:0] req_bank [NumReqs];
  logic                bank_taken [NumBanks];
  logic [$clog2(NumReqs)-1:0] bank_winner [NumBanks];
  logic                bank_has_winner [NumBanks];

  for (genvar r = 0; r < NumReqs; r++) begin : g_req_bank
    assign req_bank[r] = addr_i[r][BankSelW-1:0];
  end

  always_comb begin
    bank_taken       = '{default: 1'b0};
    bank_has_winner  = '{default: 1'b0};
    bank_winner      = '{default: '0};
    gnt_o            = '0;
    bank_req_o       = '0;
    bank_addr_o      = '0;
    // Lowest requestor index has highest priority; iterate upward so earlier
    // requestors claim their bank first.
    for (int r = 0; r < NumReqs; r++) begin
      if (req_i[r] && !bank_taken[req_bank[r]]) begin
        bank_taken[req_bank[r]]      = 1'b1;
        bank_has_winner[req_bank[r]] = 1'b1;
        bank_winner[req_bank[r]]     = r[$clog2(NumReqs)-1:0];
        gnt_o[r]                     = 1'b1;
        bank_req_o[req_bank[r]]      = 1'b1;
        bank_addr_o[req_bank[r]]     = addr_i[r][AddrWidth+BankSelW-1:BankSelW];
      end
    end
  end

  // -------------------------------------------------------------------------
  // Response routing: each bank's read data goes to its winning requestor.
  // Requestors that weren't granted keep their previous/zero rdata (caller
  // must only consume rdata_o on a granted cycle).
  // -------------------------------------------------------------------------
  for (genvar r = 0; r < NumReqs; r++) begin : g_resp
    always_comb begin
      rdata_o[r] = '0;
      for (int b = 0; b < NumBanks; b++) begin
        if (bank_has_winner[b] && (bank_winner[b] == r[$clog2(NumReqs)-1:0])) begin
          rdata_o[r] = bank_rdata_i[b];
        end
      end
    end
  end

  // clk_i is reserved for a future pipelined variant; currently combinational.
  logic unused_clk;
  assign unused_clk = clk_i;

endmodule
