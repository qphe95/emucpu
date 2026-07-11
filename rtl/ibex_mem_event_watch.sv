// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// ibex_mem_event_watch — per-lane memory-event wait/wake for the wj* family
// (DESIGN.md §3.4). Implements "go to PC P when a specific MMIO address is
// written" without polling.
//
// Semantics:
//   wjeq  pK, si, rs2, P : watch MMIO addr S[si]; when MEM[S[si]] == rs2,
//                          set predicate pK and set next bundle PC to P.
//   wjne  ...            : same, condition !=
//   wset  ...            : same, condition (MEM[S[si]] & rs2) != 0
//
// A lane that issues a wj* suspends (predicate stays false, emits no data-mem
// request) and is woken when the LSU observes a store to the watched address
// arriving on the data bus. On wake, the condition is re-tested once.
//
// This is the single-lane v1 instantiation; the per-lane version is a
// straightforward replication parameterized by W (DESIGN.md §7.2/§7.4 lists
// ibex_mem_event_watch.sv as one instance per lane).

module ibex_mem_event_watch
  import ibex_bps_pkg::*;
#(
  parameter int unsigned NumLanes = 1
) (
  input  logic              clk_i,
  input  logic              rst_ni,

  // ---- Instruction issue (one wj* op per lane, in the current bundle) ----
  input  logic [NumLanes-1:0]           wj_issue_i,   // lane issues a wj*
  input  logic [NumLanes-1:0][2:0]      wj_pred_i,    // predicate reg to set on fire
  input  logic [NumLanes-1:0][6:0]      wj_funct7_i,  // EQ / NE / SET
  input  logic [NumLanes-1:0][31:0]     wj_watch_addr_i, // the MMIO address to watch (= S[si])
  input  logic [NumLanes-1:0][31:0]     wj_sentinel_i,   // rs2 value
  input  logic [NumLanes-1:0][31:0]     wj_target_pc_i,  // P

  // ---- Bus snoop: stores observed on the data bus ----
  input  logic              bus_req_i,
  input  logic              bus_we_i,      // only stores wake a watcher
  input  logic [31:0]       bus_addr_i,

  // ---- Wake-time re-read of the watched location (from the LSU) ----
  // On wake the watcher needs the current value of MEM[S[si]] to test the
  // condition. The LSU supplies this via a small read port.
  input  logic [NumLanes-1:0][31:0]      wj_curval_i,  // current MEM[S[si]]
  input  logic [NumLanes-1:0]            wj_curval_valid_i,

  // ---- Outputs back to the predicate file and the fetch PC ----
  output logic [NumLanes-1:0]           wj_fired_o,    // condition met this cycle
  output logic [NumLanes-1:0][2:0]      wj_pred_o,     // which predicate to set
  output logic [NumLanes-1:0][31:0]     wj_target_pc_o,
  output logic [NumLanes-1:0]           wj_active_o    // lane is suspended-waiting
);

  // Per-lane state.
  typedef struct packed {
    logic        active;
    logic [2:0]  pred;
    logic [6:0]  funct7;
    logic [31:0] watch_addr;
    logic [31:0] sentinel;
    logic [31:0] target_pc;
  } watch_state_t;

  watch_state_t st_q [NumLanes];

  // A bus store matches a lane's watched address → potential wake.
  logic [NumLanes-1:0] addr_match, cond_met, do_wake;

  for (genvar l = 0; l < NumLanes; l++) begin : g_lanes
    // Address match: exact equality with the observed store address.
    assign addr_match[l] = st_q[l].active &
                           bus_req_i & bus_we_i &
                           (bus_addr_i == st_q[l].watch_addr);

    // Condition test against the current value (supplied on wake).
    always_comb begin
      unique case (st_q[l].funct7)
        ARF_WJ_EQ:  cond_met[l] = (wj_curval_i[l] == st_q[l].sentinel);
        ARF_WJ_NE:  cond_met[l] = (wj_curval_i[l] != st_q[l].sentinel);
        ARF_WJ_SET: cond_met[l] = ((wj_curval_i[l] & st_q[l].sentinel) != '0);
        default:    cond_met[l] = 1'b0;
      endcase
    end

    // Wake path: an address-matching store was observed; in the next cycle
    // the LSU provides the fresh value and we test the condition.
    assign do_wake[l] = st_q[l].active & wj_curval_valid_i[l] & cond_met[l];

    assign wj_active_o[l]    = st_q[l].active;
    assign wj_fired_o[l]     = do_wake[l];
    assign wj_pred_o[l]      = st_q[l].pred;
    assign wj_target_pc_o[l] = st_q[l].target_pc;

    // State machine: IDLE (inactive) -> WATCH (active) -> fire -> IDLE.
    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        st_q[l] <= '{active: 1'b0, pred: '0, funct7: '0,
                     watch_addr: '0, sentinel: '0, target_pc: '0};
      end else begin
        if (do_wake[l]) begin
          // Condition met: clear active; the controller sets the PC + predicate.
          st_q[l].active <= 1'b0;
        end else if (wj_issue_i[l] && !st_q[l].active) begin
          // Arm a new watch.
          st_q[l].active     <= 1'b1;
          st_q[l].pred       <= wj_pred_i[l];
          st_q[l].funct7     <= wj_funct7_i[l];
          st_q[l].watch_addr <= wj_watch_addr_i[l];
          st_q[l].sentinel   <= wj_sentinel_i[l];
          st_q[l].target_pc  <= wj_target_pc_i[l];
        end
      end
    end

    // Suppress unused-signal lint for addr_match (it is the trigger that
    // makes the LSU supply wj_curval_i; the actual condition uses curval).
    logic unused_addr_match;
    assign unused_addr_match = addr_match[l];
  end

endmodule
