// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// ibex_lane — one statically-dispatched execution lane (DESIGN.md §7.2/§7.3).
// Each lane executes the single instruction delivered to it from the bundle
// cache, in lockstep with the other W-1 lanes. A lane whose predicate guard
// is false commits nothing.
//
// The lane reuses ibex_alu for the arithmetic. The lane does NOT schedule —
// the dynarec has already ordered everything.
//
// Full v2 features:
//  - Predicate guard evaluated from the broadcast predicate file bits.
//  - ARF slot value supplied by the dispatcher's ARF crossbar (arf_rdata_i).
//  - Branch decision (from ALU comparison) + target (PC + imm, or rs1+imm)
//    reported back to the dispatcher for resolved-redirect.
//  - Effective address = ARF slot value for deref ops, ALU adder for lw/sw.

module ibex_lane
  import ibex_pkg::*;
  import ibex_bps_pkg::*;
#(
  parameter ibex_pkg::rv32b_e RV32B = ibex_pkg::RV32BNone
) (
  input  logic              clk_i,
  input  logic              rst_ni,

  // ---- Decoded control for this lane's current instruction ----
  input  logic              instr_valid_i,
  input  ibex_pkg::alu_op_e alu_operator_i,
  input  logic [31:0]       operand_a_i,    // rs1 value (from VLIW RF)
  input  logic [31:0]       operand_b_i,    // rs2 value (from VLIW RF)
  input  logic              rf_we_i,        // writeback enable (GPR)
  input  logic [4:0]        rf_waddr_i,     // GPR dest
  input  logic              is_load_i,      // this lane does a data-memory load
  input  logic              is_store_i,     // ... store

  // ---- Predicate guard ----
  input  logic [NUM_PRED-1:0] pred_bits_i,    // the broadcast predicate file
  input  logic [2:0]          pred_idx_i,     // this instr's predicate index
  input  logic                pred_invert_i,  // invert the guard
  input  logic                is_pred_set_i,  // this lane writes a predicate (cmp)
  input  logic [2:0]          pred_waddr_i,   // which predicate to write

  // ---- ARF port (phase A datapath) ----
  input  logic              arf_use_i,      // this lane touches the ARF this cycle
  input  logic              arf_we_i,       // ARF write (slotw/pina/ldp.next)
  input  logic [ARF_IDX_W-1:0] arf_idx_i,   // slot index
  input  logic [31:0]       arf_wdata_i,    // data to write to the slot
  input  logic [31:0]       arf_rdata_i,    // value read from the slot (from xbar)

  // ---- Data-memory request (banked LSU) ----
  output logic              data_req_o,
  output logic [31:0]       data_addr_o,
  output logic              data_we_o,
  output logic [31:0]       data_wdata_o,
  input  logic              data_gnt_i,
  input  logic              data_rvalid_i,
  input  logic [31:0]       data_rdata_i,

  // ---- Branch / jump resolution (resolved, non-speculative) ----
  input  logic              is_branch_i,    // this instr is a conditional branch
  input  logic              is_jump_i,      // this instr is JAL/JALR
  input  logic [31:0]       pc_i,           // this instr's PC (for target)
  input  logic [31:0]       imm_i,          // branch/jump immediate
  output logic              branch_redirect_o,   // this lane wants a redirect
  output logic [31:0]       branch_target_o,     // the resolved target

  // ---- Results to commit ----
  output logic              commit_valid_o,
  output logic [4:0]        commit_waddr_o,
  output logic [31:0]       commit_wdata_o,
  output logic              pred_we_o,
  output logic [2:0]        pred_waddr_commit_o,
  output logic              pred_wdata_commit_o,

  // ---- Busy (lane is waiting on a multi-cycle op) ----
  output logic              busy_o
);

  // -------------------------------------------------------------------------
  // Predicate guard.
  // -------------------------------------------------------------------------
  logic guard_raw, guard;
  assign guard_raw = pred_bits_i[pred_idx_i];
  assign guard     = pred_invert_i ? ~guard_raw : guard_raw;

  // -------------------------------------------------------------------------
  // ALU.
  // -------------------------------------------------------------------------
  logic [31:0] alu_result;
  logic [31:0] alu_adder_result;
  logic [31:0] imd_val_d [2];
  logic [1:0]  imd_val_we;
  logic [31:0] imd_val_q [2];
  logic        alu_cmp_result;
  logic        alu_is_equal;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      imd_val_q[0] <= '0;
      imd_val_q[1] <= '0;
    end else begin
      if (imd_val_we[0]) imd_val_q[0] <= imd_val_d[0];
      if (imd_val_we[1]) imd_val_q[1] <= imd_val_d[1];
    end
  end

  ibex_alu #(
    .RV32B(RV32B)
  ) u_alu (
    .operator_i         (alu_operator_i),
    .operand_a_i        (operand_a_i),
    .operand_b_i        (operand_b_i),
    .instr_first_cycle_i(1'b1),
    .imd_val_q_i        (imd_val_q),
    .imd_val_we_o       (imd_val_we),
    .imd_val_d_o        (imd_val_d),
    .multdiv_operand_a_i('0),
    .multdiv_operand_b_i('0),
    .multdiv_sel_i      (1'b0),
    .adder_result_o     (alu_adder_result),
    .adder_result_ext_o (),
    .result_o           (alu_result),
    .comparison_result_o(alu_cmp_result),
    .is_equal_result_o  (alu_is_equal)
  );

  // -------------------------------------------------------------------------
  // Effective address: ARF slot value for deref ops, ALU adder for native
  // lw/sw. (Both the dispatcher and the lane need to agree; the dispatcher
  // sets is_load_i/is_store_i and arf_use_i from the decode.)
  // -------------------------------------------------------------------------
  logic [31:0] eff_addr;
  assign eff_addr = arf_use_i ? arf_rdata_i : alu_adder_result;

  // -------------------------------------------------------------------------
  // Load/store: one-outstanding FSM.  ldp.next / ldp / ldpcap do a load whose
  // result writes back to the ARF slot (self-advance) and/or a GPR.
  // -------------------------------------------------------------------------
  typedef enum logic [1:0] {
    LS_IDLE,
    LS_WAIT_GNT,
    LS_WAIT_RVALID
  } ls_state_e;

  ls_state_e ls_state_q, ls_state_d;
  logic [31:0] ls_addr_q;
  logic [31:0] ls_wdata_q;
  logic        ls_we_q;

  always_comb begin
    ls_state_d = ls_state_q;
    data_req_o    = 1'b0;
    data_we_o     = 1'b0;
    data_addr_o   = '0;
    data_wdata_o  = '0;

    unique case (ls_state_q)
      LS_IDLE: begin
        if (instr_valid_i && guard && (is_load_i || is_store_i)) begin
          data_req_o   = 1'b1;
          data_we_o    = is_store_i;
          data_addr_o  = eff_addr;
          data_wdata_o = operand_b_i;
          if (data_gnt_i) begin
            ls_state_d = is_store_i ? LS_IDLE : LS_WAIT_RVALID;
          end else begin
            ls_state_d = LS_WAIT_GNT;
          end
        end
      end
      LS_WAIT_GNT: begin
        data_req_o   = 1'b1;
        data_we_o    = ls_we_q;
        data_addr_o  = ls_addr_q;
        data_wdata_o = ls_wdata_q;
        if (data_gnt_i) begin
          ls_state_d = ls_we_q ? LS_IDLE : LS_WAIT_RVALID;
        end
      end
      LS_WAIT_RVALID: begin
        if (data_rvalid_i) ls_state_d = LS_IDLE;
      end
      default: ls_state_d = LS_IDLE;
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ls_state_q <= LS_IDLE;
      ls_addr_q  <= '0;
      ls_wdata_q <= '0;
      ls_we_q    <= 1'b0;
    end else begin
      ls_state_q <= ls_state_d;
      if ((ls_state_q == LS_IDLE) && data_req_o) begin
        ls_addr_q  <= data_addr_o;
        ls_wdata_q <= data_wdata_o;
        ls_we_q    <= data_we_o;
      end
    end
  end

  assign busy_o = (ls_state_q != LS_IDLE);

  // -------------------------------------------------------------------------
  // Commit: GPR write from ALU result (non-load) or load data (load).
  // -------------------------------------------------------------------------
  logic load_complete;
  assign load_complete = (ls_state_q == LS_WAIT_RVALID) && data_rvalid_i;

  always_comb begin
    commit_valid_o  = 1'b0;
    commit_waddr_o  = rf_waddr_i;
    commit_wdata_o  = alu_result;
    if (instr_valid_i && rf_we_i && guard) begin
      if (is_load_i) begin
        commit_valid_o = load_complete;
        commit_wdata_o = data_rdata_i;
      end else begin
        commit_valid_o = 1'b1;
        commit_wdata_o = alu_result;
      end
    end

    // Predicate write from a compare (only when the lane is active).
    pred_we_o              = instr_valid_i & is_pred_set_i & guard;
    pred_waddr_commit_o    = pred_waddr_i;
    pred_wdata_commit_o    = alu_cmp_result;
  end

  // -------------------------------------------------------------------------
  // Branch / jump resolution (resolved, non-speculative — DESIGN.md §3.2).
  // A conditional branch redirects iff its guard holds AND the comparison is
  // true. JAL/JALR always redirect (guard is expected true). The target is
  // PC+imm for branches/JAL, rs1+imm for JALR.
  // -------------------------------------------------------------------------
  logic do_branch;
  assign do_branch = instr_valid_i & guard &
                     (is_branch_i ? alu_cmp_result : is_jump_i);

  assign branch_redirect_o = do_branch;
  assign branch_target_o   = is_jump_i ? (operand_a_i + imm_i)   // JALR: rs1+imm
                                       : (pc_i + imm_i);          // branch/JAL: PC+imm

endmodule
