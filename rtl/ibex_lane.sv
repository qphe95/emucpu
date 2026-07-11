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
// the dynarec has already ordered everything. For multi-cycle ops (loads,
// which have exposed Lmem latency) the lane asserts busy until the response
// returns; the dispatcher holds the bundle until all lanes are not busy.
//
// The lane has:
//  - one ALU (rv32 arithmetic; the decoder selected alu_operator)
//  - a predicate guard (from ibex_predicate, broadcast to all lanes)
//  - an ARF read/write port (for ldp/stp/slotr/slotw/pina/ldp.next)
//  - a single data-memory request port (loads/stores share the banked LSU)
//
// The LSU handshake here is deliberately simple: one outstanding load or
// store per lane. The dispatcher's bundle-completion gating (DESIGN.md §7.3)
// handles the wait.

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
  input  logic              pred_guard_i,   // 1 = lane active, 0 = commit nothing
  input  logic              is_pred_set_i,  // this lane writes a predicate (cmp)
  input  logic [2:0]        pred_waddr_i,   // which predicate to write

  // ---- ARF port (phase A datapath) ----
  input  logic              arf_use_i,      // this lane touches the ARF this cycle
  input  logic              arf_we_i,       // ARF write (slotw/pina/ldp.next)
  input  logic [ARF_IDX_W-1:0] arf_idx_i,   // slot index
  input  logic [31:0]       arf_wdata_i,    // data to write to the slot
  output logic [31:0]       arf_rdata_o,    // value read from the slot (for deref)

  // ---- Data-memory request (shared banked LSU) ----
  output logic              data_req_o,
  output logic [31:0]       data_addr_o,
  output logic              data_we_o,
  output logic [31:0]       data_wdata_o,
  input  logic              data_gnt_i,
  input  logic              data_rvalid_i,
  input  logic [31:0]       data_rdata_i,

  // ---- Results to commit ----
  output logic              commit_valid_o, // this lane has a valid GPR write
  output logic [4:0]        commit_waddr_o,
  output logic [31:0]       commit_wdata_o,
  output logic              pred_we_o,      // predicate write from this lane
  output logic [2:0]        pred_waddr_commit_o,
  output logic              pred_wdata_commit_o,

  // ---- Busy (lane is waiting on a multi-cycle op) ----
  output logic              busy_o
);

  // -------------------------------------------------------------------------
  // ALU (rv32 arithmetic). We reuse ibex_alu; multdiv not supported per lane
  // in this revision (mul/div go through a shared unit if needed — the
  // dynarec places them on a dedicated lane in a future phase).
  // -------------------------------------------------------------------------
  logic [31:0] alu_result;
  logic [31:0] alu_adder_result;
  logic [31:0] imd_val_d [2];
  logic [1:0]  imd_val_we;
  logic [31:0] imd_val_q [2];
  logic        alu_cmp_result;
  logic        alu_is_equal;

  // imd_val registers live in the lane (the ALU's intermediate values for
  // multicycle shift/extract). For single-cycle ALU ops these are unused.
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
  // ARF read (combinational from the lane's ARF port — the dispatcher wires
  // arf_idx_i to the ARF bank selected for this lane).
  // -------------------------------------------------------------------------
  // arf_rdata_o is the value stored in the slot; used as the effective
  // address for deref ops (ldp/stp).
  assign arf_rdata_o = arf_wdata_i; // placeholder: the dispatcher feeds the
                                    // bank read data back here via arf_wdata_i
                                    // naming is misleading; see dispatcher.

  // -------------------------------------------------------------------------
  // Load/store: simple one-outstanding FSM. The effective address for a deref
  // op is the ARF slot value; for a native lw/sw it is the ALU adder result.
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
  logic [4:0]  ls_waddr_q;
  logic [31:0] ls_rdata;
  logic        ls_done;

  always_comb begin
    ls_state_d = ls_state_q;
    data_req_o    = 1'b0;
    data_we_o     = 1'b0;
    data_addr_o   = '0;
    data_wdata_o  = '0;
    ls_done       = 1'b0;

    unique case (ls_state_q)
      LS_IDLE: begin
        if (instr_valid_i && pred_guard_i && (is_load_i || is_store_i)) begin
          data_req_o   = 1'b1;
          data_we_o    = is_store_i;
          data_addr_o  = arf_use_i ? arf_wdata_i : alu_adder_result;
          data_wdata_o = operand_b_i;
          if (data_gnt_i) begin
            ls_state_d = is_store_i ? LS_IDLE : LS_WAIT_RVALID;
            if (is_store_i) ls_done = 1'b1;
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
          if (ls_we_q) ls_done = 1'b1;
        end
      end
      LS_WAIT_RVALID: begin
        if (data_rvalid_i) begin
          ls_state_d = LS_IDLE;
          ls_done    = 1'b1;
        end
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
      ls_waddr_q <= '0;
    end else begin
      ls_state_q <= ls_state_d;
      // capture request params when first issued
      if ((ls_state_q == LS_IDLE) && data_req_o) begin
        ls_addr_q  <= data_addr_o;
        ls_wdata_q <= data_wdata_o;
        ls_we_q    <= data_we_o;
        ls_waddr_q <= rf_waddr_i;
      end
    end
  end

  assign ls_rdata = data_rdata_i;
  assign busy_o   = (ls_state_q != LS_IDLE);

  // -------------------------------------------------------------------------
  // Commit: GPR write from ALU result (non-load) or load data (load).
  // Predicated — a false guard commits nothing.
  // -------------------------------------------------------------------------
  logic load_complete;
  assign load_complete = (ls_state_q == LS_WAIT_RVALID) && data_rvalid_i;

  always_comb begin
    commit_valid_o  = 1'b0;
    commit_waddr_o  = rf_waddr_i;
    commit_wdata_o  = alu_result;
    if (instr_valid_i && rf_we_i && pred_guard_i) begin
      if (is_load_i) begin
        // load result commits when the response arrives
        commit_valid_o = load_complete;
        commit_wdata_o = ls_rdata;
      end else begin
        commit_valid_o = 1'b1;
        commit_wdata_o = alu_result;
      end
    end

    // Predicate write from a compare (only when the lane is active).
    pred_we_o              = instr_valid_i & is_pred_set_i & pred_guard_i;
    pred_waddr_commit_o    = pred_waddr_i;
    pred_wdata_commit_o    = alu_cmp_result;
  end

endmodule
