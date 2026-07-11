// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// ibex_hot_addr_detect — the runtime hot-address observer (DESIGN.md §8).
// Snoops the data memory bus (the same data_* signals the LSU drives) for
// loads, hashes the address, and increments a saturating counter in a small
// table. When a counter crosses HOT_THRESH, the address is pushed to a
// recommendation FIFO that the dynarec drains via spflush/splr.
//
// This block NEVER mutates the ARF address map — it only *recommends*.
// Hardware does not commit pins; the dynarec does, keeping behavior
// deterministic (DESIGN.md §5.4).
//
// The hash is a simple XOR fold of the address so it maps 32-bit addresses
// into the 6-bit table index without an associative lookup.

module ibex_hot_addr_detect
  import ibex_bps_pkg::*;
#(
  parameter int unsigned TableEntries  = HOT_TABLE_ENTRIES, // 64
  parameter int unsigned CntWidth      = HOT_CNT_WIDTH,     // 4
  parameter int unsigned Threshold     = HOT_THRESH,        // 8
  parameter int unsigned RecqDepth     = HOT_RECQ_DEPTH     // 8
) (
  input  logic        clk_i,
  input  logic        rst_ni,

  // Snoop of the data memory bus (read-only observation; outputs to the bus
  // are NOT driven by this block). We watch committed loads.
  input  logic        data_req_i,
  input  logic        data_gnt_i,
  input  logic        data_we_i,      // ignore stores
  input  logic [31:0] data_addr_i,

  // Recommendation queue drain interface (dynarec side).
  input  logic        recq_pop_i,
  output logic        recq_valid_o,
  output logic [31:0] recq_addr_o
);

  // -------------------------------------------------------------------------
  // Hash: XOR-fold the address into log2(TableEntries) bits.
  // -------------------------------------------------------------------------
  localparam int unsigned IdxW = $clog2(TableEntries);
  logic [IdxW-1:0] hash_idx;
  always_comb begin
    // XOR the four IdxW-sized slices of data_addr_i together.
    hash_idx = '0;
    for (int s = 0; s < 32; s += IdxW) begin
      hash_idx ^= data_addr_i[s +: IdxW];
    end
  end

  // A load is "observed" at the address phase (req & gnt, not a store).
  logic load_observed;
  assign load_observed = data_req_i & data_gnt_i & ~data_we_i;

  // -------------------------------------------------------------------------
  // Counter table (flip-flops; small enough). Tag = high address bits.
  // -------------------------------------------------------------------------
  localparam int unsigned TagW = 32 - IdxW;
  logic [CntWidth-1:0]       cnt_q     [TableEntries];
  logic [TagW-1:0]           tag_q     [TableEntries];
  logic                      valid_q   [TableEntries];

  logic                      hit;
  logic [IdxW-1:0]           hit_idx;
  assign hit_idx = hash_idx;
  always_comb begin
    hit = valid_q[hash_idx] & (tag_q[hash_idx] == data_addr_i[31:31-TagW+1]);
  end

  // Promotion: when the post-increment counter reaches the threshold, push the
  // address to the recommendation FIFO and reset the counter so the entry can
  // re-fire if it stays hot.
  logic [CntWidth-1:0] new_cnt;
  always_comb begin
    if (load_observed && hit) begin
      new_cnt = cnt_q[hit_idx] + 1'b1;
    end else begin
      new_cnt = cnt_q[hit_idx];
    end
  end
  // Promote when the post-increment value reaches the threshold.
  logic promote_thresh;
  assign promote_thresh = load_observed & hit &
                          (cnt_q[hit_idx] == Threshold - 1);

  // -------------------------------------------------------------------------
  // Counter/tag update.
  // -------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int i = 0; i < TableEntries; i++) begin
        cnt_q[i]   <= '0;
        tag_q[i]   <= '0;
        valid_q[i] <= 1'b0;
      end
    end else if (load_observed) begin
      if (hit) begin
        cnt_q[hit_idx] <= promote_thresh ? '0 : new_cnt; // reset on promote
      end else begin
        // Miss: allocate this entry (evict). Simple policy: overwrite.
        valid_q[hit_idx] <= 1'b1;
        tag_q[hit_idx]   <= data_addr_i[31:31-TagW+1];
        cnt_q[hit_idx]   <= 1'b1;
      end
    end
  end

  // -------------------------------------------------------------------------
  // Recommendation FIFO. push on promote_thresh; pop on recq_pop_i.
  // Drop-oldest on overflow (these are hints; lossy is fine).
  // -------------------------------------------------------------------------
  logic [31:0] recq_mem [RecqDepth];
  logic [$clog2(RecqDepth)-1:0] recq_head_q, recq_tail_q;
  logic [$clog2(RecqDepth+1)-1:0] recq_count_q;
  logic recq_not_empty, recq_not_full;
  assign recq_not_empty = (recq_count_q != 0);
  assign recq_not_full  = (recq_count_q != RecqDepth);
  assign recq_valid_o   = recq_not_empty;
  assign recq_addr_o    = recq_mem[recq_head_q];

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      recq_head_q  <= '0;
      recq_tail_q  <= '0;
      recq_count_q <= '0;
    end else begin
      // push (drop-oldest if full: advance head as well)
      if (promote_thresh) begin
        recq_mem[recq_tail_q] <= {data_addr_i};
        recq_tail_q <= recq_tail_q + 1;
        if (!recq_not_full) begin
          recq_head_q <= recq_head_q + 1; // overwrite oldest
        end else begin
          recq_count_q <= recq_pop_i ? recq_count_q : (recq_count_q + 1);
        end
      end else if (recq_pop_i & recq_not_empty) begin
        recq_head_q <= recq_head_q + 1;
        recq_count_q <= recq_count_q - 1;
      end
    end
  end

endmodule
