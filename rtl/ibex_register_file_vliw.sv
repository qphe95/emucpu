// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// ibex_register_file_vliw — the integer data register file for the W-lane VLIW
// core (DESIGN.md §7.2). Unlike the scalar ibex_register_file_*, this file has
// W×2 read ports (rs1, rs2 per lane) and W write ports (one commit per lane).
//
// This is the DATA file only — addresses live in the ARF. GPR x0 is hardwired
// to zero. RV32E (16 regs) is supported via the DataRegWidth parameter.
//
// Implementation note: a behaviorally-described array with many read ports
// will be replicated / banked by the synthesizer. On an FPGA this maps to
// distributed RAM + LUTs (the read ports are pure combinational lookups); for
// W=32 this is a significant but bounded resource (see DESIGN.md §13).

module ibex_register_file_vliw #(
  parameter int unsigned Width       = 32, // number of lanes
  parameter int unsigned DataWidth   = 32,
  parameter bit           RV32E      = 1'b0
) (
  input  logic              clk_i,
  input  logic              rst_ni,

  // Read ports: 2 per lane (rs1, rs2). Combinational read.
  input  logic [Width-1:0][4:0] raddr_a_i,
  input  logic [Width-1:0][4:0] raddr_b_i,
  output logic [Width-1:0][DataWidth-1:0] rdata_a_o,
  output logic [Width-1:0][DataWidth-1:0] rdata_b_o,

  // Write ports: 1 per lane (commit). Synchronous write.
  input  logic [Width-1:0]          we_i,
  input  logic [Width-1:0][4:0]     waddr_i,
  input  logic [Width-1:0][DataWidth-1:0] wdata_i
);

  localparam int unsigned NumRegs = RV32E ? 16 : 32;

  logic [DataWidth-1:0] rf [NumRegs];

  // Zero-initialize for simulation.
  initial begin
    for (int i = 0; i < NumRegs; i++) rf[i] = '0;
  end

  // Synchronous writes.
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int i = 0; i < NumRegs; i++) rf[i] <= '0;
    end else begin
      for (int w = 0; w < Width; w++) begin
        if (we_i[w] && (waddr_i[w] != 5'd0)) begin
          rf[waddr_i[w]] <= wdata_i[w];
        end
      end
    end
  end

  // Combinational reads (x0 reads as zero).
  for (genvar l = 0; l < Width; l++) begin : g_reads
    assign rdata_a_o[l] = (raddr_a_i[l] == 5'd0) ? '0 : rf[raddr_a_i[l]];
    assign rdata_b_o[l] = (raddr_b_i[l] == 5'd0) ? '0 : rf[raddr_b_i[l]];
  end

endmodule
