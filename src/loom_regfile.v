/*
 * loom_regfile: 4 threads x 8 registers x 16 bits, two read ports, one write
 * port, all thread-indexed.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Timing contract:
 *   Reads are combinational: `rd_a` / `rd_b` follow {ra_thread, ra_addr} /
 *   {rb_thread, rb_addr} within the cycle. loom_core drives them in the D
 *   stage of a slot and registers the data at the edge that ends D.
 *   The write port is registered: when `we` is high the addressed register
 *   takes `wdata` at the next rising edge. loom_core drives it from the W
 *   stage, so a slot's register write lands at edge k+4 (SEMANTICS 2).
 *   Synchronous active-low reset clears all 32 registers to 0 (SEMANTICS 5).
 *   A read in the same cycle as a write returns the old value; loom_core
 *   never needs the new one because a thread's next slot starts after the
 *   write edge.
 */

`default_nettype none

module loom_regfile (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [1:0]  ra_thread,
    input  wire [2:0]  ra_addr,
    output wire [15:0] rd_a,
    input  wire [1:0]  rb_thread,
    input  wire [2:0]  rb_addr,
    output wire [15:0] rd_b,
    input  wire        we,
    input  wire [1:0]  w_thread,
    input  wire [2:0]  w_addr,
    input  wire [15:0] wdata
);

  reg [15:0] regs [0:31];

  integer i;
  always @(posedge clk) begin
    if (!rst_n) begin
      for (i = 0; i < 32; i = i + 1) regs[i] <= 16'd0;
    end else if (we) begin
      regs[{w_thread, w_addr}] <= wdata;
    end
  end

  assign rd_a = regs[{ra_thread, ra_addr}];
  assign rd_b = regs[{rb_thread, rb_addr}];

endmodule
