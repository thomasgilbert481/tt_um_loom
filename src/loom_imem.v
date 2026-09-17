/*
 * loom_imem: instruction memory, single port, FLOPS backend.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Timing contract:
 *   Synchronous read. The address presented during cycle k is registered at
 *   edge k+1 and `rdata` is valid during cycle k+1, which is the D stage of
 *   the slot that fetched in F (docs/SEMANTICS.md section 2).
 *   A write with `we` high takes effect at edge k+1. Read-during-write on the
 *   same address returns the old contents; loom_top never does that, because
 *   host access is only permitted while no slot is in flight.
 *   `en` gates both the read and the write, so a bubble slot leaves `rdata`
 *   unchanged. The array has no reset (SEMANTICS 5: "Instruction memory is
 *   not reset").
 *
 * M2: an SRAM macro backend replaces the body of this module with the same
 * ports (clk, en, we, addr, wdata, rdata) and the same one-cycle read
 * latency, so nothing above it changes.
 */

`default_nettype none

module loom_imem #(
    parameter [15:0]  WORDS = 16'd256,
    parameter integer AW    = 8
) (
    input  wire          clk,
    input  wire          en,
    input  wire          we,
    input  wire [AW-1:0] addr,
    input  wire [15:0]   wdata,
    output reg  [15:0]   rdata
);

  reg [15:0] mem [0:WORDS-1];

  always @(posedge clk) begin
    if (en) begin
      if (we) mem[addr] <= wdata;
      rdata <= mem[addr];
    end
  end

endmodule
