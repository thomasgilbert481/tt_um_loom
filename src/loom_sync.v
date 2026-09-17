/*
 * loom_sync: two flip-flop synchroniser for asynchronous pad inputs.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Timing contract (docs/SEMANTICS.md section 3):
 *   ff1 samples `d` at every rising edge; ff2 samples ff1 at every rising edge.
 *   `q` is ff2, so the value visible to logic during cycle x is the pad value
 *   sampled at edge x-1. Reset drives both stages to RESET_VAL.
 */

`default_nettype none

module loom_sync #(
    parameter integer WIDTH     = 1,
    parameter [0:0]   RESET_VAL = 1'b0
) (
    input  wire             clk,
    input  wire             rst_n,
    input  wire [WIDTH-1:0] d,
    output wire [WIDTH-1:0] q
);

  reg [WIDTH-1:0] ff1;
  reg [WIDTH-1:0] ff2;

  always @(posedge clk) begin
    if (!rst_n) begin
      ff1 <= {WIDTH{RESET_VAL}};
      ff2 <= {WIDTH{RESET_VAL}};
    end else begin
      ff1 <= d;
      ff2 <= ff1;
    end
  end

  assign q = ff2;

endmodule
