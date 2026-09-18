/*
 * loom_fifo: one 16-bit FIFO, DEPTH entries (docs/SEMANTICS.md 6.7).
 * SPDX-License-Identifier: Apache-2.0
 *
 * loom_core instantiates eight: INQ[t] (host pushes, thread pops) and
 * OUTQ[t] (thread pushes, host pops) for every thread t.
 *
 * Timing contract:
 *   `count`, `head` and `next` are register values (plus a read mux), so
 *   they are exactly what the FIFO holds during the current cycle.
 *   A `push` or `pop` high during a cycle takes effect at the edge that ends
 *   it: the entry is written at `wp`, the pointers move, and the count
 *   becomes count + push - pop. Both may be high in the same cycle. The
 *   callers never push into a full FIFO or pop an empty one (loom_core
 *   checks the count visible in the same cycle, or in the slot's X cycle,
 *   which can only have become more favourable since).
 *   `clr` (CTRL.RESET of the owning thread) empties the FIFO at that edge
 *   and wins over a push or pop in the same cycle.
 *   Reset clears the count and the pointers; the entries are not reset
 *   (SEMANTICS 5), so an entry is only ever read after it was pushed.
 *   `head` is the oldest entry (valid when count > 0); `next` is the one
 *   after it (valid when count > 1). The host port needs `next` to hand out
 *   the word that follows a pop committing at the same edge.
 *
 * Parameters: DEPTH, a power of two, at least 2; AW = log2(DEPTH).
 */

`default_nettype none

module loom_fifo #(
    parameter integer DEPTH = 4,
    parameter integer AW    = $clog2(DEPTH)
) (
    input  wire          clk,
    input  wire          rst_n,
    input  wire          clr,
    input  wire          push,
    input  wire [15:0]   wdata,
    input  wire          pop,
    output wire [AW:0]   count,
    output wire [15:0]   head,
    output wire [15:0]   next
);

  reg [15:0]   mem [0:DEPTH-1];
  reg [AW-1:0] rp, wp;
  reg [AW:0]   cnt;

  // Entry storage: no reset, written only by a push.
  always @(posedge clk) begin
    if (push) mem[wp] <= wdata;
  end

  wire [AW-1:0] rp1 = rp + {{(AW-1){1'b0}}, 1'b1};

  always @(posedge clk) begin
    if (!rst_n || clr) begin
      rp  <= {AW{1'b0}};
      wp  <= {AW{1'b0}};
      cnt <= {(AW+1){1'b0}};
    end else begin
      if (push) wp <= wp + {{(AW-1){1'b0}}, 1'b1};
      if (pop)  rp <= rp1;
      cnt <= cnt + {{AW{1'b0}}, push} - {{AW{1'b0}}, pop};
    end
  end

  assign count = cnt;
  assign head  = mem[rp];
  assign next  = mem[rp1];

endmodule
