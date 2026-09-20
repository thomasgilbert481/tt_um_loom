/*
 * loom_fifo_props: FIFO-1, FIFO-2, FIFO-3 (docs/VERIFICATION.md L4) on
 * loom_fifo alone.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Formal only. Bound into every loom_fifo instance (see the bind statement at
 * the bottom); never read by synthesis or by the TT flow.
 *
 * Every property cites the sentence of docs/SEMANTICS.md it encodes. The RTL
 * was read only for signal names (rp, wp, cnt, mem).
 *
 * Environment. SEMANTICS 6.7 puts the no-overflow / no-underflow duty on the
 * caller: "PUSH ra: done iff OUTQ_CNT[t] < FIFO_DEPTH as visible in X",
 * "POP rd: done iff INQ_CNT[t] > 0 as visible in X", "Host push to INQ[t]:
 * accepted iff INQ_CNT[t] < FIFO_DEPTH". Those three are assumed here and
 * asserted on the caller side by FIFO-1B in loom_sched_props.v, so the pair
 * is a complete assume-guarantee decomposition of FIFO-1.
 *
 * Written for DEPTH = 4 (the build parameter of loom_top), because the bind
 * has to name the memory words one by one.
 */

`default_nettype none

module loom_fifo_props #(
    parameter integer DEPTH = 4,
    parameter integer AW    = 2
) (
    input wire                clk,
    input wire                rst_n,
    input wire                clr,
    input wire                push,
    input wire [15:0]         wdata,
    input wire                pop,
    input wire [AW:0]         count,
    input wire [15:0]         head,
    input wire [15:0]         next,
    input wire [AW-1:0]       rp,
    input wire [AW-1:0]       wp,
    input wire [DEPTH*16-1:0] mem_flat
);

  // One edge must have passed before any state is meaningful: reset is
  // synchronous (SEMANTICS 1), so cycle 0 is constrained to hold rst_n low.
  reg f_past_valid = 1'b0;
  always @(posedge clk) f_past_valid <= 1'b1;
  always @(*) if (!f_past_valid) assume (!rst_n);

  // ------------------------------------------------- environment (6.7)
  always @(*) if (f_past_valid && rst_n) begin
    assume (!push || ({1'b0, count} < DEPTH));   // never push into a full FIFO
    assume (!pop  || (count != {(AW+1){1'b0}})); // never pop an empty one
  end

  // ==================================================================
  // FIFO-1: never overflows or underflows.
  // SEMANTICS 6.7: "occupancy counts INQ_CNT[t] and OUTQ_CNT[t] in
  // 0 .. FIFO_DEPTH".
  // ==================================================================
  always @(*) if (f_past_valid) assert ({1'b0, count} <= DEPTH);

  // Pointer / count consistency. Not a sentence of SEMANTICS by itself; it is
  // the invariant that makes FIFO-2 inductive and it is what "the count is the
  // occupancy" means for a ring buffer.
  wire [AW-1:0] f_wp_exp = rp + count[AW-1:0];
  always @(*) if (f_past_valid) assert (wp == f_wp_exp);

  // ==================================================================
  // FIFO-3: flags correct.
  // SEMANTICS 6.7: "A push or pop takes effect at an edge and is visible from
  // the following cycle" and "the new count is count + pushes - pops";
  // "CTRL.RESET of thread t also empties INQ[t] and OUTQ[t]"; "Counts reset
  // to 0".
  // ==================================================================
  always @(posedge clk) if (f_past_valid) begin
    if (!$past(rst_n) || $past(clr))
      assert (count == {(AW+1){1'b0}});
    else
      assert ({1'b0, count} == {1'b0, $past(count)}
                               + ($past(push) ? 1 : 0) - ($past(pop) ? 1 : 0));
  end

  // ==================================================================
  // FIFO-2: data out equals data in, in order (two-token method).
  // SEMANTICS 6.7: "POP rd: rd <= the head entry and the entry is removed at
  // the commit edge" and, for the host port, "the word loaded while the
  // previous word's pop is still in flight is the entry after the head ...
  // Nothing is lost or read twice."
  //
  // Two tokens: the entries living at two symbolic, distinct addresses. Each
  // token remembers the word pushed into it and which of the two was pushed
  // later. Proving, for every pair of addresses, that each entry comes back
  // out holding what was pushed into it and that the later push is never
  // delivered first, is "data out equals data in, in order".
  // ==================================================================
  // Symbolic constants: registers with no reset that only ever hold their own
  // value, so they keep an arbitrary value for the whole trace. (The
  // `anyconst` attribute is frontend-specific; this is not.)
  reg [AW-1:0] f_a1, f_a2;
  always @(posedge clk) begin
    f_a1 <= f_a1;
    f_a2 <= f_a2;
  end
  always @(*) assume (f_a1 != f_a2);

  reg        t1_v, t2_v;     // that address holds a tracked entry
  reg [15:0] t1_d, t2_d;     // the word that was pushed into it
  reg        t1_newer;       // token 1 was pushed after token 2

  wire f_set1 = push && (wp == f_a1);
  wire f_set2 = push && (wp == f_a2);

  always @(posedge clk) begin
    if (!rst_n || clr) begin
      t1_v <= 1'b0;
      t2_v <= 1'b0;
    end else begin
      if (f_set1) begin t1_v <= 1'b1; t1_d <= wdata; end
      else if (pop && (rp == f_a1)) t1_v <= 1'b0;
      if (f_set2) begin t2_v <= 1'b1; t2_d <= wdata; end
      else if (pop && (rp == f_a2)) t2_v <= 1'b0;
      // f_a1 != f_a2, so at most one of the two is written at any edge.
      if (f_set1)      t1_newer <= 1'b1;
      else if (f_set2) t1_newer <= 1'b0;
    end
  end

  // Distance from the read pointer, i.e. the token's position in the queue.
  wire [AW-1:0] f_d1 = f_a1 - rp;
  wire [AW-1:0] f_d2 = f_a2 - rp;
  wire [15:0]   f_m1 = mem_flat[f_a1*16 +: 16];
  wire [15:0]   f_m2 = mem_flat[f_a2*16 +: 16];
  wire [AW-1:0] f_rp1 = rp + {{(AW-1){1'b0}}, 1'b1};

  always @(*) if (f_past_valid) begin
    // A tracked entry still holds what was pushed into it, and is still
    // inside the occupied window (so it is neither lost nor overwritten).
    if (t1_v) begin
      assert (f_m1 == t1_d);
      assert ({1'b0, f_d1} < {1'b0, count});
    end
    if (t2_v) begin
      assert (f_m2 == t2_d);
      assert ({1'b0, f_d2} < {1'b0, count});
    end
    // In order: the entry pushed later is the further from the head, so it is
    // popped later. Delivery order is distance-from-rp order, because rp only
    // ever advances by one per pop.
    if (t1_v && t2_v) begin
      if (t1_newer) assert (f_d1 > f_d2);
      else          assert (f_d1 < f_d2);
    end
    // Data out equals data in: when a tracked entry reaches the head it reads
    // back as the value that was pushed.
    if (t1_v && (rp == f_a1)) assert (head == t1_d);
    if (t2_v && (rp == f_a2)) assert (head == t2_d);
    // The `next` port (the host pop path of 6.7) shows the entry after it.
    if (t1_v && (f_rp1 == f_a1) && ({1'b0, count} > 1)) assert (next == t1_d);
    if (t2_v && (f_rp1 == f_a2) && ({1'b0, count} > 1)) assert (next == t2_d);
  end

  // ------------------------------------------------- cover (anti-vacuity)
  // The assumptions above are strong enough to make everything unreachable if
  // they were wrong, so the cover task shows the interesting states are live.
  always @(*) if (f_past_valid && rst_n) begin
    cover ({1'b0, count} == DEPTH);                 // the FIFO fills
    cover (t1_v && (rp == f_a1));                   // a token reaches the head
    cover (t1_v && t2_v);                           // both tokens in flight
    cover (t1_v && t2_v && t1_newer);               // ... in either push order
    cover (t1_v && t2_v && !t1_newer);
    cover (push && pop);                            // simultaneous push and pop
    cover (clr && ({1'b0, count} != 0));            // CTRL.RESET empties it
  end

endmodule

// Bound into every loom_fifo instance. The port expressions are elaborated in
// the scope of the instance, which is how the properties reach rp, wp and the
// entry array without a change to src/.
bind loom_fifo loom_fifo_props #(.DEPTH(4), .AW(2)) u_fifo_props (
    .clk(clk), .rst_n(rst_n), .clr(clr),
    .push(push), .wdata(wdata), .pop(pop),
    .count(count), .head(head), .next(next),
    .rp(rp), .wp(wp),
    .mem_flat({mem[3], mem[2], mem[1], mem[0]})
);

`default_nettype wire
