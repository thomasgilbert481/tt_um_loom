/*
 * loom_wait_props: WAIT-1 (docs/VERIFICATION.md L4) on loom_core.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Formal only. Bound into loom_core (bind statement at the bottom); never read
 * by synthesis or by the TT flow.
 *
 * WAIT-1 as L4 words it: "a timed wait terminates within TD-NOW+2 slots
 * (bounded liveness)". A liveness claim with an explicit bound is a safety
 * property, so it is checked directly, with a monitor per thread that counts
 * the thread's X cycles from the first issue of a `WAITD` and asserts the
 * count never passes the bound.
 *
 * Three groups:
 *
 *   WAIT-1A  the completion rule itself, from SEMANTICS 6.4:
 *            "`WAITD imm8`: on first issue `TD' = TD + imm8` (mod 2^16),
 *            otherwise `TD' = TD`. Commit `TD <= TD'`. `done` iff
 *            `reached(NOW, TD')`."
 *            and "A wait-class instruction that does not complete commits
 *            `WAIT_ACTIVE <= 1` and leaves `PC` unchanged; when it completes
 *            it commits `WAIT_ACTIVE <= 0` and `PC <= next`."
 *            These make the wait *re-issue the same instruction against an
 *            unchanged deadline*, which is what turns the timer's progress
 *            into the wait's progress.
 *   WAIT-1B  the bound, with the tick period at its reset value
 *            (SEMANTICS 5: "`TICK_INT, TICK_FRAC ...` Reset 1, 0"), so
 *            `NOW` advances once per clock and four times per slot.
 *   WAIT-1C  the same bound with the tick period free. It is false, and
 *            `wait.sby:xfail` reproduces the counterexample: the bound is
 *            `TD - NOW` *ticks*, and a tick is `TICK_INT + TICK_FRAC/256`
 *            clocks, so counting it in slots is only right while a tick is
 *            at most one slot (4 clocks).
 *
 * `WAIT_TICK1` selects B (1, the default) or C (0). `WAIT_D0MAX` bounds the
 * deadline distance the monitor arms on, so the BMC depth stays small.
 *
 * Nothing here is copied from the RTL; the RTL was read only for signal
 * names, as in loom_sched_props.v.
 */

`ifndef WAIT_TICK1
`define WAIT_TICK1 1
`endif
`ifndef WAIT_D0MAX
`define WAIT_D0MAX 8
`endif
`ifndef WAIT_BOUND
`define WAIT_BOUND 1
`endif

`default_nettype none

module loom_wait_props #(
    parameter integer TICK1 = `WAIT_TICK1,
    parameter integer D0MAX = `WAIT_D0MAX,
    parameter integer BOUND = `WAIT_BOUND
) (
    input wire        clk,
    input wire        rst_n,

    // The slot in X.
    input wire        vx,
    input wire [1:0]  tx_th,
    input wire [9:0]  pcx,
    input wire [15:0] irx,
    input wire        bad_op,
    input wire        dec_ok,       // an instruction is really in X (6.11: not an LD/ST completion)
    input wire        is_waitd,
    input wire        wait_class,
    input wire        first_issue,
    input wire [15:0] x_now,
    input wire [15:0] x_td,
    input wire [15:0] td_new,
    input wire        reach_td_new,
    input wire        x_done,
    input wire        x_stall,
    input wire [9:0]  x_next_pc,

    // Per-thread state.
    input wire [3:0]  wa_all,
    input wire [63:0] td_all,
    input wire [63:0] now_all,
    input wire [63:0] tick_int_all,
    input wire [31:0] tick_frac_all,
    input wire [3:0]  run_r,
    input wire [3:0]  step_req_r,
    input wire [3:0]  thread_busy,

    // Host port (legality, SEMANTICS 7, and the monitor's quiet-host window).
    input wire [3:0]  h_reset,
    input wire        h_dbg_req,
    input wire        h_dbg_wr,
    input wire [1:0]  h_dbg_thread,
    input wire [3:0]  h_dbg_sel
);

  reg f_past_valid = 1'b0;
  always @(posedge clk) f_past_valid <= 1'b1;
  always @(*) if (!f_past_valid) assume (!rst_n);

  // ------------------------------------------------- host legality (SEM 7)
  //   "The host must only reset a halted thread; resetting a running thread
  //   is undefined."
  //   "Thread t is halted (for debug access) iff RUN[t] == 0, STEP_REQ[t] == 0
  //   and no valid slot of thread t is in F, D, X or W."
  // `thread_busy` is the RTL's own vector; SCHED-1 (proved) shows it is
  // exactly SEMANTICS 7's "not halted".
  always @(*) if (f_past_valid && rst_n) begin
    assume ((h_reset & ~thread_busy) == h_reset);
    if (h_dbg_req && h_dbg_wr) assume (!thread_busy[h_dbg_thread]);
    // D-031: the debug thread select is one-hot, as loom_host_ctl makes it
    // (reset 0001, every load 0001 << addr[9:8]). ($onehot written out:
    // the slang frontend does not have it.)
    assume (h_dbg_sel != 4'd0 && (h_dbg_sel & (h_dbg_sel - 4'd1)) == 4'd0);
  end

  // `dec_ok`, not `!bad_op`: since slice B the completion slot of an LD/ST
  // carries a data word that decodes to nothing (SEMANTICS 6.11), and
  // `bad_op` is low there because it is qualified by `xins` itself.
  wire x_wd = vx && is_waitd && dec_ok;

  // ==================================================================
  // WAIT-1A. SEMANTICS 6.4, the three sentences quoted in the header.
  // ==================================================================
  always @(*) if (f_past_valid && rst_n) begin
    // "done iff reached(NOW, TD')"
    if (x_wd) assert (x_done == reach_td_new);
    // "otherwise TD' = TD": a re-issue does not move the deadline, so the
    // deadline the wait is racing is fixed from its first issue.
    if (x_wd && !first_issue) assert (td_new == x_td);
    // "A wait-class instruction that does not complete ... leaves PC
    // unchanged; when it completes it commits PC <= next."
    if (vx && wait_class && x_stall) assert (x_next_pc == pcx);
  end

  // ==================================================================
  // WAIT-1B / WAIT-1C. One monitor per thread.
  //
  // `act` is high from the edge at which a first-issue WAITD of thread t
  // stalls until the edge at which that same WAITD completes. `slots` counts
  // the thread's X cycles from the first issue inclusive, and `d0` is
  // (TD' - NOW) as the first issue saw it: the "TD - NOW" of the L4 wording.
  //
  // The window is closed if the host touches the thread (CTRL.RESET, a debug
  // write) or stops running it, because a wait cannot make progress in a
  // thread the host has parked, and SEMANTICS 7 lets the host clear
  // WAIT_ACTIVE and PC under it.
  // ==================================================================
  genvar t;
  generate
    for (t = 0; t < 4; t = t + 1) begin : g_thread

      wire [15:0] s_tint  = tick_int_all[16*t +: 16];
      wire [7:0]  s_tfrac = tick_frac_all[8*t +: 8];

      // SEMANTICS 5: "TICK_INT, TICK_FRAC, OUTGRP, INGRP | 16, 8, 10, 10 |
      // 1, 0, 0, 0". At the reset period one tick is one clock, so NOW
      // advances four times per slot, which is the reading of "TD-NOW+2
      // slots" that is true. WAIT-1C drops this.
      always @(*) if (f_past_valid && rst_n && (TICK1 != 0))
        assume ((s_tint == 16'd1) && (s_tfrac == 8'd0));

      wire x_mine  = vx && (tx_th == t[1:0]);
      wire x_wd_me = x_mine && is_waitd && dec_ok;
      wire host_t  = h_reset[t]
                     | (h_dbg_req & h_dbg_wr & (h_dbg_thread == t[1:0]));

      reg        act;
      reg [15:0] d0;
      reg [15:0] slots;
      reg [15:0] ir0;

      always @(posedge clk) begin
        if (!rst_n) begin
          act   <= 1'b0;
          d0    <= 16'd0;
          slots <= 16'd0;
          ir0   <= 16'd0;
        end else if (host_t || !run_r[t]) begin
          act   <= 1'b0;                      // window closed, not a failure
        end else if (x_wd_me && first_issue) begin
          act   <= ~reach_td_new;             // armed only if it stalls
          d0    <= td_new - x_now;            // the "TD - NOW" of the bound
          slots <= 16'd1;
          ir0   <= irx;
        end else if (act && x_mine) begin
          slots <= slots + 16'd1;
          if (x_wd_me && reach_td_new) act <= 1'b0;
        end
      end

      // The stalled wait re-issues *the same instruction*. `imem_rdata` is a
      // free input in this harness, as in sched.sby, so without this the
      // solver simply hands the re-issue a different word at the same PC and
      // the wait never retires. SEMANTICS 6.4 gives the PC ("leaves `PC`
      // unchanged", asserted above as WAIT-1A) and SEMANTICS 7 gives the
      // word: "Instruction memory is single-port. Host IMEM reads and writes
      // are valid only while `RUN == 0` and no step in flight", so the word
      // at that address cannot change under a running thread.
      always @(*) if (f_past_valid && rst_n && act && x_mine)
        assume (irx == ir0);

      // Only arm on deadlines inside the modelled window, so the BMC depth
      // stays small; `d0` itself is free inside it.
      always @(*) if (f_past_valid && rst_n && act)
        assume ({1'b0, d0} <= D0MAX);

      // WAIT-1: the wait retires within TD - NOW + 2 of the thread's slots.
      // `WAIT_BOUND = 0` drops it, leaving WAIT-1A alone: the bound is a
      // bounded-liveness claim and belongs to the `bmc` task, while WAIT-1A
      // is an ordinary safety property of the X stage that `abc pdr` closes
      // unbounded in seconds. With the bound in, `abc pdr` sticks at frame
      // 16 and k-induction at depth 12 fails its induction step (the
      // counter needs an invariant relating `slots` to `NOW - TD`, which
      // neither engine finds).
      always @(*) if (f_past_valid && rst_n && act && (BOUND != 0))
        assert ({1'b0, slots} <= ({1'b0, d0} + 17'd2));

      // ---------------------------------------------- cover (anti-vacuity)
      always @(posedge clk) if (f_past_valid && $past(rst_n)) begin
        cover (act);                                   // a wait is pending
        cover (act && x_wd_me && !first_issue);        // it re-issues
        cover ($past(act) && !act && $past(x_mine));   // and it retires
      end

    end
  endgenerate

  // ------------------------------------------------- cover (anti-vacuity)
  always @(posedge clk) if (f_past_valid && $past(rst_n)) begin
    cover (x_wd && first_issue && !reach_td_new);      // a WAITD that stalls
    cover (x_wd && !first_issue &&  reach_td_new);     // a WAITD that fires
    cover (vx && wait_class && x_stall && wa_all[tx_th]);
  end

  wire _unused = &{1'b0, td_all, now_all, step_req_r};

endmodule

// The port expressions are elaborated in the scope of the loom_core instance.
bind loom_core loom_wait_props u_wait_props (
    .clk(clk), .rst_n(rst_n),
    .vx(vx), .tx_th(tx_th), .pcx(pcx), .irx(irx), .bad_op(bad_op), .dec_ok(dec_ok),
    .is_waitd(is_waitd), .wait_class(wait_class), .first_issue(first_issue),
    .x_now(x_now), .x_td(x_td), .td_new(td_new),
    .reach_td_new(reach_td_new), .x_done(x_done), .x_stall(x_stall),
    .x_next_pc(x_next_pc),
    .wa_all(wa_all), .td_all(td_all_w), .now_all(now_all),
    .tick_int_all(tick_int_all), .tick_frac_all(tick_frac_all),
    .run_r(run_r), .step_req_r(step_req_r), .thread_busy(thread_busy),
    .h_reset(h_reset), .h_dbg_req(h_dbg_req), .h_dbg_wr(h_dbg_wr),
    .h_dbg_thread(h_dbg_thread), .h_dbg_sel(h_dbg_sel)
);

`default_nettype wire
