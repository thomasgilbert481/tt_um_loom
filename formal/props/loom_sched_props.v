/*
 * loom_sched_props: SCHED-1, SCHED-2, SCHED-3 and FIFO-1B
 * (docs/VERIFICATION.md L4) on loom_core.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Formal only. Bound into loom_core (bind statement at the bottom); never read
 * by synthesis or by the TT flow.
 *
 * The harness is loom_core on its own, so `imem_rdata` is a free input: the
 * proof holds for every instruction stream, including words that no assembler
 * would emit. The host port is free too, except for the two legality
 * conditions of SEMANTICS 7 assumed below.
 *
 * This file carries its own model of the slot grid, built from the two
 * sentences of SEMANTICS that define it, and asserts the RTL's phase counter,
 * stage registers, commit rings and busy vector against it. Nothing in the
 * model is copied from the RTL; the RTL was read only for signal names.
 *
 *   SEMANTICS 1: "`ph` is a free-running 2-bit phase counter, 0 during cycle
 *   0, incrementing every cycle: ph(k) = k mod 4. It never stalls."
 *   SEMANTICS 2: "Thread t (0..3) owns the slot that starts in every cycle k
 *   with k mod 4 == t", the slot is "valid iff RUN[t] == 1 or
 *   STEP_REQ[t] == 1, both as visible in cycle k", and its four stages F, D,
 *   X, W occupy cycles k, k+1, k+2, k+3.
 */

`default_nettype none

module loom_sched_props #(
    parameter integer FIFO_DEPTH = 4,
    parameter integer FAW        = 2
) (
    input wire        clk,
    input wire        rst_n,

    // Slot grid as the RTL keeps it.
    input wire [1:0]  ph,
    input wire        vd,
    input wire [1:0]  td_th,
    input wire        vx,
    input wire [1:0]  tx_th,
    input wire        w_valid,
    input wire [1:0]  w_thread,
    input wire [3:0]  woh_pc,
    input wire [3:0]  woh_aux,
    input wire [3:0]  woh_rf,
    input wire [3:0]  woh_tmr,
    input wire [3:0]  woh_fifo,
    input wire [3:0]  woh_be,
    input wire [3:0]  woh_lat,
    input wire [3:0]  infl,
    input wire [3:0]  thread_busy,

    // Run control.
    input wire [3:0]  run_r,
    input wire [3:0]  step_req_r,

    // Per-thread architectural state named by SCHED-2.
    input wire [39:0] pc_all,
    input wire [3:0]  z_all,
    input wire [3:0]  c_all,
    input wire [3:0]  t_all,
    input wire [63:0] td_all,
    input wire [63:0] be_sr_all,
    input wire [19:0] be_cnt_all,
    input wire [63:0] be_crc_all,
    input wire [31:0] be_enc_all,     // slice A encoder state (6.9.1)
    // SCHED-4 (BUGS 6): the F stage's consumption of STEP_REQ wins over a
    // host STEP write on the same edge (SEMANTICS 7).
    input wire        valid_f,
    input wire        h_step_we,
    input wire [3:0]  h_step,
    input wire [3:0]  rf_we_oh,

    // Host port.
    input wire [3:0]  h_reset,
    input wire        h_dbg_req,
    input wire        h_dbg_wr,
    input wire [1:0]  h_dbg_thread,

    // FIFO ports, for FIFO-1B.
    input wire [4*(FAW+1)-1:0] inq_cnt_all,
    input wire [4*(FAW+1)-1:0] outq_cnt_all,
    input wire [3:0]  cw_fifo,
    input wire        w_push,
    input wire        w_pop,
    input wire [3:0]  h_push_ok,
    input wire [3:0]  h_outq_pop
);

  reg f_past_valid = 1'b0;
  always @(posedge clk) f_past_valid <= 1'b1;
  always @(*) if (!f_past_valid) assume (!rst_n);

  // ==================================================================
  // The slot grid, from SEMANTICS 1 and 2 (not from the RTL).
  // ==================================================================
  reg [1:0] f_ph;
  reg       f_vd, f_vx, f_vw;

  // SEMANTICS 2: "valid iff RUN[t] == 1 or STEP_REQ[t] == 1 ... in cycle k".
  wire f_vf = run_r[f_ph] | step_req_r[f_ph];

  always @(posedge clk) begin
    if (!rst_n) begin
      f_ph <= 2'd0;
      f_vd <= 1'b0;
      f_vx <= 1'b0;
      f_vw <= 1'b0;
    end else begin
      f_ph <= f_ph + 2'd1;
      f_vd <= f_vf;
      f_vx <= f_vd;
      f_vw <= f_vx;
    end
  end

  // The thread owning each stage in this cycle. A slot of thread t is in F in
  // cycle t mod 4, so it is in D one cycle later, X two and W three.
  wire [1:0] f_fth = f_ph;
  wire [1:0] f_dth = f_ph - 2'd1;
  wire [1:0] f_xth = f_ph - 2'd2;
  wire [1:0] f_wth = f_ph + 2'd1;          // -3 mod 4

  wire [3:0] f_oh_f = 4'd1 << f_fth;
  wire [3:0] f_oh_d = 4'd1 << f_dth;
  wire [3:0] f_oh_x = 4'd1 << f_xth;
  wire [3:0] f_oh_w = 4'd1 << f_wth;

  // "No valid slot of thread t is in F, D, X or W" (SEMANTICS 7).
  wire [3:0] f_inflight = ({4{f_vf}} & f_oh_f) | ({4{f_vd}} & f_oh_d)
                        | ({4{f_vx}} & f_oh_x) | ({4{f_vw}} & f_oh_w);
  // SEMANTICS 7: "Thread t is halted (for debug access) iff RUN[t] == 0,
  // STEP_REQ[t] == 0 and no valid slot of thread t is in F, D, X or W."
  wire [3:0] f_halted = ~run_r & ~step_req_r & ~f_inflight;

  // ------------------------------------------------- host legality (SEM 7)
  // "The host changes RUN, STEP_REQ, RESET, RESET_PC, SFLAGS, pin registers,
  // and (while the target thread is halted) debug state."
  // "The host must only reset a halted thread; resetting a running thread is
  // undefined."
  // Everything else on the host port stays free.
  always @(*) if (f_past_valid && rst_n) begin
    assume ((h_reset & ~f_halted) == 4'd0);
    if (h_dbg_req && h_dbg_wr) assume (f_halted[h_dbg_thread]);
  end

  // ==================================================================
  // SCHED-1: at every cycle exactly one thread is in each pipeline stage and
  // the four are distinct.
  // ==================================================================
  always @(posedge clk) if (f_past_valid && $past(rst_n)) begin
    // SEMANTICS 1: ph never stalls.
    assert (ph == $past(ph) + 2'd1);
  end

  always @(*) if (f_past_valid) begin
    assert (ph == f_ph);

    // The four stages hold four different threads: one-hot each, and together
    // they cover all four threads exactly once.
    assert ((f_oh_f | f_oh_d | f_oh_x | f_oh_w) == 4'b1111);

    // The RTL's stage thread registers agree with the grid whenever the stage
    // holds a slot. (They are don't-care for the bubbles the reset leaves in
    // the pipe, which the validity bits below cover.)
    if (vd)      assert (td_th == f_dth);
    if (vx)      assert (tx_th == f_xth);
    if (w_valid) assert (w_thread == f_wth);

    // Validity travels down the pipe exactly one stage per cycle.
    assert (vd == f_vd);
    assert (vx == f_vx);
    assert (w_valid == f_vw);

    // D-019: every commit ring is one-hot on the W thread, and they all agree
    // with each other. A ring that slipped would commit one thread's result
    // into another thread's state.
    assert (woh_pc   == f_oh_w);
    assert (woh_aux  == f_oh_w);
    assert (woh_rf   == f_oh_w);
    assert (woh_tmr  == f_oh_w);
    assert (woh_fifo == f_oh_w);
    assert (woh_be   == f_oh_w);
    assert (woh_lat  == f_oh_w);

    // SEMANTICS 7's "halted" is exactly what the RTL uses to gate debug
    // access: infl is D, X, W and thread_busy adds RUN, STEP_REQ and F.
    assert (infl == (({4{f_vd}} & f_oh_d) | ({4{f_vx}} & f_oh_x)
                     | ({4{f_vw}} & f_oh_w)));
    assert (thread_busy == ~f_halted);
  end

  // ==================================================================
  // SCHED-2: thread t's architectural state (regs, PC, flags, TD, SR, CNT,
  // CRC, and from slice A the encoder state of 6.9.1) changes only in its
  // own W stage.
  //
  // SEMANTICS 2: "All effects of the slot are registered at edge k+4"; the
  // slot in W during a cycle belongs to one thread, so no other thread's
  // state may move at that edge. SEMANTICS 7 adds the host as the only other
  // writer, and only while the thread is halted (assumed above).
  //
  // SCHED-3: a thread the host has neither run nor stepped changes nothing.
  // SEMANTICS 2 again: a slot needs RUN[t] or STEP_REQ[t] in its F cycle, and
  // F to W spans four cycles, so four quiet cycles mean no slot of t is
  // anywhere in the pipe.
  //
  // (The literal wording of VERIFICATION.md L4, "a thread with RUN=0 never
  // changes architectural state except through host debug writes", is only
  // true with SEMANTICS 7's definition of halted: a STEP with RUN=0 runs a
  // full slot and commits it. The cover statement at the bottom exhibits it.)
  // ==================================================================
  genvar t;
  generate
    for (t = 0; t < 4; t = t + 1) begin : g_thread

      wire f_commit = f_vw && (f_wth == t[1:0]);
      // ---- SCHED-4: a STEP_REQ consumed by thread t's F stage is clear in
      // the next cycle whatever the host wrote at that edge (SEMANTICS 7:
      // "A host STEP that commits on the same edge ... is lost"; BUGS 6).
      wire       f_consume = valid_f && (ph == t[1:0]) && step_req_r[t];
      reg        p_consume, p_host_step;
      always @(posedge clk) begin
        p_consume   <= f_consume;
        p_host_step <= h_step_we && h_step[t] && !run_r[t];
      end
      always @(posedge clk) if (f_past_valid && $past(rst_n))
        if (p_consume) assert (!step_req_r[t]);
      always @(posedge clk) if (f_past_valid && $past(rst_n))
        cover (p_consume && p_host_step);        // the coincidence itself
      wire f_host   = h_reset[t]
                      | (h_dbg_req & h_dbg_wr & (h_dbg_thread == t[1:0]));

      wire [9:0]  s_pc  = pc_all[10*t +: 10];
      wire [2:0]  s_fl  = {t_all[t], c_all[t], z_all[t]};
      wire [15:0] s_td  = td_all[16*t +: 16];
      wire [15:0] s_sr  = be_sr_all[16*t +: 16];
      wire [4:0]  s_cnt = be_cnt_all[5*t +: 5];
      wire [15:0] s_crc = be_crc_all[16*t +: 16];
      wire [7:0]  s_enc = be_enc_all[8*t +: 8];

      // Explicit delayed copies: the slang frontend only allows $past inside
      // a clocked block, and s_changed is wanted as a wire.
      reg [9:0]  p_pc;
      reg [2:0]  p_fl;
      reg [15:0] p_td, p_sr, p_crc;
      reg [4:0]  p_cnt;
      reg [7:0]  p_enc;
      always @(posedge clk) begin
        p_pc  <= s_pc;
        p_fl  <= s_fl;
        p_td  <= s_td;
        p_sr  <= s_sr;
        p_cnt <= s_cnt;
        p_crc <= s_crc;
        p_enc <= s_enc;
      end

      wire s_changed = (s_pc  != p_pc)  | (s_fl  != p_fl)
                     | (s_td  != p_td)  | (s_sr  != p_sr)
                     | (s_cnt != p_cnt) | (s_crc != p_crc)
                     | (s_enc != p_enc);

      // Four consecutive cycles with neither RUN nor STEP_REQ.
      wire      quiet_now = ~run_r[t] & ~step_req_r[t];
      reg [2:0] quiet_hist;
      always @(posedge clk) begin
        if (!rst_n) quiet_hist <= 3'd0;
        else        quiet_hist <= {quiet_hist[1:0], quiet_now};
      end
      wire quiet4 = quiet_now & (&quiet_hist);

      always @(posedge clk) if (f_past_valid && $past(rst_n)) begin
        // ---- SCHED-2
        if (s_changed) assert ($past(f_commit) | $past(f_host));
        // ---- SCHED-3
        if ($past(quiet4) && !$past(f_host)) begin
          assert (!s_changed);
          assert (!$past(rf_we_oh[t]));
        end
      end

      // The register file write port is the only way r0..r7 move.
      always @(*) if (f_past_valid && rst_n)
        if (rf_we_oh[t]) assert (f_commit | f_host);

      // ==============================================================
      // FIFO-1B: the caller side of FIFO-1. loom_fifo_props.v assumes the
      // caller never pushes into a full FIFO or pops an empty one
      // (SEMANTICS 6.7); this discharges that assumption.
      // ==============================================================
      wire [FAW:0] s_icnt = inq_cnt_all[(FAW+1)*t +: FAW+1];
      wire [FAW:0] s_ocnt = outq_cnt_all[(FAW+1)*t +: FAW+1];
      always @(*) if (f_past_valid && rst_n) begin
        if (h_push_ok[t])          assert ({1'b0, s_icnt} < FIFO_DEPTH);
        if (cw_fifo[t] && w_pop)   assert (s_icnt != {(FAW+1){1'b0}});
        if (cw_fifo[t] && w_push)  assert ({1'b0, s_ocnt} < FIFO_DEPTH);
      end

    end
  endgenerate

  // ------------------------------------------------- cover (anti-vacuity)
  wire [9:0] c_pc0 = pc_all[9:0];
  always @(posedge clk) if (f_past_valid && $past(rst_n)) begin
    cover (f_vf && f_vd && f_vx && f_vw);            // the pipe is full
    cover (c_pc0 != $past(c_pc0));                   // thread 0 actually runs
    // A stepped thread with RUN = 0 does commit: this is why SCHED-3 is
    // stated over quiet cycles and not over RUN alone.
    cover (!run_r[0] && !$past(run_r[0]) && (c_pc0 != $past(c_pc0)));
    cover (h_reset[0]);
    cover (h_dbg_req && h_dbg_wr);
  end

endmodule

// The port expressions are elaborated in the scope of the loom_core instance.
bind loom_core loom_sched_props #(.FIFO_DEPTH(4), .FAW(2)) u_sched_props (
    .clk(clk), .rst_n(rst_n),
    .ph(ph), .vd(vd), .td_th(td_th), .vx(vx), .tx_th(tx_th),
    .w_valid(w_valid), .w_thread(w_thread),
    .woh_pc(woh_pc), .woh_aux(woh_aux), .woh_rf(woh_rf), .woh_tmr(woh_tmr),
    .woh_fifo(woh_fifo), .woh_be(woh_be), .woh_lat(woh_lat),
    .infl(infl), .thread_busy(thread_busy),
    .run_r(run_r), .step_req_r(step_req_r),
    .pc_all(pc_all), .z_all(z_all), .c_all(c_all), .t_all(t_all),
    .td_all(td_all_w),
    .be_sr_all(be_sr_all), .be_cnt_all(be_cnt_all), .be_crc_all(be_crc_all),
    .be_enc_all(be_enc_all),
    .valid_f(valid_f), .h_step_we(h_step_we), .h_step(h_step),
    .rf_we_oh(rf_we_oh),
    .h_reset(h_reset), .h_dbg_req(h_dbg_req), .h_dbg_wr(h_dbg_wr),
    .h_dbg_thread(h_dbg_thread),
    .inq_cnt_all(inq_cnt_all), .outq_cnt_all(outq_cnt_all),
    .cw_fifo(cw_fifo), .w_push(w_push), .w_pop(w_pop),
    .h_push_ok(h_push_ok), .h_outq_pop(h_outq_pop)
);

`default_nettype wire
