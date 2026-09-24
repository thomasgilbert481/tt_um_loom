/*
 * loom_iso_miter: ISO-1, thread isolation as a two-copy miter
 * (docs/VERIFICATION.md L4, docs/ARCHITECTURE.md 1, D-001).
 * SPDX-License-Identifier: Apache-2.0
 *
 * Formal only; never read by synthesis or by the TT flow. The per-copy half
 * of the harness (thread T's state, the cover conditions and the two local
 * assumption groups) is `formal/props/loom_iso_props.v`, bound into each
 * `loom_core`. This file holds the two copies and the cross-copy assertions,
 * which have to be written where both copies are in scope.
 *
 * ISO-1: "for any two traces that agree on thread t's inputs (its pins, its
 * FIFOs, SFLAGS it waits on) thread t's state sequence is identical
 * regardless of what other threads do."
 *
 * ------------------------------------------------------------------ the model
 *
 * Two `loom_core` instances, A and B, each with its own `loom_pins`, on one
 * clock and one reset. `loom_imem`, `loom_host_ctl` and `loom_spi_host` are
 * outside the miter: the instruction word is a free input per copy, exactly
 * as in `sched.sby`, and the host port is free per copy. So the proof covers
 * every instruction stream, including words no assembler emits, and every
 * host command sequence the legality conditions of SEMANTICS 7 allow.
 *
 * ------------------------------------------- the hypothesis, built structurally
 *
 * The hypothesis of ISO-1 is not written as `assume`: it is built into the
 * wiring, so it cannot be vacuous and cannot be mis-stated as a constraint on
 * a state variable. Everything the two traces must agree on is *the same
 * wire*; everything else is two independent free inputs.
 *
 *   - the pads. `pad_in` is one 13-bit free input driving both copies:
 *     SEMANTICS 3, "`pin_in(i)` for index 0..12: the pad value sampled at
 *     edge x-1". The two traces see the same pads.
 *   - thread T's program words. `imem_rdata` is free in copy A; copy B takes
 *     copy A's word in every cycle whose D slot belongs to T and a free word
 *     otherwise (SEMANTICS 2, "D, k+1: IR = imem[PC[t]]"). This is weaker
 *     than a shared constant memory in the right direction: a free word per
 *     fetch over-approximates any memory image, so an assertion proved here
 *     holds for every memory image whose thread-T quarter is the same in the
 *     two traces. (It also means a *cover* trace need not correspond to one
 *     constant memory; see formal/README.md.)
 *   - host actions that target thread T: the RUN, STEP and RESET bits for T,
 *     the pushes into INQ[T] with their data, the pops of OUTQ[T] and the
 *     debug transactions addressed to T are copy A's, wired into copy B.
 *     The same bits for the other three threads are independent free inputs,
 *     as are their program words. SEMANTICS 7 lists these as the host's
 *     actions: "The host changes RUN, STEP_REQ, RESET, RESET_PC, SFLAGS, pin
 *     registers, and (while the target thread is halted) debug state."
 *   - host actions on shared state: the SFLAGS set/clear ports and the
 *     PIN_OUT / PIN_OE / OD_MASK writes are single free inputs driving both
 *     copies, as is the RESET_PC write port (only thread T's RESET_PC is
 *     load-bearing; sharing the port fixes the other three as well, which is
 *     an over-constraint recorded in formal/README.md).
 *
 * `ISO_DBG_FREE` selects what the host's debug port may do:
 *   0 - no debug transaction at all in either copy. This is the ISO-1 claim
 *       about the *threads*.
 *   2 - the whole debug port is the same in the two traces: the host's debug
 *       traffic is one of thread t's inputs, like its pins and its FIFOs.
 *   1 - debug transactions are free per copy, with only the transactions
 *       addressed to thread T identical. This one fails, and it is the
 *       finding: the debug port is a single shared resource whose occupancy
 *       depends on the other threads' host traffic. HOST_PROTOCOL's
 *       transaction format says as much already: "a DEBUG write of r0..r7
 *       may wait up to three more clocks for the register-file write port
 *       (the thread is halted, so it cannot tell)".
 *
 * The remaining hypothesis, that the other threads never write shared state,
 * is the ARCHITECTURE 1 precondition and is assumed in loom_iso_props.v with
 * the sentence it comes from.
 */

`ifndef ISO_T
`define ISO_T 0
`endif
`ifndef ISO_DBG_FREE
`define ISO_DBG_FREE 0
`endif

`default_nettype none

module loom_iso_miter #(
    parameter integer T          = `ISO_T,
    parameter integer IMEM_AW    = 9,
    parameter [15:0]  IMEM_WORDS = 16'd512,
    parameter integer FIFO_DEPTH = 4
) (
    input wire        clk,
    input wire        rst_n,

    // ---------------------------------------------- shared: the environment
    input wire [12:0] pad_in,          // {IN4, IN3..IN0, BIDIR7..0}

    // ------------------------ shared: host actions on shared state (SEM 7)
    input wire        h_sfset_we,
    input wire [7:0]  h_sfset,
    input wire        h_sfclr_we,
    input wire [7:0]  h_sfclr,
    input wire        h_pout_we,
    input wire [15:0] h_pout,
    input wire        h_poe_we,
    input wire [7:0]  h_poe,
    input wire        h_od_we,
    input wire [7:0]  h_od,
    input wire        h_rpc_we,
    input wire [1:0]  h_rpc_sel,
    input wire [9:0]  h_rpc,
    input wire        h_badop_clr_we,
    input wire [15:0] h_badop_clr,
    input wire        h_swirq_clr_we,
    input wire [3:0]  h_swirq_clr,
    // Strobe timing of the RUN and STEP writes: one write hits all four bits
    // at once (SEMANTICS 7, "CTRL.RUN write: RUN <= value"), so the strobe is
    // shared and only the value bits differ.
    input wire        h_run_we,
    input wire        h_step_we,

    // --------------------------------------------------- copy A free inputs
    input wire [15:0] imem_rdata_a,
    input wire [3:0]  h_run_a,
    input wire [3:0]  h_step_a,
    input wire [3:0]  h_reset_a,
    input wire [3:0]  h_inq_push_a,
    input wire [15:0] h_fifo_wdata_a,
    input wire [3:0]  h_outq_pop_a,
    input wire        h_badop_set14_a,
    input wire        h_badop_set15_a,
    input wire        h_dbg_req_a,
    input wire        h_dbg_wr_a,
    input wire [1:0]  h_dbg_thread_a,
    input wire [7:0]  h_dbg_reg_a,
    input wire [15:0] h_dbg_wdata_a,

    // ------------------------ copy B free inputs (the parts that may differ)
    input wire [15:0] imem_rdata_bf,
    input wire [3:0]  h_run_bf,
    input wire [3:0]  h_step_bf,
    input wire [3:0]  h_reset_bf,
    input wire [3:0]  h_inq_push_bf,
    input wire [15:0] h_fifo_wdata_bf,
    input wire [3:0]  h_outq_pop_bf,
    input wire        h_badop_set14_b,
    input wire        h_badop_set15_b,
    input wire        h_dbg_req_bf,
    input wire        h_dbg_wr_bf,
    input wire [1:0]  h_dbg_thread_bf,
    input wire [7:0]  h_dbg_reg_bf,
    input wire [15:0] h_dbg_wdata_bf
);

  localparam [1:0]  TT  = T[1:0];
  localparam [3:0]  TM  = 4'd1 << T;          // thread T's bit
  localparam integer OTH = (T + 1) % 4;       // a thread that is not T
  localparam integer FAW = 2;                 // $clog2(FIFO_DEPTH), depth 4
  localparam integer DBG = `ISO_DBG_FREE;

  reg f_past_valid = 1'b0;
  always @(posedge clk) f_past_valid <= 1'b1;
  always @(*) if (!f_past_valid) assume (!rst_n);

  // ================================================== the slot grid, locally
  // SEMANTICS 1: "`ph` is a free-running 2-bit phase counter, 0 during cycle
  // 0, incrementing every cycle". Kept here so the instruction-word wiring
  // below does not have to reach into either copy. Asserted against both.
  reg [1:0] f_ph;
  always @(posedge clk) begin
    if (!rst_n) f_ph <= 2'd0;
    else        f_ph <= f_ph + 2'd1;
  end
  // SEMANTICS 2: a slot of thread t is in F in cycle t mod 4, so in D one
  // cycle later.
  wire [1:0] f_dth = f_ph - 2'd1;
  wire       f_d_is_t = (f_dth == TT);

  // ================================================ the hypothesis, as wiring
  wire [15:0] imem_rdata_b  = f_d_is_t ? imem_rdata_a : imem_rdata_bf;

  wire [3:0]  h_run_b       = (h_run_bf       & ~TM) | (h_run_a       & TM);
  wire [3:0]  h_step_b      = (h_step_bf      & ~TM) | (h_step_a      & TM);
  wire [3:0]  h_reset_b     = (h_reset_bf     & ~TM) | (h_reset_a     & TM);
  wire [3:0]  h_inq_push_b  = (h_inq_push_bf  & ~TM) | (h_inq_push_a  & TM);
  wire [3:0]  h_outq_pop_b  = (h_outq_pop_bf  & ~TM) | (h_outq_pop_a  & TM);
  // One data register serves all four push ports (SEMANTICS 6.7), so it is
  // copy A's exactly when the push into INQ[T] is happening.
  wire [15:0] h_fifo_wdata_b = h_inq_push_a[T] ? h_fifo_wdata_a
                                               : h_fifo_wdata_bf;

  // The debug port. In mode 1 a transaction addressed to T is copy A's, in
  // the same cycle with the same register and data, and copy B's own free
  // transaction is suppressed whenever it would address T; the other
  // threads' traffic is independent. In mode 2 the whole port is copy A's,
  // so the host's debug traffic is the same in the two traces.
  // The thread stays a binary field here, as in the host's address; each
  // copy gets the one-hot select loom_host_ctl would make of it, 0001 <<
  // thread (D-031).
  wire        dbg_all        = (DBG == 2);
  wire        dbg_t_a        = h_dbg_req_a & (h_dbg_thread_a == TT);
  wire        h_dbg_req_b1   = dbg_t_a ? 1'b1
                                       : (h_dbg_req_bf & (h_dbg_thread_bf != TT));
  wire        h_dbg_req_b    = dbg_all ? h_dbg_req_a
                                       : h_dbg_req_b1;
  wire        h_dbg_wr_b     = (dbg_all | dbg_t_a) ? h_dbg_wr_a    : h_dbg_wr_bf;
  wire [1:0]  h_dbg_thread_b = dbg_all ? h_dbg_thread_a
                                       : (dbg_t_a ? TT : h_dbg_thread_bf);
  wire [7:0]  h_dbg_reg_b    = (dbg_all | dbg_t_a) ? h_dbg_reg_a   : h_dbg_reg_bf;
  wire [15:0] h_dbg_wdata_b  = (dbg_all | dbg_t_a) ? h_dbg_wdata_a : h_dbg_wdata_bf;

  wire        dbg_on   = (DBG != 0);
  wire        dreq_a   = h_dbg_req_a & dbg_on;
  wire        dreq_b   = h_dbg_req_b & dbg_on;


  // ============================================================== copy A
  wire [31:0] pin_in_vec_a;
  wire [15:0] pin_in_reg_a, pin_out_reg_a;
  wire [7:0]  pin_oe_reg_a, od_mask_reg_a;
  wire        cwv_a, cwod_a;
  wire [13:0] cwom_a, cwod14_a, lwom_a, lwod14_a;
  wire [7:0]  cwem_a, cwed_a, cwodv_a, lwem_a, lwed_a;
  wire [47:0] fstat_a;
  wire [63:0] oqh_a, oqn_a;
  wire        dack_a;
  wire [15:0] drd_a;
  wire [3:0]  run_a, halted_a, swirq_a;
  wire [15:0] badop_a;
  wire [7:0]  sflags_a;
  wire [39:0] rpc_a;
  wire        busy_a;
  wire        trv_a, trd_a, trw_a;
  wire [1:0]  trt_a;
  wire [9:0]  trpc_a, trnpc_a;
  wire [15:0] trir_a, trval_a;
  wire [2:0]  trrd_a, trfl_a;
  wire [IMEM_AW-1:0] iaddr_a;
  wire        ien_a, iwe_a;
  wire [15:0] iwd_a;
  wire [7:0]  uio_out_a, uio_oe_a;
  wire [5:0]  out_pins_a;

  loom_core #(.IMEM_AW(IMEM_AW), .IMEM_WORDS(IMEM_WORDS),
              .FIFO_DEPTH(FIFO_DEPTH)) u_a (
      .clk(clk), .rst_n(rst_n),
      .imem_addr(iaddr_a), .imem_en(ien_a), .imem_rdata(imem_rdata_a),
      .imem_we(iwe_a), .imem_wdata(iwd_a),
      .pin_in_vec(pin_in_vec_a), .pin_in_reg(pin_in_reg_a),
      .pin_out_reg(pin_out_reg_a), .pin_oe_reg(pin_oe_reg_a),
      .od_mask_reg(od_mask_reg_a),
      .cw_valid(cwv_a), .cw_out_mask(cwom_a), .cw_out_data(cwod14_a),
      .cw_oe_mask(cwem_a), .cw_oe_data(cwed_a),
      .cw_od_we(cwod_a), .cw_od(cwodv_a),
      .lw_out_mask(lwom_a), .lw_out_data(lwod14_a),
      .lw_oe_mask(lwem_a), .lw_oe_data(lwed_a),
      .h_run_we(h_run_we), .h_run(h_run_a), .h_reset(h_reset_a),
      .h_step_we(h_step_we), .h_step(h_step_a),
      .h_rpc_we(h_rpc_we), .h_rpc_sel(h_rpc_sel), .h_rpc(h_rpc),
      .h_sfset_we(h_sfset_we), .h_sfset(h_sfset),
      .h_sfclr_we(h_sfclr_we), .h_sfclr(h_sfclr),
      .h_badop_clr_we(h_badop_clr_we), .h_badop_clr(h_badop_clr),
      .h_badop_set15(h_badop_set15_a),
      .h_swirq_clr_we(h_swirq_clr_we), .h_swirq_clr(h_swirq_clr),
      .h_inq_push(h_inq_push_a), .h_fifo_wdata(h_fifo_wdata_a),
      .h_outq_pop(h_outq_pop_a), .h_badop_set14(h_badop_set14_a),
      .fifo_stat(fstat_a), .outq_head(oqh_a), .outq_next(oqn_a),
      .h_dbg_req(dreq_a), .h_dbg_wr(h_dbg_wr_a),
      .h_dbg_sel(4'b0001 << h_dbg_thread_a), .h_dbg_reg(h_dbg_reg_a),
      .h_dbg_wdata(h_dbg_wdata_a), .h_dbg_ack(dack_a), .h_dbg_rdata(drd_a),
      .run(run_a), .halted(halted_a), .badop(badop_a), .sflags(sflags_a),
      .swirq(swirq_a), .resetpc_all(rpc_a), .core_busy(busy_a),
      .tr_valid(trv_a), .tr_thread(trt_a), .tr_pc(trpc_a), .tr_ir(trir_a),
      .tr_done(trd_a), .tr_we(trw_a), .tr_rd(trrd_a), .tr_val(trval_a),
      .tr_flags(trfl_a), .tr_next_pc(trnpc_a)
  );

  loom_pins u_pina (
      .clk(clk), .rst_n(rst_n), .pad_in(pad_in),
      .cw_valid(cwv_a), .cw_out_mask(cwom_a), .cw_out_data(cwod14_a),
      .cw_oe_mask(cwem_a), .cw_oe_data(cwed_a),
      .cw_od_we(cwod_a), .cw_od(cwodv_a),
      .lw_out_mask(lwom_a), .lw_out_data(lwod14_a),
      .lw_oe_mask(lwem_a), .lw_oe_data(lwed_a),
      .h_out_we(h_pout_we), .h_out(h_pout),
      .h_oe_we(h_poe_we), .h_oe(h_poe),
      .h_od_we(h_od_we), .h_od(h_od),
      .pin_in_vec(pin_in_vec_a), .pin_in_reg(pin_in_reg_a),
      .pin_out_reg(pin_out_reg_a), .pin_oe_reg(pin_oe_reg_a),
      .od_mask_reg(od_mask_reg_a),
      .uio_out(uio_out_a), .uio_oe(uio_oe_a), .out_pins(out_pins_a)
  );

  // ============================================================== copy B
  wire [31:0] pin_in_vec_b;
  wire [15:0] pin_in_reg_b, pin_out_reg_b;
  wire [7:0]  pin_oe_reg_b, od_mask_reg_b;
  wire        cwv_b, cwod_b;
  wire [13:0] cwom_b, cwod14_b, lwom_b, lwod14_b;
  wire [7:0]  cwem_b, cwed_b, cwodv_b, lwem_b, lwed_b;
  wire [47:0] fstat_b;
  wire [63:0] oqh_b, oqn_b;
  wire        dack_b;
  wire [15:0] drd_b;
  wire [3:0]  run_b, halted_b, swirq_b;
  wire [15:0] badop_b;
  wire [7:0]  sflags_b;
  wire [39:0] rpc_b;
  wire        busy_b;
  wire        trv_b, trd_b, trw_b;
  wire [1:0]  trt_b;
  wire [9:0]  trpc_b, trnpc_b;
  wire [15:0] trir_b, trval_b;
  wire [2:0]  trrd_b, trfl_b;
  wire [IMEM_AW-1:0] iaddr_b;
  wire        ien_b, iwe_b;
  wire [15:0] iwd_b;
  wire [7:0]  uio_out_b, uio_oe_b;
  wire [5:0]  out_pins_b;

  loom_core #(.IMEM_AW(IMEM_AW), .IMEM_WORDS(IMEM_WORDS),
              .FIFO_DEPTH(FIFO_DEPTH)) u_b (
      .clk(clk), .rst_n(rst_n),
      .imem_addr(iaddr_b), .imem_en(ien_b), .imem_rdata(imem_rdata_b),
      .imem_we(iwe_b), .imem_wdata(iwd_b),
      .pin_in_vec(pin_in_vec_b), .pin_in_reg(pin_in_reg_b),
      .pin_out_reg(pin_out_reg_b), .pin_oe_reg(pin_oe_reg_b),
      .od_mask_reg(od_mask_reg_b),
      .cw_valid(cwv_b), .cw_out_mask(cwom_b), .cw_out_data(cwod14_b),
      .cw_oe_mask(cwem_b), .cw_oe_data(cwed_b),
      .cw_od_we(cwod_b), .cw_od(cwodv_b),
      .lw_out_mask(lwom_b), .lw_out_data(lwod14_b),
      .lw_oe_mask(lwem_b), .lw_oe_data(lwed_b),
      .h_run_we(h_run_we), .h_run(h_run_b), .h_reset(h_reset_b),
      .h_step_we(h_step_we), .h_step(h_step_b),
      .h_rpc_we(h_rpc_we), .h_rpc_sel(h_rpc_sel), .h_rpc(h_rpc),
      .h_sfset_we(h_sfset_we), .h_sfset(h_sfset),
      .h_sfclr_we(h_sfclr_we), .h_sfclr(h_sfclr),
      .h_badop_clr_we(h_badop_clr_we), .h_badop_clr(h_badop_clr),
      .h_badop_set15(h_badop_set15_b),
      .h_swirq_clr_we(h_swirq_clr_we), .h_swirq_clr(h_swirq_clr),
      .h_inq_push(h_inq_push_b), .h_fifo_wdata(h_fifo_wdata_b),
      .h_outq_pop(h_outq_pop_b), .h_badop_set14(h_badop_set14_b),
      .fifo_stat(fstat_b), .outq_head(oqh_b), .outq_next(oqn_b),
      .h_dbg_req(dreq_b), .h_dbg_wr(h_dbg_wr_b),
      .h_dbg_sel(4'b0001 << h_dbg_thread_b), .h_dbg_reg(h_dbg_reg_b),
      .h_dbg_wdata(h_dbg_wdata_b), .h_dbg_ack(dack_b), .h_dbg_rdata(drd_b),
      .run(run_b), .halted(halted_b), .badop(badop_b), .sflags(sflags_b),
      .swirq(swirq_b), .resetpc_all(rpc_b), .core_busy(busy_b),
      .tr_valid(trv_b), .tr_thread(trt_b), .tr_pc(trpc_b), .tr_ir(trir_b),
      .tr_done(trd_b), .tr_we(trw_b), .tr_rd(trrd_b), .tr_val(trval_b),
      .tr_flags(trfl_b), .tr_next_pc(trnpc_b)
  );

  loom_pins u_pinb (
      .clk(clk), .rst_n(rst_n), .pad_in(pad_in),
      .cw_valid(cwv_b), .cw_out_mask(cwom_b), .cw_out_data(cwod14_b),
      .cw_oe_mask(cwem_b), .cw_oe_data(cwed_b),
      .cw_od_we(cwod_b), .cw_od(cwodv_b),
      .lw_out_mask(lwom_b), .lw_out_data(lwod14_b),
      .lw_oe_mask(lwem_b), .lw_oe_data(lwed_b),
      .h_out_we(h_pout_we), .h_out(h_pout),
      .h_oe_we(h_poe_we), .h_oe(h_poe),
      .h_od_we(h_od_we), .h_od(h_od),
      .pin_in_vec(pin_in_vec_b), .pin_in_reg(pin_in_reg_b),
      .pin_out_reg(pin_out_reg_b), .pin_oe_reg(pin_oe_reg_b),
      .od_mask_reg(od_mask_reg_b),
      .uio_out(uio_out_b), .uio_oe(uio_oe_b), .out_pins(out_pins_b)
  );

  // ==================================================================
  // A well-formed host does not start a debug transaction in the cycle in
  // which the previous one is acknowledged. HOST_PROTOCOL's electrical
  // rules put at least eight clocks between two SCK edges, and "every
  // effect of the word is registered at edge E + 4", so `loom_host_ctl`
  // cannot present two debug commands in consecutive core clocks. It is not
  // gated on `rst_n`: the host obeys its protocol across a reset too, and
  // without that the solver simply drops `rst_n` for one cycle to switch
  // the assumption off. With `ISO_DBG_FREE = 0` both sides are constantly
  // false and the assumption is free.
  // ==================================================================
  always @(*) if (f_past_valid) begin
    assume (!(dreq_a && dack_a));
    assume (!(dreq_b && dack_b));
  end

  // ==================================================================
  // ISO-1. Every cycle, thread T's architectural state is the same in the
  // two copies. The `s_*` wires are named once, in loom_iso_props.v, from
  // SEMANTICS 5's table.
  // ==================================================================
  always @(*) if (f_past_valid) begin
    // The slot grid is the same in both copies and matches SEMANTICS 1.
    assert (u_a.ph == f_ph);
    assert (u_b.ph == f_ph);

    // ---- PC, flags, waits, return stack, PREV_PINS, groups, STEPS
    assert (u_a.u_view.s_pc     == u_b.u_view.s_pc);
    assert (u_a.u_view.s_flags  == u_b.u_view.s_flags);
    assert (u_a.u_view.s_wa     == u_b.u_view.s_wa);
    assert (u_a.u_view.s_rs0    == u_b.u_view.s_rs0);
    assert (u_a.u_view.s_rs1    == u_b.u_view.s_rs1);
    assert (u_a.u_view.s_depth  == u_b.u_view.s_depth);
    assert (u_a.u_view.s_pp     == u_b.u_view.s_pp);
    assert (u_a.u_view.s_outgrp == u_b.u_view.s_outgrp);
    assert (u_a.u_view.s_ingrp  == u_b.u_view.s_ingrp);
    assert (u_a.u_view.s_steps  == u_b.u_view.s_steps);
    assert (u_a.u_view.s_rpc    == u_b.u_view.s_rpc);

    // ---- run control and the per-thread status bits
    assert (u_a.u_view.s_run    == u_b.u_view.s_run);
    assert (u_a.u_view.s_halted == u_b.u_view.s_halted);
    assert (u_a.u_view.s_step   == u_b.u_view.s_step);
    assert (u_a.u_view.s_badop  == u_b.u_view.s_badop);
    assert (u_a.u_view.s_swirq  == u_b.u_view.s_swirq);

    // ---- the register file, thread T's eight words
    assert (u_a.u_view.s_r0 == u_b.u_view.s_r0);
    assert (u_a.u_view.s_r1 == u_b.u_view.s_r1);
    assert (u_a.u_view.s_r2 == u_b.u_view.s_r2);
    assert (u_a.u_view.s_r3 == u_b.u_view.s_r3);
    assert (u_a.u_view.s_r4 == u_b.u_view.s_r4);
    assert (u_a.u_view.s_r5 == u_b.u_view.s_r5);
    assert (u_a.u_view.s_r6 == u_b.u_view.s_r6);
    assert (u_a.u_view.s_r7 == u_b.u_view.s_r7);
    // and the write port as it reaches thread T
    assert (u_a.u_view.s_rfwe == u_b.u_view.s_rfwe);
    if (u_a.u_view.s_rfwe) begin
      assert (u_a.u_view.s_rfaddr == u_b.u_view.s_rfaddr);
      assert (u_a.u_view.s_rfdata == u_b.u_view.s_rfdata);
    end

    // ---- the timebase (SEMANTICS 4)
    assert (u_a.u_view.s_now     == u_b.u_view.s_now);
    assert (u_a.u_view.s_td      == u_b.u_view.s_td);
    assert (u_a.u_view.s_dt      == u_b.u_view.s_dt);
    assert (u_a.u_view.s_acc     == u_b.u_view.s_acc);
    assert (u_a.u_view.s_tseen   == u_b.u_view.s_tseen);
    assert (u_a.u_view.s_tint    == u_b.u_view.s_tint);
    assert (u_a.u_view.s_tfrac   == u_b.u_view.s_tfrac);
    assert (u_a.u_view.s_latfire == u_b.u_view.s_latfire);

    // ---- the bit engine (6.9, 6.9.1)
    assert (u_a.u_view.s_sr     == u_b.u_view.s_sr);
    assert (u_a.u_view.s_cnt    == u_b.u_view.s_cnt);
    assert (u_a.u_view.s_crc    == u_b.u_view.s_crc);
    assert (u_a.u_view.s_becfg  == u_b.u_view.s_becfg);
    assert (u_a.u_view.s_bepins == u_b.u_view.s_bepins);
    assert (u_a.u_view.s_reload == u_b.u_view.s_reload);
    assert (u_a.u_view.s_poly   == u_b.u_view.s_poly);
    assert (u_a.u_view.s_init   == u_b.u_view.s_init);
    assert (u_a.u_view.s_enc    == u_b.u_view.s_enc);

    // ---- the deadline latch (6.10)
    assert (u_a.u_view.s_latv   == u_b.u_view.s_latv);
    assert (u_a.u_view.s_latval == u_b.u_view.s_latval);
    assert (u_a.u_view.s_latpin == u_b.u_view.s_latpin);

    // ---- data memory (6.11, slice B): MEM and the held access, and
    // thread T's own use of the memory port in its F cycle, stores
    // included (formal/README.md, F-7)
    assert (u_a.u_view.s_mpend == u_b.u_view.s_mpend);
    assert (u_a.u_view.s_mld   == u_b.u_view.s_mld);
    assert (u_a.u_view.s_mrd   == u_b.u_view.s_mrd);
    assert (u_a.u_view.s_mwe   == u_b.u_view.s_mwe);
    assert (u_a.u_view.s_maddr == u_b.u_view.s_maddr);
    assert (u_a.u_view.s_mdata == u_b.u_view.s_mdata);
    assert (u_a.u_view.s_mp_en    == u_b.u_view.s_mp_en);
    assert (u_a.u_view.s_mp_we    == u_b.u_view.s_mp_we);
    assert (u_a.u_view.s_mp_addr  == u_b.u_view.s_mp_addr);
    assert (u_a.u_view.s_mp_wdata == u_b.u_view.s_mp_wdata);

    // ---- the FIFOs (6.7). Entry words are not reset, so the head and the
    // word after it are compared only while they hold something pushed.
    assert (u_a.u_view.s_icnt == u_b.u_view.s_icnt);
    assert (u_a.u_view.s_ocnt == u_b.u_view.s_ocnt);
    if (u_a.u_view.s_ivalid)  assert (u_a.u_view.s_ihead == u_b.u_view.s_ihead);
    if (u_a.u_view.s_ovalid)  assert (u_a.u_view.s_ohead == u_b.u_view.s_ohead);
    if (u_a.u_view.s_onvalid) assert (u_a.u_view.s_onext == u_b.u_view.s_onext);

    // ---- SFLAGS: shared by design, but under the ARCHITECTURE 1
    // precondition only thread T and the host write it, so it is asserted.
    assert (u_a.u_view.s_sflags == u_b.u_view.s_sflags);

    // ---- thread T's pin writes, and the shared pin registers and pads
    assert (u_a.u_view.s_pw_omask == u_b.u_view.s_pw_omask);
    assert (u_a.u_view.s_pw_odata == u_b.u_view.s_pw_odata);
    assert (u_a.u_view.s_pw_emask == u_b.u_view.s_pw_emask);
    assert (u_a.u_view.s_pw_edata == u_b.u_view.s_pw_edata);
    assert (u_a.u_view.s_pw_odwe  == u_b.u_view.s_pw_odwe);
    assert (u_a.u_view.s_pw_od    == u_b.u_view.s_pw_od);
    assert (u_a.u_view.s_lw_omask == u_b.u_view.s_lw_omask);
    assert (u_a.u_view.s_lw_odata == u_b.u_view.s_lw_odata);
    assert (u_a.u_view.s_lw_emask == u_b.u_view.s_lw_emask);
    assert (u_a.u_view.s_lw_edata == u_b.u_view.s_lw_edata);
    assert (u_pina.pin_out == u_pinb.pin_out);
    assert (u_pina.pin_oe  == u_pinb.pin_oe);
    assert (u_pina.od_mask == u_pinb.od_mask);
    assert (uio_out_a  == uio_out_b);
    assert (uio_oe_a   == uio_oe_b);
    assert (out_pins_a == out_pins_b);

    // ---- thread T's retire record (SEMANTICS 8)
    assert (u_a.u_view.s_tr_v == u_b.u_view.s_tr_v);
    if (u_a.u_view.s_tr_v) begin
      assert (u_a.u_view.s_tr_pc   == u_b.u_view.s_tr_pc);
      assert (u_a.u_view.s_tr_ir   == u_b.u_view.s_tr_ir);
      assert (u_a.u_view.s_tr_done == u_b.u_view.s_tr_done);
      assert (u_a.u_view.s_tr_we   == u_b.u_view.s_tr_we);
      assert (u_a.u_view.s_tr_fl   == u_b.u_view.s_tr_fl);
      assert (u_a.u_view.s_tr_npc  == u_b.u_view.s_tr_npc);
      // SEMANTICS 8: "`tr_we`, `tr_rd[2:0]`, `tr_val[15:0]`: register write,
      // *if any*". `tr_val` carries the INQ head for a POP whatever the
      // count is, and "entries are not reset" (SEMANTICS 5), so on a POP of
      // an empty INQ it is an entry word no push has written: free, and
      // free independently per copy. Unguarded, that is the one assertion
      // of this set that fails, at BMC step 8.
      if (u_a.u_view.s_tr_we) begin
        assert (u_a.u_view.s_tr_val == u_b.u_view.s_tr_val);
        assert (u_a.u_view.s_tr_rd  == u_b.u_view.s_tr_rd);
      end
    end

    // ---- the pipeline registers of thread T's slot in flight. Not part of
    // the statement of ISO-1; they localise a failure a cycle or three
    // before it reaches the architectural state.
    if (u_a.u_view.s_d_is_t) begin
      assert (u_a.u_view.s_vd == u_b.u_view.s_vd);
      if (u_a.u_view.s_vd) begin
        assert (u_a.u_view.s_pcd == u_b.u_view.s_pcd);
        assert (u_a.u_view.s_ird == u_b.u_view.s_ird);
      end
    end
    if (u_a.u_view.s_x_is_t) begin
      assert (u_a.u_view.s_vx == u_b.u_view.s_vx);
      if (u_a.u_view.s_vx) begin
        assert (u_a.u_view.s_pcx == u_b.u_view.s_pcx);
        assert (u_a.u_view.s_irx == u_b.u_view.s_irx);
        assert (u_a.u_view.s_opa == u_b.u_view.s_opa);
        assert (u_a.u_view.s_opb == u_b.u_view.s_opb);
      end
    end
    if (u_a.u_view.s_w_is_t) assert (u_a.u_view.s_vw == u_b.u_view.s_vw);
  end

  // ==================================================================
  // Cover (anti-vacuity). The point of the miter is the states in which the
  // other threads have actually diverged; a cover task that cannot reach
  // them would mean the hypothesis wiring had pinned the two copies
  // together (formal/README.md, "Two habits guard against vacuity").
  // ==================================================================
  wire [9:0] a_opc = u_a.pc_all[10*OTH +: 10];
  wire [9:0] b_opc = u_b.pc_all[10*OTH +: 10];
  wire f_oth_pc_diff = (a_opc != b_opc);
  wire f_oth_ir_diff = u_a.vx && u_b.vx && (u_a.tx_th == OTH[1:0])
                       && (u_a.irx != u_b.irx);
  wire f_oth_run_diff = (u_a.run_r[OTH] != u_b.run_r[OTH]);

  reg f_diverged;
  always @(posedge clk) begin
    if (!rst_n) f_diverged <= 1'b0;
    else        f_diverged <= f_diverged | f_oth_pc_diff | f_oth_ir_diff
                              | f_oth_run_diff;
  end

  reg [9:0] p_tpc;
  always @(posedge clk) p_tpc <= u_a.u_view.s_pc;
  wire f_t_moved = (u_a.u_view.s_pc != p_tpc);

  always @(posedge clk) if (f_past_valid && $past(rst_n)) begin
    // 1. the other thread's program counters have parted company
    cover (f_oth_pc_diff);
    // 2. the other thread decodes a different instruction in the two copies
    cover (f_oth_ir_diff);
    // 3. thread T runs on while the other thread has taken a different path
    cover (f_diverged && f_t_moved);
    // 4. thread T blocked on SFLAGS while the other thread has diverged
    cover (f_diverged && u_a.u_view.c_waits_wait);
    // 5. thread T's WAITS fires (the atomic test-and-clear of SEMANTICS 2)
    cover (f_diverged && u_a.u_view.c_waits_go);
    // 6. thread T commits a pin write while the other thread has diverged
    cover (f_diverged && u_a.u_view.c_t_pinw);
    // 7. a deadline-latched pin write of thread T applies (6.10)
    cover (u_a.u_view.c_t_latw);
    // 8. both pipes full, with the other thread diverged
    cover (f_diverged && u_a.u_view.c_pipe_full && u_b.u_view.c_pipe_full);
    // 9. thread T stalls on a wait while the other thread has diverged
    cover (f_diverged && u_a.u_view.c_t_stall);
    // 10. thread T's ST stores while the other thread has diverged
    cover (f_diverged && u_a.u_view.c_t_store);
    // 11. thread T's LD reads while the other thread has diverged
    cover (f_diverged && u_a.u_view.c_t_load);
  end

  // Lint sink: outputs the miter does not compare at this level (the
  // host-side views, which belong to loom_host_ctl; the memory port, which
  // the views compare in thread T's F cycle only).
  wire _unused = &{1'b0, fstat_a, fstat_b, oqh_a, oqh_b, oqn_a, oqn_b,
                   dack_a, dack_b, drd_a, drd_b, run_a, run_b,
                   halted_a, halted_b, badop_a, badop_b, sflags_a, sflags_b,
                   swirq_a, swirq_b, rpc_a, rpc_b, busy_a, busy_b,
                   trv_a, trv_b, trt_a, trt_b, trpc_a, trpc_b, trir_a, trir_b,
                   trd_a, trd_b, trw_a, trw_b, trrd_a, trrd_b,
                   trval_a, trval_b, trfl_a, trfl_b, trnpc_a, trnpc_b,
                   iaddr_a, iaddr_b, ien_a, ien_b, iwe_a, iwe_b, iwd_a, iwd_b,
                   pin_in_vec_a, pin_in_vec_b, pin_in_reg_a, pin_in_reg_b,
                   pin_out_reg_a, pin_out_reg_b, pin_oe_reg_a, pin_oe_reg_b,
                   od_mask_reg_a, od_mask_reg_b};

endmodule

`default_nettype wire
