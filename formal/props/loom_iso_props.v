/*
 * loom_iso_props: the per-copy half of ISO-1, the thread-isolation miter
 * (docs/VERIFICATION.md L4, docs/ARCHITECTURE.md 1).
 * SPDX-License-Identifier: Apache-2.0
 *
 * Formal only. Bound into loom_core (bind statement at the bottom); never read
 * by synthesis or by the TT flow.
 *
 * ISO-1 is a relation between two traces, so it cannot be written as one
 * bound property module the way SCHED-1 or PIN-1 can. It is split in two:
 *
 *   - this file, `loom_iso_view`, bound into every loom_core instance. It
 *     names thread T's architectural state in one place (`s_*` wires, one per
 *     line of SEMANTICS 5), exposes the cover conditions (`c_*`), and carries
 *     the assumptions that are local to one copy: the two host legality
 *     conditions of SEMANTICS 7 and the ARCHITECTURE 1 precondition;
 *   - `formal/iso_miter.v`, which instantiates two `loom_core`s with two
 *     `loom_pins`, builds the hypothesis of ISO-1 structurally (see its
 *     header) and asserts `u_a.u_view.s_X == u_b.u_view.s_X` field by field.
 *
 * Both files are under `formal/`; nothing in `src/` changes. The comparison
 * lives in the wrapper because an assertion over two copies has to be written
 * where both are in scope.
 *
 * The thread under test is `ISO_T`. Re-run the group for another thread by
 * passing `-D ISO_T=<n>` to `read_slang`, or by editing the default below.
 */

`ifndef ISO_T
`define ISO_T 0
`endif

`default_nettype none

module loom_iso_view #(
    parameter integer T   = `ISO_T,
    parameter integer FAW = 2
) (
    input wire        clk,
    input wire        rst_n,

    // ---------------------------------------------------- the slot grid
    input wire [1:0]  ph,
    input wire        vd,
    input wire [1:0]  td_th,
    input wire [9:0]  pcd,
    input wire [15:0] ird,
    input wire        vx,
    input wire [1:0]  tx_th,
    input wire [9:0]  pcx,
    input wire [15:0] irx,
    input wire [15:0] op_a,
    input wire [15:0] op_b,
    input wire        w_valid,
    input wire [1:0]  w_thread,
    input wire [3:0]  thread_busy,

    // ------------------------- X-stage decode, for the ARCHITECTURE 1 gate
    input wire        is_setp,
    input wire        is_oep,
    input wire        is_out,
    input wire        is_sho,
    input wire        is_sig,
    input wire        is_clr,
    input wire        is_waits,
    input wire        is_waitd,
    input wire        is_csrw,
    input wire        csr_sflags,
    input wire        csr_pin_out,
    input wire        csr_pin_oe,
    input wire        csr_od_mask,
    input wire        csr_host_irq,
    input wire        cond_hit,
    input wire        x_stall,

    // ------------------------------------------- host port (legality, SEM 7)
    input wire [3:0]  h_reset,
    input wire        h_dbg_req,
    input wire        h_dbg_wr,
    input wire [1:0]  h_dbg_thread,
    input wire [7:0]  h_dbg_reg,

    // ------------------------------------- per-thread state (SEMANTICS 5)
    input wire [39:0] pc_all,
    input wire [3:0]  z_all,
    input wire [3:0]  c_all,
    input wire [3:0]  t_all,
    input wire [3:0]  wa_all,
    input wire [39:0] rs0_all,
    input wire [39:0] rs1_all,
    input wire [7:0]  depth_all,
    input wire [51:0] pp_all,
    input wire [39:0] outgrp_all,
    input wire [39:0] ingrp_all,
    input wire [63:0] steps_all,
    input wire [39:0] resetpc_r,
    input wire [3:0]  run_r,
    input wire [3:0]  halted_r,
    input wire [3:0]  step_req_r,
    input wire [15:0] badop_r,
    input wire [7:0]  sflags_r,
    input wire [3:0]  swirq_r,
    input wire [3:0]  lat_valid_all,
    input wire [3:0]  lat_val_all,
    input wire [19:0] lat_pin_all,

    // ------------------------------------------------------------- timer
    input wire [63:0] now_all,
    input wire [63:0] td_all,
    input wire [63:0] dt_all,
    input wire [63:0] tick_int_all,
    input wire [31:0] tick_frac_all,
    input wire [3:0]  tick_seen_all,
    input wire [23:0] acc,
    input wire [3:0]  lat_fire,

    // --------------------------------------------------------- bit engine
    input wire [63:0] be_sr_all,
    input wire [19:0] be_cnt_all,
    input wire [63:0] be_crc_all,
    input wire [31:0] be_cfg_all,
    input wire [39:0] be_pins_all,
    input wire [19:0] be_reload_all,
    input wire [63:0] be_poly_all,
    input wire [63:0] be_init_all,
    input wire [31:0] be_enc_all,

    // --------------------------------------------------------------- FIFOs
    input wire [4*(FAW+1)-1:0] inq_cnt_all,
    input wire [4*(FAW+1)-1:0] outq_cnt_all,
    input wire [63:0] inq_head_all,
    input wire [63:0] outq_head,
    input wire [63:0] outq_next,

    // -------------------------------------------- register file, thread T
    input wire [15:0] r0,
    input wire [15:0] r1,
    input wire [15:0] r2,
    input wire [15:0] r3,
    input wire [15:0] r4,
    input wire [15:0] r5,
    input wire [15:0] r6,
    input wire [15:0] r7,
    input wire [3:0]  rf_we_oh,
    input wire [2:0]  rf_w_addr,
    input wire [15:0] rf_wdata,

    // --------------------------------------------------------- pin writes
    input wire        cw_valid,
    input wire [13:0] cw_out_mask,
    input wire [13:0] cw_out_data,
    input wire [7:0]  cw_oe_mask,
    input wire [7:0]  cw_oe_data,
    input wire        cw_od_we,
    input wire [7:0]  cw_od,
    input wire [13:0] lw_out_mask,
    input wire [13:0] lw_out_data,
    input wire [7:0]  lw_oe_mask,
    input wire [7:0]  lw_oe_data,

    // ------------------------------------------------------ retire record
    input wire        tr_valid,
    input wire [1:0]  tr_thread,
    input wire [9:0]  tr_pc,
    input wire [15:0] tr_ir,
    input wire        tr_done,
    input wire        tr_we,
    input wire [2:0]  tr_rd,
    input wire [15:0] tr_val,
    input wire [2:0]  tr_flags,
    input wire [9:0]  tr_next_pc
);

  localparam [1:0] TT = T[1:0];

  reg f_past_valid = 1'b0;
  always @(posedge clk) f_past_valid <= 1'b1;
  always @(*) if (!f_past_valid) assume (!rst_n);

  // ==================================================================
  // Thread T's architectural state, one wire per line of SEMANTICS 5's
  // table plus the global registers T can reach. The miter compares these
  // between the two copies.
  // ==================================================================
  wire [9:0]  s_pc      = pc_all[10*T +: 10];
  wire [2:0]  s_flags   = {t_all[T], c_all[T], z_all[T]};     // {T, C, Z}
  wire        s_wa      = wa_all[T];                          // WAIT_ACTIVE
  wire [9:0]  s_rs0     = rs0_all[10*T +: 10];
  wire [9:0]  s_rs1     = rs1_all[10*T +: 10];
  wire [1:0]  s_depth   = depth_all[2*T +: 2];
  wire [12:0] s_pp      = pp_all[13*T +: 13];                 // PREV_PINS
  wire [9:0]  s_outgrp  = outgrp_all[10*T +: 10];
  wire [9:0]  s_ingrp   = ingrp_all[10*T +: 10];
  wire [15:0] s_steps   = steps_all[16*T +: 16];
  wire [9:0]  s_rpc     = resetpc_r[10*T +: 10];
  wire        s_run     = run_r[T];
  wire        s_halted  = halted_r[T];
  wire        s_step    = step_req_r[T];
  wire        s_badop   = badop_r[T];
  wire        s_swirq   = swirq_r[T];

  wire [15:0] s_now     = now_all[16*T +: 16];
  wire [15:0] s_td      = td_all[16*T +: 16];
  wire [15:0] s_dt      = dt_all[16*T +: 16];
  wire [15:0] s_tint    = tick_int_all[16*T +: 16];
  wire [7:0]  s_tfrac   = tick_frac_all[8*T +: 8];
  wire        s_tseen   = tick_seen_all[T];
  wire [23:0] s_acc     = acc;
  wire        s_latfire = lat_fire[T];

  wire [15:0] s_sr      = be_sr_all[16*T +: 16];
  wire [4:0]  s_cnt     = be_cnt_all[5*T +: 5];
  wire [15:0] s_crc     = be_crc_all[16*T +: 16];
  wire [7:0]  s_becfg   = be_cfg_all[8*T +: 8];
  wire [9:0]  s_bepins  = be_pins_all[10*T +: 10];
  wire [4:0]  s_reload  = be_reload_all[5*T +: 5];
  wire [15:0] s_poly    = be_poly_all[16*T +: 16];
  wire [15:0] s_init    = be_init_all[16*T +: 16];
  wire [7:0]  s_enc     = be_enc_all[8*T +: 8];               // 6.9.1

  wire        s_latv    = lat_valid_all[T];
  wire        s_latval  = lat_val_all[T];
  wire [4:0]  s_latpin  = lat_pin_all[5*T +: 5];

  wire [FAW:0] s_icnt   = inq_cnt_all[(FAW+1)*T +: FAW+1];
  wire [FAW:0] s_ocnt   = outq_cnt_all[(FAW+1)*T +: FAW+1];
  wire [15:0]  s_ihead  = inq_head_all[16*T +: 16];
  wire [15:0]  s_ohead  = outq_head[16*T +: 16];
  wire [15:0]  s_onext  = outq_next[16*T +: 16];
  // The FIFO entry words are not reset (SEMANTICS 5), so only the entries a
  // push has already written are meaningful; the heads are compared under
  // these guards.
  wire         s_ivalid = (s_icnt != {(FAW+1){1'b0}});
  wire         s_ovalid = (s_ocnt != {(FAW+1){1'b0}});
  wire         s_onvalid = ({1'b0, s_ocnt} > 1);

  wire [15:0] s_r0 = r0, s_r1 = r1, s_r2 = r2, s_r3 = r3;
  wire [15:0] s_r4 = r4, s_r5 = r5, s_r6 = r6, s_r7 = r7;

  // The register file's write stream as it reaches thread T (SEMANTICS 2:
  // a slot's register write lands at edge k+4). Compared as well as the
  // storage, because a mistimed write shows up here one cycle earlier.
  wire        s_rfwe   = rf_we_oh[T];
  wire [2:0]  s_rfaddr = rf_w_addr;
  wire [15:0] s_rfdata = rf_wdata;

  // ---------------------------------------------------- shared registers
  // SFLAGS and the pin registers are shared by design (SEMANTICS 2, "shared
  // state visibility rule"). Under the ARCHITECTURE 1 precondition assumed
  // below only thread T and the host write them, so the miter asserts them
  // equal rather than assuming it.
  wire [7:0]  s_sflags = sflags_r;

  // --------------------------------------------- thread T's own pin writes
  wire        s_pinw     = cw_valid && (w_thread == TT);
  wire [13:0] s_pw_omask = {14{s_pinw}} & cw_out_mask;
  wire [13:0] s_pw_odata = {14{s_pinw}} & cw_out_data;
  wire [7:0]  s_pw_emask = {8{s_pinw}} & cw_oe_mask;
  wire [7:0]  s_pw_edata = {8{s_pinw}} & cw_oe_data;
  wire        s_pw_odwe  = s_pinw & cw_od_we;
  wire [7:0]  s_pw_od    = {8{s_pinw}} & cw_od;
  // The staged (deadline-latched) write port carries only threads whose
  // LAT_VALID is set; under the precondition that is thread T alone.
  wire [13:0] s_lw_omask = lw_out_mask;
  wire [13:0] s_lw_odata = lw_out_data;
  wire [7:0]  s_lw_emask = lw_oe_mask;
  wire [7:0]  s_lw_edata = lw_oe_data;

  // ------------------------------------------------- the pipeline, thread T
  // Equal pipeline registers are not part of ISO-1's statement, but a
  // mismatch here localises a failure one to three cycles before it reaches
  // the architectural state.
  wire        s_d_is_t = (td_th == TT);
  wire        s_x_is_t = (tx_th == TT);
  wire        s_w_is_t = (w_thread == TT);
  wire        s_vd     = vd;
  wire        s_vx     = vx;
  wire        s_vw     = w_valid;
  wire [9:0]  s_pcd    = pcd;
  wire [15:0] s_ird    = ird;
  wire [9:0]  s_pcx    = pcx;
  wire [15:0] s_irx    = irx;
  wire [15:0] s_opa    = op_a;
  wire [15:0] s_opb    = op_b;

  // --------------------------------------------- thread T's retire record
  wire        s_tr_v    = tr_valid && (tr_thread == TT);
  wire [9:0]  s_tr_pc   = tr_pc;
  wire [15:0] s_tr_ir   = tr_ir;
  wire        s_tr_done = tr_done;
  wire        s_tr_we   = tr_we;
  wire [2:0]  s_tr_rd   = tr_rd;
  wire [15:0] s_tr_val  = tr_val;
  wire [2:0]  s_tr_fl   = tr_flags;
  wire [9:0]  s_tr_npc  = tr_next_pc;

  // ==================================================================
  // Assumption 1: the ARCHITECTURE 1 precondition.
  //
  //   ARCHITECTURE 1: "Four hardware threads share one 4-stage pipeline in
  //   strict round-robin ... there are no caches, no stalls except explicit
  //   waits, and no branch penalty, so the timing of any program can be read
  //   off the listing."
  //   SEMANTICS 2, shared state visibility rule: "An effect of a slot with X
  //   cycle x on shared state (PIN_OUT, PIN_OE, OD_MASK, global CSRs, pads)
  //   is visible to other slots from cycle x+2. Exception: effects on SFLAGS
  //   are forwarded from W to X and are visible from cycle x+1".
  //
  // The exception list is the whole content of the claim: threads are
  // isolated *except through explicit shared state*, so a thread that writes
  // shared state is outside the claim. The other three threads are therefore
  // assumed never to decode, in a valid slot, an instruction that writes
  // SFLAGS, a pin register or the host interrupt: SETP (ordinary or
  // deadline-latched, 6.10), OEP, OUT, SHO (6.9, which drives a pin and with
  // DIFF a second one), SIG, CLR, WAITS (a test-and-clear, 6.5) and
  // CSRW of SFLAGS / PIN_OUT / PIN_OE / OD_MASK / HOST_IRQ (6.6).
  // Nothing constrains thread T, and nothing constrains the other threads'
  // ALU, branch, FIFO, timer or bit-engine-register work.
  // ==================================================================
  wire x_writes_shared =
        is_setp | is_oep | is_out | is_sho | is_sig | is_clr | is_waits
      | (is_csrw & (csr_sflags | csr_pin_out | csr_pin_oe
                    | csr_od_mask | csr_host_irq));

  always @(*) if (f_past_valid && rst_n) begin
    if (vx && (tx_th != TT)) assume (!x_writes_shared);
  end

  // The same precondition on the host's side of the deadline latch. Debug
  // register 0x25 is {LAT_VALID, LAT_VAL, LAT_PIN} (6.10), so a host write
  // of it *stages a pin write* for the target thread, and the staged-write
  // port is shared by all four threads: "where two threads' staged writes
  // hit one pin at one edge the higher thread wins" (loom_core's header,
  // SEMANTICS 6.10). Writing another thread's 0x25 is therefore a write to
  // shared state, exactly like `CSRW PIN_OUT`, and unlike the other
  // shared-state host writes it cannot be handled by holding the command
  // equal in the two traces: the *edge* at which the staged write applies
  // is that thread's `lat_fire`, which diverges with the thread. So it is
  // excluded for the other three threads, as their own `SETP D` is.
  // With ISO_DBG_FREE = 0 the debug port is tied off and this is vacuous.
  always @(*) if (f_past_valid && rst_n) begin
    assume (!(h_dbg_req && h_dbg_wr && (h_dbg_thread != TT)
              && (h_dbg_reg == 8'h25)));
  end

  // ==================================================================
  // Assumption 2: the two host legality conditions of SEMANTICS 7, exactly
  // as loom_sched_props.v assumes them.
  //
  //   SEMANTICS 7: "The host must only reset a halted thread; resetting a
  //   running thread is undefined."
  //   SEMANTICS 7: "Thread t is halted (for debug access) iff RUN[t] == 0,
  //   STEP_REQ[t] == 0 and no valid slot of thread t is in F, D, X or W."
  //
  // `thread_busy` is the RTL's own vector; SCHED-1 (proved, unbounded) says
  // it is exactly `~halted` as SEMANTICS 7 defines it, which is what lets
  // this harness use it instead of rebuilding the slot grid.
  // ==================================================================
  always @(*) if (f_past_valid && rst_n) begin
    assume ((h_reset & thread_busy) == 4'd0);
    if (h_dbg_req && h_dbg_wr) assume (!thread_busy[h_dbg_thread]);
  end

  // ==================================================================
  // Cover conditions, exported for the miter's cover task. A miter that
  // cannot reach the interesting states proves nothing (formal/README.md).
  // ==================================================================
  wire c_x_t        = vx && (tx_th == TT);            // thread T has an X cycle
  wire c_x_other    = vx && (tx_th != TT);            // another thread does
  wire c_waits_wait = c_x_t && is_waits && !cond_hit; // T blocked on SFLAGS
  wire c_waits_go   = c_x_t && is_waits &&  cond_hit; // T's WAITS fires
  wire c_waitd      = c_x_t && is_waitd;
  wire c_t_stall    = c_x_t && x_stall;
  wire c_t_pinw     = cw_valid && (w_thread == TT)
                      && (|cw_out_mask || |cw_oe_mask || cw_od_we);
  wire c_t_latw     = |lw_out_mask;
  wire c_pipe_full  = vd && vx && w_valid;

endmodule

// The port expressions are elaborated in the scope of the loom_core instance,
// so the property module reads internal registers with no port added to the
// RTL. Yosys's own Verilog frontend silently drops `bind`; every .sby here
// reads the design through `plugin -i slang; read_slang` (formal/README.md).
bind loom_core loom_iso_view #(.T(`ISO_T), .FAW(2)) u_view (
    .clk(clk), .rst_n(rst_n),
    .ph(ph), .vd(vd), .td_th(td_th), .pcd(pcd), .ird(ird),
    .vx(vx), .tx_th(tx_th), .pcx(pcx), .irx(irx), .op_a(op_a), .op_b(op_b),
    .w_valid(w_valid), .w_thread(w_thread), .thread_busy(thread_busy),
    .is_setp(is_setp), .is_oep(is_oep), .is_out(is_out), .is_sho(is_sho),
    .is_sig(is_sig), .is_clr(is_clr), .is_waits(is_waits), .is_waitd(is_waitd),
    .is_csrw(is_csrw), .csr_sflags(csr_sflags), .csr_pin_out(csr_pin_out),
    .csr_pin_oe(csr_pin_oe), .csr_od_mask(csr_od_mask),
    .csr_host_irq(csr_host_irq), .cond_hit(cond_hit), .x_stall(x_stall),
    .h_reset(h_reset), .h_dbg_req(h_dbg_req), .h_dbg_wr(h_dbg_wr),
    .h_dbg_thread(h_dbg_thread), .h_dbg_reg(h_dbg_reg),
    .pc_all(pc_all), .z_all(z_all), .c_all(c_all), .t_all(t_all),
    .wa_all(wa_all), .rs0_all(rs0_all), .rs1_all(rs1_all),
    .depth_all(depth_all), .pp_all(pp_all),
    .outgrp_all(outgrp_all), .ingrp_all(ingrp_all), .steps_all(steps_all),
    .resetpc_r(resetpc_r), .run_r(run_r), .halted_r(halted_r),
    .step_req_r(step_req_r), .badop_r(badop_r), .sflags_r(sflags_r),
    .swirq_r(swirq_r), .lat_valid_all(lat_valid_all),
    .lat_val_all(lat_val_all), .lat_pin_all(lat_pin_all),
    .now_all(now_all), .td_all(td_all_w), .dt_all(dt_all_w),
    .tick_int_all(tick_int_all), .tick_frac_all(tick_frac_all),
    .tick_seen_all(tick_seen_all),
    .acc(u_timer.g_thread[`ISO_T].acc), .lat_fire(lat_fire),
    .be_sr_all(be_sr_all), .be_cnt_all(be_cnt_all), .be_crc_all(be_crc_all),
    .be_cfg_all(be_cfg_all), .be_pins_all(be_pins_all),
    .be_reload_all(be_reload_all), .be_poly_all(be_poly_all),
    .be_init_all(be_init_all), .be_enc_all(be_enc_all),
    .inq_cnt_all(inq_cnt_all), .outq_cnt_all(outq_cnt_all),
    .inq_head_all(inq_head_all), .outq_head(outq_head), .outq_next(outq_next),
    .r0(u_rf.regs[8*`ISO_T + 0]), .r1(u_rf.regs[8*`ISO_T + 1]),
    .r2(u_rf.regs[8*`ISO_T + 2]), .r3(u_rf.regs[8*`ISO_T + 3]),
    .r4(u_rf.regs[8*`ISO_T + 4]), .r5(u_rf.regs[8*`ISO_T + 5]),
    .r6(u_rf.regs[8*`ISO_T + 6]), .r7(u_rf.regs[8*`ISO_T + 7]),
    .rf_we_oh(rf_we_oh), .rf_w_addr(u_rf.w_addr), .rf_wdata(u_rf.wdata),
    .cw_valid(cw_valid), .cw_out_mask(cw_out_mask), .cw_out_data(cw_out_data),
    .cw_oe_mask(cw_oe_mask), .cw_oe_data(cw_oe_data),
    .cw_od_we(cw_od_we), .cw_od(cw_od),
    .lw_out_mask(lw_out_mask), .lw_out_data(lw_out_data),
    .lw_oe_mask(lw_oe_mask), .lw_oe_data(lw_oe_data),
    .tr_valid(tr_valid), .tr_thread(tr_thread), .tr_pc(tr_pc), .tr_ir(tr_ir),
    .tr_done(tr_done), .tr_we(tr_we), .tr_rd(tr_rd), .tr_val(tr_val),
    .tr_flags(tr_flags), .tr_next_pc(tr_next_pc)
);

`default_nettype wire
