/*
 * loom_core: the barrel pipeline (docs/SEMANTICS.md sections 1, 2, 3 and 6).
 * SPDX-License-Identifier: Apache-2.0
 *
 * Four threads share one four-stage pipeline in strict round robin. `ph` is a
 * free-running 2-bit phase counter; thread t owns the slot that starts in
 * every cycle k with k mod 4 == t.
 *
 * Timing contract (cycle numbers relative to the slot's F cycle k):
 *   F  k    imem_addr = PC[t]; the slot is valid iff RUN[t] or STEP_REQ[t]
 *           as visible in cycle k; a valid slot consumes STEP_REQ[t] at k+1.
 *   D  k+1  imem_rdata is the instruction word; decode; read the registers.
 *   X  k+2  evaluate against the state visible in cycle k+2; decide done or
 *           stall. This is the slot's "X cycle" x.
 *   W  k+3  present the commit. Every effect is registered at edge k+4 = x+2,
 *           so shared state changes are visible to other slots from cycle
 *           x+2 and pads change at edge x+2. SFLAGS is forwarded from W to X,
 *           so an SFLAGS effect is visible from cycle x+1, which is what
 *           makes WAITS an atomic test-and-clear between adjacent slots.
 *   The retire record outputs (tr_*) are valid during W.
 *
 * Every instruction bit comes from loom_decode; nothing here compares `ir`.
 * Two decoder instances are used: one in D, whose only job is to produce the
 * register file read addresses, and one in X for everything else.
 *
 * W-stage thread select (D-019). The slot in W during cycle k belongs to
 * thread (k + 1) mod 4, whatever runs, because the slot order is fixed. So
 * the W stage does not decode a thread number: every consumer group of
 * per-thread state (PC and flags; the other per-thread core state; the
 * register file; the timers) has its own 4-bit one-hot ring that is reset to
 * 4'b0010 (thread 1 during cycle 0) and rotates every cycle, and w_valid
 * qualifies it. The rings are independent self-rotating registers, so
 * synthesis cannot merge them and no single net fans out to all per-thread
 * state. `w_thread` survives only for the retire record. STEPS is
 * incremented in X (`w_steps`), so no adder sits behind the thread select,
 * and the host's "thread is busy" check uses a registered in-flight vector
 * (`infl`) instead of decoding the W stage.
 * A thread's commit and a host write to the same thread never coincide
 * (debug writes need the thread halted, SEMANTICS 7); every per-thread
 * register is written with the commit as the last mux before the flop, and
 * the commit wins if a host CTRL.RESET of a running thread (undefined in
 * SEMANTICS 7) ever hits the same edge, exactly as in M1.
 *
 * FIFOs (SEMANTICS 6.7): INQ[t] and OUTQ[t] are loom_fifo instances. PUSH
 * and POP decide in X from the counts visible there and stall like waits
 * (WAIT_ACTIVE); the entry moves at the commit edge. WAITB tests OUTQ not
 * full (1), INQ not empty (2) and TICK_SEEN (3); WAITB 0 belongs to the bit
 * engine (below). A host push into a full INQ is dropped and sets
 * BADOP[14]; the host's pops arrive already checked from loom_host_ctl,
 * which also reports a pop from an empty OUTQ on h_badop_set14. CTRL.RESET
 * empties both FIFOs of the thread.
 *
 * Bit engine, manual mode (SEMANTICS 6.9): the state lives in loom_be; SHO,
 * SHI, LDSR, CRCI and CSRW compute the new SR/CNT/CRC in X and the W stage
 * commits them. SHO's pin write uses the same single-pin path as SETP, so
 * the open-drain rule and the index checks are shared. WAITB 0 (bit engine
 * idle) is always true until auto mode exists.
 *
 * Encoders, stuffing and DIFF (SEMANTICS 6.9.1, M3 slice A, D-026): one
 * copy of the encoder, the stuffer and the decoder sits in X next to the
 * shifter and the CRC and is selected by `xsel` the same way, so no new
 * wide bus is tapped per thread (D-025). Per thread loom_be holds only the
 * eight encoder bits {FIRST, HALF, PEND, RVAL, RUN, LVL}, committed through
 * its own port with the W-stage ring, cleared by any BE_CFG write and by
 * CTRL.RESET. A stuff bit and a Manchester half-bit are not data bits: they
 * drive the pin but leave SR, CNT and the CRC alone, while Z still reports
 * CNT after the instruction. DIFF adds a second pin write of the complement
 * to (BE_PINS.out + 1) mod 32 through the same 32-bit mask the group write
 * (OUT) uses, so a non-writable index is dropped exactly as in 6.3.
 *
 * Deadline-latched SETP (SEMANTICS 6.10): `SETP pin, v, D` writes no pin;
 * its commit stages {LAT_VALID, LAT_PIN, LAT_VAL} in the thread's latch,
 * replacing what was staged. loom_timer raises lat_fire[t] in the cycle
 * before an edge at which rule 1 (NOW ticks to exactly TD, TD not written)
 * or rule 2 (TD written, reached(NOW', TD')) holds; a staged write that is
 * already valid before that edge is applied at it (the lw_* port of
 * loom_pins, 6.3 rules and open drain included) and LAT_VALID clears. So
 * the edge that arms a write never applies it, while a write staged earlier
 * does apply at the edge where a new SETP D replaces it. A slot's ordinary
 * pin write wins on every bit it writes at the same edge (loom_pins), and
 * where two threads' staged writes hit one pin at one edge the higher
 * thread wins. CTRL.RESET discards the staged write without a pin write,
 * although it also writes TD. Only a first-issue WAITD writes TD: a WAITD
 * that re-issues would write TD with its own value, so it commits nothing,
 * which keeps "TD is written" (rule 2) exactly as 6.10 lists it. Debug 0x25
 * reads {LAT_VALID, LAT_VAL, LAT_PIN} in bits 6:0 and writes it while the
 * thread is halted.
 *
 * Not built: LD/ST (data memory). They decode normally and execute as NOP
 * with BADOP[t] set (SEMANTICS section 9).
 */

`default_nettype none

module loom_core #(
    parameter integer IMEM_AW    = 8,
    parameter [15:0]  IMEM_WORDS = 16'd256,
    parameter integer FIFO_DEPTH = 4
) (
    input  wire              clk,
    input  wire              rst_n,

    // ---------------------------------------------------------- imem port
    output wire [IMEM_AW-1:0] imem_addr,
    output wire              imem_en,
    input  wire [15:0]       imem_rdata,

    // ---------------------------------------------------------- pin unit
    input  wire [31:0]       pin_in_vec,
    input  wire [15:0]       pin_in_reg,
    input  wire [15:0]       pin_out_reg,
    input  wire [7:0]        pin_oe_reg,
    input  wire [7:0]        od_mask_reg,
    output wire              cw_valid,
    output wire [13:0]       cw_out_mask,
    output wire [13:0]       cw_out_data,
    output wire [7:0]        cw_oe_mask,
    output wire [7:0]        cw_oe_data,
    output wire              cw_od_we,
    output wire [7:0]        cw_od,
    // Staged (deadline-latched) pin writes applying at the coming edge.
    output wire [13:0]       lw_out_mask,
    output wire [13:0]       lw_out_data,
    output wire [7:0]        lw_oe_mask,
    output wire [7:0]        lw_oe_data,

    // ------------------------------------------------------- host control
    input  wire              h_run_we,
    input  wire [3:0]        h_run,
    input  wire [3:0]        h_reset,
    input  wire              h_step_we,
    input  wire [3:0]        h_step,
    input  wire              h_rpc_we,
    input  wire [1:0]        h_rpc_sel,
    input  wire [9:0]        h_rpc,
    input  wire              h_sfset_we,
    input  wire [7:0]        h_sfset,
    input  wire              h_sfclr_we,
    input  wire [7:0]        h_sfclr,
    input  wire              h_badop_clr_we,
    input  wire [15:0]       h_badop_clr,
    input  wire              h_badop_set15,
    input  wire              h_swirq_clr_we,
    input  wire [3:0]        h_swirq_clr,

    // ------------------------------------------------------- host FIFO port
    input  wire [3:0]        h_inq_push,     // pulse: push h_fifo_wdata into INQ[t]
    input  wire [15:0]       h_fifo_wdata,
    input  wire [3:0]        h_outq_pop,     // pulse: pop OUTQ[t] (host checked it)
    input  wire              h_badop_set14,  // pulse: a host pop found OUTQ empty
    output wire [47:0]       fifo_stat,      // per thread: the FIFO status word
    output wire [63:0]       outq_head,      // per thread: OUTQ head entry
    output wire [63:0]       outq_next,      // per thread: the entry after it

    input  wire              h_dbg_req,
    input  wire              h_dbg_wr,
    input  wire [1:0]        h_dbg_thread,
    input  wire [7:0]        h_dbg_reg,
    input  wire [15:0]       h_dbg_wdata,
    output reg               h_dbg_ack,
    output reg  [15:0]       h_dbg_rdata,

    output wire [3:0]        run,
    output wire [3:0]        halted,
    output wire [15:0]       badop,
    output wire [7:0]        sflags,
    output wire [3:0]        swirq,
    output wire [39:0]       resetpc_all,
    output wire              core_busy,

    // ------------------------------------------------------ retire record
    output wire              tr_valid,
    output wire [1:0]        tr_thread,
    output wire [9:0]        tr_pc,
    output wire [15:0]       tr_ir,
    output wire              tr_done,
    output wire              tr_we,
    output wire [2:0]        tr_rd,
    output wire [15:0]       tr_val,
    output wire [2:0]        tr_flags,
    output wire [9:0]        tr_next_pc
);

  // ==================================================================== state
  reg [1:0]  ph;
  reg [3:0]  run_r, halted_r, step_req_r, swirq_r;
  reg [15:0] badop_r;
  reg [7:0]  sflags_r;
  reg [39:0] pc_all, rs0_all, rs1_all, resetpc_r;
  reg [3:0]  z_all, c_all, t_all, wa_all;
  reg [7:0]  depth_all;
  reg [51:0] pp_all;
  reg [39:0] outgrp_all, ingrp_all;
  reg [63:0] steps_all;
  reg [3:0]  lat_valid_all, lat_val_all;   // deadline latch (6.10)
  reg [19:0] lat_pin_all;

  assign run         = run_r;
  assign halted      = halted_r;
  assign badop       = badop_r;
  assign sflags      = sflags_r;
  assign swirq       = swirq_r;
  assign resetpc_all = resetpc_r;

  // ============================================================= F stage
  wire [31:0] fsel   = {30'd0, ph};
  wire [9:0]  pc_f   = pc_all[fsel*10 +: 10];
  wire        valid_f = run_r[ph] | step_req_r[ph];

  assign imem_addr = pc_f[IMEM_AW-1:0];
  assign imem_en   = valid_f;

  // ============================================================= D stage
  reg        vd;
  reg [1:0]  td_th;
  reg [9:0]  pcd;

  wire [15:0] ird = imem_rdata;

  wire [2:0] d_f_rd, d_f_ra, d_f_rb;
  wire       d_grp_alu;
  wire [63:0] d_unused_a;
  wire        d_unused_b;
  wire [9:0]  d_unused_rel, d_unused_abs;
  wire [15:0] d_unused_imm;
  wire [4:0]  d_unused_pin, d_unused_csr;
  wire [2:0]  d_unused_funct, d_unused_flag;
  wire [1:0]  d_unused_edge, d_unused_cond;
  wire        d_unused_val, d_unused_tmo, d_unused_lat;
  wire [21:0] d_unused_csrsel;
  wire [11:0] d_unused_grp;
  wire [1:0]  d_unused_tmg;

  loom_decode u_dec_d (
      .ir(ird),
      .is_add(d_unused_a[0]),   .is_sub(d_unused_a[1]),   .is_and(d_unused_a[2]),
      .is_or(d_unused_a[3]),    .is_xor(d_unused_a[4]),   .is_shl(d_unused_a[5]),
      .is_shr(d_unused_a[6]),   .is_ror(d_unused_a[7]),   .is_addi(d_unused_a[8]),
      .is_subi(d_unused_a[9]),  .is_andi(d_unused_a[10]), .is_ori(d_unused_a[11]),
      .is_xori(d_unused_a[12]), .is_shli(d_unused_a[13]), .is_shri(d_unused_a[14]),
      .is_cmpi(d_unused_a[15]), .is_ldi(d_unused_a[16]),  .is_ldih(d_unused_a[17]),
      .is_mov(d_unused_a[18]),  .is_not(d_unused_a[19]),  .is_neg(d_unused_a[20]),
      .is_cmp(d_unused_a[21]),  .is_test(d_unused_a[22]), .is_rev(d_unused_a[23]),
      .is_par(d_unused_a[24]),  .is_swap(d_unused_a[25]), .is_jmp(d_unused_a[26]),
      .is_call(d_unused_a[27]), .is_ret(d_unused_a[28]),  .is_halt(d_unused_a[29]),
      .is_bz(d_unused_a[30]),   .is_bnz(d_unused_a[31]),  .is_bc(d_unused_a[32]),
      .is_bnc(d_unused_a[33]),  .is_bt(d_unused_a[34]),   .is_bnt(d_unused_a[35]),
      .is_djnz(d_unused_a[36]), .is_jp(d_unused_a[37]),   .is_setp(d_unused_a[38]),
      .is_oep(d_unused_a[39]),  .is_out(d_unused_a[40]),  .is_in(d_unused_a[41]),
      .is_waitd(d_unused_a[42]),.is_waitp(d_unused_a[43]),.is_waite(d_unused_a[44]),
      .is_waits(d_unused_a[45]),.is_waitb(d_unused_a[46]),.is_dly(d_unused_a[47]),
      .is_setd(d_unused_a[48]), .is_nop(d_unused_a[49]),  .is_push(d_unused_a[50]),
      .is_pop(d_unused_a[51]),  .is_sho(d_unused_a[52]),  .is_shi(d_unused_a[53]),
      .is_ldsr(d_unused_a[54]), .is_stsr(d_unused_a[55]), .is_crci(d_unused_a[56]),
      .is_stcrc(d_unused_a[57]),.is_csrr(d_unused_a[58]), .is_csrw(d_unused_a[59]),
      .is_clr(d_unused_a[60]),  .is_sig(d_unused_a[61]),  .is_ld(d_unused_a[62]),
      .is_st(d_unused_a[63]),   .is_reserved(d_unused_b),
      .grp_alu(d_grp_alu),      .grp_alui(d_unused_grp[0]), .grp_be(d_unused_grp[1]),
      .grp_branch(d_unused_grp[2]), .grp_csr(d_unused_grp[3]), .grp_fifo(d_unused_grp[4]),
      .grp_jump(d_unused_grp[5]),   .grp_ldi(d_unused_grp[6]), .grp_mem(d_unused_grp[7]),
      .grp_pin(d_unused_grp[8]),    .grp_sflag(d_unused_grp[9]), .grp_unary(d_unused_grp[10]),
      .grp_wait(d_unused_grp[11]),
      .tmg_wait(d_unused_tmg[0]), .tmg_blocking(d_unused_tmg[1]),
      .f_funct(d_unused_funct), .f_rd(d_f_rd), .f_ra(d_f_ra), .f_rb(d_f_rb),
      .f_imm(d_unused_imm), .f_rel(d_unused_rel), .f_abs(d_unused_abs),
      .f_pin(d_unused_pin), .f_val(d_unused_val), .f_edge(d_unused_edge),
      .f_flag(d_unused_flag), .f_tmo(d_unused_tmo), .f_cond(d_unused_cond),
      .f_csr(d_unused_csr), .f_lat(d_unused_lat),
      .csr_tick_int(d_unused_csrsel[0]),  .csr_tick_frac(d_unused_csrsel[1]),
      .csr_outgrp(d_unused_csrsel[2]),    .csr_ingrp(d_unused_csrsel[3]),
      .csr_be_cfg(d_unused_csrsel[4]),    .csr_be_pins(d_unused_csrsel[5]),
      .csr_be_reload(d_unused_csrsel[6]), .csr_crc_poly(d_unused_csrsel[7]),
      .csr_crc_init(d_unused_csrsel[8]),  .csr_now(d_unused_csrsel[9]),
      .csr_td(d_unused_csrsel[10]),       .csr_flags(d_unused_csrsel[11]),
      .csr_tid(d_unused_csrsel[12]),      .csr_sr(d_unused_csrsel[13]),
      .csr_cnt(d_unused_csrsel[14]),      .csr_crc(d_unused_csrsel[15]),
      .csr_od_mask(d_unused_csrsel[16]),  .csr_pin_out(d_unused_csrsel[17]),
      .csr_pin_oe(d_unused_csrsel[18]),   .csr_pin_in(d_unused_csrsel[19]),
      .csr_sflags(d_unused_csrsel[20]),   .csr_host_irq(d_unused_csrsel[21])
  );

  wire _unused_dec_d = &{1'b0, d_unused_a, d_unused_b, d_unused_rel, d_unused_abs,
                         d_unused_imm, d_unused_pin, d_unused_csr, d_unused_funct,
                         d_unused_flag, d_unused_edge, d_unused_cond, d_unused_val,
                         d_unused_tmo, d_unused_lat, d_unused_csrsel, d_unused_grp,
                         d_unused_tmg};

  // ============================================================ X stage regs
  reg        vx;
  reg [1:0]  tx_th;
  reg [9:0]  pcx;
  reg [15:0] irx;
  reg [15:0] op_a, op_b;

  // ============================================================ W stage regs
  reg        w_valid, w_done, w_reg_we, w_wait_active, w_halt, w_badop;
  reg [1:0]  w_thread;
  reg [9:0]  w_pc, w_next_pc;
  reg [15:0] w_ir, w_rval;
  reg [2:0]  w_rd, w_flags;
  reg [12:0] w_prev_pins;
  reg        w_td_we, w_dt_we, w_tint_we, w_tfrac_we, w_outgrp_we, w_ingrp_we;
  reg        w_tseen;   // TICK_SEEN as this slot read it in X (SEMANTICS 4)
  reg [15:0] w_td, w_dt, w_csr_val;
  reg        w_rs_we;
  reg [9:0]  w_rs0, w_rs1;
  reg [1:0]  w_depth;
  reg [7:0]  w_sf_set, w_sf_clr;
  reg [13:0] w_out_mask, w_out_data;
  reg [7:0]  w_oe_mask, w_oe_data;
  reg        w_od_we;
  reg [7:0]  w_od;
  reg        w_swirq;
  reg [15:0] w_steps;
  reg        w_push, w_pop;         // a PUSH / POP completes in this slot
  reg        w_sr_we, w_cnt_we, w_crc_we, w_enc_we;
  reg [15:0] w_sr, w_crc;
  reg [4:0]  w_cnt;
  reg [7:0]  w_enc;               // the encoder state after an SHO/SHI
  reg        w_becfg_we, w_bepins_we, w_bereload_we, w_crcpoly_we, w_crcinit_we;
  reg        w_lat;                 // a SETP D completes in this slot
  reg [4:0]  w_lat_pin;
  reg        w_lat_val;

  // ============================================= W-stage thread rings (D-019)
  // Bit t of every ring is high in exactly the cycles whose W slot belongs to
  // thread t: the W slot of cycle k was fetched in cycle k - 3, so it is
  // thread (k + 1) mod 4, thread 1 in cycle 0. One ring per consumer group.
  reg [3:0] woh_pc;     // PC, flags, WAIT_ACTIVE, RUN/HALTED, BADOP, SWIRQ
  reg [3:0] woh_aux;    // PREV_PINS, STEPS, return stack, OUTGRP, INGRP
  reg [3:0] woh_rf;     // register file write port
  reg [3:0] woh_tmr;    // loom_timer commit port
  reg [3:0] woh_fifo;   // FIFO push/pop commits
  reg [3:0] woh_be;     // bit-engine state commits
  reg [3:0] woh_lat;    // deadline-latch commits (SETP D)

  always @(posedge clk) begin
    if (!rst_n) begin
      woh_pc   <= 4'b0010;
      woh_aux  <= 4'b0010;
      woh_rf   <= 4'b0010;
      woh_tmr  <= 4'b0010;
      woh_fifo <= 4'b0010;
      woh_be   <= 4'b0010;
      woh_lat  <= 4'b0010;
    end else begin
      woh_pc   <= {woh_pc[2:0],   woh_pc[3]};
      woh_aux  <= {woh_aux[2:0],  woh_aux[3]};
      woh_rf   <= {woh_rf[2:0],   woh_rf[3]};
      woh_tmr  <= {woh_tmr[2:0],  woh_tmr[3]};
      woh_fifo <= {woh_fifo[2:0], woh_fifo[3]};
      woh_be   <= {woh_be[2:0],   woh_be[3]};
      woh_lat  <= {woh_lat[2:0],  woh_lat[3]};
    end
  end

  // Per-thread commit strobes: a valid slot of thread t is in W.
  wire [3:0] cw_pc  = {4{w_valid}} & woh_pc;
  wire [3:0] cw_aux = {4{w_valid}} & woh_aux;

  // ======================================================= register file
  // A thread counts as halted for host access to r0..r7 and for every debug
  // write iff RUN[t] and STEP_REQ[t] are clear and it has no valid slot in
  // F, D, X or W, so the state the host sees is always architectural.
  // `infl` is registered from the F, D and X terms, so in every cycle it
  // holds exactly "a valid slot of thread t is in D, X or W" without any
  // logic behind the W-stage registers (D-019).
  wire [3:0] busy_f = valid_f  ? (4'd1 << ph)       : 4'd0;
  wire [3:0] busy_d = vd       ? (4'd1 << td_th)    : 4'd0;
  wire [3:0] busy_x = vx       ? (4'd1 << tx_th)    : 4'd0;
  reg  [3:0] infl;
  always @(posedge clk) begin
    if (!rst_n) infl <= 4'd0;
    else        infl <= busy_f | busy_d | busy_x;
  end
  wire [3:0] thread_busy = run_r | step_req_r | busy_f | infl;

  wire        dbg_is_reg   = (h_dbg_reg[7:3] == 5'd0);
  wire        dbg_running  = thread_busy[h_dbg_thread];
  wire        dbg_rf_wr_go = h_dbg_req &  h_dbg_wr & dbg_is_reg & ~dbg_running
                             & ~(w_valid & w_reg_we) & ~h_dbg_ack;

  // Read port A serves the D stage, and the host whenever D holds a bubble
  // (its operands are then never used), so the select is just `vd`.
  wire [1:0] rf_ra_thread = vd ? td_th : h_dbg_thread;
  wire [2:0] rf_ra_addr   = vd ? d_f_ra : h_dbg_reg[2:0];
  wire [2:0] rf_rb_addr   = d_grp_alu ? d_f_rb : d_f_rd;
  wire [15:0] rf_rd_a, rf_rd_b;

  // The W stage owns the write port whenever its slot writes a register and
  // the host borrows it otherwise (dbg_rf_wr_go already excludes that case),
  // so the address and data select is the W stage's own write strobe and the
  // thread select is the one-hot ring, never a decoded thread number.
  wire       w_rf_write = w_valid & w_reg_we;
  wire [3:0] rf_we_oh   = ({4{w_rf_write}} & woh_rf)
                          | ({4{dbg_rf_wr_go}} & (4'd1 << h_dbg_thread));

  loom_regfile u_rf (
      .clk(clk), .rst_n(rst_n),
      .ra_thread(rf_ra_thread), .ra_addr(rf_ra_addr), .rd_a(rf_rd_a),
      .rb_thread(td_th),        .rb_addr(rf_rb_addr), .rd_b(rf_rd_b),
      .we_oh (rf_we_oh),
      .w_addr(w_rf_write ? w_rd   : h_dbg_reg[2:0]),
      .wdata (w_rf_write ? w_rval : h_dbg_wdata)
  );

  // ================================================== X stage decode
  wire is_add, is_sub, is_and, is_or, is_xor, is_shl, is_shr, is_ror;
  wire is_addi, is_subi, is_andi, is_ori, is_xori, is_shli, is_shri, is_cmpi;
  wire is_ldi, is_ldih, is_mov, is_not, is_neg, is_cmp, is_test, is_rev;
  wire is_par, is_swap, is_jmp, is_call, is_ret, is_halt;
  wire is_bz, is_bnz, is_bc, is_bnc, is_bt, is_bnt, is_djnz, is_jp;
  wire is_setp, is_oep, is_out, is_in;
  wire is_waitd, is_waitp, is_waite, is_waits, is_waitb, is_dly, is_setd, is_nop;
  wire is_push, is_pop, is_sho, is_shi, is_ldsr, is_stsr, is_crci, is_stcrc;
  wire is_csrr, is_csrw, is_clr, is_sig, is_ld, is_st, is_reserved;
  wire grp_alu, grp_alui, grp_be, grp_branch, grp_csr, grp_fifo, grp_jump;
  wire grp_ldi, grp_mem, grp_pin, grp_sflag, grp_unary, grp_wait;
  wire tmg_wait, tmg_blocking;
  wire [2:0]  f_funct, f_rd, f_ra, f_rb, f_flag;
  wire [15:0] f_imm;
  wire [9:0]  f_rel, f_abs;
  wire [4:0]  f_pin, f_csr;
  wire        f_val, f_tmo;
  wire        f_lat;      // SETP D (ISA 0.4.0); built at M2, unused until then
  wire [1:0]  f_edge, f_cond;
  wire csr_tick_int, csr_tick_frac, csr_outgrp, csr_ingrp, csr_be_cfg;
  wire csr_be_pins, csr_be_reload, csr_crc_poly, csr_crc_init, csr_now;
  wire csr_td, csr_flags, csr_tid, csr_sr, csr_cnt, csr_crc, csr_od_mask;
  wire csr_pin_out, csr_pin_oe, csr_pin_in, csr_sflags, csr_host_irq;

  loom_decode u_dec_x (
      .ir(irx),
      .is_add(is_add), .is_sub(is_sub), .is_and(is_and), .is_or(is_or),
      .is_xor(is_xor), .is_shl(is_shl), .is_shr(is_shr), .is_ror(is_ror),
      .is_addi(is_addi), .is_subi(is_subi), .is_andi(is_andi), .is_ori(is_ori),
      .is_xori(is_xori), .is_shli(is_shli), .is_shri(is_shri), .is_cmpi(is_cmpi),
      .is_ldi(is_ldi), .is_ldih(is_ldih), .is_mov(is_mov), .is_not(is_not),
      .is_neg(is_neg), .is_cmp(is_cmp), .is_test(is_test), .is_rev(is_rev),
      .is_par(is_par), .is_swap(is_swap), .is_jmp(is_jmp), .is_call(is_call),
      .is_ret(is_ret), .is_halt(is_halt), .is_bz(is_bz), .is_bnz(is_bnz),
      .is_bc(is_bc), .is_bnc(is_bnc), .is_bt(is_bt), .is_bnt(is_bnt),
      .is_djnz(is_djnz), .is_jp(is_jp), .is_setp(is_setp), .is_oep(is_oep),
      .is_out(is_out), .is_in(is_in), .is_waitd(is_waitd), .is_waitp(is_waitp),
      .is_waite(is_waite), .is_waits(is_waits), .is_waitb(is_waitb),
      .is_dly(is_dly), .is_setd(is_setd), .is_nop(is_nop), .is_push(is_push),
      .is_pop(is_pop), .is_sho(is_sho), .is_shi(is_shi), .is_ldsr(is_ldsr),
      .is_stsr(is_stsr), .is_crci(is_crci), .is_stcrc(is_stcrc),
      .is_csrr(is_csrr), .is_csrw(is_csrw), .is_clr(is_clr), .is_sig(is_sig),
      .is_ld(is_ld), .is_st(is_st), .is_reserved(is_reserved),
      .grp_alu(grp_alu), .grp_alui(grp_alui), .grp_be(grp_be),
      .grp_branch(grp_branch), .grp_csr(grp_csr), .grp_fifo(grp_fifo),
      .grp_jump(grp_jump), .grp_ldi(grp_ldi), .grp_mem(grp_mem),
      .grp_pin(grp_pin), .grp_sflag(grp_sflag), .grp_unary(grp_unary),
      .grp_wait(grp_wait), .tmg_wait(tmg_wait), .tmg_blocking(tmg_blocking),
      .f_funct(f_funct), .f_rd(f_rd), .f_ra(f_ra), .f_rb(f_rb), .f_imm(f_imm),
      .f_rel(f_rel), .f_abs(f_abs), .f_pin(f_pin), .f_val(f_val),
      .f_edge(f_edge), .f_flag(f_flag), .f_tmo(f_tmo), .f_cond(f_cond),
      .f_csr(f_csr), .f_lat(f_lat),
      .csr_tick_int(csr_tick_int), .csr_tick_frac(csr_tick_frac),
      .csr_outgrp(csr_outgrp), .csr_ingrp(csr_ingrp), .csr_be_cfg(csr_be_cfg),
      .csr_be_pins(csr_be_pins), .csr_be_reload(csr_be_reload),
      .csr_crc_poly(csr_crc_poly), .csr_crc_init(csr_crc_init),
      .csr_now(csr_now), .csr_td(csr_td), .csr_flags(csr_flags),
      .csr_tid(csr_tid), .csr_sr(csr_sr), .csr_cnt(csr_cnt), .csr_crc(csr_crc),
      .csr_od_mask(csr_od_mask), .csr_pin_out(csr_pin_out),
      .csr_pin_oe(csr_pin_oe), .csr_pin_in(csr_pin_in),
      .csr_sflags(csr_sflags), .csr_host_irq(csr_host_irq)
  );

  // The X decoder exposes every strobe of the ISA; only some are used at M1
  // (and some, like the individual ALU sub-operations, are consumed by
  // loom_alu through f_funct instead). Collect them all so the lint is clean.
  wire _unused_dec_x = &{1'b0,
      is_add, is_sub, is_and, is_or, is_xor, is_shl, is_shr, is_ror,
      is_addi, is_subi, is_andi, is_ori, is_xori, is_shli, is_shri, is_cmpi,
      is_ldi, is_ldih, is_mov, is_not, is_neg, is_cmp, is_test, is_rev,
      is_par, is_swap, is_jmp, is_call, is_ret, is_halt,
      is_bz, is_bnz, is_bc, is_bnc, is_bt, is_bnt, is_djnz, is_jp,
      is_setp, is_oep, is_out, is_in,
      is_waitd, is_waitp, is_waite, is_waits, is_waitb, is_dly, is_setd, is_nop,
      is_push, is_pop, is_sho, is_shi, is_ldsr, is_stsr, is_crci, is_stcrc,
      is_csrr, is_csrw, is_clr, is_sig, is_ld, is_st, is_reserved,
      grp_alu, grp_alui, grp_be, grp_branch, grp_csr, grp_fifo, grp_jump,
      grp_ldi, grp_mem, grp_pin, grp_sflag, grp_unary, grp_wait,
      tmg_wait, tmg_blocking,
      f_funct, f_rd, f_ra, f_rb, f_imm, f_rel, f_abs, f_pin, f_val,
      f_edge, f_flag, f_tmo, f_cond, f_csr, f_lat,
      csr_tick_int, csr_tick_frac, csr_outgrp, csr_ingrp, csr_be_cfg,
      csr_be_pins, csr_be_reload, csr_crc_poly, csr_crc_init, csr_now,
      csr_td, csr_flags, csr_tid, csr_sr, csr_cnt, csr_crc, csr_od_mask,
      csr_pin_out, csr_pin_oe, csr_pin_in, csr_sflags, csr_host_irq};

  // ===================================================== per-thread timer
  wire [63:0] now_all, td_all_w, dt_all_w, tick_int_all;
  wire [31:0] tick_frac_all;
  wire [3:0]  tick_seen_all;
  wire [3:0]  lat_fire;
  wire        tmr_h_we, tmr_h_td_we, tmr_h_dt_we, tmr_h_tint_we, tmr_h_tfrac_we;
  wire        tmr_h_tseen_we;
  wire [15:0] tmr_h_wdata;

  loom_timer u_timer (
      .clk(clk), .rst_n(rst_n),
      .cm_sel({4{w_valid}} & woh_tmr),
      .cm_td_we(w_td_we), .cm_td(w_td), .cm_tseen(w_tseen),
      .cm_dt_we(w_dt_we), .cm_dt(w_dt),
      .cm_tint_we(w_tint_we), .cm_tint(w_csr_val),
      .cm_tfrac_we(w_tfrac_we), .cm_tfrac(w_csr_val[7:0]),
      .h_reset(h_reset),
      .h_we(tmr_h_we), .h_thread(h_dbg_thread),
      .h_td_we(tmr_h_td_we), .h_dt_we(tmr_h_dt_we),
      .h_tint_we(tmr_h_tint_we), .h_tfrac_we(tmr_h_tfrac_we),
      .h_tseen_we(tmr_h_tseen_we),
      .h_wdata(tmr_h_wdata),
      .now_all(now_all), .td_all(td_all_w), .dt_all(dt_all_w),
      .tick_int_all(tick_int_all), .tick_frac_all(tick_frac_all),
      .tick_seen_all(tick_seen_all),
      .lat_fire(lat_fire)
  );

  // ============================================ X stage: state it can see
  wire [31:0] xsel = {30'd0, tx_th};
  wire [9:0]  x_rs0    = rs0_all[xsel*10 +: 10];
  wire [9:0]  x_rs1    = rs1_all[xsel*10 +: 10];
  wire [1:0]  x_depth  = depth_all[xsel*2 +: 2];
  wire        x_z      = z_all[tx_th];
  wire        x_c      = c_all[tx_th];
  wire        x_t      = t_all[tx_th];
  wire        x_wa     = wa_all[tx_th];
  wire [12:0] x_pp     = pp_all[xsel*13 +: 13];
  wire [9:0]  x_outgrp = outgrp_all[xsel*10 +: 10];
  wire [9:0]  x_ingrp  = ingrp_all[xsel*10 +: 10];
  wire [15:0] x_now    = now_all[xsel*16 +: 16];
  wire [15:0] x_td     = td_all_w[xsel*16 +: 16];
  wire [15:0] x_dt     = dt_all_w[xsel*16 +: 16];
  wire [15:0] x_tint   = tick_int_all[xsel*16 +: 16];
  wire [7:0]  x_tfrac  = tick_frac_all[xsel*8 +: 8];
  wire        x_tseen  = tick_seen_all[tx_th];

  // ================================================================ FIFOs
  localparam integer FAW = $clog2(FIFO_DEPTH);
  localparam [31:0]  FDEPTH32 = FIFO_DEPTH;
  localparam [FAW:0] FDEPTH = FDEPTH32[FAW:0];

  wire [4*(FAW+1)-1:0] inq_cnt_all, outq_cnt_all;
  wire [63:0]          inq_head_all, inq_next_unused;
  wire [3:0]           cw_fifo = {4{w_valid}} & woh_fifo;
  // A host push is accepted iff INQ is not full in the cycle before the
  // commit edge (SEMANTICS 6.7); a thread pop at the same edge also applies.
  wire [3:0]           inq_full;
  wire [3:0]           h_push_ok  = h_inq_push & ~inq_full;
  wire                 h_push_bad = |(h_inq_push & inq_full);

  genvar gf;
  generate
    for (gf = 0; gf < 4; gf = gf + 1) begin : g_fifo
      wire [FAW:0] icnt, ocnt;
      loom_fifo #(.DEPTH(FIFO_DEPTH), .AW(FAW)) u_inq (
          .clk(clk), .rst_n(rst_n), .clr(h_reset[gf]),
          .push(h_push_ok[gf]), .wdata(h_fifo_wdata),
          .pop(cw_fifo[gf] & w_pop),
          .count(icnt), .head(inq_head_all[16*gf +: 16]),
          .next(inq_next_unused[16*gf +: 16]));
      // The host pops only what it saw (loom_host_ctl); the count check
      // keeps the FIFO consistent whatever arrives.
      loom_fifo #(.DEPTH(FIFO_DEPTH), .AW(FAW)) u_outq (
          .clk(clk), .rst_n(rst_n), .clr(h_reset[gf]),
          .push(cw_fifo[gf] & w_push), .wdata(w_csr_val),
          .pop(h_outq_pop[gf] & (ocnt != {(FAW+1){1'b0}})),
          .count(ocnt), .head(outq_head[16*gf +: 16]),
          .next(outq_next[16*gf +: 16]));
      assign inq_cnt_all[(FAW+1)*gf +: FAW+1]  = icnt;
      assign outq_cnt_all[(FAW+1)*gf +: FAW+1] = ocnt;
      assign inq_full[gf] = (icnt == FDEPTH);
      // Status word (docs/HOST_PROTOCOL.md SPACE 3): {OUTQ_COUNT[3:0],
      // INQ_COUNT[3:0], OUTQ_EMPTY, OUTQ_FULL, INQ_EMPTY, INQ_FULL}.
      wire [7:0] icnt8 = {{(7-FAW){1'b0}}, icnt};
      wire [7:0] ocnt8 = {{(7-FAW){1'b0}}, ocnt};
      assign fifo_stat[12*gf +: 12] = {ocnt8[3:0], icnt8[3:0],
                                       ocnt == {(FAW+1){1'b0}}, ocnt == FDEPTH,
                                       icnt == {(FAW+1){1'b0}}, icnt == FDEPTH};
      // The status word has 4-bit count fields (FIFO_DEPTH up to 8).
      wire _unused_cnt = &{1'b0, icnt8[7:4], ocnt8[7:4]};
    end
  endgenerate

  wire [FAW:0] x_inq_cnt  = inq_cnt_all[xsel*(FAW+1) +: FAW+1];
  wire [FAW:0] x_outq_cnt = outq_cnt_all[xsel*(FAW+1) +: FAW+1];
  wire [15:0]  x_inq_head = inq_head_all[xsel*16 +: 16];
  wire         x_inq_ne   = (x_inq_cnt != {(FAW+1){1'b0}});
  wire         x_outq_nf  = (x_outq_cnt != FDEPTH);

  // =========================================================== bit engine
  wire [63:0] be_sr_all, be_crc_all, be_poly_all, be_init_all;
  wire [19:0] be_cnt_all, be_reload_all;
  wire [31:0] be_cfg_all, be_enc_all;
  wire [39:0] be_pins_all;
  wire [15:0] x_sr     = be_sr_all[xsel*16 +: 16];
  wire [4:0]  x_cnt    = be_cnt_all[xsel*5 +: 5];
  wire [15:0] x_crc    = be_crc_all[xsel*16 +: 16];
  wire [15:0] x_poly   = be_poly_all[xsel*16 +: 16];
  wire [15:0] x_init   = be_init_all[xsel*16 +: 16];
  wire [4:0]  x_reload = be_reload_all[xsel*5 +: 5];
  // {DIFF, STUFF[1:0], ENC[1:0], CRC_EN, INV, DIR}
  wire [7:0]  x_becfg  = be_cfg_all[xsel*8 +: 8];
  // {FIRST, HALF, PEND, RVAL, RUN[2:0], LVL}
  wire [7:0]  x_enc    = be_enc_all[xsel*8 +: 8];
  wire [9:0]  x_bepins = be_pins_all[xsel*10 +: 10];  // {in, out}

  // The BE_CFG register view (ARCHITECTURE 8.1): the six stored fields in
  // their own bits, MODE (0), RXTX (2), AUTOPULL (8) and 12:11 reading 0
  // until slice C. Used by CSRR BE_CFG and by debug 0x14.
  function [15:0] becfg_word;
    input [7:0] cfg;                // {DIFF, STUFF, ENC, CRC_EN, INV, DIR}
    begin
      becfg_word = {5'd0, cfg[7], cfg[2], 1'b0, cfg[1],
                    cfg[6:5], cfg[4:3], 1'b0, cfg[0], 1'b0};
    end
  endfunction

  // STEPS + 1 is formed here and registered, so the commit only loads it.
  // The host cannot write STEPS between this X cycle and the commit edge,
  // because the thread has a valid slot in flight (SEMANTICS 7).
  wire [15:0] x_steps1 = steps_all[xsel*16 +: 16] + 16'd1;

  // SFLAGS is forwarded from W to X so that WAITS is an atomic test-and-clear
  // between slots in adjacent cycles (SEMANTICS 2).
  wire [7:0] sflags_x = (sflags_r | (w_valid ? w_sf_set : 8'd0))
                        & ~(w_valid ? w_sf_clr : 8'd0);

  // ---------------------------------------------------------- not built
  // Only LD/ST (data memory) are not built.
  wire unbuilt = grp_mem;
  wire bad_op  = is_reserved | unbuilt;

  wire [9:0] x_next = pcx + 10'd1;

  // ------------------------------------------------------------- the ALU
  wire        sel_alu   = grp_alu   & ~bad_op;
  wire        sel_alui  = grp_alui  & ~bad_op;
  wire        sel_unary = grp_unary & ~bad_op;
  wire [15:0] alu_a = (grp_alui | is_cmp | is_test) ? op_b : op_a;
  wire [15:0] alu_b = grp_alu ? op_b : (grp_alui ? f_imm : op_a);
  wire [15:0] alu_y;
  wire        alu_c, alu_z;

  loom_alu u_alu (
      .sel_alu(sel_alu), .sel_alui(sel_alui), .sel_unary(sel_unary),
      .funct(f_funct), .a(alu_a), .b(alu_b),
      .y(alu_y), .c_out(alu_c), .z_out(alu_z)
  );

  // ------------------------------------------------------ pin sampling
  wire [31:0] pin32  = pin_in_vec;
  wire        pin_sel = pin32[f_pin];
  wire [31:0] prev32 = {19'd0, x_pp};
  wire        pin_prev = prev32[f_pin];
  wire        pin_edge_ok = (f_pin < 5'd13);
  wire        edge_rise = ~pin_prev &  pin_sel;
  wire        edge_fall =  pin_prev & ~pin_sel;
  wire        edge_any  =  pin_prev ^  pin_sel;
  reg         edge_hit;
  always @(*) begin
    case (f_edge)
      2'd0:    edge_hit = edge_rise;
      2'd1:    edge_hit = edge_fall;
      2'd2:    edge_hit = edge_any;
      default: edge_hit = 1'b0;      // e == 3 is never true
    endcase
  end

  // ---------------------------------------------------------- IN / OUT
  wire [4:0]  og_base = x_outgrp[4:0];
  wire [4:0]  og_cnt  = (x_outgrp[9:5] > 5'd16) ? 5'd16 : x_outgrp[9:5];
  wire [16:0] og_m17  = (17'd1 << og_cnt) - 17'd1;
  wire [15:0] og_m16  = og_m17[15:0];
  wire [63:0] og_msh  = {48'd0, og_m16} << og_base;
  wire [31:0] out_mask32 = og_msh[31:0] | og_msh[63:32];
  wire [63:0] og_dsh  = {48'd0, (op_a & og_m16)} << og_base;
  wire [31:0] out_data32 = og_dsh[31:0] | og_dsh[63:32];

  wire [4:0]  ig_base = x_ingrp[4:0];
  wire [4:0]  ig_cnt  = (x_ingrp[9:5] > 5'd16) ? 5'd16 : x_ingrp[9:5];
  wire [16:0] ig_m17  = (17'd1 << ig_cnt) - 17'd1;
  wire [63:0] ig_rot  = {pin32, pin32} >> ig_base;
  wire [15:0] in_value = ig_rot[15:0] & ig_m17[15:0];

  // ------------------------------------- bit engine (6.9, 6.9.1 slice A)
  // One copy of the encoder, stuffer and decoder, in X, selected by xsel
  // exactly as the shifter and the CRC are (D-025, D-026). Per thread only
  // the eight narrow encoder bits in loom_be.
  wire        be_dir   = x_becfg[0];
  wire        be_inv   = x_becfg[1];
  wire        be_crcen = x_becfg[2];
  wire [1:0]  be_enc   = x_becfg[4:3];      // 0 NRZ, 1 NRZI, 2 Manchester
  wire [1:0]  be_stuff = x_becfg[6:5];      // 0 none, 1 USB, 2 CAN
  wire        be_diff  = x_becfg[7];
  wire        is_shift = is_sho | is_shi;

  wire        e_lvl   = x_enc[0];
  wire [2:0]  e_run   = x_enc[3:1];
  wire        e_rval  = x_enc[4];
  wire        e_pend  = x_enc[5];
  wire        e_half  = x_enc[6];
  wire        e_first = x_enc[7];

  wire        be_manch = (be_enc == 2'd2);
  wire        be_nrzi  = (be_enc == 2'd1);
  wire        be_stfg  = (be_stuff != 2'd0);
  // The stuff value: 0 for USB, ~RVAL for CAN (6.9.1).
  wire        stuff_v  = (be_stuff == 2'd2) ? ~e_rval : 1'b0;

  // ---- SHO step 1: the bit sent, x. With ENC = 2 and HALF = 1 there is
  // none (the second half of the bit kept in FIRST); otherwise a pending
  // stuff bit pre-empts the data bit and leaves SR, CNT and the CRC alone.
  wire        sho_b     = be_dir ? x_sr[15] : x_sr[0];
  wire        sho_half2 = be_manch & e_half;
  wire        sho_has   = ~sho_half2;
  wire        sho_stuff = sho_has & be_stfg & e_pend;
  wire        sho_data  = sho_has & ~sho_stuff;
  wire        sho_x     = sho_stuff ? stuff_v : sho_b;

  // ---- SHI steps 1 and 2: the sample p and the bit received, s. With
  // ENC = 2 and HALF = 0 there is no bit (steps 3 and 4 are skipped).
  wire        shi_p      = pin32[x_bepins[9:5]] ^ be_inv;
  wire        shi_half1  = be_manch & ~e_half;
  wire        shi_has    = ~shi_half1;
  wire        shi_s      = be_nrzi ? (shi_p == e_lvl) : shi_p;
  // Manchester: no transition in the middle of the bit is a violation.
  wire        shi_mviol  = be_manch & e_half & (e_first == shi_p);
  // ---- SHI step 3: a pending stuff bit is dropped, and is a violation
  // unless it carries the stuff value.
  wire        shi_stuff  = shi_has & be_stfg & e_pend;
  wire        shi_data   = shi_has & ~shi_stuff;
  wire        shi_sviol  = shi_stuff & (shi_s != stuff_v);

  // ---- common: was there a bit, was it a data bit, and what was it.
  wire        be_has  = is_shi ? shi_has  : sho_has;
  wire        be_data = is_shi ? shi_data : sho_data;
  wire        be_bit  = is_shi ? shi_s    : sho_x;     // shifted and CRC'd
  wire        be_took_stuff = is_shi ? shi_stuff : sho_stuff;

  wire [15:0] be_sr_sh = be_dir ? {x_sr[14:0], is_shi & be_bit}
                                : {is_shi & be_bit, x_sr[15:1]};
  wire [4:0]  be_cnt_d = (x_cnt == 5'd0) ? 5'd0 : (x_cnt - 5'd1);
  // A stuff bit and a Manchester half without a data bit leave CNT alone,
  // but Z still reports CNT after the instruction (6.9.1).
  wire [4:0]  be_cnt_n = be_data ? be_cnt_d : x_cnt;
  // Serial CRC, MSB first on a left-aligned register.
  wire        be_fb    = x_crc[15] ^ be_bit;
  wire [15:0] be_crc_n = {x_crc[14:0], 1'b0} ^ (be_fb ? x_poly : 16'd0);

  // ---- run accounting, on the bit sent or received, data or stuff.
  wire        ra_en    = is_shift & be_has & be_stfg;
  wire        ra_same  = (be_bit == e_rval);
  wire [2:0]  run_inc  = (e_run == 3'd7) ? 3'd7 : (e_run + 3'd1);
  wire [2:0]  run_n    = ra_same ? run_inc : 3'd1;
  wire        rval_n   = be_bit;
  // A new run of six 1s (USB) or of five equal bits (CAN) makes a stuff
  // bit due. A stuff bit itself starts a run of one, so it never chains.
  wire        pend_ra  = (be_stuff == 2'd1) ? ((run_n == 3'd6) & rval_n)
                                            : (run_n == 3'd5);
  wire        pend_s1  = be_took_stuff ? 1'b0 : e_pend;
  wire        enc_pend_n = ra_en ? (pend_s1 | pend_ra) : pend_s1;
  wire [2:0]  enc_run_n  = ra_en ? run_n  : e_run;
  wire        enc_rval_n = ra_en ? rval_n : e_rval;

  // ---- encoding into the level l (SHO step 3) and the state it carries.
  wire        nrzi_l = sho_x ? e_lvl : ~e_lvl;    // a 0 toggles, as USB has it
  reg         sho_l;
  always @(*) begin
    case (be_enc)
      2'd1:    sho_l = nrzi_l;
      // IEEE 802.3: a 0 is high then low, a 1 low then high.
      2'd2:    sho_l = e_half ? e_first : ~sho_x;
      default: sho_l = sho_x;                     // NRZ (3 is stored as 0)
    endcase
  end
  wire        enc_lvl_n   = be_nrzi ? (is_shi ? shi_p : nrzi_l) : e_lvl;
  wire        enc_half_n  = be_manch ? ~e_half : e_half;
  wire        enc_first_n = (be_manch & ~e_half) ? (is_shi ? shi_p : sho_x)
                                                 : e_first;
  wire [7:0]  be_enc_n = {enc_first_n, enc_half_n, enc_pend_n, enc_rval_n,
                          enc_run_n, enc_lvl_n};

  // T is set by an SHI violation and never cleared by SHI (6.9.1).
  wire        be_t_set = is_shi & (shi_mviol | shi_sviol);

  // ------------------------------------------------------- pin writes
  // One single-pin write path (6.3) for SETP and for SHO, which drives
  // b ^ INV onto BE_PINS.out. SETP D writes no pin: it stages (6.10).
  wire        sp_en   = (is_setp & ~f_lat) | is_sho;
  wire [4:0]  sp_pin  = is_sho ? x_bepins[4:0] : f_pin;
  wire        sp_val  = is_sho ? (sho_l ^ be_inv) : f_val;
  wire [31:0] setp_mask32 = 32'd1 << sp_pin;
  // DIFF (6.9.1): the same SHO also writes the complement to
  // (BE_PINS.out + 1) mod 32 at the same edge, as one OUT of a two-pin
  // group would. An index that is not writable falls out of pw_m14, which
  // keeps only 0..7 and 16..21, so it is ignored exactly as in 6.3.
  wire [4:0]  diff_pin = x_bepins[4:0] + 5'd1;
  wire        diff_en  = is_sho & be_diff;
  wire [31:0] diff_mask32 = 32'd1 << diff_pin;
  wire [31:0] pinw_mask = (sp_en ? setp_mask32 : 32'd0)
                          | (diff_en ? diff_mask32 : 32'd0)
                          | (is_out ? out_mask32 : 32'd0);
  wire [31:0] pinw_data = ((sp_en & sp_val) ? setp_mask32 : 32'd0)
                          | ((diff_en & ~sp_val) ? diff_mask32 : 32'd0)
                          | (is_out ? out_data32 : 32'd0);
  wire [13:0] pw_m14 = {pinw_mask[21:16], pinw_mask[7:0]};
  wire [13:0] pw_d14 = {pinw_data[21:16], pinw_data[7:0]};

  // Open drain (SEMANTICS 6.3): for BIDIR pins with OD_MASK set, a write of b
  // drives PIN_OUT to 0 and PIN_OE to ~b.
  wire [7:0] od_oe_mask = pw_m14[7:0] & od_mask_reg;
  wire [7:0] od_oe_data = ~pw_d14[7:0];
  wire [7:0] bidir_data = pw_d14[7:0] & ~od_mask_reg;

  wire csrw_pout = is_csrw & csr_pin_out;
  wire csrw_poe  = is_csrw & csr_pin_oe;
  wire csrw_od   = is_csrw & csr_od_mask;
  wire oep_ok    = is_oep & (f_pin < 5'd8);
  wire [7:0] oep_mask = oep_ok ? (8'd1 << f_pin[2:0]) : 8'd0;

  wire [13:0] x_out_mask = csrw_pout ? 14'h3FFF : {pw_m14[13:8], pw_m14[7:0]};
  wire [13:0] x_out_data = csrw_pout ? op_a[13:0] : {pw_d14[13:8], bidir_data};
  wire [7:0]  x_oe_mask  = csrw_poe ? 8'hFF : (oep_mask | od_oe_mask);
  wire [7:0]  x_oe_data  = csrw_poe ? op_a[7:0]
                                    : (is_oep ? {8{f_val}} : od_oe_data);

  // --------------------------------------------------------- CSR reads
  reg [15:0] csr_rdata;
  always @(*) begin
    csr_rdata = 16'd0;
    if (csr_tick_int)       csr_rdata = x_tint;
    else if (csr_tick_frac) csr_rdata = {8'd0, x_tfrac};
    else if (csr_outgrp)    csr_rdata = {6'd0, x_outgrp};
    else if (csr_ingrp)     csr_rdata = {6'd0, x_ingrp};
    else if (csr_now)       csr_rdata = x_now;
    else if (csr_td)        csr_rdata = x_td;
    else if (csr_flags)     csr_rdata = {13'd0, x_t, x_c, x_z};
    else if (csr_tid)       csr_rdata = {14'd0, tx_th};
    else if (csr_od_mask)   csr_rdata = {8'd0, od_mask_reg};
    else if (csr_pin_out)   csr_rdata = pin_out_reg;
    else if (csr_pin_oe)    csr_rdata = {8'd0, pin_oe_reg};
    else if (csr_pin_in)    csr_rdata = pin_in_reg;
    else if (csr_sflags)    csr_rdata = {8'd0, sflags_x};
    else if (csr_be_cfg)    csr_rdata = becfg_word(x_becfg);
    else if (csr_be_pins)   csr_rdata = {6'd0, x_bepins};
    else if (csr_be_reload) csr_rdata = {11'd0, x_reload};
    else if (csr_crc_poly)  csr_rdata = x_poly;
    else if (csr_crc_init)  csr_rdata = x_init;
    else if (csr_sr)        csr_rdata = x_sr;
    else if (csr_cnt)       csr_rdata = {11'd0, x_cnt};
    else if (csr_crc)       csr_rdata = x_crc;
  end

  // ------------------------------------------------------------- waits
  wire first_issue = ~x_wa;
  wire [15:0] td_new = is_waitd ? (first_issue ? (x_td + f_imm) : x_td)
                                : (is_setd ? (x_now + f_imm) : x_td);
  wire [15:0] dt_new = first_issue ? (x_now + f_imm) : x_dt;
  // reached(a, b) is ((a - b) mod 2^16) < 2^15, i.e. bit 15 of the difference
  // is clear (SEMANTICS 4).
  wire [15:0] d_td_new = x_now - td_new;
  wire [15:0] d_dt_new = x_now - dt_new;
  wire [15:0] d_td     = x_now - x_td;
  wire reach_td_new = ~d_td_new[15];
  wire reach_dt_new = ~d_dt_new[15];
  wire reach_td     = ~d_td[15];

  // Conditional-wait condition mux. WAITB (6.4, 6.7): 0 bit engine idle,
  // which is always true in manual mode, 1 OUTQ not full, 2 INQ not empty,
  // 3 TICK_SEEN.
  wire cond_waitp = (pin_sel == f_val);
  wire cond_waite = pin_edge_ok & edge_hit;
  wire cond_waits = sflags_x[f_flag];
  reg  cond_waitb;
  always @(*) begin
    case (f_cond)
      2'd1:    cond_waitb = x_outq_nf;
      2'd2:    cond_waitb = x_inq_ne;
      2'd3:    cond_waitb = x_tseen;
      default: cond_waitb = 1'b1;
    endcase
  end
  wire cond_hit   = (is_waitp & cond_waitp)
                    | (is_waite & cond_waite)
                    | (is_waits & cond_waits)
                    | (is_waitb & cond_waitb);
  wire is_cwait   = (is_waitp | is_waite | is_waits | is_waitb) & ~bad_op;
  wire timeout    = is_cwait & f_tmo & ~cond_hit & reach_td;

  // PUSH and POP block like waits (WAIT_ACTIVE, PC unchanged) and have no
  // timeout (SEMANTICS 6.7).
  wire wait_class = (tmg_wait | tmg_blocking) & ~bad_op;
  wire wait_done  = (is_waitd & reach_td_new)
                    | (is_dly  & reach_dt_new)
                    | (is_cwait & (cond_hit | (f_tmo & reach_td)))
                    | (is_push & x_outq_nf)
                    | (is_pop  & x_inq_ne);
  wire x_stall    = wait_class & ~wait_done;
  wire x_done     = ~x_stall;

  // --------------------------------------------------------- branches
  wire br_cond = (is_bz  &  x_z) | (is_bnz & ~x_z)
               | (is_bc  &  x_c) | (is_bnc & ~x_c)
               | (is_bt  &  x_t) | (is_bnt & ~x_t);
  wire [15:0] djnz_val = op_b - 16'd1;
  wire br_taken = (br_cond | (is_djnz & (djnz_val != 16'd0))
                   | (is_jp & (pin_sel == f_val))) & ~bad_op;

  reg [9:0] x_next_pc;
  always @(*) begin
    if (x_stall)                 x_next_pc = pcx;
    else if (bad_op)             x_next_pc = x_next;
    else if (is_jmp | is_call)   x_next_pc = f_abs;
    else if (is_ret)             x_next_pc = (x_depth != 2'd0) ? x_rs0 : x_next;
    else if (br_taken)           x_next_pc = x_next + f_rel;
    else                         x_next_pc = x_next;
  end

  // ------------------------------------------------------- write back
  wire x_reg_we = ~bad_op &
                  (grp_alu
                   | (grp_alui & ~is_cmpi)
                   | grp_ldi
                   | (grp_unary & ~is_cmp & ~is_test)
                   | is_djnz | is_in | is_csrr | is_stsr | is_stcrc
                   | (is_pop & x_inq_ne));

  reg [15:0] x_rval;
  always @(*) begin
    if (is_ldi)       x_rval = f_imm;
    else if (is_ldih) x_rval = {f_imm[7:0], op_b[7:0]};
    else if (is_djnz) x_rval = djnz_val;
    else if (is_in)   x_rval = in_value;
    else if (is_csrr) x_rval = csr_rdata;
    else if (is_pop)  x_rval = x_inq_head;
    else if (is_stsr) x_rval = x_sr;
    else if (is_stcrc) x_rval = x_crc;
    else              x_rval = alu_y;
  end

  // ------------------------------------------------------------ flags
  wire alu_flags = (grp_alu | grp_alui | (grp_unary & ~is_mov)) & ~bad_op;
  wire csrw_fl   = is_csrw & csr_flags;
  reg [2:0] x_flags;                      // {T, C, Z} after this slot
  always @(*) begin
    x_flags = {x_t, x_c, x_z};
    if (alu_flags)      x_flags = {x_t, alu_c, alu_z};
    else if (csrw_fl)   x_flags = op_a[2:0];
    else if (is_shift & ~bad_op) x_flags = {x_t | be_t_set, x_c,
                                            be_cnt_n == 5'd0};
    else if (is_cwait & f_tmo & wait_done) x_flags = {timeout, x_c, x_z};
  end

  // -------------------------------------------------- shared flag ops
  wire [7:0] flag_1hot = 8'd1 << f_flag;
  wire [7:0] x_sf_set = ((is_sig ? flag_1hot : 8'd0)
                         | ((is_csrw & csr_sflags) ? op_a[7:0] : 8'd0)) & {8{~bad_op}};
  wire [7:0] x_sf_clr = ((is_clr ? flag_1hot : 8'd0)
                         | ((is_waits & cond_hit) ? flag_1hot : 8'd0)) & {8{~bad_op}};

  // ------------------------------------------------- return stack / misc
  wire x_rs_we = ~bad_op & (is_call | (is_ret & (x_depth != 2'd0)));
  wire [9:0] x_rs0_n = is_call ? x_next : x_rs1;
  wire [9:0] x_rs1_n = is_call ? x_rs0  : x_rs1;
  wire [1:0] x_depth_n = is_call ? ((x_depth == 2'd2) ? 2'd2 : (x_depth + 2'd1))
                                 : (x_depth - 2'd1);

  // A re-issued WAITD has TD' = TD and writes nothing (6.10 counts only a
  // first issue as a TD write).
  wire x_td_we = ~bad_op & ((is_waitd & first_issue) | is_setd | (is_csrw & csr_td));
  wire [15:0] x_td_val = (is_csrw & csr_td) ? op_a : td_new;
  wire x_dt_we = ~bad_op & is_dly;

  // ================================================== pipeline registers
  always @(posedge clk) begin
    if (!rst_n) begin
      ph    <= 2'd0;
      vd    <= 1'b0;  td_th <= 2'd0;  pcd <= 10'd0;
      vx    <= 1'b0;  tx_th <= 2'd0;  pcx <= 10'd0;
      irx   <= 16'd0; op_a  <= 16'd0; op_b <= 16'd0;
    end else begin
      ph    <= ph + 2'd1;
      vd    <= valid_f;
      td_th <= ph;
      pcd   <= pc_f;
      vx    <= vd;
      tx_th <= td_th;
      pcx   <= pcd;
      irx   <= ird;
      op_a  <= rf_rd_a;
      op_b  <= rf_rd_b;
    end
  end

  always @(posedge clk) begin
    if (!rst_n) begin
      w_valid <= 1'b0; w_thread <= 2'd0; w_pc <= 10'd0; w_ir <= 16'd0;
      w_done <= 1'b0; w_reg_we <= 1'b0; w_rd <= 3'd0; w_rval <= 16'd0;
      w_flags <= 3'd0; w_next_pc <= 10'd0; w_wait_active <= 1'b0;
      w_prev_pins <= 13'd0; w_td_we <= 1'b0; w_td <= 16'd0;
      w_dt_we <= 1'b0; w_dt <= 16'd0; w_tint_we <= 1'b0; w_tfrac_we <= 1'b0;
      w_outgrp_we <= 1'b0; w_ingrp_we <= 1'b0; w_csr_val <= 16'd0;
      w_rs_we <= 1'b0; w_rs0 <= 10'd0; w_rs1 <= 10'd0; w_depth <= 2'd0;
      w_halt <= 1'b0; w_badop <= 1'b0; w_sf_set <= 8'd0; w_sf_clr <= 8'd0;
      w_out_mask <= 14'd0; w_out_data <= 14'd0; w_oe_mask <= 8'd0;
      w_oe_data <= 8'd0; w_od_we <= 1'b0; w_od <= 8'd0; w_swirq <= 1'b0;
      w_steps <= 16'd0; w_push <= 1'b0; w_pop <= 1'b0;
      w_sr_we <= 1'b0; w_cnt_we <= 1'b0; w_crc_we <= 1'b0;
      w_sr <= 16'd0; w_crc <= 16'd0; w_cnt <= 5'd0;
      w_enc_we <= 1'b0; w_enc <= 8'd0;
      w_becfg_we <= 1'b0; w_bepins_we <= 1'b0; w_bereload_we <= 1'b0;
      w_crcpoly_we <= 1'b0; w_crcinit_we <= 1'b0;
      w_lat <= 1'b0; w_lat_pin <= 5'd0; w_lat_val <= 1'b0;
      w_tseen <= 1'b0;
    end else begin
      w_valid       <= vx;
      w_thread      <= tx_th;
      w_pc          <= pcx;
      w_ir          <= irx;
      w_done        <= x_done;
      w_reg_we      <= x_reg_we;
      w_rd          <= f_rd;
      w_rval        <= x_rval;
      w_flags       <= x_flags;
      w_next_pc     <= x_next_pc;
      w_wait_active <= wait_class ? x_stall : x_wa;
      w_prev_pins   <= pin32[12:0];
      w_td_we       <= x_td_we;
      w_tseen       <= x_tseen;
      w_td          <= x_td_val;
      w_dt_we       <= x_dt_we;
      w_dt          <= dt_new;
      w_tint_we     <= ~bad_op & is_csrw & csr_tick_int;
      w_tfrac_we    <= ~bad_op & is_csrw & csr_tick_frac;
      w_outgrp_we   <= ~bad_op & is_csrw & csr_outgrp;
      w_ingrp_we    <= ~bad_op & is_csrw & csr_ingrp;
      w_csr_val     <= op_a;
      w_rs_we       <= x_rs_we;
      w_rs0         <= x_rs0_n;
      w_rs1         <= x_rs1_n;
      w_depth       <= x_depth_n;
      w_halt        <= ~bad_op & is_halt;
      w_badop       <= bad_op;
      w_sf_set      <= x_sf_set;
      w_sf_clr      <= x_sf_clr;
      w_out_mask    <= bad_op ? 14'd0 : x_out_mask;
      w_out_data    <= x_out_data;
      w_oe_mask     <= bad_op ? 8'd0 : x_oe_mask;
      w_oe_data     <= x_oe_data;
      w_od_we       <= ~bad_op & csrw_od;
      w_od          <= op_a[7:0];
      w_swirq       <= ~bad_op & is_csrw & csr_host_irq;
      w_steps       <= x_steps1;
      w_push        <= ~bad_op & is_push & x_outq_nf;
      w_pop         <= ~bad_op & is_pop  & x_inq_ne;
      // A stuff bit and a Manchester half are not data bits: they leave SR,
      // CNT and the CRC alone (6.9.1).
      w_sr_we       <= ~bad_op & ((is_shift & be_data) | is_ldsr
                                  | (is_csrw & csr_sr));
      w_sr          <= is_shift ? be_sr_sh : op_a;
      w_cnt_we      <= ~bad_op & ((is_shift & be_data) | (is_csrw & csr_cnt));
      w_cnt         <= is_shift ? be_cnt_d : op_a[4:0];
      w_crc_we      <= ~bad_op & ((is_shift & be_data & be_crcen) | is_crci
                                  | (is_csrw & csr_crc));
      w_crc         <= is_shift ? be_crc_n : (is_crci ? x_init : op_a);
      w_enc_we      <= ~bad_op & is_shift;
      w_enc         <= be_enc_n;
      w_becfg_we    <= ~bad_op & is_csrw & csr_be_cfg;
      w_bepins_we   <= ~bad_op & is_csrw & csr_be_pins;
      w_bereload_we <= ~bad_op & is_csrw & csr_be_reload;
      w_crcpoly_we  <= ~bad_op & is_csrw & csr_crc_poly;
      w_crcinit_we  <= ~bad_op & is_csrw & csr_crc_init;
      w_lat         <= ~bad_op & is_setp & f_lat;
      w_lat_pin     <= f_pin;
      w_lat_val     <= f_val;
    end
  end

  assign cw_valid    = w_valid;
  assign cw_out_mask = w_out_mask;
  assign cw_out_data = w_out_data;
  assign cw_oe_mask  = w_oe_mask;
  assign cw_oe_data  = w_oe_data;
  assign cw_od_we    = w_od_we;
  assign cw_od       = w_od;

  // ==================================================== debug read select
  wire [31:0] dsel     = {30'd0, h_dbg_thread};
  wire [9:0]  g_pc     = pc_all[dsel*10 +: 10];
  wire [2:0]  g_fl     = {t_all[h_dbg_thread], c_all[h_dbg_thread], z_all[h_dbg_thread]};
  wire [15:0] g_td     = td_all_w[dsel*16 +: 16];
  wire [15:0] g_now    = now_all[dsel*16 +: 16];
  wire [15:0] g_dt     = dt_all_w[dsel*16 +: 16];
  wire [9:0]  g_rs0    = rs0_all[dsel*10 +: 10];
  wire [9:0]  g_rs1    = rs1_all[dsel*10 +: 10];
  wire [1:0]  g_depth  = depth_all[dsel*2 +: 2];
  wire [15:0] g_tint   = tick_int_all[dsel*16 +: 16];
  wire [7:0]  g_tfrac  = tick_frac_all[dsel*8 +: 8];
  wire [9:0]  g_outgrp = outgrp_all[dsel*10 +: 10];
  wire [9:0]  g_ingrp  = ingrp_all[dsel*10 +: 10];
  wire [15:0] g_steps  = steps_all[dsel*16 +: 16];
  wire        g_wa     = wa_all[h_dbg_thread];
  wire        g_tseen  = tick_seen_all[h_dbg_thread];
  wire [15:0] g_sr     = be_sr_all[dsel*16 +: 16];
  wire [4:0]  g_cnt    = be_cnt_all[dsel*5 +: 5];
  wire [15:0] g_crc    = be_crc_all[dsel*16 +: 16];
  wire [15:0] g_poly   = be_poly_all[dsel*16 +: 16];
  wire [15:0] g_init   = be_init_all[dsel*16 +: 16];
  wire [4:0]  g_reload = be_reload_all[dsel*5 +: 5];
  wire [7:0]  g_becfg  = be_cfg_all[dsel*8 +: 8];
  wire [7:0]  g_enc    = be_enc_all[dsel*8 +: 8];
  wire [9:0]  g_bepins = be_pins_all[dsel*10 +: 10];
  wire        g_lat_valid = lat_valid_all[h_dbg_thread];
  wire        g_lat_val   = lat_val_all[h_dbg_thread];
  wire [4:0]  g_lat_pin   = lat_pin_all[dsel*5 +: 5];
  wire [FAW:0] g_icnt0 = inq_cnt_all[dsel*(FAW+1) +: FAW+1];
  wire [FAW:0] g_ocnt0 = outq_cnt_all[dsel*(FAW+1) +: FAW+1];
  wire [7:0]  g_icnt   = {{(7-FAW){1'b0}}, g_icnt0};
  wire [7:0]  g_ocnt   = {{(7-FAW){1'b0}}, g_ocnt0};

  reg [15:0] dbg_rd_other;
  always @(*) begin
    case (h_dbg_reg)
      8'h08:   dbg_rd_other = {6'd0, g_pc};
      8'h09:   dbg_rd_other = {13'd0, g_fl};
      8'h0A:   dbg_rd_other = g_td;
      8'h0B:   dbg_rd_other = g_now;
      8'h0C:   dbg_rd_other = g_sr;
      8'h0D:   dbg_rd_other = {11'd0, g_cnt};
      8'h0E:   dbg_rd_other = g_crc;
      8'h0F:   dbg_rd_other = {6'd0, g_rs0};
      8'h10:   dbg_rd_other = g_tint;
      8'h11:   dbg_rd_other = {8'd0, g_tfrac};
      8'h12:   dbg_rd_other = {6'd0, g_outgrp};
      8'h13:   dbg_rd_other = {6'd0, g_ingrp};
      8'h14:   dbg_rd_other = becfg_word(g_becfg);
      8'h15:   dbg_rd_other = {6'd0, g_bepins};
      8'h16:   dbg_rd_other = {11'd0, g_reload};
      8'h17:   dbg_rd_other = g_poly;
      8'h18:   dbg_rd_other = g_init;
      8'h19:   dbg_rd_other = g_now;
      8'h1A:   dbg_rd_other = g_td;
      8'h1B:   dbg_rd_other = {13'd0, g_fl};
      8'h1C:   dbg_rd_other = {14'd0, h_dbg_thread};
      8'h1D:   dbg_rd_other = g_sr;
      8'h1E:   dbg_rd_other = {11'd0, g_cnt};
      8'h1F:   dbg_rd_other = g_crc;
      8'h20:   dbg_rd_other = g_steps;
      8'h21:   dbg_rd_other = {4'd0, g_depth, g_rs1};
      8'h22:   dbg_rd_other = {15'd0, g_wa};
      8'h23:   dbg_rd_other = g_dt;
      8'h24:   dbg_rd_other = {15'd0, g_tseen};
      8'h25:   dbg_rd_other = {9'd0, g_lat_valid, g_lat_val, g_lat_pin};
      8'h26:   dbg_rd_other = {g_icnt, g_ocnt};
      // {8'b0, FIRST, HALF, PEND, RVAL, RUN[2:0], LVL} (6.9.1).
      8'h27:   dbg_rd_other = {8'd0, g_enc};
      default: dbg_rd_other = 16'd0;
    endcase
  end

  wire dbg_wr_done = h_dbg_req & h_dbg_wr & ~h_dbg_ack
                     & (dbg_running | ~dbg_is_reg | ~(w_valid & w_reg_we));
  wire dbg_rd_done = h_dbg_req & ~h_dbg_wr & ~h_dbg_ack
                     & (~dbg_is_reg | dbg_running | ~vd);
  wire dbg_wr_go   = dbg_wr_done & ~dbg_running;

  always @(posedge clk) begin
    if (!rst_n) begin
      h_dbg_ack   <= 1'b0;
      h_dbg_rdata <= 16'd0;
    end else begin
      h_dbg_ack <= dbg_wr_done | dbg_rd_done;
      if (dbg_rd_done)
        h_dbg_rdata <= (dbg_is_reg & ~dbg_running) ? rf_rd_a : dbg_rd_other;
    end
  end

  assign tmr_h_we       = dbg_wr_go;
  assign tmr_h_td_we    = (h_dbg_reg == 8'h0A) | (h_dbg_reg == 8'h1A);
  assign tmr_h_dt_we    = (h_dbg_reg == 8'h23);
  assign tmr_h_tint_we  = (h_dbg_reg == 8'h10);
  assign tmr_h_tfrac_we = (h_dbg_reg == 8'h11);
  assign tmr_h_tseen_we = (h_dbg_reg == 8'h24);
  assign tmr_h_wdata    = h_dbg_wdata;

  // ============================================ architectural state commit
  // RESET_PC[t] defaults to t * (IMEM_WORDS / 4): 0, 64, 128, 192 for a
  // 256-word memory, 0, 0x100, 0x200, 0x300 for 1024 words.
  localparam [9:0] RPC_STEP = IMEM_WORDS[11:2];
  localparam [9:0] RPC0 = 10'd0;
  localparam [9:0] RPC1 = RPC_STEP;
  localparam [9:0] RPC2 = RPC_STEP + RPC_STEP;
  localparam [9:0] RPC3 = RPC_STEP + RPC_STEP + RPC_STEP;

  // Host-side selects (slow paths): which thread a debug write or a
  // RESET_PC write addresses, and which debug register it is.
  wire [3:0] dbg_we   = {4{dbg_wr_go}} & (4'd1 << h_dbg_thread);
  wire [3:0] rpc_we   = {4{h_rpc_we}} & (4'd1 << h_rpc_sel);
  wire       dw_pc    = (h_dbg_reg == 8'h08);
  wire       dw_flags = (h_dbg_reg == 8'h09) | (h_dbg_reg == 8'h1B);
  wire       dw_rs0   = (h_dbg_reg == 8'h0F);
  wire       dw_og    = (h_dbg_reg == 8'h12);
  wire       dw_ig    = (h_dbg_reg == 8'h13);
  wire       dw_steps = (h_dbg_reg == 8'h20);
  wire       dw_rs1   = (h_dbg_reg == 8'h21);
  wire       dw_wa    = (h_dbg_reg == 8'h22);
  wire       dw_lat   = (h_dbg_reg == 8'h25);

  // Bit-engine state: committed by the W stage, written by the host while
  // the thread is halted (SR 0x0C/0x1D, CNT 0x0D/0x1E, CRC 0x0E/0x1F, the
  // configuration CSRs through the CSR window 0x14..0x18).
  loom_be u_be (
      .clk(clk), .rst_n(rst_n),
      .cm_sel({4{w_valid}} & woh_be),
      .cm_sr_we(w_sr_we), .cm_sr(w_sr),
      .cm_cnt_we(w_cnt_we), .cm_cnt(w_cnt),
      .cm_crc_we(w_crc_we), .cm_crc(w_crc),
      .cm_cfg_we(w_becfg_we), .cm_pins_we(w_bepins_we),
      .cm_reload_we(w_bereload_we), .cm_poly_we(w_crcpoly_we),
      .cm_init_we(w_crcinit_we), .cm_csr(w_csr_val),
      .cm_enc_we(w_enc_we), .cm_enc(w_enc),
      .h_sel(dbg_we),
      .h_sr_we((h_dbg_reg == 8'h0C) | (h_dbg_reg == 8'h1D)),
      .h_cnt_we((h_dbg_reg == 8'h0D) | (h_dbg_reg == 8'h1E)),
      .h_crc_we((h_dbg_reg == 8'h0E) | (h_dbg_reg == 8'h1F)),
      .h_cfg_we(h_dbg_reg == 8'h14),
      .h_pins_we(h_dbg_reg == 8'h15),
      .h_reload_we(h_dbg_reg == 8'h16),
      .h_poly_we(h_dbg_reg == 8'h17),
      .h_init_we(h_dbg_reg == 8'h18),
      .h_enc_we(h_dbg_reg == 8'h27),
      .h_wdata(h_dbg_wdata),
      .h_reset(h_reset),
      .sr_all(be_sr_all), .cnt_all(be_cnt_all), .crc_all(be_crc_all),
      .cfg_all(be_cfg_all), .pins_all(be_pins_all), .reload_all(be_reload_all),
      .poly_all(be_poly_all), .init_all(be_init_all), .enc_all(be_enc_all)
  );

  // Per-thread commit strobes of the fields a slot writes only sometimes.
  wire [3:0] cw_rs  = cw_aux & {4{w_rs_we}};
  wire [3:0] cw_og  = cw_aux & {4{w_outgrp_we}};
  wire [3:0] cw_ig  = cw_aux & {4{w_ingrp_we}};
  wire [3:0] cw_hlt = cw_pc  & {4{w_halt}};
  wire [3:0] cw_lat = {4{w_valid & w_lat}} & woh_lat;

  // ======================================= deadline-latched SETP (6.10)
  // lat_go[t]: thread t's staged write applies at the coming edge. It was
  // valid before that edge (so never the write being armed at it) and
  // CTRL.RESET discards instead of applying.
  wire [3:0]  lat_go = lat_valid_all & lat_fire & ~h_reset;

  // One pin write per thread (6.3), open drain from OD_MASK as visible now.
  // The decode depends only on the latch registers; lat_go gates it last.
  wire [55:0] lat_m14;
  genvar gl;
  generate
    for (gl = 0; gl < 4; gl = gl + 1) begin : g_lat
      wire [31:0] sel = 32'd1 << lat_pin_all[5*gl +: 5];
      assign lat_m14[14*gl +: 14] = {sel[21:16], sel[7:0]} & {14{lat_go[gl]}};
      wire _unused_sel = &{1'b0, sel[31:22], sel[15:8]};
    end
  endgenerate
  wire [13:0] lm0 = lat_m14[13:0],  lm1 = lat_m14[27:14];
  wire [13:0] lm2 = lat_m14[41:28], lm3 = lat_m14[55:42];
  wire [13:0] lv0 = lm0 & {14{lat_val_all[0]}}, lv1 = lm1 & {14{lat_val_all[1]}};
  wire [13:0] lv2 = lm2 & {14{lat_val_all[2]}}, lv3 = lm3 & {14{lat_val_all[3]}};
  // Two threads on one pin at one edge: the higher thread wins.
  wire [13:0] lw_m = lm0 | lm1 | lm2 | lm3;
  wire [13:0] lw_b = lv3 | (~lm3 & (lv2 | (~lm2 & (lv1 | (~lm1 & lv0)))));
  assign lw_out_mask = lw_m;
  assign lw_out_data = lw_b & {6'h3F, ~od_mask_reg};
  assign lw_oe_mask  = lw_m[7:0] & od_mask_reg;
  assign lw_oe_data  = ~lw_b[7:0];

  // Every per-thread register below is written thread by thread with a
  // constant index: the commit of the slot in W (highest priority, the last
  // mux before the flop), then a host debug write (only while the thread is
  // halted, so never together with a commit), then CTRL.RESET. This is the
  // M1 priority order, bit for bit.
  integer i;
  always @(posedge clk) begin
    if (!rst_n) begin
      pc_all     <= {RPC3, RPC2, RPC1, RPC0};
      resetpc_r  <= {RPC3, RPC2, RPC1, RPC0};
      z_all      <= 4'd0;
      c_all      <= 4'd0;
      t_all      <= 4'd0;
      wa_all     <= 4'd0;
      rs0_all    <= 40'd0;
      rs1_all    <= 40'd0;
      depth_all  <= 8'd0;
      pp_all     <= 52'd0;
      outgrp_all <= 40'd0;
      ingrp_all  <= 40'd0;
      steps_all  <= 64'd0;
      run_r      <= 4'd0;
      halted_r   <= 4'd0;
      step_req_r <= 4'd0;
      badop_r    <= 16'd0;
      sflags_r   <= 8'd0;
      swirq_r    <= 4'd0;
      lat_valid_all <= 4'd0;
      lat_val_all   <= 4'd0;
      lat_pin_all   <= 20'd0;
    end else begin
      for (i = 0; i < 4; i = i + 1) begin
        // ---------------------------------------------- host RESET_PC write
        if (rpc_we[i]) resetpc_r[i*10 +: 10] <= h_rpc;

        // ------------------------------------------ PC and WAIT_ACTIVE
        if (cw_pc[i])                   pc_all[i*10 +: 10] <= w_next_pc;
        else if (dbg_we[i] & dw_pc)     pc_all[i*10 +: 10] <= h_dbg_wdata[9:0];
        else if (h_reset[i])            pc_all[i*10 +: 10] <= resetpc_r[i*10 +: 10];

        // A debug PC write also clears WAIT_ACTIVE (SEMANTICS 7).
        if (cw_pc[i])                   wa_all[i] <= w_wait_active;
        else if (dbg_we[i] & dw_pc)     wa_all[i] <= 1'b0;
        else if (dbg_we[i] & dw_wa)     wa_all[i] <= h_dbg_wdata[0];
        else if (h_reset[i])            wa_all[i] <= 1'b0;

        // --------------------------------------------------------- flags
        if (cw_pc[i]) begin
          z_all[i] <= w_flags[0];
          c_all[i] <= w_flags[1];
          t_all[i] <= w_flags[2];
        end else if (dbg_we[i] & dw_flags) begin
          z_all[i] <= h_dbg_wdata[0];
          c_all[i] <= h_dbg_wdata[1];
          t_all[i] <= h_dbg_wdata[2];
        end else if (h_reset[i]) begin
          z_all[i] <= 1'b0;
          c_all[i] <= 1'b0;
          t_all[i] <= 1'b0;
        end

        // -------------------------------------------------- return stack
        if (cw_rs[i])                   rs0_all[i*10 +: 10] <= w_rs0;
        else if (dbg_we[i] & dw_rs0)    rs0_all[i*10 +: 10] <= h_dbg_wdata[9:0];

        if (cw_rs[i])                   rs1_all[i*10 +: 10] <= w_rs1;
        else if (dbg_we[i] & dw_rs1)    rs1_all[i*10 +: 10] <= h_dbg_wdata[9:0];

        if (cw_rs[i])                   depth_all[i*2 +: 2] <= w_depth;
        else if (dbg_we[i] & dw_rs1)    depth_all[i*2 +: 2] <= h_dbg_wdata[11:10];
        else if (h_reset[i])            depth_all[i*2 +: 2] <= 2'd0;

        // --------------------------------------- PREV_PINS, STEPS, groups
        if (cw_aux[i])                  pp_all[i*13 +: 13] <= w_prev_pins;

        if (cw_aux[i])                  steps_all[i*16 +: 16] <= w_steps;
        else if (dbg_we[i] & dw_steps)  steps_all[i*16 +: 16] <= h_dbg_wdata;

        if (cw_og[i])                   outgrp_all[i*10 +: 10] <= w_csr_val[9:0];
        else if (dbg_we[i] & dw_og)     outgrp_all[i*10 +: 10] <= h_dbg_wdata[9:0];

        if (cw_ig[i])                   ingrp_all[i*10 +: 10] <= w_csr_val[9:0];
        else if (dbg_we[i] & dw_ig)     ingrp_all[i*10 +: 10] <= h_dbg_wdata[9:0];

        // ------------------------------------------------- RUN / HALTED
        // CTRL.RUN write: RUN <= value, bits going 0 -> 1 clear HALTED; a
        // HALT committing at the same edge wins (the thread always wins).
        if (cw_hlt[i])                  run_r[i] <= 1'b0;
        else if (h_run_we)              run_r[i] <= h_run[i];

        if (cw_hlt[i])                  halted_r[i] <= 1'b1;
        else if (h_run_we & h_run[i] & ~run_r[i]) halted_r[i] <= 1'b0;

        // ------------------------------------------ deadline latch (6.10)
        // A SETP D commit stages (replacing, and winning over the clear of
        // a write applied at the same edge); a debug write of 0x25 sets it
        // while halted; CTRL.RESET discards; an applied write clears VALID.
        if (cw_lat[i]) begin
          lat_valid_all[i]      <= 1'b1;
          lat_pin_all[i*5 +: 5] <= w_lat_pin;
          lat_val_all[i]        <= w_lat_val;
        end else if (dbg_we[i] & dw_lat) begin
          lat_valid_all[i]      <= h_dbg_wdata[6];
          lat_pin_all[i*5 +: 5] <= h_dbg_wdata[4:0];
          lat_val_all[i]        <= h_dbg_wdata[5];
        end else if (h_reset[i] | lat_go[i]) begin
          lat_valid_all[i]      <= 1'b0;
        end
      end

      // ------------------------------------------------------------ STEP
      if (valid_f && step_req_r[ph]) step_req_r[ph] <= 1'b0;
      if (h_step_we) step_req_r <= step_req_r | (h_step & ~run_r);

      // --------------------------------------------------------- BADOP
      badop_r <= (badop_r & ~(h_badop_clr_we ? h_badop_clr : 16'd0))
                 | {h_badop_set15, h_badop_set14 | h_push_bad, 10'd0,
                    cw_pc & {4{w_badop}}};

      // -------------------------------------------------------- SFLAGS
      sflags_r <= ((((sflags_r | (h_sfset_we ? h_sfset : 8'd0))
                     & ~(h_sfclr_we ? h_sfclr : 8'd0))
                    | (w_valid ? w_sf_set : 8'd0))
                   & ~(w_valid ? w_sf_clr : 8'd0));

      // --------------------------------------------------------- SWIRQ
      swirq_r <= (swirq_r & ~(h_swirq_clr_we ? h_swirq_clr : 4'd0))
                 | (cw_pc & {4{w_swirq}});
    end
  end

  assign core_busy = (|run_r) | (|step_req_r) | vd | vx | w_valid;

  // ==================================================== retire record
  assign tr_valid   = w_valid;
  assign tr_thread  = w_thread;
  assign tr_pc      = w_pc;
  assign tr_ir      = w_ir;
  assign tr_done    = w_done;
  assign tr_we      = w_reg_we;
  assign tr_rd      = w_rd;
  assign tr_val     = w_rval;
  assign tr_flags   = w_flags;
  assign tr_next_pc = w_next_pc;

  // The mask bits above 16 of the group shifters, the high difference bits
  // of the reached() subtractions and INQ's second read port are carried
  // but not consumed.
  wire _unused_m2 = &{1'b0, inq_next_unused, og_m17[16], ig_m17[16],
                      ig_rot[63:16], pinw_mask[31:22], pinw_mask[15:8],
                      pinw_data[31:22], pinw_data[15:8],
                      d_td_new[14:0], d_dt_new[14:0], d_td[14:0]};

endmodule
