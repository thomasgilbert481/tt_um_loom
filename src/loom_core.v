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
 * M1 does not build the FIFOs, the bit engine, WAITB or LD/ST. Those decode
 * normally and execute as NOP with BADOP[t] set (SEMANTICS section 9); the
 * wait-condition mux has spare inputs so WAITB can be added without a
 * rewrite.
 */

`default_nettype none

module loom_core #(
    parameter integer IMEM_AW    = 8,
    parameter [15:0]  IMEM_WORDS = 16'd256
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

  // ======================================================= register file
  // A thread counts as halted for host access to r0..r7 and for every debug
  // write iff RUN[t] and STEP_REQ[t] are clear and it has no valid slot in
  // F, D, X or W, so the state the host sees is always architectural.
  wire [3:0] busy_f = valid_f  ? (4'd1 << ph)       : 4'd0;
  wire [3:0] busy_d = vd       ? (4'd1 << td_th)    : 4'd0;
  wire [3:0] busy_x = vx       ? (4'd1 << tx_th)    : 4'd0;
  wire [3:0] busy_w = w_valid  ? (4'd1 << w_thread) : 4'd0;
  wire [3:0] thread_busy = run_r | step_req_r | busy_f | busy_d | busy_x | busy_w;

  wire        dbg_is_reg   = (h_dbg_reg[7:3] == 5'd0);
  wire        dbg_running  = thread_busy[h_dbg_thread];
  wire        dbg_rf_rd_go = h_dbg_req & ~h_dbg_wr & dbg_is_reg & ~dbg_running & ~vd & ~h_dbg_ack;
  wire        dbg_rf_wr_go = h_dbg_req &  h_dbg_wr & dbg_is_reg & ~dbg_running
                             & ~(w_valid & w_reg_we) & ~h_dbg_ack;

  wire [1:0] rf_ra_thread = dbg_rf_rd_go ? h_dbg_thread : td_th;
  wire [2:0] rf_ra_addr   = dbg_rf_rd_go ? h_dbg_reg[2:0] : d_f_ra;
  wire [2:0] rf_rb_addr   = d_grp_alu ? d_f_rb : d_f_rd;
  wire [15:0] rf_rd_a, rf_rd_b;

  loom_regfile u_rf (
      .clk(clk), .rst_n(rst_n),
      .ra_thread(rf_ra_thread), .ra_addr(rf_ra_addr), .rd_a(rf_rd_a),
      .rb_thread(td_th),        .rb_addr(rf_rb_addr), .rd_b(rf_rd_b),
      .we      (dbg_rf_wr_go ? 1'b1          : (w_valid & w_reg_we)),
      .w_thread(dbg_rf_wr_go ? h_dbg_thread  : w_thread),
      .w_addr  (dbg_rf_wr_go ? h_dbg_reg[2:0]: w_rd),
      .wdata   (dbg_rf_wr_go ? h_dbg_wdata   : w_rval)
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
  wire        tmr_h_we, tmr_h_td_we, tmr_h_dt_we, tmr_h_tint_we, tmr_h_tfrac_we;
  wire [15:0] tmr_h_wdata;

  loom_timer u_timer (
      .clk(clk), .rst_n(rst_n),
      .cm_valid(w_valid), .cm_thread(w_thread),
      .cm_td_we(w_td_we), .cm_td(w_td),
      .cm_dt_we(w_dt_we), .cm_dt(w_dt),
      .cm_tint_we(w_tint_we), .cm_tint(w_csr_val),
      .cm_tfrac_we(w_tfrac_we), .cm_tfrac(w_csr_val[7:0]),
      .h_reset(h_reset),
      .h_we(tmr_h_we), .h_thread(h_dbg_thread),
      .h_td_we(tmr_h_td_we), .h_dt_we(tmr_h_dt_we),
      .h_tint_we(tmr_h_tint_we), .h_tfrac_we(tmr_h_tfrac_we),
      .h_wdata(tmr_h_wdata),
      .now_all(now_all), .td_all(td_all_w), .dt_all(dt_all_w),
      .tick_int_all(tick_int_all), .tick_frac_all(tick_frac_all),
      .tick_seen_all(tick_seen_all)
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

  // SFLAGS is forwarded from W to X so that WAITS is an atomic test-and-clear
  // between slots in adjacent cycles (SEMANTICS 2).
  wire [7:0] sflags_x = (sflags_r | (w_valid ? w_sf_set : 8'd0))
                        & ~(w_valid ? w_sf_clr : 8'd0);

  // -------------------------------------------------- not built at M1
  wire unbuilt = tmg_blocking | grp_be | grp_mem | is_waitb;
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

  // ------------------------------------------------------- pin writes
  wire [31:0] setp_mask32 = 32'd1 << f_pin;
  wire [31:0] pinw_mask = (is_setp ? setp_mask32 : 32'd0)
                          | (is_out ? out_mask32 : 32'd0);
  wire [31:0] pinw_data = (is_setp ? (f_val ? setp_mask32 : 32'd0) : 32'd0)
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

  // Conditional-wait condition mux. Input 3 (WAITB) is reserved for M2 and is
  // tied to 0 here; the instruction is treated as unbuilt (BADOP) at M1.
  wire cond_waitp = (pin_sel == f_val);
  wire cond_waite = pin_edge_ok & edge_hit;
  wire cond_waits = sflags_x[f_flag];
  wire cond_hit   = (is_waitp & cond_waitp)
                    | (is_waite & cond_waite)
                    | (is_waits & cond_waits);
  wire is_cwait   = (is_waitp | is_waite | is_waits) & ~bad_op;
  wire timeout    = is_cwait & f_tmo & ~cond_hit & reach_td;

  wire wait_class = tmg_wait & ~bad_op;
  wire wait_done  = (is_waitd & reach_td_new)
                    | (is_dly  & reach_dt_new)
                    | (is_cwait & (cond_hit | (f_tmo & reach_td)));
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
                   | is_djnz | is_in | is_csrr);

  reg [15:0] x_rval;
  always @(*) begin
    if (is_ldi)       x_rval = f_imm;
    else if (is_ldih) x_rval = {f_imm[7:0], op_b[7:0]};
    else if (is_djnz) x_rval = djnz_val;
    else if (is_in)   x_rval = in_value;
    else if (is_csrr) x_rval = csr_rdata;
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

  wire x_td_we = ~bad_op & (is_waitd | is_setd | (is_csrw & csr_td));
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

  reg [15:0] dbg_rd_other;
  always @(*) begin
    case (h_dbg_reg)
      8'h08:   dbg_rd_other = {6'd0, g_pc};
      8'h09:   dbg_rd_other = {13'd0, g_fl};
      8'h0A:   dbg_rd_other = g_td;
      8'h0B:   dbg_rd_other = g_now;
      8'h0F:   dbg_rd_other = {6'd0, g_rs0};
      8'h10:   dbg_rd_other = g_tint;
      8'h11:   dbg_rd_other = {8'd0, g_tfrac};
      8'h12:   dbg_rd_other = {6'd0, g_outgrp};
      8'h13:   dbg_rd_other = {6'd0, g_ingrp};
      8'h19:   dbg_rd_other = g_now;
      8'h1A:   dbg_rd_other = g_td;
      8'h1B:   dbg_rd_other = {13'd0, g_fl};
      8'h1C:   dbg_rd_other = {14'd0, h_dbg_thread};
      8'h20:   dbg_rd_other = g_steps;
      8'h21:   dbg_rd_other = {4'd0, g_depth, g_rs1};
      8'h22:   dbg_rd_other = {15'd0, g_wa};
      8'h23:   dbg_rd_other = g_dt;
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
  assign tmr_h_wdata    = h_dbg_wdata;

  // ============================================ architectural state commit
  // RESET_PC[t] defaults to t * (IMEM_WORDS / 4): 0, 64, 128, 192 for a
  // 256-word memory, 0, 0x100, 0x200, 0x300 for 1024 words.
  localparam [9:0] RPC_STEP = IMEM_WORDS[11:2];
  localparam [9:0] RPC0 = 10'd0;
  localparam [9:0] RPC1 = RPC_STEP;
  localparam [9:0] RPC2 = RPC_STEP + RPC_STEP;
  localparam [9:0] RPC3 = RPC_STEP + RPC_STEP + RPC_STEP;

  wire [31:0] wsel = {30'd0, w_thread};
  wire [31:0] rsel = {30'd0, h_rpc_sel};

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
    end else begin
      // ------------------------------------------------ host RESET_PC write
      if (h_rpc_we) resetpc_r[rsel*10 +: 10] <= h_rpc;

      // ------------------------------------------------ CTRL.RESET (host)
      for (i = 0; i < 4; i = i + 1) begin
        if (h_reset[i]) begin
          pc_all[i*10 +: 10]   <= resetpc_r[i*10 +: 10];
          z_all[i]             <= 1'b0;
          c_all[i]             <= 1'b0;
          t_all[i]             <= 1'b0;
          depth_all[i*2 +: 2]  <= 2'd0;
          wa_all[i]            <= 1'b0;
        end
      end

      // ------------------------------- host debug writes (thread not running)
      if (dbg_wr_go) begin
        case (h_dbg_reg)
          8'h08: begin
            pc_all[dsel*10 +: 10] <= h_dbg_wdata[9:0];
            wa_all[h_dbg_thread]  <= 1'b0;      // a PC write clears WAIT_ACTIVE
          end
          8'h09, 8'h1B: begin
            z_all[h_dbg_thread] <= h_dbg_wdata[0];
            c_all[h_dbg_thread] <= h_dbg_wdata[1];
            t_all[h_dbg_thread] <= h_dbg_wdata[2];
          end
          8'h0F: rs0_all[dsel*10 +: 10]    <= h_dbg_wdata[9:0];
          8'h12: outgrp_all[dsel*10 +: 10] <= h_dbg_wdata[9:0];
          8'h13: ingrp_all[dsel*10 +: 10]  <= h_dbg_wdata[9:0];
          8'h20: steps_all[dsel*16 +: 16]  <= h_dbg_wdata;
          8'h21: begin
            rs1_all[dsel*10 +: 10] <= h_dbg_wdata[9:0];
            depth_all[dsel*2 +: 2] <= h_dbg_wdata[11:10];
          end
          8'h22: wa_all[h_dbg_thread] <= h_dbg_wdata[0];
          default: ;
        endcase
      end

      // ------------------------------------------------------ thread commit
      if (w_valid) begin
        pc_all[wsel*10 +: 10]    <= w_next_pc;
        z_all[w_thread]          <= w_flags[0];
        c_all[w_thread]          <= w_flags[1];
        t_all[w_thread]          <= w_flags[2];
        wa_all[w_thread]         <= w_wait_active;
        pp_all[wsel*13 +: 13]    <= w_prev_pins;
        steps_all[wsel*16 +: 16] <= steps_all[wsel*16 +: 16] + 16'd1;
        if (w_rs_we) begin
          rs0_all[wsel*10 +: 10] <= w_rs0;
          rs1_all[wsel*10 +: 10] <= w_rs1;
          depth_all[wsel*2 +: 2] <= w_depth;
        end
        if (w_outgrp_we) outgrp_all[wsel*10 +: 10] <= w_csr_val[9:0];
        if (w_ingrp_we)  ingrp_all[wsel*10 +: 10]  <= w_csr_val[9:0];
      end

      // ------------------------------------------------- RUN / HALTED / STEP
      if (h_run_we) begin
        run_r    <= h_run;
        halted_r <= halted_r & ~(h_run & ~run_r);   // 0 -> 1 clears HALTED
      end
      if (valid_f && step_req_r[ph]) step_req_r[ph] <= 1'b0;
      if (h_step_we) step_req_r <= step_req_r | (h_step & ~run_r);
      if (w_valid && w_halt) begin                   // the thread always wins
        run_r[w_thread]    <= 1'b0;
        halted_r[w_thread] <= 1'b1;
      end

      // --------------------------------------------------------- BADOP
      badop_r <= (badop_r & ~(h_badop_clr_we ? h_badop_clr : 16'd0))
                 | {h_badop_set15, 11'd0, (w_valid & w_badop) ? (4'd1 << w_thread) : 4'd0};

      // -------------------------------------------------------- SFLAGS
      sflags_r <= ((((sflags_r | (h_sfset_we ? h_sfset : 8'd0))
                     & ~(h_sfclr_we ? h_sfclr : 8'd0))
                    | (w_valid ? w_sf_set : 8'd0))
                   & ~(w_valid ? w_sf_clr : 8'd0));

      // --------------------------------------------------------- SWIRQ
      swirq_r <= (swirq_r & ~(h_swirq_clr_we ? h_swirq_clr : 4'd0))
                 | ((w_valid & w_swirq) ? (4'd1 << w_thread) : 4'd0);
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

  // TICK_SEEN is architectural state that only WAITB (M2) reads; the counter
  // and the mask bits above 16 of the group shifters are likewise carried but
  // not consumed at M1.
  wire _unused_m2 = &{1'b0, tick_seen_all, og_m17[16], ig_m17[16],
                      ig_rot[63:16], pinw_mask[31:22], pinw_mask[15:8],
                      pinw_data[31:22], pinw_data[15:8],
                      d_td_new[14:0], d_dt_new[14:0], d_td[14:0]};

endmodule
