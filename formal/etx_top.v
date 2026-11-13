/*
 * loom_etx_fv: harness for ETX-1, ETX-2 and ETX-5 (SEMANTICS 6.12.10), the
 * serializer beside the pins as loom_top wires them (D-036).
 * SPDX-License-Identifier: Apache-2.0
 *
 * Formal only. u_etx and u_pins are wired as in loom_top: the block's busy,
 * rdy, tdp and tdn go into loom_pins, whose out_pins are uo_out[5:0]. u_ref
 * is a second loom_pins on the same inputs with the block absent (its four
 * inputs held at 0, as in a build without ETX, 6.12): ETX-5 says the block
 * changes nothing the reference does not also show. Everything else (the
 * three pin write ports, the pads, the ETX write port) is a free input.
 * loom_core is not here; its side of the write port is SCHED-ETX.
 *
 * Observable (6.12.7): uo_out[1:0] = out_pins[1:0], BUSY = pin_in(14),
 * ETX_CFG through u_etx.cfg_rd, PIN_OUT[9:8] through pin_out_reg.
 */

`default_nettype none

module loom_etx_fv (
    input wire        clk,
    input wire        rst_n,
    input wire        w_cfg_we, w_data_we, w_last_we, w_last1_we, w_stat_we,
    input wire [15:0] w_val,
    input wire [12:0] pad_in,
    input wire        cw_valid,
    input wire [13:0] cw_out_mask, cw_out_data,
    input wire [7:0]  cw_oe_mask, cw_oe_data,
    input wire        cw_od_we,
    input wire [7:0]  cw_od,
    input wire [13:0] lw_out_mask, lw_out_data,
    input wire [7:0]  lw_oe_mask, lw_oe_data,
    input wire        h_out_we,
    input wire [15:0] h_out,
    input wire        h_oe_we,
    input wire [7:0]  h_oe,
    input wire        h_od_we,
    input wire [7:0]  h_od
);

  reg f_past_valid = 1'b0;
  always @(posedge clk) f_past_valid <= 1'b1;
  always @(*) if (!f_past_valid) assume (!rst_n);

  // 6.12.2: "Only one slot is in W at any edge, so at most one ETX write
  // commits at any edge." The caller's duty; SCHED-ETX proves loom_core
  // keeps it.
  wire [2:0] f_nw = w_cfg_we + w_data_we + w_last_we + w_last1_we + w_stat_we;
  always @(*) assume (f_nw <= 3'd1);

  wire [9:0]  cfg_rd;
  wire [4:0]  stat_rd;
  wire        busy, rdy, tdp, tdn;
  loom_etx u_etx (
      .clk(clk), .rst_n(rst_n),
      .w_cfg_we(w_cfg_we), .w_data_we(w_data_we), .w_last_we(w_last_we),
      .w_last1_we(w_last1_we), .w_stat_we(w_stat_we), .w_val(w_val),
      .cfg_rd(cfg_rd), .stat_rd(stat_rd),
      .busy(busy), .rdy(rdy), .tdp(tdp), .tdn(tdn));

  wire [31:0] piv, piv_r;
  wire [15:0] pir, pir_r, por, por_r;
  wire [7:0]  poe, poe_r, odm, odm_r, uo, uo_r, uoe, uoe_r;
  wire [5:0]  op, op_r;
  loom_pins u_pins (
      .clk(clk), .rst_n(rst_n), .pad_in(pad_in),
      .cw_valid(cw_valid), .cw_out_mask(cw_out_mask), .cw_out_data(cw_out_data),
      .cw_oe_mask(cw_oe_mask), .cw_oe_data(cw_oe_data),
      .cw_od_we(cw_od_we), .cw_od(cw_od),
      .lw_out_mask(lw_out_mask), .lw_out_data(lw_out_data),
      .lw_oe_mask(lw_oe_mask), .lw_oe_data(lw_oe_data),
      .h_out_we(h_out_we), .h_out(h_out), .h_oe_we(h_oe_we), .h_oe(h_oe),
      .h_od_we(h_od_we), .h_od(h_od),
      .etx_rdy(rdy), .etx_busy(busy), .etx_tdp(tdp), .etx_tdn(tdn),
      .pin_in_vec(piv), .pin_in_reg(pir), .pin_out_reg(por),
      .pin_oe_reg(poe), .od_mask_reg(odm),
      .uio_out(uo), .uio_oe(uoe), .out_pins(op));
  loom_pins u_ref (
      .clk(clk), .rst_n(rst_n), .pad_in(pad_in),
      .cw_valid(cw_valid), .cw_out_mask(cw_out_mask), .cw_out_data(cw_out_data),
      .cw_oe_mask(cw_oe_mask), .cw_oe_data(cw_oe_data),
      .cw_od_we(cw_od_we), .cw_od(cw_od),
      .lw_out_mask(lw_out_mask), .lw_out_data(lw_out_data),
      .lw_oe_mask(lw_oe_mask), .lw_oe_data(lw_oe_data),
      .h_out_we(h_out_we), .h_out(h_out), .h_oe_we(h_oe_we), .h_oe(h_oe),
      .h_od_we(h_od_we), .h_od(h_od),
      .etx_rdy(1'b0), .etx_busy(1'b0), .etx_tdp(1'b0), .etx_tdn(1'b0),
      .pin_in_vec(piv_r), .pin_in_reg(pir_r), .pin_out_reg(por_r),
      .pin_oe_reg(poe_r), .od_mask_reg(odm_r),
      .uio_out(uo_r), .uio_oe(uoe_r), .out_pins(op_r));

  wire [1:0] pads = op[1:0];                     // {TD-, TD+} = uo_out[1:0]
  // The phase within busy (6.12.1 EPH), RTL encoding {busy_r, eph}: used
  // only to tell P_TAIL from the bit phases, which no read distinguishes.
  wire in_tail = busy && (u_etx.eph == 2'd3);

  // ==================================================================
  // ETX-1 (pad shape), 6.12.7: "if BUSY, uo_out[0] = TDP and uo_out[1] =
  // TDN with (TDP, TDN) = (TXL, ~TXL) in P_PRE, P_DATA and P_FCS and (1, 0)
  // in P_TAIL; if not, uo_out[0] = PIN_OUT[8] and uo_out[1] = PIN_OUT[9]."
  // Plus 6.12.3: the first half bit, in cycle s, is (~v, v) with v = p_0 = 1
  // under PRE, else w[0] of the start word.
  // ==================================================================
  reg p_busy, p_dw, p_pre;
  reg [15:0] p_val;
  always @(posedge clk) begin
    p_busy <= busy; p_dw <= w_data_we | w_last_we | w_last1_we;
    p_pre <= cfg_rd[4]; p_val <= w_val;
  end
  always @(*) if (f_past_valid) begin
    if (busy)    assert (pads[0] != pads[1]);
    if (in_tail) assert (pads == 2'b01);
    if (!busy)   assert (pads == por[9:8]);
  end
  always @(posedge clk) if (f_past_valid && busy && !p_busy) begin
    assert (p_dw);                               // only a data write starts
    assert (pads[0] == ~(p_pre ? 1'b1 : p_val[0]));
  end

  // ==================================================================
  // ETX-2 (half-bit length), 6.12.6: "half bit i of the frame starts at
  // edge s + i N and lasts N clocks", and 6.12.3 "the line level is ~v in
  // the first and v in the second". f_ph is the clock within the current
  // half bit (0 in the cycle an edge s + i N begins), f_hb the half-bit
  // parity, both counted from the start (BUSY rising) with N read through
  // ETX_CFG, which is constant while busy (ETX-READ).
  //  (a) while busy across an edge, the pad pair changes only at s + i N;
  //  (b) BUSY falls only at an edge s + i N (the release ends a half bit);
  //  (c) at an edge in the middle of a bit (f_hb 1 after it) outside the
  //      tail, the pair changes: every Manchester bit has its mid transition.
  // ==================================================================
  wire [3:0] halfn = cfg_rd[3:0];
  reg  [3:0] f_ph;
  reg        f_hb;
  always @(posedge clk) begin
    if (!rst_n || (busy == 1'b0)) begin f_ph <= 4'd0; f_hb <= 1'b0; end
    else if (f_ph == halfn)       begin f_ph <= 4'd0; f_hb <= ~f_hb; end
    else                                f_ph <= f_ph + 4'd1;
  end
  // f_ph(k) as computed from cycle k-1 (the value a boundary would give).
  reg [1:0] p_pads;
  reg       p_tail, p_bnd, p_rst_n;
  always @(posedge clk) begin
    p_rst_n <= rst_n; p_pads <= pads; p_tail <= in_tail; p_bnd <= busy && (f_ph == halfn);
  end
  always @(posedge clk) if (f_past_valid && p_rst_n) begin
    if (p_busy && busy && pads != p_pads) assert (p_bnd);          // (a)
    if (p_busy && !busy)                  assert (p_bnd);          // (b)
    if (p_busy && busy && p_bnd && f_hb && !p_tail && !in_tail)
      assert (pads == ~p_pads);                                    // (c)
  end
  // Mid-bit parity: a bit never starts at an odd half bit. Needs EPH and the
  // RTL's HB (6.12.1 HB): f_hb must equal HB outside the tail.
  always @(*) if (f_past_valid && busy && !in_tail)
    assert (f_hb == u_etx.hb && f_ph == halfn - u_etx.div);

  // ==================================================================
  // ETX-5 (isolation of the rest): "PIN_OUT, PIN_OE, every thread's state
  // and the other 22 pads are unchanged by the block". Against u_ref: the
  // pin registers, the pad view, uio_out, uio_oe and uo_out[5:2] are the
  // same; pin_in reads the same except 13 and 14, which are RDY and BUSY
  // (6.12.7); uo_out[1:0] differ only while BUSY. Thread state: loom_core
  // sees the block only through these reads and ETX_CFG / ETX_STAT, and
  // SCHED-2 / SCHED-3 hold with all of them free (sched.sby).
  // ==================================================================
  always @(*) if (f_past_valid) begin
    assert (por == por_r && poe == poe_r && odm == odm_r && pir == pir_r);
    assert (uo == uo_r && uoe == uoe_r && op[5:2] == op_r[5:2]);
    assert (piv[31:15] == piv_r[31:15] && piv[12:0] == piv_r[12:0]);
    assert (piv[14] == busy && piv[13] == rdy);
    if (!busy) assert (op[1:0] == op_r[1:0]);
  end

  // ------------------------------------------------- cover (anti-vacuity)
  always @(*) if (f_past_valid && rst_n) begin
    cover (in_tail);
    cover (busy && halfn == 4'd2 && f_hb);
    cover (busy && h_out_we && h_out[8]);         // a pin write mid-frame
    cover (!busy && p_busy && por[8]);            // PIN_OUT back on the pad
  end

endmodule

`default_nettype wire
