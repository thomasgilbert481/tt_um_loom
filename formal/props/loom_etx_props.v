/*
 * loom_etx_props: ETX-3, ETX-4 and the CSR read laws of SEMANTICS 6.12.2, on
 * loom_etx (docs/SEMANTICS.md 6.12, D-036; proposed IDs of 6.12.10).
 * SPDX-License-Identifier: Apache-2.0
 *
 * Formal only. Bound into loom_etx; never read by synthesis or the TT flow.
 * The pad-level properties (ETX-1, ETX-2, ETX-5) need loom_pins beside the
 * block and live in formal/etx_top.v. The write port is free apart from the
 * one sentence of 6.12.2 the caller owes, assumed in etx_top.v and
 * discharged on loom_core by SCHED-ETX (props/loom_sched_props.v).
 *
 * Observable here (6.12.1): ETX_CFG = cfg_rd, ETX_STAT = stat_rd,
 * pin_in(13) = rdy, pin_in(14) = busy. Where a property needs a register
 * that no read exposes, the comment above it names the register and why.
 */

`default_nettype none

module loom_etx_props (
    input wire        clk,
    input wire        rst_n,
    input wire        w_cfg_we,
    input wire        w_data_we,
    input wire        w_last_we,
    input wire        w_last1_we,
    input wire        w_stat_we,
    input wire [15:0] w_val,
    input wire [9:0]  cfg_rd,
    input wire [4:0]  stat_rd,
    input wire        busy,
    input wire        rdy,
    // internal, named in the comments where used
    input wire        start,
    input wire        accept,
    input wire        busy_r,
    input wire [1:0]  eph,
    input wire [5:0]  pos,
    input wire [3:0]  div,
    input wire        hb,
    input wire [3:0]  idl,
    input wire        txb,
    input wire [14:0] sh,
    input wire        sl,
    input wire        s1,
    input wire [15:0] hold,
    input wire        hv,
    input wire        hl,
    input wire        h1,
    input wire        lseen
);

  reg f_past_valid = 1'b0;
  always @(posedge clk) f_past_valid <= 1'b1;
  always @(*) if (!f_past_valid) assume (!rst_n);

  wire f_dw   = w_data_we | w_last_we | w_last1_we;   // a data write (6.12.4)
  wire f_last = w_last_we | w_last1_we;
  wire f_one  = w_last1_we;
  wire [4:0] f_n = {1'b0, cfg_rd[3:0]} + 5'd1;        // N = HALFN + 1
  wire f_ref = stat_rd[4], f_under = stat_rd[3], f_fdone = stat_rd[2];

  // What the cycle before the current one presented (edge e ends it).
  reg        p_rst_n, p_dw, p_last, p_one, p_cfg_we, p_stat_we, p_busy;
  reg        p_hv, p_fdone;
  reg [15:0] p_val, p_hold;
  reg [9:0]  p_cfg;
  always @(posedge clk) begin
    p_rst_n <= rst_n;  p_dw <= f_dw;  p_last <= f_last;  p_one <= f_one;
    p_cfg_we <= w_cfg_we;  p_stat_we <= w_stat_we;  p_busy <= busy;
    p_hv <= hv;  p_hold <= hold;  p_val <= w_val;  p_cfg <= cfg_rd;
    p_fdone <= f_fdone;
  end
  wire f_edge = f_past_valid && p_rst_n;              // edge e was not a reset

  // ==================================================================
  // 6.12.1: "In every reachable state, POS < IDL holds in P_TAIL". Stated
  // over the RTL's encoding (EPH is {busy_r, eph}, eph 3 = P_TAIL) because
  // 6.12.10 offers it to the induction; it is also a claim of the text.
  // ==================================================================
  always @(*) if (f_past_valid && busy_r && eph == 2'd3)
    assert ({2'b00, idl} > pos);
  // Two more facts of the text, kept as induction helpers (6.12.5 "enter
  // the tail: HV <= 0", "release: HV <= 0, LSEEN <= 0"): nothing is held
  // while idle or in the tail, and no last word is pending while idle.
  always @(*) if (f_past_valid) begin
    if (!busy_r)               assert (!hv && !lseen);
    if (busy_r && eph == 2'd3) assert (!hv);
  end

  // ==================================================================
  // ETX-READ, 6.12.2. "ETX_CFG reads 0x01B1 after reset ... and ETX_STAT
  // reads 0x0002 (RDY)." ETX_STAT bit 0 is BUSY and bit 1 RDY, the same
  // values as pin_in(14) and pin_in(13). ETX_CFG "stores bits 9:0 if
  // EPH == P_IDLE; else ignored and REF <= 1", and nothing else writes it.
  // FDONE is set by the release (BUSY falling, the only way out of busy
  // after reset, 6.12.5) and cleared by a 1 in bit 2 of an ETX_STAT write,
  // the set winning (6.12.4).
  // ==================================================================
  always @(*) if (f_past_valid) begin
    assert (stat_rd[0] == busy);
    assert (stat_rd[1] == rdy);
  end
  always @(posedge clk) if (f_past_valid && !p_rst_n) begin
    assert (cfg_rd == 10'h1B1);
    assert (stat_rd == 5'b00010);
  end
  always @(posedge clk) if (f_edge) begin
    if (p_cfg_we && !p_busy) assert (cfg_rd == p_val[9:0]);
    else                     assert (cfg_rd == p_cfg);
    if (p_cfg_we && p_busy)  assert (f_ref);
    assert (f_fdone == ((p_busy && !busy)
                        || (p_fdone && !(p_stat_we && p_val[2]))));
  end

`ifndef ETX_TERM
  // ==================================================================
  // ETX-4 (no silent loss): "every data write is accepted or sets REF at
  // its commit edge." Accepted (6.12.4) means the word was stored where the
  // text puts it: in HOLD with HV, HL, H1 (a start with PRE, or the accept
  // rule), or in SH with SL, S1 and its bit 0 begun as TXB (a start without
  // PRE). Needs HOLD, HV, HL, H1, SH, SL, S1, TXB: no read exposes the
  // words (the RTL keeps SH[15:1] in `sh` and SH[0] in `txb`).
  // ==================================================================
  always @(posedge clk) if (f_edge && p_dw) begin
    assert (f_ref
            || (hv && hold == p_val && hl == p_last && h1 == p_one)
            || (busy_r && eph == 2'd1 && sh == p_val[15:1]
                && txb == p_val[0] && sl == p_last && s1 == p_one));
  end
  // ETX-4B, the word already in HOLD is not lost either: HOLD is given up
  // (HV falls, or HOLD takes another word) only at a load, which puts it in
  // SH and begins its bit 0 (6.12.5 rule 3; "SH gets the old HOLD and HOLD
  // the new word"). Same internal registers as ETX-4.
  always @(posedge clk) if (f_edge && p_hv && (!hv || hold != p_hold)) begin
    assert (busy_r && eph == 2'd1);
    assert (sh == p_hold[15:1] && txb == p_hold[0]);
  end
`endif

  // ==================================================================
  // ETX-3 (termination): "from any reachable state, with no data write
  // accepted, BUSY falls within 2 N (64 + 16 + 32) + N IDL clocks."
  // f_cnt counts the busy cycles since the last start or accepted write
  // (cycle s has 0), so BUSY must have fallen before f_cnt reaches the
  // bound. "Accepted" is the RTL's own `start` / `accept`, the one place
  // the decision is made (6.12.4); a write it refuses does not restart the
  // count, which is the stronger reading. N and IDL are read through
  // ETX_CFG, constant while busy (ETX-READ).
  // ==================================================================
  // N * x as five shift-and-adds (N <= 16): a 13 x 13 multiplier made
  // the SMT engines crawl.
  function [12:0] f_mul;
    input [4:0]  n;
    input [12:0] x;
    begin
      f_mul = (n[0] ? x : 13'd0) + (n[1] ? {x[11:0], 1'b0} : 13'd0)
            + (n[2] ? {x[10:0], 2'b0} : 13'd0) + (n[3] ? {x[9:0], 3'b0} : 13'd0)
            + (n[4] ? {x[8:0], 4'b0} : 13'd0);
    end
  endfunction
  wire [12:0] f_bound = f_mul(f_n, 13'd224 + {9'd0, cfg_rd[9:6]});
  reg  [12:0] f_cnt;
  always @(posedge clk) begin
    if (!rst_n || !busy || start || accept) f_cnt <= 13'd0;
    else if (f_cnt != 13'h1FFF)             f_cnt <= f_cnt + 13'd1;
  end
`ifdef ETX_TERM
  // The arithmetic N * x is what the solvers cannot close for N free, so
  // `term` is run once per N: with ETX_HALFN = h defined, every assertion
  // of this block is checked on the frames sent with HALFN = h (HALFN is
  // constant while busy, ETX-READ). Nothing is assumed about the other
  // frames, so the sixteen runs h = 0 .. 15 together cover every frame.
`ifdef ETX_HALFN
  wire f_case = (cfg_rd[3:0] == 4'd`ETX_HALFN);
`else
  wire f_case = 1'b1;
`endif
  // ETX-3 is asserted only in the `term` task, with its strengthening:
  // plain PDR leaves it open after 15 minutes (formal/README.md).
  // f_cnt is updated from `busy` before the edge; at the start edge the
  // reset branch fires through `start`.
  always @(*) if (f_past_valid && busy && f_case)
    assert (f_cnt < f_bound);

  // The `term` task proves this file on loom_etx alone (no etx_top.v), so
  // it states the caller's duty here as etx_top.v does: 6.12.2, "at most
  // one ETX write commits at any edge" (discharged by SCHED-ETX).
  wire [2:0] f_nw = w_cfg_we + w_data_we + w_last_we + w_last1_we + w_stat_we;
  always @(*) assume (f_nw <= 3'd1);
  // ETX-3 is a bounded-response property over a 13-bit count, which plain
  // PDR closes slowly (etx:prove, formal/README.md). The `term` task adds
  // the remaining time as a strengthening: f_rem is the number of busy
  // cycles left before the release edge if no further write is accepted,
  // computed from the 6.12.1 state by 6.12.5 and 6.12.6 (needs EPH, DIV,
  // HB, POS, SL, S1, HV, HL, H1: none is exposed by a read). Then
  //   T1 every busy cycle without a start or accept removes exactly one,
  //      and the release comes when one is left;
  //   T2 f_cnt + f_rem <= bound, which with f_rem >= 1 is ETX-3;
  //   T3 the shape facts f_rem needs: POS within its field, HOLD full in
  //      P_PRE, and nothing held behind a last word.
  wire [6:0]  f_fcs = cfg_rd[5] ? 7'd32 : 7'd0;
  wire [6:0]  f_wbs = s1 ? 7'd8 : 7'd16;            // bits of the word in SH
  wire [6:0]  f_wbh = h1 ? 7'd8 : 7'd16;            // bits of the word in HOLD
  wire [6:0]  f_tail_h = hv ? (f_wbh + (hl ? f_fcs : 7'd0)) : 7'd0;
  reg  [7:0]  f_bits;                               // whole bits after this one
  always @(*) begin
    case (eph)
      2'd0:    f_bits = {2'b0, 6'd63 - pos} + {1'b0, f_wbh} + (hl ? {1'b0, f_fcs} : 8'd0);
      2'd1:    f_bits = {1'b0, f_wbs - 7'd1 - {1'b0, pos}}
                        + (sl ? {1'b0, f_fcs} : {1'b0, f_tail_h});
      default: f_bits = {2'b0, 6'd31 - pos};
    endcase
  end
  wire [12:0] f_cur = {9'd0, div} + 13'd1;
  wire [12:0] f_rem = (eph == 2'd3)
      ? f_cur + f_mul(f_n, {9'd0, idl} - 13'd1 - {7'd0, pos})
      : f_cur + f_mul(f_n, {12'd0, ~hb} + {4'd0, f_bits, 1'b0} + {9'd0, idl});
  reg [12:0] p_rem;
  reg        p_go;
  always @(posedge clk) begin p_rem <= f_rem; p_go <= start | accept; end
  always @(posedge clk) if (f_edge && p_busy && f_case) begin
    if (busy && !p_go) assert (f_rem == p_rem - 13'd1);          // T1
    if (!busy)         assert (p_rem == 13'd1);
  end
  // The `tcover` task: a frame of the case under test is reachable.
  always @(*) if (f_past_valid && rst_n) cover (busy && f_case && f_rem < 13'd300);
  always @(*) if (f_past_valid && busy && f_case) begin
    assert (div <= cfg_rd[3:0]);                                 // T3
    if (eph == 2'd0) assert (hv && lseen == hl);
    if (eph == 2'd1) assert ({1'b0, pos} < f_wbs);
    if (eph == 2'd1 && sl) assert (!hv);
    if (eph == 2'd1) assert (lseen == (sl || (hv && hl)));
    if (eph == 2'd2) assert (pos <= 6'd31 && !hv);
    assert (f_rem <= f_bound && f_cnt + f_rem <= f_bound);       // T2
  end
`endif

`ifndef ETX_TERM
  // ------------------------------------------------- cover (anti-vacuity)
  always @(*) if (f_past_valid && rst_n) begin
    cover (start);
    cover (accept && hv);                               // load-edge accept
    cover (busy && f_dw && !accept && !f_ref);          // a refusal sets REF
    cover (busy_r && eph == 2'd3);                      // the tail
    cover (p_busy && !busy && cfg_rd[5:4] == 2'b00);    // a release
    cover (p_busy && !busy && f_under);                 // an underrun end
  end
`endif

endmodule

bind loom_etx loom_etx_props u_etx_props (
    .clk(clk), .rst_n(rst_n),
    .w_cfg_we(w_cfg_we), .w_data_we(w_data_we), .w_last_we(w_last_we),
    .w_last1_we(w_last1_we), .w_stat_we(w_stat_we), .w_val(w_val),
    .cfg_rd(cfg_rd), .stat_rd(stat_rd), .busy(busy), .rdy(rdy),
    .start(start), .accept(accept), .busy_r(busy_r), .eph(eph), .pos(pos), .div(div), .hb(hb),
    .idl(idl), .txb(txb), .sh(sh), .sl(sl), .s1(s1), .hold(hold), .hv(hv),
    .hl(hl), .h1(h1), .lseen(lseen)
);

`default_nettype wire
