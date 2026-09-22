/*
 * loom_timer: the four per-thread timebases (docs/SEMANTICS.md section 4).
 * SPDX-License-Identifier: Apache-2.0
 *
 * Owns, per thread: ACC (24), NOW (16), TD (16), DT (16), TICK_SEEN (1) and
 * the two CSRs that set the tick period, TICK_INT (16) and TICK_FRAC (8).
 *
 * Timing contract:
 *   Every output is a register value, so it is the value "during cycle x" that
 *   SEMANTICS section 3 gives to an instruction in its X stage.
 *   At every rising edge, for every thread, whether or not the thread runs:
 *       period = max(TICK_INT,1) * 256 + TICK_FRAC          (1/256 clock units)
 *       if ACC + 256 >= period: ACC <= ACC + 256 - period, tick = 1
 *       else                    ACC <= ACC + 256,          tick = 0
 *       NOW <= NOW + tick ; if tick then TICK_SEEN <= 1
 *   A TICK_INT or TICK_FRAC write committing at the same edge clears ACC and
 *   suppresses the tick at that edge (the clear wins over the accumulate).
 *   At the commit edge of every valid slot of the thread, TICK_SEEN keeps only
 *   what the slot did not see: TICK_SEEN <= tick | (TICK_SEEN & ~seen), where
 *   `seen` (cm_tseen) is the value the slot read in its X cycle, so a tick at
 *   edge x + 1 is not lost (SEMANTICS 4, rtl-m2 question 4). A tick at the same
 *   edge sets it (set wins).
 *   The commit port selects its thread with `cm_sel`, a one-hot vector that
 *   loom_core builds from its own W-stage ring and w_valid (D-019), so no
 *   thread number is decoded here.
 *   Writes arriving from the core (W stage) win over host writes on the same
 *   register in the same cycle (SEMANTICS 7).
 *   Deadline latch (SEMANTICS 6.10): `lat_fire[t]` is high during the cycle
 *   before an edge e at which thread t's staged pin write must be applied,
 *   if one is staged (loom_core owns the latch and qualifies it):
 *     rule 1: NOW ticks at e to exactly TD, and the thread's own slot does
 *             not write TD at e;
 *     rule 2: the thread's own slot writes TD at e (commit port) and
 *             reached(NOW', TD') holds for the values after e.
 *   A host debug write of TD is neither (D-028): it is not a rule-2 write,
 *   and at its own edge rule 1 compares against the TD value before the
 *   edge, so the host thread compare and h_wdata never enter this cone
 *   (they were the slow-corner path, docs/AREA.md). From the next edge on
 *   rule 1 sees the written value.
 *   Both use one subtraction, D = NOW - TD', where TD' is the value TD takes
 *   at e: rule 1 is NOW + 1 == TD, i.e. D == 16'hFFFF with TD unchanged, and
 *   NOW' - TD' = D + tick, whose bit 15 is D[15] ^ (tick & D[14:0] == 7FFF).
 *   CTRL.RESET also writes TD (TD := NOW) but is not a rule-2 write: it
 *   discards the staged write instead (loom_core).
 *
 * All per-thread outputs are flattened: thread t occupies bits
 * [16*t +: 16] of the 16-bit vectors and [8*t +: 8] of the 8-bit ones.
 */

`default_nettype none

module loom_timer (
    input  wire        clk,
    input  wire        rst_n,

    // Commit port, driven from the W stage of loom_core (one thread per edge).
    input  wire [3:0]  cm_sel,        // bit t: a valid slot of thread t commits now
    input  wire        cm_td_we,
    input  wire        cm_tseen,      // TICK_SEEN as the committing slot read it in X
    input  wire [15:0] cm_td,
    input  wire        cm_dt_we,
    input  wire [15:0] cm_dt,
    input  wire        cm_tint_we,
    input  wire [15:0] cm_tint,
    input  wire        cm_tfrac_we,
    input  wire [7:0]  cm_tfrac,

    // Host port: debug writes and CTRL.RESET (TD <= NOW) while halted.
    input  wire [3:0]  h_reset,       // bit t: reset thread t, TD := NOW
    input  wire        h_we,
    input  wire [1:0]  h_thread,
    input  wire        h_td_we,
    input  wire        h_dt_we,
    input  wire        h_tint_we,
    input  wire        h_tfrac_we,
    input  wire        h_tseen_we,    // debug write of TICK_SEEN (0x24)
    input  wire [15:0] h_wdata,

    output wire [3:0]  lat_fire,      // 6.10 rule 1 or 2 holds at the coming edge
    output wire [63:0] now_all,
    output wire [63:0] td_all,
    output wire [63:0] dt_all,
    output wire [63:0] tick_int_all,
    output wire [31:0] tick_frac_all,
    output wire [3:0]  tick_seen_all
);

  genvar t;
  generate
    for (t = 0; t < 4; t = t + 1) begin : g_thread
      reg [23:0] acc;
      reg [15:0] now;
      reg [15:0] td;
      reg [15:0] dt;
      reg        tick_seen;
      reg [15:0] tick_int;
      reg [7:0]  tick_frac;

      wire        mine     = cm_sel[t];
      wire        h_mine   = h_we && (h_thread == t[1:0]);

      wire [15:0] tint_eff = (tick_int == 16'd0) ? 16'd1 : tick_int;
      wire [23:0] period   = {tint_eff, tick_frac};
      wire [24:0] acc_next = {1'b0, acc} + 25'd256;
      wire        over     = (acc_next >= {1'b0, period});

      // A committed TICK_INT / TICK_FRAC write clears ACC and eats the tick.
      wire        acc_clr  = (mine && (cm_tint_we || cm_tfrac_we))
                             || (h_mine && (h_tint_we || h_tfrac_we));
      wire        tick     = over && !acc_clr;

      // ------------------------------------------ deadline latch (6.10)
      // TD written at this edge in the sense of rule 2: a commit (WAITD
      // first issue, SETD, CSRW TD). A host debug write is not one (D-028),
      // and CTRL.RESET is left out on purpose (it discards the staged write).
      wire        td_w     = mine && cm_td_we;
      wire [15:0] td_new   = td_w ? cm_td : td;
      wire [15:0] lat_d    = now - td_new;
      wire        lat_all1 = &lat_d[14:0];
      wire        rule1    = tick && !td_w && !h_reset[t] && lat_d[15] && lat_all1;
      wire        rule2    = td_w && !(lat_d[15] ^ (tick && lat_all1));
      assign lat_fire[t] = rule1 || rule2;

      always @(posedge clk) begin
        if (!rst_n) begin
          acc       <= 24'd0;
          now       <= 16'd0;
          td        <= 16'd0;
          dt        <= 16'd0;
          tick_seen <= 1'b0;
          tick_int  <= 16'd1;
          tick_frac <= 8'd0;
        end else begin
          // ------------------------------------------------ tick generator
          if (acc_clr)      acc <= 24'd0;
          else if (over)    acc <= acc_next[23:0] - period;
          else              acc <= acc_next[23:0];

          if (tick) now <= now + 16'd1;

          if (tick)                 tick_seen <= 1'b1;   // set wins over clear
          else if (mine)            tick_seen <= tick_seen & ~cm_tseen;   // clear only what the slot saw
          else if (h_mine && h_tseen_we) tick_seen <= h_wdata[0];

          // ------------------------------------------------------- TD / DT
          if (mine && cm_td_we)     td <= cm_td;
          else if (h_reset[t])      td <= now;
          else if (h_mine && h_td_we) td <= h_wdata;

          if (mine && cm_dt_we)     dt <= cm_dt;
          else if (h_mine && h_dt_we) dt <= h_wdata;

          // --------------------------------------------- period CSR writes
          if (mine && cm_tint_we)        tick_int <= cm_tint;
          else if (h_mine && h_tint_we)  tick_int <= h_wdata;

          if (mine && cm_tfrac_we)       tick_frac <= cm_tfrac;
          else if (h_mine && h_tfrac_we) tick_frac <= h_wdata[7:0];
        end
      end

      assign now_all[16*t+:16]       = now;
      assign td_all[16*t+:16]        = td;
      assign dt_all[16*t+:16]        = dt;
      assign tick_int_all[16*t+:16]  = tick_int;
      assign tick_frac_all[8*t+:8]   = tick_frac;
      assign tick_seen_all[t]        = tick_seen;
    end
  endgenerate

endmodule
