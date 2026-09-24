/*
 * loom_timer_props: TIMER-1 and TIMER-2 (docs/VERIFICATION.md L4) on loom_timer.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Formal only. Bound into loom_timer; never read by synthesis or the TT flow.
 *
 * TIMER-1 as VERIFICATION.md L4 states it: "`reached` is monotone: once true
 * it stays true until TD changes."
 *
 * `reached` is defined in SEMANTICS 4: "reached(a, b) is
 * ((a - b) mod 2^16) < 2^15", and the pair that matters is reached(NOW, TD),
 * the deadline test of WAITD (6.4) and of the deadline latch (6.10).
 *
 * TIMER-2 (D-028): thread t's `lat_fire` is high only in a cycle whose
 * closing edge ticks NOW, or at which thread t's own slot writes TD (the
 * commit port). A host debug write of TD, on its own, never raises it: it is
 * not a rule-2 write, and rule 1 compares against the TD value before the
 * edge (SEMANTICS 6.10).
 *
 * The commit and host ports are free: any sequence of TD, DT, TICK_INT and
 * TICK_FRAC writes is allowed, which is what "until TD changes" has to hold
 * against, and what TIMER-2's "on its own" has to hold against.
 */

`default_nettype none

module loom_timer_props (
    input wire        clk,
    input wire        rst_n,
    input wire [63:0] now_all,
    input wire [63:0] td_all,
    input wire [3:0]  lat_fire,
    input wire [3:0]  cm_sel,
    input wire        cm_td_we,
    input wire        h_we,
    input wire [3:0]  h_sel,
    input wire        h_td_we
);

  reg f_past_valid = 1'b0;
  always @(posedge clk) f_past_valid <= 1'b1;
  always @(*) if (!f_past_valid) assume (!rst_n);

  genvar t;
  generate
    for (t = 0; t < 4; t = t + 1) begin : g_thread
      wire [15:0] now = now_all[16*t +: 16];
      wire [15:0] td  = td_all[16*t +: 16];

      // SEMANTICS 4: reached(a, b) = ((a - b) mod 2^16) < 2^15.
      wire [15:0] d       = now - td;
      wire        reached = ~d[15];

      // What the previous cycle presented: its lat_fire, its commit-port TD
      // write, and (through NOW) whether its closing edge ticked.
      reg [15:0] p_now, p_td, p_d;
      reg        p_reached, p_fire, p_cm_td;
      always @(posedge clk) begin
        p_now     <= now;
        p_td      <= td;
        p_d       <= d;
        p_reached <= reached;
        p_fire    <= lat_fire[t];
        p_cm_td   <= cm_sel[t] && cm_td_we;
      end

      always @(posedge clk) if (f_past_valid && $past(rst_n)) begin
`ifdef LOOM_FV_XFAIL
        // ---- TIMER-1, exactly as VERIFICATION.md L4 words it.
        // Expected to FAIL; see formal/README.md, finding F-1.
        if (p_reached && (td == p_td)) assert (reached);
`endif

        // ---- TIMER-1B: the only way TIMER-1 can break is the half-range
        // wrap, i.e. NOW ticking from TD + 0x7FFF to TD + 0x8000. This is a
        // separate, stronger statement about where the boundary is; it is not
        // a weakened TIMER-1, which stands above as written.
        if (p_reached && (td == p_td) && !reached)
          assert (p_d == 16'h7FFF);

        // ---- TIMER-1C: NOW only ever advances by one (SEMANTICS 4: "NOW <=
        // NOW + tick"), which is what makes TIMER-1B the whole story.
        assert ((now == p_now) || (now == p_now + 16'd1));

        // ---- TIMER-2 (D-028): a fire needs a tick at the closing edge or a
        // TD write by the thread's own slot. Nothing the host does on its own
        // raises it, whatever value it writes and to whichever thread.
        if (p_fire) assert ((now != p_now) || p_cm_td);
      end
    end
  endgenerate

  // ------------------------------------------------- cover (anti-vacuity)
  wire [15:0] c_now = now_all[15:0];
  wire [15:0] c_td  = td_all[15:0];
  wire [15:0] c_d   = c_now - c_td;
  reg  [15:0] pc_now;
  reg         pc_fire, pc_h_td, pc_cm_td;
  always @(posedge clk) begin
    pc_now   <= c_now;
    pc_fire  <= lat_fire[0];
    pc_h_td  <= h_we && h_td_we && h_sel[0];
    pc_cm_td <= cm_sel[0] && cm_td_we;
  end
  always @(posedge clk) if (f_past_valid && $past(rst_n)) begin
    cover (c_now != pc_now);          // the timebase ticks
    cover (~c_d[15]);                 // reached is true
    cover (c_d[15]);                  // and false
    cover (c_d == 16'h7FFF);          // the boundary TIMER-1B names
    cover (pc_fire && (c_now != pc_now) && !pc_cm_td);   // rule 1 fired
    cover (pc_fire && pc_cm_td);                          // rule 2 fired
    cover (pc_h_td && !pc_fire && (c_now == pc_now));     // a host TD write on a quiet edge, no fire
  end

endmodule

bind loom_timer loom_timer_props u_timer_props (
    .clk(clk), .rst_n(rst_n), .now_all(now_all), .td_all(td_all),
    .lat_fire(lat_fire), .cm_sel(cm_sel), .cm_td_we(cm_td_we),
    .h_we(h_we), .h_sel(h_sel), .h_td_we(h_td_we)
);

`default_nettype wire
