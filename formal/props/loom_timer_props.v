/*
 * loom_timer_props: TIMER-1 (docs/VERIFICATION.md L4) on loom_timer.
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
 * The commit and host ports are free: any sequence of TD, DT, TICK_INT and
 * TICK_FRAC writes is allowed, which is what "until TD changes" has to hold
 * against.
 */

`default_nettype none

module loom_timer_props (
    input wire        clk,
    input wire        rst_n,
    input wire [63:0] now_all,
    input wire [63:0] td_all
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

      reg [15:0] p_now, p_td, p_d;
      reg        p_reached;
      always @(posedge clk) begin
        p_now     <= now;
        p_td      <= td;
        p_d       <= d;
        p_reached <= reached;
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
      end
    end
  endgenerate

  // ------------------------------------------------- cover (anti-vacuity)
  wire [15:0] c_now = now_all[15:0];
  wire [15:0] c_td  = td_all[15:0];
  wire [15:0] c_d   = c_now - c_td;
  reg  [15:0] pc_now;
  always @(posedge clk) pc_now <= c_now;
  always @(posedge clk) if (f_past_valid && $past(rst_n)) begin
    cover (c_now != pc_now);          // the timebase ticks
    cover (~c_d[15]);                 // reached is true
    cover (c_d[15]);                  // and false
    cover (c_d == 16'h7FFF);          // the boundary TIMER-1B names
  end

endmodule

bind loom_timer loom_timer_props u_timer_props (
    .clk(clk), .rst_n(rst_n), .now_all(now_all), .td_all(td_all)
);

`default_nettype wire
