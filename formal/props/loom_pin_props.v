/*
 * loom_pin_props: PIN-1 (docs/VERIFICATION.md L4) plus the pin-commit rules
 * of SEMANTICS 6.3, on loom_pins.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Formal only. Bound into loom_pins; never read by synthesis or the TT flow.
 *
 * Two groups of properties:
 *   - always on: PIN-2, the bit-masking and priority rules of SEMANTICS 6.3,
 *     6.10 and 7. These hold.
 *   - under `LOOM_FV_XFAIL`: PIN-1 as VERIFICATION.md L4 states it, plus two
 *     narrower variants that locate why it does not hold. These fail; the
 *     counterexamples are the finding, see formal/README.md.
 *
 * Environment. The open-drain rule of SEMANTICS 6.3 is applied by loom_core
 * before a write reaches this module ("i in 0..7 with OD_MASK[i] == 1:
 * PIN_OUT[i] <= 0; PIN_OE[i] <= ~b"), so a write arriving here while
 * OD_MASK[i] is set never carries a 1 on that bit. That is assumed for the
 * core port and for the staged (deadline-latched) port, which 6.10 says
 * "follows the pin-write rules of 6.3 (open drain included)". The host port
 * writes PIN_OUT, PIN_OE and OD_MASK as whole registers (SEMANTICS 7, the
 * CTRL space) and is left free.
 */

`default_nettype none

module loom_pin_props (
    input wire        clk,
    input wire        rst_n,
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
    input wire        h_out_we,
    input wire [15:0] h_out,
    input wire        h_oe_we,
    input wire [7:0]  h_oe,
    input wire        h_od_we,
    input wire [7:0]  h_od,
    input wire [13:0] pin_out,
    input wire [7:0]  pin_oe,
    input wire [7:0]  od_mask,
    input wire [7:0]  uio_out,
    input wire [7:0]  uio_oe
);

  reg f_past_valid = 1'b0;
  always @(posedge clk) f_past_valid <= 1'b1;
  always @(*) if (!f_past_valid) assume (!rst_n);

  // ---- environment: SEMANTICS 6.3 open drain, applied upstream.
  always @(*) if (f_past_valid && rst_n) begin
    assume ((cw_out_mask[7:0] & cw_out_data[7:0] & od_mask & {8{cw_valid}}) == 8'd0);
    assume ((lw_out_mask[7:0] & lw_out_data[7:0] & od_mask) == 8'd0);
  end

  // ==================================================================
  // PIN-2: pin commits are bit-masked and the writers have a fixed priority.
  //
  // SEMANTICS 6.3: "All pin commits are bit-masked: a slot changes only the
  // bits it writes."
  // SEMANTICS 6.10: "At an edge where a staged write and a slot's ordinary
  // pin writes both touch a pin, the ordinary write wins."
  // SEMANTICS 7: "If a host write and a thread commit hit the same register
  // bits at the same edge, the thread wins."
  // ==================================================================
  wire [13:0] f_cw_m = cw_out_mask & {14{cw_valid}};
  wire [7:0]  f_ce_m = cw_oe_mask  & {8{cw_valid}};

  reg [13:0] p_pin_out, p_cw_m, p_cw_d, p_lw_m, p_lw_d;
  reg [15:0] p_h_out;
  reg [7:0]  p_pin_oe, p_ce_m, p_ce_d, p_le_m, p_le_d, p_h_oe, p_od_mask, p_cw_od, p_h_od;
  reg        p_h_out_we, p_h_oe_we, p_cw_od_we, p_h_od_we, p_rst_n;
  always @(posedge clk) begin
    p_pin_out  <= pin_out;   p_pin_oe <= pin_oe;   p_od_mask <= od_mask;
    p_cw_m     <= f_cw_m;    p_cw_d   <= cw_out_data;
    p_lw_m     <= lw_out_mask; p_lw_d <= lw_out_data;
    p_ce_m     <= f_ce_m;    p_ce_d   <= cw_oe_data;
    p_le_m     <= lw_oe_mask; p_le_d  <= lw_oe_data;
    p_h_out_we <= h_out_we;  p_h_out  <= h_out;
    p_h_oe_we  <= h_oe_we;   p_h_oe   <= h_oe;
    p_cw_od_we <= cw_valid & cw_od_we; p_cw_od <= cw_od;
    p_h_od_we  <= h_od_we;   p_h_od   <= h_od;
    p_rst_n    <= rst_n;
  end

  genvar b;
  generate
    for (b = 0; b < 14; b = b + 1) begin : g_out
      always @(posedge clk) if (f_past_valid && p_rst_n) begin
        if (p_cw_m[b])      assert (pin_out[b] == p_cw_d[b]);
        else if (p_lw_m[b]) assert (pin_out[b] == p_lw_d[b]);
        else if (p_h_out_we) assert (pin_out[b] == p_h_out[b]);
        else                assert (pin_out[b] == p_pin_out[b]);
      end
    end
    for (b = 0; b < 8; b = b + 1) begin : g_oe
      always @(posedge clk) if (f_past_valid && p_rst_n) begin
        if (p_ce_m[b])      assert (pin_oe[b] == p_ce_d[b]);
        else if (p_le_m[b]) assert (pin_oe[b] == p_le_d[b]);
        else if (p_h_oe_we) assert (pin_oe[b] == p_h_oe[b]);
        else                assert (pin_oe[b] == p_pin_oe[b]);
      end
    end
  endgenerate

  always @(posedge clk) if (f_past_valid && p_rst_n) begin
    if (p_cw_od_we)     assert (od_mask == p_cw_od);
    else if (p_h_od_we) assert (od_mask == p_h_od);
    else                assert (od_mask == p_od_mask);
  end

  // The pads are the register views of ARCHITECTURE 3.1 with OD_MASK applied
  // (SEMANTICS 3, D-023): an open-drain pin pulls low or is released.
  always @(*) if (f_past_valid) begin
    assert (uio_out == (pin_out[7:0] & ~od_mask));
    assert (uio_oe  == (pin_oe & ~(od_mask & pin_out[7:0])));
  end

  // ==================================================================
  // PIN-1: "a BIDIR pin with OD_MASK set never has OE=1 with OUT=1."
  // Was false until D-023 put the gate in the pads (BUGS 5); the trace that
  // broke it is finding F-2 in formal/README.md.
  // ==================================================================
  always @(*) if (f_past_valid)
    assert ((od_mask & uio_oe & uio_out) == 8'd0);

  // PIN-1CORE: PIN-1 restricted to traces in which the host has never written
  // a pin register, so only a thread's own SETP / OEP / CSRW OD_MASK, or a
  // staged SETP ... D, could have moved them. It said, while PIN-1 was false,
  // that firmware alone could break it; now a redundant but cheap witness
  // that the gate does not depend on where a register write came from.
  reg f_host_touched;
  always @(posedge clk) begin
    if (!rst_n)                            f_host_touched <= 1'b0;
    else if (h_out_we | h_oe_we | h_od_we) f_host_touched <= 1'b1;
  end

  always @(*) if (f_past_valid && !f_host_touched)
    assert ((od_mask & uio_oe & uio_out) == 8'd0);

  // PIN-1R: the stronger pad claim. The 6.3 write rules drive PIN_OUT[i] to 0
  // under open drain, but nothing stops a 1 surviving the switch into
  // open-drain mode, so this holds at the pad and not in the register.
  always @(*) if (f_past_valid)
    assert ((od_mask & uio_out) == 8'd0);

  // ------------------------------------------------- cover (anti-vacuity)
  always @(*) if (f_past_valid && rst_n) begin
    cover (od_mask != 8'd0);                       // open drain is usable
    cover ((od_mask & uio_oe) != 8'd0);            // and an OD pin drives low
    cover ((uio_oe & uio_out) != 8'd0);            // a push-pull pin drives high
    cover (cw_valid && (cw_out_mask != 14'd0));    // a slot writes pins
    cover (lw_out_mask != 14'd0);                  // a staged write lands
    cover (h_out_we);                              // the host writes PIN_OUT
  end

endmodule

bind loom_pins loom_pin_props u_pin_props (
    .clk(clk), .rst_n(rst_n),
    .cw_valid(cw_valid), .cw_out_mask(cw_out_mask), .cw_out_data(cw_out_data),
    .cw_oe_mask(cw_oe_mask), .cw_oe_data(cw_oe_data),
    .cw_od_we(cw_od_we), .cw_od(cw_od),
    .lw_out_mask(lw_out_mask), .lw_out_data(lw_out_data),
    .lw_oe_mask(lw_oe_mask), .lw_oe_data(lw_oe_data),
    .h_out_we(h_out_we), .h_out(h_out),
    .h_oe_we(h_oe_we), .h_oe(h_oe),
    .h_od_we(h_od_we), .h_od(h_od),
    .pin_out(pin_out), .pin_oe(pin_oe), .od_mask(od_mask),
    .uio_out(uio_out), .uio_oe(uio_oe)
);

`default_nettype wire
