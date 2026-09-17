/*
 * loom_pins: the pin index space, input synchronisers and the three global
 * pin registers (PIN_OUT, PIN_OE, OD_MASK).
 * SPDX-License-Identifier: Apache-2.0
 *
 * Index space (docs/ARCHITECTURE.md 3.1, isa/isa.yaml `pins`):
 *    0..7   BIDIR0..7  uio, readable, writable, per-pin OE and open drain
 *    8..11  IN0..3     ui_in[3:0], read only
 *    12     IN4        ui_in[7],   read only
 *    13..15 reserved, read 0
 *    16..21 OUT0..5    uo_out[5:0], write; reads return the driven value
 *    22..31 reserved, read 0
 * Register views: PIN_OUT[7:0] = uio_out, PIN_OUT[13:8] = uo_out[5:0],
 * PIN_OE[7:0] = uio_oe, PIN_IN = {3'b0, IN4, IN3..IN0, BIDIR7..0}.
 *
 * Timing contract:
 *   `pin_in_vec` is combinational from the synchroniser outputs and from
 *   PIN_OUT, so during cycle x it holds exactly what SEMANTICS section 3 says
 *   an instruction in its X stage sees: pads as sampled at edge x-1 and, for
 *   indices 16..21, the PIN_OUT bit visible in cycle x.
 *   The core write port is driven from the W stage; a pin write in a slot
 *   with X cycle x therefore lands on the pad register at edge x+2.
 *   Writes are bit-masked: only the bits whose mask bit is set change.
 *   On a bit written by both the core and the host at the same edge the core
 *   (thread) wins (SEMANTICS 7). Open drain is applied by loom_core before
 *   the write reaches this module.
 */

`default_nettype none

module loom_pins (
    input  wire        clk,
    input  wire        rst_n,

    // Raw pads: {IN4, IN3..IN0, BIDIR7..BIDIR0}
    input  wire [12:0] pad_in,

    // Core (W stage) write port.
    input  wire        cw_valid,
    input  wire [13:0] cw_out_mask,
    input  wire [13:0] cw_out_data,
    input  wire [7:0]  cw_oe_mask,
    input  wire [7:0]  cw_oe_data,
    input  wire        cw_od_we,
    input  wire [7:0]  cw_od,

    // Host write port (CTRL space).
    input  wire        h_out_we,
    input  wire [15:0] h_out,
    input  wire        h_oe_we,
    input  wire [7:0]  h_oe,
    input  wire        h_od_we,
    input  wire [7:0]  h_od,

    // Views
    output wire [31:0] pin_in_vec,
    output wire [15:0] pin_in_reg,
    output wire [15:0] pin_out_reg,
    output wire [7:0]  pin_oe_reg,
    output wire [7:0]  od_mask_reg,

    // Pads out
    output wire [7:0]  uio_out,
    output wire [7:0]  uio_oe,
    output wire [5:0]  out_pins
);

  wire [12:0] pad_sync;
  loom_sync #(.WIDTH(13), .RESET_VAL(1'b0)) u_sync (
      .clk(clk), .rst_n(rst_n), .d(pad_in), .q(pad_sync));

  reg [13:0] pin_out;
  reg [7:0]  pin_oe;
  reg [7:0]  od_mask;

  wire [13:0] h_out_mask = {14{h_out_we}};
  wire [13:0] core_out_m = cw_out_mask & {14{cw_valid}};
  wire [7:0]  core_oe_m  = cw_oe_mask  & {8{cw_valid}};

  integer b;
  always @(posedge clk) begin
    if (!rst_n) begin
      pin_out <= 14'd0;
      pin_oe  <= 8'd0;
      od_mask <= 8'd0;
    end else begin
      for (b = 0; b < 14; b = b + 1) begin
        if (core_out_m[b])     pin_out[b] <= cw_out_data[b];
        else if (h_out_mask[b]) pin_out[b] <= h_out[b];
      end
      for (b = 0; b < 8; b = b + 1) begin
        if (core_oe_m[b])      pin_oe[b] <= cw_oe_data[b];
        else if (h_oe_we)      pin_oe[b] <= h_oe[b];
      end
      if (cw_valid && cw_od_we) od_mask <= cw_od;
      else if (h_od_we)         od_mask <= h_od;
    end
  end

  assign pin_in_vec  = {10'd0, pin_out[13:8], 3'd0, pad_sync};
  assign pin_in_reg  = {3'd0, pad_sync};
  assign pin_out_reg = {2'd0, pin_out};
  assign pin_oe_reg  = pin_oe;
  assign od_mask_reg = od_mask;

  assign uio_out  = pin_out[7:0];
  assign uio_oe   = pin_oe;
  assign out_pins = pin_out[13:8];

endmodule
