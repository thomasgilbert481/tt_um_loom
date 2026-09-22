/*
 * loom_icebreaker: tt_um_loom on an iCEBreaker (iCE40UP5K), for the M2 bench.
 * SPDX-License-Identifier: Apache-2.0
 *
 * The chip's pins map one PMOD each, so a bench wiring reads like the
 * Tiny Tapeout pinout:
 *   PMOD1A  ui_in[7:0]   IN0..IN3 on [3:0], the host SPI port on [6:4]
 *                        (CS_n, SCK, MOSI), IN4 on [7]
 *   PMOD1B  uo_out[7:0]  OUT0..OUT5 on [5:0], HOST_IRQ on [6], MISO on [7]
 *   PMOD2   uio[7:0]     BIDIR0..BIDIR7, through SB_IO tristates
 * The instruction memory is the FLOPS backend (D-020): the IHP SRAM macro
 * exists only in silicon, and the flop array's synchronous read lets Yosys put
 * it in block RAM. Everything else is the silicon RTL, unchanged.
 *
 * Reset is a power-on counter ANDed with the user button (active low), then
 * synchronised, so the core starts from a clean reset after configuration
 * and whenever the button is pressed.
 */

`default_nettype none

module loom_icebreaker (
    input  wire       CLK,      // 12 MHz oscillator
    input  wire       BTN_N,    // user button, active low: reset
    output wire       LEDG_N,   // green LED, active low: heartbeat
    output wire       LEDR_N,   // red LED, active low: HOST_IRQ
    input  wire [7:0] P1A,      // ui_in
    output wire [7:0] P1B,      // uo_out
    inout  wire [7:0] P2        // uio
);

  // ------------------------------------------------------------ reset
  reg [3:0] por = 4'd0;                      // FPGA configuration sets it to 0
  always @(posedge CLK) if (!(&por)) por <= por + 4'd1;

  reg [1:0] rst_sync = 2'b00;
  always @(posedge CLK) rst_sync <= {rst_sync[0], (&por) & BTN_N};
  wire rst_n = rst_sync[1];

  // ------------------------------------------------------- bidir pins
  wire [7:0] uio_in, uio_out, uio_oe;
  genvar i;
  generate
    for (i = 0; i < 8; i = i + 1) begin : g_uio
      // PIN_TYPE 1010_01: output driven through OUTPUT_ENABLE, input
      // unregistered. The weak pull-up gives an open-drain line a defined
      // high on the bench; a real I2C bus still wants external pull-ups.
      SB_IO #(.PIN_TYPE(6'b1010_01), .PULLUP(1'b1)) u_io (
          .PACKAGE_PIN  (P2[i]),
          .OUTPUT_ENABLE(uio_oe[i]),
          .D_OUT_0      (uio_out[i]),
          .D_IN_0       (uio_in[i])
      );
    end
  endgenerate

  // ------------------------------------------------------------- core
  wire [7:0] uo_out;
  tt_um_loom #(.IMEM_IMPL("FLOPS"), .IMEM_WORDS(16'd512)) u_loom (
      .ui_in  (P1A),
      .uo_out (uo_out),
      .uio_in (uio_in),
      .uio_out(uio_out),
      .uio_oe (uio_oe),
      .ena    (1'b1),
      .clk    (CLK),
      .rst_n  (rst_n)
  );
  assign P1B = uo_out;

  // ------------------------------------------------------------- LEDs
  reg [23:0] beat = 24'd0;
  always @(posedge CLK) beat <= beat + 24'd1;
  assign LEDG_N = ~beat[23];                 // about 0.7 Hz at 12 MHz
  assign LEDR_N = ~uo_out[6];                // HOST_IRQ

endmodule

`default_nettype wire
