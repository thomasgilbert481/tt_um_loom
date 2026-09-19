/*
 * Loom protocol emulator, Tiny Tapeout top level.
 * SPDX-License-Identifier: Apache-2.0
 *
 * This module is pad mapping only (docs/ARCHITECTURE.md 3.1); everything else
 * lives in loom_top.
 *
 *   ui_in[4]   HOST_CS_n      ui_in[3:0] IN0..IN3  (pin index 8..11)
 *   ui_in[5]   HOST_SCK       ui_in[7]   IN4       (pin index 12)
 *   ui_in[6]   HOST_MOSI      uo_out[5:0] OUT0..5  (pin index 16..21)
 *   uo_out[7]  HOST_MISO      uio[7:0]   BIDIR0..7 (pin index 0..7)
 *   uo_out[6]  HOST_IRQ
 *
 * Timing contract: none of its own. Every output of loom_top is a register,
 * so all pads change on a clock edge. `ena` is ignored, as the template
 * requires, and every unused input appears in the `_unused` wire.
 */

`default_nettype none

module tt_um_loom (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

  // Instruction memory (D-020): the 512 x 16 SRAM macro. The flip-flop
  // fallback is IMEM_IMPL "FLOPS" with any power-of-two IMEM_WORDS; the
  // macro needs 512. See loom_imem.v.
  parameter         IMEM_IMPL  = "MACRO";
  parameter integer IMEM_WORDS = 512;

  // Retire record: simulation and debug only, reached hierarchically by the
  // testbench. Nothing above this level consumes it.
  wire        tr_valid, tr_done, tr_we;
  wire [1:0]  tr_thread;
  wire [9:0]  tr_pc, tr_next_pc;
  wire [15:0] tr_ir, tr_val;
  wire [2:0]  tr_rd, tr_flags;

  loom_top #(.IMEM_IMPL(IMEM_IMPL), .IMEM_WORDS(IMEM_WORDS)) u_loom (
      .ui_in(ui_in), .uo_out(uo_out),
      .uio_in(uio_in), .uio_out(uio_out), .uio_oe(uio_oe),
      .clk(clk), .rst_n(rst_n),
      .tr_valid(tr_valid), .tr_thread(tr_thread), .tr_pc(tr_pc),
      .tr_ir(tr_ir), .tr_done(tr_done), .tr_we(tr_we), .tr_rd(tr_rd),
      .tr_val(tr_val), .tr_flags(tr_flags), .tr_next_pc(tr_next_pc)
  );

  // List all unused inputs to prevent warnings
  wire _unused = &{ena, tr_valid, tr_thread, tr_pc, tr_ir, tr_done, tr_we,
                   tr_rd, tr_val, tr_flags, tr_next_pc, 1'b0};

endmodule
