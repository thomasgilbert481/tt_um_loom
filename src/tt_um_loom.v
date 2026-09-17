/*
 * tt_um_loom on branch `sram-smoke`: a pin-level tester for the IHP
 * RM_IHPSG13_1P_512x16_c2_bm_bist SRAM macro.
 * SPDX-License-Identifier: Apache-2.0
 *
 * This is milestone M0.5 (docs/PLAN.md, D-015): the smallest design that can
 * put a hard macro through the Tiny Tapeout cmos5l flow and let a host write
 * and read every one of the 512 x 16 words. The module name stays tt_um_loom
 * so info.yaml, the workflows and the testbench are unchanged. The Loom core
 * is not in this design; `main` still has it.
 *
 * ---------------------------------------------------------------------------
 * PIN PROTOCOL
 * ---------------------------------------------------------------------------
 * 25 bits of state (9 address + 16 data) have to be loaded through 8 data
 * pins, so the host writes four byte registers and then fires a command. All
 * strobes are level signals, synchronised into the clock domain with a
 * two-flop synchroniser and then edge-detected, so the host can be an
 * arbitrarily slow bit-banging RP2040 and never needs to meet setup time.
 *
 *   ui_in[1:0]  REGSEL   which byte register uio_in is written to:
 *                          0  ADDR_LO   addr[7:0]
 *                          1  ADDR_HI   addr[8]  (bit 0; bits 7:1 ignored)
 *                          2  WDATA_LO  wdata[7:0]
 *                          3  WDATA_HI  wdata[15:8]
 *   ui_in[2]    REG_WR   rising edge: REGSEL's register <= uio_in
 *   ui_in[3]    RD_OE    level: 1 drives uio (uio_oe = 8'hFF), 0 releases it
 *   ui_in[4]    MEM_WR   rising edge: SRAM write {WDATA_HI,WDATA_LO} -> [ADDR]
 *   ui_in[5]    MEM_RD   rising edge: SRAM read  [ADDR] -> RDATA
 *   ui_in[6]    UIO_SEL  level, only meaningful while RD_OE = 1:
 *                          0  uio_out = RDATA[15:8]
 *                          1  uio_out = STATUS
 *   ui_in[7]    unused
 *
 *   uo_out[7:0]          RDATA[7:0], the low byte of the last word read.
 *                        Always driven, so the host can poll it without
 *                        touching the bidirectional pins.
 *   uio[7:0]             input  : data for REG_WR
 *                        output : RDATA[15:8] or STATUS, per UIO_SEL, and only
 *                                 while RD_OE = 1
 *
 *   STATUS = {6'b0, RD_VALID, BUSY}
 *     BUSY      1 for the single cycle between a MEM_RD/MEM_WR edge and the
 *               clock edge that completes it. The macro is a one-access-per-
 *               clock SRAM, so this is always exactly one cycle; it exists so
 *               a host that cannot count clocks can poll instead.
 *     RD_VALID  set once RDATA has been loaded by a completed read, cleared by
 *               reset. Distinguishes "read back zero" from "never read".
 *
 * Read timing, in clocks of `clk`. Call N the cycle in which the synchronised
 * MEM_RD rising edge is seen:
 *   N    : loom_imem_macro presents ADDR; the macro samples it on the N -> N+1
 *          edge
 *   N+1  : the macro drives A_DOUT; BUSY = 1; RDATA is loaded on the
 *          N+1 -> N+2 edge
 *   N+2  : BUSY = 0, RD_VALID = 1, RDATA visible on uo_out / uio
 * Write timing: the word is stored on the N -> N+1 edge, BUSY is 1 during N+1
 * and 0 from N+2. In both cases "BUSY has gone low again" means done.
 *
 * The host must hold each strobe high for at least two clocks and low for at
 * least two clocks (the synchroniser), and must not change REGSEL, uio_in or
 * the address/data registers while a strobe is being recognised.
 *
 * Unused-pin hygiene: ena, ui_in[7] and the unused ADDR_HI bits are collected
 * into _unused, as the Tiny Tapeout template requires.
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

  // ------------------------------------------------------ input synchronisers
  // Two flops per asynchronous strobe, then a rising-edge detector. reg_wr_q2
  // etc. are the synchronised levels; *_edge is one clock wide.
  reg [2:0] reg_wr_sync;
  reg [2:0] mem_wr_sync;
  reg [2:0] mem_rd_sync;

  always @(posedge clk) begin
    if (!rst_n) begin
      reg_wr_sync <= 3'b0;
      mem_wr_sync <= 3'b0;
      mem_rd_sync <= 3'b0;
    end else begin
      reg_wr_sync <= {reg_wr_sync[1:0], ui_in[2]};
      mem_wr_sync <= {mem_wr_sync[1:0], ui_in[4]};
      mem_rd_sync <= {mem_rd_sync[1:0], ui_in[5]};
    end
  end

  wire reg_wr_edge = reg_wr_sync[1] & ~reg_wr_sync[2];
  wire mem_wr_edge = mem_wr_sync[1] & ~mem_wr_sync[2];
  wire mem_rd_edge = mem_rd_sync[1] & ~mem_rd_sync[2];

  // --------------------------------------------------------- byte registers
  reg [8:0]  addr_reg;
  reg [15:0] wdata_reg;

  always @(posedge clk) begin
    if (!rst_n) begin
      addr_reg  <= 9'b0;
      wdata_reg <= 16'b0;
    end else if (reg_wr_edge) begin
      case (ui_in[1:0])
        2'd0: addr_reg[7:0]   <= uio_in;
        2'd1: addr_reg[8]     <= uio_in[0];
        2'd2: wdata_reg[7:0]  <= uio_in;
        default: wdata_reg[15:8] <= uio_in;
      endcase
    end
  end

  // ------------------------------------------------------------ SRAM access
  // A command edge drives the macro for exactly one clock. Because the macro
  // registers its output internally, the read result is captured one clock
  // after the access cycle.
  wire        sram_we = mem_wr_edge;
  wire [15:0] sram_q;

  reg         busy;
  reg         rd_pending;
  reg         rd_valid;
  reg  [15:0] rdata_reg;

  always @(posedge clk) begin
    if (!rst_n) begin
      busy       <= 1'b0;
      rd_pending <= 1'b0;
      rd_valid   <= 1'b0;
      rdata_reg  <= 16'b0;
    end else begin
      busy       <= mem_wr_edge | mem_rd_edge;
      rd_pending <= mem_rd_edge;
      if (rd_pending) begin
        rdata_reg <= sram_q;
        rd_valid  <= 1'b1;
      end
    end
  end

  loom_imem_macro u_imem (
      .clk   (clk),
      .rst_n (rst_n),
      .addr  (addr_reg),
      .rdata (sram_q),
      .we    (sram_we),
      .waddr (addr_reg),
      .wdata (wdata_reg)
  );

  // ------------------------------------------------------------------- pins
  wire [7:0] status = {6'b0, rd_valid, busy};

  assign uo_out  = rdata_reg[7:0];
  assign uio_out = ui_in[6] ? status : rdata_reg[15:8];
  assign uio_oe  = {8{ui_in[3]}};

  // List all unused inputs to prevent warnings
  wire _unused = &{ena, ui_in[7], 1'b0};

endmodule

`default_nettype wire
