/*
 * Loom protocol emulator, Tiny Tapeout top level.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Milestone M0 placeholder: a hard-wired UART transmitter that sends
 * "LOOM\r\n" (8N1) at BAUD on OUT0 (uo_out[0]) while IN0 (ui_in[0]) is high.
 * This proves the flow end to end, as the competition brief suggests
 * ("start by getting a UART transmitter out of a pin"). M1 replaces the body
 * with the Loom core; the port list and pin map do not change.
 *
 * Timing contract: tx changes only on baud ticks; the first start bit begins
 * on the first baud tick after IN0 is sampled high, so every bit is a full
 * bit period. Frames are back to back (start, 8 data LSB first, stop). After
 * the last byte the line idles high for at least one bit period.
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

  parameter integer CLK_HZ = 50_000_000;
  parameter integer BAUD   = 115_200;
  localparam integer DIV     = CLK_HZ / BAUD;   // 434 at 50 MHz
  localparam integer MSG_LEN = 6;

  // ---------------------------------------------------------------- message
  function [7:0] msg_byte(input [2:0] i);
    case (i)
      3'd0:    msg_byte = "L";
      3'd1:    msg_byte = "O";
      3'd2:    msg_byte = "O";
      3'd3:    msg_byte = "M";
      3'd4:    msg_byte = 8'h0D;
      default: msg_byte = 8'h0A;
    endcase
  endfunction

  // ------------------------------------------------------------- baud ticks
  reg [15:0] baud_cnt;
  reg        baud_tick;

  always @(posedge clk) begin
    if (!rst_n) begin
      baud_cnt  <= 16'd0;
      baud_tick <= 1'b0;
    end else if ({16'd0, baud_cnt} == DIV - 1) begin
      baud_cnt  <= 16'd0;
      baud_tick <= 1'b1;
    end else begin
      baud_cnt  <= baud_cnt + 16'd1;
      baud_tick <= 1'b0;
    end
  end

  // ------------------------------------------------------------ transmitter
  reg       tx;
  reg       active;
  reg [3:0] bit_idx;   // 0..7 data bits sent so far, 8 = stop sent
  reg [2:0] msg_idx;
  reg [7:0] shreg;

  always @(posedge clk) begin
    if (!rst_n) begin
      tx      <= 1'b1;
      active  <= 1'b0;
      bit_idx <= 4'd0;
      msg_idx <= 3'd0;
      shreg   <= 8'd0;
    end else if (!active) begin
      tx <= 1'b1;
      if (ui_in[0] && baud_tick) begin
        active  <= 1'b1;
        bit_idx <= 4'd0;
        msg_idx <= 3'd0;
        shreg   <= msg_byte(3'd0);
        tx      <= 1'b0;                  // start bit
      end
    end else if (baud_tick) begin
      if (bit_idx < 4'd8) begin
        tx      <= shreg[0];              // data bit, LSB first
        shreg   <= {1'b0, shreg[7:1]};
        bit_idx <= bit_idx + 4'd1;
      end else if (bit_idx == 4'd8) begin
        tx      <= 1'b1;                  // stop bit
        bit_idx <= 4'd9;
      end else begin
        // stop bit finished
        if ({29'd0, msg_idx} == MSG_LEN - 1) begin
          active <= 1'b0;                 // idle high until IN0 restarts us
          tx     <= 1'b1;
        end else begin
          msg_idx <= msg_idx + 3'd1;
          shreg   <= msg_byte(msg_idx + 3'd1);
          bit_idx <= 4'd0;
          tx      <= 1'b0;                // next start bit, back to back
        end
      end
    end
  end

  // ------------------------------------------------------------------ pins
  assign uo_out  = {7'b0000000, tx};
  assign uio_out = 8'b0;
  assign uio_oe  = 8'b0;

  // List all unused inputs to prevent warnings
  wire _unused = &{ena, ui_in[7:1], uio_in, 1'b0};

endmodule
