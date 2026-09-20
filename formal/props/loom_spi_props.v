/*
 * loom_spi_props: SPI-1 (docs/VERIFICATION.md L4) on loom_spi_host.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Formal only. Bound into loom_spi_host; never read by synthesis or the TT
 * flow.
 *
 * SPI-1: "with SCK period >= 8 clocks, every MOSI byte is delivered exactly
 * once; CS high resets the byte counter."
 *
 * The reference model below counts SCK rising edges and shifts MOSI at the
 * *pads*, which is what "a MOSI byte" means. docs/HOST_PROTOCOL.md
 * "Electrical" gives the protocol: CPOL=0, CPHA=0, MSB first, one transaction
 * per CS_n low period, SCK period at least 8 core clocks, CS_n low at least
 * 4 clocks before the first SCK edge. Those are the assumptions.
 *
 * Nothing here reads the RTL's bit counter or shift register; the properties
 * are stated on `byte_done` and `rx_byte`, which are the byte layer's whole
 * output.
 */

`default_nettype none

module loom_spi_props (
    input wire       clk,
    input wire       rst_n,
    input wire       cs_n_pad,
    input wire       sck_pad,
    input wire       mosi_pad,
    input wire       byte_done,
    input wire [7:0] rx_byte
);

  reg f_past_valid = 1'b0;
  always @(posedge clk) f_past_valid <= 1'b1;
  always @(*) if (!f_past_valid) assume (!rst_n);

  // ---------------------------------------------- pad-level edge detection
  reg cs_q  = 1'b1;
  reg sck_q = 1'b0;
  always @(posedge clk) begin
    cs_q  <= cs_n_pad;
    sck_q <= sck_pad;
  end
  wire pad_rise   = sck_pad & ~sck_q;
  wire cs_release = cs_n_pad & ~cs_q;

  // Saturating counters used only by the assumptions. They are deliberately
  // free-running: tying them to rst_n would let a reset pulse relax the SCK
  // timing assumption, which is a property of the board, not of the chip.
  reg [2:0] sck_hold   = 3'd7;   // clocks since SCK last changed
  reg [2:0] cs_low     = 3'd0;   // clocks since CS_n went low
  reg [2:0] since_rise = 3'd7;   // clocks since the last SCK rising edge
  always @(posedge clk) begin
    sck_hold   <= (sck_pad != sck_q) ? 3'd1 : (sck_hold   == 3'd7 ? 3'd7 : sck_hold   + 3'd1);
    since_rise <= pad_rise           ? 3'd1 : (since_rise == 3'd7 ? 3'd7 : since_rise + 3'd1);
    cs_low     <= cs_n_pad           ? 3'd0 : (cs_low     == 3'd7 ? 3'd7 : cs_low     + 3'd1);
  end

  // ---------------------------------------------- environment (HOST_PROTOCOL)
  // Not guarded by f_past_valid: these are properties of the board's SPI
  // master, true in every cycle including the one the chip leaves reset in.
  always @(*) begin
    // "SCK period >= 8 core clocks": each level is held at least 4 clocks.
    if (sck_pad != sck_q) assume (sck_hold >= 3'd4);
    // CPOL = 0: SCK idles low, so it is low whenever CS_n is high and when
    // CS_n is asserted.
    if (cs_n_pad) assume (!sck_pad);
    // "CS_n low at least 4 clocks before the first SCK edge", and one
    // transaction per CS_n low period: CS_n also stays low for 4 clocks after
    // the last edge, which is what gives the byte layer time to report.
    if (pad_rise)   assume (cs_low >= 3'd4);
    if (cs_release) assume (since_rise >= 3'd4);
  end

`ifndef LOOM_FV_SPI_RESET_IN_TRANSACTION
  // The chip is not reset in the middle of a transaction: `rst_n` is only low
  // while `CS_n` is high. Without this the SCK synchroniser is forced low
  // while the pad is high, and releasing reset produces a phantom SCK rising
  // edge that shifts the byte framing by one bit for the rest of the
  // transaction. That is finding F-3 in formal/README.md; the `xfail` task
  // drops this line to reproduce it.
  // No f_past_valid guard: cycle 0 is exactly the cycle where the chip comes
  // out of reset, which is the case this rules out.
  always @(*) if (!rst_n) assume (cs_n_pad);
`endif

  // ---------------------------------------------- reference byte assembler
  // MSB first, one bit per SCK rising edge while CS_n is low.
  reg [2:0] ref_cnt;
  reg [7:0] ref_sh, ref_byte;
  reg       ref_pending;      // a byte is complete at the pads, not yet reported
  wire [7:0] ref_next = {ref_sh[6:0], mosi_pad};

  always @(posedge clk) begin
    if (!rst_n || cs_n_pad) begin
      // "CS high resets the byte counter": a word cut short is void.
      ref_cnt     <= 3'd0;
      ref_pending <= 1'b0;
    end else begin
      if (pad_rise) begin
        ref_sh  <= ref_next;
        ref_cnt <= ref_cnt + 3'd1;
        if (ref_cnt == 3'd7) begin
          ref_byte    <= ref_next;
          ref_pending <= 1'b1;
        end
      end
      if (byte_done) ref_pending <= 1'b0;
    end
  end

  // ==================================================================
  // SPI-1A: no byte is invented. `byte_done` only ever pulses for a byte the
  // pads have actually completed and that has not been reported yet.
  // ==================================================================
  always @(*) if (f_past_valid && rst_n)
    if (byte_done) assert (ref_pending);

  // ==================================================================
  // SPI-1B: the byte delivered is the byte shifted in, MSB first.
  // ==================================================================
  always @(*) if (f_past_valid && rst_n)
    if (byte_done) assert (rx_byte == ref_byte);

  // ==================================================================
  // SPI-1C: no byte is lost. A completed byte is always reported before the
  // next SCK rising edge, and before CS_n is released, so with SPI-1A every
  // byte is delivered exactly once.
  // ==================================================================
  always @(*) if (f_past_valid && rst_n) begin
    if (pad_rise)   assert (!ref_pending);
    if (cs_release) assert (!ref_pending);
  end

  // ------------------------------------------------- cover (anti-vacuity)
  always @(*) if (f_past_valid && rst_n) begin
    cover (byte_done);                          // a byte is delivered
    cover (byte_done && (rx_byte == 8'hA5));    // with real data
    cover (ref_pending);                        // and the window is short
    cover (cs_release && (ref_cnt != 3'd0));    // a word cut short by CS_n
  end
  reg r_done;
  always @(posedge clk) begin
    if (!rst_n || cs_n_pad) r_done <= 1'b0;
    else                    r_done <= r_done | byte_done;
  end
  always @(*) if (f_past_valid && rst_n)
    cover (byte_done && r_done);                // two bytes in one transaction

endmodule

bind loom_spi_host loom_spi_props u_spi_props (
    .clk(clk), .rst_n(rst_n),
    .cs_n_pad(cs_n_pad), .sck_pad(sck_pad), .mosi_pad(mosi_pad),
    .byte_done(byte_done), .rx_byte(rx_byte)
);

`default_nettype wire
