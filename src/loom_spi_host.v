/*
 * loom_spi_host: SPI mode 0 slave, bit layer only.
 * SPDX-License-Identifier: Apache-2.0
 *
 * docs/HOST_PROTOCOL.md "Electrical": CPOL=0, CPHA=0, MSB first, one
 * transaction per CS_n low period. SCK is *sampled* by clk through a two-flop
 * synchroniser and an edge detector, so the whole chip stays in one clock
 * domain; the protocol therefore requires SCK period >= 8 core clocks and
 * CS_n low at least 4 clocks before the first SCK edge.
 *
 * Timing contract:
 *   MOSI is captured on the detected rising edge of SCK. `byte_done` is a
 *   one-clock pulse two clocks after the eighth rising edge of a byte (two
 *   synchroniser stages plus the edge register), with `rx_byte` valid in the
 *   same cycle and held until the next byte completes.
 *   `tx_byte` is loaded into the shift register on the falling edge that
 *   follows a byte boundary, so the command layer has from `byte_done` until
 *   that edge (at least a further clock) to present the next byte. MISO then
 *   changes only on detected falling edges, and is driven 0 whenever CS_n is
 *   high (the Tiny Tapeout output is always driven, there is no tristate).
 */

`default_nettype none

module loom_spi_host (
    input  wire       clk,
    input  wire       rst_n,

    // Raw pads
    input  wire       cs_n_pad,
    input  wire       sck_pad,
    input  wire       mosi_pad,
    output wire       miso,

    // Byte layer
    output wire       cs_active,    // synchronised CS_n low
    output reg        byte_done,    // one-clock pulse, rx_byte valid
    output reg [7:0]  rx_byte,
    input  wire [7:0] tx_byte
);

  wire cs_n_s, sck_s, mosi_s;
  loom_sync #(.WIDTH(1), .RESET_VAL(1'b1)) u_cs  (
      .clk(clk), .rst_n(rst_n), .d(cs_n_pad),  .q(cs_n_s));
  loom_sync #(.WIDTH(1), .RESET_VAL(1'b0)) u_sck (
      .clk(clk), .rst_n(rst_n), .d(sck_pad),   .q(sck_s));
  loom_sync #(.WIDTH(1), .RESET_VAL(1'b0)) u_mos (
      .clk(clk), .rst_n(rst_n), .d(mosi_pad),  .q(mosi_s));

  reg  sck_q;
  wire sck_rise = sck_s & ~sck_q;
  wire sck_fall = ~sck_s & sck_q;

  assign cs_active = ~cs_n_s;

  reg [6:0] rx_shift;
  reg [7:0] tx_shift;
  reg [2:0] bit_cnt;
  reg       load_pending;

  always @(posedge clk) begin
    if (!rst_n) begin
      sck_q        <= 1'b0;
      rx_shift     <= 7'd0;
      tx_shift     <= 8'd0;
      bit_cnt      <= 3'd0;
      load_pending <= 1'b0;
      byte_done    <= 1'b0;
      rx_byte      <= 8'd0;
    end else begin
      sck_q     <= sck_s;
      byte_done <= 1'b0;
      if (!cs_active) begin
        // Idle: keep the shifter primed with the first byte to send and drop
        // any partially received word (a transaction cut mid-word is void).
        bit_cnt      <= 3'd0;
        load_pending <= 1'b0;
        rx_shift     <= 7'd0;
        tx_shift     <= tx_byte;
      end else begin
        if (sck_rise) begin
          rx_shift <= {rx_shift[5:0], mosi_s};
          bit_cnt  <= bit_cnt + 3'd1;
          if (bit_cnt == 3'd7) begin
            byte_done    <= 1'b1;
            rx_byte      <= {rx_shift, mosi_s};
            load_pending <= 1'b1;
          end
        end
        if (sck_fall) begin
          if (load_pending) begin
            tx_shift     <= tx_byte;
            load_pending <= 1'b0;
          end else begin
            tx_shift <= {tx_shift[6:0], 1'b0};
          end
        end
      end
    end
  end

  assign miso = cs_active ? tx_shift[7] : 1'b0;

endmodule
