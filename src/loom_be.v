/*
 * loom_be: the per-thread bit-engine state, manual mode (docs/SEMANTICS.md
 * 6.9 and 6.9.1). SPDX-License-Identifier: Apache-2.0
 *
 * Holds, per thread: SR (16), CNT (5), CRC (16) and the CSRs BE_CFG,
 * BE_PINS (10), BE_RELOAD (5), CRC_POLY (16) and CRC_INIT (16), all reset to
 * 0. Of BE_CFG six fields exist after M3 slice A: DIR (bit 1), ENC (4:3),
 * STUFF (6:5), INV (bit 7), CRC_EN (bit 9) and DIFF (bit 10); the reserved
 * value 3 of ENC and of STUFF is stored as 0 (6.9.1). MODE (0), RXTX (2),
 * AUTOPULL (8) and bits 12:11 are not stored, so they ignore writes and read
 * 0 until slice C. BE_RELOAD is only a register until auto mode gives it a
 * use.
 *
 * Slice A also adds the per-thread encoder state (6.9.1), one 8-bit word per
 * thread in exactly the layout debug 0x27 reports:
 *   {FIRST, HALF, PEND, RVAL, RUN[2:0], LVL}
 * It resets to 0, is cleared by every write to BE_CFG (a CSRW commit or a
 * host debug write) and by the thread's CTRL.RESET, is updated by the commit
 * of an SHO or SHI, and is written by the host at debug 0x27.
 *
 * The shift, CRC, encoder and stuffing arithmetic of SHO/SHI/CRCI is done in
 * loom_core's X stage; this module only stores what the W stage commits.
 *
 * Timing contract:
 *   Every output is a register value, i.e. what an instruction in X or a
 *   debug read sees in the current cycle.
 *   Commit port: `cm_sel` is one-hot (bit t: a valid slot of thread t is in
 *   W, built from loom_core's ring, D-019); each `*_we` with its data is
 *   written at the edge that ends the cycle.
 *   Host port: `h_sel` is one-hot (a debug write to thread t, only while the
 *   thread is halted, so never together with a commit of that thread); the
 *   commit wins if both ever hit one register. `h_reset` (CTRL.RESET) is
 *   last, as in loom_core's M1 priority order.
 *   Per-thread outputs are flattened: thread t occupies [16t +: 16] of the
 *   16-bit vectors, [5t +: 5] of cnt_all and reload_all, [10t +: 10] of
 *   pins_all, [8t +: 8] of cfg_all ({DIFF, STUFF, ENC, CRC_EN, INV, DIR})
 *   and [8t +: 8] of enc_all.
 */

`default_nettype none

module loom_be (
    input  wire        clk,
    input  wire        rst_n,

    // Commit port (W stage).
    input  wire [3:0]  cm_sel,
    input  wire        cm_sr_we,
    input  wire [15:0] cm_sr,
    input  wire        cm_cnt_we,
    input  wire [4:0]  cm_cnt,
    input  wire        cm_crc_we,
    input  wire [15:0] cm_crc,
    input  wire        cm_cfg_we,
    input  wire        cm_pins_we,
    input  wire        cm_reload_we,
    input  wire        cm_poly_we,
    input  wire        cm_init_we,
    input  wire [15:0] cm_csr,          // data of a CSRW to a configuration CSR
    input  wire        cm_enc_we,       // an SHO/SHI updates the encoder state
    input  wire [7:0]  cm_enc,

    // Host debug-write port.
    input  wire [3:0]  h_sel,
    input  wire        h_sr_we,
    input  wire        h_cnt_we,
    input  wire        h_crc_we,
    input  wire        h_cfg_we,
    input  wire        h_pins_we,
    input  wire        h_reload_we,
    input  wire        h_poly_we,
    input  wire        h_init_we,
    input  wire        h_enc_we,        // debug 0x27
    input  wire [15:0] h_wdata,

    // CTRL.RESET, one bit per thread: clears the encoder state (6.9.1).
    input  wire [3:0]  h_reset,

    output wire [63:0] sr_all,
    output wire [19:0] cnt_all,
    output wire [63:0] crc_all,
    output wire [31:0] cfg_all,
    output wire [39:0] pins_all,
    output wire [19:0] reload_all,
    output wire [63:0] poly_all,
    output wire [63:0] init_all,
    output wire [31:0] enc_all
);

  genvar t;
  generate
    for (t = 0; t < 4; t = t + 1) begin : g_thread
      reg [15:0] sr, crc, poly, init;
      reg [4:0]  cnt, reload;
      reg [7:0]  cfg;                 // {DIFF, STUFF, ENC, CRC_EN, INV, DIR}
      reg [7:0]  enc;                 // {FIRST, HALF, PEND, RVAL, RUN, LVL}
      reg [9:0]  pins;

      wire c = cm_sel[t];
      wire h = h_sel[t];
      // BE_CFG as a 16-bit value -> the six stored fields. The reserved
      // value 3 of ENC and of STUFF is stored as 0 (SEMANTICS 6.9.1).
      wire [1:0] cm_enc_f   = (cm_csr[4:3] == 2'b11) ? 2'b00 : cm_csr[4:3];
      wire [1:0] cm_stuff_f = (cm_csr[6:5] == 2'b11) ? 2'b00 : cm_csr[6:5];
      wire [1:0] h_enc_f    = (h_wdata[4:3] == 2'b11) ? 2'b00 : h_wdata[4:3];
      wire [1:0] h_stuff_f  = (h_wdata[6:5] == 2'b11) ? 2'b00 : h_wdata[6:5];
      wire [7:0] cm_cfg = {cm_csr[10], cm_stuff_f, cm_enc_f,
                           cm_csr[9], cm_csr[7], cm_csr[1]};
      wire [7:0] h_cfg  = {h_wdata[10], h_stuff_f, h_enc_f,
                           h_wdata[9], h_wdata[7], h_wdata[1]};

      always @(posedge clk) begin
        if (!rst_n) begin
          sr <= 16'd0; crc <= 16'd0; poly <= 16'd0; init <= 16'd0;
          cnt <= 5'd0; reload <= 5'd0; cfg <= 8'd0; pins <= 10'd0;
          enc <= 8'd0;
        end else begin
          if (c & cm_sr_we)            sr <= cm_sr;
          else if (h & h_sr_we)        sr <= h_wdata;

          if (c & cm_cnt_we)           cnt <= cm_cnt;
          else if (h & h_cnt_we)       cnt <= h_wdata[4:0];

          if (c & cm_crc_we)           crc <= cm_crc;
          else if (h & h_crc_we)       crc <= h_wdata;

          if (c & cm_cfg_we)           cfg <= cm_cfg;
          else if (h & h_cfg_we)       cfg <= h_cfg;

          if (c & cm_pins_we)          pins <= cm_csr[9:0];
          else if (h & h_pins_we)      pins <= h_wdata[9:0];

          if (c & cm_reload_we)        reload <= cm_csr[4:0];
          else if (h & h_reload_we)    reload <= h_wdata[4:0];

          if (c & cm_poly_we)          poly <= cm_csr;
          else if (h & h_poly_we)      poly <= h_wdata;

          if (c & cm_init_we)          init <= cm_csr;
          else if (h & h_init_we)      init <= h_wdata;

          // Encoder state (6.9.1). Every write to BE_CFG clears it, whether
          // it comes from the thread's CSRW or from the host; an SHO/SHI
          // commit updates it; the host writes it at debug 0x27; CTRL.RESET
          // clears it. Commit first, then the host, then CTRL.RESET.
          if (c & cm_cfg_we)           enc <= 8'd0;
          else if (c & cm_enc_we)      enc <= cm_enc;
          else if (h & h_cfg_we)       enc <= 8'd0;
          else if (h & h_enc_we)       enc <= h_wdata[7:0];
          else if (h_reset[t])         enc <= 8'd0;
        end
      end

      assign sr_all[16*t +: 16]    = sr;
      assign crc_all[16*t +: 16]   = crc;
      assign poly_all[16*t +: 16]  = poly;
      assign init_all[16*t +: 16]  = init;
      assign cnt_all[5*t +: 5]     = cnt;
      assign reload_all[5*t +: 5]  = reload;
      assign cfg_all[8*t +: 8]     = cfg;
      assign enc_all[8*t +: 8]     = enc;
      assign pins_all[10*t +: 10]  = pins;
    end
  endgenerate

endmodule
