/*
 * loom_be: the per-thread bit-engine state, manual mode (docs/SEMANTICS.md
 * 6.9). SPDX-License-Identifier: Apache-2.0
 *
 * Holds, per thread: SR (16), CNT (5), CRC (16) and the CSRs BE_CFG,
 * BE_PINS (10), BE_RELOAD (5), CRC_POLY (16) and CRC_INIT (16), all reset to
 * 0. Of BE_CFG only DIR (bit 1), INV (bit 7) and CRC_EN (bit 9) exist at
 * M2; the other bits are not stored, so they ignore writes and read 0.
 * BE_RELOAD is only a register until auto mode (M3) gives it a use.
 *
 * The shift and CRC arithmetic of SHO/SHI/CRCI is done in loom_core's X
 * stage; this module only stores what the W stage commits.
 *
 * Timing contract:
 *   Every output is a register value, i.e. what an instruction in X or a
 *   debug read sees in the current cycle.
 *   Commit port: `cm_sel` is one-hot (bit t: a valid slot of thread t is in
 *   W, built from loom_core's ring, D-019); each `*_we` with its data is
 *   written at the edge that ends the cycle.
 *   Host port: `h_sel` is one-hot (a debug write to thread t, only while the
 *   thread is halted, so never together with a commit of that thread); the
 *   commit wins if both ever hit one register.
 *   Per-thread outputs are flattened: thread t occupies [16t +: 16] of the
 *   16-bit vectors, [5t +: 5] of cnt_all and reload_all, [10t +: 10] of
 *   pins_all and [3t +: 3] of cfg_all ({CRC_EN, INV, DIR}).
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
    input  wire [15:0] h_wdata,

    output wire [63:0] sr_all,
    output wire [19:0] cnt_all,
    output wire [63:0] crc_all,
    output wire [11:0] cfg_all,
    output wire [39:0] pins_all,
    output wire [19:0] reload_all,
    output wire [63:0] poly_all,
    output wire [63:0] init_all
);

  genvar t;
  generate
    for (t = 0; t < 4; t = t + 1) begin : g_thread
      reg [15:0] sr, crc, poly, init;
      reg [4:0]  cnt, reload;
      reg [2:0]  cfg;                 // {CRC_EN, INV, DIR}
      reg [9:0]  pins;

      wire c = cm_sel[t];
      wire h = h_sel[t];
      // BE_CFG as a 16-bit value -> the three stored bits.
      wire [2:0] cm_cfg = {cm_csr[9], cm_csr[7], cm_csr[1]};
      wire [2:0] h_cfg  = {h_wdata[9], h_wdata[7], h_wdata[1]};

      always @(posedge clk) begin
        if (!rst_n) begin
          sr <= 16'd0; crc <= 16'd0; poly <= 16'd0; init <= 16'd0;
          cnt <= 5'd0; reload <= 5'd0; cfg <= 3'd0; pins <= 10'd0;
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
        end
      end

      assign sr_all[16*t +: 16]    = sr;
      assign crc_all[16*t +: 16]   = crc;
      assign poly_all[16*t +: 16]  = poly;
      assign init_all[16*t +: 16]  = init;
      assign cnt_all[5*t +: 5]     = cnt;
      assign reload_all[5*t +: 5]  = reload;
      assign cfg_all[3*t +: 3]     = cfg;
      assign pins_all[10*t +: 10]  = pins;
    end
  endgenerate

endmodule
