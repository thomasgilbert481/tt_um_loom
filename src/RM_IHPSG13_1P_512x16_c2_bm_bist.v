/*
 * Port-only blackbox for the IHP 512x16 SRAM macro.
 * SPDX-License-Identifier: Apache-2.0
 *
 * This file exists ONLY so that synthesis and lint have a module declaration
 * for the hard macro. It has no body: the real behaviour comes from the macro
 * GDS/LEF/LIB listed in the MACROS block of src/config.json, and the real
 * simulation model lives in
 * macro/RM_IHPSG13_1P_512x16_c2_bm_bist/ (used by test/Makefile, never by
 * synthesis). This is exactly how tt_um_urish_sram_test does it: the stub is
 * listed in info.yaml `source_files` and is what the MACROS "nl" entry points
 * at, while the behavioural model stays out of src/.
 *
 * Do not add a body. If this module ever gets one, LibreLane will synthesise
 * 8192 flops instead of instancing the macro.
 *
 * The port list is copied from the vendored model
 * macro/RM_IHPSG13_1P_512x16_c2_bm_bist/RM_IHPSG13_1P_512x16_c2_bm_bist.v.
 * Keep them identical.
 */

`default_nettype none

/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off UNDRIVEN */
module RM_IHPSG13_1P_512x16_c2_bm_bist (
    input  wire        A_CLK,
    input  wire        A_MEN,
    input  wire        A_WEN,
    input  wire        A_REN,
    input  wire [8:0]  A_ADDR,
    input  wire [15:0] A_DIN,
    input  wire        A_DLY,
    output wire [15:0] A_DOUT,
    input  wire [15:0] A_BM,
    input  wire        A_BIST_CLK,
    input  wire        A_BIST_EN,
    input  wire        A_BIST_MEN,
    input  wire        A_BIST_WEN,
    input  wire        A_BIST_REN,
    input  wire [8:0]  A_BIST_ADDR,
    input  wire [15:0] A_BIST_DIN,
    input  wire [15:0] A_BIST_BM
);
endmodule
/* verilator lint_on UNDRIVEN */
/* verilator lint_on UNUSEDSIGNAL */

`default_nettype wire
