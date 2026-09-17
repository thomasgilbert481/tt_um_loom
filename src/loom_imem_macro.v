/*
 * loom_imem_macro.v - IHP 512x16 SRAM macro behind the loom_imem interface
 *
 * Copyright (c) 2026 Thomas Gilbert
 * SPDX-License-Identifier: Apache-2.0
 *
 * ---------------------------------------------------------------------------
 * WHAT THIS IS
 * ---------------------------------------------------------------------------
 * The MACRO backend of the instruction memory (docs/ARCHITECTURE.md section 12,
 * option 1). One RM_IHPSG13_1P_512x16_c2_bm_bist holds 512 x 16 bits, which is
 * one Loom instruction per word with no lane muxing. On this branch it is
 * driven by the pin-level tester in tt_um_loom.v; later it drops in behind
 * loom_imem.v unchanged.
 *
 * ---------------------------------------------------------------------------
 * TIMING CONTRACT
 * ---------------------------------------------------------------------------
 * Single port, synchronous, one access per clock:
 *
 *   read   present `addr` on cycle N -> `rdata` is the contents of that word
 *          from the start of cycle N+1 and holds until the next completed read.
 *          There is no read enable: every cycle that is not a write is a read.
 *
 *   write  `we` with `waddr`/`wdata` on cycle N -> the word is stored on the
 *          N->N+1 edge. All 16 bits are written (the macro's bit mask is tied
 *          all-ones).
 *
 *   conflict  the port is single-ported, so a write wins: on a cycle with
 *          `we` high no read is performed and `rdata` keeps its previous
 *          value. This is why `A_REN` is `~we` rather than a constant 1: with
 *          both enables high the macro does a write-through and returns the
 *          data written to `waddr`, not the data at `addr`, which would be a
 *          silently wrong read.
 *
 *   reset  `rst_n` low disables the macro (`A_MEN` = 0), so neither a read nor
 *          a write happens and `rdata` holds. Memory contents are NOT reset;
 *          SRAM powers up undefined and the host must write before it reads.
 *
 * ---------------------------------------------------------------------------
 * MACRO PIN TIE-OFFS
 * ---------------------------------------------------------------------------
 * Port names, widths and polarities were read out of the vendored model
 * macro/RM_IHPSG13_1P_512x16_c2_bm_bist/RM_IHPSG13_1P_512x16_c2_bm_bist.v and
 * the core model RM_IHPSG13_1P_core_behavioral_bm_bist.v, not from memory.
 * Every enable in this macro is ACTIVE HIGH.
 *
 *   A_CLK       clk          single clock for the port.
 *   A_MEN       rst_n        memory enable. The core model does nothing at all
 *                            unless A_MEN is high, so holding it low during
 *                            reset guarantees no spurious access while the
 *                            rest of the design is still coming up.
 *   A_WEN       we           write enable, active high.
 *   A_REN       ~we          read enable, active high. See "conflict" above.
 *   A_ADDR[8:0] we?waddr:addr  one address port for both directions.
 *   A_DIN[15:0] wdata
 *   A_DOUT[15:0] rdata       already registered inside the macro (the model
 *                            drives it from a reg loaded on posedge A_CLK), so
 *                            no output flop is added here.
 *   A_DLY       1'b1         delay select. The vendored model contains an
 *                            explicit check that $stops the simulation if
 *                            A_DLY is ever anything but 1, and the reference
 *                            project tt_um_urish_sram_test ties it the same
 *                            way. Tied high, permanently.
 *   A_BM[15:0]  16'hFFFF     per-bit write mask, 1 = write this bit. All bits
 *                            enabled: this design never does partial writes.
 *   A_BIST_EN   1'b0         BIST disabled. With A_BIST_EN low the macro's
 *                            internal mux selects the functional port, so
 *                            every other A_BIST_* pin is a don't-care; they are
 *                            tied to 0 anyway so that synthesis, the netlist
 *                            and the extracted SPICE all agree on a constant
 *                            rather than leaving inputs floating.
 *   A_BIST_CLK  1'b0         }
 *   A_BIST_MEN  1'b0         } tied off with A_BIST_EN, see above. Tying the
 *   A_BIST_WEN  1'b0         } BIST clock to 0 also keeps it out of the clock
 *   A_BIST_REN  1'b0         } tree.
 *   A_BIST_ADDR 9'b0         }
 *   A_BIST_DIN  16'b0        }
 *   A_BIST_BM   16'b0        }
 *
 * There is no chip-select, no byte enable and no ready/valid on this macro:
 * every pin it has is listed above.
 */

`default_nettype none

module loom_imem_macro (
    input  wire        clk,
    input  wire        rst_n,      // active low; holds the macro disabled
    // read port
    input  wire [8:0]  addr,       // word address, valid the cycle before rdata
    output wire [15:0] rdata,      // contents of addr, one cycle later
    // host write port
    input  wire        we,
    input  wire [8:0]  waddr,
    input  wire [15:0] wdata
);

  wire [8:0] macro_addr = we ? waddr : addr;

  RM_IHPSG13_1P_512x16_c2_bm_bist sram (
      .A_CLK      (clk),
      .A_MEN      (rst_n),
      .A_WEN      (we),
      .A_REN      (~we),
      .A_ADDR     (macro_addr),
      .A_DIN      (wdata),
      .A_DLY      (1'b1),
      .A_DOUT     (rdata),
      .A_BM       (16'hFFFF),
      .A_BIST_CLK (1'b0),
      .A_BIST_EN  (1'b0),
      .A_BIST_MEN (1'b0),
      .A_BIST_WEN (1'b0),
      .A_BIST_REN (1'b0),
      .A_BIST_ADDR(9'b0),
      .A_BIST_DIN (16'b0),
      .A_BIST_BM  (16'b0)
  );

endmodule

`default_nettype wire
