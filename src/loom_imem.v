/*
 * loom_imem: instruction memory, single port, two backends (D-005, D-020).
 * SPDX-License-Identifier: Apache-2.0
 *
 * IMPL selects the backend when the design is elaborated:
 *
 *   "MACRO"  (default) one IHP RM_IHPSG13_1P_512x16_c2_bm_bist SRAM macro
 *            behind loom_imem_macro (docs/ARCHITECTURE.md 12 option 1,
 *            D-020). The macro is 512 x 16, so this backend needs
 *            WORDS = 512 and AW = 9: its address ports are 9 bits wide, and
 *            any other AW is a port-width mismatch that Verilator -Wall
 *            (scripts/check_all.sh, the lint workflow) rejects.
 *   "FLOPS"  a WORDS x 16 flip-flop array (option 2, the M1 build), kept as
 *            the fallback and built by test_flops.py. Any power of two.
 *
 * Timing contract (both backends):
 *   Synchronous read. The address presented during cycle k is registered at
 *   edge k+1 and `rdata` is valid during cycle k+1, which is the D stage of
 *   the slot that fetched in F (docs/SEMANTICS.md section 2).
 *   A write with `we` high takes effect at edge k+1. What `rdata` shows
 *   after a write cycle is not part of the contract (FLOPS: the old contents
 *   of the address; MACRO: unchanged, because the macro port does no read in
 *   a write cycle). Nothing reads it for its value: host access is only
 *   permitted while no slot is in flight, and the host reads IMEM only in
 *   the cycle right after its own read request, while the completion slot
 *   of an `ST` (docs/SEMANTICS.md 6.11, M3 slice B) is told by the same text
 *   that the word its D stage receives is unspecified, and section 8 has the
 *   co-simulation harness leave that `tr_ir` uncompared. Both backends do
 *   return the *old* contents or a stale word rather than X once the address
 *   has been written, which is what `test/tb.v`'s unwritten-word check
 *   needs: a program that stores into a word the host never loaded stops the
 *   run on the FLOPS backend.
 *   `en` gates both the read and the write, so a bubble slot leaves `rdata`
 *   unchanged. The array has no reset (SEMANTICS 5: "Instruction memory is
 *   not reset"); the macro powers up undefined, like the flops.
 *
 * MACRO wiring. loom_imem_macro (brought over unchanged from the sram-smoke
 * test) drives the macro's memory enable A_MEN from its `rst_n` input, active
 * high: A_MEN low means no read and no write, and A_DOUT keeps the last word
 * read. Feeding it `en` instead of the chip reset gives exactly the FLOPS
 * behaviour above: every cycle without `en` is a no-access cycle, so a bubble
 * leaves `rdata` alone and the macro is not clocked through reads nobody
 * wants. `rst_n` is not needed: loom_top's `en` is low from the first clock
 * edge of a reset on (RUN, STEP_REQ and every host request reset to 0), the
 * macro's model treats an X enable before that edge as no access, and a
 * stray access before the first reset edge in silicon only touches contents
 * that are undefined at power-up anyway. The wrapper's separate write
 * address is the same `addr` here, because the port is single: whoever owns
 * the port in a cycle where loom_top raises `we` owns the write too, and
 * `we` never rises without `en`. Since M3 slice B (6.11, D-027) that is
 * either the host or the core's own `ST`, which loom_top arbitrates: the
 * core's data access rides the thread's F cycle, where `en` is already high
 * because the slot is valid. The macro's A_DOUT is already a register, so no
 * flop is added on `rdata` and the read latency is the one cycle above.
 */

`default_nettype none

module loom_imem #(
    parameter         IMPL  = "MACRO",
    parameter [15:0]  WORDS = 16'd512,
    parameter integer AW    = 9
) (
    input  wire          clk,
    input  wire          en,
    input  wire          we,
    input  wire [AW-1:0] addr,
    input  wire [15:0]   wdata,
    output wire [15:0]   rdata
);

  generate
    if (IMPL == "MACRO") begin : g_macro
      loom_imem_macro u_macro (
          .clk  (clk),
          .rst_n(en),        // A_MEN: the macro only acts in cycles with en
          .addr (addr),
          .rdata(rdata),
          .we   (we),
          .waddr(addr),
          .wdata(wdata)
      );
    end else begin : g_flops
      reg [15:0] mem [0:WORDS-1];
      reg [15:0] rdata_q;

      always @(posedge clk) begin
        if (en) begin
          if (we) mem[addr] <= wdata;
          rdata_q <= mem[addr];
        end
      end

      assign rdata = rdata_q;
    end
  endgenerate

endmodule
