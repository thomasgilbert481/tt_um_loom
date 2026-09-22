# iCEBreaker build: does not fit (measured 2026-09-21)

`make -C fpga/icebreaker` builds `tt_um_loom` with the flop instruction memory
(`IMEM_IMPL "FLOPS"`, 512 words) for the iCEBreaker's iCE40UP5K. Synthesis
works; the design does not fit the part:

| Resource | Needed | iCE40UP5K has |
|---|---|---|
| LUT4 | **7,732** | 5,280 (146 per cent) |
| flip-flops | 3,101 | 5,280 |
| block RAM (SB_RAM40_4K) | 2 | 30 |

The instruction memory did go to block RAM (the two EBRs), so the gap is
logic, and no single block is small enough for a trim to close it: with the
hierarchy kept, `loom_core` alone is 3,930 LUTs, the timers 921, the register
file 817, the eight FIFOs 928, the host controller 530, the ALU 454. The RTL's
only build knobs are the memory type and size and the FIFO depth, and none of
them moves the total by 2,450 LUTs.

The same RTL on a Lattice ECP5-25F (`synth_ecp5`, `nextpnr-ecp5 --25k`) uses
7,948 of 24,288 LUT4 slots (32 per cent), 2,516 flip-flops (10 per cent) and one
block RAM, and closes at **47 MHz**, so any 25F-class ECP5 board (ULX3S,
OrangeCrab, iCESugar-Pro) runs the full design at a bench clock. That is the
decision in `docs/PLAN.md` (M2, FPGA box).

What is here stays useful for any iCE40 board and as the template for the next
target: `loom_icebreaker.v` maps one PMOD per chip port (ui_in, uo_out, uio
through `SB_IO` tristates with weak pull-ups), builds a power-on reset ANDed
with the button, and drives a heartbeat and HOST_IRQ onto the LEDs;
`icebreaker.pcf` has the pin map. Nothing here has run on hardware.
