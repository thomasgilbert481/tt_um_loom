<!---

This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

## How it works

**Branch `sram-smoke` (milestone M0.5).** This is not the Loom core. It is the
smallest design that puts an IHP foundry SRAM hard macro through the Tiny
Tapeout sg13cmos5l flow, so that the result -- hardening, DRC, LVS and the TT
precheck, pass or fail -- can be reported back. As far as we know no SRAM macro
has been hardened on cmos5l before; `main` carries the real project.

The design is one `RM_IHPSG13_1P_512x16_c2_bm_bist` (512 words x 16 bits,
single port, 236.80 x 191.34 um) wrapped in `src/loom_imem_macro.v`, plus a
register interface that lets a host write and read every word through the 24
Tiny Tapeout pins. The wrapper is the one the Loom instruction memory will use
later: address in, data out one cycle later, plus a host write port, with a
write winning over a read in the same cycle because the macro is single ported.

The macro is placed un-rotated at (12, 10) um in the 2x2 block and the Metal4
power stripes are pitched and offset to land inside its Metal4 power pins --
on cmos5l the PDN's vertical layer *is* Metal4, the same layer the macro's
power pins are on, so the stripes have to sit inside the pins rather than cross
them from above. `src/pdn_cfg.tcl` shows the arithmetic.

## How to test

25 bits of state have to go through 8 data pins, so the host loads four byte
registers and then fires a command. Every strobe is edge triggered after a
two-flop synchroniser, so bit-banging it slowly from a demo board is fine.

| Pin | Name | Meaning |
|---|---|---|
| `ui[1:0]` | REGSEL | 0 = ADDR_LO, 1 = ADDR_HI (bit 0 = addr[8]), 2 = WDATA_LO, 3 = WDATA_HI |
| `ui[2]` | REG_WR | rising edge: the selected register takes `uio` |
| `ui[3]` | RD_OE | high: `uio` is driven with the read-back byte |
| `ui[4]` | MEM_WR | rising edge: write WDATA to SRAM[ADDR] |
| `ui[5]` | MEM_RD | rising edge: read SRAM[ADDR] into RDATA |
| `ui[6]` | UIO_SEL | with RD_OE high: 0 = RDATA[15:8], 1 = STATUS |
| `ui[7]` | -- | unused |
| `uo[7:0]` | RDATA[7:0] | low byte of the last word read, always driven |
| `uio[7:0]` | data | in: register write data; out (RD_OE): RDATA[15:8] or STATUS |

`STATUS = {6'b0, RD_VALID, BUSY}`. BUSY is high for the one cycle an access
occupies; RD_VALID sticks once a read has completed, so "read back zero" can be
told apart from "never read".

To write one word: REG_WR the four registers, then pulse MEM_WR. To read one:
REG_WR the two address registers, pulse MEM_RD, wait two clocks, take the low
byte from `uo_out` and raise RD_OE to take the high byte from `uio`. Hold every
strobe high for at least two clocks and low for at least two clocks.

Note that SRAM powers up with undefined contents: write a word before reading
it. Reset does not clear the array.

`test/test.py` drives exactly this protocol: walking ones on all 16 data bits,
a distinct word written to and read back from all 512 addresses, random
interleaved access against a shadow model, and the write-wins-over-read rule.
The same test runs against the RTL (with the vendored behavioural model from
`macro/`) and against the gate-level netlist.

## External hardware

None. A demo board or any GPIO host that can drive 8 inputs and read 8 outputs
is enough; a logic analyser is useful if a word comes back wrong.
