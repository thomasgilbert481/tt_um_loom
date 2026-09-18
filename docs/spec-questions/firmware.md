# Spec questions from the host library, protocol models and firmware (M2 software)

Collected by the M2 software session (host library `tools/loomhost`, protocol
models `tools/protomodels`, firmware `uart_tx_fifo`, `uart_rx`, `spi_master`,
`i2c_master`), 2026-09-18. That session was lost to an infrastructure stall
before writing this file; the director transcribed it from the session's
progress notes. Each item says what the code does today.

## 1. HOST_PROTOCOL worked example 1 has the wrong CMD byte

The first worked example writes IMEM with CMD `0x81`, but the CMD format
`{RW, SPACE[2:0], 0000}` gives `0x90` for a write to space 1 (the RTL decodes
`cmd[6:4]`). Examples 2 and 3 are consistent with the format.
**Code today:** `tools/loomhost/protocol.py` follows the format (`0x90`).
**Fix wanted:** correct the example.

**Resolution (director, 2026-09-18):** fixed; the example now uses 0x90.

## 2. CAPS layout differs between HOST_PROTOCOL and SEMANTICS

HOST_PROTOCOL CTRL `0x19` still describes the v0.1 layout; SEMANTICS section 5
has the current one. **Code today:** SEMANTICS. **Fix wanted:** make
HOST_PROTOCOL point to SEMANTICS 5.

**Resolution (director, 2026-09-18):** fixed; CTRL 0x19 points to SEMANTICS 5.

## 3. RESET_PC default in HOST_PROTOCOL is stale

HOST_PROTOCOL says `t * 0x100`; SEMANTICS 5 and D-017 say
`t * (IMEM_WORDS / 4)`. **Code today:** SEMANTICS.

**Resolution (director, 2026-09-18):** fixed.

## 4. FIFO-space reads: when is the entry popped?

The M1 RTL prefetches read words one byte ahead, so the pop point of a FIFO
read (CS rising mid-word, reading past the last entry) was undefined.
**Code today:** `ModelTransport` follows the RTL reading recorded in
`docs/spec-questions/rtl-m2.md` item 1: peek when the word is loaded, pop at
the end of the word, a word cut short by CS pops nothing, BADOP[14] at the
end of an empty word. **Fix wanted:** write that into SEMANTICS 6.7 and
HOST_PROTOCOL.

**Resolution (director, 2026-09-18):** the peek/pop split is now in SEMANTICS 6.7 and HOST_PROTOCOL SPACE 3.

## 5. FIFO status word bit positions are not written down

HOST_PROTOCOL lists the fields of the status word at `0x0100 + t` without bit
positions. **Code today:** `[11:8] OUTQ_CNT, [7:4] INQ_CNT, [3] OUTQ_EMPTY,
[2] OUTQ_FULL, [1] INQ_EMPTY, [0] INQ_FULL`, which the M2 RTL was confirmed
to use. **Fix wanted:** put the positions in HOST_PROTOCOL.

**Resolution (director, 2026-09-18):** bit positions now in HOST_PROTOCOL.

## 6. Multi-word FIFO transactions do not auto-increment

HOST_PROTOCOL: "the low two bits stay fixed". **Code today:** the transport
never increments in the FIFO space, matching the RTL. Confirm in the text.

**Resolution (director, 2026-09-18):** confirmed in HOST_PROTOCOL SPACE 3.

## 7. The golden model lacks some host-visible registers

The model does not implement BADOP[14] on host FIFO misuse, CTRL.RESET
emptying the FIFOs, IRQ_EN / IRQ_STAT / IRQ_EN2 / HOST_IRQ, SWIRQ clearing,
ID / VERSION, STEPS writes, or a debug CSR window by number; its debug writes
to non-`r` registers are not gated on "halted". **Code today:**
`ModelTransport` shims these with the HOST_PROTOCOL behaviour and says so in
comments; five tests that depend on the model itself are `xfail` with the
section named. **Fix wanted:** the model's M2 update (planned).

**Resolution (director, 2026-09-18):** scheduled as the golden model's M2 update.

## 8. `firmware/uart_tx.loom`: the first start bit after idle is 0 to 1 bit long

The program anchors with `SETD 0` and then drives the start bit with
`WAITD 1` at a tick of one bit time, so the start bit after an idle period
lasts anywhere from 0 to 1 bit depending on where in the tick the `SETD`
landed. Receivers that resynchronise on the start edge tolerate a short start
bit only down to their own sampling point. **Code today:** unchanged (not this
session's file). `uart_tx_fifo.loom` avoids it by ticking faster than the bit
rate. **Fix wanted:** re-anchor with `SETD 0` and wait one extra tick, or tick
at a multiple of the bit rate, in `uart_tx.loom` and `uart_hello.loom`.

**Resolution (director, 2026-09-18):** a real firmware bug; fix scheduled with the next firmware work (PLAN M2).
