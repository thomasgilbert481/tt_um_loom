# Spec questions from the M1 RTL implementation

Written by the RTL implementer while building M1 from `docs/SEMANTICS.md`,
`docs/HOST_PROTOCOL.md`, `docs/ARCHITECTURE.md` and `isa/isa.yaml`, without
looking at `tools/loomsim` or `tools/loomasm` (VERIFICATION.md METH-1).

Each entry says what is ambiguous or contradictory, the reading the RTL takes,
and what a different reading would change. "Resolved by the director" marks
the ones answered mid-implementation; those are already in the RTL.

---

## 1. IMEM reads while a thread runs (contradiction) — RESOLVED

`docs/HOST_PROTOCOL.md` SPACE 1: "Reads are always allowed."
`docs/SEMANTICS.md` 7: "Host IMEM reads and writes are valid only while
`RUN == 0` and no step is in flight; otherwise writes are dropped, reads
return 0, and `BADOP[15]` (host access error) is set."

**Reading taken:** SEMANTICS, because it is the contract that wins. A read
while the core is busy returns 0 and sets BADOP[15]. HOST_PROTOCOL should be
corrected.

## 2. CAPS layout — RESOLVED by the director

`docs/HOST_PROTOCOL.md` gives `CAPS = {IMEM_WORDS[11:0]/16, DMEM_PRESENT,
FIFO_DEPTH_LOG2[1:0], BOOTROM}`, which allocates all 16 bits and therefore has
no way to report the bit engine absent, and no encoding for "no FIFOs at all"
(depth log2 = 0 would mean depth 1).

**Reading taken (director's clarification 7):** `[15:12]` log2(IMEM_WORDS),
`[11:7]` zero, `[6]` boot ROM, `[5]` data memory, `[4]` bit engine, `[3]`
FIFOs built, `[2:0]` FIFO_DEPTH_LOG2. The M1 256-word build reads `0x8000`.

## 3. RESET_PC defaults — RESOLVED by the director

`docs/SEMANTICS.md` 5: "`RESET_PC[t] = t * 0x100` (masked to the memory
size)". With `IMEM_WORDS = 256` that masks all four vectors to 0, so the
default is useless for four threads.

**Reading taken (director's clarification 2):** `RESET_PC[t] = t *
(IMEM_WORDS / 4)`, i.e. 0, 64, 128, 192 for 256 words and `t * 0x100` for
1024.

## 4. FLAGS bit order (contradiction)

`docs/HOST_PROTOCOL.md` DEBUG 0x09 is labelled "FLAGS {Z, C, T}";
`docs/SEMANTICS.md` 5 says "`FLAGS` CSR = `{T, C, Z}` in bits 2:0".

**Reading taken:** SEMANTICS. Bit 0 is Z, bit 1 is C, bit 2 is T, in the CSR
and in the debug register alike. The HOST_PROTOCOL line is a name list, not a
bit order, but it reads like one.

## 5. `SWIRQ` has no host address

`docs/SEMANTICS.md` 6.6: "`CSRW HOST_IRQ` sets `SWIRQ[t]` (host-visible,
host-cleared)". No address for it exists in `docs/HOST_PROTOCOL.md`.

**Reading taken:** CTRL `0x001B` is SWIRQ, read as `{12'b0, SWIRQ[3:0]}`,
write 1 to clear, and `HOST_IRQ = |(IRQ_STAT & IRQ_EN) | |SWIRQ`. Documented
in `docs/INTERFACES.md`. If the director prefers another address this is a
one-line change.

## 6. Debug addresses for WAIT_ACTIVE and DT

`docs/HOST_PROTOCOL.md` SPACE 4 lists no address for WAIT_ACTIVE or DT, and
only a note ("0x21 returns RS1 and the stack depth") for the stack.

**Reading taken:** 0x21 = `{4'b0, DEPTH[1:0], RS1[9:0]}`, 0x22 = WAIT_ACTIVE
in bit 0, 0x23 = DT. All three are readable and writable while the thread is
halted.

## 7. IRQ_STAT is described as one and a half words

`docs/HOST_PROTOCOL.md` 0x0011: "{SFLAGS[7:0], INQ_NOT_FULL[3:0],
OUTQ_NOT_EMPTY[3:0]} plus HALTED in bits 19:16 of a second word at 0x0012",
and IRQ_EN is "mask over IRQ_STAT" with no statement about which word it
masks. IRQ is not in the M1 list at all, but `uo_out[6]` has to be driven.

**Reading taken:** 0x0011 is the maskable word (the two FIFO fields read 0
until M2), 0x0012 reads `{12'b0, HALTED[3:0]}` and is not maskable, IRQ_EN
masks 0x0011 only. HOST_IRQ is the OR of that with `|SWIRQ`. With IRQ_EN
reset to 0 the pin is low unless firmware uses `CSRW HOST_IRQ`.

## 8. Does `CSRW PIN_OUT` go through the open-drain rule?

`docs/SEMANTICS.md` 6.3 defines the open-drain transform for a "pin write of
value b to index i", and lists `SETP`, `OUT` and the bit engine as the things
that perform pin writes. 6.6 says `CSRW PIN_OUT/PIN_OE/OD_MASK` "write the
whole register".

**Reading taken:** literal. `CSRW PIN_OUT` writes PIN_OUT[13:0] directly with
no open-drain transform and no bit masking; only `SETP` and `OUT` go through
the rule. Firmware that wants open-drain behaviour must use `SETP`/`OUT`.

## 9. Unimplemented CSRs and BADOP — RESOLVED by the director

`docs/SEMANTICS.md` 9 sets BADOP for "any instruction whose feature is not
built", while 6.6 says "write-only and unimplemented CSRs read 0" without
mentioning BADOP. `CSRR rd, SR` is a built instruction naming an unbuilt
feature.

**Reading taken (director's clarification 8):** only instructions set BADOP.
`CSRR`/`CSRW` of SR, CNT, CRC, the BE and CRC CSRs read 0 and ignore writes
without setting BADOP.

## 10. PC is 10 bits, IMEM is 256 words

`next = (PC + 1) mod 2^10` is unconditional, but `IMEM_WORDS` is 256 at M1.

**Reading taken:** PC stays a full 10 bits everywhere (debug, RS0/RS1, branch
targets); the fetch uses `PC[IMEM_AW-1:0]`, so addresses alias every 256
words. A program that runs off the end wraps into the same memory rather than
faulting. Nothing in the spec says otherwise, but it is worth stating.

## 11. Who counts as "halted" for host access — RESOLVED by the director

`docs/SEMANTICS.md` 7 says r0..r7 are host-accessible "only while thread t is
halted", and HALTED[t] is a specific register that only the `HALT`
instruction sets. A thread that has never run has `RUN = 0` and
`HALTED = 0`.

**Reading taken (director's clarification 4):** thread t counts as halted iff
`RUN[t] == 0`, `STEP_REQ[t] == 0` and no valid slot of thread t is in F, D, X
or W. `HALTED[t]` is not consulted. `docs/HOST_PROTOCOL.md`'s "writable only
while that thread is not running" is applied to the whole DEBUG space, not
only to r0..r7.

## 12. `WAITE` on an output pin index

`docs/SEMANTICS.md` 6.4: for `WAITE`, "`e == 3` and `pin > 12` are never
true", while section 3 defines `pin_in(i)` for 16..21 as the driven value.

**Reading taken:** literal. `WAITE` on index 13..31 never fires, even on
16..21 where the value is well defined and could change. `WAITP` and `JP` do
see those indices. The asymmetry is deliberate in the text, so the RTL keeps
it, but it is the kind of thing firmware will trip over.

## 13. `OUT`/`IN` with a count field of 0

`docs/SEMANTICS.md` 6.3 says `cnt = min(OUTGRP[9:5], 16)`; `ARCHITECTURE`
5.1 says "cnt 1..16". The field is 5 bits, so 0 is representable.

**Reading taken:** literal `min()`. `cnt = 0` writes no pins (`OUT`) and
produces 0 (`IN`). Counts of 17..31 are clamped to 16, for `INGRP` as well as
`OUTGRP` (director's clarification 8).

## 14. Timing of firmware-driven edges (corrected during implementation)

The original task brief asked for "consecutive SETP toggles separated by
`WAITD k` exactly `k*period` clocks apart ... for integer and fractional TICK
settings". That cannot hold in general: a wait completes in the first X cycle
at or after the deadline, and a thread has an X cycle only every 4 clocks.

**Reading taken (director's clarification 1):** exact when `k * period` is a
multiple of 4 clocks; otherwise each edge is 0..3 clocks late with no
accumulation. The RTL already behaved this way; only the test expectations
were written to match. `test/test_timing.py` asserts both cases, and
`test/test_uart.py` asserts that the 10-bit UART frame spacing at
`TICK_INT = 434` is exactly 4340 clocks because 4340 is a multiple of 4.

## 15. `TICK_SEEN` is built but unreadable at M1

`docs/SEMANTICS.md` 4 requires TICK_SEEN to be maintained, and the only
consumer is `WAITB c = 3`, which is M2. The RTL maintains it (4 flops) so
that M2 is a pure addition, but no test can observe it and no debug address
exposes it. A debug address for TICK_SEEN would make it testable at M1; say
the word and it is two lines.

## 16. `STEPS` counts stalled slots

`docs/SEMANTICS.md` 5: "+1 at the commit of every valid slot, done or
stalled". Implemented literally, so a thread parked in a `WAITS` increments
STEPS once every 4 clocks. That is what makes single-stepping observably
identical to free running, but it also means STEPS is not an instruction
count.
