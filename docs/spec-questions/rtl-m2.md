# Spec questions from the M2 RTL implementation

Written by the RTL implementer while building the M2 features from
`docs/SEMANTICS.md` (6.7 to 6.10, 7), `docs/HOST_PROTOCOL.md`,
`docs/DECISIONS.md` D-016 and D-019 and `isa/isa.yaml` 0.4.0, without
looking at `tools/loomsim` (VERIFICATION.md METH-1). Each entry gives the
text, what is ambiguous or wrong, the reading the RTL takes (the most literal
one unless the text cannot be built), and what another reading would change.

---

## 1. Host pops and the SPI read prefetch

`docs/SEMANTICS.md` 6.7: a host pop "returns the head iff `OUTQ_CNT[t] > 0`
as visible in the cycle before the commit edge and removes it".
`docs/HOST_PROTOCOL.md`: writes "take effect at the end of each complete
word"; a read transaction may carry any number of words.

The SPI slave has to present the first bit of a word on the falling SCK edge
right after the previous byte, before it can know whether the host will
clock that word out at all. If the pop committed when the word is loaded, as
the M1 read path does for every other space (it even fetches one word
ahead), every read transaction would pop one entry more than the host
receives, and that entry would be lost.

**Reading taken:** the word is taken from the head when it is loaded (a peek,
at the end of the dummy byte or of the previous word), and the pop commits
at the end of the word, once all 16 bits went out. A word cut short by CS_n
pops nothing (the same rule as for writes). The next word, loaded at the
edge where the previous one is popped, is the entry after the head, and
needs `OUTQ_CNT >= 2` in that cycle. The "empty" decision (returns 0,
removes nothing, BADOP[14]) is therefore made at load time; BADOP[14] is set
at the end of the word. The only observable difference from the literal text
is a thread PUSH that lands while an "empty" word is being shifted out: that
word still reads 0 and the pushed entry stays for the next read. Nothing is
ever lost or duplicated.

## 2. FIFO space addresses in multi-word transactions

HOST_PROTOCOL SPACE 3: "Multi-word transactions push or pop consecutive
words into the same FIFO (the address does not increment across thread
boundaries in this space; the low two bits stay fixed)."

**Reading taken:** the address does not auto-increment at all in SPACE 3
(so a multi-word read of 0x0100+t re-reads the status word). Addresses other
than 0x0000..0x0003 and 0x0100..0x0103 read 0 and ignore writes.

## 3. Status word count fields and FIFO_DEPTH

The status word has 4-bit count fields; CAPS allows `log2(FIFO_DEPTH)` up to
7. A count of 16 does not fit.

**Reading taken:** `FIFO_DEPTH` is a power of two from 2 to 8 in this RTL
(the default 4). Debug 0x26 has byte-wide counts and would cope with more.

## 4. WAITB 3 loses a tick that lands between a slot's X cycle and its commit

`docs/SEMANTICS.md` 4: "`TICK_SEEN` is cleared at the commit edge of every
valid slot of the thread (unless a tick sets it at the same edge)"; 6.7:
"3 is `TICK_SEEN[t]`"; ISA: "3 a tick occurred since the previous slot".

A slot with X cycle x commits at edge x + 2. A tick at edge x + 1 sets
TICK_SEEN after the slot looked at it and is cleared again by the slot's own
commit at x + 2, so no slot ever sees it. With a tick period that is a
multiple of 4 clocks and ticks at that phase, `WAITB 3` never completes; with
other periods, every fourth phase is dropped (at a period of 41 clocks a loop
of `WAITB 3; SETP` shows gaps of 40, 84, 40, 40, 84, ...).

**Reading taken:** literal (`test/test_fifo.py` checks the RTL against a
model of exactly this rule). Suggested fix for the next SEMANTICS revision:
clear only what the slot saw, i.e. `TICK_SEEN <= tick | (TICK_SEEN & ~seen_in_X)`
at the commit edge, which makes "a tick since the previous slot" true.

## 5. Debug 0x24 to 0x26 writability

HOST_PROTOCOL SPACE 4: "Every other debug register is a plain flop and is
readable at any time, and writable only while the thread is not running."

**Reading taken:** 0x24 (TICK_SEEN) and 0x25 (the staged pin write) are
writable while the thread is halted; 0x26 (FIFO counts) is read-only,
because a count written without moving the FIFO pointers would expose
entries that were never pushed. The status word, CTRL.RESET and the
push/pop paths are the only ways to change the counts.

## 6. IRQ_EN2 width

CTRL 0x1C IRQ_EN2: "mask over IRQ_STAT2", which has 4 meaningful bits.

**Reading taken:** IRQ_EN2 keeps bits 3:0; bits 15:4 read 0 and ignore
writes.
