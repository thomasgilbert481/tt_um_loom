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

**Resolution (director, 2026-09-18):** accepted. Written into SEMANTICS 6.7 and HOST_PROTOCOL SPACE 3.

## 2. FIFO space addresses in multi-word transactions

HOST_PROTOCOL SPACE 3: "Multi-word transactions push or pop consecutive
words into the same FIFO (the address does not increment across thread
boundaries in this space; the low two bits stay fixed)."

**Reading taken:** the address does not auto-increment at all in SPACE 3
(so a multi-word read of 0x0100+t re-reads the status word). Addresses other
than 0x0000..0x0003 and 0x0100..0x0103 read 0 and ignore writes.

**Resolution (director, 2026-09-18):** accepted; HOST_PROTOCOL SPACE 3 now says the address never increments there.

## 3. Status word count fields and FIFO_DEPTH

The status word has 4-bit count fields; CAPS allows `log2(FIFO_DEPTH)` up to
7. A count of 16 does not fit.

**Reading taken:** `FIFO_DEPTH` is a power of two from 2 to 8 in this RTL
(the default 4). Debug 0x26 has byte-wide counts and would cope with more.

**Resolution (director, 2026-09-18):** accepted. FIFO_DEPTH is a power of two from 2 to 8; SEMANTICS 6.7 says so.

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

**Resolution (director, 2026-09-18):** a real spec bug. The fix suggested here is adopted as the rule (SEMANTICS 4), but it is built in ONE coordinated commit for RTL and golden model after the current parallel work lands, because co-simulation compares TICK_SEEN. Until then both sides keep the literal rule.

## 5. Debug 0x24 to 0x26 writability

HOST_PROTOCOL SPACE 4: "Every other debug register is a plain flop and is
readable at any time, and writable only while the thread is not running."

**Reading taken:** 0x24 (TICK_SEEN) and 0x25 (the staged pin write) are
writable while the thread is halted; 0x26 (FIFO counts) is read-only,
because a count written without moving the FIFO pointers would expose
entries that were never pushed. The status word, CTRL.RESET and the
push/pop paths are the only ways to change the counts.

**Resolution (director, 2026-09-18):** accepted; HOST_PROTOCOL SPACE 4 records 0x26 as read-only.

## 6. IRQ_EN2 width

CTRL 0x1C IRQ_EN2: "mask over IRQ_STAT2", which has 4 meaningful bits.

**Reading taken:** IRQ_EN2 keeps bits 3:0; bits 15:4 read 0 and ignore
writes.

**Resolution (director, 2026-09-18):** accepted; HOST_PROTOCOL CTRL 0x1C says 4 bits.

## 7. The edge at which a host write commits

`docs/HOST_PROTOCOL.md`, transaction format: "Writes take effect at the end
of each complete word (the falling SCK edge of its last bit, as seen in the
core clock domain)." SEMANTICS 6.7 and 6.8 and the IRQ tests need the exact
edge.

**What the RTL does (measured, not inferred).** Let E be the first rising
clock edge at which the first synchroniser flop samples HOST_SCK high for
the last (16th) bit of a word. The second flop has it at E+1, the edge
detector fires in cycle E+1, `loom_spi_host` raises `byte_done` at E+2,
`loom_host_ctl` registers a one-clock pulse at E+3, and the target register
loads at **edge E+4**; the effect is visible from cycle E+4 on. A scratch
probe that samples the pads and the target registers every cycle shows
E+4 for all twenty paths it tried: IRQ_EN, IRQ_EN2, SFLAGS, SFLAGS_CLR,
RESET_PC, PIN_OUT and PIN_OE (on the pads), OD_MASK, RUN, RESET (PC), STEP,
BADOP clear, SWIRQ clear, an IMEM write, DEBUG writes of TD and of r0, an
INQ push, OUTQ pops (the count, for two successive read words) and BADOP[14]
from a pop of an empty OUTQ. The falling SCK edge plays no part: with the
protocol's minimum SCK (8 clocks, high for 4) E+4 happens to be the edge at
which the first flop samples SCK low again, which is probably where the
sentence comes from, but with any slower SCK the write commits long before
the falling edge.

One path did not follow the rule and was fixed: IRQ_EN and IRQ_EN2, which
`loom_host_ctl` keeps itself, were loaded straight from `byte_done` and
committed at E+3, one edge before everything else. That is why
`test/test_irq.py` saw HOST_IRQ move one clock early after an IRQ_EN write
(and only then: SFLAGS, SWIRQ and FIFO causes were already right). They now
load through a one-clock write strobe like the others, at E+4, and HOST_IRQ
changes at E+5, one edge after the cause is visible (6.8).

The one exception to E+4 is a DEBUG write of r0..r7: it needs the register
file's write port, which a slot of another thread may use in cycle E+3, so
it commits at E+4 or up to three edges later (the target thread's own W
slots are bubbles, so the wait never exceeds three). The thread is halted,
so it cannot observe the difference, and the host cannot issue another
access within 100 clocks.

**Reading taken:** keep the RTL (the word is complete at its last rising
SCK edge in mode 0; committing on the falling edge would hold every write
path pending for another edge detection and change the timing of every
host action for no gain). `test/spi_host.py` `PadMonitor.host_commit()`
computes E+4 in monitor entries (a pad level first seen in entry c is taken
by the first flop at E = c + 1 in this testbench, so the commit is entry
c + 5), and `test/test_irq.py` checks HOST_IRQ against it.

**Proposed replacement sentence for HOST_PROTOCOL** (transaction format):
"Writes take effect at the end of each complete word: if E is the first
rising core clock edge at which HOST_SCK is sampled high for the word's last
bit (by the first flop of the synchroniser), every effect of the word is
registered at edge E + 4 and visible from the cycle after it. This holds for
every write in every space, for the pop of a FIFO read word and for BADOP
bit 14; a DEBUG write of r0..r7 may wait up to three more clocks for the
register-file write port. The falling SCK edge plays no part."

**What another reading would change:** committing at the falling SCK edge
as seen in the core (the literal text) would tie every host effect to the
edge at which the core sees SCK fall after the last bit. That depends on the
host's SCK duty cycle: with the minimum 8-clock SCK it is about the edge the
RTL uses now, with a slower SCK it is later. Nothing in the M2 features
needs that, and co-simulation takes the commit cycles from the RTL either
way (SEMANTICS 10).

**Resolution (director, 2026-09-18):** accepted. The proposed sentence is in HOST_PROTOCOL (transaction format), lightly edited.
