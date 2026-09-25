# Spec questions from the M4 firmware (manchester_loopback)

Collected while writing `firmware/manchester_loopback.loom`, its reference
model `tools/protomodels/manchester.py` and its L3 test
`tools/tests/test_fw_manchester.py`, 2026-09-24. Each item says what the code
does today. Nothing here needed an RTL or a golden-model change.

## 1. The manual-mode Manchester rate, and what a word boundary costs there

SEMANTICS 6.9.1 gives the loop, `SHO; WAITD 1; SHO; WAITD 1; BNZ` at a
half-bit tick. Its longest interval is `WAITD`, `SHx`, `BNZ`: three slots,
so the deadline checker proves nothing faster than 12 clocks per half-bit,
24 clocks per bit, 2.083 Mbit/s at 50 MHz. The same holds for the receiver
with `SHI`. At that tick every other half-bit interval has exactly one spare
slot, and the loop has none.

**Code today:** each word is the 6.9.1 loop for twelve (transmit) or ten
(receive) of its bits and straight-line code for the rest, where each
half-bit carries one instruction of the word-boundary work: load the next
word, move the word registers, reload `CNT`, count words, and on the receive
side store the word and test `T` twice. Both threads prove every pair at
TICK 12 with a worst slack of 0 clocks. `LD` and `ST` take two slots
(6.11), so neither fits one of those intervals: the words of a frame live in
registers, which caps a frame at four data words. The FIFOs (four deep, a
host word in about 200 clocks) cannot feed or drain a frame in flight either.

**Fix wanted:** none for M4. If longer frames are wanted, the choices are a
slower tick (TICK 16 gives two spare slots per half-bit, enough for an `ST`)
or slice C's auto mode, which D-029 left out of the Manchester target.

**Ruled (director, 2026-09-24): recorded; this is L3-MANCH's rate.** 2.083
Mbit/s is the manual-mode Manchester rate D-029 asked for, and a four-word
frame is enough to show the loopback. Nothing changes.

## 2. `HALF` is invisible to the thread

A receiver must pair the two samples of each bit correctly, and the pairing
lives only in the engine's `HALF` (6.9.1), which the host reads at debug
0x27 and the thread cannot read at all. A receive that stops in the middle
of a bit (the program abandons a frame after a violation right after a
first-half `SHI`) leaves `HALF = 1`, and the next frame's first `SHI` would be
taken as a second half.

**Code today:** thread 1 writes `BE_CFG` before it waits for each frame,
which clears `HALF` (and `FIRST`), so the first `SHI` after the start edge is
always a first half. This is the idiom item 14 of `firmware-m3.md` rules on
for `PEND`, and it works on both backends.

**Fix wanted:** a line in 6.9.1 that a Manchester receiver rewrites `BE_CFG`
before a frame to fix the half-bit phase, as it clears `T`.

**Ruled (director, 2026-09-24): the line is added** to 6.9.1, beside the one
item 15 of `firmware-m3.md` added about `T`.

## 3. `CNT` moves on the first `SHO` of a bit but on the second `SHI`

Both follow from 6.9.1 (a data bit is one that enters or leaves `SR`: the
transmitter takes it at the first half, the receiver gets it at the second),
and `Z` is recomputed by both halves. So the same `CNT` counts the same bits
in both directions, but a loop that exits on `Z` leaves the transmitter
between the halves of its last bit and the receiver after it.

**Code today:** both threads account for it where they reload `CNT`, and the
transmitter loads the next word into `SR` right after the first half of a
word's last bit, when `SR` has given that bit up and `FIRST` holds it.

**Fix wanted:** nothing; perhaps a sentence in 6.9.1's loops paragraph.

**Ruled (director, 2026-09-24): the sentence is added** to 6.9.1's loops
paragraph.

## 4. A `SETD` right after a tick restart has less than its ticks

The checker credits `SETD m` with `m` whole ticks. After `CSRW TICK_INT`
restarts the tick generator (SEMANTICS 4), the `SETD` in the next slot
executes 2 clocks after the restart edge, so its first tick is `P - 2`
clocks away, not `P`: the entry pair of thread 1 (`SETD 1` to the first
`WAITD 1`, five slots) has 22 clocks at TICK 12, not the 24 the listing
shows. It fits either way, with 2 clocks of real slack, not 4.

**Code today:** the program header says so, and the pair has been counted by
hand. In general any `SETD m` can execute anywhere inside a tick, so the
checker's `(m + k) * P` budget is up to `P - 1` clocks optimistic for the
first interval after it; after a restart the error is known (2 clocks).

**Fix wanted:** a known-limit line in `tools/loomasm/README.md` section 7,
and perhaps a checker option to count a `SETD` interval as `(m + k - 1) * P
+ 1` clocks.

**Ruled (director, 2026-09-24): more than a known limit; the checker is
unsound here and is fixed.** A `SETD` reads `NOW` up to `P - 1` clocks after
the tick that set it, so the first interval after `SETD m` can be as short as
`(m + k - 1) * P + 1` clocks, and the checker's `(m + k) * P` is optimistic,
contrary to its README ("a real deadline miss ... is never missed").
`WAITD` anchors are exact: both ends of their interval are on the thread's
slot grid, and `floor(k * P / 4) = ceil((k * P - 3) / 4)`. Measured against
the sound bound, `ceil(((m + k - 1) * P + 1) / 4)` slots, 15 `SETD` pairs in 8
programs fail today (most are `SETD 0` followed by a first `WAITD` that
expects a whole tick); the phase-aware form the checker now has
(`--sound-setd`), which knows where a `SETD` that closely follows a `WAITD`
or a tick restart runs, leaves 11 pairs in 6 programs, this one not among
them. The checker takes the sound bound, and each failing
pair is either given its whole tick in the program (`SETD m + 1`, at most one
tick more latency at the start of a sequence) or shown harmless by an
argument the listing prints, the way `.bounded` is. Recorded as tools finding
T-1 in `docs/VERIFICATION.md`; `docs/PLAN.md` M4 has the work.

## 5. The end marker and a violation in the first bit of a word look alike

The frame ends with the line held low, so the bit after the last word has
no mid-bit transition: the same `T` that a damaged bit sets. The receiver
tests `T` once after a word's last bit (a violation inside the word) and once
after the next bit (the end, or a violation in that bit).

**Code today:** after the second test the receiver waits for an edge with a
timeout of three half-bits (`WAITE RX, ANY, T`): none means the line went
idle and the frame ended; an edge means the frame goes on, so that bit was a
violation. The test injects a violation exactly there and checks that it is
reported as one, with the words before it.

**Fix wanted:** none; this is a property of the chosen frame format.

**Ruled (director, 2026-09-24): recorded; the frame format stands.**
