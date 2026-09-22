# Spec questions from the M3 slice A RTL implementation

Written by the RTL implementer while building the bit-engine encoders,
stuffing and differential output from `docs/SEMANTICS.md` 6.9.1 (with 3, 5,
6.3, 6.9 and 7), `docs/HOST_PROTOCOL.md` SPACE 4, `docs/ARCHITECTURE.md` 8
and 8.1 and `docs/DECISIONS.md` D-019, D-025 and D-026, without looking at
`tools/loomsim` (VERIFICATION.md METH-1). Each entry gives the text, what is
ambiguous, the reading the RTL takes (the most literal one unless the text
cannot be built), and what another reading would change.

The text of 6.9.1 is unusually complete: it is ordered, it says which steps
are skipped, and it names the state each rule touches. Most of what follows
is a corner the ordering leaves open rather than a gap.

---

## 1. When `PEND` clears, if the run accounting does not set it

6.9.1 gives two rules that write `PEND`. `SHO` step 1 and `SHI` step 3 set
`PEND <= 0` when the pending stuff bit is consumed. The run accounting says
"`PEND <= 1` when the new run is six 1s ... or five equal bits of either
value". Neither says what happens to `PEND` in the ordinary case.

**Reading taken:** `PEND` holds. It is written only by those two rules, in
that order: the consuming step first, the run accounting second. The two can
never fight, because whenever `PEND` is 1 and the instruction has a bit,
that bit *is* the stuff bit (both branches are guarded by the same
`STUFF != 0 && PEND == 1`), so the consuming step has already cleared it;
and the run accounting on a stuff bit always produces a run of one, which is
neither six 1s nor five equal bits, so a stuff bit never chains into
another. The only case where `PEND` survives an instruction is the
Manchester half that has no bit at all (`SHO` with `HALF == 1`, `SHI` with
`HALF == 0`), which is what the text's "there is none" and "steps 3 and 4
are skipped" require.

**What another reading would change:** nothing reachable. A rule that
cleared `PEND` on every bit would behave identically, because the only bit
that can arrive with `PEND == 1` is the stuff bit that clears it anyway.

## 2. `PEND` and the run state when `STUFF == 0`

6.9.1: "With `STUFF == 0` the run state is not updated and `PEND` stays 0."

`PEND` "stays 0" is true of every state the engine can reach by itself, but
the host can write `PEND = 1` at debug 0x27 (and then clear `STUFF`, or
write 0x27 after the `BE_CFG` write that set `STUFF` to 0).

**Reading taken:** the sentence describes the reachable state, not an extra
clear. With `STUFF == 0` the RTL touches neither `RUN`, `RVAL` nor `PEND`, so
a `PEND` planted through 0x27 stays 1 and does nothing: the stuff branch of
`SHO` step 1 and of `SHI` step 3 is guarded by `STUFF != 0` as well.

**What another reading would change:** forcing `PEND <= 0` on every `SHO` or
`SHI` with `STUFF == 0` would make debug 0x27 unable to hold that bit while
stuffing is off. Nothing in the protocols needs either behaviour; the
literal reading keeps 0x27 a plain register.

## 3. Does a Manchester violation still deliver the bit?

6.9.1, `SHI` step 2: "Manchester, with `HALF == 0`: `FIRST <= p`,
`HALF <= 1` and there is no bit (steps 3 and 4 are skipped), and with
`HALF == 1`: `s = p`, `HALF <= 0`, and `T <= 1` if `FIRST == p`". Only the
`HALF == 0` case is told to skip steps 3 and 4.

**Reading taken:** a second half with no mid-bit transition sets `T` and
still produces `s = p`, which then goes through step 3 (a data bit into
`SR`, `CNT` and the CRC, or a dropped stuff bit) and step 4. A violation
reports itself; it does not swallow the bit.

**What another reading would change:** dropping the bit on a violation would
leave `SR` and `CNT` untouched, so a corrupt bit would not consume a count
and firmware that watches `Z` would lose framing as well as seeing `T`. The
text skips steps 3 and 4 in exactly one place, and this is not it.

## 4. A run that has already passed the stuffing length

6.9.1: "`PEND <= 1` when the new run is six 1s (`STUFF = 1`, USB) or five
equal bits of either value (`STUFF = 2`, CAN)", with `RUN` saturating at
`min(RUN + 1, 7)`.

A transmitter can never exceed the length, because `PEND` forces the stuff
bit first. A receiver can: after six 1s `PEND` is 1, the seventh 1 is taken
as the stuff bit (setting `T`, entry 5), and the run accounting on it gives
`RUN = 7`. An eighth 1 is then an ordinary data bit with `RUN` already
saturated.

**Reading taken:** literal, "is six 1s" and not "is at least six". After a
violation the receiver does not arm another stuff bit until the run breaks,
so one corrupt field costs one `T`, not one per bit. `RUN` saturating at 7
is what makes this stable.

**What another reading would change:** "six or more" would re-arm `PEND` on
every further 1, so a long idle-high line would alternate data bit and
dropped stuff bit. Both readings set `T` on the first violation, which is
what ARCHITECTURE 8.1 wants it for ("so firmware can detect EOP or a corrupt
packet"), and firmware clears `T` and resynchronises either way.

## 5. `DIFF` when one of the two indices is not writable

6.9.1 step 4: "With `DIFF == 1` a second pin write, of `~(l ^ INV)` to index
`(BE_PINS.out + 1) mod 32`, lands at the same edge, as one `OUT` of a
two-pin group would; an index that is not writable is ignored, as in 6.3."

**Reading taken:** the two indices are judged one by one, as `OUT` judges
every index of its group one by one (6.3: "Writes to any other index are
ignored"). So `BE_PINS.out = 7` with `DIFF` writes BIDIR7 and drops the
complement (index 8 is read-only), and `BE_PINS.out = 15` writes nothing to
15 and still drives the complement onto OUT0 (index 16). The open-drain rule
of 6.3 is likewise applied per index, so one of the two pins can be
open-drain and the other not. The implementation gets this for free: both
writes join the same 32-bit mask the group write already builds, and the
commit keeps only bits 0..7 and 16..21.

**What another reading would change:** treating the pair as one unit, so an
unwritable partner suppressed both writes, would make `BE_PINS.out = 15`
silent. Nothing in the text pairs them that way, and D-026 says the second
write goes "through the group-write path", which is the path that ignores
indices one at a time.

## 6. `VERSION` reads 3: which byte?

6.9.1: "`VERSION` reads 3 from slice A on." `docs/HOST_PROTOCOL.md` CTRL
0x0001 is `{major[7:0], minor[7:0]}`, and M1 shipped 0x0001, M2 0x0002.

**Reading taken:** 0x0003, i.e. minor 3 with major 0, continuing the M1 and
M2 convention. `tools/loomhost` prints it as "VERSION 0.3".

**Integration note, not a spec question:** `test/test_host.py` line 23 holds
`VERSION = 0x0002` as a literal and asserts CTRL 0x01 against it. That file
is outside this slice's ownership, so it is left alone; it needs
`VERSION = 0x0003` when slice A is integrated.

## 7. Priority of `CTRL.RESET` against a commit, for the encoder state

SEMANTICS 7 adds the encoder state to what `CTRL.RESET` clears, and says
"If a host write and a thread commit hit the same register bits at the same
edge, the thread wins", and that resetting a running thread is undefined.

**Reading taken:** the M1 order, bit for bit, as `loom_core` already uses for
`PC`, the flags and the deadline latch: the commit of the slot in W is the
last mux before the flop, then a host debug write (0x14 or 0x27), then
`CTRL.RESET`. A `BE_CFG` write clears the encoder state at whichever of the
two ports it arrives on.

## 8. `Z` when `CNT` does not change

6.9.1: "set `Z = (CNT == 0)` after the instruction whether or not `CNT`
changed", against 6.9's "`Z = (new CNT == 0)`".

**Reading taken:** 6.9.1 wins for `SHO` and `SHI`: a stuff bit and a
Manchester half leave `CNT` alone and `Z` reports that unchanged value. With
`ENC = 0` and `STUFF = 0` every `SHO` and `SHI` is a data bit, so the two
sentences agree and the M2 behaviour is unchanged, which is what
co-simulation needs until the generator sets the new fields.

## 9. Reserved `ENC` and `STUFF` read back as 0

6.9.1 says the reserved value 3 of `ENC` and of `STUFF` "is stored as 0", and
D-026 leaves `BE_CFG` readable as the register it is.

**Reading taken:** stored as 0 means read back as 0 too: `CSRR BE_CFG` and
debug 0x14 report the stored bits, so a write of 3 to either field reads 0
and the engine behaves as NRZ / no stuffing. Firmware can therefore probe
which encodings a build supports by writing a value and reading it back, the
same convention the unbuilt bits already use.

---

## Rulings (Fable, 2026-09-22, at integration)

All nine readings stand, and every one of them matches the reading the
golden model's author took independently (`loomsim-m3a.md`): a violation
still delivers the bit (3), the stuff trigger is an exact run length with
`RUN` saturating at 7 (4), `DIFF`'s two indices are judged one at a time (5),
`PEND` is a plain flop that only the rules, a `BE_CFG` write and `CTRL.RESET`
change (1, 2), `Z` reports `CNT` after the instruction (8), reserved field
values read back as 0 (9), `VERSION` is 0x0003 (6), and the M1 priority
order holds for the encoder state (7). Two things were added at integration
rather than asked: `CAPS[9]` reports slice A (SEMANTICS 5), and the test
harness reads the packed `BE_CFG` at whichever width the RTL has. The
co-simulation, with the generator driving `ENC`, `STUFF` and `DIFF`, is the
check that the two sides read 6.9.1 the same way; it found no divergence.
