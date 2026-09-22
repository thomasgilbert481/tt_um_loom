# Spec questions raised while writing slice A in `tools/loomsim`

Opus 5, 2026-09-22, written against `docs/SEMANTICS.md` 6.9.1 (M3 slice A,
D-026), `docs/HOST_PROTOCOL.md` 0.2 (space 4, register 0x27) and
`isa/isa.yaml` 0.4.0 while adding the encoders, the stuffers and the
differential output to the golden model. Written from the text alone; the
RTL (`src/`) and the cocotb tests were not opened (`docs/VERIFICATION.md`,
METH-1).

Same format as the M2 update in `docs/spec-questions/loomsim.md`: each item
is a place where the text does not decide the answer, the model implements
the reading marked **chosen**, and none is ruled on yet. Every item is
pinned by a test in `tools/tests/test_loomsim_be_enc.py`.

Ordered by how likely they are to bite.

---

## 1. Is slice A a build option of its own, or is it what the bit engine now is?

The model builds optional features by name (`Machine(features=...)`), and it
still models the M1 core with none of them. Slice A could either change what
`"BE"` means or be a feature beside it. The text points both ways: 6.9 says
the ``BE_CFG`` bits slice A adds "read 0 and ignore writes until M3 builds
them, so firmware can detect what the engine supports", and HOST_PROTOCOL
says debug 0x27 "reads 0 until slice A is built", both of which describe a
chip that has the engine without the encoders; but no `CAPS` bit tells the
two apart, so from the host side the only difference is `VERSION` and the
`BE_CFG` readback.

**Chosen:** a new optional feature `"BEENC"`, which needs `"BE"` and raises
`LoomsimError` without it. `Machine(features={"BE"})` is still exactly the M2
engine of 6.9, `Machine(features={"BE", "BEENC"})` is the slice-A engine, and
in an M2 build the three new `BE_CFG` fields read 0 and ignore writes, debug
0x27 reads 0 and `SHO`/`SHI` behave as they did. Reasons: it keeps the M2
chip co-simulatable after slice A lands (the M2 RTL exists and will be
regressed against the model), it is the same shape as the M1 build the model
already supports, and it left all 1,383 M2-era tests passing unchanged.

**What this costs at integration:** a co-simulation run against slice-A RTL
must build the model with `"BEENC"` in its feature list, and
`tools/loomgen`'s own `FEATURES` set (`FIFO`, `BE`, `SETPD`) has to learn the
name before it can generate the new `BE_CFG` bits. That is the generator
update D-026's plan already assigns to the director, and it is one word. If
the director would rather have `"BE"` mean the slice-A engine from now on,
the change is `self._be_enc = self._be` in `Machine.__init__` plus the three
M2 `BE_CFG` tests in `test_loomsim_be.py`, and this item becomes moot.

## 2. Which builds read `VERSION == 3`

6.9.1 says "`VERSION` reads 3 from slice A on" and 6.11 says that with slice B
"`VERSION` advances", so the register tracks what is built; the M2 update of
`loomsim.md` (item 1) had it as `{major, minor}` of the host protocol, 0x0002
for HOST_PROTOCOL 0.2. Read as a bare constant, an M1 build would read 3 as
well, which contradicts the existing test that pins the featureless build at
0x0002.

**Chosen:** `VERSION` is a build constant: 3 (`H.ENC_VERSION`) when the
encoders are built, 2 otherwise, still overridable with `Machine(version=...)`
for a host library that wants to test its own compatibility logic. The two
readings agree on the only chip that will be taped out.

## 3. Does a run longer than the trigger keep a stuff bit due?

"`PEND <= 1` when the new run is six 1s (`STUFF = 1`, USB) or five equal bits
of either value (`STUFF = 2`, CAN)", and `RUN` saturates at 7. Read as `==`,
a run that is somehow already past the trigger never sets `PEND` again; read
as `>=`, it sets it on every further bit.

**Chosen:** `==`, literally. In normal operation the two cannot differ: the
state resets to 0, is cleared by every `BE_CFG` write, and the stuff bit
starts the next run, so a run never passes the trigger. They differ only from
a state the host wrote at debug 0x27 (or after `STUFF` is turned on mid-stream
with a `RUN` that a shorter trigger already passed, which the `BE_CFG` write
itself prevents). Cheap either way in RTL, so worth agreeing on.

Related, same item: **`PEND` is written only where the text writes it** --
set to 0 when the stuff bit is consumed and to 1 when the new run triggers,
left alone otherwise. It is never forced to 0 by run accounting that does not
trigger, because at that point it has always just been consumed or was never
set.

## 4. Which bit the CRC sees on `SHI` when an encoder is on

6.9 says the CRC is updated with `s`, where `s = pin_in(BE_PINS.in) ^ INV`.
6.9.1 keeps the name but redefines it: step 1 samples `p`, step 2 decodes `p`
into "the bit received, `s`", and step 3 says "otherwise `s` is a data bit and
`SR`, `CNT` and the CRC follow 6.9". So under NRZI the CRC could take the
decoded bit or the line level.

**Chosen:** the decoded bit, the one that enters `SR`. Anything else would
make `STCRC` disagree with the data the firmware just received, and the same
reading makes `SHO` feed the CRC with the data bit rather than the level it
put on the pin, which 6.9.1's step order already fixes (step 1 is the bit,
step 3 is the level). A stuff bit never reaches the CRC on either side, which
the text says outright.

## 5. Does a `BE_CFG` write of the value already there clear the encoder state?

"cleared by every write to `BE_CFG` (by `CSRW` or by the host)". A write that
changes nothing is still a write.

**Chosen:** literal -- the clear follows the write strobe, not a change in
value. An RTL that clears on `csr_we && csr == BE_CFG` does the same thing for
free; one that compares old and new would differ, and firmware that reloads
the same configuration to resynchronise an encoder (the cheapest way to force
`LVL`, `HALF` and the run count to a known state mid-frame) depends on it.

## 6. The encoder state with `STUFF == 0`, and what the host may leave in it

"With `STUFF == 0` the run state is not updated and `PEND` stays 0". Read as
a constraint on the register, `PEND` would be forced to 0; read as a
description of the engine's updates, a value the host wrote at 0x27 simply
sits there unused.

**Chosen:** the second. The six bits are plain flops that only the rules of
6.9.1, a `BE_CFG` write and `CTRL.RESET` change, so a debugger can save and
restore a thread's whole state (the same reading as M2 item 9 for the
deadline latch at 0x25). With `STUFF == 0` nothing reads `PEND`, so the two
readings are observably the same until `STUFF` is turned on -- and that is a
`BE_CFG` write, which clears the state.

## 7. Debug 0x27 in a build without the encoders, and the range of a dump

HOST_PROTOCOL says 0x27 "reads 0 until slice A is built; writable while
halted". The M2 update (item 4) had already chosen that 0x27..0xFF read 0 and
ignore writes, and one existing test pins that for the M1 build.

**Chosen:** in a build without `"BEENC"` the register reads 0 and ignores
writes, exactly as an unbuilt feature's registers do everywhere else; with it,
it reads and writes `{8'b0, FIRST, HALF, PEND, RVAL, RUN[2:0], LVL}` and bits
15:8 read 0 and ignore writes. `Machine.dump_debug_space` therefore covers
0x00..0x26 in an M1 or M2 build and 0x00..0x27 in a slice-A build, rather
than always showing a register that does not exist.

## 8. Two host writes in one cycle, one of them `BE_CFG`

The host can write 0x27 and `BE_CFG` in the same cycle; both commit at the
same edge, and a `BE_CFG` write clears the encoder state.

**Chosen:** host commits apply in the order the calls were made (the model's
existing rule), so the later call wins: `BE_CFG` then 0x27 leaves the written
state, 0x27 then `BE_CFG` leaves it cleared. No new rule, but worth writing
down because a debugger restoring a thread must write `BE_CFG` first.

---

## Consequences worth stating rather than questions

* **`LVL` holds the level before `INV`.** 6.9.1 encodes into `l`, commits
  `LVL <= l` and only then writes `l ^ INV` to the pin; on `SHI` it samples
  `p = pin_in ^ INV` and commits `LVL <= p`. So `INV` lives at the pin, the
  NRZI state is in the un-inverted domain, and a transmitter and a receiver
  with the same `INV` round-trip. Debug 0x27 shows the un-inverted level.

* **The stuff bit goes through the encoder.** Step 1 picks it, step 2 counts
  it and step 3 encodes it, so under NRZI a USB stuff bit (a 0) toggles the
  line exactly as a data 0 does, which is what USB requires; under Manchester
  a stuff bit is a whole bit, two `SHO`.

* **`Z` is written by every `SHO` and `SHI`, including the ones that change
  nothing.** The second half of a Manchester `SHO`, the first half of a
  Manchester `SHI` and every stuff bit still set `Z = (CNT == 0)`. It is
  observable: another instruction may have set `Z` since the previous shift.

* **A Manchester half that carries no bit consumes no stuff bit.** With
  `ENC = 2` a pending stuff bit waits for the next first half (`SHO`), and on
  `SHI` the stuff check and the run accounting happen on the second half,
  where the bit arrives.

* **`T` can be set twice in one `SHI`.** A Manchester bit with no mid-bit
  transition that is also a stuffing violation sets `T` in step 2 and again
  in step 3; `SHI` never clears it, so firmware clears it with `CSRW FLAGS`
  or lets a timed wait clear it.

* **A bad stuff bit is still dropped.** On a stuffing violation `SHI` sets
  `T`, and the bit does not enter `SR`, does not move `CNT` and does not
  reach the CRC: the receiver stays aligned with the sender's bit count and
  the firmware sees the error in `T`.

* **`DIFF` is one commit, not two.** Both pin writes are in the slot's commit
  and land at the same edge with `OD_MASK` as visible in the X cycle, so a
  staged deadline write to either index is beaten by them (M2 item 6), an
  index that is not writable is dropped on its own, and the pair wraps:
  `BE_PINS.out = 31` writes nothing for the level and index 0 for the
  complement.

* **`CAPS` does not move.** Slice A has no capability bit: `CAPS[4]` was
  already "bit engine built (manual mode)" and `CAPS[8]` stays 0 until auto
  mode. Firmware detects slice A by `VERSION` or by writing `BE_CFG` and
  reading the `ENC`/`STUFF`/`DIFF` fields back, which is what 6.9's sentence
  about detecting "what the engine supports" promises.

---

## Rulings (Fable, 2026-09-22, at integration)

All eight readings stand, with one addition. Item 1: the build split is
right, and it gets the capability bit it lacked: `CAPS[9]` is "bit-engine
encoders, stuffing and DIFF built" (SEMANTICS 5 and 6.9.1 updated), so the
co-simulation harness, which builds the model from `CTRL.CAPS`, maps bit 9
to `"BEENC"` and never has to read `VERSION`; firmware may use either. Items
3 (the trigger is an exact run length), 4 (the CRC takes the decoded bit) and
5 (any `BE_CFG` write clears the encoder state) are the intended readings and
the RTL is held to them at integration; SEMANTICS 6.9.1 will say "exactly" and
"the data bit" in so many words if the RTL's author read them differently.
Items 2, 6, 7 and 8 are consequences of rules that already exist and need no
text. Of the consequences, the wrap of `DIFF` at index 31 to index 0 is what
the text says and stays; nothing sensible puts D+ on a read-only index.
