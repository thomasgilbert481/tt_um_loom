# Spec questions from the M3 firmware (ws2812, ps2_host, jtag_master, swd_master; usb_ls_device; i2c_slave_eeprom, can_loopback)

Collected while writing `firmware/ws2812.loom`, `firmware/ps2_host.loom`,
`firmware/jtag_master.loom` and `firmware/swd_master.loom` and their L3
tests, 2026-09-22. Each item says what the code does today.
Nothing here needed an RTL or a golden-model change: the two implementations
agreed on every scenario these programs exercise.

## 1. The deadline checker cannot see past a `POP`, and a continuous waveform fed from a FIFO needs one

`tools/loomasm/README.md` 7 lists `POP` under "Unbounded": a path between two
deadlines that contains one has no static bound, and the cure it offers is
"re-anchoring with `SETD` right after such an instruction".

A WS2812B frame is a continuous byte stream: every low phase inside it is
0.85 or 0.45 us +-0.15, so the next byte has to be fetched *between* two
deadlines, in the 44-clock low phase of a 0 bit or the 40-clock high phase of
a 1 bit. The suggested cure does not apply there:

* `SETD m` replaces the absolute deadline that the next edge is latched on
  (SEMANTICS 6.10), so the waveform would lose the edge the program exists to
  place exactly;
* putting the deadline back with `CSRR TD` / `CSRW TD` around the `SETD`
  costs three more slots, which a 44-clock (eleven-slot) low phase whose
  fetch already needs ten does not have. It would also make the proof
  vacuous: the checker would prove the seven slots after the `SETD` and stop
  looking at the six before it, so a real 52-clock path would pass.

So `firmware/ws2812.loom` assembles with `--strict` and exit 0, but with five
`UNBOUNDED` warnings rather than none. It is not the "unbounded is fine, it
is re-anchored" case the README describes: these are the intervals that carry
the waveform. The program's header bounds them by hand, and
`tools/tests/test_fw_ws2812.py` measures every pulse on both backends, which
is the real check. The `POP` provably cannot stall: it is reached only when
the `WAITB INQ_NE, T` one slot earlier completed with `T = 0`, so `INQ` held a
word in that slot's X cycle, and between then and the `POP` nothing can remove
it (only this thread pops `INQ`; the host only pushes, SEMANTICS 6.7).

**Code today (superseded by the ruling below):** the program kept the exact
schedule and the five warnings, and `test_fw_ws2812.py` asserted that exact
diagnostic set so a regression in either direction would show up.

**Fix wanted (a question for the director, not a change made here):** one of

1. let the checker treat `POP` as one slot when it is dominated by a
   `WAITB INQ_NE, T` whose `T` branch does not reach the same `WAITD`
   (likewise `PUSH` after `WAITB OUTQ_NF, T`) - that is exactly the idiom
   `uart_rx.loom` and `spi_slave.loom` already use to make a blocking
   instruction non-blocking;
2. or a per-instruction opt-out, say a `; loomasm: bounded` comment or a
   `.bounded` directive, so the source says which blocking instruction the
   author has discharged by hand and the checker counts it as one slot;
3. or nothing, and `firmware/README.md` records that the ws2812 row is
   proved by hand for two of its intervals.

**Ruled (coordinator, 2026-09-22): option 2, and it is built.** The source
now states the assumption where a reader of the code meets it, and the
checker's proof stays honest about what it did and did not verify - which
option 1 would not have done, because an inferred exemption is invisible at
the place that matters and would extend itself silently to every future
guard pattern the inference happened to match.

`.bounded "<reason>"` applies to the next emitted word, which must be a
`PUSH` or a `POP`; the reason is mandatory and non-empty. The checker counts
a declared access as one slot instead of unbounded and changes nothing else,
so an undeclared `PUSH` or `POP` is still unbounded. Because a declaration is
an assumption and not a proof, it is printed twice - under the instruction in
the listing and in the thread's summary block, as
`bounded by declaration: 0x019 POP (line 126): <reason>` - and the summary's
last line counts them. `tools/loomasm/README.md` documents it in section 4,
under "Declared bounded" in section 7, and as known limit 1a: the only input
to the analysis that is not derived from the emitted words.

`firmware/ws2812.loom` declares both of its `POP`s. It now assembles with
`--strict` and **no diagnostic at all**: 19 pairs, none unbounded, none
infeasible, worst slack 4 clocks, which is the one slot the two byte-fetch
intervals have (10 slots against a 44-clock budget, 9 against 40). What the
declaration asserts - that the `WAITB INQ_NE, T` one slot earlier completed
with `T = 0`, so `INQ` held a word in that slot's X cycle, and only this
thread pops `INQ` (SEMANTICS 6.7) - is the thing to review, and it is written
at both `POP`s and in the program header. The paragraph above this ruling is
left as it was written, so the trade-off that led here is still on record.

## 2. `SETP pin, v, D` needs a feature the L3 test harness cannot ask for

The golden model builds the deadline latch only with `features={"SETPD"}`
(`tools.loomsim.Machine`), and `tools.protomodels.bench.Bench` defaults to
`features=("FIFO",)`. The RTL has the latch unconditionally
(`src/loom_core.v`), and `test/rtl_bench.py`'s `RtlBench` takes no `features`
argument, so a test body that must run on both backends cannot simply pass
one through `backend.bench(...)`.

**Code today:** `tools/tests/test_fw_ws2812.py` has a three-line `ws_bench`
helper that adds `features=("FIFO", "SETPD")` when `backend.name == "model"`.

**Fix wanted:** either `ModelBackend.bench` defaults to the features the RTL
actually has (`FIFO` and `SETPD` today), or `RtlBench.__init__` accepts and
ignores `features`, so every future body can say what it needs in one place.

## 3. What ends a WS2812 frame is not in any document

`docs/PLAN.md` M3 names the program; nothing says how a frame is delimited.
The wire format has no length: a reset is a low longer than 50 us, and
everything between two resets is one frame.

**Code today:** the frame ends when `INQ` is empty at a byte boundary, the
line stays low, and the program waits 2750 clocks (55 us) before the first
rise of the next frame. Bytes pushed while a frame is running extend it, which
is what a strip longer than one host burst needs, and it is why the host has
to keep `INQ` fed: a byte is 512 clocks on the wire and
`tools.loomhost.Loom.push` needs a 400-clock status read plus
`(3 + 2n) * 64 + 16` clocks to deliver `n` words, which is 280 clocks a word
at `n = 4` and 432 at `n = 2`, so it keeps up and `INQ` settles two or three
deep - but a host that stalls for more than one byte time splits the frame
into two, and the strip will latch the first half.
`tools/protomodels/ws2812.py` makes that visible rather than silent: a low
that is too long for a bit and too short for a reset is a violation.

**Fix wanted:** nothing, unless the director wants a host-visible "frame
underrun" word in `OUTQ`. The program currently pushes nothing at all, so a
host cannot tell a split frame from a clean one without watching the pin.

## 4. PS/2 timeouts are a property of the standard, so the program owns its tick

Every other program leaves `TICK_INT` to the host (`firmware/README.md`), but
`ps2_host.loom` has no bit rate of its own: the device owns the clock and the
only thing the tick sets is how long a missing edge is waited for. With the
reset value `TICK_INT = 1` those timeouts would be 128 and 200 clocks and
every frame would be abandoned.

**Code today:** `ps2_host.loom` and `ws2812.loom` both write their own
`TICK_INT` with `.csr` (64 and 1 clocks respectively) and the README rows say
the host must not set it. `.csr TICK_INT, <constant>` also declares the
period to the deadline checker, so nothing is lost.

**Fix wanted:** nothing; noted because it breaks the M2 convention that the
table in `firmware/README.md` documents.

## 5. A program with no `WAITD` gets no deadline verdict at all

`ps2_host.loom` has `SETD` anchors and timed waits but no `WAITD`, so
`analyse_thread` forms no pairs and the listing says "no deadline pairs
found". That is correct - there is no deadline to prove, only timeouts - but
it means `--strict` is silent about a program that could still be wrong, and
a checklist that reads "every deadline pair proved" is vacuously satisfied.

**Code today:** `tools/tests/test_fw_ps2.py` asserts `report.pairs == []` and
`program.diagnostics == []` deliberately, so a future edit that introduces a
`WAITD` has to be looked at.

**Fix wanted:** nothing, unless the checker should say "no deadlines: this
thread is event driven" in the listing header rather than only in the notes.

## 6. Two levels of `CALL` fit the return stack but not the deadline checker

`swd_master.loom` first had two subroutines, `wbits` (send n bits) calling
`clk` (one SWCLK period). That is a nesting depth of two, exactly what the
return stack holds (`docs/ISA.md`, `CALL`). The checker's limit 3
(`tools/loomasm/README.md` 7: a `RET` flows to every return site in the
thread) then composed paths through both of them that no execution can take
- out of `clk`'s `RET`, on through `wbits`'s loop, out of `wbits`'s `RET`
into a `CALL clk` site - and reported 13 and 14 slots where the longest real
path is 7.

**Code today:** `swd_master.loom` inlines the two send loops and keeps only
`clk` as a subroutine; the report is then the real one (worst 7 slots).
`jtag_master.loom` never nested. The restriction is noted in both program
headers.

**Fix wanted:** nothing urgent, but a note in `tools/loomasm/README.md` 7
that nested `CALL` is analysed context-insensitively in *both* directions
would have saved the detour: limit 3 reads as if the only cost were a
subroutine called from two places.

## 7. Where a master may pause its own clock, the checker needs a `SETD`

In `jtag_master` and `swd_master` the master owns the clock, so a longer low
phase between one packet's last edge and the next one's first is legal -
`spi_master.loom` says so already ("between bytes SCK waits while the master
fetches the next command"). The checker sees only a missed deadline, so
every such point needs a `SETD 0` to re-anchor: in `swd_master` there are
three (before the select sequence, before the ACK loop, before each data
word) on top of the `SETD` after each `POP` and `PUSH`.

**Code today:** the `SETD 0`s are there, each with a comment saying the
clock pauses at that point.

**Fix wanted:** nothing. Noted because it is the second idiom (after the
`POP` of item 1) where the checker's verdict depends on where the author
puts an anchor rather than on what the hardware does, and a reader of
`firmware/README.md`'s "worst slack" column should know that the column
measures the intervals the author chose to bound.

---

Items 8 to 10 were collected while writing `firmware/usb_ls_device.loom`
(slices A and B) and its L3 tests, 2026-09-23. Nothing needed an RTL or
golden-model change.

## 8. usb_ls_device: firmware cannot read the stuffer's `PEND`

USB inserts a stuff bit after six 1s even when the sixth is the last bit
before the EOP (USB 2.0 7.1.9), and a device's data packets end with a
CRC16 that can end in six 1s. SEMANTICS 6.9.1 keeps `PEND` in the encoder
state, readable by the host at debug 0x27 but by no CSR, so the firmware
cannot ask whether a stuff bit is due before it writes the SE0.

**Code today:** a probe. After the last bit, `BE_PINS.out` is pointed at pin
22 (no pad; 23, the `DIFF` partner, is none either), `CNT` is set to 1 and
one `SHO` is executed: a due stuff bit leaves `CNT` at 1 (`Z = 0`), a data
bit empties it (`Z = 1`), and nothing reaches the pads. In the first case
the stuff bit is then put on the pads by hand (`OUT` of the complement of
the line level read before the probe). It relies on two things 6.9.1 says
without dwelling on: writing `BE_PINS` does not clear the encoder state
(only `BE_CFG` writes do), and a stuff bit leaves `CNT` alone while `Z` is
still recomputed. Both backends agree (every data packet the tests check
goes through it).

**Fix wanted:** none needed. If `CAPS`/`BE_CFG` are ever revisited, a
read-only view of `PEND` (and `LVL`) as a CSR would turn eight instructions
and one borrowed pin index into one `CSRR`.

**Ruled (director, 2026-09-23): no hardware change; the probe stays, on
contract.** The RTL freezes on 2026-11-08 and every hardware change costs a
hardening of its own (D-025); the probe costs eight instructions and a pin
index with no pad, and every data packet of the tests goes through it on
both backends. Of the two behaviours it relies on, 6.9.1 already stated the
second (a stuff bit leaves `CNT` alone and `Z` is recomputed either way); the
first is now stated too: the encoder state is cleared by a `BE_CFG` write
and `CTRL.RESET` "and by nothing else (a write of `BE_PINS`, `SR` or `CNT`
leaves it)". Both implementations already behaved so.

## 9. usb_ls_device: the golden model reads never-loaded memory as 0

`txdat` fetches the word after the last one it sends. For the report
descriptor, the last thing in the image, that word was never loaded. The
golden model returns 0 for it and every model test passed; the RTL
testbench (`test/tb.v`, "a valid slot is decoding an unwritten IMEM word")
stopped the simulation on the X of the `LD`'s completion slot, and cocotb
then reported every later scenario of the run as failed with no traceback
of its own. The fix was one padding word after the descriptor.

**Question:** SEMANTICS 6.11 and section 7 leave memory that was never
written unspecified, which is right for silicon. Should the golden model
flag an `LD` of a word the image never loaded (a warning, or a `BADOP`-like
record in the retire trace) so that the model run finds what only the RTL
run finds today? The model already knows which addresses were loaded.

**Ruled (director, 2026-09-23): yes, as a check of the test harness, not a
change of meaning.** Memory nothing wrote stays unspecified in SEMANTICS,
as it must for silicon. The model backend of `tools/tests/fw_backend.py`
should fail a scenario whose program fetches or `LD`s a word the image never
loaded and nothing stored, the check `test/tb.v` makes on the RTL, so the
model run finds it first and names the address. Built 2026-09-24 (b809321):
`Machine.unloaded_reads` and `on_unloaded_read` in `tools/loomsim`, and the
model backend raises `UnloadedReadError` at the read.

## 10. usb_ls_device: needs the whole memory, and one idiom per tight path

The program is 511 of the 512 words (code, 22 data words, three packed
descriptors), so it is the first firmware that cannot share the chip with
another thread. What made it fit: tokens matched whole against fields
precomputed (with the CRC unit) when the address changes, so no CRC5 is
checked at run time; the SETUP words stored with `ST` at a counting address
inside the receive loop; the next EP0 reply kept ready in memory. The
response to an IN leaves the checker's tightest pair at 31 of 41 slots
(`RESP = 6` ticks), and the measured response is 4.56 to 4.74 bit times
(USB allows 2 to 6.5).

**Fix wanted:** none; recorded for the M4 area and memory discussion: USB
low speed as specified here costs the whole memory of D-020.

**Ruled (director, 2026-09-23): recorded, and it answers a question D-029
left open.** USB low speed does not need the bit engine's auto mode: in
manual mode (slice A) the program meets every timing limit it was tested
against (response 4.56 to 4.74 bit times of 2 to 6.5, EOP width, bit time
within 1.5 per cent, source jitter under 53 ns) with a worst checker slack
of one clock. Its cost is memory, not time. That goes to the slice C
decision (2026-11-01, `docs/PLAN.md` M3).

---

Items 11 to 16 were collected while writing `firmware/i2c_slave_eeprom.loom`
(slice B) and `firmware/can_loopback.loom` (slice A) and their L3 tests,
2026-09-23. Again nothing needed an RTL or golden-model change.

## 11. `CRCI` resets the stuffing state in `isa.yaml`, not in SEMANTICS 6.9.1

`isa.yaml` (and so `docs/ISA.md`) gives `CRCI` the semantics "CRC =
CRC_INIT; stuffing state reset". SEMANTICS 6.9.1 says the encoder state
(`LVL`, `RUN`, `RVAL`, `PEND`, `HALF`, `FIRST`) is cleared by a write to
`BE_CFG` and by `CTRL.RESET`, and 6.9 gives `CRCI` only `CRC <= CRC_INIT`.
A CAN or USB program that relies on `CRCI` alone to start a frame with a
clean run count would behave differently under the two texts.

**Code today:** both threads of `can_loopback` write `BE_CFG` at the start
of every frame and then `CRCI`, so the program is right under either text.

**Fix wanted:** one text. SEMANTICS wins by the repo rule, so probably the
`sem` string in `isa.yaml` (and a regenerated `docs/ISA.md`).

**Ruled (director, 2026-09-23): SEMANTICS stands, `isa.yaml` changes.**
`CRCI`'s `sem` string now reads "CRC = CRC_INIT; the encoder and
stuffing state are left alone (6.9.1)" and `docs/ISA.md` is regenerated;
6.9.1 also says now that the encoder state is cleared by a `BE_CFG` write,
`CTRL.RESET` "and by nothing else" (item 8). Both implementations
already behaved so; only the table text was wrong.

## 12. There is no `.crc` directive

SEMANTICS 6.9 says "the assembler's `.crc` presets do that alignment" and the
comment under `crc_presets` in `isa.yaml` says the same, but `tools/loomasm`
has no `.crc` directive and nothing in it reads `crc_presets`.

**Code today:** `can_loopback` writes `.equ POLY = 0x4599 << 1` (the `can15`
preset, left-aligned by hand) and `.csr CRC_POLY, POLY`, and leaves
`CRC_INIT` at its reset value 0, which is the preset's init.

**Fix wanted:** either a `.crc NAME` directive that loads `CRC_POLY` and
`CRC_INIT` from `crc_presets` with the alignment, or the two sentences
changed to say the alignment is the programmer's.

**Ruled (director, 2026-09-23): the text changes.** 6.9 and the comment
under `crc_presets` now say the program does the alignment, from the
canonical values the presets give; there is no `.crc` directive and
nothing needs one yet (`can_loopback`'s `.equ` is one line).

## 13. Rewriting `TICK_INT` to restart the tick on an edge

A CAN receiver must hard-synchronise on the SOF edge: its bit timing starts
there. SEMANTICS 4 says any write to `TICK_INT` or `TICK_FRAC` clears `ACC`,
so a thread can restart its tick generator on an edge by writing back the
value the host gave it. Thread 1 of `can_loopback` does `CSRR r7, TICK_INT`
before `WAITE RX, FALL` and `CSRW TICK_INT, r7` right after it, then
`SETD 5`, which has to commit before the first tick after the rewrite
(12.5 clocks at 500 kbit/s): the checker cannot see that constraint, so the
`SETD` sits in the slot straight after the `CSRW` with a comment.

**Code today:** as above; the samples then fall 5/8 of a bit plus 10 to 16
clocks after each bit start on both backends.

**Fix wanted:** a sentence in SEMANTICS 4 (or ARCHITECTURE) saying this is an
intended use of the `ACC` clear, so that a later change to the tick
generator keeps it. `uart_rx` gets by without it by sampling eight times a
bit; a protocol whose sample point is set by a standard cannot.

**Ruled (director, 2026-09-23): intended, and now said so.** SEMANTICS 4
gains a sentence after the `ACC` clear: writing back the value already
there restarts the thread's tick grid at that edge, which is how a
receiver hard-synchronises to a start bit. `usb_ls_device` does the same
on each packet's first edge. The `SETD` that must commit before the
first tick after the rewrite stays the program's to place.

## 14. A pending stuff bit is invisible to the thread

With `STUFF = 2` a CAN transmitter must send the stuff bit that falls due
after the last bit of the CRC sequence (ISO 11898-1 stuffs SOF to the end of
the CRC). The usual loop (`WAITD 1; SHO; BNZ`) ends when `CNT` reaches 0,
and whether `PEND` is set then is known only to the host (debug 0x27): the
thread cannot read it, and one more `SHO` with `PEND == 0` would send a data
bit from `SR`. The receiver has the mirror problem before the delimiter.

**Code today:** both threads treat the CRC delimiter as a sixteenth data
bit (`SR = CRC | 1`, `CNT = 16`), so the engine puts a due stuff bit before
it on the wire and drops it on the way in. The receiver then checks the CRC
register against the polynomial rather than 0 (the delimiter has gone
through the CRC too), which is exact because `CRC[0]` is always 0.

**Fix wanted:** nothing if this idiom is the intended one; then 6.9.1's
"loops that result" paragraph could say so for CAN. Otherwise `PEND` (or a
"stuff bit due" flag) readable by the thread.

**Ruled (director, 2026-09-23): the idiom is the intended one**, as item 8
rules for USB. 6.9.1's loops paragraph now says so: a frame whose
stuffing runs up to a fixed-form field sends that field's first bit
through the engine as a data bit (CAN's CRC delimiter), or probes for a
due stuff bit where the field is not a data bit (USB's EOP).

## 15. `T` is both the timeout flag and the stuffing-violation flag

`SHI` sets `T` on a stuff or Manchester violation and never clears it; a
`WAITP`/`WAITE`/`WAITS`/`WAITB` with the `T` bit sets it on a timeout and
clears it when the condition wins. A receiver that waits for bus idle with a
timed `WAITP` (as `can_loopback` does) therefore starts its frame with
`T = 1` unless it clears it.

**Code today:** thread 1 writes `FLAGS` to 0 after each SOF, and checks `T`
after every `SHI` (`BT stufferr`), so a timed wait never runs between a
violation and its test.

**Fix wanted:** nothing; a line in 6.9.1 saying "clear `T` before a frame"
would save the next author the search.

**Ruled (director, 2026-09-23): nothing changes in the hardware; the line
is added.** 6.9.1 now says that the timed waits of 6.4 set `T` on a
timeout too, so a receiver clears `T` before a frame.

## 16. A thread section may run into the next thread's quarter

The assembler rejects overlapping sections and addresses past the end of
memory, but lets a thread's code run on past its quarter into another
thread's. `can_loopback`'s thread 1 is 140 words from 0x080 and ends at
0x10B, inside thread 2's quarter. Nothing breaks, because thread 2 is never
started, but a host that runs every thread (`Loom.run()` with no argument
starts all four) would start thread 2 at 0x100, in the middle of thread 1's
receive loop.

**Code today:** the program header says to start threads 0 and 1 only, and
the tests do (`loom.run([0, 1])`).

**Fix wanted:** perhaps a listing note (not an error) when a section crosses
another thread's reset vector, so the reader of the listing sees it.

**Ruled (director, 2026-09-23): yes, a listing note, not an error.** Open
as tools work before the firmware freeze (2026-12-01, `docs/PLAN.md` M4):
the listing marks a section that crosses another thread's reset vector,
and the thread summary names the threads that must then not be started.
Built 2026-09-24, wider than asked: data at a reset vector blocks the thread
too, even in its own section (`i2c_slave_eeprom`'s window starts at thread
3's vector). `Program.unstartable` carries the set; the listings now say
thread 2 for `can_loopback`, thread 3 for `i2c_slave_eeprom` and threads 1
to 3 for `usb_ls_device`.
