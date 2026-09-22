# Spec questions from the M3 firmware (ws2812, ps2_host, jtag_master, swd_master)

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
