# Spec questions raised while writing `tools/loomsim`

Opus 5, 2026-09-17, written against `docs/SEMANTICS.md` 1.0-draft and
`isa/isa.yaml` 0.2.0 while implementing the golden model.

Every item below is a place where the text did not decide the answer. The model
implemented the reading marked **chosen**, which was in each case the most
literal one available.

**All twelve were ruled on by the director on 2026-09-17 and SEMANTICS now says
what the answer is.** Each item carries a `Resolution` line: nine confirmed the
model as written, three (items 4, 10 and 11 here) changed it. The director's
own numbering merged this file's items 7 and 8 into their Q7 and promoted the
`BADOP` width note under item 5 to their Q10, so every resolution names their
label as well.

Ordered by how likely they are to bite.

---

## 1. `WAITB` condition 0 when the FIFOs exist but the bit engine does not

Section 9 says "any instruction whose feature is not built executes as `NOP`
and commits `BADOP`". `WAITB` is one instruction with four conditions that
belong to two different features: 0 is "BE idle", 1 and 2 are the FIFOs, 3 is
this thread's tick. In an M1+FIFO build, condition 0 asks a block that is not
there.

Three readings: stall forever (the condition is simply never true), complete at
once (an absent engine is trivially idle), or `NOP` + `BADOP` (the *condition*
names an unbuilt feature).

**Chosen:** `NOP` + `BADOP`, decided on the operand. An unbuilt feature should
never appear to work, and a silent forever-stall is the worst failure mode of
the three. Note this makes `BADOP` depend on an operand rather than only on the
opcode, which the decoder alone cannot do.

Related: the M1 scope ties `WAITB` conditions 1, 2 **and 3** to the FIFO
feature, so in a build with no FIFOs `WAITB 3` is a `BADOP` `NOP` even though
`TICK_SEEN` exists and is maintained. If that is wrong, condition 3 should be
available whenever the instruction is built.

**Resolution (director, their Q1): confirmed as built.** `WAITB` is built with
the FIFO feature; condition 0 additionally needs the bit engine and is `NOP` +
`BADOP` without it, so for this one instruction `BADOP` depends on the operand.
Condition 3 stays tied to the FIFO feature because `WAITB` as a whole is an M2
instruction. Now in SEMANTICS 6.4. No model change.

## 2. Reading or writing a CSR whose feature is not built

Section 6.6 says "write-only and unimplemented CSRs read 0" and "read-only CSRs
ignore writes". Section 9 says an instruction of an unbuilt feature is a `NOP`
that sets `BADOP`. `CSRR rd, SR` in an M1 build is both: the instruction
(`CSRR`) is built, the CSR (`SR`, 0x0D) belongs to the bit engine.

**Chosen:** section 6.6 wins because the instruction is built. `CSRR` of a
bit-engine CSR writes 0 to `rd` and sets no `BADOP`; `CSRW` to one is ignored
and sets no `BADOP`. The alternative (BADOP on every unbuilt CSR number) would
also be defensible and is cheap in RTL, so this one is worth an explicit line
in SEMANTICS.

**Resolution (director, their Q2): confirmed as built.** CSRs of a feature that
is not built read 0 and ignore writes **without** setting `BADOP`; only
instructions set `BADOP`. Now in SEMANTICS 6.6. No model change.

## 3. The count field of `INGRP`

Section 6.3 spells out `cnt = min(OUTGRP[9:5], 16)` for `OUT` but writes only
"with `base/cnt` from `INGRP`" for `IN`. The field is 5 bits, so it can hold 31,
and `rd` is 16 bits.

**Chosen:** the same cap for `IN`, `cnt = min(INGRP[9:5], 16)`. Without it the
loop would index bits 16..30 of a 16-bit register. Tested at counts 0, 1, 16 and
31.

**Resolution (director, their Q3): confirmed as built.** SEMANTICS 6.3 now
spells out `base = INGRP[4:0]`, `cnt = min(INGRP[9:5], 16)` for `IN`. No model
change.

## 4. Does a *host* write of `TICK_INT` or `TICK_FRAC` clear `ACC`?

Section 4: "A committed `CSRW TICK_INT` or `CSRW TICK_FRAC` also sets
`ACC <= 0` at the same edge". It names the instruction. The host can write the
same register through the debug space.

**Chosen:** only the instruction clears `ACC`; a host debug write changes the
divider and leaves the accumulator alone. A shared timer input in the RTL would
naturally clear on both, so this is a likely divergence.

**Resolution (director, their Q4): CHANGED, the model was wrong.** Any write to
`TICK_INT` or `TICK_FRAC` clears `ACC` at the same edge, by a committed `CSRW`
or by a host debug-space write alike, and `NOW` does not tick at that edge.
Now in SEMANTICS 4. The model's `_edge` no longer filters the clear by
`is_slot`; covered by
`test_loomsim_time.py::test_a_host_write_of_the_divider_also_clears_the_accumulator`
and `::test_a_host_write_of_the_divider_restarts_the_tick_phase`.

## 5. What "no step in flight" means for host IMEM access

Section 7: "Host IMEM reads and writes are valid only while `RUN == 0` and no
step in flight; otherwise writes are dropped, reads return 0, and `BADOP[15]`
is set."

**Chosen:** blocked when any `RUN` bit is set, or any `STEP_REQ` bit is set, or
any valid slot is still in F, D, X or W. So after a single step the port stays
closed for the three cycles the slot needs to drain.

Two sub-points the text does not cover:

* A host *write* is evaluated at the edge it commits at, a host *read* is
  answered immediately when the call is made. A read issued in the same cycle
  as the F stage of a stepped slot therefore sees the machine as quiet, because
  the model has not yet advanced the pipeline. Reads have no side effect other
  than `BADOP[15]`, so this only affects that bit.
* `BADOP[15]` is described nowhere except here and in `HOST_PROTOCOL.md`
  (CTRL 0x001A). Section 5 gives the reset value as `BADOP[3:0] = 0`. The model
  implements `BADOP` as 16 bits: bit `t` per thread, bit 15 for host access
  errors.

**Resolution (director, their Q5 and Q10): confirmed as built.** SEMANTICS 7
now defines "no step in flight" as `RUN == 0`, `STEP_REQ == 0` and no valid
slot anywhere in F, D, X or W; SEMANTICS 5 makes `BADOP` a 16-bit register with
bits 3:0 per thread and bit 15 the host access error, write-1-to-clear. The
read-versus-write timing note above still stands as a model implementation
detail and affects only when `BADOP[15]` is set by a read. No model change.

## 6. "While thread `t` is halted" for `r0..r7` debug access

Section 7 says `r0..r7` are host accessible "only while thread `t` is halted",
and explains it by the register-file ports being borrowed during that thread's
bubble slots. But `HALTED[t]` is only set by the `HALT` instruction: a thread
that has never run has `RUN = 0` and `HALTED = 0`, yet its slots are bubbles
and the ports are free. `HOST_PROTOCOL.md` says the debug space is "writable
only while that thread is **not running**", which is the weaker condition.

**Chosen:** accessible when `RUN[t] == 0` and `STEP_REQ[t] == 0`, not
`HALTED[t] == 1`. Otherwise a host could not set up registers before the first
run, which `firmware/` tests will want to do.

**Resolution (director, their Q6): confirmed, with the condition made
explicit.** SEMANTICS 7 now defines halted for debug access as `RUN[t] == 0`,
`STEP_REQ[t] == 0` **and no valid slot of thread `t` in F, D, X or W**;
`HALTED[t]` is only the sticky record that a `HALT` ran. The model gained the
in-flight clause (`Machine.thread_halted_for_debug`), so the port now stays
closed for the three cycles a stepped slot takes to drain; covered by
`test_loomsim_runctl.py::test_registers_stay_closed_until_a_stepped_slot_has_drained`.

## 7. `CTRL.RESET` and the return stack

Section 7 lists `PC`, flags, `TD`, `DEPTH` and `WAIT_ACTIVE`.
`HOST_PROTOCOL.md` says "stack cleared" for the same operation.

**Chosen:** `DEPTH <= 0` only; `RS0` and `RS1` keep their values, which are
unreachable at depth 0 and are overwritten by the next `CALL`. Cheaper in RTL
and matches the more precise of the two documents. Observable through the debug
space, so worth pinning down.

**Resolution (director, their Q7, first half): confirmed as built.** SEMANTICS
7 now says `DEPTH <= 0` with "the contents of `RS0`/`RS1` are left alone". No
model change.

## 8. Which `NOW` does `CTRL.RESET` copy into `TD`?

The reset happens at an edge at which the thread's tick generator may also
advance `NOW`.

**Chosen:** the pre-edge `NOW`, i.e. the value visible during the cycle the
host acted in, as a plain register-to-register transfer at that edge.

**Resolution (director, their Q7, second half): confirmed as built.** SEMANTICS
7 now says `TD <=` the `NOW` value visible in the cycle in which the host write
commits. No model change.

## 9. A host `STEP` that lands on the edge where a slot consumes `STEP_REQ`

Section 7 says a valid slot "consumes `STEP_REQ[t]` in its F cycle", and
separately that on a same-edge conflict "the thread wins".

**Chosen:** the consumption is a thread action, so it wins and the host's new
request is lost. A host that steps and then polls never sees this; a host that
writes `STEP` every cycle silently loses one. If that is the wrong trade, the
set should win and the text should say so.

**Resolution (director, their Q8): confirmed as built.** SEMANTICS 7 now says
such a `STEP` is lost, and notes that the SPI port needs far more than 4 clocks
per command so it cannot happen through the pins. No model change.

## 10. `RESET_PC[t] = t * 0x100` "masked to the memory size"

With a 512-word memory the mask makes `RESET_PC[1]` and `RESET_PC[3]` both
0x100, and with 256 words all four threads start at 0. That is what the text
says, but it is unlikely to be what is wanted.

**Chosen:** implemented literally. The model's default memory is 1024 words, so
the defaults are 0x000, 0x100, 0x200, 0x300 and the collision does not appear
unless a smaller memory is configured. If M2 picks a 512-word memory this needs
a decision (the obvious alternative is `t * (IMEM_WORDS / 4)`).

**Resolution (director, their Q9): CHANGED, the alternative was taken.**
`RESET_PC[t] = t * (IMEM_WORDS / 4)`, so the threads never alias whatever the
memory size: 0, 64, 128, 192 for 256 words, `t * 0x100` for 1024. Now in
SEMANTICS 5. The model computes the stride from `imem_words`; covered by
`test_loomsim_runctl.py::test_reset_vectors_divide_the_memory_into_four` and
`::test_each_thread_starts_at_its_own_vector_in_a_small_memory`.

## 11. `CAPS`

`HOST_PROTOCOL.md` gives `{IMEM_WORDS[11:0]/16, DMEM_PRESENT, FIFO_DEPTH_LOG2[1:0],
BOOTROM}` without bit positions, and there is no bit for "FIFOs built at all"
(depth 0 would have to mean absent, but `FIFO_DEPTH_LOG2 = 0` means depth 1).

**Chosen:** bits 11:0 words/16, bit 12 DMEM, bits 14:13 depth log2, bit 15 boot
ROM. The model's `Machine.caps` is documented as provisional; the RTL should be
treated as the source once it exists.

**Resolution (director, their Q11): CHANGED, a different layout was fixed.**
`[2:0]` log2 of the FIFO depth (0 when the FIFOs are not built), `[3]` FIFOs,
`[4]` bit engine, `[5]` data memory, `[6]` boot ROM, `[11:7]` zero, `[15:12]`
log2 of `IMEM_WORDS`. The M1 build reads 0x8000 at 256 words and 0xA000 at
1024. Now in SEMANTICS 5, and no longer provisional. Covered by
`test_loomsim_api.py::test_caps_has_the_layout_semantics_five_fixes` and
`::test_caps_reports_the_optional_features`.

## 12. `CSRW TICK_INT, 0`

Section 4 computes the period as `max(TICK_INT, 1) * 256 + TICK_FRAC`, and
`isa.yaml` documents the CSR as "integer tick divider, minimum 1". The register
could either clamp on write or clamp on use.

**Chosen:** clamp on use. A write of 0 is stored, `CSRR` reads back 0, and the
divider behaves as 1. Observable difference from clamp-on-write through `CSRR`.

**Resolution (director, their Q12): confirmed as built.** SEMANTICS 4 now says
"`TICK_INT = 0` is stored and read back as 0; the divider treats it as 1". No
model change.

---

## Consequences worth stating rather than questions

These are not ambiguous, but they surprised me while writing the tests and each
one is a place where firmware or a testbench can go wrong.

**Adopted by the director on 2026-09-17:** the first two are now in SEMANTICS
6.4 under "Programming consequences worth knowing", and the third is SEMANTICS
2's "Slot grid" bullet, which the RTL agent has been told to test for. The
model pins the slot-grid behaviour in exactly those terms in
`test_loomsim_time.py`: `test_every_edge_is_zero_to_three_clocks_after_its_deadline`,
`test_edges_are_exactly_periodic_when_the_period_is_a_multiple_of_four`,
`test_edges_dither_but_never_drift_when_the_period_is_not_a_multiple_of_four`
and `test_the_uart_bit_time_dithers_by_two_clocks_either_way`, each over 101
edges.

* **`PREV_PINS` resets to 0**, so on a thread's very first slot a pin that is
  already high looks like a rising edge and `WAITE pin, rise` completes at
  once. Firmware that arms an edge detector must spend one slot (any
  instruction) with the pin at its idle level first. Same after `CTRL.RESET`,
  which does not touch `PREV_PINS`.
* **A thread's first `WAITD` after reset** compares against `TD = 0` while
  `NOW` has already been counting since edge 0, so `reached` is true and the
  wait completes in one slot. `SETD` first, as the UART firmware in
  `tools/tests/test_loomsim_uart.py` does.
* **`WAITD` keeps the deadline exact in ticks, not in clocks.** The wait ends
  at the first X cycle at or after the deadline, and X cycles are on a 4-clock
  grid, so a pad edge sits up to 3 clocks after the deadline. When the tick
  interval is a multiple of 4 the quantisation is constant and the spacing is
  exact; otherwise it alternates around the mean (at 115200 baud, 434 clocks
  per bit, the bit edges alternate 432/436 while the ten-bit frame spacing,
  4340 clocks, is exact). A cycle-exact RTL comparison will see the same
  pattern; a "jitter must be zero in clocks" assertion would be wrong.
* **The number of slots between a `WAITD` and a pin write is part of the
  timing.** Two paths through a bit loop that write the pin at different slot
  offsets after their deadline produce a constant skew between those edges.
  This is firmware's problem, not the core's, but it is the first thing to
  check when a protocol test is off by a few clocks.
