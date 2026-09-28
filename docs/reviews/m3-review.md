# M3 review: state, and the questions for the architect

Written 2026-09-27 by the directing session (Opus 5.5) for a "Loom M3 review"
session with Fable (PLAN M3, last item: "the slices' numbers, ISO-1's state,
the freeze commit"). Everything below is measured, with the run or commit
that measured it. The M2 review (`m2-review.md`) set the rules this one is
read against: D-025 (routing time gates every hardware change) to D-030
(freeze 2026-11-08).

## Where M3 stands

Exit criterion (PLAN, M3): "slices A and B hardened inside the D-025 budget;
`ws2812`, `ps2_host`, `jtag_master`, `swd_master`, `i2c_slave_eeprom` and
`usb_ls_device` pass their L3 tests on the golden model and on the RTL;
ISO-1 attempted and its result recorded, bounded or proved."

- **Met, five weeks early.** Slices A and B were each hardened alone and
  stayed under D-025. All six programs pass on both backends, and so do
  `can_loopback` and M4's `manchester_loopback` (L3-MANCH, D-029): 108
  scenarios on the model, 94 on the RTL in every CI run, and the other 14
  (slow cases) on the RTL in a separate run, last on 2026-09-26 after the
  T-1 fixes. ISO-1 is proved unbounded for all four threads (below).
- **Slice C is not built** (D-032, Thomas, 2026-09-27, ahead of the
  2026-11-01 date): the baseline routes over D-025's stacking bar and no
  program needs auto mode. The hardware is complete.
- **D-031** (your proposal after D-028: a registered one-hot host debug
  thread select) was built after the slices and kept.

## The numbers

Hardenings since the M2 review, one change each (`docs/AREA.md`):

| Run | Change | Metal3 overflow | Detailed routing | `gds` job | Typical | Slow (1.08 V, 125 C) |
|---|---|---|---|---|---|---|
| 35779039938 | D-028 | 1,248 | 2 h 57 min | 4 h 05 min | +3.56 ns | -6.36 ns |
| 35812463112 | slice A (+1,205 cells) | 2,924 | 3 h 27 min | 4 h 36 min | +4.80 ns | -4.04 ns |
| 35871222851 | slice B (+861 cells) | 2,252 | 3 h 29 min | 4 h 38 min | +3.56 ns | -6.08 ns |
| 35940928210 | D-031 (-156 cells in generic synthesis) | 1,945 | 3 h 54 min | 5 h 02 min | +5.38 ns | -3.53 ns |

The last run (on d1ea0ac) passed every job: DRC, LVS and antenna 0,
precheck, gate level 100 of 100 (the one flop-memory test skipped), 31,393
standard cells plus the macro, 54.7 per cent utilisation. **No file in
`src/`, `info.yaml` or `macro/` has changed since d1ea0ac**, so that run's
netlist is the current design.

Two readings of the table:

- Routing time does not follow overflow. D-031's run had less congestion
  than either slice's run and routed longest. D-025 already
  names placement variance (about half an hour); the fourth point makes it
  the dominant term.
- The non-routing part of the `gds` job is steady at 68 to 69 minutes. The
  job fails GitHub's six hours if detailed routing passes about 4 h 51 min,
  57 minutes above the last run.

## Verification since M2

- **Formal.** ISO-1 proved unbounded (abc pdr) on the two-copy miter for all
  four threads with the host debug port quiet, extended on 2026-09-23 to
  slice B's data-memory state and a thread's own stores (F-7: the first
  miter predated slice B and never compared them). F-6: properties written
  before slice B took the LD/ST completion slot for an instruction; fixed.
  SCHED-4 added (BUGS 6: a host `STEP` and the thread's consumption on the
  same edge; unreachable through SPI, fixed in RTL). The L4 list has
  nothing unattempted.
- **Mutation.** 823 mutants, no open survivor since 2026-09-24. The last
  one became reachable through slice B (an `LD` of a word that encodes
  `SETD`) and was killed by a new `test_mem` check; the mutation ladder's
  module list had never run `test_mem`. The re-run on the freeze commit is
  still owed, and `tools/mutate/equivalents.json` keys the 44 equivalents
  by line number, which later commits have already moved.
- **Tools finding T-1.** The deadline checker's budget after a `SETD`
  assumed the `SETD` ran on its tick. Under the sound rule 11 pairs in six
  programs failed, and three were real timing faults that no test had
  caught (a UART start bit 4 clocks short after some idle gaps, JTAG TCK
  phases of 36/28 clocks, SWD clock-high phases of 20 and 28 where 32 were
  meant). Each program was fixed with a test that measures every edge on
  the pads; the sound rule is the default (BUGS 7 to 10).
- **Gate level with SDF.** The `gds` artefact carries SDF; an opt-in mode
  runs cocotb on the netlist with cell delays (about 1.9 s per simulated
  microsecond after about 6 minutes of annotation), so the full L3 suite
  would take days and the plan is a subset overnight on the freeze netlist.

## Questions for the architect

1. **The freeze commit.** The RTL is complete (D-032) and unchanged since
   d1ea0ac. Keep 2026-11-08 as the freeze date, or freeze now? Keeping the
   date leaves a window for a bug fix while verification continues; freezing
   now changes nothing in the RTL either way.

2. **The final `gds` run and six hours.** D-030's fallback, "the last slice
   comes out", now means D-031, and removing it would bring the slow corner
   back to -6.08 ns. Measured on 2026-09-28 (run 36366875261, the unchanged
   design hardened again, `docs/AREA.md` "The same netlist twice"): the flow
   is deterministic, every metric identical to run 35940928210's, so the
   freeze run of an unchanged design reproduces this netlist; only the
   runner's speed moves the time, here by 3 per cent (`gds` job 5 h 11 min,
   detailed routing 4 h 00 min, 49 minutes inside six hours). Is there a
   fallback other than removing D-031 if the freeze run runs long? And
   should the workflow pin the Tiny Tapeout action to a commit instead of
   the `ihp-cmos5l` branch, so that nothing can move underneath the freeze
   (a change to `gds.yaml` starts a hardening, so it would ride with the
   freeze run)?

3. **Artefact lifetime.** The runs' artefacts expire after 90 days: run
   35940928210's on 2026-12-23, before the 2027-01-18 deadline. A run on
   2026-11-08 keeps its artefacts until about 2027-02-06, and PLAN M5 already
   archives the final run as a release asset. Is anything else needed for the
   submission to point at?

4. **The slow corner.** It is stated at about 42 MHz. The two path families
   left have known fixes (an X-stage one-hot ring, the X-stage twin of D-019,
   and a pre-decoded host write class in `loom_host_ctl`; `docs/AREA.md`,
   "The slow corner after D-031"). Each would be a hardening on a baseline
   already over the stacking bar. Leave them?

5. **ISO-1's shared-memory caveat.** Isolation is proved given that no other
   thread stores into memory the thread reads; the hardware lets any thread
   store anywhere (SEMANTICS 6.11), and staying in one's quarter is the
   programs' job. A hardware guard would be a `src/` change. Leave it as a
   documented limitation?

6. **The write-up.** VERIFICATION_REPORT section 8 ("what the AI did and how
   it was checked") is filled at the freeze. Is there anything from the
   architecture phase (2026-09-14 to 2026-09-17) that it should record?

7. **8x4.** Added 2026-09-28. The organisers answered the macro question
   (a design with the IHP SRAM macro can go on the shuttle, D-020) and said
   to keep designing to 6x4 and treat 8x4 as an upgrade if Tiny Tapeout
   enables it for CMOS5L. 8x4 would be about 34 per cent more area on the
   same stripe pitch (`docs/tt_cmos5l_facts.md` section 2), which matters
   here because routing time, not area, is the constraint: at 54.7 per cent
   utilisation the design routes in 4 h 00 min on a slow runner. If 8x4
   lands before the freeze, is a switch worth one hardening of the
   unchanged design at 8x4 on a branch, adopted only if every job passes
   with clearly more routing margin? The macro placement and the PDN
   stripe keys of D-021 would need checking against the new block. Checked
   the same day: the tools branch has had the 8x4 tile and its block DEF
   since 2026-09-21 (tt-support-tools `d66cf17`), and that DEF differs from
   the 6x4 one only in die width and row length (same origin, rows and pin
   positions), so the experiment is the one-line `tiles` change in
   `info.yaml` on a branch, and could run before the organisers confirm.
   **Measured 2026-09-28** (Thomas approved the branch run; run
   36427897067 on `exp-8x4`, `docs/AREA.md` "The same design on 8x4"):
   every job passed; utilisation 40.8 per cent; global-routing overflow
   2 / 3 against 1,945 / 2,019; detailed routing 2 h 53 min and the `gds`
   job 4 h 02 min against 4 h 00 and 5 h 11; slack typical +6.78, fast
   +11.47, slow -1.14 ns on 23 endpoints (about 47 MHz) against +5.38,
   +10.60 and -3.53 ns on 96. The macro and stripe keys carried over
   unchanged. So the question is only whether to adopt it: the recommendation
   below says yes, as soon as the organisers confirm 8x4 for the shuttle.

## What the directing session recommends

1. Keep 2026-11-08 as the freeze date. Treat d1ea0ac's `src/` as the freeze
   candidate and do the freeze-time evidence on it now (the mutation re-run
   with re-keyed equivalents, the SDF subset). If nothing in `src/` changes
   by 2026-11-08, that evidence is the freeze's evidence unchanged.
2. Done 2026-09-28: Thomas started the same-netlist run by hand (the
   session's own dispatch was refused by its permission settings). It
   reproduced every metric and took 3 per cent longer, so the freeze run's
   risk is the runner's speed and a change in the action, not the design.
   Pin the action at the freeze, with the freeze run. (The review below
   decided against pinning and for recording the commits instead: Q2.)
3. Leave the slow corner at its stated number, and leave ISO-1's caveat
   documented, because both fixes are hardware changes on a design that
   D-032 declared complete.
4. Archive the freeze run's `tt_submission` artefact as a release asset as
   soon as it passes, instead of waiting for M5.
5. Move to 8x4 as soon as the organisers confirm it for the shuttle: it
   turns the six-hour risk (49 minutes of margin) into almost two hours,
   removes the congestion and one of the two slow-corner families, and
   changes nothing but the `tiles` line. The switch is a DECISIONS entry
   and one commit on main, and that commit's hardening would be the freeze
   run if it lands close to 2026-11-08.

## Review (Opus 5.5, 2026-09-28, in the directing session)

Thomas asked for the review to be done in the directing session rather than
in a separate Fable session. That costs the independence the plan wanted
from it (the risk table calls the Fable reviews "the second opinion"): the
brief and the answers below come from the same session. Section 8 of the
verification report records that. Everything below was checked against the
repository and the runs named, not taken from the brief.

### M3: met

The exit criterion holds as the brief states it, five weeks early: both
slices hardened inside D-025, the six named programs and two more pass on
both backends, and ISO-1 is proved for all four threads. D-032 removed the
one optional item. M3 is closed.

### Q1. The freeze stays on 2026-11-08; its evidence may be produced early

Freezing now gains nothing: `src/` has not changed since d1ea0ac, and no
change is planned. Keeping the date keeps a window in which a bug found by
verification can still be fixed at no procedural cost. So (D-033):

- The freeze date stays 2026-11-08 (D-030). The freeze commit is the last
  commit before that date that touches `src/`, `info.yaml` or `macro/`,
  and its hardening is started by hand.
- Freeze-time evidence (the mutation run, the SDF subset) may be produced
  now, on d1ea0ac's `src/`. It stands for the freeze as long as `src/` is
  unchanged, and the report names the commit it was produced on. A `tiles`
  change alone leaves `src/` as it is, so an 8x4 switch keeps the mutation
  evidence; the SDF subset reads the netlist and is redone on the final
  block.

### Q2. The final run: re-run first, then 8x4, and only then D-031 out

The same-netlist run changes what D-030's fallback should be. Identical
inputs give identical results, so a freeze run that passes six hours has
met a slow runner, not a problem in the design. Removing D-031 would buy
time only because slice B's netlist happened to route faster (3 h 29 min),
and would cost 2.5 ns at the slow corner. The order is therefore (D-033):

1. re-run once, by hand: the netlist is deterministic, the runner is not;
2. if that also fails and the organisers have confirmed 8x4, switch to 8x4
   (Q7): 4 h 02 min measured, almost two hours of margin;
3. only then revert D-031, whose predecessor's job took 4 h 38 min.

On pinning: do not pin the action or its tools on main. The action has not
changed since 2026-09-14, the tools last changed on 2026-09-21 (8x4 only),
and following the branch keeps CI an early warning of anything Tiny Tapeout
changes before submission, when their current flow is the one that counts.
Record instead, for every run that matters (the freeze run and the M6 run),
the action and tools commits it used. Identical inputs give identical
metrics, so any difference between two such runs names a flow change. If an
upstream change breaks the flow, pin while it is investigated: the action
by commit in `uses:`, its tools through the action's `tools-ref` input.

### Q3. Artefacts: archive every run that matters; M6 refreshes them

A run's artefacts live 90 days. The freeze run's cover the deadline, and
PLAN M6's fresh-clone run, due by 2027-01-11, produces new ones right before
submission. Archive `tt_submission`, `gds_render`, `precheck_reports` and
`gatelevel_test_results` of both as release assets as soon as each passes
(the freeze run's at `v1.0-rc1`, the M6 run's at `v1.0`), with the metrics
the report quotes. Nothing else is needed from the flow.

### Q4. The slow corner: leave it, and state it

Leave both path families. Each fix is an RTL change after D-032, with a
hardening, a mutation delta and formal re-runs to pay for a number at a
corner the flow does not sign off. If 8x4 is adopted, only one family is
left, at -1.14 ns (about 47 MHz), and the datasheet states that. A
multicycle constraint on the host debug address path was considered: the
address is registered cycles before its write strobe, so it may well hold,
but a wrong timing exception is a silicon bug, and proving this one is more
work than the 3 MHz it would state. Not taken.

### Q5. ISO-1's caveat: keep it, as a documented property of the ISA

A hardware store guard would be more than a `src/` change after D-032: it
would break a shipped program. `i2c_slave_eeprom` keeps its 256 bytes in
thread 3's quarter while running on thread 0, which is exactly what the
shared memory is for (D-027). Stores anywhere are the ISA's design, and
isolation holds for programs that keep their stores off the memory other
running threads use. SEMANTICS 6.11, the report's section 7 and the README
already say so; nothing to add.

### Q6. Section 8 of the report

It should record, with dates and commits:

- who did what: Fable 5.1 set the architecture and plan (2026-09-14 to
  2026-09-17) and reviewed M2; Opus 5 implemented from 2026-09-18; Opus
  5.5 directed from 2026-09-23 and did this review; Thomas made the owner's
  calls (D-013 solo, D-024 no FPGA, D-028 delegated, D-032 no slice C, the
  8x4 experiment), and every push went through his account;
- how METH-1 was enforced in practice: separate agents for model and RTL,
  briefs that forbade reading the other side, the spec-question files and
  the rulings that closed them;
- what went wrong in the method, not only what it caught: T-1 (the
  deadline checker's own rule was unsound, and three shipped programs had
  real faults, found by a firmware author rather than a test); F-6 and F-7
  (formal harnesses went stale when slice B added state); the last mutant,
  which the ladder never ran against `test_mem`; BUGS 4 (a test bug only
  the gate level exposed); the red lint CI from 950cafd to c5e7ca1; and
  that this review was not independent;
- the per-layer bug counts from `docs/BUGS.md` as METH-2's summary.

### Q7. 8x4: adopt on the organisers' confirmation

Yes. The branch run answers every question the brief asked: every job
passed, the macro placement and the stripe keys carried over unchanged, the
twelve Magic overlaps are the four stripe crossings D-021 already watches
(Metal4 stripes over the macro's Metal4 obstruction, at the same x as at
6x4, split into two more boxes by a different extraction boundary),
congestion is gone, and the `gds` job has almost two hours of margin. Main
moves when the organisers confirm 8x4 for the shuttle, not before: they
asked for 6x4 until then. The switch is a physical change, not an RTL one,
so it is allowed after the freeze too, up to M6, with its own hardening
(D-034, proposed). When it lands: `tiles` and the stale comment above it in
`info.yaml`, the datasheet's slow-corner clock, AREA's current row, the
README and section 6 of the report, and the SDF subset if it already ran at
6x4.

### Two more things the brief did not ask

- **The demo the architecture is built for does not exist yet.** Every
  program runs alone on thread 0 (the two loopbacks use two threads). The
  claim that sets Loom apart, that one thread cannot disturb another's
  timing, is proved formally (ISO-1) and tested with synthetic loops
  (`test_timing`), but no test runs four real protocols at once. Build it
  before the firmware freeze: four programs on four threads, pins remapped
  so they do not collide, each protocol's timing checked on the RTL against
  its single-thread run. It is the strongest demo material M5 can have, and
  it needs no hardware change.
- **The v3 demo board transfer moves up.** The chips will most likely ship
  on the RP2350B board, where `--ttboard` does not work today. It is host
  software, so it can land after the freeze, but it must land before
  submission, because the datasheet's how-to-test has to work on the board
  the chips come on.

### What Thomas decides

1. Whether 8x4 is pre-approved: if yes, the switch is made the day the
   organisers confirm it, without another round trip.
2. When the laptop can run overnight: the mutation run (about seven hours)
   and the SDF subset each need an idle night.
3. Whether to ask Fable for an independent read of this review later. It is
   optional, and the plan does not depend on it.
