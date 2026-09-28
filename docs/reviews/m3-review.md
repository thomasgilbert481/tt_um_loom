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
   back to -6.08 ns. Whether the same inputs reproduce 3 h 54 min is not
   measured, and neither is how much GitHub's runners vary. Is a same-netlist
   run before the freeze worth it, to measure that while there is time to
   act? Is there a fallback other than removing D-031 if the freeze run
   runs long?

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

## What the directing session recommends

1. Keep 2026-11-08 as the freeze date. Treat d1ea0ac's `src/` as the freeze
   candidate and do the freeze-time evidence on it now (the mutation re-run
   with re-keyed equivalents, the SDF subset). If nothing in `src/` changes
   by 2026-11-08, that evidence is the freeze's evidence unchanged.
2. Measure before the freeze: one `gds` run of the unchanged design, started
   by hand (`gh workflow run gds.yaml --ref main`), gives the run-to-run
   variance of identical inputs and whether the `ihp-cmos5l` action branch
   has moved since 2026-09-24. Thomas has to start it: the session's attempt
   to dispatch it was refused by its permission settings.
3. Leave the slow corner at its stated number, and leave ISO-1's caveat
   documented, because both fixes are hardware changes on a design that
   D-032 declared complete.
4. Archive the freeze run's `tt_submission` artefact as a release asset as
   soon as it passes, instead of waiting for M5.
