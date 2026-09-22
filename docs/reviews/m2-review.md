# M2 review: state, and the questions for the architect

Written 2026-09-20 by the implementing session (Opus 5) for a "Loom M2 review"
session with Fable, per `CLAUDE.md` ("ask Fable, not yourself, about
architecture"). Everything below is measured, with the run or commit that
measured it.

## Where M2 stands

Exit criterion (PLAN, M2): "UART RX/TX, SPI master and slave, I2C master all
pass their L3 tests with data moving through the SPI host port; FPGA prototype
runs the same tests."

- **Met.** The four programs plus the new `spi_slave` pass their L3 scenarios
  on the golden model (37) and on the RTL through the real SPI pads (29 of
  them), with the same test bodies on both sides.
- The FPGA half is dropped (D-024, Thomas, 2026-09-21): the full design needs
  7,732 LUT4 and the iCEBreaker's UP5K has 5,280, and the project will not buy
  a larger board. Nothing runs on hardware before silicon.

Verification beyond the milestone, which now has to stand in for a bench:
lockstep co-simulation covers the M2 features with the model built from
`CTRL.CAPS`; 71 gate-level tests on the hardened netlist; 18 formal
properties (L4), one of which found a real bug (D-023); a full mutation pass
(L7) killing 99.7 per cent of the non-equivalent mutants.
`docs/VERIFICATION.md` has the table of what lives where.

## The constraint that changed since M1

**Routing time, not area and not slack, is what bounds this design.**

| Run | Design | GRT overflow (Metal3) | Detailed routing | gds job |
|---|---|---|---|---|
| 35419160398 | macro core | 1,768 | 3 h 03 min | 3 h 53 min, finished |
| 35470401774 | the same, +2.2% cells (D-022) | 5,776 | > 5 h 15 min | killed at GitHub's 6 h limit |

Tiny Tapeout re-runs this flow when a project is submitted, so six hours is a
budget rather than a CI inconvenience, and a change of two per cent in cells
spent two hours of it. The block is only 51.5 per cent full, so **area is not
the scarce resource; local wiring is**. D-022 was reverted on those grounds
(its entry has the numbers), which leaves the slow corner at -2.07 ns; the
flow signs off at the typical corner, where the design has +6.21 ns.

## Questions for the architect

1. **Does M3's bit-engine auto mode fit that budget?** Auto mode (NRZI,
   Manchester, stuffing, per-thread encoders) is the largest remaining RTL
   feature, and it lands in the same part of the floorplan as the existing
   per-thread state. If it costs what the FIFOs cost (+2,274 cells, +619
   flops), the routing evidence above says that is hours, not minutes, of
   detailed routing. Is auto mode still worth it against the stretch protocols
   it exists to serve, and if so should it be built one encoder at a time with
   a hardening between each?
2. **Which stretch protocols survive that answer?** USB low-speed is the one
   with a fixed slot in the plan and the one that most needs auto mode. CAN,
   10 Mbit Manchester, PS/2, JTAG and SWD are all cheaper in firmware if the
   bit engine stays manual.
3. **Is the slow corner worth any more effort?** Options, in rising cost: leave
   it and state the silicon's safe clock at the slow corner (about 45 MHz);
   spend one hardening on a flow knob (a DELAY synthesis strategy, post-GRT
   resizer timing, timing-driven placement); or take D-022's cheaper shape
   (two subtractors per thread instead of three) and measure both slack and
   congestion.
4. **What does "done" look like for the submission?** The plan's M5 and M6 are
   docs and submission. Given that the verification story is already the
   strongest part of this project, is there a case for freezing RTL earlier
   than 2026-12-01 and spending the difference on the write-up, which with no
   FPGA (D-024) is what has to carry the case that the design works, rather
   than on another protocol?

## What the implementing session recommends

Take auto mode in one slice, harden after it, and decide the rest of M3 on
what that slice costs in routing rather than on the feature list. With no
FPGA before silicon (D-024), the write-up carries more weight: it has to show
a reader why the simulation, gate-level, formal and mutation evidence is
enough, so starting it before the RTL freeze rather than after is worth
considering too.
