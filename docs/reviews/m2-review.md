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

## Architect's answer (Fable 5.1, 2026-09-22)

Read in full before answering: this file, `docs/PLAN.md`, `docs/DECISIONS.md`
D-020 to D-024, `docs/AREA.md`, `docs/VERIFICATION.md`, `formal/README.md`,
`src/loom_timer.v`, `src/loom_be.v`, `src/loom_imem.v`,
`src/loom_imem_macro.v`, the bit-engine and pipeline parts of
`src/loom_core.v`, `fpga/icebreaker/README.md`, and the competition brief as
recorded in `docs/tt_cmos5l_facts.md` section 6. The decisions taken here are
D-025 to D-030, and the plan's M3 and M4 are rewritten to match.

### M2: met

The exit criterion without its FPGA half is met with measured evidence, and
D-024 is the right call for a solo entry: an ECP5 board would have bought a
demonstration, not a check the project lacks. What the FPGA would have been
for, the write-up now has to do, and that thread runs through every answer
below.

### The budget, as a rule the implementer can apply (D-025)

The three routing data points in `docs/AREA.md` (Metal3 overflow 1,768, 3,169
and 5,776 against detailed routing of 3 h 03 min, 3 h 37 min and more than
5 h 15 min) and the 50 to 70 minutes the rest of the `gds` job takes give:

- **A hardware change stays only if the hardening after it finishes detailed
  routing in under 4 hours.** That leaves the hour the rest of the job needs
  plus the half hour placement variance alone was measured to cost. The
  global-routing report is the early warning: a Metal3 overflow above about
  4,000 is a change that will not make it.
- **One hardware change per hardening**, so a bad reading has one cause. A
  docs or tools commit costs nothing (D-018), so there is no reason to batch.
- **Wide buses tapped by per-thread logic are the pattern that costs**
  (D-022: twelve 16-bit subtractors on the shared `cm_td` and `h_wdata`).
  Narrow per-thread control state and one shared logic cone in the X stage
  are not that pattern, and everything below is built from the second kind.
- **Reading the overflow costs six hours only because detailed routing is in
  the same job.** One session goes on finding out whether LibreLane runs
  locally, in the Docker image `CLAUDE.md` names, with the pinned PDK, to the
  end of global routing and no further. If it does, every slice is measured
  in about an hour before it goes near CI. If it does not, the CI run is the
  measurement and the rule above still stands.

### Q1. Auto mode: not in the shape that was specified

The question assumed auto mode is one feature that costs what the FIFOs cost.
It is two features, and the expensive one is not the one the protocols need.

**What USB and CAN need is the encoder and the stuffer, and they belong in
manual mode.** ARCHITECTURE 8 already says `SHO` goes "through the encoder"
and that a pending stuff bit makes the next `SHO` emit it with `CNT`
unchanged. USB low-speed at 1.5 Mbit/s is 33.3 clocks per bit, eight slots,
and the manual loop `SHO; WAITD 1; BNZ` (SEMANTICS 6.9) is three of them, so
a thread transmits and receives USB with the hardware doing NRZI and
stuffing and the firmware doing framing. Without hardware NRZI and stuffing
the per-bit firmware (extract the bit, pick the level, count the run, write
the pin, wait, loop) comes to nine or ten slots by an instruction count
against the eight available; with them the engine does not need to run on
its own. That is slice A (D-026):

- NRZI and Manchester on `SHO`'s output and `SHI`'s input, USB and CAN
  stuffing and destuffing with the pending-stuff rule of ARCHITECTURE 8.1,
  the T flag on a violation. One copy of the logic, in the X stage next to
  the shifter and the CRC, selected by `xsel` as they are today; per thread
  only the narrow state the encoder needs (the NRZI level, the run counter,
  the pending-stuff bit, the Manchester half-bit phase, the previous RX
  sample).
- Manchester in manual mode is two `SHO` per bit, one per half bit, with the
  engine tracking the half-bit phase; the tick is a half bit. Five slots per
  bit is about 2.5 Mbit/s at 50 MHz, and it needs nothing from the deadline
  latch.
- USB drives D+ and D- together. One `SHO` must write both, complementary,
  which rides on the existing group-write path (the one `OUT` uses) with a
  differential bit in `BE_CFG`; the PHASE bits are free for it (slice C
  below). Without this, USB TX needs two threads or an external inverter.
- `SEMANTICS` 6.9 gets its M3 text first; the golden model and the RTL are
  written from it separately as always; `L1-BE-ENC` and `L1-BE-STUFF` come
  from the L1 list; the co-simulation generator learns the new `BE_CFG`
  bits; and the hardening after it is the measurement.

**What autonomy costs is a per-thread datapath, and that is the version that
looks like the FIFOs.** Four engines, each with a shifter, a CRC, an encoder
and a stuffer running on their own ticks, are four copies of what the X stage
has once, plus a third writer on every SR, CNT and CRC flop and a second
requester on every FIFO port. It would also falsify SCHED-2 as proved (SR,
CNT and CRC change only in the thread's own W stage) and make ISO-1 harder.
So auto mode, if it is built at all, is built in the shape that shares the
datapath (D-026, slice C):

- In auto mode the engine acts **in the thread's own slot**: when `MODE` is
  set and `TICK_SEEN` is set, the slot performs an implicit `SHO` or `SHI`
  alongside its instruction, through the same X-stage logic. The transmit
  edge is applied through the thread's deadline latch, with the next tick
  edge as a third fire condition, so it is clock-exact as `SETP ... D` is;
  the receive sample is taken in the slot's X cycle, a fixed few clocks after
  the tick, and firmware puts the tick where it wants the sample by anchoring
  with `SETD`. The PHASE field goes away.
- `AUTOPULL` and `AUTOPUSH` are the thread's own FIFO port used by the
  engine; the SEMANTICS text names what happens in the one slot where the
  instruction and the engine want the same port (the engine wins and the
  instruction re-issues, or the instruction is a BADOP in auto mode; the
  author picks one and writes it down).
- Every proved property stays true: state still changes only in the owning
  thread's W stage, the slot grid is untouched, no new FIFO port exists.
- The cost is per-thread control bits and one more select on muxes that
  exist. The limit is one engine action per slot, so a tick period of at
  least 4 clocks: 12.5 Mbit/s NRZ and about 6 Mbit/s Manchester at 50 MHz.
  10 Mbit Manchester is out of reach in this shape, and it was the only
  protocol the autonomous shape existed for.

**Order and gate.** Slice A first, then slice B (Q2), each hardened. Slice C
goes in only if, after slice B's hardening, Metal3 overflow is under 3,500
and detailed routing under 3 h 45 min, and it can be written, modelled,
co-simulated and hardened before the RTL freeze. If it does not go in,
`BE_CFG.MODE` keeps reading 0, which is what the design already does for
what it has not built, and the write-up says why: with deadline-exact edges
from the core, autonomy buys speed the chip cannot use.

### Q2. Which stretch protocols survive (D-029)

| Protocol | Decision | Needs |
|---|---|---|
| USB low-speed device | **Keep**, the one stretch with a slot. TX and RX in manual mode at 1.5 Mbit/s, a Python host model, enumeration to SET_ADDRESS and one HID report in simulation (`L3-USB-LS`). PC enumeration went with the FPGA. | slice A |
| CAN | Keep as firmware after slice A: its stuffer is the other value of the same field. Loopback with stuffing and CRC15 (`L3-CAN`). At CAN rates (50 clocks per bit and up) stuffing fits in firmware, so if the CAN variant of the stuffer costs more than a handful of cells, the hardware variant is dropped and the firmware does it. | slice A or nothing |
| 10 Mbit Manchester at 60 MHz | **Cut as a target.** It needs the autonomous engine, a transceiver the project does not have, and a bench that does not exist. The Manchester encoder stays (a few cells in slice A) and `L3-MANCH` becomes a loopback at the manual-mode rate. | slice A |
| 10 Mbit Ethernet | Not attempted; the write-up says so. | |
| WS2812, PS/2 host, JTAG master, SWD master | Keep, firmware only, M3. WS2812's 0.4 and 0.8 us pulses are `SETP ... D`. Each is a program, a `protomodels` model and one L3 test, and together they are the evidence for "reprogrammable for a protocol nobody planned". | nothing |
| I2C slave EEPROM | Keep, on slice B. This is the device-emulation claim of ARCHITECTURE's differentiator table, and a 24C02 is 256 bytes, which a register-backed variant cannot be. | slice B |

**Slice B is data memory on the instruction memory (D-027),** which answers
OPEN-3 without a second macro and without a flop array:

- `LD` and `ST` use the thread's **own next fetch cycle**. The first slot
  computes `ra + imm5` in X, holds PC as a stall does, and commits the access
  into one shared holding register (valid, write, 9-bit address, 16-bit
  data). A thread's F cycle is the cycle right after its W edge, so one
  register serves all four threads in turn. That F cycle addresses the macro
  with the data address instead of PC (a write for `ST`), the D cycle sees
  the word, and the second slot writes `rd` (for `LD`) and advances PC. Two
  slots: the `isa.yaml` timing class changes from `one_slot` to a two-slot
  class and the assembler's deadline checker counts two.
- No other thread is touched: the access rides in the thread's own fetch
  slot, SCHED-1 and SCHED-2 hold as proved, and host IMEM access is already
  confined to halted threads. Data lives in unused instruction words (a
  thread's quarter is 128 words; the M2 programs use 18 to 105), the
  assembler gets a data directive, and the host loads and dumps an EEPROM
  image with the IMEM commands it already has.
- Cost: one holding register, a wider mux in front of the macro port, one
  more source into the register write mux. Small, but at the macro's pin
  edge, which is the region that congested in `sram-smoke` run 4, so its
  hardening is read like any other.

Closed with it: FIFO depth (OPEN-2) stays 4, since a low-speed USB data
packet is at most 8 bytes, four words; boot ROM (OPEN-4), group-match wait
and CRC-32 (OPEN-5) are not built, because none is a protocol and each is
wiring.

### Q3. The slow corner: one change that removes logic, then state the number

The flow knobs are the wrong tool. A DELAY strategy, post-GRT resizing or
timing-driven placement each change cell selection across the whole design,
which is a routing gamble that costs six hours to read, for a corner the flow
does not sign off at and a board that never runs at 1.08 V and 125 C.
D-022's smaller shape is still eight subtractors on shared buses. Leaving it
is acceptable.

Better than leaving it is the change that **deletes** the slow path (D-028,
proposed to Thomas because it narrows SEMANTICS 6.10): take the host's TD
write out of rule 2. Twenty-two of the twenty-three violations start at the
host thread decode, go through `h_mine` and its fanout into the `td_new` mux
and then the subtractor (D-022's analysis). With host writes out of the fire
rule, `td_new` is the commit value or `TD`, selected by the W ring (D-019,
fast), and `h_wdata` and the thread compare leave the cone; `TD` itself is
still written by the host exactly as now. A staged write then fires on the
thread's own deadline writes and on the exact tick (rule 1); a host `TD`
write while the thread is halted no longer fires it, which is a debug-only
corner nobody has needed. The same change settles the open mutation survivor
at `loom_timer.v` line 112 by construction, and turns "a thread's staged
write fires only on its own TD writes" from a sentence into a property to
prove. The `tx_th` path (-0.26 ns) stays. Because the change only removes
logic it cannot make routing worse; it can only fail to be enough, and one
hardening says which.

Whatever that hardening says, the datasheet states both numbers: 50 MHz at
the typical corner with +5.98 ns of margin, and the slow-corner clock as
measured (44 MHz today, from 20 + 2.48 ns; about 49 MHz if only `tx_th` is
left). That is a better paragraph than a silently closed corner, because it
shows the trade was measured (D-022) and priced.

### Q4. Done: freeze the RTL on 2026-11-08, not 2026-12-01 (D-030)

Yes, freeze earlier. Four reasons, in order of weight:

1. The plan's own calendar says course deadlines cluster in late November
   and finals are the first week of December. A freeze on 2026-12-01 puts the
   last hardware change and its six-hour hardening in the week Thomas has the
   least time to react to a killed run.
2. M2 finished four weeks early. 2026-11-08 still leaves seven weeks for what
   M3 now is: two small RTL slices, one deletion and one optional slice, each
   with one hardening, plus firmware that needs no hardening at all.
3. With no bench, the write-up is the evidence, and it has to describe one
   netlist. Every week of RTL after the freeze is a week the report describes
   something that changed.
4. The RTL freeze is not a firmware freeze. Firmware, tests, tools and docs
   do not trigger a hardening (D-018) and continue to 2026-12-01, so the
   protocol list keeps growing after the hardware stops.

After the freeze: the final `gds` run by hand on the freeze commit, which
must finish inside six hours (if it does not, the last slice comes out, which
is why slices harden one at a time); then bug fixes found by verification
only, each followed by a re-hardening. `docs/VERIFICATION_REPORT.md` starts
now as a living document, because its material already exists (the bug
ledger, the mutation table, the formal findings, the routing-budget story);
its first section is D-024's statement that nothing ran on hardware and what
stands in for it. Two additions that cost no RTL and earn their place in that
report: ISO-1, thread isolation, which is the architecture's central claim
and is still unproved (timeboxed to two sessions on a reduced configuration;
a bounded result is reported as bounded), and, if the `gds` artefact includes
SDF, one timing-annotated gate-level run of the L3 suite at the typical
corner, which is the closest thing to a bench the project can have (check the
artefact first; that it includes SDF is unverified).

M5's demo material is now the assembler listing with slot annotations, the
waveforms from the RTL L3 runs (the 433- and 434-clock `SETP ... D` edges at
a 433.5-clock tick are the picture of the timing claim), the gate-level log
and the mutation table. No video.

### What Thomas decides

2026-09-22, Thomas: "proceed based off your best judgement". Fable's calls:
D-028 taken (it deletes logic, and the debug corner has no user); the
2026-11-08 freeze stands; slice C is decided by the numbers on 2026-11-01,
and the default is no. The three items as put to him, for the record:

- D-028 (host `TD` writes leave rule 2): yes or no. The implementer does not
  start it without a yes.
- The freeze date, if 2026-11-08 collides with something the plan does not
  know about.
- Whether slice C is wanted at all if its gate opens, given that its only
  protocol is a simulation at 60 MHz.
