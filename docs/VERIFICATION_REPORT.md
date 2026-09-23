# Loom verification report

A living document, started 2026-09-22 (D-030) and finished at the RTL freeze.
It describes `main` as of the date in the changelog at the bottom; at the
freeze it is pinned to the commit the final `gds` run was made from. Every
number here has a source in the repository (`docs/VERIFICATION.md`,
`docs/AREA.md`, `docs/BUGS.md`, `formal/README.md`, `docs/DECISIONS.md`), and
the commands in section 9 reproduce them.

## 1. What this chip is, and what "verified" means here

Loom is a protocol emulator: a four-thread barrel-pipelined I/O processor
with deadline-based timing, per-thread bit engines and a host port, meant to
be reprogrammed for a protocol after fabrication (`docs/ARCHITECTURE.md`).
The claim to be verified is therefore not "a UART came out", but that the
cycle-exact contract in `docs/SEMANTICS.md` holds on the hardened netlist:
that every instruction does what the text says, on the clock it says, and
that a thread's timing does not depend on what the other threads do.

The evidence is organised in the layers of `docs/VERIFICATION.md`, L0 to
L8, each with named checks. This report says, for each layer, what exists,
what it found, and what it would miss.

## 2. Nothing has run on hardware before silicon

The plan included an FPGA prototype. It was dropped on 2026-09-21 (D-024):
the full design needs 7,732 LUT4 and the board on hand, an iCEBreaker
(iCE40UP5K), has 5,280; the project did not buy a larger board. So nothing
in this report comes from a bench. The Tiny Tapeout silicon, if the design
is selected, is the first hardware, and the L6 bench checks in
`docs/VERIFICATION.md` are the chip's bring-up plan, listed as open.

What stands in for a bench is the combination below. None of these is a
substitute for silicon on its own; together they cover what a bench would
have shown, except analogue behaviour of the pads, which the flow's sign-off
covers by construction.

| Layer | What runs | Where the numbers come from |
|---|---|---|
| L2 co-simulation | the RTL and an independently written golden model, in lockstep on every cycle, on constrained-random programs with the host port driven during the run | `test/test_cosim.py`, `tools/loomgen` |
| L3 protocol tests | the firmware programs (UART TX/RX, SPI master and slave, I2C master) against Python protocol models, the same test bodies on the model and on the RTL through the real SPI pads | `tools/tests/test_fw_*.py`, `test/test_fw.py` |
| L5 gate level | the whole cocotb suite on the hardened netlist with the foundry's cell models, in CI on every hardening | `gl_test` job; `scripts/gl/` locally |
| L4 formal | 19 properties on the scheduler, FIFOs, timer, pins, SPI port and decoder, unbounded where the engine closes them | `formal/`, `scripts/formal.sh` |
| L7 mutation | 823 one-line faults in the RTL, each run against the suite until something fails | `tools/mutate` |

## 3. Method: one contract, two independent implementations

- `docs/SEMANTICS.md` is the contract. `src/` (the RTL) and `tools/loomsim`
  (the golden model) were written from it in separate sessions, and the
  author of each was not allowed to open the other (`docs/VERIFICATION.md`,
  METH-1). Tests may read both.
- `isa/isa.yaml` is the only place an encoding lives; `tools/loomisa`
  generates the decoder module, the constants header, the ISA document and
  the formal decode properties from it, and CI fails if any is stale.
- Every bug found by a test, a proof or a mutant is a row in `docs/BUGS.md`
  with the check that caught it and the check that now covers it.
- Verification code was written by AI (Claude, directed from `docs/PLAN.md`)
  and reviewed by the owner; the split above is what keeps the two sides
  from agreeing by construction. Where a divergence between model and RTL
  was decided by quoting `docs/SEMANTICS.md`, the spec question and its
  resolution are in `docs/spec-questions/`.

## 4. Evidence by layer

[To be filled at the freeze from the tables in `docs/VERIFICATION.md`:
per layer, the checks, the counts, and the paragraph "what this layer would
miss". The state on 2026-09-22 is:]

- L0 static: Verilator `-Wall` lint on the TT top and the generated decoder;
  Yosys synthesis sanity; generated-file freshness; interface lists checked.
- L1 unit: 70 cocotb tests on the RTL modules, all of which also run on the
  netlist in CI's `gl_test`.
- L2 co-simulation: lockstep on every cycle (retire record, pads, `HOST_IRQ`,
  guard registers, periodic full state), both sides built from `CTRL.CAPS`
  so every built feature is exercised; 553 coverage bins, 33 empty with a
  recorded reason each; four of five hand-written mutants killed at the first
  seed.
- L3 protocol: 37 scenarios on the model and 29 on the RTL (the 8 left out
  are slow 115200-baud and mode-sweep cases, marked with the reason), with
  no divergence between the sides, including error cases (framing error,
  NACK, clock stretching).
- L4 formal: 19 properties (`formal/README.md` has engine, depth and time per
  property); SCHED-1..3, FIFO-1B and SPI-1A..C proved unbounded by `abc pdr`,
  the rest by k-induction or exhaustively; every group has cover points
  against vacuity; SCHED-2 was extended to the slice A encoder state on
  2026-09-22 and re-proved; ISO-1, thread isolation, proved unbounded the
  same day by `abc pdr` on a two-copy miter with the host debug port quiet
  (thread 0's whole state compared every cycle, whatever the other three
  threads run; 35 minutes; a depth-24 BMC cross-check); WAIT-1's completion
  rule proved and its bound checked to depth 40. The L4 list has nothing
  unattempted. Two more findings, F-4 and F-5, are wording corrections.
- M3 slice A (2026-09-22) as a worked example of the method: the golden
  model and the RTL of the bit-engine encoders, stuffing and differential
  output were written from one paragraph of SEMANTICS (6.9.1) by two agents
  that never saw each other's code; each recorded its spec questions (eight
  and nine) and the readings agree everywhere they overlap; the
  co-simulation, with the generator driving the new configuration bits,
  found no divergence in the default run or in a 40-seed sweep, with all 25
  new coverage bins hit.
- L5 physical: DRC, LVS and antenna clean; precheck clean; `gl_test` 71 of 71
  on the last full hardening (run 35524275302).
- L7 mutation: 823 mutants, 99.7 per cent killed with 44 documented
  equivalents set aside (`tools/mutate/equivalents.json` gives each its
  reason), every module at 98.8 per cent or better; one open survivor.

## 5. What verification found

The ledger in `docs/BUGS.md` has five rows at the time of writing; the
formal findings are F-1 to F-3 in `formal/README.md`; the mutation pass
found eight holes in the test suite. Per layer:

| Found by | What | Where recorded |
|---|---|---|
| L0 lint (CI) | width warnings in the M0 top | BUGS 1 |
| L5 gate-level (CI) | the template's gate-level Makefile omitted the PDK primitives file | BUGS 2 |
| RTL review against the spec, confirmed by a unit test | the `TICK_SEEN` rule lost a tick landing between a slot's X cycle and its commit (a spec bug: RTL, model and text fixed together) | BUGS 3, SEMANTICS 4 |
| L5 gate-level (CI), RTL-vs-netlist net diff | a test that never reset thread 1 ran unwritten memory; the netlist spread the X into a pin register where the RTL had hidden it | BUGS 4 |
| L4 formal, PIN-1 | **a real bug**: an open-drain pin could drive high on a shared bus, depending on the order three registers were written in | BUGS 5, D-023, F-2 |
| L4 formal, TIMER-1 | the property as worded was false: `reached` is a half-window compare, not monotone | F-1, SEMANTICS 4 |
| L4 formal, SPI-1 | a reset while SCK is high inserts a phantom edge; the host protocol now says to release CS_n around a reset | F-3, HOST_PROTOCOL |
| L7 mutation | eight promises no test compared (debug-register cross-talk, `CSRR TICK_FRAC`, `BE_CFG` readback, MISO idle level, CS_n rising mid-byte, `PIN_IN[15:13]`, `TICK_INT = 0` and long periods, `CTRL.RESET` setting TD) | VERIFICATION.md L7 table |

The one surviving non-equivalent mutant is in `loom_timer` and needs a
CTRL.RESET of a running thread, which the host protocol calls undefined; the
second survivor of the first pass was removed by D-028, which deleted the
logic it lived in, and TIMER-2 now proves the property it exposed.

## 6. Physical results

From `docs/AREA.md`, the last full hardening (run 35524275302, `main`
f081f4a, 2026-09-20): 28,842 standard cells plus one 512x16 SRAM macro on a
6x4 Tiny Tapeout block (utilisation 51.4 per cent), DRC, LVS and antenna
clean, precheck clean, gate-level tests 71 of 71. Timing at 20 ns: +5.98 ns
setup at the typical corner (the flow's sign-off corner), +10.92 ns fast,
and -2.48 ns at the slow corner (1.08 V, 125 C) on 23 endpoints of one
block, so the clock guaranteed over every corner is 44 MHz and the nominal
clock is 50 MHz with 6 ns of margin. D-028 (hardened 2026-09-23, run
35779039938) removed those 23 endpoints by deleting the host's thread
decode from the latch-fire path and routed fastest of any run (Metal3
overflow 1,248); the worst path is now the host debug thread select into a
pin register through weakly buffered logic, -6.36 ns at the slow corner
and +3.56 ns at the typical one, so the numbers the datasheet states are
50 MHz at the typical corner and 38 MHz over every corner, with D-031 (a
registered one-hot host thread select) proposed as the next structural
step. That a netlist which only lost logic also lost 2.4 ns of typical
margin is itself a finding about the flow's placement variance.

Routing time, not area, bounds the design: a two per cent increase in cells
in one corner of the floorplan tripled the global router's Metal3 overflow
and pushed detailed routing past GitHub's six-hour job limit (D-022, run
35470401774), which matters because Tiny Tapeout re-runs the flow at
submission. The design was returned to the shape that routes in under four
hours and every hardware change since is judged by that rule (D-025).

## 7. Known limitations and open items

- No hardware before silicon (section 2). L6 open.
- ISO-1 is proved for thread 0 with the host debug port quiet; the other
  three threads are re-runs of the same miter (35 minutes each, not yet
  done), and with the debug port in use the property is false by design
  (F-4: the port, the register-file write port and the staged-write port
  are shared, as HOST_PROTOCOL already states). WAIT-1's bound is bounded
  at depth 40, not proved unbounded.
- One mutation survivor open (section 5).
- The slow corner does not close at 20 ns (section 6); the datasheet states
  both clocks.
- Not attempted: 10 Mbit Ethernet, 10 Mbit Manchester (D-029).
- The bit-engine encoders, data memory and the M3 protocols are being built
  under the plan's M3; this report is updated with each slice.

## 8. What the AI did and how it was checked (METH-1, METH-2)

[To be written at the freeze: which artefacts were generated by AI sessions
and agents, which were reviewed by the owner and how, how the model/RTL
independence was enforced in practice (separate agents, briefs that forbade
reading the other side, spec questions resolved by quoting the text), and
the per-layer bug counts as the summary of METH-2.]

## 9. Reproducing the numbers

```
MSYS_NO_PATHCONV=1 wsl -d Ubuntu -- bash /mnt/c/Users/Thoma/asic/tt_um_loom/scripts/check_all.sh
bash scripts/formal.sh                       # in the WSL dev shell, from the repo root
python -m tools.mutate --help                # tools/mutate/README.md has the full pass recipe
bash scripts/harden_report.sh <run id>       # reads a CI hardening
```

## 10. Changelog

- 2026-09-22: started (D-030). Sections 1, 2, 3, 5, 6 and 7 describe `main`
  after the M2 review; sections 4 and 8 are outlines to be filled at the
  freeze.
