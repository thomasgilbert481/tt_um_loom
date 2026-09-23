# Decision log

Append-only. One entry per decision that changes the architecture, the ISA, the
verification plan, or the schedule. Newest at the bottom. Format: id, date,
who, decision, why, alternatives rejected, consequences.

## D-001 2026-09-15 Fable: barrel-threaded single pipeline, 4 threads

Decision: four hardware threads round-robin on one 4-stage pipeline, one
instruction per thread per 4 clocks, no hazards, no forwarding.
Why: four independent protocol engines for roughly 1.3x the area of one core;
timing determinism by construction; trivially formalisable scheduler.
Rejected: four PIO-style state machines with a shared flop instruction memory
(read-port muxes scale badly beyond 32 entries); one fast single-threaded core
(only one protocol at a time, and interrupt-free multiplexing is fragile);
8 threads (register file doubles, per-thread rate halves to 6.25 MIPS which is
too slow for USB low-speed framing).
Consequences: per-thread instruction rate is 12.5 MIPS at 50 MHz; anything
faster than about 2 Mbit/s per thread needs the bit engine.

## D-002 2026-09-15 Fable: 16-bit datapath, 16-bit instructions, 8 registers

Why: protocol data is bytes and short words; 16-bit instructions halve memory,
the dominant area item; 8 registers x 3-bit fields fit three-operand ALU ops.
Rejected: 32-bit instructions (memory cost), 4 registers (too few for framing
code with counters, data, masks, temporaries).

## D-003 2026-09-15 Fable: deadline-based timing (`WAITD`) with fractional ticks

Why: jitter-free periodic scheduling regardless of code path; removes PIO's
balanced-path burden; timed waits with timeout fall out of the same register.
Rejected: per-instruction delay field (costs 3 to 5 bits in every instruction;
kept `DLY` as an instruction instead); cycle counting by hand.

## D-004 2026-09-15 Fable: SPI slave host port on fixed pins, single clock domain

Why: every host (RP2040 on the TT board, Pico, FTDI, Raspberry Pi) has SPI;
sampling SCK keeps one clock domain, which simplifies STA and formal.
Rejected: UART host port (slower, framing ambiguity for binary data); parallel
bus (eats too many pins); SCK as a real clock (CDC everywhere).
Consequences: SCK <= clk/8; five pads reserved for the host.

## D-005 2026-09-15 Fable: instruction memory behind a wrapper with three options

Decision: `loom_imem.v` with MACRO / FLOPS-256 / FLOPS-128 options, decided at
the M2 gate from real hardening numbers.
Why: whether an SRAM macro exists for `ihp-sg13cmos5l` is unknown today; the
core must not depend on the answer.

## D-006 2026-09-15 Fable: per-thread bit engine with CRC-16, encoders, stuffing

Why: USB and CAN need NRZI/stuffing/CRC at bit rate; Manchester at 10 Mbit
needs an autonomous shifter; a 16-bit programmable CRC covers every target
except Ethernet FCS.
Rejected: shared single bit engine (threads would contend); CRC-32 per thread
(area); doing stuffing in firmware (too slow for USB at one slot per 8 bits).

## D-007 2026-09-15 Fable: `isa/isa.yaml` is the single source of truth

Decision: decoder constants, assembler tables, golden-model tables, ISA docs and
decode-completeness formal properties are generated from one file; CI fails on
stale generated files.
Why: the biggest AI-assisted design risk is silent drift between spec, model,
RTL and tests. Generation makes the drift impossible for the encoding layer.

## D-008 2026-09-15 Fable: verification layers L0..L8 with IDs, bug ledger, mutation testing

Why: reviewers should be able to see what each claim was checked by and what
the suite would miss. Mutation score is the honesty check on the testbench.

## D-009 2026-09-15 Fable: Verilog-2005 subset, cocotb, SymbiYosys, no Hardcaml

Why: Thomas's existing flow (TT template, cocotb, OSS CAD Suite in WSL,
iic-osic-tools Docker), the TT LibreLane linter, and Opus reliability all favour
plain Verilog. Hardcaml is acknowledged in the README as Jane Street's tool of
choice, and the generated-ISA approach gets some of the same benefit (one
description, many artefacts) without an OCaml toolchain on Windows.
Rejected: Amaranth (adds a generator layer to debug on top of the TT flow),
SystemVerilog features beyond what Yosys parses natively.

## D-010 2026-09-15 Fable: schedule anchored to the semester

Decision: RTL complete by 2026-11-30, feature freeze 2026-12-01, docs in
December, submit by 2027-01-11.
Why: ECE 464/564 final project and finals land in late November and Dec 3.

## D-011 2026-09-15 Fable: design to 6x4 tiles, not 8x4

Decision: `info.yaml` says `6x4`; the area budget in ARCHITECTURE section 13 is
written for 1289.28 x 710.64 um.
Why: the cmos5l flow's `tile_sizes.yaml` has no `8x4` entry and `project_info.py`
rejects unknown sizes; the blog post as fetched on 2026-09-15 says 6x4 is the
current maximum and 8x4 is being worked on. The copy Thomas pasted on
2026-09-14 said 8x4, so the page was edited after the announcement.
Rejected: waiting for 8x4 (blocks CI), designing for 8x4 and hoping (a design
that only fits in a size that may never exist).
Consequences: about 25 percent less area than first assumed; the SRAM macro
path matters more; upgrading to 8x4 later is a one-line change plus a re-budget.
Source: `docs/tt_cmos5l_facts.md` sections 2 and 6.

## D-012 2026-09-15 Fable: host SPI on ui_in[4] CS_n, ui_in[5] SCK, ui_in[6] MOSI, uo_out[7] MISO

Decision: moved from the v0.1 assignment (ui_in[7] SCK, ui_in[6] MOSI,
ui_in[5] CS_n). IN4 becomes ui_in[7] (pin index 12 unchanged).
Why: on the Tiny Tapeout demo board those pads are RP2040 GP17, GP18, GP19 and
GP16, which is exactly the SPI0 CSn/SCK/TX/RX function set, so the demo board
can drive Loom with the hardware SPI peripheral instead of PIO or bit-banging.
Costs nothing in the design.
Consequences: the v3 demo board (RP2350B) has a different map and uses PIO or
bit-bang; `PIN_IN` bit 12 comes from ui_in[7]. The M0 RTL is unaffected.
Source: `docs/tt_cmos5l_facts.md` section 5.

## D-013 2026-09-15 Thomas: solo entry, no teammates

Decision: Thomas works alone with the AI split (Fable directs, Opus implements).
Why: his call. Jane Street recommends teams, so the plan compensates: small
milestones that end green, scope cut at reviews rather than late, every
stretch protocol optional, USB low-speed the only stretch with a fixed slot.
Consequences: bench work and FPGA time are the scarce resource; the M2 gate
and the M4 freeze are hard dates, not aspirations.

## D-014 2026-09-15 Fable: SRAM macro path stays preferred, gated on precheck

Decision: after a Tiny Tapeout Discord reply reporting a community cmos5l
SRAM macro working with a macro-matched PDN, the macro remains the preferred
instruction memory, but the M2 gate now requires a hardening run that also
clears the TT precheck, which macros currently fail.
Why: the technique is credible and the organisers mention SRAM, so the
issues are likely to be resolved; but "likely" is not a submission. The
flop fallback stays live until a macro build passes precheck.
Source: `docs/tt_cmos5l_facts.md` section 9 (unverified community report).

## D-015 2026-09-17 Fable: SRAM trial pulled forward as M0.5 on branch `sram-smoke`

Decision: a standalone 512x16 macro smoke test runs through the TT flow by
2026-09-28, a month before the M2 gate. On that branch only, `src/config.json`
may gain the macro keys (`MACROS`, `PDN_MACRO_CONNECTIONS`, `PDN_CFG`,
`MAGIC_EXT_ABSTRACT_CELLS`, `ERROR_ON_MAGIC_DRC`, PDN pitch), which the
CLAUDE.md rule otherwise forbids.
Why: Matt Venn (Tiny Tapeout) said on 2026-09-16 that TT should handle
macros, that SG13G2 still has precheck DRC issues, and that CMOS5L is untried.
The open risk is the precheck, fixes on TT's side take calendar time, and only
a real failing run gives them something to fix.
Rejected: waiting for the M2 gate (leaves TT about ten weeks less), waiting
for Ken's config (may never be public).
Consequences: two sessions of effort in September; the wrapper written for the
smoke test becomes the MACRO backend of `loom_imem.v`; `main` stays on the
untouched template config until the M2 decision.

## D-016 2026-09-17 Fable: deadline-latched pin writes at M2

Finding (golden model, independent of the RTL): a thread acts only every 4
clocks, so a firmware-driven edge lands 0 to 3 clocks after its `WAITD`
deadline. There is no drift, and the edges are exactly periodic when
`k * period` is a multiple of 4 clocks, but at a 434-clock tick UART bit edges
alternate 432 and 436 clocks. The v0.1 documents claimed "zero jitter"; that
was wrong and has been corrected in SEMANTICS section 2 and ARCHITECTURE
section 6.
Decision: at M2, bit 0 of `SETP` (today a don't-care) becomes `D`. `SETP pin,
v, D` stages one pin write per thread (valid, pin, value); the hardware applies
it on the exact clock at which `NOW` becomes the deadline set by the thread's
next `WAITD` (immediately, if that deadline is already past when the `WAITD`
is first issued). `SETP TX, v, D` followed by `WAITD 1` then gives clock-exact
edges for any tick period, fractional ones included. Cost: about 7 flops and
one 16-bit equality per thread.
Rejected: making every pin write clock-exact (would need per-clock thread
execution, which is what the barrel pipeline trades away); a deferred `OUT`
group (32 more flops per thread; the bit engine covers multi-pin exactness).
Until M2: firmware that wants exact edges picks a tick period that is a
multiple of 4 clocks (432 or 436 for 115200 baud at 50 MHz, both within 0.5
percent).

## D-017 2026-09-17 Fable: default reset vectors are t * IMEM_WORDS / 4

Finding (golden model): `t * 0x100` masked to the memory size puts all four
threads at address 0 in the 256-word flop build.
Decision: `RESET_PC[t]` resets to `t * (IMEM_WORDS / 4)`. The assembler's
`.thread N` default origin follows the same rule through an `--imem-words`
option (default 1024, so existing programs are unchanged).

## D-018 2026-09-18 Opus 5: hardening runs only when the hardware changes

Finding: the first hardening of the real M1 core took 4 h 39 min, and the TT
precheck on it another 3 h 27 min, because the 6x4 block is 79 percent full
and detailed routing has only Metal1 to Metal4. Every push to any branch
started that eight-hour pipeline, including documentation and Python-only
commits.
Decision: `.github/workflows/gds.yaml` gains a `paths` filter (`src/**`,
`info.yaml`, `macro/**`, the workflow itself; `test/**` was in the first
version and was dropped the same day, because a test-only commit re-ran the
whole hardening of an unchanged design) and a `concurrency`
group that cancels a superseded run on the same branch. The jobs themselves are
the unchanged Tiny Tapeout template. This is the one sanctioned edit to that
file; `CLAUDE.md` says so.
Consequence: a commit that touches none of those paths has no GDS of its own.
Before submission, run the `gds` workflow manually (`workflow_dispatch`) on the
exact commit being submitted.

## D-019 2026-09-18 Opus 5: M1 critical path is the W-stage thread decode

Finding (post-route STA of CI run 35272974272): at the typical corner the
worst setup path is 17.8 ns of a 20 ns clock (+1.45 ns slack); at the slow
corner (1.08 V, 125 C) it fails by 9.9 ns on 1,084 endpoints. The worst path
starts at the W-stage thread register (`tr_thread`), is decoded through
NAND/NOR levels into per-thread write enables, and fans out to every piece of
per-thread state before a MUX4 into the destination flop. Several weak `_1`
gates on it drive large loads (one NOR3 alone takes 3.65 ns).
Decision (M2, after co-simulation lands so the two edits do not collide):
replace the decode with a one-hot ring counter for the W stage, which needs
no decode because the slot order is fixed, and replicate it per consumer
(register file, timers, PC/flags, pins) so no single net fans out to all
per-thread state. Target: +4 ns slack at the typical corner and a slow-corner
failure under 3 ns before M2 features are added. Re-measure after the change;
the flop instruction memory's 256:1 read mux is the next suspect.

## D-020 2026-09-18 Opus 5: instruction memory is the 512x16 SRAM macro (early M2 gate)

Evidence: the smoke test on branch `sram-smoke` (commit 565673f, Actions run
35377845679) takes `RM_IHPSG13_1P_512x16_c2_bm_bist` through the whole Tiny
Tapeout cmos5l pipeline: hardening, all nine precheck checks (the KLayout
SG13CMOS5L deck over the full macro hierarchy reports 0 violations), gate-level
test and viewer. Timing at 20 ns is clean at every corner. The recipe and its
two non-default pieces are in `docs/tt_cmos5l_facts.md` section 11: a power
script that releases the macro only while pdngen builds full-height stripes
through its power columns, and `ERROR_ON_ILLEGAL_OVERLAPS false` for five
Magic overlaps where those stripes cross a LEF obstruction that has no metal
behind it in the GDS.
Decision: the core's instruction memory becomes the macro (512 words, twice
the M1 flops, 45K um2 instead of about 350K um2 of flops and read mux), taken
through the existing `loom_imem` interface, as soon as the M2 RTL work frees
`src/`. The 256x16 latch array (ARCHITECTURE 12, option 3) stays as the
fallback behind the same interface, not built unless needed.
Why now rather than on 2026-10-26: the precheck risk the gate was waiting on
is retired, and every other M2 decision (area, timing, CI time) depends on
the memory.
Open conditions, tracked in PLAN: Tiny Tapeout's view of the waiver and the
power-script wrapper (question in the Discord thread), Jane Street's answer on
macros (email of 2026-09-17), and one 6x4 hardening of the real core with the
macro, placed on the stripe grid (macro x = 12 + 67.44k um, pin edge facing
free rows). If Tiny Tapeout rejects the waiver, the fallback is the latch array.

## D-021 2026-09-18 Opus 5: the core's hardening config takes the macro recipe (under D-020)

Decision: `src/config.json` of the real core (branch `macro-core`, for `main`)
gains the `sram-smoke` recipe (`docs/tt_cmos5l_facts.md` section 11), adapted
to the 6x4 block:
- `MACROS`: `RM_IHPSG13_1P_512x16_c2_bm_bist`, instance named by its flattened
  path `u_loom.u_imem.g_macro.u_macro.sram` (tt_um_loom, loom_top, loom_imem,
  its `g_macro` generate block, loom_imem_macro; checked with Yosys reading the
  sources as LibreLane does), orientation FS at (12, 40), libs keyed
  `*_typ_*`, `*_fast_*`, `*_slow_*`, the port blackbox as `nl`, the CDL as
  `spice`.
- `PDN_MACRO_CONNECTIONS` on the same path (`VDD!` and `VDDARRAY!` to VPWR,
  `VSS!` to VGND); `PDN_CFG` = `src/pdn_cfg.tcl`, the pdngen wrapper,
  unchanged from the smoke test.
- `MAGIC_MACRO_STD_CELL_SOURCE` PDK, `ERROR_ON_MAGIC_DRC` false,
  `ERROR_ON_ILLEGAL_OVERLAPS` false, `MAGIC_EXT_ABSTRACT_CELLS`
  `["RM_IHPSG13_.*"]`.
- Stripe grid: `FP_PDN_VPITCH` 50 -> 67.44, new `FP_PDN_VSPACING` 3.52 and
  `FP_PDN_VOFFSET` 26.36; `FP_PDN_VWIDTH` stays 2.1.
- `DRT_OPT_ITERS` is not carried over (the smoke test's cap of 12 was for a
  tiny design); every other key is the template's, unchanged.

Why: D-020 puts the macro in the core, and the CLAUDE.md rule on this file
allows only `CLOCK_PERIOD` and `PL_TARGET_DENSITY_PCT` without a decision
(D-015 covered `sram-smoke` only). The placement repeats the smoke test's
geometry relative to the core origin: the 6x4 floorplan
(`tt_block_6x4_pgvdd.def` on tt-support-tools `ihp-sg13cmos5l`: 186 rows of
2674 sites from (2.88, 3.78), die 1289.28 x 710.64) has the same core origin
as the 2x2, so the POWER stripes at 29.24 + 67.44 n (19 pairs across the core)
put four stripes per supply through the macro's same-net power columns exactly
as in CI run 35377845679, and the FS pin edge (y 231.34) faces 123 free rows.
The arithmetic is in the comments of `src/config.json`.
Rejected: another k in x = 12 + 67.44 k (valid for the stripes, but k = 0
keeps the whole right side and the top 475 um free and is the verified
geometry); y = 10 (would only move the 6 short rows under the macro to above
it, a negligible gain, on an untested placement).
Consequences: these keys now fall under the same rule as the rest of the file
(change only with a DECISIONS entry); the CLAUDE.md sentence on
`src/config.json` should say so. The illegal-overlap waiver would hide a new
overlap, so each hardening's `magic__illegal_overlap__count` must stay at the
smoke test's 5. The pitch goes from 50 to 67.44 um for the whole block (about
a quarter fewer stripes; IR drop is not checked by the flow). The macro's
A_DOUT clock-to-output (3.73 ns typical, 6.25 ns at the slow corner, against
roughly 0.5 ns for the M1 flop) now starts the D-stage paths; the first 6x4
hardening with the macro shows whether that matters at 20 ns.
Outcome (2026-09-19, run 35419160398): it does not; the worst path the macro
launches has +2.07 ns at the slow corner. The overlap count read 10, and the
rule is restated: what must not change is the set of crossings, not the
count. All 10 are the four POWER stripes crossing the macro's `obsm4` band at
y 186.1-192.3 um (x 28.2, 95.6, 163.1, 230.5 um, the 67.44 pitch), each
reported as two boxes split at y 190.95, the same crossings the smoke test
waived (facts 12). A hardening whose Magic feedback shows any other box
needs a look before the waiver covers it.

## D-022 2026-09-19 Opus 5: the deadline-latch fire logic picks among finished differences

Decision: `loom_timer` computes `lat_fire` (SEMANTICS 6.10 rules 1 and 2)
from three 16-bit differences per thread, `NOW - cm_td`, `NOW - h_wdata` and
`NOW - TD`, each straight from registers, and uses the TD write strobes
(commit, then host) and `tick` only as the selects of a final 1-bit mux. The
old code muxed TD' first (commit value, host value or TD) and subtracted
once.
Why: 22 of the 23 slow-corner setup violations of the first macro hardening
(worst -2.07 ns) start at `h_dbg_thread` and end at the pin registers: the
host write strobe `h_mine` (a thread-number compare) fanned out through
`buf_1` buffers with slews up to 1.85 ns into the 16-bit TD' mux, then the
subtract, the 15-bit AND and the staged pin write. Now the decode and `tick`
(itself a 25-bit compare) arrive at the last mux, in parallel with the
subtractors.
Evidence: Yosys `equiv_make`/`equiv_simple`/`equiv_induct` proves the new
module equivalent to the old one (964 `$equiv` cells, `lat_fire` included,
0 unproven); the RTL suite and co-simulation are unchanged.
Rejected: registering the host write strobes (moves the host TD write by a
clock, which HOST_PROTOCOL's E+4 rule fixes); leaving it to the flow's
resizer (it did not fix it, and the typical corner already passes, so the
flow has no reason to try).
Consequences: eight more 16-bit subtractors: Yosys generic synthesis of the
merged tree goes from 19,493 to 19,921 cells (+2.2 per cent), flops
unchanged at 3,028, longest path 62 -> 60 generic cells. Behaviour is
unchanged, so SEMANTICS, the model and the tests are untouched. The next
hardening shows what it buys at the slow corner.

**Outcome 2026-09-20: reverted, on routing cost.** The hardening of a33c403
(run 35470401774) never reached a slow-corner number: GitHub cancelled the
job at its six-hour limit, inside detailed routing. Against the same design
without D-022 (run 35419160398) the two per cent more cells tripled the
global router's congestion: Metal3 overflow 1,768 -> 5,776, total 1,804 ->
5,883, Metal4 usage 29.8 -> 35.0 per cent, and detailed routing went from
3 h 03 min (a 3 h 53 min gds job) to 5 h 15 min for the first pass alone,
with the antenna-repair pass still to run when the job died. Twelve 16-bit
subtractors per block, each tapping the shared `cm_td` and `h_wdata` buses,
is a lot of wire in one corner of the floorplan.
Tiny Tapeout re-runs this flow on submission, so a design that does not
harden inside six hours is a submission risk. The slow corner is not a
sign-off corner in this flow (`TIMING_VIOLATION_CORNERS` is `*typ*`) and the
typical corner passes with +6.21 ns, so the trade went the other way:
`src/loom_timer.v` is back to the one-subtraction version and the design is
back to the shape that hardened in 3 h 53 min.
If slow-corner closure is ever wanted, the cheaper shape to try first is two
subtractors per thread rather than three (`NOW - TD` for rule 1, and
`NOW - (w_cm ? cm_td : h_wdata)` for rule 2, so the host thread compare and
its fanout still stay out of the wide path and only a 2:1 mux driven by the
W-stage ring goes in front of the subtractor), together with cell padding or
a lower `PL_TARGET_DENSITY_PCT`, and one hardening to measure each step.

## D-023 2026-09-19 Opus 5: the pads apply OD_MASK, so an open-drain pin never drives high

Decision: `loom_pins` drives `uio_out = PIN_OUT[7:0] & ~OD_MASK` and
`uio_oe = PIN_OE & ~(OD_MASK & PIN_OUT[7:0])`. A BIDIR pin in open-drain mode
pulls low when its `PIN_OUT` bit is 0 and its `PIN_OE` bit is 1, and is
released otherwise. SEMANTICS 3 states the rule, the golden model applies it
in the same place, and the register views still read back what was written.
Why: the formal property PIN-1 ("an open-drain BIDIR pin never drives high",
VERIFICATION.md L4) was false (BUGS 5, formal finding F-2). SEMANTICS 6.3
makes every *pin write* open-drain-safe, but `OEP` and the host's `PIN_OE`
write carry no open-drain qualification and `OD_MASK` can be set after a pin
was driven high, so three ordinary slots (`SETP pin, 1`; `CSRW OD_MASK`;
`OEP pin, 1`) leave an open-drain pin driving high. On a shared bus that is
an electrical fault, not a firmware inconvenience: this chip exists to
bit-bang I2C and similar buses, and `firmware/i2c_master.loom` drives SDA and
SCL open drain.
Rejected: clearing `PIN_OUT` on the bits a write to `OD_MASK` turns on (a
later `CSRW PIN_OUT` sets them again, so the guarantee would still depend on
the order); leaving the silicon and weakening the property (the property is
the one a user of the chip needs).
Consequences: sixteen gates on a non-critical path. Firmware that switches a
pin into open-drain mode no longer has to clear `PIN_OUT` first. PIN-1,
PIN-1CORE and PIN-1R are ordinary proofs in `formal/pins.sby` now, and the
property that the pads are the plain register views (which they were) is
replaced by the gated rule. A hardening after this change should show the
same area to within a few cells.

## D-024 2026-09-21 Thomas: M2 drops its FPGA half; verification rests on simulation, gate level, formal and mutation

Decision: the FPGA prototype leaves M2's exit criterion and the plan. There is
no FPGA build of the full design before the submission, and L6 (FPGA and
bench) is out of scope until the Tiny Tapeout silicon arrives, which is after
the 2027-01-18 deadline.
Why: the full design does not fit the board on hand. Synthesis for the
iCEBreaker's iCE40UP5K needs 7,732 LUT4 against 5,280 (`fpga/icebreaker/`),
and no build knob or small trim closes the gap. A board that fits exists (an
ECP5-25F uses 32 per cent of its LUTs and closes at 47 MHz; the iCESugar-Pro
costs about $60 and ships from China, the ULX3S does not ship before December),
but it would buy a bench demonstration, not a check the project lacks: the
same RTL is already exercised by lockstep co-simulation against an
independent golden model, by 71 gate-level tests on the hardened netlist, by
18 formal properties, and by a mutation pass that kills 99.7 per cent of the
non-equivalent mutants.
Rejected: buying the ECP5 board (cost and a delivery that would push the
milestone for a demonstration); an FPGA-only reduced build, such as two
threads, which is an RTL change that stops the FPGA running the silicon's
design.
Consequences: M2 is complete. The write-up says plainly that nothing has run
on hardware before silicon and names the checks that stand in for a bench.
`fpga/icebreaker/` stays as the measured attempt and the pin-map template if a
board is bought later; the L6 check IDs stay in VERIFICATION.md, marked not
done, so the report lists them as open rather than dropping them silently.
The silicon, when it arrives, is the first hardware test: the bench scripts
L6 describes become the chip's bring-up plan.

## D-025 2026-09-22 Fable: every hardware change is gated by routing time, one change per hardening

Decision: an RTL change is accepted only if the hardening after it finishes
detailed routing in under 4 hours; the global-routing report's Metal3
overflow is the early warning, and a value above about 4,000 marks a change
that will not make it. One hardware change per hardening. Before a further
change is stacked on the design (slice C of D-026), the baseline must show
Metal3 overflow under 3,500 and detailed routing under 3 h 45 min. One
session is spent finding out whether LibreLane runs locally (the Docker image
in `CLAUDE.md`, the pinned PDK) to the end of global routing; if it does,
that run is the pre-check for every slice.
Why: GitHub kills a job at six hours and Tiny Tapeout re-runs the flow at
submission (`docs/AREA.md`, "Routing time is the binding constraint"). The
three measured points (Metal3 overflow 1,768, 3,169 and 5,776 against
routing of 3 h 03 min, 3 h 37 min and more than 5 h 15 min) with 50 to 70
minutes of non-routing work per job and about half an hour of placement
variance leave one to two hours of margin, so a change is judged by what it
does to routing, not to cell count.
Rejected: judging by cell count (D-022 was +2.2 per cent of cells and tripled
the overflow); raising `PL_TARGET_DENSITY_PCT` or adding cell padding ahead
of a need (each is a `src/config.json` change and its own six-hour reading).
Consequences: M3 is built as slices, each hardened alone (PLAN M3). Wide
buses tapped by per-thread logic are the pattern to avoid; narrow per-thread
control state and shared X-stage logic are the pattern to use. The final
`gds` run on the freeze commit is the last gate (D-030).

## D-026 2026-09-22 Fable: the bit engine is built as encoders and stuffing in manual mode first; auto mode, if built, acts in the thread's slot

Decision: the M3 bit-engine work is two slices. Slice A, built first: NRZI
and Manchester encoding on `SHO` and decoding on `SHI`, USB and CAN
stuffing and destuffing with the pending-stuff rule of ARCHITECTURE 8.1, the
stuffing-violation T flag, and a differential output bit (one `SHO` drives
`BE_PINS.out` and the next pin index complementary, through the group-write
path) for USB D+/D-. One copy of the logic in the X stage next to the
shifter and CRC, selected by `xsel`; per thread only the narrow state the
encoder needs. Manchester in manual mode is two `SHO` per bit with the engine
tracking the half-bit phase. Slice C, optional and last: auto mode in which
the engine acts in the thread's own slot when `MODE` and `TICK_SEEN` are set,
as an implicit `SHO`/`SHI` alongside the slot's instruction through the same
X-stage logic; the transmit edge goes through the thread's deadline latch
with the next tick edge as a third fire condition; the receive sample is the
slot's X-cycle read; `AUTOPULL`/`AUTOPUSH` use the thread's own FIFO port.
The PHASE field (BE_CFG 12:10) is dropped; its bits hold the differential
bit. Slice C is built only under D-025's stacking condition, before the
D-030 freeze, and if Thomas wants it.
Why: USB low-speed at 1.5 Mbit/s is eight slots per bit and the manual loop
is three, so the protocols the stretch list names need the encoder and the
stuffer, not an engine that runs by itself; without hardware NRZI and
stuffing the per-bit firmware is nine or ten slots. An autonomous per-thread
engine is four copies of the X-stage datapath, a third writer on every SR,
CNT and CRC flop and a second requester on every FIFO port: the wiring
pattern D-025 forbids, and it would falsify SCHED-2 as proved. The
slot-injected shape keeps every proved property and costs control bits.
Rejected: auto mode as specified in ARCHITECTURE 8 (per-thread autonomous
engines); a half-period tick for Manchester's mid-bit edge (a 24-bit compare
per thread, the wide pattern again).
Consequences: one engine action per slot, so 12.5 Mbit/s NRZ and about
6 Mbit/s Manchester at 50 MHz at most; 10 Mbit Manchester is not reachable
(D-029). SEMANTICS 6.9 gets the slice A text before any RTL, ARCHITECTURE 8
and 8.1 are updated, and `BE_CFG.MODE` reads 0 until slice C exists, as the
CAPS convention already provides.

Implemented, slice A, 2026-09-22 (Fable as director): the cycle-exact text is
SEMANTICS 6.9.1; the golden model and the RTL were written from it the same
evening by two agents in separate worktrees, neither allowed to open the
other's code (`docs/spec-questions/loomsim-m3a.md`, eight questions, and
`rtl-m3a.md`, nine; the readings agree everywhere they overlap and the
architect's rulings are appended to each). Slice A reports as `CAPS[9]`
(added at integration so the co-simulation harness builds the model from
CAPS as before) and `VERSION` 3. Model: 67 new tests; RTL: 8 new pin-level
tests with a Python transcription of 6.9.1 as the reference; the whole
suite is green (121 cocotb, 1,451 tool tests) and the co-simulation, whose
generator now drives ENC, STUFF and DIFF, finds no divergence between the
two implementations. Generic synthesis +686 cells (+3.6 per cent) and +61
flops, longest path unchanged (`docs/AREA.md`); the hardening after run
35779039938 is the D-025 reading, and slice A stays only if it routes
inside four hours.

Outcome 2026-09-23 (run 35812463112 on 8c487cc, `docs/AREA.md`): every
job passed; detailed routing 3 h 27 min, Metal3 overflow 2,924, gds 4 h 36
min, typ +4.80 ns, slow -4.04 ns, gate-level tests including the slice A
ones. Under D-025 slice A stays, and its baseline (under 3,500 overflow and
3 h 45 min of routing) lets slice B stack on it.

## D-027 2026-09-22 Fable: data memory is the instruction memory, reached by LD/ST through the thread's own fetch cycle

Decision: `LD` and `ST` are built as two-slot instructions on the 512-word
instruction memory. The first slot computes `ra + imm5` in X, holds PC, and
at its W edge loads one shared holding register (valid, write, 9-bit
address, 16-bit data); the thread's next F cycle, which is the cycle after
its W edge, presents that address (and write) to the memory instead of PC;
the D cycle receives the word; the second slot writes `rd` for `LD` and
advances PC. `isa.yaml` moves them from `one_slot` to a two-slot timing
class, the assembler's deadline checker counts two, and the assembler gains a
data directive. Data lives in instruction words a program does not use; the
host loads and dumps it through the IMEM space it already has.
Why: the device-emulation claim in ARCHITECTURE's differentiator table needs
memory a register-backed variant cannot supply (a 24C02 is 256 bytes), and
the two obvious implementations are out: a second macro repeats the PDN and
precheck risk of D-020 and a flop array is thousands of cells of the wiring
D-025 forbids. Because a thread's F cycle immediately follows its own W edge,
one holding register serves all four threads, no other thread's slot is
used, and SCHED-1, SCHED-2 and the host's halted-only IMEM rule hold
unchanged.
Rejected: a second SRAM macro; a flop or latch array; register-backed
16-byte emulation (too small to demonstrate anything).
Consequences: SEMANTICS 6.11 is written before the RTL; the cost is one
holding register, a wider mux at the macro port and one more source in the
register write mux, at the macro's pin edge, so its hardening is read under
D-025. OPEN-3 is resolved; OPEN-2 (FIFO depth) stays at 4; OPEN-4 (boot ROM)
and OPEN-5 (group-match wait, CRC-32) are closed as not built.

Amended 2026-09-22 at the model's spec question (`loomsim-m3b.md` item 1):
the held address and store word are **per thread** (26 flops each, 104 in
all, a 4:1 select of 26 bits at the port), not one shared register, and the
access is made in the F cycle of the thread's next **valid** slot. A shared
register is sound only while the access is consumed the cycle after it is
loaded, which is true of a running thread and false of one the host steps
through the first slot, and SEMANTICS 7 promises that stepping is
observably identical to running. For a running thread nothing changes: the
next valid slot starts at x+2. Also ruled with it: `tr_done` is 0 for the
first slot; the completion slot is a valid slot for `STEPS`, `TICK_SEEN` and
`PREV_PINS`; a debug write of 0x28 sets the three bits only; `CTRL.RESET`
and a debug PC write clear `MEM_PEND` only; HOST_PROTOCOL's space 2 is
reserved (the earlier dual-ported DMEM text predates this decision);
`VERSION` reads 4 whenever slice B is built.

Implemented, slice B, 2026-09-22 (Fable as director): golden model and RTL
from SEMANTICS 6.11 by two agents that never saw each other's code
(`docs/spec-questions/loomsim-m3b.md`, ten questions; `rtl-m3b.md`, nine),
the model's first question settled the held-access design above before the
RTL was finished. `CAPS[5]`, `VERSION` 4, debug 0x28. Model: 47 new tests;
RTL: 14 new pin-level tests, one of them a thread stepped across an access
while another streams `LD`/`ST`. The co-simulation generator gives every
running thread a data window of 8 words at the top of its region and emits
`LD`/`ST` into it, so the two implementations of 6.11 meet on random
programs from the first seed. Generic synthesis +593 cells and +136 flops in
`loom_core`, longest path unchanged (`docs/AREA.md`). Its hardening follows
slice A's; slice B stays only if it routes inside four hours (D-025).

## D-028 2026-09-22 Fable: the host's TD write leaves rule 2 of the deadline latch

Decision (proposed to Thomas at the M2 review; Thomas delegated the call the
same day, "proceed based off your best judgement"): SEMANTICS 6.10 rule 2 becomes
"TD is written at e by the thread (a `WAITD` first issue, `SETD` or
`CSRW TD`) and `reached(NOW', TD')` holds". A host debug write to `TD` still
writes `TD` but no longer fires a staged pin write; rule 1 (NOW ticks to
exactly TD) is unchanged and still fires on the new value at the exact tick.
In `loom_timer`, `td_new` and `td_w` lose their `h_mine && h_td_we` terms.
Why: 22 of the 23 slow-corner violations (-2.48 ns, run 35524275302) start
at the host thread decode and go through `h_mine`, its buffer chain and the
`td_new` mux into the subtractor (D-022's analysis). Removing the host term
takes the thread compare and `h_wdata` out of that cone and leaves a mux
selected by the D-019 ring. It deletes logic, so under D-025 it cannot cost
routing; it can only fail to close the corner, and one hardening says which.
It also settles the open mutation survivor at `loom_timer.v` line 112 by
construction and makes "a thread's staged write fires only on its own TD
writes" a provable property. The cost is a debug-only corner: a host `TD`
write while the thread is halted no longer applies a staged write.
Rejected: the flow knobs in `docs/AREA.md` (DELAY synthesis, post-GRT
resizer timing, timing-driven placement), each a whole-design change in cell
selection and a six-hour reading, for a corner that is not sign-off;
D-022's two-subtractor shape (still eight subtractors on shared buses).
Consequences: SEMANTICS 6.10, HOST_PROTOCOL's note on 0x0A and 0x25, the
golden model, `loom_timer`, a regression test, a formal property, and one
hardening, done first in M3 so the slices are read against the lower
baseline. Whatever the result, the datasheet states both clocks: 50 MHz at
the typical corner and the measured slow-corner clock.

Implemented 2026-09-22 (Fable, RTL and model in one commit as D-023 was):
`loom_timer` loses the host terms of `td_w` and `td_new` (a pure deletion;
rule 1 at a host write's own edge compares against the TD before the edge,
which the text now says); the golden model treats a host TD write as a plain
write and evaluates rule 1 against the pre-edge TD; `test_setpd.
test_setpd_debug_latch_and_host_td` now shows a reached and an unreached
host TD write both leaving the staged write in place and a host-written
deadline landing by rule 1 on the tick edge (`w + 8000` at 4000 clocks per
tick); `test_loomsim_setpd` gains `test_a_host_td_write_is_not_a_rule_2_write`
and re-pins the loading-edge reading through rule 1; formal TIMER-2 proved
(`timer.sby:prove`, k-induction, 1 s) with three new cover points. The
hardening of this commit is the slow-corner measurement.

Outcome 2026-09-23 (run 35779039938, `docs/AREA.md`): the change did what
it promised for the latch-fire cone (its 22 slow-corner endpoints are gone)
and routed fastest of any run so far (Metal3 overflow 1,248, detailed
routing 2 h 57 min, gds 4 h 05 min), but the corner did not close: the
worst path is now host debug thread select to `pin_oe_reg[6]` through a
weakly buffered chain, -6.36 ns at the slow corner and +3.56 ns at the
typical one (was +5.98 for a netlist that only lost logic, so the earlier
margin was partly placement luck). The datasheet states both clocks, 50
MHz at the typical corner and 38 MHz over every corner, and D-031 below is
the structural follow-up.

## D-029 2026-09-22 Fable: stretch scope: USB low-speed stays, CAN is firmware, 10 Mbit Manchester is cut as a target

Decision: USB low-speed device keeps its slot, in manual mode at 1.5 Mbit/s
on slice A, with a Python host model and enumeration to SET_ADDRESS plus one
HID report in simulation. CAN is firmware after slice A (loopback with
stuffing and CRC15); if the CAN stuffer variant costs more than a handful of
cells, the hardware variant is dropped and firmware stuffs. 10 Mbit
Manchester at 60 MHz is no longer a target; the Manchester encoder stays and
`L3-MANCH` becomes a loopback at the manual-mode rate. 10 Mbit Ethernet is
not attempted. WS2812, PS/2 host, JTAG master, SWD master and the I2C slave
EEPROM (on D-027) stay as M3 firmware.
Why: 10 Mbit Manchester was the one protocol that needed the autonomous
engine D-026 rejects, it needs a transceiver the project does not have, and
with D-024 there is no bench for it either; it would have been a number in a
simulation log. USB low-speed is the brief's named stretch and slice A is
exactly its hardware. The four firmware-only protocols are cheap and are the
evidence for the "reprogrammable after fabrication" claim.
Rejected: keeping 10 Mbit Manchester as a reason to build the autonomous
engine; dropping CAN (it costs nothing once the USB stuffer exists).
Consequences: PLAN M3 and M4 rewritten; the write-up lists Ethernet and
10 Mbit Manchester as not attempted and says why.

## D-030 2026-09-22 Fable: RTL freeze 2026-11-08; the write-up starts now

Decision: the RTL is frozen on 2026-11-08 instead of 2026-12-01. The final
`gds` run is made by hand on the freeze commit and must finish inside six
hours; if it does not, the last slice comes out and the run repeats. After
the freeze, RTL changes only for a bug found by verification, each with a
re-hardening. Firmware, tests, tools and docs continue to 2026-12-01 (they
trigger no hardening, D-018). `docs/VERIFICATION_REPORT.md` is started now
as a living document whose first section is D-024's statement. ISO-1 is
attempted in M3, timeboxed to two sessions on a reduced configuration, and
reported as bounded if that is what it is. If the `gds` artefact includes
SDF, one timing-annotated gate-level run of the L3 suite at the typical
corner is added to the report (unverified that it does; check first).
Why: the plan's calendar puts course deadlines in late November and finals in
the first week of December, so a 2026-12-01 freeze lands the last hardening
where there is least time to react to a killed run; M2 finished four weeks
early, so 2026-11-08 still leaves seven weeks for M3's slices; with no bench
the write-up is the evidence and must describe one netlist; and the RTL
freeze does not stop the protocol list growing.
Rejected: keeping 2026-12-01 and spending the difference on another protocol
(after D-029 there is no RTL-bearing protocol left to spend it on).
Consequences: PLAN M3 ends 2026-11-01 with the slice C decision, M4 is the
freeze and the evidence, M5's demo material is listings, waveforms, the
gate-level log and the mutation table, with no video.

## D-031 2026-09-23 Fable, proposed: a registered one-hot thread select for the host debug port

Proposal (after slices A and B have their D-025 readings; one hardening of
its own): the host side stops decoding `h_dbg_thread` with a 2-bit compare
on every per-thread write enable and every debug read select, and instead
loads a one-hot `h_sel[3:0]` register from the ADDR byte cycles before the
data word's write strobe, the host-side twin of D-019's W-stage rings.
Why: after D-028 the worst path at both corners still starts at
`h_dbg_thread` (`docs/AREA.md`, the slow corner after D-028), and the
typical corner's margin is down to +3.56 ns with two slices of logic still
to harden. Four flops and less fanout; the E+4 commit rule of
HOST_PROTOCOL is unchanged because the thread field is known long before
the strobe. Rejected for now: flow knobs (each a whole-design change in
cell selection); doing it inside the slice A or B hardening (D-025 wants
one change per reading).
