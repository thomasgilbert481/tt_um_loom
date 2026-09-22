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
