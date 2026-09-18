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
