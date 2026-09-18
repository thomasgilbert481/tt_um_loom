# Loom project plan

Owner: Thomas Gilbert. Direction: Fable 5.1 (this document, the architecture,
the verification plan, milestone reviews). Implementation: Opus 5 sessions in
this repo, following `CLAUDE.md`. Written 2026-09-15.

Hard deadline: **2027-01-18** (Jane Street submission). Internal deadline:
**2027-01-11**. Shuttle: Tiny Tapeout IHP CMOS5L, March 2027.

## Calendar constraints

- The owner is a full-time student. Course deadlines cluster in late November
  and finals are in the first week of December, so RTL is frozen before then.
- Any overlap with coursework needs the instructor's written approval first;
  by default this project and course projects are separate deliverables.
- Winter break (about Dec 12 to Jan 10) is the writing and polish window, not
  the RTL window.

## Milestones

Each milestone has an exit criterion that a reviewer can check without asking.
Dates are Sundays. Slips are allowed; scope cuts happen at the review, not
silently.

### M0: flow proven (by 2026-09-21)

Exit: CI green on GitHub for the trivial design, local sim works in WSL.

- [x] 2026-09-17, Thomas: Jane Street sign-up form submitted.
- [x] 2026-09-17: public repo `thomasgilbert481/tt_um_loom` created and pushed
      (Apache-2.0; first commit is the untouched template, second is Loom),
      GitHub Pages enabled with the workflow build type for the viewer job.
- [x] Template imported from `ttihp-verilog-template@cmos5l`
      (upstream commit b86a2a7). `info.yaml`: tiles **6x4**, top `tt_um_loom`.
      `8x4` is not a valid size in the cmos5l flow yet (largest is 6x4; see
      `docs/tt_cmos5l_facts.md`), and the live blog post now says 6x4 is the
      current maximum with 8x4 in progress.
- [x] 2026-09-17, Thomas: sent; awaiting reply, follow up 2026-09-29. Was:
      email asic-competition@janestreet.com with two questions: will
      `8x4` be enabled in the cmos5l flow before the deadline, and are IHP SRAM
      macros acceptable in a submission for the March 2027 shuttle. The M2
      memory decision needs the second answer. Draft and a matching Tiny
      Tapeout Discord post are in the vault under `60-applications/outreach/`
      (drafted 2026-09-15, follow up 2026-09-25 if no reply).
- [x] `src/tt_um_loom.v` M0 version = hard-wired UART transmitter that
      sends "LOOM\r\n" at 115200 baud on OUT0 whenever IN0 is high, with a
      cocotb test that decodes it. This is the blog's own "start by getting a
      UART transmitter out of a pin" step and it exercises the whole flow.
- [x] 2026-09-17, first CI run (35244479499): `test`, `lint`, `docs`, `gds`
      hardening on 6x4, precheck and viewer all pass. `gl_test` failed on a
      template bug, not the design: the GL Makefile omitted the PDK primitives
      file `sg13cmos5l_udp.v` (BUGS #2). Fixed and verified locally at gate
      level on the CI netlist (2 of 2 pass). Second CI run (35248880086) is
      fully green: gds, precheck, gl_test, viewer, test, lint, docs. Baseline
      numbers in `docs/AREA.md`.
- [x] WSL dev loop documented in `CLAUDE.md` and working (`make` in `test/`,
      Verilator lint, `make GATES=yes` with a scratch copy of the cell models).
      `test/requirements.txt` stays at the template's cocotb 2.0.1, which CI
      uses; the tests pass on 2.0.1 (CI) and 2.1-dev (WSL).

### M0.5: SRAM smoke test on cmos5l (by 2026-09-28, runs alongside the start of M1)

Exit: a minimal design containing `RM_IHPSG13_1P_512x16_c2_bm_bist` has been
pushed through the TT `gds` workflow on branch `sram-smoke`, and the full
precheck output is posted in the Tiny Tapeout Discord thread, pass or fail.

Why now: on 2026-09-16 Matt Venn (Tiny Tapeout) said TT should be able to
handle macros, that precheck DRC issues still exist on SG13G2, and that he has
not tried CMOS5L. Any fix on their side takes calendar time, so the first
failing run has to exist in September, not at the October gate. Being the
first public cmos5l SRAM example is also worth doing in its own right.

- [x] 2026-09-17 Opus: branch `sram-smoke`. Vendored the macro views (GDS, LEF,
      the three lib corners, CDL, Verilog model plus the shared behavioural
      core it instantiates) from IHP-Open-PDK `2bbec755` into
      `macro/RM_IHPSG13_1P_512x16_c2_bm_bist/`, 1.1 MB total, every file's git
      blob hash matching upstream, README naming commit and Apache-2.0 licence.
      Port names and polarities read out of the vendored model.
- [x] 2026-09-17 Opus: `src/loom_imem_macro.v` wrapping the macro behind the
      `loom_imem` interface (this file becomes the MACRO backend later), BIST,
      bit-mask and `A_DLY` pins tied off as the model requires, plus the
      pin-level tester in `src/tt_um_loom.v` (byte registers over `uio`, edge
      strobes on `ui_in`) that can write and read every word through the TT
      pins. Tiles cut from 6x4 to 2x2 on this branch.
- [x] 2026-09-17 Opus: cocotb test: reset/pin hygiene, walking ones on all 16
      data bits, a distinct word written to and read back from all 512
      addresses, random interleaved access, write-wins-over-read. 5/5 pass on
      Icarus 14 / cocotb 2.1 in 2.2 s. Verilator `-Wall` clean; Yosys
      `synth -top tt_um_loom` gives 74 cells, 53 flops and exactly one macro
      instance.
- [x] 2026-09-17 Opus: `src/config.json` macro keys from the
      `tt_um_urish_sram_test` recipe (`MACROS`, `PDN_MACRO_CONNECTIONS`,
      `PDN_CFG`, `MAGIC_MACRO_STD_CELL_SOURCE`, `MAGIC_EXT_ABSTRACT_CELLS`,
      `ERROR_ON_MAGIC_DRC: false`) plus `src/pdn_cfg.tcl`. Allowed on this
      branch only, under D-015. **The reference's Metal4 connection does not
      transfer from SG13G2**: on cmos5l `FP_PDN_VERTICAL_LAYER` is Metal4
      itself and TopMetal1 belongs to `tt_top`, so the stripes are pitched
      (67.44 = 6 x 11.24 um) and offset (26.36) to sit *inside* the macro's
      Metal4 power pins instead of crossing them from above. Full derivation in
      `src/pdn_cfg.tcl`.
- [x] 2026-09-18 Opus: CI runs 4 to 8 on `sram-smoke`. Run 8 (`565673f`,
      Actions run 35377845679) passes `gds`, `precheck` (all nine checks,
      KLayout SG13CMOS5L DRC 0 violations over the full macro), `gl_test`
      (5/5, all 512 words at gate level) and `viewer`. Recipe, failure table
      and root causes in `docs/tt_cmos5l_facts.md` section 11: macro FS at
      (12, 40), stripes full height through the macro's same-net Metal4 power
      columns (the precheck pin check needs every power port to span the
      block), one documented Magic illegal-overlap waiver.
- [ ] Thomas: post the result in the Discord thread (draft ready outside the
      repo), record the outcome in `docs/AREA.md`.
- [ ] If Ken's config becomes public, diff it against ours before a second try.

Timebox: two sessions. If it fails for reasons on Tiny Tapeout's side, stop,
report, and wait for their fix. M1 proceeds on the flop memory regardless, and
the M2 gate rules do not change.

### M1: core runs firmware (by 2026-10-05)

Exit: a UART TX written in Loom assembly runs on the RTL core, decoded by the
same cocotb UART model as M0; first synthesis numbers recorded.

- [x] 2026-09-17: `docs/SEMANTICS.md`, the cycle-exact contract for RTL and
      golden model (slot timing, visibility rules, ticks, waits, pins, run
      control, retire record).
- [ ] `isa/isa.yaml` v1.0 frozen. Every mnemonic, encoding, flag effect, and
      timing class from `docs/ARCHITECTURE.md` section 11. Frozen means changes
      need a `DECISIONS.md` entry. (Now 0.2.0: 64 instructions, no overlaps,
      75.8 percent of the encoding space used.)
- [x] 2026-09-17: `tools/loomisa`, the only parser of `isa.yaml`: encode,
      decode, overlap check, and generators for `src/loom_decode.v` (the
      decoder module itself, so RTL never hand-decodes), `src/loom_isa.vh` and
      `docs/ISA.md`. 29 tests. CI job `isa-and-tools` fails on stale files.
- [ ] `tools/loomasm`: two-pass assembler, labels, `.thread`, `.pins`, pseudo-ops,
      listing output with slot counts, deadline check (warn when code between
      two `WAITD`s cannot fit the tick interval).
- [ ] `tools/loomsim`: Python golden model, slot-accurate at the thread level,
      with the same debug-state dump format the RTL exposes.
- [ ] RTL: `loom_core` (scheduler, fetch, decode, regfile, ALU, branches,
      CALL/RET, flags), `loom_timer`, `loom_pins` (SETP/OUT/IN/JP/WAITP/WAITE/
      WAITD/SETD/DLY, OD mode), `loom_imem` FLOPS option, minimal host path
      (enough to load imem and set RUN; full SPI is M2).
- [ ] Verification L1 unit tests for regfile, ALU, timer, pins; L2 directed
      tests generated from `isa.yaml` (at least one per mnemonic) passing on
      Icarus; first constrained-random co-sim run (1000 programs) passing.
- [ ] `firmware/uart_tx.loom` passes the L3 UART test at 115200 and 1 Mbaud.
- [ ] First hardening run (GitHub `gds` workflow) with the flop imem: record
      cells, utilisation, worst slack at 20 ns in `docs/AREA.md`.
- [ ] Fable review: ISA freeze, area numbers, decide FIFO depth.

### M2: host interface and the three required protocols (by 2026-10-19)

Exit: UART RX/TX, SPI master and slave, I2C master all pass their L3 tests with
data moving through the SPI host port; FPGA prototype runs the same tests.

- [ ] `loom_spi_host` + `loom_host_ctl`: full `docs/HOST_PROTOCOL.md` including
      the debug space, single-step, IRQ.
- [ ] `loom_fifo` x 8, `PUSH`/`POP`, `WAITB`, `SIG`/`CLR`/`WAITS`.
- [ ] Bit engine manual mode: `SHO`/`SHI`, SR/CNT, CRC with presets, NRZ only.
- [ ] `tools/loomhost`: transport-agnostic host library with `SimTransport`
      (cocotb) and `PicoTransport` (Raspberry Pi Pico as USB-to-SPI bridge;
      firmware in `tools/loomhost/pico/`).
- [ ] Firmware: `uart_rx`, `spi_master`, `spi_slave`, `i2c_master`. Each with an
      L3 test against a Python reference model, including error cases (framing
      error, NACK, clock stretching).
- [ ] FPGA: `fpga/icebreaker/` build with Yosys + nextpnr, host over Pico SPI,
      L3 tests re-run on hardware through `loomhost` (same scripts).
- [ ] Memory decision gate (2026-10-26): trial hardening of
      `RM_IHPSG13_1P_512x16_c2_bm_bist` (16 bits wide, 236.80 x 191.34 um)
      following the `tt_um_urish_sram_test` recipe (`MACROS`, custom
      `pdn_cfg.tcl` with the Metal4 macro connection, `MAGIC_EXT_ABSTRACT_CELLS`,
      `ERROR_ON_MAGIC_DRC: false`) with the PDN stripe pitch tuned to the
      macro's power-pin pitch (the approach a community member reportedly has
      working on cmos5l; ask "Ken" on the TT Discord for his config once
      public), alongside the FLOPS-256 run. The gate is only passed by a run
      that also clears the TT **precheck**, which macros currently fail; if
      precheck is still failing at the gate, take FLOPS and keep the macro
      branch alive until M4. Results in `docs/AREA.md`; Jane Street's answer
      on macro acceptance in hand; record the pick in `docs/DECISIONS.md`.
- [ ] Fable review: protocol coverage, host protocol, area and timing.

### M3: bit engines, device emulation, formal (by 2026-11-09)

Exit: I2C slave EEPROM emulation, WS2812, PS/2 and JTAG/SWD programs pass;
formal properties SCHED, FIFO, TIMER, SPI, PIN proven; mutation score
reported.

- [ ] Bit engine complete: auto mode, NRZI, Manchester, USB and CAN stuffing,
      sample phase, stuffing-violation T flag.
- [ ] Data memory decision implemented (LD/ST) if chosen at M2.
- [ ] Firmware: `i2c_slave_eeprom`, `ws2812`, `ps2_host`, `jtag_master`
      (IDCODE read), `swd_master` (DPIDR read), each with L3 tests.
- [ ] Formal: `formal/` SymbiYosys jobs for SCHED-1..3, FIFO-1..3, TIMER-1,
      SPI-1, PIN-1, ISA-1 (decode completeness generated from `isa.yaml`).
- [ ] `tools/mutate`: mutation testing over `src/`, kill ratio reported in CI
      artefacts; target 90 percent, survivors triaged in `docs/BUGS.md`.
- [ ] Boot ROM demo if area allows (OPEN-4).
- [ ] Fable review: what stretch goals are affordable in time and area.

### M4: stretch protocols and closure (by 2026-11-30, feature freeze Dec 1)

Exit: at least one of USB LS or CAN passes an L3 test; 10 Mbit Manchester
loopback at 60 MHz in simulation; STA clean at 20 ns on the final netlist; GL
simulation passes the full L3 suite.

- [ ] USB low-speed device: enumeration as a HID with a Python host model
      (tokens, CRC5/16, NRZI, stuffing, EOP, handshake). Stretch within stretch:
      real enumeration on a PC through the FPGA with resistor-level D+/D-.
- [ ] CAN frame TX and RX loopback with stuffing and CRC15.
- [ ] 10 Mbit Manchester TX/RX loopback in auto mode at 60 MHz (sim only; real
      10BASE-T needs a transceiver and is documented as future work).
- [ ] Timing closure: worst negative slack 0 at 20 ns, report at 16.7 ns.
- [ ] Gate-level simulation (TT `gl_test` job) runs the L3 suite.
- [ ] Feature freeze 2026-12-01. Bug fixes only after that.

### M5: documentation and submission package (by 2026-12-20)

- [ ] `docs/info.md` (Tiny Tapeout datasheet page) complete with pinout,
      how-to-test, external hardware.
- [ ] `README.md`: architecture, why it is different, results table (area,
      clock, protocols, coverage, mutation score, formal properties), how to
      build, how to program, honest limitations.
- [ ] `docs/VERIFICATION_REPORT.md`: numbers, bug ledger, what AI did and how
      it was checked.
- [ ] Demo material: FPGA video of two or three protocols on real devices,
      waveform screenshots, the assembler listing with slot annotations.
- [ ] Tag `v1.0-rc1`; final `gds` run archived as a release asset.

### M6: submit (by 2027-01-11)

- [ ] Fresh clone builds green: `test`, `gds`, `docs`, precheck, `gl_test`.
- [ ] Submit through the Jane Street form when it appears (they said it will
      be added to the blog page), and to Tiny Tapeout as instructed.
- [ ] Tag `v1.0`. Buffer until 2027-01-18.

## Roles

- **Thomas**: owner. Signs up, owns the GitHub repo and CI, runs the FPGA and
  bench work, reviews every ISA change, writes the human parts of the writeup,
  makes the M2 memory decision, does all Tiny Tapeout and Jane Street
  communication (`asic-competition@janestreet.com`).
- **Opus 5 sessions**: implement RTL, tools, firmware, tests, formal, in the
  order above. Read `CLAUDE.md` first every session. Never change
  `isa/isa.yaml` after M1 without a DECISIONS entry. Log every bug found by
  verification in `docs/BUGS.md` with the layer that caught it.
- **Fable 5.1**: milestone reviews (M1, M2, M3), architecture changes,
  disputes. Ask for a review by opening a session with "Loom Mx review".
- **Team**: none. Thomas is working solo (decided 2026-09-15, D-013). Jane
  Street recommends teams, so the plan compensates by keeping the AI split
  strict, cutting scope at reviews rather than late, and treating every
  stretch protocol as optional. If help appears later it goes on firmware and
  bench work first.

## Risks and what we do about them

| Risk | Likelihood | Mitigation |
|---|---|---|
| Instruction memory too large in flops | high | M1 synth numbers early; three imem options behind one wrapper; 128-entry fallback |
| SRAM macro on cmos5l: reportedly working in the community with a macro-matched PDN, but macros currently fail the TT precheck and no cmos5l example is public | medium | decided by M2 gate from a real hardening run that includes precheck, plus Jane Street's answer; FLOPS fallback stays live until macros pass precheck; not assumed anywhere in the core |
| Solo project: no second pair of hands for bench work or reviews | certain | milestones small and green at every session end; scope cut at reviews, never late; stretch protocols optional; Fable reviews are the second opinion |
| Area is 6x4, not the 8x4 the brief first described (about 25 percent less) | certain today | budget written for 6x4; 8x4 is a one-line upgrade if it appears |
| One fewer routing layer than SG13G2 (`RT_MAX_LAYER = Metal4`) | medium | keep density at or below 60 percent; check congestion at M1 hardening |
| Register-file read mux limits clock | medium | close at 60 MHz target in STA; if needed, split X into two cycles per slot (5-stage, still hazard-free with 4 threads only if we add one bubble; prefer shrinking to 6 registers per thread instead) |
| LibreLane linter (Verilator) rejects code | medium | `verilator --lint-only -Wall` in CI from day one |
| Host SPI sampling errors at high SCK | low | spec SCK <= clk/8; formal SPI-1; test at the limit |
| Scope creep on stretch protocols | high | feature freeze Dec 1; USB LS is the only stretch with a fixed slot |
| Semester crunch (564 project, finals) | high | RTL done by Nov 30; December is docs; buffer week before the deadline |
| Model rate limits stall sessions | medium | milestones are small; every session ends with a committed, green state; `CLAUDE.md` explains how to resume |
| TT precheck failures (pin hygiene, unused signals, `ena`) | medium | run precheck on every push from M0 |
| Verification looks impressive but is hollow | medium | mutation testing and a public bug ledger; reviewers can see what was caught and by what |

## Deliverables at submission

1. Public repo, Apache-2.0, green CI, GDS built by the TT action.
2. `docs/info.md` datasheet, README with results, `docs/ARCHITECTURE.md`,
   `docs/ISA.md`, `docs/HOST_PROTOCOL.md`, `docs/VERIFICATION_REPORT.md`,
   `docs/DECISIONS.md`, `docs/BUGS.md`.
3. Tools: assembler with deadline checker, golden model, host library with
   sim/Pico/TT-board transports, mutation tool, codegen.
4. Firmware library: UART, SPI, I2C (master and slave), WS2812, PS/2, JTAG,
   SWD, plus USB LS and CAN as far as they got, each with tests.
5. FPGA build and a short demo video.

## Session log

Newest at the bottom. One line per session: date, model, what changed, next step.

- 2026-09-14/15, Fable 5.1: project started. Template imported, architecture
  v0.1, plan, verification plan, host protocol, decision log, ISA seed,
  `CLAUDE.md`, M0 UART-TX RTL + cocotb test (passing in WSL, Verilator lint
  clean), `lint` workflow, `firmware/uart_tx.loom`. Nothing committed yet.
  Next: Thomas signs up and creates the GitHub repo; first Opus session
  confirms M0 in CI and starts M1 with `tools/gen` from `isa/isa.yaml`.
- 2026-09-15, Fable 5.1 (research agent on Opus): `docs/tt_cmos5l_facts.md`
  written from primary sources. Consequences applied: tiles 6x4 (8x4 invalid in
  the flow today), host SPI moved to ui_in[4..6] to match the RP2040's SPI0,
  memory options rewritten with real macro sizes, budget rewritten for 6x4 and
  the Metal4 routing limit. Nothing committed yet.
- 2026-09-17, Fable 5.1: Discord thread outcome recorded (community macro
  reportedly working; Matt Venn: TT should handle macros, precheck DRC issues
  remain on SG13G2, CMOS5L untried). Added M0.5 SRAM smoke test and D-013,
  D-014, D-015. Solo entry noted. Repo still has no commits and no GitHub
  remote: M0 push is the next action, due 2026-09-21.
- 2026-09-17, Fable 5.1: published the repo through Thomas's account at his
  request. First CI run green except `gl_test` (template Makefile missing the
  PDK UDP file, BUGS #2); fixed, verified locally at gate level, pushed. M0
  baseline: 214 synthesised cells, 33 flops, 0.52 percent utilisation on 6x4,
  +13.2 ns setup slack at 20 ns, DRC/LVS/antenna 0, precheck pass. Remaining
  M0 items are Thomas's: sign-up form and the Jane Street email. Next session
  (Opus): M0.5 SRAM smoke test branch, then M1 from `isa/isa.yaml`.
- 2026-09-17, Opus 5 (branch `sram-smoke` only, nothing on `main`): M0.5 built.
  Macro vendored and hash-verified, `loom_imem_macro.v` + pin-level tester,
  cocotb 5/5, Verilator clean, one macro instance in the Yosys netlist, 2x2
  tiles, macro keys and a cmos5l-specific `pdn_cfg.tcl` in `src/`. Key finding
  for the Discord write-up: **the SG13G2 recipe's "Metal4 to TopMetal1" macro
  connection cannot be used on cmos5l**, because there the PDN's vertical layer
  is Metal4 (the macro's own pin layer) and TopMetal1 is what `tt_top` uses to
  feed each user block; the only route is to align the Metal4 stripes inside
  the macro's Metal4 pins, which is exactly what the community report meant.
  Next: Thomas pushes `sram-smoke`, collects the `gds` and precheck logs
  (expect the precheck to be the failure point: the reference project's own
  `gds` job is green today while its `precheck` job fails), and posts them.
- 2026-09-18, Opus 5 (branch `sram-smoke` only): M0.5 result. The macro passes
  the whole cmos5l pipeline (run 8, `565673f`): `gds`, `precheck` with the
  KLayout SG13CMOS5L deck clean over the macro internals, `gl_test`, `viewer`.
  The blockers were ours, not TT's: the pin edge facing the die floor
  (routing), a macro that pdngen left unpowered, and the precheck pin check,
  which needs every Metal4 power port to span the block, so the stripes now run
  through the macro's own power columns. Details, run table and the placement
  rule for M2 in `docs/tt_cmos5l_facts.md` section 11. Next: Thomas posts the
  result on Discord; the M2 memory decision can now treat the macro as viable.
