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

- [ ] Thomas: fill in the Jane Street sign-up form (link in the blog post). Not
      a commitment, but it gets template and submission updates.
- [ ] Thomas: create the public GitHub repo `thomasgilbert481/tt_um_loom`
      (Apache-2.0), push the template import plus these docs, enable GitHub Pages
      for the `gds` viewer job. See `CLAUDE.md` for the exact commands.
- [x] Template imported from `ttihp-verilog-template@cmos5l`
      (upstream commit b86a2a7). `info.yaml`: tiles **6x4**, top `tt_um_loom`.
      `8x4` is not a valid size in the cmos5l flow yet (largest is 6x4; see
      `docs/tt_cmos5l_facts.md`), and the live blog post now says 6x4 is the
      current maximum with 8x4 in progress.
- [ ] Thomas: email asic-competition@janestreet.com with two questions: will
      `8x4` be enabled in the cmos5l flow before the deadline, and are IHP SRAM
      macros acceptable in a submission for the March 2027 shuttle. The M2
      memory decision needs the second answer. Draft and a matching Tiny
      Tapeout Discord post are in the vault under `60-applications/outreach/`
      (drafted 2026-09-15, follow up 2026-09-25 if no reply).
- [ ] Opus: `src/tt_um_loom.v` M0 version = hard-wired UART transmitter that
      sends "LOOM\r\n" at 115200 baud on OUT0 whenever IN0 is high, with a
      cocotb test that decodes it. This is the blog's own "start by getting a
      UART transmitter out of a pin" step and it exercises the whole flow.
- [ ] Thomas: push, confirm `test`, `gds`, `docs` workflows pass. Record the
      utilisation and cell count of the trivial design in `docs/AREA.md`
      (baseline for the flow overhead).
- [ ] Opus: WSL dev loop documented and working (`make -C test`, Verilator
      lint), `test/requirements.txt` pinned to what WSL has.

### M0.5: SRAM smoke test on cmos5l (by 2026-09-28, runs alongside the start of M1)

Exit: a minimal design containing `RM_IHPSG13_1P_512x16_c2_bm_bist` has been
pushed through the TT `gds` workflow on branch `sram-smoke`, and the full
precheck output is posted in the Tiny Tapeout Discord thread, pass or fail.

Why now: on 2026-09-16 Matt Venn (Tiny Tapeout) said TT should be able to
handle macros, that precheck DRC issues still exist on SG13G2, and that he has
not tried CMOS5L. Any fix on their side takes calendar time, so the first
failing run has to exist in September, not at the October gate. Being the
first public cmos5l SRAM example is also worth doing in its own right.

- [ ] Opus: branch `sram-smoke`. Vendor the macro views (GDS, LEF, the three
      lib corners, CDL, Verilog model) from the pinned IHP-Open-PDK commit named
      in `docs/tt_cmos5l_facts.md` into `macro/`, with a README naming the
      commit. Port names come from the macro's Verilog model, not from memory.
- [ ] Opus: `src/loom_imem_macro.v` wrapping the macro behind the `loom_imem`
      interface (this file becomes the MACRO backend later), BIST and bit-mask
      pins tied off as the model requires, plus a pin-level test top that can
      write and read every word through the TT pins.
- [ ] Opus: cocotb test against the macro's Verilog model: walking ones,
      address uniqueness, full 512 x 16 sweep.
- [ ] Opus: `src/config.json` macro keys from the `tt_um_urish_sram_test` recipe
      (`MACROS`, `PDN_MACRO_CONNECTIONS`, `PDN_CFG` with the Metal4 connection,
      `MAGIC_EXT_ABSTRACT_CELLS`, `ERROR_ON_MAGIC_DRC: false`) with the PDN
      stripe pitch matched to the macro's power pins. Allowed on this branch
      only, under D-015.
- [ ] Thomas: push the branch, collect the `gds` and precheck logs, post them
      in the Discord thread, record the outcome in `docs/AREA.md` and in
      `docs/tt_cmos5l_facts.md` section 9.
- [ ] If Ken's config becomes public, diff it against ours before a second try.

Timebox: two sessions. If it fails for reasons on Tiny Tapeout's side, stop,
report, and wait for their fix. M1 proceeds on the flop memory regardless, and
the M2 gate rules do not change.

### M1: core runs firmware (by 2026-10-05)

Exit: a UART TX written in Loom assembly runs on the RTL core, decoded by the
same cocotb UART model as M0; first synthesis numbers recorded.

- [ ] `isa/isa.yaml` v1.0 frozen. Every mnemonic, encoding, flag effect, and
      timing class from `docs/ARCHITECTURE.md` section 11. Frozen means changes
      need a `DECISIONS.md` entry.
- [ ] `tools/gen`: generates `src/loom_isa.vh`, assembler tables, golden-model
      tables, `docs/ISA.md`. CI fails if generated files are stale.
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
