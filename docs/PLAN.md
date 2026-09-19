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

- [x] (2026-09-17, on `sram-smoke`) Opus: branch `sram-smoke`. Vendor the macro views (GDS, LEF, the three
      lib corners, CDL, Verilog model) from the pinned IHP-Open-PDK commit named
      in `docs/tt_cmos5l_facts.md` into `macro/`, with a README naming the
      commit. Port names come from the macro's Verilog model, not from memory.
- [x] (2026-09-17, on `sram-smoke`) Opus: `src/loom_imem_macro.v` wrapping the macro behind the `loom_imem`
      interface (this file becomes the MACRO backend later), BIST and bit-mask
      pins tied off as the model requires, plus a pin-level test top that can
      write and read every word through the TT pins.
- [x] (2026-09-17, on `sram-smoke`, 5 of 5 pass) Opus: cocotb test against the macro's Verilog model: walking ones,
      address uniqueness, full 512 x 16 sweep.
- [x] (2026-09-17, on `sram-smoke`) Opus: `src/config.json` macro keys from the `tt_um_urish_sram_test` recipe
      (`MACROS`, `PDN_MACRO_CONNECTIONS`, `PDN_CFG` with the Metal4 connection,
      `MAGIC_EXT_ABSTRACT_CELLS`, `ERROR_ON_MAGIC_DRC: false`) with the PDN
      stripe pitch matched to the macro's power pins. Allowed on this branch
      only, under D-015.
- [x] 2026-09-18: hardened and prechecked on the eighth CI run (565673f, run
      35377845679): gds, all nine precheck checks, gl_test and viewer pass.
      Recipe in `docs/tt_cmos5l_facts.md` section 11, decision D-020.
- [ ] Thomas: post the outcome in the Discord thread (draft in the vault
      outreach note), record the outcome in `docs/AREA.md` and in
      `docs/tt_cmos5l_facts.md` section 9.
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
      need a `DECISIONS.md` entry. (Now 0.4.0: 64 instructions, no overlaps,
      75.8 percent of the encoding space used; 0.4.0 added `SETP ... D`.)
- [x] 2026-09-17: `tools/loomisa`, the only parser of `isa.yaml`: encode,
      decode, overlap check, and generators for `src/loom_decode.v` (the
      decoder module itself, so RTL never hand-decodes), `src/loom_isa.vh` and
      `docs/ISA.md`. 29 tests. CI job `isa-and-tools` fails on stale files.
- [x] 2026-09-17: `tools/loomasm`: two-pass assembler (labels, expressions,
      `.thread/.org/.equ/.pins/.word/.csr/.tick/.deadline_check`, pseudo-ops,
      symbolic operand names taken from `isa.yaml` enums), JSON image, listing,
      disassembler, and the deadline checker (interprocedural through
      CALL/RET; infeasible = error, unbounded = warning; `SETD m` credited;
      `WAITD 0` transparent; limits in `tools/loomasm/README.md`). 232 tests.
      `firmware/uart_hello.loom` (M1 acceptance program) and `uart_tx.loom`
      both prove feasible with 410 clocks of worst-case slack.
- [x] 2026-09-17: `tools/loomsim`: cycle-accurate Python golden model written
      from `docs/SEMANTICS.md` alone (its author was not allowed to read the
      RTL): pipeline and slot validity, shared-state visibility with a pending
      commit list, SFLAGS forwarding, tick generators, waits and timeouts, pin
      index space and synchroniser, run control, host actions, retire record,
      FIFOs behind a feature flag, CLI. 318 tests, including four that pin the
      slot-grid timing property over 101 edges. Its twelve spec questions are
      resolved in `docs/spec-questions/loomsim.md` and folded into SEMANTICS,
      D-016 and D-017.
- [x] 2026-09-17: M1 RTL, written from `docs/SEMANTICS.md` alone (its author
      was not allowed to read the golden model): `loom_core` (barrel pipeline,
      SFLAGS forwarding, waits, run control, retire record), `loom_regfile`,
      `loom_alu`, `loom_timer`, `loom_pins`, `loom_imem` (flops, macro-ready
      ports), `loom_spi_host`, `loom_host_ctl` (CTRL, IMEM, DEBUG, STEP),
      `loom_top`, pad-map-only `tt_um_loom`. Every decode signal comes from the
      generated `loom_decode`. 42 cocotb tests, all driven through the pins
      over SPI so they also run at gate level; `firmware`-style UART program
      sends "LOOM" with frame starts exactly 4340 clocks apart. Verilator
      `-Wall` clean, Yosys `check -assert` clean. Interfaces in
      `docs/INTERFACES.md`; its spec questions in `docs/spec-questions/rtl.md`.
- [ ] Verification L1 unit tests for regfile, ALU, timer, pins; L2 directed
      tests generated from `isa.yaml` (at least one per mnemonic) passing on
      Icarus; first constrained-random co-sim run (1000 programs) passing.
      Progress 2026-09-18: co-simulation is in (`tools/loomgen`, 120 generator
      tests; `test/test_cosim.py` compares RTL and golden model on every cycle:
      retire record, pads, guards, slot-grid PC rule, periodic full state).
      Default run: 12 backdoor seeds x 4000 cycles plus 2 SPI-loaded seeds, 24 s,
      **zero divergences** between the independently written RTL and model.
      Fault injection: an inverted SUB carry and a one-step-late tick were both
      caught in the first seed. Still open: 1000-program run, coverage holes
      (57 reachable bins: per-thread mnemonic bins, ten flag outcomes, BNC not
      taken, CSRW NOW, WAITS over CLR; unbuilt instructions should be marked
      unreachable rather than counted).
- [ ] `firmware/uart_tx.loom` passes the L3 UART test at 115200 and 1 Mbaud.
- [x] 2026-09-18: first hardening of the M1 core (CI run 35272974272): DRC,
      LVS and antenna clean, precheck and gate-level test pass, but 78.8
      percent utilisation, +1.45 ns setup slack at the typical corner and
      -9.9 ns at the slow corner. Numbers in `docs/AREA.md`; critical path and
      the planned fix in D-019; CI cost fix in D-018.
- [ ] Fable review: ISA freeze, area numbers, decide FIFO depth.

### M2: host interface and the three required protocols (by 2026-10-19)

Exit: UART RX/TX, SPI master and slave, I2C master all pass their L3 tests with
data moving through the SPI host port; FPGA prototype runs the same tests.

- [x] 2026-09-18: D-019 built. Four self-rotating one-hot rings replace the
      decoded W-stage thread (Yosys merges identical flops, so the replicas
      are independent rings). Generic netlist: thread-select cone depth 22 to
      5 cells, worst fanout 308 to 33; core 7,807 to 6,512 cells. Co-sim still
      zero divergences (plus 40 extra seeds). Real slack comes from the next
      hardening. Area and timing before features (M1 left 79 percent
      utilisation and +1.45 ns typical slack); then re-harden. M2 features (FIFOs, bit
      engines, about 80-100K um2) do not fit until the memory decision frees
      area, so the memory gate below is also the area gate.
- [x] 2026-09-18: `docs/SEMANTICS.md` M2 text: FIFOs with blocking PUSH/POP
      and host-side rules (6.7), WAITB, the host interrupt (6.8), bit engine
      manual mode with NRZ, INV, DIR, CNT-as-loop-counter and serial CRC
      (6.9), deadline-latched `SETP ... D` (6.10), CAPS and BADOP bits. The
      ISA 0.4.0 edits it needs (SETP `D` field, SHO/SHI set Z, canonical CRC
      presets) land together with the M2 RTL. Auto mode, NRZI, Manchester and
      stuffing are specified before M3.
- [x] `loom_spi_host` + `loom_host_ctl`: full `docs/HOST_PROTOCOL.md` including
      the debug space, single-step, IRQ. 2026-09-18: FIFO space and IRQ
      built. Every host write now commits at edge E+4 after the word's last
      SCK rise is sampled (measured on twenty paths; IRQ_EN/IRQ_EN2 were one
      edge early and are fixed); rule written into HOST_PROTOCOL; `test_irq`
      5 of 5.
- [x] 2026-09-18: `loom_fifo` x 8, `PUSH`/`POP`, `WAITB`, host FIFO space
      (peek at load, pop at word end), BADOP[14], CTRL.RESET emptying: 8 cocotb
      tests. Generic netlist +2,274 cells, +619 flops.
- [x] 2026-09-18: bit engine manual mode (`loom_be.v`): `SHO`/`SHI`, SR/CNT
      with Z, left-aligned serial CRC matching the catalogue check values for
      the USB CRC5/CRC16, CAN CRC15 and SMBus CRC-8 presets, NRZ only.
- [x] 2026-09-18: deadline-latched `SETP ... D` (SEMANTICS 6.10, D-016): at a
      433.5-clock tick every edge lands exactly on a tick edge (gaps 433 and
      434 only), where plain `SETP` stays on the 4-clock slot grid; 8 tests.
      CAPS reports FIFO, bit engine and SETP D; VERSION 0x0002. 73 cocotb
      tests. Co-simulation still avoids the M2 instructions until the golden
      model's M2 branch is merged.
- [x] 2026-09-18: `tools/loomhost`: protocol encoder, `Loom` API, transports
      (`ModelTransport` over the golden model as an executable reference of
      the host protocol, `PicoTransport`, `TTBoardTransport` using the RP2040's
      SPI0), MicroPython side in `tools/loomhost/micropython/`, CLI. A cocotb
      `SimTransport` is still to do.
- [ ] Firmware: `uart_rx`, `spi_master`, `spi_slave`, `i2c_master`. Each with an
      L3 test against a Python reference model, including error cases (framing
      error, NACK, clock stretching). 2026-09-18: `uart_tx_fifo`, `uart_rx`,
      `spi_master`, `i2c_master` pass end to end on the golden model with
      `tools/protomodels` (framing error, NACK and clock stretching
      included); `spi_slave` and the same tests on the RTL are open. Fix the
      short first start bit in `uart_tx.loom` (firmware spec question 8).
- [ ] FPGA: `fpga/icebreaker/` build with Yosys + nextpnr, host over Pico SPI,
      L3 tests re-run on hardware through `loomhost` (same scripts).
- [x] Memory decision taken early, 2026-09-18 (D-020): the 512x16 SRAM macro,
      with the latch array as fallback. Remaining conditions: Tiny Tapeout's
      view of the overlap waiver and power-script wrapper, Jane Street's answer
      on macros, and a 6x4 hardening of the real core with the macro.
      Original gate text, kept for the record: trial hardening of
      `RM_IHPSG13_1P_512x16_c2_bm_bist` (16 bits wide, 236.80 x 191.34 um)
      following the `tt_um_urish_sram_test` recipe (`MACROS`, custom
      `pdn_cfg.tcl` with the Metal4 macro connection, `MAGIC_EXT_ABSTRACT_CELLS`,
      `ERROR_ON_MAGIC_DRC: false`) with the PDN stripe pitch tuned to the
      macro's power-pin pitch (the approach a community member reportedly has
      working on cmos5l; ask "Ken" on the TT Discord for his config once
      public), alongside the FLOPS-256 run. The gate is only passed by a run
      that also clears the TT **precheck**, which macros currently fail; if
      precheck is still failing at the gate, take the latch array (option 3 in
      ARCHITECTURE 12, about 140K um2 smaller than the M1 flops) rather than
      FLOPS, and keep the macro branch alive until M4. Results in
      `docs/AREA.md`; Jane Street's answer on macro acceptance in hand; record
      the pick in `docs/DECISIONS.md`.
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
- 2026-09-17, Fable 5.1: Thomas submitted the sign-up form and sent the Jane
  Street email, and asked Fable to continue the work. Wrote
  `docs/SEMANTICS.md` and `tools/loomisa` with the generated decoder (commit
  97a508a). Launched four Opus agents in parallel, each confined to its own
  paths: golden model (`tools/loomsim`), assembler (`tools/loomasm` plus
  `firmware/uart_hello.loom`), M1 RTL (`src/`, `test/`), and the M0.5 SRAM
  smoke test in the worktree `../tt_um_loom_sram` on branch `sram-smoke`. The
  model and RTL agents may not read each other's code. Director integrates,
  reviews and commits; spec questions land in `docs/spec-questions/`.
- 2026-09-17 (later), Fable 5.1: integrated all three. Assembler d39a5c8,
  golden model d37b6a6, M1 RTL 03a1bca (42 pin-level cocotb tests, 615 tool
  tests, lint clean). 21 spec questions from the agents resolved into
  SEMANTICS, HOST_PROTOCOL, D-016 (deadline-latched pin writes, M2) and D-017
  (reset vectors). First synthesis: 27.7K generic cells, half of it the flop
  memory. `sram-smoke`: three fast failures understood (instance path, then
  pdngen cannot stripe a Metal4-pin macro on cmos5l); run 4 without a macro
  grid got past PDN generation. Launched the co-simulation agent
  (`tools/loomgen`, `test/test_cosim.py`): RTL vs model, slot by slot, on
  constrained-random programs, with a rule that every divergence is decided by
  quoting SEMANTICS and logged in `docs/BUGS.md`.
- 2026-09-18, Opus 5 (the owner switched models as planned; the session had
  also hit a usage limit overnight): the co-simulation agent had been killed
  by the limit mid-task; relaunched with its partial files and a progress
  note. M1 core hardened overnight: tape-out clean, 78.8 percent full, +1.45
  ns typical slack (D-019 has the critical path), 4 h 39 min hardening plus
  3 h 27 min precheck (D-018 limits hardening to hardware changes and cancels
  superseded runs). `sram-smoke` run 4 was killed by GitHub's 6-hour limit in
  detailed routing: all 108 macro signal pins are on Metal2 along the macro's
  bottom edge, which the placement put 6 um from the die floor, and the custom
  power script left every standard-cell rail unconnected (654 PSM-0038
  warnings; the standard flow has none). Next: SRAM agent fixes both, with a
  capped router so failures end fast.
- 2026-09-18 (afternoon), Opus 5: all three agents died together to a stream
  stall. Co-simulation was nearly done, so the director finished it (0812cc7:
  generator tests, Makefile wiring, fault-injection proof; zero divergences;
  CI green on GitHub's Icarus). ISA 0.4.0 landed early (c26d022: `SETP ... D`
  encoding with assembler token `D`, SHO/SHI set Z, canonical CRC presets;
  RTL decodes `D` as a plain SETP until M2). Relaunched the SRAM and firmware
  agents with progress-file-first instructions, and started the M2 RTL agent:
  D-019 timing fix first, then FIFOs, host interrupt, manual bit engine and
  deadline-latched SETP, with co-simulation kept green by a generator flag
  until the golden model gets its M2 update. M2 hardening is deliberately
  deferred to the memory decision: at M1 density plus M2 features the block
  would be near 87 percent full.
