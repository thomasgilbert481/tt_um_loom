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
data moving through the SPI host port. (The original criterion also asked for an
FPGA prototype running the same tests; D-024 dropped it on 2026-09-21.)
**Met 2026-09-21.**

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
      SPI0), MicroPython side in `tools/loomhost/micropython/`, CLI.
      2026-09-19: `SimTransport` too, which moves host bytes over the RTL's
      own SPI pads as one more pin model of the cocotb bench, so the host
      library drives the RTL exactly as it drives the model.
- [x] Firmware: `uart_rx`, `spi_master`, `spi_slave`, `i2c_master`. Each with an
      L3 test against a Python reference model, including error cases (framing
      error, NACK, clock stretching). 2026-09-18: `uart_tx_fifo`, `uart_rx`,
      `spi_master`, `i2c_master` pass end to end on the golden model with
      `tools/protomodels` (framing error, NACK and clock stretching
      included). The short start bit after idle in `uart_tx.loom` (firmware
      spec question 8) is fixed. 2026-09-19: `spi_slave.loom` (modes 0 and 3,
      MSB first, 50 words, `.tick 32`) and **the same test bodies on the
      RTL**: `test/rtl_bench.py` gives the RTL the interface of the model's
      bench, so the unchanged scenarios and the unchanged `tools/protomodels`
      models run on both, with the program and the data going through the SPI
      pads (`SimTransport`). 37 scenarios on the model, 29 of them on the RTL
      (the 8 left out are slow 115200-baud and mode-sweep cases, marked with
      the reason); no divergence between the two sides.
- [x] ~~FPGA: `fpga/icebreaker/` build with Yosys + nextpnr, host over Pico SPI,
      L3 tests re-run on hardware through `loomhost` (same scripts). The
      macro does not exist on the FPGA: build `tt_um_loom` with
      `IMEM_IMPL "FLOPS"` (or add an iCE40 block-RAM backend to `loom_imem`).
      2026-09-19: confirmed that the template's own `fpga` workflow cannot do
      it (run 35478423175: it reads the `info.yaml` source list with no
      defines and stops at "Module `RM_IHPSG13_1P_512x16_c2_bm_bist`
      referenced in module `loom_imem_macro`"), so the FPGA build is ours to
      write, with a top that sets `IMEM_IMPL`. Hardware time needs Thomas.
      2026-09-21: **the full design does not fit the iCEBreaker.** Synthesis
      for the iCE40UP5K needs 7,732 LUT4 against 5,280 (the instruction memory
      does go to block RAM; the gap is logic, and no build knob closes it),
      so `fpga/icebreaker/` stays as the measured attempt and the pin-map
      template. The same RTL on an ECP5-25F uses 32 per cent of the LUTs and
      closes at 47 MHz.~~ **Dropped 2026-09-21 (D-024, Thomas's call):** no
      FPGA before silicon; verification rests on co-simulation, gate level,
      formal and mutation, and the write-up says so.
- [x] Memory decision taken early, 2026-09-18 (D-020): the 512x16 SRAM macro,
      with the latch array as fallback. Remaining conditions: Tiny Tapeout's
      view of the overlap waiver and power-script wrapper, and Jane Street's
      answer on macros. The third, a 6x4 hardening of the real core with the
      macro, is met (2026-09-19, run 35419160398: precheck pass, 51.5 percent,
      +6.21 ns typical).
      2026-09-18: integrated on branch `macro-core` (D-021): `loom_imem`
      `IMPL "MACRO"` by default, FLOPS kept and tested (`test_flops.py`),
      the recipe in `src/config.json` with the macro FS at (12, 40); Yosys
      generic 33,425 -> 19,456 cells, 7,138 -> 3,027 flops. The first 6x4
      hardening with the macro was started from that branch.
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
- [x] 2026-09-22, Fable review (`docs/reviews/m2-review.md`, architect's
      answer): M2 met. Decisions D-025 to D-030; M3 and M4 below rewritten.

### M3: bit engine encoders, data memory, the M3 protocols (by 2026-11-01)

Rewritten at the M2 review (2026-09-22, D-025 to D-030). The original exit's
formal and mutation items were met in M2. Every hardware change below is one
slice, hardened alone and judged by D-025 (detailed routing under 4 hours,
Metal3 overflow the early warning).

Exit: slices A and B hardened inside the D-025 budget; `ws2812`, `ps2_host`,
`jtag_master`, `swd_master`, `i2c_slave_eeprom` and `usb_ls_device` pass
their L3 tests on the golden model and on the RTL; ISO-1 attempted and its
result recorded, bounded or proved.

- [x] D-028 (the host's TD write leaves rule 2; Thomas delegated the call,
      Fable took it 2026-09-22). Done the same day: SEMANTICS 6.10,
      HOST_PROTOCOL, model, `loom_timer`, regression tests on both sides,
      formal TIMER-2 proved. Open: its hardening, which is the slow-corner
      measurement and sets the baseline the slices are read against: run
      35773244433 was cancelled at 54 minutes by a push that carried a local
      commit touching `src/loom_isa.vh` (the trap is now in CLAUDE.md); run
      35779039938 on e64f18a, the same netlist, started 20:15 UTC and passed
      every job on 2026-09-23: routing 2 h 57 min, Metal3 overflow 1,248
      (the lowest yet), typ +3.56 ns, slow -6.36 ns on a new worst path
      (host debug thread select to `pin_oe_reg[6]`); D-031 proposed.
- [ ] One session: can LibreLane run locally (the Docker image in `CLAUDE.md`,
      the pinned PDK) to the end of global routing and stop? Record the answer
      in `docs/tt_cmos5l_facts.md`. If yes, every slice reads its overflow
      there before it goes to CI. Deferred 2026-09-24 until the slice C
      decision: no other slice is planned, so its only use would be slice C
      or a post-freeze fix. What it takes, from `tt-gds-action@ihp-cmos5l`:
      `pip install librelane==3.1.0.dev3`, tt-support-tools branch
      `ihp-sg13cmos5l`, the PDK from the action's `install_sg13cmos5l.sh`,
      and Docker reachable from WSL for LibreLane's container; then
      LibreLane's `--to` on the config `tt_tool.py --harden --ihp` writes.
- [x] 2026-09-22: SEMANTICS 6.9.1 (D-026 slice A): NRZI, Manchester as two
      `SHO` per bit, USB and CAN stuffing and destuffing with a uniform run
      rule, the T flag, the differential output bit, the encoder state at
      debug 0x27, `CAPS[9]`. ARCHITECTURE 8.1 kept in step.
- [x] Slice A: 2026-09-22, RTL and golden model written from that text by
      two agents that never saw each other's code (spec questions and
      rulings in `docs/spec-questions/{rtl,loomsim}-m3a.md`); L1-BE-ENC and
      L1-BE-STUFF on both sides (8 cocotb, 67 model tests); the
      co-simulation generator drives `ENC`, `STUFF` and `DIFF` with 25 new
      coverage bins and finds no divergence; generic synthesis +686 cells,
      +61 flops, longest path unchanged (`docs/AREA.md`). Hardened
      2026-09-23 (run 35812463112): every job passed, routing 3 h 27 min,
      Metal3 overflow 2,924, typ +4.80 ns, slow -4.04 ns; stays under D-025.
- [x] 2026-09-22: firmware with no RTL, by an agent in parallel with slice A:
      `ws2812` (`SETP ... D` pulses, every edge clock-exact), `ps2_host`,
      `jtag_master` (IDCODE read), `swd_master` (DPIDR read, bidirectional
      turnaround), each with a `protomodels` model and an L3 test on the
      model and on the RTL: 23 new RTL scenarios, no divergence anywhere.
      The checker gained `.bounded "<reason>"` for a `PUSH`/`POP` the author
      has discharged by hand (ws2812's byte fetch inside a frame; the listing
      prints the declaration, `docs/spec-questions/firmware-m3.md` item 1).
- [x] 2026-09-22: ISO-1 (thread isolation) **proved unbounded** by `abc pdr`
      (35 min) on a two-copy miter of `loom_core` + `loom_pins`, thread 0's
      whole state compared every cycle, with the host debug port quiet;
      depth-24 BMC cross-check, 9 cover points. WAIT-1A (the completion
      rule) proved, WAIT-1's bound checked to depth 40. Findings F-4 and F-5
      are wording corrections (`formal/README.md`); BUGS 6 is an
      observation from the same work. One formal agent, one run.
      2026-09-23, F-7: the miter predated slice B and compared none of its
      state or a thread's stores; extended (99 assertions, 11 covers) and
      proved for all four threads, thread 0 in 33 min.
- [x] 2026-09-22: SEMANTICS 6.11 and `isa.yaml` for D-027: `LD`/`ST` as
      two-slot instructions on the instruction memory through the thread's
      own fetch cycle (text in 6.11, state in section 5, debug 0x28, the
      completion slot's retire record in section 8); `isa.yaml` timing class
      `two_slot`, the deadline checker prices them at two slots (test added),
      generated files refreshed. Data placement uses the existing `.org` and
      `.word`; the host's existing IMEM commands load and dump an image.
- [x] Slice B: 2026-09-22, RTL and model written from 6.11 by two agents
      (per-thread held access after the model author's question 1; 14
      cocotb and 47 model tests; the generator addresses a data window;
      generic synthesis +593 cells, +136 flops in `loom_core`). Hardened
      2026-09-23 (run 35871222851 on 54ebaa1): every job passed, routing
      3 h 29 min, Metal3 overflow 2,252, typ +3.56 ns, slow -6.08 ns,
      54.7 per cent; stays under D-025, and meets the bar below for slice C.
      Two formal properties (ISA-2A/2C, WAIT-1A) had to learn that the
      completion slot decodes nothing; the design was right.
- [x] D-031 (Fable's proposal after D-028's reading): the host's debug
      thread select becomes a one-hot register in `loom_host_ctl`, so the
      timers' host write enables are ANDs with a flop instead of a 2-bit
      compare in the slow-corner cone. Built 2026-09-23 (generic synthesis
      -156 cells, +2 flops; check_all and the whole formal suite green).
      Hardened 2026-09-24 (run 35940928210): every job passed, routing
      3 h 54 min, Metal3 overflow 1,945, typ +5.38 ns, slow -3.53 ns (from
      -6.08); stays. The gds job took 5 h 02 min, the longest yet.
- [x] Firmware on slice A and B: `i2c_slave_eeprom` (24C02-style, 256 bytes
      in an unused quarter, L3-I2C-S), `usb_ls_device` in manual mode with a
      Python host model, enumeration to SET_ADDRESS and one HID report
      (L3-USB-LS), `can_loopback` with stuffing and CRC15 (L3-CAN). Done
      2026-09-23 by two agents, on both backends with no divergence.
      `usb_ls_device` enumerates as a HID boot mouse (device, configuration
      and report descriptors, SET_ADDRESS after its status stage,
      SET_CONFIGURATION, interrupt IN with host-pushed reports, STALL for
      the rest) and meets every USB 2.0 limit it is checked against
      (response 4.56 to 4.74 bit times of 2 to 6.5), in 511 of the 512
      words, so it runs alone in thread 0. `i2c_slave_eeprom` (239 words
      with its data) at 100 and 400 kHz with no clock stretching;
      `can_loopback` (TX thread 0, RX thread 1 that ACKs and reports CRC,
      stuff, form and no-ACK errors) at 125 and 500 kbit/s. Spec questions
      8 to 16 of `docs/spec-questions/firmware-m3.md` ruled; the text fixes
      (SEMANTICS 4, 6.9, 6.9.1, `CRCI` in `isa.yaml`) came with them.
- [ ] 2026-11-01, slice C decided by the numbers: auto mode in the
      slot-injected shape (D-026) only if slice B's hardening shows Metal3
      overflow under 3,500 and detailed routing under 3 h 45 min, and Thomas
      wants it. If built: SEMANTICS text, model, RTL, co-simulation,
      hardening, all before 2026-11-08. Input from the firmware: slice B's
      numbers meet the bar, and USB low speed, the stretch protocol D-029
      kept, meets its timing in manual mode (firmware-m3 item 10), so no
      program on the plan needs auto mode.
- [ ] Fable review, M3: the slices' numbers, ISO-1's state, the freeze commit.

### M4: RTL freeze and the evidence (freeze 2026-11-08; by 2026-12-01)

Exit: RTL frozen on 2026-11-08 and the final `gds` run of the freeze commit
finished inside six hours; the L3 suite passes at gate level on that netlist;
`docs/VERIFICATION_REPORT.md` complete in draft.

- [ ] 2026-11-08: RTL freeze (D-030). Final `gds` run by hand on the freeze
      commit; if it does not finish inside six hours the last slice comes out
      and the run repeats. After the freeze, RTL changes only for a bug found
      by verification, each with a re-hardening.
- [x] Manchester loopback at the manual-mode rate (L3-MANCH, in place of the
      10 Mbit target, D-029). Done 2026-09-24 by an agent (finished by the
      director after two stalls): `manchester_loopback.loom`, TX thread 0 and
      RX thread 1, 173 words, 2.083 Mbit/s (TICK 12, the fastest the checker
      proves) and 1 Mbit/s, on both backends; five spec questions ruled in
      `docs/spec-questions/firmware-m4.md`.
- [ ] Tools finding T-1 (`docs/VERIFICATION.md`): the deadline checker takes
      the sound budget after `SETD m`, `ceil(((m + k - 1) * P + 1) / 4)` slots,
      and the 15 pairs in 8 programs that fail it today are each given their
      whole tick in the program (`SETD m + 1`) or a printed argument that a
      late first `WAITD` is harmless, then retested on both backends. Before
      the firmware freeze.
- [ ] Firmware and tools continue to 2026-12-01 (no hardening, D-018): the
      remaining L3 cases, the slow 115200-baud and mode-sweep scenarios on
      the RTL (done 2026-09-24: all 14 `model_only` scenarios pass on the
      RTL with `LOOM_FW_SET=model_only` in `test/test_fw.py`, 351 s), the two open timer mutants closed (done: one by D-028, the
      other killed 2026-09-24 by a `test_mem` check once slice B made it
      reachable, with `test_mem` added to the mutation ladder).
      Two tools items from the M3 firmware rulings, both done 2026-09-24:
      the model backend fails a scenario that fetches or `LD`s a word the
      image never loaded (firmware-m3 item 9, the check `test/tb.v` makes
      on the RTL), and the assembler's listing marks a thread whose reset
      vector holds another section's word or data (item 16).
- [ ] Mutation re-run on the freeze commit. The 99.7 per cent in M2 was
      measured on the M2 RTL; slices A and B, D-028 and D-031 have changed
      `loom_core`, `loom_timer`, `loom_be` and `loom_host_ctl` since, and a
      mutant's id carries its line number (`tools/mutate/operators.py`), so
      every recorded equivalent below a changed line in `loom_core` is
      already keyed to the wrong line. Re-run the campaign on the frozen RTL,
      re-key `equivalents.json` (each reason re-checked, not copied), and
      quote that score in the report.
- [ ] `docs/VERIFICATION_REPORT.md` drafted from what exists: the bug ledger,
      the mutation table, the formal findings, the routing budget, D-024's
      statement that nothing ran on hardware and what stands in for it.
      Started 2026-09-22: sections 1 to 3 and 5 to 7 describe `main` after
      the M2 review; sections 4 (evidence by layer) and 8 (what the AI did)
      are outlines to fill at the freeze; updated with each slice.
- [ ] If the `gds` artefact includes SDF (checked 2026-09-24: it does, the
      `GDS_logs` artifact carries `runs/wokwi/final/sdf/` for all three
      corners, run 35940928210): one timing-annotated gate-level run of the
      L3 suite at the typical corner, recorded as PHY-GL-SDF. Checked
      2026-09-24: Icarus annotates the cells (not the SRAM macro, which
      `scripts/gl/sdf_filter.py` drops), ignores timing checks, and runs about
      1.9 s per simulated microsecond with cell delays, so the whole L3 suite
      (181 ms) would take days; a smoke test passed (`test_mem`, 330 us, 11
      min; `docs/tt_cmos5l_facts.md` section 12). What remains is a chosen
      subset: one scenario per protocol, run overnight on the freeze netlist.
- [ ] Timing as stated in the datasheet: 50 MHz at the typical corner and the
      measured slow-corner clock, both with their slack.
- [ ] Firmware freeze 2026-12-01.

### M5: documentation and submission package (by 2026-12-20)

- [ ] `docs/info.md` (Tiny Tapeout datasheet page) complete with pinout,
      how-to-test, external hardware.
- [ ] `README.md`: architecture, why it is different, results table (area,
      clock, protocols, coverage, mutation score, formal properties), how to
      build, how to program, honest limitations.
- [ ] `docs/VERIFICATION_REPORT.md`: numbers, bug ledger, what AI did and how
      it was checked.
- [ ] Demo material (no FPGA, D-024): the assembler listing with slot
      annotations, waveforms from the RTL L3 runs (the 433- and 434-clock
      `SETP ... D` edges at a 433.5-clock tick), the gate-level log, the
      mutation table.
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
| Scope creep on stretch protocols | high | RTL freeze 2026-11-08 (D-030); USB LS is the only stretch with a fixed slot; slice C only under D-025's numbers |
| Semester crunch (564 project, finals) | high | RTL frozen 2026-11-08, before the crunch; November is firmware and the report; December is docs; buffer week before the deadline |
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
5. The iCEBreaker attempt as measured (`fpga/icebreaker/`, D-024); no FPGA
   build and no video.

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
- 2026-09-18 (night), Opus 5, macro agent on branch `macro-core` (from main
  52ee9f8): the SRAM macro in the real core (D-020, D-021). The views, the
  wrapper, `pdn_cfg.tcl` and the blackbox came over byte-exact from
  `sram-smoke`; `loom_imem` gained `IMPL` (`"MACRO"` by default, the wrapper's
  enable driven by `en` so bubbles do not access the macro; `"FLOPS"` kept);
  `src/config.json` got the recipe with the instance at
  `u_loom.u_imem.g_macro.u_macro.sram`, FS at (12, 40), on the 67.44 um
  stripe grid (the 6x4 core origin is the 2x2's). Yosys generic: 33,425 ->
  19,456 cells, 7,138 -> 3,027 flops; the FLOPS 256 variant still gives
  exactly main's numbers. Tests: 512-word expectations (CAPS 0x909A, reset
  vectors 0/128/256/384, walking-one IMEM addressing), co-simulation on the
  macro model, and `test_flops.py` on a second, FLOPS instance in `tb.v`;
  `check_all.sh` green (75 cocotb tests, zero co-simulation divergences).
  Next: read the first 6x4 hardening with the macro (utilisation, the
  macro's 3.7 ns typical / 6.25 ns slow clock-to-output on the D-stage paths,
  illegal-overlap count 5, precheck), then merge `macro-core` into `main`.
- 2026-09-19, Opus 5: the first 6x4 hardening with the macro in the core
  (run 35419160398, `macro-core` d41fe34) passed gds, precheck and viewer:
  utilisation 51.5 percent (was 78.8), setup +6.21 ns typical and +11.07 ns
  fast, -2.07 ns at the slow corner on 23 endpoints (was -9.95 ns on 1,084),
  hold clean everywhere, DRC, LVS and antenna 0 (AREA.md). The macro's slow
  clock-to-output does not matter: its worst path has +2.07 ns at the slow
  corner. The illegal-overlap count read 10, not 5; they are the same four
  stripe crossings as the smoke test, each reported as two boxes (facts 12).
  `gl_test` failed 1 of 15, which was a test bug, not a netlist bug (BUGS 4):
  `test_other_threads_do_not_move_the_edges` never reset thread 1 after
  setting its RESET_PC, so in the 512-word build the thread ran unwritten
  memory at 0x80; RTL decoded the X words quietly, the netlist spread them into
  the pin register. Found by reproducing the job locally (netlist from the
  run, cell models from IHP-Open-PDK at CI's commit) and diffing 982 named
  nets between RTL and gate level clock by clock (facts 12). The test now
  resets the thread and checks the loop ran, and `tb.v` stops any RTL run
  whose valid slot decodes an X word. Merged `macro-core` into `main`.
  The slow-corner group is the deadline-latch fire logic: the host debug
  thread decode fanned out ahead of a 16-bit subtract (D-022 restructures it,
  proven equivalent with Yosys `equiv_*`). The remaining -0.26 ns path is
  `tx_th` fanout through a chain of six `buf_1`; the flow signs off at the
  typical corner only (`TIMING_VIOLATION_CORNERS` `*typ*`), so slow-corner
  closure stays a stretch item with the flow knobs in AREA.md.
- 2026-09-19 (evening), Opus 5, two agents in worktrees: **co-simulation over
  the M2 features** and **the L3 firmware tests on the RTL**, both landed on
  `main` after the director re-ran every check. Co-simulation now builds the
  model from `CTRL.CAPS` instead of generating around the M2 features (the
  `m2_built` flag is gone), generates FIFO, bit-engine and `SETP ... D` code
  with a static no-deadlock rule, drives the host port throughout the two SPI
  seeds (pushes, pops, CTRL writes, all mirrored into the model at the cycle
  the RTL commits them) and compares `HOST_IRQ` every cycle. Coverage 553
  bins, 33 empty, each with a reason. Four of five hand-written M2 mutants die
  at seed 1 (FIFO full off by one, bit-engine shift direction, the `SETP D`
  rule-2 compare, host-side INQ full); the fifth is documented as equivalent
  in practice. Zero divergences. The firmware side gives the RTL the golden
  model's bench interface (`test/rtl_bench.py` + `SimTransport`), so the same
  test bodies and the same `tools/protomodels` models run on both: 37
  scenarios on the model, 29 on the RTL, no divergence, plus the new
  `spi_slave.loom`. Suite 104 cocotb tests and 1,293 tool tests. Two spec
  items came out of it: SEMANTICS 6.7 now names the edge the next FIFO read
  word is loaded at, and the model gained `host_fifo_error()`. M2's exit
  criterion is met except the FPGA box.
- 2026-09-20 (night), Opus 5: the L4 formal agent landed `formal/` (7 sby
  groups, 18 properties, `scripts/formal.sh`, a `formal` CI job) and two of
  the L4 properties turned out to be false. **PIN-1 was a real bug**: an
  open-drain pin could drive high if `OD_MASK` was set after the pin was
  driven, or if `OEP` wrote the enable afterwards, which on a shared bus is an
  electrical fault. Fixed in D-023 by applying the mask at the pads, in RTL,
  model and SEMANTICS 3 together, with regression tests on both sides; the
  property is proved now. TIMER-1 was a wording bug (the deadline compare is a
  half window, so it is not monotone): VERIFICATION and SEMANTICS 4 corrected,
  and SPI-1 gained the assumption it needs (no chip reset inside a host
  transaction, now in HOST_PROTOCOL). Yosys's own Verilog frontend ignores
  `bind` silently, so every proof reads the design through the `slang` plugin.
  **D-022 was reverted**: its hardening (run 35470401774) was killed at
  GitHub's six-hour limit inside detailed routing, and against the run before
  it the two per cent of extra cells had tripled the global router's Metal3
  overflow (1,768 -> 5,776) and taken routing from 3 h 03 min to over 5 h 15
  min. Six hours is a hard budget because Tiny Tapeout re-runs the flow at
  submission, and the slow corner is not a sign-off corner, so the design goes
  back to the shape that hardened in 3 h 53 min (AREA.md has the numbers and
  the cheaper shape to try if slow-corner closure is ever wanted). Also
  confirmed that the template's `fpga` workflow cannot build this design.
- 2026-09-20 (evening), Opus 5: the hardening of that state (run 35524275302,
  main f081f4a) **passed every job**: gds 4 h 45 min, precheck 1 h 47 min,
  `gl_test` 71 of 71 on the widened list in 12.7 minutes, viewer. DRC, LVS and
  antenna 0; the Magic overlap count is still the 10 boxes D-021 watches; typ
  +5.98 ns, slow -2.48 ns on the same 23 endpoints. Detailed routing took
  3 h 37 min with 3,169 Metal3 overflow, against 3 h 03 min and 1,768 for a
  design seventeen cells away, so placement variance alone is worth half an
  hour of routing and the margin under the six-hour limit is one to two hours,
  not three (AREA.md). Mutation round 2 closed the three holes round 1 found
  (`test_debug_register_cross_talk`, `test_csr_tick_frac_readback`,
  `test_be_cfg_fields_readback`: ten mutants that had survived now die), and
  the full pass is running.
  Next: the FPGA prototype (brief written), then the M2 review with Fable
  (`docs/reviews/m2-review.md`) before M3 starts.
- 2026-09-22, Fable 5.1, M2 review (`docs/reviews/m2-review.md`, architect's
  answer): M2 met. Routing time gates every hardware change, one per
  hardening (D-025). The bit engine is built encoders-and-stuffing first, in
  manual mode, and auto mode, if at all, acts in the thread's slot on the
  shared datapath (D-026); data memory is `LD`/`ST` on the instruction memory
  through the thread's own fetch cycle (D-027); taking the host's TD write
  out of rule 2 is proposed to Thomas as the slow-corner fix (D-028); USB LS
  stays, CAN is firmware, 10 Mbit Manchester is cut as a target (D-029); RTL
  freeze moved to 2026-11-08 and the verification report starts now (D-030).
  M3 and M4 rewritten, ARCHITECTURE 8, 8.1 and 15 updated. Next: Thomas
  answers D-028; the implementer starts with the local global-routing check
  and the SEMANTICS 6.9 M3 text, then slice A.
- 2026-09-22 (later), Fable 5.1: Thomas delegated the three open calls of the
  review ("proceed based off your best judgement"): D-028 taken, the
  2026-11-08 freeze kept, slice C by the numbers with no as the default.
  D-028 implemented in RTL, model, both test suites and formal (TIMER-2
  proved, 7 covers); `check_all` green; pushed so the hardening runs. Next:
  read that run against the slow corner (-2.48 ns before) and the D-025
  routing numbers; then the SEMANTICS 6.9 M3 text for slice A and 6.11 for
  slice B, and the local global-routing check.
- 2026-09-22 (night), Fable 5.1 as director: slice A landed from two agents
  that wrote the golden model and the RTL from SEMANTICS 6.9.1 without
  seeing each other's code; the readings in their spec-question files agree
  everywhere they overlap, and the co-simulation with the generator driving
  ENC, STUFF and DIFF finds no divergence. `CAPS[9]` added for slice A at
  integration. Suite: 121 cocotb, 1,451 tool tests, lint clean. Committed
  locally and NOT pushed: run 35779039938 (D-028's measurement) is still
  going and a src push would cancel it. Firmware agent (ws2812, ps2_host,
  then jtag/swd) still running. Next: read run 35779039938 (AREA.md, D-028
  outcome), push, and its follow-on hardening is slice A's D-025 reading.
- 2026-09-23 (small hours), Fable 5.1: the D-028 hardening passed and was
  read (AREA.md); the slow corner did not close and the typical margin
  dropped to +3.56 ns on a logic deletion, so D-031 (registered one-hot
  host thread select) is proposed for after the slices. Six agents'
  work integrated in one night: slice A (RTL + model, cosim 40 seeds
  clean), slice B (RTL + model, per-thread held access, cosim 40 seeds
  clean), four M3 firmware programs with models and tests plus the
  `.bounded` declaration, ISO-1 proved unbounded, WAIT-1A proved, SCHED-2
  extended and SCHED-4 added (BUGS 6). Pushed up to 8c487cc (slice A's
  hardware only): run 35812463112 is slice A's D-025 reading. Slice B
  (b32c152) and later docs stay local until that run finishes; then push
  b32c152 for slice B's reading. Next: read 35812463112; push slice B;
  then D-031; then the USB LS, CAN and I2C EEPROM firmware on the slices.
