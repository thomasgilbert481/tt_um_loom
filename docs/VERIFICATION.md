# Loom verification plan

Jane Street said verification is the part they care most about as AI-assisted
design becomes common. This plan is built so that a reviewer can see, for every
claim, what checked it and what it would have missed. The plan has eight layers;
each layer has named checks with IDs that tests reference in their docstrings, so
coverage of the plan itself can be scripted (`tools/vplan_status.py` lists
every ID and whether a test or proof exists for it).

## Principles

1. **One golden model, many targets.** `tools/loomsim` is the reference for
   architectural behaviour. RTL simulation, the FPGA, and the silicon are all
   compared against it through the same host protocol and the same scripts.
2. **The ISA has one source.** `isa/isa.yaml` generates the decoder tables, the
   assembler, the model, the docs and the formal decode properties. Spec and
   implementation cannot drift silently.
3. **Every bug gets a line.** `docs/BUGS.md` records each bug found by
   verification: symptom, root cause, the layer that caught it, and the check
   ID that now covers it. This is the evidence for the writeup.
4. **The testbench is tested.** Mutation testing measures whether the suite
   would notice a broken RTL. A high pass rate with a low kill rate is a red
   flag, not a success.
5. **Determinism is a feature under test.** Timing properties (slot exactness,
   deadline behaviour, thread isolation) are checked by formal proof and by
   cycle-exact assertions, not only by "the UART decoded correctly".

## Tooling

| Purpose | Tool | Where |
|---|---|---|
| RTL simulation | Icarus Verilog 14 (default), Verilator 5 (fast co-sim) | WSL `~/oss-cad-suite`, CI |
| Testbench | cocotb 2.x, pytest | `test/` |
| Coverage | cocotb-coverage (functional), Verilator `--coverage` (line/toggle) | `test/coverage/` |
| Formal | SymbiYosys with smtbmc (yices/boolector) and abc pdr | `formal/` |
| Lint | `verilator --lint-only -Wall`, Yosys `synth` sanity | CI `lint` job |
| Gate level | TT `gl_test` action (cmos5l cell models) | CI |
| Physical | TT `gds` action (LibreLane), local iic-osic-tools Docker for iteration | CI, laptop |
| FPGA | Yosys + nextpnr-ice40, iCEBreaker (iCE40UP5K), Pico SPI bridge | `fpga/` |
| Mutation | `tools/mutate` (own, small) | CI nightly |

## Layers and checks

### L0: static

- L0-LINT: Verilator lint clean with `-Wall` (the TT flow runs a linter and
  fails on warnings it considers errors; we hold the stricter bar).
- L0-SYNTH: Yosys generic synth of `tt_um_loom` with no latches inferred
  outside `loom_imem` LATCH option, no undriven signals.
- L0-GEN: generated files match `isa/isa.yaml` (CI regenerates and diffs).
- L0-IFACE: `docs/INTERFACES.md` port lists match the RTL (script).

### L1: unit tests (cocotb, one test module per RTL module)

- L1-REGF: thread-indexed writes never leak across threads; two read ports
  with random addresses; write-then-read in the same thread's next slot.
- L1-ALU: every op against a Python model over 10k random operand pairs plus
  edge cases (0, 0xFFFF, 0x8000, shift amounts 0..15 and >15); flag rules
  from ARCHITECTURE 11.2.
- L1-TIMER-DIV: tick divider integer and fractional; measured mean period over
  4096 ticks equals TICK_INT + TICK_FRAC/256 within one clock; jitter at most one
  clock.
- L1-TIMER-REACH: wrap-safe "reached" over all quadrant cases (NOW and TD near
  0, 0x7FFF, 0x8000, 0xFFFF).
- L1-FIFO: fill, drain, simultaneous push/pop at every occupancy, flags exact.
- L1-SPI: byte framing at SCK = clk/8 and slower, CS deassert mid-byte aborts
  cleanly, back-to-back transactions, MISO turnaround byte.
- L1-PINS: index space mapping for every index 0..31 (reserved reads 0,
  read-only writes ignored), OD mode never drives 1, OE control, group base/count
  including counts of 1 and 16, synchroniser latency exactly as documented.
- L1-BE-ENC: NRZ/NRZI/Manchester encoder and decoder round trip for random bit
  strings; INV.
- L1-BE-STUFF: USB stuffing inserts after six 1s and never counts stuff bits
  in CNT or CRC; CAN stuffing after five equal bits; RX destuff; violation sets
  T exactly once.
- L1-BE-CRC: CRC5/CRC16 USB, CRC15 CAN, CRC8 SMBus against Python `crcmod`
  style references over random messages; left-alignment convention.
- L1-IMEM: all three implementations pass the same read/write test; read
  latency one cycle.

### L2: ISA co-simulation against the golden model

- L2-DIR: generated directed test per mnemonic (from `isa.yaml`): each test
  runs a short program on RTL and on `loomsim` and compares the full
  architectural state after every retired instruction. Includes every flag
  outcome and every branch taken/not-taken.
- L2-RAND: constrained-random programs. Generator constraints: bounded loops
  (DJNZ counts <= 16), waits only with timeouts or on stimulus the harness
  guarantees, no self-modifying anything (there is none), balanced CALL/RET
  depth <= 2, pins driven by a random stimulus process with recorded values so
  the model sees the same inputs at the same slots. Target: 10k programs per
  CI run on Verilator, 1k on Icarus.
- L2-TRACE: RTL exposes a retire trace (thread, PC, instruction, rd, value,
  flags, NOW) via `$display` in simulation only, which the harness compares to
  the model trace line by line. Any divergence prints the first differing slot.
- L2-COV: functional coverage: every opcode x thread, flag set/clear per
  producer, wait ended by condition vs by timeout, FIFO full/empty stalls, stuff
  insert and violation, CRC preset used, CALL stack overflow, reserved opcode
  hit. Target 100 percent of bins that are reachable.
- L2-SLOT: assertion in the RTL testbench that thread t's PC changes only on
  cycles where `cycle mod 4 == (t + 3) mod 4` (its W stage) and that no
  instruction takes more or less than one slot unless it is a wait or blocking
  FIFO op. This is the simulation twin of SCHED-2.
- L2-DEADLINE: for programs that use `WAITD`, the recorded pin-change
  timestamps match the assembler's predicted schedule exactly (not
  approximately).

### L3: protocol end to end (firmware on RTL vs reference models)

Each firmware program has a test with the same structure: load the program
through the host protocol, configure pins, run, drive or observe with a Python
reference model, check data and timing.

- L3-UART-TX/RX: 9600, 115200, 1M baud; 8N1 and 8E1; framing error detection;
  jitter measured at every edge <= 1 tick.
- L3-SPI-M/S: modes 0..3, 1 to 8 MHz, MSB/LSB first; slave with CS gating;
  master reading a simulated SPI flash JEDEC ID.
- L3-I2C-M: 100k and 400k, START/STOP/repeated START, ACK/NACK, clock
  stretching by the model, timeout via T flag.
- L3-I2C-S: EEPROM emulation (24C02 style) with random reads and page writes
  from a Python master; requires data memory or a register-backed 16-byte
  variant if DMEM is not built.
- L3-WS2812: 800 kHz timing within spec for 8 LEDs, decoded by a timing
  checker that fails on any pulse outside the datasheet window.
- L3-PS2: host receiving scancodes from a PS/2 keyboard model with parity.
- L3-JTAG: TAP state machine walk and IDCODE read from a TAP model.
- L3-SWD: line reset, JTAG-to-SWD sequence, DPIDR read from a model.
- L3-USB-LS (M4): Python low-speed host model: SYNC, PID, token CRC5, data
  CRC16, NRZI, stuffing, EOP, handshake; device enumeration up to SET_ADDRESS
  and a HID report.
- L3-CAN (M4): frame TX and RX loopback with stuffing, CRC15, ACK slot.
- L3-MANCH (M4): Manchester TX/RX loopback at the manual-mode rate (D-029
  cut the 10 Mbit, auto-mode target): 2.083 Mbit/s and 1 Mbit/s at 50 MHz,
  frames of one to four words, a node at a rate offset, an injected
  violation.

**Tools finding T-1 (2026-09-24): the deadline checker is optimistic after
`SETD`.** Every program's header says each deadline pair is proved, and
`tools/loomasm/README.md` says a real miss is never missed. That holds for
intervals that start at a `WAITD` (both ends are on the thread's slot
grid, and `floor(k * P / 4) = ceil((k * P - 3) / 4)`), but not for those that
start at `SETD m`: the `SETD` reads `NOW` up to `P - 1` clocks after the tick
that set it, so the deadline can come after `(m + k - 1) * P + 1` clocks, not
`(m + k) * P`. Found by the Manchester program's author (`firmware-m4.md`
item 4). Against the sound bound 15 `SETD` pairs in 8 programs fail, most of
them a `SETD 0` whose first `WAITD` expects a whole tick; what a late first
`WAITD` does is shorten the interval that follows it. No test has failed
on it; whether any test reaches the late case has not been checked. Fix
in `docs/PLAN.md` M4.

### L4: formal (SymbiYosys)

- SCHED-1: at every cycle exactly one thread is in each pipeline stage and the
  four are distinct (induction).
- SCHED-2: thread t's architectural state (regs, PC, flags, TD, SR, CNT, CRC,
  and from slice A the encoder state of SEMANTICS 6.9.1) changes only in its
  own W stage.
- SCHED-3: a thread with RUN=0 never changes architectural state except through
  host debug writes.
- SCHED-4: a `STEP_REQ` consumed by thread t's F stage is clear in the next
  cycle whatever the host wrote at that edge: the thread wins the
  coincidence SEMANTICS 7 describes (BUGS 6).
- ISO-1: for any two traces that agree on thread t's inputs (its pins, its
  FIFOs, SFLAGS it waits on) thread t's state sequence is identical regardless
  of what other threads do, given that no other thread writes shared state
  (ARCHITECTURE 1's "except through explicit shared state"). Checked as a
  two-copy miter on `loom_core` + `loom_pins`: **proved unbounded** by
  `abc pdr` for each of the four threads, with a depth-24 BMC cross-check
  for thread 0. Since slice B the compared state includes the data-memory
  state and the thread's own stores (formal finding F-7), and the claim
  needs one more precondition: no other thread stores into a word thread t
  reads (SEMANTICS 6.11 lets any thread store anywhere; programs keep to
  their own quarter). The host's
  debug traffic has to be counted among
  thread t's inputs, and even then the claim is false while the host uses the
  debug port, because that port, the register-file write port and the deadline
  latch's staged-write port are shared between the threads: formal finding
  F-4, which `docs/HOST_PROTOCOL.md` already describes for one of the three
  ("a DEBUG write of r0..r7 may wait up to three more clocks for the
  register-file write port"). Two `expect fail` tasks hold the counterexample.
- FIFO-1..3: never overflows or underflows; data out equals data in, in order
  (two-token method); flags correct.
- TIMER-1: the only way `reached(NOW, TD)` falls with `TD` unchanged is
  `NOW - TD` passing `0x7FFF -> 0x8000`, where the half-window wraps. (The
  first wording, "`reached` is monotone: once true it stays true until TD
  changes", is false and was proved false: formal finding F-1, SEMANTICS 4.)
- TIMER-2: thread `t`'s `lat_fire` is high only for an edge at which `NOW`
  ticks or at which `t`'s own slot writes `TD`; a host `TD` write alone never
  raises it, whatever it writes and to whichever thread (D-028).
- SPI-1: with SCK period >= 8 clocks, every MOSI byte is delivered exactly
  once; CS high resets the byte counter. Holds given that the chip is not
  reset in the middle of a transaction; without that assumption a reset
  while SCK is high inserts a phantom edge (formal finding F-3), which is
  why HOST_PROTOCOL tells a host to release CS_n around a reset.
- PIN-1: a BIDIR pin with OD_MASK set never has OE=1 with OUT=1. (Was false
  until D-023 put the mask in the pads: formal finding F-2, BUGS 5.)
- ISA-1: every 16-bit pattern decodes to exactly one instruction class
  (generated from `isa.yaml`); reserved patterns decode to NOP and set the bad
  opcode bit.
- WAIT-1: a timed wait terminates within `TD-NOW` **ticks**, and so within
  `TD-NOW+2` of the thread's slots while a tick is at most one slot
  (`TICK_INT <= 4`; the reset period is 1, SEMANTICS 5). Checked as a
  bounded-liveness counter on all four threads: **bounded, depth 40**, with
  the tick period at its reset value. The original wording, "within
  `TD-NOW+2` slots" with no condition on the period, counts the deadline
  distance in ticks and the budget in slots and is false — a tick is
  `TICK_INT + TICK_FRAC/256` clocks against a slot's 4: formal finding F-5.
- WAIT-1A: the completion rule itself (SEMANTICS 6.4): a `WAITD` is `done`
  exactly when `reached(NOW, TD')`, a re-issue leaves `TD` where the first
  issue put it, and a wait-class slot that stalls leaves `PC` unchanged.

Proof depth and engines are recorded per property in `formal/README.md`. A
property that only reaches bounded depth is listed as bounded, not proven.

### L5: physical

- PHY-STA: no setup or hold violations at 20 ns on the final netlist across the
  corners the TT flow checks; report at 16.7 ns as information.
- PHY-AREA: utilisation and cell count tracked per milestone in `docs/AREA.md`.
- PHY-GL: the L3 suite runs on the gate-level netlist through the TT `gl_test`
  job (cell models, functional).
- PHY-PRECHECK: TT precheck clean on every push to main.

### L6: FPGA and bench

**Not done, by decision (D-024, 2026-09-21).** The full design does not fit
the iCEBreaker's iCE40UP5K (7,732 LUT4 against 5,280), and the project will
not buy a larger board. Nothing runs on hardware before the Tiny Tapeout
silicon, which arrives after the submission deadline; these IDs stay listed
so the report shows them as open, and they become the chip's bring-up plan.

- FPGA-BUILD: iCEBreaker bitstream from the same RTL, imem in block RAM,
  host over the Pico SPI bridge.
- FPGA-COSIM: `loomhost` single-steps a random program on the FPGA and
  compares every step to `loomsim`. Same script as L2 with a different
  transport.
- BENCH-UART, BENCH-SPI, BENCH-I2C: real USB-UART adapter, real SPI flash,
  real I2C EEPROM on the PMODs, captured on a logic analyser.
- BENCH-USB (stretch): enumeration on a PC with the FPGA behind 1.5 kOhm
  pull-up and series resistors.

### L7: mutation testing

- MUT-RUN: `tools/mutate` generates mutants of `src/*.v` (operator swaps,
  constant edits, condition inversions, stuck-at on control bits, off-by-one on
  comparators), runs L1 + L2-DIR on each, and reports killed / survived.
- MUT-TARGET: at least 90 percent killed on the core, timer, pins, BE, FIFO,
  SPI modules. Every survivor is either an equivalent mutant (documented) or a
  new check ID.

### L8: methodology record

- METH-1: `docs/VERIFICATION_REPORT.md` states what was generated by AI
  (RTL, tests, models), what was reviewed by a human, and what the independent
  checks were (the golden model and RTL were written in separate sessions from
  the spec, never from each other; the mutation tool and formal properties were
  written from the spec, not from the RTL).
- METH-2: the bug ledger with per-layer counts.

## CI matrix

| Job | Trigger | Content | Time budget |
|---|---|---|---|
| `test` (TT template) | push | Icarus, cocotb: L1 + L2-DIR + L3 required protocols | < 15 min |
| `lint` | push | L0-LINT, L0-SYNTH, L0-GEN, L0-IFACE | < 3 min |
| `cosim` | push | Verilator: L2-RAND 10k programs, coverage upload | < 20 min |
| `formal-quick` | push | bounded runs of every property, depth 20 | < 15 min |
| `formal-full` | nightly | full induction / pdr | hours |
| `mutation` | nightly | L7 | hours |
| `gds` (TT) | push | hardening, precheck, GL test, viewer | ~30 to 60 min |
| `fpga` | manual | bitstream artefact | minutes |

## Where the checks live, 2026-09-19

The plan above is the target; this is what exists and where, so a reader can
run it.

| Check | Implemented in | State |
|---|---|---|
| L0 static | `scripts/check_all.sh`, CI `lint` | Verilator `-Wall` on the TT top and the generated decoder |
| L1 unit | `test/test_{host,alu,ctrl,pins,timing,uart,fifo,irq,be,setpd,mem}.py` | 92 cocotb tests (8 added with slice A, `test_be.py`, whose reference is a Python transcription of SEMANTICS 6.9.1; 14 with slice B, `test_mem.py`), all of which also run on the netlist in CI's `gl_test` |
| L2-RAND, L2-TRACE, L2-SLOT | `tools/loomgen` + `test/test_cosim.py` | lockstep on every cycle: retire record, pads, `HOST_IRQ`, guard registers, periodic full state. Both sides built from `CTRL.CAPS`, so the M2 features (FIFOs, bit engine, `SETP ... D`) are exercised, and two seeds drive the host port throughout the run. Slice A (2026-09-22): the generator drives `ENC`, `STUFF` and `DIFF` in a build with `CAPS[9]`; the default run and a 36 + 4 seed sweep at seed base 1000 (`LOOM_COSIM_SEEDS=36 LOOM_COSIM_SEED_BASE=1000 LOOM_COSIM_SPI_SEEDS=4`) both end with zero divergences between the two independently written implementations of 6.9.1, with every one of the 25 slice A bins hit. Slice B (the same day): the generator emits `LD`/`ST` into a per-thread data window in a build with `CAPS[5]`; the default run and a 36 + 4 seed sweep at seed base 2000 end with zero divergences between the two implementations of 6.11, all five `LD`/`ST` bins hit |
| L2-COV | `test/cosim_coverage.py`, `test/cosim_coverage_m2.py` | 578 bins (25 added with slice A: the encoder, stuffer and DIFF each shift ran with, `SHI` leaving T set, the `BE_CFG` fields written), 33 empty at the default run, each listed with its reason |
| L2-DEADLINE | `test/test_timing.py`, `test/test_setpd.py`, the assembler's checker | |
| L3-UART-TX/RX, L3-SPI-M, L3-SPI-S, L3-I2C-M, from 2026-09-22 L3-WS2812, L3-PS2, L3-JTAG, L3-SWD, from 2026-09-23 L3-USB-LS, L3-I2C-S, L3-CAN, and from 2026-09-24 L3-MANCH | `tools/tests/test_fw_*.py` (golden model) and `test/test_fw.py` through `test/rtl_bench.py` (RTL) | the same test bodies and the same `tools/protomodels` models on both sides; 105 scenarios on the model, 91 on the RTL (the rest marked `model_only` with the reason; all 14 of those passed on the RTL too in one run with `LOOM_FW_SET=model_only`, 2026-09-24, 351 s), no divergence between the sides |
| L4 formal | `formal/` (9 groups, `scripts/formal.sh`) | 22 properties: SCHED-1..3, FIFO-1..3, PIN-1, PIN-2, TIMER-1B/C, TIMER-2, ISA-1, ISA-2, SPI-1A..C, ISO-1, WAIT-1, WAIT-1A; unbounded where the engine closes it, k-induction otherwise, each recorded in `formal/README.md` with engine and depth. The two-copy miter `iso` is proved unbounded by `abc pdr` for all four threads with the debug port quiet, with a depth-24 BMC cross-check; `wait` is bounded at depth 40 at the reset tick period. Nothing in the list is unattempted. Four findings: F-1 (the TIMER-1 wording, corrected above), F-2 (PIN-1, a real bug, D-023), F-4 (the ISO-1 wording and the shared debug / register-file / staged-write ports) and F-5 (the WAIT-1 bound's unit); two harness findings came with slice B, F-6 (properties that took the LD/ST completion slot for an instruction) and F-7 (ISO-1 not comparing slice B's state or a thread's stores) |
| L7 mutation | `tools/mutate` (operators, runner, report; `make SRC_DIR=<mutated copy>` under it) | full pass: 823 mutants, 99.7 per cent killed with the 44 equivalents set aside, every module over MUT-TARGET; no open survivor since 2026-09-24 (the last one killed by a new `test_mem` check); the results section below |
| L5 physical, L6 FPGA | | L5 is the CI `gds` run (DRC, LVS, antenna, precheck, gate-level tests); L6 is not done, by decision (D-024): nothing runs on hardware before silicon |

## Mutation testing results (L7), 2026-09-21

`tools/mutate` generates one-line faults in a copy of `src/`, runs a cheap-first
ladder (Verilator lint, a short co-simulation, then the unit modules) and stops
at the first check that fails. The full pass covered all of `loom_timer`,
`loom_pins`, `loom_be`, `loom_fifo` and `loom_spi_host` and a stratified 300 of
`loom_core`'s 2,014 mutants: **823 mutants, 68 CPU-hours of checks**, about seven
hours of wall clock at twelve jobs. Most of that is the survivors, which run
the whole ladder (45 minutes each against two minutes for a killed mutant).

| Module | Mutants | Killed by the pass | Killed by the new tests | Equivalent | Open | Kill rate, equivalents excluded |
|---|---|---|---|---|---|---|
| `loom_be` | 181 | 181 | 0 | 0 | 0 | 100 % |
| `loom_core` | 301 | 288 | 0 | 13 | 0 | 100 % |
| `loom_fifo` | 36 | 32 | 0 | 4 | 0 | 100 % |
| `loom_pins` | 82 | 75 | 2 | 5 | 0 | 100 % |
| `loom_spi_host` | 57 | 36 | 2 | 19 | 0 | 100 % |
| `loom_timer` | 166 | 156 | 5 | 3 | 2 | 98.8 % |
| **all** | **823** | **768** | **9** | **44** | **2** | **99.7 %** |

Raw, before equivalents are set aside, the pass killed 93.3 per cent and every
module but `loom_spi_host` (63.2) met the 90 per cent MUT-TARGET. The table's
right-hand column is the honest one only because each equivalent carries its
reason in `tools/mutate/equivalents.json`, in one of two kinds:
**by construction** (no input sequence can tell the mutant from the design:
the SPI byte layer rewrites its state on every clock while CS_n is high, so its
reset values are dead; payload registers whose consumers are gated by a valid
bit that resets to 0; loop bounds past the end of a vector; a lint sink wire),
or **under a documented constraint** (the difference needs an input that
HOST_PROTOCOL or the reset sequence rules out: a host transaction across a
reset, a debug command inside the three clocks after RUN clears, MISO during
the command byte, a deadline at the far edge of `reached()`'s half window).

**What the mutants taught the suite.** Across the two rounds, eight groups of
survivors were real holes, each a promise no test compared, and each is now a
test that kills the mutants that named it (proved one mutant at a time with
`--only`). The first three were written after round 1's partial pass and were
in the ladder for the full pass, so their kills are inside its 768; the other
five came out of the full pass and account for the "new tests" column:

| Test | Hole | Mutants it kills |
|---|---|---|
| `test_host.test_debug_register_cross_talk` | a debug write reaching the wrong register, thread or an unused alias (0x1A, 0x1B) | 7 |
| `test_ctrl.test_csr_tick_frac_readback` | `CSRR TICK_FRAC` never compared (L2-COV had the bin empty) | 2 |
| `test_be.test_be_cfg_fields_readback` | `CSRR BE_CFG`'s CRC_EN bit: the old test wrote 0xFFFF | 1 |
| `test_host.test_miso_idles_low` | MISO driven 0 while CS_n is high (HOST_PROTOCOL) | 1 |
| `test_host.test_cs_rising_mid_byte_voids_the_byte` | CS_n rising mid-byte resets the framing; a word cut short is not written (formal SPI-1C proves it, no simulation checked it) | 1 |
| `test_pins.test_pin_csrs` (new asserts) | `PIN_IN[15:13]` read 0: the test read the register and never checked it | 2 |
| `test_timing.test_tick_int_zero_is_one`, `test_long_tick_period` | `TICK_INT = 0` means 1; ticks of 32,768 clocks and more (the accumulator's top bit) | 4 |
| `test_host.test_reset_sets_td_to_now` | CTRL.RESET sets TD := NOW | 1 |

**No survivor is open** (two until 2026-09-22, one until 2026-09-24). The
first, at the old `loom_timer.v:112`, let a host debug write to *another*
thread's TD (or any host write to this thread) feed this thread's
deadline-latch compare, so a staged `SETP ... D` could fire early if that
value happened to equal `NOW + 1` at a tick. D-028 removed the host terms
from that compare, so the line and its mutant no longer exist, and TIMER-2
proves what the mutant exposed: a thread's latch fire depends only on the
tick and its own TD writes. The second (`loom_timer_L0146C023_stuck_79d2`,
the `td` register's commit branch taken by every commit, not only a TD
write) was thought to differ only when CTRL.RESET hits a *running* thread,
which SEMANTICS 7 leaves undefined, and so to be an equivalent. Slice B made
that stale: an `LD`/`ST` completion slot runs its data word through the X
stage's decode, the word can encode `SETD`, and the TD value W carries is
then `NOW + imm8`; only the decode gate on the TD write enable keeps it out
of TD, which is the gate the mutant removes. So a legal program kills it,
and `test_mem.test_a_loaded_setd_word_never_moves_td` (with its model twin)
does. It had survived the re-check on 2026-09-24 only because the mutation
ladder's module list predated slice B and never ran `test_mem`; it does now
(`tools/mutate/runner.py`). Two lessons for the freeze re-run: an
equivalent's reason can be broken by a later feature, so every reason is
re-checked, not copied, and the ladder must list every L1 module.

Cost for the next pass: a run that skips the 44 recorded equivalents saves
about 33 CPU-hours of the 68, because an equivalent is a survivor and pays for
the whole ladder.

## What "done" means for a check

A check is done when: a test or proof references its ID; it passes in CI; the
first failure it ever produced is either in `docs/BUGS.md` or the check was
shown to catch a mutant. Checks that have never failed and kill no mutant are
listed separately in the report as "unproven checks", because that is what
they are.
