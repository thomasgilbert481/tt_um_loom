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
- L3-MANCH (M4): 10 Mbit Manchester TX/RX loopback in auto mode at 60 MHz.

### L4: formal (SymbiYosys)

- SCHED-1: at every cycle exactly one thread is in each pipeline stage and the
  four are distinct (induction).
- SCHED-2: thread t's architectural state (regs, PC, flags, TD, SR, CNT, CRC)
  changes only in its own W stage.
- SCHED-3: a thread with RUN=0 never changes architectural state except through
  host debug writes.
- ISO-1: for any two traces that agree on thread t's inputs (its pins, its
  FIFOs, SFLAGS it waits on) thread t's state sequence is identical regardless of
  what other threads do (checked as a two-copy miter on a reduced configuration).
- FIFO-1..3: never overflows or underflows; data out equals data in, in order
  (two-token method); flags correct.
- TIMER-1: the only way `reached(NOW, TD)` falls with `TD` unchanged is
  `NOW - TD` passing `0x7FFF -> 0x8000`, where the half-window wraps. (The
  first wording, "`reached` is monotone: once true it stays true until TD
  changes", is false and was proved false: formal finding F-1, SEMANTICS 4.)
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
- WAIT-1: a timed wait terminates within TD-NOW+2 slots (bounded liveness).

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
| L1 unit | `test/test_{host,alu,ctrl,pins,timing,uart,fifo,irq,be,setpd}.py` | 70 cocotb tests, all of which also run on the netlist in CI's `gl_test` |
| L2-RAND, L2-TRACE, L2-SLOT | `tools/loomgen` + `test/test_cosim.py` | lockstep on every cycle: retire record, pads, `HOST_IRQ`, guard registers, periodic full state. Both sides built from `CTRL.CAPS`, so the M2 features (FIFOs, bit engine, `SETP ... D`) are exercised, and two seeds drive the host port throughout the run |
| L2-COV | `test/cosim_coverage.py`, `test/cosim_coverage_m2.py` | 553 bins, 33 empty at the default run, each listed with its reason |
| L2-DEADLINE | `test/test_timing.py`, `test/test_setpd.py`, the assembler's checker | |
| L3-UART-TX/RX, L3-SPI-M, L3-SPI-S, L3-I2C-M | `tools/tests/test_fw_*.py` (golden model) and `test/test_fw.py` through `test/rtl_bench.py` (RTL) | the same test bodies and the same `tools/protomodels` models on both sides; 37 scenarios on the model, 29 on the RTL |
| L4 formal | `formal/` (7 groups, `scripts/formal.sh`) | 18 properties: SCHED-1..3, FIFO-1..3, PIN-1, PIN-2, TIMER-1B/C, ISA-1, ISA-2, SPI-1A..C; unbounded where the engine closes it, k-induction otherwise, each recorded in `formal/README.md` with engine and depth. Two findings: F-1 (the TIMER-1 wording, corrected above) and F-2 (PIN-1, a real bug, D-023). ISO-1 and WAIT-1 open |
| L7 mutation, by hand | `make SRC_DIR=<mutated copy>` | five mutants tried, four caught at seed 1 or 2 by co-simulation, one documented as equivalent in practice (`test/README.md`). `tools/mutate` itself is not written |
| L5 physical, L6 FPGA | | L5 is the CI `gds` run (DRC, LVS, antenna, precheck, gate-level tests); L6 is open |

## What "done" means for a check

A check is done when: a test or proof references its ID; it passes in CI; the
first failure it ever produced is either in `docs/BUGS.md` or the check was
shown to catch a mutant. Checks that have never failed and kill no mutant are
listed separately in the report as "unproven checks", because that is what
they are.
