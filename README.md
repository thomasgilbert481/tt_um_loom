![](../../workflows/gds/badge.svg) ![](../../workflows/docs/badge.svg) ![](../../workflows/test/badge.svg)

# Loom: a barrel-threaded protocol emulator ASIC

An entry for the [Jane Street protocol emulator ASIC competition](https://blog.janestreet.com/protocol-emulator-asic-competition/),
built on [Tiny Tapeout](https://tinytapeout.com) for IHP's 130 nm CMOS5L
process, 6x4 tiles (the largest size the cmos5l flow accepts today; 8x4 if
Tiny Tapeout enables it). Open source, Apache-2.0.

**Status (2026-09-18): M1 core hardened; M2 in progress.** See `docs/PLAN.md`
for milestones and the session log, `docs/DECISIONS.md` for why things are the
way they are.

## Results so far

| What | Result | Where |
|---|---|---|
| M1 core, 6x4 tiles, hardened in CI | DRC, LVS and antenna clean; Tiny Tapeout precheck and gate-level test pass; 78.8% utilisation; setup slack +1.45 ns at 50 MHz (typical corner) | `docs/AREA.md`, D-019 |
| RTL against an independently written golden model | lockstep co-simulation on random programs, compared every clock cycle: zero divergences; two injected bugs caught in the first seed | `test/test_cosim.py`, `test/README.md` |
| Directed tests through the pins (SPI host port) | 45 cocotb tests, also run on the gate-level netlist in CI | `test/` |
| Toolchain | assembler with a static deadline checker, cycle-accurate golden model, ISA generator, random program generator: 740 Python tests | `tools/` |
| IHP SRAM macro on cmos5l | 512x16 macro passes hardening, all nine precheck checks (0 DRC violations over the macro) and gate-level test; to our knowledge the first public cmos5l SRAM result; recipe written up | branch `sram-smoke`, `docs/tt_cmos5l_facts.md` section 11 |

Two findings changed the design along the way. Firmware-driven edges land on a
thread's 4-clock slot grid, so the edges dither by up to 3 clocks at arbitrary
tick periods (no drift); the answer is a deadline-latched pin write,
`SETP pin, v, D`, which lands on the exact clock of the deadline (D-016). And
the flop instruction memory was half the chip, which the SRAM macro solves
(D-020).

## The idea

A protocol emulator is a tiny CPU built to bit-bang: read pins, write pins,
count time, and do it with timing you can trust. Loom does that with four
hardware threads sharing one 4-stage pipeline in strict round robin. Every
instruction takes exactly one thread slot, so the timing of a program is
visible in its listing, and one thread can never disturb another's timing.

Three things set it apart from a PIO or PRU clone:

1. **Deadline-based timing.** `WAITD k` advances a per-thread deadline by k
   ticks and waits for it, so a schedule never drifts no matter which branch
   the code took, and the assembler proves statically that every deadline can
   be met. Firmware-driven edges land within one slot (4 clocks) of the
   deadline; deadline-latched pin writes make them clock-exact. Every wait can
   carry a timeout against the same deadline, so "wait for SCL to rise, or give
   up" is one instruction.
2. **Bit engines.** Each thread owns a shift register with a programmable CRC,
   NRZ/NRZI/Manchester coding and USB/CAN bit-stuffing, usable one bit at a
   time under firmware control or autonomously at tick rate. That is what makes
   USB low-speed and 10 Mbit Manchester reachable from firmware.
3. **Verification as a product.** The ISA lives in one YAML file that generates
   the decoder tables, the assembler, the golden model, the docs and the formal
   decode properties. The same host protocol drives RTL simulation, the FPGA
   prototype and the silicon, so one script single-steps all three against the
   model. The testbench is scored by mutation testing and every bug it found is
   in a public ledger.

Required protocols (UART, SPI, I2C) run as firmware on the same core, along
with device emulation (I2C EEPROM, SPI flash), WS2812, PS/2, JTAG and SWD.
Stretch: USB low-speed device, CAN, 10 Mbit Manchester.

## Documents

| File | What |
|---|---|
| `docs/ARCHITECTURE.md` | the spec: execution model, timing, pins, bit engine, ISA, memory, budget |
| `docs/HOST_PROTOCOL.md` | SPI host port, register map, debug and single-step |
| `docs/VERIFICATION.md` | the plan: layers L0 to L8, check IDs, CI matrix |
| `docs/PLAN.md` | milestones and dates to the 2027-01-18 deadline |
| `docs/DECISIONS.md` | why each major choice was made and what was rejected |
| `docs/info.md` | Tiny Tapeout datasheet page |
| `CLAUDE.md` | working rules for the AI sessions that implement this repo |

## Repository layout

```
src/        RTL (Verilog-2005 subset), tt_um_loom.v is the Tiny Tapeout top
isa/        isa.yaml, the single source of truth for the instruction set
tools/      gen (codegen), loomasm (assembler), loomsim (golden model), loomhost (host library), mutate
firmware/   .loom programs and their build outputs
test/       cocotb tests, protocol reference models, co-simulation harness
formal/     SymbiYosys jobs and properties
fpga/       iCEBreaker build
docs/       everything listed above
```

## Building and testing

The Tiny Tapeout GitHub workflows build the GDS (`gds`), the datasheet
(`docs`) and run the cocotb tests (`test`) on every push. Locally:

```bash
cd test && make            # cocotb + Icarus Verilog
```

See `CLAUDE.md` for the WSL invocation used on the development laptop.

## Acknowledgements

Tiny Tapeout and the IHP open PDK make this possible. Jane Street's own
hardware work uses [Hardcaml](https://hardcaml.org/); Loom is written in plain
Verilog with a generated ISA layer, which gets some of the same
one-description-many-artefacts benefit on a Windows laptop with open tools.

## Authors

Thomas Gilbert (NC State ECE). AI-assisted: architecture and plan by Claude
Fable 5.1, implementation with Claude Opus 5; the human review points and the
independence between model and RTL are described in `docs/VERIFICATION.md`.
