![](../../workflows/gds/badge.svg) ![](../../workflows/docs/badge.svg) ![](../../workflows/test/badge.svg)

# Loom: a barrel-threaded protocol emulator ASIC

An entry for the [Jane Street protocol emulator ASIC competition](https://blog.janestreet.com/protocol-emulator-asic-competition/),
built on [Tiny Tapeout](https://tinytapeout.com) for IHP's 130 nm CMOS5L
process, 6x4 tiles (the largest size the cmos5l flow accepts today; 8x4 if
Tiny Tapeout enables it). Open source, Apache-2.0.

**Status (2026-09-15): architecture set, flow scaffolded, M0 in progress.**
See `docs/PLAN.md` for milestones.

## The idea

A protocol emulator is a tiny CPU built to bit-bang: read pins, write pins,
count time, and do it with timing you can trust. Loom does that with four
hardware threads sharing one 4-stage pipeline in strict round robin. Every
instruction takes exactly one thread slot, so the timing of a program is
visible in its listing, and one thread can never disturb another's timing.

Three things set it apart from a PIO or PRU clone:

1. **Deadline-based timing.** `WAITD k` advances a per-thread deadline by k
   ticks and waits for it. Bit timing is jitter-free no matter which branch the
   code took. Every wait can carry a timeout against the same deadline, so
   "wait for SCL to rise, or give up" is one instruction.
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
