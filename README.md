![](../../workflows/gds/badge.svg) ![](../../workflows/docs/badge.svg) ![](../../workflows/test/badge.svg)

# Loom: a barrel-threaded protocol emulator ASIC

An entry for the [Jane Street protocol emulator ASIC competition](https://blog.janestreet.com/protocol-emulator-asic-competition/),
built on [Tiny Tapeout](https://tinytapeout.com) for IHP's 130 nm CMOS5L
process, 6x4 tiles. Open source, Apache-2.0.

Loom is a small programmable I/O processor for bit-level protocols. Four
hardware threads share one pipeline in strict round robin, each with its own
deadline timebase and bit engine, so firmware can bit-bang a protocol with
timing that is exact and checked before the program is loaded. A host (the
Tiny Tapeout demo board's microcontroller, or any SPI master) loads programs
and moves data through a four-pin SPI port.

**Status (2026-10-06):** the hardware is complete, hardened and verified.
The one optional feature, an autonomous mode for the bit engine, was left
out (D-032): no program needs it and the routing budget has no room for it.
The RTL freezes on 2026-11-08, after which it changes only for a bug.
`docs/PLAN.md` has the milestones and the session log, `docs/DECISIONS.md`
the reasons for each choice.

## Results

| What | Result | Where |
|---|---|---|
| Hardened in CI, 6x4 tiles | 31,197 standard cells and one 512x16 IHP SRAM macro, 54.8 % utilisation; DRC, LVS and antenna clean; Tiny Tapeout precheck passes; gate-level tests 109 of 109 | `docs/AREA.md`, run 37475045879 |
| Clock | 50 MHz with +5.16 ns of setup slack at the typical corner (the flow's sign-off corner); about 42 MHz at the slow corner (1.08 V, 125 C) | `docs/AREA.md` |
| Protocols | thirteen firmware programs, each tested on the golden model and on the RTL through the real SPI pads with the same test bodies: 109 scenarios on the model, 95 of them on the RTL in every CI run and the other 14 (slow cases) in a separate run. One runs four of the programs at once, one per thread, and checks that every pin edge of each lands on the same clock as when it runs alone | `firmware/README.md`, `test/test_fw.py` |
| RTL against an independent golden model | lockstep co-simulation compared on every clock cycle, on constrained-random programs with the host port driven during the run, in every CI run | `test/test_cosim.py` |
| Formal (SymbiYosys) | properties of the scheduler, FIFOs, timer, pins, SPI port, decoder and wait rule, proved for every depth (PDR or k-induction) apart from one bound checked to depth 40; thread isolation (a thread's state does not depend on what the other three run) proved for each of the four threads on a two-copy miter. Four properties were false as first worded; each is a recorded finding, one of them a real bug | `formal/README.md` |
| Mutation testing | the freeze-time run: 911 one-line faults in the RTL, 99.9 % killed with 60 documented equivalents set aside, after it found eleven holes in the tests (closed) and one real bug (BUGS 11, fixed) | `docs/VERIFICATION.md` L7 |
| Test counts | 209 cocotb tests on the RTL, most of them through the pins; 1,676 Python tests on the tools and the golden model | `test/`, `tools/tests/` |
| IHP SRAM macro on cmos5l | the 512x16 macro hardens and passes all nine precheck checks and the gate-level test; as far as we know the first published cmos5l SRAM result that passes the Tiny Tapeout precheck; the recipe is written up | `docs/tt_cmos5l_facts.md` section 11 |

`docs/VERIFICATION_REPORT.md` puts the evidence together, with what each
layer would miss.

Some findings changed the design along the way:

- Firmware-driven edges land on a thread's four-clock slot grid, so at an
  arbitrary tick they dither by up to three clocks (without drift). The
  answer is a deadline-latched pin write, `SETP pin, v, D`, which lands on
  the exact clock of the deadline (D-016).
- The flop instruction memory was half the chip; the SRAM macro replaced it
  (D-020).
- Routing time, not area, bounds the design: two per cent more cells in one
  corner pushed detailed routing past GitHub's six-hour job limit. Every
  hardware change since has had to route in under four hours (D-025).
- A formal proof found a real bug: an open-drain pin could drive high on a
  shared bus, depending on the order three registers were written in
  (`docs/BUGS.md` 5, D-023).
- The deadline checker's own rule was unsound after a `SETD`, and it hid
  real timing faults in three of the shipped programs. The rule, the three
  programs and an edge-by-edge test for each were fixed together (T-1,
  `docs/VERIFICATION.md`).

## The idea

A protocol emulator is a tiny CPU built to bit-bang: read pins, write pins,
count time, and do it with timing you can trust. Loom does that with four
hardware threads sharing one four-stage pipeline in strict round robin.
Every instruction takes one slot of its own thread (`LD` and `ST` take two),
so the timing of a program is visible in its listing, and one thread can
never disturb another's timing.

Three things set it apart from a PIO or PRU clone:

1. **Deadline-based timing.** `WAITD k` advances a per-thread deadline by k
   ticks and waits for it, so a schedule never drifts whichever branch the
   code took, and the assembler checks statically that every deadline can
   be met. Firmware-driven edges land within one slot (four clocks) of the
   deadline; deadline-latched pin writes make them clock-exact. Every wait
   can carry a timeout against the same deadline, so "wait for SCL to rise,
   or give up" is one instruction.
2. **Bit engines.** Each thread owns a 16-bit shift register with a bit
   counter, a programmable CRC, NRZ/NRZI/Manchester coding, USB and CAN bit
   stuffing with violation detection, and a differential output, driven one
   instruction per bit. That is what makes USB low speed reachable from
   firmware.
3. **Verification as a product.** The ISA lives in one YAML file: the
   decoder, the constants header, the ISA document and the formal decode
   properties are generated from it (CI fails if any is stale), and the
   assembler and the golden model read it directly. The RTL and the golden
   model were written from one cycle-exact contract, `docs/SEMANTICS.md`,
   by sessions that were not allowed to read each other's code. One host
   library drives the model and the RTL simulation, and will drive the
   silicon through the same calls, so a test body runs unchanged on both
   backends. The testbench is scored by mutation testing, and every bug
   verification found is in a public ledger, `docs/BUGS.md`.

## Firmware

| Program | What | Tested at (50 MHz clock) |
|---|---|---|
| `uart_tx_fifo`, `uart_rx` | 8N1 UART | 115200 baud; 1.5625 Mbaud (TX), 781 kbaud (RX) |
| `spi_master`, `spi_slave` | SPI master (modes 0 to 3, MSB or LSB first), SPI slave (modes 0 and 3) | 781 kHz SCK (master), 390 kHz (slave) |
| `i2c_master` | I2C master: NACK, clock stretching, stuck-SCL timeout | 390 kHz SCL |
| `i2c_slave_eeprom` | a 24C02-style I2C EEPROM, 256 bytes | 100 and 400 kHz SCL |
| `ws2812` | WS2812B LED driver | the datasheet waveform |
| `ps2_host` | PS/2 host | 10 and 16.67 kHz device clock |
| `jtag_master` | JTAG: TAP reset and IDCODE | 781 kHz TCK |
| `swd_master` | SWD: JTAG-to-SWD switch and DPIDR | 781 kHz SWCLK |
| `can_loopback` | a CAN 2.0A node, transmitter and receiver on two threads | 500 and 125 kbit/s |
| `usb_ls_device` | a USB low-speed HID boot mouse: enumeration and an interrupt IN endpoint | 1.5 Mbit/s |
| `manchester_loopback` | Manchester transmitter and receiver on two threads | 2.083 and 1 Mbit/s |

`firmware/README.md` has the pins, sizes, proved ticks and error cases of
each; `uart_hello` and `uart_tx` are the two small demos of the first
milestone. To assemble a program and run it on the golden model:

```
python -m tools.loomasm firmware/uart_tx_fifo.loom --strict --listing
python -m tools.loomhost --model firmware/uart_tx_fifo.loom --uart-rx OUT0:32 \
    load "csr 0 TICK_INT 32" "run 0" "push 0 'Hello'" "idle 3000"
```

`docs/info.md` (the Tiny Tapeout datasheet page) shows the same on the chip.

## Limitations

- Nothing has run on hardware. The FPGA prototype was dropped when the design
  did not fit the board on hand (D-024), so the silicon is the first
  hardware; `docs/VERIFICATION_REPORT.md` section 2 says what stands in for
  a bench.
- The slow corner closes at about 42 MHz, not 50 MHz.
- The bit engine has no autonomous mode (D-032): every bit costs its thread
  at least one instruction slot. The fastest rate a shipped program is
  tested at is 2.083 Mbit/s, the Manchester loopback's.
- Thread isolation is proved with the host's debug port quiet, and it takes
  as given that no other thread stores into the memory the thread reads.
  With the debug port in use it is false by design: the port is shared (F-4).
- The USB device fills the whole 512-word memory, so it runs alone. The CAN
  receiver resynchronises only at the start of a frame, so a sender must be
  within about 0.2 per cent of the bit rate.
- Three intervals, two in `ws2812` and one in `uart_tx_fifo`, are declared
  bounded in the source (a `POP` just after a wait that found the FIFO not
  empty), and the deadline checker believes the declaration rather than
  proving it.
- 10 Mbit Manchester and 10 Mbit Ethernet were not attempted (D-029).
- The host tool's `--ttboard` transport fits the RP2040 demo board only; the
  v3 board (RP2350B) needs a transfer that is not written yet.

## Documents

| File | What |
|---|---|
| `docs/ARCHITECTURE.md` | the design: execution model, timing, pins, bit engine, memory |
| `docs/SEMANTICS.md` | the cycle-exact contract for the RTL and the golden model |
| `docs/ISA.md` | the instruction set, generated from `isa/isa.yaml` |
| `docs/HOST_PROTOCOL.md` | the SPI host port: register map, FIFOs, debug and single-step |
| `docs/INTERFACES.md` | every port of every RTL module, and the cycle it is valid in |
| `docs/VERIFICATION.md` | the verification plan: layers L0 to L8, check IDs, CI matrix |
| `docs/VERIFICATION_REPORT.md` | the evidence, per layer, and what it would miss |
| `docs/BUGS.md` | every bug found, with the check that caught it |
| `docs/AREA.md` | every hardening: cells, timing, routing time |
| `docs/DECISIONS.md` | why each major choice was made and what was rejected |
| `docs/PLAN.md` | milestones and dates to the 2027-01-18 deadline |
| `docs/tt_cmos5l_facts.md` | what the cmos5l flow and the demo board actually do, with sources |
| `docs/info.md` | the Tiny Tapeout datasheet page |
| `CLAUDE.md` | working rules for the AI sessions that implement this repo |

## Repository layout

```
src/        RTL (Verilog-2005 subset); tt_um_loom.v is the Tiny Tapeout top,
            loom_decode.v is generated from isa/isa.yaml
macro/      the IHP 512x16 SRAM macro views
isa/        isa.yaml, the single source of truth for the instruction set
tools/      loomisa (ISA loader and generator), loomasm (assembler and
            deadline checker), loomsim (golden model), loomgen (random
            programs), loomhost (host library, CLI and MicroPython bridges),
            protomodels (protocol models), mutate (mutation testing), tests
firmware/   the .loom programs
test/       cocotb tests through the pins, co-simulation, firmware on the RTL
formal/     SymbiYosys properties and their record
scripts/    the local checks, formal runs, gate-level runs, hardening reports
fpga/       the iCEBreaker build, kept to show it does not fit (D-024)
docs/       everything listed above
```

## Building and testing

The Tiny Tapeout GitHub workflows build the GDS (`gds`, when `src/`,
`info.yaml` or `macro/` change), the datasheet (`docs`) and run the cocotb
tests (`test`) on every push. Locally, in a Linux shell with OSS CAD Suite:

```bash
bash scripts/check_all.sh      # ISA checks, Python tests, Verilator lint, cocotb
bash scripts/formal.sh quick   # the formal proofs
```

See `CLAUDE.md` for the WSL invocation used on the development laptop.

## Acknowledgements

Tiny Tapeout and the IHP open PDK make this possible. Jane Street's own
hardware work uses [Hardcaml](https://hardcaml.org/); Loom is written in plain
Verilog with a generated ISA layer, which gets some of the same
one-description-many-artefacts benefit on a Windows laptop with open tools.

## Authors

Thomas Gilbert (NC State ECE). AI-assisted: architecture and plan by Claude
Fable 5.1, implementation with Claude Opus 5 and 5.5; the human review
points and the independence between model and RTL are described in
`docs/VERIFICATION.md`.
