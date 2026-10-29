![](../../workflows/gds/badge.svg) ![](../../workflows/docs/badge.svg) ![](../../workflows/test/badge.svg)

# Loom: a barrel-threaded protocol emulator ASIC

An entry for the [Jane Street protocol emulator ASIC competition](https://blog.janestreet.com/protocol-emulator-asic-competition/),
built on [Tiny Tapeout](https://tinytapeout.com) for IHP's 130 nm CMOS5L
process, 8x4 tiles (D-034). Open source, Apache-2.0.

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
| Hardened in CI, 8x4 tiles | 31,268 standard cells and one 512x16 IHP SRAM macro, 41.1 % utilisation; DRC, LVS and antenna clean; Tiny Tapeout precheck passes; gate-level tests 109 of 109; detailed routing 2 h 52 min, the build job 4 h 04 min of GitHub's six hours | `docs/AREA.md`, run 37547764383 |
| Clock | 50 MHz with +5.49 ns of setup slack at the typical corner (the flow's sign-off corner); about 43 MHz at the slow corner (1.08 V, 125 C) | `docs/AREA.md` |
| Protocols | twenty-six firmware programs, thirteen written with the hardware and thirteen after it was complete for protocols Loom was not designed for (NEC infrared, 1-Wire, LIN, HDLC, Wiegand, DMX512, DShot), each tested on the golden model and on the RTL through the real SPI pads with the same test bodies: 275 scenarios on the model, 166 of them on the RTL in every CI run and the other 109 (slow cases, each with its reason) on the model only. One runs four of the programs at once, one per thread, and checks that every pin edge of each lands on the same clock as when it runs alone | `firmware/README.md`, `test/test_fw.py` |
| RTL against an independent golden model | lockstep co-simulation compared on every clock cycle, on constrained-random programs with the host port driven during the run, in every CI run | `test/test_cosim.py` |
| Formal (SymbiYosys) | properties of the scheduler, FIFOs, timer, pins, SPI port, decoder and wait rule, proved for every depth (PDR or k-induction) apart from one bound checked to depth 40; thread isolation (a thread's state does not depend on what the other three run) proved for each of the four threads on a two-copy miter. Four properties were false as first worded; each is a recorded finding, one of them a real bug | `formal/README.md` |
| Mutation testing | the freeze-time run: 911 one-line faults in the RTL, 99.9 % killed with 60 documented equivalents set aside, after it found eleven holes in the tests (closed) and one real bug (BUGS 11, fixed) | `docs/VERIFICATION.md` L7 |
| Test counts | 283 cocotb tests on the RTL, most of them through the pins (CI run 40821101047); 1,905 Python tests on the tools and the golden model | `test/`, `tools/tests/` |
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
| `spi_master`, `spi_slave` | SPI master and slave, modes 0 to 3 (master MSB or LSB first) | 781 kHz SCK (master), 390 kHz (slave) |
| `i2c_master` | I2C master: standard and fast mode, NACK, clock stretching, stuck-SCL timeout; every UM10204 table 10 bound measured on the pins | 100 kHz and 390.6 kHz SCL |
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

## Protocols written after the hardware was complete

The test of a protocol emulator is the protocol nobody planned for. After
the hardware was complete (D-032) four protocols that nothing in Loom was
built for were written as firmware only, with no RTL change (D-035): each
by one agent in its own worktree, all four started at 11:11 EDT on
2026-10-11, then reviewed and integrated by the director. Three more
(Wiegand, DMX512 and DShot) followed the same way, their agents started
at 23:12 EDT on 2026-10-28. Each meets the
shipped programs' bar: `--strict` assembly (every deadline proved), a
reference model written from the specification rather than from the
firmware, the same test bodies on the golden model and on the RTL through
the SPI pads, a timing-annotated gate-level scenario (PHY-GL-SDF) and a
header saying what hardware it reuses.

| Protocols | Programs (words) | Rates tested | Scenarios: model, RTL | Hours to green |
|---|---|---|---|---|
| NEC infrared | `ir_nec_tx` (71), `ir_nec_rx` (112) | 38 kHz carrier and NEC frames at 50 MHz; RTL on a 1.824 MHz board, the same waveform in microseconds | 13, 4 | 1.9 |
| 1-Wire | `onewire_master` (157), `onewire_device` (157) | standard speed, 1 us tick, AN126 slots; also a 10-clock tick | 20, 4 | 1.7 |
| LIN | `lin_master` (128), `lin_slave` (218) | 9600, 10417 and 19200 bit/s; a master 14 % off for the auto-baud slave | 30, 6 | 2.0 |
| HDLC | `hdlc_loopback` (279, two threads) | 64 and 329 kbit/s NRZI; a sender up to 3 % off | 14, 6 | 2.0 |
| Wiegand | `wiegand_reader` (107), `wiegand_panel` (109) | SIA AC-01's 20 to 100 us pulses and 200 us to 20 ms intervals; RTL on a 5 MHz board | 27, 24 | 0.4 |
| DMX512 | `dmx_controller` (60), `dmx_fixture` (127) | 250 kbit/s, packets up to 512 slots; a sender 2 % off, short breaks and framing errors | 14, 4 | 0.5 |
| DShot | `dshot_tx` (65), `dshot_bidir` (154) | DShot600, 300 and 150 at 50 and 48 MHz; bidirectional DShot600 and 300 with eRPM telemetry | 19, 12 | 0.6 |

Hours to green run from the session start to the first run green on both
backends (`PROGRESS.md` timestamps); the summaries, recounts and some
re-runs took later sessions after the account's session limit cut the
agents off.

- **NEC infrared.** Loom has no PWM, carrier generator or pulse capture.
  The transmitter makes the 38 kHz carrier from deadline-latched pin
  writes (`SETP ... D`, built for clock-exact WS2812 pulses) on a
  fractional tick (built for UART rates). The receiver measures each mark
  and space with timed waits (built for the PS/2 and I2C timeouts) and
  re-phases its tick on every edge, the hard synchronisation built for the
  UART, CAN and Manchester receivers.
- **1-Wire.** One open-drain wire with timed slots, a reset and presence
  pulse and a binary-tree ROM search. The master and a DS18B20-style
  device reuse the open-drain BIDIR pin (built for I2C), `SETP ... D`,
  the shift register and the CRC unit (built for CAN and USB, here
  Maxim's CRC-8) and `LD`/`ST` (built for the EEPROM). The slots, the
  presence detection and application note 187's search are firmware.
- **LIN.** The car-body bus: a 13-bit break, a sync byte and a checksum
  with an end-around carry. The master reads back every bit it sends. The
  slave does not know the bus rate: it measures the sync byte with timed
  waits and `NOW` and writes its own `TICK_INT`, a divider built so the
  host could pick a rate, so the clock recovery is firmware.
- **HDLC.** Bit-stuffed frames with an FCS-16. It reuses the bit
  engine's NRZI encoder and decoder (built for USB), the CRC unit for the
  FCS and `LD`/`ST` for the frame buffers. HDLC stuffs a 0 after five 1s,
  a rule the stuffing hardware does not know (it knows USB's and CAN's),
  so the firmware inserts and removes the stuff bits and finds flags and
  aborts.
- **Wiegand.** The access-control link from a card reader to a door
  controller: a low pulse on D0 for a 0 and on D1 for a 1. Loom has no
  pulse counter on two pins and no frame timeout. The reader reuses the
  open-drain BIDIR pins (built for I2C) and `SETP ... D` (built for
  WS2812), so every pulse width and interval is exact to the clock. The
  panel measures pulses with timed waits (built for the PS/2 and I2C
  timeouts) and the tick restart (built for the UART receiver), and ends
  a frame on `NOW`; the bit count, the glitch and both-low checks and the
  26-bit parities are firmware.
- **DMX512.** The stage-lighting bus: a break, a mark after break and up
  to 512 slots at 250 kbit/s. The controller sends each 8N2 slot through
  the bit engine's shift register (built for the SPI, JTAG and SWD
  programs and the USB and CAN encoders); the fixture finds the break by
  its length with timed waits and keeps a host-set window of slots. The
  break, the timing between packets and the slot window are firmware.
- **DShot.** The digital throttle protocol between a drone's flight
  controller and its motor ESCs, which has no PWM or pulse-width encoder
  in Loom. The transmitter makes every pulse with `SETP ... D` on a
  fractional tick (built for UART rates). The bidirectional version
  releases the line and decodes the ESC's GCR telemetry with the bit
  engine's NRZI decoder (built for USB) and a 5b/4b table read with `LD`
  (built for device emulation).

`firmware/README.md` has each program's pins, host words, proved ticks and
what it does not do; `docs/spec-questions/firmware-d035.md` has the open
questions and the decisions taken.

## Limitations

- Nothing has run on hardware. The FPGA prototype was dropped when the design
  did not fit the board on hand (D-024), so the silicon is the first
  hardware; `docs/VERIFICATION_REPORT.md` section 2 says what stands in for
  a bench.
- The slow corner closes at about 43 MHz, not 50 MHz.
- The I/O pads and Tiny Tapeout's multiplexer sit outside the hardened
  tile, so no simulation here includes their delays.
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
- The host transfers have run only in simulation: the demo-board helper's
  own code against the RTL, the Pico bridge against a fake serial port.

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
