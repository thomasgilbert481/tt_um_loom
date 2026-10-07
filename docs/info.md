<!---

This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

## How it works

Loom is a small programmable I/O processor for bit-level protocols: firmware
decides what the pins do, and the chip's timing model makes that firmware's
timing exact and checkable before it runs.

- **Four hardware threads** share one four-stage pipeline in strict round
  robin. Every instruction takes one slot of its own thread, four clocks
  (`LD` and `ST` take two), whatever the other threads do, so the timing of
  a program can be read off its assembler listing. There are no caches and
  no stalls except the waits a program asks for.
- **A timebase per thread**: a tick of `TICK_INT + TICK_FRAC/256` clocks, a
  tick counter `NOW` and a deadline register `TD`. `SETD m` anchors the
  deadline at `NOW + m`, and `WAITD k` moves it on by `k` ticks and waits
  for it, so edges land on the tick grid whatever path the code took. The
  timed waits (for a pin level, an edge, a shared flag, a FIFO or the bit
  engine) give up at the same deadline, and `SETP pin, v, D` stages a pin
  write that is applied exactly at the next one. The assembler checks every
  deadline against the tick before the program is loaded: it warns about a
  path it cannot bound, and with `--strict` it writes no image for a
  schedule that can miss a deadline.
- **A bit engine per thread**, used one instruction per bit: a 16-bit shift
  register with a bit counter, a programmable CRC of up to 16 bits,
  NRZ/NRZI/Manchester coding, USB and CAN bit stuffing with a violation
  flag, and a differential output for USB's D+ and D-.
- **Memory and FIFOs**: a 512 x 16 SRAM holds the four threads' programs and
  their data (`LD`/`ST`); each thread has a four-word input and output FIFO
  to the host.
- **The host port** is an SPI slave (mode 0) on `ui[4]` (CS_n), `ui[5]`
  (SCK), `ui[6]` (MOSI) and `uo[7]` (MISO), with an interrupt line on
  `uo[6]`. Through it the host loads programs, moves data through the FIFOs,
  runs, halts and single-steps the threads, and reads or writes every
  register.
- **The protocol pins** are the other 19, one index space for firmware:
  IN0..IN4 (`ui[0..3]`, `ui[7]`), OUT0..OUT5 (`uo[0..5]`) and
  BIDIR0..BIDIR7 (`uio[7:0]`), each bidirectional pin with its own
  open-drain mode.

Thirteen protocol programs come with the design (besides two small UART
demos from the first milestone), each with a test whose bodies run both on
the golden model and on the RTL through the real SPI pads: UART transmit
and receive, SPI master and slave, I2C master, a 24C02-style I2C EEPROM,
WS2812, PS/2 host, JTAG (IDCODE read), SWD (DPIDR read), a CAN 2.0A node, a
USB low-speed HID mouse (enumeration and an interrupt IN endpoint) and a
Manchester loopback. `firmware/README.md` in the repository lists their
pins, sizes and rates.

Clock: 50 MHz at the typical corner. At the slow corner (1.08 V, 125 C) the
design closes at about 43 MHz.

## How to test

After reset all four threads are halted: nothing runs until the host loads
a program and starts a thread.

1. **The host.** `tools/loomhost` in the repository talks to Loom through
   the host port. `--ttboard PORT` drives it through the demo board's
   MicroPython and works on both demo boards: the helper it pastes reads
   the pin numbers from the board firmware's own map. On the RP2040 board
   the RP2040's SPI0 pins are exactly Loom's host pins; on the v3 board
   (RP2350B) MISO lands on an SPI1 pin, so the helper bit-bangs SPI mode 0
   on GP21, GP22, GP23 and GP40. A Raspberry Pi Pico wired to the host pins
   and running `tools/loomhost/micropython/pico_bridge.py` works too
   (`--pico PORT`); that folder's README has the wiring. SCK must be at
   most the Loom clock divided by 8. No hardware existed before the chip:
   both transports are tested against simulated serial ports, and the
   demo-board helper's own code also runs against the RTL's host port in
   simulation, for both boards.
2. **Assemble** a program to an image:
   `python -m tools.loomasm --strict firmware/uart_tx_fifo.loom -o uart.json`
3. **Load and run** it. The UART transmitter sends on OUT0 (`uo[0]`); at
   50 MHz, `TICK_INT 434` and `TICK_FRAC 8` make 115200 baud:

   ```
   python -m tools.loomhost --ttboard COM5 --image uart.json load "csr 0 TICK_INT 434" "csr 0 TICK_FRAC 8" "run 0" "push 0 'Hello'"
   ```

   and a USB-UART adapter on `uo[0]` at 115200 8N1 shows `Hello`.

Every other program runs the same way; its header gives the host commands,
the pins and the tick to set. The repository's documents describe the rest:
`docs/ARCHITECTURE.md`, `docs/SEMANTICS.md` (the cycle-exact definition of
every instruction), `docs/HOST_PROTOCOL.md` and
`docs/VERIFICATION_REPORT.md`.

## External hardware

The SPI host: the demo board's RP2040, or a Pico. For the protocol programs,
as wanted: a USB-UART adapter, an SPI device or master, I2C pull-ups, a
WS2812 strip, a PS/2 keyboard, a JTAG or SWD target, a CAN transceiver
(TXD on OUT0, RXD on IN0), for USB low speed the D+ and D- lines with a
1.5 kOhm pull-up on D-, and a logic analyser to watch it all.
