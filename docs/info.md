<!---

This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

## How it works

Loom is a programmable protocol emulator: a tiny I/O processor whose
instruction set is built for reading pins, writing pins and hitting timing.
Four hardware threads share one 4-stage pipeline in strict round robin, so
every instruction takes exactly one thread slot (4 clocks) and the timing of a
program can be read off its listing. Each thread has a local timebase with a
fractional prescaler and a deadline register: `WAITD k` advances the deadline
by k ticks and waits for it, which gives jitter-free bit timing regardless of
the code path, and every wait can time out against the same deadline. Each
thread also owns a bit engine (shift register, programmable CRC, NRZ/NRZI/
Manchester coding, USB/CAN bit stuffing) for the per-bit work that firmware
cannot do fast enough, and a pair of FIFOs to the host.

A SPI slave port (mode 0) on ui[6:4] and uo[7] loads programs into the
instruction memory, moves data through the FIFOs, and can halt, single-step
and read or write every register of every thread. The remaining 20 pins (5
inputs, 6 outputs, 8 bidirectional with per-pin open-drain mode) are the
emulated protocol pins, addressed by firmware as a uniform pin index space.

UART, SPI and I2C (master and slave) are firmware programs shipped in the
repository, along with WS2812, PS/2, JTAG and SWD. USB low-speed, CAN and
10 Mbit Manchester are stretch targets that the bit engines are designed for.

Current state of this file: M0. The silicon-facing description above is the
architecture; the RTL in `src/` at M0 is a hard-wired UART transmitter used to
prove the flow ("LOOM\r\n" at 115200 on OUT0 while IN0 is high). This
section is rewritten at M5.

## How to test

M0: hold ui[0] high and watch uo[0] with a serial adapter at 115200 8N1; you
should see "LOOM" repeating.

Final (M5): connect a SPI master to ui[4] (CS_n), ui[5] (SCK), ui[6] (MOSI)
and uo[7] (MISO); on the RP2040 demo board these are the SPI0 pins. Use the Python host library in `tools/loomhost` (works with
the Tiny Tapeout demo board's RP2040, a Raspberry Pi Pico bridge, or an FTDI
adapter) to load a program from `firmware/` and run it. The repository has a
test for every firmware program, and the same scripts run against the RTL
simulation, the FPGA prototype and the chip.

## External hardware

None required. Optional: a USB-UART adapter, a SPI flash or I2C EEPROM
breakout, a WS2812 LED strip, a PS/2 keyboard, and a logic analyser for the
protocol demos. A Raspberry Pi Pico or FTDI adapter can act as the SPI host
instead of the demo board's RP2040.
