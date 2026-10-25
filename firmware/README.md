# Loom firmware

Programs in Loom assembly (`tools/loomasm/README.md`). The M1 programs
(`uart_hello`, `uart_tx`) need nothing beyond the M1 core; the M2 programs
below need the FIFOs (`PUSH`, `POP`, `WAITB`) and nothing else: no bit engine
and no deadline-latched `SETP ... D`, so they run on the golden model built
with `features={"FIFO"}` and on the M2 RTL as soon as it has FIFOs.

Every M2 program is thread 0, takes its work from the host through `INQ[0]`
and gives results back through `OUTQ[0]`, and leaves its bit rate to the host:
the host writes `TICK_INT` (and `TICK_FRAC`) through the debug space before
`RUN`, and the program's `.tick` declares the fastest tick its deadline
schedule is proved for. The reset value `TICK_INT = 1` is always too fast, so
set the rate first.

The four M3 programs after `i2c_master` in the table are thread 0 as well
and need nothing the M2 hardware does not have. Two of them break that
convention: `ws2812` and `ps2_host` have a rate fixed by a datasheet rather
than chosen by the host, so they write their own `TICK_INT` with `.csr` and
**the host must not set it**; so does `usb_ls_device`, the last M3 row, which
also needs slices A and B (the encoders of `SEMANTICS.md` 6.9.1 and `LD`/`ST`
of 6.11). `ws2812` also uses the deadline-latched pin write of
`SEMANTICS.md` 6.10 (`SETP pin, v, D`), which the M2 programs deliberately do
not. `jtag_master` and `swd_master` keep the M2 convention: they own their
clock, so the host sets its rate.

The two rows before `usb_ls_device` use the M3 slices too.
`i2c_slave_eeprom` keeps its 256 bytes in the instruction memory through
`LD`/`ST` (slice B, `SEMANTICS.md` 6.11) and has no rate of its own: the
master owns SCL. `can_loopback` runs on two threads, a transmitter on
thread 0 and a receiver on thread 1, with the bit engine's CAN stuffing,
destuffing and CRC15 (slice A, 6.9.1); the host sets both threads' ticks,
as the M2 convention has it.

The last row, `manchester_loopback`, is the M4 program for L3-MANCH (D-029):
a Manchester transmitter on thread 0 and a receiver on thread 1 at the rate
the bit engine's manual mode reaches, with slice A's encoder (6.9.1) and
the M2 convention for the rate.

The rows after it are the D-035 programs, written after the hardware was
complete for protocols Loom was not designed for (see the section of that
name below). Each keeps the M2 convention for the rate, except where the
protocol fixes its slots in microseconds: the 1-Wire programs then set
their own tick, as `ws2812` and `ps2_host` do; and `lin_slave` sets its
own from the bit rate it measures on every sync byte.

| Program | Pins | Words | `.tick` (proved) | Worst slack | Rates tested |
|---|---|---|---|---|---|
| `uart_tx_fifo.loom` | TX = OUT0 | 27 | 24 clocks per bit | 4 clocks | 32 clocks/bit (1.5625 Mbaud at 50 MHz); 115200 baud (TICK 434 + 8/256) |
| `uart_rx.loom` | RX = IN0 | 47 | 8 clocks per tick, 8 ticks per bit | 20 clocks | 64 clocks/bit (781 kbaud); 115200 baud (TICK 54 + 64/256); +-3 % sender error |
| `spi_master.loom` | MOSI = OUT0, SCK = OUT1, CS_n = OUT2, MISO = IN0 | 84 | 28 clocks per half SCK period | 0 clocks | TICK 32: 781 kHz SCK, modes 0-3, MSB and LSB first |
| `spi_slave.loom` | MISO = OUT0, SCK = IN1, MOSI = IN2, CS_n = IN3 | 80 | none: the master's SCK times it, no deadline pairs | no deadline pairs; MISO 28 to 35 clocks after each sampling edge | 390 kHz SCK (128 clocks, TICK 32 for the silence timeout), modes 0 to 3 |
| `i2c_master.loom` | SCL = BIDIR0, SDA = BIDIR1 (open drain) | 121 | 6 clocks per tick, 16 ticks per SCL period | 0 clocks | 100 kHz standard mode (TICK 31 + 64/256) and 390.6 kHz fast mode (TICK 7, r5 = 1), UM10204 table 10 measured at both; most tests at TICK 8 (347 kHz) |
| `ws2812.loom` | DOUT = OUT0 | 52 | 1 clock per tick (set by the program) | 4 clocks (two `.bounded` POPs) | the WS2812B waveform at 50 MHz: T0H 20, T1H 40, period 64 clocks |
| `ps2_host.loom` | CLK = IN0, DATA = IN1 | 45 | 64 clocks per tick (set by the program), timeouts only | no deadline pairs | 10 kHz and 16.67 kHz device clock |
| `jtag_master.loom` | TCK = OUT0, TMS = OUT1, TDI = OUT2, TDO = IN0 | 43 | 32 clocks per half TCK period | 0 clocks | TICK 32: 781 kHz TCK |
| `swd_master.loom` | SWCLK = OUT0, SWDIO = BIDIR0 (push-pull, pull-up) | 91 | 32 clocks per half SWCLK period | 0 clocks | TICK 32: 781 kHz SWCLK |
| `i2c_slave_eeprom.loom` | SCL = BIDIR0, SDA = BIDIR1 (open drain) | 111, and 128 of data at 0x180 | none: the master owns SCL, no deadline pairs | no deadline pairs; SDA valid about 30 clocks after SCL falls (tVD 45 at 400 kHz) | 100 kHz and 400 kHz SCL (UM10204 Standard and Fast mode timing) |
| `can_loopback.loom` | TX = OUT0, RX = IN0 | 79 (thread 0) + 140 (thread 1) | thread 0: 100 clocks per bit; thread 1: 12 clocks per tick, 8 ticks per bit | 28 clocks (thread 0), 4 clocks (thread 1) | 500 kbit/s (TICK 100; thread 1 TICK 12 + 128/256) and 125 kbit/s (TICK 400; TICK 50) |
| `usb_ls_device.loom` | D+ = BIDIR0, D- = BIDIR1 (push-pull while sending; 1.5 kOhm pull-up on D-) | 511 of 512 (all four quarters, slices A and B) | 33 clocks per tick, one tick per bit (set by the program: 33 + 85/256) | 1 clock | USB low speed, 1.5 Mbit/s at 50 MHz; host at +-0.25 %; 2-bit host gaps |
| `manchester_loopback.loom` | TX = OUT0, RX = IN0 | 60 (thread 0) + 113 (thread 1) | 12 clocks per tick, one tick per half-bit, both threads | 0 clocks (both threads) | 2.083 Mbit/s (TICK 12) and 1 Mbit/s (TICK 25), wire loopback; a sender at +-0.1 % (TICK 12) and +-0.2 % (TICK 25) |
| `ir_nec_tx.loom` | LED = OUT0 (high = on, through a transistor) | 71 | 16 clocks per tick, a third of a carrier period (host-set) | 8 clocks (two `.bounded` POPs, in an interval with 28) | 38 kHz carrier at 50 MHz (TICK 438 + 153/256: 37,999.9 Hz) and at a 1.824 MHz clock (TICK 16); duty 1/3 |
| `ir_nec_rx.loom` | RX = IN0 (a receiver module's output, low = carrier) | 112 | 64 clocks per tick, 1/16 of the NEC unit (host-set); timeouts only | no deadline pairs; pulses measured to about 30 clocks | NEC at 50 MHz (TICK 1757 + 208/256) and at 1.824 MHz (TICK 64 + 32/256); every mark and space +-25 % + 140 us; glitches under 70 us ignored |
| `onewire_master.loom` | OW = BIDIR0 (open drain, board pull-up) | 157: 124 at 0x000, 33 at 0x100 (thread 2's quarter) | 50 clocks per tick, 1 us (set by the program); also proved at 10 | 288 clocks (12 at a tick of 10) | 1-Wire standard speed at 50 MHz, AN126's slots to the clock (read sample E = 8, see its section); searches and scratchpads also on a build with a 10-clock tick |
| `onewire_device.loom` | OW = BIDIR1 (open drain) | 157: 116 at 0x080, 8 of data at 0x0F8, 33 at 0x140 (thread 2's quarter) | 50 clocks per tick, 1 us (set by the program); also proved at 10 | 1446 clocks (286 at 10) | standard speed against an AN126 master and one at the datasheet's limits (write-1 low 14 us, read low 1 us sampled at 15 us); MATCH, search, scratchpad also on the 10-clock build |
| `lin_master.loom` | TX = OUT0 (to a LIN transceiver's TXD), RX = IN0 (its RXD) | 128, and an 11-word frame buffer at 0x1F5 | 10 clocks per tick, 16 ticks per bit (the deadline pairs alone would allow 7) | 28 clocks | 19200, 10417 and 9600 bit/s (TICK 162 + 195/256, 300, 325 + 133/256); 195 kbit/s (TICK 16) and 100 kbit/s (TICK 31 + 64/256) as test rates; a slave 2 % off |
| `lin_slave.loom` | TX = OUT0, RX = IN0 (thread 0); OUT1, IN1 beside the master | 218 (runs 90 words into the next quarter), and an 80-word table at 0x180 | 32 clocks (set by the program; the half-bit tick it measures is 1250 clocks or more at LIN rates) | 4 clocks | a master at 19200, 10417 and 9600 bit/s, each 14 % slow, exact and 14 % fast; 100 kbit/s as the test rate |

Every program assembles with `--strict` and no diagnostic at all: each
deadline pair is proved, apart from the five intervals declared `.bounded`
below, and none is unbounded (the M2 five in
`tools/tests/test_fw_build.py`, the M3 and M4 programs in their own test
modules). Each fits in thread 0's quarter of the 512-word memory of D-020,
except three of the M3 programs: `i2c_slave_eeprom`'s data fills thread 3's quarter;
`can_loopback`'s thread 1 (140 words from 0x080) runs 12 words into thread
2's quarter, which no thread of that program runs from; and
`usb_ls_device` needs the whole memory for itself (a thread-0-only
program: the other three threads must not be started with it loaded).
`manchester_loopback`'s two threads each stay in their own quarter (60
words from 0x000, 113 from 0x080), and so do the two NEC infrared
programs run together (71 words from 0x000, 112 from 0x080).

`ws2812` gets there with two `.bounded` declarations
(`tools/loomasm/README.md` sections 4 and 7). Its byte fetch needs a `POP`
between two deadlines; `POP` has no static bound for the checker, and the
usual cure, a re-anchoring `SETD`, would destroy the very deadline the next
edge is latched on. Each `POP` is guarded by a `WAITB INQ_NE, T` one slot
earlier, so it cannot stall, and the declaration states that argument in the
source. **A declaration is believed, not verified**, so those two intervals
are only as sound as the argument in the program header: 10 slots against a
44-clock budget and 9 against 40, which is where the 4-clock worst slack in
the table comes from. `docs/spec-questions/firmware-m3.md` item 1 has the
history.

`uart_tx_fifo` has the third declaration since its T-1 fix: the `POP` of a
word queued behind the stop bit, guarded by a `WAITB INQ_NE, T` two slots
earlier (a `BT` between them leaves for the idle path when nothing is
queued), on an interval of 5 slots against the 24-clock budget of its
proved `.tick`. `ir_nec_tx` has the fourth and fifth, the ws2812 idiom
again: the two `POP`s of the next frame or repeat word, each guarded by a
`WAITB INQ_NE, T` right after a deadline, on an interval of 25 slots
against 8 ticks of at least 16 clocks.

```
python -m tools.loomasm firmware/uart_rx.loom --strict --listing
python -m tools.loomhost --model firmware/uart_tx_fifo.loom --uart-rx OUT0:32 \
    load "csr 0 TICK_INT 32" "run 0" "push 0 'Hello'" "idle 3000"
```

```python
from tools.loomhost import Loom, PicoTransport
loom = Loom(PicoTransport("COM7"))
loom.load("firmware/uart_rx.loom")                  # assembled on the fly
loom.write_csr(0, "TICK_INT", 54); loom.write_csr(0, "TICK_FRAC", 64)
loom.run(0)
words = loom.pop(0, 4)                              # four received frames
```

## Four at once

`tools/tests/test_fw_four_at_once.py` runs four of these programs side by
side, one per thread, each in its own quarter of the memory and on its own
pins: `uart_tx_fifo` on thread 0 (TX = OUT0), `spi_master` on thread 1 (MOSI
= OUT1, SCK = OUT2, CS_n = OUT3, MISO = IN0, its `OUTGRP` moved with its
pins), `i2c_master` on thread 2 (BIDIR0, BIDIR1) and `ws2812` on thread 3
(DOUT = OUT4). Nothing else in the sources changes: the test rewrites the
`.thread` and `.pins` lines and assembles each for the 512-word memory. With
the same host transactions before RUN, every pin edge of each protocol is on
the same clock with all four running as when it runs alone, which is the
claim the barrel pipeline makes (SEMANTICS 2, ISO-1).

## uart_tx_fifo.loom: UART transmitter fed from INQ

- **Host command** (push to thread 0): one word per byte; bits 7:0 are sent,
  LSB first, 8N1; bits 15:8 are ignored.
- **Results**: none.
- **Rate**: one tick is one bit, `TICK_INT` = clocks per bit.
- Every edge is the first instruction after the `WAITD` that times it, so all
  edges sit one slot after their deadlines: exact when the bit is a multiple
  of 4 clocks, within one slot otherwise (tested at 115200 baud). After each
  stop bit a `WAITB INQ_NE, T` (its deadline has passed, so it returns at
  once) tells a queued word from none: a queued word goes out back to back,
  timed from the stop bit, with one stop bit; otherwise `POP` blocks and
  `SETD 1` puts the start bit on the second tick boundary after the word
  arrives, a whole bit whatever point of a tick it arrived at. The first
  version used `SETD 0` after the `POP` in both cases and started 10 in 64
  words after an idle gap a slot late and 4 clocks short (tools finding T-1);
  `test_fw_uart.py` sweeps the arrival over every point of the tick.

## uart_rx.loom: UART receiver into OUTQ

- **Host command**: none.
- **Result** (pop thread 0): one word per frame. Bits 7:0 the byte; bit 8 a
  framing error (low stop bit; the byte is still delivered); bit 9 an overrun
  (OUTQ was full, so at least one earlier frame was dropped).
- **Rate**: 8 ticks per bit, `TICK_INT` = clocks per bit / 8. The program
  reads `TICK_INT` once at start to pick its first-sample offset.
- `WAITP` finds the start bit and `SETD 0` re-anchors the schedule on it;
  each bit is sampled near its middle by the `JP` right after its `WAITD`
  (measured: exactly mid-bit at 64 clocks per bit, 17 to 22 clocks early of
  434 at 115200 baud). A start bit high again at its middle is ignored as a
  glitch; after a framing error the receiver waits for the line to go high,
  so a break is one error. Back-to-back frames: the start-bit wait is reached
  16 clocks after the stop-bit sample. Tolerance: +-3 % back to back at
  115200 baud; at 64 clocks per bit, -2 % .. +4 % back to back, +-3 % with an
  idle bit between frames.

## UART rates at 50 MHz

`uart_tx_fifo` takes one tick per bit and `uart_rx` eight, so the tick is
50 MHz / baud and 50 MHz / (8 x baud) clocks. `tools.loomhost.rates.tick`
gives the nearest setting (TICK_FRAC resolves 1/256 of a clock); the error
is the mean bit-rate error that rounding leaves. The transmitter's edges
are within one slot (4 clocks) of the ideal grid at every rate, and both
programs move bytes correctly at every rate from 9600 baud up
(`test_uart_tx_at_the_common_rates_edges_within_one_slot` and
`test_uart_rx_at_the_common_rates`, on the model). The limits are the
programs' proved ticks: 24 clocks for the transmitter (2.08 Mbaud) and 8
for the receiver (781 kbaud); TICK_INT is 16 bits, which puts the slowest
transmitter rate at 763 baud.

| Baud | `uart_tx_fifo` TICK_INT + TICK_FRAC/256 | `uart_rx` TICK_INT + TICK_FRAC/256 |
|---|---|---|
| 1200 | 41666 + 171 | 5208 + 85 |
| 2400 | 20833 + 85 | 2604 + 43 |
| 4800 | 10416 + 171 | 1302 + 21 |
| 9600 | 5208 + 85 | 651 + 11 |
| 19200 | 2604 + 43 | 325 + 133 |
| 31250 (MIDI) | 1600 + 0 | 200 + 0 |
| 38400 | 1302 + 21 | 162 + 195 |
| 57600 | 868 + 14 | 108 + 130 |
| 115200 | 434 + 7 | 54 + 65 |
| 230400 | 217 + 4 | 27 + 32 |
| 250000 (DMX512) | 200 + 0 | 25 + 0 |
| 460800 | 108 + 130 | 13 + 144 |
| 921600 | 54 + 65 | too fast |
| 1000000 | 50 + 0 | too fast |

Every error is under 0.01 %. The examples elsewhere use 434 + 8/256 for
115200 baud, 1/256 of a clock slower: both are far inside a UART's
tolerance.

## Pmod builds

A program keeps its pins in its `.pins` line, and the assembler's
`--pin NAME=PIN` moves them without editing the source
(`tools/loomasm/README.md`, section 2). Tiny Tapeout's recommended pinouts
(<https://tinytapeout.com/specs/pinouts/>) put the common peripherals on
fixed pins; these builds put the programs there, and
`tools/tests/test_fw_pmod.py` runs each one on both backends (its `PMODS` is
this table). Every command starts
`python -m tools.loomasm --strict firmware/<program>.loom`:

| Build | Program and pins | Where |
|---|---|---|
| UART on the demo board's own USB-UART link | `uart_tx_fifo --pin TX=OUT4`, `uart_rx --pin RX=IN3` | the design's TX on `uo_out[4]`, RX on `ui_in[3]` |
| UART on the UART Pmod's top row | `uart_tx_fifo --pin TX=BIDIR1`, `uart_rx --pin RX=BIDIR2` | TXD `uio[1]`, RXD `uio[2]` |
| I2C on the I2C Pmod's top row | `i2c_master --pin SCL=BIDIR2 --pin SDA=BIDIR3` | SCL `uio[2]`, SDA `uio[3]` |
| SPI on the SPI Pmod's top row, which is also how the QSPI Pmod wires its flash | `spi_master --pin CS_N=BIDIR0 --pin MOSI=BIDIR1 --pin MISO=BIDIR2 --pin SCK=BIDIR3` | CS `uio[0]`, MOSI `uio[1]`, MISO `uio[2]`, SCK `uio[3]`; the test reads a flash's JEDEC ID |

`uart_tx_fifo` as shipped (TX on `uo_out[0]`) already matches the pinouts
page's other UART link pair; its receiver is `uart_rx --pin RX=IN1`. TXD and
RXD are named as the pinouts page names them; swap the two for a Pmod named
from its own side. A program re-pins correctly only when everything it
derives from its pins is written with `pin()`, so that moves too: the
`OD_MASK` bits of `i2c_master`, the `OUT` group and bit positions of
`spi_master` (its three outputs must lie within six consecutive pins), and
the `OEP` that drives an output that lands on a BIDIR pin (`uart_tx_fifo`,
`spi_master`). Those three were changed for that on 2026-10-11 (D-035);
`uart_rx` needed nothing. The other programs have not been checked for
constants that depend on their pins, so check before moving them.

## spi_master.loom: full-duplex SPI master

- **Host command** (push to thread 0): one word per byte.
  - bits 7:0: the byte to send on MOSI;
  - bit 13 CONFIG: send nothing; bits 1:0 set the SPI mode (CPOL, CPHA) and
    bit 2 selects LSB first (default after reset: mode 0, MSB first). Only
    between transactions;
  - bit 14 SKIP: do not push the byte received during this one;
  - bit 15 LAST: raise CS_n after this byte.
  The first byte while CS_n is high starts a transaction: CS_n falls with SCK
  at its idle level, at least half an SCK period before the first edge.
- **Result** (pop thread 0): one word per byte without SKIP, bits 7:0 the
  byte received on MISO.
- **Rate**: one tick is half an SCK period, SCK = clk / (2 `TICK_INT`).
- Example, JEDEC ID of a flash: push `0x409F, 0x0000, 0x0000, 0x8000`, pop
  three words. READ: `0x4003`, three `0x40xx` address bytes, then
  `0x0000` per data byte and `0x8000` for the last.
- All four modes are one loop: an `OUT` writes CS_n, SCK and MOSI together,
  each bit is a shift step (SCK to CPOL xor CPHA) and a sample step (SCK
  toggled, MISO sampled). Inside a byte every half period is exact; between
  bytes SCK waits while the master fetches the next command (legal: the
  master owns the clock).

## spi_slave.loom: SPI slave answering a master

- **Host command** (push to thread 0): one word per byte to send, bits 7:0;
  the other bits are ignored. The slave pops the next response byte as soon
  as it has finished one, so the head of `INQ[0]` is what the master's next
  byte will read. With `INQ` empty it sends `0xFF`, and it replaces that
  filler with a real byte as soon as one is queued and nothing of it has
  gone out yet.
- **Result** (pop thread 0): one word per byte the master clocked in, bits
  7:0 as sampled on MOSI. A byte that arrives while `OUTQ` is full is
  dropped (the host is not keeping up); nothing else is lost.
- **Rate**: the master's SCK times everything; the tick only times the
  silence (16 ticks) after which CS_n is looked at, and the tests set it to a
  quarter of the SCK period, TICK 32. What limits the rate is not the bit
  loop but the work between two bytes (push the byte received, pop the next
  one to send, put its first bit on MISO): the slave waits for the next edge
  27 slots after the X cycle that saw a byte's last one, so bytes back to back
  need an SCK period of about 110 clocks or more (tested at 128). A master
  that pauses between bytes can clock the bits themselves faster.
- **Modes**: modes 0 and 3 sample MOSI and MISO on the rising edge, modes 1
  and 2 on the falling edge, and the edge is all the slave needs to know (it
  never looks at the idle level, and its first bit is on MISO before the
  first edge of either kind). The host picks it before `RUN` by writing r5:
  0, the reset value, for the rising edge, 1 for the falling edge
  (`reg 0 r5 1` in `tools.loomhost`). A `WAITE`'s edge is part of the
  instruction, so the program holds two copies of its loop and picks one at
  start: modes 1 and 2 have the timing of modes 0 and 3 with the edges
  swapped. Modes 1 and 2 were added on 2026-10-11 (D-035).
- **Queue first, then select**: a word that lands in `INQ` while the slave
  is deselected has its first bit on MISO, and the slave waiting for the
  first sampling edge, at most 21 slots later. A word that lands later than
  that before the master's first sampling edge has its first bit out in
  time but that edge missed, and the whole transaction comes out one bit
  late (BUGS 12). The tests start the master 21 slots minus half an SCK
  period after the push, and one test checks every phase of the poll loop
  at that lead.
- Each bit is one `WAITE SCK, RISE` (the master samples there), then MOSI is
  sampled and the next MISO bit is written six slots after the X cycle that
  saw the edge (seven after a 1 bit): 28 to 35 clocks after the edge, long
  before the next. The first version wrote it at a deadline one tick after
  the edge (`SETD 0` on the edge, then `WAITD 1`); after an edge the `SETD`
  can fall anywhere in a tick, so that deadline cannot be proved (tools
  finding T-1), and the slave now keeps no schedule of its own.
  While deselected the slave watches `INQ` and pre-loads, so the master may
  raise SCK two slots after CS_n falls; MISO holds its last bit between
  transactions (a Tiny Tapeout pad cannot be let go). SCK silence for 16
  ticks inside a byte makes the slave look at CS_n: a master that still
  selects us is only pausing and the wait is resumed, one that has
  deselected us cut the transaction short, so the partial byte is dropped
  and the slave resynchronises. That timeout is also how the end of a
  transaction is noticed, so a host that queues bytes for the next one
  should allow those 16 ticks first.

## i2c_master.loom: I2C master with clock stretching

- **Host command** (push to thread 0): one word per operation.
  - bits 7:0: the byte to write (ignored for READ and NOBYTE);
  - bit 8 START: a START first (a repeated START if the bus is already ours);
  - bit 9 STOP: a STOP afterwards;
  - bit 10 READ: read a byte instead of writing one;
  - bit 11 NACK: with READ, answer NACK (the last byte of a read);
  - bit 12 NOBYTE: only the START and/or STOP asked for.
  Example, random read of one byte from a 24C02 at 0x50: `0x01A0` (START,
  address write), `0x0010` (word address), `0x01A1` (repeated START, address
  read), `0x0E00` (read, NACK, STOP).
- **Result** (pop thread 0): one word per byte transferred. Bits 7:0 the byte
  as seen on SDA (the data read, or the echo of the byte written); bit 15 NACK
  (the ACK slot was high); bit 14 TIMEOUT (SCL stayed low more than 255 ticks
  after release: the operation was abandoned and both lines released).
- **Rate and mode**: 16 ticks per SCL period. In standard mode (r5 = 0, the
  reset value) a bit is 8 ticks low and 8 high; in fast mode (r5 = 1,
  `reg 0 r5 1`) 10 low and 6 high, because fast mode's minimum low time
  (1.3 us) is over half its period. At 50 MHz, 100 kHz standard mode is
  TICK 31 + 64/256, and fast mode's fastest rate at or under 400 kHz is
  TICK 7: 390.6 kHz, since every SCL edge falls in one of the thread's slots
  and the period is a whole number of slots (128 clocks; 124 would be
  403 kHz).
- **UM10204 table 10**: at both settings every bound a master has to meet
  holds, measured on the pins by `tools.protomodels.i2c.I2cTiming`, written
  from the table: f_SCL, t_LOW, t_HIGH, t_SU;STA, t_HD;STA, t_SU;STO, t_BUF,
  t_SU;DAT and t_HD;DAT (t_VD;DAT is reported but, by the table's note 4,
  binds a master only where it does not stretch its own low phase, which
  t_SU;DAT covers). START and STOP use the same tick counts in both modes:
  10 ticks of SCL low before SCL is let go, a set-up of 9 ticks from the
  re-anchor, a hold of 8, and 10 ticks of bus free time after a STOP.
  (Before 2026-10-11 the set-up and hold were a quarter period, 2.5 us at
  100 kHz where the table asks 4.0 to 4.7 us: BUGS 13.)
- SCL and SDA are open drain through `OD_MASK` (set with a read-modify-write).
  After releasing SCL the program waits for it with a timed `WAITP SCL, 1, T`
  (clock stretching, timeout 255 ticks), then re-anchors with `SETD 0`, so
  whatever a slave did to the low phase the high phase that follows is a full
  one: 8 ticks (6 in fast mode) less up to one tick, plus a few slots of pin
  latency. Measured at TICK 8 in standard mode: low 68 clocks inside a byte
  (8 ticks and one slot), high 76.

## ws2812.loom: WS2812B / NeoPixel strip driver

- **Host command** (push to thread 0): one word per byte; bits 7:0 are sent
  MSB first, bits 15:8 are ignored. The bytes go out in the order pushed,
  which for a WS2812B is G, R, B per LED, first LED of the chain first. A
  word per LED was not used: 24 bits do not fit in one, and a byte stream
  lets a frame be any length.
- **Results**: none.
- **Rate**: fixed by the datasheet. The program writes `TICK_INT = 1` and
  `TICK_FRAC = 0` itself, so one tick is one clock and every `WAITD` counts
  clocks at 50 MHz: T0H 20, T1H 40, T0L 44, T1L 24, period 64 (1.28 us).
  The high times are exactly nominal; the low times and the period are
  0.03 us long, which buys the four extra slots the byte fetch needs and
  stays well inside the +-0.15 us (pulse) and +-0.6 us (period) windows.
- **Frames**: a frame ends when `INQ` is empty at a byte boundary. The line
  then stays low for 2750 clocks (55 us, the datasheet asks for more than
  50) before the next frame's first rise, so the strip always latches
  between frames. Bytes pushed while a frame is running extend it, which is
  exactly the wire format: one reset, a byte stream, one reset. The host
  must keep up - a stall longer than one byte time (512 clocks) splits the
  frame and the strip latches the first half.
- **Every edge is a deadline-latched write** (`SETP DOUT, v, D` then the
  `WAITD` that lands it, `SEMANTICS.md` 6.10), so each pulse is exact to the
  clock rather than to the thread's 4-clock slot grid. The byte fetch runs
  in the 44-clock low phase of a 0 bit and in the 40-clock high phase of a
  1 bit, whose 24-clock low phase is two slots too short for it.

## ps2_host.loom: PS/2 host receiver

- **Host command**: none; this program only listens, so it never drives
  either line and needs no open-drain configuration.
- **Result** (pop thread 0): one word per frame. Bits 7:0 the byte; bit 8 a
  parity error (the nine bits were not odd); bit 9 a framing error (the stop
  bit was low, the first edge of a frame had DATA high, or the frame stopped
  part-way). A frame with a bad stop bit is still delivered, byte and all:
  it is eleven bits long like any other, so the receiver stays in step. The
  two ways of losing sync give bit 9 with a zero byte, once per bad frame.
- **Rate**: set by the device, 10 to 16.7 kHz (60 to 100 us per bit). The
  program writes `TICK_INT = 64` itself; that tick is only the unit of its
  timeouts, 128 ticks (164 us) for a bit that never comes and 200 ticks
  (256 us) of quiet to end a resync.
- DATA is sampled by the `JP` right after each `WAITE CLK, FALL`, one slot
  after the edge, in the middle of a half period of setup. There is no
  `WAITD` in the program: the device owns the clock and the host follows it,
  so the checker has no deadline pair to prove. After a first edge that was
  not a start bit the receiver drains the rest of the frame (`resync`); a
  frame cut short needs no drain, because the timeout that ended it already
  proves the line is quiet.

## jtag_master.loom: JTAG master, TAP reset and IDCODE

- **Host command** (push to thread 0): one word per operation, bits 15:0
  reserved and zero. Each word asks for the same thing: five TCK clocks
  with TMS high (Test-Logic-Reset, which also selects the IDCODE
  instruction), the four-clock path to Shift-DR, 32 shifts, and the
  three-clock path back to Run-Test/Idle.
- **Result** (pop thread 0): two words per operation, IDCODE bits 15:0 then
  bits 31:16. A part with no IDCODE register selects BYPASS at reset and
  gives 32 zeros; a real IDCODE always has bit 0 set.
- **Rate**: one tick is half a TCK period, TCK = clk / (2 `TICK_INT`).
- TMS and TDI change while TCK is low, the TAP samples them at the rising
  edge and moves TDO at the falling one, so TDO is read in the low phase
  before the edge that shifts it. Leaving Shift-DR clocks one more bit
  (IEEE 1149.1 figure 6-5), which is harmless for a read-only register.
  Between operations TCK simply stops: the master owns the clock.
- Every half period is one tick; the only longer ones are the two low phases
  in which a result word is pushed. The first version ran a slot over on the
  way into the shift sequence and on every bit that read a 1, giving 36/28
  clock low/high phases at TICK 32: a `SETD 0` after each falling edge,
  meant to cut a checker loop that does not exist, hid the 9-slot path from
  the checker (tools finding T-1). `test_fw_jtag.py` now measures every edge.

## swd_master.loom: SWD master, connect and DPIDR

- **Connect**: once, before the first command, as a debug probe does it
  (ADIv5): at least 50 SWCLK cycles with SWDIO high, the 16-bit
  JTAG-to-SWD select sequence `0xE79E` least significant bit first, a
  second line reset, two idle cycles.
- **Host command** (push to thread 0): one word per operation, bits 15:0
  reserved and zero. Each word is one DP read of register 0x00, DPIDR: the
  packet request is `0xA5` LSB first (start 1, APnDP 0, RnW 1, A[2:3] 00,
  parity 1, stop 0, park 1).
- **Result** (pop thread 0): three words. DPIDR bits 15:0, DPIDR bits
  31:16, then a status word: bits 2:0 the three ACK bits as received
  (1 OK, 2 WAIT, 4 FAULT, 0 or 7 no target), bit 3 a parity error over the
  32 data bits and their parity bit. On anything but OK there is no data
  phase and the two data words are zero. The status is last because the
  parity is only known at the end of the packet.
- **Rate**: one tick is half an SWCLK period, SWCLK = clk / (2 `TICK_INT`).
- SWDIO is BIDIR0, push-pull (`OD_MASK` stays 0) with a board pull-up for
  the two turnaround cycles, when neither end drives; `OEP` releases it and
  takes it back. The bits the host sends go out with `OUT` and
  `OUTGRP = {cnt 1, base SWDIO}`, which drives SWDIO from bit 0 of the
  shifter (SWD is LSB first) in one slot instead of a branch and a `SETP`.
  Only `clk` is a subroutine: a second level of `CALL` fits the two-entry
  return stack but not the deadline checker, which lets every `RET` return
  to every call site.
- Every high phase of SWCLK is one tick; the program pauses the clock (a
  longer low phase) with `SETD 1` where the path between two clocks is
  longer than a tick. The first version claimed a 28-clock tick and paused
  with `SETD 0`: its data loop needs 8 slots (32 clocks) from a falling edge
  to the next rising one, which a `SETD 0` before `clk`'s `RET` hid from the
  checker, and a pause that fell late in a tick gave high phases of 20 and 28
  clocks (tools finding T-1). `test_fw_swd.py` now measures every edge.

## i2c_slave_eeprom.loom: 24C02-style I2C EEPROM

- **Bus**: device address 0x50, SCL = BIDIR0 and SDA = BIDIR1, open drain;
  the board supplies the pull-ups. The device only ever pulls SDA low or
  lets it go, and never touches SCL.
- **Data**: 256 bytes in the 128 words 0x180..0x1FF (thread 3's quarter,
  which no thread runs from), reached with `LD` and `ST` (`SEMANTICS.md`
  6.11). Byte `A` is word `0x180 + A/2`, an even byte in 15:8 and an odd
  byte in 7:0. The image fills the window with 0xFFFF (an erased part); the
  host loads other contents, or reads them back, with its IMEM commands
  while the thread is halted.
- **Transactions**: the 24C02 set. Byte write and page write (8-byte pages,
  wrapping inside the page), current-address read, random read (address
  write, repeated START, read) and sequential read wrapping from 0xFF to
  0x00, with one address counter for reads and writes. Another device
  address is not ACKed and the device waits for the next START; when the
  master NACKs a byte it read, the device lets SDA go.
- **Host command and results**: none; the program never touches the FIFOs.
- **Rate**: the master's. No tick and no deadline: every SCL fall is seen
  within 8 clocks (a two-slot poll) and answered within three slots, so SDA
  is valid about 30 clocks after SCL falls, against a tVD of 45 clocks at
  400 kHz (UM10204 table 10). Storing a received byte takes 19 slots and
  fetching the next byte to send 8, both inside one SCL period at 400 kHz,
  so the device never stretches the clock.
- **Differences from a 24C02**: each byte is written to the array in its own
  ACK clock rather than committed at the STOP, so a page write cut short
  keeps the bytes already ACKed, and there is no write cycle time (tWR): the
  next transaction is answered at once.

## can_loopback.loom: CAN 2.0A node

- **Pins**: TX = OUT0 to a transceiver's TXD (1 = recessive), RX = IN0 from
  its RXD. Thread 0 transmits, thread 1 receives everything on the bus, the
  chip's own frames included (the loopback), and ACKs good frames of other
  nodes. `SFLAGS[0]` is set by thread 0 while it sends, so thread 1 does not
  ACK the chip's own frame.
- **Host command** (push to thread 0): `{ID[10:0], RTR, DLC[3:0]}`, then the
  data bytes two to a word, first byte in 15:8: `min(DLC, 8)` bytes for a
  data frame, none for a remote frame.
- **Results** (pop from thread 1): a status word, then the `n` words it
  announces in bits 10:8: the header in the command's format and the data
  words. Status bits: 0 CRC error, 1 stuff error, 2 form error (CRC
  delimiter, ACK delimiter, EOF bits 1..6), 3 no ACK, 4 own frame, 5
  extended frame (not decoded). A stuff error or an extended frame reports
  `n = 0` and thread 1 waits for 11 recessive bits before the next frame.
- **Rate**: set by the host for both threads. Thread 0 ticks once a bit and
  thread 1 eight times a bit: 500 kbit/s is TICK 100 and TICK 12 + 128/256,
  125 kbit/s TICK 400 and TICK 50.
- **The engine does the bit work**: `BE_CFG` = MSB first, `STUFF = 2`,
  `CRC_EN`, CRC15 left-aligned (`CRC_POLY = 0x4599 << 1`, init 0). The CRC
  and its delimiter go out as 16 data bits so that a stuff bit due after the
  last CRC bit is inserted by the engine; thread 1 reads them the same way
  and finds the CRC register equal to the polynomial exactly when the CRC
  was right and the delimiter recessive.
- **Receive timing**: hard synchronisation on each SOF (thread 1 rewrites its
  `TICK_INT`, which restarts the tick generator on the edge) and a sample 5
  ticks into each bit plus 10 to 16 clocks: 72-79 % of the bit at 500
  kbit/s, 66 % at 125 kbit/s. There is no resynchronisation, so a sender
  must be within about 0.2 % of the bit rate.
- **The ACK** is `SETP TX, 0, D` staged before the `WAITD` that ends on the
  ACK slot's first tick, and released the same way one bit later, so both
  edges land on the tick; the decision (the CRC register against the
  expected value) takes 8 slots of the 36-clock budget, the worst slack of
  the program.
- **SOF** goes out on the second tick after the frame is set up (`SETD 1`):
  the checker cannot tell that the idle check ended by timeout, on a tick,
  so a `SETD 0` there could not be proved on time (tools finding T-1).
- **Not done**: arbitration (thread 0 waits for 10 idle bits before SOF and
  does not read its bits back), error and overload frames, retransmission,
  extended frames, resynchronisation.

## usb_ls_device.loom: USB low-speed device, a HID boot mouse

- **What it does**: a low-speed (1.5 Mbit/s) USB device in manual mode on
  slice A and B (`SEMANTICS.md` 6.9.1, 6.11). The bit engine does NRZI, bit
  stuffing, CRC16 and D+/D- together (`DIFF`); the firmware does SYNC, PID,
  EOP, handshakes, data toggles and the control transfers. Enumeration:
  GET_DESCRIPTOR of the device, configuration (any wLength, odd counts
  included) and HID report descriptors, SET_ADDRESS (the new address is
  taken after the status stage, 9.4.6), SET_CONFIGURATION and every other
  request without a data stage (a ZLP status stage); an unknown
  device-to-host request is STALLed. The device is a boot mouse (VID 0x1209,
  PID 0x0001: the pid.codes test pair, fine for a bench) with an interrupt
  IN endpoint 1 of 3 bytes every 10 ms.
- **Host command** (push to thread 0): one HID report as two words, bits 7:0
  buttons and 15:8 X, then bits 7:0 Y (bits 15:8 ignored). A report goes out
  on the next IN to endpoint 1 once both words are in, DATA0 and DATA1
  alternating (reset by SET_CONFIGURATION); an IN before that gets a NAK.
  The words are taken from INQ only while the bus is idle (every 250 bit
  times, and after each transaction), never in the response path.
- **Results**: none. The whole memory holds the program (511 words: code,
  state in words 1..16, descriptors packed two bytes per word at the end),
  so it runs alone, in thread 0.
- **Rate**: fixed by the specification: the program sets `TICK_INT = 33`,
  `TICK_FRAC = 85` (33.33 clocks, one bit at 50 MHz), **the host must not**.
  On each packet's first J-to-K edge the program rewrites `TICK_INT`, which
  clears the tick accumulator (`SEMANTICS.md` 4), so the tick grid starts at
  that edge and every sample lands within a few clocks of mid-bit.
- **Receive**: `WAITE D+, RISE` finds a SYNC (D+ only rises at J-to-K), the
  SYNC is followed with `SHI` until its closing KK (so the NRZI level and the
  stuffing run include it, 7.1.9), then words of 16 bits with `SHI; WAITD 1;
  BNZ` and the EOP looked for where the length says it ends. Tokens are
  matched whole: the PID byte and the 16-bit field of each of our endpoints,
  CRC5 included, computed with the engine when the address changes, so no
  CRC5 is checked while the host waits.
- **Transmit**: `SHO; WAITD 1; BNZ`; SYNC and PID are one SR load. The
  firmware cannot read the stuffer's `PEND`, and USB wants a stuff bit even
  right before the EOP: after the last bit the program points
  `BE_PINS.out` at pin 22 (no pad) and sends one probe bit, which leaves
  `CNT` at 1 only if it was a stuff bit, and then puts that bit on the pads
  by hand. EOP: `OUT` of both pins, two bits of SE0, one of J, then the
  pins are released.
- **Timing**, measured on the model over a whole enumeration (26 device
  packets) against USB 2.0: response 4.56 to 4.74 bit times after the
  host's EOP (2 to 6.5 allowed), SE0 of the EOP 1.28 to 1.36 us (1.25 to
  1.50), bit time 33.25 to 33.43 clocks (1.5 % is 32.83 to 33.83), source
  jitter at most 53 ns (95 ns next transition, 150 ns paired). The host may
  send at 1.5 Mbit/s +-0.25 % and use its minimum 2-bit gaps. After
  SET_ADDRESS the device needs about 11 bit times before it answers the new
  address (9.2.6.3 allows 2 ms).
- **Not implemented**: suspend and resume (the device never looks at a 3 ms
  idle bus), remote wakeup, isochronous and bulk endpoints, OUT data stages
  (SET_REPORT and the like: the data packet gets no handshake), string
  descriptors (none are named), and GET_STATUS / GET_CONFIGURATION answers
  (they are STALLed as unknown reads). Keep-alive EOPs and bus reset are
  ignored harmlessly; after a bus reset the device keeps its address, so a
  host that resets it must reload the program.

## manchester_loopback.loom: Manchester transmitter and receiver

- **Pins**: TX = OUT0 (push-pull, idle low), RX = IN0; wire OUT0 to IN0 for
  the loopback, or let any sender of the same frames drive IN0. Thread 0
  sends, thread 1 receives every frame on RX. Start threads 0 and 1.
- **Frame**: IEEE 802.3 Manchester (0 = high then low, 1 = low then high),
  most significant bit first, done by the engine (`ENC = 2`, `DIR` MSB
  first). The sync word 0xAAAB (fourteen preamble bits 1010..10, then the
  start marker 11), 0 to 4 data words, then the line low: the bit after the
  last word has no mid-bit transition, and that code violation ends the
  frame. Frames are 16 idle bit times apart.
- **Host command** (push to thread 0): a word with n in bits 2:0 (5..7 are
  taken as 4), then the n data words. Thread 0 pops all of them before the
  frame starts, so a slow host delays a frame but never breaks one.
- **Result** (pop thread 1): a status word, then the d data words it
  announces. Status bits 2:0 d; bit 3 violation (a bit with no mid-bit
  transition inside the frame: reception stops there and d counts the words
  received whole before it); bit 4 bad sync (S misread, or hit by the
  violation); bit 5 too long (more than four data words: d = 4, the last
  four). After a report thread 1 waits for two idle bit times before it
  looks for the next frame, so a report must fit OUTQ (four words) or be
  popped before the next frame starts.
- **Rate**: set by the host for both threads, one tick per half-bit:
  TICK 12 is 2.083 Mbit/s at 50 MHz, the fastest the checker proves; TICK
  25 is 1 Mbit/s. `TICK_FRAC` may be used.
- **Why 12 clocks**: the 6.9.1 loop (`WAITD 1; SHx; WAITD 1; SHx; BNZ`) has a
  three-slot interval. Each word is that loop for most of its bits and
  straight-line code around the word boundary, where each half-bit carries
  one more instruction (load or store the word, move the word registers,
  reload `CNT`, count words, test `T`), so every interval of both threads
  is three slots at most: 0 clocks of slack. `LD`/`ST` take two slots, which
  is why the words live in registers and a frame carries at most four
  (`docs/spec-questions/firmware-m4.md` item 1).
- **Transmit timing**: every edge is one slot after its tick, so at TICK 12
  (a multiple of the slot) every half-bit is exactly 12 clocks; at TICK 25
  the edges are within 3 clocks of their grid (SEMANTICS 2).
- **Receive timing**: thread 1 hard-synchronises once, on the first rising
  edge after two idle bit times (the middle of the sync word's first bit),
  by rewriting its `TICK_INT` after a delay of `(3P/2 - 11) / 4` slots that
  it computes from `TICK_INT`. Each sample then falls 4 clocks into its
  half-bit at TICK 12 and 9 to 12 at TICK 25 (measured in loopback; up to 3
  more for a sender not on the chip's clock grid). There is no resynchronisation: with 4-word frames the
  golden model receives a sender off by -0.25 % to +0.30 % at TICK 12 and
  +-0.30 % at TICK 25 (and fails at -0.30 %, +0.40 % and +-0.40 %); the
  tests use +-0.1 % and +-0.2 %. A missing transition in a word's first bit
  looks like the end marker; the receiver tells them apart by waiting three
  half-bits for an edge (item 5 of the spec questions).
- **Not done**: frames longer than four words, a length or CRC field (the
  end marker delimits the frame, the status reports what arrived),
  resynchronisation on data edges, and any medium access: thread 0 sends
  whenever the host asks.

## The D-035 programs: protocols Loom was not designed for

The programs below were written after the hardware was complete (D-032),
for protocols nothing in Loom was built for (D-035). Each says what
hardware it reuses and what that hardware was built for. Their open
questions and the decisions taken are in
`docs/spec-questions/firmware-d035.md`.

## ir_nec_tx.loom: NEC infrared transmitter

NEC is the remote-control code of most TVs. Loom has no PWM, no carrier
generator and no pulse-width capture. The transmitter makes the 38 kHz
carrier itself: every carrier edge is a deadline-latched pin write
(`SETP pin, v, D`, SEMANTICS 6.10, D-016), built so that WS2812 pulses land
on the exact clock, on a fractional tick (`TICK_FRAC`, SEMANTICS 4), built
for UART rates such as 115200 baud. The NEC burst/space envelope is a count
of those ticks.

- **Host command** (push to thread 0): a frame is two words: bits 7:0 the
  command and bit 8 EXT, then the address. Standard NEC (EXT = 0) sends
  address bits 7:0 and their inverse; extended NEC (EXT = 1) sends bits
  15:0, low byte first, with no inverse. The command is always followed by
  its inverse. A repeat is one word with bit 15 set.
- **Results**: none.
- **Rate**: TICK = a third of a carrier period, set by the host
  (TICK = f_clk / 114 kHz): 438 + 153/256 at 50 MHz (37,999.90 Hz,
  -2.7 ppm), 16 at a 1.824 MHz clock. The schedule is proved for 16 clocks
  or more.
- **The carrier**: each cycle is `SETP LED, 1, D; WAITD 2; SETP LED, 0, D;
  WAITD 1`, so every edge lands on the exact clock edge of its tick
  (SEMANTICS 6.10 rule 1). At 50 MHz a high lasts 438 or 439 clocks and a
  period 1315 or 1316: every edge is within one clock (20 ns) of the ideal
  grid. Duty 1/3: a receiver module filters the carrier's fundamental,
  which at 1/3 duty is sin(60 deg) = 87 % of a square wave's, for two
  thirds of the mean LED current.
- **The envelope**: the NEC unit (562.5 us = 64.125 ticks) is taken as 64
  ticks, 561.4 us, 0.19 % short, so every time is an exact tick count: a
  one-unit burst is 22 cycles, the 9 ms leader 342 cycles (1024 ticks), the
  spaces 64, 192, 256 and 512 ticks, start to start 12288 ticks =
  107.79 ms. Measured on the pin, every burst and space is 0.19 % short of
  nominal (the tests allow 0.5 %).
- **Repeats**: a frame or repeat that starts at S is followed by the next
  at S + 192 units (107.79 ms) if a word is queued 10 ticks (88 us at
  50 MHz) before that time: a repeat word sends a repeat code there, a
  frame sends that frame there. The test is `WAITB INQ_NE, T` right after a
  deadline (the ws2812 idiom), and the two `POP`s it guards carry
  `.bounded` declarations (25 slots against 8 ticks of 16 clocks: 28 clocks
  of slack). The worst slack, 8 clocks, is the carrier loop. With INQ empty
  at that moment the chain ends and the next command starts as soon as it
  arrives. A repeat word taken outside a chain is dropped: a repeat code
  with no frame before it would be wrong on the wire.
- **Not done**: other IR protocols (RC5, Sony SIRC). The thread is busy for
  the whole frame: each carrier edge costs it two slots, and the rest of
  its time is idle rather than free for other work.

## ir_nec_rx.loom: NEC infrared receiver

The receiver measures every mark and space of a receiver module's output
with the timed waits (`WAITP ... T` against `SETD` deadlines, SEMANTICS
6.4), built for the PS/2 and I2C timeouts, and restarts its tick generator
on every edge it accepts (a `CSRW TICK_INT` clears ACC, SEMANTICS 4), the
hard synchronisation built for the UART, CAN and Manchester receivers.

- **Input**: RX = IN0, the output of a demodulating receiver module (a
  TSOP38238 or alike: low while it sees a 38 kHz burst), idle high.
- **Results** (pop thread 0), bits 15:14 the kind: a frame is two words,
  bits 7:0 the command and bit 8 EXT, then the address (bits 15:8 as
  received: the inverse for standard NEC; EXT says they were not, extended
  NEC); a repeat code is 0x8000; an error is one word, 0xC000 | n << 8 |
  item | fault, n the data bits received whole, item bit 4 a repeat code's
  burst, bit 3 the leader, bit 2 a space, fault 1 out of window, 2 too
  long (a timeout), 3 the command's inverse. A report is never split; the
  program blocks on a full OUTQ, so a host pops within about 40 ms.
- **Rate**: TICK = 1/16 of the NEC unit, 35.156 us, set by the host: 1757 +
  208/256 at 50 MHz, 64 + 32/256 at 1.824 MHz (both exact); 64 clocks or
  more. The program reads TICK_INT once at start for the glitch filter.
- **Windows**: each mark and space within +-25 % of nominal plus 4 ticks
  (140.6 us) for the receiver module's output-pulse error (the header gives
  the datasheet figure). Leader 188 .. 324 ticks (6.61 .. 11.39 ms), its
  space 92 .. 164 (3.23 .. 5.77 ms) for a frame or 44 .. 84 (1.55 ..
  2.95 ms) for a repeat code, a mark 8 .. 24 (281 .. 844 us), the space of
  a 0 8 .. 24 and of a 1 32 .. 64 (1.125 .. 2.25 ms). On an idle line a low
  pulse shorter than a leader is ignored silently; every other violation
  is one error word, and the receiver waits for an idle line again.
- **How**: at each accepted edge the program restarts its tick grid, then
  walks the window edges with `SETD` and a timed `WAITP` per edge:
  completing before a deadline puts the pulse in the window below it,
  timing out moves to the next. No `WAITD` and no deadline pair: like
  `ps2_host`, everything is a timeout.
- **Glitch filter**: an edge counts only if the new level is still there 2
  ticks (70.3 us) later, a delay that scales with the tick; a shorter pulse
  of either level is ignored and the interrupted wait goes on against the
  same deadline.
- **Measurement error**: an edge that comes while the program moves from
  one wait to the next is seen up to 24 clocks late, so a pulse is measured
  to within about 30 clocks: 0.6 us at 50 MHz, 0.4 tick at 1.824 MHz.
- **Not done**: no check that a repeat code follows a frame within 108 ms,
  no remembered command (the host has both); other IR protocols.

Both programs fit a quarter of the memory and run together: the
transmitter on thread 0 (0x000..0x046), the receiver moved to thread 1
(0x080..0x0EF) by changing its `.thread` line.

**Tests.** `tools/tests/test_fw_ir_nec.py`, with `tools.protomodels.ir_nec`
(an encoder and decoder, a carrier analyser that rebuilds the carrier and
the envelope from the LED pin's edges, a receiver-module model between the
two programs' pins, and a source of demodulated frames with timing
variations, glitches and faults). On the RTL (the default set, all on a
board clocked at 1.824 MHz, where the waveform in microseconds is the same
in 27 times fewer clocks than at 50 MHz): a frame and its repeat code from
thread 0 through the receiver-module model to thread 1, glitches of both
levels, and the two PHY-GL-SDF candidates (the transmitter's leader and
four bits, the receiver's repeat code), 627,600 clocks. On the model only,
with the reason in each mark: the 50 MHz carrier and a 50 MHz receive, the
chain rules, the tolerance edges of every window, thirteen faults, and the
two programs together at the receiver module's timing limits.

## onewire_master.loom: 1-Wire master with the ROM search

- **Written** 2026-10-11, after the hardware was complete (D-032), as a
  protocol Loom was not designed for (D-035). Hardware reused: the
  open-drain BIDIR pin (built for I2C), the deadline-latched `SETP ... D`
  (built for clock-exact edges, WS2812), `SHI` and the CRC unit (built for CAN
  and USB), here Maxim's CRC-8 with `CRC_POLY = 0x31 << 8`. What only the
  firmware does: the time slots, the reset and presence detection, and
  application note 187's search.
- **Pin**: OW = BIDIR0, open drain (`OD_MASK` bit 0, read-modify-write);
  the board's pull-up (4.7 kOhm typical) makes the high level.
- **Host command** (push to thread 0): one word per operation, bits 15:8
  the operation, bits 7:0 the byte for WRITE.
  - `0x0100` RESET: the reset pulse and presence detect. Result: bit 0
    PRESENCE, bit 1 SHORT (the line still low 480 us after the release).
  - `0x02bb` WRITE byte `bb`, LSB first. No result.
  - `0x0300` READ a byte. Result: bits 7:0 the byte, bits 15:8 the CRC-8
    of the bytes read since the last RESET or WRITE, this one included:
    after a DS18B20's nine scratchpad bytes or a READ ROM's eight it is 0
    exactly when the CRC byte matched.
  - `0x0400` SEARCH: SEARCH ROM over the whole bus. Results, per device in
    AN187's order: a status word (bits 7:0 the CRC-8 the CRC unit computed
    over all 64 bits: 0 for a good ID; bit 15 clear), then the ID as four
    words, bits 15:0 first (family code in 7:0 of the first). Then an end
    marker: bit 15 END, plus bit 1 NO PRESENCE, bit 2 SHORT or bit 3 NO
    DEVICE (bit and complement both 1) when the search stopped on a fault.
    A CRC error ends the search after that ID (AN187 restarts on one; a
    search that went on over a faulty line could branch without end).
  - Example, a DS18B20 scratchpad: `0x0100` (0x0001 back), `0x02CC`,
    `0x02BE`, nine `0x0300`; the ninth result is `0x00nn`.
- **Rate**: fixed by AN126, so the program writes `TICK_INT = 50` itself (1
  us at 50 MHz); **the host must not**. Another board clock: change `US`.
- **Timing**: AN126 table 2, standard speed, every edge deadline-latched
  and so exact to the clock: write 1 low 6 us, write 0 low 60 us, slots
  70 us apart inside a byte (recovery 10 us after a write 0, 64 after a
  write 1), reset low 480 us, presence sampled 70 us after the release and
  the next slot at least 490 us after it (J = 410 plus the 10 us
  recovery; 519 in the READ ROM test, where the host sends the next
  command). One subroutine makes every slot: fall, level at 6 us
  (released for a 1 or a read), sample by `SHI` (into SR and the CRC),
  release at 60 us. The one
  departure from AN126 is the read sample: E = 8, not 9. AN126's sample
  at 15.0 us is where the DS18B20's guarantee (tRDV) ends, and Loom's
  sample sees the line 2 to 5 clocks after its deadline (one slot after
  the `WAITD`, through the synchroniser), so E = 9 would be up to 0.1 us
  late; at E = 8 it is 14.04 to 14.10 us after the fall.
- **The search**: AN187's algorithm with the ID in r0..r3 (the word being
  searched in r0, the previous pass's bit consumed from its bottom while
  the new bit enters SR through the direction slot's `SHI`), LastDiscrepancy
  in r4, last_zero in r5, the bit number in r6. Each bit is two read slots
  and a write slot, 210 us; a pass is about 15 ms. The CRC unit sees exactly
  the 64 direction bits, so its residue is the ID's CRC check.
- **The first version** re-anchored with `SETD 1` after `POP`: a staged
  write lands on the thread's next deadline whatever set it (SEMANTICS
  6.10), and the slot's fall, staged before that tick, landed on it ten
  ticks early (a 490 us reset pulse). `SETD 0` sets a deadline that is
  already reached, so the fall lands on the `WAITD` meant for it.
- **Weaker than a dedicated master** (DS2482 and the like): no overdrive,
  no strong pull-up for parasite-powered parts, no active pull-up for long
  lines, a fixed 1 us grid (the read sample is 14 us, not adjustable), and
  a host that must pop the search reports (the search waits while OUTQ is
  full; the line idles high meanwhile).

## onewire_device.loom: a DS18B20-style 1-Wire device

- **Written** 2026-10-11, after the hardware was complete (D-032), as a
  protocol Loom was not designed for (D-035). Hardware reused: the
  open-drain BIDIR pin (I2C), `WAITP` and the timed waits, `SHO`/`SHI` with
  `BE_PINS` pointing both ways at the one pin (built for the serial
  protocols), the CRC unit (CAN, USB) for the scratchpad's CRC-8, and
  `LD`/`ST` (slice B, built for the EEPROM) for the ID and the scratchpad.
  What only the firmware does: the slot protocol, reset detection, the
  presence pulse, the ROM and function commands and the search's bit,
  complement and compare.
- **Pin**: OW = BIDIR1, open drain. With `onewire_master.loom` on BIDIR0
  (thread 0) the board joins the two pins into one line: two threads never
  share a pin.
- **Host words** (thread 1): at start, the ID as four words, bits 15:0
  first (family code 0x28 in 7:0 of the first; sent as given, a corrupt CRC
  included), then scratchpad bytes 0..7 as four words, byte 2k in bits 7:0.
  They are stored at 0x0F8..0x0FF, where the host may rewrite them while the
  thread is halted. The device stays off the line until all eight are in.
  After each WRITE SCRATCHPAD it pushes `{TL, TH}` (TH in 7:0) and the
  configuration byte, and keeps the three bytes as scratchpad bytes 2..4.
- **Commands**: READ ROM, MATCH ROM (silent until the next reset for
  another ID), SKIP ROM, SEARCH ROM (the bit, its complement, then the
  master's choice; a different choice drops the device out); then READ
  SCRATCHPAD (bytes 0..7 through `SHO` with the CRC unit on, then the CRC
  as byte 8; after it the line stays high) and WRITE SCRATCHPAD. Any other
  command: silent until the next reset.
- **Rate**: fixed by the datasheet, the program writes `TICK_INT = 50`
  itself; the host must not.
- **Timing**, from L, the moment the device sees the line low (a few slots
  after the master's fall F; times in 1 us ticks from a `SETD 0` at L, so
  up to one tick short): write slots sampled by `SHI` at L + 30 (window
  15..60); in a read slot a 0 driven by `SHO` one slot after L and
  released at L + 30 (measured against the AN126 master: 0.16 us and 29.28
  us after F, against 15 us valid and 60 us released); a low still there at
  L + 240 is a reset (slots are at most 120 us, resets at least 480), and
  the presence pulse follows its rise after 30 us and lasts 120 (measured
  29.28 us high, 120.0 us low, against 15..60 and 60..240). Every slot starts with `WAITP OW, 0`, not `WAITE`, so a low that
  began while the device was busy between bytes is still caught while it
  lasts, and ends with `WAITP OW, 1, T`, whose timeout is the reset test.
- **Weaker than a DS18B20**: no temperature conversion (CONVERT T is
  ignored: the host supplies the scratchpad), no ALARM SEARCH, COPY or
  RECALL, no parasite power; the configuration byte is kept as written
  (a DS18B20 keeps bits 6:5 only); and the report to the host is a blocking
  `PUSH`, so with OUTQ full the device misses the master's slots.

**Tests.** `tools/tests/test_fw_onewire.py` with `tools.protomodels.onewire`: a
`OneWireBus` (the wired AND of the chip's open-drain pads, joined when there
are two, with the bench pull-up), `Ds18b20` devices (each with its own ID
and scratchpad, written from the datasheet: it samples each write slot at
both ends of the 15..60 us window and records a bit that differs, and holds
a read 0 for exactly the guaranteed 15 us, so a late master sample reads a
1) and `OneWireMaster` (AN126's delays, AN187's search; it measures the
presence pulse and watches every bit a device drives). Every low pulse the
chip's master makes is held to AN126 to the clock and to the datasheet's
windows; every 0 the device sends is measured from the master's fall. The
golden model is built with `FIFO`, `BE`, `SETPD` (and `DMEM` for the
device) and the 512-word memory. 1-Wire at standard speed is slow for a
simulator (a reset is 48000 clocks, a byte 28000, a search pass 750000;
the golden model runs these programs at about 16000 clocks a second and
Icarus at about 6400), so:

- at the real rate run the resets and presence pulses, both READ ROMs, the
  master's search faults (no presence, nobody answering, a shorted line:
  the read slots of the search, at the real rate) and the device against a
  master at the edges of the datasheet's windows. The RTL set is four of
  them: `test_master_reset_and_presence`, `test_master_read_rom_of_one_device`,
  `test_device_presence_pulse`, `test_device_presence_and_read_rom`, about
  17.8 million ns of simulation (2 min 20 s on an idle laptop);
- the searches (1, 2, 3 and 8 devices, and a corrupt ROM CRC), the
  scratchpad scenarios of both programs and the two programs together run
  on a time-scaled build: the same source with `US = 10` instead of 50, so
  every interval is the same number of ticks and one fifth as many clocks
  (the deadline checker proves both programs at that tick), against models
  clocked at 10 MHz. They test the protocol and the search order; the
  timing to the clock is the real-rate scenarios' job.

`test_model_*` check the reference models against each other with no chip
(`OneWireBus.run_alone`): the catalogue CRC-8/MAXIM check value, and the
reference master's AN187 search over five reference devices.

## lin_master.loom: LIN master

- **Written** 2026-10-11, after the hardware was complete (D-032), as a
  protocol Loom was not designed for (D-035): LIN, the single-wire bus of
  car body electronics. Hardware reused: the M2 core (pins, deadlines, the
  FIFOs) and `LD`/`ST` of slice B, built for the I2C EEPROM's array and
  USB's descriptors, here the frame as it goes on the bus. No hardware
  knows LIN: the break, the PID parity, the checksum's end-around carry,
  the read-back bit check and the response timeout are firmware.
- **What it does**: a LIN 2.2A / ISO 17987 master for a transceiver such as
  the TJA1021 (TX = OUT0 to its TXD, RX = IN0 from its RXD). Per frame: a
  break of 13 bits (208 to 209 ticks of a sixteenth of a bit), a one-bit
  delimiter, the sync byte 0x55, the protected identifier (P0 and P1
  worked out with `PAR`), then either its own response (1 to 8 data bytes
  and the checksum) or a slave's response, read and checked. Every bit it
  sends is read back from RX at its middle; a mismatch is a bit error.
- **Host command** (push to thread 0): `{N - 1 in 15:13, PUB in 12, ID in
  5:0}`, then for PUB the N data bytes, one per word in 7:0.
- **Result** (pop thread 0): a status word, then n words of one byte each:
  the bytes on the bus after the PID (data, then checksum), what the
  master sent or what the slave sent. Status: bits 7:4 n (N + 1 for a whole
  response, 0 for none), bit 0 checksum error, bit 1 no response, bit 2 bit
  error, bit 3 framing error.
- **Checksum**: enhanced (the PID and the data) except for the diagnostic
  frames 0x3C and 0x3D (classic), worked out with `ADD` and the carry folded
  back by hand; a response is checked by summing the data and the checksum,
  which must give 0xFF.
- **Rate**: sixteen ticks a bit, set by the host (M2 convention): 19200
  bit/s is TICK 162 + 195/256, 10417 is TICK 300, 9600 is TICK 325 +
  133/256.
- **Reading a slave**: every byte is sampled from its own start edge (`SETD
  0` on the edge, then `WAITD 8` and `WAITD 16`), 7/16 to 1/2 of a bit into
  each bit plus the latency (measured at 256 clocks a bit: 7 to 11 clocks
  after mid-bit), so a slave within the 2 % a synchronised node may be off is
  read whole, however late it starts (tested at +-2 %). The response
  deadline is TResponse_max = 1.4 x 10 (N + 1) bit times: byte j must start
  by 4 (N + 1) + 10 j bit times after the header, or the response cannot
  end in time; a late or missing byte gives "no response" and the bytes
  that did come.
- **Memory**: 128 words, all of thread 0's quarter, and the frame (sync,
  PID, data, checksum, one byte per word) in 11 words at 0x1F5..0x1FF, the
  top of thread 3's quarter, through `LD`/`ST`: threads 1 and 2 may run
  other programs (the LIN slave on thread 1, say), thread 3 must not.
- **Not done**: schedule tables (the host sends each frame), sleep and
  wake-up, the diagnostic transport layer, collision handling of
  event-triggered frames.

## lin_slave.loom: LIN slave with automatic bit rate

- **Written** 2026-10-11, after the hardware was complete (D-032), as a
  protocol Loom was not designed for (D-035). Loom has no baud-rate
  detector: the slave times the bus with timed waits and `NOW` and writes
  its own `TICK_INT`, the divider built so that the host could pick a
  rate, from what it measured: clock recovery in firmware. `LD`/`ST` of
  slice B (built for the I2C EEPROM) hold its table.
- **What it does**: a LIN slave that does not know the bus rate. It finds
  each break, measures the master's bit time on the sync byte that follows
  and writes its own `TICK_INT` from it, checks the protected identifier's
  parity, and publishes or subscribes according to a table the host gives
  at start. Pins TX = OUT0, RX = IN0 (OUT1, IN1 beside the master).
- **Host command** (push to thread 0, once, before the first header): c
  (0 .. 8), then c entries: a header `{N - 1 in 15:13, PUB in 12, ID in
  5:0}` and, for PUB, the N data bytes one per word. The slave works out
  each checksum (enhanced, or classic for 0x3C and 0x3D) and keeps the
  table, eight entries of ten words, at 0x180..0x1CF through `LD`/`ST`.
- **Result** (pop thread 0): per header with a good sync whose identifier
  is in the table, or whose PID parity is wrong: a status (bits 7:0 the
  PID as received, 11:8 n, bit 12 published, bit 13 checksum error, bit 14
  parity error, the frame ignored), the bit time measured in clocks, then
  the n bytes received of a subscribed frame (data and checksum). The
  report waits for room in OUTQ between frames.
- **Rate**: any from 763 bit/s up; tested against a master at 9600, 10417
  and 19200 bit/s with its clock 14 % slow, exact and 14 % fast, and at 100
  kbit/s. The program writes `TICK_INT` and `TICK_FRAC`; the host must not.
- **Finding the break**: on a 32-clock tick the slave times every falling
  edge and the rise after it. A low run D followed by five falling edges
  e1 .. e5 is a header when the four intervals are a sync byte's (2 bits
  each, the first one i1, the others within i1/8 of it, e1's low run half
  of i1) and 2 D >= 11 i1: a break of at least 11 of the bits the sync
  measures, whatever the rate. The checks run edge by edge and the first
  that fails makes the low run just timed the new candidate, so a break
  that starts while a false candidate is checked is not lost. Data cannot
  pass: no byte has a low run longer than 9 bits.
- **Clock recovery**: e5 - e1 is 8 bit times, n ticks of 32 clocks, so a
  half-bit is 2 n clocks: `TICK_INT = 2 n`, `TICK_FRAC = 0`. Each edge is
  read 6 to 9 clocks after the pad changed and rounded down to the tick, so
  32 n is within 35 clocks of the true 8 bit times: the half-bit within
  2.2 clocks, 0.18 % at 20 kbit/s and 0.08 % at 9600 bit/s against the 2 %
  a synchronised slave may be off (F_TOL_SYNC as the model reads LIN 2.2A;
  see the spec questions, LIN 1). Measured: the reported bit
  time within 0.2 % of the master's in every test.
- **Bytes**: each byte received restarts the tick grid on its start edge
  (a `TICK_INT` write clears the accumulator, SEMANTICS 4) and is sampled
  on the half-bit ticks: measured 14 to 21 clocks after the middle of each
  bit, at 19200 bit/s and at 100 kbit/s. Each byte sent keeps the grid of
  its start bit, every edge two slots after its tick.
- **Weaker than a peripheral**: a LIN controller with a baud-rate detector
  times the sync with its full clock; here the slot grid and the 32-clock
  tick bound the error to 35 clocks, still eleven times inside the
  tolerance at 20 kbit/s. The slave cannot change its published data while
  it runs (the table is given once), does not read back its own bits, and
  a break longer than 0x7FFF ticks (21 bit times at 1 kbit/s) is not seen.
- **Memory**: 218 words, 90 of them in the next quarter (beside the master:
  0x080..0x159), and the 80-word table. With `lin_master` on thread 0:
  threads 2 and 3 must not be started.

**Tests.** `tools/tests/test_fw_lin.py` runs both programs against
`tools.protomodels.lin`: `LinBus` (the wired AND of the chip's TX pins and
the model nodes, seen by the chip's RX pins one clock later),
`LinMasterNode` (a master with its own clock, a settable break and
delimiter, a spoiled parity or checksum on request), `LinSlaveNode` (a
slave with a table, a spoiled checksum or silence on request) and
`LinSniffer`; every node decodes each byte with its edges, so the tests
measure the chip's pins against the specification. It builds the model
with `FIFO` and `DMEM` and the 512-word memory. On the RTL it runs six
scenarios (the two programs together on one bus, the master's own frame,
no response, a slave's checksum error, a bit error, and the slave
ignoring a bad parity); the other 24 are `model_only`, each with its
reason: the LIN-rate cases (150,000 to 600,000 clocks a frame) and those
whose code paths the six already cover.

## Tests

`tools/tests/test_fw_uart.py`, `test_fw_spi.py`, `test_fw_spi_slave.py`,
`test_fw_i2c.py`, `test_fw_ws2812.py`, `test_fw_ps2.py`, `test_fw_jtag.py`,
`test_fw_swd.py`, `test_fw_i2c_slave.py`, `test_fw_can.py`,
`test_fw_manchester.py`, `test_fw_ir_nec.py`, `test_fw_onewire.py`, `test_fw_lin.py` and `test_fw_usb.py` (with `tools.protomodels.usb.UsbHost`,
a low-speed host that checks every device packet against the USB 2.0
timing limits) run each program end to end, loaded, configured and fed
through `tools.loomhost.Loom` (every host access is SPI bytes, 64 clocks
each, with the firmware running meanwhile), against the reference models in
`tools/protomodels` on the pads. Each body takes a *backend*
(`tools/tests/fw_backend.py`) and builds its bench and its transport through
it, so the same bodies run twice:

- under pytest on the **golden model**, `tools.protomodels.bench.Bench` over
  `ModelTransport`;
- under cocotb on the **RTL** (`test/test_fw.py`), `test/rtl_bench.py`'s
  `RtlBench` clocking `tb.v` with the same pad resolution, and
  `tools.loomhost.SimTransport` moving the host bytes through the real SPI
  pads of the design. That is the M2 exit criterion of `docs/PLAN.md`.

A body marked `model_only` runs on the model alone, with the reason in the
mark: the 115200-baud UART receive cases, most of the SPI master mode sweep,
the two longest PS/2 scenarios (a PS/2 frame is 33000 clocks at 16.7 kHz
and 55000 at 10 kHz) and the USB host at +-0.25 % of the bit rate, which
would add minutes of simulation and prove what their faster or shorter
siblings already prove.

`usb_ls_device` needs the golden model built like the RTL, with slices A
and B (`features` `BEENC` and `DMEM`) and the 512-word memory it is laid out
for; `test_fw_usb.py` asks for both on the model backend only.

`ws2812` needs the golden model built with the deadline latch
(`features={"FIFO", "SETPD"}`), which the RTL has unconditionally, so
`test_fw_ws2812.py` adds that feature for the model backend only; see
`docs/spec-questions/firmware-m3.md` item 2. In the same way
`test_fw_i2c_slave.py` builds the model with `DMEM`,
`test_fw_can.py` with `BE`, `BEENC` and `SETPD`, and
`test_fw_manchester.py` with `BE` and `BEENC`, all with the 512-word
memory. Their references are `tools.protomodels.i2c_master.I2cBusMaster`
(a master timed by UM10204 table 10 at 100 and 400 kHz, which checks tVD on
every bit the device drives) and `tools.protomodels.can` (the bus as a wired
AND, and a node with its own bit clock that ACKs, sends, spoils its own
frames with a stuff or a CRC error on request, and can force a dominant bit
into the chip's frame to make a form error), and
`tools.protomodels.manchester` (a wire from TX to RX, or a sender with
its own clock, a settable rate offset and an injectable missing
transition, plus a decoder that holds the chip's transmitter to its
half-bit grid). The one `model_only` body among them is the 257-byte sequential read of the whole array (300000 clocks);
its 12-byte sibling covers the wrap at 0xFF on both backends.
