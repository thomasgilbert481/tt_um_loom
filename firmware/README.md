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

| Program | Pins | Words | `.tick` (proved) | Worst slack | Rates tested |
|---|---|---|---|---|---|
| `uart_tx_fifo.loom` | TX = OUT0 | 26 | 24 clocks per bit | 4 clocks | 32 clocks/bit (1.5625 Mbaud at 50 MHz); 115200 baud (TICK 434 + 8/256) |
| `uart_rx.loom` | RX = IN0 | 47 | 8 clocks per tick, 8 ticks per bit | 20 clocks | 64 clocks/bit (781 kbaud); 115200 baud (TICK 54 + 64/256); +-3 % sender error |
| `spi_master.loom` | MOSI = OUT0, SCK = OUT1, CS_n = OUT2, MISO = IN0 | 72 | 28 clocks per half SCK period | 0 clocks | TICK 32: 781 kHz SCK, modes 0-3, MSB and LSB first |
| `spi_slave.loom` | MISO = OUT0, SCK = IN1, MOSI = IN2, CS_n = IN3 | 48 | none: the master's SCK times it, no deadline pairs | no deadline pairs; MISO 28 to 35 clocks after each sampling edge | 390 kHz SCK (128 clocks, TICK 32 for the silence timeout), modes 0 and 3 |
| `i2c_master.loom` | SCL = BIDIR0, SDA = BIDIR1 (open drain) | 105 | 5 clocks per tick, 16 ticks per SCL period | 0 clocks | TICK 8: 390 kHz SCL; 100 kHz is TICK 31 + 64/256 |
| `ws2812.loom` | DOUT = OUT0 | 52 | 1 clock per tick (set by the program) | 4 clocks (two `.bounded` POPs) | the WS2812B waveform at 50 MHz: T0H 20, T1H 40, period 64 clocks |
| `ps2_host.loom` | CLK = IN0, DATA = IN1 | 45 | 64 clocks per tick (set by the program), timeouts only | no deadline pairs | 10 kHz and 16.67 kHz device clock |
| `jtag_master.loom` | TCK = OUT0, TMS = OUT1, TDI = OUT2, TDO = IN0 | 41 | 32 clocks per half TCK period | 0 clocks | TICK 32: 781 kHz TCK |
| `swd_master.loom` | SWCLK = OUT0, SWDIO = BIDIR0 (push-pull, pull-up) | 92 | 28 clocks per half SWCLK period | 0 clocks | TICK 32: 781 kHz SWCLK |
| `i2c_slave_eeprom.loom` | SCL = BIDIR0, SDA = BIDIR1 (open drain) | 111, and 128 of data at 0x180 | none: the master owns SCL, no deadline pairs | no deadline pairs; SDA valid about 30 clocks after SCL falls (tVD 45 at 400 kHz) | 100 kHz and 400 kHz SCL (UM10204 Standard and Fast mode timing) |
| `can_loopback.loom` | TX = OUT0, RX = IN0 | 79 (thread 0) + 140 (thread 1) | thread 0: 100 clocks per bit; thread 1: 12 clocks per tick, 8 ticks per bit | 28 clocks (thread 0), 4 clocks (thread 1) | 500 kbit/s (TICK 100; thread 1 TICK 12 + 128/256) and 125 kbit/s (TICK 400; TICK 50) |
| `usb_ls_device.loom` | D+ = BIDIR0, D- = BIDIR1 (push-pull while sending; 1.5 kOhm pull-up on D-) | 511 of 512 (all four quarters, slices A and B) | 33 clocks per tick, one tick per bit (set by the program: 33 + 85/256) | 1 clock | USB low speed, 1.5 Mbit/s at 50 MHz; host at +-0.25 %; 2-bit host gaps |
| `manchester_loopback.loom` | TX = OUT0, RX = IN0 | 60 (thread 0) + 113 (thread 1) | 12 clocks per tick, one tick per half-bit, both threads | 0 clocks (both threads) | 2.083 Mbit/s (TICK 12) and 1 Mbit/s (TICK 25), wire loopback; a sender at +-0.1 % (TICK 12) and +-0.2 % (TICK 25) |

Every program assembles with `--strict` and no diagnostic at all: each
deadline pair is proved and none is unbounded (the M2 five in
`tools/tests/test_fw_build.py`, the M3 and M4 programs in their own test
modules). Each fits in thread 0's quarter of the 512-word memory of D-020,
except three of the M3 programs: `i2c_slave_eeprom`'s data fills thread 3's quarter;
`can_loopback`'s thread 1 (140 words from 0x080) runs 12 words into thread
2's quarter, which no thread of that program runs from; and
`usb_ls_device` needs the whole memory for itself (a thread-0-only
program: the other three threads must not be started with it loaded).
`manchester_loopback`'s two threads each stay in their own quarter (60
words from 0x000, 113 from 0x080).

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
- **Modes 0 and 3**: both sample MOSI and MISO on the rising edge, so one
  loop serves both and the idle level and the falling edge are never looked
  at. Modes 1 and 2 sample on the falling edge and are not supported.
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
- **Rate**: 16 ticks per SCL period (4 per quarter), `TICK_INT` = clocks per
  SCL period / 16.
- SCL and SDA are open drain through `OD_MASK` (set with a read-modify-write).
  After releasing SCL the program waits for it with a timed `WAITP SCL, 1, T`
  (clock stretching, timeout 255 ticks), then re-anchors with `SETD 0`, so
  whatever a slave did to the low phase the high phase that follows is a full
  one: 8 ticks less up to one tick, plus a few slots of pin latency. Measured
  at TICK 8: low 68 clocks inside a byte (8 ticks and one slot), high 76.

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

## Tests

`tools/tests/test_fw_uart.py`, `test_fw_spi.py`, `test_fw_spi_slave.py`,
`test_fw_i2c.py`, `test_fw_ws2812.py`, `test_fw_ps2.py`, `test_fw_jtag.py`,
`test_fw_swd.py`, `test_fw_i2c_slave.py`, `test_fw_can.py`,
`test_fw_manchester.py` and `test_fw_usb.py` (with `tools.protomodels.usb.UsbHost`,
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
