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

The four M3 programs at the bottom of the table are thread 0 as well and need
nothing the M2 hardware does not have. Two of them break that convention:
`ws2812` and `ps2_host` have a rate fixed by a datasheet rather than chosen
by the host, so they write their own `TICK_INT` with `.csr` and **the host
must not set it**. `ws2812` also uses the deadline-latched pin write of
`SEMANTICS.md` 6.10 (`SETP pin, v, D`), which the M2 programs deliberately do
not. `jtag_master` and `swd_master` keep the M2 convention: they own their
clock, so the host sets its rate.

| Program | Pins | Words | `.tick` (proved) | Worst slack | Rates tested |
|---|---|---|---|---|---|
| `uart_tx_fifo.loom` | TX = OUT0 | 18 | 24 clocks per bit | 4 clocks | 32 clocks/bit (1.5625 Mbaud at 50 MHz); 115200 baud (TICK 434 + 8/256) |
| `uart_rx.loom` | RX = IN0 | 47 | 8 clocks per tick, 8 ticks per bit | 20 clocks | 64 clocks/bit (781 kbaud); 115200 baud (TICK 54 + 64/256); +-3 % sender error |
| `spi_master.loom` | MOSI = OUT0, SCK = OUT1, CS_n = OUT2, MISO = IN0 | 72 | 28 clocks per half SCK period | 0 clocks | TICK 32: 781 kHz SCK, modes 0-3, MSB and LSB first |
| `spi_slave.loom` | MISO = OUT0, SCK = IN1, MOSI = IN2, CS_n = IN3 | 50 | 32 clocks per tick, 4 ticks per SCK period | 8 clocks | TICK 32: 390 kHz SCK, modes 0 and 3 |
| `i2c_master.loom` | SCL = BIDIR0, SDA = BIDIR1 (open drain) | 105 | 5 clocks per tick, 16 ticks per SCL period | 0 clocks | TICK 8: 390 kHz SCL; 100 kHz is TICK 31 + 64/256 |
| `ws2812.loom` | DOUT = OUT0 | 52 | 1 clock per tick (set by the program) | 4 clocks (two `.bounded` POPs) | the WS2812B waveform at 50 MHz: T0H 20, T1H 40, period 64 clocks |
| `ps2_host.loom` | CLK = IN0, DATA = IN1 | 45 | 64 clocks per tick (set by the program), timeouts only | no deadline pairs | 10 kHz and 16.67 kHz device clock |
| `jtag_master.loom` | TCK = OUT0, TMS = OUT1, TDI = OUT2, TDO = IN0 | 41 | 32 clocks per half TCK period | 0 clocks | TICK 32: 781 kHz TCK |
| `swd_master.loom` | SWCLK = OUT0, SWDIO = BIDIR0 (push-pull, pull-up) | 92 | 28 clocks per half SWCLK period | 0 clocks | TICK 32: 781 kHz SWCLK |

Every program assembles with `--strict` and no diagnostic at all: each
deadline pair is proved and none is unbounded (the M2 five in
`tools/tests/test_fw_build.py`, the M3 four in their own test modules). Each
fits in thread 0's quarter of the 512-word memory of D-020.

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
  of 4 clocks, within one slot otherwise (tested at 115200 baud). Queued words
  go out back to back with one stop bit; after an idle period the start bit
  waits for the next tick boundary, so it is always a full bit.

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
- **Rate**: one tick is a quarter of an SCK period, `TICK_INT` = clocks per
  SCK period / 4. What limits the rate is not the bit loop but the 21 slots
  between two bytes (push the byte received, pop the next one to send, put
  its first bit on MISO): 84 clocks inside the three quarters of a period
  (96 clocks at TICK 32) that separate a byte's last sampling edge from the
  next byte's first one. A master that pauses between bytes can clock the
  bits themselves faster.
- **Modes 0 and 3**: both sample MOSI and MISO on the rising edge, so one
  loop serves both and the idle level and the falling edge are never looked
  at. Modes 1 and 2 sample on the falling edge and are not supported.
- Each bit is one `WAITE SCK, RISE` (the master samples there), then MOSI is
  sampled and the next MISO bit is written at a deadline one tick later, a
  quarter period before the trailing edge: that `SETD 0` to `WAITD 1`
  interval is the deadline pair the checker proves (24 of 32 clocks).
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

## Tests

`tools/tests/test_fw_uart.py`, `test_fw_spi.py`, `test_fw_spi_slave.py`,
`test_fw_i2c.py`, `test_fw_ws2812.py`, `test_fw_ps2.py`, `test_fw_jtag.py`
and `test_fw_swd.py` run each program end to end, loaded, configured and fed
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
and the two longest PS/2 scenarios (a PS/2 frame is 33000 clocks at 16.7 kHz
and 55000 at 10 kHz), which would add minutes of simulation and prove what
their faster or shorter siblings already prove.

`ws2812` needs the golden model built with the deadline latch
(`features={"FIFO", "SETPD"}`), which the RTL has unconditionally, so
`test_fw_ws2812.py` adds that feature for the model backend only; see
`docs/spec-questions/firmware-m3.md` item 2.
