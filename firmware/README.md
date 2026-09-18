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

| Program | Pins | Words | `.tick` (proved) | Worst slack | Rates tested |
|---|---|---|---|---|---|
| `uart_tx_fifo.loom` | TX = OUT0 | 18 | 24 clocks per bit | 4 clocks | 32 clocks/bit (1.5625 Mbaud at 50 MHz); 115200 baud (TICK 434 + 8/256) |
| `uart_rx.loom` | RX = IN0 | 47 | 8 clocks per tick, 8 ticks per bit | 20 clocks | 64 clocks/bit (781 kbaud); 115200 baud (TICK 54 + 64/256); +-3 % sender error |
| `spi_master.loom` | MOSI = OUT0, SCK = OUT1, CS_n = OUT2, MISO = IN0 | 72 | 28 clocks per half SCK period | 0 clocks | TICK 32: 781 kHz SCK, modes 0-3, MSB and LSB first |
| `i2c_master.loom` | SCL = BIDIR0, SDA = BIDIR1 (open drain) | 105 | 5 clocks per tick, 16 ticks per SCL period | 0 clocks | TICK 8: 390 kHz SCL; 100 kHz is TICK 31 + 64/256 |

All four assemble with `--strict` and no diagnostic at all: every deadline
pair is proved and none is unbounded (`tools/tests/test_fw_build.py`). Each
fits in thread 0's quarter of the 512-word memory of D-020.

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

## Tests

`tools/tests/test_fw_uart.py`, `test_fw_spi.py`, `test_fw_i2c.py` run each
program end to end on the golden model: loaded, configured and fed through
`tools.loomhost.Loom` over `ModelTransport` (every host access is SPI bytes,
64 clocks each, with the firmware running meanwhile), against the reference
models in `tools/protomodels` on the pads.
