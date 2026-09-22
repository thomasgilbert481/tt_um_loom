# Loom host protocol (SPI)

Version 0.2, 2026-09-18 (M2 FIFO, IRQ and debug additions). Implemented by `loom_spi_host.v` (bit layer) and
`loom_host_ctl.v` (command layer). Driven by `tools/loomhost` from Python over
any transport.

## Electrical

- SPI slave, mode 0 (CPOL=0, CPHA=0): SCK idles low, MOSI sampled on the rising
  edge, MISO changes on the falling edge. MSB first.
- Pins: HOST_CS_n = ui_in[4], HOST_SCK = ui_in[5], HOST_MOSI = ui_in[6],
  HOST_MISO = uo_out[7], HOST_IRQ = uo_out[6]. On the Tiny Tapeout demo board
  (RP2040) these are GP17, GP18, GP19 and GP16, which is the RP2040's SPI0
  CSn/SCK/TX/RX function set, so `machine.SPI(0)` drives Loom without PIO. The
  v3 demo board (RP2350B) maps project pins differently; use PIO or bit-bang
  there (`docs/tt_cmos5l_facts.md` section 5).
- SCK is sampled by the core clock through a 2-flop synchroniser and edge
  detector. Constraint: SCK period >= 8 core clocks (6.25 MHz at 50 MHz). CS_n
  must be low at least 4 core clocks before the first SCK edge and stay low
  until 4 clocks after the last.
- CS_n rising ends the transaction and resets the byte counter, at any point.
  A transaction cut mid-word is discarded (no partial writes).
- Release CS_n around a chip reset. The SCK edge detector and its synchroniser
  reset to 0, so a reset released while SCK is high walks that high level
  through the synchroniser and counts as a rising edge two clocks later. With
  CS_n still low the byte boundary is then one bit early for the rest of that
  CS_n low period (formal finding F-3 in `formal/README.md`); with CS_n high,
  as after any normal reset, nothing is in flight and the phantom edge is
  discarded.
- HOST_MISO is driven 0 when CS_n is high (no tristate; the TT output is always
  driven).

## Transaction format

```
byte 0        CMD   = {RW, SPACE[2:0], 0000}       RW: 1 write, 0 read
byte 1..2     ADDR  = 16-bit, MSB first
write:        byte 3..  DATA words, 16-bit MSB first, ADDR increments per word
read:         byte 3    dummy (turnaround), then DATA words out on MISO, ADDR increments
```

Every word is 16 bits. A transaction may transfer any number of words; the
address wraps within the space. Writes take effect at the end of each complete
word: if E is the first rising core clock edge at which the first flop of the
SCK synchroniser samples HOST_SCK high for the word's last bit, every effect
of the word is registered at edge **E + 4** and visible from the cycle after
it. This holds for every write in every space, for the pop of a FIFO read word
and for BADOP bit 14; a DEBUG write of r0..r7 may wait up to three more clocks
for the register-file write port (the thread is halted, so it cannot tell).
The falling SCK edge plays no part. (Measured on the RTL for twenty write
paths, `docs/spec-questions/rtl-m2.md` item 7.)

## Address spaces

### SPACE 0: CTRL

| ADDR | Name | R/W | Contents |
|---|---|---|---|
| 0x0000 | ID | R | 0x4C4D ("LM") |
| 0x0001 | VERSION | R | {major[7:0], minor[7:0]} |
| 0x0002 | RUN | RW | bit t = thread t running. Writing 1 starts at the current PC; writing 0 halts after the current instruction retires |
| 0x0003 | HALTED | R | bit t = thread t halted by HALT (cleared by writing RUN bit) |
| 0x0004 | RESET | W | bit t = reset thread t: PC := RESET_PC[t], flags := 0, TD := NOW, DEPTH := 0 (RS0/RS1 kept), WAIT_ACTIVE := 0, INQ/OUTQ emptied, staged pin write discarded, and from M3 the encoder state (6.9.1) and MEM_PEND (6.11) cleared; registers and CSRs untouched (`docs/SEMANTICS.md` 7) |
| 0x0008..0x000B | RESET_PC[0..3] | RW | 10-bit reset vectors, default t * (IMEM_WORDS / 4) (D-017) |
| 0x0010 | IRQ_EN | RW | mask over IRQ_STAT |
| 0x0011 | IRQ_STAT | R | {SFLAGS[7:0], INQ_NOT_FULL[3:0], OUTQ_NOT_EMPTY[3:0]}; the FIFO fields read 0 until M2 |
| 0x0012 | IRQ_STAT2 | R | {12'b0, HALTED[3:0]} |
| 0x001B | SWIRQ | R, W1C | {12'b0, SWIRQ[3:0]}: set by `CSRW HOST_IRQ` in thread t, write 1 to clear. HOST_IRQ = any(IRQ_STAT & IRQ_EN) or any(IRQ_STAT2 & IRQ_EN2) or any(SWIRQ), registered (`docs/SEMANTICS.md` 6.8) |
| 0x001C | IRQ_EN2 | RW | mask over IRQ_STAT2, bits 3:0; bits 15:4 read 0 (M2) |
| 0x0013 | SFLAGS | RW | shared flags; write sets the bits written as 1 |
| 0x0014 | SFLAGS_CLR | W | write clears the bits written as 1 |
| 0x0015 | OD_MASK | RW | open-drain mode per BIDIR pin |
| 0x0016 | PIN_OUT | RW | raw output register (host may drive pins while threads are halted) |
| 0x0017 | PIN_OE | RW | raw OE register |
| 0x0018 | PIN_IN | R | synchronised inputs |
| 0x0019 | CAPS | R | build capabilities; layout in `docs/SEMANTICS.md` section 5 |
| 0x001A | BADOP | RW | bits 3:0: thread t executed a reserved or unbuilt instruction; bit 14: host FIFO error; bit 15: host access error; write 1 to clear |

### SPACE 1: IMEM

ADDR = instruction address (0..IMEM_WORDS-1). The memory is single-port and
the core fetches from it every cycle, so host reads **and** writes are valid
only while `RUN == 0` and no single-step is in flight. Otherwise a write is
dropped, a read returns 0, and CTRL BADOP bit 15 (host access error) is set.
(`docs/SEMANTICS.md` section 7.)

### SPACE 2: DMEM (reserved)

Reads 0 and ignores writes, in every build. Data memory is the instruction
memory (D-027, `docs/SEMANTICS.md` 6.11): `CAPS[5]` means `LD`/`ST` exist,
not that a second address space does, and a data image is loaded and read
back through SPACE 1 under its rules (no step in flight). The earlier text
here, a separate dual-ported memory writable while running, predates D-027.

### SPACE 3: FIFO

| ADDR | Access | Meaning |
|---|---|---|
| 0x0000 + t | W | push one word into INQ[t]; dropped if full, which sets CTRL BADOP bit 14 (check status first or use IRQ) |
| 0x0000 + t | R | pop one word from OUTQ[t]; returns 0, does not pop and sets BADOP bit 14 if empty |
| 0x0100 + t | R | status: `[11:8]` OUTQ_COUNT, `[7:4]` INQ_COUNT, `[3]` OUTQ_EMPTY, `[2]` OUTQ_FULL, `[1]` INQ_EMPTY, `[0]` INQ_FULL; `[15:12]` read 0 |

The address never increments in this space: a multi-word transaction pushes or
pops consecutive words of the same FIFO, and a multi-word read of the status
word reads it again. Other addresses read 0 and ignore writes. A pop is split
into a peek when the word is loaded and the pop itself when the word's last bit
has gone out, so a word cut short by `CS_n` pops nothing (`docs/SEMANTICS.md`
6.7).

### SPACE 4: DEBUG

ADDR = {thread[9:8], reg[7:0]}. `r0..r7` are readable and writable only while
that thread is halted (the register-file ports are borrowed during the
thread's bubble slots); while it runs, reads return 0 and writes are dropped.
Every other debug register is a plain flop and is readable at any time, and
writable only while the thread is not running. Per-thread state changes only
at that thread's commit edge, so a read is always a consistent architectural
state. (`docs/SEMANTICS.md` section 7.)

| reg | Name |
|---|---|
| 0x00..0x07 | r0..r7 |
| 0x08 | PC |
| 0x09 | FLAGS: {T, C, Z} in bits 2:0 (bit 0 = Z), as in `docs/SEMANTICS.md` |
| 0x0A | TD (a host write here, or through the CSR window at 0x1A, never applies a staged pin write: `docs/SEMANTICS.md` 6.10, D-028) |
| 0x0B | NOW (read only) |
| 0x0C | SR (reads 0 until the bit engine is built) |
| 0x0D | CNT (same) |
| 0x0E | CRC (same) |
| 0x0F | RS0 |
| 0x10..0x1F | CSR 0x00..0x0F of that thread |
| 0x20 | STEPS: valid slots since reset (16-bit, wraps), done or stalled, for trace alignment |
| 0x21 | {4'b0, DEPTH[1:0], RS1[9:0]} |
| 0x22 | WAIT_ACTIVE in bit 0 |
| 0x23 | DT (the hidden target of `DLY`) |
| 0x24 | TICK_SEEN in bit 0 (M2) |
| 0x25 | staged pin write: {9'b0, LAT_VALID, LAT_VAL, LAT_PIN[4:0]}, i.e. bits 6:0 (M2, `docs/SEMANTICS.md` 6.10); writable while halted; LAT_PIN and LAT_VAL keep their values when LAT_VALID clears; a host TD write never applies it (D-028), the thread's own deadline writes and the exact tick do |
| 0x26 | {INQ_CNT, OUTQ_CNT} as {byte, byte} (M2), read-only: a written count would expose entries never pushed |
| 0x27 | bit-engine encoder state (M3 slice A, `docs/SEMANTICS.md` 6.9.1): {8'b0, FIRST, HALF, PEND, RVAL, RUN[2:0], LVL} in bits 7:0; writable while halted; reads 0 until slice A is built |
| 0x28 | data-memory access in progress (M3 slice B, 6.11): {11'b0, MEM_PEND, MEM_LD, MEM_RD[2:0]} in bits 4:0; writable while halted; reads 0 until slice B is built |

`CSRW PIN_OUT`, `CSRW PIN_OE` and the host's PIN_OUT/PIN_OE writes are raw
register writes and do not apply the open-drain rule; only pin writes (`SETP`,
`OUT`, the bit engine) do. `PC` is 10 bits whatever the memory size, so with a
memory smaller than 1024 words fetch addresses alias modulo `IMEM_WORDS`.

### SPACE 5: STEP

Write any value to ADDR = t: thread t executes exactly one instruction (one
slot) and halts again. If the instruction is a wait whose condition is false,
the step still consumes one slot and PC does not advance; STEPS increments.
This makes single-stepping observably identical to free-running execution one
slot at a time, which is what the co-simulation harness relies on.

## Worked examples

Load a 3-word program at address 0 and run thread 0:

```
CS low  90 00 00  12 34  56 78  9A BC   CS high     (write IMEM, addr 0, 3 words)
CS low  80 00 02  00 01              CS high     (write CTRL RUN = 0b0001)
```

Read OUTQ[1] twice:

```
CS low  30 00 01  xx  d1h d1l  d2h d2l  CS high    (read FIFO, thread 1, dummy, two words)
```

Single-step thread 2 and read its PC and r0:

```
CS low  D0 00 02  00 01   CS high                    (STEP t2)
CS low  40 02 08  xx  pch pcl   CS high              (DEBUG t2 reg 0x08)
CS low  40 02 00  xx  r0h r0l   CS high              (DEBUG t2 reg 0x00)
```

## Python API (`tools/loomhost`)

```python
from loomhost import Loom, PicoTransport, SimTransport

loom = Loom(PicoTransport("COM7"))          # or SimTransport(dut) inside cocotb
loom.load("firmware/uart_tx.bin")            # writes IMEM while halted, verifies by readback
loom.write_csr(thread=0, csr="TICK_INT", value=434)   # 115200 baud at 50 MHz
loom.run(threads=[0])
loom.push(0, [0x48, 0x69])                   # data for the thread
state = loom.dump(0)                         # dict of the DEBUG space for thread 0
loom.step(0); assert loom.dump(0) == model.step()
```

Transports: `SimTransport` (the host port of a cocotb run: a pin model of
`test/rtl_bench.py` that drives CS_n, SCK and MOSI and reads MISO and
`HOST_IRQ` on `uo_out[6]`, at the timing `ModelTransport` uses), `PicoTransport` (USB serial to a Raspberry Pi Pico running
`tools/loomhost/pico/` which converts a tiny line protocol to SPI),
`TTBoardTransport` (tt-micropython-firmware REPL over USB on the Tiny Tapeout
demo board; the firmware has no SPI driver for project pins, so the transport
uses `machine.SPI(0)` on the RP2040 board, where SPI0 lands on the host pins,
and falls back to bit-banging through the firmware's byte helpers elsewhere).
All three implement `transfer(tx_bytes) -> rx_bytes` with CS framed around the
call; everything above that line is shared.
