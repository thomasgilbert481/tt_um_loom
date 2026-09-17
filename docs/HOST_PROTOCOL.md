# Loom host protocol (SPI)

Version 0.1, 2026-09-15. Implemented by `loom_spi_host.v` (bit layer) and
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
word (the falling SCK edge of its last bit, as seen in the core clock domain).

## Address spaces

### SPACE 0: CTRL

| ADDR | Name | R/W | Contents |
|---|---|---|---|
| 0x0000 | ID | R | 0x4C4D ("LM") |
| 0x0001 | VERSION | R | {major[7:0], minor[7:0]} |
| 0x0002 | RUN | RW | bit t = thread t running. Writing 1 starts at the current PC; writing 0 halts after the current instruction retires |
| 0x0003 | HALTED | R | bit t = thread t halted by HALT (cleared by writing RUN bit) |
| 0x0004 | RESET | W | bit t = reset thread t: PC := RESET_PC[t], flags := 0, TD := NOW, stack cleared; registers untouched |
| 0x0008..0x000B | RESET_PC[0..3] | RW | 10-bit reset vectors, default t * 0x100 |
| 0x0010 | IRQ_EN0 | RW | mask over IRQ_STAT0 (M2) |
| 0x0011 | IRQ_STAT0 | R | {SFLAGS[7:0], INQ_NOT_FULL[3:0], OUTQ_NOT_EMPTY[3:0]} (M2) |
| 0x0012 | IRQ_STAT1 | R, W1C | {8'b0, SWIRQ[3:0], HALTED[3:0]}; write 1 to a SWIRQ bit to clear it (M2) |
| 0x001B | IRQ_EN1 | RW | mask over IRQ_STAT1; HOST_IRQ = any enabled bit of either word (M2) |
| 0x0013 | SFLAGS | RW | shared flags; write sets the bits written as 1 |
| 0x0014 | SFLAGS_CLR | W | write clears the bits written as 1 |
| 0x0015 | OD_MASK | RW | open-drain mode per BIDIR pin |
| 0x0016 | PIN_OUT | RW | raw output register (host may drive pins while threads are halted) |
| 0x0017 | PIN_OE | RW | raw OE register |
| 0x0018 | PIN_IN | R | synchronised inputs |
| 0x0019 | CAPS | R | build capabilities: {IMEM_WORDS[11:0]/16, DMEM_PRESENT, FIFO_DEPTH_LOG2[1:0], BOOTROM} |
| 0x001A | BADOP | RW | bit t = thread t executed a reserved opcode; write 1 to clear |

### SPACE 1: IMEM

ADDR = instruction address (0..IMEM_WORDS-1). The memory is single-port and
the core fetches from it every cycle, so host reads **and** writes are valid
only while `RUN == 0` and no single-step is in flight. Otherwise a write is
dropped, a read returns 0, and CTRL BADOP bit 15 (host access error) is set.
(`docs/SEMANTICS.md` section 7.)

### SPACE 2: DMEM

Only if built (CAPS.DMEM_PRESENT). Same rules as IMEM but writes are allowed
while running (data memory is dual-ported or arbitrated; threads win).

### SPACE 3: FIFO

| ADDR | Access | Meaning |
|---|---|---|
| 0x0000 + t | W | push one word into INQ[t]; dropped if full (check status first or use IRQ) |
| 0x0000 + t | R | pop one word from OUTQ[t]; returns 0 and does not pop if empty |
| 0x0100 + t | R | status: {OUTQ_COUNT[3:0], INQ_COUNT[3:0], OUTQ_EMPTY, OUTQ_FULL, INQ_EMPTY, INQ_FULL} |

Multi-word transactions push or pop consecutive words into the same FIFO (the
address does not increment across thread boundaries in this space; the low two
bits stay fixed).

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
| 0x09 | FLAGS {Z, C, T} |
| 0x0A | TD |
| 0x0B | NOW (read only) |
| 0x0C | SR |
| 0x0D | CNT |
| 0x0E | CRC |
| 0x0F | RS {RS1[15:8]... see note} |
| 0x10..0x1F | CSR 0x00..0x0F of that thread |
| 0x20 | STEPS: number of instructions retired since reset (16-bit, wraps), for trace alignment |

Note: RS is 20 bits; 0x0F returns RS0, 0x21 returns RS1 and the stack depth.

### SPACE 5: STEP

Write any value to ADDR = t: thread t executes exactly one instruction (one
slot) and halts again. If the instruction is a wait whose condition is false,
the step still consumes one slot and PC does not advance; STEPS increments.
This makes single-stepping observably identical to free-running execution one
slot at a time, which is what the co-simulation harness relies on.

## Worked examples

Load a 3-word program at address 0 and run thread 0:

```
CS low  81 00 00  12 34  56 78  9A BC   CS high     (write IMEM, addr 0, 3 words)
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

Transports: `SimTransport` (cocotb coroutine driving the SPI pins with a
cocotb clock), `PicoTransport` (USB serial to a Raspberry Pi Pico running
`tools/loomhost/pico/` which converts a tiny line protocol to SPI),
`TTBoardTransport` (tt-micropython-firmware REPL over USB on the Tiny Tapeout
demo board; the firmware has no SPI driver for project pins, so the transport
uses `machine.SPI(0)` on the RP2040 board, where SPI0 lands on the host pins,
and falls back to bit-banging through the firmware's byte helpers elsewhere).
All three implement `transfer(tx_bytes) -> rx_bytes` with CS framed around the
call; everything above that line is shared.
