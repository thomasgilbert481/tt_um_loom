# Loom: a barrel-threaded protocol emulator ASIC

Status: v0.1 architecture, 2026-09-14. Direction set by Fable 5.1; implementation
by Opus 5 with Thomas Gilbert as owner. Everything here is a decision unless it is
marked OPEN or VERIFY. Change a decision only by adding an entry to
`docs/DECISIONS.md` first.

Target: Jane Street protocol emulator ASIC competition, Tiny Tapeout on IHP
`ihp-sg13cmos5l`, **6x4 tiles** (1289.28 x 710.64 um, the largest size the
cmos5l flow accepts today; the blog's 8x4 is "in progress" and not yet in
`tile_sizes.yaml`), deadline 2027-01-18. See `docs/PLAN.md` and
`docs/tt_cmos5l_facts.md` (verified 2026-09-15).

## 1. What it is, in one paragraph

Loom is a tiny I/O processor whose only job is to read pins, write pins, and hit
timing. Four hardware threads share one 4-stage pipeline in strict round-robin
(a "barrel" pipeline, like XMOS xCORE or the CDC 6600 peripheral processors,
shrunk to fit a square millimetre). Every instruction retires in exactly one
thread slot, there are no caches, no stalls except explicit waits, and no branch
penalty, so the timing of any program can be read off the listing. Each thread
has a local timebase with a fractional prescaler, a deadline register that makes
periodic bit timing jitter-free, and a small "bit engine" (shift register,
programmable CRC, NRZ/NRZI/Manchester encoder, bit-stuffing) that does the
per-bit work that firmware cannot do fast enough. A SPI slave port lets a host
load programs, exchange data through per-thread FIFOs, and halt, single-step and
inspect any thread. The same host protocol drives the RTL simulation, the FPGA
prototype and the silicon.

## 2. Why this and not a PIO clone or a PRU clone

| Property | RP2040 PIO | TI PRU | Loom |
|---|---|---|---|
| Sequencers | 4 SMs per block, 32 shared instructions | 2 full 32-bit RISC cores | 4 threads on one pipeline, 512 to 1024 shared instructions |
| Timing model | per-instruction delay + side-set; paths balanced by hand | count cycles by hand | deadline register (`WAITD`) with fractional ticks; timing independent of the path taken |
| Data-dependent behaviour | weak (few branches, no ALU) | strong | 16-bit ALU, flags, DJNZ, CALL/RET, pin-conditional branches |
| Bit-level helpers | shifters, autopush/pull | none | shifter + CRC + NRZI/Manchester + stuffing + auto mode |
| Waits with timeout | no | no | every wait can abort at the deadline and set a T flag |
| Emulating a *device* (I2C EEPROM, SPI flash, PS/2 keyboard) | awkward | fine | first-class: data memory + FIFOs + pattern waits |
| Debug | none on-chip | JTAG | halt / step / read-write every register over the host port |

What we deliberately leave out: interrupts, memory-mapped peripherals,
multiply, a wide datapath. Everything that would make timing data-dependent
was removed.

## 3. Top level

```
              TT pads                       Loom (tt_um_loom)
  ui_in[5]  HOST_SCK  --+
  ui_in[6]  HOST_MOSI --+--> SPI slave --> command FSM --> +------------+---------+
  ui_in[4]  HOST_CS_n --+        ^                         |  control   |  debug  |
  uo_out[7] HOST_MISO <----------+                         |  regs      |  port   |
  uo_out[6] HOST_IRQ  <----- irq <-------------------------+-----+------+---------+
                                                                 |
  ui_in[3:0],[7] IN0..4 --> sync --> pin unit <--> barrel core (4 threads) <--> imem
  uo_out[5:0] OUT0..5   <----------- pin unit      |  |  |  |                  (512x16)
  uio[7:0]    BIDIR0..7 <--oe/out/in-- pin unit    BE BE BE BE  (one bit engine per thread)
                                                   ^^ ^^ ^^ ^^
                                             INQ/OUTQ FIFOs per thread <--> host
```

Single clock domain (`clk`, 50 MHz nominal). Host SCK is sampled, not used as a
clock; that keeps the whole chip one clock domain, which is what makes formal and
timing closure simple. `rst_n` is synchronous, active low, as in the TT template.

### 3.1 Pin map (default build)

| TT pad | Loom name | Direction | Notes |
|---|---|---|---|
| ui_in[4] | HOST_CS_n | in | frames every transaction |
| ui_in[5] | HOST_SCK | in | SPI mode 0, SCK <= clk/8 |
| ui_in[6] | HOST_MOSI | in | |
| ui_in[3:0] | IN0..IN3 | in | pin index 8..11 |
| ui_in[7] | IN4 | in | pin index 12 |
| uo_out[7] | HOST_MISO | out | |
| uo_out[6] | HOST_IRQ | out | any enabled shared flag or halted thread |
| uo_out[5:0] | OUT0..OUT5 | out | pin index 16..21 |
| uio[7:0] | BIDIR0..BIDIR7 | bidir | pin index 0..7, per-pin OE and open-drain mode |

The host pins are fixed by the top-level wrapper, not by firmware. They were
chosen so that on the Tiny Tapeout demo board (RP2040) the hardware SPI0
function set lands on them: GP17 CSn = ui_in[4], GP18 SCK = ui_in[5], GP19 TX =
ui_in[6], GP16 RX = uo_out[7] (verified in `docs/tt_cmos5l_facts.md` section 5),
so `machine.SPI(0)` works without PIO. The v3 demo board (RP2350B) has a
different GPIO map; there the host transport bit-bangs or uses PIO, which works
on any pins. Everything else is a 5-bit "pin index" that instructions and CSRs
use uniformly:

```
index  0..7   BIDIR0..7  read, write, OE          (uio)
index  8..11  IN0..3     read only                 (ui_in[3:0])
index 12      IN4        read only                 (ui_in[7])
index 13..15  reserved   reads 0
index 16..21  OUT0..5    write; read returns the driven value  (uo_out[5:0])
index 22..31  reserved
```

Register views: `PIN_IN[15:0] = {3'b0, ui_in[7], ui_in[3:0], uio_in[7:0]}`,
`PIN_OUT[15:0] = {2'b0, uo_out[5:0], uio_out[7:0]}`, `PIN_OE[7:0] = uio_oe`.

## 4. Execution model

### 4.1 Threads, slots, pipeline

- 4 hardware threads, T0..T3. One 4-stage pipeline: **F** (imem address),
  **D** (imem data returns, decode, register read), **X** (ALU, pin sample, timer
  compare, wait evaluation), **W** (register write, pin write, PC update).
- Strict round robin: thread t is in stage F when `cycle mod 4 == t`. Each
  thread issues one instruction every 4 clocks. A **slot** = 4 clocks = 80 ns at
  50 MHz. Slot rate per thread: 12.5 M instructions/s.
- Because a thread's next instruction is fetched only after its previous one
  wrote back, there are no data hazards, no forwarding, no branch prediction and
  no branch penalty. Every instruction, including a taken branch, costs exactly
  one slot.
- Wait-class instructions and blocking `PUSH`/`POP` re-issue the same PC every
  slot until their condition holds. Only that thread stalls. The condition is
  sampled once per slot in stage X, so reaction latency to an external event is
  between 1 and 2 slots, and it is the same every time for a given program.
- No thread can affect another thread's timing except through explicit shared
  flags. This is the property the formal testbench proves (see
  `docs/VERIFICATION.md`, properties SCHED-1..3).

### 4.2 Reset and run control

After reset all threads are halted with PC = `RESET_PC[t]` (host writable,
default `t * IMEM_WORDS / 4`, D-017). The host loads instruction memory while
threads are halted, then sets bits in `RUN`. `HALT` clears the thread's RUN bit
and raises HOST_IRQ if enabled. The host can also halt, resume and single-step
(execute exactly one instruction) any thread.

OPEN (area-gated, M3): a 16-entry boot ROM selected by IN4 (`ui_in[7]`) high
at reset that transmits a fixed string on OUT0 at 115200 baud, so first
silicon can be checked with nothing but a scope. Costs about 200 cells.
(`ui_in[4]` was the v0.1 choice; it is now the host chip select.)

## 5. Programmer's model (per thread unless stated)

| Name | Width | Meaning |
|---|---|---|
| r0..r7 | 16 | general registers; r0 is not special |
| PC | 10 | instruction address (up to 1024 x 16-bit) |
| Z, C, T | 1 each | zero, carry / not-borrow / shift-out, timeout |
| RS0, RS1 | 10 each | 2-entry hardware return stack; CALL pushes, RET pops, overflow drops the oldest |
| NOW | 16 | local time, increments once per tick |
| TD | 16 | deadline; `WAITD` adds to it and waits for it |
| SR | 16 | bit-engine shift register |
| CNT | 5 | bit-engine remaining bit count |
| CRC | 16 | bit-engine CRC accumulator |
| CSRs | see 5.1 | configuration |
| INQ | 4 x 16 | host -> thread FIFO (`POP`) |
| OUTQ | 4 x 16 | thread -> host FIFO (`PUSH`) |

Global: `SFLAGS[7:0]` shared flags, `OD_MASK[7:0]` open-drain mode per BIDIR pin,
`PIN_OUT`, `PIN_OE`, `PIN_IN`, `RUN[3:0]`, `RESET_PC[t]`, `IRQ_EN`, `IRQ_STAT`,
`ID` (read-only: 0x4C4D "LM" plus a version byte).

### 5.1 CSR map (accessed with `CSRR rd, csr` / `CSRW csr, rs`)

| # | Name | R/W | Bits | Meaning |
|---|---|---|---|---|
| 0x00 | TICK_INT | RW | 16 | integer part of tick divider (>= 1) |
| 0x01 | TICK_FRAC | RW | 8 | fractional part; tick period = (TICK_INT + TICK_FRAC/256) clocks |
| 0x02 | OUTGRP | RW | base[4:0], cnt[9:5] | `OUT` writes cnt pins starting at base (cnt 1..16) |
| 0x03 | INGRP | RW | base[4:0], cnt[9:5] | `IN` reads cnt pins starting at base |
| 0x04 | BE_CFG | RW | see 8.1 | bit-engine mode, direction, encoding, stuffing, sample phase |
| 0x05 | BE_PINS | RW | out[4:0], in[9:5] | bit-engine output pin index and input pin index |
| 0x06 | BE_RELOAD | RW | 5 | CNT reload value in auto mode (1..16) |
| 0x07 | CRC_POLY | RW | 16 | polynomial, convention in 8.2 |
| 0x08 | CRC_INIT | RW | 16 | value loaded by `CRCI` |
| 0x09 | NOW | R | 16 | local time |
| 0x0A | TD | RW | 16 | deadline |
| 0x0B | FLAGS | RW | Z,C,T | |
| 0x0C | TID | R | 2 | thread id, so one program can serve several threads |
| 0x0D | SR | RW | 16 | shift register (also `LDSR`/`STSR`) |
| 0x0E | CNT | RW | 5 | remaining bits |
| 0x0F | CRC | RW | 16 | accumulator |
| 0x10 | OD_MASK | RW | 8 | global open-drain mode per BIDIR pin |
| 0x11 | PIN_OUT | RW | 16 | global raw output register |
| 0x12 | PIN_OE | RW | 8 | global raw OE register |
| 0x13 | PIN_IN | R | 16 | synchronised inputs |
| 0x14 | SFLAGS | RW | 8 | shared flags (write sets bits; use `CLR` to clear one) |
| 0x15 | HOST_IRQ | W | 1 | pulse HOST_IRQ |

Reserved CSR numbers read 0 and ignore writes.

## 6. Time

- Each thread has a tick generator: a 16.8 fixed-point accumulator divider off
  `clk`. `TICK_INT=1, TICK_FRAC=0` gives one tick per clock. Fractional dividers
  let 50 MHz produce a 1.5 Mbit USB bit clock (33.33 clocks) with one clock of
  jitter, which USB tolerates. For 10 Mbit Manchester run the chip at 40 or
  60 MHz instead; see section 13.
- `NOW` increments on every tick and wraps at 2^16.
- `WAITD k`: `TD <= TD + k`, then stall until `NOW` has reached `TD`. "Reached" is
  the wrap-safe test `(NOW - TD) mod 2^16 < 2^15`. Because the deadline advances
  from the previous deadline and not from "now", a schedule built from `WAITD`
  never drifts, regardless of how many instructions ran in between (as long as
  fewer than k ticks' worth) or which branch was taken. This is what makes a
  UART written in five lines have zero accumulated jitter.
- Resolution of a firmware-driven edge is one slot: a thread acts only every 4
  clocks, so each edge lands 0 to 3 clocks after its deadline. It is exactly
  periodic when `k * period` is a multiple of 4 clocks, and dithers by one slot
  otherwise (432/436 clocks at a 434-clock tick, 0.5 percent of a 115200 baud
  bit). Two mechanisms give clock-exact edges for any period: the bit engine in
  auto mode, and deadline-latched pin writes, where `SETP pin, v, D` stages the
  write and the hardware applies it on the exact clock the thread's next
  deadline is reached, normally the one the following `WAITD` sets (M2,
  decision D-016, cycle-exact rules in `docs/SEMANTICS.md` 6.10).
- `SETD k`: `TD <= NOW + k`. Use it to re-anchor the schedule to an external
  event (for example right after `WAITE` sees the start-bit edge; then
  `WAITD` of 1.5 bit times lands the first sample mid-bit).
- `DLY k`: stall k ticks relative to now; TD unchanged. For one-off gaps.
- Timeout: `WAITP`, `WAITE`, `WAITS`, `WAITB` take a T bit. With T=1 the wait also
  ends when `NOW` reaches `TD`; the T flag is set if that is why it ended and
  cleared otherwise. "Wait for SCL to rise, but give up at the deadline" is one
  instruction, and `BT`/`BNT` branch on the outcome.
- The assembler analyses every path between deadline instructions. A schedule
  that cannot be met (the worst-case slot count between an anchor and the next
  `WAITD k` exceeds the budget) is an **error**-level diagnostic; a path with
  an unbounded wait or a loop and no re-anchoring `SETD` is a **warning**. The
  budget from a `SETD m` anchor to `WAITD k` is `(m + k)` ticks, from a
  `WAITD` anchor it is `k` ticks, and `WAITD 0` is transparent (it neither
  starts nor ends an interval). Limits of the analysis are documented in
  `tools/loomasm/README.md`.

## 7. Pins

- Inputs go through 2 flip-flop synchronisers, then a per-thread previous-value
  register for edge detection. Input latency to the X stage is 2 to 3 clocks
  plus slot phase.
- Pin writes (`SETP`, `OUT`, `OEP`, bit-engine output) land on the output
  register at stage W, so from the pad's point of view every thread writes at a
  fixed phase of its slot.
- Open-drain mode: if `OD_MASK[i]` is set for BIDIR i, writing 1 releases the pin
  (OE=0) and writing 0 drives low (OE=1, OUT=0). I2C firmware then looks exactly
  like push-pull firmware. Reads always return the pad value.
- Ownership is by convention, not enforced: two threads writing the same pin get
  last-writer-wins at clock granularity. Firmware assigns pins to threads; the
  assembler can check that no two threads' `OUTGRP`/`SETP` sets overlap when
  given the whole program (M3 nice-to-have).
- `JP pin, v, rel` branches on a single pin. `WAITP`/`WAITE` wait on a level or
  an edge (rise, fall, any). Pattern waits on a pin *group* (mask + value, for
  I2C START = SDA falling while SCL high) are done with `WAITE` on the edge pin
  followed by `JP` on the other pin; a dedicated group-match wait is OPEN for M3
  if the area allows.

## 8. Bit engine (one per thread)

The bit engine owns SR, CNT, CRC and an encoder. Firmware uses it two ways.

**Manual mode.** `SHO` moves one bit from SR to the BE output pin (through the
encoder), updates CRC, decrements CNT. `SHI` samples the BE input pin (through
the decoder), shifts it into SR, updates CRC, decrements CNT. One slot each. With
`WAITD 1` between bits a thread reaches 6 Mbit/s; USB low-speed at 1.5 Mbit/s
leaves 6 slots per bit for framing logic.

**Auto mode.** The engine runs itself on ticks: TX shifts one bit per tick and,
when CNT hits zero, reloads SR from INQ if `AUTOPULL` is set; RX samples one bit
per tick at the configured phase and pushes SR to OUTQ when CNT hits zero if
`AUTOPUSH` is set. The thread supervises with `WAITB` and handles framing. This
is the path for 10 Mbit Manchester, where there is only about one slot per bit.

### 8.1 BE_CFG bits

| Bits | Field | Values |
|---|---|---|
| 0 | MODE | 0 manual, 1 auto |
| 1 | DIR | 0 LSB first, 1 MSB first |
| 2 | RXTX | 0 transmit (drive out pin), 1 receive (sample in pin) |
| 4:3 | ENC | 0 NRZ, 1 NRZI (USB: 0 = toggle), 2 Manchester (IEEE 802.3: 0 = high-to-low), 3 reserved |
| 6:5 | STUFF | 0 none, 1 USB (insert 0 after six 1s), 2 CAN (insert complement after five equal bits), 3 reserved |
| 7 | INV | invert the pin sense |
| 8 | AUTOPULL / AUTOPUSH | reload from INQ (TX) or push to OUTQ (RX) at CNT==0 |
| 9 | CRC_EN | update CRC on data bits (stuffed bits never touch CRC) |
| 12:10 | PHASE | RX sample position within the tick period, in eighths |

Stuffed bits consume a tick but not a data bit and never enter SR or CRC. In
manual mode a pending stuff bit makes the next `SHO` emit the stuff bit instead
of a data bit and leave CNT unchanged; firmware never needs to know. RX
destuffing mirrors this. A stuffing violation (seven 1s in USB RX) sets the T
flag so firmware can detect EOP or a corrupt packet.

### 8.2 CRC

Serial LFSR, one data bit per update, `CRC_POLY` programmable, `CRCI` loads
`CRC_INIT`. Convention: an n-bit CRC is left-aligned in the 16-bit accumulator,
and so is its polynomial. The assembler ships presets: USB CRC5 (0x05, init
0x1F), USB CRC16 (0x8005, init 0xFFFF), CAN CRC15 (0x4599), CRC-8 SMBus (0x07).
Ethernet FCS (CRC-32) is not covered by the 16-bit unit; it is OPEN as a single
shared 32-bit unit at M4 if area allows, otherwise the host computes it.

## 9. FIFOs, shared flags, host interrupt

- INQ and OUTQ are 4 x 16 per thread (depth is a parameter; 8 is the fallback if
  USB packet handling needs it and area allows). `POP` stalls on empty, `PUSH`
  stalls on full; `WAITB` lets firmware poll instead.
- `SFLAGS[7:0]`: `SIG n` sets, `CLR n` clears, `WAITS n` waits for set and then
  clears atomically. The host can read and write them too. They are the only way
  threads talk to each other; there is no shared register file.
- HOST_IRQ = `|(IRQ_STAT & IRQ_EN)` where IRQ_STAT bits are: thread halted (4),
  OUTQ not empty (4), INQ not full (4), SFLAGS (8). Level-sensitive; the host
  clears the cause.

## 10. Host interface

SPI slave, mode 0, MSB first, one transaction per CS_n low period. Every
transaction is `CMD` (1 byte), `ADDR` (2 bytes), then data words as 2 bytes each,
address auto-incrementing. Reads insert one dummy byte after ADDR. Full
definition, register map and worked examples: `docs/HOST_PROTOCOL.md`.

| CMD[7] | CMD[6:4] space | Contents |
|---|---|---|
| 1 = write, 0 = read | 0 CTRL | RUN, RESET_PC, IRQ, SFLAGS, ID, OD_MASK, PIN_* |
| | 1 IMEM | instruction memory (host writes only while the target thread is halted) |
| | 2 DMEM | data memory, if present |
| | 3 FIFO | ADDR[1:0] = thread; write pushes INQ, read pops OUTQ; ADDR[8] = peek status |
| | 4 DEBUG | ADDR[9:8] = thread, ADDR[7:0] = register: r0..r7, PC, FLAGS, TD, NOW, SR, CNT, CRC, CSRs |
| | 5 STEP | write 1 = single-step the thread in ADDR[1:0] |

The debug space is readable at any time and writable while the thread is
halted. This is what lets `tools/loomhost` compare a thread's full architectural
state against the Python golden model after every step, on RTL, on the FPGA and
on silicon, with one script.

## 11. Instruction set

16-bit instructions. Fields: `rd/ra/rb` = 3-bit register numbers, `imm6/imm8`
unsigned, `rel6/rel8` signed PC-relative in instructions, `abs10` absolute,
`pin` = 5-bit pin index, `T` = timeout enable. **`isa/isa.yaml` is the single
source of truth**; the tables below are the v0.1 proposal that seeds it. The
codegen (`tools/gen`) produces the Verilog opcode header, the assembler tables,
the golden model dispatch table and `docs/ISA.md` from that one file.

### 11.1 Summary

| Class | Mnemonics | Slots | Flags |
|---|---|---|---|
| ALU rd, ra, rb | ADD SUB AND OR XOR SHL SHR ROR | 1 | Z, C |
| ALUI rd, imm6 | ADDI SUBI ANDI ORI XORI SHLI SHRI CMPI | 1 | Z, C |
| Load imm | LDI rd, imm8 (zero-extend); LDIH rd, imm8 (high byte) | 1 | none |
| Unary | MOV NOT NEG CMP TEST REV PAR SWAP | 1 | Z, C (PAR sets C to even parity) |
| Control | JMP abs10; CALL abs10; RET; HALT | 1 | none |
| Cond. branch | BZ BNZ BC BNC BT BNT rel8 | 1 | none |
| Loop | DJNZ rd, rel8 | 1 | none |
| Pin branch | JP pin, v, rel6 | 1 | none |
| Pins | SETP pin, v; OEP pin, v; OUT ra; IN rd | 1 | none |
| Wait | WAITD imm8; WAITP pin, v [,T]; WAITE pin, e [,T]; WAITS n [,T]; WAITB c [,T]; DLY imm8; SETD imm8; NOP | 1 or more | T |
| Data | PUSH ra; POP rd | 1 or more | none |
| Bit engine | SHO; SHI; LDSR ra; STSR rd; CRCI; STCRC rd | 1 | T (stuff violation) |
| CSR | CSRR rd, csr; CSRW csr, rs | 1 | none |
| Flags | SIG n; CLR n | 1 | none |
| Memory (optional) | LD rd, [ra+imm5]; ST [ra+imm5], rd | 1 | none |

### 11.2 Proposed encoding (top 4 bits select the class)

```
0000 fff ddd aaa bbb   ALU    rd = ra op rb              f: ADD SUB AND OR XOR SHL SHR ROR (amount = rb[3:0])
0001 fff ddd iiiiii    ALUI   rd = rd op imm6            f: ADDI SUBI ANDI ORI XORI SHLI SHRI CMPI
0010 0 ddd iiiiiiii    LDI    rd = imm8
0010 1 ddd iiiiiiii    LDIH   rd[15:8] = imm8
0011 fff ddd aaa ---   UN     f: MOV NOT NEG CMP TEST REV PAR SWAP   (CMP/TEST: flags only)
0100 00 aaaaaaaaaa     JMP abs10
0100 01 aaaaaaaaaa     CALL abs10
0100 10 ----------     RET
0100 11 ----------     HALT
0101 ccc 0 rrrrrrrr    Bcc rel8            c: Z NZ C NC T NT (6,7 reserved)
0110 0 ddd rrrrrrrr    DJNZ rd, rel8
0111 v ppppp rrrrrr    JP pin, v, rel6
1000 00 v ppppp ----   SETP pin, v
1000 01 v ppppp ----   OEP  pin, v          (pin must be 0..7)
1000 10 --- aaa ----   OUT ra
1000 11 --- ddd ----   IN  rd
1001 000 - iiiiiiii    WAITD imm8
1001 001 v ppppp T -   WAITP pin, v [,T]
1001 010 ee ppppp T    WAITE pin, e [,T]    e: 0 rise 1 fall 2 any
1001 011 nnn T ----    WAITS n [,T]
1001 100 cc T -----    WAITB c [,T]         c: 0 BE idle, 1 OUTQ not full, 2 INQ not empty, 3 tick
1001 101 - iiiiiiii    DLY imm8
1001 110 - iiiiiiii    SETD imm8
1001 111 ---------     NOP
1010 00 000 aaa ----   PUSH ra
1010 00 001 ddd ----   POP  rd
1010 01 000 --- ----   SHO
1010 01 001 --- ----   SHI
1010 10 000 aaa ----   LDSR ra
1010 10 001 ddd ----   STSR rd
1010 11 000 --- ----   CRCI
1010 11 001 ddd ----   STCRC rd
1011 w ccccc ddd ---   CSRW csr, r (w=1) / CSRR r, csr (w=0)
1100 s nnn ---------   SIG n (s=1) / CLR n (s=0)
1101 w ddd aaa iiiii   LD rd,[ra+imm5] (w=0) / ST [ra+imm5], rd (w=1)   (only if DMEM built)
1110, 1111             reserved: decode as NOP and set a "bad opcode" debug bit
```

Flag rules: ADD sets C = carry out; SUB/CMP set C = 1 when there is no borrow
(ra >= rb unsigned); shifts set C to the last bit shifted out; logic ops clear
C. Z is set whenever the 16-bit result is zero. Only instructions marked as
setting flags touch them. `T` is set or cleared only by timed waits and by
stuffing violations.

### 11.3 Assembler pseudo-ops and directives

Not listed here, to avoid a third copy: pseudo-ops (`MOV16`, `BRA`, `INC`,
`DEC`) and the symbolic operand names (`RISE/FALL/ANY`, `BE_IDLE/OUTQ_NF/
INQ_NE/TICK`) live in `isa/isa.yaml` and appear in `docs/ISA.md`; directives
(`.thread`, `.org`, `.equ`, `.pins`, `.word`, `.csr`, `.tick`,
`.deadline_check`) and the full language are defined in
`tools/loomasm/README.md`. `.crc <preset>` and tick-unit literals arrive with
the bit engine at M2.

## 12. Instruction and data memory: implementation options

Instruction memory is the single biggest area item. Decision gate at PLAN M2
(2026-10-26). Options, in order of preference:

1. **IHP SRAM macro.** The cmos5l PDK re-exports the SG13G2 macros
   (`sg13cmos5l_sram` is a symlink to `sg13g2_sram`), and two of them are
   exactly 16 bits wide, so one word is one instruction and no lane muxing is
   needed: `RM_IHPSG13_1P_512x16_c2_bm_bist` (236.80 x 191.34 um, 45.3K um²,
   about 5 percent of the 6x4 block) and `RM_IHPSG13_1P_1024x16_c2_bm_bist`
   (236.80 x 336.46 um, 79.7K um², about 9 percent). Single port: host writes
   happen only while all threads are halted, which is already the rule. The
   1024x8 sibling has worked in SG13G2 silicon (Tiny Tapeout `tt_um_urish_sram_test`,
   2x2) and its LEF uses only Metal1..Metal4, which fits the cmos5l stack; but
   **no SRAM macro has been taped out on cmos5l yet**, and integration needs the
   `MACROS` block, a custom `pdn_cfg.tcl` (macro power pins on Metal4), abstract
   cells for LVS and Magic DRC disabled for the macro, as in that project's
   `src/config.json`. Details and sources: `docs/tt_cmos5l_facts.md` section 3.
   Data memory would be a 256x8 or 512x8 macro, or flops.
2. **Flip-flop array, 256 x 16** (built at M1). Measured: about half of the
   710K um² of placed cells and 68 percent of the flops, which puts the block
   at 79 percent utilisation with no M2 features yet (`docs/AREA.md`). Each bit
   costs a `dfrbpq_1` (49.0 um²) plus a write-enable `mux2_1` (18.1 um²).
3. **Latch array, 256 x 16**, the way Ibex builds its latch register file:
   one integrated clock gate (`lgcp_1`, 27.2 um²) per word opens that word's
   latches while a flop-held write word is stable, and each bit is a
   `dlhq_1` latch (30.8 um²) with no per-bit mux. Storage drops from about
   275K um² to about 135K um², roughly 15 percent of the core. Costs: latch
   timing in STA, a hold-safe write scheme, and Verilator/TT-lint waivers for
   intentional latches inside `loom_imem.v` only. Fallback if the macro fails
   the precheck.
4. **Flop array 128 x 16** as the emergency fallback. Enough for UART, SPI, I2C
   and one stretch protocol at a time.

The macro is the preferred path because it turns the biggest area item into
the smallest; the risk is entirely in flow and foundry acceptance, which is why
the M2 gate runs a real hardening of both options and Thomas asks Jane Street
and Tiny Tapeout whether macros are acceptable on the March 2027 shuttle.
Update 2026-09-15: a Tiny Tapeout Discord reply reports a community member
has a macro working on cmos5l by matching the PDN stripe pitch to the macro's
power pins, and expects TT to accept DRC-clean macros; macros currently fail
the TT precheck, so the gate requires a precheck-clean run (D-014,
`docs/tt_cmos5l_facts.md` section 9).

`loom_imem.v` wraps all four behind one interface (`addr`, `rdata` one cycle
later, host `we/waddr/wdata`), selected by a parameter, so the choice never
leaks into the core.

## 13. Clock and area budget

Clock: 50 MHz nominal (`CLOCK_PERIOD` 20 ns in `src/config.json`). Close timing at
16.7 ns (60 MHz) in STA so that 50 MHz has margin and 60 MHz is usable for
10 Mbit Manchester (6 clocks per bit, 3 per half bit). USB low-speed prefers
48 MHz (32 clocks per bit); the fractional tick divider makes 50 MHz acceptable.

Area, first-order estimate in standard cells (VERIFY at M1 synthesis). The 6x4
block is 1289.28 x 710.64 um = 0.916 mm² of die area; at the flow's 60 percent
placement density that is about 550K um² of cells. The competition brief
budgets roughly 1K cells per tile, so about 24K cells for 24 tiles before
routing and clock-tree overhead. cmos5l gives user projects one fewer routing
layer than SG13G2 (`RT_MAX_LAYER = Metal4`; TopMetal1 belongs to the TT mux),
so keep density at or below 60 percent and avoid wide buses. Plan: no more than
16K logic cells plus the macro, or no more than 20K cells all-flops with the
trims listed under section 12. Table entries are for the flop option:

| Block | Flops | Cells (est.) |
|---|---|---|
| Register files 4 x 8 x 16 + 2 read ports | 512 | 2000 |
| Pipeline, PCs, return stacks, flags | ~250 | 800 |
| Per-thread NOW, TD, tick divider (16+16+24) x 4 | 224 | 700 |
| ALU, shifter, REV/PAR/SWAP | 0 | 700 |
| Decode and control | ~40 | 500 |
| Bit engines x 4 (SR, CNT, CRC, poly, cfg, encoder) | ~280 | 1400 |
| FIFOs 4 x 2 x 4 x 16 | 512 | 1200 |
| Pin unit (sync, edge, OE, OD, groups) | ~120 | 600 |
| Host SPI, command FSM, address regs | ~80 | 600 |
| Debug mux and control regs | ~60 | 600 |
| Subtotal without instruction memory | ~2100 | ~9100 |
| Instruction memory, flop option 256 x 16 | 4096 | ~9500 |
| Total, flop option | ~6200 | ~18600 |

If a macro is available the logic total is about 9K cells plus the macro, which
leaves room for 8-deep FIFOs, data memory, the boot ROM and the CRC-32 unit.

## 14. Module hierarchy (for the implementer)

```
tt_um_loom.v            TT wrapper: pad mapping, host pin split, unused-signal hygiene
  loom_top.v            instantiates everything below; all parameters live here
    loom_sync.v         2-FF synchronisers for ui_in / uio_in and the SPI pins
    loom_spi_host.v     SPI slave bit layer -> byte stream with CS framing
    loom_host_ctl.v     command FSM, address spaces, control regs, IRQ, debug mux
    loom_imem.v         parameterised memory wrapper (MACRO | FLOPS | SMALL)
    loom_core.v         barrel pipeline: scheduler, fetch, decode, X, W
      loom_regfile.v    4 x 8 x 16, 2 read ports, 1 write port (thread-indexed)
      loom_alu.v        16-bit ALU + shifter + REV/PAR/SWAP
      loom_timer.v      per-thread tick divider, NOW, TD, reached logic
      loom_be.v         bit engine (x4), encoder/decoder, stuffing, CRC
      loom_fifo.v       parameterised sync FIFO (used 8 times)
    loom_pins.v         pin index space, groups, OD mode, OE, edge registers
    loom_bootrom.v      optional (M3)
```

Interfaces are recorded in `docs/INTERFACES.md` as they are implemented (Opus
writes this file as part of M1; every port list there must match the RTL, and a
CI lint compares them).

Coding rules: Verilog-2005 subset that Icarus 14, Verilator 5, Yosys 0.63 and the
TT LibreLane flow all accept without special flags. `default_nettype none`. No
latches unless in `loom_imem.v` behind the LATCH option. One always block per
register group, synchronous reset, no initial blocks in synthesizable code.
Parameters, never macros, for sizes. Every module has a header comment stating
its timing contract (which cycle outputs change relative to inputs).

## 15. Open questions (owner: Thomas, unless noted)

1. RESOLVED 2026-09-15: SRAM macros are reachable in `ihp-sg13cmos5l` (symlink
   to the SG13G2 set) and the TT flow passes `MACROS` through to LibreLane, but
   none has been taped out on cmos5l. Decision still at M2, from a real
   hardening of both options plus Jane Street's and Tiny Tapeout's answer on
   macro acceptance.
7. WATCH: `8x4` is not a valid `tiles` value in the cmos5l flow (largest is
   `6x4`; `8x2`/`6x4` were never used on the first cmos5l shuttle). If Tiny
   Tapeout adds `8x4` before M4, switching is a one-line `info.yaml` change and
   a re-budget; do not design for it.
2. OPEN: FIFO depth 4 vs 8. Decide after M1 synthesis numbers.
3. OPEN: data memory (LD/ST) size and implementation. Needed for device
   emulation demos (I2C EEPROM, SPI flash). Decide at M2 with the memory choice.
4. OPEN: boot ROM demo. Include if the design is under 20K cells at M3.
5. OPEN: group-match wait and CRC-32. M3/M4, area permitting.
6. Name: "Loom" (threads, weaving, and a Tiny Tapeout shuttle). Change before the
   first public push if you want something else; it is only in `info.yaml`,
   the module prefix and the docs.
