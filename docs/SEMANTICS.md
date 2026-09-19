# Loom reference semantics (cycle-exact)

Version 1.0-draft, 2026-09-17, Fable 5.1. This document is the contract that
both the RTL (`src/`) and the golden model (`tools/loomsim`) implement. They
are written from this text and from `isa/isa.yaml`, never from each other
(`docs/VERIFICATION.md`, METH-1). Where `docs/ARCHITECTURE.md` and this file
differ, this file wins and the difference is a bug in ARCHITECTURE.md.

Scope markers: **[M1]** must exist at milestone M1. **[M2]**, **[M3]** are
specified now so that encodings and state are stable, but are built later.
Until a feature is built its instructions execute as `NOP` and set `BADOP[t]`
(section 9), and `CAPS` reports the feature absent.

## 1. Clock, reset, cycle numbering

- One clock, `clk`. Every register updates on the rising edge. `rst_n` is
  synchronous and active low.
- **Edge 0** is the last rising edge at which `rst_n` is sampled low. **Cycle
  k** is the interval between edge k and edge k+1. During cycle 0 all state
  holds its reset value.
- `ph` is a free-running 2-bit phase counter, 0 during cycle 0, incrementing
  every cycle: `ph(k) = k mod 4`. It never stalls.

## 2. Slots and pipeline stages

Thread `t` (0..3) owns the slot that starts in every cycle `k` with
`k mod 4 == t`. A slot has four stages, one cycle each:

| Stage | Cycle | What happens |
|---|---|---|
| F | k | slot is **valid** iff `RUN[t] == 1` or `STEP_REQ[t] == 1`, both as visible in cycle k. If valid, `imem_addr = PC[t]`. A valid slot consumes `STEP_REQ[t]`. |
| D | k+1 | `IR = imem[PC[t]]` (synchronous read). Decode. Read registers. |
| X | k+2 | Evaluate the instruction using the state **visible in cycle k+2** (section 3). Decide `done` (instruction completes) or `stall`. |
| W | k+3 | Present the commit. All effects of the slot are registered at **edge k+4** and are visible from cycle k+4 on, which is the same thread's next F. |

We call `x = k+2` the slot's **X cycle**. Exactly one slot is in each stage in
every cycle, and the four slots in flight always belong to four different
threads. An invalid slot is a bubble: it changes nothing.

Consequences that both implementations must reproduce:

- Every valid slot either completes its instruction (`done`) or re-issues the
  same `PC` in the thread's next slot (`stall`). Nothing takes more than one
  slot except by stalling.
- A thread's own state is never stale: it is read in D/X of a slot and the
  previous slot of that thread committed at edge k (before F).
- **Shared state visibility rule.** An effect of a slot with X cycle `x` on
  shared state (`PIN_OUT`, `PIN_OE`, `OD_MASK`, global CSRs, pads) is visible to
  other slots from cycle `x+2`. Exception: effects on `SFLAGS` are forwarded
  from W to X and are visible from cycle `x+1`, which makes `WAITS` an atomic
  test-and-clear across threads. The model implements this rule with a list
  of pending commits tagged with their visibility cycle.
- Pad outputs (`uo_out`, `uio_out`, `uio_oe`) are registers; a pin write in a
  slot with X cycle `x` changes the pad at edge `x+2`.
- **Slot grid.** A wait completes in the first X cycle at or after its
  condition becomes true, and a thread has an X cycle only every 4 clocks. So
  firmware-driven pin edges land on the thread's 4-clock grid. A `WAITD`
  schedule has **no accumulated drift** (deadlines are absolute in ticks), and
  each edge is late by 0 to 3 clocks relative to its deadline. When `k * period`
  is a multiple of 4 clocks the lateness is constant and the edges are exactly
  periodic; otherwise they dither (at 434 clocks per tick, UART bit edges
  alternate 432 and 436 clocks while the 10-bit frame spacing is exactly 4340).
  Clock-exact edges for any period are the job of the bit engine in auto mode
  (M2/M3) and of deadline-latched pin writes (planned for M2, D-016).

## 3. What an instruction sees in its X cycle

| Input | Value in X cycle `x` |
|---|---|
| Own registers, flags, `PC`, `TD`, `DT`, return stack, `WAIT_ACTIVE`, `PREV_PINS`, `TICK_SEEN`, per-thread CSRs | as committed by the thread's previous valid slot (or by the host while halted) |
| `NOW[t]` | the value of the `NOW` register during cycle `x` (section 4) |
| `pin_in(i)` for index 0..12 | the pad value sampled at **edge x-1** (two-flop synchroniser: FF1 samples the pad at every edge, FF2 samples FF1; logic uses FF2) |
| `pin_in(i)` for index 16..21 | the `PIN_OUT` bit for that pin as visible in cycle `x` |
| `pin_in(i)` for any other index | 0 |
| `SFLAGS` | committed value, plus the effect of the slot currently in W (forwarding) |
| Global CSRs | committed value visible in cycle `x` |
| FIFO status **[M2]** | occupancy visible in cycle `x` |

BIDIR pins always read the pad through the synchroniser, even while driven.
In simulation the testbench must loop `uio_out` back to `uio_in` for bits with
`uio_oe == 1`, as the Tiny Tapeout pad does.

## 4. Time

Per thread: `ACC` (24 bits), `NOW` (16), `TD` (16), `DT` (16), `TICK_SEEN` (1).

```
period = max(TICK_INT, 1) * 256 + TICK_FRAC          # in 1/256 clock units
at every edge:
    if ACC + 256 >= period:  ACC <= ACC + 256 - period ; tick = 1
    else:                    ACC <= ACC + 256          ; tick = 0
    NOW <= NOW + tick                                  # wraps at 2^16
    if tick: TICK_SEEN <= 1                            # set wins over clear
```

- Reset: `ACC = 0`, `NOW = 0`, `TD = 0`, `DT = 0`, `TICK_INT = 1`,
  `TICK_FRAC = 0`, so one tick per clock.
- The tick generator runs whether or not the thread is running.
- Any write to `TICK_INT` or `TICK_FRAC`, by a committed `CSRW` or by a host
  debug-space write, also sets `ACC <= 0` at the same edge (the clear wins over
  the accumulate; `NOW` does not tick at that edge).
- `TICK_INT = 0` is stored and read back as 0; the divider treats it as 1.
- `reached(a, b)` is `((a - b) mod 2^16) < 2^15`.
- At the commit edge of every valid slot of the thread, `TICK_SEEN` keeps only
  what the slot did not see: `TICK_SEEN <= tick | (TICK_SEEN & ~seen)`, where
  `seen` is the value the slot read in its X cycle. A tick at the same edge
  sets it. (Until 2026-09-18 the rule cleared `TICK_SEEN` outright, which lost
  a tick landing at edge `x + 1`, between a slot's X cycle and its commit, so
  `WAITB 3` could miss ticks indefinitely; rtl-m2 question 4. RTL and golden
  model changed in the same commit.)

## 5. Per-thread architectural state and reset values

| State | Bits | Reset | Notes |
|---|---|---|---|
| `r0..r7` | 16 each | 0 | |
| `PC` | 10 | `RESET_PC[t]` | |
| `Z, C, T` | 1 each | 0 | `FLAGS` CSR = `{T, C, Z}` in bits 2:0 |
| `RS0, RS1, DEPTH` | 10, 10, 2 | 0 | return stack, `DEPTH` in 0..2 |
| `WAIT_ACTIVE` | 1 | 0 | a stalled instruction is in progress |
| `PREV_PINS` | 13 | 0 | `pin_in(0..12)` as seen in the thread's previous valid slot |
| `NOW, TD, DT, ACC, TICK_SEEN` | section 4 | 0 | |
| `TICK_INT, TICK_FRAC, OUTGRP, INGRP` | 16, 8, 10, 10 | 1, 0, 0, 0 | |
| `STEPS` | 16 | 0 | +1 at the commit of every valid slot, done or stalled |
| BE state (`SR, CNT, CRC, BE_*`, `CRC_*`) **[M2/M3]** | 16, 5, 16, ... | 0 | 6.9 |
| `INQ_CNT, OUTQ_CNT` **[M2]** | log2(`FIFO_DEPTH`)+1 each | 0 | 6.7; entries are not reset |
| `LAT_VALID, LAT_PIN, LAT_VAL` **[M2]** | 1, 5, 1 | 0 | 6.10 |

Global: `RUN[3:0] = 0`, `HALTED[3:0] = 0`, `STEP_REQ[3:0] = 0`, `SFLAGS = 0`,
`OD_MASK = 0`, `PIN_OUT = 0`, `PIN_OE = 0`. `BADOP` is a 16-bit register, reset
0: bits 3:0 are per thread, bit 14 is the host FIFO error (6.7, M2), bit 15 is
the host access error, the rest read 0; the host writes 1 to clear a bit. `RESET_PC[t] = t * (IMEM_WORDS / 4)`, so the
four threads never alias whatever the memory size (0, 64, 128, 192 for 256
words; `t * 0x100` for 1024). Instruction memory is **not** reset.

`CAPS` (read-only, 16 bits): `[2:0]` log2 of the FIFO depth, `[3]` FIFOs built,
`[4]` bit engine built (manual mode), `[5]` data memory built, `[6]` boot ROM
built, `[7]` deadline-latched `SETP` built (M2), `[8]` bit engine auto mode
built (M3), `[11:9]` zero, `[15:12]` log2 of `IMEM_WORDS`. The M1 build with
256 words reads 0x8000.

ISA note: ISA 0.4.0 (2026-09-18) carries the encoding side of the M2 text:
bit 0 of `SETP` is the `D` field (assembler token `D`), `SHO` and `SHI` list
`Z` as a flag they set, and the CRC presets are stored in canonical `n`-bit
form for the assembler to left-align. The M1 RTL decodes `SETP ... D` as an
ordinary `SETP` until the M2 RTL builds the latch.

`PREV_PINS` is updated at the commit of every valid slot with the `pin_in`
values that slot saw in X, whatever the instruction was.

## 6. Instruction semantics

Notation: `next = (PC + 1) mod 2^10`; `rel` is sign-extended and branches go to
`(next + rel) mod 2^10`. Unless stated, an instruction is `done` in one slot,
commits `PC <= next`, and leaves flags and `WAIT_ACTIVE` alone. `Z` is always
"the 16-bit result is zero" for instructions listed as setting `Z`.

### 6.1 ALU, immediates, unary **[M1]**

| Instr | Result | C |
|---|---|---|
| ADD / ADDI | `a + b` | carry out of bit 15 |
| SUB / SUBI / CMP / CMPI | `a - b` (CMP, CMPI write nothing) | 1 if `a >= b` unsigned |
| AND OR XOR / ANDI ORI XORI | bitwise | 0 |
| SHL / SHLI by `n = amount[3:0]` | `a << n` | bit `16 - n` of `a` if `n > 0`, else 0 |
| SHR / SHRI | `a >> n` logical | bit `n - 1` of `a` if `n > 0`, else 0 |
| ROR | rotate right by `n` | bit 15 of the result if `n > 0`, else 0 |
| MOV | `a` | unchanged (MOV sets no flags) |
| NOT | `~a` | 0 |
| NEG | `0 - a` | 1 if `a == 0` |
| TEST | nothing written; `Z = ((rd & ra) == 0)` | 0 |
| REV | bit-reverse 16 | 0 |
| PAR | `rd = ra`; | XOR of all 16 bits of `ra` |
| SWAP | `{a[7:0], a[15:8]}` | 0 |
| LDI | `imm8` zero-extended | flags unchanged |
| LDIH | `{imm8, rd[7:0]}` | flags unchanged |

For three-operand ALU `a = ra, b = rb`. For ALUI `a = rd, b = imm6`
zero-extended, and the shift amount is `imm6[3:0]`. For unary `a = ra`; CMP and
TEST use `rd` as the left operand and `ra` as the right one. All except MOV,
LDI, LDIH set `Z`.

### 6.2 Control **[M1]**

- `JMP abs`: `PC <= abs`.
- `CALL abs`: `RS1 <= RS0; RS0 <= next; DEPTH <= min(DEPTH + 1, 2); PC <= abs`.
- `RET`: if `DEPTH > 0`: `PC <= RS0; RS0 <= RS1; DEPTH <= DEPTH - 1`; else
  `PC <= next`.
- `HALT`: `RUN[t] <= 0; HALTED[t] <= 1; PC <= next`.
- `Bcc rel8`: branch if the condition holds, else `next`.
- `DJNZ rd, rel8`: `rd <= rd - 1`; branch if the **new** `rd != 0`. No flags.
- `JP pin, v, rel6`: branch if `pin_in(pin) == v`.

### 6.3 Pins **[M1]**

Writable indices are 0..7 (BIDIR) and 16..21 (OUT). Writes to any other index
are ignored. A "pin write of value `b` to index `i`" means:

- `i` in 16..21: `PIN_OUT[i - 8] <= b` (register view in ARCHITECTURE 3.1).
- `i` in 0..7 with `OD_MASK[i] == 0`: `PIN_OUT[i] <= b`; `PIN_OE` unchanged.
- `i` in 0..7 with `OD_MASK[i] == 1`: `PIN_OUT[i] <= 0; PIN_OE[i] <= ~b`.

All pin commits are bit-masked: a slot changes only the bits it writes.

- `SETP pin, v`: one pin write.
- `OEP pin, v`: `PIN_OE[pin] <= v` for `pin` in 0..7, else ignored.
- `OUT ra`: for `j` in `0 .. cnt-1`: pin write of `ra[j]` to index
  `(base + j) mod 32`, with `base = OUTGRP[4:0]`, `cnt = min(OUTGRP[9:5], 16)`.
- `IN rd`: `rd[j] = pin_in((base + j) mod 32)` for `j < cnt`, other bits 0,
  with `base = INGRP[4:0]`, `cnt = min(INGRP[9:5], 16)`. No flags.

### 6.4 Waits **[M1 except where noted]**

A wait-class instruction that does not complete commits `WAIT_ACTIVE <= 1`
and leaves `PC` unchanged; when it completes it commits `WAIT_ACTIVE <= 0` and
`PC <= next`. "First issue" means `WAIT_ACTIVE == 0` in X.

- `WAITD imm8`: on first issue `TD' = TD + imm8` (mod 2^16), otherwise
  `TD' = TD`. Commit `TD <= TD'`. `done` iff `reached(NOW, TD')`.
- `DLY imm8`: on first issue `DT' = NOW + imm8`, otherwise `DT' = DT`. Commit
  `DT <= DT'`. `done` iff `reached(NOW, DT')`. So `DLY 0` is a one-slot NOP.
- `SETD imm8`: `TD <= NOW + imm8`. Always one slot.
- `NOP`: nothing.
- Conditional waits. `cond` is:
  - `WAITP pin, v`: `pin_in(pin) == v`.
  - `WAITE pin, e`: with `p = PREV_PINS[pin]`, `c = pin_in(pin)`, for `pin` in
    0..12: rise `(!p & c)`, fall `(p & !c)`, any `(p != c)`; `e == 3` and
    `pin > 12` are never true.
  - `WAITS n`: `SFLAGS[n] == 1` (with forwarding). On `done` by condition,
    commit `SFLAGS[n] <= 0`.
  - `WAITB c` **[M2]**: 0 BE idle, 1 OUTQ not full, 2 INQ not empty,
    3 `TICK_SEEN == 1`. `WAITB` is built together with the FIFOs; condition 0
    additionally needs the bit engine and is `NOP` + `BADOP` without it, so for
    this one instruction `BADOP` depends on the operand.
  - Completion: if `cond`: `done`, and if the `T` bit of the instruction is set,
    commit `T <= 0`. Else if the `T` bit is set and `reached(NOW, TD)`: `done`,
    commit `T <= 1`. Else stall. Without the `T` bit the `T` flag is untouched.

Pulses shorter than one slot (4 clocks) can be missed by `WAITE`; this is a
documented property, not a bug.

Programming consequences worth knowing (all follow from the rules above):

- `TD` resets to 0 and `NOW` runs from reset, so the first `WAITD` of a program
  completes at once unless a `SETD` anchors the schedule first. Always `SETD`
  before the first deadline and after any unbounded wait.
- `PREV_PINS` resets to 0 and is refreshed by every valid slot, so a `WAITE`
  for a rising edge as the very first instruction of a thread fires
  immediately if the pin is already high. Execute any instruction first.

### 6.5 Shared flags **[M1]**

`SIG n`: `SFLAGS[n] <= 1`. `CLR n`: `SFLAGS[n] <= 0`. If the host writes
`SFLAGS` in the same cycle as a thread commit, the thread's effect wins on the
bits it touches.

### 6.6 CSR access **[M1 for 0x00-0x03, 0x09-0x0C, 0x10-0x15]**

`CSRR rd, n`: `rd <=` the CSR value visible in X, zero-extended; no flags.
`CSRW n, ra`: the CSR takes `ra` truncated to its width. Read-only CSRs ignore
writes; write-only and unimplemented CSRs read 0. CSRs that belong to a feature
that is not built (`SR`, `CNT`, `CRC`, `BE_*`, `CRC_*`) read 0 and ignore
writes **without** setting `BADOP`: only instructions set `BADOP`. `NOW` reads `NOW` in X.
`TID` reads `t`. `CSRW SFLAGS` ORs `ra[7:0]` into `SFLAGS`. `CSRW HOST_IRQ`
sets `SWIRQ[t]` (host-visible, host-cleared). `CSRW FLAGS` writes `{T, C, Z}`.
`CSRW TD` writes `TD`. `CSRW PIN_OUT/PIN_OE/OD_MASK` write the whole register.

### 6.7 FIFOs **[M2]**

Per thread `t`: `INQ[t]` (host to thread) and `OUTQ[t]` (thread to host), each
`FIFO_DEPTH` entries of 16 bits (a build parameter, a power of two from 2 to 8,
default 4, so the counts fit the 4-bit fields of the host status word)
with occupancy counts `INQ_CNT[t]` and `OUTQ_CNT[t]` in `0 .. FIFO_DEPTH`.
Counts reset to 0; entry contents are not reset. A push or pop takes effect at
an edge and is visible from the following cycle, like every other register.

- `PUSH ra`: `done` iff `OUTQ_CNT[t] < FIFO_DEPTH` as visible in X; the entry
  is appended at the commit edge. Otherwise stall.
- `POP rd`: `done` iff `INQ_CNT[t] > 0` as visible in X; `rd <=` the head
  entry and the entry is removed at the commit edge. Otherwise stall.
- Both use `WAIT_ACTIVE` exactly like the waits of 6.4 and change no flags.
  Between a thread's X cycle and its commit edge only the host can touch that
  thread's FIFOs, and the host can only pop OUTQ and push INQ, so the X-cycle
  decision can never be invalidated.
- Host push to `INQ[t]` (SPI FIFO space): accepted iff `INQ_CNT[t] <
  FIFO_DEPTH` as visible in the cycle before the commit edge; otherwise the
  word is dropped and `BADOP[14]` (host FIFO error) is set. Host pop from
  `OUTQ[t]`: the SPI port must present a word's first bit before it knows the
  host will clock the word out, so a pop is split in two. When the word is
  loaded into the shift register (at the end of the dummy byte or of the
  previous word) it **peeks**: the head if `OUTQ_CNT[t] > 0` then, else 0. The
  **pop** commits at the edge where the word's last bit has gone out, and only
  if the peek found an entry; an empty peek sets `BADOP[14]` at that edge. A
  word cut short by `CS_n` pops nothing. The word loaded while the previous
  word's pop is still in flight is the entry after the head, and needs
  `OUTQ_CNT[t] >= 2` as visible in the cycle the load is decided, a count that
  still includes the entry that pop is about to remove. Nothing is lost or
  read twice. A thread push or pop committing at the same edge is
  applied too: the new count is `count + pushes - pops`.
- `CTRL.RESET` of thread `t` also empties `INQ[t]` and `OUTQ[t]`.
- `WAITB c` conditions (6.4): 1 is `OUTQ_CNT[t] < FIFO_DEPTH`, 2 is
  `INQ_CNT[t] > 0`, 3 is `TICK_SEEN[t]`, 0 is "bit engine idle", which is
  always true until auto mode exists (M3).

### 6.8 Host interrupt **[M2]**

`HOST_IRQ` (pad `uo_out[6]`) is a register, updated at every edge to
`any(IRQ_STAT & IRQ_EN) | any(IRQ_STAT2 & IRQ_EN2) | any(SWIRQ)` computed from
the values visible in the cycle before the edge, where `IRQ_STAT =
{SFLAGS[7:0], INQ_NOT_FULL[3:0], OUTQ_NOT_EMPTY[3:0]}` and `IRQ_STAT2 =
{12'b0, HALTED[3:0]}` (register addresses in `docs/HOST_PROTOCOL.md`). It is
level-sensitive: the host clears the cause, not the pin.

### 6.9 Bit engine, manual mode **[M2]**

Per-thread state: `SR` (16), `CNT` (5), `CRC` (16) and the CSRs `BE_CFG`,
`BE_PINS`, `BE_RELOAD`, `CRC_POLY`, `CRC_INIT`, all reset to 0. At M2 only
three `BE_CFG` fields exist: `DIR` (bit 1: 0 LSB first, 1 MSB first), `INV`
(bit 7: invert the pin level) and `CRC_EN` (bit 9). Writes to the other
`BE_CFG` bits are ignored and they read 0 until M3 builds them, so firmware can
detect what the engine supports. `BE_PINS = {in[9:5], out[4:0]}` holds two pin
indices.

- `SHO`: `b = DIR ? SR[15] : SR[0]`. One pin write (6.3 rules, open drain
  included) of `b ^ INV` to pin index `BE_PINS.out`. `SR <= DIR ? {SR[14:0],
  1'b0} : {1'b0, SR[15:1]}`. `CNT <= (CNT == 0) ? 0 : CNT - 1`, and `Z = (new
  CNT == 0)`; `C` and `T` unchanged. If `CRC_EN`, the CRC is updated with `b`.
- `SHI`: `s = pin_in(BE_PINS.in) ^ INV`. `SR <= DIR ? {SR[14:0], s} : {s,
  SR[15:1]}`, so after `n` LSB-first bits the value sits in `SR[15:16-n]` and
  `STSR r` then `SHRI r, 16-n` right-aligns it. `CNT` and `Z` as for `SHO`. If
  `CRC_EN`, the CRC is updated with `s`.
- CRC update with bit `x`, serial and MSB-first on a left-aligned register:
  `fb = CRC[15] ^ x; CRC <= {CRC[14:0], 1'b0} ^ (fb ? CRC_POLY : 16'h0000)`. An
  `n`-bit CRC lives in `CRC[15:16-n]` with its polynomial and initial value
  left-aligned the same way (`poly << (16 - n)`); the assembler's `.crc`
  presets do that alignment. The bits enter the register in transmission order.
- `LDSR ra`: `SR <= ra`. `STSR rd`: `rd <= SR`. `CRCI`: `CRC <= CRC_INIT`.
  `STCRC rd`: `rd <= CRC`. None changes flags. `CSRR`/`CSRW` reach `SR`,
  `CNT`, `CRC` and the configuration CSRs as well.
- With `CNT` as the loop counter a transmit loop needs no general register:
  `load: LDSR r0; CSRW CNT, r1; bit: SHO; WAITD 1; BNZ bit` is three slots per
  bit.

Auto mode, NRZI and Manchester coding, bit stuffing, the RX sample phase and
the stuffing-violation `T` flag are **[M3]** and get their cycle-exact text
here before M3 work starts. `SHO`/`SHI` executed while the bit engine is not
built are `NOP` + `BADOP`, as section 9 says.

### 6.10 Deadline-latched pin write **[M2]** (D-016)

`SETP pin, v, D` (the `D` form, encoding bit 0 set) does not write the pin.
It stages the write in the thread's latch `LAT = {LAT_VALID, LAT_PIN[4:0],
LAT_VAL}`: at the commit edge `LAT_VALID <= 1, LAT_PIN <= pin, LAT_VAL <= v`,
replacing any write already staged. A staged write is applied, and
`LAT_VALID` cleared, at the first later edge `e` at which either

1. `NOW` ticks at `e` to exactly the value of `TD` (TD not written at `e`), or
2. `TD` is written at `e` (by a `WAITD` first issue, `SETD`, `CSRW TD` or the
   host) and `reached(NOW', TD')` holds for the values `NOW'` and `TD'` visible
   after `e`.

In words: the staged write lands on the thread's next deadline, whichever
instruction set it. So `SETP TX, v, D` followed by `WAITD k` changes the pad
on the exact edge at which that deadline is reached, for any tick period,
integer or fractional;
case 2 covers a thread that was already late, which then writes at its `WAITD`
commit edge exactly as an ordinary `SETP` would. The staged write follows the
pin-write rules of 6.3 (open drain included). At an edge where a staged write
and a slot's ordinary pin writes both touch a pin, the ordinary write wins.
`CTRL.RESET` of the thread clears `LAT_VALID`. The latch is readable through
the debug space (`docs/HOST_PROTOCOL.md`, address 0x25).

### 6.11 Data memory **[optional, not planned before M3]**

`LD`, `ST`: as in `isa.yaml`; one slot. Until built they are `NOP` + `BADOP`.

## 7. Run control and the host **[M1 subset]**

- The host changes `RUN`, `STEP_REQ`, `RESET`, `RESET_PC`, `SFLAGS`, pin
  registers, and (while the target thread is halted) debug state. A host write
  commits at a clock edge like any other register and is visible from the next
  cycle. If a host write and a thread commit hit the same register bits at the
  same edge, the thread wins.
- `CTRL.RUN` write: `RUN <= value`; bits going 0 to 1 clear `HALTED[t]`.
- Definitions. Thread `t` is **halted** (for debug access) iff `RUN[t] == 0`,
  `STEP_REQ[t] == 0` and no valid slot of thread `t` is in F, D, X or W. **No
  step in flight** (for IMEM access) means `RUN == 0`, `STEP_REQ == 0` and no
  valid slot anywhere in the pipeline. `HALTED[t]` is only the sticky record
  that a `HALT` instruction ran; it is not the access condition, so registers
  can be preloaded before a thread's first run.
- `CTRL.RESET` bit `t`: `PC <= RESET_PC[t]`, flags `<= 0`, `DEPTH <= 0` (the
  contents of `RS0`/`RS1` are left alone), `WAIT_ACTIVE <= 0`, and `TD <=` the
  `NOW` value visible in the cycle in which the host write commits. Registers
  and CSRs are untouched. The host must only reset a halted thread; resetting a
  running thread is undefined.
- A host `STEP` that commits on the same edge at which the thread's F stage
  consumes an earlier `STEP_REQ` is lost (thread wins). The SPI port needs far
  more than 4 clocks per command, so this cannot happen through the pins.
- `STEP` write for thread `t`: `STEP_REQ[t] <= 1`; ignored if `RUN[t] == 1`. The
  next slot of thread `t` is valid and clears `STEP_REQ[t]` in its F cycle. A
  stepped wait that stalls leaves `WAIT_ACTIVE = 1`, exactly as in free
  running, so stepping `n` times is observably identical to running `n` slots.
- Debug writes to `PC` also clear `WAIT_ACTIVE`.
- Instruction memory is single-port. Host IMEM reads and writes are valid only
  while `RUN == 0` and no step is in flight; otherwise writes are dropped, reads
  return 0, and `BADOP[15]` (host access error) is set.
- `r0..r7` of thread `t` are readable and writable by the host only while
  thread `t` is halted (the register file ports are borrowed during that
  thread's bubble slots). While it runs, reads return 0 and writes are dropped.
  All other debug registers are readable at any time.

M1 builds: CTRL (`ID, VERSION, RUN, HALTED, RESET, RESET_PC, BADOP, CAPS,
SFLAGS, SFLAGS_CLR, OD_MASK, PIN_OUT, PIN_OE, PIN_IN`), IMEM, DEBUG, STEP.
FIFO and DMEM spaces read 0 and ignore writes until M2.

## 8. Retire record (for co-simulation)

`loom_core` exposes, for simulation and debug only, a record that is valid
during the W cycle of every valid slot:

| Signal | Meaning |
|---|---|
| `tr_valid` | a valid slot is in W |
| `tr_thread[1:0]`, `tr_pc[9:0]`, `tr_ir[15:0]` | which slot |
| `tr_done` | 1 if the instruction completes, 0 if it stalls |
| `tr_we`, `tr_rd[2:0]`, `tr_val[15:0]` | register write, if any |
| `tr_flags[2:0]` | `{T, C, Z}` **after** this slot |
| `tr_next_pc[9:0]` | `PC` after this slot |

The golden model produces the same record per slot, plus the X cycle number.
The harness compares them slot by slot and reports the first difference.

## 9. Reserved and unbuilt instructions

Any word that matches no instruction in `isa.yaml`, and any instruction whose
feature is not built, executes as `NOP` (one slot, `PC <= next`) and commits
`BADOP[t] <= 1`. `BADOP` is sticky until the host clears it.

## 10. What the model must be given

The model is deterministic given: the instruction memory image, the host
actions with the cycle at which each commits (observed from the RTL in
co-simulation, chosen freely in stand-alone use), and the pad input values at
every edge. It must never read RTL state to decide an outcome.
