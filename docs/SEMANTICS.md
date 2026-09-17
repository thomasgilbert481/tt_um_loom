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
- A committed `CSRW TICK_INT` or `CSRW TICK_FRAC` also sets `ACC <= 0` at the
  same edge (the clear wins over the accumulate; `NOW` does not tick at that
  edge).
- `reached(a, b)` is `((a - b) mod 2^16) < 2^15`.
- `TICK_SEEN` is cleared at the commit edge of every valid slot of the thread
  (unless a tick sets it at the same edge).

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
| BE state (`SR, CNT, CRC, BE_*`, `CRC_*`) **[M2/M3]** | | 0 | |

Global: `RUN[3:0] = 0`, `HALTED[3:0] = 0`, `STEP_REQ[3:0] = 0`, `BADOP[3:0] = 0`,
`RESET_PC[t] = t * 0x100` (masked to the memory size), `SFLAGS = 0`,
`OD_MASK = 0`, `PIN_OUT = 0`, `PIN_OE = 0`. Instruction memory is **not** reset.

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
  with `base/cnt` from `INGRP`. No flags.

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
    3 `TICK_SEEN == 1`.
  - Completion: if `cond`: `done`, and if the `T` bit of the instruction is set,
    commit `T <= 0`. Else if the `T` bit is set and `reached(NOW, TD)`: `done`,
    commit `T <= 1`. Else stall. Without the `T` bit the `T` flag is untouched.

Pulses shorter than one slot (4 clocks) can be missed by `WAITE`; this is a
documented property, not a bug.

### 6.5 Shared flags **[M1]**

`SIG n`: `SFLAGS[n] <= 1`. `CLR n`: `SFLAGS[n] <= 0`. If the host writes
`SFLAGS` in the same cycle as a thread commit, the thread's effect wins on the
bits it touches.

### 6.6 CSR access **[M1 for 0x00-0x03, 0x09-0x0C, 0x10-0x15]**

`CSRR rd, n`: `rd <=` the CSR value visible in X, zero-extended; no flags.
`CSRW n, ra`: the CSR takes `ra` truncated to its width. Read-only CSRs ignore
writes; write-only and unimplemented CSRs read 0. `NOW` reads `NOW` in X.
`TID` reads `t`. `CSRW SFLAGS` ORs `ra[7:0]` into `SFLAGS`. `CSRW HOST_IRQ`
sets `SWIRQ[t]` (host-visible, host-cleared). `CSRW FLAGS` writes `{T, C, Z}`.
`CSRW TD` writes `TD`. `CSRW PIN_OUT/PIN_OE/OD_MASK` write the whole register.

### 6.7 FIFOs **[M2]**, bit engine **[M2/M3]**, data memory **[M2, optional]**

- `PUSH ra`: if OUTQ not full in X: push at commit, `done`; else stall.
- `POP rd`: if INQ not empty in X: `rd <=` head, pop at commit, `done`; else
  stall. Both use `WAIT_ACTIVE` like waits.
- `SHO, SHI, LDSR, STSR, CRCI, STCRC` and the BE CSRs follow ARCHITECTURE
  section 8; their cycle-exact text is added to this file before M2 work
  starts.
- `LD, ST`: as in `isa.yaml`; one slot.

## 7. Run control and the host **[M1 subset]**

- The host changes `RUN`, `STEP_REQ`, `RESET`, `RESET_PC`, `SFLAGS`, pin
  registers, and (while the target thread is halted) debug state. A host write
  commits at a clock edge like any other register and is visible from the next
  cycle. If a host write and a thread commit hit the same register bits at the
  same edge, the thread wins.
- `CTRL.RUN` write: `RUN <= value`; bits going 0 to 1 clear `HALTED[t]`.
- `CTRL.RESET` bit `t`: `PC <= RESET_PC[t]`, flags `<= 0`, `TD <= NOW`,
  `DEPTH <= 0`, `WAIT_ACTIVE <= 0`. Registers and CSRs are untouched. The host
  must only reset a halted thread; resetting a running thread is undefined.
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
