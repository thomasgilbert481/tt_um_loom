# Formal properties (VERIFICATION.md L4)

SymbiYosys proofs of the L4 property list. Run them all with

```bash
bash scripts/formal.sh          # from the repo root, in the WSL dev shell
bash scripts/formal.sh quick    # skips bounded runs an unbounded proof subsumes
bash scripts/formal.sh fifo     # one group
```

`scripts/formal.sh` exits non-zero if any group ends in a state other than the
one recorded here, including a property that is recorded as failing and starts
passing.

## How the properties reach the design

Properties live in `formal/props/*.v` and never in `src/`. Each file ends in a
SystemVerilog `bind` statement that instantiates the property module inside the
design module, so the property module's ports are elaborated in the design's
scope and can read internal registers without a port being added to the RTL.
Synthesis and the Tiny Tapeout flow never read these files.

**Yosys's built-in Verilog frontend silently ignores `bind`.** It reports no
error: the property module is simply dropped as unused, every assertion
disappears and the proof passes vacuously. The first version of the FIFO proof
did exactly that. Every `.sby` here therefore reads the design through the
bundled `slang` plugin, which implements `bind` properly:

```
[script]
plugin -i slang
read_slang --top <module> <design files> <property file>
prep -top <module>
```

Two habits guard against vacuity, and both are part of the harness:

- every group has a `cover` task whose statements exercise the states the
  assertions talk about. A cover task that fails means an assumption is too
  strong or the properties were optimised away;
- assumptions are written only where a sentence of `docs/SEMANTICS.md` or
  `docs/HOST_PROTOCOL.md` puts the duty on the other side of the interface,
  and each one quotes that sentence. Where an assumption stands in for a
  caller's obligation it is discharged by a property on the caller (FIFO-1
  below).

`prove` means the property holds for all reachable states, with no depth
bound: `abc pdr` (property-directed reachability) or k-induction in
`smtbmc`. `bmc` means it holds up to the depth given and no further.

## Results

Recorded 2026-09-19 (TIMER-2 added 2026-09-22 with D-028, ISO-1 the same day),
OSS CAD Suite: SBY 0.63, Yosys 0.63+161, yices 2.6, btormc (boolector), abc
pdr. Times are wall clock on the laptop. `scripts/formal.sh quick` is 21 sby
tasks in about 5 minutes, the slowest single task being `iso:cover` at about 30 s —
inside the `formal-quick` budget of the CI matrix in `docs/VERIFICATION.md`.
The runs `quick` leaves out are the `formal-full` nightly's work and take
about 55 minutes between them, nearly all of it `iso:prove` (35) and
`iso:bmc` (14).

| ID | Property | File / task | Mode | Engine | Depth | Result | Time |
|---|---|---|---|---|---|---|---|
| ISA-1A | every 16-bit pattern matches at most one instruction of `isa.yaml` | `isa.sby:prove` | prove | smtbmc yices | comb. | **proved** | 1 s |
| ISA-1B | `loom_decode`'s `is_*` strobes are exactly those matches | `isa.sby:prove` | prove | smtbmc yices | comb. | **proved** | 1 s |
| ISA-1C | `is_reserved` is exactly "no instruction matches" | `isa.sby:prove` | prove | smtbmc yices | comb. | **proved** | 1 s |
| — | 66 cover points: every instruction encoding, and a reserved word | `isa.sby:cover` | cover | smtbmc yices | 1 | 66/66 | 4 s |
| ISA-2A | a reserved word retires as a NOP: `done`, no register write, `PC <= next` | `isacore.sby:prove` | prove | abc pdr | unbounded | **proved** | 10 s |
| ISA-2C | it sets `BADOP[t]` at its commit edge, and `BADOP` is sticky until the host clears it | `isacore.sby:prove` | prove | abc pdr | unbounded | **proved** | 10 s |
| ISA-2 | same, bounded cross-check | `isacore.sby:bmc` | bmc | btor btormc | 24 | pass | 4 s |
| — | 4 cover points | `isacore.sby:cover` | cover | smtbmc yices | 20 | 4/4 | 8 s |
| FIFO-1 | occupancy stays in `0 .. DEPTH`; `wp == rp + count` | `fifo.sby:prove` | prove | smtbmc yices (k-ind.) | 12 | **proved** | 3 s |
| FIFO-2 | data out equals data in, in order (two-token) | `fifo.sby:prove` | prove | smtbmc yices (k-ind.) | 12 | **proved** | 3 s |
| FIFO-3 | count law `count + push - pop`, `clr` empties, reset to 0 | `fifo.sby:prove` | prove | smtbmc yices (k-ind.) | 12 | **proved** | 3 s |
| FIFO-1B | the caller never pushes into a full FIFO or pops an empty one | `sched.sby:prove` | prove | abc pdr | unbounded | **proved** | 26 s |
| — | 7 cover points | `fifo.sby:cover` | cover | smtbmc yices | 20 | 7/7 | 1 s |
| SCHED-1 | exactly one thread in each pipeline stage, the four distinct, every commit ring one-hot on the W thread | `sched.sby:prove` | prove | abc pdr | unbounded | **proved** | 26 s |
| SCHED-2 | thread `t`'s regs, PC, flags, TD, SR, CNT, CRC and, from slice A (2026-09-22), its encoder state of 6.9.1 change only in its own W stage (or by a host write) | `sched.sby:prove` | prove | abc pdr | unbounded | **proved** (21 s with the encoder state) | 26 s |
| SCHED-3 | a thread the host has neither run nor stepped for four cycles changes nothing | `sched.sby:prove` | prove | abc pdr | unbounded | **proved** | 26 s |
| SCHED-4 | a `STEP_REQ` consumed by thread `t`'s F stage is clear in the next cycle whatever the host wrote at that edge (SEMANTICS 7, "the thread wins"; BUGS 6, added 2026-09-22) | `sched.sby:prove` | prove | abc pdr | unbounded | **proved** | 26 s |
| SCHED-1..4, FIFO-1B | bounded cross-check | `sched.sby:bmc` | bmc | btor btormc | 24 | pass | 39 s |
| — | 9 cover points (the five before, plus the SCHED-4 coincidence itself, per thread) | `sched.sby:cover` | cover | smtbmc yices | 20 | 9/9 | 7 s |
| PIN-2 | pin commits are bit-masked; core beats staged beats host | `pins.sby:prove` | prove | smtbmc yices (k-ind.) | 12 | **proved** | 7 s |
| **PIN-1** | an open-drain BIDIR pin never drives high | `pins.sby:prove` | prove | smtbmc yices (k-ind.) | 12 | **proved** (was FAILS, step 2, before D-023: finding F-2) | 3 s |
| PIN-1CORE | same, firmware only (no host pin write in the trace) | `pins.sby:prove` | prove | smtbmc yices (k-ind.) | 12 | **proved** (was F-2) | 3 s |
| PIN-1R | the pads never carry a one on an open-drain pin: `OD_MASK & uio_out == 0` | `pins.sby:prove` | prove | smtbmc yices (k-ind.) | 12 | **proved** (was F-2, as the register claim `OD_MASK & PIN_OUT == 0`) | 3 s |
| — | 6 cover points | `pins.sby:cover` | cover | smtbmc yices | 16 | 6/6 | 1 s |
| **TIMER-1** | `reached` is monotone: once true it stays true until TD changes | `timer.sby:xfail` | bmc | smtbmc yices | 16 | **FAILS, step 4, all four threads** — finding F-1 | 1 s |
| TIMER-1B | the only way `reached` falls with TD unchanged is `NOW - TD` going `0x7FFF -> 0x8000` | `timer.sby:prove` | prove | smtbmc yices (k-ind.) | 12 | **proved** | 2 s |
| TIMER-1C | `NOW` only ever advances by one | `timer.sby:prove` | prove | smtbmc yices (k-ind.) | 12 | **proved** | 2 s |
| TIMER-2 | a thread's `lat_fire` needs a tick at the closing edge or a TD write by its own slot; a host TD write alone never raises it (D-028, added 2026-09-22) | `timer.sby:prove` | prove | smtbmc yices (k-ind.) | 12 | **proved** | 1 s |
| — | 7 cover points (the four above, rule 1 fired, rule 2 fired, a host TD write on a quiet edge with no fire) | `timer.sby:cover` | cover | smtbmc yices | 16 | 7/7 | 1 s |
| SPI-1A | `byte_done` only pulses for a byte the pads completed and that is unreported | `spi.sby:prove` | prove | abc pdr | unbounded | **proved** | 10 s |
| SPI-1B | `rx_byte` is that byte, MSB first | `spi.sby:prove` | prove | abc pdr | unbounded | **proved** | 10 s |
| SPI-1C | a completed byte is reported before the next SCK rise and before `CS_n` is released; `CS_n` high voids a partial byte | `spi.sby:prove` | prove | abc pdr | unbounded | **proved** | 10 s |
| SPI-1 | bounded cross-check (one full byte plus margin) | `spi.sby:bmc` | bmc | smtbmc yices | 70 | pass | 19 s |
| SPI-1 | same, without "the chip is not reset mid-transaction" | `spi.sby:xfail` | bmc | smtbmc yices | 80 | **FAILS, steps 63-76** — finding F-3 | 16 s |
| — | 5 cover points, including two bytes in one transaction | `spi.sby:cover` | cover | smtbmc yices | 140 | 5/5 | 65 s |
| **ISO-1** | two traces that agree on thread 0's inputs keep thread 0's whole architectural state identical, whatever the other three threads run (two-copy miter, no host debug traffic) | `iso.sby:prove` | prove | abc pdr | unbounded | **proved** | 2 118 s |
| ISO-1 | same, bounded cross-check | `iso.sby:bmc` | bmc | btor btormc | 24 | pass | 819 s |
| ISO-1 | same, with the host's whole debug traffic identical in the two traces | `iso.sby:dbg` | bmc | btor btormc | 16 | **FAILS, step 5** — finding F-4 | 23 s |
| ISO-1 | same, with only the debug transactions addressed to thread 0 identical (the L4 wording taken literally) | `iso.sby:xfail` | bmc | btor btormc | 16 | **FAILS, step 5** — finding F-4 | 20 s |
| — | 9 cover points: the other thread's PCs parted, it decodes a different word in the two copies, thread 0 advances / stalls / blocks on SFLAGS / fires a WAITS / commits a pin write / applies a deadline-latched write with the other thread diverged, both pipes full | `iso.sby:cover` | cover | smtbmc yices | 20 | 9/9 | 29 s |
| WAIT-1A | a `WAITD` is `done` exactly when `reached(NOW, TD')`; a re-issue leaves `TD` where the first issue put it; a wait-class slot that stalls leaves `PC` alone | `wait.sby:prove` | prove | abc pdr (k-induction at 12 also closes it) | unbounded | **proved** | 11 s |
| **WAIT-1** | a timed wait retires within `TD - NOW + 2` of the thread's slots, all four threads, at the reset tick period (`TICK_INT = 1`, `TICK_FRAC = 0`), for deadlines up to 8 ticks away | `wait.sby:bmc` | bmc | btor btormc | 40 | pass | 163 s |
| WAIT-1 | the same bound with the tick period free | `wait.sby:xfail` | bmc | btor btormc | 40 | **FAILS, step 17, thread 1** — finding F-5 | 23 s |
| — | 15 cover points (per thread: a wait pending, a wait that re-issues, a wait that retires; plus a `WAITD` that stalls, one that fires, a stall with `WAIT_ACTIVE` already set) | `wait.sby:cover` | cover | smtbmc yices | 32 | 15/15 | 32 s |

`spi.sby:prove` is proved by `abc pdr`; k-induction at depth 20 does not close
it (the byte counter needs a deeper invariant than 20 cycles), which is why the
group lists two engines and takes the first conclusive answer.

**ISO-1 is proved, unbounded**, by `abc pdr` in 35 minutes: 89 assertions
over a design of two `loom_core`s and two `loom_pins`. k-induction at depth
12 does not close it (`smtbmc boolector` passes the base case and fails the
induction step on six assertions, the register-file write port among them),
which is why the group lists two engines and takes the first conclusive
answer, as `spi` does. `iso:bmc` at depth 24 is the bounded cross-check and
is the only ISO-1 run cheap enough to keep in a nightly if `prove` ever
stops closing.

**WAIT-1 is bounded too**, and its L4 wording is wrong in the unit: see
finding F-5. `wait.sby:bmc` holds to depth 40 with the tick period at its
reset value; `wait.sby:xfail` reproduces the counterexample with the period
free. Nothing in the L4 list is now unattempted.

## Findings

Five findings. Four are properties of the L4 list that do not hold as worded;
the other is a condition SPI-1 needs that the L4 wording does not mention.
None has been weakened: each is kept in the harness exactly as
`docs/VERIFICATION.md` words it, under a task that carries `expect fail`, so
the counterexample is reproduced on every run and the harness turns red if the
behaviour ever changes silently.

### F-1 `TIMER-1` is false: `reached` is not monotone across the half-range wrap

`timer.sby:xfail`, `props/loom_timer_props.v` line 55, all four threads, BMC
step 4.

`SEMANTICS` 4 defines `reached(a, b) = ((a - b) mod 2^16) < 2^15`. With `TD`
fixed, `NOW - TD` grows by one per tick, so `reached` is true exactly while
`NOW - TD` is in `0 .. 0x7FFF` and goes false on the tick that takes it to
`0x8000`. "Once true it stays true until TD changes" can therefore only hold
for a window of 2^15 ticks.

Counterexample (thread 2; the other three are identical):

| cycle | what happens | `NOW` | `TD` | `NOW-TD` | `reached` |
|---|---|---|---|---|---|
| edge 0 | `rst_n` low: reset, `TICK_INT = 1` so one tick per clock | 0 | 0 | 0 | true |
| 1 | a slot commits a TD write for thread 2 (`cm_sel=4'b0100`, `cm_td_we=1`, `cm_td=0x8002`): a `SETD`, a `CSRW TD` or a first-issue `WAITD`. A tick fires at the same edge | 0 -> 1 | 0 -> 0x8002 | | |
| 2 | nothing writes thread 2's TD; a tick fires at edge 2 | 1 -> 2 | 0x8002 | 0x7FFF | **true** |
| 3 | — | 2 | 0x8002 | 0x8000 | **false** |

`TD` did not change between cycle 2 and cycle 3.

TIMER-1B (proved) says this is the *only* way it can happen, and TIMER-1C
(proved) says `NOW` never jumps, so the window is exactly 2^15 ticks wide.

The RTL computes `reached` exactly as SEMANTICS 4 defines it, so this reads as
a wording bug in `docs/VERIFICATION.md` rather than an RTL bug. The property
that is true is TIMER-1B, or TIMER-1 with an explicit "within 2^15 ticks of
the TD write" hypothesis. The firmware consequence is worth a line in
SEMANTICS 6.4: a deadline more than 2^15 ticks in the past reads as *not*
reached again, so `WAITD` against a very stale `TD` blocks instead of
completing at once.

### F-2 `PIN-1` was false: enabling open drain did not stop a pin already driving high

**Fixed 2026-09-19 in D-023 (BUGS 5), by option 1 below.** The three
assertions are now part of `pins:prove` and hold; what follows is the
record of the finding, because a property that was once false and is now
proved is worth more than one that was always proved.

SEMANTICS 6.3 makes every *pin write* open-drain-safe ("i in 0..7 with
`OD_MASK[i] == 1`: `PIN_OUT[i] <= 0; PIN_OE[i] <= ~b`"), and PIN-2 (proved)
confirms `loom_pins` implements the masking and the priority faithfully. But
nothing clears `PIN_OUT[i]` when `OD_MASK[i]` is *set*, and `OEP pin, v`
writes `PIN_OE[i]` with no open-drain qualification. So a `PIN_OUT[i] = 1`
set before the pin became open drain survives, and the pad drives high.

Shortest counterexample, firmware only, one edge:

| cycle | port activity | `OD_MASK[7]` | `PIN_OUT[7]` | `PIN_OE[7]` |
|---|---|---|---|---|
| 0 | `rst_n` low: reset | 0 | 0 | 0 |
| 1 | a slot in W commits `CSRW OD_MASK` (`cw_valid=1, cw_od_we=1, cw_od=8'h80`) while the same thread's staged `SETP pin7, 1, D` fires at that edge (`lw_out_mask[7]=1, lw_out_data[7]=1, lw_oe_mask[7]=1, lw_oe_data[7]=1`) | 0 | 0 | 0 |
| 2 | — | **1** | **1** | **1** |

The solver picks the one-edge race because it is shortest. The same bug over
three ordinary slots needs no simultaneity: `SETP pin7, 1` while `OD_MASK[7]`
is 0, then `CSRW OD_MASK, 0x80`, then `OEP pin7, 1`. PIN-1R failing is exactly
that: `OD_MASK[i] & PIN_OUT[i]` is reachable. The host path
(`CTRL.PIN_OUT` then `CTRL.OD_MASK`) does it too, which is the counterexample
sby produced before PIN-1CORE was added to separate the two.

Three ways out were on the table; the director took the first (D-023):

1. **taken:** gate the pads. `loom_pins` now drives
   `uio_out = pin_out[7:0] & ~od_mask` and
   `uio_oe = pin_oe & ~(od_mask & pin_out[7:0])`, so an open-drain pin pulls
   low or lets go and never drives high, whatever order the registers were
   written in; SEMANTICS 3 states the rule and the golden model applies it in
   the same place. Sixteen gates on a non-critical path, and the register
   views still read back what was written;
2. have a write to `OD_MASK` clear `PIN_OUT` on the bits it turns on (a change
   to SEMANTICS 6.6 for `CSRW OD_MASK` and to the CTRL space for the host).
   Rejected: a later `CSRW PIN_OUT` can set the bit again, so the guarantee
   would depend on the order after all;
3. keep the silicon and correct `docs/VERIFICATION.md` L4 to the property that
   was true. Rejected: the chip exists to bit-bang shared buses, where a pin
   that drives high against another device's low is an electrical fault, not a
   firmware inconvenience.

### F-3 a reset while SCK is high inserts a phantom SCK edge, shifting the byte framing

`spi.sby:xfail`, four assertions, from BMC step 63.

SPI-1 holds — SPI-1A, SPI-1B and SPI-1C are proved unbounded — *given* that
the chip is not reset in the middle of a transaction. Drop that one assumption
and the property fails.

`loom_spi_host` detects SCK edges on the synchronised level:
`sck_rise = sck_s & ~sck_q`, and both the `loom_sync` stages and `sck_q` reset
to 0. If SCK is high while `rst_n` is low, the synchroniser holds 0; when reset
is released the high level walks through the two stages and `sck_rise` fires
two clocks later although no edge happened at the pad. If `CS_n` is already low
(the transaction is in progress) the phantom edge is counted as a bit, so the
byte boundary is one bit early for the rest of that `CS_n` low period:
`byte_done` pulses after seven real rising edges, with `rx_byte` holding a
shifted byte. `CS_n` going high resets the counter and the next transaction is
clean.

Counterexample (`CS_n` low throughout):

| cycle | `rst_n` | `SCK` pad | what happens |
|---|---|---|---|
| 0-3 | 0 | 0 | reset held; the SCK synchroniser is 0 |
| 4-5 | 1 | 1 | a normal SCK high phase |
| 6 | **0** | 1 | a reset pulse while SCK is high: both `loom_sync` stages and `sck_q` are forced to 0 |
| 7 | 1 | 1 | reset released, SCK still high at the pad |
| 9 | 1 | 1 | `sck_s` reaches 1 with `sck_q` still 0: **`sck_rise` fires with no pad edge**, `bit_cnt` counts a bit that was never shifted in |
| 12, 20, ... 60 | 1 | rises | the seven remaining real edges of the byte |
| 63 | | | `byte_done` with only seven MOSI bits taken from the pads |

The same mechanism fires if `CS_n` is asserted with SCK already high, which is
a CPOL=0 violation and is ruled out by the `docs/HOST_PROTOCOL.md`
"Electrical" assumptions.

For the director: the host is not expected to reset the chip mid-transaction,
so this may be filed as a documented limitation. If it is worth closing, the
cheap fix is to hold the edge detector off until the synchroniser has settled
after reset — for instance by resetting `sck_q` to 1 instead of 0, which makes
the first post-reset comparison see no edge whatever the pad level, or by
ignoring SCK edges until `cs_active` has been high for two clocks. Either way
`docs/HOST_PROTOCOL.md` should state whether "one transaction per `CS_n` low
period" also means "no reset inside one".

### F-4 `ISO-1` is false while the host's debug port is in use: the port, the register-file write port and the staged-write port are shared

`iso.sby:dbg` and `iso.sby:xfail`, `iso_miter.v` line 417
(`rf_we_oh[t]`), BMC step 5, btor btormc.

ISO-1 says thread `t`'s state sequence is identical "for any two traces that
agree on thread `t`'s inputs (its pins, its FIFOs, SFLAGS it waits on)". That
list is missing the host's debug traffic, and adding it is not enough: the
debug port is one resource for all four threads and its *timing* depends on
what the other three are doing.

- `iso:bmc` and `iso:prove`, with no debug transaction in either trace,
  **pass**; `iso:prove` closes it unbounded with `abc pdr`.
- `iso:xfail`, with only the transactions addressed to `t` held identical,
  fails at step 5.
- `iso:dbg`, with the host's *whole* debug traffic held identical — same
  commands, same cycles, both traces — **still fails at step 5**, on the same
  assertion. So it is not which transactions the host issues; it is the port.

Three mechanisms were found, in this order:

1. **The register-file write port** (the one that survives). `loom_core`
   defers a debug write of `r0..r7` while any thread's slot is writing a
   register: `dbg_rf_wr_go` carries `& ~(w_valid & w_reg_we)` and
   `dbg_wr_done` carries the same term, so no `h_dbg_ack` is raised and the
   host's word retries next clock. Whether a slot is writing a register in
   that cycle is another thread's business, so `rf_we_oh[t]` — thread `t`'s
   register write — lands at a different edge in the two traces.
   `docs/HOST_PROTOCOL.md`, "Transaction format", already says so: "a DEBUG
   write of `r0..r7` may wait up to three more clocks for the register-file
   write port (the thread is halted, so it cannot tell)", and SEMANTICS 7
   gives the reason: "the register file ports are borrowed during that
   thread's bubble slots".
2. **The debug handshake.** `dbg_wr_done` and `dbg_rd_done` both carry
   `& ~h_dbg_ack`, and a debug *read* of `r0..r7` is additionally held off
   while `vd` is high. Both make the ack for one thread's transaction land at
   a cycle that depends on the other threads, and the next transaction — to
   `t` — is gated by that ack. The miter's first counterexample was this one,
   at step 2, with two host commands in consecutive core clocks. The SPI bit
   layer cannot produce that (HOST_PROTOCOL's electrical rules put at least
   eight clocks between SCK edges), so the harness now assumes a well-formed
   host: no new debug transaction in the cycle in which the previous one is
   acknowledged.
3. **Debug register 0x25 is a pin-writing register.** It is
   `{LAT_VALID, LAT_VAL, LAT_PIN}` (SEMANTICS 6.10), so a host write of it
   *stages a pin write* for the target thread, and the staged-write port
   `lw_*` is the OR of all four threads' staged writes. A host write of 0x25
   to a thread other than `t` therefore reaches the pins, at an edge given by
   that thread's `lat_fire`, which diverges with the thread. This was the
   counterexample at step 4 on `lw_out_mask`. It is not a bug either: it is
   the host writing shared state on another thread's behalf, the same
   category as `CSRW PIN_OUT`, and unlike the other shared-state host writes
   it cannot be handled by holding the command equal in the two traces
   because the *edge* is the target thread's. The harness excludes it, with
   the reasoning in `props/loom_iso_props.v`.

**For the director.** None of this is an RTL bug, and (1) is a deliberate
area trade: a second register-file write port to make a debug write
cycle-transparent would cost far more than the three clocks it saves, and the
thread is halted while it happens, so no program can observe it. The change
the finding asks for is to `docs/VERIFICATION.md`'s wording of ISO-1: thread
`t`'s inputs are its pins, its FIFOs, the SFLAGS it waits on **and the host's
debug traffic**, and even then the claim is about thread `t`'s state as a
*sequence of committed values*, not about the exact edge at which a host
debug write lands. The property that is true, and proved unbounded, is
ISO-1 with a quiet debug port.

### F-5 `WAIT-1`'s bound counts the deadline distance in ticks and the budget in slots

`wait.sby:xfail`, `props/loom_wait_props.v` line 183, thread 1, BMC step 17.

L4 says "a timed wait terminates within `TD-NOW+2` slots". `TD - NOW` is a
number of **ticks** (SEMANTICS 4: `NOW` counts ticks, one every
`TICK_INT + TICK_FRAC/256` clocks), and a slot is **4 clocks** (SEMANTICS 2).
The two are the same unit only while a tick is at most one slot, that is
`TICK_INT <= 4` — which the reset period, `TICK_INT = 1`, satisfies
(SEMANTICS 5). Set a longer period and the bound is wrong by the ratio.

Counterexample (`wait.sby:xfail`, thread 1; `TICK_INT` reaches 0x2000 through
an ordinary debug write of 0x10 while the thread is halted, and a `CSRW
TICK_INT` would do the same):

| slot | what happens | `TICK_INT[1]` | `TD - NOW` at the first issue | `slots` | bound |
|---|---|---|---|---|---|
| step 2 | the host sets `TICK_INT[1] = 0x2000`: one tick per 8192 clocks, i.e. per 2048 slots | 0x2000 | | | |
| step 5 | thread 1's `WAITD` first-issues one tick short of its deadline and stalls | 0x2000 | 1 | 1 | 3 |
| step 9 | thread 1's next slot: no tick yet, it re-issues | | | 2 | 3 |
| step 13 | and again | | | 3 | 3 |
| step 17 | and again: **`slots` is 4 against a bound of 3** | | | 4 | 3 |

`NOW` does not tick until 8192 clocks after the write, so the wait cannot
retire for another 2044 slots.

`wait.sby:bmc` is the same property with the tick period held at its reset
value, and it passes to depth 40 for all four threads. So the statement that
is true is one of:

- **in ticks:** a timed wait retires within `TD - NOW` ticks plus at most one
  more slot (the wait completes in the first X cycle at or after
  `reached(NOW, TD)` becomes true, SEMANTICS 2's slot-grid paragraph); or
- **in slots, with the period fixed:** within `TD - NOW + 2` slots while
  `TICK_INT <= 4`, and at the reset period `TICK_INT = 1` the wait is much
  earlier than that, retiring within `ceil((TD - NOW) / 4) + 1` slots because
  `NOW` advances four times per slot.

Neither is an RTL bug: WAIT-1A (`wait.sby:prove`) shows the completion rule
is exactly SEMANTICS 6.4's, and TIMER-1C (proved) shows `NOW` never jumps.
This is a wording correction to `docs/VERIFICATION.md`, like F-1.

### F-6 the properties went stale when slice B added a slot that decodes nothing

Not a design finding, and recorded because it is the failure mode to watch for
whenever the pipeline gains a new kind of slot. ISA-2A/2C (generated) and
WAIT-1A (hand-written) took `!bad_op` to mean "an instruction is in X".
Slice B's LD/ST completion slot (SEMANTICS 6.11) carries the *data* word
through D and X and decodes none of it; the RTL gates every decode effect
with `dec_ok = xins & ~bad_op`, and `bad_op` is itself qualified by `xins`,
so it reads 0 there. A data word that happened to look like a reserved word
or a `WAITD` therefore satisfied each property's antecedent in a slot that
executes no instruction, and the CI run on 54ebaa1 (35871222710) failed both
at depth 4 and 5. ISA-2A/2C now exclude `w_mem_done` (the completion slot in
W; `formal/gen_isa_props.py`) with a new cover showing the exclusion is
reachable, and WAIT-1A uses `dec_ok`. The co-simulation had covered LD/ST all
along and found no divergence: the design was right and the proofs were the
thing that had not been told.

## ISO-1: what the miter assumes

ISO-1 is the one L4 property that is a relation between two traces, so it is
the one harness that is not a property module bound into a single design.
`formal/iso_miter.v` instantiates two `loom_core`s (A and B), each with its
own `loom_pins`, on one clock and one reset, and asserts that thread `t`'s
architectural state is the same in both at every cycle. `t` is `ISO_T`,
0 by default; re-run the group for another thread with
`-DISO_T=<n>` on the `read_slang` line. `props/loom_iso_props.v` holds
`loom_iso_view`, bound into each `loom_core`, which names thread `t`'s state
once (one `s_*` wire per line of SEMANTICS 5's table) and carries the two
assumption groups. The comparison is in the wrapper because an assertion over
two copies has to be written where both are in scope.

### The hypothesis is wiring, not `assume`

Everything the two traces must agree on is *the same wire*; everything else is
two independent free inputs. A hypothesis built this way cannot be vacuous and
cannot accidentally constrain a state variable.

| Agrees in the two copies | Why | Free per copy |
|---|---|---|
| the pads (`pad_in`, one 13-bit input driving both `loom_pins`) | SEMANTICS 3: "`pin_in(i)` for index 0..12: the pad value sampled at edge x-1" | — |
| the instruction word in every cycle whose D slot belongs to `t` | SEMANTICS 2: "D, k+1: `IR = imem[PC[t]]`" — thread `t`'s program words are among its inputs | the word in every other cycle, i.e. the other three threads' programs |
| the `RUN`, `STEP` and `RESET` bits for `t`; pushes into `INQ[t]` with their data; pops of `OUTQ[t]` | SEMANTICS 7: "The host changes `RUN`, `STEP_REQ`, `RESET`, `RESET_PC`, `SFLAGS`, pin registers, and (while the target thread is halted) debug state" | the same bits for the other three threads |
| the debug port, as `ISO_DBG_FREE` says: 0 tied off in both copies (`bmc`, `prove`, `cover`), 2 the whole port shared (`dbg`), 1 only the transactions addressed to `t` (`xfail`) | finding F-4: the port is shared, so mode 1 and mode 2 both fail and mode 0 is the property that holds | in mode 1, the other threads' transactions |
| the host's `SFLAGS` set/clear and `PIN_OUT` / `PIN_OE` / `OD_MASK` writes | SEMANTICS 2's shared state visibility rule names exactly these as shared | — |
| the `RESET_PC` write port, whole | over-constraint, see below | — |
| the `RUN` and `STEP` write strobes | one write hits all four bits at once (SEMANTICS 7, "`CTRL.RUN` write: `RUN <= value`"), so only the value bits can differ | the value bits for the other threads |

### The four assumptions

Three are in `props/loom_iso_props.v` and one in `iso_miter.v`, each with the
sentence that puts the duty on the other side.

1. **The ARCHITECTURE 1 precondition.** In a valid slot, a thread other than
   `t` never decodes an instruction that writes shared state: `SETP` (plain
   or deadline-latched, 6.10), `OEP`, `OUT`, `SHO` (6.9, which drives a pin,
   and a second one with `DIFF`), `SIG`, `CLR`, `WAITS` (a test-and-clear,
   6.5) and `CSRW` of `SFLAGS` / `PIN_OUT` / `PIN_OE` / `OD_MASK` /
   `HOST_IRQ` (6.6). This is the "except through explicit shared state" of
   ARCHITECTURE 1 and of SEMANTICS 2's shared state visibility rule, and it
   is what makes the property say something: a thread that writes `SFLAGS` or
   a pin is *meant* to be visible to the others. Nothing constrains thread
   `t`, and nothing constrains the other threads' ALU, branch, FIFO, timer or
   bit-engine-register work.
2. **The same precondition on the host's side of the deadline latch.** The
   host does not write debug register 0x25 of a thread other than `t`. That
   register is `{LAT_VALID, LAT_VAL, LAT_PIN}` (6.10), so writing it stages a
   *pin* write for the target thread on a port all four threads share: it is
   a write to shared state, like `CSRW PIN_OUT`, and unlike the other
   shared-state host writes it cannot be handled by holding the command equal
   in the two traces, because the edge at which the staged write applies is
   the target thread's `lat_fire`, which diverges with that thread. Vacuous
   in the `bmc`, `prove` and `cover` tasks, where the debug port is tied off.
3. **The two host legality conditions of SEMANTICS 7**, exactly as
   `loom_sched_props.v` assumes them: "The host must only reset a halted
   thread; resetting a running thread is undefined", and a debug write needs
   the target thread halted. The harness reads the RTL's own `thread_busy`
   for "halted", which SCHED-1 (proved, unbounded) shows is exactly
   SEMANTICS 7's definition.
4. **A well-formed host**, in `iso_miter.v`: no new debug transaction in the
   cycle in which the previous one is acknowledged. HOST_PROTOCOL's
   electrical rules put at least eight clocks between two SCK edges and
   "every effect of the word is registered at edge E + 4", so
   `loom_host_ctl` cannot present two debug commands in consecutive core
   clocks. It is deliberately not gated on `rst_n`, because the solver's
   first move was to drop `rst_n` for one cycle to switch it off. Vacuous
   with the debug port tied off.

### What is asserted

Thread `t`'s `PC`, `{T,C,Z}`, `WAIT_ACTIVE`, `RS0`/`RS1`/`DEPTH`,
`PREV_PINS`, `OUTGRP`, `INGRP`, `STEPS`, `RESET_PC`, `RUN`, `HALTED`,
`STEP_REQ`, its `BADOP` and `SWIRQ` bits; `r0..r7` through the register
file's storage, plus the write port as it reaches `t`; `NOW`, `TD`, `DT`,
`ACC`, `TICK_SEEN`, `TICK_INT`, `TICK_FRAC` and `lat_fire`; `SR`, `CNT`,
`CRC`, `BE_CFG`, `BE_PINS`, `BE_RELOAD`, `CRC_POLY`, `CRC_INIT` and the
6.9.1 encoder state; `LAT_VALID`/`LAT_PIN`/`LAT_VAL`; `INQ_CNT`, `OUTQ_CNT`
and the FIFO head words; `SFLAGS` whole; thread `t`'s pin-write commit
(masks and data, ordinary and staged), the three shared pin registers and
the three pad buses; thread `t`'s retire record; and, to localise a failure
early, `t`'s own D, X and W pipeline registers. The slot grid (`ph`) is
asserted against SEMANTICS 1 in both copies.

Two fields are compared under a guard, because SEMANTICS defines them only
under that guard, and the unguarded version fails:

- `tr_val` and `tr_rd` only when `tr_we`. SEMANTICS 8 lists them as
  "register write, *if any*". A `POP` of an empty `INQ` still routes the INQ
  head into `tr_val`, and "entries are not reset" (SEMANTICS 5), so that word
  is free, and free independently in the two copies. Unguarded this is the
  one assertion of the set that fails, at BMC step 8.
- the `INQ`/`OUTQ` head words only while the count says they hold something
  pushed, for the same reason.

### What is outside the miter, and the two ways it is not the real chip

`loom_imem`, `loom_host_ctl` and `loom_spi_host` are not in the miter.

- **Over-approximations** (they make the result stronger, and a `cover`
  trace weaker): `imem_rdata` is a free input per copy rather than a read of
  a constant array, so the proof covers every memory image whose thread-`t`
  quarter is the same in the two traces, and also images that change under
  the reader; the host port can act on every cycle, where the SPI bit layer
  needs tens of clocks per word (HOST_PROTOCOL, "Transaction format"). An
  assertion proved here holds for the real, narrower environment. A `cover`
  trace need not correspond to one constant memory or to a reachable SPI
  command sequence.
- **Over-constraints** (they make the result weaker, and are the honest
  limits of this run): the `RESET_PC` write port is shared, so the other
  three threads' `RESET_PC` is also the same in the two traces — only thread
  `t`'s is load-bearing, and the other threads have four other ways to
  diverge, which the cover task exercises; the `RUN` and `STEP` write strobes
  are simultaneous in the two traces; and thread `t`'s pads are the same
  input vector in both copies rather than a BIDIR loopback of what each copy
  drives.

## Files

| File | What |
|---|---|
| `isa.sby`, `props/loom_isa_props.v` | ISA-1 on `loom_decode` |
| `isacore.sby`, `props/loom_isa_core_props.v` | ISA-2 (reserved -> NOP + BADOP) on `loom_core` |
| `gen_isa_props.py` | writes both of the above from `isa/isa.yaml` through `tools.loomisa`; `--check` fails if they are stale |
| `fifo.sby`, `props/loom_fifo_props.v` | FIFO-1..3 on `loom_fifo` |
| `sched.sby`, `props/loom_sched_props.v` | SCHED-1..3 and FIFO-1B on `loom_core` |
| `pins.sby`, `props/loom_pin_props.v` | PIN-1 (fails) and PIN-2 on `loom_pins` |
| `timer.sby`, `props/loom_timer_props.v` | TIMER-1 (fails), TIMER-1B, TIMER-1C on `loom_timer` |
| `spi.sby`, `props/loom_spi_props.v` | SPI-1 on `loom_spi_host` |
| `iso.sby`, `iso_miter.v`, `props/loom_iso_props.v` | ISO-1, the two-copy thread-isolation miter: two `loom_core` and two `loom_pins` |
| `wait.sby`, `props/loom_wait_props.v` | WAIT-1 (bounded liveness of a timed wait) and WAIT-1A (the completion rule of SEMANTICS 6.4) on `loom_core` |

The two ISA property files are generated. Everything else is hand-written from
`docs/SEMANTICS.md`; each property carries the sentence it encodes in a comment
above it, and the RTL was read only for signal names.

## Harnesses and what is free

| Group | Top | Free inputs |
|---|---|---|
| `fifo` | `loom_fifo` | everything, minus the SEMANTICS 6.7 caller contract, discharged by FIFO-1B |
| `sched`, `isacore` | `loom_core` | `imem_rdata` (so the proof covers every instruction stream, including words no assembler emits), pads, and the whole host port minus the two legality conditions of SEMANTICS 7 (only a halted thread is reset, only a halted thread takes a debug write) |
| `wait` | `loom_core` | the same, minus two more: a stalled wait re-issues the *same* instruction word (SEMANTICS 7, "Host IMEM reads and writes are valid only while `RUN == 0` and no step in flight", so the word at a running thread's PC cannot change), and the deadline distance the monitor arms on is at most 8 ticks, to keep the BMC depth small |
| `pins` | `loom_pins` | everything, minus "open drain is applied upstream" (SEMANTICS 6.3, the documented `loom_core` -> `loom_pins` contract) |
| `timer` | `loom_timer` | everything: any sequence of TD, DT, TICK_INT and TICK_FRAC writes |
| `spi` | `loom_spi_host` | the three pads, minus the HOST_PROTOCOL electrical rules (SCK period >= 8 clocks, CPOL 0, `CS_n` low 4 clocks before the first edge and 4 after the last) |
| `isa` | `loom_decode` | `ir`: all 2^16 words |
| `iso` | `loom_iso_miter` (two `loom_core` + two `loom_pins`, `IMEM_WORDS = 512`, `FIFO_DEPTH = 4`) | `imem_rdata` and the whole host port, per copy and independently, minus what the ISO-1 hypothesis ties together and the assumptions listed below |

`loom_fifo` is proved at `DEPTH = 4`, the build parameter of `loom_top`; the
`bind` has to name the entry words one by one, so a different depth needs the
bind widened.
