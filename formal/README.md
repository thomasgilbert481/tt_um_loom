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

Recorded 2026-09-19 (TIMER-2 added 2026-09-22 with D-028), OSS CAD Suite: SBY
0.63, Yosys 0.63+161, yices 2.6, btormc (boolector), abc pdr. Times are wall
clock on the laptop. The whole of `scripts/formal.sh` is 20 sby tasks in about
5 minutes, the slowest single task
being `sched:bmc` at 62 s — inside the `formal-quick` budget of the CI matrix in
`docs/VERIFICATION.md`, with nothing left for a `formal-full` nightly to do
except ISO-1 and WAIT-1.

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
| SCHED-1..3, FIFO-1B | bounded cross-check | `sched.sby:bmc` | bmc | btor btormc | 24 | pass | 39 s |
| — | 5 cover points | `sched.sby:cover` | cover | smtbmc yices | 20 | 5/5 | 7 s |
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

`spi.sby:prove` is proved by `abc pdr`; k-induction at depth 20 does not close
it (the byte counter needs a deeper invariant than 20 cycles), which is why the
group lists two engines and takes the first conclusive answer.

Not attempted: ISO-1 (the two-copy thread-isolation miter) and WAIT-1 (bounded
liveness of a timed wait). Both are still open in `docs/VERIFICATION.md` L4.

## Findings

Three findings. Two are properties of the L4 list that do not hold; the third
is a condition SPI-1 needs that the L4 wording does not mention. None has been
weakened: each is kept in the harness exactly as `docs/VERIFICATION.md` words
it, under a task that carries `expect fail`, so the counterexample is
reproduced on every run and the harness turns red if the behaviour ever
changes silently.

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

The two ISA property files are generated. Everything else is hand-written from
`docs/SEMANTICS.md`; each property carries the sentence it encodes in a comment
above it, and the RTL was read only for signal names.

## Harnesses and what is free

| Group | Top | Free inputs |
|---|---|---|
| `fifo` | `loom_fifo` | everything, minus the SEMANTICS 6.7 caller contract, discharged by FIFO-1B |
| `sched`, `isacore` | `loom_core` | `imem_rdata` (so the proof covers every instruction stream, including words no assembler emits), pads, and the whole host port minus the two legality conditions of SEMANTICS 7 (only a halted thread is reset, only a halted thread takes a debug write) |
| `pins` | `loom_pins` | everything, minus "open drain is applied upstream" (SEMANTICS 6.3, the documented `loom_core` -> `loom_pins` contract) |
| `timer` | `loom_timer` | everything: any sequence of TD, DT, TICK_INT and TICK_FRAC writes |
| `spi` | `loom_spi_host` | the three pads, minus the HOST_PROTOCOL electrical rules (SCK period >= 8 clocks, CPOL 0, `CS_n` low 4 clocks before the first edge and 4 after the last) |
| `isa` | `loom_decode` | `ir`: all 2^16 words |

`loom_fifo` is proved at `DEPTH = 4`, the build parameter of `loom_top`; the
`bind` has to name the entry words one by one, so a different depth needs the
bind widened.
