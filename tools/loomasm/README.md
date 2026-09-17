# loomasm: the Loom assembly language

`tools/loomasm` turns a `.loom` source file into the JSON instruction image the
golden model and the host library load, a listing with slot timing, and a
static **deadline analysis**: the check that the code between two deadline
instructions always fits inside the tick budget.

No encoding lives here. Every word is produced by `tools.loomisa`, which is the
only code that parses `isa/isa.yaml`, so adding an instruction to the YAML
makes it assemblable without touching this package.

```
python -m tools.loomasm firmware/uart_hello.loom \
        -o firmware/build/uart_hello.json --listing
```

```python
from tools.loomasm import assemble, disassemble

program = assemble("firmware/uart_hello.loom")   # or a Path, or source text
program.words                # {address: 16-bit word}
program.symbols              # {label or .equ name: value}
program.threads[0].entry     # first address of thread 0's section
program.deadlines[0].worst_slack
disassemble(0x9001)          # 'WAITD 1'
```

---

## 1. Lines

One statement per line. A line is

```
[label:] [label:] [ mnemonic|directive [operand [, operand ...]] ] [; comment]
```

- `;` starts a comment that runs to the end of the line.
- A label ends with `:` and may share its line with a statement.
- Blank lines and comment-only lines are legal and appear in the listing.
- There is no line continuation and no multi-line construct: that is what lets
  the assembler report **every** bad line in a file instead of stopping at the
  first one.

**Case.** Mnemonics, pseudo-ops, directives, registers (`r0`..`r7`), CSR names,
pin names, `RISE`/`FALL`/`ANY`, the `WAITB` conditions and the timeout token `T`
are **case-insensitive**. Labels and `.equ` symbols are **case-sensitive**.

`r0`..`r7` are reserved: they cannot be used as a label or an `.equ` name.

## 2. Numbers and expressions

| Form | Example |
|---|---|
| decimal | `434` |
| hexadecimal | `0x1F` |
| binary | `0b1010` |
| character | `'A'`, `'\r'`, `'\n'`, `'\0'`, `'\t'`, `'\\'`, `'\''` |

Digits may be grouped with `_` (`0b1010_0101`). The recognised escapes are
`\r \n \t \0 \\ \' \" \a \b \f \v \e`.

Constant expressions use, from lowest precedence to highest:

```
|        &        << >>        + -        * /        unary - +
```

plus `( )`, symbols (labels and `.equ` names), and the two byte selectors
`lo8(x)` = `x & 0xFF` and `hi8(x)` = `(x >> 8) & 0xFF`. Arithmetic is
arbitrary-precision; division truncates towards zero; division by zero and a
negative or absurd shift count are errors. The range check happens where the
value is used, and comes from `tools.loomisa`, so the message names the field:
`LDI: operand imm8=300 outside 0..255`.

A **label may be referenced before it is defined**; an `.equ` symbol may not.
`.equ` is evaluated where it is written.

## 3. Operands

Operand kinds come from the operand names in `isa/isa.yaml`, so the table below
follows the ISA rather than a list kept in this package.

| Kind in `isa.yaml` | Written as |
|---|---|
| `rd`, `ra`, `rb` | `r0` .. `r7` |
| `imm5`, `imm6`, `imm8` | an unsigned constant expression |
| `rel6`, `rel8` | a **label** (see below) or a numeric offset |
| `abs10` | a label or an address expression |
| `pin` | a pin name from `isa.yaml` (`BIDIR0`.., `IN0`.., `OUT0`..), a name declared with `.pins`, an `.equ` symbol, or a number 0..31 |
| `val` | `0` or `1` |
| `edge` | a name from `isa.yaml` `enums.edge`: `RISE`, `FALL`, `ANY` (or 0, 1, 2) |
| `flag` | 0..7 |
| `cond2` | a name from `isa.yaml` `enums.cond`: `BE_IDLE`, `OUTQ_NF`, `INQ_NE`, `TICK` (or 0..3) |
| `csr` | a CSR name from `isa.yaml` (`TICK_INT`, `TD`, `NOW`, ...) or a number 0..31 |
| `tmo` | the bare token `T` as the last operand, or omitted |

**Enumerated operands.** Any operand whose base name appears in the `enums:`
section of `isa.yaml` accepts those names case-insensitively, or a number in
`0 .. max(value named there)`. This package holds none of the spellings, so
adding an enum to the YAML makes it assemblable and disassemblable at once, and
the disassembler prints the YAML's own spelling. The numeric range deliberately
follows the names rather than the field width, which keeps an encoding the ISA
does not define out of reach: the `edge` field is two bits, but only 0..2 are
named and `SEMANTICS.md` 6.4 says `e == 3` is never true. Use `.word` if you
ever need to place one of those words on purpose.

**Pin indices.** A pin operand that resolves to an index `isa.yaml` does not
name (13, 14, 15 and 22..31) is a **warning**, not an error: those indices are
not pads, so writes to them are ignored and reads return 0
(`SEMANTICS.md` 6.3). The warning is raised at each *use*, whether the index
arrived as a literal, a `.pins` alias or an `.equ` symbol, and never for a
named pin.

**Relative operands.** If the expression mentions at least one symbol it is a
**target address**, and the assembler emits `target - (pc + 1)` and checks that
it fits the field. If it is purely numeric it is a **raw offset**, emitted as
written. So `BZ loop` and `BZ loop + 1` are addresses; `BZ -3` is an offset, and
`BZ 0` is "fall through". Out of range gives, for example:

```
demo.loom:31: error: BZ cannot reach 0x180 from 0x004: offset 379, rel8 holds -128..127
```

**Timeouts.** `WAITP`, `WAITE`, `WAITS` and `WAITB` take an optional trailing
`T`, which is the deadline-timeout bit:

```
        WAITP   SCL, 1, T               ; give up when NOW reaches TD, set the T flag
        WAITE   SDA, FALL               ; wait forever for a falling edge
```

## 4. Directives

| Directive | Meaning |
|---|---|
| `.imem W` | The instruction-memory size to lay out for. Must come before any code. |
| `.thread N` | Switch to thread `N`'s section (`N` in 0..3). |
| `.org ADDR` | Set the current thread's location counter. |
| `.equ NAME = expr` | Define a constant. Evaluated where written. |
| `.pins NAME = PIN [, NAME = PIN ...]` | Name pins. `PIN` is an ISA pin name, an earlier alias, or a number. |
| `.word expr [, expr ...]` | Emit raw 16-bit words (`-0x8000..0xFFFF`). |
| `.csr NAME, expr [, rN]` | Load a constant into a CSR. **Clobbers the scratch register**, `r7` by default. |
| `.tick CLOCKS` | Declare the current thread's tick period, for the deadline checker only. Emits nothing. |
| `.deadline_check on\|off` | Enable or disable the deadline analysis for the current thread. Default `on`. |

**Thread sections and the memory size.** Each thread's location counter starts
at that thread's reset vector, which the hardware sets to
`N * (IMEM_WORDS / 4)` (`docs/DECISIONS.md` D-017, `docs/SEMANTICS.md`
section 5). The assembler lays out for `IMEM_WORDS = 1024` unless told
otherwise, so the default origins are 0, 0x100, 0x200, 0x300:

| `IMEM_WORDS` | thread 0 | 1 | 2 | 3 |
|---|---|---|---|---|
| 64 | 0 | 16 | 32 | 48 |
| 128 | 0 | 32 | 64 | 96 |
| 256 | 0 | 64 | 128 | 192 |
| 512 | 0 | 128 | 256 | 384 |
| **1024** (default) | 0 | 256 | 512 | 768 |

Say which build a program is for with `.imem W` in the source or
`--imem-words W` on the command line (`imem_words=` on `assemble()`). Valid
sizes are the powers of two from 64 to 1024, the range the 10-bit PC can
address. The command-line option wins over `.imem`, and warns if the two
differ. `.imem` must come before any code, because it moves every thread's
origin: after the first label or emitted word it is an error.

Switching away from a thread and back resumes where its section left off. An
address at or past the end of memory, and two threads writing the same address,
are both errors:

```
demo.loom:12: error: address 0x100 is already used by thread 0 (line 7): thread sections overlap
demo.loom:31: error: address 0x100 is past the end of instruction memory (256 words)
```

The **PC still wraps at 2^10** whatever the memory size, so a branch's reach is
computed from the 10-bit next-PC and not from `IMEM_WORDS`.

**Labels on a `.thread` or `.org` line** bind *after* the directive has moved
the location counter, which is the reading `entry: .org 0x40` suggests.

**`.csr`** expands to the `MOV16` pseudo-op followed by `CSRW`, so
`.csr TICK_INT, 434` is three words (`LDI`, `LDIH`, `CSRW`) and
`.csr TICK_FRAC, 0` is two (`LDI`, `CSRW`). Use the third operand to pick a
different scratch register: `.csr TICK_INT, 434, r6`.

**`.csr TICK_INT, <constant>`** also declares the tick period for the deadline
checker, so firmware that sets the divider needs no separate `.tick`. An
explicit `.tick` and a `.csr TICK_INT` may both appear; the last one wins.
`TICK_FRAC` is ignored by the checker, which rounds the period down.

## 5. Pseudo-ops

From the `pseudo_ops` section of `isa/isa.yaml`:

| Pseudo-op | Expands to |
|---|---|
| `MOV16 rd, imm16` | `LDI rd, lo8(v)` and `LDIH rd, hi8(v)`, or `LDI` alone |
| `BRA label` | `JMP label` |
| `INC rd` | `ADDI rd, 1` |
| `DEC rd` | `SUBI rd, 1` |

`MOV16` emits **one** word when the value is known during the first pass and
fits in eight bits, and two words otherwise. A forward-referenced symbol is not
known during the first pass, so it always produces the two-word form. The size
is decided once, in pass 1, and never changes in pass 2, so labels never move.

`CLRF` is listed in `isa.yaml` but is **not implemented**: it depends on a
`.zero` register-tracking directive that this language does not have. Write
`CSRW FLAGS, rN` with a register you know is zero.

## 6. Output

**Image** (`-o FILE`, or stdout). The format the golden model and the host
library load:

```json
{
  "isa": "0.3.0",
  "imem_words": 1024,
  "words": {"0": 10162, "1": 12033},
  "symbols": {"bit": 23, "start": 5},
  "threads": {"0": {"entry": 0, "size": 33}},
  "source": "firmware/uart_hello.loom"
}
```

`words` and `threads` use decimal string keys (JSON object keys are strings);
values are plain integers. `imem_words` is the memory size the program was laid
out for, so a loader can check it against the build it is loading into. `entry`
is the first address the thread's section emitted, which is its reset vector
unless `.org` moved it, and `size` is how many words the thread emitted.

**Listing** (`--listing [FILE]`): address, word, thread, timing class from
`isa.yaml` (`one_slot`, `wait`, `blocking`), source line number and source text,
with the extra words of a multi-word statement shown disassembled underneath.
Each thread's deadline analysis follows.

**Diagnostics** go to stderr as `file:line:col: severity: message`. Every error
in the file is reported. Diagnostics carry a `kind`: `syntax`, `symbol`,
`range` and `layout` are fatal (no image), while `deadline` and `pin` are
advisory and never suppress the image.

**Exit status** is 0 when the image was produced, 1 when assembly failed. A
deadline that cannot be met is an *error-level diagnostic* but does not by
itself suppress the image; `--strict` makes it fail the run.

---

## 7. Deadline checker

Loom's timing contract is the deadline register `TD`. `SETD m` anchors it at
`NOW + m`; `WAITD k` advances it by `k` ticks and stalls until `NOW` reaches
it. Firmware is correct only if the code between two deadline instructions
always finishes before the later deadline. Every slot is exactly
`meta.slot_clocks` = 4 clocks (`docs/SEMANTICS.md` section 2), so the check is
arithmetic once the longest path is known.

For each thread the checker

1. builds the control-flow graph of the thread's section from the **emitted
   words**, decoded through `loomisa` (so pseudo-op and `.csr` expansions are
   analysed exactly as the hardware will see them);
2. cuts the graph at every anchor, which is a `SETD` or a `WAITD k` with
   `k != 0`;
3. for every anchor, finds each `WAITD k` reachable along a path with no other
   anchor on it, and takes the **longest** such path, counting one slot per
   instruction strictly between them plus the target `WAITD` itself;
4. compares `slots * 4` clocks against the budget, which is `P` clocks per
   tick times

   - `m + k` ticks from a `SETD m` anchor, because the interval starts `m`
     ticks before that `SETD`'s own deadline, and
   - `k` ticks from a `WAITD` anchor, which adds nothing of its own.

Control flow: `JMP`/`CALL` follow their absolute target, `RET` returns to the
address after **every** `CALL` in the thread, `HALT` ends the path, and the
conditional transfers (`BZ BNZ BC BNC BT BNT DJNZ JP`) take both edges.

**`WAITD 0` is transparent.** It commits `TD <= TD + 0`, so it sets no new
deadline: it costs one slot on the path, it does not end an interval, it does
not start one, and it is never itself reported as infeasible. On any path that
reaches it from a `WAITD` anchor the deadline has already been reached, so it
behaves as a one-slot `NOP`; after a `SETD m` anchor it does wait, but it still
leaves the next real `WAITD k` as the deadline that matters.

**`CSRW TD` withdraws the `SETD` credit.** If a `CSRW TD, rN` (or a
`.csr TD, ...`) lies between a `SETD m` and the `WAITD`, the deadline that
`SETD` set has been replaced by a run-time value, so the `m` ticks are not
credited and the budget falls back to `k * P`. The listing marks those rows
`(TD rewritten: SETD ticks not credited)` and the thread summary says so.

**Unbounded.** A path is unbounded, and reported as a *warning*, when it
contains a loop with no anchor on it, or an instruction with no static bound:
`DLY`, `PUSH`, `POP`, or `WAITP`/`WAITE`/`WAITS`/`WAITB` **without** `T`.
This is usually not a bug: re-anchoring with `SETD` right after such an
instruction is the correct idiom, and then no path reaches a `WAITD` without
passing a `SETD`, so no pair is formed at all. That is what `uart_tx.loom` does
after `POP`.

**Infeasible.** Reported as an *error*:

```
demo.loom:24: error: deadline cannot be met (thread 0: 0x004 WAITD -> 0x00C WAITD 1):
  9 slots = 36 clocks, budget 1 x 20 = 20 clocks, short by 16
```

The listing shows the slack of every pair, so the margin is visible before it
runs out.

### What it is sound about

- Paths are over-approximated, never under-approximated: unreachable paths and
  impossible `RET` targets can make the reported worst case **pessimistic**,
  but a real deadline miss on any executable path is never missed.
- It never needs to know the values in registers.

### Known limits

1. **Wait instructions with `T` count as one slot.** A timed-out wait can burn
   the whole budget up to `TD`. If a `WAITD` follows one in the same interval,
   the reported slack is optimistic. Anchor with `SETD` after a timed wait.
2. **A run-time `TD` is not tracked.** A `CSRW TD, rN` only withdraws the
   `SETD` credit (above); the checker does not try to work out what value the
   register held. That is conservative, never optimistic.
3. **`CALL`/`RET` are context-insensitive.** A `RET` flows to every return site
   in the thread, so a subroutine called from two places is analysed as though
   any caller could follow any callee path.
4. **The tick period must be static.** A `CSRW TICK_INT, rN` at run time is
   invisible to the checker; only `.tick` and a constant `.csr TICK_INT` are
   seen. With no period, the listing reports slot counts and no verdict.
5. **`TICK_FRAC` is rounded down**, so a fractional divider is treated as
   slightly faster than it is - conservative again.
6. **Control flow out of the thread's section is not followed.** Those paths
   simply end, and the thread summary says so. Data placed with `.word` is not
   an instruction, so a path that falls into it also ends.
7. **The pipeline is not modelled.** One slot is one instruction issue; a
   stalling instruction re-issues, which is what makes the unbounded classes
   above unbounded.
8. The analysis is O(n^2) in the size of a thread's section. At 1024 words that
   is irrelevant; it is not meant for a much bigger machine.

## 8. Module layout

| File | Holds |
|---|---|
| `lexer.py` | tokens, character literals, comma splitting |
| `expr.py` | constant expressions, `lo8`/`hi8` |
| `parser.py` | labels, mnemonic or directive, operand groups |
| `assembler.py` | the two passes, directives, pseudo-ops, `Program` |
| `deadline.py` | control-flow graph and the deadline analysis |
| `disasm.py` | `disassemble(word)`, via `loomisa.decode` |
| `listing.py` | the `--listing` report |
| `diag.py` | `Diagnostic`, `AsmError`, the diagnostic kinds |
| `names.py` | ISA name lookups, plus the only two spellings the YAML has no place for: `r0`..`r7` and the timeout token `T` |
