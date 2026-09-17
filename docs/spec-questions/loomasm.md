# Spec questions raised while building `tools/loomasm`

Raised by the assembler session, 2026-09-17. Each item states the question as
it was raised, then carries the director's ruling under **Resolution**. All
nine are now closed.

Where a resolution says **Implemented**, the change is in `tools/loomasm` with
tests (items 4, 5, 6 and 9). The others were settled in `isa/isa.yaml` (now ISA
0.3.0) and `docs/ARCHITECTURE.md` by the director; this session did not touch
`isa/isa.yaml`, `tools/loomisa`, `docs/SEMANTICS.md` or any generated file.

---

## 1. `CLRF` is specified in terms of a directive that does not exist

`isa/isa.yaml`, `pseudo_ops`:

```yaml
- {name: CLRF, expands: "CSRW FLAGS, rz (a register holding 0)",
   doc: assembler tracks a zero register if declared with .zero}
```

There is no `.zero` directive anywhere in `ARCHITECTURE.md` section 11.3, and
an assembler cannot in general know that a register holds zero. Tracking a
"declared zero register" across `CALL`/`RET` and across threads is a dataflow
problem, not a pseudo-op.

**Assembler today:** `CLRF` is not implemented. Firmware writes
`CSRW FLAGS, rN` with a register it knows is zero.

**Decision wanted:** drop `CLRF` from `pseudo_ops`, or replace it with
something self-contained (for example make it two words, `LDI rz, 0` then
`CSRW FLAGS, rz`, with the scratch register named in the operand the way
`.csr` does it).

**Resolution (director, 2026-09-17):** dropped. `CLRF` is gone from `isa.yaml` `pseudo_ops` in ISA 0.3.0; firmware writes `CSRW FLAGS, rN` with a register it knows is zero. No assembler change.

## 2. The header comment of `isa/isa.yaml` names generated files that do not exist

```
#   tools/loomasm/tables.py    assembler tables
#   tools/loomsim/tables.py    golden model dispatch
#   formal/isa_decode.sv       decode-completeness properties (ISA-1)
```

`tools/loomisa` generates `src/loom_isa.vh`, `src/loom_decode.v` and
`docs/ISA.md` and nothing else. More importantly, a generated
`tools/loomasm/tables.py` would contradict the `CLAUDE.md` rule that
`tools/loomisa` is the *only* code that parses the YAML: the assembler now
calls `loomisa.load()`/`encode()`/`decode()` at run time and holds no table of
its own. The comment also still says "Version 0.1 seed" while `meta.version`
is `0.2.0`.

**Decision wanted:** update the comment block to the three files that are
really generated, and drop the two `tables.py` lines.

**Resolution (director, 2026-09-17):** fixed. The header comment of `isa/isa.yaml` now names only the three files `tools/loomisa` really generates, and the stale "0.1 seed" line is gone. No assembler change.

## 3. `ARCHITECTURE.md` 11.3 and `isa.yaml` `pseudo_ops` disagree

| ARCHITECTURE 11.3 | `isa.yaml` `pseudo_ops` | Assembler today |
|---|---|---|
| `MOV rd, imm16` | `MOV16 rd, imm16` | `MOV16`. `MOV rd, ra` is a real instruction, so `MOV rd, 0x1234` would be a genuine ambiguity. |
| `BRA rel` | `BRA label` -> `JMP label` | `BRA label`, an absolute `JMP`. It is not a relative branch, so the ARCHITECTURE spelling is misleading. |
| `WAITD n.bits` with symbolic tick units | absent | not implemented |
| `.crc usb16` presets | absent (`crc_presets` exist, no directive) | not implemented; no M1 firmware needs the bit engine |
| `.thread t`, `.pins`, `.deadline_check on\|off` | absent | implemented |

**Decision wanted:** make 11.3 a pointer to `isa.yaml pseudo_ops` plus the
directive list in `tools/loomasm/README.md`, rather than a third list. If
`WAITD n.bits` and `.crc <preset>` are still wanted, they are cheap to add once
the bit engine lands at M2.

**Resolution (director, 2026-09-17):** `ARCHITECTURE.md` 11.3 is now a pointer to `isa.yaml` `pseudo_ops` and to `tools/loomasm/README.md` instead of a third list. `WAITD n.bits` and `.crc <preset>` stay unimplemented until the bit engine lands at M2. No assembler change.

## 4. Symbolic names for `WAITB` conditions and `WAITE` edges are not in `isa.yaml`

`isa.yaml` gives the `WAITB` conditions only inside a prose `sem:` string
("0 BE idle, 1 OUTQ not full, 2 INQ not empty, 3 a tick occurred") and the
`edge` field only in `field_types` ("0 rise, 1 fall, 2 any"). Neither has a
machine-readable name list, so the assembler had to invent the spellings:

- `edge`: `RISE`, `FALL`, `ANY`
- `cond2`: `BE_IDLE`, `OUTQ_NF`, `INQ_NE`, `TICK`

They live in `tools/loomasm/names.py`, which is the one place the assembler
holds an operand spelling that did not come from the YAML.

**Decision wanted:** add an `enums:` section to `isa.yaml` (for example
`edge: {RISE: 0, FALL: 1, ANY: 2}`) so these names are single-sourced like
everything else, or bless the four `WAITB` names above as they stand.

**Resolution (director, 2026-09-17):** `isa.yaml` gained an `enums:` section (`enums.edge`, `enums.cond`), exposed by the loader as `isa.enums`. **Implemented:** the assembler and the disassembler now take every enumerated spelling from there, case-insensitively, and accept numbers in `0 .. max(named value)` so `edge == 3` stays unreachable except through `.word`. `tools/loomasm/names.py` holds no spelling that lives in the YAML, and a test asserts it.

## 5. Does a `SETD m` anchor's own `m` count toward the next `WAITD k`?

From `SEMANTICS.md` 6.4, `SETD m` sets `TD = NOW + m` and `WAITD k` sets
`TD = TD + k`. So the wall-clock budget from a `SETD m` to the next `WAITD k`
is `(m + k)` ticks, not `k`.

**Assembler today:** the budget of a `SETD m` -> `WAITD k` pair is reported as
`k * P`, which under-reports the margin whenever `m > 0`. It is conservative
(it can only produce a false "cannot be met", never a false "fine"), and both
M1 firmware programs use `SETD 0`, where the two agree exactly.

**Decision wanted:** confirm that `(m + k) * P` is the intended budget, in
which case the checker should be tightened before anyone writes `SETD` with a
non-zero operand.

**Resolution (director, 2026-09-17):** `(m + k) * P`. **Implemented:** a `SETD m` anchor now contributes its own `m` ticks to the budget of the next `WAITD k`; a `WAITD` anchor still contributes nothing. When a `CSRW TD` (including `.csr TD, ...`) lies on the path the anchor's value has been replaced at run time, so the credit is withdrawn and the budget falls back to `k * P`; the listing marks those rows and the thread summary says so.

## 6. `WAITD 0` has a zero budget

`WAITD 0` leaves `TD` where it was, so the budget for any code preceding it in
the same interval is zero clocks and the checker calls it infeasible as soon as
one instruction sits between the anchors. That may be the honest answer (the
deadline really has already passed), or `WAITD 0` may be intended as a
"re-synchronise to the current deadline" idiom that the checker should exempt.
No M1 firmware uses it.

**Decision wanted:** is `WAITD 0` an idiom worth exempting, or is flagging it
correct?

**Resolution (director, 2026-09-17):** `WAITD 0` is transparent to the checker. **Implemented:** it costs one slot on the path, does not end an interval, does not start one, and is never itself reported as infeasible. It leaves `TD` unchanged, so on any path reaching it from a `WAITD` anchor the deadline has already been reached and it behaves as a one-slot `NOP`; after a `SETD m` anchor it does wait, but it still sets no new deadline.

## 7. `ARCHITECTURE.md` 6 calls the deadline check a warning; this deliverable makes it an error

> "The assembler ... **warns** when the slot count between two `WAITD`
> instructions exceeds the tick interval (the schedule cannot be met)."

The assembler emits a schedule that cannot be met as an **error**-level
diagnostic (the image is still written; `--strict` refuses it), and reserves
*warning* for the "unbounded between deadlines" case, which is usually the
correct `SETD` re-anchoring idiom rather than a bug. That split reads better in
practice, but it is not what 11.3's sentence says.

**Decision wanted:** confirm the severity split, and update the sentence in
`ARCHITECTURE.md` 6.

**Resolution (director, 2026-09-17):** confirmed as built. `ARCHITECTURE.md` section 6 now states the split: a schedule that cannot be met is an **error**, an unbounded interval is a **warning**. No assembler change.

## 8. `PAR`'s `sem:` string contradicts itself

```yaml
- {name: PAR, ..., sem: "rd = ra; C = even parity of ra (1 if the number of 1s is odd)"}
```

"Even parity ... 1 if the number of 1s is odd" names the value wrongly: the
quantity described is the XOR of all 16 bits, which is what `SEMANTICS.md` 6.1
says ("XOR of all 16 bits of `ra`"). The number is unambiguous, only the phrase
"even parity" is wrong. `docs/ISA.md` inherits the phrase because it is
generated from the YAML.

**Decision wanted:** reword to "C = XOR of all 16 bits of `ra` (odd parity)".

**Resolution (director, 2026-09-17):** reworded in `isa.yaml`; `docs/ISA.md` is regenerated. No assembler change.

## 9. Minor: pin indices 13, 14, 15 and 22..31 have no name

The `pin` field is 5 bits and `SEMANTICS.md` 6.3 says writes to unwritable
indices are ignored and reads of unmapped indices return 0, so a numeric pin
operand in 0..31 is always legal. The assembler therefore accepts `SETP 13, 1`
silently even though index 13 is not a pad (`ui[4..6]` are the host SPI pins).

**Decision wanted:** should the assembler warn on a pin index that `isa.yaml`
does not name? It is a one-line change, but it would be a new class of
diagnostic and it is not in any spec today.

**Resolution (director, 2026-09-17):** warn. **Implemented:** a pin operand that resolves to an index `isa.yaml` does not name (13, 14, 15, 22..31) raises a new `pin`-kind **warning** at each use, whether it arrived as a literal, a `.pins` alias or an `.equ` symbol. Named pins never warn, and the warning never suppresses the image.
