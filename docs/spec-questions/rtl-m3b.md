# Spec questions from the M3 slice B RTL implementation

Written by the RTL implementer while building the data memory (`LD` and
`ST`) from `docs/SEMANTICS.md` 6.11 (with 2, 3, 5, 7, 8 and 9),
`docs/HOST_PROTOCOL.md` SPACE 1, SPACE 2 and SPACE 4, and
`docs/DECISIONS.md` D-019, D-020, D-025 and D-027, without looking at
`tools/loomsim` (VERIFICATION.md METH-1). Each entry gives the text, what is
ambiguous, the reading the RTL takes, and what another reading would change.

6.11 is short and almost everything in it is unambiguous. Most of what
follows is a corner it does not reach, and question 1 is the one place where
the most literal reading of the text is not implementable as D-027 costs it.

The architect ruled on questions 1, 2, 3, 4, 6, 7 and 8 on 2026-09-22,
during the implementation; those rulings are recorded under each entry and
the RTL takes them. 5 and 9 he did not need to: the RTL's reading of each is
the only one the text allows.

---

## 1. Where the pending access is held, and what happens if the thread's next slot is not valid

6.11 step 1: "in cycle `x+2`, which is the F cycle of the thread's next
slot, the memory port carries `a` instead of `PC`". D-027 costs this as
"one shared holding register (valid, write, 9-bit address, 16-bit data)",
which works because "a thread's F cycle immediately follows its own W edge,
one holding register serves all four threads".

Both sentences assume the thread's next slot is **valid**. A slot is valid
only if `RUN[t]` or `STEP_REQ[t]` (section 2), so a thread stepped one slot
at a time has no valid slot in cycle `x+2` at all: the first slot of the
`LD` consumed the only `STEP_REQ`, and the next one arrives many hundreds of
clocks later, over SPI. Section 7 nevertheless promises that "stepping `n`
times is observably identical to running `n` slots", and the access must
therefore still happen, with the right address, whenever that next valid
slot comes. One register shared by four threads cannot promise that: another
thread's `LD`/`ST` in between overwrites it.

**Reading taken (the architect's ruling of 2026-09-22):** the access rides
the thread's next **valid** slot, and the address, the store word and the
write bit are held **per thread** next to `MEM_PEND`, which is their valid
bit. In every F cycle `mem_fetch = valid_f & MEM_PEND[ph]` and the port
takes that thread's held access. For a running thread this is cycle `x+2`
and the cycle count of 6.11 is unchanged in every respect. For a stepped or
halted thread the access waits, so two `STEP`s run the two slots and debug
0x28 shows `MEM_PEND` in between, which is what section 7 requires.
`test_mem.py::test_mem_step_survives_another_threads_accesses` is the case
that separates the two designs: it steps thread 0 through the first slot of
an `LD`, lets thread 1 run a stream of `LD`/`ST`, and then steps thread 0
through the completion.

**Cost of the ruling against D-027's costing:** four copies of
`{address[8:0], store word[15:0], write}` instead of one, so 104 flops
instead of 26, plus a 4:1 select of 26 bits in the F stage in place of a
single register read. Nothing else moves: the adder that computes
`ra + imm5` is the X stage's, shared by all four threads, and no new
thread-number decode appears (D-019). The measured cost is in PROGRESS /
`docs/AREA.md`.

**What another reading would change:** driving the port in cycle `x+2`
unconditionally, out of one shared register, is one line shorter and matches
D-027's costing exactly, but a stepped `LD` then writes `r[MEM_RD]` with
whatever word the D stage happened to receive - in practice the `LD`
instruction itself, because the fetch it did not replace used `PC`. That
breaks section 7's stepping identity and the golden model, which has no
reason to model a shared register, would disagree.

## 2. What happens to `MEM_LD` and `MEM_RD` when the access completes

6.11 step 2: "Commit at edge `x+6`: for `LD`, `r[MEM_RD] <= the word read`;
`MEM_PEND <= 0`, `WAIT_ACTIVE <= 0`, `PC <= next`". `MEM_LD` and `MEM_RD`
are not mentioned, and section 5 gives all three a reset value of 0.

**Reading taken:** they hold. Only `MEM_PEND` is written, exactly as
`LAT_VALID` is the only field of the deadline latch that clears when a
staged write is applied while `LAT_PIN` and `LAT_VAL` keep their values
(`docs/HOST_PROTOCOL.md` 0x25, which says so in as many words). So debug
0x28 reads `{0, MEM_LD, MEM_RD}` of the last access once it has finished,
not 0. The same goes for `CTRL.RESET` and a debug `PC` write, which 6.11
and section 7 say clear `MEM_PEND`, not the other two.

**Architect's ruling of 2026-09-22:** confirmed, in these words - "CTRL.RESET
and a debug PC write clear MEM_PEND only, MEM_LD and MEM_RD keep their
values".

**What another reading would change:** clearing all three would make 0x28
read 0 between accesses, which is tidier to test but costs a second write
port's worth of logic on fields nothing reads while `MEM_PEND` is 0. No
behaviour outside 0x28 depends on it, because every use of `MEM_LD` and
`MEM_RD` is guarded by `MEM_PEND`.

## 3. The retire record of the first slot

Section 8 spells out the completion slot's record and says nothing about the
first slot's, although the first slot is a valid slot and therefore produces
one.

**Reading taken:** the first slot retires as a stalling slot does.
`tr_done = 0`, `tr_next_pc = PC` (unchanged), `tr_we = 0`, `tr_ir` is the
`LD`/`ST` word, `tr_flags` is the flags unchanged. This falls out of section
2 - "every valid slot either completes its instruction (`done`) or
re-issues the same `PC`" - and of the first slot setting `WAIT_ACTIVE`,
which is the mark of an instruction in progress (section 5). It is also why
section 8 bothers to say `tr_done` is 1 for the completion slot.

**Architect's ruling of 2026-09-22:** confirmed - "the first slot's retire
record has `tr_done = 0`, like a stalled slot".

**What another reading would change:** `tr_done = 1` on both slots would
make the pair look like two completed instructions to the harness and would
contradict `tr_next_pc` being unchanged.

## 4. Is the completion slot a valid slot for the rules that speak of valid slots?

Section 5 updates `PREV_PINS` "at the commit of every valid slot", section 4
keeps `TICK_SEEN` at "the commit edge of every valid slot of the thread",
and 6.11 says `STEPS` counts both slots. `PREV_PINS` and `TICK_SEEN` are not
mentioned in 6.11 at all.

**Reading taken:** the completion slot is a valid slot for every such rule.
`STEPS` counts it, `TICK_SEEN <= tick | (TICK_SEEN & ~seen)` with `seen` as
the completion slot read it in its X cycle, and `PREV_PINS` takes the pins
that slot saw. Those rules are written for every valid slot, without an
instruction in them, and the completion slot has no instruction to except it.

**Architect's ruling of 2026-09-22:** confirmed, for all three.

**What another reading would change:** skipping `TICK_SEEN` on the
completion slot would let a tick landing in the middle of an access be seen
twice by `WAITB 3`, which is the bug section 4 records for 2026-09-18.

## 5. What the completion slot may decode

6.11: "the thread's next slot, `MEM_PEND == 1` in X ... no instruction is
decoded. The word the D stage received in cycle `x+3` is the word read for
`LD` and unspecified for `ST`."

The word in the D stage is real data, so it decodes as *something*: for the
512-word macro every 16-bit value matches an instruction or is reserved, and
a reserved one would set `BADOP` (section 9).

**Reading taken:** "no instruction is decoded" is absolute. The RTL gates
every decode-driven effect with one signal, `dec_ok`, which replaces
`~bad_op` in all 33 places it appeared: no register write except the `LD`'s
own, no pin write, no `SFLAGS`, no FIFO push or pop, no CSR or bit-engine
write, no `HALT`, no `BADOP`, no branch, no flags. The slot commits only
what 6.11 lists for it, and `PC <= next`.

**What another reading would change:** everything. A data word that happens
to encode `HALT` or a pin write would otherwise be executed.

## 6. `CAPS[5]`, and whether SPACE 2 (DMEM) becomes a real address space

`docs/HOST_PROTOCOL.md` SPACE 2 is "Only if built (CAPS.DMEM_PRESENT). Same
rules as IMEM but writes are allowed while running (data memory is
dual-ported or arbitrated; threads win)." Slice B sets `CAPS[5]`, which
section 5 calls "data memory built". But D-027's data memory is the
instruction memory, it is single-ported, and 6.11 says the host "loads and
reads back a data image with the IMEM space it already has ... with no new
host feature".

**Reading taken:** `CAPS[5]` means `LD` and `ST` exist, not that a second
address space does. SPACE 2 stays reserved: it reads 0 and ignores writes,
exactly as before, and `docs/INTERFACES.md` now says so. Host access to the
data image goes through SPACE 1 and keeps SPACE 1's rule, that it is valid
only while `RUN == 0` and no step is in flight.

**Architect's ruling of 2026-09-22:** confirmed - "HOST_PROTOCOL space 2
(DMEM) stays reserved, reading 0 and ignoring writes: CAPS[5] means LD/ST
exist, not a second address space".

**What another reading would change:** building SPACE 2 as a second view of
the same memory would duplicate SPACE 1 for no gain, and its "writes are
allowed while running" cannot be honoured on a single-ported memory the core
fetches from every cycle.

## 7. A host write of debug 0x28

`docs/HOST_PROTOCOL.md` 0x28 is writable while halted and carries
`{MEM_PEND, MEM_LD, MEM_RD}`. The held address and store word are not in it,
because section 5 does not make them architectural state.

**Reading taken (the architect's ruling of 2026-09-22):** a host write of
0x28 sets those three fields and leaves the held address and store word
alone. So a host that plants `MEM_PEND = 1` makes the thread's next valid
slot a completion slot which reads the address last held for that thread -
undefined data if the thread never ran an `LD`/`ST`, and no write, because
the write bit is part of the held access and not of 0x28. This is the same
class of "the host can plant a state the machine cannot reach" as writing
`WAIT_ACTIVE` at 0x22 or the encoder state at 0x27, and needs no rule of its
own.

**What another reading would change:** deriving the memory-port write from
`MEM_LD` instead of holding it would let a host write of 0x28 turn a pending
`LD` into a store of a stale word. Keeping the two separate makes the
planted state inert.

## 8. What an `ST`'s completion slot receives, and `test/tb.v`

6.11 and section 8 both say the word the D stage receives for an `ST` is
unspecified and that the harness does not compare it.

**Reading taken:** whatever the memory backend returns; the RTL adds nothing.
On the macro that is the last word read (the macro does no read in a write
cycle) and on the flop array it is the old contents of the address written.

**Architect's ruling of 2026-09-22:** confirmed - "for ST the word the D
stage receives is unspecified and the harness masks it, so take whatever the
memory backend returns".

**A consequence for tests, not for the RTL:** `test/tb.v` stops the run on
an X in the D stage of a valid slot (BUGS 4), and on the flop array the old
contents of an address the host never loaded *is* X. So a program that
stores into a word nobody has written stops an RTL run on the FLOPS build.
`test_mem.py` seeds its whole data area before using it, and
`src/loom_imem.v`'s header now says why. Firmware that uses `.org`/`.word`
data (6.11's last paragraph) is loaded by the host and so is never affected.

## 9. `imm5` is unsigned, so the offset is forwards only

6.11: "`imm5` is zero-extended". Every other immediate that addresses
something (`rel8`, `rel6`) is signed.

**Reading taken:** literally zero-extended, so the offset is 0..31 and
`LD rd, ra, imm5` can only reach forwards from `ra`. `loom_decode` already
generates `f_imm = {11'd0, ir[4:0]}` for both, so the RTL had no choice to
make; it is recorded here because a reader coming from the branch encodings
will expect a sign.
