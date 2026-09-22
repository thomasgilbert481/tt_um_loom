# Spec questions raised while writing slice B in `tools/loomsim`

Opus 5, 2026-09-22, written against `docs/SEMANTICS.md` 6.11 (M3 slice B,
D-027), `docs/HOST_PROTOCOL.md` 0.2 (space 1, space 2 and space 4 register
0x28) and `isa/isa.yaml` 0.4.0 while adding `LD`/`ST` and the data memory to
the golden model. Written from the text alone; the RTL (`src/`) and the
cocotb tests were not opened (`docs/VERIFICATION.md`, METH-1).

Same format as the M2 update in `docs/spec-questions/loomsim.md` and as
slice A's `loomsim-m3a.md`: each item is a place where the text does not
decide the answer, the model implements the reading marked **chosen**, and
none is ruled on yet. Every item is pinned by a test in
`tools/tests/test_loomsim_mem.py`.

Ordered by how likely they are to bite.

---

## 1. Which slot carries the access when the thread is not free-running

6.11 states the access twice in absolute cycles: it happens "in cycle `x+2`,
which is the F cycle of the thread's next slot", and the completion slot is
"the thread's next slot, `MEM_PEND == 1` in X, X cycle `x+4`". Both
sentences describe a thread that is running, where a slot starts every four
clocks. They do not say what happens when the slot starting at `x+2` is a
bubble, which is every case where the host stops the thread between the two
halves of the pair: a single step (`STEP` requests exactly one slot, and the
first slot consumes it in its F cycle) or a `CTRL.RUN` write of 0 that lands
between them.

Read literally, the access would go out at `x+2` whatever else is true and
the completion slot would be the bubble, so the word read would be thrown
away and `MEM_PEND` would stay set with nothing left to complete it. That
also makes `HOST_PROTOCOL` space 5's contract false for these two
instructions, and section 7 states that contract as a rule of the design:
"stepping `n` times is observably identical to running `n` slots".

**Chosen:** the access rides the thread's next **valid** slot, which is the
completion slot. The first slot leaves `WAIT_ACTIVE = 1` and holds `PC`
exactly as a stalled wait does, and the pair then behaves like every other
multi-slot sequence in the machine: for a running thread the F cycle is
`x+2`, the D cycle `x+3`, the X cycle `x+4` and the commit edge `x+6`,
literally as 6.11 says; for a stepped thread each half is one step. Between
the two the thread is halted in the sense of section 7, so debug 0x28 is
readable and writable, and 6.11's own "a data access is a valid slot" reads
as a statement about which slots are valid rather than about cycles.

**What this costs at integration:** D-027 sizes the address holding register
at one shared copy across the four threads, which works because "a thread's
F cycle immediately follows its own W edge". That argument holds only while
the access is consumed one cycle after it is loaded, i.e. only for a
free-running thread; under this reading the address and the store word must
survive an arbitrary gap, so in the model they are per-thread (item 9). If
the RTL keeps one shared register and drives the port at `x+2` regardless of
validity, then single-stepping a `LD` is simply not supported and 6.11 and
HOST_PROTOCOL space 5 need a sentence saying so. Pinned by
`test_stepping_through_an_access_shows_the_mem_state_in_between`,
`test_stepping_is_identical_to_free_running_across_an_access` and
`test_clearing_run_mid_access_leaves_it_pending_until_run_returns`.

## 2. Which builds read `VERSION == 4`

6.11 says only "`CAPS[5]` reads 1 from slice B on, and `VERSION` advances".
6.9.1 gave slice A 3 and the M2 update of `loomsim.md` (item 1) had the
register as `{major, minor}` of the host protocol, 0x0002 for
HOST_PROTOCOL 0.2. "Advances" does not say from what, so a build with slice
B but not slice A could read 3 (one past the M2 chip) or 4 (the next value
after slice A's, whether or not slice A is there).

**Chosen:** `VERSION` is a build constant that counts the slices that exist
in the document, not the ones in this build: 4 (`H.DMEM_VERSION`) whenever
`"DMEM"` is built, 3 when only the slice-A encoders are, 2 otherwise, still
overridable with `Machine(version=...)`. One number then means one feature
set on the only chip that will be taped out, and a host library that reads
4 knows both slices' registers are there. The alternative (3 for a
slice-B-only build) makes `VERSION` ambiguous between the two slices, which
is the one thing the register is for. Pinned by
`test_version_reads_4_from_slice_b_on`.

## 3. `tr_ir` of a store's completion slot

Section 8: `tr_ir` is "the word the D stage received: the word read for
`LD`, and for `ST` a value the memory backend leaves unspecified, which the
harness does not compare". The model has to produce something, and
SEMANTICS 10 requires it to be deterministic.

**Chosen:** the word the access address holds during the D cycle, which for
a store is the word the store just wrote (it lands at edge `x+3` and the D
cycle is `x+3`). It is one rule for both instructions -- the completion slot
takes whatever the memory port returns in its D cycle -- it invents no state,
and it is what a write-first single-port macro returns. If the RTL's macro
returns the old word, or holds its previous output, the harness masks the
difference and nothing else in the model depends on the value. Pinned by
`test_the_two_slots_of_a_store`.

## 4. `tr_done` of the first slot

Section 8 spells out the completion slot's record and says `tr_done` is 1
there, but says nothing about the first slot. Section 2 defines `tr_done` as
"1 if the instruction completes, 0 if it stalls" and says a slot that is not
done "re-issues the same `PC` in the thread's next slot", which the first
slot of a `LD` does not do: the next slot completes the access instead of
re-fetching.

**Chosen:** `tr_done = 0`. The instruction plainly does not complete in that
slot, `PC` is held and `WAIT_ACTIVE` is set, which is the state every stalled
slot leaves; "re-issues the same PC" is the description of the M1 wait
instructions, not the definition. Pinned by `test_the_two_slots_of_a_load`
and `test_the_two_slots_of_a_store`.

## 5. `TICK_SEEN` at the commit of a completion slot

Section 4: "At the commit edge of every valid slot of the thread,
`TICK_SEEN` keeps only what the slot did not see: `TICK_SEEN <= tick |
(TICK_SEEN & ~seen)`, where `seen` is the value the slot read in its X
cycle." A completion slot decodes nothing and therefore reads nothing, so
`seen` is either the value visible in its X cycle (every valid slot behaves
the same) or 0 (a slot that reads nothing saw nothing).

**Chosen:** the literal reading -- the completion slot is a valid slot, so it
clears the `TICK_SEEN` visible in its X cycle. It is also the cheaper
hardware (one rule at every valid slot commit, `seen` being the registered
X-cycle value), and it is the same shape as `PREV_PINS`, which section 5 says
is updated "at the commit of every valid slot ... whatever the instruction
was". The cost is that a `LD` swallows a tick that a `WAITB 3` would
otherwise have seen, exactly as any other instruction does. Worth a sentence
in 6.11 either way, because it is a place where the RTL could differ without
any test noticing until a UART frame drifts.

## 6. What a host write of debug 0x28 does to the address

0x28 carries `{MEM_PEND, MEM_LD, MEM_RD[2:0]}` and HOST_PROTOCOL makes it
"writable while halted"; the access address and the word an `ST` will write
are in neither the register nor section 5's state table, so a host write
that sets `MEM_PEND` names no address.

**Chosen:** a 0x28 write sets the three architectural bits and leaves the
address and the store word alone, as a debug 0x25 write leaves `LAT_PIN` and
`LAT_VAL` alone when it clears `LAT_VALID` (M2 item 11). A debugger can
therefore cancel an access (write 0) or save and restore the visible state;
a debugger that *starts* an access by hand gets the address left by the
thread's last one, or 0 after reset. The alternative would be to widen 0x28,
which HOST_PROTOCOL does not do. Pinned by
`test_debug_0x28_round_trips_while_the_thread_is_halted`.

## 7. `CTRL.RESET` and a debug `PC` write clear `MEM_PEND` and nothing else

6.11 and section 7 both say "clear `MEM_PEND`" and name neither `MEM_LD` nor
`MEM_RD`.

**Chosen:** literal -- the other two keep their values, so 0x28 reads
`{0, ld, rd}` after a reset, exactly as 0x25 reads `{0, val, pin}` after a
staged write lands (M2 item 11). Nothing can act on them while `MEM_PEND` is
0. Pinned by `test_ctrl_reset_clears_a_pending_access` and
`test_a_debug_pc_write_clears_a_pending_access`.

## 8. Does HOST_PROTOCOL space 2 come alive with `CAPS[5]`?

HOST_PROTOCOL 0.2 (2026-09-18) has a **SPACE 2: DMEM**: "Only if built
(CAPS.DMEM_PRESENT). Same rules as IMEM but writes are allowed while running
(data memory is dual-ported or arbitrated; threads win)." D-027 (four days
later) rejects exactly that memory: there is one single-port macro, "the host
loads and dumps it through the IMEM space it already has", and 6.11 repeats
it -- "Host IMEM access stays as section 7 says", "with no new host feature".
Section 7 still lists "FIFO and DMEM spaces read 0 and ignore writes until
M2" among the M1 builds.

**Chosen:** space 2 stays unimplemented in a slice-B build: reads return 0,
writes are ignored, and the model offers no space-2 API. `CAPS[5]` means
`LD`/`ST` exist, not that a second address space does. The space-2 paragraph
of HOST_PROTOCOL predates D-027 and should say "reserved; data memory is the
instruction memory, reached through space 1 (SEMANTICS 6.11)" -- in
particular "writes are allowed while running" is now false, since the macro
is the one the core fetches from every cycle. This is a documentation fix
rather than a model choice, but `tools/loomhost` still defines
`SPACE_DMEM = 2`, so it wants a ruling before a transport tries to use it.

## 9. Where the access address and the store word live

Section 5's state table lists `MEM_PEND`, `MEM_LD` and `MEM_RD` and nothing
else; D-027 puts the address and the data in "one shared holding register
(valid, write, 9-bit address, 16-bit data)". The model has to keep them
somewhere, and the choice is visible only through item 1's gap.

**Chosen:** per thread (`ThreadState.mem_addr`, `ThreadState.mem_data`), not
host-visible and not in any dump, so the model matches a shared register
exactly for free-running threads and still survives a stepped one. If the
RTL keeps one shared copy, the two agree on everything a co-simulation run
can compare unless a thread is stopped mid-pair, which is item 1's question.

## 10. The model's `mnemonic` on a completion slot

`RetireRecord.mnemonic` is a model-only field ("the mnemonic of the decoded
instruction, or `None` for a reserved word"), and a completion slot decodes
nothing.

**Chosen:** `"LD"` or `"ST"` from `MEM_LD`, because the slot belongs to that
instruction and a trace of `["LD", "LD", "HALT"]` reads better than one with
a hole in it. The field is not in the RTL's record, so nothing compares it.

---

## Consequences worth stating rather than questions

* **A pending access holds the IMEM space open, a running one closes it.**
  "A data access is a valid slot", so while a thread runs through a pair the
  host's space-1 reads and writes are refused and set `BADOP[15]`, as section
  7 says. Between two steps of a pair nothing is in flight and `RUN` is 0, so
  the space is open -- the host can read or even rewrite the word the access
  is about to fetch, and the completion slot reads the new value. Pinned by
  `test_the_host_reaches_the_memory_between_the_two_slots`.

* **No two threads write the memory at the same edge.** A thread's store
  lands at edge `x+3`, and `x ≡ t+2 (mod 4)`, so its write edge is `≡ t+1
  (mod 4)`: the four threads write on four different phases. The single port
  is never contended, which is the point of hanging the access off the
  thread's own F cycle.

* **"Visible to fetches from cycle `x+3`" includes a fetch already in
  flight.** A slot whose D cycle is `x+3` receives the word from the memory
  in that cycle and therefore sees the new value, even though it put its
  address on the port in cycle `x+2`, before the write. The model reads the
  memory in the D cycle, after the edge, so it has this for free.

* **An unbuilt `LD`/`ST` costs one slot, not two.** Section 9 makes it a
  `NOP`, and a `NOP` is one slot, so `STEPS` counts 1. The assembler's
  deadline checker counts two for the `two_slot` timing class whatever the
  build (`isa.yaml`), so an M1 or M2 image is checked against a schedule one
  slot per `LD`/`ST` slacker than it will run. That is the safe direction,
  and there is no reason to put `LD`/`ST` in such an image, but it is a real
  difference between the timing class and the model.

* **The word a `LD` returns is never decoded.** It goes into `tr_ir` and into
  `r[MEM_RD]` and nowhere else, so a data word that happens to encode `HALT`
  or that matches no instruction changes nothing and sets no `BADOP`. Pinned
  by `test_the_word_read_is_never_decoded`.

* **A completion slot is a valid slot for everything else too.** It
  increments `STEPS`, it updates `PREV_PINS` with the pins it saw in X, it
  counts for "no valid slot in flight", and it consumes a `STEP_REQ` in its F
  cycle like any other slot.

* **`CAPS` for the M3 chip.** 6.11 gives `CAPS[5]`; with the 512-word macro
  and the M2 features the register reads 0x90BA, and 0x92BA with slice A's
  `CAPS[9]` as well. SEMANTICS 5's worked values (0x8000, 0x909A, 0x929A)
  want the two slice-B numbers added.

* **`LD rd, rd, imm` is well defined.** `MEM_RD` and the address are both
  captured in the first slot, so a load into the register that held the
  address works; the completion slot writes `r[MEM_RD]` from state, not from
  a re-read of the instruction. Pinned by
  `test_a_load_into_the_register_that_held_the_address`.

---

## Rulings (Fable, 2026-09-22, at integration)

Item 1 is the one that mattered, and the model's reading is now the text:
the access rides the thread's next valid slot, the address and the store
word are held per thread, and stepping stays identical to running (SEMANTICS
6.11 and section 5 updated; D-027 amended; the RTL, which had started from
one shared register, was told the same evening). Items 2 to 7 and 10 stand
as chosen and are now written into 6.11 and section 8 where the text was
silent (`tr_done = 0` for the first slot; the completion slot is a valid
slot for `STEPS`, `TICK_SEEN` and `PREV_PINS`; a 0x28 write sets the three
bits only; `CTRL.RESET` and a debug PC write clear `MEM_PEND` only; `VERSION`
4 whenever slice B is built). Item 8: HOST_PROTOCOL's space 2 is reserved,
as the model has it; the paragraph is rewritten. Item 9 follows from item 1.
