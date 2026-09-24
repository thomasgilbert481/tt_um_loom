"""Data memory on the golden model: ``docs/SEMANTICS.md`` 6.11 (M3 slice B,
D-027), feature ``"DMEM"``.

Slice B makes the instruction memory the data memory. ``LD rd, ra, imm5`` and
``ST rd, ra, imm5`` address the word ``a = (ra + imm5) mod 2^16`` taken modulo
``IMEM_WORDS`` (``imm5`` zero-extended) and take exactly two slots of their own
thread and no cycle of any other's:

* the **first slot** (X cycle ``x``) computes ``a``, reads ``rd`` for ``ST``,
  and commits at edge ``x+2`` ``MEM_PEND = 1``, ``MEM_LD``, ``MEM_RD = rd`` and
  ``WAIT_ACTIVE = 1`` with ``PC`` and the flags unchanged;
* the access rides the thread's next slot: its F cycle puts ``a`` on the memory
  port instead of ``PC`` (no instruction is fetched), an ``ST``'s word takes
  effect at edge ``x+3`` and is visible to fetches from cycle ``x+3``, and the
  D cycle receives the word;
* the **completion slot** (X cycle ``x+4``) decodes nothing, writes
  ``r[MEM_RD]`` for ``LD``, clears ``MEM_PEND`` and ``WAIT_ACTIVE`` and commits
  ``PC <= next`` at edge ``x+6``. ``STEPS`` counts both slots.

``MEM = {MEM_PEND, MEM_LD, MEM_RD[2:0]}`` is per-thread state at debug 0x28;
``CTRL.RESET`` and a debug ``PC`` write clear ``MEM_PEND``. ``CAPS[5]`` reads 1
and ``VERSION`` advances. Without the feature ``LD``/``ST`` are ``NOP`` +
``BADOP`` (section 9).

Timing reference, as in ``test_loomsim_be_enc.py``: every machine here burns
cycle 0 on the register preload and writes ``RUN`` in cycle 1, so thread 0's
slots have X cycles 6, 10, 14, ... The readings this file pins where the text
did not decide are in ``docs/spec-questions/loomsim-m3b.md``; the items are
named where a test depends on one.
"""

import pytest

from tools.loomisa import load
from tools.loomsim import LoomsimError, Machine
from tools.loomsim.harness import assemble

ISA = load()

FEATURES = ("DMEM",)
#: The whole M3 chip as it will be built: slice B beside M2 and slice A.
FULL = ("FIFO", "BE", "BEENC", "SETPD", "DMEM")
MEM_REG = 0x28               # HOST_PROTOCOL space 4: the access in progress
ENC_REG = 0x27
SLOT = 4                     # clocks per slot
FIRST_X = 6                  # X cycle of thread 0's first slot


def mem_word(pend, ld, rd):
    """Debug 0x28 as HOST_PROTOCOL lays it out: bits 4:0."""
    return ((pend & 1) << 4) | ((ld & 1) << 3) | (rd & 7)


def build(program, data=None, thread=0, imem_words=256, features=FEATURES,
          **kwargs):
    """A machine with ``program`` at ``thread``'s reset vector plus ``data``.

    ``RESET_PC[t] = t * (IMEM_WORDS / 4)`` (SEMANTICS 5), which is not
    ``t * 0x100`` in a memory smaller than 1024 words.
    """
    image = assemble(ISA, program, thread * (imem_words // 4))
    image.update(data or {})
    return Machine(image, features=features, imem_words=imem_words, isa=ISA,
                   **kwargs)


def run(machine, regs=None, mask=0b0001, thread=0, max_cycles=600):
    """Preload ``regs`` into ``thread``, write ``RUN`` and run to the halt.

    The preload commits at the edge that ends cycle 0 and ``RUN`` is written in
    cycle 1, so thread 0's first X cycle is :data:`FIRST_X` in every test here,
    preload or no preload.
    """
    for index, value in (regs or {}).items():
        machine.host_write_debug(thread, "r%d" % index, value)
    machine.step_cycle()
    machine.host_set_run(mask)
    records = []
    for _ in range(max_cycles):
        record = machine.step_cycle()
        if record is not None:
            records.append(record)
        if (machine.halted & mask) == mask:
            return records
    raise AssertionError("threads %X did not halt in %d cycles"
                         % (mask, max_cycles))


def of_thread(records, thread):
    return [r for r in records if r.thread == thread]


def step(machine, thread=0, limit=8):
    """One host ``STEP``; returns that slot's record, with its commit landed."""
    machine.host_step(thread)
    for _ in range(limit):
        record = machine.step_cycle()
        if record is not None and record.thread == thread:
            return record
    raise AssertionError("no slot of thread %d within %d cycles" % (thread, limit))


def probe(machine, cycles, fn):
    """``{cycle: fn(machine)}`` sampled during each of the next ``cycles``."""
    out = {}
    for _ in range(cycles):
        out[machine.cycle] = fn(machine)
        machine.step_cycle()
    return out


def first_cycle(series, predicate):
    for cycle in sorted(series):
        if predicate(series[cycle]):
            return cycle
    return None


# --------------------------------------------------------------- the build
def test_caps_bit_5_reports_the_data_memory():
    """SEMANTICS 5: ``CAPS[5]`` is the data memory, ``[15:12]`` log2 of the
    memory size.  The M2 build with the 512-word macro reads 0x909A."""
    assert Machine({}, imem_words=256, features=FEATURES, isa=ISA).caps == 0x8020
    assert Machine({}, imem_words=512, features=FEATURES, isa=ISA).caps == 0x9020
    assert Machine({}, imem_words=256, isa=ISA).caps == 0x8000
    m2 = Machine({}, imem_words=512, features=("FIFO", "BE", "SETPD"), isa=ISA)
    assert m2.caps == 0x909A
    assert Machine({}, imem_words=512, features=("FIFO", "BE", "SETPD", "DMEM"),
                   isa=ISA).caps == 0x909A | (1 << 5)


def test_version_reads_4_from_slice_b_on():
    """6.11: ``VERSION`` advances with slice B; 6.9.1 gave slice A 3 and
    HOST_PROTOCOL 0.2 gave the M2 build 2 (loomsim-m3b.md item 2)."""
    assert Machine({}, features=FEATURES, isa=ISA).host_read_ctrl("VERSION") == 4
    assert Machine({}, features=FULL, isa=ISA).host_read_ctrl("VERSION") == 4
    assert Machine({}, features=("BE", "BEENC"), isa=ISA).host_read_ctrl("VERSION") == 3
    assert Machine({}, isa=ISA).host_read_ctrl("VERSION") == 2
    assert Machine({}, features=FEATURES, version=0x0105,
                   isa=ISA).host_read_ctrl("VERSION") == 0x0105


def test_ld_and_st_stay_nops_that_set_badop_without_the_feature():
    """SEMANTICS 9 and the last line of 6.11.  The memory is untouched."""
    for name in ("LD", "ST"):
        machine = build([(name, dict(rd=3, ra=2, imm=0)), ("HALT", {})],
                        data={0x40: 0xAAAA}, features=())
        records = run(machine, regs={2: 0x40, 3: 0x5555})
        # An unbuilt feature's word carries no mnemonic in the model's record,
        # exactly as a reserved word does.
        assert [r.mnemonic for r in records] == [None, "HALT"]
        assert records[0].ir == ISA.encode(name, rd=3, ra=2, imm=0)
        assert records[0].done is True and records[0].we is False
        assert records[0].next_pc == 1
        assert machine.badop == 0b0001
        assert machine.imem[0x40] == 0xAAAA         # no store
        assert machine.threads[0].regs[3] == 0x5555  # no load
        assert machine.threads[0].steps == 2         # one slot, not two


# ------------------------------------------------------------- debug 0x28
def test_debug_0x28_reads_zero_until_slice_b_is_built():
    for features in ((), ("BE", "BEENC")):
        machine = build([("HALT", {})], features=features)
        machine.host_write_debug(0, MEM_REG, 0xFFFF)
        machine.step_cycle()
        assert machine.host_read_debug(0, MEM_REG) == 0
        assert machine.host_read_debug(0, "MEM") == 0
        assert machine.host_read_debug(0, "MEM_PEND") == 0
        assert MEM_REG not in machine.dump_debug_space(0)
    assert max(build([("HALT", {})]).dump_debug_space(0)) == MEM_REG
    assert max(build([("HALT", {})], features=FULL).dump_debug_space(0)) == MEM_REG


def test_the_mem_state_resets_to_zero():
    """SEMANTICS 5: ``MEM_PEND``, ``MEM_LD`` and ``MEM_RD`` reset to 0."""
    machine = build([("HALT", {})])
    for thread in range(4):
        assert machine.host_read_debug(thread, MEM_REG) == 0
        assert machine.host_read_debug(thread, "MEM_RD") == 0


def test_debug_0x28_round_trips_while_the_thread_is_halted():
    """HOST_PROTOCOL 0x28: ``{11'b0, MEM_PEND, MEM_LD, MEM_RD[2:0]}``,
    writable while halted; the bits above 4 read 0."""
    machine = build([("HALT", {})])
    machine.host_write_debug(0, MEM_REG, 0xFFFF)
    machine.step_cycle()
    assert machine.host_read_debug(0, MEM_REG) == mem_word(1, 1, 7)
    assert machine.host_read_debug(0, "MEM_PEND") == 1
    assert machine.host_read_debug(0, "MEM_LD") == 1
    assert machine.host_read_debug(0, "MEM_RD") == 7
    machine.host_write_debug(0, "MEM", mem_word(0, 0, 5))
    machine.step_cycle()
    assert machine.host_read_debug(0, MEM_REG) == mem_word(0, 0, 5)
    with pytest.raises(LoomsimError):
        machine.host_write_debug(0, "MEM_PEND", 1)   # a model view, read-only


def test_debug_0x28_writes_are_dropped_while_the_thread_runs():
    """SEMANTICS 7: a debug write needs a halted thread."""
    machine = build([("JMP", dict(abs=0))])
    machine.host_set_run(0b0001)
    machine.run_cycles(8)
    machine.host_write_debug(0, MEM_REG, mem_word(1, 1, 3))
    machine.step_cycle()
    assert machine.host_read_debug(0, MEM_REG) == 0
    machine.host_set_run(0)
    machine.run_cycles(8)
    machine.host_write_debug(0, MEM_REG, mem_word(1, 1, 3))
    machine.step_cycle()
    assert machine.host_read_debug(0, MEM_REG) == mem_word(1, 1, 3)


# ------------------------------------------------------------------ loads
def test_ld_reads_the_word_the_host_wrote():
    """The host loads a data image through the IMEM space it already has."""
    machine = Machine({}, features=FEATURES, imem_words=256, isa=ISA)
    for addr, word in {0: ISA.encode("LD", rd=1, ra=2, imm=0),
                       1: ISA.encode("HALT"), 0x40: 0xBEEF}.items():
        machine.host_write_imem(addr, word)
    machine.step_cycle()
    assert machine.host_read_imem(0x40) == 0xBEEF
    records = run(machine, regs={2: 0x40})
    assert machine.threads[0].regs[1] == 0xBEEF
    assert records[0].mnemonic == "LD" and machine.badop == 0


def test_the_two_slots_of_a_load():
    """6.11 steps 1 and 2, and the retire record of section 8."""
    word = 0xC0DE
    machine = build([("LD", dict(rd=5, ra=2, imm=3)), ("HALT", {})],
                    data={0x43: word})
    records = run(machine, regs={2: 0x40, 5: 0x1111})
    issue, done, halt = records
    encoded = ISA.encode("LD", rd=5, ra=2, imm=3)

    # First slot: PC held, nothing written, not done, the instruction word.
    assert issue.x_cycle == FIRST_X
    assert (issue.thread, issue.pc, issue.ir) == (0, 0, encoded)
    assert issue.done is False and issue.we is False
    assert issue.next_pc == 0                  # PC unchanged
    assert issue.flags == 0
    assert issue.mnemonic == "LD"

    # Completion slot: one slot later, no decode, the word read in tr_ir.
    assert done.x_cycle == issue.x_cycle + SLOT
    assert (done.thread, done.pc) == (0, 0)    # the instruction's PC
    assert done.ir == word                     # the word the D stage received
    assert done.done is True
    assert (done.we, done.rd, done.val) == (True, 5, word)
    assert done.next_pc == 1
    assert done.flags == 0                     # flags untouched
    assert done.mnemonic == "LD"

    assert halt.x_cycle == issue.x_cycle + 2 * SLOT
    assert machine.threads[0].regs[5] == word
    assert machine.threads[0].steps == 3       # both slots, and the HALT
    assert machine.badop == 0


def test_the_mem_state_and_wait_active_span_exactly_the_two_slots():
    """The first slot commits ``MEM_PEND``/``WAIT_ACTIVE`` at edge ``x+2`` and
    the completion slot clears them at edge ``x+6``."""
    machine = build([("LD", dict(rd=6, ra=2, imm=0)), ("HALT", {})],
                    data={0x40: 0x0042})
    for index, value in {2: 0x40}.items():
        machine.host_write_debug(0, "r%d" % index, value)
    machine.step_cycle()
    machine.host_set_run(0b0001)
    series = probe(machine, 24, lambda m: (m.host_read_debug(0, MEM_REG),
                                           m.host_read_debug(0, "WAIT_ACTIVE"),
                                           m.host_read_debug(0, "PC")))
    pending = [cycle for cycle, value in series.items() if value[0] >> 4]
    assert pending == list(range(FIRST_X + 2, FIRST_X + 6))
    assert all(series[cycle] == (mem_word(1, 1, 6), 1, 0) for cycle in pending)
    assert series[FIRST_X + 6] == (mem_word(0, 1, 6), 0, 1)   # MEM_LD/RD keep


def test_a_load_costs_exactly_two_slots():
    """6.11: exactly two slots.  The X cycle of the instruction after a ``LD``
    is 8 clocks after the ``LD``'s own, where after a ``NOP`` it is 4."""
    after_nop = run(build([("NOP", {}), ("ADDI", dict(rd=1, imm=1)),
                           ("HALT", {})]), regs={2: 0x40})
    after_ld = run(build([("LD", dict(rd=3, ra=2, imm=0)),
                          ("ADDI", dict(rd=1, imm=1)), ("HALT", {})],
                         data={0x40: 0x1234}), regs={2: 0x40})
    assert [r.mnemonic for r in after_nop] == ["NOP", "ADDI", "HALT"]
    assert [r.mnemonic for r in after_ld] == ["LD", "LD", "ADDI", "HALT"]
    assert after_nop[1].x_cycle - after_nop[0].x_cycle == SLOT
    assert after_ld[2].x_cycle - after_ld[0].x_cycle == 2 * SLOT
    # One extra slot for the whole rest of the program, and nothing more.
    assert after_ld[2].x_cycle - after_nop[1].x_cycle == SLOT
    assert after_ld[3].x_cycle - after_nop[2].x_cycle == SLOT


def test_a_load_leaves_every_flag_alone():
    """Both slots leave the flags as they were (6.11)."""
    machine = build([("CMPI", dict(rd=1, imm=0)),          # Z = 1, C = 1
                     ("LD", dict(rd=3, ra=2, imm=0)),
                     ("HALT", {})], data={0x40: 0xFFFF})
    records = run(machine, regs={1: 0, 2: 0x40})
    assert records[0].flags == 0b011
    assert [r.flags for r in records[1:3]] == [0b011, 0b011]
    assert machine.threads[0].flags == 0b011


def test_the_word_read_is_never_decoded():
    """The completion slot decodes nothing: a data word that happens to be a
    ``HALT``, and one that matches no instruction, are both just data."""
    for word, expect_badop in ((ISA.encode("HALT"), 0), (0xE000, 0)):
        machine = build([("LD", dict(rd=1, ra=2, imm=0)),
                         ("ADDI", dict(rd=4, imm=1)), ("HALT", {})],
                        data={0x40: word})
        records = run(machine, regs={2: 0x40})
        assert [r.mnemonic for r in records] == ["LD", "LD", "ADDI", "HALT"]
        assert machine.threads[0].regs[1] == word
        assert machine.threads[0].regs[4] == 1
        assert machine.badop == expect_badop


def test_a_load_into_the_register_that_held_the_address():
    """``MEM_RD`` is captured in the first slot, so ``LD r2, r2, imm`` is fine."""
    machine = build([("LD", dict(rd=2, ra=2, imm=2)), ("HALT", {})],
                    data={0x42: 0x7F7F})
    run(machine, regs={2: 0x40})
    assert machine.threads[0].regs[2] == 0x7F7F


# ----------------------------------------------------------------- stores
def test_st_then_ld_returns_the_stored_word():
    machine = build([("ST", dict(rd=3, ra=2, imm=1)),
                     ("LD", dict(rd=4, ra=2, imm=1)),
                     ("HALT", {})], data={0x41: 0x0000})
    records = run(machine, regs={2: 0x40, 3: 0xA55A})
    assert [r.mnemonic for r in records] == ["ST", "ST", "LD", "LD", "HALT"]
    assert machine.imem[0x41] == 0xA55A
    assert machine.threads[0].regs[4] == 0xA55A
    assert machine.threads[0].steps == 5


def test_the_two_slots_of_a_store():
    """As the load, except that the completion slot writes no register.

    ``tr_ir`` of a store's completion slot is "a value the memory backend
    leaves unspecified, which the harness does not compare" (section 8).  The
    model produces the word the address holds after the write, which is the
    word the store put there (loomsim-m3b.md item 3).
    """
    machine = build([("ST", dict(rd=5, ra=2, imm=3)), ("HALT", {})],
                    data={0x43: 0x1111})
    records = run(machine, regs={2: 0x40, 5: 0x2468})
    issue, done, _halt = records
    assert issue.ir == ISA.encode("ST", rd=5, ra=2, imm=3)
    assert issue.done is False and issue.we is False and issue.next_pc == 0
    assert issue.mnemonic == "ST"

    assert done.x_cycle == issue.x_cycle + SLOT
    assert done.pc == 0 and done.next_pc == 1
    assert done.done is True
    assert done.we is False                       # no register write
    assert done.flags == 0
    assert done.mnemonic == "ST"
    assert done.ir == 0x2468                      # the model's chosen value
    assert machine.imem[0x43] == 0x2468


def test_a_store_lands_at_edge_x_plus_3():
    """6.11: the write takes effect at edge ``x+3`` and is visible to fetches
    from cycle ``x+3`` on."""
    machine = build([("ST", dict(rd=5, ra=2, imm=0)), ("HALT", {})],
                    data={0x40: 0x0000})
    for index, value in {2: 0x40, 5: 0x9ABC}.items():
        machine.host_write_debug(0, "r%d" % index, value)
    machine.step_cycle()
    machine.host_set_run(0b0001)
    series = probe(machine, 20, lambda m: m.imem.get(0x40, 0))
    assert first_cycle(series, lambda word: word == 0x9ABC) == FIRST_X + 3


def test_a_thread_writes_its_own_next_instruction():
    """6.11: a word written by ``ST`` is an instruction word like any other."""
    new_word = ISA.encode("LDI", rd=1, imm=0x5A)
    program = [("LDI", dict(rd=3, imm=new_word & 0xFF)),
               ("LDIH", dict(rd=3, imm=new_word >> 8)),
               ("LDI", dict(rd=2, imm=4)),
               ("ST", dict(rd=3, ra=2, imm=0)),     # imem[4] <= LDI r1, 0x5A
               ("NOP", {}),                          # overwritten before its fetch
               ("HALT", {})]
    machine = build(program)
    records = run(machine)
    assert machine.imem[4] == new_word
    assert [r.mnemonic for r in records] == \
        ["LDI", "LDIH", "LDI", "ST", "ST", "LDI", "HALT"]
    fetched = records[5]
    assert fetched.pc == 4 and fetched.ir == new_word
    assert machine.threads[0].regs[1] == 0x5A       # the NOP never ran
    assert machine.badop == 0


# ---------------------------------------------------- address arithmetic
#: ``imem_words``, ``r2``, ``imm5``, and the address 6.11 computes.
ADDRESSES = [
    (256, 0x0004, 0, 0x004),                 # plain
    (256, 0x0000, 31, 0x01F),                # imm5 is zero-extended, max value
    (256, 0x00FE, 6, 0x004),                 # carries past the memory size
    (256, 0x0104, 0, 0x004),                 # ra above the memory size
    (256, 0xFFFF, 5, 0x004),                 # wraps at 2^16, then mod 256
    (512, 0x0004, 0, 0x004),
    (512, 0x01FE, 8, 0x006),                 # 0x206 mod 512
    (512, 0x0204, 0, 0x004),
    (512, 0xFFFF, 7, 0x006),                 # 0x10006 mod 2^16, then mod 512
]


@pytest.mark.parametrize("words,ra,imm,addr", ADDRESSES,
                         ids=["%d-%04X+%d" % (c[0], c[1], c[2]) for c in ADDRESSES])
def test_load_address_arithmetic_and_wrap(words, ra, imm, addr):
    """``a = (ra + imm5) mod 2^16``, taken modulo ``IMEM_WORDS`` (6.11)."""
    machine = build([("LD", dict(rd=1, ra=2, imm=imm)), ("HALT", {})],
                    data={addr: 0x3C3C}, imem_words=words)
    run(machine, regs={2: ra})
    assert machine.threads[0].regs[1] == 0x3C3C


@pytest.mark.parametrize("words,ra,imm,addr", ADDRESSES,
                         ids=["%d-%04X+%d" % (c[0], c[1], c[2]) for c in ADDRESSES])
def test_store_address_arithmetic_and_wrap(words, ra, imm, addr):
    machine = build([("ST", dict(rd=1, ra=2, imm=imm)), ("HALT", {})],
                    data={addr: 0x0000}, imem_words=words)
    run(machine, regs={1: 0xD00D, 2: ra})
    assert machine.imem[addr] == 0xD00D


def test_both_memory_sizes_address_their_own_top_word():
    """The access is modulo ``IMEM_WORDS``, which the build sets."""
    for words in (256, 512):
        top = words - 1
        machine = build([("ST", dict(rd=1, ra=2, imm=0)),
                         ("LD", dict(rd=3, ra=2, imm=0)),
                         ("HALT", {})], imem_words=words)
        run(machine, regs={1: 0x00FF + words, 2: top})
        assert machine.imem[top] == (0x00FF + words) & 0xFFFF
        assert machine.threads[0].regs[3] == (0x00FF + words) & 0xFFFF
        # One word past the top aliases address 0, which holds the ``ST``.
        assert machine.host_read_imem(words) == machine.imem[0]


# ------------------------------------------------------------ other threads
OTHER = [("LDI", dict(rd=1, imm=0x11)),
         ("ADDI", dict(rd=1, imm=1)),
         ("SETP", dict(pin=17, val=1)),
         ("CMPI", dict(rd=1, imm=0x12)),
         ("HALT", {})]


def _two_thread_run(thread0):
    """``thread0`` on thread 0 and :data:`OTHER` on thread 1, 512 words."""
    machine = build(thread0, data={0x40: 0xF00D}, imem_words=512)
    machine.load_image(assemble(ISA, OTHER, 512 // 4))
    records = run(machine, regs={2: 0x40}, mask=0b0011)
    return machine, records


def test_an_access_costs_no_cycle_of_another_thread():
    """6.11: two slots of its own thread and no cycle of any other thread's,
    so thread 1's slots land on exactly the same cycles either way."""
    _, with_nops = _two_thread_run([("NOP", {}), ("NOP", {}), ("NOP", {}),
                                    ("HALT", {})])
    _, with_ld = _two_thread_run([("LD", dict(rd=1, ra=2, imm=0)),
                                  ("NOP", {}), ("HALT", {})])
    _, with_st = _two_thread_run([("ST", dict(rd=1, ra=2, imm=0)),
                                  ("NOP", {}), ("HALT", {})])

    def trace(records):
        return [(r.x_cycle, r.as_tuple()) for r in of_thread(records, 1)]

    assert len(trace(with_nops)) == len(OTHER)
    assert trace(with_ld) == trace(with_nops)
    assert trace(with_st) == trace(with_nops)


def test_the_pair_is_the_same_length_whatever_the_other_threads_do():
    """"is never late": the two slots are 4 clocks apart in both builds."""
    program = [("LD", dict(rd=1, ra=2, imm=0)), ("NOP", {}), ("HALT", {})]
    alone = of_thread(run(build(program, data={0x40: 0xF00D}, imem_words=512),
                          regs={2: 0x40}), 0)
    _, busy = _two_thread_run(program)
    busy = of_thread(busy, 0)
    assert [r.as_tuple() for r in busy] == [r.as_tuple() for r in alone]
    assert [r.x_cycle for r in busy] == [r.x_cycle for r in alone]
    assert busy[1].x_cycle - busy[0].x_cycle == SLOT


def test_each_thread_has_its_own_mem_state():
    """SEMANTICS 5: ``MEM_PEND``/``MEM_LD``/``MEM_RD`` are per thread."""
    machine = build([("HALT", {})], imem_words=512)
    machine.host_write_debug(1, MEM_REG, mem_word(1, 0, 4))
    machine.host_write_debug(2, MEM_REG, mem_word(0, 1, 2))
    machine.step_cycle()
    assert [machine.host_read_debug(t, MEM_REG) for t in range(4)] == \
        [0, mem_word(1, 0, 4), mem_word(0, 1, 2), 0]


# -------------------------------------------------------------- stepping
def test_stepping_through_an_access_shows_the_mem_state_in_between():
    """SEMANTICS 7: stepping ``n`` times is observably identical to running
    ``n`` slots, so the access rides the thread's next *valid* slot and the
    thread is halted for debug between the two (loomsim-m3b.md item 1)."""
    machine = build([("LD", dict(rd=1, ra=2, imm=1)), ("HALT", {})],
                    data={0x41: 0xC0DE})
    machine.host_write_debug(0, "r2", 0x40)
    machine.step_cycle()

    issue = step(machine)
    assert issue.mnemonic == "LD" and issue.done is False
    assert machine.thread_halted_for_debug(0)
    assert machine.host_read_debug(0, MEM_REG) == mem_word(1, 1, 1)
    assert machine.host_read_debug(0, "WAIT_ACTIVE") == 1
    assert machine.host_read_debug(0, "PC") == 0
    assert machine.host_read_debug(0, "STEPS") == 1
    assert machine.threads[0].regs[1] == 0

    done = step(machine)
    assert done.done is True and done.ir == 0xC0DE
    assert (done.we, done.rd, done.val) == (True, 1, 0xC0DE)
    assert machine.host_read_debug(0, MEM_REG) == mem_word(0, 1, 1)
    assert machine.host_read_debug(0, "WAIT_ACTIVE") == 0
    assert machine.host_read_debug(0, "PC") == 1
    assert machine.host_read_debug(0, "STEPS") == 2
    assert machine.threads[0].regs[1] == 0xC0DE


def test_stepping_is_identical_to_free_running_across_an_access():
    program = [("LD", dict(rd=1, ra=2, imm=0)),
               ("ADDI", dict(rd=1, imm=1)),
               ("ST", dict(rd=1, ra=2, imm=1)),
               ("HALT", {})]
    free = run(build(program, data={0x40: 0x0007}), regs={2: 0x40})

    machine = build(program, data={0x40: 0x0007})
    machine.host_write_debug(0, "r2", 0x40)
    machine.step_cycle()
    stepped = []
    while not (machine.halted & 1):
        stepped.append(step(machine))
    assert [r.as_tuple() for r in stepped] == [r.as_tuple() for r in free]
    assert machine.imem[0x41] == 0x0008


def test_the_host_reaches_the_memory_between_the_two_slots():
    """6.11: "a data access is a valid slot".  Between the two steps no slot
    is in flight, so the IMEM space is open and the host can change the very
    word the access will read."""
    machine = build([("LD", dict(rd=1, ra=2, imm=0)), ("HALT", {})],
                    data={0x40: 0x1111})
    machine.host_write_debug(0, "r2", 0x40)
    machine.step_cycle()
    step(machine)
    assert machine.host_read_imem(0x40) == 0x1111
    machine.host_write_imem(0x40, 0x2222)
    machine.step_cycle()
    assert machine.badop == 0                   # no host access error
    done = step(machine)
    assert done.val == 0x2222 and machine.threads[0].regs[1] == 0x2222


# -------------------------------------------------- abandoning an access
def test_ctrl_reset_clears_a_pending_access():
    """6.11 and section 7: a thread never completes an access it did not
    start.  ``MEM_LD`` and ``MEM_RD`` are not named and keep their values."""
    machine = build([("LD", dict(rd=1, ra=2, imm=0)), ("HALT", {})],
                    data={0x40: 0xDEAD})
    machine.host_write_debug(0, "r2", 0x40)
    machine.step_cycle()
    step(machine)
    assert machine.host_read_debug(0, MEM_REG) == mem_word(1, 1, 1)

    machine.host_reset_thread(0)
    machine.step_cycle()
    assert machine.host_read_debug(0, MEM_REG) == mem_word(0, 1, 1)
    assert machine.host_read_debug(0, "WAIT_ACTIVE") == 0
    assert machine.host_read_debug(0, "PC") == 0        # RESET_PC[0]

    # The next slot starts the LD again rather than completing the old one.
    again = step(machine)
    assert again.mnemonic == "LD" and again.done is False
    assert machine.threads[0].regs[1] == 0


def test_a_debug_pc_write_clears_a_pending_access():
    """SEMANTICS 7: a debug ``PC`` write clears ``WAIT_ACTIVE`` and, from
    slice B, ``MEM_PEND`` (6.11)."""
    machine = build([("LD", dict(rd=1, ra=2, imm=0)),
                     ("ADDI", dict(rd=4, imm=1)), ("HALT", {})],
                    data={0x40: 0xDEAD})
    machine.host_write_debug(0, "r2", 0x40)
    machine.step_cycle()
    step(machine)
    assert machine.host_read_debug(0, MEM_REG) == mem_word(1, 1, 1)

    machine.host_write_debug(0, "PC", 1)
    machine.step_cycle()
    assert machine.host_read_debug(0, MEM_REG) == mem_word(0, 1, 1)
    assert machine.host_read_debug(0, "WAIT_ACTIVE") == 0

    machine.host_set_run(0b0001)
    for _ in range(40):
        machine.step_cycle()
        if machine.halted & 1:
            break
    assert machine.threads[0].regs[1] == 0      # the load never completed
    assert machine.threads[0].regs[4] == 1      # execution resumed at PC 1


def test_clearing_run_mid_access_leaves_it_pending_until_run_returns():
    """The first slot leaves ``WAIT_ACTIVE = 1``, exactly as a stalled wait
    does, so the access simply waits for the thread's next valid slot."""
    machine = build([("LD", dict(rd=1, ra=2, imm=0)), ("HALT", {})],
                    data={0x40: 0x4321})
    machine.host_write_debug(0, "r2", 0x40)
    machine.step_cycle()
    step(machine)
    machine.run_cycles(40)                      # nothing runs: RUN is clear
    assert machine.host_read_debug(0, MEM_REG) == mem_word(1, 1, 1)
    assert machine.threads[0].regs[1] == 0
    machine.host_set_run(0b0001)
    for _ in range(40):
        machine.step_cycle()
        if machine.halted & 1:
            break
    assert machine.threads[0].regs[1] == 0x4321
    assert machine.host_read_debug(0, MEM_REG) == mem_word(0, 1, 1)


# ------------------------------------------------------- the whole M3 build
def test_slice_b_beside_the_rest_of_the_chip():
    """``LD``/``ST`` in a build that also has the FIFOs, the bit engine, the
    slice-A encoders and the deadline latch."""
    machine = build([("LD", dict(rd=1, ra=2, imm=0)),
                     ("PUSH", dict(ra=1)),
                     ("ST", dict(rd=1, ra=2, imm=1)),
                     ("HALT", {})], data={0x40: 0x00C3}, features=FULL,
                    imem_words=512)
    records = run(machine, regs={2: 0x40})
    assert [r.mnemonic for r in records] == ["LD", "LD", "PUSH", "ST", "ST", "HALT"]
    assert machine.host_fifo_pop(0) == 0x00C3
    assert machine.imem[0x41] == 0x00C3
    assert machine.host_read_debug(0, ENC_REG) == 0
    assert machine.host_read_ctrl("CAPS") == 0x9000 | (1 << 9) | (1 << 7) \
        | (1 << 5) | (1 << 4) | (1 << 3) | 2
    assert machine.badop == 0


# ------------------------------------------ reads of memory never loaded
# docs/spec-questions/firmware-m3.md item 9: the model records a valid slot's
# read of a word no image loaded and nothing stored, and changes nothing
# else; the L3 model backend fails a scenario on it, as test/tb.v does on the
# RTL, where such a word is X.

def test_an_ld_of_a_word_never_loaded_is_recorded_and_reads_0():
    machine = build([("LD", dict(rd=1, ra=2, imm=0)), ("HALT", {})])
    run(machine, regs={1: 0x1234, 2: 0x40})
    assert machine.threads[0].regs[1] == 0
    assert [(t, a) for _, t, a in machine.unloaded_reads] == [(0, 0x40)]


def test_a_stored_word_counts_as_loaded():
    """The store lands before the completion slot's D cycle (6.11)."""
    machine = build([("ST", dict(rd=3, ra=2, imm=1)),
                     ("LD", dict(rd=4, ra=2, imm=1)),
                     ("HALT", {})])
    run(machine, regs={2: 0x40, 3: 0xA55A})
    assert machine.threads[0].regs[4] == 0xA55A
    assert machine.unloaded_reads == []


def test_a_fetch_past_the_image_is_recorded():
    machine = build([("NOP", {})])          # no HALT: the next fetch is at 1
    machine.step_cycle()
    machine.host_set_run(0b0001)
    for _ in range(12):
        machine.step_cycle()
    assert machine.unloaded_reads[0][1:] == (0, 1)


def test_the_hook_sees_the_read_as_it_happens():
    calls = []
    machine = build([("LD", dict(rd=1, ra=2, imm=0)), ("HALT", {})])
    machine.on_unloaded_read = lambda cycle, thread, addr: calls.append(
        (cycle, thread, addr))
    run(machine, regs={2: 0x40})
    assert calls == machine.unloaded_reads and len(calls) == 1


def test_a_loaded_setd_word_never_moves_td():
    """6.11: the completion slot decodes nothing; the RTL twin of this is
    test/test_mem.py's, which kills mutant loom_timer_L0146C023_stuck_79d2."""
    setd = ISA.encode("SETD", imm=200)
    csr_td = 0x0A
    machine = build([("CSRR", dict(rd=3, csr=csr_td)),
                     ("LD", dict(rd=1, ra=2, imm=0)),
                     ("CSRR", dict(rd=4, csr=csr_td)),
                     ("HALT", {})], data={0x40: setd})
    run(machine, regs={2: 0x40})
    regs = machine.threads[0].regs
    assert regs[1] == setd
    assert regs[4] == regs[3]
