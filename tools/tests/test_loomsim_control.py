"""Control flow: CALL/RET and the return stack, DJNZ, Bcc, JP (SEMANTICS 6.2).

Check ID: L2-DIR.  The return stack is two entries deep and drops the oldest
entry on overflow, which is the case this file spends most of its lines on.
"""

import pytest

from tools.loomisa import load
from tools.loomsim import Machine
from tools.loomsim.harness import assemble, run_thread

ISA = load()


def run(program, extra=None, setup=None, **kwargs):
    return run_thread(ISA, program, extra=extra, setup=setup, **kwargs)


def pcs(records):
    """The PC of every slot, which is the whole control-flow story."""
    return [r.pc for r in records]


# ------------------------------------------------------------------ CALL/RET
def test_ret_with_an_empty_stack_falls_through():
    machine, records = run([("RET", {}), ("HALT", {})])
    assert pcs(records) == [0, 1]
    assert records[0].next_pc == 1
    assert machine.threads[0].depth == 0


def test_call_and_ret_depth_one():
    program = [("CALL", dict(abs=0x20)), ("HALT", {})]
    extra = {0x20: ISA.encode("RET")}
    machine, records = run(program, extra=extra)
    assert pcs(records) == [0, 0x20, 1]
    assert records[0].next_pc == 0x20
    assert records[1].next_pc == 1
    assert machine.threads[0].depth == 0


def test_call_depth_two_nests_and_unwinds():
    program = [("CALL", dict(abs=0x20)), ("HALT", {})]
    extra = {
        0x20: ISA.encode("CALL", abs=0x30),
        0x21: ISA.encode("RET"),
        0x30: ISA.encode("RET"),
    }
    machine, records = run(program, extra=extra)
    assert pcs(records) == [0, 0x20, 0x30, 0x21, 1]
    assert machine.threads[0].depth == 0


def test_return_stack_depth_saturates_and_drops_the_oldest_entry():
    """A third CALL keeps DEPTH at 2 and loses the outermost return address."""
    program = [("CALL", dict(abs=0x20)), ("LDI", dict(rd=0, imm=0xEE)), ("HALT", {})]
    extra = {
        0x20: ISA.encode("CALL", abs=0x30),
        0x30: ISA.encode("CALL", abs=0x40),
        0x40: ISA.encode("RET"),          # back to 0x31
        0x31: ISA.encode("RET"),          # back to 0x21
        0x21: ISA.encode("RET"),          # stack empty: falls through to 0x22
        0x22: ISA.encode("HALT"),
    }
    machine, records = run(program, extra=extra)
    assert pcs(records) == [0, 0x20, 0x30, 0x40, 0x31, 0x21, 0x22]
    # The return to address 1 was dropped when the third CALL overflowed.
    assert 1 not in pcs(records)
    assert machine.threads[0].depth == 0


def test_call_pushes_the_next_pc_and_shifts_rs0_into_rs1():
    program = [("CALL", dict(abs=0x20)), ("HALT", {})]
    extra = {0x20: ISA.encode("CALL", abs=0x30), 0x30: ISA.encode("HALT")}
    machine, records = run(program, extra=extra)
    assert machine.threads[0].rs0 == 0x21
    assert machine.threads[0].rs1 == 0x01
    assert machine.threads[0].depth == 2


def test_call_and_ret_each_cost_one_slot():
    program = [("CALL", dict(abs=0x20)), ("HALT", {})]
    extra = {0x20: ISA.encode("RET")}
    _, records = run(program, extra=extra)
    assert len(records) == 3
    assert all(r.done for r in records)


# ---------------------------------------------------------------------- DJNZ
@pytest.mark.parametrize("count,expected_iterations", [(1, 1), (2, 2), (5, 5)])
def test_djnz_loops_count_times(count, expected_iterations):
    program = [("LDI", dict(rd=1, imm=count)),
               ("DJNZ", dict(rd=1, rel=-1)),
               ("HALT", {})]
    machine, records = run(program)
    loops = [r for r in records if r.mnemonic == "DJNZ"]
    assert len(loops) == expected_iterations
    assert machine.threads[0].regs[1] == 0
    assert loops[-1].next_pc == 2                 # the last one falls through


def test_djnz_from_zero_wraps_and_branches():
    """0 - 1 is 0xFFFF, which is not zero, so the loop runs 65536 times."""
    machine = Machine(assemble(ISA, [("DJNZ", dict(rd=1, rel=-1))]), isa=ISA)
    machine.host_write_debug(0, "r1", 0)
    machine.host_set_run(0b0001)
    records = machine.run_cycles(12)
    assert records[0].val == 0xFFFF
    assert records[0].next_pc == 0
    assert [r.val for r in records] == [0xFFFF, 0xFFFE]      # it keeps looping


def test_djnz_sets_no_flags():
    program = [("DJNZ", dict(rd=1, rel=1)), ("HALT", {}), ("HALT", {})]

    def setup(machine):
        machine.host_write_debug(0, "r1", 1)
        machine.host_write_debug(0, "FLAGS", 0b111)

    _, records = run_thread(ISA, program, setup=setup)
    assert records[0].flags == 0b111
    assert records[0].val == 0
    assert records[0].next_pc == 1                # result 0: not taken


# ----------------------------------------------------------------------- Bcc
CONDITIONS = [
    ("BZ", 0b001, 0b000),
    ("BNZ", 0b000, 0b001),
    ("BC", 0b010, 0b000),
    ("BNC", 0b000, 0b010),
    ("BT", 0b100, 0b000),
    ("BNT", 0b000, 0b100),
]


@pytest.mark.parametrize("name,taken_flags,not_taken_flags", CONDITIONS)
def test_branch_conditions_both_ways(name, taken_flags, not_taken_flags):
    program = [(name, dict(rel=1)),
               ("LDI", dict(rd=0, imm=0x11)),     # not taken lands here
               ("LDI", dict(rd=1, imm=0x22)),     # taken lands here
               ("HALT", {})]

    def make_setup(value):
        def setup(machine):
            machine.host_write_debug(0, "FLAGS", value)
        return setup

    machine, records = run_thread(ISA, program, setup=make_setup(taken_flags))
    assert records[0].next_pc == 2, name
    assert pcs(records) == [0, 2, 3], name
    assert machine.threads[0].regs[0] == 0, name

    machine, records = run_thread(ISA, program, setup=make_setup(not_taken_flags))
    assert records[0].next_pc == 1, name
    assert pcs(records) == [0, 1, 2, 3], name
    assert machine.threads[0].regs[0] == 0x11, name


def test_branch_offsets_are_signed_and_wrap_within_the_pc_width():
    """rel8 is sign extended; the target is (PC + 1 + rel) mod 2^10."""
    program = [("LDI", dict(rd=1, imm=1)),
               ("HALT", {}),
               ("BZ", dict(rel=-2))]              # at PC 2, target is 1
    extra = {2: ISA.encode("BZ", rel=-2)}

    def setup(machine):
        machine.host_write_debug(0, "FLAGS", 0b001)
        machine.host_write_debug(0, "PC", 2)

    machine, records = run_thread(ISA, program, setup=setup, extra=extra)
    assert records[0].pc == 2
    assert records[0].next_pc == 1
    assert pcs(records) == [2, 1]


def test_branch_target_wraps_modulo_the_pc_space():
    program = [("HALT", {})]                      # unreached; execution starts at 1
    extra = {1: ISA.encode("BZ", rel=-4),         # at PC 1: 1 + 1 - 4 = -2 -> 0x3FE
             0x3FE: ISA.encode("HALT")}

    def setup(machine):
        machine.host_write_debug(0, "FLAGS", 0b001)
        machine.host_write_debug(0, "PC", 1)

    machine, records = run_thread(ISA, program, setup=setup, extra=extra)
    assert records[0].next_pc == 0x3FE
    assert pcs(records) == [1, 0x3FE]


# ------------------------------------------------------------------------ JP
@pytest.mark.parametrize("pad,value,taken", [(0x01, 1, True), (0x00, 1, False),
                                             (0x00, 0, True), (0x01, 0, False)])
def test_jp_branches_on_an_input_pin(pad, value, taken):
    program = [("JP", dict(pin=8, val=value, rel=1)),
               ("LDI", dict(rd=0, imm=0x11)),
               ("LDI", dict(rd=1, imm=0x22)),
               ("HALT", {})]

    def setup(machine):
        machine.set_pad_inputs(ui_in=pad)

    machine, records = run_thread(ISA, program, setup=setup)
    assert records[0].next_pc == (2 if taken else 1)


def test_jp_reads_an_output_pin_as_its_driven_value():
    """pin_in(16..21) is the PIN_OUT bit, so JP can test what this chip drives."""
    program = [("SETP", dict(pin=17, val=1)),
               ("NOP", {}),                       # let the write become visible
               ("JP", dict(pin=17, val=1, rel=1)),
               ("LDI", dict(rd=0, imm=0x11)),
               ("HALT", {})]
    machine, records = run_thread(ISA, program)
    assert records[2].next_pc == 4
    assert machine.threads[0].regs[0] == 0


def test_jp_on_a_reserved_index_reads_zero():
    program = [("JP", dict(pin=13, val=0, rel=1)),
               ("LDI", dict(rd=0, imm=0x11)),
               ("HALT", {})]
    machine, records = run_thread(ISA, program)
    assert records[0].next_pc == 2                # reserved indices read 0
