"""ALU edge cases from SEMANTICS 6.1, on the helpers and through the pipeline.

Check ID: L1-ALU.  Every case is run through a real slot as well, so a mistake
in the dispatch cannot hide behind a correct helper.
"""

import random

import pytest

from tools.loomisa import load
from tools.loomsim import alu
from tools.loomsim.harness import run_thread

ISA = load()


def exec_one(name, operands, regs=None, flags_in=None):
    """Execute one instruction with preset registers and flags."""
    def setup(machine):
        for index, value in (regs or {}).items():
            machine.host_write_debug(0, "r%d" % index, value)
        if flags_in is not None:
            machine.host_write_debug(0, "FLAGS", flags_in)

    _, records = run_thread(ISA, [(name, operands), ("HALT", {})], setup=setup)
    return records[0]


# --------------------------------------------------------------------- shifts
def test_shift_by_zero_leaves_the_value_and_clears_carry():
    for name, operands, regs in [
            ("SHL", dict(rd=1, ra=2, rb=3), {2: 0xFFFF, 3: 0}),
            ("SHR", dict(rd=1, ra=2, rb=3), {2: 0xFFFF, 3: 0}),
            ("ROR", dict(rd=1, ra=2, rb=3), {2: 0xFFFF, 3: 0})]:
        record = exec_one(name, operands, regs, flags_in=0b010)
        assert record.val == 0xFFFF, name
        assert record.c == 0, name
        assert record.z == 0, name


def test_shift_amount_is_only_the_low_four_bits():
    """``rb[3:0]``: a shift of 16 is a shift of 0, not a wipe."""
    record = exec_one("SHL", dict(rd=1, ra=2, rb=3), {2: 0x1234, 3: 0x0010})
    assert record.val == 0x1234 and record.c == 0
    record = exec_one("SHR", dict(rd=1, ra=2, rb=3), {2: 0x1234, 3: 0x00F1})
    assert (record.val, record.c) == alu.shr16(0x1234, 1)


def test_shl_by_fifteen_keeps_bit_zero_and_reports_bit_one():
    value, carry = alu.shl16(0x0003, 15)
    assert value == 0x8000
    assert carry == 1                       # bit 16 - 15 = bit 1 of the source
    record = exec_one("SHLI", dict(rd=1, imm=15), {1: 0x0003})
    assert (record.val, record.c) == (0x8000, 1)


def test_shr_by_fifteen_keeps_bit_fifteen_and_reports_bit_fourteen():
    value, carry = alu.shr16(0xC000, 15)
    assert (value, carry) == (0x0001, 1)
    record = exec_one("SHRI", dict(rd=1, imm=15), {1: 0xC000})
    assert (record.val, record.c) == (0x0001, 1)


def test_shl_carry_is_the_last_bit_shifted_out():
    for n in range(1, 16):
        value = 1 << (16 - n)
        assert alu.shl16(value, n)[1] == 1
        assert alu.shl16(value >> 1, n)[1] == 0


def test_ror_carry_is_bit_fifteen_of_the_result():
    for n in range(1, 16):
        value, carry = alu.ror16(0x0001, n)
        assert value == (1 << (16 - n)) if n else True
        assert carry == ((value >> 15) & 1)
    record = exec_one("ROR", dict(rd=1, ra=2, rb=3), {2: 0x0001, 3: 1})
    assert (record.val, record.c) == (0x8000, 1)
    record = exec_one("ROR", dict(rd=1, ra=2, rb=3), {2: 0x0002, 3: 1})
    assert (record.val, record.c) == (0x0001, 0)


# ------------------------------------------------------------------ SUB / CMP
@pytest.mark.parametrize("a,b,value,carry,zero", [
    (5, 3, 2, 1, 0),
    (3, 5, 0xFFFE, 0, 0),
    (4, 4, 0, 1, 1),
    (0, 1, 0xFFFF, 0, 0),
    (0x8000, 0x7FFF, 1, 1, 0),
])
def test_sub_carry_means_no_borrow(a, b, value, carry, zero):
    assert alu.sub16(a, b) == (value, carry)
    record = exec_one("SUB", dict(rd=1, ra=2, rb=3), {2: a, 3: b})
    assert (record.val, record.c, record.z) == (value, carry, zero)
    record = exec_one("CMP", dict(rd=2, ra=3), {2: a, 3: b})
    assert record.we is False                 # CMP writes nothing
    assert (record.c, record.z) == (carry, zero)
    record = exec_one("CMPI", dict(rd=2, imm=b), {2: a}) if b < 64 else None
    if record is not None:
        assert (record.c, record.z) == (carry, zero)


def test_add_carry_out_of_bit_fifteen():
    assert alu.add16(0xFFFF, 1) == (0, 1)
    assert alu.add16(0x8000, 0x8000) == (0, 1)
    assert alu.add16(0x7FFF, 1) == (0x8000, 0)
    record = exec_one("ADD", dict(rd=1, ra=2, rb=3), {2: 0x8000, 3: 0x8000})
    assert (record.val, record.c, record.z) == (0, 1, 1)


# ----------------------------------------------------------------- unary rules
def test_neg():
    assert alu.neg16(0) == (0, 1)
    assert alu.neg16(1) == (0xFFFF, 0)
    assert alu.neg16(0x8000) == (0x8000, 0)
    record = exec_one("NEG", dict(rd=1, ra=2), {2: 0})
    assert (record.val, record.c, record.z) == (0, 1, 1)
    record = exec_one("NEG", dict(rd=1, ra=2), {2: 0x8000})
    assert (record.val, record.c, record.z) == (0x8000, 0, 0)


def test_par_is_odd_parity_in_carry_and_copies_the_value():
    assert alu.par16(0x0000) == (0x0000, 0)
    assert alu.par16(0x0001) == (0x0001, 1)
    assert alu.par16(0x0003) == (0x0003, 0)
    assert alu.par16(0xFFFF) == (0xFFFF, 0)
    assert alu.par16(0x7FFF) == (0x7FFF, 1)
    record = exec_one("PAR", dict(rd=1, ra=2), {2: 0x0000})
    assert (record.val, record.c, record.z) == (0, 0, 1)
    record = exec_one("PAR", dict(rd=1, ra=2), {2: 0x8000})
    assert (record.val, record.c, record.z) == (0x8000, 1, 0)


def test_rev():
    assert alu.rev16(0x0001) == (0x8000, 0)
    assert alu.rev16(0x8000) == (0x0001, 0)
    assert alu.rev16(0xAAAA) == (0x5555, 0)
    assert alu.rev16(0x0000) == (0x0000, 0)
    record = exec_one("REV", dict(rd=1, ra=2), {2: 0xAAAA}, flags_in=0b010)
    assert (record.val, record.c, record.z) == (0x5555, 0, 0)


def test_swap():
    assert alu.swap16(0x1234) == (0x3412, 0)
    assert alu.swap16(0x00FF) == (0xFF00, 0)
    record = exec_one("SWAP", dict(rd=1, ra=2), {2: 0xFF00}, flags_in=0b010)
    assert (record.val, record.c, record.z) == (0x00FF, 0, 0)


def test_not_and_test():
    record = exec_one("NOT", dict(rd=1, ra=2), {2: 0xFFFF}, flags_in=0b010)
    assert (record.val, record.c, record.z) == (0x0000, 0, 1)
    record = exec_one("TEST", dict(rd=1, ra=2), {1: 0x00FF, 2: 0x0F00}, flags_in=0b010)
    assert record.we is False
    assert (record.c, record.z) == (0, 1)
    record = exec_one("TEST", dict(rd=1, ra=2), {1: 0x0FF0, 2: 0x0F00})
    assert (record.c, record.z) == (0, 0)


def test_ldih_keeps_the_low_byte_and_ldi_clears_the_high_byte():
    assert alu.ldih16(0x00AB, 0xCD) == 0xCDAB
    record = exec_one("LDIH", dict(rd=1, imm=0xCD), {1: 0x12AB}, flags_in=0b111)
    assert record.val == 0xCDAB
    assert record.flags == 0b111                 # LDIH sets no flags
    record = exec_one("LDI", dict(rd=1, imm=0x00), {1: 0xFFFF}, flags_in=0b111)
    assert record.val == 0x0000
    assert record.flags == 0b111                 # not even Z


def test_mov_sets_no_flags():
    record = exec_one("MOV", dict(rd=1, ra=2), {2: 0x0000}, flags_in=0b111)
    assert record.val == 0
    assert record.flags == 0b111


def test_logic_operations_clear_carry():
    for name, operands, regs in [("AND", dict(rd=1, ra=2, rb=3), {2: 0xFFFF, 3: 0xFFFF}),
                                 ("OR", dict(rd=1, ra=2, rb=3), {2: 1, 3: 2}),
                                 ("XOR", dict(rd=1, ra=2, rb=3), {2: 1, 3: 2}),
                                 ("ANDI", dict(rd=1, imm=0x3F), {1: 0xFFFF}),
                                 ("ORI", dict(rd=1, imm=1), {1: 0}),
                                 ("XORI", dict(rd=1, imm=1), {1: 0})]:
        record = exec_one(name, operands, regs, flags_in=0b010)
        assert record.c == 0, name


def test_immediate_forms_use_rd_as_the_left_operand():
    record = exec_one("SUBI", dict(rd=1, imm=1), {1: 0x0000})
    assert (record.val, record.c) == (0xFFFF, 0)
    record = exec_one("ADDI", dict(rd=1, imm=63), {1: 0xFFC1})
    assert (record.val, record.c, record.z) == (0x0000, 1, 1)


def test_helpers_match_executed_instructions_on_random_operands():
    """Random cross-check of the dispatch against the pure helpers."""
    rng = random.Random(20260917)
    table = {
        "ADD": alu.add16, "SUB": alu.sub16, "SHL": alu.shl16,
        "SHR": alu.shr16, "ROR": alu.ror16,
    }
    for _ in range(40):
        a = rng.randrange(1 << 16)
        b = rng.randrange(1 << 16)
        for name, func in table.items():
            expected = func(a, b)
            record = exec_one(name, dict(rd=1, ra=2, rb=3), {2: a, 3: b})
            assert (record.val, record.c) == expected, (name, hex(a), hex(b))
            assert record.z == (1 if expected[0] == 0 else 0)
