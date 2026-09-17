"""Tests for the ISA loader (check IDs L0-GEN, ISA-1 in docs/VERIFICATION.md)."""

import random

import pytest

from tools.loomisa import IsaError, generated_files, load, operand_base

ISA = load()


def test_no_overlapping_encodings():
    assert ISA.overlaps() == []
    assert ISA.check() == []


def test_every_word_decodes_to_at_most_one_instruction():
    for word in range(1 << 16):
        hits = [i.name for i in ISA.instructions if i.matches(word)]
        assert len(hits) <= 1, f"{word:04X} matches {hits}"


@pytest.mark.parametrize("name,operands,expected", [
    ("ADD", dict(rd=1, ra=2, rb=3), 0x0053),
    ("SUB", dict(rd=7, ra=0, rb=1), 0x03C1),
    ("ADDI", dict(rd=2, imm=63), 0x10BF),
    ("CMPI", dict(rd=0, imm=1), 0x1E01),
    ("LDI", dict(rd=2, imm=0x41), 0x2241),
    ("LDIH", dict(rd=2, imm=0xFF), 0x2AFF),
    ("MOV", dict(rd=1, ra=2), 0x3050),
    ("JMP", dict(abs=0x155), 0x4155),
    ("CALL", dict(abs=0x3FF), 0x47FF),
    ("RET", dict(), 0x4800),
    ("HALT", dict(), 0x4C00),
    ("BNZ", dict(rel=-2), 0x52FE),
    ("DJNZ", dict(rd=1, rel=-5), 0x61FB),
    ("JP", dict(pin=8, val=1, rel=3), 0x7A03),
    ("SETP", dict(pin=16, val=1), 0x8300),
    ("WAITD", dict(imm=1), 0x9001),
    ("WAITP", dict(pin=8, val=0, tmo=1), 0x9244),
    ("NOP", dict(), 0x9E00),
    ("CSRW", dict(csr=0x00, ra=3), 0xB818),
    ("CSRR", dict(rd=3, csr=0x09), 0xB258),
    ("SIG", dict(flag=5), 0xCD00),
    ("CLR", dict(flag=5), 0xC500),
])
def test_known_encodings(name, operands, expected):
    assert ISA.encode(name, **operands) == expected


def test_round_trip_random_operands():
    rng = random.Random(1)
    for instr in ISA.instructions:
        for _ in range(200):
            operands = {}
            for op in instr.ops:
                fld = instr.field_for(op)
                if fld.letter == "r":
                    operands[operand_base(op)] = rng.randrange(-(1 << (fld.width - 1)), 1 << (fld.width - 1))
                else:
                    operands[operand_base(op)] = rng.randrange(1 << fld.width)
            word = instr.encode(**operands)
            decoded = ISA.decode(word)
            assert decoded is not None
            assert decoded[0].name == instr.name
            assert decoded[1] == operands


def test_optional_timeout_defaults_to_zero():
    assert ISA.encode("WAITP", pin=3, val=1) == ISA.encode("WAITP", pin=3, val=1, tmo=0)


def test_range_errors():
    with pytest.raises(IsaError):
        ISA.encode("LDI", rd=8, imm=0)
    with pytest.raises(IsaError):
        ISA.encode("BZ", rel=200)
    with pytest.raises(IsaError):
        ISA.encode("ADD", rd=0, ra=0)            # missing operand
    with pytest.raises(IsaError):
        ISA.encode("NOP", rd=1)                  # unexpected operand


def test_reserved_words_decode_to_none():
    assert ISA.decode(0xE000) is None
    assert ISA.decode(0xFFFF) is None


def test_generated_files_are_current():
    for path, text in generated_files(ISA).items():
        assert path.exists(), f"{path} missing: run python -m tools.loomisa gen"
        assert path.read_text(encoding="utf-8") == text, f"{path} is stale"
