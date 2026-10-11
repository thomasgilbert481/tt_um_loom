"""Re-pinning a program without editing it (D-035): ``min``/``max`` and
``pin(NAME)`` in expressions, and ``pins=`` / ``--pin NAME=PIN``."""

import json

import pytest

from tools.loomasm import AsmError, assemble_text
from tools.loomasm.__main__ import main
from tools.loomisa import load

ISA = load()


def src(*lines):
    return "\n".join(lines) + "\n"


def asm(text, **kwargs):
    return assemble_text(text, "t.loom", isa=ISA, **kwargs)


def errors_of(text, **kwargs):
    with pytest.raises(AsmError) as info:
        asm(text, **kwargs)
    return [d.message for d in info.value.diagnostics if d.severity == "error"]


# ------------------------------------------------------------- expressions
def test_min_and_max_take_two_or_more_values():
    program = asm(src(".equ A = min(3, 1, 2)", ".equ B = max(3, 1, 2)",
                      ".equ C = min(4, 9) + max(-1, 0)", "NOP"))
    assert (program.symbols["A"], program.symbols["B"], program.symbols["C"]) == (1, 3, 4)


@pytest.mark.parametrize("expr", ["min(5)", "max(1)", "lo8(1, 2)"])
def test_a_function_with_the_wrong_number_of_values_is_an_error(expr):
    assert errors_of(src(".equ A = " + expr, "NOP"))


def test_pin_gives_the_index_of_an_alias_or_an_isa_pin():
    program = asm(src(".pins TX = OUT0, SDA = BIDIR3",
                      ".equ A = pin(TX)", ".equ B = pin(SDA)", ".equ C = pin(bidir5)",
                      ".equ M = (1 << pin(SDA)) | (1 << pin(BIDIR0))", "NOP"))
    s = program.symbols
    assert (s["A"], s["B"], s["C"], s["M"]) == (ISA.pin_by_name["OUT0"], 3, 5, 0b1001)


def test_pin_of_an_unknown_or_later_alias_is_an_error():
    assert errors_of(src(".equ A = pin(NOPE)", "NOP"))
    assert errors_of(src(".equ A = pin(TX)", ".pins TX = OUT0", "NOP"))


def test_a_pin_alias_does_not_shadow_a_label_of_the_same_name():
    """jtag_master has a label ``tck`` and a pin alias ``TCK``: only pin()
    reads the alias, so a forward reference to the label stays a label."""
    program = asm(src(".pins TCK = OUT0", "JMP tck", "NOP", "tck: NOP"))
    assert program.words[0] == ISA.encode("JMP", abs=2)


# --------------------------------------------------------------- overrides
PROGRAM = src(".pins SCL = BIDIR0, SDA = BIDIR1",
              ".equ ODM = (1 << pin(SCL)) | (1 << pin(SDA))",
              "SETP SCL, 1",
              "SETP SDA, 0")


def test_an_override_moves_every_use_of_the_alias():
    moved = asm(PROGRAM, pins={"SCL": "BIDIR2", "sda": 3})
    assert moved.symbols["ODM"] == 0b1100
    assert moved.words[0] == ISA.encode("SETP", pin=2, val=1, lat=0)
    assert moved.words[1] == ISA.encode("SETP", pin=3, val=0, lat=0)
    plain = asm(PROGRAM)
    assert plain.symbols["ODM"] == 0b0011
    assert plain.words[0] == ISA.encode("SETP", pin=0, val=1, lat=0)


def test_an_override_for_an_alias_the_source_lacks_is_an_error():
    messages = errors_of(PROGRAM, pins={"SCK": "BIDIR3"})
    assert any("SCK" in m for m in messages)


@pytest.mark.parametrize("value", ["BIDIR9", "nonsense", 13])
def test_an_override_to_a_pin_that_is_not_a_pad_is_refused(value):
    with pytest.raises(ValueError):
        asm(PROGRAM, pins={"SCL": value})


# --------------------------------------------------------------- command line
def test_the_command_line_takes_pin_overrides(tmp_path):
    source = tmp_path / "p.loom"
    source.write_text(PROGRAM)
    out = tmp_path / "p.json"
    assert main([str(source), "--pin", "SCL=BIDIR2", "--pin", "SDA=BIDIR3",
                 "-o", str(out)]) == 0
    image = json.loads(out.read_text())
    assert image["symbols"]["ODM"] == 0b1100


@pytest.mark.parametrize("bad", ["SCL", "=BIDIR2", "SCL=", "SCL=BIDIR9", "SCK=BIDIR2"])
def test_the_command_line_refuses_a_bad_pin_override(tmp_path, bad):
    source = tmp_path / "p.loom"
    source.write_text(PROGRAM)
    assert main([str(source), "--pin", bad]) == 1
