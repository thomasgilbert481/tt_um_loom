"""ISA 0.4.0 additions: the deadline-latched SETP (``D``, SEMANTICS 6.10),
SHO/SHI setting Z, and canonical CRC presets."""

import pytest

from tools.loomasm import AsmError, assemble, disassemble
from tools.loomisa import load

ISA = load()


def test_setp_d_encoding():
    assert ISA.encode("SETP", pin=16, val=1) == 0x8300           # unchanged from 0.3.0
    assert ISA.encode("SETP", pin=16, val=1, lat=1) == 0x8301
    instr, ops = ISA.decode(0x8301)
    assert instr.name == "SETP" and ops == {"pin": 16, "val": 1, "lat": 1}


def test_setp_d_assembles_and_round_trips():
    prog = assemble("        SETP OUT0, 1, D\n        SETP OUT0, 0\n        HALT\n")
    assert prog.ok, [str(d) for d in prog.diagnostics]
    assert prog.words[0] == 0x8301
    assert prog.words[1] == 0x8100
    assert disassemble(0x8301) == "SETP OUT0, 1, D"
    assert disassemble(0x8100) == "SETP OUT0, 0"


def test_d_token_rejected_where_it_does_not_belong():
    # WAITP's optional flag is the timeout T; a D there is an unknown symbol.
    with pytest.raises(AsmError):
        assemble("        WAITP IN0, 1, D\n")


def test_sho_shi_list_z():
    assert "Z" in ISA.by_name["SHO"].flags
    assert set(ISA.by_name["SHI"].flags) >= {"Z", "T"}


def test_crc_presets_are_canonical_and_fit_their_width():
    for name, preset in ISA.crc_presets.items():
        width = preset["width"]
        assert 0 < width <= 16, name
        assert preset["poly"] < (1 << width), f"{name} poly is not in canonical {width}-bit form"
        assert preset["init"] < (1 << width), f"{name} init is not in canonical {width}-bit form"
