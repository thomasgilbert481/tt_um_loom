"""Operand spellings that are *not* in ``isa/isa.yaml``, plus ISA lookups.

Everything the YAML knows about - pin names, CSR names, and the symbolic
values of enumerated operands (the ``enums:`` section, ISA 0.3.0) - is read
from ``tools.loomisa`` here rather than written out again. The only spellings
this module owns are the two the YAML has no place for: the register syntax
``r0``..``r7`` and the timeout token ``T``, which are assembler surface syntax
rather than encoding.
"""

from __future__ import annotations

import re
from typing import Dict

from tools.loomisa import Isa

REGISTER_RE = re.compile(r"^r([0-7])$", re.IGNORECASE)

#: The optional trailing operand that sets an instruction's deadline-timeout bit.
TIMEOUT_TOKEN = "T"


def register_number(text: str) -> "int | None":
    match = REGISTER_RE.match(text)
    return int(match.group(1)) if match else None


def pin_table(isa: Isa) -> Dict[str, int]:
    """Upper-cased pin name -> index, from ``isa.yaml`` ``pins``."""
    return {name.upper(): number for name, number in isa.pin_by_name.items()}


def csr_table(isa: Isa) -> Dict[str, int]:
    """Upper-cased CSR name -> number, from ``isa.yaml`` ``csrs``."""
    return {name.upper(): number for name, number in isa.csr_by_name.items()}


def enum_table(isa: Isa, base: str) -> Dict[str, int]:
    """Upper-cased symbolic name -> value, from ``isa.yaml`` ``enums``.

    ``base`` is an operand base name such as ``edge`` or ``cond``.
    """
    return {name.upper(): value
            for name, value in isa.enums.get(base, {}).items()}


def enum_tables(isa: Isa) -> Dict[str, Dict[str, int]]:
    """Every enumerated operand the ISA defines, keyed by operand base name."""
    return {base: enum_table(isa, base) for base in isa.enums}


def enum_by_value(isa: Isa, base: str) -> Dict[int, str]:
    """Value -> the spelling as written in ``isa.yaml``, for the disassembler.

    The first name listed for a value wins, so an alias added to the YAML later
    never changes what the disassembler prints.
    """
    out: Dict[int, str] = {}
    for name, value in isa.enums.get(base, {}).items():
        out.setdefault(value, name)
    return out
