"""Word -> assembly text, through ``tools.loomisa.decode``.

The output is valid input for this assembler, so ``assemble -> disassemble ->
assemble`` reproduces the original words. Relative branch offsets come back as
raw numbers (never labels), which the language treats as offsets, and absolute
targets come back as numbers.
"""

from __future__ import annotations

from typing import Optional

from tools.loomisa import Isa, load, operand_base

from .names import FLAG_TOKENS, TIMEOUT_TOKEN, enum_by_value

_ISA_CACHE: "Optional[Isa]" = None


def _isa(isa: "Optional[Isa]" = None) -> Isa:
    global _ISA_CACHE
    if isa is not None:
        return isa
    if _ISA_CACHE is None:
        _ISA_CACHE = load()
    return _ISA_CACHE


def _render(base: str, value: int, isa: Isa) -> "Optional[str]":
    if base in ("rd", "ra", "rb"):
        return "r%d" % value
    if base == "pin":
        pin = isa.pins.get(value)
        return pin["name"] if pin else str(value)
    if base == "csr":
        csr = isa.csrs.get(value)
        return csr["name"] if csr else str(value)
    if base in isa.enums:                            # edge, cond, ... see enums:
        return enum_by_value(isa, base).get(value, str(value))
    if base in FLAG_TOKENS:                          # T, D: omitted when 0
        return FLAG_TOKENS[base] if value else None
    return str(value)


def disassemble(word: int, isa: "Optional[Isa]" = None) -> str:
    """Render one 16-bit word. Reserved words come back as a ``.word``."""
    isa = _isa(isa)
    word &= 0xFFFF
    decoded = isa.decode(word)
    if decoded is None:
        return ".word 0x%04X" % word
    instr, fields = decoded
    parts = []
    for op in instr.ops:
        base = operand_base(op)
        text = _render(base, fields[base], isa)
        if text is not None:
            parts.append(text)
    return instr.name + (" " + ", ".join(parts) if parts else "")
