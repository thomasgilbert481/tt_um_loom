"""Pure 16-bit datapath helpers for the Loom golden model.

Every function here implements one line of ``docs/SEMANTICS.md`` section 6.1
(or section 4 for :func:`reached`) and has no state of its own, so the tests can
exercise the arithmetic without building a :class:`~tools.loomsim.Machine`.

All results are ``(value, carry)`` pairs with ``value`` masked to 16 bits and
``carry`` in ``{0, 1}``.  The zero flag is never computed here: SEMANTICS says
``Z`` is "the 16-bit result is zero" for every instruction that sets it, so the
caller derives it from the returned value (except for ``CMP``/``TEST``, which
have their own rule).
"""

from __future__ import annotations

from typing import Tuple

WORD_BITS = 16
WORD_MASK = (1 << WORD_BITS) - 1
PC_BITS = 10
PC_MASK = (1 << PC_BITS) - 1

Result = Tuple[int, int]


def mask16(value: int) -> int:
    """Truncate to the 16-bit datapath width."""
    return value & WORD_MASK


def add16(a: int, b: int) -> Result:
    """``a + b``; C is the carry out of bit 15."""
    total = (a & WORD_MASK) + (b & WORD_MASK)
    return total & WORD_MASK, (total >> WORD_BITS) & 1


def sub16(a: int, b: int) -> Result:
    """``a - b``; C is 1 when ``a >= b`` unsigned (carry, not borrow)."""
    a &= WORD_MASK
    b &= WORD_MASK
    return (a - b) & WORD_MASK, 1 if a >= b else 0


def shl16(a: int, amount: int) -> Result:
    """``a << n``; C is bit ``16 - n`` of ``a`` when ``n > 0``, else 0."""
    a &= WORD_MASK
    n = amount & 0xF
    if n == 0:
        return a, 0
    return (a << n) & WORD_MASK, (a >> (WORD_BITS - n)) & 1


def shr16(a: int, amount: int) -> Result:
    """``a >> n`` logical; C is bit ``n - 1`` of ``a`` when ``n > 0``, else 0."""
    a &= WORD_MASK
    n = amount & 0xF
    if n == 0:
        return a, 0
    return (a >> n) & WORD_MASK, (a >> (n - 1)) & 1


def ror16(a: int, amount: int) -> Result:
    """Rotate right by ``n``; C is bit 15 of the *result* when ``n > 0``."""
    a &= WORD_MASK
    n = amount & 0xF
    if n == 0:
        return a, 0
    value = ((a >> n) | (a << (WORD_BITS - n))) & WORD_MASK
    return value, (value >> (WORD_BITS - 1)) & 1


def not16(a: int) -> Result:
    """``~a``; C is 0."""
    return (~a) & WORD_MASK, 0


def neg16(a: int) -> Result:
    """``0 - a``; C is 1 when ``a == 0``."""
    a &= WORD_MASK
    return (-a) & WORD_MASK, 1 if a == 0 else 0


def rev16(a: int) -> Result:
    """Bit-reverse the 16-bit word; C is 0."""
    a &= WORD_MASK
    value = 0
    for i in range(WORD_BITS):
        if (a >> i) & 1:
            value |= 1 << (WORD_BITS - 1 - i)
    return value, 0


def par16(a: int) -> Result:
    """``rd = ra``; C is the XOR of all 16 bits of ``ra`` (odd parity -> 1)."""
    a &= WORD_MASK
    return a, bin(a).count("1") & 1


def swap16(a: int) -> Result:
    """``{a[7:0], a[15:8]}``; C is 0."""
    a &= WORD_MASK
    return ((a << 8) | (a >> 8)) & WORD_MASK, 0


def ldih16(old: int, imm8: int) -> int:
    """``LDIH``: replace the high byte, keep ``rd[7:0]``."""
    return ((imm8 & 0xFF) << 8) | (old & 0xFF)


def sign_extend(value: int, bits: int) -> int:
    """Interpret ``value`` as a two's complement number of ``bits`` bits."""
    value &= (1 << bits) - 1
    if value >> (bits - 1):
        value -= 1 << bits
    return value


def branch_target(pc: int, rel: int) -> int:
    """``(PC + 1 + rel) mod 2^10``; ``rel`` is already sign-extended."""
    return (pc + 1 + rel) & PC_MASK


def next_pc(pc: int) -> int:
    """``(PC + 1) mod 2^10``."""
    return (pc + 1) & PC_MASK


def reached(now: int, deadline: int) -> bool:
    """SEMANTICS 4: ``((now - deadline) mod 2^16) < 2^15``.

    Wrap-safe: true once ``now`` is at or past ``deadline`` within half the
    16-bit range, which is how ``WAITD`` survives ``NOW`` wrapping.
    """
    return ((now - deadline) & WORD_MASK) < (1 << (WORD_BITS - 1))


# ---------------------------------------------------------------- bit engine
# SEMANTICS 6.9, manual mode.  ``msb_first`` is BE_CFG.DIR.

def be_out_bit(sr: int, msb_first: bool) -> int:
    """``SHO``'s data bit: ``DIR ? SR[15] : SR[0]`` (before any inversion)."""
    return (sr >> (WORD_BITS - 1)) & 1 if msb_first else sr & 1


def be_shift_out(sr: int, msb_first: bool) -> int:
    """``SHO``'s new ``SR``: ``DIR ? {SR[14:0], 0} : {0, SR[15:1]}``."""
    sr &= WORD_MASK
    return (sr << 1) & WORD_MASK if msb_first else sr >> 1


def be_shift_in(sr: int, bit: int, msb_first: bool) -> int:
    """``SHI``'s new ``SR``: ``DIR ? {SR[14:0], s} : {s, SR[15:1]}``."""
    sr &= WORD_MASK
    bit &= 1
    if msb_first:
        return ((sr << 1) & WORD_MASK) | bit
    return (bit << (WORD_BITS - 1)) | (sr >> 1)


def be_count(cnt: int) -> int:
    """``CNT <= (CNT == 0) ? 0 : CNT - 1`` (5 bits); ``Z`` is the result == 0."""
    cnt &= 0x1F
    return 0 if cnt == 0 else cnt - 1


def crc_step(crc: int, bit: int, poly: int) -> int:
    """One serial, MSB-first CRC step on the left-aligned 16-bit register.

    ``fb = CRC[15] ^ x; CRC <= {CRC[14:0], 1'b0} ^ (fb ? CRC_POLY : 0)``.  An
    ``n``-bit CRC lives in ``CRC[15:16-n]`` with its polynomial and initial
    value shifted left by ``16 - n`` (SEMANTICS 6.9).
    """
    crc &= WORD_MASK
    feedback = ((crc >> (WORD_BITS - 1)) ^ bit) & 1
    crc = (crc << 1) & WORD_MASK
    return crc ^ (poly & WORD_MASK) if feedback else crc
