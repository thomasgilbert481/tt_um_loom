"""Sony SIRC infrared reference models, for firmware/ir_sirc_tx.loom and
ir_sirc_rx.loom.

Written from public descriptions of the protocol and a receiver datasheet,
not from the firmware. No public Sony specification exists; the figures
come from two descriptions, both read on 2026-11-05:

* SB-Projects, "Sony SIRC protocol" (sbprojects.net/knowledge/ir/sirc.php):
  a 40 kHz carrier, duty 1/4 or 1/3, pulse-width coding; a 2.4 ms header
  mark and 0.6 ms space; a 1 is a 1.2 ms mark, a 0 a 0.6 ms mark, each
  followed by 0.6 ms of space; least significant bit first; 7 command bits,
  then 5 address bits (12 bit), 8 address bits (15 bit), or 5 address and 8
  extended bits (20 bit); repeated every 45 ms start to start while a key is
  held. The page asks a receiver to wait at least 10 ms after 12 or 15 bits
  before deciding the length.
* Linux, drivers/media/rc/ir-sony-decoder.c: SONY_UNIT 600 us, header pulse
  4 units, header space 1, bit pulses 1 (0) and 2 (1), bit space 1, trailer
  space 10 units minimum; every comparison within SONY_UNIT / 2 (300 us);
  the length is the bit count, 12, 15 or 20, anything else rejected.
* The receiver module (Vishay TSOP38240, in the TSOP382.., TSOP384..
  datasheet, document 82491 rev. 2.1, figure 1 notes): output low while it
  sees a burst, falling td after it starts (7/f0 < td < 13/f0), its low
  pulse the burst length changed by -5/f0 .. +5/f0 (+-125 us at 40 kHz).
  These are the figures tools/protomodels/ir_nec.py checked on 2026-10-12
  for the 38 kHz part of the same family; the 40 kHz part's own page was
  not opened again here: UNCHECKED for 40 kHz specifically.

A mark of k units is taken as 24 k whole carrier periods (600 us at 40 kHz
is 24 periods). Measured from its first rise to its last fall, a burst of n
periods lasts between n - 1 and n periods, so a transmitter's envelope is
held to one carrier period (25 us) of nominal: :data:`ENVELOPE_TOL_US`.

Pulse lists are ``[(level, microseconds), ...]`` with ``MARK`` (carrier on;
the receiver module's output low) and ``SPACE``, starting with a mark, as in
:mod:`tools.protomodels.ir_nec`, whose generic pieces (the edge recorder,
the carrier analyser, the receiver-module model, the demodulated source and
the timing variations) this module reuses with the carrier set to 40 kHz.

* :func:`frame_bits`, :func:`frame_pulses`, :func:`bits_pulses`: the encoder;
  :func:`burst_cycles`: the carrier cycles of every mark.
* :func:`decode_pulses`: a decoder with the Linux decoder's 300 us margin.
* :func:`check_timing`: a transmitter envelope against nominal, with a
  stated tolerance.
* :func:`short_header`, :func:`bad_width`, :func:`truncate`, plus
  ``glitch``, ``vary`` and ``tsop_distort`` from ``ir_nec``: error injection.
"""

from __future__ import annotations

import dataclasses
from typing import List, Sequence, Tuple

from .ir_nec import (MARK, SPACE, CarrierAnalysis, EdgeRecorder, IrSource,  # noqa: F401
                     TsopModel as _NecTsop, glitch, start_of, truncate, tsop_distort,
                     vary)

CARRIER_HZ = 40_000.0
UNIT_US = 600.0                     # T
CYCLES_PER_UNIT = 24                # 600 us of 40 kHz
HEADER_MARK, HEADER_SPACE = 4, 1    # in units
BIT_MARK = {0: 1, 1: 2}
BIT_SPACE = 1
TRAILER_UNITS = 10                  # Linux: the minimum dark time after a frame
PERIOD_US = 45_000.0                # start to start while a key is held
MARGIN_US = UNIT_US / 2             # the Linux decoder's margin
ENVELOPE_TOL_US = 1e6 / CARRIER_HZ  # one carrier period
LENGTHS = (12, 15, 20)
ADDRESS_BITS = {12: 5, 15: 8, 20: 5}

Pulses = List[Tuple[int, float]]


def TsopModel(tx, rx, clock_hz, **kw):
    """``ir_nec.TsopModel`` at 40 kHz: td and delta in 40 kHz periods."""
    kw.setdefault("f0", CARRIER_HZ)
    return _NecTsop(tx, rx, clock_hz, **kw)


# ------------------------------------------------------------------ encoder
def frame_bits(length: int, command: int, address: int, extended: int = 0) -> List[int]:
    """The data bits in wire order: the command, the address and (20 bit)
    the extended byte, each least significant bit first."""
    if length not in LENGTHS:
        raise ValueError("SIRC frames are 12, 15 or 20 bits, not %r" % (length,))
    abits = ADDRESS_BITS[length]
    word = (command & 0x7F) | ((address & ((1 << abits) - 1)) << 7)
    if length == 20:
        word |= (extended & 0xFF) << 12
    return [(word >> i) & 1 for i in range(length)]


def bits_pulses(bits: Sequence[int]) -> Pulses:
    """A header and ``bits`` as given (any count). The last mark has no
    space after it: the dark time to the next frame is not the frame's."""
    u = UNIT_US
    out: Pulses = [(MARK, HEADER_MARK * u), (SPACE, HEADER_SPACE * u)]
    for bit in bits:
        out += [(MARK, BIT_MARK[bit] * u), (SPACE, BIT_SPACE * u)]
    out.pop()
    return out


def frame_pulses(length: int, command: int, address: int, extended: int = 0) -> Pulses:
    return bits_pulses(frame_bits(length, command, address, extended))


def burst_cycles(length: int, command: int, address: int, extended: int = 0) -> List[int]:
    """Carrier cycles of each mark: 24 a unit."""
    return [CYCLES_PER_UNIT * HEADER_MARK] + [
        CYCLES_PER_UNIT * BIT_MARK[b]
        for b in frame_bits(length, command, address, extended)]


def frame_units(length: int, command: int, address: int, extended: int = 0) -> int:
    """Units from the header's start to the end of the last mark."""
    return int(round(sum(us for _, us in frame_pulses(
        length, command, address, extended)) / UNIT_US))


# ------------------------------------------------------------------ decoder
@dataclasses.dataclass
class SircMessage:
    """What a decoder made of one header and what followed it."""

    kind: str                       # "frame" or "error"
    length: int = 0                 # 12, 15 or 20
    command: int = 0
    address: int = 0
    extended: int = 0               # 20 bit only
    bits: int = 0                   # data bits received whole
    reason: str = ""                # for an error


def _near(us: float, units: float, margin_us: float) -> bool:
    return abs(us - units * UNIT_US) <= margin_us


def decode_pulses(pulses: Sequence[Tuple[int, float]], *,
                  margin_us: float = MARGIN_US) -> List[SircMessage]:
    """Decode a pulse list as the Linux decoder does: every mark and space
    within ``margin_us`` of nominal. A mark that is not a header is skipped.
    A frame ends at a space that is not a bit space, or at the end of the
    list (the dark time after it is not in the list); its bit count must be
    12, 15 or 20."""
    out: List[SircMessage] = []
    p = list(pulses)
    i = 0
    while i < len(p):
        if p[i][0] != MARK or not _near(p[i][1], HEADER_MARK, margin_us):
            i += 1
            continue
        if not (i + 1 < len(p) and _near(p[i + 1][1], HEADER_SPACE, margin_us)):
            out.append(SircMessage("error", reason="header space"))
            i += 2
            continue
        msg, i = _bits(p, i + 2, margin_us)
        out.append(msg)
    return out


def _bits(p, k, margin_us) -> Tuple[SircMessage, int]:
    bits: List[int] = []
    while True:
        if k >= len(p):
            return SircMessage("error", bits=len(bits), reason="no mark"), k
        us = p[k][1]
        if _near(us, BIT_MARK[1], margin_us) and us > 1.5 * UNIT_US:
            bits.append(1)
        elif _near(us, BIT_MARK[0], margin_us):
            bits.append(0)
        else:
            return (SircMessage("error", bits=len(bits),
                                reason="mark of bit %d" % len(bits)), k + 1)
        k += 1
        if k < len(p) and _near(p[k][1], BIT_SPACE, margin_us):
            k += 1
            continue
        if k < len(p) and p[k][1] < TRAILER_UNITS * UNIT_US - margin_us:
            return (SircMessage("error", bits=len(bits),
                                reason="space of bit %d" % (len(bits) - 1)), k + 1)
        return _frame(bits), k + 1


def _frame(bits: Sequence[int]) -> SircMessage:
    n = len(bits)
    if n not in LENGTHS:
        return SircMessage("error", bits=n, reason="length %d" % n)
    word = sum(b << i for i, b in enumerate(bits))
    abits = ADDRESS_BITS[n]
    return SircMessage("frame", length=n, command=word & 0x7F,
                       address=(word >> 7) & ((1 << abits) - 1),
                       extended=(word >> 12) & 0xFF if n == 20 else 0, bits=n)


# ------------------------------------------------------- timing checker
def check_timing(got: Sequence[Tuple[int, float]], nominal: Sequence[Tuple[int, float]],
                 tol_us: float = ENVELOPE_TOL_US) -> List[Tuple[int, float, float]]:
    """``[(index, us, nominal us)]`` of every pulse of a transmitter's
    envelope more than ``tol_us`` from nominal, and of every mark whose
    start is more than ``tol_us`` off the unit grid (from the header's
    start). Empty when the envelope is good; a different number of pulses
    or a level out of order is reported as index -1."""
    if [lv for lv, _ in got] != [lv for lv, _ in nominal]:
        return [(-1, float(len(got)), float(len(nominal)))]
    bad = [(i, us, nus) for i, ((_, us), (_, nus)) in enumerate(zip(got, nominal))
           if abs(us - nus) > tol_us]
    for i in range(0, len(got), 2):
        t, nt = start_of(got, i), start_of(nominal, i)
        if abs(t - nt) > tol_us:
            bad.append((i, t, nt))
    return bad


# ------------------------------------------------------- error injection
def bit_index(bit: int, part: int = MARK) -> int:
    """Index in a frame's pulse list of data bit ``bit``'s mark (or space)."""
    return 2 + 2 * bit + (0 if part == MARK else 1)


def with_pulse(pulses: Sequence[Tuple[int, float]], index: int, us: float) -> Pulses:
    """``pulses`` with pulse ``index`` lasting ``us``."""
    out = list(pulses)
    out[index] = (out[index][0], us)
    return out


def short_header(pulses: Sequence[Tuple[int, float]], us: float) -> Pulses:
    """The header mark cut to ``us``."""
    return with_pulse(pulses, 0, us)


def bad_width(pulses: Sequence[Tuple[int, float]], bit: int, us: float,
              part: int = MARK) -> Pulses:
    """Data bit ``bit``'s mark (or space) lasting ``us``."""
    return with_pulse(pulses, bit_index(bit, part), us)
