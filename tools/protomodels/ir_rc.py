"""Philips RC-5 and RC-6 mode 0 reference models, for firmware/ir_rc_tx.loom
and ir_rc_rx.loom.

Written from public descriptions of the protocols and a receiver datasheet,
not from the firmware. Philips published both codes but there is no formal
public standard, and Philips's own documents could not be reached; the
figures are SB-Projects' (sbprojects.net, "knowledge/ir/rc5.php" and
"knowledge/ir/rc6.php", read 2026-11-02):

* RC-5: a 36 kHz carrier, duty 1/3 or 1/4. 14 bits, most significant
  first: two start bits S1 = S2 = 1, a toggle bit T (inverted each time a
  key is released, not on repeats), 5 address bits, 6 command bits. In
  extended RC-5 S2 carries the inverted command bit 6. Each bit lasts 64
  carrier periods (1.778 ms), bi-phase coded: a 0 is carrier in the first
  half, a 1 carrier in the second. The frame repeats every 114 ms while the
  key is held (here 64 bit times, 113.78 ms: the page gives 114).
* RC-6 mode 0: 36 kHz, duty 25 .. 50 %. One unit t is 16 carrier periods,
  444 us. A leader of 6t carrier and 2t space, a start bit 1, three mode
  bits (000), a trailer bit with halves of 2t that is the toggle, 8 address
  and 8 command bits, most significant first; each other bit has halves of
  1t, and a 1 is carrier first: the opposite sense to RC-5. After a frame
  6t of signal-free time. The repeat period is not on the page:
  UNCHECKED, taken as 240t (106.7 ms) here.
* The receiver module (Vishay TSOP38236, in the TSOP382.., TSOP384..
  datasheet, document 82491 rev. 2.1, 27 May 2025, which names it "best
  choice" for RC-5 and RC-6) outputs low while it sees a burst: the output
  falls td after the burst starts, 7/f0 < td < 13/f0, and its low pulse is
  the burst length tpi changed by -5/f0 .. +5/f0; it needs bursts of 10
  cycles or more and, after a burst of 10 to 70 cycles, a gap of 12 cycles
  or more (figure 1 notes and the data-format table, checked 2026-11-02).

Pulse lists are ``[(level, microseconds), ...]`` with ``MARK`` (carrier on;
the receiver module's output low) and ``SPACE``, starting with a mark, as in
:mod:`tools.protomodels.ir_nec`, whose generic pieces (the edge recorder, the
carrier analyser, the receiver-module model, the demodulated source and the
timing variations) this module reuses with the carrier set to 36 kHz.

* :func:`rc5_levels`, :func:`rc6_levels`: a frame as half-units, the
  encoder; :func:`frame_pulses` makes the envelope, :func:`carrier_edges`
  the full carrier.
* :func:`decode_pulses`: a decoder with stated windows.
* :func:`with_pulse`, :func:`drop_bit`, plus ``glitch``, ``vary``,
  ``tsop_distort`` and ``truncate`` from ``ir_nec``: error injection.
"""

from __future__ import annotations

import dataclasses
from typing import List, Optional, Sequence, Tuple

from .ir_nec import (MARK, SPACE, CarrierAnalysis, EdgeRecorder, IrSource,  # noqa: F401
                     TsopModel as _NecTsop, glitch, start_of, truncate, tsop_distort,
                     vary)

CARRIER_HZ = 36_000.0
UNIT_US = 16 * 1e6 / CARRIER_HZ             # RC-6 t, 444.44 us
RC5_HALF_UNITS = 2                          # an RC-5 half-bit, 32 periods
RC5_PERIOD_UNITS = 256                      # 64 bit times, 113.78 ms
RC6_PERIOD_UNITS = 240                      # 106.7 ms, UNCHECKED
RC5, RC6 = "rc5", "rc6"

Pulses = List[Tuple[int, float]]


def TsopModel(tx, rx, clock_hz, **kw):
    """``ir_nec.TsopModel`` at 36 kHz: td and delta in 36 kHz periods."""
    kw.setdefault("f0", CARRIER_HZ)
    return _NecTsop(tx, rx, clock_hz, **kw)


# ------------------------------------------------------------------ encoder
def rc5_bits(address: int, command: int, toggle: int) -> List[int]:
    """The 14 bits in wire order. Command bit 6 goes out inverted as S2."""
    s2 = 0 if command & 0x40 else 1
    word = (1 << 13) | (s2 << 12) | ((toggle & 1) << 11) | \
        ((address & 0x1F) << 6) | (command & 0x3F)
    return [(word >> i) & 1 for i in range(13, -1, -1)]


def rc5_levels(address: int, command: int, toggle: int,
               bits: Optional[Sequence[int]] = None) -> List[int]:
    """RC-5 as a list of units (1 lit, 0 dark), from S1's first half to the
    last bit's second half: 56 units. ``bits`` overrides the 14 bits."""
    out: List[int] = []
    for b in (rc5_bits(address, command, toggle) if bits is None else bits):
        first = 0 if b else 1                  # a 1 is dark, then lit
        out += [first] * RC5_HALF_UNITS + [1 - first] * RC5_HALF_UNITS
    return out


def rc6_levels(address: int, command: int, toggle: int,
               data: Optional[Sequence[int]] = None) -> List[int]:
    """RC-6 mode 0 as units, from the leader's first to the last half: 52.
    ``data`` overrides everything after the leader as a bit list whose
    fifth bit is the trailer."""
    word = ((address & 0xFF) << 8) | (command & 0xFF)
    if data is None:
        data = [1, 0, 0, 0, toggle & 1] + [(word >> i) & 1 for i in range(15, -1, -1)]
    out = [1] * 6 + [0] * 2
    for i, b in enumerate(data):
        n = 2 if i == 4 else 1                 # the trailer is twice as long
        out += [b] * n + [1 - b] * n           # a 1 is lit, then dark
    return out


def frame_pulses(levels: Sequence[int]) -> Pulses:
    """The envelope of a frame given as units: runs of equal units merged,
    a leading and a trailing dark run dropped (an idle line hides them)."""
    runs: List[List[int]] = []
    for lv in levels:
        if runs and runs[-1][0] == lv:
            runs[-1][1] += 1
        else:
            runs.append([lv, 1])
    while runs and runs[0][0] == 0:
        runs.pop(0)
    while runs and runs[-1][0] == 0:
        runs.pop()
    return [(MARK if lv else SPACE, n * UNIT_US) for lv, n in runs]


def first_mark_unit(levels: Sequence[int]) -> int:
    """Units from a frame's start to its first carrier (RC-5: 2 or 4)."""
    return list(levels).index(1)


def bursts(levels: Sequence[int]) -> List[Tuple[int, int]]:
    """``(start unit, carrier periods)`` of every burst of a frame."""
    out: List[Tuple[int, int]] = []
    for i, lv in enumerate(levels):
        if lv and (i == 0 or not levels[i - 1]):
            out.append((i, 0))
        if lv:
            out[-1] = (out[-1][0], out[-1][1] + 16)
    return out


def carrier_edges(levels: Sequence[int], clock_hz: float, *, start: int = 0,
                  duty: float = 1 / 3.0) -> List[Tuple[int, int]]:
    """The full carrier: ``(cycle, level)`` of every LED edge, each lit unit
    16 periods of 36 kHz, high for ``duty`` of a period, from cycle
    ``start`` at the frame's start."""
    period = clock_hz / CARRIER_HZ
    out: List[Tuple[int, int]] = []
    for u, lv in enumerate(levels):
        for k in range(16 if lv else 0):
            t = start + (16 * u + k) * period
            out += [(int(round(t)), 1), (int(round(t + duty * period)), 0)]
    return out


# ------------------------------------------------------------------ decoder
@dataclasses.dataclass
class RcMessage:
    """What a decoder made of one frame."""

    kind: str                       # RC5, RC6 or "error"
    address: int = 0
    command: int = 0                # RC-5: 7 bits, bit 6 the inverse of S2
    toggle: int = 0
    halves: int = 0                 # halves (RC-5) or units (RC-6) accepted
    reason: str = ""
    protocol: str = ""              # for an error: as far as known

    @property
    def extended(self) -> bool:
        return self.kind == RC5 and bool(self.command & 0x40)


def _count(us: float, q: float, tol: float, extra_us: float) -> Optional[int]:
    """n when ``us`` is n * q within ``tol * q + extra_us``, else None."""
    n = int(round(us / q))
    if n >= 1 and abs(us - n * q) <= tol * q + extra_us:
        return n
    return None


def decode_pulses(pulses: Sequence[Tuple[int, float]], *, tol: float = 0.25,
                  extra_us: float = 0.0, gap_us: float = 2_500.0) -> List[RcMessage]:
    """Decode a pulse list into frames. Every mark and space must be a whole
    number of halves (RC-5: 889 us; RC-6: units of 444 us) within ``tol`` of
    a half plus ``extra_us``; a space longer than ``gap_us`` ends a frame. A
    first mark that is neither RC-5 (one or two halves) nor an RC-6 leader
    (6 units) is skipped."""
    p = list(pulses)
    out: List[RcMessage] = []
    i = 0
    while i < len(p):
        if p[i][0] != MARK:
            i += 1
            continue
        lead = _count(p[i][1], UNIT_US, tol, extra_us)
        if lead == 6:
            msg, i = _decode(p, i + 1, RC6, [], tol, extra_us, gap_us)
        else:
            n = _count(p[i][1], 2 * UNIT_US, tol, extra_us)
            if n not in (1, 2):
                i += 1
                continue
            msg, i = _decode(p, i + 1, RC5, [0] + [1] * n, tol, extra_us, gap_us)
        out.append(msg)
    return out


def _decode(p, k, proto, halves, tol, extra_us, gap_us):
    """Collect the halves of one frame from pulse ``k`` on; returns the
    message and the index of the first pulse after it."""
    total = 28 if proto == RC5 else 44
    q = 2 * UNIT_US if proto == RC5 else UNIT_US
    err = lambda why: (RcMessage("error", halves=len(halves), reason=why,
                                 protocol=proto), k)
    if proto == RC6:
        if k >= len(p) or _count(p[k][1], q, tol, extra_us) != 2:
            return err("leader space")
        k += 1
    while len(halves) < total:
        if k >= len(p) or (p[k][0] == SPACE and p[k][1] > gap_us):
            if len(halves) == total - 1 and (k >= len(p) or p[k][0] == SPACE):
                halves.append(0)               # the last half, dark
                break
            return err("frame cut short")
        n = _count(p[k][1], q, tol, extra_us)
        if n is None or n > 3:
            return err("pulse %d out of window" % k)
        halves += [1 if p[k][0] == MARK else 0] * n
        k += 1
    if len(halves) > total:
        return err("frame too long")
    if proto == RC5:
        bits = []
        for j in range(0, 28, 2):
            if halves[j] == halves[j + 1]:
                return err("bi-phase at half %d" % j)
            bits.append(halves[j + 1])         # a 1 is dark, then lit
        word = sum(b << (13 - j) for j, b in enumerate(bits))
        command = (word & 0x3F) | (0 if word & 0x1000 else 0x40)
        return RcMessage(RC5, address=(word >> 6) & 0x1F, command=command,
                         toggle=(word >> 11) & 1, halves=28), k
    pairs = [(halves[j], halves[j + 1]) for j in range(0, 44, 2)]
    t1, t2 = pairs[4], pairs[5]
    if t1[0] != t1[1] or t2[0] != t2[1] or t1[0] == t2[0]:
        return err("trailer")
    data = []
    for j, (a, b) in enumerate(pairs[:4] + pairs[6:]):
        if a == b:
            return err("bi-phase at pair %d" % j)
        data.append(a)                         # a 1 is lit, then dark
    if data[:4] != [1, 0, 0, 0]:
        return err("start or mode")
    word = sum(b << (15 - j) for j, b in enumerate(data[4:]))
    return RcMessage(RC6, address=word >> 8, command=word & 0xFF, toggle=t1[0],
                     halves=44), k


# ------------------------------------------------------------ fault helpers
def with_pulse(pulses: Sequence[Tuple[int, float]], index: int, us: float) -> Pulses:
    """``pulses`` with pulse ``index`` lasting ``us`` (a half out of
    tolerance when ``us`` is off its nominal by more than the window)."""
    out = list(pulses)
    out[index] = (out[index][0], us)
    return out


def drop_bit(levels: Sequence[int], bit: int, proto: str) -> List[int]:
    """A frame's units with one data bit missing (RC-5: bit ``bit`` of 14;
    RC-6: data bit ``bit`` of 16): the rest close up."""
    lv = list(levels)
    if proto == RC5:
        a = 4 * bit
        return lv[:a] + lv[a + 4:]
    a = 8 + 2 * 4 + 4 + 2 * bit
    return lv[:a] + lv[a + 2:]
