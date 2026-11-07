"""SMPTE linear timecode (LTC) reference models, for firmware/ltc_gen.loom and
ltc_rx.loom.

Written from the public descriptions of SMPTE ST 12-1, not from the firmware.
ST 12-1 itself is not free and was not read: every figure below is
**secondary**, read 2026-11-07 from the English Wikipedia articles "Linear
timecode" (the frame layout, the sync word, the coding, the polarity bit)
and "SMPTE timecode" (the frame rates and the drop-frame rule). A figure
the articles only imply is marked "derived"; the director should check the
lot against ST 12-1 if a copy is to hand (QUESTIONS.md).

* A frame is 80 bits a video frame, bit 0 first, BCD digits least
  significant bit first ("Linear timecode", frame table). So the bit rate
  is 80 times the frame rate: 1920 bit/s at 24 fps, 2000 at 25, 2400 at 30,
  2397.6 at 29.97 (derived).
* Bit assignment (same article, the 24/30 fps column): frame units 0-3
  (1, 2, 4, 8), user bits 4-7, frame tens 8-9 (10, 20), drop-frame flag 10,
  colour-frame flag 11, user bits 12-15, seconds units 16-19, user bits
  20-23, seconds tens 24-26 (10, 20, 40), the polarity correction bit 27,
  user bits 28-31, minutes units 32-35, user bits 36-39, minutes tens 40-42,
  BGF0 43, user bits 44-47, hours units 48-51, user bits 52-55, hours tens
  56-57 (10, 20), BGF1 58, BGF2 59, user bits 60-63, the sync word 64-79.
  At 25 fps the article moves three flags: 27 is BGF0, 43 BGF2 and 59 the
  polarity correction bit.
* The sync word, bits 64..79 in transmission order, is 0011 1111 1111
  1101: two 0s, twelve 1s, a 0 and a 1. Twelve 1s in a row occur nowhere
  else in a frame, and "the sync pattern is preceded by 00 and followed by
  01", which is how a reader tells forward from reverse play (the article).
* Biphase mark code: a transition at the start of every bit and another in
  the middle of a 1; a 0 has none in the middle (the article). The code is
  self-clocking and polarity-free: only the spacing of the edges matters.
* The polarity correction bit makes the number of 0 bits in the frame, the
  sync word included, even, so every frame starts with the same edge
  direction (the article). With 80 bits that is the same as an even
  number of 1s.
* Drop-frame counting at 29.97 fps (30/1.001 frame/s, "SMPTE timecode"):
  frame numbers 0 and 1 of the first second of every minute are skipped,
  except when the number of minutes is divisible by ten; the drop-frame flag
  (bit 10) is set. Frames otherwise count 0..fps-1 (24, 25, 30; derived from
  the frame tens weights and the rates), seconds and minutes 0..59, hours
  0..23.
* The articles give no tolerance for the bit rate, the edge timing or the
  rise time; the tests hold the generator to its own nominal tick and the
  reader to its documented lock range instead.

Contents:

* :class:`Rate`, :class:`Time`: a frame rate (24, 25, 30, 29.97 DF) and a
  time with ``next`` and ``prev`` that apply the drop-frame rule.
* :func:`frame_bits`, :func:`decode_frame`: a time to its 80 bits and back.
* :func:`bmc_levels`: biphase mark code, two half-bit levels a bit.
* :class:`LtcSource`: a bench model that plays a list of bits onto a pin at
  any half-bit length, with a speed profile and reverse play.
* :func:`intervals_to_bits`, :func:`find_frames`: the decoder, from edge
  times to bits (edge spacing, with tracking) to frames in either direction.
* :class:`EdgeRecorder`, :class:`Wire`: the recorder and the board wire, as
  in ``tools.protomodels.irig``.
"""

from __future__ import annotations

import dataclasses
from fractions import Fraction
from typing import Callable, List, Optional, Sequence, Tuple

from .bench import Drive, Lines, Model, PinSpec, pad_of
from .irig import EdgeRecorder, Wire                     # noqa: F401 (re-exported)

BITS = 80
SYNC = [0, 0] + [1] * 12 + [0, 1]           # bits 64..79, transmission order
SYNC_WORD = sum(b << i for i, b in enumerate(SYNC))     # bit 64 at bit 0: 0xBFFC
DF_BIT, CF_BIT = 10, 11
#: (bit, weight) of the BCD digits, the 24/30 fps column of the article
FRAMES = [(0, 1), (1, 2), (2, 4), (3, 8), (8, 10), (9, 20)]
SECONDS = [(16, 1), (17, 2), (18, 4), (19, 8), (24, 10), (25, 20), (26, 40)]
MINUTES = [(32, 1), (33, 2), (34, 4), (35, 8), (40, 10), (41, 20), (42, 40)]
HOURS = [(48, 1), (49, 2), (50, 4), (51, 8), (56, 10), (57, 20)]
USER = [4, 12, 20, 28, 36, 44, 52, 60]      # first bit of user groups 1..8


def polarity_bit(fps: int) -> int:
    """Bit 27 at 24 and 30 fps, bit 59 at 25 fps (the article's two columns)."""
    return 59 if fps == 25 else 27


@dataclasses.dataclass(frozen=True)
class Rate:
    """A frame rate: ``fps`` frames counted a second (24, 25, 30) and, at 30,
    drop-frame counting at 30/1.001 frame/s."""

    fps: int
    drop: bool = False

    @property
    def frame_rate(self) -> Fraction:
        return Fraction(30000, 1001) if self.drop else Fraction(self.fps)

    @property
    def bit_rate(self) -> Fraction:
        return BITS * self.frame_rate

    def half_bit_clocks(self, clock_hz: float) -> float:
        return float(clock_hz / (2 * self.bit_rate))

    def __str__(self) -> str:
        return "29.97DF" if self.drop else "%dfps" % self.fps


R24, R25, R30, R2997 = Rate(24), Rate(25), Rate(30), Rate(30, True)
RATES = (R24, R25, R30, R2997)


def bcd(value: int) -> int:
    return (value // 10) << 4 | value % 10


@dataclasses.dataclass(frozen=True)
class Time:
    """A timecode, hours:minutes:seconds:frames."""

    hour: int
    minute: int
    second: int
    frame: int

    def valid(self, rate: Rate) -> bool:
        if not (0 <= self.hour < 24 and 0 <= self.minute < 60
                and 0 <= self.second < 60 and 0 <= self.frame < rate.fps):
            return False
        return not (rate.drop and self.dropped())

    def dropped(self) -> bool:
        """A drop-frame label that is never used ("SMPTE timecode")."""
        return self.second == 0 and self.frame < 2 and self.minute % 10 != 0

    def next(self, rate: Rate) -> "Time":
        h, m, s, f = self.hour, self.minute, self.second, self.frame + 1
        if f == rate.fps:
            f, s = 0, s + 1
            if s == 60:
                s, m = 0, m + 1
                if m == 60:
                    m, h = 0, (h + 1) % 24
        t = Time(h, m, s, f)
        if rate.drop and t.dropped():
            t = Time(h, m, s, 2)
        return t

    def prev(self, rate: Rate) -> "Time":
        h, m, s, f = self.hour, self.minute, self.second, self.frame - 1
        if rate.drop and s == 0 and f == 1 and m % 10 != 0:
            f = -1                                  # 2 back to the last frame
        if f < 0:
            f, s = rate.fps - 1, s - 1
            if s < 0:
                s, m = 59, m - 1
                if m < 0:
                    m, h = 59, (h - 1) % 24
        return Time(h, m, s, f)

    def later(self, rate: Rate, frames: int) -> "Time":
        t = self
        for _ in range(frames):
            t = t.next(rate)
        return t

    def __str__(self) -> str:
        return "%02d:%02d:%02d:%02d" % (self.hour, self.minute, self.second, self.frame)


def _put(bits: List[int], table, value: int) -> None:
    for i, w in reversed(table):
        if value >= w:
            bits[i], value = 1, value - w
    assert value == 0


def frame_bits(t: Time, rate: Rate, user: int = 0, cf: int = 0,
               fix_polarity: bool = True) -> List[int]:
    """The 80 bits of the frame carrying ``t``, bit 0 first. ``user`` holds
    the eight 4-bit user groups, group 1 in bits 3:0. BGF0..2 are 0."""
    bits = [0] * BITS
    _put(bits, FRAMES, t.frame)
    _put(bits, SECONDS, t.second)
    _put(bits, MINUTES, t.minute)
    _put(bits, HOURS, t.hour)
    for g, first in enumerate(USER):
        for k in range(4):
            bits[first + k] = (user >> (4 * g + k)) & 1
    bits[DF_BIT] = int(rate.drop)
    bits[CF_BIT] = cf & 1
    bits[64:] = SYNC
    if fix_polarity and sum(bits) % 2:
        bits[polarity_bit(rate.fps)] = 1
    return bits


@dataclasses.dataclass
class Decoded:
    time: Time
    drop: bool
    cf: int
    user: int
    polarity_ok: bool


def _get(bits: Sequence[int], table) -> int:
    return sum(w for i, w in table if bits[i])


def decode_frame(bits: Sequence[int]) -> Decoded:
    """Bits 0..63 (and the sync word, if given) back to the fields."""
    assert len(bits) in (64, BITS)
    t = Time(_get(bits, HOURS), _get(bits, MINUTES), _get(bits, SECONDS),
             _get(bits, FRAMES))
    user = 0
    for g, first in enumerate(USER):
        for k in range(4):
            user |= bits[first + k] << (4 * g + k)
    full = list(bits[:64]) + SYNC
    return Decoded(t, bool(bits[DF_BIT]), bits[CF_BIT], user, sum(full) % 2 == 0)


def words(bits: Sequence[int]) -> List[int]:
    """Bits 0..63 as four 16-bit words, bit 0 in word 0's bit 0 (the order
    ltc_rx.loom reports)."""
    return [sum(bits[16 * w + i] << i for i in range(16)) for w in range(4)]


def bits_of_words(ws: Sequence[int]) -> List[int]:
    return [(ws[i // 16] >> (i % 16)) & 1 for i in range(64)]


def bmc_levels(bits: Sequence[int], level: int = 0) -> List[int]:
    """Biphase mark code: the line level in each half-bit, two a bit. The
    line changes at the start of every bit and in the middle of a 1;
    ``level`` is the level before the first bit."""
    out = []
    for b in bits:
        level ^= 1
        out.append(level)
        if b:
            level ^= 1
        out.append(level)
    return out


def stream(t: Time, rate: Rate, count: int, user: int = 0) -> List[int]:
    """``count`` frames from ``t`` on, back to back, bit 0 of the first first."""
    out: List[int] = []
    for _ in range(count):
        out += frame_bits(t, rate, user)
        t = t.next(rate)
    return out


def reverse_play(bits: Sequence[int]) -> List[int]:
    """The bits a reader sees when the tape runs backwards. A biphase mark
    waveform played backwards is the code of the reversed bits: edges at the
    bit boundaries stay at boundaries and mid-bit edges stay mid-bit."""
    return list(reversed(bits))


class LtcSource(Model):
    """Drives a reader pin with biphase mark code. :meth:`send` queues bits
    at ``half`` clocks a half-bit (a float; the edges are rounded from the
    start of the burst, so they do not drift), each half-bit divided by
    ``speed(n)`` for half-bit ``n`` (1.1 is tape running 10 % fast).
    :attr:`edges` lists every edge's cycle; :attr:`bit_ends` the cycle at
    which each bit ends."""

    def __init__(self, pin: PinSpec, level: int = 0) -> None:
        self.pad = pad_of(pin)
        self.level = level
        self.free_at = 0
        self.edges: List[int] = []
        self.bit_ends: List[int] = []
        self._changes: List[Tuple[int, int]] = []

    def send(self, bits: Sequence[int], half: float, at: int,
             speed: Optional[Callable[[int], float]] = None) -> int:
        start = max(int(at), self.free_at)
        level = self._changes[-1][1] if self._changes else self.level
        when = float(start)
        n = 0
        for lv in bmc_levels(bits, level):
            c = start + int(round(when - start))
            if lv != level:
                self._changes.append((c, lv))
                self.edges.append(c)
                level = lv
            when += half / (speed(n) if speed else 1.0)
            n += 1
            if n % 2 == 0:
                self.bit_ends.append(start + int(round(when - start)))
        self.free_at = start + int(round(when - start))
        return start

    def drive(self, drive: Drive, cycle: int) -> None:
        changes = self._changes
        while changes and changes[0][0] <= cycle:
            self.level = changes.pop(0)[1]
        drive.set(self.pad, self.level)


# --------------------------------------------------------- checkers, decoder
def bits_from_edges(edges: Sequence[Tuple[int, int]], start: int, half: int,
                    nbits: int) -> List[int]:
    """The strict checker for a generator: every edge of the recorded pin
    must be exactly on the half-bit grid ``start + n * half``, with one at
    every bit boundary (n even); a 1 is a bit with an edge at its middle.
    Raises AssertionError at the first edge off the grid or missing."""
    at = {c for c, _ in edges}
    for c in at:
        assert (c - start) % half == 0, "edge at cycle %d is off the grid" % c
    bits = []
    for k in range(nbits):
        b0 = start + 2 * k * half
        assert b0 in at, "no edge at the start of bit %d (cycle %d)" % (k, b0)
        bits.append(int(b0 + half in at))
    return bits


def intervals_to_bits(times: Sequence[int], period: float) -> List[int]:
    """A reference decoder by edge spacing: an interval of about a bit
    period is a 0, two of about half are a 1. The period estimate follows
    every bit (a mean of the old estimate and the bit just measured), so the
    decoder tracks a tape whose speed drifts; an interval over twice the
    estimate is a gap and is skipped, a lone half interval a glitch."""
    bits: List[int] = []
    first: Optional[float] = None
    for a, b in zip(times, times[1:]):
        d = b - a
        if d >= 2 * period:
            first = None
            continue
        if d >= 0.75 * period:
            first = None
            bits.append(0)
            period = (period + d) / 2
        elif first is None:
            first = d
        else:
            bits.append(1)
            period = (period + first + d) / 2
            first = None
    return bits


def find_frames(bits: Sequence[int]) -> List[Tuple[int, bool, List[int]]]:
    """Every frame framed by two sync words 80 bits apart, in either
    direction: ``(index of the bit after the second sync, reverse, bits
    0..63 in frame order)``."""
    fwd, rev = SYNC, list(reversed(SYNC))
    found = []
    last = {False: None, True: None}
    for i in range(16, len(bits) + 1):
        tail = list(bits[i - 16:i])
        for reverse, pat in ((False, fwd), (True, rev)):
            if tail != pat:
                continue
            p = last[reverse]
            if p is not None and i - p == BITS:
                data = list(bits[p:p + 64])
                found.append((i, reverse, data[::-1] if reverse else data))
            last[reverse] = i
    return found
