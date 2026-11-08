"""Magnetic stripe card (ISO/IEC 7811 F2F) reference models, for
firmware/magstripe_reader.loom and magstripe_emu.loom.

Written from public descriptions of the track format, not from the
firmware. ISO/IEC 7811-2 and 7813 are not free and were not read: every
figure below is **secondary**, read 2026-11-07 from

* [MT] MagTek, "I/O Interface for TTL Magnetic Stripe Readers", P/N
  99875148 (the TTL reader manual), read as the manualsdir.com pages:
  p. 7 "Track 1 data format", p. 8 "Tracks 2 and 3 data format" (with the
  parity and LRC rules), p. 34 "Detecting start sentinel", "Bidirectional
  reading", "Converting card data to ASCII";
* [WP] Wikipedia, "Magnetic stripe card" (densities, sentinels, the
  track 2 character set);
* [P] US patent 5,434,400, "Method and apparatus for decoding F2F signals
  read from a magnetic data carrier", background (F2F, hand swipes);
* [IC] the MagTek 21006516 read IC spec sheet (TTL manual appendix B,
  p. 27): 150 to over 12,000 F2F bit/s, "automatic acquisition and
  tracking". No speed-change tolerance is given by any of them.

Items, each with its source:

* F2F ("Aiken biphase", [P], and the brief): every bit cell has the same
  length; a flux transition at every cell boundary and one more in the
  middle of a 1. The code is polarity-free, and played backwards it is the
  code of the reversed bits.
* Density [WP]: track 2 75 bpi, tracks 1 and 3 210 bpi.
* Track 2 [MT p. 8]: clocking zeros (about 22), start sentinel hex B, up to
  37 data characters, end sentinel hex F, LRC, trailing zeros (22 or more).
  "5-bit (4 plus parity)" characters, odd parity, written "starting with
  the least significant bit and ending with the parity bit" (1, 2, 4, 8,
  P). ASCII = value + 30h [MT p. 34]: '0'..'9', ':', ';', '<', '=', '>',
  '?' [WP]; ';' is the start sentinel, '?' the end, '=' the field
  separator [WP].
* Track 1 [MT p. 7]: about 62 clocking zeros, '%' start sentinel, up to 76
  characters, '?' end sentinel, LRC, "7-bit (6 plus parity)", odd parity,
  LSB first; ASCII = value + 20h [MT p. 34], so '%' is 05h and '?' 1Fh;
  '^' the field separator [WP].
* LRC [MT p. 8]: for the LRC "parity bits are ignored and each column must
  contain an even number of ones": the XOR of the data bits of every
  character from the start sentinel to the end sentinel. The LRC character
  "receives its own odd-parity bit". (The sentinels counted: MT's rule is
  per column of the whole record; derived, see QUESTIONS.md Q1.)
* Start-sentinel search and direction [MT p. 34]: the track is searched
  for the start sentinel; read "in the other direction" if the checks
  fail, and "the card is unreadable only after the software has tried
  both directions". The LRC can look like a start sentinel.
* Swipe speed: a hand swipe changes speed within the swipe [P]; this model
  gives a speed profile (accelerating, slowing, both) and jitter.

Contents: :data:`TRACK1`, :data:`TRACK2`; :func:`encode` (text to bits,
with errors on request); :func:`f2f_levels`; :class:`SwipeSource` (a bench
model playing bits at a speed profile with jitter); :func:`intervals_to_bits`
and :func:`decode` (an independent decoder, last-window cell estimate [P]
and MagTek's bidirectional framing); :func:`report` (the words
magstripe_reader.loom's header promises); ``bits_from_edges``,
``EdgeRecorder``, ``Wire`` re-exported from the LTC and IRIG models.
"""

from __future__ import annotations

import dataclasses
import random
from typing import Callable, List, Optional, Sequence, Tuple

from .bench import Drive, Model, PinSpec, pad_of
from .irig import EdgeRecorder, Wire                     # noqa: F401 (re-exported)
from .ltc import bits_from_edges                          # noqa: F401 (same code)

# status bits of a report (firmware/magstripe_reader.loom header)
E_PARITY, E_LRC, E_NO_ES, E_NO_SS = 1, 2, 4, 8


@dataclasses.dataclass(frozen=True)
class Track:
    name: str
    data_bits: int          # 4 or 6, parity follows
    offset: int             # ASCII = value + offset
    ss: int                 # start sentinel value
    es: int                 # end sentinel value
    bpi: int
    zeros: int              # typical clocking zeros [MT]
    max_chars: int          # data characters [MT]

    def value(self, ch: str) -> int:
        v = ord(ch) - self.offset
        assert 0 <= v < (1 << self.data_bits), "%r is not in track %s's set" % (ch, self.name)
        return v

    def char_bits(self, v: int, parity_ok: bool = True) -> List[int]:
        """``v`` LSB first, then the odd parity bit (inverted on request)."""
        bits = [(v >> i) & 1 for i in range(self.data_bits)]
        p = 1 - (sum(bits) & 1)
        return bits + [p if parity_ok else 1 - p]


TRACK2 = Track("2", 4, 0x30, 0x0B, 0x0F, 75, 22, 37)
TRACK1 = Track("1", 6, 0x20, 0x05, 0x1F, 210, 62, 76)


def lrc(values: Sequence[int], track: Track) -> int:
    """The LRC value: every data column even over ``values`` (start
    sentinel to end sentinel) and the LRC itself [MT p. 8]."""
    out = 0
    for v in values:
        out ^= v
    return out & ((1 << track.data_bits) - 1)


def record(text: str, track: Track = TRACK2) -> List[int]:
    """The character values on the card: SS, the data, ES, LRC."""
    vals = [track.ss] + [track.value(c) for c in text] + [track.es]
    return vals + [lrc(vals, track)]


def encode(text: str, track: Track = TRACK2, zeros: Optional[int] = None,
           bad_parity: Sequence[int] = (), bad_lrc: bool = False) -> List[int]:
    """The bits of a track carrying ``text`` (data characters only, the
    sentinels and LRC added), in card order, with ``zeros`` clocking zeros
    each side. ``bad_parity`` lists record positions (0 = SS) written with
    even parity; ``bad_lrc`` writes an LRC with one data bit wrong (its
    parity still odd)."""
    z = track.zeros if zeros is None else zeros
    vals = record(text, track)
    if bad_lrc:
        vals[-1] ^= 1
    bits = [0] * z
    for i, v in enumerate(vals):
        bits += track.char_bits(v, parity_ok=i not in bad_parity)
    return bits + [0] * z


def f2f_levels(bits: Sequence[int], level: int = 0) -> List[int]:
    """F2F: the line level in each half cell, two a bit; a transition at
    the start of every cell and in the middle of a 1 [P]."""
    out = []
    for b in bits:
        level ^= 1
        out.append(level)
        if b:
            level ^= 1
        out.append(level)
    return out


def reverse_swipe(bits: Sequence[int]) -> List[int]:
    """The bits a reader sees when the card is swiped the other way."""
    return list(reversed(bits))


# ---------------------------------------------------------- speed profiles
Profile = Callable[[float], float]


def ramp(start: float, end: float) -> Profile:
    """Speed going linearly from ``start`` to ``end`` over the swipe (of
    the nominal; 2.0 is twice as fast)."""
    return lambda x: start + (end - start) * x


def hump(edge: float, middle: float) -> Profile:
    """Speeding up to ``middle`` at the centre of the swipe and slowing to
    ``edge`` again at its end (a hand's push)."""
    return lambda x: edge + (middle - edge) * (1.0 - abs(2.0 * x - 1.0))


class SwipeSource(Model):
    """Drives a reader pin with F2F as a read head's peak detector gives it,
    a level that toggles at every flux transition. :meth:`send` plays
    ``bits`` with a half cell of ``half`` clocks at nominal speed; half cell
    ``n`` of ``N`` lasts ``half / speed(n / N)``. Each transition is then
    moved by up to ``jitter`` clocks either way (uniform, ``seed``), never
    past its neighbours. :attr:`edges` lists every transition's cycle."""

    def __init__(self, pin: PinSpec, level: int = 0) -> None:
        self.pad = pad_of(pin)
        self.level = level
        self.free_at = 0
        self.edges: List[int] = []
        self._changes: List[Tuple[int, int]] = []

    def send(self, bits: Sequence[int], half: float, at: int,
             speed: Optional[Profile] = None, jitter: float = 0.0,
             seed: int = 1) -> int:
        start = max(int(at), self.free_at)
        level = self._changes[-1][1] if self._changes else self.level
        rng = random.Random(seed)
        levels = f2f_levels(bits, level)
        when, times = float(start), []
        for n, lv in enumerate(levels):
            if lv != level:
                times.append((when, lv))
                level = lv
            when += half / (speed(n / len(levels)) if speed else 1.0)
        times.append((when, 1 - level))           # the transition closing the last cell
        last = start - 1
        for t, lv in times:
            c = int(round(t + (rng.uniform(-jitter, jitter) if jitter else 0.0)))
            c = max(c, last + 1)
            self._changes.append((c, lv))
            self.edges.append(c)
            last = c
        self.free_at = last + 1
        return start

    def drive(self, drive: Drive, cycle: int) -> None:
        changes = self._changes
        while changes and changes[0][0] <= cycle:
            self.level = changes.pop(0)[1]
        drive.set(self.pad, self.level)


# ----------------------------------------------------------------- decoder
def intervals_to_bits(times: Sequence[int]) -> List[int]:
    """An F2F decoder by transition spacing with the "last window" cell
    reference of [P]: the reference is the last cell measured. The first
    interval is the first clocking zero. An interval nearer a whole cell
    than a half (3/4 of the reference or more) is a 0; two halves are a 1.
    Everything before the first 1 is a clocking zero and is returned too."""
    if len(times) < 2:
        return []
    ref = float(times[1] - times[0])
    bits = [0]
    half: Optional[float] = None
    for a, b in zip(times[1:], times[2:]):
        d = b - a
        if half is None and d >= 0.75 * ref:
            bits.append(0)
            ref = d
        elif half is None:
            half = d
        else:
            bits.append(1)
            ref, half = half + d, None
    return bits


@dataclasses.dataclass
class Decoded:
    text: str               # the data characters, ASCII
    reverse: bool
    errors: int             # E_* bits


def frame(bits: Sequence[int], track: Track, reverse: bool) -> Decoded:
    """Frame one direction [MT p. 8, p. 34]: from the first 1 (forward) or
    the last 1 read backwards, a start sentinel, characters up to the end
    sentinel, then the LRC; parity checked on every character."""
    seq = list(reversed(bits)) if reverse else list(bits)
    if 1 not in seq:
        return Decoded("", reverse, E_NO_SS)
    seq = seq[seq.index(1):]
    width = track.data_bits + 1
    errors, vals, i = 0, [], 0

    def char():
        nonlocal errors, i
        c = (seq[i:i + width] + [0] * width)[:width]
        i += width
        if sum(c) % 2 == 0:
            errors |= E_PARITY
        return sum(b << k for k, b in enumerate(c[:-1]))

    if char() != track.ss:
        return Decoded("", reverse, errors | E_NO_SS)
    vals.append(track.ss)
    text = ""
    while True:
        if i >= len(seq) or 1 not in seq[i:]:
            return Decoded(text, reverse, errors | E_NO_ES)
        v = char()
        vals.append(v)
        if v == track.es:
            break
        text += chr(v + track.offset)
    if char() != lrc(vals, track):
        errors |= E_LRC
    return Decoded(text, reverse, errors)


def decode(bits: Sequence[int], track: Track = TRACK2) -> Decoded:
    """Both directions; forward wins unless reverse has fewer errors (a
    missing start sentinel counting worst), as [MT p. 34] has it: the
    card is unreadable only when both fail."""
    f, r = frame(bits, track, False), frame(bits, track, True)
    return r if r.errors < f.errors else f


def report(d: Decoded) -> List[int]:
    """The words magstripe_reader.loom pushes for ``d``."""
    words = [] if d.errors & E_NO_SS else [ord(c) for c in d.text]
    return words + [0x8000 | (int(d.reverse) << 8) | d.errors]
