"""Kansas City Standard (and CUTS) cassette reference models: an encoder of
the ideal recorded square wave, a source that plays it into a chip input
with tape speed error, edge jitter and a dropout, and a reference decoder
for what a chip output sends.

Written from the specification, not from ``firmware/kcs_*.loom``.

Sources (figures I could not check against a primary document are marked
*unchecked*):

* "Report: BYTE's Audio Cassette Standards Symposium", Manfred and Virginia
  Peschke, BYTE vol. 1 no. 6, February 1976, pp. 72-73, the "Provisional
  Audio Cassette Data Interchange Standard" it prints. Read in the
  archive.org OCR text of the issue
  (archive.org/stream/byte-magazine-1976-02, the ``_djvu.txt`` view), not
  the page scan, so the wording is the OCR's. It says:

  - "A mark (logical one) bit consists of eight cycles at a frequency of
    2400 Hz"; a space (logical zero) "four cycles at a frequency of 1200
    Hz". No tone tolerance is given.
  - serial data "at rates of up to 300 baud"; no rate tolerance.
  - a character is "a space as a start bit, eight data bits, and two or
    more marks as stop bits", the data least significant bit first; an
    optional parity bit counts inside the eight (not modelled: the host
    sends whatever eight bits it wants).
  - "The interval between characters consists of an unspecified amount of
    time at the mark frequency"; a block is "preceded by a minimum of five
    seconds of marks".
  - "Bit clocking information may be extracted from the recorded waveform,
    which is always an integer multiple of the bit rate, regardless of
    tape speed", and the technique "provides for long and short term tape
    speed variation" (no figure).
  - It does not state the phase of a bit's cycles. This model starts every
    bit on a rising edge (a whole number of cycles from a common origin),
    which is what "an integer multiple of the bit rate" implies for a
    continuous wave; a decoder that counts crossings does not care.
* CUTS, the Computer Users' Tape Standard (Processor Technology, 1976):
  1200 baud, a 1 two cycles of 2400 Hz, a 0 one cycle of 1200 Hz, one start
  bit and two stop bits. From the Wikipedia article "Kansas City standard"
  only, *unchecked* against Processor Technology's documents or the
  Popular Electronics article of March 1977.

The signal is modelled at logic level: a square wave whose edges are the
zero crossings of the recorded tone (the cassette, its filters and the
playback comparator are not modelled). Tape speed scales every time by
``1 / (1 + speed)``: +0.05 is a tape playing 5 % fast.
"""

from __future__ import annotations

import bisect
import dataclasses
import random
from collections import deque
from typing import Iterable, List, Optional, Sequence, Tuple

from .bench import Drive, Model, PinSpec, pad_of
from .bell202 import EdgeRecorder, Wire  # noqa: F401  (generic pad models)

MARK_HZ = 2400.0            # binary 1 (BYTE: eight cycles at 300 baud)
SPACE_HZ = 1200.0           # binary 0 (BYTE: four cycles at 300 baud)
KCS, CUTS = 300.0, 1200.0   # the two baud rates (CUTS *unchecked*)
STOP_BITS = 2               # BYTE: "two or more marks as stop bits"
LEADER_S = 5.0              # BYTE: "a minimum of five seconds of marks"
#: The midpoint of the two half-periods (208.3 and 416.7 us). A choice of
#: this model's, not a spec figure: any threshold between them would do.
THRESH_S = 0.75 / MARK_HZ


def bit_s(baud: float = KCS) -> float:
    return 1.0 / baud


def tone_hz(bit: int) -> float:
    return MARK_HZ if bit else SPACE_HZ


def cycles_per_bit(bit: int, baud: float = KCS) -> int:
    """8 or 4 at 300 baud, 2 or 1 at 1200 (a whole number, which is the
    point of the standard)."""
    n = tone_hz(bit) / baud
    assert abs(n - round(n)) < 1e-9
    return int(round(n))


def char_bits(byte: int, *, stop: Sequence[int] = (1, 1)) -> List[int]:
    """Start 0, eight data bits LSB first, the stop bits (``stop``
    overrides them, to make framing errors)."""
    return [0] + [(byte >> i) & 1 for i in range(8)] + list(stop)


def stream_bits(data: Iterable[int], *, lead: int = 8, tail: int = 4,
                gap: int = 0) -> List[int]:
    """A leader of ``lead`` mark bits, the characters with ``gap`` extra
    marks between them, then ``tail`` marks."""
    bits = [1] * lead
    for i, byte in enumerate(data):
        if i:
            bits += [1] * gap
        bits += char_bits(byte)
    return bits + [1] * tail


def crossings(bits: Sequence[int], baud: float = KCS, *,
              speed: float = 0.0) -> Tuple[List[float], float]:
    """The ideal recording of ``bits``: every edge time in seconds from the
    start of the first bit (the first is the rising edge at 0), and the
    stream's length. Each bit is ``cycles_per_bit`` whole cycles of its
    tone, high half first."""
    scale = 1.0 / (1.0 + speed)
    b = bit_s(baud)
    times: List[float] = []
    for k, bit in enumerate(bits):
        n = 2 * cycles_per_bit(bit, baud)
        for j in range(n):
            times.append((k * b + j * b / n) * scale)
    return times, len(bits) * b * scale


class KcsSource(Model):
    """Plays a recording of ``bits`` onto a pad, starting ``start_us``
    after cycle 0 (the pad is low before and after).

    ``speed`` is the tape speed error, ``jitter_us`` moves every edge by an
    independent uniform offset in +-``jitter_us`` (seeded, so a test is
    repeatable, and kept in order), and ``dropout`` = ``(at_s, length_s)``,
    in seconds of the played stream, removes every edge inside that window:
    the comparator sees no signal there and its output stands still.
    """

    def __init__(self, pin: PinSpec, clock_hz: float, bits: Sequence[int],
                 baud: float = KCS, *, speed: float = 0.0,
                 jitter_us: float = 0.0, seed: int = 1976,
                 start_us: float = 100.0,
                 dropout: Optional[Tuple[float, float]] = None) -> None:
        self.pad = pad_of(pin)
        self.clock_hz = float(clock_hz)
        self.bits = list(bits)
        times, length = crossings(self.bits, baud, speed=speed)
        if dropout is not None:
            at, span = dropout
            times = [t for t in times if not at <= t < at + span]
        rng = random.Random(seed)
        start = start_us * 1e-6
        cycles: List[int] = []
        for t in times:
            if jitter_us:
                t += rng.uniform(-jitter_us, jitter_us) * 1e-6
            c = int(round((start + t) * self.clock_hz))
            if cycles and c <= cycles[-1]:
                c = cycles[-1] + 1
            cycles.append(c)
        self.start_cycle = int(round(start * self.clock_hz))
        self.end_cycle = int(round((start + length) * self.clock_hz))
        self.edges = cycles
        self._next = deque(cycles)
        self.level = 0

    def drive(self, drive: Drive, cycle: int) -> None:
        while self._next and self._next[0] <= cycle:
            self._next.popleft()
            self.level ^= 1
        drive.set(self.pad, self.level)

    @property
    def done(self) -> bool:
        return not self._next


# ---------------------------------------------------------------- decoding
@dataclasses.dataclass
class Char:
    """One received character: the byte, whether a stop bit was 0, and
    where its start bit began (seconds)."""

    byte: int
    framing_error: bool
    start_s: float


def half_periods(times: Sequence[float]) -> List[Tuple[float, float]]:
    """``(start, length)`` of every half-period between two edges."""
    return [(a, b - a) for a, b in zip(times, times[1:])]


def decode(times: Sequence[float], baud: float = KCS, *,
           speed: float = 0.0) -> List[Char]:
    """A reference receiver, independent of the firmware's method: the tone
    at time t is that of the half-period holding t (long: 1200 Hz, a 0); a
    character starts where the first long half-period after a short one
    starts, and its bits are sampled at the bit centres of a clock that
    knows the tape speed (as a UART at the right rate would)."""
    hp = half_periods(times)
    if not hp:
        return []
    starts = [a for a, _ in hp]
    tones = [0 if n > THRESH_S else 1 for _, n in hp]
    end = times[-1]
    b = bit_s(baud) / (1.0 + speed)
    nbits = 9 + STOP_BITS

    def tone_at(t: float) -> Optional[int]:
        i = bisect.bisect_right(starts, t) - 1
        return tones[i] if 0 <= i < len(tones) and t < end else None

    chars: List[Char] = []
    pos = starts[0]
    for i in range(1, len(hp)):
        if not (tones[i] == 0 and tones[i - 1] == 1 and starts[i] >= pos):
            continue
        t0 = starts[i]
        sample = [tone_at(t0 + (k + 0.5) * b) for k in range(nbits)]
        if None in sample:
            break
        if sample[0] != 0:
            continue
        byte = sum(v << k for k, v in enumerate(sample[1:9]))
        chars.append(Char(byte, sample[9:] != [1] * STOP_BITS, t0))
        pos = t0 + (nbits - 0.5) * b
    return chars


def bits_from_chars(chars: Sequence[Char], baud: float = KCS) -> List[int]:
    """The bit stream from the first start bit on: the characters on a
    whole-bit grid from the first one's start, marks between them."""
    bits: List[int] = []
    if not chars:
        return bits
    origin = chars[0].start_s
    for ch in chars:
        k = int(round((ch.start_s - origin) / bit_s(baud)))
        bits += [1] * (k - len(bits))
        bits += char_bits(ch.byte)
    return bits


def edge_error(times: Sequence[float], bits: Sequence[int], t0: float,
               baud: float = KCS) -> float:
    """The worst distance in seconds between the edges ``times`` (from the
    start bit's first edge on, as many as ``bits`` make) and the ideal
    recording of ``bits`` starting at ``t0``. Also fails (returns inf) if
    the edges are fewer, so a missing or extra cycle anywhere shows."""
    ideal, _ = crossings(bits, baud)
    i = bisect.bisect_left(times, t0 - 0.25 / MARK_HZ)
    got = list(times[i:i + len(ideal)])
    if len(got) < len(ideal):
        return float("inf")
    return max(abs(g - (t0 + x)) for g, x in zip(got, ideal))


def steady_half_periods(times: Sequence[float]) -> Tuple[List[float], List[float]]:
    """The 2400 Hz and the 1200 Hz half-periods, in seconds."""
    hp = [n for _, n in half_periods(times)]
    return [n for n in hp if n <= THRESH_S], [n for n in hp if n > THRESH_S]
