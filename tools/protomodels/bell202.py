"""Bell 202 FSK reference models: a phase-continuous modulator that drives a
chip input, an edge recorder, a zero-crossing demodulator and a
phase-continuity checker for what a chip output sends.

Written from the specifications, not from ``firmware/bell202_*.loom``.

Sources (section numbers I could not check against the documents are
marked *unchecked*; secondary sources are marked *secondary*):

* Bell System Technical Reference PUB 41212, "Data Sets 202S and 202T
  Interface Specification" (1976 and later issues): FSK, mark (binary 1)
  1200 Hz, space (binary 0) 2200 Hz, asynchronous data up to 1200 bit/s
  (the transmitter characteristics section; *unchecked* section number).
* HART Communication Foundation HCF_SPEC-54, "FSK Physical Layer
  Specification": 1200 Hz for a 1 and 2200 Hz for a 0, each within +-1 %;
  1200 bit/s +-1 %; the phase is continuous at a change of frequency, so
  the signal never jumps (the modulator requirements; *unchecked* section
  number, the values are those every HART modem datasheet repeats, e.g.
  the AD5700 and DS8500 datasheets, *secondary*).
* HCF_SPEC-81, "Data Link Layer Specification": each character is a start
  bit, 8 data bits LSB first, an odd parity bit and a stop bit (11 bits);
  a message starts with a preamble of 5 to 20 0xFF characters
  (*unchecked* section numbers).
* Telcordia GR-30-CORE (caller ID, on-hook data transmission) uses the
  same Bell 202 tones with 8N1 characters after a channel seizure of
  alternating bits and a mark signal; it is cited for the 8N1 framing
  only (*unchecked*).
* ITU-T V.23 (mode 2: 1300 Hz mark, 2100 Hz space, 1200 bit/s) is the
  CCITT counterpart; nothing here models it.

What the models take from those: mark 1200 Hz is a 1, space 2200 Hz a 0,
1200 baud, the +-1 % tolerances, continuous phase, and the two character
formats. The signal is modelled at logic level: a square wave whose edges
are the zero crossings of the line's sine (an analog front end, a filter on
the transmit side and a limiter on the receive side, is assumed and not
modelled).
"""

from __future__ import annotations

import bisect
import dataclasses
import random
from collections import deque
from typing import Iterable, List, Optional, Sequence, Tuple

from .bench import Drive, Lines, Model, PinSpec, pad_of

MARK_HZ = 1200.0            # binary 1 (PUB 41212; HCF_SPEC-54)
SPACE_HZ = 2200.0           # binary 0
BAUD = 1200.0
TOLERANCE = 0.01            # +-1 % on each tone and on the bit rate (HCF_SPEC-54)
BIT_S = 1.0 / BAUD
#: The midpoint frequency of the two tones; a half-period longer than
#: half of 1/1700 s is taken for mark. A choice of this model's, not a spec
#: figure: any threshold between the two tones' half-periods would do.
CENTRE_HZ = (MARK_HZ + SPACE_HZ) / 2.0

NONE, ODD = "8N1", "8O1"


def tone_hz(bit: int) -> float:
    return MARK_HZ if bit else SPACE_HZ


def char_bits(byte: int, framing: str = NONE, *, stop: int = 1,
              parity: Optional[int] = None) -> List[int]:
    """The bits of one character in the order they go out: start 0, eight
    data bits LSB first, in 8O1 the odd-parity bit, the stop bit. ``stop``
    and ``parity`` override the stop and parity bits (to make errors)."""
    data = [(byte >> i) & 1 for i in range(8)]
    bits = [0] + data
    if framing == ODD:
        good = 1 - (sum(data) & 1)          # makes the count of 1s odd
        bits.append(good if parity is None else parity)
    elif framing != NONE:
        raise ValueError("framing %r" % (framing,))
    bits.append(stop)
    return bits


def stream_bits(data: Iterable[int], framing: str = NONE, *, lead: int = 4,
                tail: int = 4, gap: int = 0) -> List[int]:
    """Mark for ``lead`` bits, the characters (``gap`` mark bits between
    them), then ``tail`` bits of mark."""
    bits = [1] * lead
    for i, byte in enumerate(data):
        if i:
            bits += [1] * gap
        bits += char_bits(byte, framing)
    return bits + [1] * tail


# ------------------------------------------------------------- modulation
def fsk_crossings(bits: Sequence[int], *, offset: float = 0.0,
                  phase0: float = 0.0) -> Tuple[List[float], float]:
    """Ideal continuous-phase FSK: the zero-crossing times (seconds from the
    start of the first bit) of ``bits``, and the stream's length.

    The phase is counted in half-cycles from ``phase0``; the square wave is
    high while ``floor(phase)`` is even. ``offset`` scales both tones and the
    bit rate together (+0.01 is a transmitter whose clock is 1 % fast), so
    every bit still holds the same number of cycles of its tone.
    """
    scale = 1.0 + offset
    bit_s = BIT_S / scale
    times: List[float] = []
    phase = phase0
    n = int(phase0) + 1                             # the next crossing, counted once
    t = 0.0
    for bit in bits:
        rate = 2.0 * tone_hz(bit) * scale           # half-cycles a second
        end = phase + rate * bit_s
        while n <= end + 1e-9:
            times.append(t + (n - phase) / rate)
            n += 1
        phase = end
        t += bit_s
    return times, t


class FskSource(Model):
    """Drives a phase-continuous Bell 202 square wave onto a pad.

    ``bits`` start ``start_us`` after cycle 0 (the pad is high before, the
    level a half-cycle starts with). ``jitter_us`` moves every edge by an
    independent uniform offset in +-``jitter_us`` (seeded, so a test is
    repeatable), keeping the edges in order.
    """

    def __init__(self, pin: PinSpec, clock_hz: float, bits: Sequence[int], *,
                 offset: float = 0.0, jitter_us: float = 0.0, seed: int = 202,
                 start_us: float = 100.0) -> None:
        self.pad = pad_of(pin)
        self.clock_hz = float(clock_hz)
        self.bits = list(bits)
        self.offset = offset
        times, length = fsk_crossings(self.bits, offset=offset)
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
        self.level = 1

    def drive(self, drive: Drive, cycle: int) -> None:
        while self._next and self._next[0] <= cycle:
            self._next.popleft()
            self.level ^= 1
        drive.set(self.pad, self.level)

    @property
    def done(self) -> bool:
        return not self._next


class EdgeRecorder(Model):
    """Every change of one pad: ``edges`` holds ``(cycle, level)``, the cycle
    being the first with the new level. The level before the first edge is
    ``initial``."""

    def __init__(self, pin: PinSpec, initial: int = 0) -> None:
        self.pad = pad_of(pin)
        self.level = initial
        self.edges: List[Tuple[int, int]] = []

    def observe(self, lines: Lines) -> None:
        level = lines.get(self.pad)
        if level != self.level:
            self.edges.append((lines.cycle, level))
            self.level = level

    def times(self, clock_hz: float) -> List[float]:
        """The edge times in seconds."""
        return [c / float(clock_hz) for c, _ in self.edges]


# ----------------------------------------------------------- demodulation
@dataclasses.dataclass
class Char:
    """One received character: the byte, its errors, where its start bit began."""

    byte: int
    framing_error: bool
    parity_error: bool
    start_s: float


def half_periods(times: Sequence[float]) -> List[Tuple[float, float]]:
    """``(start, length)`` of every half-period between two recorded edges."""
    return [(a, b - a) for a, b in zip(times, times[1:])]


def tone_of(length_s: float) -> int:
    """1 (mark) for a half-period longer than half a period of CENTRE_HZ."""
    return 1 if length_s > 0.5 / CENTRE_HZ else 0


def demodulate(times: Sequence[float], framing: str = NONE) -> List[Char]:
    """A zero-crossing receiver, independent of the firmware's method: the
    tone at time t is the tone of the half-period that holds t; a character
    starts where the first space half-period after mark starts, and its bits
    are sampled at the nominal bit centres from there (no re-timing inside
    a character, as an ordinary UART receiver)."""
    hp = half_periods(times)
    if not hp:
        return []
    starts = [a for a, _ in hp]
    tones = [tone_of(n) for _, n in hp]
    end = times[-1]

    def tone_at(t: float) -> Optional[int]:
        i = bisect.bisect_right(starts, t) - 1
        return tones[i] if 0 <= i < len(tones) and t < end else None

    nbits = 11 if framing == ODD else 10
    chars: List[Char] = []
    pos = starts[0]
    i = 1
    while i < len(hp):
        if not (tones[i] == 0 and tones[i - 1] == 1 and starts[i] >= pos):
            i += 1
            continue
        t0 = starts[i]
        sample = [tone_at(t0 + (k + 0.5) * BIT_S) for k in range(nbits)]
        if None in sample:
            break
        if sample[0] != 0:
            i += 1
            continue
        data = sample[1:9]
        byte = sum(b << k for k, b in enumerate(data))
        parity_error = framing == ODD and (sum(data) + sample[9]) % 2 == 0
        chars.append(Char(byte, sample[-1] != 1, parity_error, t0))
        pos = t0 + (nbits - 0.5) * BIT_S
        i += 1
    return chars


def bits_from_chars(chars: Sequence[Char], framing: str = NONE) -> List[int]:
    """The bit stream the characters make, from the first start bit on, with
    mark wherever no character is (characters placed on a whole-bit grid
    from the first one's start)."""
    if not chars:
        return []
    bits: List[int] = []
    origin = chars[0].start_s
    for ch in chars:
        k = int(round((ch.start_s - origin) / BIT_S))
        bits += [1] * (k - len(bits))
        bits += char_bits(ch.byte, framing, stop=0 if ch.framing_error else 1,
                          parity=None if not ch.parity_error else
                          (sum((ch.byte >> j) & 1 for j in range(8)) & 1))
    return bits


# ------------------------------------------------------- phase continuity
def _phase_table(bits: Sequence[int]) -> List[float]:
    cum = [0.0]
    for bit in bits:
        cum.append(cum[-1] + 2.0 * tone_hz(bit) * BIT_S)  # cycles a bit: no offset
    return cum


def phase_at(t: float, t0: float, bits: Sequence[int], cum: Sequence[float],
             scale: float = 1.0) -> Tuple[float, float]:
    """The ideal phase in half-cycles at ``t`` (0 at ``t0``, where ``bits``
    start; mark before and after them) and the rate there."""
    bit_s = BIT_S / scale
    x = t - t0
    k = int(x // bit_s) if x >= 0 else -1
    if k < 0 or k >= len(bits):
        rate = 2.0 * MARK_HZ * scale
        base = 0.0 if k < 0 else cum[len(bits)]
        dx = x if k < 0 else x - len(bits) * bit_s
        return base + rate * dx, rate
    rate = 2.0 * tone_hz(bits[k]) * scale
    return cum[k] + rate * (x - k * bit_s), rate


def phase_error(times: Sequence[float], bits: Sequence[int], t0: float, *,
                offset: float = 0.0) -> float:
    """The worst distance, in seconds, of an edge from where a phase-
    continuous modulator sending ``bits`` from ``t0`` puts the crossing: the
    edges must be consecutive half-cycles, so the phase at edge i is i plus
    one constant, the constant taken midway between the extremes."""
    scale = 1.0 + offset
    cum = _phase_table(bits)
    res = []
    for i, t in enumerate(times):
        phase, rate = phase_at(t, t0, bits, cum, scale)
        res.append((phase - i, rate))
    lo = min(r for r, _ in res)
    hi = max(r for r, _ in res)
    c = (lo + hi) / 2.0
    return max(abs(r - c) / rate for r, rate in res)


def check_phase_continuity(times: Sequence[float], bits: Sequence[int],
                           t0_hint: float, *, offset: float = 0.0,
                           span_s: float = BIT_S / 2) -> Tuple[float, float]:
    """Fit the bit grid of ``bits`` to the edges ``times`` and return
    ``(t0, worst error in seconds)``. A phase-continuous modulator gives an
    error no larger than its edge rounding; one that restarts its square
    wave at bit boundaries, or switches tone at a half-cycle's end, misses by
    a large part of a half-period (tools/tests checks both kinds)."""
    def best(lo, hi, step):
        n = max(1, int(round((hi - lo) / step)))
        cand = [lo + j * (hi - lo) / n for j in range(n + 1)]
        return min((phase_error(times, bits, t, offset=offset), t) for t in cand)

    err, t0 = best(t0_hint - span_s, t0_hint + span_s, 4e-6)
    err, t0 = best(t0 - 4e-6, t0 + 4e-6, 0.1e-6)
    return t0, err


def steady_half_periods(times: Sequence[float]) -> Tuple[List[float], List[float]]:
    """The mark and the space half-periods whose neighbours on both sides
    are of the same tone (so no tone change touches them), in seconds."""
    hp = [n for _, n in half_periods(times)]
    mark, space = [], []
    for a, b, c in zip(hp, hp[1:], hp[2:]):
        if tone_of(a) == tone_of(b) == tone_of(c):
            (mark if tone_of(b) else space).append(b)
    return mark, space


class Wire(Model):
    """A board wire from one pad to another: ``dst`` follows ``src`` one
    clock later, as a registered model must (the modulator's output to the
    demodulator's input when both programs run on one chip)."""

    def __init__(self, src: PinSpec, dst: PinSpec) -> None:
        self.src, self.dst = pad_of(src), pad_of(dst)
        self.level = 0

    def drive(self, drive: Drive, cycle: int) -> None:
        drive.set(self.dst, self.level)

    def observe(self, lines: Lines) -> None:
        self.level = lines.get(self.src)
