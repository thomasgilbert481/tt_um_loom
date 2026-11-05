"""ARINC 429 reference model: words, a two-line player, a timing checker.

Written from public secondary sources, never from ``firmware/arinc429_*.loom``.
ARINC Specification 429 Part 1 is sold by SAE-ITC and was not read; every
figure below is from one of these, and says which:

* **Holt HI-8282A** datasheet, "ARINC 429 serial transmitter and dual
  receiver", DS8282A Rev. New, May 2001 (fetched 2026-11-04). Page 3,
  "Receiver logic operation", the timing "the ARINC 429 specification
  contains" for received data:

  ============== ===================== ======================
  ..             high speed            low speed
  ============== ===================== ======================
  bit rate       100 kbit/s +- 1 %     12 to 14.5 kbit/s
  pulse rise     1.5 +- 0.5 us         10 +- 5 us
  pulse fall     1.5 +- 0.5 us         10 +- 5 us
  pulse width    5 us +- 5 %           34.5 to 41.7 us
  ============== ===================== ======================

  and its receiver's rules: 10 samples a bit; each data bit must follow its
  predecessor by not less than 8 and not more than 12 samples (83 to 125
  kbit/s and 10.4 to 15.6 kbit/s accepted at the nominal sample clocks);
  a NULL in the word gap needs 3 samples a bit apart before the next word;
  parity counts the ones including bit 32 and odd is good. Line levels
  (differential): ONE +6.5 to +13 V, NULL +2.5 to -2.5 V, ZERO -6.5 to -13
  V. Page 4, its transmitter: a bit is 10 clocks, 5 of data and 5 of NULL,
  then a word gap of 40 clocks (4 bit times); bit 32 is set to make the
  parity odd.
* **Alta Data Technologies**, "ARINC Protocol Tutorial" (altadt.com), and
  **Wikipedia**, "ARINC 429" (both read 2026-11-04): the word is 32 bits,
  sent bit 1 first: label bits 1 to 8, SDI 9 and 10, data 11 to 29, SSM 30
  and 31, parity 32; the label goes out most significant bit first, so its
  octal value appears bit-reversed in bits 1 to 8 (octal 012 = 00001010 is
  sent 0, 1, 0, 1, 0, 0, 0, 0); the other fields go out least significant
  bit first; at least 4 bit times of NULL between words; the line is
  bipolar return-to-zero, each bit a HI or LO half then a NULL half.

Derived, and marked so where used: the HI/NULL ratio of 50 % +- 5 % is the
high-speed pulse width (5 us +- 5 %) over the 10 us bit; at low speed the
source gives the width only as 34.5 to 41.7 us, half a bit at 14.5 and at
12 kbit/s, which is the same 50 % at every rate in the range. The rise and
fall times are the line driver's (its slope control) and do not exist on
the logic pins this model sees: :class:`ArincLine` takes a ``slew_us`` and
checks that a pulse of that slew still has its width inside the window at
its 50 % points and reaches the ONE or ZERO level before it falls (the
level reached is ``(width - slew)`` of full swing at the plateau; a pulse
that never plateaus fails), which is the only check the logic timing can
fail.

The logic interface is a line driver's two inputs (HI-8585 / HI-3182
style): line A high is HI, line B high is LO, both low NULL; and a line
receiver's two outputs (HI-8482 / HI-8588 style) the same way round. Both
high is not a line state.
"""

from __future__ import annotations

import dataclasses
from typing import List, Optional, Sequence, Tuple

from tools.protomodels.bench import Drive, Lines, Model, PinSpec, pad_of

# ------------------------------------------------ HI-8282A page 3 (above)
HS_RATE, HS_TOL = 100_000.0, 0.01          # bit/s, +- 1 %
LS_RATE_MIN, LS_RATE_MAX = 12_000.0, 14_500.0
HS_WIDTH_US, HS_WIDTH_TOL = 5.0, 0.05      # a HI or LO pulse
LS_WIDTH_MIN_US, LS_WIDTH_MAX_US = 34.5, 41.7
HS_RISE_US = (1.0, 2.0)                    # 1.5 +- 0.5 us, rise and fall
LS_RISE_US = (5.0, 15.0)                   # 10 +- 5 us
RX_SAMPLES = 10                            # samples a bit (Holt receiver)
RX_MIN_SAMPLES, RX_MAX_SAMPLES = 8, 12     # bit to bit, Holt rule 3
GAP_BITS_MIN = 4                           # NULL between words, bit times
RATIO_NOM, RATIO_TOL = 0.5, 0.05           # derived (docstring)

SPEEDS = ("high", "low")


def rate_window(speed: str) -> Tuple[float, float]:
    """The transmit bit-rate window, bit/s."""
    if speed == "high":
        return HS_RATE * (1 - HS_TOL), HS_RATE * (1 + HS_TOL)
    return LS_RATE_MIN, LS_RATE_MAX


def width_window(speed: str) -> Tuple[float, float]:
    """The HI or LO pulse width window, us."""
    if speed == "high":
        return HS_WIDTH_US * (1 - HS_WIDTH_TOL), HS_WIDTH_US * (1 + HS_WIDTH_TOL)
    return LS_WIDTH_MIN_US, LS_WIDTH_MAX_US


def rise_window(speed: str) -> Tuple[float, float]:
    return HS_RISE_US if speed == "high" else LS_RISE_US


# ------------------------------------------------------------- the word
def reverse8(x: int) -> int:
    return int("{:08b}".format(x & 0xFF)[::-1], 2)


def word(label: int, data: int = 0, *, sdi: int = 0, ssm: int = 0) -> int:
    """A word as a 32-bit value ``V`` in octal-label form: ``V`` bit ``k``
    is ARINC bit ``k + 1`` except bits 7:0, which hold the label as it is
    written in octal (``0o205`` is ``0x85``). Bit 32 (``V`` bit 31) is 0:
    :func:`with_parity` sets it."""
    if not 0 <= label <= 0o377:
        raise ValueError("label %r outside 0o000 .. 0o377" % (label,))
    if not 0 <= data < 1 << 19 or not 0 <= sdi < 4 or not 0 <= ssm < 4:
        raise ValueError("field out of range")
    return label | sdi << 8 | data << 10 | ssm << 29


def ones(v: int) -> int:
    return bin(v & 0xFFFFFFFF).count("1")


def with_parity(v: int, odd: bool = True) -> int:
    """``v`` with bit 32 set so the word has odd (or even) parity."""
    v &= 0x7FFFFFFF
    p = ones(v) & 1
    return v | ((p ^ 1) if odd else p) << 31


def parity_ok(v: int) -> bool:
    """Odd parity over all 32 bits (the receiver rule)."""
    return ones(v) & 1 == 1


def wire_bits(v: int) -> List[int]:
    """The 32 bits in the order they are sent: the label's MSB first, then
    ARINC bits 9 .. 32 (``V`` bits 8 .. 31)."""
    lab = v & 0xFF
    return [(lab >> (7 - j)) & 1 for j in range(8)] + \
           [(v >> k) & 1 for k in range(8, 32)]


def from_wire(bits: Sequence[int]) -> int:
    """The inverse of :func:`wire_bits` (32 bits)."""
    if len(bits) != 32:
        raise ValueError("%d bits" % len(bits))
    lab = sum(b << (7 - j) for j, b in enumerate(bits[:8]))
    return lab | sum(b << (8 + k) for k, b in enumerate(bits[8:]))


def arinc_bits_value(v: int) -> int:
    """``V`` with the label reversed: bit ``k`` is exactly ARINC bit
    ``k + 1`` (bit 1, the label's MSB, in bit 0)."""
    return (v & ~0xFF) | reverse8(v & 0xFF)


def host_words(v: int) -> List[int]:
    """The two FIFO words of ``arinc429_tx`` and ``arinc429_rx`` (their
    headers): ``V[15:0]`` then ``V[31:16]``. This is the programs' host
    format, not part of the standard."""
    return [v & 0xFFFF, (v >> 16) & 0xFFFF]


def from_host(lo: int, hi: int) -> int:
    return (lo & 0xFFFF) | (hi & 0xFFFF) << 16


# --------------------------------------------------- the firmware's reports
S_MARK, S_WORD = 0x8000, 0x0080
E_PAR, E_RATE, E_SHORT, E_BOTH, E_NULL, E_GAP = 0x01, 0x02, 0x04, 0x08, 0x10, 0x20


@dataclasses.dataclass
class Report:
    status: int
    value: Optional[int] = None        # the word, octal-label form

    @property
    def errors(self) -> int:
        return self.status & 0x3F

    @property
    def bits(self) -> int:
        return (self.status >> 8) & 0x3F


def parse_reports(words: Sequence[int]) -> List[Report]:
    """``arinc429_rx``'s OUTQ words (its header): a status word, bit 15
    set, and when its bit 7 is set the word's low and high halves."""
    out, i = [], 0
    while i < len(words):
        s = words[i]
        assert s & S_MARK, "word %d (0x%04X) is not a status word" % (i, s)
        if s & S_WORD:
            out.append(Report(s, from_host(words[i + 1], words[i + 2])))
            i += 3
        else:
            out.append(Report(s))
            i += 1
    return out


# ------------------------------------------------------------ the player
class ArincPlayer(Model):
    """Drives two receiver-output lines (A high = HI, B high = LO) with
    words, for a receiver under test. Edges sit at the rounded running sum
    of the durations, so long streams do not drift.

    :meth:`queue_words` takes values in octal-label form and the error
    injection: ``bit_us`` (any rate), ``hi_us`` (the HI or LO part, default
    half a bit), ``gap_bits`` of NULL after each word's last bit time,
    ``parity`` (``"odd"``, ``"even"`` for a bad word, or ``"raw"``: bit 32
    as given), ``nbits`` (fewer than 32: a short word), ``both_at`` (a bit
    index whose HI drives both lines)."""

    def __init__(self, pin_a: PinSpec, pin_b: PinSpec, *, clock_mhz: float = 50.0) -> None:
        self.pads = pad_of(pin_a), pad_of(pin_b)
        self.clock_mhz = clock_mhz
        self.a = self.b = 0
        self._events: List[Tuple[int, int, int]] = []
        self._t_us: Optional[float] = None
        self._now = 0

    def queue(self, segments: Sequence[Tuple[float, int, int]]) -> None:
        """``(duration_us, a, b)`` segments, played in order."""
        if self._t_us is None:
            self._t_us = (self._now + 4) / self.clock_mhz
        for us, a, b in segments:
            self._events.append((int(round(self._t_us * self.clock_mhz)), a, b))
            self._t_us += us
        self._events.append((int(round(self._t_us * self.clock_mhz)), 0, 0))

    def queue_words(self, values: Sequence[int], *, bit_us: float = 10.0,
                    hi_us: Optional[float] = None, gap_bits: float = GAP_BITS_MIN,
                    parity: str = "odd", nbits: int = 32,
                    both_at: Optional[int] = None, lead_bits: float = 0.0) -> None:
        hi = bit_us / 2 if hi_us is None else hi_us
        seg: List[Tuple[float, int, int]] = []
        if lead_bits:
            seg.append((lead_bits * bit_us, 0, 0))
        for v in values:
            if parity != "raw":
                v = with_parity(v, odd=(parity == "odd"))
            for k, bit in enumerate(wire_bits(v)[:nbits]):
                a, b = (1, 1) if k == both_at else (bit, 1 - bit)
                seg += [(hi, a, b), (bit_us - hi, 0, 0)]
            seg.append((gap_bits * bit_us, 0, 0))
        self.queue(seg)

    @property
    def idle(self) -> bool:
        return not self._events

    def drive(self, drive: Drive, cycle: int) -> None:
        drive.set(self.pads[0], self.a)
        drive.set(self.pads[1], self.b)

    def observe(self, lines: Lines) -> None:
        self._now = lines.cycle + 1
        while self._events and self._events[0][0] <= self._now:
            _, self.a, self.b = self._events.pop(0)


class Wire(Model):
    """A board wire from a chip output to a chip input, one clock late as
    a registered model is: the transmitter's OUT0/OUT1 into the receiver's
    IN0/IN1 (through the line driver, the bus and the line receiver)."""

    def __init__(self, src: PinSpec, dst: PinSpec) -> None:
        self.src, self.dst = pad_of(src), pad_of(dst)
        self.level = 0

    def drive(self, drive: Drive, cycle: int) -> None:
        drive.set(self.dst, self.level)

    def observe(self, lines: Lines) -> None:
        self.level = lines.get(self.src)


# ------------------------------------------------- the line timing checker
@dataclasses.dataclass
class Pulse:
    rise: int                          # clocks
    fall: int
    line: int                          # 1: A (HI, a 1), 0: B (LO, a 0)


@dataclasses.dataclass
class Violation:
    kind: str
    cycle: int
    detail: str = ""

    def __str__(self) -> str:                               # pragma: no cover
        return "%s at %d: %s" % (self.kind, self.cycle, self.detail)


class ArincLine(Model):
    """Watches a transmitter's two lines, decodes words and checks them.

    A pulse is a run of one line high. A word is the pulses between two
    NULLs longer than twice the pulse that ends them (inside a word a NULL
    is one pulse long, between words nine or more). Both lines high in any
    cycle is a ``both`` violation at once. :meth:`check` then holds every
    word to the transmit windows of ``speed`` (module docstring):

    * ``count``: 32 pulses;
    * ``rate``: the word's mean bit time (rise to rise) inside the rate
      window, and every single one inside it to the clock (an edge sits on
      a clock edge, so a bit at the window's end can measure one clock
      over);
    * ``width``: every pulse inside the width window, measured at its 50 %
      points, which with a symmetric slew are the logic edges;
    * ``ratio``: pulse over bit time within 50 % +- 5 % (derived);
    * ``gap``: the NULL after the last bit time, to the next word's first
      rise, at least 4 bit times;
    * ``slew``: with ``slew_us`` given, the slew inside the rise window and
      shorter than every pulse (or the pulse never reaches its level).

    Decoded words (octal-label form, bit 32 as sent) are in :attr:`words`.
    """

    def __init__(self, pin_a: PinSpec, pin_b: PinSpec, *, clock_mhz: float = 50.0) -> None:
        self.pads = pad_of(pin_a), pad_of(pin_b)
        self.clock_mhz = clock_mhz
        self.pulses: List[Pulse] = []
        self.violations: List[Violation] = []
        self._level = (0, 0)
        self._rise: Optional[int] = None

    def us(self, clocks: float) -> float:
        return clocks / self.clock_mhz

    def observe(self, lines: Lines) -> None:
        a, b = lines.get(self.pads[0]), lines.get(self.pads[1])
        c = lines.cycle
        if a and b:
            self.violations.append(Violation("both", c, "A and B high"))
        if (a, b) == self._level:
            return
        pa, pb = self._level
        if (pa or pb) and self._rise is not None:
            self.pulses.append(Pulse(self._rise, c, 1 if pa else 0))
            self._rise = None
        if (a or b) and not (pa or pb):
            self._rise = c
        self._level = (a, b)

    def word_pulses(self) -> List[List[Pulse]]:
        """The pulses grouped into words."""
        out: List[List[Pulse]] = []
        for k, p in enumerate(self.pulses):
            if k == 0:
                out.append([p])
                continue
            null = p.rise - self.pulses[k - 1].fall
            if null > 2 * (p.fall - p.rise):
                out.append([p])
            else:
                out[-1].append(p)
        return out

    @property
    def words(self) -> List[int]:
        return [from_wire([p.line for p in w]) for w in self.word_pulses() if len(w) == 32]

    def check(self, speed: str, slew_us: Optional[float] = None) -> List[Violation]:
        """The transmit checks; returns (and keeps) the violations."""
        rlo, rhi = rate_window(speed)
        tmin, tmax = 1e6 / rhi, 1e6 / rlo                  # bit time, us
        wmin, wmax = width_window(speed)
        groups = self.word_pulses()
        v = self.violations
        if slew_us is not None:
            slo, shi = rise_window(speed)
            if not slo <= slew_us <= shi:
                v.append(Violation("slew", 0, "%.2f us outside %s" % (slew_us, (slo, shi))))
        for n, w in enumerate(groups):
            if len(w) != 32:
                v.append(Violation("count", w[0].rise, "%d pulses" % len(w)))
            times = [self.us(q.rise - p.rise) for p, q in zip(w, w[1:])]
            q = self.us(1)                  # edges sit on clock edges
            for k, t in enumerate(times):
                if not tmin - q <= t <= tmax + q:
                    v.append(Violation("rate", w[k].rise, "bit %d: %.3f us" % (k + 1, t)))
            mean = sum(times) / len(times) if times else tmax
            if times and not tmin - q / len(times) <= mean <= tmax + q / len(times):
                v.append(Violation("rate", w[0].rise, "mean bit %.4f us" % mean))
            for k, p in enumerate(w):
                width = self.us(p.fall - p.rise)
                if not wmin - q <= width <= wmax + q:
                    v.append(Violation("width", p.rise, "bit %d: %.3f us" % (k + 1, width)))
                bt = times[k] if k < len(times) else mean
                if abs(width / bt - RATIO_NOM) > RATIO_NOM * RATIO_TOL:
                    v.append(Violation("ratio", p.rise, "bit %d: %.4f" % (k + 1, width / bt)))
                if slew_us is not None and slew_us >= width:
                    v.append(Violation("slew", p.rise, "pulse %.3f us, slew %.3f" % (width, slew_us)))
            if n + 1 < len(groups):
                gap = self.us(groups[n + 1][0].rise - w[-1].rise) - mean
                if gap < GAP_BITS_MIN * mean - 1e-9:
                    v.append(Violation("gap", w[-1].fall, "%.3f bit times" % (gap / mean)))
        return v

    def report(self) -> str:                                # pragma: no cover
        return "%d pulses, %d words, violations %s" % (
            len(self.pulses), len(self.words), [str(x) for x in self.violations[:4]])
