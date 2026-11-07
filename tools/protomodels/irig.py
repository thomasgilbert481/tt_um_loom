"""IRIG-B (B00x, dc level shift) reference models, for firmware/irig_b_gen.loom
and irig_b_rx.loom.

Written from IRIG Standard 200-16, *IRIG Serial Time Code Formats* (Range
Commanders Council, August 2016), not from the firmware. Read 2026-11-07 from
the RCC document mirror at irig106.org (the 200-16 PDF). The figures, and
where each comes from:

* Format B is 100 bits a second, an index count interval of 10 ms (5.3,
  Table 3-1, Table 5-6), a frame of 1 s (Table 3-2) and 100 index counts.
* Every bit starts with its leading edge, which is its on-time point (3.3).
  Pulse-width coding (3.6, 3.8, 3.9, 5.3, Table 5-6): a binary 0 and an index
  marker last 2 ms (0.2 of the interval), a binary 1 5 ms (0.5), a position
  identifier and the reference bit 8 ms (0.8, 3.4, 3.5). In the dc level
  shift form the bit is the high level (Table 5-6 "dc level"; the high-is-
  the-pulse reading is the usual one, the text does not say "high").
* The frame starts with two consecutive 8 ms bits, P0 then Pr; the leading
  edge of Pr is the on-time reference point, index count 0 (3.3, 3.4, 5.3).
  P0 occurs one index count before Pr and P1..P9 every tenth count after
  it, at 9, 19, ..., 89, with P0 at 99 (3.5, Table 5-4).
* BCD time of year, least significant bit first, an index marker (a 0)
  between the decimal digits (5.3, Table 3-3, Table 5-4): seconds 1-4 (1, 2,
  4, 8) and 6-8 (10, 20, 40); minutes 10-13, 15-17; hours 20-23, 25-26; days
  30-33, 35-38 (10, 20, 40, 80), 40-41 (100, 200); 42-48 index markers. The
  bit times in Table 5-4 refer to the leading edge of Pr (its note 1), so
  a frame carries the time of its own Pr.
* Year (B004-B007): units 50-53, tens 55-58, 54 unused (Table 3-3, Table
  5-5). Table 5-5 prints the tens weights as 10, 20, 20, 20: taken as 10,
  20, 40, 80 (the units row is 1, 2, 4, 8; W-075 QUESTIONS.md item 1).
* 18 control functions at 60-68 and 70-78 (5.3, Table 3-4, Table 5-5),
  user-defined (3.8, 4.1).
* Straight binary seconds of the day, 2^0 at 80 to 2^16 at 97, P9 between
  2^8 and 2^9; 98 is an index marker (3.7, 5.3).
* Coded expressions, the last designation digit (Table 4-1, Table 4-3):
  0 BCD TOY, CF, SBS; 1 BCD TOY, CF; 2 BCD TOY; 3 BCD TOY, SBS; 4 BCD TOY,
  BCD YEAR, CF, SBS; 5 BCD TOY, BCD YEAR, CF; 6 BCD TOY, BCD YEAR; 7 BCD
  TOY, BCD YEAR, SBS. A word a signal does not carry is sent as index
  markers (3.9: every position not assigned is an index marker, 2 ms).
* Counting (3.6): the BCD TOY reads 0 h 0 min 0 s at 2400 each day and day
  001 at 2400 of day 365, or 366 in a leap year; the year counts to 2099;
  SBS reads 0 at 2400 (leap-second days excepted: not modelled). The year
  is two BCD digits, so for 2000..2099 a year is leap when yy % 4 == 0.
* Hardware (Appendix C, Table C-1, "minimum design considerations" for a
  generator): format B dc level shift rise time 1 us (10 to 90 %), jitter
  200 ns pulse to pulse at the 50 % points of successive leading edges
  (3.1, 3.2). 200-16 gives no width tolerance for the dc level shift bits.

Contents:

* :class:`Time`, :func:`frame_symbols`: a time and its frame of 100 symbols
  ``"0"``, ``"1"``, ``"P"`` for any coded expression 0..7.
* :func:`decode_symbols`: a frame of symbols back to the time, checking every
  marker position and every index marker.
* :class:`IrigSource`: a bench model that drives a reader pin with frames
  at any element length in clocks, with injected faults.
* :class:`EdgeRecorder`: a pad's edges; :func:`elements`: the timing checker,
  which turns edges into symbols with their leading-edge cycles and holds
  every element to its nominal width and the 10 ms grid; :func:`frames`
  finds P0 Pr and decodes.
"""

from __future__ import annotations

import dataclasses
from typing import Dict, List, Optional, Sequence, Tuple

from .bench import Drive, Lines, Model, PinSpec, pad_of

ELEMENTS = 100                      # index counts a frame (Table 3-1, 3-2)
ELEMENT_MS = 10.0                   # index count interval (5.3)
WIDTH_MS = {"0": 2.0, "1": 5.0, "P": 8.0}           # Table 5-6
WIDTH_FRACTION = {k: v / ELEMENT_MS for k, v in WIDTH_MS.items()}
MARKERS = (0,) + tuple(range(9, 100, 10))           # Pr, P1..P9, P0 (3.5)
RISE_TIME_US = 1.0                  # Table C-1, format B, dc level
JITTER_NS = 200.0                   # Table C-1, pulse to pulse

#: (index, weight) of the BCD digits (Table 3-3, Table 5-4, Table 5-5)
SECONDS = [(1, 1), (2, 2), (3, 4), (4, 8), (6, 10), (7, 20), (8, 40)]
MINUTES = [(10, 1), (11, 2), (12, 4), (13, 8), (15, 10), (16, 20), (17, 40)]
HOURS = [(20, 1), (21, 2), (22, 4), (23, 8), (25, 10), (26, 20)]
DAYS = [(30, 1), (31, 2), (32, 4), (33, 8), (35, 10), (36, 20), (37, 40),
        (38, 80), (40, 100), (41, 200)]
YEARS = [(50, 1), (51, 2), (52, 4), (53, 8), (55, 10), (56, 20), (57, 40), (58, 80)]
CF = list(range(60, 69)) + list(range(70, 79))      # 18 control functions
SBS = [(i, 1 << b) for b, i in enumerate(list(range(80, 89)) + list(range(90, 98)))]


def has_year(z: int) -> bool:
    return z in (4, 5, 6, 7)


def has_cf(z: int) -> bool:
    return z in (0, 1, 4, 5)


def has_sbs(z: int) -> bool:
    return z in (0, 3, 4, 7)


def bcd(value: int) -> int:
    """``value`` (0..999) as packed BCD."""
    return (value // 100) << 8 | (value // 10 % 10) << 4 | value % 10


def leap(year: int) -> bool:
    """Two-digit year 00..99 of 2000..2099 (3.6 counts the year to 2099)."""
    return year % 4 == 0


@dataclasses.dataclass(frozen=True)
class Time:
    """A time of year as IRIG-B carries it: year 0..99, day 1..366."""

    year: int
    day: int
    hour: int
    minute: int
    second: int

    @property
    def sbs(self) -> int:
        """Straight binary seconds of the day (3.6, 3.7)."""
        return self.hour * 3600 + self.minute * 60 + self.second

    def next(self) -> "Time":
        """One second later, by the counting rules of 3.6 (no leap seconds)."""
        y, d, h, m, s = self.year, self.day, self.hour, self.minute, self.second + 1
        if s == 60:
            s, m = 0, m + 1
        if m == 60:
            m, h = 0, h + 1
        if h == 24:
            h, d = 0, d + 1
        if d > (366 if leap(y) else 365):
            d, y = 1, (y + 1) % 100
        return Time(y, d, h, m, s)

    def later(self, seconds: int) -> "Time":
        t = self
        for _ in range(seconds):
            t = t.next()
        return t


# ------------------------------------------------------------- the frame
def frame_symbols(t: Time, z: int = 0, cf: int = 0) -> List[str]:
    """The 100 symbols of the frame whose Pr is at ``t``, signal B00``z``.
    ``cf`` gives the 18 control functions, bit 0 at index 60 (sent only when
    the coded expression carries CF; index markers otherwise)."""
    sym = ["0"] * ELEMENTS
    for i in MARKERS:
        sym[i] = "P"

    def put(table, value):
        # weight w of decimal unit u (1, 10, 100) is bit log2(w / u) of
        # that digit, so its bit in the packed BCD is 4 log10(u) + log2(w / u)
        digits = bcd(value)
        for index, weight in table:
            unit = 100 if weight >= 100 else 10 if weight >= 10 else 1
            shift = {1: 0, 10: 4, 100: 8}[unit] + (weight // unit).bit_length() - 1
            if (digits >> shift) & 1:
                sym[index] = "1"

    put(SECONDS, t.second)
    put(MINUTES, t.minute)
    put(HOURS, t.hour)
    put(DAYS, t.day)
    if has_year(z):
        put(YEARS, t.year)
    if has_cf(z):
        for b, index in enumerate(CF):
            if (cf >> b) & 1:
                sym[index] = "1"
    if has_sbs(z):
        for index, weight in SBS:
            if t.sbs & weight:
                sym[index] = "1"
    return sym


@dataclasses.dataclass
class Decoded:
    """A frame read back: the BCD fields as numbers, the 17-bit SBS, the 18
    CF bits, and ``errors`` naming every rule of 200-16 the frame breaks
    ("marker <i>": a position identifier missing or misplaced, "digit": a
    BCD digit above 9, "index <i>": an index marker that is not a 0)."""

    year: int
    day: int
    hour: int
    minute: int
    second: int
    sbs: int
    cf: int
    errors: List[str]

    @property
    def time(self) -> Time:
        return Time(self.year, self.day, self.hour, self.minute, self.second)


INDEX_MARKERS = [i for i in range(ELEMENTS)
                 if i not in MARKERS and i not in dict(SECONDS + MINUTES + HOURS + DAYS + YEARS)
                 and i not in CF and i not in dict(SBS)]


def decode_symbols(sym: Sequence[str], z: int = 0) -> Decoded:
    """Read a frame of 100 symbols (index 0 is Pr) for signal B00``z``."""
    assert len(sym) == ELEMENTS
    errors = ["marker %d" % i for i in range(ELEMENTS)
              if (sym[i] == "P") != (i in MARKERS)]
    errors += ["index %d" % i for i in INDEX_MARKERS if sym[i] != "0"]

    def field(table):
        digits = {}
        value = 0
        for index, weight in table:
            if sym[index] == "1":
                unit = 100 if weight >= 100 else 10 if weight >= 10 else 1
                digits[unit] = digits.get(unit, 0) + weight // unit
                value += weight
        if any(d > 9 for d in digits.values()):
            errors.append("digit")
        return value

    sec, mi, hr, dy = field(SECONDS), field(MINUTES), field(HOURS), field(DAYS)
    yr = field(YEARS) if has_year(z) else 0
    sbs = sum(w for i, w in SBS if sym[i] == "1")
    cf = sum(1 << b for b, i in enumerate(CF) if sym[i] == "1")
    return Decoded(yr, dy, hr, mi, sec, sbs, cf, errors)


# ------------------------------------------------------------ bench models
Element = object                    # "0", "1", "P", or a width as a fraction


class IrigSource(Model):
    """Drives a reader pin with IRIG-B dc level shift: each element high for
    its width, low for the rest of the interval, idle low. :meth:`send`
    queues elements back to back at ``element`` clocks each (a float: the
    edges are rounded to whole clocks from the start of the burst, so they
    do not drift). An element is a symbol or, for fault injection, a width
    as a fraction of the interval (0 is a missing pulse). Returns the cycle
    of the first leading edge; :attr:`rises` lists every leading edge and
    :attr:`leads` every element's start, pulse or not."""

    def __init__(self, pin: PinSpec) -> None:
        self.pad = pad_of(pin)
        self.level = 0
        self.free_at = 0
        self.rises: List[int] = []
        self.leads: List[int] = []
        self._changes: List[Tuple[int, int]] = []

    def send(self, symbols: Sequence[Element], element: float, at: int) -> int:
        start = max(int(at), self.free_at)
        for k, s in enumerate(symbols):
            width = WIDTH_FRACTION[s] if isinstance(s, str) else float(s)
            lead = start + int(round(k * element))
            self.leads.append(lead)
            if width > 0:
                self.rises.append(lead)
                self._changes += [(lead, 1), (lead + int(round(width * element)), 0)]
        self.free_at = start + int(round(len(symbols) * element))
        self._changes.sort()
        return start

    def drive(self, drive: Drive, cycle: int) -> None:
        changes = self._changes
        while changes and changes[0][0] <= cycle:
            self.level = changes.pop(0)[1]
        drive.set(self.pad, self.level)


class EdgeRecorder(Model):
    """Every change of one pad: ``edges`` holds ``(cycle, level)``, the cycle
    being the first with the new level. The level before the first edge is
    ``idle`` (dc level shift idles low)."""

    def __init__(self, pin: PinSpec, idle: int = 0) -> None:
        self.pad = pad_of(pin)
        self.level = idle
        self.edges: List[Tuple[int, int]] = []

    def observe(self, lines: Lines) -> None:
        level = lines.get(self.pad)
        if level != self.level:
            self.edges.append((lines.cycle, level))
            self.level = level

    def rises(self) -> List[int]:
        return [c for c, lv in self.edges if lv]


class Wire(Model):
    """The board wire from a chip output to a chip input, one clock late."""

    def __init__(self, src: PinSpec, dst: PinSpec) -> None:
        self.src, self.dst = pad_of(src), pad_of(dst)
        self.level = 0

    def drive(self, drive: Drive, cycle: int) -> None:
        drive.set(self.dst, self.level)

    def observe(self, lines: Lines) -> None:
        self.level = lines.get(self.src)


# --------------------------------------------------------- timing checker
@dataclasses.dataclass
class Pulse:
    lead: int                       # cycle of the leading edge
    width: int                      # clocks high
    symbol: Optional[str]           # "0", "1", "P", or None: no nominal width


def elements(edges: Sequence[Tuple[int, int]], element: float,
             tolerance: float = 0.0) -> List[Pulse]:
    """The pulses of a recorded pin (high from a rise to the next fall), each
    named by the nominal width of Table 5-6 it is within ``tolerance``
    clocks of. With ``tolerance`` 0 a width must be exactly 2, 5 or 8 ms."""
    out = []
    lead = None
    for cycle, level in edges:
        if level:
            lead = cycle
        elif lead is not None:
            width = cycle - lead
            sym = None
            for s, f in WIDTH_FRACTION.items():
                if abs(width - f * element) <= tolerance:
                    sym = s
            out.append(Pulse(lead, width, sym))
            lead = None
    return out


@dataclasses.dataclass
class Frame:
    pr: int                         # cycle of Pr's leading edge (on time)
    decoded: Decoded
    grid_errors: List[int]          # indices whose leading edge is off the grid


def frames(pulses: Sequence[Pulse], element: float, z: int = 0,
           tolerance: float = 0.0, first_is_pr: bool = False) -> List[Frame]:
    """Every whole frame in ``pulses``: a Pr is a P whose previous pulse is
    a P (3.4), or with ``first_is_pr`` the first pulse if it is a P (a
    generator's first frame has no P0 before it); the frame is decoded and
    each leading edge held to ``pr + k * element`` within ``tolerance``
    clocks (3.3)."""
    out = []
    i = 0 if first_is_pr else 1
    while i + ELEMENTS <= len(pulses):
        if pulses[i].symbol == "P" and (i == 0 or pulses[i - 1].symbol == "P"):
            run = pulses[i:i + ELEMENTS]
            pr = run[0].lead
            grid = [k for k, p in enumerate(run)
                    if abs(p.lead - pr - k * element) > tolerance]
            sym = [p.symbol or "?" for p in run]
            out.append(Frame(pr, decode_symbols(sym, z), grid))
            i += ELEMENTS
        else:
            i += 1
    return out
