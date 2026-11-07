"""X10 powerline carrier at logic level: codes, frames and pin models.

Written 2026-11-07 (W-067) from the X10 sources below, not from any
firmware. Logic level as on a TW523 or PL513 interface: ``ZC`` is the
60 (or 50) Hz zero-crossing square wave, every edge a zero crossing; ``TX``
carries the 120 kHz bursts; ``RX`` is the receive envelope, high while a
burst is present.

Sources:

* [PLC] X10 Ltd., technical note "X-10 Powerline Carrier (PLC) Technology",
  as reproduced at https://kbase.x10.com/wiki/PLC (read 2026-11-07):
  - a 1 is "a 1 millisecond burst of 120 kHz at the zero crossing point",
    a 0 its absence;
  - transmission "within 200 microseconds of the zero crossing point";
  - signal envelope "1 ms (-50 +100)" us (the page drops the "u");
  - eleven power-line cycles a frame: two for the start code, four for
    the house code, five for the key code;
  - the start code "is always 1110" and is the only code not sent in
    true/complement form; every other bit is sent true then complement
    on alternate half-cycles;
  - frames "in groups of 2 with 3 power line cycles between each group";
    bright and dim "continuously (at least twice) with no gaps";
  - the 1 ms bursts "transmitted three times to coincide with the zero
    crossing point of all three phases".
* [WP] Wikipedia, "X10 (industry standard)" (read 2026-11-07): the house
  code table (A 0110 ... P 1100, as H1 H2 H4 H8), the unit codes 1..16 in
  the same order with D16 = 0, and the function codes (D1 D2 D4 D8, with
  D16 = 1). [PLC]'s own table (its Figure 4) is an image that could not
  be read, so the tables are from [WP] only: unchecked against the
  primary source.
* [TW] X10 PL513/TW523 technical note (authinx.com/manuals/X10/
  technicalnote.pdf): downloaded but a scanned image; nothing in it read.
  Where it would decide something (the RX output polarity and sample
  point) the brief's logic-level definition is used and marked.

Bit order in the 9-bit command value used here and by the programs:
bit 8 = H1 (sent first) ... bit 5 = H8, bit 4 = D1 ... bit 0 = D16.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Sequence, Tuple

from tools.protomodels.bench import Drive, Lines, Model, pad_of

CARRIER_HZ = 120e3                  # [PLC]
BURST_S = 1e-3                      # [PLC]
BURST_MIN_S, BURST_MAX_S = 1e-3 - 50e-6, 1e-3 + 100e-6   # [PLC] envelope
ZC_WINDOW_S = 200e-6                # [PLC]
FRAME_HALVES = 22                   # [PLC]: eleven cycles
GAP_HALVES = 6                      # [PLC]: three cycles between pairs
START = (1, 1, 1, 0)                # [PLC]

# [WP], H1 H2 H4 H8; unit n uses the n-th row
_ROWS = ("0110", "1110", "0010", "1010", "0001", "1001", "0101", "1101",
         "0111", "1111", "0011", "1011", "0000", "1000", "0100", "1100")
HOUSE: Dict[str, int] = {chr(ord("A") + i): int(r, 2) for i, r in enumerate(_ROWS)}
# [WP], D1 D2 D4 D8 (D16 = 1 is added by function())
FUNCTION: Dict[str, int] = {
    "ALL_UNITS_OFF": 0b0000, "ALL_LIGHTS_ON": 0b0001, "ON": 0b0010,
    "OFF": 0b0011, "DIM": 0b0100, "BRIGHT": 0b0101, "ALL_LIGHTS_OFF": 0b0110,
    "EXTENDED_CODE": 0b0111, "HAIL_REQUEST": 0b1000, "HAIL_ACK": 0b1001,
    "PRESET_DIM": 0b1010, "STATUS_ON": 0b1101, "STATUS_OFF": 0b1110,
    "STATUS_REQUEST": 0b1111}


def unit(n: int) -> int:
    """Key code of unit ``n`` (1..16): D1 D2 D4 D8 then D16 = 0."""
    return int(_ROWS[n - 1], 2) << 1


def function(name: str) -> int:
    """Key code of a function: D1 D2 D4 D8 then D16 = 1."""
    return FUNCTION[name] << 1 | 1


def command(house: str, key: int) -> int:
    """The 9-bit value: house code in 8..5, key code in 4..0."""
    return HOUSE[house] << 5 | key


def frame_halves(value: int) -> List[int]:
    """One frame, one entry per half-cycle (1 = burst): 22 entries."""
    halves = list(START)
    for i in range(8, -1, -1):
        b = (value >> i) & 1
        halves += [b, 1 - b]
    return halves


def pair_halves(value: int, gap: bool = True) -> List[int]:
    """A frame twice, back to back, then six clear half-cycles."""
    return frame_halves(value) * 2 + ([0] * GAP_HALVES if gap else [])


def decode_halves(halves: Sequence[int]) -> List[Tuple[str, int]]:
    """Frames found in a half-cycle sequence, from the rules of [PLC]:
    ``("ok", value)`` for a frame whose every bit is followed by its
    complement, ``("bad", position)`` otherwise. A start code is three
    bursts and a clear half-cycle; 111 cannot occur in true/complement
    data, so a search for it is unambiguous."""
    out: List[Tuple[str, int]] = []
    i = 0
    n = len(halves)
    while i + 4 <= n:
        if tuple(halves[i:i + 4]) != START:
            i += 1
            continue
        i += 4
        if i + 18 > n:
            break
        value, bad = 0, None
        for k in range(9):
            a, b = halves[i + 2 * k], halves[i + 2 * k + 1]
            if a == b and bad is None:
                bad = k
            value = value << 1 | a
        out.append(("ok", value) if bad is None else ("bad", bad))
        i += 18 if bad is None else 2 * bad + 2
    return out


# ------------------------------------------------------------- pin models
class Mains(Model):
    """The ZC square wave on one or more pins: level flips every
    ``half`` clocks (a float, so 60 Hz at 50 MHz is exact on average),
    the first edge at ``first``. ``edges`` lists the cycles from which the
    new level is driven."""

    def __init__(self, pins, half: float, first: int = 100, level: int = 0):
        pins = pins if isinstance(pins, (list, tuple)) else [pins]
        self.pads = [pad_of(p) for p in pins]
        self.half = float(half)
        self.level = level
        self.n = 0
        self.first = first
        self.edges: List[int] = []

    def _next(self) -> int:
        return self.first + int(round(self.n * self.half))

    def drive(self, drive: Drive, cycle: int) -> None:
        if cycle >= self._next():
            self.level ^= 1
            self.edges.append(cycle)
            self.n += 1
        for pad in self.pads:
            drive.set(pad, self.level)


class EdgeLog(Model):
    """Every change of one line: ``(cycle, new level)``."""

    def __init__(self, pin, initial: int = 0):
        self.pad = pad_of(pin)
        self.level = initial
        self.edges: List[Tuple[int, int]] = []

    def observe(self, lines: Lines) -> None:
        v = lines.get(self.pad)
        if v != self.level:
            self.level = v
            self.edges.append((lines.cycle, v))


def bursts_of(edges: Sequence[Tuple[int, int]], quiet: int) -> List[Tuple[int, int, int]]:
    """Group carrier edges into bursts: ``(first rise, last fall, rises)``;
    a low stretch longer than ``quiet`` clocks ends a burst."""
    out: List[List[int]] = []
    for cycle, level in edges:
        if level:
            if out and cycle - out[-1][1] <= quiet:
                out[-1][2] += 1
            else:
                out.append([cycle, cycle, 1])
        elif out:
            out[-1][1] = cycle
    return [tuple(b) for b in out]


def halves_of(zc_edges: Sequence[int], bursts, window: int) -> List[int]:
    """One entry per half-cycle (from each ZC edge to the next): 1 if a
    burst starts within ``window`` clocks after the edge."""
    out = []
    starts = [b[0] for b in bursts]
    for e in zc_edges:
        out.append(int(any(e <= s <= e + window for s in starts)))
    return out


def check_bursts(zc_edges, bursts, clock_hz: float, mains_hz: float,
                 phases: int = 1) -> List[str]:
    """The [PLC] windows for every burst: it starts within 200 us after a
    zero crossing of one of ``phases`` phases (the ZC edge plus k/6 of a
    cycle, k < phases), its carrier lasts 1 ms -50/+100 us, and its mean
    carrier frequency is within 1 % of 120 kHz (that tolerance is not in
    the sources read: unchecked). Returns a list of violations."""
    errs = []
    win = ZC_WINDOW_S * clock_hz
    sixth = clock_hz / mains_hz / 6
    for first, last, rises in bursts:
        before = [e for e in zc_edges if e <= first]
        if not before:
            errs.append("burst at %d before any zero crossing" % first)
            continue
        off = first - before[-1]
        if not any(0 <= off - k * sixth <= win for k in range(phases)):
            errs.append("burst at %d: %d clocks after the ZC edge" % (first, off))
        length = (last - first) / clock_hz + 0.5 / CARRIER_HZ
        if not BURST_MIN_S <= length <= BURST_MAX_S:
            errs.append("burst at %d lasts %.1f us" % (first, length * 1e6))
        if rises > 1:
            f = (rises - 1) * clock_hz / (last - first - 0.5 * clock_hz / CARRIER_HZ)
            if abs(f / CARRIER_HZ - 1) > 0.01:
                errs.append("burst at %d: carrier %.0f Hz" % (first, f))
    return errs


class Envelope(Model):
    """A TW523 receive path between the chip's TX and RX pins (logic
    level): ``rx`` is high while a carrier edge has been seen on ``tx``
    within the last ``hold`` clocks. Registered: it drives from what it
    saw in earlier cycles."""

    def __init__(self, tx, rx, hold: int):
        self.tx, self.rx = pad_of(tx), pad_of(rx)
        self.hold = hold
        self.last = -10 ** 9
        self.prev = 0
        self.cycle = 0

    def drive(self, drive: Drive, cycle: int) -> None:
        self.cycle = cycle
        drive.set(self.rx, int(cycle - self.last <= self.hold))

    def observe(self, lines: Lines) -> None:
        v = lines.get(self.tx)
        if v != self.prev:
            self.prev = v
            self.last = lines.cycle


class Sender(Model):
    """A transmitter on the line, as the chip's RX pin sees it through a
    TW523: at each ZC edge it takes the next half-cycle of ``halves`` and,
    if 1, drives ``rx`` high from ``delay`` to ``delay + length`` clocks
    after the edge. ``halves`` may be extended while it runs."""

    def __init__(self, zc, rx, delay: int, length: int, halves=()):
        self.zc, self.rx = pad_of(zc), pad_of(rx)
        self.delay, self.length = delay, length
        self.halves: List[int] = list(halves)
        self.zc_level: Optional[int] = None
        self.edge = None
        self.bit = 0
        self.sent: List[int] = []

    def drive(self, drive: Drive, cycle: int) -> None:
        on = (self.edge is not None and self.bit
              and self.delay <= cycle - self.edge < self.delay + self.length)
        drive.set(self.rx, int(bool(on)))

    def observe(self, lines: Lines) -> None:
        v = lines.get(self.zc)
        if self.zc_level is None:
            self.zc_level = v
        elif v != self.zc_level:
            self.zc_level = v
            self.edge = lines.cycle
            self.bit = self.halves.pop(0) if self.halves else 0
            self.sent.append(self.bit)
