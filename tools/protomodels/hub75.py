"""A HUB75 RGB LED matrix panel at its connector: 32 x 16 pixels, 1/8 scan.

Written from the panel's descriptions and a column driver's data sheet,
not from any firmware (``firmware/hub75.loom`` is checked against it).

Sources. There is no HUB75 standard.

* Panel (secondary): Adafruit, "32x16 and 32x32 RGB LED Matrix", section
  "How the Matrix Works": the 32 x 16 panel is "divided into 8 interleaved
  sections/strips", the first being "the 1st 'line' and the 9th 'line'",
  chosen with the A, B and C address pins; the controller "clocks out 192
  bits of data (24 bytes) and latches it" per section; the panel has no
  PWM of its own. H. Zeller, rpi-rgb-led-matrix, ``wiring.md``: the
  signals R1 G1 B1 (upper half) R2 G2 B2 (lower half), A B C (D for 32
  rows at 1:16), clock, strobe (also called latch or LAT), OE- (output
  enable, active low).
* Column drivers: 74HC595-style constant-current shift registers with an
  output latch and an output enable (ICN2037, MBI5124 and the like, whose
  data sheets could not be read for this model: unchecked). Their logic
  and timing are taken from TI TLC5916, SLVS695D (June 2007, revised
  January 2015): section 6 Pin Functions: CLK "for data shift on rising
  edge"; LE: "Serial data is transferred to the respective latch when LE
  is high" and "latched when LE goes low"; OE: low enables the output
  drivers, high blanks them. Section 7.9 Timing Requirements (VDD 3 to
  5.5 V, normal mode), in :data:`TLC5916`. The direction of each interval
  (tsu(L) from the last CLK rising edge to LE rising, th(L) from LE falling
  to the next CLK rising edge) is read from the parameter names: the
  timing diagram itself was not read (unchecked).
* Shift direction: not given by either panel source (unchecked). The model
  takes the usual 74HC595 chain: the bit clocked in last is nearest the
  connector (column 0), so after 32 clocks the first bit is at column 31.

The model registers its lines as the bench resolves them, one sample per
clock, so every interval is a whole number of clocks; ``clock`` converts
them to nanoseconds for the checks. It integrates, per pixel and colour,
the clocks during which OE is low with that pixel's latched bit set on the
row pair A B C selects (:meth:`on_time`), which recovers what the eye
averages; and it records as a violation every timing miss and every clock
at which the panel is lit while the address or the latch changes (the
ghosting a driver must avoid by blanking).
"""

from __future__ import annotations

from typing import Dict, List, Optional, Sequence, Tuple

from .bench import Lines, Model, PinSpec, pad_of

#: TLC5916 SLVS695D section 7.9, normal mode, in ns (minimums), and the
#: 30 MHz clock of its Features list.
TLC5916 = {
    "tw_clk": 20.0,         # CLK pulse duration, high and low
    "tw_le": 20.0,          # LE pulse duration
    "tw_oe": 500.0,         # OE pulse duration, IOUT < 60 mA (700 above)
    "tsu_d": 3.0,           # SDI setup to CLK rising
    "th_d": 2.0,            # SDI hold after CLK rising
    "tsu_le": 15.0,         # LE: last CLK rising to LE rising (unchecked)
    "th_le": 15.0,          # LE: LE falling to next CLK rising (unchecked)
    "f_clk_max": 30e6,
}

DATA = ("R1", "G1", "B1", "R2", "G2", "B2")
#: Default wiring of the 12 inputs (firmware/hub75.loom's pins).
PINS = {"R1": "BIDIR0", "G1": "BIDIR1", "B1": "BIDIR2", "R2": "BIDIR3",
        "G2": "BIDIR4", "B2": "BIDIR5", "CLK": "BIDIR6", "OE": "BIDIR7",
        "A": "OUT0", "B": "OUT1", "C": "OUT2", "LAT": "OUT3"}


class Pulse:
    """One interval with OE low: ``[start, end)`` cycles, the row pair, and
    the six latched rows of ``width`` bits (bit x = column x)."""

    __slots__ = ("start", "end", "addr", "latch")

    def __init__(self, start: int, addr: int, latch: Tuple[int, ...]) -> None:
        self.start, self.end, self.addr, self.latch = start, None, addr, latch

    @property
    def clocks(self) -> int:
        return self.end - self.start

    def __repr__(self) -> str:
        return "Pulse(%d..%s, addr %d)" % (self.start, self.end, self.addr)


class Hub75Panel(Model):
    """The panel: six column shift registers, their output latch, the row
    decoder and OE, with the TLC5916 checks and the on-time integration.

    Args:
        clock: the bench clock in Hz, to turn clocks into ns.
        width: columns (32). scan: row pairs (8 for 1/8 scan: A B C; 16
            adds D).
        pins: overrides of :data:`PINS`, signal name to pin.
    """

    def __init__(self, *, clock: float = 50e6, width: int = 32, scan: int = 8,
                 pins: Optional[Dict[str, PinSpec]] = None,
                 timing: Optional[Dict[str, float]] = None) -> None:
        self.clock = float(clock)
        self.width, self.scan = width, scan
        self.mask = (1 << width) - 1
        self.timing = dict(timing or TLC5916)
        self.addr_names = ("A", "B", "C", "D")[:max(scan - 1, 1).bit_length()]
        names = dict(PINS, **(pins or {}))
        if scan > 8:
            names.setdefault("D", "OUT4")
        self.pads = {n: pad_of(names[n]) for n in
                     DATA + ("CLK", "LAT", "OE") + self.addr_names}
        self.sr = [0] * 6
        self.latch = [0] * 6
        self.prev: Optional[Dict[str, int]] = None
        self.last_data: Optional[int] = None    # cycle of the last data change
        self.last_rise = self.last_fall = None
        self.le_rise = self.le_fall = None
        self.pulse: Optional[Pulse] = None
        self.pulses: List[Pulse] = []
        self.latches: List[Tuple[int, int, Tuple[int, ...]]] = []
        self.violations: List[Tuple[int, str, float]] = []
        self.measured: Dict[str, List[float]] = {k: [] for k in self.timing}
        self.clocks_shifted = 0

    def ns(self, clocks: float) -> float:
        return clocks * 1e9 / self.clock

    def _check(self, name: str, clocks: int, cycle: int) -> None:
        value = self.ns(clocks)
        self.measured[name].append(value)
        if value < self.timing[name]:
            self.violations.append((cycle, name, value))

    def address(self, v: Dict[str, int]) -> int:
        return sum(v[n] << i for i, n in enumerate(self.addr_names))

    # ------------------------------------------------------------ the pins
    def observe(self, lines: Lines) -> None:
        c = lines.cycle
        v = {n: lines.get(p) for n, p in self.pads.items()}
        p = self.prev
        self.prev = v
        if p is None:
            if v["OE"] == 0:
                self._open(c, v)
            return
        if any(v[n] != p[n] for n in DATA):
            if self.last_rise is not None:
                self._check("th_d", c - self.last_rise, c)
            self.last_data = c
        if v["CLK"] and not p["CLK"]:
            self._rise(c, v)
        elif p["CLK"] and not v["CLK"] and self.last_rise is not None:
            self._check("tw_clk", c - self.last_rise, c)
            self.last_fall = c
        if v["LAT"] and not p["LAT"]:
            if self.last_rise is not None:
                self._check("tsu_le", c - self.last_rise, c)
            self.le_rise = c
        if v["LAT"]:
            self.latch = list(self.sr)          # transparent while LE is high
        elif p["LAT"]:
            if self.le_rise is not None:
                self._check("tw_le", c - self.le_rise, c)
            self.le_fall = c
            self.latches.append((c, self.address(v), tuple(self.latch)))
        lit, was_lit = v["OE"] == 0, p["OE"] == 0
        if any(v[n] != p[n] for n in self.addr_names) and (lit or was_lit):
            self.violations.append((c, "address changed while lit", 0.0))
        if lit and v["LAT"]:
            self.violations.append((c, "latch open while lit", 0.0))
        if lit and not was_lit:
            self._open(c, v)
        elif was_lit and not lit:
            self._close(c)

    def _rise(self, c: int, v: Dict[str, int]) -> None:
        if self.last_rise is not None:
            period = self.ns(c - self.last_rise)
            self.measured.setdefault("t_clk", []).append(period)
            if period < 1e9 / self.timing["f_clk_max"]:
                self.violations.append((c, "f_clk_max", period))
        if self.last_fall is not None:
            self._check("tw_clk", c - self.last_fall, c)
        if self.last_data is not None and (self.last_rise is None
                                           or self.last_data > self.last_rise):
            self._check("tsu_d", c - self.last_data, c)
        if self.le_fall is not None and (self.last_rise is None
                                         or self.le_fall > self.last_rise):
            self._check("th_le", c - self.le_fall, c)
        for i, n in enumerate(DATA):
            self.sr[i] = ((self.sr[i] << 1) | v[n]) & self.mask
        self.last_rise = c
        self.clocks_shifted += 1

    def _open(self, c: int, v: Dict[str, int]) -> None:
        self.pulse = Pulse(c, self.address(v), tuple(self.latch))

    def _close(self, c: int) -> None:
        pulse, self.pulse = self.pulse, None
        if pulse is None:
            return
        pulse.end = c
        self._check("tw_oe", c - pulse.start, c)
        self.pulses.append(pulse)

    # ------------------------------------------------------- what is seen
    def pixel_of(self, addr: int, line: int, x: int) -> Tuple[int, int, int]:
        """``(y, x, colour)`` lit by bit ``x`` of data line ``line`` (0..5,
        R1 G1 B1 R2 G2 B2) on row pair ``addr``: the upper half's row
        ``addr``, the lower half's ``addr + scan``. Colour 0 R, 1 G, 2 B."""
        return addr + self.scan * (line // 3), x, line % 3

    def on_time(self, pulses: Optional[Sequence[Pulse]] = None,
                start: Optional[int] = None, end: Optional[int] = None
                ) -> List[List[List[int]]]:
        """Clocks lit per pixel and colour, ``[y][x][colour]``, over the
        closed ``pulses`` (default all), clipped to ``[start, end)``."""
        rows = 2 * self.scan
        on = [[[0, 0, 0] for _ in range(self.width)] for _ in range(rows)]
        for pulse in self.pulses if pulses is None else pulses:
            lo = pulse.start if start is None else max(pulse.start, start)
            hi = pulse.end if end is None else min(pulse.end, end)
            if hi <= lo:
                continue
            for line, bits in enumerate(pulse.latch):
                for x in range(self.width):
                    if (bits >> x) & 1:
                        y, xx, colour = self.pixel_of(pulse.addr, line, x)
                        on[y][xx][colour] += hi - lo
        return on

    def lit_clocks(self, pulses: Optional[Sequence[Pulse]] = None) -> int:
        """Clocks with OE low in ``pulses`` (default all closed ones)."""
        return sum(p.clocks for p in (self.pulses if pulses is None else pulses))


def levels(on: List[List[List[int]]], unit: int) -> List[List[Tuple[int, ...]]]:
    """Brightness levels from :meth:`Hub75Panel.on_time`: each on-time in
    ``unit`` clocks (the least significant plane's share), which must divide
    it exactly; a remainder means the planes were not lit for whole
    multiples of one weight, and raises."""
    out = []
    for row in on:
        line = []
        for pixel in row:
            for value in pixel:
                if value % unit:
                    raise ValueError("on-time %d is not a multiple of %d"
                                     % (value, unit))
            line.append(tuple(value // unit for value in pixel))
        out.append(line)
    return out
