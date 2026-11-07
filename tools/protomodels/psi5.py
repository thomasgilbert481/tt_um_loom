"""PSI5 (Peripheral Sensor Interface 5) at logic level: a reference model.

Written from the protocol's published descriptions, not from the firmware
(``firmware/psi5_sensor.loom``, ``firmware/psi5_ecu.loom``). The PSI5
Technical Specification (psi5.org, v2.x) is downloadable only after accepting
the consortium's legal terms, which this work did not do; the figures below
come from vendor documents that cite it, and each says where it is from.
"Unchecked" marks what no source we read states; those are our choices.

Checked (two or more sources agree):

* Bit rate 125 kbit/s, optionally 189 kbit/s: Infineon, "AURIX TC4xx PSI5
  Peripheral Sensor Interface" training v1.0 (2024-09), slide 2; Keysight,
  "Triggering on and Decoding the PSI5 Sensor Serial Bus", p. 2. A bit is
  8 us at 125 kbit/s (Keysight p. 8).
* Manchester coding: a 0 is a rising edge (low to high current) in the
  middle of the bit, a 1 a falling edge (Keysight p. 3, figure 1; ADwin/CAS
  application note 22, p. 2; Infineon slide 4, "Manchester-2 (convention as
  per G.E. Thomas)"). Edges at bit boundaries carry nothing.
* Frame: two start bits "0 0", then the data LSB first, then one parity bit
  or a 3-bit CRC (Infineon slides 4-5; Keysight figure 2 and table 1,
  "based on PSI5 Technical Specification, v2.2": 13 to 33 bits, data region
  A of 10 to 24 bits, CRC sent C2 (MSB) first). 10 data bits with parity is
  the 13-bit standard frame (ADwin p. 2).
* Parity is even, recommended for 10 or fewer data bits; the CRC is
  g(x) = x^3 + x + 1 with seed 111 (Infineon slide 5).
* Synchronous modes time their slots from the sync pulse's rising edge
  (Zephyr devicetree binding ``psi5-controller.yaml``, ``start-offset-us``);
  the mode name P10P-500/3L means synchronous parallel bus, 10 data bits with
  parity, 500 us sync period, three slots, 125 kbit/s (PSI5 v2.0 summary on
  Scribd).

Unchecked (our choices, see QUESTIONS.md):

* The CRC covers D0..D9 only, not the start bits, and is the direct
  (non-reflected) register computation in transmission order with no final
  XOR. Since the start bits are 0 and the seed is not, including them would
  change the CRC; no source we read says.
* Slot offsets, the sync pulse's width and the sensor's bit-time tolerance.
  The tests use offsets and widths of their own and hold the Loom programs
  to exact timing; :data:`BIT_TOL` is the receiver window this model
  accepts, not a figure from the specification.
* The minimum gap between frames: 1.5 bits is the Keysight decoder's
  "Idle Bits" setting (p. 6), used as :data:`IDLE_BITS`.

Logic-level mapping (ours, stated in the program headers): DATA = 1 is the
sensor's high current and 0 the quiescent current (idle); SYNC = 1 during the
ECU's sync pulse.
"""

from __future__ import annotations

import dataclasses
from typing import Dict, List, Optional, Sequence, Tuple

from tools.protomodels.bench import Drive, Lines, Model, PinSpec, pad_of

RATES = {"125k": 125_000.0, "189k": 189_000.0}
POLY = 0b011                    # x^3 + x + 1 without the x^3 term
SEED = 0b111
START_BITS = (0, 0)
DATA_BITS = 10
BIT_TOL = 0.12                  # this model's receiver window (unchecked)
IDLE_BITS = 1.5                 # Keysight decoder default (unchecked)


def bit_clocks(rate: str, clock_hz: float) -> float:
    """Clocks per bit at ``rate`` ("125k" or "189k")."""
    return clock_hz / RATES[rate]


def parity(data: int, bits: int = DATA_BITS) -> int:
    """The even parity bit over ``bits`` data bits."""
    return bin(data & ((1 << bits) - 1)).count("1") & 1


def crc3(bits: Sequence[int], seed: int = SEED) -> int:
    """The 3-bit CRC of ``bits`` in transmission order: x^3 + x + 1, seed
    111, the bits entering at the register's top (C2 is bit 2)."""
    c = seed
    for b in bits:
        fb = ((c >> 2) & 1) ^ (b & 1)
        c = (c << 1) & 0b111
        if fb:
            c ^= POLY
    return c


def data_bits(data: int, bits: int = DATA_BITS) -> List[int]:
    """D0 .. D[bits-1], LSB first."""
    return [(data >> i) & 1 for i in range(bits)]


def frame_bits(data: int, crc: bool = False, *, check: Optional[int] = None) -> List[int]:
    """S1 S2 D0..D9 then P, or C2 C1 C0. ``check`` overrides the parity bit
    or the CRC value (error injection)."""
    d = data_bits(data)
    if crc:
        c = crc3(d) if check is None else check
        tail = [(c >> 2) & 1, (c >> 1) & 1, c & 1]
    else:
        tail = [parity(data) if check is None else check & 1]
    return list(START_BITS) + d + tail


def half_levels(bits: Sequence[int], violate: Optional[int] = None) -> List[int]:
    """Two levels a bit: 0 is (0, 1), a rising middle; 1 is (1, 0). Bit
    ``violate`` gets no mid-bit transition (both halves at its first level)."""
    out: List[int] = []
    for i, b in enumerate(bits):
        first = b
        out += [first, first if i == violate else 1 - first]
    return out


# ------------------------------------------------------------------ pins
class Wire(Model):
    """The board wire from a chip output to a chip input, one clock late,
    low before the first cycle (PSI5 idles at the quiescent current)."""

    def __init__(self, src: PinSpec, dst: PinSpec) -> None:
        self.src, self.dst = pad_of(src), pad_of(dst)
        self.level = 0

    def drive(self, drive: Drive, cycle: int) -> None:
        drive.set(self.dst, self.level)

    def observe(self, lines: Lines) -> None:
        self.level = lines.get(self.src)


class EdgeRecorder(Model):
    """Every change of one pad as ``(cycle, level)``, the cycle being the
    first with the new level; the level before the first edge is 0."""

    def __init__(self, pin: PinSpec, idle: int = 0) -> None:
        self.pad = pad_of(pin)
        self.level = idle
        self.edges: List[Tuple[int, int]] = []

    def observe(self, lines: Lines) -> None:
        level = lines.get(self.pad)
        if level != self.level:
            self.edges.append((lines.cycle, level))
            self.level = level


class Schedule:
    """A pin level as a list of timed changes, read by cycle."""

    def __init__(self) -> None:
        self.level = 0
        self.changes: List[Tuple[int, int]] = []
        self.free_at = 0

    def add(self, start: int, half: float, levels: Sequence[int]) -> int:
        """``levels`` one half-bit of ``half`` clocks each from ``start``,
        rounded from ``start`` so they do not drift, then back to 0."""
        for i, lv in enumerate(levels):
            self.changes.append((start + int(round(i * half)), lv))
        end = start + int(round(len(levels) * half))
        self.changes.append((end, 0))
        self.changes.sort()
        self.free_at = max(self.free_at, end)
        return end

    def at(self, cycle: int) -> int:
        ch = self.changes
        while ch and ch[0][0] <= cycle:
            self.level = ch.pop(0)[1]
        return self.level


class SyncSource(Model):
    """An ECU's sync pulses on a chip input: high for ``width`` clocks every
    ``period`` clocks from ``first``, ``count`` times (None: forever)."""

    def __init__(self, pin: PinSpec, period: int, width: int, first: int,
                 count: Optional[int] = None) -> None:
        self.pad = pad_of(pin)
        self.period, self.width, self.first, self.count = period, width, first, count

    def rises(self, upto: int) -> List[int]:
        out, t = [], self.first
        while t < upto and (self.count is None or len(out) < self.count):
            out.append(t)
            t += self.period
        return out

    def drive(self, drive: Drive, cycle: int) -> None:
        k, r = divmod(cycle - self.first, self.period)
        on = cycle >= self.first and r < self.width and (self.count is None or k < self.count)
        drive.set(self.pad, 1 if on else 0)


@dataclasses.dataclass
class Fault:
    """Error injection for one frame of :class:`Sensor`: ``flip`` sends data
    bit i inverted after the check was computed (a bit error), ``violate``
    leaves frame bit i without its mid-bit edge, ``check`` sends that parity
    bit or CRC value, ``late`` delays the frame by that many clocks."""

    flip: Optional[int] = None
    violate: Optional[int] = None
    check: Optional[int] = None
    late: int = 0


class Sensor:
    """A PSI5 sensor's data line (no pad of its own: a :class:`Bus` reads
    :meth:`level`). In synchronous mode it watches ``sync`` and sends the
    next queued frame ``offset_us`` after each rising edge (as the bench
    sees it, the cycle after the pad changed); :meth:`send` sends one at a
    given cycle. ``scale`` stretches its bit time (a slow or fast oscillator).
    A sync with nothing queued sends nothing."""

    def __init__(self, clock_hz: float, rate: str = "125k", *, crc: bool = False,
                 sync: Optional[PinSpec] = None, offset_us: float = 0.0,
                 scale: float = 1.0) -> None:
        self.clock_hz, self.crc, self.scale = float(clock_hz), crc, scale
        self.bit = bit_clocks(rate, clock_hz) * scale
        self.sync = pad_of(sync) if sync is not None else None
        self.offset = int(round(offset_us * self.clock_hz / 1e6))
        self.queue: List[Tuple[int, Fault]] = []
        self.sent: List[Tuple[int, int, Fault]] = []        # (S1 start, data, fault)
        self.line = Schedule()
        self._prev = 0

    def queue_frames(self, frames: Sequence[Tuple[int, Fault]]) -> None:
        self.queue += list(frames)

    def levels(self, data: int, fault: Fault) -> List[int]:
        bits = frame_bits(data, self.crc, check=fault.check)
        if fault.flip is not None:
            bits[2 + fault.flip] ^= 1
        return half_levels(bits, violate=fault.violate)

    def send(self, data: int, at: int, fault: Optional[Fault] = None) -> int:
        """Start S1 at cycle ``at`` (plus ``fault.late``); returns the end."""
        fault = fault or Fault()
        start = at + fault.late
        self.sent.append((start, data, fault))
        return self.line.add(start, self.bit / 2, self.levels(data, fault))

    def level(self, cycle: int) -> int:
        return self.line.at(cycle)

    def see(self, lines: Lines) -> None:
        if self.sync is None:
            return
        s = lines.get(self.sync)
        if s and not self._prev and self.queue:
            data, fault = self.queue.pop(0)
            self.send(data, lines.cycle + 1 + self.offset, fault)
        self._prev = s


class Bus(Model):
    """The PSI5 line at logic level, on one chip input: the OR of the chip
    outputs ``chips`` (each one clock late, as a board wire) and the model
    sensors ``sensors`` (current adds: any sensor at the high current makes
    the line 1). Two senders at once is a slot collision; the line shows
    their OR, as a comparator on the summed current would."""

    def __init__(self, dst: PinSpec, chips: Sequence[PinSpec] = (),
                 sensors: Sequence[Sensor] = ()) -> None:
        self.dst = pad_of(dst)
        self.chips = [pad_of(p) for p in chips]
        self.sensors = list(sensors)
        self._chip = 0

    def drive(self, drive: Drive, cycle: int) -> None:
        lv = self._chip
        for s in self.sensors:
            lv |= s.level(cycle)
        drive.set(self.dst, lv)

    def observe(self, lines: Lines) -> None:
        self._chip = 0
        for p in self.chips:
            self._chip |= lines.get(p)
        for s in self.sensors:
            s.see(lines)


# --------------------------------------------------------------- decoder
@dataclasses.dataclass
class Decoded:
    """A frame as read from the line. ``first`` is the cycle of its first
    edge (the rising middle of S1); ``mids`` the cycles of every mid-bit
    edge, S1's included; ``errors`` names each failed check: "start" (S2 is
    not 0), "check" (parity or CRC), "manchester" (a bit with no mid-bit
    edge in its window, reading stopped there), "edge" (an edge between
    windows that is not a boundary edge)."""

    first: int
    bits: List[int]
    mids: List[int]
    bounds: List[int]
    crc: bool
    errors: List[str]

    @property
    def start(self) -> float:
        """Where S1 began: half a bit before its middle (needs ``mids[1]``)."""
        bit = (self.mids[-1] - self.mids[0]) / max(1, len(self.mids) - 1)
        return self.first - bit / 2

    @property
    def data(self) -> int:
        return sum(b << i for i, b in enumerate(self.bits[2:2 + DATA_BITS]))

    @property
    def check(self) -> int:
        tail = self.bits[2 + DATA_BITS:]
        return tail[0] if not self.crc else (tail[0] << 2) | (tail[1] << 1) | tail[2]

    @property
    def ok(self) -> bool:
        return not self.errors


def frame_length(crc: bool) -> int:
    return 2 + DATA_BITS + (3 if crc else 1)


def decode(edges: Sequence[Tuple[int, int]], bit: float, crc: bool = False,
           since: int = 0, tol: float = 0.25) -> List[Decoded]:
    """Every frame in ``edges`` (``(cycle, level)`` as :class:`EdgeRecorder`
    keeps them) after cycle ``since``. A frame starts at a rising edge after
    at least :data:`IDLE_BITS` of low line; each next mid-bit edge must come
    ``bit * (1 +- tol)`` after the previous one, and edges half way between
    are boundary edges."""
    ev = [(c, lv) for c, lv in edges if c >= since]
    n = frame_length(crc)
    out: List[Decoded] = []
    i, last = 0, None
    while i < len(ev):
        c, lv = ev[i]
        idle = last is None or c - last >= IDLE_BITS * bit
        if lv != 1 or not idle:
            last = c
            i += 1
            continue
        d = Decoded(c, [0], [c], [], crc, [])
        i += 1
        while len(d.bits) < n:
            prev = d.mids[-1]
            lo, hi = prev + bit * (1 - tol), prev + bit * (1 + tol)
            while i < len(ev) and ev[i][0] < lo:
                if abs(ev[i][0] - prev - bit / 2) > bit * tol:
                    d.errors.append("edge")
                d.bounds.append(ev[i][0])
                i += 1
            if i >= len(ev) or ev[i][0] > hi:
                d.errors.append("manchester")
                break
            d.mids.append(ev[i][0])
            d.bits.append(0 if ev[i][1] else 1)
            i += 1
        if len(d.bits) == n:
            if d.bits[1] != 0:
                d.errors.append("start")
            want = crc3(d.bits[2:2 + DATA_BITS]) if crc else parity(d.data)
            if d.check != want:
                d.errors.append("check")
        last = ev[i - 1][0]
        out.append(d)
    return out


def pulses(edges: Sequence[Tuple[int, int]]) -> List[Tuple[int, int]]:
    """The sync pulses on a recorded line: ``(rise, fall)`` cycles."""
    out, rise = [], None
    for c, lv in edges:
        if lv:
            rise = c
        elif rise is not None:
            out.append((rise, c))
            rise = None
    return out


def check_timing(d: Decoded, bit: float, tol: float = BIT_TOL) -> List[str]:
    """Timing faults of a decoded frame against ``bit`` clocks: every
    mid-to-mid interval within ``bit * (1 +- tol)``, every boundary edge
    within ``tol`` bits of half way. Empty when it passes."""
    bad = []
    for a, b in zip(d.mids, d.mids[1:]):
        if abs((b - a) - bit) > bit * tol:
            bad.append("bit %d..%d is %d clocks" % (a, b, b - a))
    for e in d.bounds:
        prev = max(m for m in d.mids if m < e)
        if abs(e - prev - bit / 2) > bit * tol:
            bad.append("boundary edge at %d" % e)
    return bad


# ------------------------------------------- the ECU program's report word
SLOT2, E_CHECK, E_START, E_MANCH, E_NONE = 1, 2, 4, 8, 16


def report(data: int, slot: int = 0, flags: int = 0) -> int:
    """``psi5_ecu.loom``'s OUTQ word: D9..D0 in bits 15:6 (0 for a
    Manchester error or no frame), the slot in bit 0, the flags."""
    if flags & (E_MANCH | E_NONE):
        data = 0
    return ((data & 0x3FF) << 6) | (slot & 1) | flags
