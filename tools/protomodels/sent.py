"""SENT (SAE J2716) reference models, for firmware/sent_tx.loom and sent_rx.loom.

Written from the SENT description, not from the firmware. The figures, and
where each comes from:

* A frame is a calibration/synchronisation pulse of 56 clock ticks, a
  status-and-communication nibble, 1 to 6 data nibbles, a CRC nibble and an
  optional pause pulse. Each nibble is the time between two falling edges,
  12 + value ticks (12..27). Every pulse starts with a low time of at least
  4 ticks (commonly 5). The tick is 3 us nominal and may be 3 to 90 us. The
  transmitter's clock may be off by +-20 %, and a receiver rejects a frame
  whose sync pulse differs from the previous one by more than 1/64. The pause
  pulse is 12 to 768 ticks.
* CRC-4, x^4 + x^3 + x^2 + 1, seed 0101, over the data nibbles only (the
  status nibble is not covered), in the table form ``c = T[c] ^ d`` with
  ``T[i] = i * x^4 mod P``; the "recommended" variant of J2716 rev. JAN2010
  and later runs one zero nibble more after the data; the "legacy" variant
  of the earlier revisions does not.

All of this is SAE J2716 (rev. JAN2010, section 5 on the frame and pulse
timing, section 5.4 on the CRC, as these are usually cited). **Unchecked:**
the standard itself was not available to this work, so every figure above
was taken from the task brief (D-035 batch, W-001) and the common public
descriptions of SENT (vendor SENT application notes and the "SENT
(protocol)" encyclopaedia entry), not checked against J2716's text. The
section numbers are the usual citations, unchecked too. The questions this
leaves are in the W-001 QUESTIONS.md.

Contents:

* :func:`crc4`, :func:`crc_table`: the CRC in the table form, both variants.
* :class:`Frame`, :func:`frame_ticks`: a frame and its pulses in ticks, with
  error injection (a wrong CRC, a nibble of any length, a sync off by a
  factor).
* :class:`SentSource`: a bench model that drives a receiver pin with frames,
  each at its own tick (so a stream can drift frame by frame), with a
  settable low time.
* :class:`EdgeRecorder`: a pin's edges; :func:`decode`: a receiver that
  learns the tick from each sync and checks every timing of J2716 named
  above (sync length against the nominal tick +-20 %, the low time, the
  nibble range, the 1/64 rule, the CRC).
* :class:`Wire`: a bench model that copies one chip output to one chip input,
  the board wire between a transmitter and a receiver pin.
"""

from __future__ import annotations

import dataclasses
from typing import Dict, List, Optional, Sequence, Tuple

from .bench import Drive, Lines, Model, PinSpec, pad_of

SYNC_TICKS = 56
NIBBLE_BASE = 12                    # a nibble is 12 + value ticks
NIBBLE_MAX = 15
LOW_MIN = 4                         # every pulse's low time, ticks, at least
LOW_TYPICAL = 5
PAUSE_MIN, PAUSE_MAX = 12, 768
TICK_MIN_US, TICK_NOMINAL_US, TICK_MAX_US = 3.0, 3.0, 90.0
CLOCK_TOL = 0.20                    # the transmitter's clock, +-
SYNC_RATIO = 1.0 / 64               # successive syncs differ by at most this
POLY = 0b11101                      # x^4 + x^3 + x^2 + 1
SEED = 0b0101


def _mul_x4(c: int) -> int:
    """``c * x^4 mod P`` for a 4-bit ``c``, bit by bit."""
    for _ in range(4):
        c <<= 1
        if c & 0x10:
            c ^= POLY
    return c & 0xF


def crc_table() -> List[int]:
    """``T[i] = i * x^4 mod P``, the 16-entry table of the J2716 CRC."""
    return [_mul_x4(i) for i in range(16)]


def crc4(data: Sequence[int], legacy: bool = False) -> int:
    """The SENT CRC of the data nibbles (the status nibble is not covered):
    seed 0101, ``c = T[c] ^ d`` per nibble, then, unless ``legacy``, one zero
    nibble more (the recommended variant)."""
    table = crc_table()
    c = SEED
    for d in data:
        c = table[c] ^ (d & 0xF)
    if not legacy:
        c = table[c]
    return c


# ------------------------------------------------------------------ frames
@dataclasses.dataclass
class Frame:
    """One SENT frame. ``crc`` overrides the computed CRC nibble (a bad CRC);
    ``ticks`` overrides the length in ticks of nibble ``i`` (0 the status,
    1..N the data, N + 1 the CRC: a nibble of 28 ticks, say); ``sync_scale``
    stretches the sync pulse (1 + 1/32: a sync 1/32 off); ``pause`` is the
    pause pulse in ticks, or None."""

    status: int
    data: Sequence[int]
    legacy: bool = False
    crc: Optional[int] = None
    ticks: Dict[int, float] = dataclasses.field(default_factory=dict)
    sync_scale: float = 1.0
    pause: Optional[int] = None

    @property
    def crc_nibble(self) -> int:
        return crc4(self.data, self.legacy) if self.crc is None else self.crc & 0xF

    def nibbles(self) -> List[int]:
        return [self.status & 0xF] + [d & 0xF for d in self.data] + [self.crc_nibble]


def frame_ticks(frame: Frame) -> List[float]:
    """The frame's pulses, falling edge to falling edge, in ticks."""
    out = [SYNC_TICKS * frame.sync_scale]
    for i, v in enumerate(frame.nibbles()):
        out.append(frame.ticks.get(i, NIBBLE_BASE + v))
    if frame.pause is not None:
        out.append(float(frame.pause))
    return out


# ------------------------------------------------------------------ source
class SentSource(Model):
    """Drives a receiver pin with SENT frames: low ``low`` ticks at the start
    of every pulse, high for the rest, idle high. :meth:`send` queues frames
    back to back, each ``(frame, tick_us)`` at its own tick, and closes the
    last pulse with a falling edge and ``low`` ticks of low. Edge times are
    rounded to whole clocks from the start of the burst, so they do not
    drift; returns the cycle of the first falling edge."""

    def __init__(self, pin: PinSpec, clock_hz: float, *, low: float = LOW_TYPICAL) -> None:
        self.pad = pad_of(pin)
        self.clock_hz = float(clock_hz)
        self.low = low
        self.level = 1
        self.free_at = 0
        self.falls: List[int] = []
        self._changes: List[Tuple[int, int]] = []

    def cycles(self, us: float) -> int:
        return int(round(us * self.clock_hz / 1e6))

    def send(self, frames: Sequence[Tuple[Frame, float]], *, at: Optional[int] = None,
             gap_us: float = 100.0) -> int:
        start = self.free_at + self.cycles(gap_us)
        if at is not None:
            start = max(start, int(at))
        t, tick = 0.0, frames[0][1]
        for frame, tick in frames:
            for ticks in frame_ticks(frame):
                self._fall(start + self.cycles(t), start + self.cycles(t + self.low * tick))
                t += ticks * tick
        end = start + self.cycles(t + self.low * tick)
        self._fall(start + self.cycles(t), end)
        self._changes.sort()
        self.free_at = end
        return start

    def _fall(self, fall: int, rise: int) -> None:
        self.falls.append(fall)
        self._changes += [(fall, 0), (rise, 1)]

    @property
    def busy(self) -> bool:
        return bool(self._changes)

    def drive(self, drive: Drive, cycle: int) -> None:
        changes = self._changes
        while changes and changes[0][0] <= cycle:
            self.level = changes.pop(0)[1]
        drive.set(self.pad, self.level)


class Wire(Model):
    """The board wire from a chip output (``src``) to a chip input (``dst``):
    ``dst`` follows ``src`` one clock later. High before the first cycle."""

    def __init__(self, src: PinSpec, dst: PinSpec) -> None:
        self.src, self.dst = pad_of(src), pad_of(dst)
        self.level = 1

    def drive(self, drive: Drive, cycle: int) -> None:
        drive.set(self.dst, self.level)

    def observe(self, lines: Lines) -> None:
        self.level = lines.get(self.src)


class EdgeRecorder(Model):
    """Every change of one pad: ``edges`` holds ``(cycle, level)``, the cycle
    being the first with the new level. The level before the first edge is
    ``idle`` (SENT idles high)."""

    def __init__(self, pin: PinSpec, idle: int = 1) -> None:
        self.pad = pad_of(pin)
        self.level = idle
        self.edges: List[Tuple[int, int]] = []

    def observe(self, lines: Lines) -> None:
        level = lines.get(self.pad)
        if level != self.level:
            self.edges.append((lines.cycle, level))
            self.level = level


# ----------------------------------------------------------------- decoder
@dataclasses.dataclass
class Decoded:
    """A frame as a J2716 receiver sees it. ``errors`` names every check that
    failed ("crc", "range", "sync ratio", "low time", "pause"); ``first``: no
    previous sync to apply the 1/64 rule to. Times in clocks."""

    nibbles: List[int]
    sync_clocks: int
    pulses: List[int]
    lows: List[Optional[int]]
    errors: List[str]
    first: bool

    @property
    def status(self) -> int:
        return self.nibbles[0]

    @property
    def data(self) -> List[int]:
        return self.nibbles[1:-1]

    @property
    def crc(self) -> int:
        return self.nibbles[-1]

    @property
    def tick(self) -> float:
        """The transmitter's tick learnt from the sync, in clocks."""
        return self.sync_clocks / float(SYNC_TICKS)


def decode(edges: Sequence[Tuple[int, int]], n: int, nominal_tick: float, *,
           pause: bool = False, legacy: bool = False) -> List[Decoded]:
    """Decode a pin's edges (``(cycle, level)``) into frames of ``n`` data
    nibbles. A sync is a pulse of 56 nominal ticks (``nominal_tick`` clocks)
    +-20 %; the frame's tick is its sync / 56. Each nibble is rounded to whole
    ticks and must be 12..27; every low time must be 4 ticks or more; the
    CRC must match the variant; the sync must be within 1/64 of the
    previous frame's. A pulse that should be a sync and is not loses the
    stream: the decoder hunts for the next sync. The last frame is decoded
    only if it is closed by a falling edge."""
    falls = [c for c, lv in edges if lv == 0]
    rises = [c for c, lv in edges if lv == 1]

    def low_of(fall: int) -> Optional[int]:
        later = [r for r in rises if r > fall]
        return later[0] - fall if later else None

    lo, hi = SYNC_TICKS * nominal_tick * (1 - CLOCK_TOL), SYNC_TICKS * nominal_tick * (1 + CLOCK_TOL)
    out: List[Decoded] = []
    prev: Optional[int] = None
    i, count = 0, n + 2 + (1 if pause else 0)
    while i + 1 < len(falls):
        sync = falls[i + 1] - falls[i]
        if not lo <= sync <= hi or i + 1 + count >= len(falls):
            prev, i = None, i + 1                     # not a sync, or cut short
            continue
        tick = sync / float(SYNC_TICKS)
        pulses = [falls[i + 1 + k + 1] - falls[i + 1 + k] for k in range(count)]
        lows = [low_of(falls[i + k]) for k in range(count + 1)]
        errors: List[str] = []
        nibbles = []
        for p in pulses[:n + 2]:
            v = int(round(p / tick)) - NIBBLE_BASE
            if not 0 <= v <= NIBBLE_MAX:
                errors.append("range")
                v = min(max(v, 0), NIBBLE_MAX)
            nibbles.append(v)
        if any(lw is None or lw < LOW_MIN * tick for lw in lows):
            errors.append("low time")
        if pause and not PAUSE_MIN <= round(pulses[-1] / tick) <= PAUSE_MAX:
            errors.append("pause")
        if crc4(nibbles[1:-1], legacy) != nibbles[-1]:
            errors.append("crc")
        if prev is not None and abs(sync - prev) > prev * SYNC_RATIO:
            errors.append("sync ratio")
        out.append(Decoded(nibbles, sync, pulses, lows, sorted(set(errors)), prev is None))
        prev, i = sync, i + 1 + count
    return out
