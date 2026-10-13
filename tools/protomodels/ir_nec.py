"""NEC infrared reference models, for firmware/ir_nec_tx.loom and ir_nec_rx.loom.

Written from the NEC protocol description and a TSOP receiver datasheet, not
from the firmware:

* NEC: a 38 kHz carrier, keyed on and off in units of 562.5 us. A frame is a
  9 ms burst (16 units), 4.5 ms of space (8), 32 data bits and a final
  562.5 us burst. The bits are the address, its inverse, the command and its
  inverse, each byte least significant bit first; each bit is a one-unit
  burst followed by one unit of space (0) or three (1): pulse-distance
  coding. Extended NEC sends a 16-bit address (low byte first) in place of
  the address and its inverse. A repeat code is a 9 ms burst, 2.25 ms of
  space (4 units) and a one-unit burst. Frames and repeat codes start
  108 ms apart while a key is held.
* The receiver module (Vishay TSOP382.., the TSOP38238 among them) outputs
  low while it sees a burst: the output falls td after the burst starts,
  7/f0 < td < 13/f0, and its low pulse is the burst length tpi changed by
  -5/f0 .. +5/f0 (tpi - 5/f0 < tpo < tpi + 5/f0); it needs bursts of 10
  cycles or more. Figures from the TSOP382.., TSOP384.. datasheet
  (Vishay document 82491, rev. 2.1, 27 May 2025, figure 1 and its notes),
  checked on 2026-10-12. The older TSOP18238 sheet allows 15/f0 and
  +6/f0; the tests run one case there too, beyond the TSOP382 limits.

Pulse lists are ``[(level, microseconds), ...]`` with ``MARK`` (carrier on;
the receiver module's output low) and ``SPACE``, starting with a mark.

* :func:`frame_pulses`, :func:`repeat_pulses`: the encoder;
  :func:`decode_pulses`: a decoder with a stated tolerance.
* :func:`vary`, :func:`tsop_distort`, :func:`glitch`, :func:`truncate`:
  timing variations, receiver-module distortion and faults for a generator.
* :class:`EdgeRecorder` and :class:`CarrierAnalysis`: the transmitter pin's
  edges and the carrier analyser (frequency, duty, burst envelope).
* :class:`TsopModel`: a bench model between a transmitter pin and a receiver
  pin, low while the carrier is present, with the datasheet's delays.
* :class:`IrSource`: a bench model that drives a receiver pin with a
  demodulated pulse list (what a TSOP would output), for the receiver tests.
"""

from __future__ import annotations

import dataclasses
import random
from typing import List, Optional, Sequence, Tuple

from .bench import Drive, Lines, Model, PinSpec, pad_of

CARRIER_HZ = 38_000.0
UNIT_US = 562.5
LEADER_MARK, LEADER_SPACE, REPEAT_SPACE = 16, 8, 4      # in units
BIT_MARK, ZERO_SPACE, ONE_SPACE = 1, 1, 3
PERIOD_US = 108_000.0               # start to start while a key is held
MARK, SPACE = 1, 0

Pulses = List[Tuple[int, float]]


def frame_bits(address: int, command: int, extended: bool = False) -> List[int]:
    """The 32 data bits in wire order."""
    if extended:
        first = address & 0xFFFF
    else:
        first = (address & 0xFF) | ((~address & 0xFF) << 8)
    second = (command & 0xFF) | ((~command & 0xFF) << 8)
    word = first | (second << 16)
    return [(word >> i) & 1 for i in range(32)]


def frame_pulses(address: int, command: int, extended: bool = False) -> Pulses:
    """One NEC frame: leader, 32 bits, final burst."""
    return bits_pulses(frame_bits(address, command, extended))


def bits_pulses(bits: Sequence[int]) -> Pulses:
    """A frame carrying ``bits`` as given (any count and values, a wrong
    inverse included)."""
    u = UNIT_US
    out: Pulses = [(MARK, LEADER_MARK * u), (SPACE, LEADER_SPACE * u)]
    for bit in bits:
        out += [(MARK, BIT_MARK * u), (SPACE, (ONE_SPACE if bit else ZERO_SPACE) * u)]
    out.append((MARK, BIT_MARK * u))
    return out


def repeat_pulses() -> Pulses:
    u = UNIT_US
    return [(MARK, LEADER_MARK * u), (SPACE, REPEAT_SPACE * u), (MARK, BIT_MARK * u)]


@dataclasses.dataclass
class NecMessage:
    """What a decoder made of one leader and what followed it."""

    kind: str                       # "frame", "repeat" or "error"
    address: int = 0                # 16 bits as received
    command: int = 0
    extended: bool = False          # the address bytes are not inverses
    bits: int = 0                   # data bits received whole
    reason: str = ""                # for an error

    @property
    def standard(self) -> bool:
        return self.kind == "frame" and not self.extended


def _within(us: float, units: float, tol: float, extra_us: float) -> bool:
    nominal = units * UNIT_US
    return nominal * (1 - tol) - extra_us <= us <= nominal * (1 + tol) + extra_us


def decode_pulses(pulses: Sequence[Tuple[int, float]], *, tol: float = 0.25,
                  extra_us: float = 0.0) -> List[NecMessage]:
    """Decode a pulse list; every mark and space must be within ``tol`` of
    nominal plus ``extra_us``. A mark that is not a leader is skipped."""
    out: List[NecMessage] = []
    p = list(pulses)
    i = 0
    ok = lambda k, units: k < len(p) and _within(p[k][1], units, tol, extra_us)
    while i < len(p):
        if p[i][0] != MARK or not ok(i, LEADER_MARK):
            i += 1
            continue
        if ok(i + 1, REPEAT_SPACE):
            good = ok(i + 2, BIT_MARK)
            out.append(NecMessage("repeat") if good else
                       NecMessage("error", reason="repeat burst"))
            i += 3
            continue
        if not ok(i + 1, LEADER_SPACE):
            out.append(NecMessage("error", reason="leader space"))
            i += 2
            continue
        out.append(_bits(p, i + 2, ok))
        i += 2 + 2 * out[-1].bits + 1
    return out


def _bits(p, k, ok) -> NecMessage:
    word = 0
    for n in range(32):
        if not ok(k + 2 * n, BIT_MARK):
            return NecMessage("error", bits=n, reason="mark of bit %d" % n)
        if ok(k + 2 * n + 1, ZERO_SPACE):
            bit = 0
        elif ok(k + 2 * n + 1, ONE_SPACE):
            bit = 1
        else:
            return NecMessage("error", bits=n, reason="space of bit %d" % n)
        word |= bit << n
    if not ok(k + 64, BIT_MARK):
        return NecMessage("error", bits=32, reason="final burst")
    address, command, inverse = word & 0xFFFF, (word >> 16) & 0xFF, word >> 24
    if inverse != (~command & 0xFF):
        return NecMessage("error", address=address, command=command, bits=32,
                          reason="command inverse")
    extended = (address >> 8) != (~address & 0xFF)
    return NecMessage("frame", address=address, command=command,
                      extended=extended, bits=32)


# --------------------------------------------------------------- variations
def vary(pulses: Sequence[Tuple[int, float]], *, mark: float = 1.0,
         space: float = 1.0, jitter_us: float = 0.0,
         rng: Optional[random.Random] = None) -> Pulses:
    """Every mark times ``mark`` and every space times ``space``, then each
    moved by up to ``jitter_us`` either way (uniform)."""
    rng = rng or random.Random(0)
    out: Pulses = []
    for level, us in pulses:
        us *= mark if level == MARK else space
        if jitter_us:
            us += rng.uniform(-jitter_us, jitter_us)
        out.append((level, us))
    return out


def tsop_distort(pulses: Sequence[Tuple[int, float]], delta_us: float) -> Pulses:
    """A receiver module's output pulse width error: each mark longer by
    ``delta_us`` (tpo = tpi + delta) and the space after it shorter by as
    much, so mark starts keep their places. The TSOP382.. datasheet allows
    -5/f0 .. +5/f0."""
    out: Pulses = []
    carry = 0.0
    for level, us in pulses:
        if level == MARK:
            out.append((level, us + delta_us))
            carry = delta_us
        else:
            out.append((level, us - carry))
            carry = 0.0
    return out


def glitch(pulses: Sequence[Tuple[int, float]], at_us: float,
           width_us: float) -> Pulses:
    """A pulse of the opposite level, ``width_us`` long, ``at_us`` after the
    start of the list (it must fall inside one pulse)."""
    out: Pulses = []
    t = 0.0
    for level, us in pulses:
        if t < at_us and at_us + width_us < t + us:
            out += [(level, at_us - t), (1 - level, width_us),
                    (level, t + us - at_us - width_us)]
        else:
            out.append((level, us))
        t += us
    return out


def truncate(pulses: Sequence[Tuple[int, float]], count: int) -> Pulses:
    """The first ``count`` pulses: a frame that stops part-way."""
    return list(pulses[:count])


def start_of(pulses: Sequence[Tuple[int, float]], index: int) -> float:
    """Microseconds from the start of the list to the start of pulse ``index``."""
    return sum(us for _, us in pulses[:index])


def bit_index(bit: int, part: int = MARK) -> int:
    """Index in a frame's pulse list of data bit ``bit``'s mark (or space)."""
    return 2 + 2 * bit + (0 if part == MARK else 1)


# ---------------------------------------------------------- carrier analyser
class EdgeRecorder(Model):
    """Every change of one pad: ``edges`` holds ``(cycle, level)``, the cycle
    being the first with the new level. The level before the first edge is 0."""

    def __init__(self, pin: PinSpec) -> None:
        self.pad = pad_of(pin)
        self.level = 0
        self.edges: List[Tuple[int, int]] = []

    def observe(self, lines: Lines) -> None:
        level = lines.get(self.pad)
        if level != self.level:
            self.edges.append((lines.cycle, level))
            self.level = level


@dataclasses.dataclass
class Burst:
    """One burst of carrier: ``rises`` and ``falls`` in cycles."""

    rises: List[int]
    falls: List[int]

    @property
    def start(self) -> int:
        return self.rises[0]

    @property
    def end(self) -> int:
        return self.falls[-1]

    @property
    def cycles(self) -> int:
        return len(self.rises)

    @property
    def periods(self) -> List[int]:
        return [b - a for a, b in zip(self.rises, self.rises[1:])]

    @property
    def highs(self) -> List[int]:
        return [f - r for r, f in zip(self.rises, self.falls)]


class CarrierAnalysis:
    """The carrier and its envelope, from a transmitter pin's edges.

    A burst is a run of carrier cycles in which no low lasts longer than
    ``gap_us`` (100 us, almost four periods at 38 kHz; the shortest NEC space
    is 562.5 us). It lasts from its first rise to its last fall; the space
    after it runs to the next burst's first rise."""

    def __init__(self, edges: Sequence[Tuple[int, int]], clock_hz: float,
                 gap_us: float = 100.0) -> None:
        self.clock_hz = float(clock_hz)
        gap = gap_us * self.clock_hz / 1e6
        self.bursts: List[Burst] = []
        for cycle, level in edges:
            if level == 1:
                last = self.bursts[-1] if self.bursts else None
                if last is None or not last.falls or cycle - last.falls[-1] > gap:
                    self.bursts.append(Burst([cycle], []))
                else:
                    last.rises.append(cycle)
            elif self.bursts:
                self.bursts[-1].falls.append(cycle)

    def us(self, clocks: float) -> float:
        return clocks * 1e6 / self.clock_hz

    def frequency(self, burst: Burst) -> float:
        """Carrier frequency over the whole burst, first rise to last rise."""
        return (burst.cycles - 1) * self.clock_hz / (burst.rises[-1] - burst.rises[0])

    def duty(self, burst: Burst) -> float:
        return sum(burst.highs) / float(burst.rises[-1] + burst.periods[-1]
                                        - burst.rises[0])

    def pulses(self) -> Pulses:
        """The envelope as a pulse list: each burst, then the space after it
        (the last burst has none)."""
        out: Pulses = []
        for a, b in zip(self.bursts, self.bursts[1:] + [None]):
            out.append((MARK, self.us(a.end - a.start)))
            if b is not None:
                out.append((SPACE, self.us(b.start - a.end)))
        return out

    def starts_us(self) -> List[float]:
        return [self.us(b.start) for b in self.bursts]


# ------------------------------------------------------- the receiver module
class TsopModel(Model):
    """A demodulating receiver module (TSOP382.. style) between a transmitter
    pin ``tx`` and a receiver pin ``rx``: its output is low while it sees a
    burst of carrier, high otherwise (idle high).

    The output falls ``td`` after a burst's first rise, if the burst is still
    going then, and rises ``td + delta`` after the burst's last fall, so the
    output pulse is the burst length plus ``delta``. ``td`` and ``delta`` are
    in carrier periods (datasheet: 7 < td < 15, -5 < delta < 6); each output
    edge also moves by up to ``jitter_us`` (uniform). A burst shorter than
    ``td`` gives no output (the datasheet asks for 10 cycles or more). The
    burst ends when no rise comes for 1.5 carrier periods. ``events`` holds
    ``(cycle, level)`` of every output change.
    """

    def __init__(self, tx: PinSpec, rx: PinSpec, clock_hz: float, *,
                 td: float = 10.0, delta: float = 0.0, jitter_us: float = 0.0,
                 seed: int = 0, f0: float = CARRIER_HZ) -> None:
        self.tx_pad = pad_of(tx)
        self.rx_pad = pad_of(rx)
        period = clock_hz / f0
        self.td = td * period
        self.delta = delta * period
        self.gap = 1.5 * period
        self.jitter = jitter_us * clock_hz / 1e6
        self.rng = random.Random(seed)
        self.level = 1
        self.events: List[Tuple[int, int]] = []
        self._pending: List[Tuple[int, int]] = []
        self._prev = 0
        self._in = False                # inside a burst
        self._low = False               # the output went low for this burst
        self._fall_at: Optional[int] = None
        self._last_rise = self._last_fall = 0

    def _at(self, cycle: float) -> int:
        return int(round(cycle + self.rng.uniform(-self.jitter, self.jitter)))

    def drive(self, drive: Drive, cycle: int) -> None:
        while self._pending and self._pending[0][0] <= cycle:
            _, level = self._pending.pop(0)
            if level != self.level:
                self.level = level
                self.events.append((cycle, level))
        drive.set(self.rx_pad, self.level)

    def observe(self, lines: Lines) -> None:
        cycle = lines.cycle
        tx = lines.get(self.tx_pad)
        if tx and not self._prev:                                   # a rise
            if not self._in:
                self._in, self._low = True, False
                self._fall_at = self._at(cycle + self.td)
            self._last_rise = cycle
        elif self._prev and not tx:
            self._last_fall = cycle
        self._prev = tx
        if not self._in:
            return
        if not self._low and cycle >= self._fall_at:
            self._low = True
            self._pending.append((cycle + 1, 0))
        if not tx and cycle - self._last_rise > self.gap:             # the end
            self._in = False
            if self._low:
                rise = self._at(self._last_fall + self.td + self.delta)
                self._pending.append((max(rise, cycle + 1), 1))
                self._pending.sort()


# ------------------------------------------------------ a demodulated source
class IrSource(Model):
    """Drives a receiver pin with demodulated pulse lists, as a receiver
    module's output: low for a mark, high for a space and when idle.

    :meth:`send` queues a list and returns the cycle it starts at: ``at`` if
    given, else ``gap_us`` after the end of the previous list (the line is
    idle from cycle 0). Times are rounded to whole clocks from the start of
    the list, so they do not drift."""

    def __init__(self, pin: PinSpec, clock_hz: float, *,
                 gap_us: float = 40_000.0) -> None:
        self.pad = pad_of(pin)
        self.clock_hz = float(clock_hz)
        self.gap_us = gap_us
        self.level = 1
        self.free_at = 0
        self.starts: List[int] = []
        self._changes: List[Tuple[int, int]] = []

    def cycles(self, us: float) -> int:
        return int(round(us * self.clock_hz / 1e6))

    def send(self, pulses: Sequence[Tuple[int, float]], *, at: Optional[int] = None,
             gap_us: Optional[float] = None) -> int:
        gap = self.gap_us if gap_us is None else gap_us
        start = self.free_at + (self.cycles(gap) if self.starts else 0)
        if at is not None:
            start = max(start, int(at))
        t = 0.0
        for level, us in pulses:
            self._changes.append((start + self.cycles(t), 0 if level == MARK else 1))
            t += us
        end = start + self.cycles(t)
        self._changes.append((end, 1))
        self._changes.sort()
        self.free_at = end
        self.starts.append(start)
        return start

    @property
    def busy(self) -> bool:
        return bool(self._changes)

    def drive(self, drive: Drive, cycle: int) -> None:
        changes = self._changes
        while changes and changes[0][0] <= cycle:
            self.level = changes.pop(0)[1]
        drive.set(self.pad, self.level)
