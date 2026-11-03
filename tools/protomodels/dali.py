"""DALI reference models for firmware/dali_controller.loom and dali_gear.loom.

Written from IEC 62386-101 (the system: physical layer, frames, timing) and
62386-102 (control gear commands), never from the firmware. The standard
itself was not at hand, so every figure below names the public source it was
taken from; those marked UNCHECKED could not be traced to any public copy of
the standard's tables and come from the author's recollection of edition 2.0
(``QUESTIONS.md`` of the W-002 scratch directory lists them):

* Bit timing, 62386-101 ed.2.0. Receiver tables 18 (from a bit's start
  edge, "first section") and 19 (from its middle edge, "second section"),
  as reproduced in Renesas application note R01AN4653EJ0101 rev 1.01
  (2019), tables 4-8 and 4-10: a half-bit 333.3 to 500 us, a double
  half-bit 666.7 to 1000 us, a bit timing violation 750 (first section) or
  1200 (second) to 1400 us, the stop condition 2400 us or more, grey areas
  between (a receiver may take either reading). Transmitter bit timing, the
  same note's table 4-18 (the ed.2.0 table number is not given there): a
  half-bit 366.7 / 416.7 / 466.7 us (min / typical / max), a double half-bit
  733.3 / 833.3 / 933.3 us, the stop condition at least 2450 us.
* Settling times, ed.2.0 as summarised by Microchip TB3200 (2018, which
  cites IEC 62386-101 ed.2.0 2014-11): control gear starts the backward
  frame 5.5 to 10.5 ms after the forward frame; the control device waits at
  least 2.4 ms after a backward frame before its next forward frame.
  UNCHECKED: the control device takes a backward frame that starts up to
  12.4 ms after the forward frame (its receive window); the settling time
  between forward frames at priority 1 is 13.5 to 14.7 ms. All settling
  times are counted here from the end of the previous frame's last bit
  (QUESTIONS.md: the reading taken).
* Frames, 62386-101: bi-phase coding, a 1 is low then high and a 0 high
  then low, most significant bit first; the bus idles high. A forward frame
  is a start bit (1), the address byte and the data byte; a backward frame
  a start bit and one byte; each ends in the stop condition.
* Commands, 62386-102 ed.2.0 (opcodes as in its command table): address
  byte 0AAAAAAS short address A, 100GGGGS group G, 1111111S broadcast,
  1111110S broadcast unaddressed; S = 0 direct arc power control (255 is
  MASK, no change), S = 1 a command. OFF 0x00, RECALL MAX LEVEL 0x05,
  RECALL MIN LEVEL 0x06, QUERY STATUS 0x90, QUERY CONTROL GEAR PRESENT
  0x91, QUERY ACTUAL LEVEL 0xA0, QUERY GROUPS 0-7 0xC0 and 8-15 0xC1. YES
  is 0xFF. Status bit 2 is "lamp on", bit 6 "short address is MASK".

Time: every model works in clocks, and ``us`` is clocks per microsecond.
At the standard rate on a 50 MHz chip ``us = 50``; the RTL tests run the
whole bus faster, with the half-bit a few hundred clocks, and every figure
scales with it (:func:`scale_for`).

:class:`DaliBus` is the bench model: the wired AND of the chip's TX pins and
of every model node, seen by the chip's RX pins. :class:`DaliDecoder` turns
the bus's edges back into frames, and :func:`check_tx` holds a frame's
half-bits to the transmitter's limits. :class:`DaliControllerNode` sends
forward frames with a settable half-bit and injected errors;
:class:`DaliGearNode` is a control gear with a settable answer time.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, List, Optional, Sequence, Tuple

from tools.protomodels.bench import Drive, Lines, Model, PinSpec, pad_of

CLOCK = 50_000_000                  # the chip's clock in the tests
TE_US = 1e6 / 2400                  # the nominal half-bit: 1200 bit/s

# 62386-101 ed.2.0 transmitter bit timing (Renesas R01AN4653 table 4-18).
# The tables print to 0.1 us; the figures are thirds of a millisecond
# (Te -12 %, +12 %, and so on), kept exact here so that a half-bit of a whole
# number of clocks at the edge is not lost to the rounding of the print.
TX_HALF = (1100 / 3, 1400 / 3)
TX_DOUBLE = (2200 / 3, 2800 / 3)
TX_STOP = 2450.0
# receiver bit timing, tables 18 and 19 (Renesas tables 4-8 and 4-10)
RX_HALF = (1000 / 3, 500.0)
RX_DOUBLE = (2000 / 3, 1000.0)
RX_VIOLATION = (1200.0, 1400.0)     # second section; first section from 750
RX_STOP = 2400.0
# settling times (Microchip TB3200; the last two UNCHECKED)
GEAR_ANSWER = (5500.0, 10500.0)     # forward frame to backward frame, sent
AFTER_BACKWARD = 2400.0             # backward frame to forward frame, least
BACKWARD_WINDOW = 12400.0           # forward to backward, received (UNCHECKED)
FORWARD_SETTLE = (13500.0, 14700.0) # forward to forward, priority 1 (UNCHECKED)

# 62386-102
YES = 0xFF
MASK = 0xFF
MAX_LEVEL, MIN_LEVEL, POWER_ON_LEVEL = 254, 1, 254
OFF, RECALL_MAX, RECALL_MIN = 0x00, 0x05, 0x06
QUERY_STATUS, QUERY_PRESENT, QUERY_LEVEL = 0x90, 0x91, 0xA0
QUERY_GROUPS_LO, QUERY_GROUPS_HI = 0xC0, 0xC1


def scale_for(te_clocks: float) -> float:
    """Clocks per microsecond for a bus whose half-bit is ``te_clocks``."""
    return te_clocks / TE_US


def short(a: int, command: bool = False) -> int:
    """The address byte for short address ``a``."""
    return (a << 1) | int(command)


def group(g: int, command: bool = False) -> int:
    return 0x80 | (g << 1) | int(command)


def broadcast(command: bool = False, unaddressed: bool = False) -> int:
    return (0xFC if unaddressed else 0xFE) | int(command)


def frame_bits(value: int, nbits: int) -> List[int]:
    """The start bit and ``nbits`` bits of ``value``, most significant first."""
    return [1] + [(value >> (nbits - 1 - i)) & 1 for i in range(nbits)]


def half_levels(bits: Sequence[int]) -> List[int]:
    """Two bus levels per bit: a 1 is low then high, a 0 high then low."""
    out: List[int] = []
    for b in bits:
        out += [0, 1] if b else [1, 0]
    return out


def classify(interval_us: float, from_middle: bool) -> str:
    """The receiver's reading of the time between two edges (tables 18 and
    19): ``half``, ``double`` (only from a bit's middle edge), ``violation``
    or ``grey`` (either reading is allowed)."""
    lo, hi = RX_HALF
    if lo <= interval_us <= hi:
        return "half"
    if from_middle:
        dlo, dhi = RX_DOUBLE
        if dlo <= interval_us <= dhi:
            return "double"
        if RX_VIOLATION[0] <= interval_us <= RX_VIOLATION[1]:
            return "violation"
    elif 750.0 <= interval_us <= RX_VIOLATION[1]:
        return "violation"
    return "grey"


# ----------------------------------------------------------------- frames
@dataclass
class Frame:
    """One frame as seen on the bus.

    ``edges`` are ``(cycle, level after)``, the first the start bit's fall;
    ``intervals`` are ``(clocks, halves)`` between consecutive edges, halves
    being 1 or 2 as the decoder read them (0: a violation); ``end`` is the
    end of the last bit (the last edge, or a nominal half-bit after it when
    the frame ends on a 1)."""

    start: int
    edges: List[Tuple[int, int]] = field(default_factory=list)
    bits: List[int] = field(default_factory=list)
    intervals: List[Tuple[int, int]] = field(default_factory=list)
    violation: bool = False
    end: int = 0

    @property
    def nbits(self) -> int:
        """Bits after the start bit."""
        return max(len(self.bits) - 1, 0)

    @property
    def value(self) -> int:
        v = 0
        for b in self.bits[1:]:
            v = (v << 1) | b
        return v

    @property
    def is_forward(self) -> bool:
        return not self.violation and self.nbits == 16

    @property
    def is_backward(self) -> bool:
        return not self.violation and self.nbits == 8


class DaliDecoder:
    """Turns the bus level, clock by clock, into :class:`Frame`s.

    An interval under 1.5 nominal half-bits is one half-bit, under 2.5 two,
    else a violation (the thresholds sit in the receiver's grey areas, so a
    sender inside tables 18 and 19 always reads right). A frame closes when
    the bus has been high for the stop condition, ``RX_STOP``; a low that
    long is a violation and closes the frame at the next rise."""

    def __init__(self, us: float) -> None:
        self.us = us
        self.te = TE_US * us
        self.frames: List[Frame] = []
        self.on_frame = None            # called with each closed frame
        self._level = 1
        self._cur: Optional[Frame] = None
        self._mid = False               # the last edge was a bit's middle

    def clock(self, level: int, cycle: int) -> None:
        cur = self._cur
        if level != self._level:
            self._level = level
            if cur is None:
                if level == 0:
                    self._cur = Frame(start=cycle, edges=[(cycle, 0)])
                    self._mid = False
                return
            self._edge(cur, level, cycle)
        elif cur is not None:
            idle = cycle - cur.edges[-1][0]
            if idle >= RX_STOP * self.us and (level == 1 or cur.violation):
                if level == 0:
                    cur.violation = True
                self._close(cur)

    def _edge(self, cur: Frame, level: int, cycle: int) -> None:
        d = cycle - cur.edges[-1][0]
        cur.edges.append((cycle, level))
        if cur.violation:
            cur.intervals.append((d, 0))
            return
        n = 1 if d < 1.5 * self.te else 2 if d < 2.5 * self.te else 0
        if n == 0 or (n == 2 and not self._mid):
            cur.violation = True
            cur.intervals.append((d, 0))
            return
        cur.intervals.append((d, n))
        if n == 1:
            self._mid = not self._mid
        if self._mid:
            cur.bits.append(level)

    def _close(self, cur: Frame) -> None:
        last = cur.edges[-1][0]
        cur.end = int(round(last + self.te)) if self._mid else last
        self.frames.append(cur)
        self._cur = None
        if self.on_frame is not None:
            self.on_frame(cur)


def check_tx(frame: Frame, us: float) -> List[str]:
    """Every half-bit and double half-bit of ``frame`` against the
    transmitter's limits (table 4-18 of R01AN4653); returns the faults."""
    faults = []
    for i, (d, n) in enumerate(frame.intervals):
        t = d / us
        lim = TX_HALF if n == 1 else TX_DOUBLE if n == 2 else None
        if lim is None:
            faults.append("interval %d: a violation (%.1f us)" % (i, t))
        elif not lim[0] <= t <= lim[1]:
            faults.append("interval %d: %.1f us outside %s" % (i, t, lim))
    return faults


def check_rx(frame: Frame, us: float) -> List[str]:
    """Every interval of ``frame`` against the receiver's tables 18 and 19:
    the faults are the intervals a receiver must reject (``violation``) or
    may read either way (``grey``). A frame sent inside the transmitter's
    limits has none; one injected with a bad half-bit has one there."""
    faults = []
    middle = False                      # the first edge is a bit boundary
    for i, (d, n) in enumerate(frame.intervals):
        kind = classify(d / us, middle)
        if kind not in ("half", "double"):
            faults.append("interval %d: %.1f us is %s" % (i, d / us, kind))
        if n == 1:
            middle = not middle
    return faults


def settling_us(before: Frame, after: Frame, us: float) -> float:
    """The settling time between two frames: from the end of the last bit
    of ``before`` to the first edge of ``after``."""
    return (after.start - before.end) / us


# ----------------------------------------------------------- control gear
class GearState:
    """A control gear's variables and its answer to a forward frame, for the
    command subset named in the module docstring (62386-102)."""

    def __init__(self, address: Optional[int], groups: int = 0,
                 level: int = POWER_ON_LEVEL) -> None:
        self.address = address          # None: no short address (MASK)
        self.groups = groups & 0xFFFF
        self.level = level

    def addressed(self, a: int) -> bool:
        y = a >> 1
        if y < 64:
            return self.address is not None and y == self.address
        if y < 80:
            return bool((self.groups >> (y - 64)) & 1)
        if y == 127:
            return True
        return y == 126 and self.address is None

    def known(self, a: int, d: int) -> bool:
        """Whether this gear implements the frame (direct arc power always)."""
        return not (a & 1) or d in (OFF, RECALL_MAX, RECALL_MIN, QUERY_STATUS,
                                    QUERY_PRESENT, QUERY_LEVEL,
                                    QUERY_GROUPS_LO, QUERY_GROUPS_HI)

    def apply(self, a: int, d: int) -> Optional[int]:
        """Act on an addressed frame; returns the answer byte or ``None``."""
        if not a & 1:
            if d != MASK:
                self.level = 0 if d == 0 else min(max(d, MIN_LEVEL), MAX_LEVEL)
            return None
        if d == OFF:
            self.level = 0
        elif d == RECALL_MAX:
            self.level = MAX_LEVEL
        elif d == RECALL_MIN:
            self.level = MIN_LEVEL
        elif d == QUERY_STATUS:
            return (4 if self.level else 0) | (64 if self.address is None else 0)
        elif d == QUERY_PRESENT:
            return YES
        elif d == QUERY_LEVEL:
            return self.level
        elif d == QUERY_GROUPS_LO:
            return self.groups & 0xFF
        elif d == QUERY_GROUPS_HI:
            return self.groups >> 8
        return None


# ------------------------------------------------------------- model nodes
class _Sender:
    """A node's own drive: a schedule of level changes, clocked by the bus."""

    def __init__(self, us: float) -> None:
        self.us = us
        self.level = 1
        self._plan: List[Tuple[int, int]] = []      # (cycle, level), sorted

    def schedule(self, start: int, levels: Sequence[int],
                 halves_us: Sequence[float]) -> int:
        """Drive ``levels``, one per half-bit of ``halves_us``, from cycle
        ``start``; then the bus is released high. Returns the end cycle."""
        t = float(start)
        for lvl, h in zip(levels, halves_us):
            self._plan.append((int(round(t)), lvl))
            t += h * self.us
        end = int(round(t))
        self._plan.append((end, 1))
        self._plan.sort(key=lambda e: e[0])
        return end

    def clock(self, cycle: int) -> None:
        """Set the level the node drives from cycle ``cycle + 1``."""
        while self._plan and self._plan[0][0] <= cycle + 1:
            self.level = self._plan.pop(0)[1]


@dataclass
class Transfer:
    """A forward frame a model controller sent, and what came back."""

    address: int
    data: int
    start: int
    end: int
    reply: Optional[Frame] = None


class DaliControllerNode:
    """A control device on the bus: sends forward frames with a settable
    half-bit and errors, and attaches the backward frame that starts within
    ``BACKWARD_WINDOW`` of a forward frame's end to its :class:`Transfer`."""

    def __init__(self, us: float, half_us: float = TE_US) -> None:
        self.us = us
        self.half_us = half_us
        self.tx = _Sender(us)
        self.decoder = DaliDecoder(us)
        self.decoder.on_frame = self._frame
        self.transfers: List[Transfer] = []
        self.cycle = 0

    @property
    def level(self) -> int:
        return self.tx.level

    def send(self, address: int, data: int, at: Optional[int] = None,
             bad_half: Optional[Tuple[int, float]] = None,
             stop_gap_us: Optional[float] = None) -> Transfer:
        """Schedule a forward frame at cycle ``at`` (default: now).
        ``bad_half = (i, us)`` makes half-bit ``i`` that long; ``stop_gap_us``
        sends the frame again that long after its end, with no stop
        condition between."""
        start = self.cycle + 2 if at is None else at
        levels = half_levels(frame_bits((address << 8) | data, 16))
        halves = [self.half_us] * len(levels)
        if bad_half is not None:
            halves[bad_half[0]] = bad_half[1]
        end = self.tx.schedule(start, levels, halves)
        if stop_gap_us is not None:
            end = self.tx.schedule(end + int(stop_gap_us * self.us), levels, halves)
        tr = Transfer(address, data, start, end)
        self.transfers.append(tr)
        return tr

    def busy(self) -> bool:
        return bool(self.tx._plan) or self.decoder._cur is not None

    def _frame(self, frame: Frame) -> None:
        for tr in reversed(self.transfers):
            if tr.end <= frame.start:
                if (frame.start - tr.end <= BACKWARD_WINDOW * self.us
                        and tr.reply is None and frame.start > tr.end):
                    tr.reply = frame
                break

    def clock(self, level: int, cycle: int) -> None:
        self.cycle = cycle
        self.decoder.clock(level, cycle)
        self.tx.clock(cycle)


class DaliGearNode:
    """A control gear on the bus (:class:`GearState`): acts on every forward
    frame addressed to it once its stop condition has passed, and starts the
    answer to a query ``answer_us`` after the end of the frame's last bit,
    with half-bits of ``half_us``. ``acted`` lists ``(frame, answer)``."""

    def __init__(self, us: float, address: Optional[int], groups: int = 0,
                 answer_us: float = 8000.0, half_us: float = TE_US) -> None:
        self.us = us
        self.state = GearState(address, groups)
        self.answer_us = answer_us
        self.half_us = half_us
        self.tx = _Sender(us)
        self.decoder = DaliDecoder(us)
        self.decoder.on_frame = self._frame
        self.acted: List[Tuple[Frame, Optional[int]]] = []

    @property
    def level(self) -> int:
        return self.tx.level

    def _frame(self, frame: Frame) -> None:
        if not frame.is_forward:
            return
        a, d = frame.value >> 8, frame.value & 0xFF
        if not self.state.addressed(a) or not self.state.known(a, d):
            return
        answer = self.state.apply(a, d)
        self.acted.append((frame, answer))
        if answer is not None:
            levels = half_levels(frame_bits(answer, 8))
            start = frame.end + int(round(self.answer_us * self.us))
            self.tx.schedule(start, levels, [self.half_us] * len(levels))

    def clock(self, level: int, cycle: int) -> None:
        self.decoder.clock(level, cycle)
        self.tx.clock(cycle)


class DaliSniffer:
    """A listener: drives nothing, decodes everything."""

    level = 1

    def __init__(self, us: float) -> None:
        self.decoder = DaliDecoder(us)

    def clock(self, level: int, cycle: int) -> None:
        self.decoder.clock(level, cycle)


class DaliBus(Model):
    """The bus on the bench: a wired AND with the power supply's pull-up.

    The level of cycle ``c`` is the AND of the chip's TX pins as they were in
    cycle ``c - 1`` (the interface's loop delay, a clock here; a TX pin at
    ``tx_pulls_low`` pulls the bus low, the interface's transistor) and of every
    node's drive; all the chip's RX pins see it, every node is clocked with
    it, and :attr:`decoder` decodes it. :attr:`edges` records every change
    as ``(cycle, level)``."""

    def __init__(self, us: float, nodes: Iterable = (),
                 tx: Sequence[PinSpec] = ("OUT0",),
                 rx: Sequence[PinSpec] = ("IN0",), tx_pulls_low: int = 1) -> None:
        self.tx_pulls_low = tx_pulls_low
        self.nodes = list(nodes)
        self.tx_pads = [pad_of(p) for p in tx]
        self.rx_pads = [pad_of(p) for p in rx]
        self.decoder = DaliDecoder(us)
        self.level = 1
        self.edges: List[Tuple[int, int]] = []
        self._chip = 1

    def add(self, node):
        self.nodes.append(node)
        return node

    def drive(self, drive: Drive, cycle: int) -> None:
        level = self._chip
        for node in self.nodes:
            level &= node.level
        if level != self.level:
            self.edges.append((cycle, level))
        self.level = level
        for pad in self.rx_pads:
            drive.set(pad, level)

    def observe(self, lines: Lines) -> None:
        chip = 1
        for pad in self.tx_pads:
            chip &= lines.get(pad) ^ self.tx_pulls_low
        self._chip = chip
        self.decoder.clock(self.level, lines.cycle)
        for node in self.nodes:
            node.clock(self.level, lines.cycle)
