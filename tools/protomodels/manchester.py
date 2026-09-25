"""Manchester reference models for firmware/manchester_loopback.loom.

Written from the frame format in the program header and from IEEE 802.3's
bit convention, not from the firmware or the engine:

* line code: a 0 is high then low, a 1 low then high, so every bit has a
  transition in its middle; most significant bit first; the idle line is low;
* a frame is the sync word ``SYNC`` = 0xAAAB (fourteen bits 1010..10 of
  preamble, then 11, the start marker), 0 to 4 data words, and the end: the
  line low from the end of the last bit, so the bit after it has no mid-bit
  transition (a code violation);
* from idle low the first edge of a frame is the rising middle of its first
  bit, and every preamble edge is a mid-bit edge.

:class:`ManchesterLine` is the bench model (``tools.protomodels.bench.Model``)
between the chip's TX pin and its RX pin. As a **wire** it gives RX the
chip's TX one clock later (the loopback); with a :class:`ManchesterNode` on it
RX sees the node instead. Either way it records every edge of the chip's TX,
and :func:`decode` turns those edges back into frames on a nominal half-bit
grid, with the distance of each edge from that grid, so a test can hold the
transmitter to its bit time independently of the chip's receiver.

:class:`ManchesterNode` sends frames of its own with its own clock: a bit rate
off the nominal one by a settable fraction, edges rounded to whole clocks, and
on request one bit whose second half repeats its first (a missing mid-bit
transition, the violation the receiver must report).
"""

from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

from tools.protomodels.bench import Drive, Lines, Model, PinSpec, pad_of

CLOCK = 50_000_000          # the chip's clock in the tests
SYNC = 0xAAAB               # preamble 1010..10 and the start marker 11
MAX_WORDS = 4               # data words a frame may carry
GAP_BITS = 16               # idle bits the chip's transmitter leaves after a frame


def word_bits(word: int) -> List[int]:
    """The 16 bits of ``word``, most significant first."""
    return [(word >> (15 - i)) & 1 for i in range(16)]


def frame_bits(words: Sequence[int]) -> List[int]:
    """Every bit of a frame, the sync word first."""
    bits = word_bits(SYNC)
    for w in words:
        bits += word_bits(w & 0xFFFF)
    return bits


def half_bits(bits: Sequence[int], violate: Optional[int] = None) -> List[int]:
    """The line levels, two per bit (0: high then low, 1: low then high).

    ``violate`` is the index of a bit whose second half repeats its first:
    no transition in the middle of that bit."""
    levels: List[int] = []
    for i, b in enumerate(bits):
        first = 1 - b
        levels += [first, first if i == violate else b]
    return levels


# ------------------------------------------------------------------ decode
@dataclass
class Decoded:
    """One frame as seen on a line.

    ``start`` is the cycle of its first edge (the rising middle of the first
    bit); ``bits`` are the bits read at the middle of each half-bit of the
    nominal grid from there, up to the first bit without a mid-bit transition
    (``end`` is its index; for a well-formed frame it is the bit after the
    last word). ``edges`` holds ``(half-bit index, offset in clocks)`` for
    every edge from ``start`` to ``end``, the index counted from ``start``."""

    start: int
    bits: List[int] = field(default_factory=list)
    end: Optional[int] = None
    edges: List[Tuple[int, int]] = field(default_factory=list)

    @property
    def sync_ok(self) -> bool:
        return len(self.bits) >= 16 and self.bits[:16] == word_bits(SYNC)

    @property
    def words(self) -> List[int]:
        """The data words after the sync word (whole words only)."""
        out = []
        for i in range(16, len(self.bits) - 15, 16):
            v = 0
            for b in self.bits[i:i + 16]:
                v = (v << 1) | b
            out.append(v)
        return out

    @property
    def well_formed(self) -> bool:
        """The sync word, whole words, and the end right after the last."""
        return (self.sync_ok and self.end is not None and self.end % 16 == 0
                and self.end == len(self.bits))


class EdgeLog:
    """The edges of one line: ``(cycle, level)`` for each change, where
    ``cycle`` is the first cycle with the new level. The level before the
    first edge is ``initial``."""

    def __init__(self, initial: int = 0) -> None:
        self.initial = initial
        self.cycles: List[int] = []
        self.levels: List[int] = []

    def add(self, cycle: int, level: int) -> None:
        """Record ``level`` at ``cycle``; cycles must not go backwards."""
        if level != (self.levels[-1] if self.levels else self.initial):
            self.cycles.append(cycle)
            self.levels.append(level)

    def level_at(self, cycle: int) -> int:
        i = bisect_right(self.cycles, cycle)
        return self.levels[i - 1] if i else self.initial

    def edges_in(self, lo: int, hi: int) -> List[Tuple[int, int]]:
        """``(cycle, level)`` for the edges with ``lo <= cycle < hi``."""
        i = bisect_right(self.cycles, lo - 1)
        j = bisect_right(self.cycles, hi - 1)
        return list(zip(self.cycles[i:j], self.levels[i:j]))


def decode(log: EdgeLog, half: float, since: int = 0,
           idle_bits: int = 2) -> List[Decoded]:
    """Every frame on the line from cycle ``since``, on a grid of ``half``
    clocks per half-bit. A frame starts with a rising edge after at least
    ``idle_bits`` bit times without an edge (or at ``since`` on a low line);
    the half-bit ``j`` of it spans ``[start + j * half, start + (j + 1) *
    half)`` and is read in its middle; half-bit 0 is the second half of bit
    0, whose first half is the idle level."""
    frames: List[Decoded] = []
    quiet = idle_bits * 2 * half
    last_edge = since - quiet
    i = bisect_right(log.cycles, since - 1)
    while i < len(log.cycles):
        c, level = log.cycles[i], log.levels[i]
        if level != 1 or c - last_edge < quiet:
            last_edge = c
            i += 1
            continue
        frame = _read(log, c, half)
        frames.append(frame)
        stop = c + (2 * frame.end + 1) * half if frame.end is not None else c
        while i < len(log.cycles) and log.cycles[i] < stop:
            i += 1
        last_edge = log.cycles[i - 1]
    return frames


def _read(log: EdgeLog, start: int, half: float, max_bits: int = 200) -> Decoded:
    frame = Decoded(start)
    sample = lambda j: log.level_at(int(start + (j + 0.5) * half))
    for b in range(max_bits):
        first = 0 if b == 0 else sample(2 * b - 1)
        second = sample(2 * b)
        if first == second:
            frame.end = b
            break
        frame.bits.append(second)
    last = start + (2 * (frame.end or max_bits)) * half
    for c, _ in log.edges_in(start, int(last)):
        j = int(round((c - start) / half))
        frame.edges.append((j, int(round(c - start - j * half))))
    return frame


def data_bit(word: int, bit: int) -> int:
    """Index in a frame of bit ``bit`` (0 = most significant, sent first) of
    data word ``word`` (0 = the first after the sync word)."""
    return 16 * (word + 1) + bit


# --------------------------------------------------------------------- node
@dataclass
class Sent:
    """A frame the node was asked to send, and when it went out."""

    words: List[int]
    violate: Optional[int] = None
    start: Optional[int] = None         # first cycle of the frame's first bit
    done: bool = False                  # its last half-bit has gone out


class ManchesterNode:
    """A Manchester transmitter with its own clock, for the chip's receiver.

    Args:
        rate: the nominal bit rate.
        offset: the node's bit rate is ``rate * (1 + offset)``, so a positive
            offset sends short bits. Edges fall on whole clocks: half-bit
            ``i`` of a frame starts ``ceil(i * half)`` clocks after the frame.
        gap_bits: idle bit times before each frame (and after the last).
    """

    def __init__(self, rate: float, *, offset: float = 0.0, clock: int = CLOCK,
                 gap_bits: int = GAP_BITS) -> None:
        self.half = clock / (2.0 * rate * (1.0 + offset))
        self.gap = int(gap_bits * 2 * self.half)
        self.sent: List[Sent] = []
        self._queue: List[Sent] = []
        self._cur: Optional[Sent] = None
        self._levels: List[int] = []
        self._t0 = 0
        self._free_at = self.gap            # the line is idle from reset

    def send(self, words: Sequence[int], *, violate: Optional[int] = None) -> Sent:
        """Queue a frame of ``words``; ``violate`` is the index of a bit (see
        :func:`data_bit`) sent without its mid-bit transition. More than
        ``MAX_WORDS`` words is not a valid frame, and is allowed, so that a
        receiver's answer to one can be tested."""
        s = Sent([w & 0xFFFF for w in words], violate)
        self._queue.append(s)
        self.sent.append(s)
        return s

    @property
    def busy(self) -> bool:
        return self._cur is not None or bool(self._queue)

    def level(self, cycle: int) -> int:
        """The level the node drives in ``cycle`` (cycles must not go back)."""
        if self._cur is None:
            if not self._queue or cycle < self._free_at:
                return 0
            self._cur = self._queue.pop(0)
            self._cur.start = self._t0 = cycle
            self._levels = half_bits(frame_bits(self._cur.words), self._cur.violate)
        i = int((cycle - self._t0) // self.half)
        if i < len(self._levels):
            return self._levels[i]
        self._cur.done = True
        self._cur = None
        self._free_at = cycle + self.gap
        return 0


# ---------------------------------------------------------------------- line
class ManchesterLine(Model):
    """What the chip's RX pin sees, and a record of its TX pin.

    Without a node the line is a wire: RX gets the level TX had in the cycle
    before (``drive`` may only use registered state, ``bench.py``; that clock
    is the loop delay). With ``node`` RX sees the node instead, and TX goes
    nowhere. ``tx_log`` holds every edge of TX, ``rx_log`` every edge on RX,
    for :func:`decode`.
    """

    def __init__(self, node: Optional[ManchesterNode] = None,
                 tx: PinSpec = "OUT0", rx: PinSpec = "IN0") -> None:
        self.node = node
        self.tx_pad = pad_of(tx)
        self.rx_pad = pad_of(rx)
        self.tx_log = EdgeLog(0)
        self.rx_log = EdgeLog(0)
        self._tx = 0

    def drive(self, drive: Drive, cycle: int) -> None:
        level = self._tx if self.node is None else self.node.level(cycle)
        drive.set(self.rx_pad, level)

    def observe(self, lines: Lines) -> None:
        self._tx = lines.get(self.tx_pad)
        self.tx_log.add(lines.cycle, self._tx)
        self.rx_log.add(lines.cycle, lines.get(self.rx_pad))
