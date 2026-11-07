"""HDMI-CEC reference models for firmware/cec_initiator.loom and cec_follower.loom.

Written from the CEC line protocol as its specification and public
secondary sources describe it, never from the firmware. The specification
is HDMI Specification 1.3a, Supplement 1, "Consumer Electronics Control"
(CEC); no copy was opened while this was written, so the section and figure
numbers below are the author's recollection and every figure is UNCHECKED
against it. Each one is also in the Linux kernel's software CEC
implementation, drivers/media/cec/core/cec-pin.c, whose ``CEC_TIM_*``
constants name the same values. Checked against that file on 2026-11-06
(torvalds/linux master): START_BIT_LOW 3700 (3500..3900), START_BIT_TOTAL
4500 (4300..4700), DATA_BIT_0_LOW 1500 (1300..1700), DATA_BIT_1_LOW 600
(400..800), DATA_BIT_TOTAL 2400 (2050..2750) and DATA_BIT_SAMPLE 850 (the
earliest safe sample) all agree with the windows below. The section
numbers, the signal free times and the retransmission limit are not in
that file and remain UNCHECKED:

* Start bit (CEC 5.2.1, "Start Bit Timing" figure): low 3.7 ms +-0.2, then
  high, the whole bit 4.5 ms +-0.2 (:data:`START_LOW_WIN`,
  :data:`START_PERIOD_WIN`).
* Data bit (CEC 5.2.2, "Data Bit Timing" figure): a bit starts with a fall;
  a logical 0 is low 1.5 ms +-0.2, a logical 1 low 0.6 ms +-0.2; the bit
  period is 2.4 ms nominal, 2.05 to 2.75 ms accepted; a follower samples the
  line 1.05 ms after the fall (the safe sample window 0.85 to 1.25 ms).
* Frame (CEC 6, "Frame Description"): a start bit, a header block and up to
  15 data blocks (16 blocks in all). Every block is 8 information bits,
  most significant first, then EOM (1 in the last block) and ACK. The
  header's bits 7:4 are the initiator's logical address and 3:0 the
  destination's; 15 is broadcast. The first data block is the opcode.
* Acknowledge (CEC 6.1.2 / 5.2.3, "ACK" figure): the initiator sends the ACK
  bit as a 1; a follower that accepts a directly addressed block holds the
  line low for a 0's time (1.5 ms from the fall). For a broadcast the sense
  is inverted: a follower that rejects the block makes it a 0 (NACK), so a
  1 is the ACK.
* Signal free time (CEC 9.1, "Signal Free Time" table): the line must have
  been idle, before a transmission, for 3 bit periods to retransmit after a
  failed attempt, 5 for a new initiator, 7 for the present initiator to send
  its next frame. Counted from the end of the last bit period.
* Arbitration (CEC 9.2): every initiator monitors the line while it sends;
  one that sends a 1 and sees a 0 has lost and stops driving at once.
  Arbitration is decided in the start bit and the initiator address bits;
  this model, like the firmware, checks every bit it sends as a 1.
* Retransmission (CEC 7.1, "Frame Re-transmissions"): a frame that is not
  acknowledged may be retransmitted, up to 5 times (:data:`MAX_RETRIES`).

Time here is in clocks, from microseconds through ``us`` (clocks per
microsecond, :func:`scale_for`). The firmware counts ticks of 50 us; the
tests run a tick of a few dozen clocks, so ``us`` is below 1 there and every
time on the bus scales with it.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, List, Optional, Sequence, Tuple

from tools.protomodels.bench import Drive, Lines, Model, PinSpec, pad_of

START_LOW_US = 3700.0
START_LOW_WIN = (3500.0, 3900.0)
START_PERIOD_US = 4500.0
START_PERIOD_WIN = (4300.0, 4700.0)
BIT_US = 2400.0
BIT_PERIOD_WIN = (2050.0, 2750.0)
ZERO_LOW_US = 1500.0
ZERO_LOW_WIN = (1300.0, 1700.0)
ONE_LOW_US = 600.0
ONE_LOW_WIN = (400.0, 800.0)
SAMPLE_US = 1050.0
SAMPLE_WIN = (850.0, 1250.0)
SFT_RETRY, SFT_NEW, SFT_NEXT = 3, 5, 7      # signal free time, bit periods
MAX_RETRIES = 5
MAX_BLOCKS = 16
BROADCAST = 15
TICK_US = 50.0                              # the firmware's tick

# the decoder's own thresholds (it parses; the checks judge)
START_GUESS_US = 2500.0     # a low this long is taken as a start bit
FRAME_IDLE_US = 4000.0      # no fall this long after a bit ends the frame


def scale_for(tick_clocks: float) -> float:
    """Clocks per microsecond when the firmware's 50 us tick is ``tick_clocks``."""
    return tick_clocks / TICK_US


def header(initiator: int, destination: int) -> int:
    return ((initiator & 15) << 4) | (destination & 15)


def blocks_of(initiator: int, destination: int, data: Sequence[int]) -> List[int]:
    """The frame's block bytes: the header, then the data (opcode first)."""
    if len(data) > MAX_BLOCKS - 1:
        raise ValueError("a CEC frame has at most 15 data blocks")
    return [header(initiator, destination)] + [b & 0xFF for b in data]


def frame_bits(blocks: Sequence[int], acks: Optional[Sequence[int]] = None) -> List[int]:
    """The bits after the start bit: 8 per block MSB first, EOM, ACK (as the
    initiator sends it, 1, unless ``acks`` gives the line's value)."""
    bits: List[int] = []
    for k, byte in enumerate(blocks):
        bits += [(byte >> (7 - i)) & 1 for i in range(8)]
        bits.append(1 if k == len(blocks) - 1 else 0)
        bits.append(1 if acks is None else acks[k])
    return bits


def acked(destination: int, ack_line: int) -> bool:
    """Whether a block was acknowledged, from the line's ACK bit value."""
    return ack_line == 1 if destination == BROADCAST else ack_line == 0


# ------------------------------------------------------------------ decoder
@dataclass
class Bit:
    """One data bit on the line: its fall, low time and period (to the next
    fall), in clocks; ``low`` is None until the rise, ``period`` until the
    next fall (the frame's last bit never gets one)."""

    fall: int
    low: Optional[int] = None
    period: Optional[int] = None

    def value(self, us: float) -> int:
        """The bit a follower reads: the line at the sample point."""
        return 1 if self.low is not None and self.low < SAMPLE_US * us else 0


@dataclass
class Frame:
    """A frame as the line carried it."""

    start: int                              # the start bit's fall
    us: float
    start_low: Optional[int] = None
    start_period: Optional[int] = None
    bits: List[Bit] = field(default_factory=list)

    def values(self) -> List[int]:
        return [b.value(self.us) for b in self.bits if b.low is not None]

    @property
    def blocks(self) -> List[Tuple[int, int, int]]:
        """Every complete block as ``(byte, eom, ack line)``."""
        v = self.values()
        out = []
        for k in range(len(v) // 10):
            b = v[10 * k:10 * k + 10]
            out.append((int("".join(map(str, b[:8])), 2), b[8], b[9]))
        return out

    @property
    def data(self) -> List[int]:
        return [b[0] for b in self.blocks]

    @property
    def destination(self) -> Optional[int]:
        v = self.values()
        return int("".join(map(str, v[4:8])), 2) if len(v) >= 8 else None

    @property
    def initiator(self) -> Optional[int]:
        v = self.values()
        return int("".join(map(str, v[:4])), 2) if len(v) >= 4 else None

    @property
    def end(self) -> float:
        """The nominal end of the last bit period (where the signal free
        time is counted from)."""
        last = self.bits[-1].fall if self.bits else self.start
        return last + BIT_US * self.us

    def complete(self) -> bool:
        """Whole blocks, the last one with EOM and no other."""
        b = self.blocks
        return (bool(b) and len(self.bits) == 10 * len(b) and b[-1][1] == 1
                and all(x[1] == 0 for x in b[:-1]))


class CecDecoder:
    """Clocked with the line's level every cycle; collects :attr:`frames`.

    A low of :data:`START_GUESS_US` or more opens a frame (closing any frame
    that was open); every later fall is a data bit; a frame closes when no
    fall comes for :data:`FRAME_IDLE_US` after its last bit's fall. Lows
    outside a frame that are too short for a start bit go to
    :attr:`glitches`."""

    def __init__(self, us: float) -> None:
        self.us = us
        self.level = 1
        self.frames: List[Frame] = []
        self.open: Optional[Frame] = None
        self.glitches: List[Tuple[int, int]] = []
        self.fell = False                   # this cycle's level fell
        self._fall = 0
        self._last_fall = 0

    def clock(self, level: int, cycle: int) -> None:
        self.fell = bool(self.level and not level)
        rose = not self.level and level
        self.level = level
        f = self.open
        if self.fell:
            self._fall = cycle
            if f is not None:
                if f.bits:
                    f.bits[-1].period = cycle - f.bits[-1].fall
                else:
                    f.start_period = cycle - f.start
                f.bits.append(Bit(cycle))
            self._last_fall = cycle
        elif rose:
            low = cycle - self._fall
            if low >= START_GUESS_US * self.us:
                if f is not None and f.bits and f.bits[-1].fall == self._fall:
                    f.bits.pop()            # that fall was this start bit
                    if f.bits:
                        f.bits[-1].period = None
                self._close()
                self.open = Frame(self._fall, self.us, start_low=low)
            elif f is not None and f.bits:
                f.bits[-1].low = low
            elif f is None:
                self.glitches.append((self._fall, low))
        elif (f is not None and level and f.bits
              and cycle - self._last_fall > FRAME_IDLE_US * self.us):
            self._close()

    def _close(self) -> None:
        if self.open is not None:
            self.frames.append(self.open)
            self.open = None

    def all_frames(self) -> List[Frame]:
        return self.frames + ([self.open] if self.open is not None else [])


# ------------------------------------------------------------------ checks
def _win(name: str, value: float, win: Tuple[float, float], us: float) -> List[str]:
    lo, hi = win
    t = value / us
    return [] if lo <= t <= hi else ["%s %.0f us outside %.0f..%.0f" % (name, t, lo, hi)]


def check_frame(frame: Frame, us: float) -> List[str]:
    """Every timing of ``frame`` against the specification's windows: the
    start bit's low and period, each bit's low (a 0 or a 1, an ACK stretched
    by a follower is a 0) and each period but the last. Empty if all pass."""
    out = []
    if frame.start_low is None or frame.start_period is None:
        return ["no complete start bit"]
    out += _win("start low", frame.start_low, START_LOW_WIN, us)
    out += _win("start period", frame.start_period, START_PERIOD_WIN, us)
    for i, b in enumerate(frame.bits):
        if b.low is None:
            out.append("bit %d never rose" % i)
            continue
        win = ONE_LOW_WIN if b.value(us) else ZERO_LOW_WIN
        out += _win("bit %d low" % i, b.low, win, us)
        if b.period is not None:
            out += _win("bit %d period" % i, b.period, BIT_PERIOD_WIN, us)
    return out


def free_time(before: Frame, after: Frame, us: float) -> float:
    """The idle line between two frames, in bit periods, from the end of the
    first one's last bit period to the second one's start."""
    return (after.start - before.end) / (BIT_US * us)


# ------------------------------------------------------------------ devices
SHORT_PERIOD_US = 1600.0    # error injection: a bit period below 2.05 ms
LONG_LOW_US = 2000.0        # error injection: a low above a 0's 1.7 ms


@dataclass
class Send:
    """A frame a :class:`CecNode` is to send, and what became of it.

    ``with_chip`` starts its first attempt at the bus's next fall, in the
    same clock (a second initiator starting with the chip's start bit);
    ``short_at`` and ``long_at`` give that bit (counted after the start bit)
    a short period or a long low; ``no_eom`` sends EOM 0 in every block, so
    the frame ends without one; ``retries`` retransmits a NACKed frame after
    :data:`SFT_RETRY` periods."""

    destination: int
    data: List[int]
    initiator: Optional[int] = None
    sft: int = SFT_NEW
    with_chip: bool = False
    short_at: Optional[int] = None
    long_at: Optional[int] = None
    no_eom: bool = False
    retries: int = 0
    start: Optional[int] = None
    starts: List[int] = field(default_factory=list)
    result: Optional[str] = None            # "ack", "nack" or "lost"
    lost_at: Optional[int] = None           # the bit index it lost in
    acks: List[int] = field(default_factory=list)   # ACK lines, last attempt


class CecNode:
    """A CEC device on the bus: an initiator for its :meth:`send` frames
    (signal free time, arbitration on every 1 it sends, the ACK check of
    every block, retransmission) and, with an ``address``, a follower that
    holds the ACK bit low for 1.5 ms in each block of a frame sent to it
    (``ack = False``: it never does, a NACK) and, with ``nack_broadcast``,
    in each block of a broadcast (a broadcast rejected)."""

    def __init__(self, us: float, address: Optional[int] = None, ack: bool = True,
                 nack_broadcast: bool = False) -> None:
        self.us = us
        self.address = address
        self.ack, self.nack_broadcast = ack, nack_broadcast
        self.decoder = CecDecoder(us)
        self.queue: List[Send] = []
        self.sent: List[Send] = []
        self.level = 1
        self.ack_falls: List[int] = []      # ACK bits it held low
        self.high_since = 0
        self._lows: List[Tuple[float, float]] = []
        self._samples: List[Tuple[float, int]] = []
        self._cur: Optional[Send] = None
        self._end = 0.0

    def send(self, destination: int, data: Sequence[int] = (), **kw) -> Send:
        s = Send(destination, list(data), **kw)
        self.queue.append(s)
        return s

    def _plan(self, s: Send, t0: float) -> None:
        us = self.us
        init = self.address if s.initiator is None else s.initiator
        bits = frame_bits(blocks_of(init or 0, s.destination, s.data))
        if s.no_eom:
            bits = [0 if i % 10 == 8 else b for i, b in enumerate(bits)]
        s.start, s.result, s.lost_at, s.acks = int(t0), None, None, []
        s.starts.append(int(t0))
        self._cur = s
        self._lows = [(t0, t0 + START_LOW_US * us)]
        self._samples = []
        t = t0 + START_PERIOD_US * us
        for i, b in enumerate(bits):
            low = LONG_LOW_US if i == s.long_at else (ONE_LOW_US if b else ZERO_LOW_US)
            self._lows.append((t, t + low * us))
            if b:
                self._samples.append((t + SAMPLE_US * us, i))
            t += (SHORT_PERIOD_US if i == s.short_at else BIT_US) * us
        self._end = t

    def _stop(self, cycle: int) -> None:
        self._lows = [p for p in self._lows if p[0] <= cycle]
        self._samples = []
        self._end = cycle

    def clock(self, level: int, cycle: int) -> None:
        if level and not self.decoder.level:
            self.high_since = cycle
        self.decoder.clock(level, cycle)
        us = self.us
        s = self._cur
        while self._samples and self._samples[0][0] <= cycle:
            _, i = self._samples.pop(0)
            if i % 10 == 9:                         # the ACK bit
                s.acks.append(level)
                if not acked(s.destination, level):
                    s.result = "nack"
                    self._stop(cycle)
            elif level == 0:                        # a 1 read back 0: lost
                s.result, s.lost_at = "lost", i
                self._stop(cycle)
        if s is not None and (s.result is not None or cycle >= self._end):
            self._cur = None
            s.result = s.result or "ack"
            if s.result == "nack" and s.retries > 0:
                s.retries -= 1
                s.sft = SFT_RETRY
            else:
                self.queue.remove(s)
                self.sent.append(s)
        self._follow(cycle)
        if self._cur is None and self.queue:
            q = self.queue[0]
            if q.with_chip and not q.starts:
                if self.decoder.fell:
                    self._plan(q, cycle)
            else:
                ref = self.high_since
                if self.decoder.frames:
                    ref = max(ref, self.decoder.frames[-1].end)
                if (level and self.decoder.open is None
                        and cycle - ref >= q.sft * BIT_US * us):
                    self._plan(q, cycle + 1)
        nxt = cycle + 1
        self._lows = [p for p in self._lows if p[1] > nxt]
        self.level = 0 if any(a <= nxt < b for a, b in self._lows) else 1

    def _follow(self, cycle: int) -> None:
        f = self.decoder.open
        if (not self.decoder.fell or self.address is None or self._cur is not None
                or f is None or len(f.bits) % 10 != 0):
            return
        dest = f.destination                    # this fall is an ACK bit's
        if (dest == self.address and self.ack) or (dest == BROADCAST and self.nack_broadcast):
            self._lows.append((cycle, cycle + ZERO_LOW_US * self.us))
            self.ack_falls.append(cycle)


class CecBus(Model):
    """The CEC wire on the bench: a wired AND of open-drain drivers.

    Each chip pin in ``pins`` is a BIDIR pin in open-drain mode with its own
    pad (two Loom threads never share a pin); the board joins them. The
    line's level in cycle ``c`` is low if any chip pin pulled low in cycle
    ``c - 1`` (``uio_oe`` set, ``uio_out`` 0: the board wire's one clock)
    or any node is low; the bus pulls every chip pad low while it is low,
    and the bench's pull-ups (the caller sets ``pullups`` for these pads)
    make the high level. :attr:`decoder` decodes the line, every node is
    clocked with it, and :attr:`pin_lows` holds each chip pin's own lows as
    ``(fall, length)`` in clocks, measured on the pin."""

    def __init__(self, us: float, nodes: Iterable[CecNode] = (),
                 pins: Sequence[PinSpec] = ("BIDIR0",)) -> None:
        self.us = us
        self.nodes = list(nodes)
        self.pads = [pad_of(p) for p in pins]
        self.decoder = CecDecoder(us)
        self.level = 1
        self.pin_lows = {p: [] for p in self.pads}
        self._chip = 0
        self._fall = {p: None for p in self.pads}

    @property
    def pullups(self) -> int:
        """The ``uio`` bits a bench needs pulled up for this bus."""
        return sum(1 << bit for _, bit in self.pads)

    def add(self, node: CecNode) -> CecNode:
        self.nodes.append(node)
        return node

    def drive(self, drive: Drive, cycle: int) -> None:
        level = 0 if self._chip else 1
        for node in self.nodes:
            level &= node.level
        self.level = level
        if not level:
            for pad in self.pads:
                drive.pull_low(pad)

    def observe(self, lines: Lines) -> None:
        chip = 0
        for pad in self.pads:
            bit = 1 << pad[1]
            low = bool(lines.uio_oe & bit) and not lines.uio_out & bit
            chip |= low
            if low and self._fall[pad] is None:
                self._fall[pad] = lines.cycle
            elif not low and self._fall[pad] is not None:
                self.pin_lows[pad].append((self._fall[pad], lines.cycle - self._fall[pad]))
                self._fall[pad] = None
        self._chip = chip
        self.decoder.clock(self.level, lines.cycle)
        for node in self.nodes:
            node.clock(self.level, lines.cycle)

    def frames(self) -> List[Frame]:
        return self.decoder.all_frames()

    def lows_of(self, pin: PinSpec) -> List[Tuple[int, int]]:
        return self.pin_lows[pad_of(pin)]
