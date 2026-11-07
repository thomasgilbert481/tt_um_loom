"""SAE J1708 reference models: the bus, a node, a decoder and timing checks.

Written from the public descriptions of SAE J1708 (and of SAE J1587, the
message layer on top of it), not from the firmware. The SAE documents are
not free and no copy was at hand, so every figure here is **secondary**:

* Kvaser, "Introduction to SAE J1708" (kvaser.com/about-can/can-standards/
  j1708/, sections "Quick Facts", "Communication On Bit-Level", "The
  anatomy of a J1708 message", "Calculation of Checksum"): 9600 bit/s; a
  byte is a start bit, eight data bits LSB first and a stop bit; a message
  is a MID, data bytes and a one-byte checksum, at most 21 bytes; "the
  time between two bytes in a message is not allowed to be more than two
  bit times"; the bus access time is "the shortest idle time (at least 10
  bit times) plus two times the priority of the message", priority 1
  (highest) to 8; a logical 0 is dominant; after a collision both senders
  release the bus and wait a bus access time before they send again; the
  eight-bit sum of the MID, the data and the checksum is 0.
* Copperhill Technologies, "A Brief Introduction to SAE J1708 and J1587"
  (section "J1708 Half-Duplex Collision Detection"): a sender monitors the
  bus while it sends (the page names the MID) and drops out at once when
  another node's dominant bit overwrites one of its recessive bits.
* Wikipedia, "SAE J1708": 8N1 characters, RS-485 transceivers wired for
  open-collector operation, at most 21 characters while the vehicle moves.
* Not checked against the SAE text: that an idle of 10 bit times is what
  ends a message (Kvaser gives it as the shortest idle before one), the
  exact point from which idle and gaps are measured (taken here as the
  end of the last stop bit), and whether collision detection is per bit
  or per byte (taken here as per bit, the only reading under which the
  winner's message survives). docs/spec-questions: see QUESTIONS.md of
  W-082.

:class:`J1708Bus` is the bench model: the bus level is the AND of the
chip's TX pins (as registered one clock earlier, the transceiver loop
delay) and every model node's drive; every chip RX pin sees it.
:class:`J1708Node` sends queued messages at its own bit rate after the bus
access time of each message's priority, reads every bit back at its middle
and on a difference releases the bus, counts a collision and retries.
:class:`J1708Decoder` turns the bus into bytes and messages (a gap of more
than two bit times ends a message); :func:`check_message` and
:func:`check_access` are the timing checks the tests apply.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Iterable, List, Optional, Sequence, Tuple

from tools.protomodels.bench import Drive, Lines, Model, PinSpec, pad_of

CLOCK = 50_000_000          # the chip's clock in the tests
BIT_RATE = 9600             # Kvaser, Quick Facts
IDLE_BITS = 10              # the shortest idle (Kvaser, bit level)
MAX_GAP_BITS = 2            # between the bytes of one message (Kvaser)
MAX_LEN = 21                # bytes in a message, MID and checksum included
PRIORITIES = range(1, 9)    # 1 highest


def access_bits(priority: int) -> int:
    """The bus access time of a message of ``priority``, in bit times."""
    assert priority in PRIORITIES, priority
    return IDLE_BITS + 2 * priority


def checksum(data: Iterable[int]) -> int:
    """The J1708 checksum of the MID and the data: the two's complement of
    their eight-bit sum."""
    return (-sum(data)) & 0xFF


def sum_ok(message: Iterable[int]) -> bool:
    """True when a whole message (MID, data, checksum) sums to 0."""
    return sum(message) & 0xFF == 0


def frame(mid: int, data: Sequence[int]) -> List[int]:
    """A message as it goes on the bus: MID, data, checksum."""
    body = [mid & 0xFF] + [b & 0xFF for b in data]
    return body + [checksum(body)]


def byte_bits(value: int) -> List[int]:
    """One byte on the wire: start bit, eight data bits LSB first, stop bit."""
    return [0] + [(value >> i) & 1 for i in range(8)] + [1]


# ------------------------------------------------------------------ decoding
@dataclass
class ByteOnBus:
    """One byte as a receiver saw it: its start edge, its value, whether
    its stop bit was high, and every bus edge from the start edge to the
    stop bit's sample as ``(cycle, level)``."""

    start: int
    T: float
    value: int = 0
    framing_ok: bool = True
    edges: List[Tuple[int, int]] = field(default_factory=list)

    @property
    def end(self) -> float:
        """The end of the stop bit, at the receiver's bit time."""
        return self.start + 10 * self.T


@dataclass
class Message:
    """Bytes separated by gaps of at most two bit times."""

    bytes: List[ByteOnBus] = field(default_factory=list)

    @property
    def values(self) -> List[int]:
        return [b.value for b in self.bytes]

    @property
    def start(self) -> int:
        return self.bytes[0].start

    @property
    def end(self) -> float:
        return self.bytes[-1].end

    def gaps(self) -> List[float]:
        """Idle clocks between consecutive bytes (stop end to start edge)."""
        return [b.start - a.end for a, b in zip(self.bytes, self.bytes[1:])]


class J1708Decoder:
    """Decodes the bus at a bit time of ``T`` clocks, as a J1708 receiver:
    each byte from the falling edge of its start bit, each bit sampled at
    its middle; a byte whose start edge comes more than
    :data:`MAX_GAP_BITS` bit times after the previous byte's stop bit
    ended starts a new :class:`Message`."""

    def __init__(self, T: float) -> None:
        self.T = float(T)
        self.bytes: List[ByteOnBus] = []
        self.messages: List[Message] = []
        self.prev = 1
        self.last_low = -1          # the last cycle the bus was low
        self._byte: Optional[ByteOnBus] = None
        self._samples: List[int] = []
        self._next = 0.0

    def feed(self, level: int, cycle: int) -> None:
        """The bus level during ``cycle``."""
        if level == 0:
            self.last_low = cycle
        if level != self.prev:
            if self._byte is not None:
                self._byte.edges.append((cycle, level))
            elif level == 0:
                self._byte = ByteOnBus(cycle, self.T, edges=[(cycle, 0)])
                self._samples = []
                self._next = cycle + 0.5 * self.T
            self.prev = level
        if self._byte is not None and cycle >= self._next:
            self._sample(level)

    @property
    def busy(self) -> bool:
        """A byte is being received."""
        return self._byte is not None

    def idle_since(self) -> float:
        """Where the current idle began: the end of the last byte's stop
        bit, or the cycle after the bus was last low if that is later."""
        end = self.bytes[-1].end if self.bytes else -1e9
        return max(end, self.last_low + 1.0)

    def _sample(self, level: int) -> None:
        byte = self._byte
        self._samples.append(level)
        k = len(self._samples)
        if k == 1 and level:
            self._byte = None                   # a glitch, not a start bit
            return
        if k < 10:
            self._next = byte.start + (k + 0.5) * self.T
            return
        byte.value = sum(bit << i for i, bit in enumerate(self._samples[1:9]))
        byte.framing_ok = level == 1
        self._byte = None
        if self.bytes and self.messages and \
                byte.start - self.bytes[-1].end <= MAX_GAP_BITS * self.T:
            self.messages[-1].bytes.append(byte)
        else:
            self.messages.append(Message([byte]))
        self.bytes.append(byte)


# ------------------------------------------------------------------ checks
def check_message(msg: Message, T: float, *, tol: float = 0.01,
                  slack: float = 8) -> List[str]:
    """What is wrong with ``msg`` as a sender's message at bit time ``T``:
    every edge of every byte on the bit grid its start edge sets (within
    ``tol`` of the elapsed time plus ``slack`` clocks: the chip's slot
    grid), every gap at most two bit times, every stop bit high, the sum
    0 and the length at most 21. An empty list means it is good."""
    bad = []
    if not 2 <= len(msg.bytes) <= MAX_LEN:
        bad.append("length %d" % len(msg.bytes))
    if not sum_ok(msg.values):
        bad.append("sum 0x%02X" % (sum(msg.values) & 0xFF))
    for gap in msg.gaps():
        if gap > MAX_GAP_BITS * T:
            bad.append("gap %.1f bit times" % (gap / T))
    for byte in msg.bytes:
        if not byte.framing_ok:
            bad.append("framing at %d" % byte.start)
        for cycle, _ in byte.edges:
            k = round((cycle - byte.start) / T)
            off = cycle - byte.start - k * T
            if abs(off) > tol * k * T + slack:
                bad.append("edge %d of byte at %d off by %.1f" % (k, byte.start, off))
    return bad


def check_access(idle_from: float, start: int, priority: int, T: float) -> bool:
    """A message of ``priority`` whose start edge is ``start`` waited its bus
    access time after the bus went idle at ``idle_from``."""
    return start - idle_from >= access_bits(priority) * T


# --------------------------------------------------------------------- nodes
@dataclass
class Sent:
    """A message a model node was asked to send and what became of it."""

    values: List[int]
    priority: int
    gaps: List[float]                # idle bit times before bytes 1 .. n-1
    collisions: int = 0
    starts: List[int] = field(default_factory=list)  # every attempt's start edge
    done: bool = False


class J1708Node:
    """A J1708 node with its own clock (bit time ``clock / (rate (1 +
    clock_error))``). :meth:`send` queues a message; the node starts it once
    the bus has been idle for the message's access time, reads every bit
    back at its middle and, when the bus differs from what it sent, lets go
    at once and tries again after the next access time. It decodes the
    bus in :attr:`decoder`."""

    def __init__(self, rate: float, *, clock: int = CLOCK,
                 clock_error: float = 0.0) -> None:
        self.T = clock / (rate * (1 + clock_error))
        self.decoder = J1708Decoder(self.T)
        self.queue: Deque[Sent] = deque()
        self.sent: List[Sent] = []
        self.level = 1                  # what the node drives next cycle
        self._bits: List[Tuple[float, float, int]] = []  # (start, middle, bit)
        self._current: Optional[Sent] = None
        self._k = 0

    def send(self, mid: int, data: Sequence[int], *, priority: int = 8,
             checksum_error: bool = False,
             gaps: Optional[Sequence[float]] = None) -> Sent:
        """Queue MID + data + checksum (spoiled on request); ``gaps`` are
        the idle bit times before each byte after the MID (default 0)."""
        access_bits(priority)
        values = frame(mid, data)
        if checksum_error:
            values[-1] ^= 0x01
        gaps = list(gaps) if gaps is not None else [0.0] * (len(values) - 1)
        assert len(gaps) == len(values) - 1
        job = Sent(values, priority, gaps)
        self.queue.append(job)
        return job

    def _start(self, t0: float, job: Sent) -> None:
        self._current = job
        job.starts.append(int(round(t0)))
        t = t0
        bits: List[Tuple[float, float, int]] = []
        for k, value in enumerate(job.values):
            if k:
                t += job.gaps[k - 1] * self.T
            for bit in byte_bits(value):
                bits.append((t, t + 0.5 * self.T, bit))
                t += self.T
        self._bits = bits
        self._k = 0                             # the next bit to read back

    def clock(self, bus: int, cycle: int) -> None:
        """Take the bus level of ``cycle``; set the drive for ``cycle + 1``."""
        self.decoder.feed(bus, cycle)
        job = self._current
        if job is not None:
            while self._k < len(self._bits) and self._bits[self._k][1] <= cycle:
                bit = self._bits[self._k][2]
                self._k += 1
                if bit != bus:                  # read back: lost, or a fault
                    job.collisions += 1
                    self._bits = []
                    self._current = None
                    self.level = 1
                    return
            while self._bits and self._bits[0][0] + self.T <= cycle + 1:
                self._bits.pop(0)               # over: no longer driven
                self._k -= 1
            if self._k >= len(self._bits):     # every bit read back
                job.done = True
                self.sent.append(job)
                self.queue.popleft()
                self._current = None
                self.level = 1
                return
        elif self.queue and bus == 1 and not self.decoder.busy:
            access = access_bits(self.queue[0].priority) * self.T
            if cycle + 1 - self.decoder.idle_since() >= access:
                self._start(cycle + 1, self.queue[0])
        level = 1
        nxt = cycle + 1
        for start, _, bit in self._bits:
            if start > nxt:
                break                           # a gap before this byte
            if start + self.T > nxt:
                level = bit
                break
        self.level = level


# ----------------------------------------------------------------------- bus
class J1708Bus(Model):
    """The bus on the bench: a wired AND, 1 recessive.

    The level of cycle ``c`` is the AND of the chip's TX pins as they were
    in cycle ``c - 1`` (the transceivers' loop delay) and of every node's
    drive; all the chip's RX pins see it and every node is clocked with
    it. :attr:`edges` records every change as ``(cycle, level)``."""

    def __init__(self, nodes: Iterable = (), tx: Sequence[PinSpec] = ("OUT0",),
                 rx: Sequence[PinSpec] = ("IN0",)) -> None:
        self.nodes = list(nodes)
        self.tx_pads = [pad_of(p) for p in tx]
        self.rx_pads = [pad_of(p) for p in rx]
        self.level = 1
        self.edges: List[Tuple[int, int]] = []
        self._chip = 1

    def add(self, node) -> None:
        self.nodes.append(node)

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
            chip &= lines.get(pad)
        self._chip = chip
        for node in self.nodes:
            node.clock(self.level, lines.cycle)


class J1708Sniffer:
    """A node that only listens, for tests that time the chip's nodes."""

    level = 1

    def __init__(self, T: float) -> None:
        self.decoder = J1708Decoder(T)

    def clock(self, bus: int, cycle: int) -> None:
        self.decoder.feed(bus, cycle)
