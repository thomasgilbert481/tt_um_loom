"""LIN reference models: the bus, a master node and a slave node.

Written from the LIN 2.2A specification (ISO 17987-2 and -3 say the same),
not from the firmware. No copy of the text was at hand when it was written:
the figures below are the author's reading, not yet checked against the
document (docs/spec-questions/firmware-d035.md, LIN item 1). The checksum
example the specification gives is a test.

* the bus is a wired AND with a pull-up: recessive 1, dominant 0. Every
  node's transmitter can pull it low, and every node reads it back;
* a byte field is a start bit (dominant), eight data bits LSB first and a
  stop bit (recessive), 8N1; a receiver re-synchronises on the falling
  edge of every start bit and samples each bit near its middle;
* a frame is a header sent by the master - a break of at least 13 nominal
  bit times dominant, a break delimiter of at least one bit recessive, the
  sync byte 0x55, the protected identifier - then a response of 1 to 8
  data bytes and a checksum, sent by the master or by one slave. A slave
  detects a break as a dominant run of at least 11 nominal bit times;
* the protected identifier (PID) is the 6-bit identifier with two parity
  bits: P0 = ID0 ^ ID1 ^ ID2 ^ ID4 in bit 6, P1 = ~(ID1 ^ ID3 ^ ID4 ^ ID5)
  in bit 7;
* the checksum is the inverted eight-bit sum with carry (each carry out of
  bit 7 is added back into bit 0) over the data bytes (classic) or over the
  PID and the data bytes (enhanced). LIN 2.x frames use the enhanced
  checksum, the diagnostic frames 0x3C and 0x3D always the classic one;
* timing: THeader_Nominal = 34 bit times, TResponse_Nominal = 10 (N + 1)
  bit times; the header may take up to 1.4 times its nominal length and so
  may the response (TResponse_Maximum = 1.4 TResponse_Nominal, counted from
  the end of the header to the end of the checksum's stop bit);
* clock tolerance: the master within +-0.5 % of the nominal bit rate; a
  slave that does not synchronise within +-1.5 %; a slave that
  synchronises on the sync byte may be off by up to +-14 % before it does
  (F_TOL_UNSYNC) and must be within +-2 % of the master's bit rate after
  it (F_TOL_SYNC). Its measurement takes the time between the first and the
  fifth falling edge of the sync byte, which is 8 bit times.

:class:`LinBus` is the bench model (``tools.protomodels.bench.Model``): the
bus level is the AND of the chip's TX pins and every model node's driver,
and that level is what the chip's RX pins see, one clock after the chip
drove it (the loop delay of a transceiver, at its shortest). Several TX
and RX pins may be given: two Loom threads each with their own pins on one
bus is how ``test_fw_lin.py`` runs a master and a slave together.

:class:`LinMasterNode` sends headers at a bit rate of its own (nominal rate
times ``1 + clock_error``), with a settable break and delimiter length, a
PID with spoiled parity on request, and either its own response (a
spoiled checksum on request) or nothing, in which case it records the
slave's response. :class:`LinSlaveNode` is a slave at a fixed bit rate
that publishes or subscribes according to a table, can spoil its
checksum or stay silent, and records every frame it sees. Both decode the
bus with :class:`LinDecoder`, which records every byte with its edges, so
a test can measure any node's bit timing on the wire.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, Iterable, List, Optional, Sequence, Tuple

from tools.protomodels.bench import Drive, Lines, Model, PinSpec, pad_of

CLOCK = 50_000_000          # the chip's clock in the tests
SYNC = 0x55
BREAK_BITS = 13             # the master's minimum break
BREAK_THRESHOLD = 11        # a slave's break detection threshold
CLASSIC_IDS = (0x3C, 0x3D)  # diagnostic frames: classic checksum
F_TOL_SYNC = 0.02           # a synchronised slave against the master
F_TOL_UNSYNC = 0.14         # a slave before it synchronises


def bit_clocks(rate: float, clock: int = CLOCK) -> float:
    """Clocks per bit at ``rate`` bit/s."""
    return clock / rate


def pid(ident: int) -> int:
    """The protected identifier of a 6-bit frame identifier."""
    assert 0 <= ident < 64
    b = [(ident >> i) & 1 for i in range(6)]
    p0 = b[0] ^ b[1] ^ b[2] ^ b[4]
    p1 = 1 ^ (b[1] ^ b[3] ^ b[4] ^ b[5])
    return ident | (p0 << 6) | (p1 << 7)


def parity_ok(value: int) -> bool:
    """True when the byte ``value`` is a valid protected identifier."""
    return pid(value & 0x3F) == value


def classic(ident: int) -> bool:
    """The frames that always use the classic checksum."""
    return ident in CLASSIC_IDS


def checksum(data: Iterable[int], protected_id: Optional[int] = None) -> int:
    """The LIN checksum of ``data``: classic, or enhanced when the PID is
    given. An eight-bit sum with every carry added back into bit 0, then
    inverted."""
    total = 0 if protected_id is None else protected_id
    for byte in data:
        total += byte
        if total > 0xFF:
            total -= 0xFF
    return (~total) & 0xFF


def frame_checksum(ident: int, data: Iterable[int]) -> int:
    """The checksum a LIN 2.x node uses for frame ``ident``."""
    return checksum(data, None if classic(ident) else pid(ident))


def byte_bits(value: int) -> List[int]:
    """A byte field on the wire: start bit, eight data bits LSB first, stop
    bit."""
    return [0] + [(value >> i) & 1 for i in range(8)] + [1]


# ------------------------------------------------------------------ decoding
@dataclass
class ByteField:
    """One byte as a receiver saw it on the bus. ``edges`` holds every bus
    edge from the start bit's falling edge to the stop bit's sample, as
    ``(cycle, level)``."""

    start: int
    value: int = 0
    framing_ok: bool = True
    edges: List[Tuple[int, int]] = field(default_factory=list)


@dataclass
class BusFrame:
    """A break and the bytes that followed it, up to the next break."""

    break_start: int
    break_end: int
    bytes: List[ByteField] = field(default_factory=list)

    @property
    def break_clocks(self) -> int:
        return self.break_end - self.break_start

    @property
    def delimiter_clocks(self) -> Optional[int]:
        return self.bytes[0].start - self.break_end if self.bytes else None

    @property
    def pid(self) -> Optional[int]:
        return self.bytes[1].value if len(self.bytes) > 1 else None

    @property
    def response(self) -> List[int]:
        """The values of the bytes after the PID: data, then checksum."""
        return [b.value for b in self.bytes[2:]]


class LinDecoder:
    """Decodes the bus at a bit time of ``T`` clocks, as a LIN receiver:
    every byte from the falling edge of its start bit, each bit sampled at
    its middle, and a dominant run of at least ``threshold`` bit times
    taken as a break, which starts a new :class:`BusFrame`.

    ``listener(frame, byte)`` is called for every byte that completes
    inside a frame."""

    def __init__(self, T: float, threshold: float = BREAK_THRESHOLD,
                 listener=None) -> None:
        self.T = float(T)
        self.threshold = threshold
        self.listener = listener
        self.frames: List[BusFrame] = []
        self.bytes: List[ByteField] = []        # every byte, framed or not
        self.prev = 1
        self._low_since: Optional[int] = None
        self._byte: Optional[ByteField] = None
        self._samples: List[int] = []
        self._next = 0.0

    def feed(self, level: int, cycle: int) -> None:
        """The bus level during ``cycle``."""
        if level != self.prev:
            if self._byte is not None:
                self._byte.edges.append((cycle, level))
            if level == 0:
                self._low_since = cycle
                if self._byte is None:
                    self._byte = ByteField(cycle, edges=[(cycle, 0)])
                    self._samples = []
                    self._next = cycle + 0.5 * self.T
            else:
                low = self._low_since
                if low is not None and cycle - low >= self.threshold * self.T:
                    self._break(low, cycle)
                self._low_since = None
            self.prev = level
        if self._byte is not None and cycle >= self._next:
            self._sample(level)

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
        self.bytes.append(byte)
        if self.frames:
            frame = self.frames[-1]
            frame.bytes.append(byte)
            if self.listener is not None:
                self.listener(frame, byte)

    def _break(self, start: int, end: int) -> None:
        """A break from ``start`` to ``end``: the zero byte with a framing
        error that its first ten bit times decoded to is not a byte."""
        if self.bytes and self.bytes[-1].start == start:
            self.bytes.pop()
            if self.frames and self.frames[-1].bytes \
                    and self.frames[-1].bytes[-1].start == start:
                self.frames[-1].bytes.pop()
        self._byte = None
        self.frames.append(BusFrame(start, end))


# --------------------------------------------------------------------- nodes
class _Node:
    """A node's transmitter: a queue of ``(cycle, level)`` changes, and the
    level it drives in the next cycle in :attr:`tx` (1 = recessive)."""

    def __init__(self, T: float) -> None:
        self.T = float(T)
        self.tx = 1
        self.now = 0
        self._changes: Deque[Tuple[int, int]] = deque()
        self._force: List[Tuple[int, int]] = []      # dominant (from, to)

    def _send(self, t0: float, bits: Sequence[int]) -> float:
        """Queue ``bits`` from ``t0``, one every :attr:`T` clocks, then idle;
        returns the end of the last bit."""
        for k, bit in enumerate(bits):
            self._changes.append((int(round(t0 + k * self.T)), bit))
        end = t0 + len(bits) * self.T
        self._changes.append((int(round(end)), 1))
        return end

    def _next_level(self, cycle: int) -> None:
        nxt = cycle + 1
        changes = self._changes
        while changes and changes[0][0] <= nxt:
            self.tx = changes.popleft()[1]
        self._forced = any(a <= nxt < b for a, b in self._force)

    @property
    def level(self) -> int:
        """What the node drives in the next cycle, forced bits included."""
        return 0 if getattr(self, "_forced", False) else self.tx

    def force_dominant(self, start: float, clocks: float) -> None:
        """Drive the bus dominant for ``clocks`` from cycle ``start``, over
        whatever the node sends: the way to make another node's bit wrong."""
        self._force.append((int(round(start)), int(round(start + clocks))))


@dataclass
class Transfer:
    """A frame the master node was asked for and what became of it."""

    ident: int
    publish: bool
    data: List[int]
    length: int
    start: int                      # the break's falling edge
    header_end: float               # the end of the PID's stop bit
    parity_error: bool = False
    checksum_error: bool = False


class LinMasterNode(_Node):
    """A LIN master with its own clock.

    Args:
        rate: the nominal bit rate.
        clock_error: the master's bit rate is ``rate * (1 + clock_error)``.
        break_bits, delimiter_bits: the header's break and delimiter, in
            the master's bit times (13 and 1 are the minimum).
        gap_bits: idle bit times between the end of one frame slot and the
            next header.
    """

    def __init__(self, rate: float, *, clock: int = CLOCK, clock_error: float = 0.0,
                 break_bits: float = BREAK_BITS, delimiter_bits: float = 1,
                 gap_bits: float = 4) -> None:
        super().__init__(clock / (rate * (1 + clock_error)))
        self.break_bits = break_bits
        self.delimiter_bits = delimiter_bits
        self.gap_bits = gap_bits
        self.decoder = LinDecoder(self.T)
        self.transfers: List[Transfer] = []
        self._free = 0.0

    def _header(self, ident: int, parity_error: bool, start: Optional[int],
                length: int, publish: bool, data: List[int],
                checksum_error: bool) -> Transfer:
        t0 = max(self._free, float(self.now + 2))
        if start is not None:
            t0 = max(t0, float(start))
        p = pid(ident) ^ (0x40 if parity_error else 0)
        brk = int(round(self.break_bits * self.T))
        dlm = int(round(self.delimiter_bits * self.T))
        self._changes.append((int(round(t0)), 0))
        self._changes.append((int(round(t0)) + brk, 1))
        t1 = round(t0) + brk + dlm
        end = self._send(t1, byte_bits(SYNC) + byte_bits(p))
        tr = Transfer(ident, publish, list(data), length, int(round(t0)), end,
                      parity_error, checksum_error)
        self.transfers.append(tr)
        return tr

    def publish(self, ident: int, data: Sequence[int], *, checksum_error: bool = False,
                parity_error: bool = False, start: Optional[int] = None) -> Transfer:
        """A frame whose response the master sends itself."""
        data = list(data)
        tr = self._header(ident, parity_error, start, len(data), True, data,
                          checksum_error)
        ck = frame_checksum(ident, data) ^ (0x01 if checksum_error else 0)
        bits: List[int] = []
        for byte in data + [ck]:
            bits += byte_bits(byte)
        end = self._send(tr.header_end, bits)
        self._free = end + self.gap_bits * self.T
        return tr

    def request(self, ident: int, length: int, *, parity_error: bool = False,
                start: Optional[int] = None) -> Transfer:
        """A header only: a slave is to send the ``length``-byte response.
        The next header waits for the whole TResponse_Maximum."""
        tr = self._header(ident, parity_error, start, length, False, [], False)
        self._free = (tr.header_end + 1.4 * 10 * (length + 1) * self.T
                      + self.gap_bits * self.T)
        return tr

    @property
    def busy(self) -> bool:
        return bool(self._changes) or self.now < self._free

    def frame_of(self, tr: Transfer) -> Optional[BusFrame]:
        """The frame on the bus that ``tr``'s header started."""
        for frame in self.decoder.frames:
            if abs(frame.break_start - tr.start) <= 4:
                return frame
        return None

    def clock(self, bus: int, cycle: int) -> None:
        """Take the bus level of ``cycle``; set the drive for ``cycle + 1``."""
        self.now = cycle
        self.decoder.feed(bus, cycle)
        self._next_level(cycle)


@dataclass
class SlaveFrame:
    """A header the slave node saw, and what it did with the frame."""

    pid: int
    parity_ok: bool
    sync_ok: bool
    published: bool = False             # the slave sent the response
    data: List[int] = field(default_factory=list)   # what it sent or received
    checksum_ok: Optional[bool] = None  # a received response's checksum
    complete: bool = False

    @property
    def ident(self) -> int:
        return self.pid & 0x3F


class LinSlaveNode(_Node):
    """A LIN slave with its own clock, at a bit rate it knows.

    :meth:`publish` and :meth:`subscribe` fill its table; it answers a
    header with a good PID whose identifier it publishes, after
    ``response_space`` bit times, and checks the response of one it
    subscribes to. :attr:`frames` records every header it saw.
    """

    def __init__(self, rate: float, *, clock: int = CLOCK, clock_error: float = 0.0,
                 response_space: float = 0.0) -> None:
        super().__init__(clock / (rate * (1 + clock_error)))
        self.response_space = response_space
        self.decoder = LinDecoder(self.T, listener=self._byte)
        self.publishes: Dict[int, List[int]] = {}
        self.subscribes: Dict[int, int] = {}
        self.spoil: set = set()             # identifiers sent with a bad checksum
        self.silent: set = set()            # identifiers it publishes but ignores
        self.frames: List[SlaveFrame] = []

    def publish(self, ident: int, data: Sequence[int], *,
                checksum_error: bool = False, silent: bool = False) -> None:
        self.publishes[ident] = list(data)
        (self.spoil.add if checksum_error else self.spoil.discard)(ident)
        (self.silent.add if silent else self.silent.discard)(ident)

    def subscribe(self, ident: int, length: int) -> None:
        self.subscribes[ident] = length

    def received(self) -> List[SlaveFrame]:
        """The frames whose response the slave received."""
        return [f for f in self.frames if f.ident in self.subscribes and f.complete]

    def _byte(self, frame: BusFrame, byte: ByteField) -> None:
        n = len(frame.bytes)
        if n == 2:
            sync = frame.bytes[0]
            rec = SlaveFrame(byte.value, parity_ok(byte.value) and byte.framing_ok,
                             sync.value == SYNC and sync.framing_ok)
            self.frames.append(rec)
            ident = rec.ident
            if not (rec.parity_ok and rec.sync_ok):
                return                              # not for anyone
            if ident in self.publishes and ident not in self.silent:
                data = self.publishes[ident]
                ck = frame_checksum(ident, data) ^ (0x01 if ident in self.spoil else 0)
                bits: List[int] = []
                for value in data + [ck]:
                    bits += byte_bits(value)
                start = byte.start + 10 * self.T + self.response_space * self.T
                self._send(max(start, self.now + 2.0), bits)
                rec.published, rec.data, rec.complete = True, list(data), True
        elif n > 2 and self.frames:
            rec = self.frames[-1]
            ident = rec.ident
            if rec.published or rec.complete or ident not in self.subscribes:
                return
            if not (rec.parity_ok and rec.sync_ok):
                return
            length = self.subscribes[ident]
            if len(rec.data) < length:
                rec.data.append(byte.value)
            else:
                rec.checksum_ok = byte.value == frame_checksum(ident, rec.data)
                rec.complete = True

    def clock(self, bus: int, cycle: int) -> None:
        """Take the bus level of ``cycle``; set the drive for ``cycle + 1``."""
        self.now = cycle
        self.decoder.feed(bus, cycle)
        self._next_level(cycle)


# ----------------------------------------------------------------------- bus
class LinBus(Model):
    """The bus on the bench: a wired AND with a pull-up.

    The level of cycle ``c`` is the AND of the chip's TX pins as they were
    in cycle ``c - 1`` (``drive`` may only use registered state, so that
    clock is the transceivers' loop delay) and of every node's drive; all
    the chip's RX pins see it, and every node is clocked with it.
    :attr:`edges` records every change of the bus as ``(cycle, level)``.
    """

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


class LinSniffer:
    """A node that only listens: it drives nothing and decodes the bus at a
    bit time of ``T`` clocks, for tests that measure the chip's own nodes."""

    level = 1

    def __init__(self, T: float) -> None:
        self.decoder = LinDecoder(T)

    def clock(self, bus: int, cycle: int) -> None:
        self.decoder.feed(bus, cycle)
