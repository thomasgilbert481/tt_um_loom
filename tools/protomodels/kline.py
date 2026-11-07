"""ISO 9141-2 K-line reference models: the line, a tester, an ECU, a decoder
and a timing checker (W-081, D-035).

Written from public descriptions of ISO 9141-2 (the standard itself is not
free, and the copy found, a scanned PDF, could not be read here), never
from the firmware. Every figure is **secondary**; the director should check
them against ISO 9141-2:1994 section 5 (initialisation) and its timing
table before anything quotes them as the standard's:

* [DGT] DG Technologies, Gryphon hardware manual, "KWP2000 / BDLC card",
  section "Card-specific IOCTL definitions", GKWPSETWTIMES and
  GKWPSETPTIMES defaults (https://www.dgtech.com/product/gryphon/manual/
  html/hw/kwp/, read 2026-11-07);
* [M0] m0agx, "Reading OBD2 data without ELM327, part 2: K-line"
  (2018-01-02, repost at matt2005.github.io, read 2026-11-07): the
  5-baud sequence, the key bytes, the headers, the checksum and the mode 1
  PID 0 example frames;
* [TS] P. Dzhelekarski et al., Technical University of Sofia, quoted at
  cnblogs.com/shangdawei/p/3593598.html (read 2026-11-07): the 200 ms
  5-baud bit, P3 max 5000 ms.

What the models take as the protocol:

* the K line is one wire with a pull-up: recessive 1, dominant 0, every
  node's transmitter can pull it low and every node hears it, its own
  bytes included. Bytes are 8N1, LSB first [M0];
* initialisation [M0, TS]: the tester sends the address 0x33 at 5 baud
  (a 200 ms bit [TS], 2 s for the byte); the ECU answers at 10400 bit/s
  with the sync byte 0x55 and the key bytes KB1 and KB2, which are 0x08
  0x08 or 0x94 0x94 for ISO 9141-2 [M0]; the tester sends KB2 inverted and
  the ECU the address inverted (0xCC);
* messages [M0]: request header 0x68 0x6A 0xF1, response header 0x48
  0x6B and the ECU's address, then the data bytes and the checksum, the
  sum of every byte before it modulo 256. Examples: 68 6A F1 01 00 C4 and
  48 6B 11 41 00 FF FF FF FF 01. There is no length byte;
* timing in ms, (minimum, maximum), :data:`TIMING` [DGT defaults; the
  minimum is DGT's transmit default and the maximum its receive default]:
  W1 sync after the address (60, 300); W2 KB1 after the sync (5, 20); W3
  KB2 after KB1 (0, 20); W4 inverted KB2 after KB2 and inverted address
  after inverted KB2 (25, 50); W5 line idle before the address (300, -);
  P1 between ECU bytes (0, 20); P2 request end to response (25, 50); P3
  response end to the next request (55, 5000); P4 between tester bytes
  (5, 20). Each is measured from the end of one byte's stop bit to the
  start of the next byte's start bit (the reading the firmware also
  takes; unchecked against the standard's figure).

The bit-rate tolerance of the 10400 bit/s bytes is not in these sources;
the checker holds every edge to its byte's own grid within
:data:`BIT_TOL` (1 %, an assumption) plus the 4-clock slot grid.

Every time here is in clocks. A test runs the firmware's scaled builds by
giving the models the same scale: ``ms`` (clocks per ms) and ``bit``
(clocks per 10400 bit/s bit); the 5-baud bit is always ``200 * ms``.

:class:`KLineBus` is the bench model: the line is the AND of the chip's TX
pins (as they were one clock earlier, the transceiver loop) and every model
node, and that is what every chip RX pin sees. :class:`KLineTester` and
:class:`KLineEcu` are nodes written as Python generators over the line.
:func:`decode` turns the line's edges into bytes and :func:`check_session`
holds a session to :data:`TIMING`.
"""

from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from tools.protomodels.bench import Drive, Lines, Model, PinSpec, pad_of

CLOCK = 50_000_000
BAUD = 10400
ADDR = 0x33
SYNC = 0x55
KEYS_9141 = ((0x08, 0x08), (0x94, 0x94))
REQ_HEADER = (0x68, 0x6A, 0xF1)
RESP_HEADER = (0x48, 0x6B)
BIT_TOL = 0.01                       # assumption: not in the sources
SLOT = 4

#: name -> (minimum ms, maximum ms or None, what it spans) [DGT]
TIMING: Dict[str, Tuple[float, Optional[float], str]] = {
    "W1": (60, 300, "address end to sync start"),
    "W2": (5, 20, "sync end to KB1 start"),
    "W3": (0, 20, "KB1 end to KB2 start"),
    "W4": (25, 50, "KB2 end to ~KB2 start, ~KB2 end to ~address start"),
    "W5": (300, None, "line idle before the address"),
    "P1": (0, 20, "between ECU response bytes"),
    "P2": (25, 50, "request end to response start"),
    "P3": (55, 5000, "response end to the next request"),
    "P4": (5, 20, "between tester request bytes"),
}


def ms_clocks(clock: int = CLOCK) -> float:
    return clock / 1000.0


def bit_clocks(baud: float = BAUD, clock: int = CLOCK) -> float:
    return clock / baud


def checksum(data: Iterable[int]) -> int:
    """The ISO 9141-2 checksum: the sum of the bytes modulo 256 [M0]."""
    return sum(data) & 0xFF


def request(mode: int, pid: int) -> List[int]:
    body = list(REQ_HEADER) + [mode & 0xFF, pid & 0xFF]
    return body + [checksum(body)]


def response(src: int, data: Sequence[int]) -> List[int]:
    body = list(RESP_HEADER) + [src & 0xFF] + [d & 0xFF for d in data]
    return body + [checksum(body)]


def frame_bits(value: int) -> List[int]:
    """Start bit, eight data bits LSB first, stop bit."""
    return [0] + [(value >> i) & 1 for i in range(8)] + [1]


# ---------------------------------------------------------------- the line
class KLineBus(Model):
    """The K line on the bench: a wired AND with a pull-up.

    The level of cycle ``c`` is the AND of the chip's TX pins as they were
    in cycle ``c - 1`` (that clock is the transceiver loop) and of every
    node's drive; every chip RX pin sees it and every node is clocked with
    it. :attr:`edges` records each change as ``(cycle, level)``.
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


# ---------------------------------------------------------------- the nodes
class _Node:
    """A K-line node written as a generator, :meth:`_run`, that yields

    * ``("wait", cycle)``: resume at ``cycle``;
    * ``("tx", byte, bit)``: send ``byte`` 8N1 at ``bit`` clocks a bit from
      now; resume at the end of its stop bit;
    * ``("rx", bit, deadline)``: receive a byte at ``bit`` clocks a bit
      whose start bit comes before ``deadline`` (``None``: any time);
      resume at the end of its stop bit with the byte, or with ``None`` if
      the deadline passed, the start bit was high at its middle or the stop
      bit low. :attr:`rx_start`, :attr:`rx_end` time the last byte.

    :attr:`now` is the cycle the generator was resumed in.
    """

    def __init__(self, ms: float, bit: float) -> None:
        self.ms, self.bit = ms, bit
        self.level = 1
        self.now = 0
        self.done = False
        self.rx_start = self.rx_end = None
        self._tx: List[Tuple[int, int]] = []
        self._proc = None
        self._want = None
        self._pending = None
        self._rxs = None

    def _run(self):                                  # pragma: no cover
        yield ("wait", 0)

    def _resume(self, value, cycle: int) -> None:
        self.now = cycle
        try:
            want = self._proc.send(value)
        except StopIteration:
            self._want, self.done = None, True
            return
        if want[0] == "tx":
            bits = frame_bits(want[1])
            for i, b in enumerate(bits):
                self._tx.append((round(cycle + 1 + i * want[2]), b))
            self._pending = None
            self._want = ("wait", round(cycle + 1 + 10 * want[2]))
        else:
            self._rxs = None
            self._want = want

    def clock(self, bus: int, cycle: int) -> None:
        while self._tx and self._tx[0][0] <= cycle + 1:
            self.level = self._tx.pop(0)[1]
        if self._proc is None:
            self._proc = self._run()
            self._resume(None, cycle)
        want = self._want
        if want is None:
            return
        if want[0] == "wait":
            if cycle >= want[1]:
                value, self._pending = self._pending, None
                self._resume(value, cycle)
            return
        bit, deadline = want[1], want[2]
        st = self._rxs
        if st is None:
            if bus == 0:
                self._rxs = [cycle, []]
            elif deadline is not None and cycle >= deadline:
                self._resume(None, cycle)
            return
        start, bits = st
        if cycle >= start + (len(bits) + 0.5) * bit:
            bits.append(bus)
            if len(bits) == 1 and bus:
                self._resume(None, cycle)            # a glitch
            elif len(bits) == 10:
                value = sum(b << i for i, b in enumerate(bits[1:9]))
                self.rx_start, self.rx_end = start, round(start + 10 * bit)
                self._pending = value if bus else None
                self._want = ("wait", self.rx_end)

    # helpers for the generators
    def _ms(self, n: float) -> int:
        return round(n * self.ms)

    def _rx(self, deadline: Optional[int]):
        return ("rx", self.bit, deadline)

    def _tx_byte(self, value: int):
        return ("tx", value & 0xFF, self.bit)


@dataclass
class TesterResult:
    """What :class:`KLineTester` saw: ``status`` is ``"ok"`` or the step
    that failed (``"sync"``, ``"keys"``, ``"address"``, ``"response"``,
    ``"checksum"``)."""

    status: str = "running"
    keys: List[int] = field(default_factory=list)
    inverted_address: Optional[int] = None
    response: List[int] = field(default_factory=list)


class KLineTester(_Node):
    """A tester: W5, the 5-baud address, the key exchange and one request.

    Its own times (``w5``, ``w4``, ``p3``, ``p4``, ms) default to values
    inside :data:`TIMING`; it waits for each byte for the window's maximum
    plus one ms. ``inverted_key`` replaces ~KB2 with a byte of the test's
    choosing; ``spoil_checksum`` adds 1 to the request's checksum.
    """

    def __init__(self, mode: int, pid: int, *, ms: float = None, bit: float = None,
                 address: int = ADDR, w5: float = 300, w4: float = 30,
                 p3: float = 70, p4: float = 10, inverted_key: Optional[int] = None,
                 spoil_checksum: bool = False) -> None:
        super().__init__(ms or ms_clocks(), bit or bit_clocks())
        self.address = address
        self.mode, self.pid = mode, pid
        self.w5, self.w4, self.p3, self.p4 = w5, w4, p3, p4
        self.inverted_key = inverted_key
        self.spoil_checksum = spoil_checksum
        self.result = TesterResult()

    def _run(self):
        res = self.result
        yield ("wait", self.now + self._ms(self.w5))
        yield ("tx", self.address, 200 * self.ms)
        b = yield self._rx(self.now + self._ms(301))
        if b != SYNC:
            res.status = "sync"
            return
        for _ in range(2):
            b = yield self._rx(self.now + self._ms(21))
            if b is None:
                res.status = "keys"
                return
            res.keys.append(b)
        yield ("wait", self.now + self._ms(self.w4))
        inv = 0xFF ^ res.keys[1] if self.inverted_key is None else self.inverted_key
        yield self._tx_byte(inv)
        b = yield self._rx(self.now + self._ms(51))
        res.inverted_address = b
        if b != 0xFF ^ self.address:
            res.status = "address"
            return
        req = request(self.mode, self.pid)
        if self.spoil_checksum:
            req[-1] = (req[-1] + 1) & 0xFF
        yield ("wait", self.now + self._ms(self.p3))
        for i, byte in enumerate(req):
            if i:
                yield ("wait", self.now + self._ms(self.p4))
            yield self._tx_byte(byte)
        b = yield self._rx(self.now + self._ms(51))
        while b is not None:
            res.response.append(b)
            b = yield self._rx(self.now + self._ms(21))
        if not res.response:
            res.status = "response"
        elif checksum(res.response[:-1]) != res.response[-1]:
            res.status = "checksum"
        else:
            res.status = "ok"


class KLineEcu(_Node):
    """An ECU: the 5-baud address, sync and keys, the inverted-key check,
    the inverted address, then answers from ``table`` ({(mode, pid): data
    bytes}) until 5 s pass without a request (P3 max).

    Its own times (``w1`` .. ``w4``, ``p1``, ``p2``, ms) default inside
    :data:`TIMING`. A request ends at 20 ms (P4 max) of silence; it is
    answered when it is six bytes with the request header, a right checksum
    and a key in the table. ``spoil_checksum`` adds 1 to every answer's
    checksum. :attr:`events` logs what happened, as tuples.
    """

    def __init__(self, table: Dict[Tuple[int, int], Sequence[int]], *,
                 ms: float = None, bit: float = None, address: int = ADDR,
                 keys: Tuple[int, int] = (0x08, 0x08), src: int = 0x10,
                 w1: float = 100, w2: float = 10, w3: float = 10, w4: float = 30,
                 p1: float = 5, p2: float = 30, spoil_checksum: bool = False) -> None:
        super().__init__(ms or ms_clocks(), bit or bit_clocks())
        self.table = {k: list(v) for k, v in table.items()}
        self.address, self.keys, self.src = address, keys, src
        self.w = (w1, w2, w3, w4)
        self.p1, self.p2 = p1, p2
        self.spoil_checksum = spoil_checksum
        self.events: List[tuple] = []

    def _run(self):
        w1, w2, w3, w4 = self.w
        while True:
            b = yield ("rx", 200 * self.ms, None)
            if b != self.address:
                continue
            yield ("wait", self.now + self._ms(w1))
            yield self._tx_byte(SYNC)
            yield ("wait", self.now + self._ms(w2))
            yield self._tx_byte(self.keys[0])
            yield ("wait", self.now + self._ms(w3))
            yield self._tx_byte(self.keys[1])
            b = yield self._rx(self.now + self._ms(51))
            if b != 0xFF ^ self.keys[1]:
                self.events.append(("no_session", b))
                continue
            yield ("wait", self.now + self._ms(w4))
            yield self._tx_byte(0xFF ^ self.address)
            self.events.append(("session",))
            yield from self._session()

    def _session(self):
        while True:
            b = yield self._rx(self.now + self._ms(5000))
            if b is None:
                self.events.append(("closed",))
                return
            req = [b]
            while True:
                b = yield self._rx(self.now + self._ms(21))
                if b is None:
                    break
                req.append(b)
            end = self.rx_end
            key = (req[3], req[4]) if len(req) == 6 else None
            good = (key is not None and tuple(req[:3]) == REQ_HEADER
                    and checksum(req[:-1]) == req[-1] and key in self.table)
            self.events.append(("request", req, good))
            if not good:
                continue
            ans = response(self.src, self.table[key])
            if self.spoil_checksum:
                ans[-1] = (ans[-1] + 1) & 0xFF
            yield ("wait", end + self._ms(self.p2))
            for i, byte in enumerate(ans):
                if i:
                    yield ("wait", self.now + self._ms(self.p1))
                yield self._tx_byte(byte)


# ------------------------------------------------------- decoding, timing
@dataclass
class KByte:
    """One byte on the line: its start edge and nominal stop-bit end, in
    clocks, its value, whether it was at 5 baud, its framing, and every
    edge inside it."""

    start: int
    end: float
    value: int
    slow: bool
    framed: bool
    edges: List[Tuple[int, int]]
    bit: float


def decode(edges: Sequence[Tuple[int, int]], ms: float, bit: float) -> List[KByte]:
    """The bytes in a line's ``edges`` (``KLineBus.edges``; the line is
    high before the first). A low run longer than 12 bits at ``bit``
    (a 10400 bit/s byte is at most 9 bits low) is a 5-baud byte, sampled at
    ``200 * ms`` a bit; anything else is sampled at ``bit``. A low run longer
    than nine and a half bits at its rate is no byte, nor is the low the line
    starts with (the chip's TX pins are low from reset until the program
    sets them): both are skipped."""
    cycles = [c for c, _ in edges]
    levels = [lv for _, lv in edges]

    def at(t: float) -> int:
        i = bisect_right(cycles, t) - 1
        return levels[i] if i >= 0 else 1

    out: List[KByte] = []
    free = float("-inf")
    for k, (f, lv) in enumerate(edges):
        if lv or f < free:
            continue
        rise = edges[k + 1][0] if k + 1 < len(edges) else float("inf")
        slow = rise - f > 12 * bit
        b = 200 * ms if slow else bit
        if f < SLOT or rise - f > 9.5 * b:
            continue        # the chip's reset low, or longer than any byte's low run
        bits = [at(f + (i + 0.5) * b) for i in range(10)]
        inside = [(c, x) for c, x in edges[k:] if c < f + 10 * b]
        out.append(KByte(f, f + 10 * b, sum(v << i for i, v in enumerate(bits[1:9])),
                         slow, bits[0] == 0 and bits[9] == 1, inside, b))
        free = f + 9.5 * b
    return out


def bit_errors(byte: KByte, tol: float = BIT_TOL) -> List[str]:
    """Edges of ``byte`` off its own grid (from its start edge) by more
    than ``tol`` of the elapsed time plus two slots."""
    errs = []
    for c, _ in byte.edges:
        k = round((c - byte.start) / byte.bit)
        off = c - byte.start - k * byte.bit
        if abs(off) > tol * k * byte.bit + 2 * SLOT:
            errs.append("byte 0x%02X at %d: edge %d off bit %d by %.1f clocks"
                        % (byte.value, byte.start, c, k, off))
    return errs


@dataclass
class Session:
    """A decoded session: the bytes in protocol order and each W/P time
    measured, in ms, as ``{name: [values]}``; :attr:`violations` lists
    every time outside :data:`TIMING` and every bad bit edge."""

    address: Optional[KByte] = None
    init: List[KByte] = field(default_factory=list)
    request: List[KByte] = field(default_factory=list)
    response: List[KByte] = field(default_factory=list)
    times: Dict[str, List[float]] = field(default_factory=dict)
    violations: List[str] = field(default_factory=list)


def check_session(edges: Sequence[Tuple[int, int]], ms: float, bit: float,
                  slack: float = 2 * SLOT) -> Session:
    """Split the line into the address, the five init bytes (sync, KB1, KB2,
    ~KB2, ~address), the request (bytes until a gap over P4 max) and the
    response (the rest), and hold every gap to :data:`TIMING` within
    ``slack`` clocks. Only the parts that are there are checked."""
    s = Session()
    seq = decode(edges, ms, bit)
    slow = [i for i, b in enumerate(seq) if b.slow]
    if not slow:
        s.violations.append("no 5-baud address")
        return s
    i0 = slow[-1]
    s.address = seq[i0]
    # W5: the line high without a break up to the address's start edge
    # (from the start of the run if it never went low before).
    rises = [c for c, lv in edges if lv and c < seq[i0].start]
    s.times["W5"] = [(seq[i0].start - (rises[-1] if rises else 0)) / ms]
    rest = seq[i0 + 1:]
    s.init = rest[:5]
    msg = rest[5:]
    split = len(msg)
    for k in range(1, len(msg)):
        if msg[k].start - msg[k - 1].end > TIMING["P4"][1] * ms:
            split = k
            break
    s.request, s.response = msg[:split], msg[split:]

    def gap(name, a, b):
        s.times.setdefault(name, []).append((b.start - a.end) / ms)

    chain = [s.address] + s.init
    for k, name in enumerate(("W1", "W2", "W3", "W4", "W4")):
        if k + 1 < len(chain):
            gap(name, chain[k], chain[k + 1])
    if s.request and len(s.init) == 5:
        gap("P3", s.init[-1], s.request[0])
    for a, b in zip(s.request, s.request[1:]):
        gap("P4", a, b)
    if s.request and s.response:
        gap("P2", s.request[-1], s.response[0])
    for a, b in zip(s.response, s.response[1:]):
        gap("P1", a, b)
    for name, values in s.times.items():
        lo, hi, _ = TIMING[name]
        for v in values:
            if v * ms < lo * ms - slack or (hi is not None and v * ms > hi * ms + slack):
                s.violations.append("%s = %.2f ms outside (%s, %s)" % (name, v, lo, hi))
    for b in seq[i0:]:
        if not b.framed:
            s.violations.append("byte 0x%02X at %d: framing" % (b.value, b.start))
        s.violations += bit_errors(b)
    return s
