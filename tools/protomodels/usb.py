"""USB low-speed reference model: the packet layer and a host on D+ / D-.

Everything here is written from the USB 2.0 specification, never from the
firmware or the chip:

* **Packets** (chapter 8): SYNC, the PID with its check nibble (8.3.1),
  tokens with the 7-bit address, 4-bit endpoint and CRC5 (8.3.5.1), data
  packets with CRC16 (8.3.5.2), handshakes. CRCs are computed over the
  protected field in transmission order (least significant bit of every
  field first), seeded with all ones, and the inverted remainder is sent
  most significant bit first; a good packet leaves the residual 01100b
  (CRC5) or 1000000000001101b (CRC16) in the checker.
* **Bit level** (7.1.8, 7.1.9): bit stuffing inserts a 0 after six
  consecutive 1s, counted from the SYNC field on (its last bit counts), and
  also before the EOP; NRZI keeps the level for a 1 and toggles it for a 0.
* **Line** (7.1.7): low speed, so J is D- high and D+ low, K the reverse,
  SE0 both low. The idle bus is J, held by the device's pull-up on D-
  (the bench's ``pullups``) against the host's pull-downs (an undriven bench
  line reads 0). A packet is SYNC (KJKJKJKK), the NRZI bits, then the EOP:
  SE0 for two bit times and J for one, after which the driver lets go.
* **Timing** (7.1.11, 7.1.18, 7.1.19): 1.5 Mbit/s, 33.33 clocks per bit at
  50 MHz; the host transmits within +-0.25 %, a low-speed device within
  +-1.5 %. Packets from the same source are at least two bit times apart;
  a responder starts its packet 2 to 6.5 bit times after the SE0-to-J
  transition that ends the packet it answers, measured at its port; the
  waiting side times out between 16 and 18 bit times.

:class:`UsbHost` is a bench model (``tools.protomodels.bench``): it drives
both lines push-pull while it transmits and releases them otherwise, and it
records every change of the line state, so the tests can measure the
device's bit rate, jitter, EOP and response time against the numbers above.
"""

from __future__ import annotations

import dataclasses
from collections import deque
from typing import Callable, Deque, Generator, List, Optional, Sequence, Tuple

from .bench import Drive, Lines, Model, PinSpec, pad_of

# ------------------------------------------------------------------ PIDs
OUT, IN, SOF, SETUP = 0x1, 0x9, 0x5, 0xD                     # tokens
DATA0, DATA1 = 0x3, 0xB                                     # data
ACK, NAK, STALL = 0x2, 0xA, 0xE                             # handshakes
PRE = 0xC

PID_NAMES = {OUT: "OUT", IN: "IN", SOF: "SOF", SETUP: "SETUP", DATA0: "DATA0",
             DATA1: "DATA1", ACK: "ACK", NAK: "NAK", STALL: "STALL", PRE: "PRE"}
TOKENS = (OUT, IN, SOF, SETUP)
DATAS = (DATA0, DATA1)
HANDSHAKES = (ACK, NAK, STALL)

#: The SYNC field as bits in transmission order: seven 0s and a 1 (KJKJKJKK).
SYNC_BITS = (0, 0, 0, 0, 0, 0, 0, 1)

#: Residuals a checker holds after a good field and its CRC (8.3.5).
CRC5_RESIDUAL = 0b01100
CRC16_RESIDUAL = 0b1000000000001101

# ------------------------------------------------------------- line states
J, K, SE0, SE1 = "J", "K", "0", "1"

#: Nominal low-speed bit time in 50 MHz core clocks.
CLOCKS_PER_BIT = 50e6 / 1.5e6

# Timing limits of the specification, in bit times.
IPD_MIN = 2.0               # 7.1.18.1: packets from one source, idle between
RESPONSE_MIN = 2.0          # 7.1.18.1: TRSPIPD1, responder's minimum delay
RESPONSE_MAX = 6.5          # 7.1.18.1: TRSPIPD1, responder's maximum delay
TIMEOUT_MIN = 16.0          # 7.1.19.1: waiting side must not time out before
TIMEOUT_MAX = 18.0          # 7.1.19.1: ... and must have timed out after
EOP_SE0_BITS = 2            # 7.1.13.2: SE0 for two bit times, then J
LS_RATE_TOL = 0.015         # 7.1.11: low-speed function, 1.5 Mbit/s +-1.5 %
HOST_RATE_TOL = 0.0025      # 7.1.11: host/hub low-speed data, +-0.25 %
# Table 7-10, low speed, 1.25 us .. 1.50 us of SE0 at the source (TLEOPT)
EOP_SE0_MIN_US, EOP_SE0_MAX_US = 1.25, 1.50
# Table 7-10, upstream facing (device) port source jitter, next transition
# and paired transitions (TUDJ1, TUDJ2), in ns
DEVICE_JITTER_NEXT_NS, DEVICE_JITTER_PAIRED_NS = 95.0, 150.0


class UsbError(ValueError):
    """A malformed packet or line sequence."""


# ------------------------------------------------------------------- CRCs
def _crc(bits: Sequence[int], width: int, poly: int, init: int) -> int:
    """The serial checker of 8.3.5: returns the remainder (not inverted)."""
    top = 1 << (width - 1)
    mask = (1 << width) - 1
    reg = init
    for bit in bits:
        fb = (1 if reg & top else 0) ^ (bit & 1)
        reg = (reg << 1) & mask
        if fb:
            reg ^= poly
    return reg


def field_bits(value: int, nbits: int) -> List[int]:
    """``value`` as ``nbits`` bits in transmission order (LSB first)."""
    return [(value >> i) & 1 for i in range(nbits)]


def bytes_bits(data: Sequence[int]) -> List[int]:
    """Bytes as bits in transmission order (each byte LSB first)."""
    out: List[int] = []
    for byte in data:
        out += field_bits(byte, 8)
    return out


def bits_value(bits: Sequence[int]) -> int:
    """Inverse of :func:`field_bits`."""
    return sum((b & 1) << i for i, b in enumerate(bits))


def bits_bytes(bits: Sequence[int]) -> bytes:
    """Inverse of :func:`bytes_bits` (``len(bits)`` must be a multiple of 8)."""
    if len(bits) % 8:
        raise UsbError("%d bits is not a whole number of bytes" % len(bits))
    return bytes(bits_value(bits[i:i + 8]) for i in range(0, len(bits), 8))


def _reverse(value: int, nbits: int) -> int:
    return sum(((value >> i) & 1) << (nbits - 1 - i) for i in range(nbits))


def crc5(value: int, nbits: int = 11) -> int:
    """The CRC5 a transmitter sends for an ``nbits`` field (inverted
    remainder, bit 4 is the one sent first)."""
    return _crc(field_bits(value, nbits), 5, 0b00101, 0b11111) ^ 0b11111


def crc16(data: Sequence[int]) -> int:
    """The CRC16 a transmitter sends for ``data`` (inverted remainder, bit 15
    is the one sent first)."""
    return _crc(bytes_bits(data), 16, 0x8005, 0xFFFF) ^ 0xFFFF


def crc5_field(crc: int) -> int:
    """The 5-bit CRC as it sits in the token's LSB-first field (bits 15:11
    of the two bytes after the PID): the MSB is sent first, so it is
    reversed."""
    return _reverse(crc, 5)


def crc16_bytes(crc: int) -> bytes:
    """The two CRC bytes of a data packet, in transmission order."""
    return bytes([_reverse(crc >> 8, 8), _reverse(crc & 0xFF, 8)])


def check_crc5(field16: int) -> bool:
    """True if a received 16-bit token field (address, endpoint, CRC5, LSB
    first) leaves the CRC5 residual in the checker."""
    return _crc(field_bits(field16, 16), 5, 0b00101, 0b11111) == CRC5_RESIDUAL


def check_crc16(data_and_crc: Sequence[int]) -> bool:
    """True if data bytes followed by their two CRC bytes leave the residual."""
    return _crc(bytes_bits(data_and_crc), 16, 0x8005, 0xFFFF) == CRC16_RESIDUAL


# -------------------------------------------------------------- packets
def pid_byte(pid: int) -> int:
    """The PID byte: the four PID bits and their complement above (8.3.1)."""
    pid &= 0xF
    return pid | ((~pid & 0xF) << 4)


def pid_ok(byte: int) -> bool:
    """The check nibble is the complement of the PID nibble."""
    return ((byte >> 4) ^ byte) & 0xF == 0xF


def token_bytes(pid: int, addr: int, ep: int) -> bytes:
    """A token packet after SYNC: PID, then address, endpoint and CRC5."""
    if not 0 <= addr < 128 or not 0 <= ep < 16:
        raise UsbError("address %r or endpoint %r out of range" % (addr, ep))
    value = addr | (ep << 7)
    field = value | (crc5_field(crc5(value)) << 11)
    return bytes([pid_byte(pid), field & 0xFF, field >> 8])


def sof_bytes(frame: int) -> bytes:
    """An SOF packet after SYNC (full-speed only on the wire; here for the
    CRC5 examples over an 11-bit frame number)."""
    field = (frame & 0x7FF) | (crc5_field(crc5(frame & 0x7FF)) << 11)
    return bytes([pid_byte(SOF), field & 0xFF, field >> 8])


def data_bytes(pid: int, payload: Sequence[int]) -> bytes:
    """A data packet after SYNC: PID, payload, CRC16."""
    payload = bytes(payload)
    return bytes([pid_byte(pid)]) + payload + crc16_bytes(crc16(payload))


def handshake_bytes(pid: int) -> bytes:
    return bytes([pid_byte(pid)])


# ------------------------------------------------------------- bit level
def stuff(bits: Sequence[int]) -> List[int]:
    """Insert a 0 after every six consecutive 1s (also at the very end)."""
    out: List[int] = []
    run = 0
    for bit in bits:
        out.append(bit)
        run = run + 1 if bit else 0
        if run == 6:
            out.append(0)
            run = 0
    return out


def unstuff(bits: Sequence[int], final: bool = False) -> List[int]:
    """Remove the stuffed 0s; a 1 where a stuffed 0 is due is an error.

    A stuffed bit may be the last bit of the sequence: a packet whose data
    ends in six 1s carries its stuff bit before the EOP (7.1.9). With
    ``final`` the sequence is a whole packet, so a stuff bit still due at
    its end is an error too."""
    out: List[int] = []
    run = 0
    skip = False
    for i, bit in enumerate(bits):
        if skip:
            if bit:
                raise UsbError("bit stuffing violation at bit %d" % i)
            skip = False
            run = 0
            continue
        out.append(bit)
        run = run + 1 if bit else 0
        if run == 6:
            skip = True
    if final and skip:
        raise UsbError("the packet ends in six 1s without its stuff bit")
    return out


def nrzi(bits: Sequence[int], level: str = J) -> List[str]:
    """NRZI: a 1 keeps the line level, a 0 toggles it. Starts from ``level``
    (the idle J before a SYNC)."""
    out: List[str] = []
    for bit in bits:
        if not bit:
            level = K if level == J else J
        out.append(level)
    return out


def nrzi_decode(symbols: Sequence[str], level: str = J) -> List[int]:
    """Inverse of :func:`nrzi`; only J and K are data symbols."""
    out: List[int] = []
    for sym in symbols:
        if sym not in (J, K):
            raise UsbError("line state %r inside a packet" % (sym,))
        out.append(1 if sym == level else 0)
        level = sym
    return out


def packet_symbols(packet: Sequence[int], eop: bool = True) -> List[str]:
    """The line states of a whole packet: SYNC, the stuffed and NRZI-coded
    bytes of ``packet`` (PID first), then SE0, SE0, J."""
    symbols = nrzi(stuff(list(SYNC_BITS) + bytes_bits(packet)))
    if eop:
        symbols += [SE0] * EOP_SE0_BITS + [J]
    return symbols


@dataclasses.dataclass
class Packet:
    """A decoded packet."""

    pid: int
    payload: bytes = b""              # bytes after the PID, CRC included
    addr: Optional[int] = None        # tokens
    ep: Optional[int] = None          # tokens
    crc_ok: bool = True
    pid_ok: bool = True

    @property
    def name(self) -> str:
        return PID_NAMES.get(self.pid, "PID%X" % self.pid)

    @property
    def data(self) -> bytes:
        """The payload of a data packet without its CRC."""
        return self.payload[:-2] if self.pid in DATAS else self.payload

    @property
    def ok(self) -> bool:
        return self.crc_ok and self.pid_ok


def decode_bytes(packet: Sequence[int]) -> Packet:
    """Decode a packet given as its bytes after SYNC (PID first)."""
    if not packet:
        raise UsbError("empty packet")
    byte = packet[0]
    pid = byte & 0xF
    rest = bytes(packet[1:])
    p = Packet(pid=pid, payload=rest, pid_ok=pid_ok(byte))
    if pid in (OUT, IN, SETUP):
        if len(rest) != 2:
            raise UsbError("%s token with %d bytes after the PID" % (p.name, len(rest)))
        field = rest[0] | (rest[1] << 8)
        p.addr, p.ep = field & 0x7F, (field >> 7) & 0xF
        p.crc_ok = check_crc5(field)
    elif pid == SOF:
        if len(rest) != 2:
            raise UsbError("SOF with %d bytes after the PID" % len(rest))
        p.crc_ok = check_crc5(rest[0] | (rest[1] << 8))
    elif pid in DATAS:
        if len(rest) < 2:
            raise UsbError("data packet with %d bytes after the PID" % len(rest))
        p.crc_ok = check_crc16(rest)
    elif rest:
        raise UsbError("%s with %d bytes after the PID" % (p.name, len(rest)))
    return p


def decode_symbols(symbols: Sequence[str]) -> Packet:
    """Decode the J/K states of one packet (SYNC included, EOP excluded)."""
    bits = unstuff(nrzi_decode(symbols), final=True)
    if tuple(bits[:8]) != SYNC_BITS:
        raise UsbError("no SYNC: first bits %s" % "".join(map(str, bits[:8])))
    body = bits[8:]
    if len(body) % 8:
        raise UsbError("packet of %d bits is not whole bytes" % len(body))
    return decode_bytes(bits_bytes(body))


# ------------------------------------------------------------- the bus
#: Longest packet the host waits for before it gives up on an EOP, in bits
#: (a low-speed data packet is at most 8 + 8 + 64 + 16 bits plus stuffing).
MAX_PACKET_BITS = 160

_LEVELS = {J: (0, 1), K: (1, 0), SE0: (0, 0), SE1: (1, 1)}
_STATE = {(0, 0): SE0, (1, 0): K, (0, 1): J, (1, 1): SE1}


@dataclasses.dataclass
class Received:
    """A packet the host saw on the bus, with its timing in core cycles.

    ``edges`` holds every change of the line state from the first K of the
    SYNC to the J that ends the EOP; ``symbols`` the J/K state of each bit
    (SYNC included), counted at the nominal bit time. A segment shorter than
    half a bit is not a bit: it is kept in ``glitches``.
    """

    packet: Optional[Packet]
    error: Optional[str]
    start: int
    eop_start: int
    eop_end: int
    edges: List[Tuple[int, str]]
    symbols: List[str]
    eop_bits: int
    glitches: List[Tuple[int, str, int]]
    #: ``(cycle, bit index)`` of every J/K transition of the packet, the
    #: SYNC's first K (bit 0) included and the EOP excluded
    bit_edges: List[Tuple[int, int]] = dataclasses.field(default_factory=list)


@dataclasses.dataclass
class Transaction:
    """One host transaction and what the device did in it.

    ``kind`` is ``"setup"``, ``"out"``, ``"in"`` or ``"raw"``. After it has
    run, ``reply`` is the device's packet (``None`` if it stayed silent until
    the timeout, ``timeout`` then set), ``response_bits`` its delay from the
    SE0-to-J transition that ended the host's last packet to the device's
    first K, in bit times at the nominal rate.
    """

    kind: str
    addr: int = 0
    ep: int = 0
    pid: int = 0                        # the data PID the host sends (setup/out)
    payload: bytes = b""
    ack: bool = True                    # IN: acknowledge good data
    packets: Sequence[bytes] = ()       # raw: what to send
    expect_reply: bool = True           # raw: wait for a reply
    sent: List[Tuple[int, int, bytes]] = dataclasses.field(default_factory=list)
    reply: Optional[Received] = None
    response_bits: Optional[float] = None
    timeout: bool = False
    acked: bool = False
    done: bool = False

    @property
    def reply_pid(self) -> Optional[int]:
        if self.reply is None or self.reply.packet is None:
            return None
        return self.reply.packet.pid

    @property
    def data(self) -> Optional[bytes]:
        """The payload of the device's data packet (IN), CRC removed."""
        if self.reply is None or self.reply.packet is None:
            return None
        return self.reply.packet.data

    def describe(self) -> str:
        if self.timeout:
            what = "no reply"
        elif self.reply is None:
            what = "not run"
        elif self.reply.packet is None:
            what = "bad reply (%s)" % self.reply.error
        else:
            p = self.reply.packet
            what = "%s%s%s" % (p.name, " " + p.data.hex() if p.pid in DATAS else "",
                               "" if p.ok else " (bad CRC or PID)")
        return "%s addr %d ep %d: %s" % (self.kind.upper(), self.addr, self.ep, what)


class UsbHost(Model):
    """A low-speed USB host on the bus (7.1): drives D+ and D- push-pull while
    it transmits, releases them otherwise, and runs queued transactions in
    order.

    Args:
        dp, dm: the two lines (pin names, indices or raw ``uio`` pads).
        bit: the host's bit time in core clocks (nominal 33.33 at 50 MHz;
            scale it to transmit off-rate).
        gap: idle bit times between the host's own packets and between
            transactions (at least 2, 7.1.18.1).
        ack_gap: bit times from the end of the device's data packet to the
            host's ACK (2 to 6.5).
        timeout: bit times the host waits for a reply (16 to 18, 7.1.19.1).

    :attr:`trace` records every change of the line state as ``(cycle,
    state)``, :attr:`oe_trace` every change of the chip's output enables on
    the two pads as ``(cycle, bits)`` (bit 0 D+, bit 1 D-), and
    :attr:`collisions` every cycle in which the host drove while the chip had
    either pad enabled.
    """

    def __init__(self, dp: PinSpec = "BIDIR0", dm: PinSpec = "BIDIR1", *,
                 bit: float = CLOCKS_PER_BIT, gap: float = 4.0, ack_gap: float = 3.0,
                 timeout: float = TIMEOUT_MAX) -> None:
        self.dp, self.dm = pad_of(dp), pad_of(dm)
        self.bit = float(bit)
        self.gap = float(gap)
        self.ack_gap = float(ack_gap)
        self.timeout = float(timeout)
        self.cycle = -1
        self.state: Optional[str] = None
        self.trace: List[Tuple[int, str]] = []
        self.oe_trace: List[Tuple[int, int]] = []
        self.collisions: List[int] = []
        self.log: List[Transaction] = []
        self.last_eop_end: Optional[int] = None     # the host's last SE0 -> J
        self._oe = 0
        self._out: Optional[str] = None
        self._queue: Deque[Callable[[], Generator]] = deque()
        self._busy = False
        self._bus_free = 0
        self._last_se0_j = -1
        self.failure: Optional[str] = None
        self._proc = self._main()

    # ------------------------------------------------------------ queueing
    @property
    def idle(self) -> bool:
        """Nothing queued and nothing running."""
        return not self._queue and not self._busy

    def _add(self, t: Transaction, body) -> Transaction:
        self._queue.append(lambda: body(t))
        self.log.append(t)
        return t

    def setup(self, addr: int, ep: int, payload: Sequence[int]) -> Transaction:
        """SETUP token, DATA0 with the 8-byte request, handshake."""
        return self._add(Transaction("setup", addr, ep, DATA0, bytes(payload)), self._setup)

    def out(self, addr: int, ep: int, payload: Sequence[int] = b"",
            pid: int = DATA1) -> Transaction:
        """OUT token, a data packet, handshake."""
        return self._add(Transaction("out", addr, ep, pid, bytes(payload)), self._out_tr)

    def in_(self, addr: int, ep: int, ack: bool = True) -> Transaction:
        """IN token, then the device's data or handshake; good data is ACKed
        unless ``ack`` is false."""
        return self._add(Transaction("in", addr, ep, ack=ack), self._in)

    def raw(self, packets: Sequence[bytes], expect_reply: bool = True) -> Transaction:
        """Send packets (bytes after SYNC) back to back, ``gap`` apart, then
        wait for a reply (or for the timeout, to prove there is none)."""
        return self._add(Transaction("raw", packets=[bytes(p) for p in packets],
                                     expect_reply=expect_reply), self._raw)

    def raw_symbols(self, symbols: Sequence[str]) -> Transaction:
        """Drive arbitrary line states, one per bit time (a keep-alive EOP is
        ``[SE0, SE0, J]``), and wait for no reply."""
        t = Transaction("raw", expect_reply=False)
        t.packets = [tuple(symbols)]
        return self._add(t, self._raw_symbols)

    def pause(self, bits: float) -> None:
        """Keep the bus idle for ``bits`` bit times before the next job."""
        self._queue.append(lambda: self._pause(bits))

    # ----------------------------------------------------------- pin model
    def drive(self, drive: Drive, cycle: int) -> None:
        if self._out is not None:
            dp, dm = _LEVELS[self._out]
            drive.set(self.dp, dp)
            drive.set(self.dm, dm)

    def observe(self, lines: Lines) -> None:
        self.cycle = cycle = lines.cycle
        state = _STATE[(lines.get(self.dp), lines.get(self.dm))]
        if state != self.state:
            if self.state == SE0 and state == J:
                self._last_se0_j = cycle
            self.trace.append((cycle, state))
            self.state = state
        oe = ((lines.uio_oe >> self.dp[1]) & 1) | (((lines.uio_oe >> self.dm[1]) & 1) << 1)
        if oe != self._oe:
            self.oe_trace.append((cycle, oe))
            self._oe = oe
        if self._out is not None and oe:
            self.collisions.append(cycle)
        if self.failure is not None:
            return
        try:
            next(self._proc)
        except Exception as exc:          # noqa: BLE001 - kept, then raised
            # Never let StopIteration or anything else escape from here as
            # is: the RTL bench calls observe() inside a coroutine, where a
            # StopIteration ends the simulator with no traceback. Record it,
            # stop the process, and fail the bench with the reason.
            import traceback
            self.failure = "".join(traceback.format_exception(exc))
            self._out = None
            raise AssertionError("USB host model failed at cycle %d:\n%s"
                                 % (cycle, self.failure)) from None

    # ---------------------------------------------------------- processes
    def _main(self) -> Generator:
        while True:
            if not self._queue:
                yield
                continue
            job = self._queue.popleft()
            self._busy = True
            yield from self._wait_until(self._bus_free)
            yield from job()
            self._busy = False

    def _wait_until(self, cycle: float) -> Generator:
        """Return when the next cycle is ``cycle`` (or later)."""
        while self.cycle + 1 < cycle:
            yield

    def _pause(self, bits: float) -> Generator:
        yield from self._wait_until(self.cycle + 1 + bits * self.bit)
        self._bus_free = self.cycle + 1

    def _send_symbols(self, symbols: Sequence[str]) -> Generator:
        """Drive ``symbols`` one bit time each from the next cycle on, then
        release. Returns ``(first cycle, cycle of the last symbol)``."""
        start = self.cycle + 1
        edges = [start + int(i * self.bit + 0.5) for i in range(len(symbols) + 1)]
        for i, sym in enumerate(symbols):
            self._out = sym
            while self.cycle + 1 < edges[i + 1]:
                yield
        self._out = None
        return start, edges[len(symbols) - 1]

    def _send(self, packet: bytes, t: Transaction) -> Generator:
        start, end = yield from self._send_symbols(packet_symbols(packet))
        self.last_eop_end = end
        self._bus_free = end + int(self.gap * self.bit + 0.5)
        t.sent.append((start, end, bytes(packet)))
        return end

    def _receive(self, since: int, t: Transaction) -> Generator:
        """Wait for the device's packet, starting with a K no later than
        ``timeout`` bit times after ``since``; returns a :class:`Received`
        or ``None``."""
        deadline = since + self.timeout * self.bit
        while self.state != K:
            if self.cycle >= deadline:
                t.timeout = True
                self._bus_free = self.cycle + 1
                return None
            yield
        first = len(self.trace) - 1
        start = self.trace[first][0]
        limit = start + MAX_PACKET_BITS * self.bit
        while self._last_se0_j <= start:
            if self.cycle >= limit:
                self._bus_free = self.cycle + 1
                return Received(None, "no EOP within %d bits" % MAX_PACKET_BITS, start,
                                -1, -1, list(self.trace[first:]), [], 0, [])
            yield
        last = max(i for i, (c, s) in enumerate(self.trace) if c == self._last_se0_j)
        edges = list(self.trace[first:last + 1])
        rec = self._decode(edges)
        t.reply = rec
        t.response_bits = (start - since) / self.bit
        self._bus_free = rec.eop_end + int(self.gap * self.bit + 0.5)
        return rec

    def _decode(self, edges: List[Tuple[int, str]]) -> Received:
        symbols: List[str] = []
        glitches = []
        bit_edges = []
        eop_start = edges[-1][0]
        for (c0, s), (c1, _) in zip(edges, edges[1:]):
            n = int((c1 - c0) / self.bit + 0.5)
            if n == 0:
                glitches.append((c0, s, c1 - c0))
                continue
            if s in (J, K) and (not symbols or symbols[-1] != s):
                bit_edges.append((c0, len(symbols)))
            symbols += [s] * n
        eop_bits = 0
        while symbols and symbols[-1] == SE0:
            symbols.pop()
            eop_bits += 1
        for c, s in reversed(edges[:-1]):
            if s == SE0:
                eop_start = c
            else:
                break
        packet, error = None, None
        try:
            packet = decode_symbols(symbols)
        except UsbError as exc:
            error = str(exc)
        return Received(packet, error, edges[0][0], eop_start, edges[-1][0], edges,
                        symbols, eop_bits, glitches, bit_edges)

    # ------------------------------------------------------- transactions
    def _setup(self, t: Transaction) -> Generator:
        end = yield from self._send(token_bytes(SETUP, t.addr, t.ep), t)
        yield from self._wait_until(end + self.gap * self.bit)
        end = yield from self._send(data_bytes(DATA0, t.payload), t)
        yield from self._receive(end, t)
        t.done = True

    def _out_tr(self, t: Transaction) -> Generator:
        end = yield from self._send(token_bytes(OUT, t.addr, t.ep), t)
        yield from self._wait_until(end + self.gap * self.bit)
        end = yield from self._send(data_bytes(t.pid, t.payload), t)
        yield from self._receive(end, t)
        t.done = True

    def _in(self, t: Transaction) -> Generator:
        end = yield from self._send(token_bytes(IN, t.addr, t.ep), t)
        rec = yield from self._receive(end, t)
        if (t.ack and rec is not None and rec.packet is not None
                and rec.packet.pid in DATAS and rec.packet.ok):
            yield from self._wait_until(rec.eop_end + self.ack_gap * self.bit)
            yield from self._send(handshake_bytes(ACK), t)
            t.acked = True
        elif rec is not None and rec.packet is not None and rec.packet.pid in DATAS:
            # Data left without a handshake: the device waits up to
            # TIMEOUT_MAX bit times for one (7.1.19.1); the bus is not the
            # host's again until then.
            self._bus_free = max(self._bus_free, rec.eop_end
                                 + int((TIMEOUT_MAX + self.gap) * self.bit + 0.5))
        t.done = True

    def _raw(self, t: Transaction) -> Generator:
        end = None
        for i, packet in enumerate(t.packets):
            if i:
                yield from self._wait_until(end + self.gap * self.bit)
            end = yield from self._send(packet, t)
        if end is not None:
            if t.expect_reply:
                yield from self._receive(end, t)
            else:
                yield from self._silence(end, t)
        t.done = True

    def _raw_symbols(self, t: Transaction) -> Generator:
        start, end = yield from self._send_symbols(t.packets[0])
        t.sent.append((start, end, b""))
        yield from self._silence(end, t)
        t.done = True

    def _silence(self, since: int, t: Transaction) -> Generator:
        """Wait ``timeout`` bit times; a K in that time is recorded as a
        reply the device should not have sent."""
        deadline = since + self.timeout * self.bit
        while self.cycle < deadline:
            if self.state == K:
                yield from self._receive(since, t)
                t.done = True
                return
            yield
        t.timeout = True
        self._bus_free = self.cycle + 1


def run(bench, host: UsbHost, max_cycles: int, every: int = 32) -> None:
    """Step ``bench`` until ``host`` has run everything queued."""
    if not bench.run_until(lambda: host.idle, max_cycles, every=every):
        pending = [t.describe() for t in host.log if not t.done]
        raise AssertionError("USB host still busy after %d clocks: %s"
                             % (max_cycles, "; ".join(pending) or "(queue)"))
