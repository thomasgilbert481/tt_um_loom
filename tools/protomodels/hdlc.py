"""HDLC reference models for firmware/hdlc_loopback.loom.

Written from ISO/IEC 13239 (HDLC frame structure, transparency, abort) and
RFC 1662 (the FCS-16 algorithm and its check values), never from the
firmware or the engine:

* a frame is the flag 0x7E, the address octet, the control octet, 0 or more
  information octets, the 16-bit FCS, and a closing flag; every octet is
  sent least significant bit first;
* FCS-16: generator x^16 + x^12 + x^5 + 1, register preset to all ones,
  computed over address, control and information in transmission order,
  sent as the ones complement, low-order octet first (RFC 1662 section C.2,
  ``pppfcs16`` with the reflected constant 0x8408); a receiver that runs the
  same register over the FCS too holds the good-frame residue 0xF0B8;
* transparency (zero-bit insertion): between the flags, a 0 is inserted
  after every five contiguous 1s, the FCS included; the receiver deletes a 0
  that follows five contiguous 1s. Six 1s and a 0 are a flag;
* abort: seven or more contiguous 1s end a frame without a closing flag; a
  receiver discards it;
* a frame shorter than 32 bits between its flags is not a frame (ISO/IEC
  13239 has receivers ignore it), and one whose length between the flags is
  not a whole number of octets is invalid;
* line coding: NRZI, the convention of SDLC (a 0 is a transition of the line,
  a 1 keeps the level).

The bit-level functions (:func:`fcs16`, :func:`stuff`, :func:`frame_bits`,
:func:`nrzi`, :func:`deframe`) build and take apart the bit stream, with
error injection: a flipped bit, an abort in place of the FCS, and a missing
stuff bit (a stuffing violation). :class:`HdlcNode` sends such streams with a
clock of its own (a rate offset settable), and :class:`HdlcLine` is the bench
model between the chip's TX and RX pins: a wire (the loopback) or the node's
line, recording every edge of both so :func:`recover` can turn the chip's TX
back into bits independently of the chip's receiver.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

from tools.protomodels.bench import Drive, Lines, Model, PinSpec, pad_of
from tools.protomodels.manchester import EdgeLog

CLOCK = 50_000_000          # the chip's clock in the tests
FLAG = 0x7E
GOOD_RESIDUE = 0xF0B8       # RFC 1662 PPPGOODFCS16: the register after a good frame
MIN_BITS = 32               # fewer bits between two flags: not a frame

#: FCS-16 check value: the transmitted FCS of the ASCII octets "123456789" is
#: 0x906E (CRC-16/X-25, the RFC 1662 algorithm), sent as 0x6E then 0x90.
CHECK_INPUT, CHECK_FCS = b"123456789", 0x906E


# ------------------------------------------------------------------- FCS
def fcs16(data: Sequence[int], fcs: int = 0xFFFF) -> int:
    """RFC 1662 ``pppfcs16``: the register after ``data`` (not complemented)."""
    for byte in data:
        fcs ^= byte & 0xFF
        for _ in range(8):
            fcs = (fcs >> 1) ^ 0x8408 if fcs & 1 else fcs >> 1
    return fcs


def fcs_octets(data: Sequence[int]) -> bytes:
    """The two FCS octets as transmitted: the complement, low octet first."""
    fcs = fcs16(data) ^ 0xFFFF
    return bytes([fcs & 0xFF, fcs >> 8])


def frame_ok(octets: Sequence[int]) -> bool:
    """A received frame, FCS included, leaves the good residue (RFC 1662)."""
    return fcs16(octets) == GOOD_RESIDUE


# ------------------------------------------------------------------ bits
def octet_bits(data: Sequence[int]) -> List[int]:
    """Octets as bits in transmission order, each least significant first."""
    return [(byte >> i) & 1 for byte in data for i in range(8)]


def bits_octets(bits: Sequence[int]) -> bytes:
    """Inverse of :func:`octet_bits` for a whole number of octets."""
    assert len(bits) % 8 == 0, len(bits)
    return bytes(sum(b << i for i, b in enumerate(bits[k:k + 8]))
                 for k in range(0, len(bits), 8))


FLAG_BITS = octet_bits([FLAG])          # 0 1 1 1 1 1 1 0


def stuff(bits: Sequence[int], *, skip: Optional[int] = None) -> Tuple[List[int], List[int]]:
    """Zero-bit insertion: a 0 after every five contiguous 1s. Returns the
    stuffed bits and the index in them of every inserted 0. ``skip`` leaves
    out the stuff bit with that number (0 = the first one due): a stuffing
    violation; the count of 1s starts again there as if it had been sent."""
    out: List[int] = []
    where: List[int] = []
    ones = due = 0
    for b in bits:
        out.append(b)
        ones = ones + 1 if b else 0
        if ones == 5:
            ones = 0
            if due != skip:
                where.append(len(out))
                out.append(0)
            due += 1
    return out, where


def frame_bits(address: int, control: int, info: Sequence[int] = b"", *,
               abort: bool = False, flip: Optional[int] = None,
               skip_stuff: Optional[int] = None) -> Tuple[List[int], List[int]]:
    """The bits a transmitter sends between the opening flag and the closing
    flag (or the abort), stuffed, and the index of every stuff bit in them.

    ``abort`` leaves out the FCS: the caller follows the bits with seven or
    more 1s instead of a flag (:func:`frame_stream`). ``flip`` inverts the bit
    at that index after stuffing (a line error); ``skip_stuff`` leaves out the
    stuff bit with that number (:func:`stuff`)."""
    octets = bytes([address & 0xFF, control & 0xFF]) + bytes(info)
    body = octet_bits(octets)
    if not abort:
        body += octet_bits(fcs_octets(octets))
    bits, where = stuff(body, skip=skip_stuff)
    if flip is not None:
        bits[flip] ^= 1
    return bits, where


def frame_stream(address: int, control: int, info: Sequence[int] = b"", *,
                 abort: bool = False, abort_ones: int = 8, **errors) -> List[int]:
    """A whole frame on the line: opening flag, :func:`frame_bits`, and the
    closing flag, or ``abort_ones`` 1s for an aborted frame."""
    bits, _ = frame_bits(address, control, info, abort=abort, **errors)
    return FLAG_BITS + bits + ([1] * abort_ones if abort else FLAG_BITS)


def nrzi(bits: Sequence[int], level: int = 0) -> List[int]:
    """NRZI line levels for ``bits`` from ``level``: a 0 toggles, a 1 keeps."""
    out: List[int] = []
    for b in bits:
        if not b:
            level ^= 1
        out.append(level)
    return out


def un_nrzi(levels: Sequence[int], level: int = 0) -> List[int]:
    """Inverse of :func:`nrzi`: 1 where the level did not change."""
    out: List[int] = []
    for lv in levels:
        out.append(1 if lv == level else 0)
        level = lv
    return out


# --------------------------------------------------------------- deframe
@dataclass
class Frame:
    """One frame as a receiver takes it from the bit stream: the bits between
    the opening flag and the closing flag with the stuff bits deleted, or
    those received before an abort. ``start`` is the index in the stream of
    its first bit after the opening flag, ``end`` of the bit that closed it."""

    bits: List[int] = field(default_factory=list)
    aborted: bool = False
    start: int = 0
    end: int = 0

    @property
    def whole(self) -> bool:
        """A whole number of octets between the flags."""
        return len(self.bits) % 8 == 0

    @property
    def octets(self) -> bytes:
        """The octets, FCS included (whole octets only)."""
        return bits_octets(self.bits[:len(self.bits) - len(self.bits) % 8])

    @property
    def good(self) -> bool:
        """Not aborted, whole octets, 32 bits or more, and the FCS right."""
        return (not self.aborted and self.whole and len(self.bits) >= MIN_BITS
                and frame_ok(self.octets))

    @property
    def data(self) -> bytes:
        """Address, control and information: the octets before the FCS."""
        return self.octets[:-2]


def deframe(stream: Sequence[int]) -> List[Frame]:
    """Every frame in a bit stream (NRZI already undone), as ISO/IEC 13239's
    receiver finds them: hunt for a flag; then delete each 0 that follows five
    1s, until six 1s and a 0 (a flag: the frame ends, and the flag may open
    the next) or seven 1s (an abort: the frame is recorded as aborted and the
    receiver hunts again). Fewer than :data:`MIN_BITS` bits between two flags
    (idle flags) is no frame and is not recorded."""
    frames: List[Frame] = []
    cur: Optional[Frame] = None
    ones = 0
    for i, b in enumerate(stream):
        if b:
            ones += 1
            if ones == 7 and cur is not None:
                cur.bits = cur.bits[:len(cur.bits) - 5]    # the abort's 1s
                cur.aborted, cur.end = True, i
                frames.append(cur)
                cur = None
            elif ones <= 5 and cur is not None:
                cur.bits.append(1)
            continue
        if ones == 6:                                       # a flag
            if cur is not None:
                cur.bits = cur.bits[:len(cur.bits) - 6]     # its 0 and five 1s
                cur.end = i
                if len(cur.bits) >= MIN_BITS:
                    frames.append(cur)
            cur = Frame(start=i + 1)
        elif ones != 5 and cur is not None:                 # (5: a stuff bit)
            cur.bits.append(0)
        ones = 0
    return frames


# --------------------------------------------------------------------- node
@dataclass
class Sent:
    """A stream the node was asked to send, and when it went out."""

    bits: List[int]
    start: Optional[int] = None         # first cycle of its first bit
    done: bool = False                  # its last bit has gone out


class HdlcNode:
    """An HDLC sender with its own bit clock, NRZI on the line.

    Args:
        rate: the nominal bit rate.
        offset: the node's rate is ``rate * (1 + offset)`` (positive: short
            bits). Bit ``i`` starts ``ceil(i * period)`` clocks after reset,
            so every edge falls on a whole clock.
        idle: what the node sends between streams, ``"flags"`` (0x7E back to
            back) or ``"marks"`` (1s, the line still).
    A queued stream (:meth:`send`) starts at the next idle octet boundary.
    """

    def __init__(self, rate: float, *, offset: float = 0.0, clock: int = CLOCK,
                 idle: str = "flags") -> None:
        assert idle in ("flags", "marks")
        self.period = clock / (rate * (1.0 + offset))
        self.idle = FLAG_BITS if idle == "flags" else [1] * 8
        self.sent: List[Sent] = []
        self._queue: List[Sent] = []
        self._tape: List[Tuple[int, Optional[Sent], bool, bool]] = []
        self._i = -1                    # the last bit index put on the line
        self._level = 0

    def send(self, bits: Sequence[int]) -> Sent:
        s = Sent(list(bits))
        self._queue.append(s)
        self.sent.append(s)
        return s

    @property
    def busy(self) -> bool:
        return bool(self._queue) or any(t[1] is not None for t in self._tape)

    def _refill(self) -> None:
        """The next stream if one is queued, else one idle octet."""
        if self._queue:
            s = self._queue.pop(0)
            n = len(s.bits)
            self._tape = [(b, s, k == 0, k == n - 1) for k, b in enumerate(s.bits)]
        else:
            self._tape = [(b, None, False, False) for b in self.idle]

    def level(self, cycle: int) -> int:
        """The line level in ``cycle`` (cycles must not go back)."""
        while (self._i + 1) * self.period <= cycle:
            self._i += 1
            if not self._tape:
                self._refill()
            b, s, first, last = self._tape.pop(0)
            if first:
                s.start = cycle
            if not b:
                self._level ^= 1
            if last:
                s.done = True
        return self._level


# ---------------------------------------------------------------------- line
class HdlcLine(Model):
    """What the chip's RX pin sees, and a record of its TX pin.

    Without a node the line is a wire: RX gets the level TX had in the cycle
    before (the loop delay; ``drive`` may only use registered state). With
    ``node`` RX sees the node's line instead and TX goes nowhere. ``tx_log``
    and ``rx_log`` hold every edge of each, for :func:`recover`."""

    def __init__(self, node: Optional[HdlcNode] = None,
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


# ------------------------------------------------------------------ recover
def recover(log: EdgeLog, period: float, since: int = 0,
            until: Optional[int] = None) -> Tuple[List[int], List[int], int]:
    """The bits on an NRZI line from its edges, the way a receiver with an
    ideal clock of ``period`` clocks per bit recovers them: the first edge at
    or after ``since`` starts bit 0 (a 0: every edge is one), and each later
    edge is ``round(gap / period)`` bits after the one before, the bits
    between them 1s. Returns the bits up to ``until`` (default: the last edge),
    the offset in clocks of every edge from the grid of ``period`` anchored at
    the first edge, and the cycle of the first edge."""
    edges = [c for c, _ in log.edges_in(since, until if until is not None else 1 << 62)]
    if not edges:
        return [], [], -1
    t0 = edges[0]
    bits: List[int] = [0]
    offsets: List[int] = [0]
    prev = 0
    for c in edges[1:]:
        k = int(round((c - t0) / period))
        bits += [1] * (k - prev - 1) + [0]
        offsets.append(int(round(c - t0 - k * period)))
        prev = k
    if until is not None:
        bits += [1] * max(0, int((until - t0) / period) - prev)
    return bits, offsets, t0
