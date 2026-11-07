"""RDM reference model (ANSI E1.20, Remote Device Management over DMX512):
a controller on the RS-485 line, a packet builder and parser, the
discovery encoding and search, and a timing checker for the responder.

Written from the standard's figures and packet layout, not from the
firmware, for ``firmware/rdm_responder.loom`` (D-035).

Sources. ANSI E1.20 itself could not be read: ESTA's TSP refused the
download (HTTP 403, 2026-11-07). Every figure below comes from a
secondary source that quotes it, named per figure; section numbers of
E1.20 given without a source are from memory and unchecked.

* OLE: the Open Lighting Project's embedded-firmware documentation,
  docs.openlighting.org/ole, ``transceiver_timing.h`` and ``constants.h``,
  which quote E1.20-2010's tables by line.
* OLA: the Open Lighting Architecture source, ``include/ola/rdm/RDMEnums.h``
  (command classes, PIDs, NACK reasons, DEVICE_INFO values),
  ``common/rdm/RDMCommand.cpp`` (field order, message length, checksum) and
  ``common/rdm/DiscoveryAgent.cpp`` (the DUB response and the search).

Timing (microseconds):

==========================================  ==============  =====================
figure                                      value           source
==========================================  ==============  =====================
packet sent (break)                         176 .. 352      OLE: Table 3-1
packet sent (mark after break)              12 .. 88        OLE: Table 3-1
controller receives: break                  88 .. 352       OLE: Table 3-1 line 2
controller receives: mark after break       <= 88           OLE: Table 3-1 line 2
controller receives: inter-slot time        <= 2100         OLE: Table 3-1 line 2
responder receives: break                   >= 88           OLE: Table 3-3 line 1
responder receives: mark after break        >= 8            OLE: Table 3-3 line 1
responder receives: inter-slot time         <= 2100         OLE: Table 3-3 line 1
DUB response limit                          2900            OLE: Table 3-3 line 3
responder delay (turnaround)                176 .. 2000     OLE: Table 3-4
controller response timeout                 2800            OLE: Table 3-2 lines 1, 3
controller back-off after a DUB             5800            OLE: Table 3-2 line 2
controller back-off, missing response       3000            OLE: Table 3-2 line 5
controller back-off after a broadcast       176             OLE: Table 3-2 line 6
==========================================  ==============  =====================

Where each interval is measured from and to is not in those sources. This
model takes the responder delay from the end of the request's last stop
bit to the response's first falling edge (its break, or a DUB response's
first start bit), and the DUB response limit from the same end to the end
of the response's last stop bit: unchecked readings. E1.20 3.2.3 sets a
"discovery response driver enable time"; its figure was not found, so the
DE check here (DE rises after the request has ended, at least one bit
before the first edge, and falls within one bit of the response's last
stop bit) is this model's own, marked unchecked.

The packet (OLA RDMCommand.cpp; E1.20 6.2, section number unchecked):
START_CODE 0xCC, SUB_START_CODE 0x01, MESSAGE_LENGTH (from the start code
to the end of the parameter data: 24 + PDL), destination UID (6), source
UID (6), transaction number, port ID (a response type in responses),
message count, sub-device (2), command class (offset 20: OLA's
COMMAND_CLASS_OFFSET 19 counts from the sub-start code), parameter ID (2),
PDL, the parameter data, then the checksum: the 16-bit sum of every byte
from the start code to the end of the parameter data, high byte first.
The start code and sub-start code values and the response types (ACK 0,
ACK_TIMER 1, NACK_REASON 2, ACK_OVERFLOW 3: OLA has only the last) are
from memory, unchecked.

DISC_UNIQUE_BRANCH response (OLA DiscoveryAgent.cpp): up to seven
preamble bytes 0xFE, the separator 0xAA, the encoded UID (twelve bytes,
two per UID byte, most significant first; each decodes as the AND of its
pair) and the encoded checksum (four bytes, the 16-bit sum of the twelve,
high byte first), sent with no break. That the first byte of each pair is
``b | 0xAA`` and the second ``b | 0x55`` is from memory, unchecked; the
AND decoding OLA uses works either way round.
"""

from __future__ import annotations

import dataclasses
from collections import deque
from typing import Callable, List, Optional, Sequence, Tuple

from .bench import Drive, Lines, Model, PinSpec, pad_of

CLOCK = 50e6                        # the datasheet's clock, Hz
US = CLOCK / 1e6                    # clocks per microsecond
BIT_US = 4.0                        # 250 kbit/s (E1.11, as DMX512)
SLOT_BITS = 11                      # 8N2

START_CODE, SUB_START_CODE = 0xCC, 0x01
DISCOVERY, DISCOVERY_RESPONSE = 0x10, 0x11
GET, GET_RESPONSE, SET, SET_RESPONSE = 0x20, 0x21, 0x30, 0x31
ACK, ACK_TIMER, NACK_REASON, ACK_OVERFLOW = 0, 1, 2, 3
DISC_UNIQUE_BRANCH, DISC_MUTE, DISC_UN_MUTE = 0x0001, 0x0002, 0x0003
DEVICE_INFO, IDENTIFY_DEVICE = 0x0060, 0x1000
NR_UNKNOWN_PID, NR_FORMAT_ERROR = 0x0000, 0x0001
NR_UNSUPPORTED_COMMAND_CLASS, NR_DATA_OUT_OF_RANGE = 0x0005, 0x0006
NR_SUB_DEVICE_OUT_OF_RANGE = 0x0009
BROADCAST = 0xFFFFFFFFFFFF          # all devices
MAX_UID = 0xFFFFFFFFFFFE            # the top of the discovery range

# Timing, microseconds (table above).
TX_BREAK, TX_MAB = (176.0, 352.0), (12.0, 88.0)
CTRL_RX_BREAK, CTRL_RX_MAB_MAX, INTERSLOT_MAX = (88.0, 352.0), 88.0, 2100.0
RESP_RX_BREAK_MIN, RESP_RX_MAB_MIN = 88.0, 8.0
DUB_LIMIT = 2900.0
RESPONDER_DELAY = (176.0, 2000.0)
RESPONSE_TIMEOUT = 2800.0
DUB_BACKOFF, MISSING_BACKOFF, BROADCAST_BACKOFF = 5800.0, 3000.0, 176.0


# ----------------------------------------------------------------- packets
def uid_bytes(uid: int) -> List[int]:
    """A 48-bit UID as six bytes, most significant first."""
    return [(uid >> (8 * (5 - i))) & 0xFF for i in range(6)]


def uid_of(data: Sequence[int]) -> int:
    value = 0
    for b in data:
        value = (value << 8) | b
    return value


def checksum(data: Sequence[int]) -> int:
    """The 16-bit additive checksum of ``data``."""
    return sum(data) & 0xFFFF


@dataclasses.dataclass
class Packet:
    """One RDM message, a request or a response."""

    dst: int
    src: int
    tn: int                         # transaction number
    port: int                       # port ID, or the response type
    cc: int                         # command class
    pid: int
    pd: List[int] = dataclasses.field(default_factory=list)
    msg_count: int = 0
    sub_device: int = 0

    def encode(self, bad_checksum: bool = False) -> List[int]:
        body = ([START_CODE, SUB_START_CODE, 24 + len(self.pd)]
                + uid_bytes(self.dst) + uid_bytes(self.src)
                + [self.tn & 0xFF, self.port & 0xFF, self.msg_count & 0xFF,
                   (self.sub_device >> 8) & 0xFF, self.sub_device & 0xFF,
                   self.cc & 0xFF, (self.pid >> 8) & 0xFF, self.pid & 0xFF,
                   len(self.pd)] + [b & 0xFF for b in self.pd])
        cs = checksum(body) ^ (1 if bad_checksum else 0)
        return body + [cs >> 8, cs & 0xFF]


def parse(data: Sequence[int]) -> Optional[Packet]:
    """A packet from its bytes (start code first), or None if the start
    codes, the length or the checksum are wrong."""
    data = list(data)
    if len(data) < 26 or data[0] != START_CODE or data[1] != SUB_START_CODE:
        return None
    length = data[2]
    if length < 24 or len(data) != length + 2 or data[23] != length - 24:
        return None
    if checksum(data[:length]) != (data[length] << 8 | data[length + 1]):
        return None
    return Packet(dst=uid_of(data[3:9]), src=uid_of(data[9:15]), tn=data[15],
                  port=data[16], msg_count=data[17],
                  sub_device=data[18] << 8 | data[19], cc=data[20],
                  pid=data[21] << 8 | data[22], pd=data[24:length])


def request(cc: int, pid: int, pd: Sequence[int] = (), *, dst: int, src: int,
            tn: int = 0, port: int = 1, sub_device: int = 0) -> Packet:
    return Packet(dst=dst, src=src, tn=tn, port=port, cc=cc, pid=pid,
                  pd=list(pd), sub_device=sub_device)


def dub_request(lower: int, upper: int, *, src: int, tn: int = 0) -> Packet:
    """DISC_UNIQUE_BRANCH, always broadcast: the bounds as its data."""
    return request(DISCOVERY, DISC_UNIQUE_BRANCH,
                   uid_bytes(lower) + uid_bytes(upper), dst=BROADCAST,
                   src=src, tn=tn)


def dub_response(uid: int, preamble: int = 7) -> List[int]:
    """The bytes a responder sends for DISC_UNIQUE_BRANCH."""
    euid: List[int] = []
    for b in uid_bytes(uid):
        euid += [b | 0xAA, b | 0x55]
    cs = checksum(euid)
    ecs = [(cs >> 8) | 0xAA, (cs >> 8) | 0x55, (cs & 0xFF) | 0xAA,
           (cs & 0xFF) | 0x55]
    return [0xFE] * preamble + [0xAA] + euid + ecs


def decode_dub(data: Sequence[int]) -> Optional[int]:
    """The UID in a DUB response, or None: a collision or noise (a bad
    preamble, no separator, a wrong length or checksum)."""
    data = list(data)
    i = 0
    while i < len(data) and i < 7 and data[i] == 0xFE:
        i += 1
    if i >= len(data) or data[i] != 0xAA or len(data) != i + 17:
        return None
    euid, ecs = data[i + 1:i + 13], data[i + 13:i + 17]
    for k in range(0, 12, 2):
        if (euid[k] & 0xAA) != 0xAA or (euid[k + 1] & 0x55) != 0x55:
            return None             # not the b | 0xAA, b | 0x55 pattern
    uid = uid_of([euid[k] & euid[k + 1] for k in range(0, 12, 2)])
    cs = (ecs[0] & ecs[1]) << 8 | (ecs[2] & ecs[3])
    return uid if cs == checksum(euid) else None


# ---------------------------------------------------------------- the line
@dataclasses.dataclass
class Transmission:
    """One burst on the line, decoded from its edges."""

    first_fall: int                 # cycle of the first falling edge
    break_clocks: Optional[int]     # None: no break (a DUB response)
    mab_clocks: Optional[int]
    data: List[int]
    slot_starts: List[int]
    framing_errors: List[int]       # slot indices with a stop bit low
    end: int                        # end of the last stop bit (nominal)


def decode(edges: Sequence[Tuple[int, int]], t0: int = 0,
           bit_us: float = BIT_US) -> Optional[Transmission]:
    """Decode the first transmission after cycle ``t0`` from line edges
    ``(cycle, level)``: a low run of at least 88 us first is a break, the
    high run after it the mark after break, then each falling edge starts
    an 8N2 slot sampled at the middle of its bits."""
    bit = bit_us * US
    edges = [e for e in edges if e[0] >= t0]

    def level(t: float) -> int:
        v = 1
        for c, lv in edges:
            if c > t:
                break
            v = lv
        return v

    falls = [c for c, lv in edges if lv == 0]
    if not falls:
        return None
    first = falls[0]
    rise = next((c for c, lv in edges if c > first and lv == 1), None)
    if rise is None:
        return None
    brk = mab = None
    t = first
    if rise - first >= RESP_RX_BREAK_MIN * US:
        brk = rise - first
        nxt = next((c for c, lv in edges if c > rise and lv == 0), None)
        if nxt is None:
            return None
        mab, t = nxt - rise, nxt
    data, starts, ferrs = [], [], []
    while True:
        samples = [level(t + (k + 0.5) * bit) for k in range(SLOT_BITS)]
        data.append(sum(b << k for k, b in enumerate(samples[1:9])))
        starts.append(t)
        if not (samples[9] and samples[10]):
            ferrs.append(len(data) - 1)
        end = t + SLOT_BITS * bit
        nxt = next((c for c, lv in edges if c >= end - bit / 2 and lv == 0), None)
        if nxt is None or nxt - end > INTERSLOT_MAX * US:
            return Transmission(first, brk, mab, data, starts, ferrs, int(round(end)))
        t = nxt


class RdmLine(Model):
    """The RS-485 pair, a model controller on it, and the chip's driver.

    The chip's TX and DE (uo pads) drive the line when DE is 1; the model
    controller drives it while it sends; otherwise the bias resistors hold
    it at mark (1). The resolved line goes to the chip's RX, one clock late
    like every bench model, and every edge of it, every DE edge and every
    cycle both drive are recorded.
    """

    def __init__(self, rx: PinSpec = "IN0", tx: PinSpec = "OUT0",
                 de: PinSpec = "OUT1", *, bit_us: float = BIT_US) -> None:
        self.rx, self.tx, self.de = pad_of(rx), pad_of(tx), pad_of(de)
        self.bit = bit_us * US
        self.now = 0
        self.ctrl: Optional[int] = None         # the controller's level, None: off
        self._changes: deque = deque()          # (cycle, level or None)
        self.chip_tx, self.chip_de = 1, 0
        self.line = 1
        self.edges: List[Tuple[int, int]] = []
        self.de_edges: List[Tuple[int, int]] = []
        self.contention: List[int] = []

    def send(self, data: Sequence[int], *, brk_us: Optional[float] = 176.0,
             mab_us: float = 12.0, gap_us: float = 0.0,
             bad_stop: Sequence[int] = (), start: Optional[int] = None,
             bit_us: Optional[float] = None) -> Tuple[int, int]:
        """Queue one controller transmission; returns the cycles of its
        first edge and of the end of its last stop bit (where the
        controller releases the line). ``brk_us=None`` sends no break."""
        bit = (bit_us * US) if bit_us else self.bit
        t = float(max(self.now + 2, start or 0))
        t0 = t
        self._changes.append((int(round(t)), 1))         # the driver on, mark
        t += bit
        t0 = t
        if brk_us is not None:
            self._changes.append((int(round(t)), 0))
            t += brk_us * US
            self._changes.append((int(round(t)), 1))
            t += mab_us * US
        for i, value in enumerate(data):
            if i:
                t += gap_us * US
            bits = [0] + [(value >> k) & 1 for k in range(8)]
            bits += [0 if i in bad_stop else 1, 1]
            for k, lv in enumerate(bits):
                self._changes.append((int(round(t + k * bit)), lv))
            t += SLOT_BITS * bit
        end = int(round(t))
        self._changes.append((end, None))                # released
        return int(round(t0)), end

    @property
    def busy(self) -> bool:
        return bool(self._changes)

    def drive(self, drive: Drive, cycle: int) -> None:
        self.now = cycle
        while self._changes and self._changes[0][0] <= cycle:
            self.ctrl = self._changes.popleft()[1]
        if self.ctrl is not None and self.chip_de:
            self.contention.append(cycle)
        if self.ctrl is not None:
            self.line = self.ctrl
        else:
            self.line = self.chip_tx if self.chip_de else 1
        drive.set(self.rx, self.line)

    def observe(self, lines: Lines) -> None:
        v = lines.get(self.rx)
        if not self.edges or self.edges[-1][1] != v:
            if self.edges or v == 0:
                self.edges.append((lines.cycle, v))
        de = lines.get(self.de)
        if de != self.chip_de:
            self.de_edges.append((lines.cycle, de))
        self.chip_de = de
        self.chip_tx = lines.get(self.tx)


# ------------------------------------------------------------ the checker
def response_violations(req_end: int, resp: Transmission,
                        de_edges: Sequence[Tuple[int, int]], *,
                        bit_us: float = BIT_US, slack: float = 0.0) -> List[str]:
    """Every way a responder's ``resp`` to a request that ended at cycle
    ``req_end`` breaks the timing table (and this model's DE rule, marked
    unchecked in the module text). ``slack`` widens every window by that
    many clocks; ``de_edges`` are the chip's DE edges, ``(cycle, level)``."""
    bit = bit_us * US
    out: List[str] = []

    def window(name: str, clocks: Optional[float], lo: Optional[float],
               hi: Optional[float]) -> None:
        if clocks is None:
            out.append("%s missing" % name)
            return
        if lo is not None and clocks < lo * US - slack:
            out.append("%s %.2f us < %.2f us" % (name, clocks / US, lo))
        if hi is not None and clocks > hi * US + slack:
            out.append("%s %.2f us > %.2f us" % (name, clocks / US, hi))

    window("responder delay", resp.first_fall - req_end, *RESPONDER_DELAY)
    window("start after the request", resp.first_fall - req_end, None,
           RESPONSE_TIMEOUT)
    if resp.break_clocks is not None:
        window("break", resp.break_clocks, *TX_BREAK)
        window("mark after break", resp.mab_clocks, *TX_MAB)
    else:
        window("DUB response end", resp.end - req_end, None, DUB_LIMIT)
    for i in range(1, len(resp.slot_starts)):
        gap = resp.slot_starts[i] - resp.slot_starts[i - 1] - SLOT_BITS * bit
        window("inter-slot time before slot %d" % i, gap, None, INTERSLOT_MAX)
    for i in resp.framing_errors:
        out.append("slot %d: framing error" % i)
    rises = [c for c, v in de_edges if v == 1 and c > req_end - SLOT_BITS * bit]
    falls = [c for c, v in de_edges if v == 0 and c > resp.first_fall]
    if not rises or rises[0] > resp.first_fall - bit + slack:
        out.append("DE does not rise a bit before the response")
    elif rises[0] < req_end:
        out.append("DE rises %.2f us before the request ends"
                   % ((req_end - rises[0]) / US))
    if not falls:
        out.append("DE does not fall")
    elif not resp.end - slack <= falls[0] <= resp.end + bit + slack:
        out.append("DE falls %.2f us after the last stop bit"
                   % ((falls[0] - resp.end) / US))
    return out


# -------------------------------------------------------------- controller
@dataclasses.dataclass
class Exchange:
    """A request and what came back."""

    request: List[int]
    start: int                      # the request's first edge
    end: int                        # the end of its last stop bit
    response: Optional[Transmission]
    de_edges: List[Tuple[int, int]]


CONTROLLER_UID = 0x7A70_00000001    # an arbitrary controller UID


class Controller:
    """An RDM controller on an :class:`RdmLine`: sends a request, waits
    for the response the way Table 3-2 says, and decodes it.

    ``step(n)`` advances the bench ``n`` clocks. ``wait_us`` is how long it
    listens for a response to start: RESPONSE_TIMEOUT by default; a test
    may shorten it to just past RESPONDER_DELAY's maximum, after which a
    responder that has not started has broken Table 3-4 anyway.
    """

    def __init__(self, step: Callable[[int], None], line: RdmLine, *,
                 uid: int = CONTROLLER_UID, wait_us: float = RESPONSE_TIMEOUT) -> None:
        self.step, self.line, self.uid, self.wait_us = step, line, uid, wait_us
        self.tn = 0
        self.log: List[Exchange] = []

    def transact(self, packet, *, wait_us: Optional[float] = None,
                 **send_kw) -> Exchange:
        """Send ``packet`` (a :class:`Packet` or its bytes) and wait."""
        data = packet.encode() if isinstance(packet, Packet) else list(packet)
        line = self.line
        n_edges, n_de = len(line.edges), len(line.de_edges)
        start, end = line.send(data, **send_kw)
        self.step(end - line.now + 1)
        wait = int((self.wait_us if wait_us is None else wait_us) * US)
        while line.now - end < wait:
            if any(c > end for c, _ in line.edges[n_edges:]):
                break
            self.step(min(400, end + wait - line.now))
        resp = None
        if any(c > end for c, _ in line.edges[n_edges:]):
            # A response: listen until DE falls and the line is quiet.
            limit = end + int(DUB_LIMIT * US * 2)
            while line.now < limit and not any(
                    v == 0 and c > end for c, v in line.de_edges[n_de:]):
                self.step(400)
            self.step(int(2 * line.bit))
            resp = decode(line.edges[n_edges:], end)
        ex = Exchange(data, start, end, resp, list(line.de_edges[n_de:]))
        self.log.append(ex)
        return ex

    def next_tn(self) -> int:
        self.tn = (self.tn + 1) & 0xFF
        return self.tn

    def request(self, cc: int, pid: int, pd: Sequence[int] = (), *, dst: int,
                **kw) -> Packet:
        return request(cc, pid, pd, dst=dst, src=self.uid, tn=self.next_tn(), **kw)

    # --------------------------------------------------------- discovery
    def dub(self, lower: int, upper: int):
        """One DISC_UNIQUE_BRANCH: the UID found, None for silence, or
        "collision" for a response that does not decode."""
        ex = self.transact(dub_request(lower, upper, src=self.uid, tn=self.next_tn()))
        if ex.response is None:
            return None
        uid = decode_dub(ex.response.data)
        return "collision" if uid is None else uid

    def mute(self, uid: int) -> bool:
        ex = self.transact(self.request(DISCOVERY, DISC_MUTE, dst=uid))
        reply = parse(ex.response.data) if ex.response else None
        return (reply is not None and reply.src == uid
                and reply.cc == DISCOVERY_RESPONSE and reply.pid == DISC_MUTE)

    def unmute_all(self) -> None:
        self.transact(self.request(DISCOVERY, DISC_UN_MUTE, dst=BROADCAST),
                      wait_us=BROADCAST_BACKOFF)

    def discover(self, lower: int = 0, upper: int = MAX_UID) -> List[int]:
        """The binary search (as OLA's DiscoveryAgent does it): un-mute
        all, then DUB a branch; a clean response is muted and the branch
        tried again, a collision splits it at the middle, silence closes it."""
        self.unmute_all()
        found: List[int] = []
        branches = [(lower, upper)]
        while branches:
            lo, hi = branches.pop()
            reply = self.dub(lo, hi)
            if reply is None:
                continue
            if reply != "collision" and lo <= reply <= hi and self.mute(reply):
                found.append(reply)
                branches.append((lo, hi))
            elif lo < hi:
                mid = (lo + hi) // 2
                branches += [(mid + 1, hi), (lo, mid)]
        return found
