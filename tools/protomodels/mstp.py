"""BACnet MS/TP (ANSI/ASHRAE 135 clause 9): frames, CRCs, the RS-485 line,
a master node, and the checks the tests make. Written from the protocol's
public descriptions, never from ``firmware/mstp_master.loom``.

Sources. Clause 9 itself is not free, so everything here is **secondary**,
from the open-source BACnet stack (S. Karg et al., github.com/bacnet-stack/
bacnet-stack, branch master, read 2026-11-07 with the web fetch tool):

* ``src/bacnet/datalink/mstp.c``: the frame format (comment above
  ``MSTP_Create_Frame``: preamble X'55' X'FF', frame type, destination,
  source, length most significant octet first, header CRC, data, data CRC
  least significant octet first, an optional pad of one X'FF'), the
  Receive Frame state machine (``MSTP_Receive_Frame_FSM``: IDLE eats octets
  until X'55', a repeated X'55' stays in PREAMBLE, the header CRC residue
  must be X'55' or the frame is invalid and dropped, the data CRC residue
  X'F0B8', a silence over Tframe_abort aborts a frame) and the Master Node
  state machine (``MSTP_Master_Node_FSM``), which :class:`ModelMaster`
  follows state by state;
* ``src/bacnet/datalink/crc.c``: ``CRC_Calc_Header`` and ``CRC_Calc_Data``,
  copied below as :func:`crc8_step` and :func:`crc16_step`. The file names
  no polynomial. Clause 9 names them x^8 + x^7 + 1 and x^16 + x^12 + x^5 +
  1 (CRC-CCITT), both reflected (from memory, unchecked: the director should
  check against the standard's Annex G); ``tools/tests/test_fw_mstp.py``
  checks that a bit-serial reflected LFSR with those polynomials gives the
  same values as the stack's code on random data, so the firmware's use of
  them stands on the stack's code either way;
* ``src/bacnet/datalink/mstpdef.h`` and ``mstp.h``: the frame type codes
  (Token 0, Poll For Master 1, Reply To Poll For Master 2, Test_Request 3,
  Test_Response 4, BACnet Data Expecting Reply 5, BACnet Data Not Expecting
  Reply 6, Reply Postponed 7) and the timing constants: Tno_token 500 ms,
  Tslot 10 ms, Tusage_delay 15 ms, Tturnaround 40 bit times, Tframe_gap 20
  bit times, Tpostdrive 15 bit times, Npoll 50, Nretry_token 1,
  Nmin_octets 4, Nmax_master 127; Tframe_abort 60 bit times (mstp.h's
  comment; 95 ms by default, at most 100 ms); Tusage_timeout 20 .. 35 ms
  (default 35); Treply_timeout 255 .. 300 ms. The characters are 8N1
  (one start bit, eight data bits LSB first, one stop bit) at 9600 ..
  115200 baud (mstp.c's auto-baud list: 9600 .. 115200).

Units. The line models count in clocks of the 50 MHz chip clock; the
protocol's times in bit times (``bit`` clocks each). The tests scale the
millisecond timers (Tno_token, Tslot, Tusage_timeout) down to bit times so
that a token round fits a simulation; the bit-time rules (Tturnaround,
Tframe_gap, Tframe_abort, Tpostdrive) are kept as the stack has them.
"""

from __future__ import annotations

import dataclasses
from collections import deque
from typing import Dict, List, Optional, Sequence, Tuple

from tools.protomodels.bench import Drive, Lines, Model, PinSpec, pad_of

CLOCK = 50e6

# Frame types (mstpdef.h).
TOKEN, PFM, RPFM, TEST_REQ, TEST_RESP, DER, DNER, POSTPONED = range(8)
BROADCAST = 255
PREAMBLE = (0x55, 0xFF)
CHAR_BITS = 10                       # 8N1

# Bit-time rules (mstpdef.h, mstp.h).
T_TURNAROUND = 40                    # bit times: no driver sooner after a frame
T_FRAME_GAP = 20                     # bit times: the most idle between octets sent
T_POSTDRIVE = 15                     # bit times: driver off after the last stop bit
T_FRAME_ABORT = 60                   # bit times: a receiver gives up a frame
N_POLL = 50
N_RETRY_TOKEN = 1
N_MIN_OCTETS = 4
HEADER_RESIDUE = 0x55
DATA_RESIDUE = 0xF0B8


# --------------------------------------------------------------------- CRCs
def crc8_step(data: int, crc: int) -> int:
    """crc.c ``CRC_Calc_Header``, as written there."""
    c = (crc ^ data) & 0xFF
    c = c ^ (c << 1) ^ (c << 2) ^ (c << 3) ^ (c << 4) ^ (c << 5) ^ (c << 6) ^ (c << 7)
    return ((c & 0xFE) ^ ((c >> 8) & 1)) & 0xFF


def crc16_step(data: int, crc: int) -> int:
    """crc.c ``CRC_Calc_Data``, as written there."""
    low = (crc & 0xFF) ^ data
    return ((crc >> 8) ^ (low << 8) ^ (low << 3) ^ (low << 12) ^ (low >> 4)
            ^ (low & 0x0F) ^ ((low & 0x0F) << 7)) & 0xFFFF


def header_crc(octets: Sequence[int]) -> int:
    """The header CRC octet sent: the complement of the CRC-8 (preset 0xFF)
    over type, destination, source and the two length octets."""
    crc = 0xFF
    for b in octets:
        crc = crc8_step(b, crc)
    return ~crc & 0xFF


def data_crc(octets: Sequence[int]) -> int:
    """The data CRC sent (complemented, preset 0xFFFF); its low octet goes first."""
    crc = 0xFFFF
    for b in octets:
        crc = crc16_step(b, crc)
    return ~crc & 0xFFFF


def residue8(octets: Sequence[int]) -> int:
    crc = 0xFF
    for b in octets:
        crc = crc8_step(b, crc)
    return crc


def residue16(octets: Sequence[int]) -> int:
    crc = 0xFFFF
    for b in octets:
        crc = crc16_step(b, crc)
    return crc


def lfsr_reflected(octets: Sequence[int], poly: int, width: int, init: int) -> int:
    """A bit-serial reflected CRC (bits LSB first, the reflected polynomial,
    no final complement), to check the polynomial names against crc.c."""
    rpoly = int(format(poly, "0%db" % width)[::-1], 2)
    crc = init
    for b in octets:
        for i in range(8):
            bit = ((b >> i) & 1) ^ (crc & 1)
            crc >>= 1
            if bit:
                crc ^= rpoly
    return crc


# ------------------------------------------------------------------- frames
def build_frame(ftype: int, da: int, sa: int, data: Sequence[int] = ()) -> List[int]:
    """The octets of a frame (mstp.c ``MSTP_Create_Frame``), without the pad."""
    n = len(data)
    head = [ftype & 0xFF, da & 0xFF, sa & 0xFF, n >> 8, n & 0xFF]
    out = list(PREAMBLE) + head + [header_crc(head)]
    if n:
        crc = data_crc(data)
        out += list(data) + [crc & 0xFF, crc >> 8]
    return out


def bad_header_crc(frame: Sequence[int]) -> List[int]:
    """``frame`` with its header CRC octet wrong."""
    out = list(frame)
    out[7] ^= 0x01
    return out


# ------------------------------------------------------------- line models
class NodeTx:
    """A model station's RS-485 driver: queued frames as 8N1 characters back
    to back, the driver enabled one bit before the first start bit and
    released at the end of the last stop bit. ``level``/``driving`` are what
    it puts on the bus in the current cycle (updated by :meth:`tick`)."""

    def __init__(self, bit: float) -> None:
        self.bit = bit
        self.level = 1
        self.driving = False
        self._changes: deque = deque()       # (cycle, level, driving)
        self.free_at = 0.0
        self.sent: List[Tuple[int, List[int]]] = []

    def send(self, octets: Sequence[int], start: float) -> int:
        """Queue a frame whose driver goes on at ``start`` (or when free)."""
        t = max(float(start), self.free_at)
        self._changes.append((int(round(t)), 1, True))
        t += self.bit
        t0 = t
        for value in octets:
            levels = [0] + [(value >> i) & 1 for i in range(8)] + [1]
            for k, lv in enumerate(levels):
                self._changes.append((int(round(t + k * self.bit)), lv, True))
            t += CHAR_BITS * self.bit
        self._changes.append((int(round(t)), 1, False))
        self.free_at = t
        self.sent.append((int(round(t0)), list(octets)))
        return int(round(t0))

    @property
    def busy(self) -> bool:
        return bool(self._changes)

    def tick(self, cycle: int) -> None:
        ch = self._changes
        while ch and ch[0][0] <= cycle:
            _, self.level, self.driving = ch.popleft()


@dataclasses.dataclass
class Octet:
    value: int
    start: int                  # the cycle the start bit was first seen low
    end: float                  # start + 10 bits: the end of the stop bit
    stop_ok: bool
    edges: List[int]            # every edge inside the character
    drivers: Tuple[str, ...]    # who drove the bus at the start bit


class OctetRx:
    """Decodes 8N1 octets from a level, sampled at the bit middles from the
    first low cycle after a high one (with the nominal ``bit``)."""

    def __init__(self, bit: float) -> None:
        self.bit = bit
        self.octets: List[Octet] = []
        self._prev = 1
        self._cur: Optional[Octet] = None
        self._levels: List[int] = []

    def feed(self, cycle: int, level: int, drivers: Tuple[str, ...] = ()) -> Optional[Octet]:
        done = None
        cur = self._cur
        if cur is None:
            if level == 0 and self._prev == 1:
                self._cur = Octet(0, cycle, cycle + CHAR_BITS * self.bit, False, [cycle], drivers)
                self._levels = []
        else:
            if level != self._prev:
                cur.edges.append(cycle)
            k = len(self._levels)
            if cycle >= cur.start + (k + 0.5) * self.bit:
                self._levels.append(level)
                if len(self._levels) == CHAR_BITS:
                    cur.value = sum(b << i for i, b in enumerate(self._levels[1:9]))
                    cur.stop_ok = self._levels[9] == 1
                    self.octets.append(cur)
                    self._cur = None
                    done = cur
        self._prev = level
        return done


class MstpBus(Model):
    """The two-wire RS-485 bus (one clock late, like every bench model).
    Chip nodes are ``name: (tx, de)`` pads; model nodes are :class:`NodeTx`
    objects by name. The bus carries the level of whoever drives it, mark
    (1) when nobody does (the fail-safe bias), the lowest level if several
    do, which is recorded in :attr:`contentions`. The level is driven onto
    every pad in ``receivers``. :attr:`rx` decodes every octet on the bus
    and :attr:`de_edges` keeps every node's driver edges."""

    def __init__(self, bit: float, chips: Dict[str, Tuple[PinSpec, PinSpec]],
                 receivers: Sequence[PinSpec], models: Optional[Dict[str, NodeTx]] = None) -> None:
        self.chips = {n: (pad_of(tx), pad_of(de)) for n, (tx, de) in chips.items()}
        self.receivers = [pad_of(p) for p in receivers]
        self.models: Dict[str, NodeTx] = dict(models or {})
        self.level = 1
        self.drivers: Tuple[str, ...] = ()
        self.contentions: List[Tuple[int, Tuple[str, ...]]] = []
        self.rx = OctetRx(bit)
        self.de_edges: Dict[str, List[Tuple[int, int]]] = {n: [] for n in list(self.chips) + list(self.models)}
        self._de: Dict[str, int] = {n: 0 for n in self.de_edges}
        self.listeners: List["ModelMaster"] = []

    def drive(self, drive: Drive, cycle: int) -> None:
        for m in self.models.values():
            m.tick(cycle)
        for pad in self.receivers:
            drive.set(pad, self.level)

    def observe(self, lines: Lines) -> None:
        c = lines.cycle
        levels, names = [], []
        for n, (tx, de) in self.chips.items():
            d = lines.get(de)
            self._edge(n, c, d)
            if d:
                levels.append(lines.get(tx))
                names.append(n)
        for n, m in self.models.items():
            self._edge(n, c, int(m.driving))
            if m.driving:
                levels.append(m.level)
                names.append(n)
        if len(levels) > 1:
            self.contentions.append((c, tuple(names)))
        self.level = min(levels) if levels else 1
        self.drivers = tuple(names)
        octet = self.rx.feed(c, self.level, self.drivers)
        for lst in self.listeners:
            lst.hear(c, self.level, octet)

    def _edge(self, name: str, cycle: int, d: int) -> None:
        if d != self._de[name]:
            self.de_edges[name].append((cycle, d))
            self._de[name] = d


# ----------------------------------------------------------- frame decode
@dataclasses.dataclass
class Frame:
    octets: List[Octet]
    valid: bool
    why: str = ""                   # for an invalid frame: what was wrong

    @property
    def ftype(self) -> int:
        return self.octets[2].value

    @property
    def da(self) -> int:
        return self.octets[3].value

    @property
    def sa(self) -> int:
        return self.octets[4].value

    @property
    def data(self) -> List[int]:
        n = (self.octets[5].value << 8) | self.octets[6].value
        return [o.value for o in self.octets[8:8 + n]]

    @property
    def start(self) -> int:
        return self.octets[0].start

    @property
    def end(self) -> float:
        return self.octets[-1].end

    @property
    def sender(self) -> Tuple[str, ...]:
        return self.octets[0].drivers


class FrameRx:
    """The Receive Frame state machine (mstp.c ``MSTP_Receive_Frame_FSM``)
    fed one octet at a time; complete frames, valid or not, land in
    :attr:`frames`. ``abort`` is Tframe_abort in clocks."""

    def __init__(self, bit: float, abort_bits: float = T_FRAME_ABORT) -> None:
        self.abort = abort_bits * bit
        self.frames: List[Frame] = []
        self._cur: List[Octet] = []
        self._need = 0
        self._last_end: Optional[float] = None

    def feed(self, o: Octet) -> Optional[Frame]:
        out = None
        if self._cur and self._last_end is not None and o.start - self._last_end > self.abort:
            out = self._finish(False, "Tframe_abort")
        self._last_end = o.end
        cur = self._cur
        if not cur:                                 # IDLE
            if o.value == 0x55:
                cur.append(o)
            return out
        if len(cur) == 1:                           # PREAMBLE
            if o.value == 0xFF:
                cur.append(o)
            elif o.value != 0x55:
                self._cur = []
            return out
        cur.append(o)
        if len(cur) == 8:                           # the header is in
            head = [x.value for x in cur[2:8]]
            if residue8(head) != HEADER_RESIDUE:
                return self._finish(False, "header CRC")
            n = (head[3] << 8) | head[4]
            if n == 0:
                return self._finish(True)
            self._need = 8 + n + 2
        elif len(cur) > 8 and len(cur) == self._need:
            n = self._need - 10
            ok = residue16([x.value for x in cur[8:8 + n + 2]]) == DATA_RESIDUE
            return self._finish(ok, "" if ok else "data CRC")
        return out

    def _finish(self, valid: bool, why: str = "") -> Frame:
        f = Frame(self._cur, valid, why)
        self.frames.append(f)
        self._cur = []
        self._need = 0
        return f


def decode(octets: Sequence[Octet], bit: float) -> List[Frame]:
    rx = FrameRx(bit)
    for o in octets:
        rx.feed(o)
    return rx.frames


# ------------------------------------------------------------------ checks
def frame_violations(frame: Frame, de_edges: Sequence[Tuple[int, int]], bit: float,
                     edge_tol: float, prev_end: Optional[float]) -> List[str]:
    """What the line rules forbid a sender of ``frame`` (a chip node, whose
    driver edges are ``de_edges``): an edge more than ``edge_tol`` clocks
    off its bit boundary, a low stop bit, more than Tframe_gap of idle
    between two octets, the driver not on from before the first start bit
    to the end of the last stop bit or on longer than Tpostdrive after it,
    and a driver enabled sooner than Tturnaround after the end of the frame
    before (``prev_end``, from another station)."""
    bad = []
    prev = None
    for i, o in enumerate(frame.octets):
        if not o.stop_ok:
            bad.append("octet %d: stop bit low" % i)
        for e in o.edges:
            off = (e - o.start) / bit
            err = abs(off - round(off)) * bit
            if err > edge_tol:
                bad.append("octet %d: edge at +%d clocks is %.1f off" % (i, e - o.start, err))
        if prev is not None and o.start - prev.end > T_FRAME_GAP * bit + edge_tol:
            bad.append("octet %d: %.1f bit times of idle before it, Tframe_gap is %d"
                       % (i, (o.start - prev.end) / bit, T_FRAME_GAP))
        prev = o
    on = [c for c, d in de_edges if d == 1 and c <= frame.start]
    off = [c for c, d in de_edges if d == 0 and c > frame.start]
    if not on:
        bad.append("DE not on before the first start bit")
    else:
        if prev_end is not None and on[-1] < prev_end + T_TURNAROUND * bit - edge_tol:
            bad.append("DE on %.1f bit times after the frame before, Tturnaround is %d"
                       % ((on[-1] - prev_end) / bit, T_TURNAROUND))
    if off:
        if off[0] < frame.end - edge_tol:
            bad.append("DE off before the last stop bit ends")
        if off[0] > frame.end + T_POSTDRIVE * bit:
            bad.append("DE on more than Tpostdrive after the frame")
    return bad


def token_violations(frames: Sequence[Frame], station: Dict[str, int],
                     lost_silence: Optional[float] = None,
                     usage_silence: Optional[float] = None) -> List[str]:
    """The token rules over the valid frames on the bus, with ``station``
    naming each sender's address: a Token, Poll For Master or data frame
    comes only from the station that holds the token (the one the last
    Token was sent to; before the first Token, or after a silence of
    ``lost_silence`` clocks or more (a lost token), the one that sends a
    Poll For Master; after a silence of ``usage_silence`` or more following
    a Token, the station that passed it, which keeps it when the successor
    does not use it, PASS_TOKEN); a Reply To Poll For Master only from the
    station the last Poll For Master was sent to, to its sender."""
    bad = []
    holder: Optional[int] = None
    polled: Optional[Tuple[int, int]] = None
    passer: Optional[int] = None
    for i, f in enumerate(frames):
        if not f.valid:
            continue
        who = [station[n] for n in f.sender if n in station]
        sa = who[0] if who else f.sa
        if sa != f.sa:
            bad.append("frame %d: SA %d sent by station %d" % (i, f.sa, sa))
        if f.ftype == RPFM:
            if polled is None or (sa, f.da) != polled:
                bad.append("frame %d: Reply To Poll For Master nobody asked for" % i)
            polled = None
            continue
        lost = (lost_silence is not None and i > 0
                and f.start - frames[i - 1].end >= lost_silence)
        if (holder is None or lost) and f.ftype == PFM:
            holder = sa
        unused = (usage_silence is not None and i > 0 and sa == passer
                  and frames[i - 1].ftype == TOKEN
                  and f.start - frames[i - 1].end >= usage_silence)
        if unused:
            holder = sa
        if sa != holder:
            bad.append("frame %d: type %d from %d, the token is with %s" % (i, f.ftype, sa, holder))
        if f.ftype == TOKEN:
            passer, holder = sa, f.da
        elif f.ftype == PFM:
            polled = (f.da, sa)
    return bad


# ------------------------------------------------------------ master node
class ModelMaster:
    """A master station on an :class:`MstpBus`, following mstp.c's
    ``MSTP_Master_Node_FSM`` (states IDLE, USE_TOKEN, DONE_WITH_TOKEN,
    PASS_TOKEN, POLL_FOR_MASTER and NO_TOKEN; no WAIT_FOR_REPLY or
    ANSWER_DATA_REQUEST: it sends only Data Not Expecting Reply frames and
    is sent no Data Expecting Reply). Times are in clocks; ``t_no_token``,
    ``t_slot`` and ``t_usage`` are the scaled Tno_token, Tslot and
    Tusage_timeout. It answers Poll For Master, sends the frames in
    :attr:`queue` (``(da, data)``) one per token, and keeps the valid data
    frames for it or broadcast in :attr:`received`. ``alive = False``
    makes it deaf and mute (a station that has gone away)."""

    def __init__(self, name: str, bus: MstpBus, ts: int, nmax: int, bit: float, *,
                 t_usage: float, t_no_token: float, t_slot: float,
                 npoll: int = N_POLL) -> None:
        self.name, self.ts, self.nmax, self.bit = name, ts, nmax, bit
        self.t_usage, self.t_no_token, self.t_slot, self.npoll = t_usage, t_no_token, t_slot, npoll
        self.tx = NodeTx(bit)
        bus.models[name] = self.tx
        bus.de_edges[name] = []
        bus._de[name] = 0
        bus.listeners.append(self)
        self.frx = FrameRx(bit)
        self.state = "IDLE"
        self.ns = self.ps = ts
        self.token_count = npoll
        self.sole = False
        self.retry = 0
        self.events = 0
        self.last = 0.0                 # the end of the last octet on the bus
        self.now = 0
        self.alive = True
        self.queue: deque = deque()
        self.received: List[Frame] = []
        self.log: List[Tuple[int, str]] = []

    # ------------------------------------------------------------- helpers
    def _next(self, a: int) -> int:
        return (a + 1) % (self.nmax + 1)

    def _send(self, ftype: int, da: int, data: Sequence[int] = ()) -> None:
        start = max(self.now + 1.0, self.last + T_TURNAROUND * self.bit, self.tx.free_at)
        self.tx.send(build_frame(ftype, da, self.ts, data), start)
        self.log.append((self.now, "send %d to %d" % (ftype, da)))

    # --------------------------------------------------------------- input
    def hear(self, cycle: int, level: int, octet: Optional[Octet]) -> None:
        self.now = cycle
        if not self.alive:
            return
        if self.tx.busy:
            self.last = float(cycle)
            return
        if octet is not None:
            self.last = max(self.last, octet.end)
            if self.name not in octet.drivers:
                self.events += 1
                frame = self.frx.feed(octet)
                if frame is not None:
                    self._frame(frame)
            return
        self._time(cycle - self.last)

    def _frame(self, f: Frame) -> None:
        ours = f.valid and f.da == self.ts
        if self.state == "POLL_FOR_MASTER":
            if ours and f.ftype == RPFM:
                self.sole = False
                self.ns = f.sa
                self.ps = self.ts
                self.token_count = 0
                self._pass()
                return
            if not f.valid:
                self._poll_timeout()
                return
            self.state = "IDLE"
        elif self.state == "PASS_TOKEN":
            self.state = "IDLE"
        if self.state != "IDLE" or not f.valid:
            return
        if f.ftype in (DER, DNER) and f.da in (self.ts, BROADCAST):
            self.received.append(f)
        if not ours:
            return
        if f.ftype == TOKEN:
            self.sole = False
            self._use_token()
        elif f.ftype == PFM:
            self._send(RPFM, f.sa)

    def _time(self, silence: float) -> None:
        if self.state == "IDLE":
            if silence >= self.t_no_token + self.t_slot * self.ts:
                # NO_TOKEN: this station's slot has come.
                self.ps = self._next(self.ts)
                self.ns = self.ts
                self.token_count = 0
                self._poll(self.ps)
        elif self.state == "PASS_TOKEN":
            if silence > self.t_usage:
                if self.retry < N_RETRY_TOKEN:
                    self.retry += 1
                    self.events = 0
                    self._send(TOKEN, self.ns)
                else:
                    self.ps = self._next(self.ns)
                    self.ns = self.ts
                    self.token_count = 0
                    self._poll(self.ps)
        elif self.state == "POLL_FOR_MASTER":
            if silence > self.t_usage:
                self._poll_timeout()

    # -------------------------------------------------------------- states
    def _poll(self, station: int) -> None:
        self.retry = 0
        self.state = "POLL_FOR_MASTER"
        self._send(PFM, station)

    def _pass(self) -> None:
        self.retry = 0
        self.events = 0
        self.state = "PASS_TOKEN"
        self._send(TOKEN, self.ns)

    def _poll_timeout(self) -> None:
        if self.sole:
            self._use_token()
        elif self.ns != self.ts:
            self._pass()
        elif self._next(self.ps) != self.ts:
            self.ps = self._next(self.ps)
            self._poll(self.ps)
        else:
            self.sole = True
            self._use_token()

    def _use_token(self) -> None:
        if self.queue:
            da, data = self.queue.popleft()
            self._send(DNER, da, data)
        self._done_with_token()

    def _done_with_token(self) -> None:
        # Nmax_info_frames = 1: the frame count is always used up here.
        if not self.sole and self.ns == self.ts:
            self.ps = self._next(self.ts)
            self._poll(self.ps)
        elif self.token_count < self.npoll - 1:
            self.token_count += 1
            if self.sole and self.ns != self._next(self.ts):
                self.state = "USE_TOKEN"        # keeps it; looped by _time
                self._sole_wait()
            else:
                self._pass()
        elif self._next(self.ps) == self.ns:
            if self.sole:
                self.ps = self._next(self.ns)
                self.ns = self.ts
                self.token_count = 1
                self._poll(self.ps)
            else:
                self.ps = self.ts
                self.token_count = 1
                self._pass()
        else:
            self.ps = self._next(self.ps)
            self._poll(self.ps)

    def _sole_wait(self) -> None:
        # A sole master with nothing to send keeps the token; the model
        # goes back to polling rather than spin (it is never one in the tests).
        self._poll(self._next(self.ts))
