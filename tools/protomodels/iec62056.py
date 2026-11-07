"""IEC 62056-21 (formerly IEC 61107, "IEC 1107") protocol mode C reference models.

The optical port of an electricity meter: the hand-held unit (HHU, the
client) asks with a request message at 300 baud, the tariff device (the
meter) answers with its identification, whose baud rate character Z offers
a faster rate, the client confirms it with an acknowledgement/option select
message, both switch, and the meter sends its data message at the new rate.

Written from IEC 62056-21:2002, first edition (read 2026-11-07, the text of
the edition, not a secondary source). Every figure below names its clause:

* 5.2: initial baud rate 300; standard rates 300, 600, 1 200, 2 400, 4 800,
  9 600, 19 200.
* 5.4, 5.6: ISO/IEC 1177 character format, 1 start bit, 7 data bits, 1
  parity bit (even), 1 stop bit; LSB first (ISO/IEC 1177's start-stop
  order; that standard itself was not read, the order is the universal
  asynchronous one and is marked unchecked).
* 4.3.5.2, 4.3.5.3: the optical levels, ON = SPACE = binary 0, OFF = MARK =
  binary 1 (the quiescent state). At logic level, as here, the line idles
  at 1 (the reading head's or the meter's receiver output).
* 6.3.1 to 6.3.4, 6.3.14 items 1 to 15: the request ``/ ? [address] ! CR
  LF``, the identification ``/ X X X Z identification CR LF`` (three
  manufacturer letters, the baud character, at most 16 identification
  characters), the acknowledgement/option select ``ACK V Z Y CR LF``, the
  data message ``STX data-block ! CR LF ETX BCC``.
* 6.3.14 item 13 c): Z for protocol mode C, '0' .. '6' = 300 .. 19 200 baud;
  '7' .. '9' reserved. The request, the identification and the
  acknowledgement go at 300 baud, the data message at the rate the protocol
  settles on.
* 6.3.14 item 12: a third manufacturer letter in lower case means the
  device's minimum reaction time is 20 ms instead of 200 ms.
* 6.4.3.2: ``ACK 0 Z 0 CR LF`` asks for data readout; the communication
  switches to Z only if the Z of the identification and of the
  acknowledgement are identical; it stays at 300 if Z is '0', if the
  acknowledgement is incorrect or unsupported, or if none is received.
* 6.4.3.6: reaction time between the reception of a message and the
  transmission of the answer, (20 ms) 200 ms <= tr <= 1 500 ms; without a
  response the meter waits 1 500 ms < tt <= 2 200 ms after its
  identification and then continues (the fallback data readout, Figure 13);
  between two characters of a message ta < 1 500 ms.
* 6.2 NOTE (under Figure 7): the BCC (ISO/IEC 1745, 1155) runs from the
  character after the first SOH or STX up to and including the ETX, and
  follows the ETX. Figure 7 itself (the worked example) is a drawing that
  was not readable; the BCC as the exclusive OR of the characters is ISO
  1155's longitudinal check, marked unchecked against that figure. With
  even parity it makes no difference whether the parity bits are XORed
  too: the XOR of even-parity characters is the even-parity character of
  the XOR of their data bits.

Not taken from the standard: the edge window of :data:`EDGE_TOL` (5.3 cites
ISO/IEC 7480 category 1, which was not read) is this bench's own, 1/16 bit.

Times are in core clocks; ``clock`` (Hz) turns milliseconds into clocks.
"""

from __future__ import annotations

import bisect
import dataclasses
from typing import Callable, List, Optional, Tuple

from .bench import Drive, Lines, Model, PinSpec, pad_of

# ------------------------------------------------------------ the standard
INITIAL_BAUD = 300                                  # 5.2
BAUD = {z: 300 << z for z in range(7)}              # 6.3.14 13 c), 5.2
SOH, STX, ETX, EOT, ACK, NAK = 0x01, 0x02, 0x03, 0x04, 0x06, 0x15
CR, LF = 0x0D, 0x0A
TR_MIN_MS = 200.0                                   # 6.4.3.6
TR_MIN_FAST_MS = 20.0                               # 6.3.14 item 12
TR_MAX_MS = 1500.0                                  # 6.4.3.6
TT_MIN_MS, TT_MAX_MS = 1500.0, 2200.0               # 6.4.3.6 (exclusive min)
TA_MAX_MS = 1500.0                                  # 6.4.3.6 (exclusive)
IDENT_MAX = 16                                      # 6.3.14 item 14
FRAME_BITS = 10                                     # 5.4: start, 7, parity, stop
EDGE_TOL = 1.0 / 16                                 # bits; this bench's window


def even_parity(c: int) -> int:
    """The parity bit that makes the 7 data bits and it even (5.6)."""
    return bin(c & 0x7F).count("1") & 1


def frame_bits(c: int) -> List[int]:
    """The ten line bits of character ``c``: start 0, d0 .. d6, parity, stop 1."""
    return [0] + [(c >> i) & 1 for i in range(7)] + [even_parity(c), 1]


def bcc(chars) -> int:
    """XOR of the 7-bit characters ``chars`` (6.2 NOTE: after STX, ETX included)."""
    v = 0
    for c in chars:
        v ^= (ord(c) if isinstance(c, str) else c) & 0x7F
    return v


def request(address: str = "") -> str:
    """6.3.1: the request message (no address = the general address)."""
    return "/?" + address + "!\r\n"


def identification(mfr: str, z: str, ident: str) -> str:
    """6.3.2: ``/XXXZ<ident>CR LF``."""
    assert len(mfr) == 3 and len(z) == 1 and len(ident) <= IDENT_MAX
    return "/" + mfr + z + ident + "\r\n"


def ack_option(z: str, v: str = "0", y: str = "0") -> str:
    """6.3.3: ``ACK V Z Y CR LF``; V '0' normal protocol, Y '0' data readout."""
    return chr(ACK) + v + z + y + "\r\n"


def data_message(block: str, *, bad_bcc: bool = False) -> str:
    """6.3.4: ``STX block ! CR LF ETX BCC``; ``bad_bcc`` flips the BCC's bit 0."""
    body = block + "!\r\n" + chr(ETX)
    check = bcc(body) ^ (1 if bad_bcc else 0)
    return chr(STX) + body + chr(check)


def reaction_min_ms(mfr: str) -> float:
    """6.3.14 item 12: 20 ms for a lower-case third letter, else 200 ms."""
    return TR_MIN_FAST_MS if mfr[2:3].islower() else TR_MIN_MS


def baud_of(z_char: str) -> Optional[int]:
    """Mode C baud rate of a Z character, ``None`` if not '0' .. '6'."""
    if len(z_char) == 1 and "0" <= z_char <= "6":
        return BAUD[ord(z_char) - ord("0")]
    return None


def parse_identification(text: str) -> Tuple[str, str, str]:
    """``(manufacturer, Z, identification)`` of a 6.3.2 message; ValueError if
    it is not one."""
    if len(text) < 7 or text[0] != "/" or not text.endswith("\r\n"):
        raise ValueError("not an identification message: %r" % text)
    ident = text[5:-2]
    if len(ident) > IDENT_MAX or "/" in ident or "!" in ident:
        raise ValueError("bad identification field: %r" % ident)
    return text[1:4], text[4], ident


def negotiated_baud(ident_z: str, ack: Optional[str]) -> int:
    """6.4.3.2: the data message's rate, from the identification's Z and the
    acknowledgement received (``None``: none)."""
    if ack is None or len(ack) != 6 or ack[0] != chr(ACK) or ack[4:] != "\r\n":
        return INITIAL_BAUD
    if ack[1] != "0" or ack[3] != "0" or ack[2] != ident_z:
        return INITIAL_BAUD
    return baud_of(ident_z) or INITIAL_BAUD


# ------------------------------------------------------------ pin models
@dataclasses.dataclass
class Char:
    """One received character."""
    value: int                      # the 7 data bits
    start: int                      # cycle of its falling start edge
    end: int                        # cycle its stop bit ends (nominal)
    parity_ok: bool
    stop_ok: bool


class SerialPort(Model):
    """A half-duplex 7E1 port at a rate that can change between characters.

    ``tx`` is the pad it drives (a chip input), ``rx`` the pad it watches (a
    chip output). It sends with :meth:`send` and hands every received
    character to :meth:`on_char`; :meth:`at` schedules a callback at a cycle.
    Both directions use :attr:`baud`, which :meth:`set_baud` changes for the
    frames that have not started.
    """

    def __init__(self, tx: PinSpec, rx: PinSpec, clock: float,
                 baud: int = INITIAL_BAUD) -> None:
        self.tx_pad = pad_of(tx)
        self.rx_pad = pad_of(rx)
        self.clock = float(clock)
        self.baud = baud
        self.level = 1
        self.now = 0
        self._changes: List[Tuple[int, int]] = []
        self._ci = 0
        self._free = 0.0
        self._events: List[Tuple[int, int, Callable[[], None]]] = []
        self._seq = 0
        self._prev = 0                  # armed only after the line was idle
        self._start: Optional[int] = None
        self._bits: List[int] = []
        self._next = 0.0
        self._rx_bit = 0.0
        self.received: List[Char] = []

    # ------------------------------------------------------------ helpers
    def ms(self, ms: float) -> int:
        return int(round(ms * self.clock / 1000.0))

    @property
    def bit(self) -> float:
        return self.clock / self.baud

    def set_baud(self, baud: int) -> None:
        self.baud = baud

    def at(self, cycle: int, fn: Callable[[], None]) -> None:
        self._seq += 1
        bisect.insort(self._events, (int(cycle), self._seq, fn))

    def send(self, text, start: Optional[float] = None) -> int:
        """Queue ``text`` back to back from ``start`` (default: now); returns
        the cycle its last stop bit ends."""
        t = max(self._free, float(self.now + 1))
        if start is not None:
            t = max(t, float(start))
        b = self.bit
        for c in text:
            c = ord(c) if isinstance(c, str) else c
            for k, v in enumerate(frame_bits(c)):
                self._changes.append((int(round(t + k * b)), v))
            t += FRAME_BITS * b
        self._free = t
        return int(round(t))

    @property
    def sending(self) -> bool:
        return self._ci < len(self._changes)

    def on_char(self, ch: Char) -> None:
        """A character was received (stop bit sampled)."""

    # ------------------------------------------------------------- bench
    def drive(self, drive: Drive, cycle: int) -> None:
        self.now = cycle
        ch = self._changes
        while self._ci < len(ch) and ch[self._ci][0] <= cycle:
            self.level = ch[self._ci][1]
            self._ci += 1
        drive.set(self.tx_pad, self.level)

    def observe(self, lines: Lines) -> None:
        cycle = lines.cycle
        v = lines.get(self.rx_pad)
        if self._start is None:
            if self._prev == 1 and v == 0:
                self._start = cycle
                self._rx_bit = self.bit
                self._bits = []
                self._next = cycle + 0.5 * self._rx_bit
        elif cycle >= self._next:
            self._bits.append(v)
            k = len(self._bits)
            if k == 1 and v:
                self._start = None              # a glitch
            elif k == FRAME_BITS:
                d = self._bits[1:8]
                value = sum(b << i for i, b in enumerate(d))
                par = (sum(d) + self._bits[8]) % 2 == 0
                end = int(round(self._start + FRAME_BITS * self._rx_bit))
                ch = Char(value, self._start, end, par, v == 1)
                self._start = None
                self.received.append(ch)
                self.on_char(ch)
            else:
                self._next = self._start + (k + 0.5) * self._rx_bit
        self._prev = v
        ev = self._events
        while ev and ev[0][0] <= cycle:
            ev.pop(0)[2]()


class Meter(SerialPort):
    """A tariff device in protocol mode C, data readout only (6.4.3).

    It answers a request for the general address, or for ``address``, with
    ``identification(mfr, z, ident)`` after ``tr_ms``; then waits ``tt_ms``
    for an acknowledgement. With ``ACK 0 Z 0`` and the Z it offered it
    switches to that rate; with any other acknowledgement, or none in
    ``tt_ms`` (sent then, Figure 13's fallback), it stays at 300 (6.4.3.2).
    The data message goes ``tr_ms`` after the acknowledgement, then it is
    back at 300. ``bad_bcc`` corrupts every data message's BCC; with
    ``answers`` False it only keeps its transmitter idle.
    """

    def __init__(self, tx: PinSpec, rx: PinSpec, clock: float, *, mfr: str = "LGZ",
                 z: str = "2", ident: str = "ZMD120", block: str = "1.8.0(001234.5*kWh)\r\n",
                 address: str = "", tr_ms: Optional[float] = None, tt_ms: float = 1800.0,
                 bad_bcc: bool = False, answers: bool = True) -> None:
        super().__init__(tx, rx, clock)
        self.answers = answers
        self.mfr, self.z, self.ident, self.block = mfr, z, ident, block
        self.address = address
        self.tr_ms = tr_ms if tr_ms is not None else reaction_min_ms(mfr) + 10.0
        self.tt_ms = tt_ms
        self.bad_bcc = bad_bcc
        self.state = "idle"
        self.buf = ""
        self.acks: List[Optional[str]] = []         # per session, None = none
        self.rates: List[int] = []                  # data message rate per session
        self._ident_end = 0
        self._session = 0

    def _addressed(self, addr: str) -> bool:
        """6.3.14 item 22: leading zeros are not evaluated; empty = general."""
        if addr == "":
            return True
        return addr.lstrip("0") == self.address.lstrip("0")

    def on_char(self, ch: Char) -> None:
        c = chr(ch.value)
        if self.state == "idle":
            if c == "/":
                self.buf = ""
            self.buf += c if ch.parity_ok and ch.stop_ok else "\0"
            if self.buf.endswith("\r\n"):
                b, self.buf = self.buf, ""
                if (self.answers and b.startswith("/?") and b.endswith("!\r\n")
                        and self._addressed(b[2:-3])):
                    self.state = "ident"
                    self.at(ch.end + self.ms(self.tr_ms), self._send_ident)
        elif self.state == "ack":
            self.buf += c if ch.parity_ok and ch.stop_ok else "\0"
            if self.buf.endswith("\r\n") or len(self.buf) > 8:
                ack, self.buf = self.buf, ""
                self._readout(ack, ch.end + self.ms(self.tr_ms))

    def _send_ident(self) -> None:
        self._ident_end = self.send(identification(self.mfr, self.z, self.ident))
        self.state = "ack"
        self.buf = ""
        self._session += 1
        s = self._session
        self.at(self._ident_end + self.ms(self.tt_ms), lambda: self._timeout(s))

    def _timeout(self, session: int) -> None:
        if self.state == "ack" and session == self._session and self.buf == "":
            self._readout(None, self.now)

    def _readout(self, ack: Optional[str], when: int) -> None:
        self.state = "data"
        self.acks.append(ack)
        baud = negotiated_baud(self.z, ack)
        self.rates.append(baud)

        def go() -> None:
            self.set_baud(baud)
            end = self.send(data_message(self.block, bad_bcc=self.bad_bcc))
            self.at(end, self._done)
        self.at(when, go)

    def _done(self) -> None:
        self.set_baud(INITIAL_BAUD)
        self.state = "idle"
        self.buf = ""


class Client(SerialPort):
    """A hand-held unit reading a meter in protocol mode C (6.4.3.2).

    :meth:`read` sends the request; the identification's Z is accepted if it
    is '0' .. '6' and at most ``max_z``, else the acknowledgement asks for
    '0' (Figure 15's rejection). The acknowledgement goes ``tr_ms`` after
    the identification; the client switches right after its last stop bit
    if the two Zs are identical. :attr:`results` gets one :class:`Readout`
    per session once the BCC is in. With ``acks`` False it sends no
    acknowledgement and waits at 300 baud (Figure 13: the meter's fallback).
    """

    def __init__(self, tx: PinSpec, rx: PinSpec, clock: float, *, max_z: int = 6,
                 tr_ms: float = 210.0, z_override: Optional[str] = None,
                 acks: bool = True) -> None:
        super().__init__(tx, rx, clock)
        self.acks = acks
        self.max_z = max_z
        self.tr_ms = tr_ms
        self.z_override = z_override
        self.state = "idle"
        self.buf: List[Char] = []
        self.results: List["Readout"] = []
        self._ident = ""

    def read(self, address: str = "", start: Optional[int] = None) -> int:
        self.state = "ident"
        self.buf = []
        return self.send(request(address), start)

    def on_char(self, ch: Char) -> None:
        if self.state == "ident":
            self.buf.append(ch)
            if len(self.buf) >= 2 and chr(self.buf[-2].value) + chr(ch.value) == "\r\n":
                self._ident = "".join(chr(c.value) for c in self.buf)
                z = self._ident[4:5]
                ok = baud_of(z) is not None and int(z) <= self.max_z
                zs = self.z_override or (z if ok else "0")
                self.buf = []
                if not self.acks:
                    self.state = "data"
                    return
                self.state = "wait"
                end = self.send(ack_option(zs), ch.end + self.ms(self.tr_ms))
                baud = negotiated_baud(z, ack_option(zs))

                def switch() -> None:
                    self.set_baud(baud)
                    self.state = "data"
                self.at(end, switch)
        elif self.state == "data":
            if not self.buf and ch.value != STX:
                return
            self.buf.append(ch)
            if len(self.buf) >= 2 and self.buf[-2].value == ETX:
                chars = [c.value for c in self.buf]
                block = "".join(chr(c) for c in chars[1:-2])
                good = all(c.parity_ok and c.stop_ok for c in self.buf)
                self.results.append(Readout(self._ident, block, self.baud,
                                            bcc(chars[1:-1]) == chars[-1], good))
                self.state = "idle"
                self.set_baud(INITIAL_BAUD)


@dataclasses.dataclass
class Readout:
    ident: str                      # the identification message, CR LF included
    block: str                      # between STX and ETX: data block, '!', CR LF
    baud: int                       # the data message's rate
    bcc_ok: bool
    chars_ok: bool                  # every parity and stop bit good


class Wire(Model):
    """Records every edge of one pad: ``edges = [(cycle, level)]`` from 1."""

    def __init__(self, pin: PinSpec) -> None:
        self.pad = pad_of(pin)
        self.level = 1
        self.seen = False
        self.edges: List[Tuple[int, int]] = []

    def observe(self, lines: Lines) -> None:
        v = lines.get(self.pad)
        if v != self.level:
            self.level = v
            if self.seen:
                self.edges.append((lines.cycle, v))
        self.seen = self.seen or v == 1     # edges count once it has idled


class Link(Model):
    """Two chip ports facing each other through the air: what ``a_tx`` drove
    in cycle c reaches ``b_rx`` in cycle c + 1, and the other way round.
    ``wires`` records both directions (``a`` and ``b``, the transmitters)."""

    def __init__(self, a_tx: PinSpec, a_rx: PinSpec, b_tx: PinSpec, b_rx: PinSpec) -> None:
        self.a, self.b = Wire(a_tx), Wire(b_tx)
        self.a_rx, self.b_rx = pad_of(a_rx), pad_of(b_rx)

    def drive(self, drive: Drive, cycle: int) -> None:
        drive.set(self.b_rx, self.a.level)
        drive.set(self.a_rx, self.b.level)

    def observe(self, lines: Lines) -> None:
        self.a.observe(lines)
        self.b.observe(lines)


# --------------------------------------------------------------- checker
@dataclasses.dataclass
class Frame:
    value: int
    start: int
    end: float                      # nominal end of the stop bit
    parity_ok: bool
    stop_ok: bool
    edge_err: float                 # worst edge off the bit grid, in bits


def _level(edges, times, cycle: float) -> int:
    i = bisect.bisect_right(times, cycle) - 1
    return 1 if i < 0 else edges[i][1]


def decode(edges, clock: float, baud: int, since: float = 0,
           count: Optional[int] = None, until: Optional[float] = None) -> List[Frame]:
    """7E1 frames at ``baud`` on a recorded line, from the first falling edge
    at or after ``since``, sampled mid-bit from each start edge."""
    times = [c for c, _ in edges]
    bit = clock / baud
    out: List[Frame] = []
    i = bisect.bisect_left(times, since)
    while i < len(edges) and (count is None or len(out) < count):
        start, v = edges[i]
        if until is not None and start > until:
            break
        if v != 0:
            i += 1
            continue
        s = [_level(edges, times, start + (k + 0.5) * bit) for k in range(FRAME_BITS)]
        if s[0]:
            i += 1
            continue
        end = start + FRAME_BITS * bit
        j = bisect.bisect_left(times, end - 0.5 * bit)
        err = 0.0
        for c, _ in edges[i:j]:
            pos = (c - start) / bit
            err = max(err, abs(pos - round(pos)))
        value = sum(b << k for k, b in enumerate(s[1:8]))
        out.append(Frame(value, start, end, sum(s[1:9]) % 2 == 0, s[9] == 1, err))
        i = j
    return out


def text(frames) -> str:
    return "".join(chr(f.value) for f in frames)


def _message(frames, end_pair=b"\r\n") -> List[Frame]:
    """The frames up to and including the first CR LF."""
    for k in range(1, len(frames)):
        if frames[k - 1].value == end_pair[0] and frames[k].value == end_pair[1]:
            return frames[:k + 1]
    return list(frames)


@dataclasses.dataclass
class Session:
    """One mode C readout as seen on the two lines (:func:`check_session`)."""
    request: str = ""
    ident: str = ""
    ack: Optional[str] = None
    data: str = ""                  # the data message, STX .. BCC
    baud: int = INITIAL_BAUD        # the data message's rate (6.4.3.2)
    bcc_ok: bool = False
    tr_ident_ms: float = 0.0        # request end -> identification
    tr_ack_ms: Optional[float] = None
    tr_data_ms: float = 0.0         # acknowledgement (or identification) end -> data
    gap_max_ms: float = 0.0         # longest gap inside a message (ta)
    edge_err: float = 0.0           # worst edge off its bit grid, bits
    violations: List[str] = dataclasses.field(default_factory=list)


def check_session(client, meter, clock: float, since: float = 0) -> Session:
    """Decode one session from the client's line (``client``: edges) and the
    meter's (``meter``) and check it against 6.3, 6.4.3.2 and 6.4.3.6.

    The request and the acknowledgement are decoded at 300 baud, the
    identification too; the data message at the rate 6.4.3.2 gives for the
    identification's Z and the acknowledgement seen. A data message that is
    not at that rate does not decode, which is the baud rate check.
    """
    ses = Session()
    v = ses.violations
    ms = 1000.0 / clock
    req = _message(decode(client, clock, INITIAL_BAUD, since))
    ses.request = text(req)
    if not ses.request.endswith("\r\n"):
        v.append("no complete request message")
        return ses
    idf = _message(decode(meter, clock, INITIAL_BAUD, req[-1].end))
    ses.ident = text(idf)
    try:
        mfr, z, _ = parse_identification(ses.ident)
    except ValueError as exc:
        v.append(str(exc))
        return ses
    tr_min = reaction_min_ms(mfr)
    ident_end = idf[-1].end
    # The acknowledgement, if the client sent one before the data started.
    ack = _message(decode(client, clock, INITIAL_BAUD, ident_end))
    frames: List[Frame] = []
    if ack and ack[0].start < ident_end + TT_MIN_MS / ms:
        ses.ack = text(ack)
        ses.tr_ack_ms = (ack[0].start - ident_end) * ms
        data_from = ack[-1].end
    else:
        ack = []
        data_from = ident_end
    ses.baud = negotiated_baud(z, ses.ack)
    dat = decode(meter, clock, ses.baud, data_from)
    for k, f in enumerate(dat):
        frames.append(f)
        if k >= 1 and dat[k - 1].value == ETX:
            break
    ses.data = text(frames)
    if len(frames) < 3 or frames[0].value != STX or frames[-2].value != ETX:
        v.append("no data message at %d baud: %r" % (ses.baud, ses.data))
        return ses
    ses.bcc_ok = bcc(ses.data[1:-1]) == frames[-1].value
    if not ses.data[1:-2].endswith("!\r\n"):
        v.append("data block does not end with ! CR LF")
    ses.tr_ident_ms = (idf[0].start - req[-1].end) * ms
    ses.tr_data_ms = (frames[0].start - data_from) * ms
    # 6.4.3.6: reaction times; the fallback after tt without acknowledgement.
    if not tr_min <= ses.tr_ident_ms <= TR_MAX_MS:
        v.append("identification after %.1f ms" % ses.tr_ident_ms)
    if ses.ack is not None:
        if not tr_min <= ses.tr_ack_ms <= TR_MAX_MS:
            v.append("acknowledgement after %.1f ms" % ses.tr_ack_ms)
        if not tr_min <= ses.tr_data_ms <= TR_MAX_MS:
            v.append("data message after %.1f ms" % ses.tr_data_ms)
    elif not TT_MIN_MS < ses.tr_data_ms <= TT_MAX_MS:
        v.append("fallback data message after %.1f ms (tt)" % ses.tr_data_ms)
    for msg in (req, idf, ack, frames):
        for a, b in zip(msg, msg[1:]):
            ses.gap_max_ms = max(ses.gap_max_ms, (b.start - a.end) * ms)
        for f in msg:
            ses.edge_err = max(ses.edge_err, f.edge_err)
            if not (f.parity_ok and f.stop_ok):
                v.append("bad parity or stop bit in %r at %d" % (chr(f.value), f.start))
    if ses.gap_max_ms >= TA_MAX_MS:
        v.append("gap of %.1f ms inside a message (ta)" % ses.gap_max_ms)
    if ses.edge_err > EDGE_TOL:
        v.append("an edge %.3f bit off its grid" % ses.edge_err)
    return ses
