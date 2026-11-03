"""Modbus RTU reference models: frames and their CRC, a reference server, a
line transmitter and receiver with a character-timing checker, error
injection, and an RS-485 half-duplex bus.

Written from the specifications, not from the firmware, for
``firmware/modbus_server.loom`` and ``firmware/modbus_client.loom`` (D-035):

* SL: "MODBUS over Serial Line Specification and Implementation Guide
  V1.02" (modbus.org, 2006).
* AP: "MODBUS Application Protocol Specification V1.1b3" (modbus.org, 2012).

What is taken from where:

=============================  ==============================================
figure                         source
=============================  ==============================================
11-bit character: start, 8     SL 2.5.1 (RTU transmission mode): even parity
data LSB first, parity, stop;  the default, odd allowed, no parity with a
no parity: two stop bits       second stop bit
silences: t3.5 between         SL 2.5.1.1 (RTU message framing) and its
frames, at most t1.5 between   figures 13 and 14; above 19200 baud the fixed
characters of a frame          values 750 us (t1.5) and 1750 us (t3.5)
a frame with a silence over    SL 2.5.1.1, figure 14: "the frame is NOK"
t1.5 is discarded              and is dropped at t3.5
CRC-16: init 0xFFFF,           SL 2.5.1.2 (CRC checking) and appendix B
polynomial 0xA001 reflected,   (CRC generation; the table method there and
no final XOR, low byte first   the bitwise method described in the text)
addresses: 0 broadcast,        SL 2.2 (Modbus addressing rules); a broadcast
1 .. 247 servers               is never answered (SL 2.1)
ADU at most 256 bytes          SL 2.5.1.1 (and AP 4.1: PDU at most 253)
03 read holding registers:     AP 6.3: quantity 1 .. 125 (0x7D), else
                               exception 03; start + quantity in range,
                               else 02; response: byte count 2N, then the
                               registers high byte first (AP 4.2)
06 write single register       AP 6.6: register in range, else 02; the
                               response echoes the request
16 write multiple registers    AP 6.12: quantity 1 .. 123 (0x7B) and byte
                               count 2N, else 03; range, else 02; the
                               response is the request's first five fields
exceptions                     AP 7: function | 0x80 and a code: 01 illegal
                               function, 02 illegal data address, 03 illegal
                               data value
=============================  ==============================================

Section numbers were checked against the V1.02 and V1.1b3 texts as the
author knows them; the figure numbers (13, 14) are unchecked. The length
rules for 03 and 06 (a request of exactly 8 bytes) are this model's
reading: the AP state diagrams check values, not lengths, so a request of
the wrong length is answered with exception 03, the safest reading (see
``QUESTIONS.md`` of W-005).

Times are core clocks on the bench (:data:`CLOCK`, 50 MHz). A bit is
``CLOCK / baud`` clocks, a fraction in general; edges a model drives are
placed at ``round(t)`` of exact fractional times, so nothing drifts.
"""

from __future__ import annotations

import dataclasses
from collections import deque
from typing import Dict, List, Optional, Sequence, Tuple

from .bench import Drive, Lines, Model, PinSpec, pad_of

CLOCK = 50e6                        # the datasheet's clock, Hz
CHAR_BITS = 11                      # SL 2.5.1: every RTU character
EVEN, ODD, NONE = "even", "odd", "none"
BROADCAST = 0
MAX_ADU = 256                       # SL 2.5.1.1
FIXED_T15_S = 750e-6                # SL 2.5.1.1, above 19200 baud
FIXED_T35_S = 1750e-6

READ_HOLDING, WRITE_SINGLE, WRITE_MULTIPLE = 3, 6, 16
ILLEGAL_FUNCTION, ILLEGAL_ADDRESS, ILLEGAL_VALUE = 1, 2, 3
MAX_READ, MAX_WRITE = 125, 123      # AP 6.3, 6.12


# ------------------------------------------------------------------- CRC
def crc16(data: Sequence[int]) -> int:
    """The Modbus CRC-16 by the bitwise procedure of SL 2.5.1.2: load
    0xFFFF; for each byte XOR it into the low byte; then eight times shift
    right, and if the bit shifted out was 1 XOR 0xA001."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte & 0xFF
        for _ in range(8):
            lsb = crc & 1
            crc >>= 1
            if lsb:
                crc ^= 0xA001
    return crc


def encode_frame(address: int, pdu: Sequence[int]) -> List[int]:
    """An RTU ADU: address, PDU, CRC low byte then high byte (SL 2.5.1.2)."""
    body = [address & 0xFF] + [b & 0xFF for b in pdu]
    crc = crc16(body)
    return body + [crc & 0xFF, crc >> 8]


def crc_ok(frame: Sequence[int]) -> bool:
    """True for a frame of 4 bytes or more whose last two are its CRC."""
    if len(frame) < 4:
        return False
    crc = crc16(frame[:-2])
    return frame[-2] == (crc & 0xFF) and frame[-1] == (crc >> 8)


def decode_frame(frame: Sequence[int]) -> Optional[Tuple[int, List[int]]]:
    """``(address, pdu)`` of a good frame, else None."""
    if not crc_ok(frame):
        return None
    return frame[0], list(frame[1:-2])


def bad_crc(frame: Sequence[int]) -> List[int]:
    """Error injection: the same frame with its CRC's high byte flipped."""
    out = list(frame)
    out[-1] ^= 0xFF
    return out


# ------------------------------------------------------------------ PDUs
def word(value: int) -> List[int]:
    """A 16-bit field, high byte first (AP 4.2)."""
    return [(value >> 8) & 0xFF, value & 0xFF]


def read_holding(start: int, quantity: int) -> List[int]:
    return [READ_HOLDING] + word(start) + word(quantity)


def write_single(register: int, value: int) -> List[int]:
    return [WRITE_SINGLE] + word(register) + word(value)


def write_multiple(start: int, values: Sequence[int],
                   quantity: Optional[int] = None,
                   byte_count: Optional[int] = None) -> List[int]:
    """``quantity`` and ``byte_count`` override the true ones, to inject a
    request AP 6.12 calls bad."""
    n = len(values) if quantity is None else quantity
    data = [b for v in values for b in word(v)]
    count = len(data) if byte_count is None else byte_count
    return [WRITE_MULTIPLE] + word(start) + word(n) + [count & 0xFF] + data


class ReferenceServer:
    """A Modbus server with ``size`` holding registers at 0 .. size - 1,
    as AP 6.3, 6.6, 6.12 and 7 specify it (length rules: module docstring).

    :meth:`handle` takes a whole frame and returns the response frame, or
    None when SL says nothing is sent: a bad CRC, a frame under 4 bytes,
    another address, or a broadcast (which is still executed)."""

    def __init__(self, address: int, size: int = 16,
                 registers: Optional[Sequence[int]] = None) -> None:
        self.address = address
        self.size = size
        self.registers = list(registers) if registers is not None else [0] * size
        assert len(self.registers) == size

    def _exception(self, function: int, code: int) -> List[int]:
        return [function | 0x80, code]

    def pdu(self, request: Sequence[int], length: int) -> List[int]:
        """The response PDU to a request PDU from a frame of ``length``
        bytes (CRC included)."""
        fc = request[0]
        if fc not in (READ_HOLDING, WRITE_SINGLE, WRITE_MULTIPLE):
            return self._exception(fc, ILLEGAL_FUNCTION)
        if length < 8:
            return self._exception(fc, ILLEGAL_VALUE)
        first = (request[1] << 8) | request[2]
        second = (request[3] << 8) | request[4]
        if fc == WRITE_SINGLE:
            if length != 8:
                return self._exception(fc, ILLEGAL_VALUE)
            if first >= self.size:
                return self._exception(fc, ILLEGAL_ADDRESS)
            self.registers[first] = second
            return list(request[:5])
        if fc == READ_HOLDING:
            if length != 8 or not 1 <= second <= MAX_READ:
                return self._exception(fc, ILLEGAL_VALUE)
            if first >= self.size or first + second > self.size:
                return self._exception(fc, ILLEGAL_ADDRESS)
            data = [b for r in self.registers[first:first + second] for b in word(r)]
            return [fc, len(data)] + data
        if not 1 <= second <= MAX_WRITE or request[5] != 2 * second \
                or length != 9 + 2 * second:
            return self._exception(fc, ILLEGAL_VALUE)
        if first >= self.size or first + second > self.size:
            return self._exception(fc, ILLEGAL_ADDRESS)
        for i in range(second):
            self.registers[first + i] = (request[6 + 2 * i] << 8) | request[7 + 2 * i]
        return list(request[:5])

    def handle(self, frame: Sequence[int]) -> Optional[List[int]]:
        decoded = decode_frame(frame)
        if decoded is None:
            return None
        address, request = decoded
        if address not in (self.address, BROADCAST) or not request:
            return None
        response = self.pdu(request, len(frame))
        if address == BROADCAST:
            return None
        return encode_frame(self.address, response)


# ---------------------------------------------------------------- timing
def bit_clocks(baud: float) -> float:
    return CLOCK / baud


def silences(baud: float) -> Tuple[float, float]:
    """``(t1.5, t3.5)`` in clocks, SL 2.5.1.1: 1.5 and 3.5 character times,
    or 750 us and 1750 us above 19200 baud."""
    if baud > 19200:
        return FIXED_T15_S * CLOCK, FIXED_T35_S * CLOCK
    char = CHAR_BITS * bit_clocks(baud)
    return 1.5 * char, 3.5 * char


def char_levels(value: int, parity: str = EVEN, bad_parity: bool = False) -> List[int]:
    """The eleven line levels of one character (SL 2.5.1). ``bad_parity``
    inverts the parity bit (without parity: the first stop bit goes low)."""
    data = [(value >> i) & 1 for i in range(8)]
    ones = sum(data)
    if parity == EVEN:
        ninth = ones & 1
    elif parity == ODD:
        ninth = (ones & 1) ^ 1
    elif parity == NONE:
        ninth = 1
    else:
        raise ValueError("parity %r" % parity)
    if bad_parity:
        ninth ^= 1
    return [0] + data + [ninth, 1]


def parity_ok(levels: Sequence[int], parity: str) -> bool:
    """Bit 9 of a received character is right for ``parity``."""
    return char_levels(sum(b << i for i, b in enumerate(levels[1:9])), parity)[9] == levels[9]


# ------------------------------------------------------------ line models
class RtuTx(Model):
    """Drives RTU frames onto a pad (a model client or server on a chip
    input). Characters go back to back unless told otherwise.

    :meth:`send` queues one frame and returns the cycle its first start bit
    falls. Error injection: ``gaps`` maps a character index to extra
    silence, in clocks, after that character (a silence over t1.5 inside the
    frame); ``bad_parity`` lists characters whose parity bit is inverted.
    A bad CRC is a frame built with :func:`bad_crc`. ``bit`` is clocks per
    bit, so a rate error is a ``bit`` a little off the nominal.
    """

    def __init__(self, pin: PinSpec, bit: float, parity: str = EVEN,
                 idle: int = 1) -> None:
        self.pad = pad_of(pin)
        self.bit = bit
        self.parity = parity
        self.level = idle
        self.now = 0
        self._changes: deque = deque()
        self._free_at = 0.0
        self.sent: List[Tuple[int, List[int]]] = []

    def send(self, frame: Sequence[int], *, start: Optional[float] = None,
             gaps: Optional[Dict[int, float]] = None,
             bad_parity: Sequence[int] = (), bit: Optional[float] = None,
             parity: Optional[str] = None) -> int:
        bit = self.bit if bit is None else bit
        parity = self.parity if parity is None else parity
        t = max(self._free_at, float(self.now + 2))
        if start is not None:
            t = max(t, float(start))
        t0 = t
        for i, value in enumerate(frame):
            for k, level in enumerate(char_levels(value, parity, i in bad_parity)):
                self._changes.append((int(round(t + k * bit)), level))
            t += CHAR_BITS * bit
            self._changes.append((int(round(t)), 1))
            if gaps and i in gaps:
                t += gaps[i]
        self._free_at = t
        self.sent.append((int(round(t0)), list(frame)))
        return int(round(t0))

    @property
    def busy(self) -> bool:
        return bool(self._changes)

    @property
    def free_at(self) -> int:
        """The cycle the last queued frame's last stop bit ends."""
        return int(round(self._free_at))

    def drive(self, drive: Drive, cycle: int) -> None:
        self.now = cycle
        changes = self._changes
        while changes and changes[0][0] <= cycle:
            self.level = changes.popleft()[1]
        drive.set(self.pad, self.level)


@dataclasses.dataclass
class Char:
    value: int
    start: int                      # cycle the start bit was first seen low
    end: float                      # start + 11 bits: the stop bit's end
    levels: List[int]               # the eleven sampled levels
    edges: List[int]                # cycles of every edge inside the character

    def parity_ok(self, parity: str) -> bool:
        return parity_ok(self.levels, parity)

    @property
    def stop_ok(self) -> bool:
        return self.levels[10] == 1


@dataclasses.dataclass
class Frame:
    chars: List[Char]
    gap_max: float                  # the longest silence between two characters
    silence_before: Optional[float]     # since the last frame's end (None: first)

    @property
    def data(self) -> List[int]:
        return [c.value for c in self.chars]

    @property
    def start(self) -> int:
        return self.chars[0].start

    @property
    def end(self) -> float:
        return self.chars[-1].end


class RtuRx(Model):
    """Decodes and times the RTU characters on a pad (a chip output) or on
    an :class:`Rs485Bus`, and the driver enable on a second pad.

    A character starts at the first cycle the line is seen low after being
    high; its eleven bits are sampled at their middles from there (with the
    nominal ``bit``). Every edge inside it is kept for the timing check.
    """

    def __init__(self, pin: Optional[PinSpec] = None, bit: float = 1.0, *,
                 bus: Optional["Rs485Bus"] = None,
                 de: Optional[PinSpec] = None) -> None:
        if (pin is None) == (bus is None):
            raise ValueError("give a pin or a bus")
        self.pad = None if pin is None else pad_of(pin)
        self.bus = bus
        self.de_pad = None if de is None else pad_of(de)
        self.bit = bit
        self.chars: List[Char] = []
        self.de_edges: List[Tuple[int, int]] = []
        self._prev = 1
        self._de = 0
        self._cur: Optional[Char] = None

    def observe(self, lines: Lines) -> None:
        c = lines.cycle
        level = self.bus.level if self.bus is not None else lines.get(self.pad)
        if self.de_pad is not None:
            d = lines.get(self.de_pad)
            if d != self._de:
                self.de_edges.append((c, d))
                self._de = d
        cur = self._cur
        if cur is None:
            if level == 0 and self._prev == 1:
                self._cur = Char(0, c, c + CHAR_BITS * self.bit, [], [c])
        else:
            if level != self._prev:
                cur.edges.append(c)
            k = len(cur.levels)
            if c >= cur.start + (k + 0.5) * self.bit:
                cur.levels.append(level)
                if len(cur.levels) == CHAR_BITS:
                    cur.value = sum(b << i for i, b in enumerate(cur.levels[1:9]))
                    self.chars.append(cur)
                    self._cur = None
        self._prev = level

    # ------------------------------------------------------------ frames
    def frames(self, t35: float, since: int = 0) -> List[Frame]:
        """The characters from cycle ``since`` on, split into frames at
        every silence of ``t35`` clocks or more (SL 2.5.1.1)."""
        out: List[Frame] = []
        last_end: Optional[float] = None
        for ch in (c for c in self.chars if c.start >= since):
            silence = None if last_end is None else ch.start - last_end
            if silence is None or silence >= t35:
                out.append(Frame([ch], 0.0, silence))
            else:
                frame = out[-1]
                frame.chars.append(ch)
                frame.gap_max = max(frame.gap_max, silence)
            last_end = ch.end
        return out

    def violations(self, frame: Frame, parity: str, t15: float, t35: float,
                   edge_tol: float) -> List[str]:
        """Everything about ``frame`` that SL 2.5.1 and 2.5.1.1 forbid a
        sender: a wrong parity or stop bit, an edge more than ``edge_tol``
        clocks off its bit boundary, a silence over t1.5 inside the frame,
        under t3.5 before it; and, with a DE pad, a driver that was not on
        from before the first start bit to the end of the last stop bit."""
        bad = []
        for i, ch in enumerate(frame.chars):
            if not ch.parity_ok(parity):
                bad.append("char %d: parity bit" % i)
            if not ch.stop_ok:
                bad.append("char %d: stop bit low" % i)
            for e in ch.edges:
                off = (e - ch.start) / self.bit
                err = abs(off - round(off)) * self.bit
                if err > edge_tol:
                    bad.append("char %d: edge at +%d clocks is %.1f off" % (i, e - ch.start, err))
        if frame.gap_max > t15:
            bad.append("silence of %.0f clocks inside the frame, t1.5 is %.0f"
                       % (frame.gap_max, t15))
        if frame.silence_before is not None and frame.silence_before < t35:
            bad.append("silence of %.0f clocks before the frame, t3.5 is %.0f"
                       % (frame.silence_before, t35))
        if self.de_pad is not None:
            before = [d for c, d in self.de_edges if c < frame.start]
            off = [c for c, d in self.de_edges if d == 0 and c > frame.start]
            if not before or before[-1] != 1:
                bad.append("DE not on before the first start bit")
            elif off and off[0] < frame.end - edge_tol:
                bad.append("DE off at %d, before the last stop bit ends at %.0f"
                           % (off[0], frame.end))
        return bad

    def de_window(self, frame: Frame) -> Tuple[Optional[int], Optional[int]]:
        """``(rise, fall)``: the DE edges around ``frame`` (None if absent)."""
        on = [c for c, d in self.de_edges if d == 1 and c <= frame.start]
        off = [c for c, d in self.de_edges if d == 0 and c > frame.start]
        return (on[-1] if on else None), (off[0] if off else None)


# ------------------------------------------------------------------- bus
class Rs485Bus(Model):
    """A two-wire RS-485 bus (SL 3.3) between chip pins, one clock late like
    every bench model. Each chip node is ``(tx, de)``, two chip outputs; the
    bus carries the TX of the node whose DE is on, a model transmitter's
    level while it is busy, and mark (1) when nobody drives it, which is
    what SL 3.4's line polarisation gives. Two drivers at once are
    recorded in :attr:`contentions` (cycle, number of drivers). The bus
    level is driven onto every pad in ``receivers`` (the chips' RX inputs).
    """

    def __init__(self, nodes: Sequence[Tuple[PinSpec, PinSpec]],
                 receivers: Sequence[PinSpec],
                 models: Sequence[RtuTx] = ()) -> None:
        self.nodes = [(pad_of(tx), pad_of(de)) for tx, de in nodes]
        self.receivers = [pad_of(p) for p in receivers]
        self.models = list(models)
        self.level = 1
        self.contentions: List[Tuple[int, int]] = []

    def drive(self, drive: Drive, cycle: int) -> None:
        for pad in self.receivers:
            drive.set(pad, self.level)

    def observe(self, lines: Lines) -> None:
        levels = [lines.get(tx) for tx, de in self.nodes if lines.get(de)]
        levels += [m.level for m in self.models if m.busy or m.level == 0]
        if len(levels) > 1:
            self.contentions.append((lines.cycle, len(levels)))
        self.level = min(levels) if levels else 1


class ModelServer(Model):
    """A model server on the chip's client pins: an :class:`RtuRx` on the
    client's TX, a :class:`ReferenceServer`, and an :class:`RtuTx` that
    answers on the client's RX ``delay`` clocks after the request's frame
    end (t3.5 of silence, SL 2.5.1.1). ``fault`` injects an error into
    the next responses: None, "silent" (no response: a timeout), "bad_crc",
    "gap" (a silence of ``gap`` clocks after the response's second
    character) or "bad_parity" (the third character's parity bit inverted).
    """

    def __init__(self, tx_of_client: PinSpec, rx_of_client: PinSpec, bit: float,
                 server: ReferenceServer, t35: float, *, parity: str = EVEN,
                 delay: float = 0.0) -> None:
        self.rx = RtuRx(tx_of_client, bit)
        self.tx = RtuTx(rx_of_client, bit, parity)
        self.server = server
        self.t35 = t35
        self.delay = delay
        self.fault: Optional[str] = None
        self.gap = 0.0
        self.requests: List[List[int]] = []
        self._seen = 0

    def drive(self, drive: Drive, cycle: int) -> None:
        self.tx.drive(drive, cycle)

    def observe(self, lines: Lines) -> None:
        self.rx.observe(lines)
        chars = self.rx.chars
        if len(chars) == self._seen or self.rx._cur is not None:
            return
        if lines.cycle < chars[-1].end + self.t35:
            return
        new = chars[self._seen:]
        self._seen = len(chars)
        if not all(c.stop_ok and c.parity_ok(self.tx.parity) for c in new):
            return                  # SL 2.5.1: a bad character spoils the frame
        frame = [c.value for c in new]
        self.requests.append(frame)
        reply = self.server.handle(frame)
        if reply is None or self.fault == "silent":
            return
        start = lines.cycle + 1 + self.delay
        if self.fault == "bad_crc":
            self.tx.send(bad_crc(reply), start=start)
        elif self.fault == "gap":
            self.tx.send(reply, start=start, gaps={1: self.gap})
        elif self.fault == "bad_parity":
            self.tx.send(reply, start=start, bad_parity=[2])
        else:
            self.tx.send(reply, start=start)
