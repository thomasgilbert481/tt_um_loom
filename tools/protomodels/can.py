"""CAN 2.0A reference models: a bus and a node, for firmware/can_loopback.loom.

Written from ISO 11898-1 (classical CAN, base frame format), not from the
firmware or the engine:

* a data or remote frame is SOF, the 11-bit identifier, RTR, IDE, r0, the
  4-bit DLC, 0 to 8 data bytes (none for a remote frame; DLC 9..15 means 8),
  the CRC15 sequence, then the fixed-form CRC delimiter, ACK slot, ACK
  delimiter and seven EOF bits, then three bits of intermission, all MSB
  first, dominant = 0;
* CRC15: generator x^15+x^14+x^10+x^8+x^7+x^4+x^3+1 (0x4599), register
  initialised to 0, over the destuffed bits from SOF to the end of the data
  field;
* bit stuffing from SOF to the end of the CRC sequence: after five equal
  bits the transmitter inserts one of the other value, and that stuff bit
  counts as the first bit of the next run; a receiver that samples a sixth
  equal bit there has a stuff error;
* the ACK: every receiver that found the CRC right drives the ACK slot
  dominant; the transmitter sends it recessive and reads it back.

:class:`CanBus` is the bench model (``tools.protomodels.bench.Model``): the bus
level is the wired AND of the chip's TX pin and the node's driver, and that
level is what the chip's RX pin sees, one clock after the chip drove it (the
loop delay of a transceiver, at its shortest). :class:`CanNode` is the node on
it: it receives every frame, ACKs the good ones unless told to stay silent,
sends frames of its own on request, can spoil one of them with a stuff
error or a CRC error, and can drive one bit of another node's frame
dominant (a form error when the bit is in a fixed-form field). It keeps its own bit clock (hard synchronisation on each
start of frame, as a receiver must), samples at ``sample`` of the bit, and
records how far every edge of a frame lies from its nominal bit boundary, so a
test can hold the transmitter to its bit time.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

from tools.protomodels.bench import Drive, Lines, Model, PinSpec, pad_of

POLY = 0x4599               # CRC15 generator without the x^15 term
CLOCK = 50_000_000          # the chip's clock in the tests
EOF_BITS = 7
INTERMISSION = 3
IDLE_BITS = 11              # recessive bits a node counts as "bus idle"


def crc15(bits: Sequence[int]) -> int:
    """The CAN CRC15 of ``bits`` (register initialised to 0), ISO 11898-1."""
    crc = 0
    for b in bits:
        fb = ((crc >> 14) & 1) ^ (b & 1)
        crc = (crc << 1) & 0x7FFF
        if fb:
            crc ^= POLY
    return crc


def stuff(bits: Sequence[int]) -> List[int]:
    """``bits`` with a complementary bit after every run of five equal bits;
    the stuff bit starts the next run."""
    out: List[int] = []
    run, last = 0, None
    for b in bits:
        out.append(b)
        run = run + 1 if b == last else 1
        last = b
        if run == 5:
            out.append(1 - b)
            run, last = 1, 1 - b
    return out


def data_length(dlc: int, rtr: bool) -> int:
    """Bytes in the data field: none for a remote frame, else min(DLC, 8)."""
    return 0 if rtr else min(dlc, 8)


@dataclass
class Frame:
    """A base-format frame. ``dlc`` defaults to ``len(data)``."""

    ident: int
    data: bytes = b""
    rtr: bool = False
    dlc: Optional[int] = None

    def __post_init__(self) -> None:
        self.data = bytes(self.data)
        if self.dlc is None:
            self.dlc = len(self.data)
        assert 0 <= self.ident < 1 << 11 and 0 <= self.dlc < 16
        assert len(self.data) == data_length(self.dlc, self.rtr), self

    def header(self) -> int:
        """The firmware's header word: {ID[10:0], RTR, DLC[3:0]}."""
        return (self.ident << 5) | (int(self.rtr) << 4) | self.dlc

    def words(self) -> List[int]:
        """The data bytes two to a word, first byte in 15:8, an odd last byte
        in 15:8 with 7:0 = 0 (the firmware's INQ and OUTQ format)."""
        d = self.data + (b"\x00" if len(self.data) % 2 else b"")
        return [(d[i] << 8) | d[i + 1] for i in range(0, len(d), 2)]

    def bits(self) -> List[int]:
        """SOF to the end of the data field, destuffed, MSB first."""
        out = [0]
        out += [(self.ident >> (10 - i)) & 1 for i in range(11)]
        out += [int(self.rtr), 0, 0]                    # RTR, IDE, r0
        out += [(self.dlc >> (3 - i)) & 1 for i in range(4)]
        for byte in self.data:
            out += [(byte >> (7 - i)) & 1 for i in range(8)]
        return out

    def crc(self) -> int:
        return crc15(self.bits())


def encode(frame: Frame, *, crc_error: bool = False,
           stuff_error: bool = False) -> Tuple[List[int], int]:
    """The bits a transmitter sends, SOF to the last EOF bit, and the index of
    the ACK slot among them.

    ``crc_error`` sends the CRC with its last bit inverted (stuffed as sent,
    so only the CRC check can find it). ``stuff_error`` inverts the first
    stuff bit, which makes a run of six equal bits; the frame must need one.
    """
    crc = frame.crc() ^ (1 if crc_error else 0)
    body = frame.bits() + [(crc >> (14 - i)) & 1 for i in range(15)]
    sent = stuff(body)
    if stuff_error:
        first = _first_stuff_bit(body)
        assert first is not None, "the frame has no stuff bit to spoil"
        sent[first] ^= 1
    ack_slot = len(sent) + 1
    return sent + [1, 1, 1] + [1] * EOF_BITS, ack_slot


def _first_stuff_bit(body: Sequence[int]) -> Optional[int]:
    """Index in the stuffed stream of the first stuff bit, if any."""
    run, last = 0, None
    for i, b in enumerate(body):
        run = run + 1 if b == last else 1
        last = b
        if run == 5:
            return i + 1
    return None


# ------------------------------------------------------------------ receive
@dataclass
class Received:
    """One frame as the node saw it on the bus (its own frames included).

    ``frame`` is ``None`` when decoding stopped before the control field was
    complete. ``edges`` holds ``(bit, offset)`` for every edge from SOF to the
    end of EOF: the bit it starts and its distance in clocks from that bit's
    nominal start, counted from the hard synchronisation at SOF."""

    start: int
    own: bool = False
    frame: Optional[Frame] = None
    crc_ok: bool = False
    stuff_error: bool = False
    form_error: bool = False
    extended: bool = False
    acked: bool = False
    bit_error: bool = False
    complete: bool = False
    edges: List[Tuple[int, int]] = field(default_factory=list)

    @property
    def clean(self) -> bool:
        return (self.complete and self.crc_ok and not self.stuff_error
                and not self.form_error and not self.bit_error)


class _Decoder:
    """Destuffs and checks sampled bits; ``feed`` returns an event name or
    ``None``: ``"error"`` (decoding stops), ``"delim"`` (the CRC delimiter
    was just sampled: the ACK slot is next), ``"done"`` (EOF complete)."""

    def __init__(self, rec: Received) -> None:
        self.rec = rec
        self.body: List[int] = []
        self.run, self.last = 0, -1
        self.need: Optional[int] = None
        self.stage = "stuffed"
        self.eof = 0

    def feed(self, b: int) -> Optional[str]:
        rec = self.rec
        if self.stage == "stuffed":
            if self.run == 5:                       # a stuff bit is due
                if b == self.last:
                    rec.stuff_error = True
                    return "error"
                self.run, self.last = 1, b
                if self.need is not None and len(self.body) == self.need:
                    self.stage = "crc_delim"
                return None
            self.body.append(b)
            self.run = self.run + 1 if b == self.last else 1
            self.last = b
            n = len(self.body)
            if n == 19:                             # SOF .. DLC
                if self.body[13]:
                    rec.extended = True
                    return "error"
                rtr = bool(self.body[12])
                dlc = _value(self.body[15:19])
                self.need = 19 + 8 * data_length(dlc, rtr) + 15
            if n == self.need and self.run != 5:
                self.stage = "crc_delim"
            return None
        if self.stage == "crc_delim":
            self._finish_body()
            if not b:
                rec.form_error = True
                return "error"
            self.stage = "ack"
            return "delim"
        if self.stage == "ack":
            rec.acked = not b
            self.stage = "ack_delim"
            return None
        if self.stage == "ack_delim":
            if not b:
                rec.form_error = True
                return "error"
            self.stage = "eof"
            return None
        self.eof += 1                               # EOF
        if not b and self.eof < EOF_BITS:           # the 7th: no receiver error
            rec.form_error = True
            return "error"
        if self.eof == EOF_BITS:
            rec.complete = True
            return "done"
        return None

    def _finish_body(self) -> None:
        body, rec = self.body, self.rec
        rtr = bool(body[12])
        dlc = _value(body[15:19])
        n = data_length(dlc, rtr)
        data = bytes(_value(body[19 + 8 * i:27 + 8 * i]) for i in range(n))
        rec.frame = Frame(_value(body[1:12]), data, rtr, dlc)
        rec.crc_ok = crc15(body[:-15]) == _value(body[-15:])


def _value(bits: Sequence[int]) -> int:
    v = 0
    for b in bits:
        v = (v << 1) | b
    return v


# --------------------------------------------------------------------- node
@dataclass
class Sent:
    """A frame the node was asked to send, and what became of it."""

    frame: Frame
    crc_error: bool = False
    stuff_error: bool = False
    start: Optional[int] = None         # cycle its SOF went out
    acked: Optional[bool] = None        # the ACK slot read back dominant
    done: bool = False                  # the last EOF bit has gone out


class CanNode:
    """A CAN node with its own bit clock, stepped by :class:`CanBus`.

    Args:
        rate: bit rate; ``clock / rate`` must be a whole number of clocks.
        sample: the sample point as a fraction of the bit (ISO 11898-1 leaves
            it to the system; 75 % is the usual choice).
        ack: ACK every frame of another node whose CRC is right.

    It sends no error flags: an error stops its decoding, as it stops the
    firmware's, and it waits for 11 recessive bits before the next SOF. It
    starts a frame of its own only after 11 recessive bits too, and does not
    arbitrate: a test must not make it and the chip send at once.
    """

    def __init__(self, rate: int, *, clock: int = CLOCK, sample: float = 0.75,
                 ack: bool = True) -> None:
        assert clock % rate == 0, "a whole number of clocks per bit"
        self.period = clock // rate
        self.sample_at = int(round(sample * self.period))
        self.ack = ack
        self.tx = 1                             # the driver in the next cycle
        self.frames: List[Received] = []        # every frame seen, own included
        self.sent: List[Sent] = []
        self._queue: List[Sent] = []
        self._cur: Optional[Sent] = None
        self._tx_bits: Optional[List[int]] = None
        self._tx_t0 = self._tx_ack = 0
        self._ack_from = self._ack_to = -1
        self._disturb_bit: Optional[int] = None
        self._dist_from = self._dist_to = -1
        self._idle = 0
        self._integrated = False
        self._prev = 1
        self._rec: Optional[Received] = None
        self._dec: Optional[_Decoder] = None
        self._t0 = self._bit = 0
        self._next_sample = -1

    def disturb(self, bit: int) -> None:
        """Drive bit ``bit`` (counted from SOF = 0) of the next frame of
        another node dominant: in a fixed-form field that is a form error for
        every receiver, the transmitter's own included."""
        self._disturb_bit = bit

    def send(self, frame: Frame, *, crc_error: bool = False,
             stuff_error: bool = False) -> Sent:
        """Queue ``frame``; it goes out after 11 recessive bits."""
        s = Sent(frame, crc_error, stuff_error)
        self._queue.append(s)
        self.sent.append(s)
        return s

    @property
    def busy(self) -> bool:
        """A frame is being received or sent, or one is queued."""
        return (self._rec is not None or self._tx_bits is not None
                or bool(self._queue))

    def received(self) -> List[Received]:
        """The frames of other nodes (the chip's), in order."""
        return [r for r in self.frames if not r.own]

    # ---------------------------------------------------------------- clock
    def clock(self, bus: int, cycle: int) -> None:
        """Take the bus level of ``cycle``; set :attr:`tx` for ``cycle + 1``."""
        P = self.period
        if bus != self._prev:
            if self._rec is None:
                if not bus and self._integrated:
                    self._start(cycle)
            else:
                rel = cycle - self._t0
                off = rel % P
                if off >= P // 2:
                    off -= P
                self._rec.edges.append(((rel + P // 2) // P, off))
        self._prev = bus
        self._idle = self._idle + 1 if bus else 0
        if self._idle >= IDLE_BITS * P:
            self._integrated = True
        if self._rec is not None and cycle == self._next_sample:
            self._sample(bus)
        if (self._tx_bits is not None
                and cycle == self._tx_t0 + self._tx_ack * P + self.sample_at):
            self._cur.acked = not bus
        nxt = cycle + 1
        level = 1
        if self._tx_bits is not None:
            i = (nxt - self._tx_t0) // P
            if i < len(self._tx_bits):
                level = self._tx_bits[i]
            else:
                self._cur.done = True
                self._tx_bits = self._cur = None
        elif self._queue and self._rec is None and self._idle >= IDLE_BITS * P:
            self._cur = self._queue.pop(0)
            self._tx_bits, self._tx_ack = encode(
                self._cur.frame, crc_error=self._cur.crc_error,
                stuff_error=self._cur.stuff_error)
            self._tx_t0 = self._cur.start = nxt
            level = self._tx_bits[0]
        if self._ack_from <= nxt < self._ack_to or self._dist_from <= nxt < self._dist_to:
            level = 0
        self.tx = level

    def _start(self, cycle: int) -> None:
        """Hard synchronisation on a start of frame."""
        own = self._tx_bits is not None and self._tx_t0 == cycle
        self._rec = Received(start=cycle, own=own)
        self._dec = _Decoder(self._rec)
        self._t0, self._bit = cycle, 0
        self._next_sample = cycle + self.sample_at
        if not own and self._disturb_bit is not None:
            self._dist_from = cycle + self._disturb_bit * self.period
            self._dist_to = self._dist_from + self.period
            self._disturb_bit = None

    def _sample(self, bus: int) -> None:
        rec = self._rec
        if rec.own and self._tx_bits is not None and self._bit != self._tx_ack:
            if self._bit < len(self._tx_bits) and bus != self._tx_bits[self._bit]:
                rec.bit_error = True
        event = self._dec.feed(bus)
        self._bit += 1
        self._next_sample += self.period
        if event == "delim":
            if not rec.own and self.ack and rec.crc_ok:
                self._ack_from = self._t0 + self._bit * self.period
                self._ack_to = self._ack_from + self.period
        elif event is not None:
            self.frames.append(rec)
            self._rec = self._dec = None
            if event == "error":
                self._integrated = False


# ---------------------------------------------------------------------- bus
class CanBus(Model):
    """The bus on the bench: the chip's RX pin sees the wired AND of its TX pin
    and the node's driver, one clock after the chip drove it.

    ``drive`` may use only registered state (``bench.py``), so the chip's TX
    is taken from the lines of the cycle before: that clock is the loop delay
    of the transceiver pair. The node sees the same level as the chip's RX.
    """

    def __init__(self, node: CanNode, tx: PinSpec = "OUT0",
                 rx: PinSpec = "IN0") -> None:
        self.node = node
        self.tx_pad = pad_of(tx)
        self.rx_pad = pad_of(rx)
        self._chip = 1

    def drive(self, drive: Drive, cycle: int) -> None:
        drive.set(self.rx_pad, self._chip & self.node.tx)

    def observe(self, lines: Lines) -> None:
        self._chip = lines.get(self.tx_pad)
        self.node.clock(lines.get(self.rx_pad), lines.cycle)
