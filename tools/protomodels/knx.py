"""KNX TP1 reference models for firmware/knx_device.loom.

Written from the KNX TP1 physical and data link layer as public sources
describe them, never from the firmware. The KNX standard itself (KNX System
Specifications, Volume 3 "System Specifications", Part 3 "Communication",
Chapter 2 "Physical Layer Type TP1" and Chapter 3 "Data Link Layer General")
is sold to members and was not at hand. Every figure names the public
source it is taken from; the page and figure numbers are given as the
author remembers them and are all UNCHECKED (no copy was opened while this
was written), and so are the figures marked UNCHECKED on their own
(the readings taken are listed in knx_device.loom's header):

* Bit coding and rate. onsemi NCN5120/D, "Transceiver for KNX Twisted Pair
  Networks", the section on the analog (transparent) mode and its figure of
  a KNX bit; Siemens "TP-UART 2-IC" technical data, the KNX bus signal
  figure: 9600 bit/s (104.17 us a bit); a logical 0 is an active pulse at
  the start of the bit, 35 us long, followed by the equalisation pulse that
  the transceiver handles and the logic level does not see; a logical 1 is
  no pulse. 0 dominates: a 0 from any device wins over a 1 (the bus is a
  wired AND at logic level). UNCHECKED: the 33 to 37 us accepted here for a
  sent pulse's width (:data:`PULSE_TX`).
* Characters. The same two datasheets and the KNX Association's "KNX
  Basics" (KNX System Specifications overview): a start bit (0), 8 data
  bits LSB first, an even parity bit, a stop bit (1), then 2 bit times of
  idle before the next character of the same telegram: 13 bit times from
  start to start.
* Telegrams ("KNX Basics", the standard frame figure): at least 50 bit
  times of idle bus before a telegram; control byte (bit 7 = 1, bit 6 = 0
  for a standard frame, bit 5 = 1 "not repeated", bits 3:2 the priority,
  bit 4 = 1, bits 1:0 = 0: 0xBC is low priority, not repeated); source
  address (2 bytes); destination (2 bytes); a byte with the address type in
  bit 7 (1 = group), the hop count in 6:4 and the length in 3:0 (the bytes
  after the TPCI byte); the TPCI byte and the APCI/data bytes (length + 1);
  the check byte, the NOT of the XOR of all the bytes before it ("odd
  parity" across each bit position). A GroupValue_Write of up to 6 bits is
  TPCI 0x00 then 0x80 | value, length 1. UNCHECKED: the length field's
  exact meaning (taken: the count of bytes after the TPCI byte).
* Acknowledge ("KNX Basics"; TP-UART 2 technical data): the addressed
  receivers answer with one character 15 bit times after the end of the
  telegram's last character: ACK 0xCC, NAK 0x0C, BUSY 0xC0. UNCHECKED: the
  15 bit times counted from the end of the last stop bit (taken), and the
  window of +-1 bit time accepted for it here (:data:`ACK_GAP`).
* Arbitration ("KNX Basics", CSMA/CA): devices that start together send
  bit by bit; one that reads a 0 where it sent a 1 has lost, stops sending
  at once and becomes a receiver.

Time: every model works in clocks, and ``us`` is clocks per microsecond.
At the standard rate on a 50 MHz chip ``us = 50``; the RTL tests run the
whole bus faster and every figure scales with it (:func:`scale_for`).

:class:`KnxBus` is the bench model: the wired AND of the chip's TX pins
(each delayed by the transceiver's delay, a parameter) and of every model
node, seen by the chip's RX pins. :class:`KnxDecoder` turns the bus back
into characters and telegrams; :func:`check_chars` holds a sender's
characters to the timing above, :func:`check_pulses` its pulse widths.
:class:`KnxNode` is a model device: it sends telegrams with injected errors,
arbitrates, receives and answers.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, List, Optional, Sequence, Tuple

from tools.protomodels.bench import Drive, Lines, Model, PinSpec, pad_of

CLOCK = 50_000_000                  # the chip's clock in the tests
BIT_US = 1e6 / 9600                 # 104.17 us
PULSE_US = 35.0                     # the active pulse of a 0
PULSE_TX = (33.0, 37.0)             # accepted width of a sent pulse (UNCHECKED)
CHAR_BITS = 11                      # start, 8 data, parity, stop
CHAR_PERIOD = 13                    # bit times start to start in a telegram
IDLE_BITS = 50                      # idle bus before a telegram, at least
ACK_GAP = (14.0, 16.0)              # end of the last stop bit to the answer (15)
END_BITS = 4                        # no start this long after a stop: the end
ACK, NAK, BUSY = 0xCC, 0x0C, 0xC0
CTRL_LOW = 0xBC                     # standard frame, not repeated, low priority
NPCI_GROUP = 0xE1                   # group address, hop count 6, length 1


def scale_for(bit_clocks: float) -> float:
    """Clocks per microsecond for a bus whose bit is ``bit_clocks``."""
    return bit_clocks / BIT_US


def parity(byte: int) -> int:
    """The even parity bit of ``byte``."""
    return bin(byte & 0xFF).count("1") & 1


def check_byte(data: Sequence[int]) -> int:
    """The NOT of the XOR of ``data``."""
    x = 0
    for b in data:
        x ^= b
    return ~x & 0xFF


def group_write(src: int, dst: int, apci: int = 0x81, ctrl: int = CTRL_LOW) -> List[int]:
    """The bytes of a GroupValue_Write telegram, check byte included."""
    body = [ctrl, src >> 8, src & 0xFF, dst >> 8, dst & 0xFF, NPCI_GROUP, 0x00, apci]
    return body + [check_byte(body)]


def individual(area: int, line: int, device: int) -> int:
    """An individual address area.line.device as 16 bits."""
    return (area << 12) | (line << 8) | device


def char_bits(byte: int, bad_parity: bool = False) -> List[int]:
    """The 11 bits of a character on the bus, start first."""
    p = parity(byte) ^ int(bad_parity)
    return [0] + [(byte >> i) & 1 for i in range(8)] + [p, 1]


@dataclass
class Char:
    """One character read from the bus. ``start`` is the start pulse's
    falling edge (cycle); ``zeros`` the bit indices that carried a pulse."""

    start: int
    zeros: set = field(default_factory=set)

    @property
    def bits(self) -> List[int]:
        return [0 if i in self.zeros else 1 for i in range(CHAR_BITS)]

    @property
    def value(self) -> int:
        return sum(b << i for i, b in enumerate(self.bits[1:9]))

    @property
    def parity_ok(self) -> bool:
        return self.bits[9] == parity(self.value)

    @property
    def framing_ok(self) -> bool:
        return self.bits[10] == 1


@dataclass
class Telegram:
    """Characters 13 bit times apart (a gap of at most END_BITS)."""

    chars: List[Char]

    @property
    def data(self) -> List[int]:
        return [c.value for c in self.chars]

    @property
    def ok(self) -> bool:
        """Every character well formed and the check byte right."""
        return (all(c.parity_ok and c.framing_ok for c in self.chars)
                and len(self.chars) >= 8
                and self.data[-1] == check_byte(self.data[:-1]))

    @property
    def destination(self) -> Optional[Tuple[int, bool]]:
        """``(address, is_group)``, or None for a telegram too short."""
        if len(self.chars) < 6:
            return None
        d = self.data
        return (d[3] << 8) | d[4], bool(d[5] & 0x80)


class KnxDecoder:
    """Characters and telegrams from the bus level, clock by clock.

    A falling edge with no character open starts one; a later falling edge
    is the pulse of bit ``round((edge - start) / bit)``. A character closes
    half a bit after its stop bit's start. :attr:`pulses` holds every pulse
    as ``(fall, width)`` in clocks."""

    def __init__(self, us: float) -> None:
        self.bit = BIT_US * us
        self.level = 1
        self.fall = None
        self.open: Optional[Char] = None
        self.chars: List[Char] = []
        self.telegrams: List[Telegram] = []
        self.pulses: List[Tuple[int, int]] = []

    def clock(self, level: int, cycle: int) -> None:
        if self.open is not None and cycle >= self.open.start + 10.5 * self.bit:
            self._close()
        if self.level and not level:
            self.fall = cycle
            if self.open is None:
                self.open = Char(cycle, {0})
            else:
                i = int(round((cycle - self.open.start) / self.bit))
                self.open.zeros.add(min(max(i, 1), CHAR_BITS - 1))
        elif level and not self.level and self.fall is not None:
            self.pulses.append((self.fall, cycle - self.fall))
        self.level = level

    def _close(self) -> None:
        c, self.open = self.open, None
        last = self.telegrams[-1].chars[-1] if self.telegrams else None
        if last is not None and c.start - (last.start + CHAR_BITS * self.bit) <= (END_BITS + 0.5) * self.bit:
            self.telegrams[-1].chars.append(c)
        else:
            self.telegrams.append(Telegram([c]))
        self.chars.append(c)


def check_pulses(pulses: Iterable[Tuple[int, int]], us: float) -> List[str]:
    """Every pulse's width inside :data:`PULSE_TX`."""
    lo, hi = PULSE_TX[0] * us, PULSE_TX[1] * us
    return ["pulse at %d: %d clocks (%.1f us)" % (f, w, w / us)
            for f, w in pulses if not lo <= w <= hi]


def check_chars(tel: Telegram, us: float, previous_end: Optional[int] = None,
                tol_bits: float = 1 / 16) -> List[str]:
    """A sender's telegram: characters 13 bit times apart (+-``tol_bits``,
    UNCHECKED: the standard's tolerance was not at hand), and at least
    IDLE_BITS of idle bus after ``previous_end`` (the end of the last stop
    bit before it)."""
    bit = BIT_US * us
    out = []
    for a, b in zip(tel.chars, tel.chars[1:]):
        d = (b.start - a.start) / bit
        if abs(d - CHAR_PERIOD) > tol_bits:
            out.append("characters at %d and %d: %.3f bit times apart" % (a.start, b.start, d))
    if previous_end is not None:
        idle = (tel.chars[0].start - previous_end) / bit
        if idle < IDLE_BITS:
            out.append("telegram at %d after %.2f bit times idle" % (tel.chars[0].start, idle))
    return out


def end_of(tel: Telegram, us: float) -> float:
    """The end of the telegram's last stop bit, in clocks."""
    return tel.chars[-1].start + CHAR_BITS * BIT_US * us


def answer_gap(tel: Telegram, answer: Telegram, us: float) -> float:
    """Bit times from the end of ``tel`` to the start of ``answer``."""
    return (answer.chars[0].start - end_of(tel, us)) / (BIT_US * us)


@dataclass
class Send:
    """A telegram a :class:`KnxNode` is to send, and what became of it.

    ``bad_parity_at`` flips the parity bit of that character; ``bad_check``
    flips the check byte's low bit; ``with_chip`` starts it on the first
    start pulse of the bus after the idle time, in the same clock (a
    collision with whoever made that pulse); ``arbitrate = False`` keeps
    sending after a lost bit."""

    data: List[int]
    bad_parity_at: Optional[int] = None
    bad_check: bool = False
    with_chip: bool = False
    arbitrate: bool = True
    start: Optional[int] = None
    lost_at: Optional[Tuple[int, int]] = None   # (character, bit)
    done: bool = False
    end: Optional[float] = None
    losses: int = 0
    lost_bits: List[Tuple[int, int]] = field(default_factory=list)   # every loss


class KnxNode:
    """A KNX device on the bus: sends :class:`Send` telegrams after the idle
    time (retrying one it lost, as a device does), samples every 1 it sends
    ``sample_us`` into the bit, and answers telegrams addressed to its
    ``address`` or one of its ``groups``: ACK, or NAK if the telegram is not
    :attr:`Telegram.ok`; ``answer`` overrides that (a byte, or None for no
    answer) and ``answer_bits`` the time (15 bit times). ``skew`` stretches
    its bit (0.01 = 1 % slow)."""

    def __init__(self, us: float, address: int = 0x1101, groups: Iterable[int] = (),
                 answer="auto", answer_bits: float = 15.0, sample_us: float = 15.0,
                 pulse_us: float = PULSE_US, skew: float = 0.0) -> None:
        self.us = us
        self.bit = BIT_US * us * (1 + skew)
        self.pulse = pulse_us * us
        self.sample = sample_us * us
        self.address, self.groups = address, set(groups)
        self.answer, self.answer_bits = answer, answer_bits
        self.decoder = KnxDecoder(us)
        self.queue: List[Send] = []
        self.sent: List[Send] = []
        self.level = 1
        self.last_low = 0
        self._pulses: List[Tuple[float, float]] = []
        self._samples: List[Tuple[float, int, int]] = []
        self._current: Optional[Send] = None
        self._seen = 0                  # telegrams of the decoder looked at
        self._own: List[int] = []       # start cycles of telegrams it won
        self.answers: List[Tuple[int, int]] = []    # (cycle, byte) sent

    def send(self, data: Sequence[int], **kw) -> Send:
        s = Send(list(data), **kw)
        self.queue.append(s)
        return s

    def _plan(self, data: Sequence[int], t0: float, send: Optional[Send]) -> None:
        for k, byte in enumerate(data):
            bad = send is not None and send.bad_parity_at == k
            s = t0 + CHAR_PERIOD * k * self.bit
            for i, b in enumerate(char_bits(byte, bad)):
                at = s + i * self.bit
                if b == 0:
                    self._pulses.append((at, at + self.pulse))
                elif send is not None:
                    self._samples.append((at + self.sample, k, i))

    def _start(self, cycle: int) -> None:
        s = self.queue[0]
        data = list(s.data)
        if s.bad_check:
            data[-1] ^= 1
        s.start, s.lost_at, self._current = cycle, None, s
        self._plan(data, cycle, s)
        s.end = cycle + ((len(data) - 1) * CHAR_PERIOD + CHAR_BITS) * self.bit

    def clock(self, level: int, cycle: int) -> None:
        fell = self.decoder.level and not level
        ends = [self.last_low]          # the idle time counts from the last
        if self.decoder.chars:          # pulse or the last stop bit's end
            ends.append(self.decoder.chars[-1].start + CHAR_BITS * self.decoder.bit)
        quiet = cycle - max(ends) >= IDLE_BITS * self.bit
        busy = self.decoder.open is not None
        self.decoder.clock(level, cycle)
        if not level:
            self.last_low = cycle
        s = self._current
        while self._samples and self._samples[0][0] <= cycle:
            _, k, i = self._samples.pop(0)
            if not level and s.arbitrate and s.lost_at is None:
                s.lost_at = (k, i)          # a 0 where it sent a 1: lost
                s.losses += 1
                s.lost_bits.append((k, i))
                self._pulses = [p for p in self._pulses if p[0] <= cycle]
                self._samples = []
        if s is not None and (s.lost_at is not None or cycle >= s.end):
            self._current = None
            if s.lost_at is None:
                s.done = True
                self.queue.pop(0)
                self._own.append(s.start)
            self.sent.append(s)
        if self._current is None and self.queue and not busy:
            q = self.queue[0]
            first = q.with_chip and q.losses == 0     # a retry waits as usual
            if first and fell and quiet:
                self._start(cycle)
            elif not first and quiet and level:
                self._start(cycle + 1)
        self._answer(cycle)
        nxt = cycle + 1
        self._pulses = [p for p in self._pulses if p[1] > nxt]
        self.level = 0 if any(a <= nxt < b for a, b in self._pulses) else 1

    def _answer(self, cycle: int) -> None:
        tels = self.decoder.telegrams
        while self._seen < len(tels):
            t = tels[self._seen]
            if self.decoder.open is not None and self._seen == len(tels) - 1:
                return                      # a character may still join it
            if cycle < end_of(t, self.us) + (END_BITS + 0.5) * self.decoder.bit:
                return
            self._seen += 1
            dest = t.destination
            if dest is None or any(abs(t.chars[0].start - o) <= 2 for o in self._own):
                continue
            addr, group = dest
            if not (addr in self.groups if group else addr == self.address):
                continue
            byte = self.answer
            if byte == "auto":
                byte = ACK if t.ok else NAK
            if byte is None:
                continue
            at = end_of(t, self.us) + self.answer_bits * self.decoder.bit
            self.answers.append((int(at), byte))
            self._plan([byte], at, None)


class KnxBus(Model):
    """The bus on the bench at logic level: a wired AND.

    The level of cycle ``c`` is the AND of every chip TX pin as it was in
    cycle ``c - delay`` (the transceiver's delay from TXD to the bus and back
    to RXD, ``delay`` >= 1 clocks; a TX pin at 1 asks for an active pulse,
    which pulls the bus to 0) and of every node's level. All the chip's RX
    pins see it, every node is clocked with it, and :attr:`decoder` decodes
    it. :attr:`tx_pulses` holds each chip TX pin's pulses as ``(rise,
    width)`` in clocks, measured on the pin."""

    def __init__(self, us: float, nodes: Iterable[KnxNode] = (),
                 tx: Sequence[PinSpec] = ("OUT0",), rx: Sequence[PinSpec] = ("IN0",),
                 delay: int = 1) -> None:
        if delay < 1:
            raise ValueError("the transceiver delay is at least one clock")
        self.us = us
        self.nodes = list(nodes)
        self.tx_pads = [pad_of(p) for p in tx]
        self.rx_pads = [pad_of(p) for p in rx]
        self.decoder = KnxDecoder(us)
        self.level = 1
        self.edges: List[Tuple[int, int]] = []
        self._history = [1] * delay
        self.tx_pulses = {p: [] for p in self.tx_pads}
        self._tx_prev = {p: 0 for p in self.tx_pads}
        self._tx_rise = {p: None for p in self.tx_pads}

    def add(self, node: KnxNode) -> KnxNode:
        self.nodes.append(node)
        return node

    def drive(self, drive: Drive, cycle: int) -> None:
        level = self._history[0]
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
            v = lines.get(pad)
            chip &= v ^ 1
            if v and not self._tx_prev[pad]:
                self._tx_rise[pad] = lines.cycle
            elif not v and self._tx_prev[pad] and self._tx_rise[pad] is not None:
                self.tx_pulses[pad].append((self._tx_rise[pad], lines.cycle - self._tx_rise[pad]))
            self._tx_prev[pad] = v
        self._history = self._history[1:] + [chip]
        self.decoder.clock(self.level, lines.cycle)
        for node in self.nodes:
            node.clock(self.level, lines.cycle)

    def chip_pulses(self) -> List[Tuple[int, int]]:
        """Every chip TX pulse, all pins, in time order."""
        return sorted(p for pulses in self.tx_pulses.values() for p in pulses)
