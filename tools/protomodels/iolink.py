"""IO-Link reference models: the C/Q line behind PHYs, a master and a device.

Written from the IO-Link Interface and System Specification v1.1.3 (June
2019, order no. 10.002, the free download from io-link.com; IEC 61131-9 is
the same text), not from the firmware. Every figure below is the
document's, by section and table; none is unchecked.

* the UART frame (5.3.3.1, Figure 21): a start bit 0, b0..b7 LSB first,
  an even parity bit, one stop bit 1; eleven bits a character. Levels here
  are the logic levels at the PHYs' UART side (1 idle, 0 start bit); the
  PHY inverts them to C/Q, where a start bit is +24 V;
* rates (Table 9): COM1 4.8, COM2 38.4, COM3 230.4 kbit/s; the master's
  rate within +-0.1 %; TBIT is 26.04 us at COM2;
* the wake-up request (5.3.3.3, Table 10): the master drives the line to
  the level opposite to the one it reads for TWU = 75 to 85 us; the device
  is ready to receive within TREN = 500 us. The master waits TDMT = 27 to
  37 TBIT before its first message (7.3.2.2, Table 42), which is a TYPE_0
  read of MinCycleTime, tried at COM3, COM2 and COM1 in turn;
* timing (Annex A.3): t1, the gap between the master's characters, 0 to
  1 TBIT (A.3.3); t2, the device's, 0 to 3 TBIT (A.3.4); tA, from the end
  of the stop bit of the master's last character to the start bit of the
  device's first, 1 to 10 TBIT (A.3.5); Tinitcyc, at least 100 TBIT
  between the starts of two TYPE_0 messages in STARTUP (Table A.7);
* the M-sequence control octet MC (A.1.2, Figure A.1, Tables A.1, A.2):
  bit 7 R/W (1 read), bits 6:5 the channel (0 process, 1 page, 2
  diagnosis, 3 ISDU), bits 4:0 the address; a read of what a device does
  not implement returns 0, a write of it is ignored;
* CKT (A.1.3, Figure A.2, Table A.3): bits 7:6 the M-sequence type (0, 1,
  2; 3 reserved), bits 5:0 the checksum. CKS (A.1.5, Figure A.3): bit 7
  the Event flag, bit 6 PD status (1 = invalid), bits 5:0 the checksum;
* the checksum (A.1.6, equation (A.1)): the XOR of every octet of the
  message, the CKT or CKS with its checksum bits 0, and the seed 0x52,
  compressed from 8 to 6 bits: D5 = c7^c5^c3^c1, D4 = c6^c4^c2^c0,
  D3 = c7^c6, D2 = c5^c4, D1 = c3^c2, D0 = c1^c0;
* the M-sequences (A.2.2, A.2.4, Figures A.5, A.9 to A.14, Table A.10):
  the master sends MC, CKT, its PD out octets, then OD for a write; the
  device replies OD for a read, its PD in octets, CKS. TYPE_0 has no PD;
  TYPE_2_1 one PD in octet, 2_2 two, 2_3 one PD out, 2_4 two, 2_5 one each;
* Direct Parameter page 1 (B.1.1, Table B.1): 0x00 MasterCommand, 0x01
  MasterCycleTime, 0x02 MinCycleTime, 0x03 M-sequence capability, 0x04
  RevisionID, 0x05 ProcessDataIn, 0x06 ProcessDataOut, 0x07..0x08 VendorID
  (MSB first), 0x09..0x0B DeviceID (MSB first), 0x0C..0x0D FunctionID,
  0x0F SystemCommand.

What a device's receiver must accept (Table 9, Figure 23) is written as
the detection window r = 0.65 and s = 0.22 TBIT; the models here sample
each bit at its middle from the start edge, which is what a UART does.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, Iterable, List, Optional, Sequence, Tuple

from tools.protomodels.bench import Drive, Lines, Model, PinSpec, pad_of

CLOCK = 50_000_000              # the chip's clock in the tests
COM2 = 38_400                   # bit/s (Table 9)
SEED = 0x52                     # A.1.6
TWU_US = (75.0, 85.0)           # Table 10
TREN_US = 500.0                 # Table 10
TDMT_BITS = (27, 37)            # Table 42
T1_MAX_BITS = 1                 # A.3.3
T2_MAX_BITS = 3                 # A.3.4
TA_BITS = (1, 10)               # A.3.5
TINITCYC_BITS = 100             # Table A.7
MASTER_RATE_TOL = 0.001         # Table 9
READ, WRITE = 1, 0
CH_PROCESS, CH_PAGE, CH_DIAGNOSIS, CH_ISDU = 0, 1, 2, 3
TYPE_0, TYPE_1, TYPE_2 = 0, 1, 2

# Direct Parameter page 1 addresses (Table B.1).
MIN_CYCLE_TIME, VENDOR_ID, DEVICE_ID = 0x02, 0x07, 0x09


def bit_clocks(rate: float = COM2, clock: int = CLOCK) -> float:
    """Clocks per bit at ``rate`` bit/s."""
    return clock / rate


def compress(c: int) -> int:
    """The 8-to-6-bit compression of equation (A.1)."""
    b = [(c >> i) & 1 for i in range(8)]
    d = [b[1] ^ b[0], b[3] ^ b[2], b[5] ^ b[4], b[7] ^ b[6],
         b[6] ^ b[4] ^ b[2] ^ b[0], b[7] ^ b[5] ^ b[3] ^ b[1]]
    return sum(bit << i for i, bit in enumerate(d))


def checksum6(octets: Iterable[int]) -> int:
    """The checksum of a message whose check octet is among ``octets`` with
    its checksum bits 0 (A.1.6)."""
    x = SEED
    for o in octets:
        x ^= o & 0xFF
    return compress(x)


def mc(rw: int, channel: int, address: int) -> int:
    """The M-sequence control octet (Figure A.1)."""
    return ((rw & 1) << 7) | ((channel & 3) << 5) | (address & 31)


def master_message(mc_octet: int, mtype: int, pd_out: Sequence[int] = (),
                   od: Optional[int] = None) -> List[int]:
    """MC, CKT, PD out, OD (a write only), with the checksum in CKT."""
    body = [mc_octet & 0xFF, (mtype & 3) << 6] + [p & 0xFF for p in pd_out]
    if not (mc_octet >> 7) & 1:
        body.append((od or 0) & 0xFF)
    body[1] |= checksum6(body)
    return body


def device_reply(od: Optional[int] = None, pd_in: Sequence[int] = (),
                 event: int = 0, pd_invalid: int = 0) -> List[int]:
    """OD (for a read), PD in, CKS with its flags and checksum (A.1.5)."""
    body = ([] if od is None else [od & 0xFF]) + [p & 0xFF for p in pd_in]
    cks = ((event & 1) << 7) | ((pd_invalid & 1) << 6)
    body.append(cks)
    body[-1] |= checksum6(body)
    return body


def check_ok(octets: Sequence[int], check_index: int) -> bool:
    """True if the checksum bits of ``octets[check_index]`` are right."""
    zeroed = list(octets)
    zeroed[check_index] &= 0xC0
    return checksum6(zeroed) == octets[check_index] & 63


def parity(octet: int) -> int:
    """The even parity bit of an octet (Figure 21)."""
    return bin(octet & 0xFF).count("1") & 1


def char_bits(octet: int, parity_error: bool = False) -> List[int]:
    """The eleven bits of one UART frame, in transmission order."""
    data = [(octet >> i) & 1 for i in range(8)]
    return [0] + data + [parity(octet) ^ int(parity_error), 1]


# ------------------------------------------------------------------ receiving
@dataclass
class Char:
    """One UART frame as a receiver saw it: the cycle of its start edge,
    the octet, its parity and stop bit, and every edge inside it."""

    start: int
    value: int = 0
    parity_ok: bool = True
    stop_ok: bool = True
    edges: List[Tuple[int, int]] = field(default_factory=list)
    T: float = 0.0

    @property
    def end(self) -> float:
        """The nominal end of the stop bit: eleven bit times on."""
        return self.start + 11 * self.T

    @property
    def ok(self) -> bool:
        return self.parity_ok and self.stop_ok


class UartRx:
    """A UART receiver for the IO-Link frame at ``T`` clocks a bit: a start
    is a falling edge seen from the idle level; each bit is sampled at its
    middle, counted from that edge (bits 1..8 data, 9 parity, 10 stop)."""

    def __init__(self, T: float) -> None:
        self.T = float(T)
        self.prev = 1
        self.cur: Optional[Char] = None
        self._bits: List[int] = []
        self.chars: List[Char] = []

    def feed(self, level: int, cycle: int) -> Optional[Char]:
        done = None
        cur = self.cur
        if cur is None:
            if self.prev == 1 and level == 0:
                self.cur = cur = Char(start=cycle, T=self.T)
                self._bits = []
        if cur is not None:
            if level != self.prev:
                cur.edges.append((cycle, level))
            k = len(self._bits)
            if cycle >= cur.start + (k + 0.5) * self.T:
                self._bits.append(level)
                if len(self._bits) == 11:
                    b = self._bits
                    cur.value = sum(bit << i for i, bit in enumerate(b[1:9]))
                    cur.parity_ok = (sum(b[1:10]) & 1) == 0
                    cur.stop_ok = b[10] == 1 and b[0] == 0
                    self.chars.append(cur)
                    done, self.cur = cur, None
        self.prev = level
        return done


# ------------------------------------------------------------------ the line
class Driver:
    """What a model node drives on C/Q (through its PHY): ``enabled`` and
    ``level`` for the next cycle, from a queue of ``(cycle, en, level)``."""

    def __init__(self, name: str) -> None:
        self.name = name
        self.enabled = False
        self.level = 1
        self._q: Deque[Tuple[int, bool, int]] = deque()

    def schedule(self, cycle: float, enabled: bool, level: int) -> None:
        self._q.append((int(round(cycle)), enabled, level))

    def advance(self, cycle: int) -> None:
        nxt = cycle + 1
        q = self._q
        while q and q[0][0] <= nxt:
            _, self.enabled, self.level = q.popleft()

    def busy(self) -> bool:
        return bool(self._q)

    def send_bits(self, t0: float, T: float, bits: Sequence[int]) -> float:
        """Drive ``bits`` from ``t0``, one every ``T`` clocks; the end."""
        for k, bit in enumerate(bits):
            self.schedule(t0 + k * T, True, bit)
        return t0 + len(bits) * T

    def send_chars(self, t0: float, T: float, octets: Sequence[int],
                   gaps: Sequence[float] = (), parity_error: Sequence[int] = (),
                   enable_early: float = 0.0) -> float:
        """Enable the driver ``enable_early`` clocks before ``t0``, send the
        octets with ``gaps[i]`` bit times of idle after octet ``i``, then
        release it; returns the end of the last stop bit."""
        self.schedule(t0 - enable_early, True, 1)
        t = t0
        for i, octet in enumerate(octets):
            t = self.send_bits(t, T, char_bits(octet, i in parity_error))
            if i < len(octets) - 1:
                t += (gaps[i] if i < len(gaps) else 0.0) * T
        self.schedule(t, False, 1)
        return t


class IoLinkLine(Model):
    """The C/Q line on the bench, at the PHYs' logic level.

    ``ports`` are the chip's ``(TX, TXEN, RX)`` pin triples, one per Loom
    thread on the line; ``nodes`` are model masters and devices, each with
    a :class:`Driver`. The line is the level of whoever drives it (two
    drivers that disagree are a contention: recorded, and 0 wins); nobody
    driving leaves it at 1, where the master port's sink current IPDM holds
    C/Q (5.3.3.1). Each PHY delays by ``phy_delay`` clocks: the chip's TX
    and TXEN reach the line ``1 + phy_delay`` cycles after the pins change
    (one cycle because :meth:`drive` may only use registered state), and
    the line reaches the chip's RX pins ``phy_delay`` cycles later. Model
    nodes see and drive the line itself.

    For every source (``"chip0"``, ``"chip1"``, ... and each node's name)
    the line keeps a :class:`UartRx` of what that source drove
    (:attr:`rx`), its 0 runs (:attr:`low_runs`, ``(start, clocks)``) and
    its driver-enable intervals (:attr:`bursts`), so a test measures each
    side's timing on the wire; :attr:`edges` has every change of the line.
    """

    def __init__(self, T: float, nodes: Iterable = (),
                 ports: Sequence[Tuple[PinSpec, PinSpec, PinSpec]] = (("OUT0", "OUT1", "IN0"),),
                 phy_delay: int = 0) -> None:
        self.T = float(T)
        self.nodes = list(nodes)
        self.ports = [tuple(pad_of(p) for p in port) for port in ports]
        self.phy_delay = int(phy_delay)
        self._chip = [(False, 1)] * len(self.ports)
        self._chip_q: Deque[List[Tuple[bool, int]]] = deque()
        self._rx_q: Deque[int] = deque()
        self.level = 1
        self.edges: List[Tuple[int, int]] = []
        self.contentions: List[int] = []
        names = ["chip%d" % i for i in range(len(self.ports))]
        names += [n.driver.name for n in self.nodes]
        self.rx = {n: UartRx(T) for n in names}
        self.low_runs: Dict[str, List[Tuple[int, int]]] = {n: [] for n in names}
        self.bursts: Dict[str, List[Tuple[int, int]]] = {n: [] for n in names}
        self._low: Dict[str, Optional[int]] = {n: None for n in names}
        self._on: Dict[str, Optional[int]] = {n: None for n in names}

    def _sources(self):
        for i, (en, lvl) in enumerate(self._chip):
            yield "chip%d" % i, en, lvl
        for node in self.nodes:
            yield node.driver.name, node.driver.enabled, node.driver.level

    def drive(self, drive: Drive, cycle: int) -> None:
        driven = []
        for name, en, lvl in self._sources():
            eff = lvl if en else 1
            self.rx[name].feed(eff, cycle)
            if eff == 0 and self._low[name] is None:
                self._low[name] = cycle
            elif eff == 1 and self._low[name] is not None:
                self.low_runs[name].append((self._low[name], cycle - self._low[name]))
                self._low[name] = None
            if en and self._on[name] is None:
                self._on[name] = cycle
            elif not en and self._on[name] is not None:
                self.bursts[name].append((self._on[name], cycle))
                self._on[name] = None
            if en:
                driven.append(lvl)
        level = min(driven) if driven else 1
        if len(set(driven)) > 1:
            self.contentions.append(cycle)
        if level != self.level:
            self.edges.append((cycle, level))
        self.level = level
        self._rx_q.append(level)
        out = self._rx_q.popleft() if len(self._rx_q) > self.phy_delay else 1
        for _, _, rx in self.ports:
            drive.set(rx, out)

    def observe(self, lines: Lines) -> None:
        self._chip_q.append([(lines.get(en) == 1, lines.get(tx))
                             for tx, en, _ in self.ports])
        while len(self._chip_q) > self.phy_delay:
            self._chip = self._chip_q.popleft()
        for node in self.nodes:
            node.clock(self.level, lines.cycle)


# ------------------------------------------------------------------ the nodes
@dataclass
class Message:
    """A master message as a device model received it."""

    chars: List[Char]
    complete: bool = False
    reply: Optional[List[int]] = None       # what the device sent, if anything

    @property
    def octets(self) -> List[int]:
        return [c.value for c in self.chars]

    @property
    def ok(self) -> bool:
        return (self.complete and all(c.ok for c in self.chars)
                and check_ok(self.octets, 1))


def message_length(mc_octet: int, ckt: int, n_out: int) -> Optional[int]:
    """The length of a master message from its MC and CKT (A.2), with
    ``n_out`` PD out octets for TYPE_2; None for a type not modelled."""
    mtype = ckt >> 6
    write = not (mc_octet >> 7) & 1
    if mtype == TYPE_0:
        return 2 + write
    if mtype == TYPE_2:
        return 2 + n_out + write
    return None


class DeviceNode:
    """A device at ``T`` clocks a bit (its own clock, ``clock_error`` off).

    Asleep until a 0 on the line, driven by someone else, lasts within
    ``wake_window`` clocks (default TWU, 75 to 85 us at :data:`CLOCK`,
    Table 10); then in COM mode it collects master messages (a character
    more than ``1 + t1_slack`` bit times after the previous one's end
    starts a new message), checks parity, stop bits and the checksum, and
    replies ``ta_bits`` after the end of the last stop bit with ``t2_bits``
    between its characters: OD from :attr:`page` (Direct Parameter page 1
    and 2, 32 octets) for a page read and 0 for other reads, the
    :attr:`pd_in` octets for TYPE_2, and CKS. Page writes go into the page;
    PD out octets go to :attr:`pd_out_log`. Error injection: ``spoil`` (the
    next replies' checksums are wrong), ``silent``, ``ta_bits`` and
    ``t2_bits`` outside the windows, ``parity_error`` (characters of the
    next reply).
    """

    def __init__(self, T: float, *, n_in: int = 0, n_out: int = 0,
                 page: Optional[Sequence[int]] = None, ta_bits: float = 2.0,
                 t2_bits: float = 0.0, clock_error: float = 0.0,
                 wake_window: Optional[Tuple[float, float]] = None,
                 awake: bool = False, t1_slack: float = 0.5,
                 name: str = "device", clock: int = CLOCK) -> None:
        self.T = float(T) * (1 + clock_error)
        self.driver = Driver(name)
        self.rx = UartRx(self.T)
        self.n_in, self.n_out = n_in, n_out
        self.page = list(page or [0] * 16) + [0] * 32
        self.page = self.page[:32]
        self.pd_in = [0] * n_in
        self.pd_out_log: List[List[int]] = []
        self.ta_bits, self.t2_bits = ta_bits, t2_bits
        self.t1_slack = t1_slack
        lo, hi = TWU_US
        self.wake_window = wake_window or (lo * clock / 1e6, hi * clock / 1e6)
        self.awake = awake
        self.wakeups: List[Tuple[int, int]] = []
        self.messages: List[Message] = []
        self.spoil = 0
        self.silent = False
        self.parity_error: Tuple[int, ...] = ()
        self._low: Optional[int] = None
        self._cur: Optional[Message] = None

    def clock(self, line: int, cycle: int) -> None:
        drv = self.driver
        drv.advance(cycle)
        if drv.enabled or drv.busy():
            self._low = None
            return
        if not self.awake:
            if line == 0 and self._low is None:
                self._low = cycle
            elif line == 1 and self._low is not None:
                width = cycle - self._low
                self._low = None
                if self.wake_window[0] <= width <= self.wake_window[1]:
                    self.awake = True
                    self.wakeups.append((cycle - width, width))
                    self.rx = UartRx(self.T)
            return
        ch = self.rx.feed(line, cycle)
        if ch is not None:
            self._char(ch)

    def _char(self, ch: Char) -> None:
        cur = self._cur
        if cur is not None and ch.start - cur.chars[-1].end > (1 + self.t1_slack) * self.T:
            self._cur = cur = None              # t1 exceeded: a new message
        if cur is None:
            self._cur = cur = Message(chars=[])
            self.messages.append(cur)
        cur.chars.append(ch)
        if len(cur.chars) < 2:
            return
        need = message_length(cur.chars[0].value, cur.chars[1].value, self.n_out)
        if need is None or len(cur.chars) >= need:
            cur.complete = need is not None
            self._cur = None
            self._answer(cur)

    def _answer(self, msg: Message) -> None:
        if not msg.ok or self.silent:
            return
        o = msg.octets
        mc_octet, mtype = o[0], o[1] >> 6
        read = (mc_octet >> 7) & 1
        channel, addr = (mc_octet >> 5) & 3, mc_octet & 31
        od = None
        if read:
            od = self.page[addr] if channel == CH_PAGE else 0
        elif channel == CH_PAGE:
            self.page[addr] = o[-1]
        pd_in: List[int] = []
        if mtype == TYPE_2:
            pd_in = list(self.pd_in)
            if self.n_out:
                self.pd_out_log.append(o[2:2 + self.n_out])
        reply = device_reply(od, pd_in)
        if self.spoil:
            self.spoil -= 1
            reply[-1] ^= 1
        msg.reply = reply
        t0 = msg.chars[-1].end + self.ta_bits * self.T
        self.driver.send_chars(t0, self.T, reply, [self.t2_bits] * len(reply),
                               self.parity_error, enable_early=self.T / 2)
        self.parity_error = ()


@dataclass
class Exchange:
    """One M-sequence a master model ran: what it sent, when its last stop
    bit ended, and the reply characters that followed."""

    sent: List[int]
    start: float
    end: float
    n_reply: int
    reply: List[Char] = field(default_factory=list)

    @property
    def octets(self) -> List[int]:
        return [c.value for c in self.reply]

    @property
    def complete(self) -> bool:
        return len(self.reply) >= self.n_reply

    @property
    def ok(self) -> bool:
        return (self.complete and all(c.ok for c in self.reply)
                and check_ok(self.octets, len(self.reply) - 1))

    @property
    def ta_clocks(self) -> Optional[float]:
        return self.reply[0].start - self.end if self.reply else None


class MasterNode:
    """A master port at ``T`` clocks a bit: wake-up requests and
    M-sequences on request, each recorded as an :class:`Exchange` with the
    device's reply as it came on the line."""

    def __init__(self, T: float, *, name: str = "master") -> None:
        self.T = float(T)
        self.driver = Driver(name)
        self.rx = UartRx(self.T)
        self.exchanges: List[Exchange] = []
        self.now = 0

    def wake(self, start: float, width: float) -> None:
        """Drive the line to 0 (the opposite of its idle 1) for ``width``."""
        self.driver.schedule(start, True, 0)
        self.driver.schedule(start + width, False, 1)

    def send(self, start: float, mc_octet: int, mtype: int, pd_out: Sequence[int] = (),
             od: Optional[int] = None, n_in: int = 0, spoil: bool = False,
             gaps: Sequence[float] = ()) -> Exchange:
        sent = master_message(mc_octet, mtype, pd_out, od)
        if spoil:
            sent[1] ^= 1
        end = self.driver.send_chars(start, self.T, sent, gaps, enable_early=self.T)
        n_reply = (mc_octet >> 7) + (n_in if mtype == TYPE_2 else 0) + 1
        ex = Exchange(sent=sent, start=start, end=end, n_reply=n_reply)
        self.exchanges.append(ex)
        return ex

    def clock(self, line: int, cycle: int) -> None:
        self.now = cycle
        self.driver.advance(cycle)
        if self.driver.enabled:
            return
        ch = self.rx.feed(line, cycle)
        if ch is not None and self.exchanges:
            ex = self.exchanges[-1]
            if ch.start >= ex.end and not ex.complete:
                ex.reply.append(ch)


# ------------------------------------------------------------ timing checks
def wake_width_ok(clocks: float, clock: int = CLOCK) -> bool:
    """A wake-up pulse of ``clocks`` within TWU (Table 10)."""
    us = clocks * 1e6 / clock
    return TWU_US[0] <= us <= TWU_US[1]


def ta_ok(ta_clocks: float, T: float) -> bool:
    """tA within 1 to 10 bit times (A.3.5)."""
    return TA_BITS[0] * T <= ta_clocks <= TA_BITS[1] * T


def gaps_bits(chars: Sequence[Char], T: float) -> List[float]:
    """The idle time between successive characters, in bit times: t1 for a
    master's message (at most 1, A.3.3), t2 for a device's (at most 3)."""
    return [(b.start - a.end) / T for a, b in zip(chars, chars[1:])]


def edge_offsets(ch: Char, T: float) -> List[float]:
    """Each edge inside a character against the bit grid its start edge
    sets, in clocks (0 is exactly on a bit boundary)."""
    out = []
    for cycle, _ in ch.edges:
        k = round((cycle - ch.start) / T)
        out.append(cycle - ch.start - k * T)
    return out


def characters_after(rx: UartRx, start: float) -> List[Char]:
    """The characters a source sent from cycle ``start`` on."""
    return [c for c in rx.chars if c.start >= start]
