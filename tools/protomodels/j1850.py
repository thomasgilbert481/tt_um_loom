"""SAE J1850 VPW reference model for firmware/j1850_vpw.loom.

Written from the protocol as public secondary sources describe it, never
from the firmware. SAE J1850 itself is not free and was not read; every
figure below is SECONDARY:

* Motorola/Freescale application note AN1731, "VPW J1850 Multiplexing and
  Motorola's Byte Data Link Controller (BDLC) Module" (read 2026-11-07):
  Table 3, "VPW Pulse Width Times" (p. 7), gives every timing window below
  (:data:`TX_WIN`, :data:`RX_WIN`); Figure 2 (p. 6) the bit coding (a 0 is a
  short passive or a long active symbol, a 1 a long passive or a short
  active one); pp. 8-9 the frame (SOF, header, data MSB first, CRC, EOD, NB,
  IFR, EOF, IFS; at most 12 data bytes), the arbitration (active dominates
  passive, a 0 dominates a 1, the lower header wins) and the CRC
  (polynomial x^8 + x^4 + x^3 + x^2 + 1; a receiver that runs header, data
  and CRC through the register finds 0xC4).
* The CRC register's preset (all ones) and the inversion of the sent CRC
  are not in AN1731's text: they are the "CRC-8/SAE-J1850" entry of the
  public CRC catalogues (from memory, UNCHECKED). They are the only reading
  under which AN1731's residue 0xC4 holds, which :func:`residue` checks.
* The normalization bit: 64 us before an IFR without CRC, 128 us before
  one with a CRC (Intel 8XC196Lx user manual, figure 86; ST92F124 data
  sheet, NBSYMS = 0; from search-result extracts, pages not opened:
  UNCHECKED). One-byte IFRs (types 1 and 2) carry no CRC.
* The header's K bit, bit 3 of the first byte of a three-byte header, 0 =
  IFR required (SAE J2178, from memory, UNCHECKED).

Time here is in clocks, from microseconds through ``us`` (clocks per
microsecond, :func:`scale_for`). The firmware counts ticks of 2 us; the
tests run a tick of 16 clocks, so ``us`` is 8 there and every time on the
bus scales with it.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, List, Optional, Sequence, Tuple

from tools.protomodels.bench import Drive, Lines, Model, PinSpec, pad_of

TICK_US = 2.0                               # the firmware's tick

# AN1731 Table 3, microseconds: transmit (min, nominal, max) and receive
# (exclusive min, inclusive max); None where the table has N/A.
SHORT, LONG, SOF, EOF, IFS = "short", "long", "sof", "eof", "ifs"
TX_WIN = {SHORT: (49.0, 64.0, 79.0), LONG: (112.0, 128.0, 145.0),
          SOF: (182.0, 200.0, 218.0), EOF: (261.0, 280.0, None),
          IFS: (280.0, 300.0, None)}
RX_WIN = {SHORT: (34.0, 96.0), LONG: (96.0, 163.0), SOF: (163.0, 239.0),
          EOF: (239.0, None)}
EOD = SOF                                   # EOD has SOF's length, passive
NB_NO_CRC, NB_WITH_CRC = SHORT, LONG        # the NB's length (UNCHECKED)
MAX_DATA = 12
CRC_POLY, CRC_INIT, CRC_XOR, CRC_RESIDUE = 0x1D, 0xFF, 0xFF, 0xC4


def scale_for(tick_clocks: float) -> float:
    """Clocks per microsecond when the firmware's 2 us tick is ``tick_clocks``."""
    return tick_clocks / TICK_US


def crc_register(data: Iterable[int], reg: int = CRC_INIT) -> int:
    """The CRC-8 register after ``data``, MSB first, from ``reg``."""
    for byte in data:
        reg ^= byte & 0xFF
        for _ in range(8):
            reg = ((reg << 1) ^ CRC_POLY) & 0xFF if reg & 0x80 else (reg << 1) & 0xFF
    return reg


def crc8(data: Iterable[int]) -> int:
    """The CRC byte a transmitter appends: the register, inverted."""
    return crc_register(data) ^ CRC_XOR


def residue(frame: Sequence[int]) -> int:
    """The register after the whole frame, CRC byte included: 0xC4 if good."""
    return crc_register(frame)


def with_crc(data: Sequence[int]) -> List[int]:
    data = [b & 0xFF for b in data]
    return data + [crc8(data)]


def classify(length_us: float) -> Optional[str]:
    """A symbol's length class by the receive windows, None if invalid
    (34 us or less). EOF stands for any length over 239 us."""
    for name in (SHORT, LONG, SOF):
        lo, hi = RX_WIN[name]
        if lo < length_us <= hi:
            return name
    return EOF if length_us > RX_WIN[EOF][0] else None


def bit_symbols(byte: int, level: int) -> Tuple[List[Tuple[int, str]], int]:
    """A byte's eight symbols ``(level, class)`` MSB first, starting at
    ``level`` (0 passive, 1 active); returns them and the level after."""
    out = []
    for i in range(8):
        bit = (byte >> (7 - i)) & 1
        out.append((level, SHORT if bit == level else LONG))
        level ^= 1
    return out, level


def frame_symbols(data: Sequence[int]) -> List[Tuple[int, str]]:
    """The symbols of a frame's SOF and bytes (``data`` already holds the
    CRC); EOD is the passive time after the last one."""
    out: List[Tuple[int, str]] = [(1, SOF)]
    level = 0
    for byte in data:
        syms, level = bit_symbols(byte, level)
        out += syms
    return out


def ifr_symbols(data: Sequence[int], nb: str = NB_NO_CRC) -> List[Tuple[int, str]]:
    """An IFR after EOD: the NB (active), then the bytes from passive."""
    out: List[Tuple[int, str]] = [(1, nb)]
    level = 0
    for byte in data:
        syms, level = bit_symbols(byte, level)
        out += syms
    return out


# ------------------------------------------------------------------ decoder
@dataclass
class Symbol:
    """One symbol on the bus: its level (1 active), first cycle and length
    in clocks."""

    level: int
    start: int
    length: int

    def us(self, us: float) -> float:
        return self.length / us


@dataclass
class Frame:
    """A frame as the bus carried it: every symbol from SOF on, the bytes
    before EOD (``data``, CRC included), the NB and the IFR bytes."""

    start: int
    symbols: List[Symbol] = field(default_factory=list)
    data: List[int] = field(default_factory=list)
    ifr: List[int] = field(default_factory=list)
    eod: Optional[Symbol] = None
    nb: Optional[Symbol] = None
    errors: List[str] = field(default_factory=list)
    phase: str = "data"                     # data, eod, ifr, eof
    nbits: int = 0
    acc: int = 0

    def crc_ok(self) -> bool:
        return len(self.data) >= 2 and residue(self.data) == CRC_RESIDUE


class VpwDecoder:
    """Clocked with the bus level every cycle; collects :attr:`frames`.

    An active symbol in SOF's receive window opens a frame. Each later
    symbol is classified by its length (:func:`classify`): short and long
    ones are bits (the level and the length give the value), a passive
    symbol in SOF's window is EOD, an active one after EOD is the NB, and
    the bus passive for more than 239 us closes the frame (EOF). Anything
    else is recorded in the frame's ``errors``."""

    def __init__(self, us: float) -> None:
        self.us = us
        self.level = 0
        self.since = 0
        self.frames: List[Frame] = []
        self.open: Optional[Frame] = None

    def clock(self, level: int, cycle: int) -> None:
        if level != self.level:
            self._symbol(Symbol(self.level, self.since, cycle - self.since))
            self.level, self.since = level, cycle
        elif (self.open is not None and not level
              and cycle - self.since > RX_WIN[EOF][0] * self.us):
            self._symbol(Symbol(0, self.since, cycle - self.since))
            self._close()

    def _close(self) -> None:
        if self.open is not None:
            self.open.phase = "eof"
            self.frames.append(self.open)
            self.open = None

    def _symbol(self, s: Symbol) -> None:
        cls = classify(s.us(self.us))
        f = self.open
        if f is None:
            if s.level and cls == SOF:
                self.open = Frame(s.start, [s])
            return
        f.symbols.append(s)
        if s.level and cls == EOF:
            f.errors.append("break: active %.0f us" % s.us(self.us))
            self._close()
        elif f.phase in ("data", "ifr") and cls in (SHORT, LONG):
            bit = int(cls == LONG) if s.level == 0 else int(cls == SHORT)
            f.acc = (f.acc << 1) | bit
            f.nbits += 1
            if f.nbits == 8:
                (f.data if f.phase == "data" else f.ifr).append(f.acc)
                f.acc = f.nbits = 0
        elif not s.level and cls in (SOF, EOF) and f.phase in ("data", "ifr"):
            if f.nbits:
                f.errors.append("EOD after %d bits of a byte" % f.nbits)
            if f.phase == "data":
                f.eod, f.phase = s, "eod"
        elif s.level and cls in (SHORT, LONG) and f.phase == "eod":
            f.nb, f.phase = s, "ifr"
        else:
            f.errors.append("unexpected %s %s symbol, %.1f us" % (
                "active" if s.level else "passive", cls, s.us(self.us)))

    def all_frames(self) -> List[Frame]:
        return self.frames + ([self.open] if self.open is not None else [])


# ------------------------------------------------------------------ checks
def _win(name: str, t: float, lo: Optional[float], hi: Optional[float]) -> List[str]:
    ok = (lo is None or t >= lo) and (hi is None or t <= hi)
    return [] if ok else ["%s %.2f us outside %s..%s" % (name, t, lo, hi)]


def check_tx(frame: Frame, us: float) -> List[str]:
    """Every transmitted symbol of ``frame`` against AN1731's transmit
    windows: SOF, each bit by its class, EOD when an NB ends it, the NB.
    The passive symbol that ends the frame (EOD and EOF) has no maximum.
    Empty if all pass."""
    out = list(frame.errors)
    for i, s in enumerate(frame.symbols):
        t = s.us(us)
        cls = classify(t)
        if i == 0:
            out += _win("SOF", t, TX_WIN[SOF][0], TX_WIN[SOF][2])
        elif s is frame.eod:
            if frame.nb is not None:
                out += _win("EOD", t, TX_WIN[SOF][0], TX_WIN[SOF][2])
            else:
                out += _win("EOD+EOF", t, TX_WIN[SOF][0], None)
        elif cls in (SHORT, LONG):
            out += _win("symbol %d (%s)" % (i, cls), t, TX_WIN[cls][0], TX_WIN[cls][2])
        elif not (s.level == 0 and i == len(frame.symbols) - 1):
            out.append("symbol %d: %s %.2f us" % (i, cls, t))
    return out


# ------------------------------------------------------------------ devices
def nominal(cls: str) -> float:
    return TX_WIN[cls][1]


class VpwNode:
    """A model node on the bus. :meth:`send` queues a transmission, a list
    of ``(level, length)`` symbols (a class name for the nominal length, or
    microseconds), driven open loop from the first cycle the bus has been
    passive for IFS (300 us), or from ``at``. It does not arbitrate: the
    tests that use it never send while another node does. With an
    ``address`` it answers a frame whose second byte is that address, whose
    CRC is good and whose K bit is 0 with a one-byte IFR (its address,
    ``nb`` the NB's class), the NB starting 200 us after the last fall."""

    def __init__(self, us: float, address: Optional[int] = None,
                 nb: str = NB_NO_CRC) -> None:
        self.us = us
        self.address, self.nb = address, nb
        self.decoder = VpwDecoder(us)
        self.queue: List[Tuple[List[Tuple[int, object]], Optional[int]]] = []
        self.sent: List[int] = []           # start cycles of what it sent
        self.answered: List[int] = []       # NB starts of its IFRs
        self.level = 0
        self._active: List[Tuple[float, float]] = []
        self._end = 0.0
        self._answered_frame: Optional[Frame] = None

    def send(self, symbols: Sequence[Tuple[int, object]], at: Optional[int] = None) -> None:
        self.queue.append((list(symbols), at))

    def _plan(self, symbols, t0: float) -> None:
        t = t0
        for level, length in symbols:
            n = nominal(length) if isinstance(length, str) else float(length)
            if level:
                self._active.append((t, t + n * self.us))
            t += n * self.us
        self._end = t

    def clock(self, level: int, cycle: int) -> None:
        d = self.decoder
        d.clock(level, cycle)
        us = self.us
        busy = cycle < self._end
        if not busy and self.queue:
            symbols, at = self.queue[0]
            idle = not level and cycle - d.since >= nominal(IFS) * us
            if (at is not None and cycle >= at) or (at is None and idle):
                self.queue.pop(0)
                self.sent.append(cycle + 1)
                self._plan(symbols, cycle + 1)
        f = d.open
        if (self.address is not None and f is not None and f is not self._answered_frame
                and f.phase == "data" and not level and f.nbits == 0
                and cycle - d.since > RX_WIN[SOF][0] * us):
            self._answered_frame = f
            k = (f.data[0] >> 3) & 1 if f.data else 1
            if (len(f.data) >= 3 and f.data[1] == self.address and k == 0
                    and residue(f.data) == CRC_RESIDUE):
                t0 = d.since + nominal(SOF) * us
                self.answered.append(int(t0))
                self._plan(ifr_symbols([self.address], self.nb), t0)
        nxt = cycle + 1
        self._active = [p for p in self._active if p[1] > nxt]
        self.level = 1 if any(a <= nxt < b for a, b in self._active) else 0


class VpwBus(Model):
    """The J1850 wire on the bench: a wired OR, active (1) dominant.

    Each chip TX pin in ``tx`` is an OUT pin feeding its own transceiver;
    every ``rx`` pin (a ``ui`` input) gets the bus level, as each
    transceiver's receive output would. The bus in cycle ``c`` is active if
    a chip TX pin was 1 in cycle ``c - 1`` (the board wire's one clock) or a
    node drives it. :attr:`decoder` decodes the bus; :attr:`tx_edges` holds
    each chip TX pin's own edges ``(cycle, value)``."""

    def __init__(self, us: float, nodes: Iterable[VpwNode] = (),
                 tx: Sequence[PinSpec] = ("OUT0",), rx: Sequence[PinSpec] = ("IN0",)) -> None:
        self.us = us
        self.nodes = list(nodes)
        self.tx = [pad_of(p) for p in tx]
        self.rx = [pad_of(p) for p in rx]
        self.decoder = VpwDecoder(us)
        self.level = 0
        self.tx_edges = {p: [] for p in self.tx}
        self._tx_last = {p: 0 for p in self.tx}

    def add(self, node: VpwNode) -> VpwNode:
        self.nodes.append(node)
        return node

    def drive(self, drive: Drive, cycle: int) -> None:
        for pad in self.rx:
            drive.set(pad, self.level)

    def observe(self, lines: Lines) -> None:
        cycle = lines.cycle
        self.decoder.clock(self.level, cycle)
        for node in self.nodes:
            node.clock(self.level, cycle)
        chip = 0
        for pad in self.tx:
            v = lines.get(pad)
            if v != self._tx_last[pad]:
                self.tx_edges[pad].append((cycle, v))
                self._tx_last[pad] = v
            chip |= v
        self.level = int(bool(chip) or any(n.level for n in self.nodes))

    def frames(self) -> List[Frame]:
        return self.decoder.all_frames()

    def edges_of(self, pin: PinSpec) -> List[Tuple[int, int]]:
        return self.tx_edges[pad_of(pin)]
