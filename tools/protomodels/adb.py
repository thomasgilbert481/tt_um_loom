"""Apple Desktop Bus (ADB) reference models: a host, a device and a checker.

Written from the protocol's documents, never from ``firmware/adb_*.loom``.
Every figure is cited; "unchecked" marks one this module could not hold
against Apple's own text (Apple's "Guide to the Macintosh Family Hardware",
2nd edition, chapter "Apple Desktop Bus", was not readable here: the PDF is
past the fetch limit, so none of its figures is cited by page).

* Microchip AN591B, "Apple Desktop Bus (ADB)" (1997), which restates the
  ADB specification: p.2, the host's signals within +-3 % and the devices'
  within +-30 %; reset "low for 3 ms", attention "low for 800 us", sync
  "high for 70 us" (Fig. 2 has 70 too, where TMK has 65, below), Srq "low
  for 300 us", an Srq given "within the 65 us of the Stop Bit's low time"
  and held "for a total of 300 us" (Fig. 2); p.3 and Fig. 3, the 100 us
  bit cell, a '1' 35 us low and 65 high, a '0' 65 low and 35 high, the
  command byte (address in the upper nibble, then the command and the
  register), the stop bit a '0', the data packet a '1' start bit, 2 to 8
  bytes and a '0' stop bit, an Srq only from a device the command does not
  address; p.5, the Tlt window 140-260 us, AN591's own receive thresholds
  ('1' low under 50 us, '0' over 50) and the address-conflict rule.
* Apple Technical Note HW01, "ADB - The Untold Story: Space Aliens Ate My
  Mouse": Tlt is 200 us nominal and devices faster than the 140 us minimum
  fail; a Talk register 3 answers a random 4-bit address; a device that saw
  a collision ignores the next Listen register 3 with handler 0xFE; an SRQ
  holds the bus low 300 us after the stop bit; a device with no new data
  does not answer a Talk register 0 and "the bus times out"; registers hold
  2 to 8 bytes; default addresses 2 keyboard, 3 relative pointer.
* TMK's adb.c (tmk_keyboard, ``tmk_core/protocol/adb.c``), a secondary
  source, citing Apple's IIgs Hardware Reference (2nd ed., ch. 6 p.121): the
  bit cell 70-130 us, a 0's low 60-70 % of the cell and a 1's 30-40 %; the
  attention 560-1040 us; the global reset 2.8-5.2 ms; Tlt 140-260 us; the
  attention's 800 us written as 765 + the 35 us low of a 1, so the sync is
  that 1's 65 us high (the 65 us of the D-035 brief; AN591 says 70).
* Register 3 (the Guide, unchecked; consistent with HW01's walk-through):
  bit 13 SRQ enable, bits 11:8 the address, 7:0 the handler ID; handler
  0xFE moves the address only without a collision, 0x00 sets the address
  and the SRQ enable bit; 0xFD and 0xFF and handler changes are not modelled
  here (the device keeps its one handler).

Times are clocks of the chip (50 MHz unless ``clock`` says otherwise); the
constants are microseconds. The wire is ``OneWireBus`` of
``tools.protomodels.onewire``: one open-drain line with its pull-up, one
pad per Loom thread.
"""

from __future__ import annotations

import random
from typing import Dict, List, Optional, Sequence, Tuple

from .onewire import CLOCK, Agent, OneWireBus  # noqa: F401  (the line model)

#: Nominal figures, microseconds (module doc).
ADB = dict(
    attention=800, sync=65, cell=100, low0=65, low1=35, srq=300,
    tlt=200, reset=3000,
)

#: The checker's windows, microseconds, inclusive, with their source.
WINDOWS: Dict[str, Tuple[float, float, str]] = dict(
    attention=(776, 824, "AN591 p.2: 800 us, host +-3 %"),
    sync=(63, 72.1, "TMK 65 +-3 %, AN591 70 +-3 %"),
    cell_host=(97, 103, "AN591 p.2: 100 us, host +-3 %"),
    cell_dev=(70, 130, "AN591 p.2 +-30 %; TMK 70-130"),
    ratio0=(0.60, 0.70, "TMK (IIgs HW Ref p.121): a 0 is low 60-70 % of its cell"),
    ratio1=(0.30, 0.40, "TMK (IIgs HW Ref p.121): a 1 is low 30-40 % of its cell"),
    stop_host=(63.05, 66.95, "the stop bit is a 0: 65 us low, host +-3 %"),
    stop_dev=(45.5, 84.5, "a 0's low, device +-30 %"),
    tlt=(140, 260, "HW01 140 min, 200 nominal; AN591 p.5 140-260"),
    srq=(210, 390, "AN591, HW01: 300 us, device +-30 % (unchecked)"),
    reset=(2800, 5200, "TMK 2.8-5.2 ms; AN591 3 ms"),
)

THRESHOLD_US = 50          # AN591 p.5: a '1' low under 50 us, a '0' over
TALK, LISTEN = 3, 2


def talk(addr: int, reg: int) -> int:
    """The command byte AAAA 11 RR (AN591 p.3)."""
    return (addr & 15) << 4 | 0b1100 | (reg & 3)


def listen(addr: int, reg: int) -> int:
    return (addr & 15) << 4 | 0b1000 | (reg & 3)


def flush(addr: int) -> int:
    return (addr & 15) << 4 | 0b0001


SEND_RESET = 0x00


def bits_of(data: bytes) -> List[int]:
    return [(b >> (7 - i)) & 1 for b in data for i in range(8)]


def bytes_of(bits: Sequence[int]) -> bytes:
    out = bytearray()
    for i in range(0, len(bits) - len(bits) % 8, 8):
        v = 0
        for b in bits[i:i + 8]:
            v = v << 1 | b
        out.append(v)
    return bytes(out)


def _cyc(us: float, clock: int) -> int:
    return int(round(us * clock / 1_000_000))


def _us(cycles: float, clock: int) -> float:
    return cycles * 1_000_000 / clock


def check_cells(edges: Sequence[Tuple[int, int]], *, clock: int = CLOCK,
                role: str = "dev", srq_ok: bool = False) -> dict:
    """Decode and check a run of cells: ``edges`` are the line's changes
    ``(cycle, level)`` from the first cell's fall on, ending with the rise
    of the last cell's low (the stop bit). Every cell but the last is held
    to the cell window of ``role`` (``"host"`` +-3 %, ``"dev"`` +-30 %) and
    to its bit's low ratio (TMK); the last is the stop bit, a 0 by its low,
    or an SRQ when ``srq_ok`` and its low is in the SRQ window. Returns
    ``bits`` (by the low against :data:`THRESHOLD_US`, the stop bit
    included), ``cells`` (``(low_us, period_us or None)``), ``srq`` (the
    stop low in us if it was one, else None) and ``violations`` (``(name,
    index, value, window)``)."""
    viol: List[tuple] = []
    bits: List[int] = []
    cells: List[Tuple[float, Optional[float]]] = []
    falls = [c for c, lvl in edges if lvl == 0]
    rises = [c for c, lvl in edges if lvl == 1]
    if not falls or len(rises) < len(falls) or edges[0][1] != 0:
        return dict(bits=[], cells=[], srq=None, violations=[("missing", 0, None, None)])
    srq = None
    for i, f in enumerate(falls):
        low = _us(rises[i] - f, clock)
        nxt = falls[i + 1] if i + 1 < len(falls) else None
        period = _us(nxt - f, clock) if nxt is not None else None
        cells.append((low, period))
        bit = int(low < THRESHOLD_US)
        if period is None:                          # the stop bit
            lo, hi, _ = WINDOWS["stop_" + role]
            if srq_ok and WINDOWS["srq"][0] <= low <= WINDOWS["srq"][1]:
                srq = low
                bit = 0
            elif not lo <= low <= hi:
                viol.append(("stop", i, low, (lo, hi)))
            if bit != 0:
                viol.append(("stop_value", i, bit, 0))
        else:
            lo, hi, _ = WINDOWS["cell_" + role]
            if not lo <= period <= hi:
                viol.append(("cell", i, period, (lo, hi)))
            rlo, rhi, _ = WINDOWS["ratio%d" % bit]
            if not rlo - 1e-9 <= low / period <= rhi + 1e-9:
                viol.append(("ratio", i, round(low / period, 3), (rlo, rhi)))
        bits.append(bit)
    return dict(bits=bits, cells=cells, srq=srq, violations=viol)


def check_command(edges: Sequence[Tuple[int, int]], *, clock: int = CLOCK) -> dict:
    """A host's command from the attention's fall: the attention, the sync,
    eight cells and the stop bit (or an SRQ on it), all at the host's +-3 %.
    Returns ``command`` (the byte, or None), ``srq`` and ``violations``."""
    viol: List[tuple] = []
    if len(edges) < 3 or edges[0][1] != 0:
        return dict(command=None, srq=None, violations=[("missing", 0, None, None)])
    att = _us(edges[1][0] - edges[0][0], clock)
    sync = _us(edges[2][0] - edges[1][0], clock)
    for name, value in (("attention", att), ("sync", sync)):
        lo, hi, _ = WINDOWS[name]
        if not lo <= value <= hi:
            viol.append((name, 0, value, (lo, hi)))
    res = check_cells(edges[2:20], clock=clock, role="host", srq_ok=True)
    viol += res["violations"]
    command = None
    if len(res["bits"]) == 9:
        command = 0
        for b in res["bits"][:8]:
            command = command << 1 | b
    else:
        viol.append(("cells", 0, len(res["bits"]), 9))
    return dict(command=command, srq=res["srq"], violations=viol)


def check_reply(edges: Sequence[Tuple[int, int]], bus_high: int, *,
                clock: int = CLOCK, role: str = "dev") -> dict:
    """A data packet after a command: ``edges`` from the start bit's fall,
    ``bus_high`` the cycle the line rose after the command's stop bit (or
    its SRQ). Holds Tlt to its window, the start bit to a 1, every cell to
    ``role``'s windows; returns ``data``, ``tlt`` and ``violations``."""
    viol: List[tuple] = []
    if not edges:
        return dict(data=b"", tlt=None, bits=[], violations=[("missing", 0, None, None)])
    tlt = _us(edges[0][0] - bus_high, clock)
    lo, hi, _ = WINDOWS["tlt"]
    if not lo <= tlt <= hi:
        viol.append(("tlt", 0, tlt, (lo, hi)))
    res = check_cells(edges, clock=clock, role=role)
    viol += res["violations"]
    bits = res["bits"]
    if not bits or bits[0] != 1:
        viol.append(("start", 0, bits[:1], 1))
    body = bits[1:-1]
    if len(body) % 8 or not 2 <= len(body) // 8 <= 8:
        viol.append(("length", 0, len(body), "8 x (2..8)"))
    return dict(data=bytes_of(body), tlt=tlt, bits=bits, violations=viol)


class _Driver(Agent):
    """An open-drain agent that runs a plan: ``(cycle, pull)`` entries take
    effect from that cycle (the bus registers the answer, so each is given
    one clock early), ``(cycle, fn)`` entries call ``fn(cycle, level)`` in
    that cycle. ``on_edge`` sees every change of the line and ``on_timer``
    the cycle :attr:`timer` names."""

    def __init__(self, clock: int) -> None:
        self.clock_hz = clock
        self._plan: List[tuple] = []
        self._pull = False
        self._level = 1
        self.timer: Optional[int] = None
        self.wake = 0

    def cyc(self, us: float) -> int:
        return _cyc(us, self.clock_hz)

    def us(self, cycles: float) -> float:
        return _us(cycles, self.clock_hz)

    def cells_plan(self, t: int, bits: Sequence[int], *, l0: float, l1: float,
                   cell: float, check=None) -> int:
        """Append the cells of ``bits`` from the fall at ``t``; a ``check``
        callback goes 50 us into each 1. Returns the next cell's fall."""
        for b in bits:
            self._plan.append((t, True))
            self._plan.append((t + self.cyc(l1 if b else l0), False))
            if b and check is not None:
                self._plan.append((t + self.cyc(THRESHOLD_US), check))
            t += self.cyc(cell)
        return t

    def on_edge(self, cycle: int, level: int) -> None:
        pass

    def on_timer(self, cycle: int) -> None:
        pass

    def clock(self, level: int, cycle: int) -> bool:
        if level != self._level:
            self._level = level
            self.on_edge(cycle, level)
        while self._plan:
            t, what = self._plan[0]
            if callable(what):
                if t > cycle:
                    break
                self._plan.pop(0)
                what(cycle, level)
            else:
                if t > cycle + 1:
                    break
                self._plan.pop(0)
                self._pull = what
        if self.timer is not None and cycle >= self.timer:
            self.timer = None
            self.on_timer(cycle)
        nxt = [self.timer] if self.timer is not None else []
        if self._plan:
            t, what = self._plan[0]
            nxt.append(t if callable(what) else t - 1)
        self.wake = max(cycle + 1, min(nxt)) if nxt else 1 << 62
        return self._pull


class Transaction:
    """One transaction of :class:`AdbHost`, filled in when ``done``."""

    def __init__(self, command: Optional[int], data: bytes = b"") -> None:
        self.command, self.data = command, bytes(data)
        self.done = False
        self.edges: List[Tuple[int, int]] = []
        self.srq = False
        self.bus_high: Optional[int] = None
        self.reply_edges: List[Tuple[int, int]] = []
        self.reply: Optional[dict] = None          # check_reply's result

    @property
    def answer(self) -> Optional[bytes]:
        """The reply's bytes, or None for no reply."""
        return self.reply["data"] if self.reply else None

    @property
    def violations(self) -> List[tuple]:
        return self.reply["violations"] if self.reply else []

    def __repr__(self) -> str:
        return "Transaction(%r, done %s, srq %s, answer %r)" % (
            self.command, self.done, self.srq, self.answer)


class AdbHost(_Driver):
    """An ADB host for testing a device (module doc). :meth:`command` starts
    a transaction at the next clock the bus gives it: the attention, the
    sync, the byte, the stop bit; an SRQ is seen 10 us after the stop bit's
    release; then a Talk waits ``reply_to_us`` after the bus goes high for
    a start bit and records the reply until the line has been high
    ``quiet_us``, a Listen sends its data packet Tlt after the bus goes
    high. Error injection: ``l0_us``, ``l1_us``, ``cell_us``,
    ``attention_us``, ``sync_us`` and ``tlt_us`` change the host's widths;
    ``stop=False`` leaves out the command's stop bit."""

    name = "host"

    def __init__(self, *, clock: int = CLOCK, l0_us: float = 65, l1_us: float = 35,
                 cell_us: float = 100, attention_us: float = 800, sync_us: float = 65,
                 tlt_us: float = 200, stop: bool = True, reply_to_us: float = 300,
                 quiet_us: float = 200) -> None:
        super().__init__(clock)
        self.l0_us, self.l1_us, self.cell_us = l0_us, l1_us, cell_us
        self.attention_us, self.sync_us, self.tlt_us = attention_us, sync_us, tlt_us
        self.stop, self.reply_to_us, self.quiet_us = stop, reply_to_us, quiet_us
        self.transactions: List[Transaction] = []
        self.tx: Optional[Transaction] = None
        self._state = "idle"
        self._start = False
        self._last_rise = 0
        self.wake = 1 << 62

    # ------------------------------------------------------------ requests
    def command(self, byte: int, data: bytes = b"") -> Transaction:
        tx = Transaction(byte & 0xFF, data)
        self.transactions.append(tx)
        self.tx, self._start, self.wake = tx, True, 0
        return tx

    def talk(self, addr: int, reg: int) -> Transaction:
        return self.command(talk(addr, reg))

    def listen(self, addr: int, reg: int, data: bytes) -> Transaction:
        return self.command(listen(addr, reg), data)

    def reset(self, low_us: float = 3000) -> Transaction:
        """The global reset: the line low ``low_us``."""
        tx = Transaction(None)
        self.transactions.append(tx)
        self.tx, self._start, self.wake = tx, low_us, 0
        return tx

    # ---------------------------------------------------------------- clock
    def clock(self, level: int, cycle: int) -> bool:
        if self._start is not False:
            self._begin(cycle + 1)
        return super().clock(level, cycle)

    def _begin(self, t: int) -> None:
        tx, start = self.tx, self._start
        self._start = False
        self.timer = None
        if tx.command is None:                       # the global reset
            self._plan = [(t, True), (t + self.cyc(start), False),
                          (t + self.cyc(start + self.quiet_us), self._finish)]
            self._state = "reset"
            return
        self._plan = [(t, True)]
        t += self.cyc(self.attention_us)
        self._plan.append((t, False))
        t += self.cyc(self.sync_us)
        bits = bits_of(bytes([tx.command]))
        t = self.cells_plan(t, bits + ([0] if self.stop else []), l0=self.l0_us,
                            l1=self.l1_us, cell=self.cell_us)
        if self.stop:
            release = self._plan[-1][0]
            self._plan.append((release + self.cyc(10), self._srq_test))
        else:
            self._plan.append((t, self._srq_test))
        self._state = "command"

    def _srq_test(self, cycle: int, level: int) -> None:
        self.tx.srq = level == 0
        self._state = "stop"
        if level == 1:
            self._bus_high(self._last_rise)

    def on_edge(self, cycle: int, level: int) -> None:
        tx = self.tx
        if tx is None:
            return
        if level == 1:
            self._last_rise = cycle
        if self._state in ("command", "stop", "reset"):
            tx.edges.append((cycle, level))
            if self._state == "stop" and level == 1:
                self._bus_high(cycle)
        elif self._state == "reply":
            tx.reply_edges.append((cycle, level))
            self.timer = cycle + self.cyc(self.quiet_us) if level == 1 else None

    def _bus_high(self, cycle: int) -> None:
        tx = self.tx
        tx.bus_high = cycle
        code = tx.command >> 2 & 3
        if code == TALK and tx.command & 15:
            self._state = "reply"
            self.timer = cycle + self.cyc(self.reply_to_us)
        elif code == LISTEN:
            self._state = "send"
            t = cycle + self.cyc(self.tlt_us)
            t = self.cells_plan(t, [1] + bits_of(tx.data) + [0], l0=self.l0_us,
                                l1=self.l1_us, cell=self.cell_us)
            self._plan.append((self._plan[-1][0] + self.cyc(self.quiet_us), self._finish))
        else:
            self._state = "quiet"
            self._plan.append((cycle + self.cyc(self.quiet_us), self._finish))

    def on_timer(self, cycle: int) -> None:
        tx = self.tx
        if self._state == "reply":
            if tx.reply_edges:
                tx.reply = check_reply(tx.reply_edges, tx.bus_high, clock=self.clock_hz)
            self._finish(cycle, 1)

    def _finish(self, cycle: int, level: int) -> None:
        self.tx.done = True
        self._state = "idle"
        self.wake = 1 << 62

    def command_check(self, tx: Transaction) -> dict:
        """The host's own command held to the windows (a self-check)."""
        return check_command(tx.edges, clock=self.clock_hz)


class AdbDevice(_Driver):
    """An ADB device (module doc), keyboard- or mouse-like: register 0 is a
    queue of reports (:meth:`report`), register 3 the address, SRQ enable
    and handler; registers 1 and 2 hold what a Listen wrote (a Talk of an
    empty one is not answered). It answers Tlt after the bus goes high,
    detects collisions 50 us into each 1 it sends, asks for service on
    another device's command while it has a report, and keeps every
    command it saw (:attr:`commands`) with the host's widths checked
    (:attr:`violations`). Error injection: ``l0_us``, ``l1_us``,
    ``cell_us``, ``tlt_us``, ``srq_us``; ``stop=False`` leaves out the
    reply's stop bit; ``srq=False`` never asks. Give two devices on one bus
    different ``name``s."""

    def __init__(self, address: int = 3, handler: int = 1, *, clock: int = CLOCK,
                 name: str = "device", seed: int = 1, l0_us: float = 65,
                 l1_us: float = 35, cell_us: float = 100, tlt_us: float = 200,
                 srq_us: float = 300, stop: bool = True, srq: bool = True) -> None:
        super().__init__(clock)
        self.name = name
        self.default = (address & 15, handler & 0xFF)
        self.rng = random.Random(seed)
        self.l0_us, self.l1_us, self.cell_us = l0_us, l1_us, cell_us
        self.tlt_us, self.srq_us, self.stop, self.srq = tlt_us, srq_us, stop, srq
        self.queue: List[bytes] = []
        self.regs: Dict[int, bytes] = {1: b"", 2: b""}
        self.commands: List[Tuple[int, int]] = []     # (cycle of the stop, byte)
        self.violations: List[tuple] = []
        self.listens: List[Tuple[int, bytes]] = []
        self.sent: List[bytes] = []
        self.srqs: List[int] = []
        self.collisions = 0
        self.resets = 0
        self.wake = 1 << 62
        self._reset()
        self._state, self._fall = "idle", None

    def _reset(self) -> None:
        self.address, self.handler = self.default
        self.srq_enable, self.collided = True, False
        self.queue = []

    def report(self, data: bytes) -> None:
        """Queue a register-0 report (2..8 bytes)."""
        self.queue.append(bytes(data))

    @property
    def register3(self) -> int:
        return self.srq_enable << 13 | self.address << 8 | self.handler

    # -------------------------------------------------------------- receive
    def on_edge(self, cycle: int, level: int) -> None:
        st = self._state
        if st == "tx":
            return
        if level == 0:
            if st == "cmd":
                self._edges.append((cycle, 0))
                self.timer = None
                if len(self._bits) == 8:
                    self._stop_fall(cycle)
            elif st == "lrx":
                self._edges.append((cycle, 0))
                self.timer = None
            self._fall = cycle
            return
        if self._fall is None:
            return
        low = self.us(cycle - self._fall)
        if st == "idle":
            if low >= WINDOWS["reset"][0]:
                self.resets += 1
                self._reset()
            elif 560 <= low <= 1040:                   # TMK: the attention
                self._state, self._bits = "cmd", []
                self._edges = [(self._fall, 0), (cycle, 1)]
                self.timer = cycle + self.cyc(200)
        elif st == "cmd":
            self._edges.append((cycle, 1))
            self._bits.append(int(low < THRESHOLD_US))
            self.timer = cycle + self.cyc(200)
        elif st == "stop":
            self._edges.append((cycle, 1))
            self._command_done(cycle)
        elif st == "lrx":
            self._edges.append((cycle, 1))
            self.timer = cycle + self.cyc(150)

    def on_timer(self, cycle: int) -> None:
        if self._state == "lrx" and self._edges:
            res = check_reply(self._edges, self._bus, clock=self.clock_hz, role="host")
            self.violations += res["violations"]
            self._listened(res["data"])
        self._state, self._fall = "idle", None

    def _stop_fall(self, cycle: int) -> None:
        """The command's stop bit begins: the byte is known; an SRQ now."""
        byte = 0
        for b in self._bits:
            byte = byte << 1 | b
        self._byte = byte
        self._state = "stop"
        mine = byte >> 4 == self.address
        if (not mine and byte & 15 and self.queue and self.srq and self.srq_enable):
            self.srqs.append(cycle)
            self._plan = [(cycle + 1, True), (cycle + self.cyc(self.srq_us), False)]

    def _command_done(self, cycle: int) -> None:
        """The bus is high after the stop bit (or an SRQ): act on the byte."""
        byte = self._byte
        self.commands.append((cycle, byte))
        self.violations += check_command(self._edges, clock=self.clock_hz)["violations"]
        self._state, self._fall, self._bus = "idle", None, cycle
        if byte & 15 == 0:                              # SendReset
            self._reset()
            return
        if byte >> 4 != self.address:
            return
        code, reg = byte >> 2 & 3, byte & 3
        if code == TALK:
            data = self._talk_data(reg)
            if data:
                self._send(cycle + self.cyc(self.tlt_us), data, reg)
        elif code == LISTEN:
            self._state, self._edges = "lrx", []
            self.timer = cycle + self.cyc(300)
        elif byte & 15 == 1:                            # Flush
            self.queue = []

    def _talk_data(self, reg: int) -> bytes:
        if reg == 0:
            return self.queue[0] if self.queue else b""
        if reg == 3:
            addr = self.rng.randrange(16)               # HW01: a random address
            return bytes([self.srq_enable << 5 | addr, self.handler])
        return self.regs[reg]

    def _listened(self, data: bytes) -> None:
        reg = self._byte & 3
        self.listens.append((reg, data))
        if reg != 3:
            self.regs.setdefault(reg, b"")
            if reg in (1, 2):
                self.regs[reg] = data
            return
        if len(data) < 2:
            return
        if data[1] == 0xFE and not self.collided:
            self.address = data[0] & 15
        elif data[1] == 0x00:
            self.address = data[0] & 15
            self.srq_enable = bool(data[0] >> 5 & 1)

    # ----------------------------------------------------------------- send
    def _send(self, t: int, data: bytes, reg: int) -> None:
        self._state = "tx"
        bits = [1] + bits_of(data) + ([0] if self.stop else [])
        self._plan = []
        self.cells_plan(t, bits, l0=self.l0_us, l1=self.l1_us, cell=self.cell_us,
                        check=self._collision_test)
        self._plan.sort(key=lambda e: e[0])
        end = self._plan[-1][0] + self.cyc(10)
        self._plan.append((end, lambda c, lvl: self._sent(data, reg)))

    def _collision_test(self, cycle: int, level: int) -> None:
        if level == 0:                  # a 1 we sent is held low: another device
            self.collisions += 1
            self.collided = True
            self._plan, self._pull = [], False
            self._state, self._fall = "idle", None

    def _sent(self, data: bytes, reg: int) -> None:
        self.sent.append(data)
        if reg == 0 and self.queue:
            self.queue.pop(0)
        elif reg == 3:
            self.collided = False
        self._state, self._fall = "idle", None
