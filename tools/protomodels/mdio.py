"""MDIO reference models: the IEEE 802.3 Ethernet PHY management bus.

Written from the specification, not from the firmware:

* IEEE 802.3, clause 22.2.2.13 (MDC: sourced by the station management
  entity, the manager, "aperiodic", no maximum high or low time),
  22.2.2.14 (MDIO: bidirectional, open drain on most boards with a pull-up),
  22.2.4.5 (the management frame: PRE, ST, OP, PHYAD, REGAD, TA, DATA, and
  22.2.4.5.1 the 32-bit preamble the PHY shall observe) and 22.3.4 (MDC and
  MDIO timing). Clause 45.3 adds the ST = 00 frames (OP 00 address, 01
  write, 11 read, 10 read-increment) with PRTAD and DEVAD in place of PHYAD
  and REGAD.
* The timing figures used here, 22.3.4: MDC period at least 400 ns, high
  and low each at least 160 ns; MDIO driven by the manager has set-up and
  hold of at least 10 ns about the MDC rise; MDIO driven by the PHY is valid
  0 to 300 ns after the MDC rise (clock to output).
  **Unchecked against the IEEE text in this session** (the author had no
  copy open): these are the figures as quoted in PHY datasheets' MDIO timing
  tables, for example Microchip LAN8742A, "MDIO timing" (also unchecked
  against a printed copy). QUESTIONS.md records it.
* The clause and table numbers are from IEEE 802.3-2018/2022 (the clause
  numbers have not moved since 802.3-2002); figure and table numbers are not
  quoted for that reason.

What is here:

* frame helpers (:func:`header`, :func:`frame_bits`, :func:`decode_bits`);
* :class:`MdioMonitor`, a bus analyser on MDC and MDIO: it decodes every
  frame from the resolved pads (the bit at each MDC rise) and checks every
  edge against 22.3.4, listing what it finds in ``violations``;
* :class:`MdioManager`, a model manager that drives MDC and MDIO, with
  error injection (a short preamble, a wrong turnaround on a write);
* :class:`MdioPhy`, a model PHY with a register file, a chosen clock-to-
  output delay, and error injection (no answer, a turnaround driven early);
* :class:`OpenDrainWire` and :class:`Wire`, the board wiring between two
  Loom threads (COMMON: two threads never share a pin).

Times are in core clocks; ``clock_hz`` converts the nanosecond figures.
"""

from __future__ import annotations

from collections import deque
from typing import Deque, Dict, List, Optional, Sequence, Tuple

from .bench import UIO, Drive, Lines, Model, PinSpec, pad_of

# Frame fields (22.2.4.5, 45.3)
ST_C22, ST_C45 = 0b01, 0b00
OP_WRITE, OP_READ = 0b01, 0b10                  # clause 22
OP45_ADDR, OP45_WRITE, OP45_READ, OP45_RINC = 0b00, 0b01, 0b11, 0b10
PREAMBLE = 32                                   # 22.2.4.5.1

# Timing (22.3.4; see the docstring for what was checked)
MDC_PERIOD_NS = 400.0
MDC_HIGH_NS = MDC_LOW_NS = 160.0
SETUP_NS = HOLD_NS = 10.0
CLK_TO_OUT_NS = 300.0


def ns_to_clocks(ns: float, clock_hz: float) -> float:
    return ns * clock_hz / 1e9


def header(st: int, op: int, phyad: int, regad: int) -> int:
    """The 16-bit word ST OP PHYAD REGAD TA as it goes on the wire, MSB
    first, with TA = 10 (what the manager sends on a write)."""
    return (((st & 3) << 14) | ((op & 3) << 12) | ((phyad & 31) << 7)
            | ((regad & 31) << 2) | 0b10)


def phy_drives_data(op: int) -> bool:
    """OP[1] = 1: the PHY drives TA's second bit and DATA (22.2.4.5, 45.3)."""
    return bool(op & 2)


def frame_bits(st: int, op: int, phyad: int, regad: int, data: int = 0xFFFF,
               preamble: int = PREAMBLE, ta: Optional[Tuple[int, int]] = None
               ) -> List[Optional[int]]:
    """The bits of one frame as the manager puts them on MDIO, one per MDC
    cycle; ``None`` where the manager lets go (the turnaround and data of a
    read). ``ta`` overrides the turnaround of a write (error injection)."""
    bits: List[Optional[int]] = [1] * preamble
    word = header(st, op, phyad, regad)
    bits += [(word >> i) & 1 for i in range(15, 1, -1)]
    if phy_drives_data(op):
        bits += [None] * 18
    else:
        t = ta if ta is not None else (1, 0)
        bits += list(t) + [(data >> i) & 1 for i in range(15, -1, -1)]
    return bits


def decode_bits(bits: Sequence[int]) -> Optional[Dict[str, int]]:
    """Decode the 32 bits after a preamble (ST to the last data bit)."""
    if len(bits) < 32:
        return None
    v = 0
    for b in bits[:32]:
        v = (v << 1) | (b & 1)
    return {"st": v >> 30, "op": (v >> 28) & 3, "phyad": (v >> 23) & 31,
            "regad": (v >> 18) & 31, "ta": (v >> 16) & 3, "data": v & 0xFFFF}


class MdioMonitor(Model):
    """A bus analyser: decodes frames on MDC and MDIO and checks 22.3.4.

    A bit is the MDIO level in the cycle before the MDC rise (the value the
    rising edge samples). Hunting, it counts ones; the first 0 is ST[1] and
    the frame is the 32 bits from there, recorded in ``frames`` with the
    preamble length it had. ``violations`` lists every timing fault as
    ``(cycle, text)``: MDC high, low or period too short; a manager-driven
    bit changing within the set-up or hold time of its rise; a PHY-driven
    bit (TA[0] and DATA of a read) still changing later than the clock-to-
    output time after the rise before the one that samples it.
    """

    def __init__(self, mdc: PinSpec, mdio: PinSpec, *, clock_hz: float = 50e6) -> None:
        self.mdc_pad, self.io_pad = pad_of(mdc), pad_of(mdio)
        c = lambda ns: ns_to_clocks(ns, clock_hz)
        self.period_c, self.high_c, self.low_c = c(MDC_PERIOD_NS), c(MDC_HIGH_NS), c(MDC_LOW_NS)
        self.setup_c, self.hold_c, self.cto_c = c(SETUP_NS), c(HOLD_NS), c(CLK_TO_OUT_NS)
        self.prev_mdc: Optional[int] = None
        self.prev_io: Optional[int] = None
        self.last_rise = self.last_fall = None
        self.changes: Deque[int] = deque(maxlen=8)  # recent MDIO change cycles
        self.rises: List[int] = []
        self.falls: List[int] = []
        self.ones = 0
        self.bits: Optional[List[int]] = None       # inside a frame
        self.frame_rises: List[int] = []
        self.pending_hold: Optional[int] = None     # rise whose hold is open
        self.frames: List[Dict[str, int]] = []
        self.violations: List[Tuple[int, str]] = []

    def _bad(self, cycle: int, text: str) -> None:
        self.violations.append((cycle, text))

    # bit k of a frame (0 = ST[1]) is driven by the manager?
    def _manager_bit(self, k: int) -> Optional[bool]:
        if k < 14:
            return True
        op = ((self.bits[2] << 1) | self.bits[3]) if len(self.bits) >= 4 else 0
        if phy_drives_data(op):
            return None if k == 14 else False     # TA[1]: nobody
        return True

    def _rise(self, r: int, bit: int) -> None:
        if self.last_rise is not None and r - self.last_rise < self.period_c:
            self._bad(r, "MDC period %d clocks" % (r - self.last_rise))
        if self.last_fall is not None and r - self.last_fall < self.low_c:
            self._bad(r, "MDC low %d clocks" % (r - self.last_fall))
        prev_rise = self.last_rise
        self.last_rise = r
        self.rises.append(r)
        if self.bits is None:
            if bit:
                self.ones += 1
                return
            self.bits, self.frame_rises, self.preamble = [], [], self.ones
            self.ones = 0
        k = len(self.bits)
        self.bits.append(bit)
        self.frame_rises.append(r)
        who = self._manager_bit(k)
        last = self.changes[-1] if self.changes else None
        if who is True:
            if last is not None and r - last < self.setup_c:
                self._bad(r, "frame bit %d: MDIO set-up %d clocks" % (k, r - last))
            self.pending_hold = r
        elif who is False and prev_rise is not None:
            if last is not None and last > prev_rise + self.cto_c:
                self._bad(r, "frame bit %d: PHY output %d clocks after the rise"
                          % (k, last - prev_rise))
        if len(self.bits) == 32:
            frame = decode_bits(self.bits)
            frame.update(preamble=self.preamble, start=self.frame_rises[0],
                         end=r, rises=list(self.frame_rises))
            self.frames.append(frame)
            self.bits = None

    def observe(self, lines: Lines) -> None:
        cyc = lines.cycle
        mdc, io = lines.get(self.mdc_pad), lines.get(self.io_pad)
        if self.prev_io is not None and io != self.prev_io:
            self.changes.append(cyc)
            if self.pending_hold is not None and cyc - self.pending_hold < self.hold_c:
                self._bad(cyc, "MDIO hold %d clocks" % (cyc - self.pending_hold))
            self.pending_hold = None
        if self.prev_mdc is not None and mdc != self.prev_mdc:
            if mdc:
                self._rise(cyc, self.prev_io if self.prev_io is not None else io)
            else:
                if self.last_rise is not None and cyc - self.last_rise < self.high_c:
                    self._bad(cyc, "MDC high %d clocks" % (cyc - self.last_rise))
                self.last_fall = cyc
                self.falls.append(cyc)
        self.prev_mdc, self.prev_io = mdc, io


class MdioManager(Model):
    """A model manager: drives MDC (push-pull) and MDIO (open drain).

    Each MDC cycle is ``low`` clocks low then ``high`` clocks high. MDIO
    changes at the fall (so its hold after a rise is the whole high phase)
    and the manager samples it in the last low cycle before each rise.
    Frames queue with :meth:`read` and :meth:`write` and run back to back,
    ``gap`` clocks apart (MDC low, MDIO released); each finished frame is
    appended to ``results`` with the bits sampled after its preamble decoded
    (``data`` and ``ta`` are what the PHY drove on a read).

    Error injection: ``preamble`` shorter than 32 (a PHY must ignore the
    frame) and ``ta`` other than (1, 0) on a write.
    """

    def __init__(self, mdc: PinSpec, mdio: PinSpec, *, period: int = 20,
                 high: Optional[int] = None, gap: int = 0) -> None:
        self.mdc_pad, self.io_pad = pad_of(mdc), pad_of(mdio)
        if self.io_pad[0] != UIO:
            raise ValueError("MDIO must be a bidirectional (uio) pad")
        self.high = period // 2 if high is None else high
        self.low = period - self.high
        self.gap = gap
        self.queue: Deque[Tuple[List[Optional[int]], Dict[str, int]]] = deque()
        self.results: List[Dict[str, int]] = []
        self.mdc, self.out = 0, None
        self.cur: Optional[List[Optional[int]]] = None
        self.info: Dict[str, int] = {}
        self.sampled: List[int] = []
        self.i = 0
        self.phase, self.count = "idle", gap

    def read(self, phyad: int, regad: int, *, st: int = ST_C22, op: int = OP_READ,
             preamble: int = PREAMBLE) -> None:
        self._queue(st, op, phyad, regad, 0xFFFF, preamble, None)

    def write(self, phyad: int, regad: int, data: int, *, st: int = ST_C22,
              op: int = OP_WRITE, preamble: int = PREAMBLE,
              ta: Optional[Tuple[int, int]] = None) -> None:
        self._queue(st, op, phyad, regad, data, preamble, ta)

    def _queue(self, st, op, phyad, regad, data, preamble, ta) -> None:
        bits = frame_bits(st, op, phyad, regad, data, preamble, ta)
        info = dict(st=st, op=op, phyad=phyad, regad=regad, data=data,
                    preamble=preamble)
        self.queue.append((bits, info))

    @property
    def busy(self) -> bool:
        return self.cur is not None or bool(self.queue)

    def drive(self, drive: Drive, cycle: int) -> None:
        drive.set(self.mdc_pad, self.mdc)
        if self.out == 0:
            drive.pull_low(self.io_pad)

    def observe(self, lines: Lines) -> None:
        if self.count > 1:
            self.count -= 1
            return
        if self.phase == "idle":
            if self.queue:
                self.cur, self.info = self.queue.popleft()
                self.sampled, self.i = [], 0
                self.out = self.cur[0]
                self.phase, self.count = "low", self.low
            return
        if self.phase == "low":                     # the rise: sample first
            self.sampled.append(lines.get(self.io_pad))
            self.mdc, self.phase, self.count = 1, "high", self.high
            return
        self.mdc = 0                                # the fall: next bit
        self.i += 1
        if self.i < len(self.cur):
            self.out = self.cur[self.i]
            self.phase, self.count = "low", self.low
            return
        n = self.info["preamble"]
        got = decode_bits(self.sampled[n:])
        result = dict(self.info)
        result.update(ta=got["ta"], data=got["data"], sampled=got)
        self.results.append(result)
        self.cur, self.out = None, None
        self.phase, self.count = "idle", max(self.gap, 1)


class MdioPhy(Model):
    """A model PHY on MDC and MDIO (open drain), clause 22 and clause 45.

    It needs 32 ones with their rises before ST (22.2.4.5.1: a shorter
    preamble is ignored), samples MDIO in the cycle before each MDC rise,
    and answers a read addressed to it by pulling MDIO for TA[0] and the
    data bits, each ``delay`` clocks after the rise before the one that
    samples it (the clock-to-output time; 22.3.4 allows 0 to 300 ns).

    Clause 22: ``regs[n]``. Clause 45: ``mmd[(devad, address)]``, with the
    address register of each MMD set by an OP 00 frame and moved on by a
    read-increment (OP 10). Every frame seen is appended to ``frames`` with
    ``ours`` (addressed to this PHY) and, for writes, ``ta`` as received.

    Error injection: ``absent`` (never drives: the manager must see no
    answer) and ``early_ta`` (drives TA[1] low as well, which nobody may).
    """

    def __init__(self, mdc: PinSpec, mdio: PinSpec, *, phyad: int = 1,
                 regs: Optional[Dict[int, int]] = None, delay: int = 5,
                 absent: bool = False, early_ta: bool = False) -> None:
        self.mdc_pad, self.io_pad = pad_of(mdc), pad_of(mdio)
        self.phyad, self.delay = phyad, delay
        self.absent, self.early_ta = absent, early_ta
        self.regs: Dict[int, int] = dict(regs or {})
        self.mmd: Dict[Tuple[int, int], int] = {}
        self.mmd_addr: Dict[int, int] = {}
        self.frames: List[Dict[str, int]] = []
        self.prev_mdc, self.prev_io = 0, 1
        self.ones = 0
        self.bits: Optional[List[int]] = None
        self.plan: Dict[int, Optional[int]] = {}    # frame bit -> drive
        self.due: Deque[Tuple[int, Optional[int]]] = deque()
        self.out: Optional[int] = None
        self.cycle = 0

    def drive(self, drive: Drive, cycle: int) -> None:
        if self.out == 0:
            drive.pull_low(self.io_pad)

    def _fields(self) -> Tuple[int, int, int, int]:
        v = 0
        for b in self.bits[:14]:
            v = (v << 1) | b
        return v >> 12, (v >> 10) & 3, (v >> 5) & 31, v & 31

    def _read_value(self, st: int, op: int, regad: int) -> int:
        if st == ST_C22:
            return self.regs.get(regad, 0)
        addr = self.mmd_addr.get(regad, 0)
        value = self.mmd.get((regad, addr), 0)
        if op == OP45_RINC:
            self.mmd_addr[regad] = (addr + 1) & 0xFFFF
        return value

    def _decide(self) -> None:
        """At the rise of REGAD[0]: plan what to drive for the rest."""
        st, op, phyad, regad = self._fields()
        ok = st in (ST_C22, ST_C45) and not (st == ST_C22 and op in (0, 3))
        self.ours = ok and phyad == self.phyad and not self.absent
        if not self.ours or self.absent or not phy_drives_data(op):
            return
        value = self._read_value(st, op, regad)
        self.plan = {15: 0}
        for k in range(16, 32):
            self.plan[k] = (value >> (31 - k)) & 1
        self.plan[32] = None                        # let go after the last
        if self.early_ta:
            self.plan[14] = 0

    def _finish(self) -> None:
        frame = decode_bits(self.bits)
        frame["ours"] = self.ours
        self.frames.append(frame)
        st, op, regad, data = frame["st"], frame["op"], frame["regad"], frame["data"]
        if not self.ours or phy_drives_data(op):
            return
        if st == ST_C22:
            self.regs[regad] = data
        elif op == OP45_ADDR:
            self.mmd_addr[regad] = data
        else:
            self.mmd[(regad, self.mmd_addr.get(regad, 0))] = data

    def observe(self, lines: Lines) -> None:
        cyc = lines.cycle
        while self.due and self.due[0][0] <= cyc:
            self.out = self.due.popleft()[1]
        mdc, io = lines.get(self.mdc_pad), lines.get(self.io_pad)
        if mdc and not self.prev_mdc:
            bit = self.prev_io
            if self.bits is None:
                if not bit and self.ones >= PREAMBLE:
                    self.bits, self.plan, self.ours = [], {}, False
                self.ones = self.ones + 1 if bit else 0
            if self.bits is not None:
                k = len(self.bits)
                self.bits.append(bit)
                if k == 13:
                    self._decide()
                if k + 1 in self.plan:
                    # set at the end of cycle cyc + delay - 1: the pad changes
                    # exactly delay clocks after the cycle MDC went high
                    self.due.append((cyc + max(self.delay, 1) - 1, self.plan[k + 1]))
                if len(self.bits) == 32:
                    self._finish()
                    self.bits = None
        self.prev_mdc, self.prev_io = mdc, io


class Wire(Model):
    """A board wire from one pad to a ``ui`` input: ``dst`` follows ``src``
    one clock later, as a registered model must (MDC from the manager
    thread's OUT pin to the PHY thread's IN pin)."""

    def __init__(self, src: PinSpec, dst: PinSpec) -> None:
        self.src, self.dst = pad_of(src), pad_of(dst)
        self.level = 0

    def drive(self, drive: Drive, cycle: int) -> None:
        drive.set(self.dst, self.level)

    def observe(self, lines: Lines) -> None:
        self.level = lines.get(self.src)


class OpenDrainWire(Model):
    """One open-drain board line joining two ``uio`` pads of the chip (the
    manager thread's MDIO and the PHY thread's MDIO; both need the bench
    pull-up). Each pad is pulled low one clock after the chip pulls the
    other one low. It follows the chip's own drive (``uio_oe`` and
    ``uio_out``), not the resolved lines: following the lines would keep
    each side pulling the other forever once either went low."""

    def __init__(self, a: PinSpec, b: PinSpec) -> None:
        self.a, self.b = pad_of(a), pad_of(b)
        if self.a[0] != UIO or self.b[0] != UIO:
            raise ValueError("an open-drain wire joins two uio pads")
        self.low_a = self.low_b = False

    def drive(self, drive: Drive, cycle: int) -> None:
        if self.low_a:
            drive.pull_low(self.a)
        if self.low_b:
            drive.pull_low(self.b)

    def observe(self, lines: Lines) -> None:
        pulled = lines.uio_oe & ~lines.uio_out
        self.low_b = bool(pulled >> self.a[1] & 1)  # chip pulls a: b follows
        self.low_a = bool(pulled >> self.b[1] & 1)
