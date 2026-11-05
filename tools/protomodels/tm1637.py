"""TM1637 reference models: the two-wire bus, the chip, a controller, a checker.

Written from the Titan Micro Electronics TM1637 datasheet, version V2.4,
the English translation (12 pages; the copy in
github.com/avishorp/TM1637, ``docs/TM1637_V2.4_EN.pdf``), never from the
firmware. Its figures are drawings; where only their labels could be read
the citation says so. What it gives:

* p.3, "Interface interpretation": the bus is not I2C ("there is no slave
  address"); DIO may change only while CLK is low; START is DIO falling
  while CLK is high, STOP is DIO rising while CLK is high; the chip pulls DIO
  low as ACK "at the falling edge of the 8th clock" and releases it "at the
  end of the 9th clock"; key data is read "from low level to high level"
  (bit 0 first) and the clock "should be less than 250K" when reading keys.
  Bytes are LSB first in both directions (p.3, and the p.3 key figure's
  S0 S1 S2 K1 K2 order).
* p.2, pin table: CLK is "used for data input and output at rising edge";
  the key code table (below).
* p.4: B7:B6 of a command: 01 data, 10 display control, 11 address. The data
  command: B1:B0 00 write, 10 read keys (01 and 11 not allowed); B2 0 auto
  increment, 1 fixed address; B3 test mode. "When STOP command is sent
  during command or data transfer, serial communication is initialized and
  command or data transferring becomes invalid (Command or data transferred
  before remain effective.)". The two write figures (labels only): auto
  mode START 40 STOP, START C0+a d1 .. dN STOP, START 8x STOP; fixed mode
  START 44 STOP, then START C0+a d STOP per byte, START 8x STOP.
* p.5: addresses C0H..C5H; "If the address is set as C6H or a higher one,
  the data will be ignored until effective address is set"; C0H after power
  on. Display control 80H + B3 (display on) + B2:B0 (pulse width 1/16, 2/16,
  4/16, 10/16 .. 14/16).
* p.10, switching characteristics: maximum clock frequency 500 kHz at 50 %
  duty; output delays tPLZ <= 300 ns, tPZL <= 100 ns. Timing
  characteristics: clock pulse width PWCLK >= 400 ns, data set-up and hold
  >= 100 ns, waiting time tWAIT >= 1 us. The timing figure did not come
  out, so which edges tWAIT spans is **unchecked**; the checker reads it as
  the eighth CLK fall to the ninth CLK rise, the usual reading.

Everything here is a registered circuit clocked by ``clk`` (the bench
contract, ``tools/protomodels/bench.py``): a model sees the lines of cycle
``c`` and changes what it pulls from ``c + 1``.
"""

from __future__ import annotations

from collections import deque
from typing import Dict, List, Optional, Sequence, Tuple

from .bench import UIO, Drive, Lines, Model, PinSpec, pad_of

CLOCK = 50_000_000

CMD_DATA, CMD_DISPLAY, CMD_ADDRESS = 0b01, 0b10, 0b11      # B7:B6 (p.4)
WRITE_AUTO, WRITE_FIXED, READ_KEYS = 0x40, 0x44, 0x42      # data commands (p.4)
ADDRESS = 0xC0                                             # C0H + register (p.5)
DISPLAY = 0x80                                             # 80H + on + width (p.5)
DISPLAY_ON = 0x08
REGISTERS = 6                                              # C0H..C5H
WIDTHS = ("1/16", "2/16", "4/16", "10/16", "11/16", "12/16", "13/16", "14/16")

#: p.2's key table, as printed: the bits in wire order, bit 0 first ("1110_1111
#: stands for 0xF7"), for SG1..SG8 with K1 and with K2. No key: 1111_1111.
_KEY_TABLE = {
    "K1": ("11101111", "01101111", "10101111", "00101111",
           "11001111", "01001111", "10001111", "00001111"),
    "K2": ("11110111", "01110111", "10110111", "00110111",
           "11010111", "01010111", "10010111", "00010111"),
}


def _wire_to_byte(bits: str) -> int:
    return int(bits[::-1], 2)


#: ``(sg, k) -> byte``: sg 1..8, k 1..2. ``KEY_CODES[(1, 1)] == 0xF7`` (p.2).
KEY_CODES: Dict[Tuple[int, int], int] = {
    (sg + 1, k): _wire_to_byte(bits)
    for k, row in ((1, _KEY_TABLE["K1"]), (2, _KEY_TABLE["K2"]))
    for sg, bits in enumerate(row)}
NO_KEY = _wire_to_byte("11111111")

#: The datasheet's limits, in nanoseconds (frequencies in hertz), with where
#: each comes from. The waiting time's edges are the one reading not checked.
TIMING = {
    "pw_clk_min": (400, "p.10 timing characteristics, PWCLK"),
    "t_setup_min": (100, "p.10 timing characteristics, tSETUP"),
    "t_hold_min": (100, "p.10 timing characteristics, tHOLD"),
    "t_wait_min": (1000, "p.10 timing characteristics, tWAIT; edges unchecked"),
    "f_max": (500_000, "p.10 switching characteristics, Fmax at 50 % duty"),
    "f_key_max": (250_000, "p.3, the key read: 'less than 250K'"),
    "t_plz_max": (300, "p.10 switching characteristics, tPLZ"),
    "t_pzl_max": (100, "p.10 switching characteristics, tPZL"),
}


def _od_pad(pin: PinSpec):
    pad = pad_of(pin)
    if pad[0] != UIO:
        raise ValueError("TM1637 lines must be bidirectional (uio) pads, got %r" % (pin,))
    return pad


class Agent:
    """Something on the bus besides the chip: each cycle it sees both lines
    and says whether it pulls CLK and DIO low in the next cycle (a registered
    open-drain driver)."""

    name = "agent"

    def clock(self, clk: int, dio: int, cycle: int) -> Tuple[bool, bool]:
        return False, False


class TwoWireBus(Model):
    """CLK and DIO, each an open-drain line on one or more ``uio`` pads with
    the board's pull-up (p.8: 10 kOhm on each). A Loom thread's pull on one of
    a line's pads reaches the others one cycle later, as through a board wire;
    agents (:class:`Tm1637Chip`, :class:`Tm1637Controller`) pull the whole
    line. Build the bench with ``pullups=bus.pullups``.

    Recorded: :attr:`edges`, ``(cycle, clk, dio)`` at every change of either
    line, and :attr:`pulls`, the ``[start, end)`` cycles in which each driver
    (``"chip:uio<n>"`` or an agent's name) pulled each line.
    """

    def __init__(self, clk: Sequence[PinSpec] = ("BIDIR0",),
                 dio: Sequence[PinSpec] = ("BIDIR1",), *, clock: int = CLOCK) -> None:
        self.lines = {"clk": [_od_pad(p) for p in clk], "dio": [_od_pad(p) for p in dio]}
        self.masks = {k: sum(1 << b for _, b in pads) for k, pads in self.lines.items()}
        self.clock_hz = clock
        self.agents: List[Agent] = []
        self.level = {"clk": 1, "dio": 1}
        self.edges: List[Tuple[int, int, int]] = []
        self.pulls: Dict[Tuple[str, str], List[Tuple[int, int]]] = {}
        self._open: Dict[Tuple[str, str], int] = {}
        self._agent = {"clk": False, "dio": False}
        self._chip = 0
        self._alone = 0

    @property
    def pullups(self) -> int:
        return self.masks["clk"] | self.masks["dio"]

    def add(self, agent: Agent) -> Agent:
        self.agents.append(agent)
        return agent

    def ns(self, cycles: float) -> float:
        return cycles * 1e9 / self.clock_hz

    def drive(self, drive: Drive, cycle: int) -> None:
        for line, pads in self.lines.items():
            for pad in pads:
                if self._agent[line] or self._chip & self.masks[line] & ~(1 << pad[1]):
                    drive.pull_low(pad)

    def observe(self, lines: Lines) -> None:
        cycle = lines.cycle
        clk = int((lines.uio & self.masks["clk"]) == self.masks["clk"])
        dio = int((lines.uio & self.masks["dio"]) == self.masks["dio"])
        if (clk, dio) != (self.level["clk"], self.level["dio"]):
            self.edges.append((cycle, clk, dio))
            self.level = {"clk": clk, "dio": dio}
        chip = lines.uio_oe & ~lines.uio_out & self.pullups
        pull = {"clk": False, "dio": False}
        names = []
        for agent in self.agents:
            pc, pd = agent.clock(clk, dio, cycle)
            pull["clk"] |= bool(pc)
            pull["dio"] |= bool(pd)
            names += [(agent.name, "clk")] * bool(pc) + [(agent.name, "dio")] * bool(pd)
        for line, pads in self.lines.items():
            names += [("chip:uio%d" % b, line) for _, b in pads if chip >> b & 1]
        self._record(cycle, set(names))
        self._agent, self._chip = pull, chip

    def _record(self, cycle: int, active) -> None:
        for key in active:
            self._open.setdefault(key, cycle)
        for key in [k for k in self._open if k not in active]:
            self.pulls.setdefault(key, []).append((self._open.pop(key), cycle))

    def run_alone(self, until, max_cycles: int) -> bool:
        """Clock the agents with no chip: a line is low exactly when an agent
        pulls it. Runs until ``until()`` or ``max_cycles``; returns ``until()``."""
        lines = Lines()
        end = self._alone + max_cycles
        while self._alone < end and not until():
            lines.cycle = self._alone
            lines.uio = ((0 if self._agent["clk"] else self.masks["clk"])
                         | (0 if self._agent["dio"] else self.masks["dio"]))
            self.observe(lines)
            self._alone += 1
        return until()

    def intervals(self, name: str, line: str) -> List[Tuple[int, Optional[int]]]:
        out = list(self.pulls.get((name, line), []))
        if (name, line) in self._open:
            out.append((self._open[(name, line)], None))
        return out

    @staticmethod
    def chip_name(pin: PinSpec) -> str:
        port, bit = pad_of(pin)
        return "chip:%s%d" % (port, bit)


class Tm1637Chip(Agent):
    """The TM1637's side of the bus (p.3 to p.5, module doc): the three
    commands, the six display registers and the address counter in both
    modes, the display control byte and the key scan, answered with
    :attr:`key`. Answers a CLK fall ``delay`` cycles later (5 at 50 MHz is
    tPZL's 100 ns).

    Error injection: ``present=False`` never pulls DIO (no chip on the bus);
    ``nack=`` a set of byte numbers (0 = the first byte the chip receives) it
    does not acknowledge, taking them as it would otherwise.

    :attr:`log` records ``("start", c)``, ``("stop", c)``, ``("byte", value,
    acked, c)`` for each byte received, ``("key", value, c)`` for each key
    byte sent and ``("cut", bits, c)`` for a byte a START or STOP broke off.
    """

    name = "tm1637"

    def __init__(self, *, key: int = NO_KEY, delay: int = 5, present: bool = True,
                 nack: Sequence[int] = ()) -> None:
        self.key = key
        self.delay = delay
        self.present = present
        self.nack = set(nack)
        self.registers = [0] * REGISTERS
        self.control: Optional[int] = None      # no power-on value in the datasheet
        self.address = 0                        # C0H after power on (p.5)
        self.fixed = False
        self.reading = False
        self.log: List[tuple] = []
        self.received = 0
        self._prev = (1, 1)
        self._phase = "idle"                    # idle rx ack nak tx tx_ack
        self._bits = self._shift = 0
        self._first = self._writing = False
        self._tx = self._tx_bits = 0
        self._pull = False
        self._sched: deque = deque()

    @property
    def on(self) -> bool:
        return self.control is not None and bool(self.control & DISPLAY_ON)

    def _set(self, cycle: int, low: bool) -> None:
        if self.present:
            self._sched.append((cycle + self.delay, low))

    def clock(self, clk: int, dio: int, cycle: int) -> Tuple[bool, bool]:
        pclk, pdio = self._prev
        self._prev = (clk, dio)
        if clk and pclk and dio != pdio:
            if self._phase == "rx" and self._bits > 1:      # this high is the condition's
                self.log.append(("cut", self._bits - 1, cycle))
            if not dio:                                   # START
                self.log.append(("start", cycle))
                self._phase, self._first, self._writing = "rx", True, False
            else:                                         # STOP
                self.log.append(("stop", cycle))
                self._phase = "idle"
            self._bits = self._shift = 0
            self._set(cycle, False)
        elif clk and not pclk and self._phase == "rx":
            self._shift |= dio << self._bits              # LSB first
            self._bits += 1
        elif not clk and pclk:
            self._fall(cycle)
        while self._sched and self._sched[0][0] <= cycle:
            self._pull = self._sched.popleft()[1]
        return False, self._pull

    def _fall(self, cycle: int) -> None:
        phase = self._phase
        if phase == "rx" and self._bits == 8:
            acked = self.received not in self.nack
            self.log.append(("byte", self._shift, acked, cycle))
            self.received += 1
            self._take(self._shift)
            self._phase = "ack" if acked else "nak"
            self._set(cycle, acked)                      # ACK from the 8th fall
        elif phase in ("ack", "nak"):                     # the 9th fall
            if phase == "ack" and self.reading:
                self.reading = False
                self._phase, self._tx, self._tx_bits = "tx", self.key & 0xFF, 0
                self.log.append(("key", self._tx, cycle))
                self._set(cycle, not (self._tx & 1))
            else:
                self._phase, self._bits, self._shift = "rx", 0, 0
                self._set(cycle, False)                   # released at the 9th's end
        elif phase == "tx":
            self._tx_bits += 1
            if self._tx_bits < 8:
                self._set(cycle, not (self._tx >> self._tx_bits & 1))
            else:
                self._phase = "ack"                       # the key byte's ACK
                self._set(cycle, True)

    def _take(self, byte: int) -> None:
        """A received byte, at its eighth fall."""
        if self._first:
            self._first = False
            kind = byte >> 6
            if kind == CMD_DATA:
                self.fixed = bool(byte & 0x04)
                self.reading = bool(byte & 0x02)
            elif kind == CMD_ADDRESS:
                self.address, self._writing = byte & 0x3F, True
            elif kind == CMD_DISPLAY:
                self.control = byte
        elif self._writing and self.address < REGISTERS:
            self.registers[self.address] = byte
            if not self.fixed:
                self.address += 1


class Tm1637Controller(Agent):
    """A controller (an MCU) on the bus. A bit is ``low`` cycles with CLK low
    (DIO changed ``hold`` cycles after the fall) and ``high`` cycles high,
    DIO sampled in the middle of the high phase. Queue with :meth:`start`,
    :meth:`stop`, :meth:`byte`, :meth:`read`, or the datasheet's sequences
    (:meth:`write_auto`, :meth:`write_fixed`, :meth:`display`,
    :meth:`read_keys`). :attr:`results` gets ``("ack", acked)`` for every byte
    sent and ``("read", byte, acked)`` for every byte read.

    Error injection: :meth:`cut` sends the first ``bits`` bits of a byte and
    then a STOP (a stray STOP in mid-byte), and the default ``cut_start``
    turns it into a START instead.
    """

    name = "controller"

    def __init__(self, *, low: int = 150, high: int = 100, hold: int = 10) -> None:
        if not 0 < hold < low or high < 2:
            raise ValueError("need 0 < hold < low and high >= 2")
        self.low, self.high, self.hold = low, high, hold
        self.results: List[tuple] = []
        self._ops: deque = deque()
        self._gen = self._main()
        self._wait = 0
        self._clk = self._dio = False           # pulling low?
        self._lines = (1, 1)
        self._active = False

    # ---- queueing
    def start(self) -> None:
        self._ops.append(("start", None))

    def stop(self) -> None:
        self._ops.append(("stop", None))

    def byte(self, value: int) -> None:
        self._ops.append(("byte", value & 0xFF))

    def read(self) -> None:
        self._ops.append(("read", None))

    def cut(self, value: int, bits: int, then: str = "stop") -> None:
        self._ops.append(("cut", (value & 0xFF, bits, then)))

    def transaction(self, *values: int) -> None:
        self.start()
        for v in values:
            self.byte(v)
        self.stop()

    def write_auto(self, address: int, data: Sequence[int]) -> None:
        """p.4's first figure: S 40 P, S C0+a d.. P."""
        self.transaction(WRITE_AUTO)
        self.transaction(ADDRESS + address, *data)

    def write_fixed(self, address: int, value: int) -> None:
        """p.4's second figure: S 44 P, S C0+a d P."""
        self.transaction(WRITE_FIXED)
        self.transaction(ADDRESS + address, value)

    def display(self, on: bool, width: int) -> None:
        self.transaction(DISPLAY | (DISPLAY_ON if on else 0) | (width & 7))

    def read_keys(self) -> None:
        self.start()
        self.byte(READ_KEYS)
        self.read()
        self.stop()

    @property
    def busy(self) -> bool:
        return bool(self._ops) or self._active

    # ---- the clock
    def clock(self, clk: int, dio: int, cycle: int) -> Tuple[bool, bool]:
        self._lines = (clk, dio)
        if self._wait > 1:
            self._wait -= 1
        else:
            self._wait = next(self._gen)
        return self._clk, self._dio

    def _bit(self, value: Optional[int]):
        """One clock pulse from CLK low: DIO set (None: released) after the
        hold, the sample in mid-high. Yields waits; the sample is returned."""
        yield self.hold
        self._dio = value == 0
        yield self.low - self.hold
        self._clk = False
        yield self.high // 2
        sample = self._lines[1]
        yield self.high - self.high // 2
        self._clk = True
        return sample

    def _main(self):
        h = self.high
        while True:
            if not self._ops:
                self._active = False
                yield 1
                continue
            self._active = True
            op, arg = self._ops.popleft()
            if op == "start":                       # from the idle bus
                self._clk = self._dio = False
                yield h
                self._dio = True                    # DIO falls, CLK high
                yield h
                self._clk = True
            elif op == "stop":                      # from CLK low
                yield self.hold
                self._dio = True
                yield self.low - self.hold
                self._clk = False
                yield h
                self._dio = False                   # DIO rises, CLK high
                yield h
            elif op in ("byte", "cut"):
                value, bits, then = (arg, 8, None) if op == "byte" else arg
                for i in range(bits):
                    yield from self._bit(value >> i & 1)
                if op == "cut":
                    yield self.hold
                    self._dio = then == "stop"      # STOP: low first, START: high
                    yield self.low - self.hold
                    self._clk = False
                    yield h
                    self._dio = then != "stop"
                    yield h
                    if then == "start":
                        self._clk = True
                    continue
                ack = yield from self._bit(None)    # released: the chip's ACK
                self.results.append(("ack", ack == 0))
            else:                                   # read: 8 bits and the ACK
                value = 0
                for i in range(8):
                    value |= (yield from self._bit(None)) << i
                ack = yield from self._bit(None)
                self.results.append(("read", value, ack == 0))


def check(edges: Sequence[Tuple[int, int, int]], clock_hz: int = CLOCK, *,
          f_limit: Optional[float] = None) -> dict:
    """Check and decode the bus from :attr:`TwoWireBus.edges` (both lines,
    as resolved; the lines idle high before the first edge).

    Decodes ``events``: ``("start", c)``, ``("stop", c)``, ``("byte", value,
    acked, c)`` (nine clocks, LSB first, the ninth's DIO low = ACK) and
    ``("cut", bits, c)`` (a START or STOP after ``bits`` clocks of a byte).
    Measures, in nanoseconds: ``lows`` and ``highs`` (CLK pulse widths),
    ``periods`` (rise to rise inside a transaction), ``setups`` (the last DIO
    change of a low phase to the rise), ``holds`` (the fall to the first DIO
    change after it), ``waits`` (each eighth fall to the ninth rise).
    ``violations`` lists ``(kind, cycle, value, limit)`` against
    :data:`TIMING`: pulse widths under PWCLK, set-up or hold under 100 ns,
    a wait under tWAIT, a clock period under ``1 / f_limit`` (default the
    500 kHz maximum), DIO and CLK changing in one cycle, and every ``cut``
    (DIO changed while CLK was high inside a byte: p.3 forbids it, except as
    the START or STOP that it is then read as).
    """
    ns = 1e9 / clock_hz
    limit = 1e9 / (f_limit or TIMING["f_max"][0])
    pw, t_su, t_hd, t_wait = (TIMING[k][0] for k in
                              ("pw_clk_min", "t_setup_min", "t_hold_min", "t_wait_min"))
    out = {k: [] for k in ("events", "violations", "lows", "highs", "periods",
                           "setups", "holds", "waits")}
    ev, bad = out["events"], out["violations"]
    clk = dio = 1
    fall = rise = dio_at = None
    in_frame, bits, samples, hold_open = False, 0, [], False
    fall8 = None
    for cycle, c, d in edges:
        if c != clk and d != dio:
            bad.append(("simultaneous", cycle, 0, 0))
        if c == clk == 1 and d != dio:                       # START or STOP
            if in_frame and bits > 1:                        # this high is the condition's
                ev.append(("cut", bits - 1, cycle))
                bad.append(("cut", cycle, bits - 1, 0))
            ev.append(("start" if not d else "stop", cycle))
            in_frame, bits, samples = not d, 0, []
        elif c and not clk:                                  # rise
            if fall is not None:
                low = (cycle - fall) * ns
                out["lows"].append(low)
                if low < pw:
                    bad.append(("pw_low", cycle, low, pw))
                if dio_at is not None and dio_at > fall:
                    su = (cycle - dio_at) * ns
                    out["setups"].append(su)
                    if su < t_su:
                        bad.append(("setup", cycle, su, t_su))
            if in_frame and rise is not None and bits:
                period = (cycle - rise) * ns
                out["periods"].append(period)
                if period < limit:
                    bad.append(("period", cycle, period, limit))
            if in_frame and bits == 8 and fall8 is not None:
                w = (cycle - fall8) * ns
                out["waits"].append(w)
                if w < t_wait:
                    bad.append(("wait", cycle, w, t_wait))
            rise = cycle
            if in_frame:
                samples.append(d)
                bits += 1
                if bits == 9:
                    value = sum(b << i for i, b in enumerate(samples[:8]))
                    ev.append(("byte", value, samples[8] == 0, cycle))
                    bits, samples = 0, []
        elif not c and clk:                                  # fall
            if rise is not None:
                high = (cycle - rise) * ns
                out["highs"].append(high)
                if high < pw:
                    bad.append(("pw_high", cycle, high, pw))
            fall, hold_open = cycle, True
            fall8 = cycle if in_frame and bits == 8 else None
        if d != dio and not c:                               # DIO moved, CLK low
            if hold_open and fall is not None:
                hd = (cycle - fall) * ns
                out["holds"].append(hd)
                if hd < t_hd:
                    bad.append(("hold", cycle, hd, t_hd))
                hold_open = False
            dio_at = cycle
        clk, dio = c, d
    return out


def frames(events: Sequence[tuple]) -> List[List[int]]:
    """The bytes of each START..STOP transaction in :func:`check`'s events
    (a cut transaction keeps the whole bytes before the cut)."""
    out: List[List[int]] = []
    for e in events:
        if e[0] == "start":
            out.append([])
        elif e[0] == "byte" and out:
            out[-1].append(e[1])
    return out
