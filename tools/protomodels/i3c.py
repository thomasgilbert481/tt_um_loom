"""MIPI I3C Basic SDR: the bus wire, a target model, a controller model, a checker.

Written from the specification as far as it could be read, never from
``firmware/i3c_controller.loom`` or ``firmware/i3c_target.loom``.

Sources. MIPI Alliance, *Specification for I3C Basic*, version 1.1.1
(9 June 2021). The PDF is sent only after a request form
(mipi.org/mipi-i3c-basic-download), so it could not be fetched for this
model. Read instead, as a secondary source that quotes it: the Linux
kernel's ``include/linux/i3c/master.h`` (constants "taken from the I3C Spec
V1.1.1, chapter 6.2 Timing specification") and ``include/linux/i3c/ccc.h``.
Everything below marked *unchecked* is from memory of the specification and
waits for the director to check it against the PDF.

What each figure rests on:

* Broadcast address 0x7E (section 5.1.2.2; read: ``I3C_BROADCAST_ADDR``).
* CCC codes, section 5.1.9: RSTDAA 0x06 broadcast, ENTDAA 0x07 broadcast,
  bit 7 set for a direct CCC (read: ``ccc.h``).
* SDR timing, section 6.2, table "I3C Timing Requirements" (ns):
  tLOW_OD >= 200 and tHIGH_INIT_OD >= 200 (read: ``I3C_BUS_TLOW_OD_MIN_NS``,
  ``I3C_BUS_THIGH_INIT_OD_MIN_NS``); tHIGH <= 41 on a mixed bus only (read:
  ``I3C_BUS_THIGH_MIXED_MAX_NS``; not checked here, this is a pure I3C bus).
  *Unchecked*: push-pull tLOW >= 24 and tHIGH >= 24; tCAS >= 38.4 (START to
  the first SCL fall) and <= 1000 in activity state ENTAS0; tCBP >= tCAS/2
  (SCL rise to SDA rise in a STOP); Sr set-up (tCBSr) and hold (tCASr)
  >= tCAS/2; tBUF >= 500 for a pure bus; tSCO <= 12 (target clock to data).
* Frames, section 5.1 (*unchecked*): after a START or Sr the address byte
  (7 bits, RnW) and its ACK are open drain; a 0x7E/W header is followed by a
  CCC byte with a T-bit, push-pull; private write data bytes are push-pull,
  each followed by a T-bit of odd parity (the nine bits hold an odd number
  of ones); private read data bytes are driven by the target, push-pull,
  each followed by a T-bit that is 1 if the target has more and 0 on its
  last byte (end of data); after a T = 1 the target lets go at the SCL rise
  and the controller may end the read by pulling SDA low while SCL is high
  (an Sr), then P.
* ENTDAA, section 5.1.4.2 (*unchecked*): S 7E/W, CCC 0x07 T, then rounds of
  Sr 7E/R; every target without a dynamic address ACKs and sends its 48-bit
  provisioned ID, BCR and DCR, MSB first, open drain; a target that sends a
  1 but sees a 0 has lost and stops, so the lowest 64-bit value wins. The
  controller sends the dynamic address, seven bits and a parity bit (odd
  parity over the eight), and the winner ACKs. A round that nobody ACKs
  ends the procedure with P. Whether the address byte may be push-pull is
  QUESTIONS.md item 2; this checker accepts open drain there and flags a
  push-pull high.
* Provisioned ID, section 5.1.4.1.1 (*unchecked*): PID[47:33] manufacturer,
  PID[32] ID type, PID[31:0] part/instance/vendor fields.
* RSTDAA, section 5.1.9 (*unchecked*): every target forgets its dynamic
  address.

Pins: SCL and SDA are each one board wire joining one or more ``uio`` pads
(the controller's and the target's on one chip) and the model agents, with a
pull-up. Unlike I2C the wire carries push-pull drive both ways, so the bus
resolves it itself: low if anyone drives low, else high if anyone drives
high, else the pull-up; a line driven both ways in one cycle is a
contention. It records every change of either line and of the set of
drivers holding SDA high, which is what the open-drain check reads.
"""

from __future__ import annotations

import bisect
from dataclasses import dataclass, field
from typing import Dict, Iterator, List, Optional, Sequence, Tuple

from tools.protomodels.bench import Drive, Lines, Model, PinSpec, pad_of, UIO

CLOCK = 50_000_000

BROADCAST = 0x7E
CCC_RSTDAA = 0x06
CCC_ENTDAA = 0x07

T_LOW_OD_MIN = 200.0            # read (Linux master.h, spec 6.2)
T_HIGH_INIT_OD_MIN = 200.0      # read (Linux master.h, spec 6.2)
T_LOW_PP_MIN = 24.0             # unchecked
T_HIGH_PP_MIN = 24.0            # unchecked
T_CAS_MIN = 38.4                # unchecked
T_CAS_MAX = 1000.0              # unchecked (activity state ENTAS0)
T_CBP_MIN = T_CAS_MIN / 2       # unchecked
T_SR_SU_MIN = T_CAS_MIN / 2     # unchecked (tCBSr)
T_SR_HD_MIN = T_CAS_MIN / 2     # unchecked (tCASr)
T_BUF_MIN = 500.0               # unchecked (pure I3C bus)


def ones(value: int) -> int:
    return bin(value).count("1")


def t_bit(byte: int) -> int:
    """The write T-bit: odd parity, so ``byte`` and T hold an odd number of ones."""
    return 1 - (ones(byte & 0xFF) & 1)


def daa_par(da: int) -> int:
    """The parity bit after a dynamic address in ENTDAA (odd parity, unchecked)."""
    return 1 - (ones(da & 0x7F) & 1)


def daa_value(pid: int, bcr: int, dcr: int) -> int:
    """The 64 bits a target sends in ENTDAA, in arbitration order."""
    return ((pid & ((1 << 48) - 1)) << 16) | ((bcr & 0xFF) << 8) | (dcr & 0xFF)


class Agent:
    """A model on the bus besides the chip. Each cycle it sees both lines and
    returns what it drives on SCL and SDA in the next cycle: ``None``
    (released), 0 or 1 (a registered driver; open drain is 0 or ``None``)."""

    name = "agent"

    def clock(self, scl: int, sda: int, cycle: int) -> Tuple[Optional[int], Optional[int]]:
        return None, None


class I3cBus(Model):
    """SCL and SDA as two board wires, each joining ``uio`` pads and agents.

    A chip pad's drive reaches the line's other pads one cycle later (the
    board wire, as in ``tm1637.TwoWireBus``); agents are registered drivers.
    Build the bench with ``pullups=bus.pullups``.

    Recorded: :attr:`trace`, ``(cycle, scl, sda, highs)`` at every change of
    either line or of ``highs``, the names of the drivers holding SDA high
    (``"chip:uio<n>"`` or an agent's name); :attr:`contentions`,
    ``(cycle, line, lows, highs)`` for every cycle a line is driven both ways.
    """

    def __init__(self, scl: Sequence[PinSpec] = ("BIDIR0", "BIDIR2"),
                 sda: Sequence[PinSpec] = ("BIDIR1", "BIDIR3"), *,
                 clock: int = CLOCK) -> None:
        self.pads = {"scl": [self._pad(p) for p in scl], "sda": [self._pad(p) for p in sda]}
        self.masks = {k: sum(1 << b for _, b in v) for k, v in self.pads.items()}
        self.clock_hz = clock
        self.agents: List[Agent] = []
        self.level = {"scl": 1, "sda": 1}
        self.trace: List[Tuple[int, int, int, frozenset]] = []
        self.contentions: List[Tuple[int, str, Tuple[str, ...], Tuple[str, ...]]] = []
        self._ag: Dict[str, Dict[str, Optional[int]]] = {}
        self._chip_lo = self._chip_hi = 0
        self._last = (1, 1, frozenset())
        self._cycle = 0

    @staticmethod
    def _pad(pin: PinSpec):
        pad = pad_of(pin)
        if pad[0] != UIO:
            raise ValueError("an I3C line needs a uio pad, not %r" % (pad,))
        return pad

    @property
    def pullups(self) -> int:
        return self.masks["scl"] | self.masks["sda"]

    def add(self, agent: Agent) -> Agent:
        self.agents.append(agent)
        self._ag[agent.name] = {"scl": None, "sda": None}
        return agent

    def ns(self, cycles: float) -> float:
        return cycles * 1e9 / self.clock_hz

    def drive(self, drive: Drive, cycle: int) -> None:
        for line, pads in self.pads.items():
            alow = any(d[line] == 0 for d in self._ag.values())
            ahigh = any(d[line] == 1 for d in self._ag.values())
            for pad in pads:
                others = self.masks[line] & ~(1 << pad[1])
                if alow or self._chip_lo & others:
                    drive.set(pad, 0)
                elif ahigh or self._chip_hi & others:
                    drive.set(pad, 1)

    def _resolve(self, line: str, chip_lo: int, chip_hi: int):
        lows = [n for n, d in self._ag.items() if d[line] == 0]
        highs = [n for n, d in self._ag.items() if d[line] == 1]
        for _, b in self.pads[line]:
            if chip_lo >> b & 1:
                lows.append("chip:uio%d" % b)
            if chip_hi >> b & 1:
                highs.append("chip:uio%d" % b)
        if lows and highs:
            self.contentions.append((self._cycle, line, tuple(lows), tuple(highs)))
        return (0 if lows else 1), highs

    def step_wire(self, chip_lo: int, chip_hi: int, cycle: int) -> Tuple[int, int]:
        """Resolve both lines for one cycle, record, clock the agents."""
        self._cycle = cycle
        scl, _ = self._resolve("scl", chip_lo, chip_hi)
        sda, highs = self._resolve("sda", chip_lo, chip_hi)
        state = (scl, sda, frozenset(highs))
        if state != self._last:
            self.trace.append((cycle,) + state)
            self._last = state
        self.level = {"scl": scl, "sda": sda}
        for agent in self.agents:
            s, d = agent.clock(scl, sda, cycle)
            self._ag[agent.name] = {"scl": s, "sda": d}
        self._chip_lo, self._chip_hi = chip_lo, chip_hi
        return scl, sda

    def observe(self, lines: Lines) -> None:
        oe = lines.uio_oe & self.pullups
        self.step_wire(oe & ~lines.uio_out, oe & lines.uio_out, lines.cycle)

    def run_alone(self, until, max_cycles: int, start: int = 0) -> bool:
        """Clock the agents with no chip (a model controller and model
        targets). Returns ``until()``."""
        cycle = max(start, self._cycle + 1)
        end = cycle + max_cycles
        while cycle < end and not until():
            self.step_wire(0, 0, cycle)
            cycle += 1
        return until()


class I3cTarget(Agent):
    """An I3C Basic SDR target with a provisioned ID, from the frame rules of
    the module docstring: RSTDAA, ENTDAA (open-drain arbitration, odd-parity
    check of the address), private write into a buffer (a bad T-bit refuses
    the rest of the transfer), private read of that buffer (NACKed while it
    is empty). It changes SDA ``delay`` cycles after the SCL fall it reacts
    to (its tSCO) and samples SDA at the SCL rise.

    Recorded: :attr:`da` (``None`` without an address), :attr:`buffer`,
    :attr:`refused` (writes cut by a bad T-bit), :attr:`events`.
    """

    def __init__(self, pid: int, bcr: int = 0x00, dcr: int = 0x00, *,
                 name: str = "target", delay: int = 2) -> None:
        self.name = name
        self.pid, self.bcr, self.dcr = pid, bcr, dcr
        self.delay = delay
        self.da: Optional[int] = None
        self.armed = False
        self.buffer: List[int] = []
        self.refused = 0
        self.events: List[Tuple[int, str, int]] = []
        self.mode = "idle"
        self.bits = 0
        self.shift = 0
        self.after_ack = ""
        self.tx: List[int] = []
        self.sda: Optional[int] = None
        self._pend: List[Tuple[int, Optional[int]]] = []
        self._prev = (1, 1)

    # -- output, delayed by ``delay`` cycles
    def _set(self, cycle: int, value: Optional[int]) -> None:
        self._pend.append((cycle + self.delay, value))

    def _release_now(self) -> None:
        self._pend.clear()
        self.sda = None

    def clock(self, scl: int, sda: int, cycle: int):
        pscl, psda = self._prev
        self._prev = (scl, sda)
        if pscl and scl and psda != sda:
            self._release_now()
            if sda == 0:
                self.mode, self.bits, self.shift = "hdr", 0, 0
            else:
                self.mode, self.armed = "idle", False
        elif scl and not pscl:
            self._rise(sda, cycle)
        elif pscl and not scl:
            self._fall(cycle)
        while self._pend and self._pend[0][0] <= cycle:
            self.sda = self._pend.pop(0)[1]
        return None, self.sda

    def _rise(self, sda: int, cycle: int) -> None:
        self.shift = (self.shift << 1) | sda
        self.bits += 1
        if self.mode == "arb" and self.tx and self.tx[0] == 1 and sda == 0:
            self.mode = "skip"                  # lost the arbitration
            self._set(cycle, None)
        elif self.mode == "rd_t" and self.tx_t == 1:
            self._set(cycle, None)              # T = 1: let go at the rise

    def _ack(self, cycle: int, then: str) -> None:
        self.mode, self.after_ack = "ack", then
        self._set(cycle, 0)

    def _fall(self, cycle: int) -> None:
        mode = self.mode
        if mode == "hdr" and self.bits == 8:
            addr, rnw = self.shift >> 1 & 0x7F, self.shift & 1
            if addr == BROADCAST and not rnw:
                self._ack(cycle, "ccc")
            elif addr == BROADCAST and rnw and self.armed and self.da is None:
                self._ack(cycle, "arb")
            elif self.da is not None and addr == self.da and not rnw:
                self._ack(cycle, "wr")
            elif self.da is not None and addr == self.da and rnw and self.buffer:
                self._ack(cycle, "rd")
            else:
                self.mode = "skip"
        elif mode == "ack":
            self._after_ack(cycle)
        elif mode == "ccc" and self.bits == 9:
            self._ccc(self.shift)
            self.mode = "skip"
        elif mode == "arb":
            self.tx.pop(0)
            if self.tx:
                self._set(cycle, 0 if self.tx[0] == 0 else None)
            else:
                self._set(cycle, None)
                self.mode, self.bits, self.shift = "daa_addr", 0, 0
        elif mode == "daa_addr" and self.bits == 8:
            if ones(self.shift & 0xFF) & 1:
                self.pending_da = self.shift >> 1 & 0x7F
                self._ack(cycle, "daa_done")
            else:
                self.mode = "skip"              # bad parity: NACK, stay armed
        elif mode == "wr" and self.bits == 9:
            byte, t = self.shift >> 1 & 0xFF, self.shift & 1
            if ones(self.shift & 0x1FF) & 1:
                self.buffer.append(byte)
                self.bits = self.shift = 0
            else:
                self.refused += 1
                self.events.append((cycle, "refused", byte))
                self.mode = "skip"
        elif mode == "rd":
            self._rd_next_bit(cycle)
        elif mode == "rd_t":
            if self.tx_t == 0:
                self._set(cycle, None)
                self.mode = "skip"
            else:
                self._rd_byte(cycle)

    def _after_ack(self, cycle: int) -> None:
        then = self.after_ack
        self.bits = self.shift = 0
        if then == "arb":
            value = daa_value(self.pid, self.bcr, self.dcr)
            self.tx = [value >> (63 - k) & 1 for k in range(64)]
            self.mode = "arb"
            self._set(cycle, 0 if self.tx[0] == 0 else None)
        elif then == "daa_done":
            self._set(cycle, None)
            self.da, self.armed = self.pending_da, False
            self.events.append((cycle, "da", self.da))
            self.mode = "skip"
        elif then == "wr":
            self._set(cycle, None)
            self.buffer = []
            self.mode = "wr"
        elif then == "rd":
            self.queue = list(self.buffer)
            self._rd_byte(cycle)
        else:                                   # "ccc"
            self._set(cycle, None)
            self.mode = "ccc"

    def _ccc(self, shift: int) -> None:
        if not ones(shift & 0x1FF) & 1:
            return                              # bad T-bit: ignored
        code = shift >> 1 & 0xFF
        if code == CCC_RSTDAA:
            self.da = None
            self.events.append((-1, "rstdaa", 0))
        elif code == CCC_ENTDAA and self.da is None:
            self.armed = True

    def _rd_byte(self, cycle: int) -> None:
        byte = self.queue.pop(0)
        self.tx = [byte >> (7 - k) & 1 for k in range(8)]
        self.tx_t = 1 if self.queue else 0
        self.bits = self.shift = 0
        self.mode = "rd"
        self._set(cycle, self.tx[0])

    def _rd_next_bit(self, cycle: int) -> None:
        if self.bits < 8:
            self._set(cycle, self.tx[self.bits])
        else:
            self._set(cycle, self.tx_t)
            self.mode = "rd_t"


class I3cController(Agent):
    """An I3C Basic SDR controller, push-pull SCL, cycle-scripted.

    One bit: SCL falls; SDA is set ``la`` cycles later; SCL rises ``lb``
    cycles after that; SDA is sampled ``ha`` cycles after the rise and SCL
    falls ``hb`` cycles later. START and Sr hold, Sr and STOP set-up are
    ``hb`` and ``ha``; ``buf`` cycles of free bus follow a STOP.
    Queue operations with :meth:`rstdaa`, :meth:`entdaa`, :meth:`write`,
    :meth:`read`; :attr:`results` collects one dict per operation.
    """

    def __init__(self, *, la: int = 40, lb: int = 40, ha: int = 16, hb: int = 32,
                 buf: int = 64, name: str = "controller") -> None:
        self.name = name
        self.la, self.lb, self.ha, self.hb, self.buf = la, lb, ha, hb, buf
        self.ops: List[Tuple] = []
        self.results: List[dict] = []
        self.scl, self.sda = 1, None
        self.seen_sda = 1
        self._gen: Optional[Iterator] = None

    def rstdaa(self):
        self.ops.append(("rstdaa",))

    def entdaa(self, first_da: int):
        self.ops.append(("entdaa", first_da))

    def write(self, da: int, data: Sequence[int], bad_t: bool = False):
        self.ops.append(("write", da, list(data), bad_t))

    def read(self, da: int, most: int):
        self.ops.append(("read", da, most))

    @property
    def idle(self) -> bool:
        return self._gen is None and not self.ops

    def clock(self, scl: int, sda: int, cycle: int):
        self.seen_sda = sda
        if self._gen is None and self.ops:
            op = self.ops.pop(0)
            self._gen = getattr(self, "_op_" + op[0])(*op[1:])
        if self._gen is not None:
            try:
                next(self._gen)
            except StopIteration:
                self._gen = None
        return self.scl, self.sda

    # ------------------------------------------------------------ pieces
    def _wait(self, n: int):
        for _ in range(n):
            yield

    def _bit(self, value: Optional[int], pp: bool, box: list):
        """One bit. ``value`` None releases SDA; ``pp`` drives a 1 high."""
        yield from self._wait(self.la)
        self.sda = None if value is None or (value and not pp) else value
        yield from self._wait(self.lb)
        self.scl = 1
        yield from self._wait(self.ha)
        box.append(self.seen_sda)
        yield from self._wait(self.hb)
        self.scl = 0

    def _start(self):
        self.sda = 0
        yield from self._wait(self.hb)
        self.scl = 0

    def _sr(self):
        yield from self._wait(self.la)
        self.sda = None
        yield from self._wait(self.lb)
        self.scl = 1
        yield from self._wait(self.ha)
        self.sda = 0
        yield from self._wait(self.hb)
        self.scl = 0

    def _stop(self):
        yield from self._wait(self.la)
        self.sda = 0
        yield from self._wait(self.lb)
        self.scl = 1
        yield from self._wait(self.ha)
        self.sda = None
        yield from self._wait(self.buf)

    def _header(self, addr: int, rnw: int, box: list):
        """Address byte and ACK slot, open drain. Returns via ``box``: the ACK."""
        bits: list = []
        byte = addr << 1 | rnw
        for k in range(8):
            yield from self._bit(byte >> (7 - k) & 1, False, bits)
        yield from self._bit(None, False, bits)
        box.append(bits[-1] == 0)

    def _send(self, byte: int, t: int):
        sink: list = []
        for k in range(8):
            yield from self._bit(byte >> (7 - k) & 1, True, sink)
        yield from self._bit(t, True, sink)

    # ------------------------------------------------------------ operations
    def _op_rstdaa(self):
        ack: list = []
        yield from self._start()
        yield from self._header(BROADCAST, 0, ack)
        if ack[0]:
            yield from self._send(CCC_RSTDAA, t_bit(CCC_RSTDAA))
        yield from self._stop()
        self.results.append({"op": "rstdaa", "ack": ack[0]})

    def _op_entdaa(self, first_da: int):
        ack: list = []
        res = {"op": "entdaa", "ack": False, "rounds": []}
        yield from self._start()
        yield from self._header(BROADCAST, 0, ack)
        res["ack"] = ack[0]
        da = first_da
        if ack[0]:
            yield from self._send(CCC_ENTDAA, t_bit(CCC_ENTDAA))
            while True:
                ack = []
                yield from self._sr()
                yield from self._header(BROADCAST, 1, ack)
                if not ack[0]:
                    break
                bits: list = []
                for _ in range(64):
                    yield from self._bit(None, False, bits)
                value = 0
                for b in bits:
                    value = value << 1 | b
                byte = da << 1 | daa_par(da)
                for k in range(8):
                    yield from self._bit(byte >> (7 - k) & 1, False, [])
                got: list = []
                yield from self._bit(None, False, got)
                round_ = {"pid": value >> 16, "bcr": value >> 8 & 0xFF,
                          "dcr": value & 0xFF, "da": da, "ack": got[0] == 0}
                res["rounds"].append(round_)
                if round_["ack"]:
                    da += 1
        yield from self._stop()
        self.results.append(res)

    def _op_write(self, da: int, data: List[int], bad_t: bool):
        ack: list = []
        yield from self._start()
        yield from self._header(da, 0, ack)
        if ack[0]:
            for k, byte in enumerate(data):
                yield from self._send(byte, t_bit(byte) ^ (1 if bad_t and k == 0 else 0))
        yield from self._stop()
        self.results.append({"op": "write", "ack": ack[0], "data": data})

    def _op_read(self, da: int, most: int):
        ack: list = []
        data: List[Tuple[int, int]] = []
        yield from self._start()
        yield from self._header(da, 1, ack)
        while ack[0] and len(data) < most:
            bits: list = []
            for _ in range(8):
                yield from self._bit(None, False, bits)
            byte = 0
            for b in bits:
                byte = byte << 1 | b
            t: list = []
            if len(data) + 1 < most:
                yield from self._bit(None, False, t)
            else:                               # last allowed: end it ourselves
                yield from self._wait(self.la)
                self.sda = None
                yield from self._wait(self.lb)
                self.scl = 1
                yield from self._wait(self.ha)
                t.append(self.seen_sda)
                yield from self._wait(self.hb // 2)
                self.sda = 0                    # Sr after a T = 1
                yield from self._wait(self.hb - self.hb // 2)
                self.scl = 0
            data.append((byte, t[0]))
            if t[0] == 0:
                break
        yield from self._stop()
        self.results.append({"op": "read", "ack": ack[0], "data": data})


# ---------------------------------------------------------------- checker
@dataclass
class Bit:
    low: int                    # the cycle the window opened (SCL fall or START)
    rise: int
    fall: Optional[int]
    value: int
    phase: str = "?"            # "od", "pp" (controller), "rd" (target data), "rd_t"


@dataclass
class Segment:
    """The bits between a START or Sr and the next START, Sr or P."""
    start: int                  # the cycle SDA fell
    repeated: bool
    bits: List[Bit] = field(default_factory=list)
    end: Optional[int] = None   # the cycle of the Sr or P that ended it
    stop: bool = False


@dataclass
class Report:
    segments: List[Segment]
    violations: List[str]
    writes: List[dict]          # {"da", "data": [(byte, t)], "t_errors": [index]}
    reads: List[dict]           # {"da", "data": [(byte, t)]}
    daa: List[dict]             # {"pid", "bcr", "dcr", "da", "par_ok", "ack"}
    cccs: List[int]


def segments(trace: Sequence[Tuple[int, int, int, frozenset]]) -> List[Segment]:
    """Cut the line trace into segments and bits."""
    out: List[Segment] = []
    cur: Optional[Segment] = None
    pscl, psda = 1, 1
    low = None
    open_bit: Optional[Bit] = None
    for cycle, scl, sda, _ in trace:
        if pscl and scl and sda != psda:
            if cur is not None:
                cur.end, cur.stop = cycle, bool(sda)
            if sda == 0:
                cur = Segment(cycle, repeated=cur is not None and not cur.stop)
                out.append(cur)
                low = cycle
            else:
                cur = None
            open_bit = None
        elif scl != pscl and cur is not None:
            if scl:
                open_bit = Bit(low if low is not None else cycle, cycle, None, sda)
                cur.bits.append(open_bit)
            else:
                if open_bit is not None:
                    open_bit.fall = cycle
                open_bit = None
                low = cycle
        pscl, psda = scl, sda
    return out


def _spans(trace) -> List[Tuple[int, int, frozenset]]:
    """``[start, end)`` cycle spans in which someone held SDA high."""
    out = []
    since, cur = None, frozenset()
    for cycle, _, _, highs in trace:
        if highs != cur:
            if cur:
                out.append((since, cycle, cur))
            since, cur = cycle, highs
    if cur:
        out.append((since, 1 << 62, cur))
    return out


def _byte(bits: Sequence[Bit]) -> int:
    v = 0
    for b in bits:
        v = v << 1 | b.value
    return v


def check(bus: I3cBus, *, read_grace_ns: float = 400.0) -> Report:
    """Decode the bus trace and check it against the frame and timing rules.

    Every bit gets a phase from the frame rules (module docstring) and is
    checked against the SDR table: open drain (the address byte and its ACK,
    the ENTDAA arbitration, address and ACK) must have nobody driving SDA
    high anywhere in its window, from the SCL fall that opens it to the one
    that closes it; a read T-bit lets the target drive high until
    ``read_grace_ns`` after the rise (its hand-off). Write and CCC T-bits
    are checked for odd parity and listed (a test may force a bad one).
    """
    ns = bus.ns
    segs = segments(bus.trace)
    spans = _spans(bus.trace)
    starts = [s[0] for s in spans]
    v: List[str] = []
    writes, reads, daa, cccs = [], [], [], []
    in_daa = False
    last_stop = None
    for seg in segs:
        bits = seg.bits
        # A bit whose SCL rise is followed by an Sr or a P instead of a fall
        # is the set-up of that Sr or P (or, ending a read, a T-bit).
        carrier = bits[-1] if bits and bits[-1].fall is None else None
        if carrier is not None:
            bits = bits[:-1]
            if seg.end is not None:
                su = ns(seg.end - carrier.rise)
                if su < (T_CBP_MIN if seg.stop else T_SR_SU_MIN):
                    v.append("cycle %d: %s set-up %.0f ns" % (seg.end, "P" if seg.stop else "Sr", su))
        if not seg.repeated and last_stop is not None and ns(seg.start - last_stop) < T_BUF_MIN:
            v.append("cycle %d: tBUF %.0f ns" % (seg.start, ns(seg.start - last_stop)))
        if bits:
            cas = ns(bits[0].low - seg.start)
            lo = T_SR_HD_MIN if seg.repeated else T_CAS_MIN
            if cas < lo or (not seg.repeated and cas > T_CAS_MAX):
                v.append("cycle %d: START hold (tCAS) %.0f ns" % (seg.start, cas))
        if len(bits) < 9:
            if bits:
                v.append("cycle %d: segment of %d bits" % (seg.start, len(bits)))
            in_daa = in_daa and not seg.stop
            last_stop = seg.end if seg.stop else last_stop
            continue
        for b in bits[:9]:
            b.phase = "od"
        addr, rnw, ack = _byte(bits[:7]), bits[7].value, bits[8].value == 0
        rest = bits[9:]
        if not ack:
            if rest:
                v.append("cycle %d: bits after a NACK" % seg.start)
        elif addr == BROADCAST and not rnw:
            for b in rest:
                b.phase = "pp"
            if len(rest) >= 9:
                code = _byte(rest[:8])
                cccs.append(code)
                if ones(_byte(rest[:9])) % 2 == 0:
                    v.append("cycle %d: CCC T-bit parity" % rest[8].rise)
                in_daa = code == CCC_ENTDAA
        elif addr == BROADCAST and rnw:
            if not in_daa:
                v.append("cycle %d: 7E/R ACKed outside ENTDAA" % seg.start)
            for b in rest[:73]:
                b.phase = "od"
            if len(rest) >= 73:
                value = _byte(rest[:64])
                da_byte = _byte(rest[64:72])
                daa.append({"pid": value >> 16, "bcr": value >> 8 & 0xFF,
                            "dcr": value & 0xFF, "da": da_byte >> 1,
                            "par_ok": ones(da_byte) % 2 == 1,
                            "ack": rest[72].value == 0})
        elif not rnw:
            data, terr = [], []
            for b in rest:
                b.phase = "pp"
            for k in range(0, len(rest) - 8, 9):
                group = rest[k:k + 9]
                data.append((_byte(group[:8]), group[8].value))
                if ones(_byte(group)) % 2 == 0:
                    terr.append(k // 9)
            writes.append({"da": addr, "data": data, "t_errors": terr})
        else:
            data = []
            if carrier is not None and len(rest) % 9 == 8 and not seg.stop:
                rest = rest + [carrier]         # the T-bit the controller ended with an Sr
                bits = bits + [carrier]
            for k in range(0, len(rest) - 8, 9):
                group = rest[k:k + 9]
                for b in group[:8]:
                    b.phase = "rd"
                group[8].phase = "rd_t"
                data.append((_byte(group[:8]), group[8].value))
            reads.append({"da": addr, "data": data})
        v += _check_bits(seg, bits, ns, spans, starts, read_grace_ns)
        if seg.stop:
            in_daa = False
            last_stop = seg.end
    return Report(segs, v, writes, reads, daa, cccs)


def _highs_in(spans, starts, lo: int, hi: int) -> List[Tuple[int, frozenset]]:
    """The SDA-high spans that overlap ``[lo, hi)``: ``(first cycle, drivers)``."""
    out = []
    k = max(0, bisect.bisect_right(starts, lo) - 1)
    while k < len(spans) and spans[k][0] < hi:
        s, e, who = spans[k]
        if e > lo:
            out.append((max(s, lo), who))
        k += 1
    return out


def _check_bits(seg: Segment, bits: Sequence[Bit], ns, spans, starts,
                read_grace_ns: float) -> List[str]:
    v: List[str] = []
    for k, b in enumerate(bits):
        end = b.fall if b.fall is not None else seg.end
        t_low = ns(b.rise - b.low)
        t_high = ns(end - b.rise) if end is not None else None
        if b.phase == "od":
            init = not seg.repeated and k < 9
            if t_low < T_LOW_OD_MIN:
                v.append("cycle %d: open-drain tLOW %.0f ns" % (b.rise, t_low))
            if t_high is not None and t_high < (T_HIGH_INIT_OD_MIN if init else T_HIGH_PP_MIN):
                v.append("cycle %d: open-drain tHIGH %.0f ns" % (b.rise, t_high))
            if end is not None:
                for first, who in _highs_in(spans, starts, b.low, end):
                    v.append("cycle %d: push-pull high in an open-drain bit by %s"
                             % (first, ",".join(sorted(who))))
        elif b.phase in ("pp", "rd", "rd_t"):
            if t_low < T_LOW_PP_MIN or (t_high is not None and t_high < T_HIGH_PP_MIN):
                v.append("cycle %d: push-pull tLOW/tHIGH %.0f/%s ns" % (b.rise, t_low, t_high))
            if b.phase == "rd_t" and end is not None:
                late = b.rise + int(read_grace_ns * bus_cycles_per_ns(ns))
                for first, who in _highs_in(spans, starts, late, end):
                    v.append("cycle %d: read T-bit still driven high by %s after the hand-off"
                             % (first, ",".join(sorted(who))))
    return v


def bus_cycles_per_ns(ns) -> float:
    """Cycles per ns for a bus whose ``ns(cycles)`` is given."""
    return 1.0 / ns(1)
