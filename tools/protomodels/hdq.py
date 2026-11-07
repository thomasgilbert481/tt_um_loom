"""TI HDQ reference models: a host, a gauge and a checker of every bit.

Written from TI SLUS556D, the bq27000/bq27200 datasheet (September 2004,
revised March 2006), never from ``firmware/hdq_*.loom``:

* page 4, "Standard HDQ serial communication timing (bq27000 only)": the
  table :data:`TABLE` copies, min and max in microseconds (``None`` where
  the table gives none);
* page 5, Figure 1, "HDQ bit timing diagram": a bit cell starts with a
  fall; a host 1 is low t_HW1, a 0 low t_HW0, the cell lasts t_CYCH; the
  gauge's cells the same with t_DW1, t_DW0, t_CYCD; a break is a low of
  t_B followed by t_BR high; (d): break, 7-bit address, 1-bit R/W, 8-bit
  data, t_RSPS between the R/W bit and the data;
* pages 24-25, "Communicating with the bq27000 (HDQ interface)": open
  drain with a pull-up; "the least-significant bit ... is transmitted
  first"; the command byte is W/R in bit 7 (1 write, 0 read) and the
  address in bits 6:0; the gauge "detects a BREAK when the HDQ pin is
  driven to a logic-low state for a time T(B) or greater"; the line "can
  remain high for an indefinite period of time between each bit"; after a
  read command the gauge "starts outputting the data after T(RSPS)"; a host
  that waits longer than t_RSPS should send a break; a break before each
  string of communications is recommended.

Readings the datasheet leaves open, the same as ``QUESTIONS.md`` in the
W-050 scratch directory: t_RSPS is held from the fall of the R/W bit's cell
(item 1; :class:`HdqHost` waits until ``rsps_from_end`` would allow too, if
asked); a low between t_HW0's maximum (145) and t_B's minimum (190) is
neither a bit nor a break and is recorded as a violation (item 3).

* :class:`HdqGauge` holds a register file (``regs``, any of the 128
  addresses), takes writes, answers reads and checks every host cell
  against the table. Error injection: ``respond=False`` (silent),
  ``stop_after`` bits, every time of its own (``rsps_us``, ``cycd_us``,
  ``dw1_us``, ``dw0_us``, ``bits_us`` per bit index).
* :class:`HdqHost` runs a queue of :class:`Op` (break, write, read) with
  every time of its own settable, so a cell can be put at the edge of its
  window or outside it, and a byte can be cut by a break
  (``break_after``). It checks every gauge cell against the table.
* :func:`check_host_pulls` and :func:`check_gauge_pulls` hold a list of
  low intervals (what a bus recorded for one driver) to the table; the
  tests use them on the chip's own pad.

The line is :class:`tools.protomodels.onewire.OneWireBus`: the same
open-drain wire with a pull-up, one pad per Loom thread. Times are in
clocks of the chip (50 MHz unless ``clock`` says otherwise); the table is
in microseconds.
"""

from __future__ import annotations

import dataclasses
from typing import Dict, List, Optional, Sequence, Tuple

from .onewire import CLOCK, Agent, OneWireBus  # noqa: F401  (the line model)

#: SLUS556D page 4, microseconds: name -> (min, max); None where not given.
TABLE: Dict[str, Tuple[float, Optional[float]]] = dict(
    t_B=(190, None),        # break
    t_BR=(40, None),        # break recovery
    t_CYCH=(190, None),     # host bit window
    t_HW1=(0.5, 50),        # host sends 1
    t_HW0=(86, 145),        # host sends 0
    t_RSPS=(190, 320),      # gauge to host response
    t_CYCD=(190, 250),      # gauge bit window
    t_DW1=(32, 50),         # gauge sends 1
    t_DW0=(80, 145),        # gauge sends 0
)

WRITE = 0x80                # command byte bit 7: 1 write, 0 read


def command(addr: int, write: bool) -> int:
    """The command byte: W/R in bit 7, the 7-bit address below it."""
    if not 0 <= addr <= 0x7F:
        raise ValueError("HDQ addresses are 7 bits: %r" % addr)
    return (WRITE if write else 0) | addr


def lsb_bits(byte: int) -> List[int]:
    """A byte as it goes on the wire, LSB first."""
    return [(byte >> i) & 1 for i in range(8)]


def bits_byte(bits: Sequence[int]) -> int:
    return sum((b & 1) << i for i, b in enumerate(bits))


def within(name: str, value: float) -> bool:
    lo, hi = TABLE[name]
    return value >= lo and (hi is None or value <= hi)


def _v(name: str, index: int, value: float) -> tuple:
    """A violation record: (parameter, index, measured us, the table's window)."""
    return (name, index, round(value, 3), TABLE[name])


def _us(value_us: float, clock: int) -> int:
    return int(round(value_us * clock / 1_000_000))


# ------------------------------------------------------------- checkers
def classify_host_low(low_us: float) -> str:
    """What a host low is by the table: "1", "0", "break", or "bad" (in
    neither window: under t_HW1's minimum, between the two bit windows, or
    between t_HW0's maximum and t_B's minimum)."""
    if within("t_HW1", low_us):
        return "1"
    if within("t_HW0", low_us):
        return "0"
    if within("t_B", low_us):
        return "break"
    return "bad"


def check_host_pulls(pulls: Sequence[Tuple[int, Optional[int]]], clock: int = CLOCK
                     ) -> Tuple[List[str], List[tuple]]:
    """Hold a host's lows (``[(fall, rise)]`` in clocks, as
    ``OneWireBus.intervals`` gives them) to the table. Returns the items in
    order ("1", "0", "break", "bad") and the violations: each low against
    its window, t_CYCH from fall to fall between two bits with no break
    between, t_BR from a break's rise to the next fall. An interval still
    open (rise None) is left out."""
    us = clock / 1_000_000
    items: List[str] = []
    out: List[tuple] = []
    prev: Optional[Tuple[str, int, int]] = None
    for i, (fall, rise) in enumerate(pulls):
        if rise is None:
            break
        low = (rise - fall) / us
        kind = classify_host_low(low)
        items.append(kind)
        if kind == "bad":
            name = ("t_HW1" if low < TABLE["t_HW1"][0] else
                    "t_B" if low > TABLE["t_HW0"][1] else "t_HW0")
            out.append(_v(name, i, low))
        if prev is not None:
            pkind, pfall, prise = prev
            if pkind == "break":
                if (fall - prise) / us < TABLE["t_BR"][0]:
                    out.append(_v("t_BR", i, (fall - prise) / us))
            elif kind != "break" and (fall - pfall) / us < TABLE["t_CYCH"][0]:
                out.append(_v("t_CYCH", i, (fall - pfall) / us))
        prev = (kind, fall, rise)
    return items, out


def host_bytes(items: Sequence[str]) -> List[object]:
    """The host's items grouped: "break", and each run of eight bits as a
    byte (LSB first); a run cut short by a break is a list of its bits."""
    out: List[object] = []
    run: List[int] = []
    for kind in items:
        if kind == "break":
            if run:
                out.append(list(run))
                run = []
            out.append("break")
        elif kind in ("0", "1"):
            run.append(int(kind))
            if len(run) == 8:
                out.append(bits_byte(run))
                run = []
    if run:
        out.append(list(run))
    return out


def check_gauge_pulls(pulls: Sequence[Tuple[int, Optional[int]]], rw_fall: int,
                      clock: int = CLOCK, n: int = 8) -> Tuple[List[int], List[tuple]]:
    """Hold a gauge's reply to the table: its lows after the host's R/W fall
    ``rw_fall`` (clocks). t_RSPS from ``rw_fall`` to the first fall
    (QUESTIONS item 1), t_DW1 or t_DW0 for each low (by which window it is
    in; a low in neither is a violation of the nearer), t_CYCD from fall to
    fall, over the first ``n`` lows. Returns the bits (a low of 65 us or
    less is a 1) and the violations."""
    us = clock / 1_000_000
    lows = [(f, r) for f, r in pulls if f > rw_fall and r is not None][:n]
    bits: List[int] = []
    out: List[tuple] = []
    for i, (fall, rise) in enumerate(lows):
        low = (rise - fall) / us
        if i == 0 and not within("t_RSPS", (fall - rw_fall) / us):
            out.append(_v("t_RSPS", 0, (fall - rw_fall) / us))
        if i and not within("t_CYCD", (fall - lows[i - 1][0]) / us):
            out.append(_v("t_CYCD", i, (fall - lows[i - 1][0]) / us))
        bit = 1 if low <= (TABLE["t_DW1"][1] + TABLE["t_DW0"][0]) / 2 else 0
        name = "t_DW1" if bit else "t_DW0"
        if not within(name, low):
            out.append(_v(name, i, low))
        bits.append(bit)
    return bits, out


# ---------------------------------------------------------------- gauge
class HdqGauge(Agent):
    """A bq27000-style gauge on the line (module doc). It measures every low
    it does not make itself: a bit by ``sample_us`` (68: the line is high
    there for a 1, low for a 0), a break from t_B's minimum (and any low
    over t_HW0's maximum, which is also a violation), after which the next
    fall starts a command byte. Each host low is held to its window and
    each cell to t_CYCH; a fall during the gauge's own last cell (the
    gauge's fall + ``cycd_us``) is recorded as ``host_early``.

    A read is answered with the register's byte, the first fall
    ``rsps_us`` after the R/W bit's fall, each cell ``cycd_us``, lows
    ``dw1_us`` / ``dw0_us`` (``bits_us`` overrides single bits by index);
    ``respond=False`` stays silent, ``stop_after`` sends only that many
    bits. A low by someone else while the gauge is sending (the host's
    break) stops the reply and is measured like any other low.

    ``writes`` lists ``(addr, data)``; ``reads`` lists ``(addr, byte)``;
    ``breaks`` the cycles of the breaks' rises; ``violations`` the table
    records."""

    name = "gauge"

    def __init__(self, regs: Optional[Dict[int, int]] = None, *, clock: int = CLOCK,
                 sample_us: float = 68, rsps_us: float = 255, cycd_us: float = 220,
                 dw1_us: float = 41, dw0_us: float = 112, respond: bool = True,
                 stop_after: Optional[int] = None,
                 bits_us: Optional[Dict[int, float]] = None) -> None:
        self.regs = {a: 0 for a in range(128)}
        self.regs.update(regs or {})
        self.clock_hz, self.us = clock, clock / 1_000_000
        self.sample_us, self.rsps_us, self.cycd_us = sample_us, rsps_us, cycd_us
        self.dw1_us, self.dw0_us = dw1_us, dw0_us
        self.respond, self.stop_after = respond, stop_after
        self.bits_us = dict(bits_us or {})
        self.writes: List[Tuple[int, int]] = []
        self.reads: List[Tuple[int, int]] = []
        self.breaks: List[int] = []
        self.violations: List[tuple] = []
        self.wake = 1 << 62
        self._pull = False
        self._plan: List[Tuple[int, bool]] = []
        self._fall: Optional[int] = None
        self._bits: List[int] = []
        self._cmd: Optional[int] = None
        self._prev: Optional[Tuple[str, int, int]] = None   # last host low
        self._cell_end = -1                                 # the gauge's last cell
        self._n = 0                                         # host lows seen

    # -------------------------------------------------------- receiving
    def _rise(self, cycle: int) -> None:
        fall, self._fall = self._fall, None
        low = (cycle - fall) / self.us
        kind = classify_host_low(low)
        i = self._n
        self._n += 1
        if kind == "bad":
            name = ("t_HW1" if low < TABLE["t_HW1"][0] else
                    "t_B" if low > TABLE["t_HW0"][1] else "t_HW0")
            self.violations.append(_v(name, i, low))
            kind = ("break" if low > TABLE["t_HW0"][1] else
                    "1" if low < self.sample_us else "0")
        if self._prev is not None:
            pkind, pfall, prise = self._prev
            if pkind == "break" and (fall - prise) / self.us < TABLE["t_BR"][0]:
                self.violations.append(_v("t_BR", i, (fall - prise) / self.us))
            elif (pkind != "break" and kind != "break"
                  and (fall - pfall) / self.us < TABLE["t_CYCH"][0]):
                self.violations.append(_v("t_CYCH", i, (fall - pfall) / self.us))
        self._prev = (kind, fall, cycle)
        if kind == "break":
            self.breaks.append(cycle)
            self._bits, self._cmd = [], None
            return
        self._bits.append(int(kind))
        if len(self._bits) < 8:
            return
        byte, self._bits = bits_byte(self._bits), []
        if self._cmd is None and not byte & WRITE:
            self._answer(byte & 0x7F, fall)
        elif self._cmd is None:
            self._cmd = byte
        else:
            addr, self._cmd = self._cmd & 0x7F, None
            self.regs[addr] = byte
            self.writes.append((addr, byte))

    # ---------------------------------------------------------- sending
    def _answer(self, addr: int, rw_fall: int) -> None:
        byte = self.regs[addr] & 0xFF
        self.reads.append((addr, byte))
        if not self.respond:
            return
        bits = lsb_bits(byte)[:self.stop_after]
        plan, t = [], self.rsps_us
        for i, bit in enumerate(bits):
            low = self.bits_us.get(i, self.dw1_us if bit else self.dw0_us)
            plan += [(t, True), (t + low, False)]
            last, t = t, t + self.cycd_us
        self._plan = sorted((rw_fall + _us(t, self.clock_hz), p) for t, p in plan)
        if plan:
            self._cell_end = rw_fall + _us(last + self.cycd_us, self.clock_hz)
            self.wake = self._plan[0][0] - 1

    def clock(self, level: int, cycle: int) -> bool:
        if self._plan:
            if level == 0 and not self._pull:
                self._plan = []                 # someone else pulls: the host's break
                self._cell_end = cycle
            else:
                while self._plan and self._plan[0][0] <= cycle + 1:
                    self._pull = self._plan.pop(0)[1]
                self.wake = self._plan[0][0] - 1 if self._plan else 1 << 62
                return self._pull
        self.wake = 1 << 62
        if level == 0 and not self._pull and self._fall is None:
            self._fall = cycle
            if cycle < self._cell_end:
                self.violations.append(("host_early", self._n,
                                        round((self._cell_end - cycle) / self.us, 3), None))
        elif level == 1 and self._fall is not None:
            self._rise(cycle)
        return self._pull


# ----------------------------------------------------------------- host
@dataclasses.dataclass
class Op:
    """One host operation. ``kind`` is "break", "write" or "read";
    ``break_after`` cuts it with a break after that many bits. When
    ``done``: ``status`` "ok", or for a read "no_answer" (no fall in the
    response window), "short" (a fall missing within t_CYCD), "stuck" (a
    low past t_DW0); ``result`` the byte read; ``bits`` the gauge's bits;
    ``rw_fall`` the cycle of the R/W bit's fall."""

    kind: str
    addr: int = 0
    data: int = 0
    break_after: Optional[int] = None
    done: bool = False
    status: Optional[str] = None
    result: Optional[int] = None
    bits: List[int] = dataclasses.field(default_factory=list)
    rw_fall: Optional[int] = None
    violations: List[tuple] = dataclasses.field(default_factory=list)


class HdqHost(Agent):
    """An HDQ host on the line (module doc). Queue operations with
    :meth:`break_`, :meth:`write` and :meth:`read`; they run in order, the
    first fall of each no sooner than the later of: its own last cell's end
    (t_CYCH), the break's rise plus ``br_us``, the gauge's last fall plus
    t_CYCD's maximum. Every time is settable: ``hw1_us``, ``hw0_us``,
    ``cych_us``, ``b_us``, ``br_us``; ``bits_us`` overrides the low of single
    bits of every operation by index. A gauge bit is a 1 when its low is
    under ``sample_us`` (65, between t_DW1's maximum and t_DW0's minimum).
    Each reply is held to the table (t_RSPS from the R/W fall, t_DW1, t_DW0,
    t_CYCD) in ``op.violations``; the response window ends
    ``rsps_timeout_us`` after the R/W fall (t_RSPS's maximum + 15, reading
    A), each later fall t_CYCD's maximum + 15 after the one before."""

    name = "host"

    def __init__(self, *, clock: int = CLOCK, hw1_us: float = 25, hw0_us: float = 115,
                 cych_us: float = 205, b_us: float = 250, br_us: float = 90,
                 sample_us: float = 65, rsps_timeout_us: float = 335,
                 bits_us: Optional[Dict[int, float]] = None) -> None:
        self.clock_hz, self.us = clock, clock / 1_000_000
        self.hw1_us, self.hw0_us, self.cych_us = hw1_us, hw0_us, cych_us
        self.b_us, self.br_us, self.sample_us = b_us, br_us, sample_us
        self.rsps_timeout_us = rsps_timeout_us
        self.bits_us = dict(bits_us or {})
        self.ops: List[Op] = []
        self.wake = 1 << 62
        self._queue: List[Op] = []
        self._op: Optional[Op] = None
        self._plan: List[Tuple[int, bool]] = []
        self._ready = 0
        self._pull = False
        self._listen: Optional[int] = None       # deadline of the next gauge fall
        self._gfall: Optional[int] = None
        self._gprev: Optional[int] = None

    def _add(self, op: Op) -> Op:
        self.ops.append(op)
        self._queue.append(op)
        self.wake = 0
        return op

    def break_(self) -> Op:
        return self._add(Op("break"))

    def write(self, addr: int, data: int, **kw) -> Op:
        return self._add(Op("write", addr, data & 0xFF, **kw))

    def read(self, addr: int, **kw) -> Op:
        return self._add(Op("read", addr, **kw))

    @property
    def idle(self) -> bool:
        return self._op is None and not self._queue

    # ----------------------------------------------------------- engine
    def _start(self, op: Op, cycle: int) -> None:
        t0 = max(cycle + 1, self._ready)
        u = lambda us: _us(us, self.clock_hz)       # noqa: E731
        plan: List[Tuple[float, bool]] = []
        if op.kind == "break":
            plan, end = [(0, True), (self.b_us, False)], self.b_us + self.br_us
        else:
            bits = lsb_bits(command(op.addr, op.kind == "write"))
            if op.kind == "write":
                bits += lsb_bits(op.data)
            t = last = 0.0
            for i, bit in enumerate(bits):
                if op.break_after is not None and i == op.break_after:
                    break
                low = self.bits_us.get(i, self.hw1_us if bit else self.hw0_us)
                plan += [(t, True), (t + low, False)]
                last, t = t, t + self.cych_us
            end = t
            if op.break_after is not None:
                plan += [(t, True), (t + self.b_us, False)]
                end = t + self.b_us + self.br_us
            elif op.kind == "read":
                op.rw_fall = t0 + u(last)
        self._plan = [(t0 + u(t), p) for t, p in plan]
        self._ready = t0 + u(end)
        self._op = op
        self.wake = self._plan[0][0] - 1

    def _finish(self, status: str, ready: Optional[int] = None) -> None:
        op = self._op
        op.status, op.done = status, True
        if ready is not None:
            self._ready = max(self._ready, ready)
        self._op, self._listen, self._gfall = None, None, None
        self.wake = 0 if self._queue else 1 << 62

    def _gauge_low(self, fall: int, rise: int) -> None:
        op = self._op
        low = (rise - fall) / self.us
        bit = 1 if low < self.sample_us else 0
        name = "t_DW1" if bit else "t_DW0"
        if not within(name, low):
            op.violations.append(_v(name, len(op.bits), low))
        op.bits.append(bit)

    def clock(self, level: int, cycle: int) -> bool:
        u = lambda us: _us(us, self.clock_hz)       # noqa: E731
        if self._op is None and self._queue:
            if cycle + 1 < self._ready:
                self.wake = self._ready - 1
                return self._pull
            self._start(self._queue.pop(0), cycle)
        if self._plan:
            while self._plan and self._plan[0][0] <= cycle + 1:
                self._pull = self._plan.pop(0)[1]
            if self._plan:
                self.wake = self._plan[0][0] - 1
            elif self._op.kind == "read" and self._op.break_after is None:
                self._listen = self._op.rw_fall + u(self.rsps_timeout_us)
                self._gprev = None
                self.wake = cycle + 2                # past our own release
            else:
                self._finish("ok")
            return self._pull
        op = self._op
        if self._listen is not None:
            if self._gfall is None:
                if level == 0 and not self._pull:
                    self._gfall = cycle
                    if self._gprev is None:
                        rsps = (cycle - op.rw_fall) / self.us
                        if not within("t_RSPS", rsps):
                            op.violations.append(_v("t_RSPS", 0, rsps))
                    elif not within("t_CYCD", (cycle - self._gprev) / self.us):
                        op.violations.append(_v("t_CYCD", len(op.bits),
                                                (cycle - self._gprev) / self.us))
                    self._gprev = cycle
                    self.wake = cycle + u(TABLE["t_DW0"][1] + 15)
                elif cycle >= self._listen:
                    self._finish("short" if op.bits else "no_answer", cycle + u(self.br_us))
                else:
                    self.wake = self._listen
            elif level == 1:
                self._gauge_low(self._gfall, cycle)
                self._gfall = None
                if len(op.bits) == 8:
                    op.result = bits_byte(op.bits)
                    self._finish("ok", self._gprev + u(TABLE["t_CYCD"][1]))
                else:
                    self._listen = self._gprev + u(TABLE["t_CYCD"][1] + 15)
                    self.wake = self._listen
            elif cycle >= self.wake:
                self._finish("stuck", cycle + u(self.br_us))
            return self._pull
        return self._pull
