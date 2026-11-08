"""Microchip UNI/O: reference master and 11AA02E48-like device on one wire.

Written from Microchip DS20002122B, "11AA02E48/11AA02E64 2K UNI/O Serial
EEPROM with EUI-48 or EUI-64 Node Identity" (2013), never from the Loom
firmware. Sections and table rows cited where used:

* Table 1-2 (AC characteristics), :data:`TABLE`: bit period TE 10..100 us
  (param 2); input edge jitter TIJIT +-0.06 UI (3); FDRIFT 0.5 % a byte (4);
  FDEV 5 % a command (5); output edge jitter TOJIT +-0.25 UI (6); TSTBY
  standby >= 600 us (9); TSS header setup >= 10 us (10); THDR header low
  >= 5 us (11); TWC write cycle <= 5 ms (13). UI = one TE.
* 8.1: every bit is a transition in the middle of its period (Manchester).
  The polarity is shown only in Figures 1-1, 1-2 and 3-2 (the header
  "01010101" after THDR's low must start high, so a 0 is high then low);
  the UNI/O bus specification (DS22076, cited by Wikipedia's UNI/O page)
  says it in words: a 0 is high-to-low, a 1 low-to-high. Unchecked against
  DS22076 itself (it could not be fetched).
* 3.1 standby pulse; 3.2 start header: THDR low, then 0x55, then MAK and a
  NoSAK; 3.3 acknowledge: MAK = 1, NoMAK = 0 from the master; SAK = 1 from
  the slave, NoSAK = no middle edge; 3.4 device address 0xA0; 3.6/3.7
  standby and idle; 3.8 the slave re-synchronises on each MAK's middle
  edge.
* Table 4-1 instructions; 4.1 READ (sequential, rolls over at 0xFF); 4.3
  WRITE (16-byte page buffer, address rolls in the page, written at the
  NoMAK if any data came); 4.4 WREN/WRDI (WEL reset after a write); 4.5 RDSR
  (WIP bit 0, WEL bit 1, BP0 bit 2, BP1 bit 3; repeated after each MAK);
  5.0 and Table 6-1 (no write without WEL); 3.3 (READ and WRITE during a
  write cycle give NoSAK).
* Bytes MSB first (Figures 4-1 to 4-6).

The wire is :class:`tools.protomodels.onewire.OneWireBus` (an open-drain
line with the board's pull-up). The data sheet's parts drive SCIO push-pull
with a current limit (3.5); on the bench every driver only pulls low, which
is the same line as long as no two drivers disagree (QUESTIONS.md item 3).

Times are in clocks of the chip (50 MHz unless ``clock`` says otherwise).
"""

from __future__ import annotations

from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from .onewire import Agent, OneWireBus  # noqa: F401  (the wire, re-exported)

CLOCK = 50_000_000

#: DS20002122B Table 1-2, microseconds unless noted: (min, max).
TABLE = dict(
    TE=(10, 100),           # bit period, param 2
    TIJIT=0.06,             # input edge jitter tolerance, UI, param 3
    FDRIFT=0.005,           # bus frequency drift a byte, param 4
    FDEV=0.05,              # drift limit a command, param 5
    TOJIT=0.25,             # output edge jitter, UI, param 6
    TSTBY=(600, None),      # standby pulse, param 9
    TSS=(10, None),         # start header setup, param 10
    THDR=(5, None),         # start header low, param 11
    TWC=(None, 5000),       # write cycle, param 13
)

DEVICE_ADDRESS = 0xA0       # 3.4
READ, CRRD, WRITE, WREN, WRDI, RDSR, WRSR, ERAL, SETAL = (
    0x03, 0x06, 0x6C, 0x96, 0x91, 0x05, 0x6E, 0x6D, 0x67)   # Table 4-1
WIP, WEL = 0x01, 0x02      # 4.5
PAGE = 16                   # 4.3
SIZE = 256


def msb_bits(byte: int) -> List[int]:
    return [(byte >> (7 - i)) & 1 for i in range(8)]


def bits_byte(bits: Sequence[int]) -> int:
    v = 0
    for b in bits:
        v = v << 1 | (b & 1)
    return v


class _Proc(Agent):
    """An agent run by a generator. The generator yields requests:

    * ``("at", t)``: resume deciding the pull for cycle ``t``; ``self.level``
      is then the line in cycle ``t - 1``;
    * ``("edge", t)``: resume at the next change of the line (the value sent
      back is the cycle of the new level) or, if ``t`` is not None, at ``t``
      as above with None sent back.
    """

    name = "unio"

    def __init__(self, clock: int) -> None:
        self.clock_hz = clock
        self.us = clock / 1_000_000
        self.pull = False
        self.level = 1
        self.cycle = -1
        self.wake = 1 << 62
        self._gen = None
        self._mode: Optional[str] = None
        self._t: Optional[int] = None
        self.last_edge = 0
        self.violations: List[Tuple[str, int, float]] = []

    def _start(self, gen) -> None:
        self._gen = gen
        self._mode, self._t = "at", 0
        self.wake = 0

    def _send(self, value) -> None:
        try:
            req = self._gen.send(value)
        except StopIteration:
            self._gen, self._mode, self._t = None, None, None
            return
        self._mode, t = req
        self._t = None if t is None else int(round(t))

    def clock(self, level: int, cycle: int) -> bool:
        edge = level != self.level
        self.level, self.cycle = level, cycle
        if edge:
            self.last_edge = cycle
        while self._gen is not None:
            if self._mode == "edge" and edge:
                edge = False
                self._send(cycle)
            elif self._t is not None and cycle + 1 >= self._t:
                self._send(None)
            else:
                break
        self.wake = (self._t - 1) if self._t is not None else 1 << 62
        return self.pull

    # ------------------------------------------------------- bit primitives
    def _viol(self, name: str, value: float) -> None:
        self.violations.append((name, self.cycle, value))

    def _rbit(self, mid: float, te: float):
        """Read the bit whose ideal middle edge is at ``mid``: the line a
        quarter bit before and after it (8.1). Returns ``(bit, edge)``, or
        None when the halves are equal (no middle edge)."""
        q = te / 4
        yield ("at", mid - q + 1)
        first = self.level
        e = yield ("edge", mid + q + 1)
        if e is not None:
            yield ("at", mid + q + 1)
        second = self.level
        if first == second:
            return None
        return second, e

    def _wbit(self, start: float, te: float, bit: int):
        """Send ``bit`` in the period from ``start``: a 1 low then high, a 0
        high then low (8.1, Figure 1-2); open drain, so high is released."""
        yield ("at", start)
        self.pull = bool(bit)
        yield ("at", start + te / 2)
        self.pull = not bit


class UnioDevice(_Proc):
    """An 11AA02E48 on the wire (DS20002122B, module doc): 256 bytes, a
    16-byte page, WEL and WIP, the write cycle ``busy_us`` long.

    Starts in the power-on state (6.0: a low-to-high transition, then a
    standby pulse). Measures TE on every start header (3.2, 3.8) and
    resynchronises on each MAK's middle edge. Records, in clocks:
    :attr:`violations` (``(name, cycle, value)``) for a master outside
    Table 1-2 (``THDR``, ``TSS``, ``TE``, ``TIJIT``) or one that ends a
    command badly; :attr:`commands` (each command taken, ``(instruction,
    address or None, bytes)``); :attr:`writes` (``(address, bytes)`` per
    write cycle started); :attr:`te` (every TE measured, clocks).

    ``respond = False`` makes a device that never answers (no SAK); ``skew``
    moves every edge it drives by that many UI (TOJIT allows +-0.25).
    """

    name = "unio_device"

    def __init__(self, memory: Optional[Iterable[int]] = None, *, clock: int = CLOCK,
                 busy_us: float = 1000.0, respond: bool = True, skew: float = 0.0,
                 por: bool = True) -> None:
        super().__init__(clock)
        mem = list(memory) if memory is not None else [(i * 7 + 3) & 0xFF for i in range(SIZE)]
        self.memory = bytearray(mem + [0] * (SIZE - len(mem)))
        self.busy = busy_us * self.us
        self.respond, self.skew, self.por = respond, skew, por
        self.wel = False
        self.wip_until: Optional[int] = None
        self.pointer = 0
        self.commands: List[Tuple[int, Optional[int], List[int]]] = []
        self.writes: List[Tuple[int, List[int]]] = []
        self.te: List[float] = []
        self.headers: List[int] = []
        self.standbys = 0
        self.last_end: Optional[int] = None
        self._te = 0.0
        self._start(self._run())

    def status(self) -> int:
        wip = self.wip_until is not None and self.cycle < self.wip_until
        return (WIP if wip else 0) | (WEL if self.wel else 0)

    # ---------------------------------------------------------------- run
    def _run(self):
        if self.por:                        # 6.0: a low-to-high first
            while True:
                yield ("edge", None)
                if self.level == 1:
                    break
        good = False
        while True:
            if not good:                    # Idle (3.7): wait for a standby
                yield from self._standby()
            good = yield from self._command()

    def _standby(self):
        while True:
            if self.level == 0:
                yield ("edge", None)
                continue
            e = yield ("edge", self.last_edge + TABLE["TSTBY"][0] * self.us)
            if e is None:
                self.standbys += 1
                self.last_end = None
                return

    def _end(self, good: bool) -> bool:
        """The command is over (``good``: NoMAK then SAK, 3.1). Called at
        the SAK's middle, so the bit ends half a TE later."""
        self.pull = False
        if good:
            self.last_end = self.cycle + int(self._te / 2)
        return good

    def _rbyte(self, m: float, te: float):
        """A byte from the master and its acknowledge bit; ``m`` is the
        middle edge of the last acknowledge (MAK) bit, so the SAK bit is
        next and the byte's bits have their middles at m + 2 TE .. m + 9 TE.
        Returns ``(byte, mak, new m)`` or None (a missed edge: Idle, 3.7)."""
        bits = []
        for k in range(8):
            r = yield from self._rbit(m + (2 + k) * te, te)
            if r is None:
                return None
            bits.append(r[0])
        r = yield from self._rbit(m + 10 * te, te)
        if r is None:
            return None
        return bits_byte(bits), r[0], r[1]

    def _sak(self, m: float, te: float):
        """SAK, a 1, in the bit after the acknowledge bit whose middle was m."""
        yield from self._wbit(m + (0.5 + self.skew) * te, te, 1)

    def _sbyte(self, m: float, te: float, value: int):
        """Send a byte after a SAK (data bits from m + 1.5 TE), release the
        line for the master's acknowledge and read it. Returns ``(mak, new
        m)`` or None."""
        for k, bit in enumerate(msb_bits(value)):
            yield from self._wbit(m + (1.5 + k + self.skew) * te, te, bit)
        yield ("at", m + (9.5 + self.skew) * te)
        self.pull = False
        r = yield from self._rbit(m + 10 * te, te)
        return None if r is None else (r[0], r[1])

    def _command(self):
        us = self.us
        while self.level == 0:              # our own SAK's low, still seen
            yield ("edge", None)
        while self.level == 1:
            yield ("edge", None)
        f0 = self.last_edge
        self.headers.append(f0)
        if self.last_end is not None and f0 - self.last_end < TABLE["TSS"][0] * us:
            self._viol("TSS", (f0 - self.last_end) / us)
        r0 = yield ("edge", f0 + 20_000 * us)
        if r0 is None:
            return self._end(False)
        if r0 - f0 < TABLE["THDR"][0] * us:
            self._viol("THDR", (r0 - f0) / us)
        edges: List[int] = []
        for _ in range(8):                  # 0101 0101: an edge every TE
            e = yield ("edge", (edges[-1] if edges else r0) + 1.5 * TABLE["TE"][1] * us)
            if e is None:
                return self._end(False)
            edges.append(e)
        te = (edges[7] - edges[0]) / 7
        self.te.append(te)
        lo, hi = TABLE["TE"]
        if not lo * us <= te <= hi * us:
            self._viol("TE", te / us)
        for k, e in enumerate(edges):
            if abs(e - edges[0] - k * te) > TABLE["TIJIT"] * te:
                self._viol("TIJIT", (e - edges[0] - k * te) / te)
        self._te = te
        r = yield from self._rbit(edges[7] + te, te)
        if r is None or r[0] != 1:          # 3.2 note: MAK after the header
            return self._end(False)
        m = r[1]                            # its NoSAK slot: nothing sent (3.3)
        r = yield from self._rbyte(m, te)
        if r is None or r[0] != DEVICE_ADDRESS or not self.respond:
            return self._end(False)
        _, mak, m = r
        yield from self._sak(m, te)
        if not mak:
            return self._end(True)          # 3.3: NoMAK after the address is fine
        r = yield from self._rbyte(m, te)
        if r is None:
            return self._end(False)
        ins, mak, m = r
        busy = self.status() & WIP
        if ins in (WREN, WRDI):
            if mak:                         # 4.4: must end with NoMAK
                return self._end(False)
            self.wel = ins == WREN
            yield from self._sak(m, te)
            self.commands.append((ins, None, []))
            return self._end(True)
        if ins == RDSR:
            return (yield from self._rdsr(m, te, mak))
        if ins not in (READ, WRITE) or busy or not mak:
            return self._end(False)         # 3.3: NoSAK, then Idle
        yield from self._sak(m, te)
        addr = 0
        for _ in range(2):                  # word address MSB, LSB
            r = yield from self._rbyte(m, te)
            if r is None or not r[1]:
                return self._end(False)
            addr = (addr << 8 | r[0]) & 0xFF
            m = r[2]
            yield from self._sak(m, te)
        if ins == READ:
            return (yield from self._read(m, te, addr))
        return (yield from self._write(m, te, addr))

    def _rdsr(self, m: float, te: float, mak: int):
        if not mak:
            return self._end(False)
        yield from self._sak(m, te)
        values = []
        while True:
            value = self.status()
            values.append(value)
            r = yield from self._sbyte(m, te, value)
            if r is None:
                return self._end(False)
            mak, m = r
            yield from self._sak(m, te)
            if not mak:
                self.commands.append((RDSR, None, values))
                return self._end(True)

    def _read(self, m: float, te: float, addr: int):
        self.pointer = addr
        out = []
        while True:
            value = self.memory[self.pointer]
            out.append(value)
            self.pointer = (self.pointer + 1) % SIZE      # 4.1: rolls over
            r = yield from self._sbyte(m, te, value)
            if r is None:
                return self._end(False)
            mak, m = r
            yield from self._sak(m, te)
            if not mak:
                self.commands.append((READ, addr, out))
                return self._end(True)

    def _write(self, m: float, te: float, addr: int):
        page: Dict[int, int] = {}
        data = []
        p = addr
        while True:
            r = yield from self._rbyte(m, te)
            if r is None:                   # 4.3 note: no NoMAK, no write
                return self._end(False)
            value, mak, m = r
            page[p] = value
            data.append(value)
            p = (p & ~(PAGE - 1)) | ((p + 1) & (PAGE - 1))   # rolls in the page
            yield from self._sak(m, te)
            if not mak:
                break
        self.commands.append((WRITE, addr, data))
        if self.wel:                        # 5.0, Table 6-1
            for a, v in page.items():
                self.memory[a] = v
            self.writes.append((addr, data))
            self.wip_until = self.cycle + int(self.busy)
            self.wel = False                # 4.4
        return self._end(True)


class Op:
    """One master operation and its outcome."""

    def __init__(self, kind: str, out: Sequence[int], n_in: int, poll: bool = False) -> None:
        self.kind, self.out, self.n_in, self.poll = kind, list(out), n_in, poll
        self.status = "pending"         # then "ok", "nosak" or "badbit"
        self.data: List[int] = []
        self.failed_at: Optional[int] = None    # byte number, the address 1
        self.header_at: Optional[int] = None    # the header's fall, cycle

    def __repr__(self) -> str:
        return "Op(%s %s %s %s)" % (self.kind, self.status, self.data, self.failed_at)


class UnioMaster(_Proc):
    """A UNI/O master (DS20002122B, module doc) for testing a device: the
    standby pulse (before the first command and after an error, 3.1), TSS,
    the header, MAK/NoMAK and the SAK check after every byte (3.3).

    Every edge it drives is on an exact grid of ``te_us`` from the end of
    THDR (``jitter`` moves every other edge by that many UI, to test a
    device against TIJIT). It holds every SAK and data bit the device sends
    to TOJIT (+-0.25 UI around the ideal edge): :attr:`violations`.
    """

    name = "unio_master"

    def __init__(self, *, clock: int = CLOCK, te_us: float = 10.0, thdr_us: float = 5.0,
                 tss_us: float = 10.0, tstby_us: float = 600.0, jitter: float = 0.0,
                 max_polls: int = 200, por_pulse: bool = True) -> None:
        super().__init__(clock)
        self.te_us, self.thdr_us, self.tss_us, self.tstby_us = te_us, thdr_us, tss_us, tstby_us
        self.jitter, self.max_polls = jitter, max_polls
        self.te = te_us * self.us
        self.queue: List[Op] = []
        self.ops: List[Op] = []
        self.need_standby = True
        self.end = 0
        self._waiting = False
        self.por_pulse = por_pulse
        self._start(self._run())

    @property
    def idle(self) -> bool:
        return not self.queue and self._waiting

    def _add(self, op: Op) -> Op:
        self.queue.append(op)
        self.ops.append(op)
        if self._waiting:
            self._t, self.wake = 0, 0
        return op

    def command(self, out: Sequence[int], n_in: int = 0, poll: bool = False,
                address: int = DEVICE_ADDRESS) -> Op:
        """The device ``address``, then ``out``, then ``n_in`` bytes read;
        the last byte sent ends with NoMAK when nothing is read."""
        return self._add(Op("raw", [address] + list(out), n_in, poll))

    def read(self, addr: int, n: int) -> Op:
        return self._add(Op("read", [DEVICE_ADDRESS, READ, 0, addr], n))

    def write(self, addr: int, data: Sequence[int]) -> Op:
        return self._add(Op("write", [DEVICE_ADDRESS, WRITE, 0, addr] + list(data), 0))

    def wren(self) -> Op:
        return self._add(Op("wren", [DEVICE_ADDRESS, WREN], 0))

    def rdsr(self, poll: bool = False) -> Op:
        """Read STATUS; with ``poll``, again after a MAK while WIP is 1."""
        return self._add(Op("rdsr", [DEVICE_ADDRESS, RDSR], 1, poll))

    # ---------------------------------------------------------------- run
    def _run(self):
        if self.por_pulse:                      # 3.2 note: a low-to-high first
            yield ("at", 1)
            self.pull = True
            yield ("at", 1 + self.thdr_us * self.us)
            self.pull = False
        while True:
            if not self.queue:
                self._waiting = True
                yield ("at", None)
                self._waiting = False
                continue
            op = self.queue.pop(0)
            yield from self._do(op)

    def _edge_t(self, t: float, k: int) -> float:
        return t + (self.jitter * self.te if k % 2 else 0.0)

    def _do(self, op: Op):
        us, te = self.us, self.te_us * self.us
        self.te = te
        now = self.cycle + 1
        if self.need_standby:
            start = max(now, self.last_edge) + self.tstby_us * us
            self.need_standby = False
        else:
            start = max(now, self.end + self.tss_us * us)
        yield ("at", start)
        op.header_at = int(round(start))
        self.pull = True                        # THDR
        t0 = start + self.thdr_us * us
        bit = 0
        for b in msb_bits(0x55) + [1]:          # the header and its MAK
            yield from self._wbit(self._edge_t(t0 + bit * te, bit), te, b)
            bit += 1
        yield ("at", t0 + bit * te)
        self.pull = False                       # the NoSAK slot
        bit += 1
        n_out = len(op.out)
        for j, byte in enumerate(op.out):
            more = j + 1 < n_out or op.n_in > 0
            for b in msb_bits(byte) + [1 if more else 0]:
                yield from self._wbit(self._edge_t(t0 + bit * te, bit), te, b)
                bit += 1
            ok = yield from self._sak(t0 + bit * te, te)
            bit += 1
            if not ok:
                return self._fail(op, "nosak", j + 1)
        k = 0
        while k < op.n_in:
            bits = []
            for _ in range(8):
                r = yield from self._rbit(t0 + (bit + 0.5) * te, te)
                if r is None:
                    return self._fail(op, "badbit", n_out + k + 1)
                self._tojit(r[1], t0 + (bit + 0.5) * te)
                bits.append(r[0])
                bit += 1
            value = bits_byte(bits)
            op.data.append(value)
            k += 1
            if op.poll and value & WIP and k < self.max_polls:
                op.n_in += 1                    # 4.5: MAK, the status again
            mak = 1 if k < op.n_in else 0
            yield from self._wbit(self._edge_t(t0 + bit * te, bit), te, mak)
            bit += 1
            ok = yield from self._sak(t0 + bit * te, te)
            bit += 1
            if not ok:
                return self._fail(op, "nosak", n_out + k)
        self.end = int(round(t0 + bit * te))
        op.status = "ok"

    def _sak(self, start: float, te: float):
        yield ("at", start)
        self.pull = False                       # the slave's bit
        r = yield from self._rbit(start + te / 2, te)
        if r is None or r[0] != 1:
            return False
        self._tojit(r[1], start + te / 2)
        return True

    def _tojit(self, edge: Optional[int], ideal: float) -> None:
        if edge is not None and abs(edge - ideal) > TABLE["TOJIT"] * self.te:
            self._viol("TOJIT", (edge - ideal) / self.te)

    def _fail(self, op: Op, status: str, k: int) -> None:
        op.status, op.failed_at = status, k
        self.need_standby = True                # 3.3: standby, then again
        self.pull = False
