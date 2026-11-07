"""Microwire reference models: the 93C46 serial EEPROM and its host.

Written from the data sheet, not from the firmware. Source: Microchip
DS21749 (93AA46A/B/C, 93LC46A/B/C, 93C46A/B/C, "1K Microwire Compatible
Serial EEPROM"), revision H, which the author downloaded but **could not
read in this session**: the PDF is encrypted and no text extractor was
installed. Every figure below is the author's recollection of that data
sheet's tables and is marked **unchecked**; QUESTIONS.md lists them. Other
vendors' 93C46 parts (Atmel/Microchip AT93C46, ST M93C46) use the same
instruction set; their AC tables differ in detail.

* Instruction set table, x16 organisation (ORG = 1, or the B parts): every
  instruction is a start bit (the first 1 on DI at an SK rise while CS is
  high), a two-bit op code and six address bits, MSB first:
  READ ``1 10 A5..A0`` (DO: a dummy 0, then D15..D0), WRITE ``1 01 A5..A0
  D15..D0``, ERASE ``1 11 A5..A0``, EWEN ``1 00 11xxxx``, EWDS ``1 00
  00xxxx``, ERAL ``1 00 10xxxx``, WRAL ``1 00 01xxxx D15..D0``. In x8 (ORG
  = 0) the address is seven bits and the data eight. Checked against the
  shape of the instruction table as remembered (unchecked).
* Sequential read: with CS held high and SK running, the part goes on to
  the next address after D0 with no further dummy bit, wrapping from the
  last address to 0 (unchecked).
* Erase/write enable: the part powers up with erase/write disabled; EWEN
  enables until EWDS or power-off (unchecked).
* Ready/busy: the self-timed cycle of WRITE, ERASE, ERAL and WRAL starts
  after the instruction (the model starts it at the CS fall that ends the
  instruction; QUESTIONS.md item 2); while it runs, DO is 0 whenever CS is
  high, and 1 once it is done, TSV after CS rises (unchecked).
* AC characteristics, by Vcc row (:data:`ROWS`; nanoseconds, unchecked):
  FCLK, TCKH, TCKL, TCSS, TCSH, TCSL, TDIS, TDIH, TPD, TCZ, TSV. Write
  cycle TWC 6 ms max (2 ms on the 93C46 at 5 V, unchecked), TEC (ERAL) 6
  ms, TWL (WRAL) 15 ms: tests use microseconds, set by the host.

What is here:

* :func:`instr` and the op-code constants;
* :class:`MicrowireMonitor`, a bus analyser on CS, SK, DI and DO: it
  decodes every instruction from the pads and checks every edge against a
  row of the AC table, listing what it finds in ``violations``;
* :class:`Eeprom93C46`, a model part with a chosen output delay and write
  cycle time;
* :class:`MicrowireHost`, a model host with a chosen half period.

Times are in core clocks; ``clock_hz`` converts the nanosecond figures.
"""

from __future__ import annotations

from collections import deque
from typing import Deque, Dict, Iterator, List, Optional

from .bench import Drive, Lines, Model, PinSpec, pad_of

# Op codes (instruction set table, x16) and the 00-group extensions (A5 A4)
OP_EXT, OP_WRITE, OP_READ, OP_ERASE = 0b00, 0b01, 0b10, 0b11
EXT_EWDS, EXT_WRAL, EXT_ERAL, EXT_EWEN = 0b00, 0b01, 0b10, 0b11
WORDS, ABITS = 64, 6                            # x16: 64 words, A5..A0

# AC table rows (ns; FCLK in Hz). Unchecked: see the module docstring.
ROWS: Dict[str, Dict[str, float]] = {
    "4.5-5.5V": dict(fclk=3e6, tckh=200, tckl=100, tcss=50, tcsh=0, tcsl=250,
                     tdis=50, tdih=50, tpd=100, tcz=100, tsv=100),
    "2.5-4.5V": dict(fclk=2e6, tckh=250, tckl=200, tcss=100, tcsh=0, tcsl=250,
                     tdis=100, tdih=100, tpd=250, tcz=200, tsv=250),
    "1.8-2.5V": dict(fclk=1e6, tckh=450, tckl=450, tcss=250, tcsh=0, tcsl=250,
                     tdis=250, tdih=250, tpd=400, tcz=200, tsv=500),
}


def ns_to_clocks(ns: float, clock_hz: float) -> float:
    return ns * clock_hz / 1e9


def instr(op: int, addr: int = 0) -> int:
    """The nine instruction bits (start bit, op code, A5..A0) as a number."""
    return 1 << 8 | (op & 3) << 6 | (addr & 0x3F)


def ext(e: int) -> int:
    """A 00-group instruction: EWEN, EWDS, ERAL, WRAL."""
    return instr(OP_EXT, (e & 3) << 4)


def bits_of(value: int, n: int) -> List[int]:
    return [(value >> (n - 1 - i)) & 1 for i in range(n)]


def kind(op: int, addr: int) -> str:
    """The instruction's name."""
    if op == OP_EXT:
        return {EXT_EWEN: "EWEN", EXT_EWDS: "EWDS", EXT_ERAL: "ERAL",
                EXT_WRAL: "WRAL"}[(addr >> 4) & 3]
    return {OP_WRITE: "WRITE", OP_READ: "READ", OP_ERASE: "ERASE"}[op]


def has_data(name: str) -> bool:
    return name in ("WRITE", "WRAL")


def starts_cycle(name: str) -> bool:
    return name in ("WRITE", "ERASE", "ERAL", "WRAL")


class MicrowireMonitor(Model):
    """A bus analyser on CS, SK, DI and DO (the resolved pads).

    Every CS-high window with SK rises is decoded into ``instructions``
    (dicts: ``name``, ``op``, ``addr``, ``data``, ``dummy``, ``words``, the
    rise cycles); a window with no rise is a ready/busy poll and goes into
    ``polls`` (``start``, ``end``, ``do`` at the end, and ``ready_at``, the
    first cycle DO was 1). The host's edges are checked against ``row``
    (TCKH, TCKL, FCLK, TCSS, TCSL, TDIS, TDIH, all while CS is high) and
    the part's DO changes in a READ against TPD; findings go in
    ``violations`` as ``(cycle, text)``. ``do_delays`` lists, for every DO
    change in a read, the clocks since the latest SK rise.
    """

    def __init__(self, cs: PinSpec, sk: PinSpec, di: PinSpec, do: PinSpec, *,
                 row: str = "2.5-4.5V", clock_hz: float = 50e6) -> None:
        self.pads = [pad_of(p) for p in (cs, sk, di, do)]
        self.t = {k: (ns_to_clocks(v, clock_hz) if k != "fclk" else clock_hz / v)
                  for k, v in ROWS[row].items()}
        self.prev: Optional[List[int]] = None
        self.violations: List[tuple] = []
        self.instructions: List[Dict] = []
        self.polls: List[Dict] = []
        self.do_delays: List[int] = []
        self.rises: List[int] = []
        self.falls: List[int] = []
        self.cs_rise = self.cs_fall = None
        self.last_rise = self.last_fall = self.last_di = None
        self.win: List[tuple] = []
        self.win_do: List[tuple] = []
        self.ready_at = None

    def _bad(self, cycle: int, text: str) -> None:
        self.violations.append((cycle, text))

    def _check(self, c: int, what: str, got: float, need: float) -> None:
        if got < need:
            self._bad(c, "%s %d clocks < %.1f" % (what, got, need))

    def observe(self, lines: Lines) -> None:
        now = [lines.get(p) for p in self.pads]
        c = lines.cycle
        if self.prev is None:
            self.prev = now
            return
        cs, sk, di, do = now
        pcs, psk, pdi, pdo = self.prev
        t = self.t
        if cs and not pcs:                                  # CS rises
            if self.cs_fall is not None:
                self._check(c, "TCSL", c - self.cs_fall, t["tcsl"])
            self.cs_rise, self.win, self.win_do = c, [], []
            self.ready_at = c if do else None
        if pcs and sk and not psk:                          # SK rises, selected
            if not self.win:
                self._check(c, "TCSS", c - self.cs_rise, t["tcss"])
            else:
                self._check(c, "FCLK period", c - self.last_rise, t["fclk"])
            if self.last_fall is not None and self.last_fall > self.cs_rise:
                self._check(c, "TCKL", c - self.last_fall, t["tckl"])
            if self.last_di is not None:
                self._check(c, "TDIS", c - self.last_di, t["tdis"])
            self.win.append((c, di, pdo))
            self.last_rise = c
            self.rises.append(c)
        if pcs and psk and not sk and self.last_rise is not None \
                and self.last_rise >= self.cs_rise:         # SK falls, selected
            self._check(c, "TCKH", c - self.last_rise, t["tckh"])
            self.falls.append(c)
        if not sk and psk:
            self.last_fall = c
        if di != pdi:
            if pcs and self.win:
                self._check(c, "TDIH", c - self.last_rise, t["tdih"])
            self.last_di = c
        if pcs and do != pdo:
            self.win_do.append((c, do))
            if do and self.ready_at is None and not self.win:
                self.ready_at = c
        if pcs and not cs:                                  # CS falls
            self.cs_fall = c
            self._close(c)
        self.prev = now

    def _close(self, c: int) -> None:
        win = self.win
        if not win:
            self.polls.append(dict(start=self.cs_rise, end=c, do=self.prev[3],
                                   ready_at=self.ready_at))
            return
        dis = [b for _, b, _ in win]
        if 1 not in dis:
            return
        s = dis.index(1)
        body = win[s + 1:]
        if len(body) < 8:
            self.instructions.append(dict(name="SHORT", rises=len(win)))
            return
        op = body[0][1] << 1 | body[1][1]
        addr = 0
        for _, b, _ in body[2:8]:
            addr = addr << 1 | b
        name = kind(op, addr)
        got = dict(name=name, op=op, addr=addr, data=None, dummy=None,
                   words=[], start=win[s][0], rises=[r for r, _, _ in win])
        if has_data(name) and len(body) >= 24:
            got["data"] = sum(b << (15 - i) for i, (_, b, _) in
                              enumerate(body[8:24]))
        if name == "READ":
            a0 = body[7][0]
            tail = body[8:]
            if tail:
                got["dummy"] = tail[0][2]
            outs = [d for _, _, d in tail[1:]]
            for k in range(len(outs) // 16):
                word = 0
                for b in outs[16 * k:16 * k + 16]:
                    word = word << 1 | b
                got["words"].append(word)
            rises = [r for r, _, _ in win]
            for cyc, _ in self.win_do:
                if cyc > a0:
                    d = cyc - max(r for r in rises if r < cyc)
                    self.do_delays.append(d)
                    if d > self.t["tpd"]:
                        self._bad(cyc, "TPD %d clocks > %.1f" % (d, self.t["tpd"]))
        self.instructions.append(got)


class Eeprom93C46(Model):
    """A model 93C46, x16, on CS, SK, DI (observed) and DO (driven).

    DO changes ``delay`` clocks after the SK rise (or the CS edge) that
    causes the change, and is released (not driven: the bench's pull-up or
    a ``ui`` pad's idle 0) while CS is low. A WRITE, ERASE, ERAL or WRAL
    that is complete when CS falls starts a cycle of ``tw`` clocks if
    erase/write is enabled; the array changes when the cycle ends. During
    the cycle instructions are ignored and DO shows 0 while CS is high;
    after it, 1 while CS is high, until CS falls. ``mem`` is the array;
    ``log`` lists every instruction taken (name, address, data, refused).
    """

    def __init__(self, cs: PinSpec, sk: PinSpec, di: PinSpec, do: PinSpec, *,
                 delay: int = 10, tw: int = 2000,
                 mem: Optional[List[int]] = None, enabled: bool = False) -> None:
        self.cs, self.sk, self.di = pad_of(cs), pad_of(sk), pad_of(di)
        self.do = pad_of(do)
        self.delay, self.tw = delay, tw
        self.mem = list(mem) if mem is not None else [0xFFFF] * WORDS
        self.enabled = enabled
        self.log: List[Dict] = []
        self.prev = (0, 0)
        self.out: Optional[int] = None
        self.sched: Deque[tuple] = deque()
        self.state, self.bits = "hunt", []
        self.ptr = self.bitn = 0
        self.busy_until: Optional[int] = None
        self.pending: Optional[tuple] = None
        self.ready_shown = False

    def drive(self, drive: Drive, cycle: int) -> None:
        if self.out is not None:
            drive.set(self.do, self.out)

    def _put(self, cycle: int, value: Optional[int]) -> None:
        self.sched.append((cycle + self.delay - 1, value))   # seen from c + delay

    def _status(self, cycle: int) -> Optional[int]:
        if self.busy_until is not None:
            return 0
        return 1 if self.ready_shown else None

    def observe(self, lines: Lines) -> None:
        c = lines.cycle
        while self.sched and self.sched[0][0] <= c:
            self.out = self.sched.popleft()[1]
        cs, sk, di = lines.get(self.cs), lines.get(self.sk), lines.get(self.di)
        pcs, psk = self.prev
        self.prev = (cs, sk)
        if self.busy_until is not None and c >= self.busy_until:
            self._finish()
            if cs:
                self._put(c, 1)
        if cs and not pcs:
            st = self._status(c)
            self.state, self.bits = "hunt", []
            if st is not None:
                self.state = "status"
                self._put(c, st)
            return
        if pcs and not cs:
            self._end(c)
            self._put(c, None)
            return
        if not cs or not (sk and not psk):
            return
        if self.state == "status":
            if self.busy_until is not None or not di:
                return
            self.ready_shown = False                    # a start bit ends it
            self._put(c, None)
            self.state, self.bits = "instr", []
            return
        self._rise(c, di)

    def _rise(self, c: int, di: int) -> None:
        if self.state == "hunt":
            if di:
                self.state, self.bits = "instr", []
            return
        if self.state == "read":
            if self.bitn == 16:
                self.ptr, self.bitn = (self.ptr + 1) % WORDS, 0
            self._put(c, (self.mem[self.ptr] >> (15 - self.bitn)) & 1)
            self.bitn += 1
            return
        if self.state in ("instr", "data"):
            self.bits.append(di)
        if self.state == "instr" and len(self.bits) == 8:
            op = self.bits[0] << 1 | self.bits[1]
            addr = sum(b << (7 - i) for i, b in enumerate(self.bits) if i >= 2)
            self.name, self.addr = kind(op, addr), addr
            if self.name == "READ":
                self.state, self.ptr, self.bitn = "read", addr, 0
                self._put(c, 0)                             # the dummy bit
            elif has_data(self.name):
                self.state, self.bits = "data", []
            else:
                self.state, self.data = "done", None
        elif self.state == "data" and len(self.bits) == 16:
            self.data = sum(b << (15 - i) for i, b in enumerate(self.bits))
            self.state = "done"

    def _end(self, c: int) -> None:
        if self.state == "status":
            if self.busy_until is None:
                self.ready_shown = False
        elif self.state == "done":
            name = self.name
            entry = dict(name=name, addr=self.addr, data=self.data,
                         refused=False, cycle=c)
            if name == "EWEN":
                self.enabled = True
            elif name == "EWDS":
                self.enabled = False
            elif not self.enabled:
                entry["refused"] = True
            elif self.busy_until is None:
                self.pending = (name, self.addr, self.data)
                self.busy_until = c + self.tw
            self.log.append(entry)
        self.state = "hunt"

    def _finish(self) -> None:
        name, addr, data = self.pending
        if name == "WRITE":
            self.mem[addr] = data
        elif name == "ERASE":
            self.mem[addr] = 0xFFFF
        elif name in ("ERAL", "WRAL"):
            self.mem = [0xFFFF if name == "ERAL" else data] * WORDS
        self.pending, self.busy_until, self.ready_shown = None, None, True


class MicrowireHost(Model):
    """A model host: drives CS, SK and DI (push-pull), reads DO.

    SK is ``half`` clocks low then ``half`` clocks high per bit; DI changes
    at the fall (set-up and hold of a whole half period) and DO is sampled
    in the last low cycle before each rise. CS rises ``half`` clocks before
    the first rise and stays low ``gap`` clocks between instructions. After
    WRITE, ERASE, ERAL and WRAL the host drops CS for ``gap`` clocks, raises
    it and polls DO every cycle for up to ``poll`` clocks. Each finished
    instruction is appended to ``results``: ``name``, ``addr``, ``words``
    (READ), ``dummy`` (READ), ``ready`` (cycle DO went high, or None) and
    ``busy`` (clocks from the CS rise to ready).
    """

    def __init__(self, cs: PinSpec, sk: PinSpec, di: PinSpec, do: PinSpec, *,
                 half: int = 28, gap: int = 40, poll: int = 20000) -> None:
        self.pcs, self.psk, self.pdi = pad_of(cs), pad_of(sk), pad_of(di)
        self.pdo = pad_of(do)
        self.half, self.gap, self.poll = half, gap, poll
        self.cs = self.sk = self.di = 0
        self.queue: Deque[tuple] = deque()
        self.results: List[Dict] = []
        self.lines: Optional[Lines] = None
        self.gen: Optional[Iterator[None]] = None

    def read(self, addr: int, n: int = 1) -> None:
        self.queue.append((instr(OP_READ, addr), None, n))

    def write(self, addr: int, data: int) -> None:
        self.queue.append((instr(OP_WRITE, addr), data, 0))

    def erase(self, addr: int) -> None:
        self.queue.append((instr(OP_ERASE, addr), None, 0))

    def command(self, e: int, data: Optional[int] = None) -> None:
        """EWEN, EWDS, ERAL or WRAL (``data`` for WRAL)."""
        self.queue.append((ext(e), data, 0))

    @property
    def busy(self) -> bool:
        return self.gen is not None or bool(self.queue)

    def drive(self, drive: Drive, cycle: int) -> None:
        drive.set(self.pcs, self.cs)
        drive.set(self.psk, self.sk)
        drive.set(self.pdi, self.di)

    def observe(self, lines: Lines) -> None:
        self.lines = lines
        if self.gen is None:
            if not self.queue:
                return
            self.gen = self._run(*self.queue.popleft())
        try:
            next(self.gen)
        except StopIteration:
            self.gen = None

    def _wait(self, n: int) -> Iterator[None]:
        for _ in range(n):
            yield

    def _bit(self, b: int, sampled: List[int]) -> Iterator[None]:
        self.di = b
        yield from self._wait(self.half - 1)
        sampled.append(self.lines.get(self.pdo))
        yield
        self.sk = 1
        yield from self._wait(self.half)
        self.sk = 0

    def _run(self, word: int, data: Optional[int], n: int) -> Iterator[None]:
        op, addr = word >> 6 & 3, word & 0x3F
        name = kind(op, addr)
        res = dict(name=name, addr=addr, words=[], dummy=None, ready=None,
                   busy=None)
        bits = bits_of(word, 9)
        if has_data(name):
            bits += bits_of(data, 16)
        if name == "READ":
            bits += [0] * (1 + 16 * n)
        self.cs = 1
        sampled: List[int] = []
        for b in bits:
            yield from self._bit(b, sampled)
        yield from self._wait(self.half)
        self.cs, self.di = 0, 0
        if name == "READ":
            outs = sampled[9:]
            res["dummy"] = outs[0]
            for k in range(n):
                w = 0
                for b in outs[1 + 16 * k:17 + 16 * k]:
                    w = w << 1 | b
                res["words"].append(w)
        if starts_cycle(name):
            yield from self._wait(self.gap)
            self.cs = 1
            start = self.lines.cycle
            for _ in range(self.poll):
                yield
                if self.lines.get(self.pdo):
                    res["ready"] = self.lines.cycle
                    res["busy"] = self.lines.cycle - start
                    break
            self.cs = 0
        self.results.append(res)
        yield from self._wait(self.gap)
