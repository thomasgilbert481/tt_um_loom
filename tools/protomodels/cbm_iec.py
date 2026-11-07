"""The Commodore serial bus ("IEC" bus of the VIC-20, C64, C128 and their
disk drives and printers): three open-drain lines, a controller, a device
and a timing checker, at the clock level, for ``tools.protomodels.bench``.

Written from the secondary sources below, never from the Loom firmware
(``firmware/cbm_iec_host.loom``, ``firmware/cbm_iec_device.loom``). No
Commodore standard is published, so **every timing here is secondary**:

* [B] J. Butterfield, "How the VIC/64 Serial Bus Works", COMPUTE! issue 38,
  July 1983 (the atarimagazines.com reprint): a line is *true* when pulled
  to 0 V; the byte handshake, steps 0 to 5 (talker holds CLK, listener
  DATA; ready to send = CLK released; ready for data = DATA released; the
  talker pulls CLK within 200 us, usually 60, or it is EOI; the listener
  acknowledges EOI by pulling DATA at least 60 us; the talker then pulls CLK
  within 60 us; bits LSB first, the listener reading DATA when CLK is
  released; bit hold at least 20 us, 60 with a C64 listening; frame
  handshake within 1 ms); the ATN sequence (all devices pull DATA within
  1 ms); the TALK turnaround.
* [S4] M. Steil, "Commodore Peripheral Bus: Part 4: Standard Serial",
  pagetable.com, 2019-05-06: steps 0 to 30 of a byte (CLK pulled at least
  60 us before a bit, released at least 60 us with it, for devices; 20 us
  for controllers); "between bytes time" 100 us; EOI delay at least 200 us
  (256 in the diagram); receivers busy within 1000 us of the last bit; ATN
  response 1000 us; the turnaround A to D.
* [S2] M. Steil, "Commodore Peripheral Bus: Part 2: Bus Arbitration,
  TALK/LISTEN", pagetable.com, 2019-04-02: LISTEN 0x20+pa, UNLISTEN 0x3F,
  TALK 0x40+pa, UNTALK 0x5F, SECOND 0x60+sa, CLOSE 0xE0+sa, OPEN 0xF0+sa.
* From memory, unchecked: a 1 bit is DATA released (5 V) and a 0 bit DATA
  pulled, as the C64 KERNAL sends it. Neither [B] nor [S4] states it in
  the text read.

Times are in microseconds; a bench converts with ``us``, the clocks per
microsecond (50 at 50 MHz). The tests also run the whole bus ``us = 10``,
five times faster: every window scales with it.

The checker :func:`check` decodes the wire alone (the recorded edges of
:class:`IecBus`), so it judges a Loom program and a model by the same rules.
"""

from __future__ import annotations

from collections import deque
from typing import Dict, List, Optional, Sequence, Tuple

from tools.protomodels.bench import UIO, Drive, Lines, Model, PinSpec, pad_of

#: (microseconds, source). min / max as the source gives them.
TIMING: Dict[str, Tuple[float, str]] = {
    "atn_response_max": (1000, "[B] ATN sequence; [S4] 'ATN Response Timing'"),
    "frame_ack_max": (1000, "[B] step 4; [S4] steps 29-30"),
    "non_eoi_max": (200, "[B] step 2: CLK within 200 us of ready for data"),
    "eoi_min": (200, "[B] step 2, [S4] 'End of Stream': at least 200 us"),
    "eoi_ack_min": (60, "[B] and [S4]: the listener holds DATA at least 60 us"),
    "eoi_resume_max": (60, "[B]: the talker pulls CLK within 60 us after it"),
    "bit_min": (20, "[B] step 3: hold at least 20 us; [S4]: controllers 20"),
    "bit_device_min": (60, "[S4] steps 4 and 6: devices 60 us; [B] 60 with a C64"),
    "between_bytes_min": (100, "[S4] steps 29-30, 'between bytes time'"),
    "turnaround_max": (1000, "not stated: the 1 ms of [B]'s other limits, assumed"),
}

LISTEN, UNLISTEN, TALK, UNTALK, SECOND = 0x20, 0x3F, 0x40, 0x5F, 0x60
CLOSE, OPEN = 0xE0, 0xF0


def _od_pad(pin: PinSpec):
    pad = pad_of(pin)
    if pad[0] != UIO:
        raise ValueError("IEC lines must be bidirectional (uio) pads, got %r" % (pin,))
    return pad


class Agent:
    """A model on the bus: each cycle it sees the three lines (1 = released,
    high) and says which it pulls low from the next cycle on."""

    name = "agent"

    def clock(self, atn: int, clk: int, data: int, cycle: int) -> Tuple[bool, bool, bool]:
        return False, False, False


LINES = ("atn", "clk", "data")


class IecBus(Model):
    """ATN, CLK and DATA, each an open-drain line on one or more ``uio`` pads
    with the board's pull-up. A Loom thread's pull on one of a line's pads
    reaches the line's other pads one cycle later, as through a board wire;
    agents pull the whole line. Build the bench with ``pullups=bus.pullups``.

    Recorded: :attr:`edges`, ``(cycle, atn, clk, data)`` at every change of
    any line (and the idle state at cycle -1), and :attr:`pulls`,
    ``{(driver, line): [(start, end), ...]}`` for each driver
    (``"chip:uio<n>"`` or an agent's name).
    """

    def __init__(self, atn: Sequence[PinSpec] = ("BIDIR0",),
                 clk: Sequence[PinSpec] = ("BIDIR1",),
                 data: Sequence[PinSpec] = ("BIDIR2",), *, us: int = 50) -> None:
        self.lines = {"atn": [_od_pad(p) for p in atn], "clk": [_od_pad(p) for p in clk],
                      "data": [_od_pad(p) for p in data]}
        self.masks = {k: sum(1 << b for _, b in pads) for k, pads in self.lines.items()}
        self.us = us
        self.agents: List[Agent] = []
        self.level = (1, 1, 1)
        self.edges: List[Tuple[int, int, int, int]] = [(-1, 1, 1, 1)]
        self.pulls: Dict[Tuple[str, str], List[Tuple[int, int]]] = {}
        self._open: Dict[Tuple[str, str], int] = {}
        self._agent = (False, False, False)
        self._chip = 0
        self._alone = 0

    @property
    def pullups(self) -> int:
        return self.masks["atn"] | self.masks["clk"] | self.masks["data"]

    def add(self, agent: Agent) -> Agent:
        self.agents.append(agent)
        return agent

    def drive(self, drive: Drive, cycle: int) -> None:
        for i, line in enumerate(LINES):
            mask = self.masks[line]
            for pad in self.lines[line]:
                if self._agent[i] or self._chip & mask & ~(1 << pad[1]):
                    drive.pull_low(pad)

    def observe(self, lines: Lines) -> None:
        cycle = lines.cycle
        level = tuple(int((lines.uio & self.masks[k]) == self.masks[k]) for k in LINES)
        if level != self.level:
            self.edges.append((cycle,) + level)
            self.level = level
        chip = lines.uio_oe & ~lines.uio_out & self.pullups
        pull = [False, False, False]
        names = []
        for agent in self.agents:
            got = agent.clock(level[0], level[1], level[2], cycle)
            for i, p in enumerate(got):
                if p:
                    pull[i] = True
                    names.append((agent.name, LINES[i]))
        for line in LINES:
            names += [("chip:uio%d" % b, line) for _, b in self.lines[line] if chip >> b & 1]
        self._record(cycle, set(names))
        self._agent, self._chip = tuple(pull), chip

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
            lines.uio = sum(0 if self._agent[i] else self.masks[k]
                            for i, k in enumerate(LINES))
            self.observe(lines)
            self._alone += 1
        return until()

    def intervals(self, name: str, line: str) -> List[Tuple[int, Optional[int]]]:
        out = list(self.pulls.get((name, line), []))
        if (name, line) in self._open:
            out.append((self._open[(name, line)], None))
        return out


class _Proc(Agent):
    """An agent run by a generator that yields cycle counts to wait. The
    pulls are registered: what the generator sets is driven next cycle."""

    def __init__(self, us: int) -> None:
        self.us = us
        self.pull = [False, False, False]       # ATN, CLK, DATA pulled low?
        self.lv = (1, 1, 1)
        self.cycle = 0
        self._gen = self._main()
        self._wait = 0

    def clock(self, atn: int, clk: int, data: int, cycle: int) -> Tuple[bool, bool, bool]:
        self.lv = (atn, clk, data)
        self.cycle = cycle
        if self._wait > 1:
            self._wait -= 1
        else:
            self._wait = next(self._gen)
        return tuple(self.pull)

    def _main(self):                            # pragma: no cover - overridden
        while True:
            yield 1

    def _t(self, us: float) -> int:
        return max(1, int(round(us * self.us)))

    def _until(self, cond, limit_us: Optional[float] = None):
        """Wait until ``cond()``; False if ``limit_us`` passes first."""
        left = None if limit_us is None else self._t(limit_us)
        while not cond():
            if left is not None:
                if left <= 0:
                    return False
                left -= 1
            yield 1
        return True

    def _send(self, value: int, eoi: bool, hold_us: float, tne_us: float = 20):
        """Talker, one byte ([B] steps 1 to 4, [S4] steps 1 to 30), entered
        holding CLK. True if the listener's frame handshake came in 1 ms."""
        self.pull[1] = False                    # ready to send
        yield 2
        yield from self._until(lambda: self.lv[2] == 1)     # ready for data
        if eoi:                                 # hold CLK back: EOI ([B] step 2)
            yield from self._until(lambda: self.lv[2] == 0)
            yield from self._until(lambda: self.lv[2] == 1)
        yield self._t(tne_us)
        self.pull[1] = True                     # data not valid
        for i in range(8):
            self.pull[2] = not (value >> i & 1)    # 1 = released (from memory)
            yield self._t(hold_us)
            self.pull[1] = False                # data valid
            yield self._t(hold_us)
            self.pull[1] = True
            self.pull[2] = False
        yield 2
        ok = yield from self._until(lambda: self.lv[2] == 0, TIMING["frame_ack_max"][0])
        return ok

    def _recv(self, delay: int, ack: bool = True):
        """Listener, one byte, entered holding DATA after ready to send was
        seen. Returns ``(value, eoi)``; DATA held after (the frame) if ``ack``."""
        yield delay
        self.pull[2] = False                    # ready for data
        yield 2
        quick = yield from self._until(lambda: self.lv[1] == 0, TIMING["eoi_min"][0])
        if not quick:                           # EOI: acknowledge it ([B])
            self.pull[2] = True
            yield self._t(80)
            self.pull[2] = False
            yield from self._until(lambda: self.lv[1] == 0)
        value = 0
        for i in range(8):
            yield from self._until(lambda: self.lv[1] == 1)
            value |= self.lv[2] << i
            yield from self._until(lambda: self.lv[1] == 0)
        if ack:
            yield delay
            self.pull[2] = True                 # frame handshake
        return value, not quick


class IecController(_Proc):
    """A controller (a C64) on the bus. Queue :meth:`send` (LISTEN pa,
    SECOND sa, the data with EOI on the last byte, UNLISTEN) and
    :meth:`receive` (TALK pa, SECOND sa, the turnaround, bytes up to EOI,
    UNTALK). :attr:`results` gets ``("send", error)`` and ``("receive",
    bytes, error)``, ``error`` None or ``"nodev"`` (no DATA within 1 ms of
    ATN, or no listener holding DATA before a data byte), ``"frame"`` (no
    frame handshake in 1 ms) or ``"turn"`` (the
    device did not take CLK in 1 ms). ``hold_us`` is the bit's CLK low and
    CLK high time ([S4]: controllers may use 20; the C64 holds 42)."""

    name = "controller"

    def __init__(self, *, us: int = 50, hold_us: float = 42, delay: int = 20) -> None:
        self.hold_us = hold_us
        self.delay = delay
        self.results: List[tuple] = []
        self._ops: deque = deque()
        self._active = False
        super().__init__(us)

    def send(self, pa: int, sa: int, data: Sequence[int]) -> None:
        if not data:
            raise ValueError("a send needs at least one byte (the EOI byte)")
        self._ops.append(("send", pa, sa, [d & 0xFF for d in data]))

    def receive(self, pa: int, sa: int) -> None:
        self._ops.append(("receive", pa, sa, None))

    @property
    def busy(self) -> bool:
        return bool(self._ops) or self._active

    def _release(self) -> None:
        self.pull = [False, False, False]

    def _atn(self, commands: Sequence[int]):
        """ATN with the command bytes ([B] ATN sequence); None or an error."""
        self.pull = [True, True, False]
        yield 4
        if not (yield from self._until(lambda: self.lv[2] == 0,
                                       TIMING["atn_response_max"][0])):
            return "nodev"
        for c in commands:
            yield self._t(TIMING["between_bytes_min"][0])
            if not (yield from self._send(c, False, self.hold_us)):
                return "frame"
        yield self._t(30)
        self.pull[0] = False                    # ATN released
        return None

    def _main(self):
        while True:
            if not self._ops:
                self._active = False
                yield 1
                continue
            self._active = True
            op, pa, sa, data = self._ops.popleft()
            if op == "send":
                err = yield from self._atn([LISTEN | pa, SECOND | sa])
                for i, b in enumerate(data if err is None else []):
                    yield self._t(TIMING["between_bytes_min"][0])
                    if self.lv[2] == 1:         # no listener holds DATA ([S4] Errors)
                        err = "nodev"
                        break
                    if not (yield from self._send(b, i == len(data) - 1, self.hold_us)):
                        err = "frame"
                        break
                if err is None:
                    yield self._t(TIMING["between_bytes_min"][0])
                    err = yield from self._atn([UNLISTEN])
                self._release()
                self.results.append(("send", err))
                yield self._t(TIMING["between_bytes_min"][0])     # the bus idle a while
                continue
            got: List[int] = []
            err = yield from self._atn([TALK | pa, SECOND | sa])
            if err is None:                     # the turnaround ([B], [S4] C, D)
                self.pull[2] = True
                yield self._t(30)
                self.pull[1] = False
                yield 4
                if not (yield from self._until(lambda: self.lv[1] == 0,
                                               TIMING["turnaround_max"][0])):
                    err = "turn"
            while err is None:
                yield from self._until(lambda: self.lv[1] == 1)     # ready to send
                value, eoi = yield from self._recv(self.delay)
                got.append(value)
                if eoi:
                    break
            if err is None:
                yield self._t(80)
                self.pull[2] = False
                yield self._t(TIMING["between_bytes_min"][0])
                err = yield from self._atn([UNTALK])
            self._release()
            self.results.append(("receive", got, err))
            yield self._t(TIMING["between_bytes_min"][0])


class IecDevice(_Proc):
    """A device (a disk drive) with primary address ``pa``. It answers every
    ATN within ``delay`` cycles, follows the commands of [S2] for its
    address, records what it receives as listener and talks back
    :attr:`reply` (last byte with EOI) when told to TALK. ``hold_us`` is its
    bit's CLK low and high time ([S4]: devices 60).

    :attr:`log`: ``("cmd", byte)`` for every byte under ATN, ``("data",
    byte, eoi)`` for every byte received as listener, ``("sent", byte, ok)``
    for every byte talked.

    Error injection: ``present=False`` pulls nothing (no device on the bus);
    ``nack=`` a set of numbers of bytes (0 = the first received, commands
    included) it does not acknowledge, after which it lets the bus go and
    waits for the next ATN; ``no_turn=True`` never takes CLK at a TALK.
    """

    name = "device"

    def __init__(self, pa: int = 8, *, reply: Sequence[int] = (0x30,), us: int = 50,
                 hold_us: float = 60, delay: int = 20, present: bool = True,
                 nack: Sequence[int] = (), no_turn: bool = False) -> None:
        self.pa = pa
        self.reply = list(reply)
        self.hold_us = hold_us
        self.delay = delay
        self.present = present
        self.nack = set(nack)
        self.no_turn = no_turn
        self.log: List[tuple] = []
        self.received = 0
        self.state = 0                           # 1 listener, 2 talker
        self._nacked = False
        super().__init__(us)

    @property
    def data(self) -> List[int]:
        return [e[1] for e in self.log if e[0] == "data"]

    def _byte(self):
        ack = self.received not in self.nack
        self.received += 1
        self._nacked = not ack
        value, eoi = yield from self._recv(self.delay, ack)
        return value, eoi, ack

    def _command(self, b: int) -> None:
        group, low = b >> 5, b & 0x1F
        if b == UNLISTEN:
            self.state &= 2
        elif b == UNTALK:
            self.state &= 1
        elif group == 1 and low == self.pa:
            self.state = 1
        elif group == 2:
            self.state = 2 if low == self.pa else self.state & 1

    def _main(self):
        while True:
            self.pull = [False, False, False]
            if self._nacked:                    # the controller gives up on ATN first
                yield from self._until(lambda: self.lv[0] == 1)
                self._nacked = False
            yield from self._until(lambda: self.lv[0] == 0 and self.present)
            again = True
            while again:
                again = False
                yield self.delay
                self.pull = [False, False, True]        # ATN acknowledged
                ok = True
                while ok:
                    yield from self._until(lambda: self.lv[0] == 1 or self.lv[1] == 1)
                    if self.lv[0] == 1:
                        break
                    b, _, ok = yield from self._byte()
                    self.log.append(("cmd", b))
                    self._command(b)
                if not ok or self.state == 0:
                    break
                if self.state == 1:                     # listener
                    while True:
                        yield from self._until(lambda: self.lv[0] == 0 or self.lv[1] == 1)
                        if self.lv[0] == 0:
                            again = True
                            break
                        b, eoi, ok = yield from self._byte()
                        self.log.append(("data", b, eoi))
                        if not ok or eoi:
                            yield self._t(80)
                            break
                elif not self.no_turn:                  # talker: the turnaround
                    yield from self._until(lambda: self.lv[1] == 1)
                    yield self.delay
                    self.pull = [False, True, False]
                    for i, b in enumerate(self.reply):
                        yield self._t(TIMING["between_bytes_min"][0])
                        ok = yield from self._send(b, i == len(self.reply) - 1, self.hold_us)
                        self.log.append(("sent", b, ok))
                        if not ok:
                            break
                    yield self._t(80)


def check(edges: Sequence[Tuple[int, int, int, int]], us: int = 50) -> dict:
    """Decode the bus from its edges alone and check every window above.

    Returns ``events``: ``("atn", cycle, response_us)`` (``None``: no DATA
    pull before ATN was released), ``("atn_off", cycle)``, ``("byte",
    value, eoi, under_atn, ack_us, low_us, high_us)`` (``ack_us`` None: no
    frame handshake within 1 ms; ``low_us``/``high_us`` the shortest CLK
    low before a bit and CLK high with one), ``("turn", cycle, us)`` (CLK
    released with DATA held, then pulled back without a byte: the talk
    turnaround); ``violations``, ``(cycle, text)`` for every window broken;
    ``lows`` and ``highs``, every bit's two widths in microseconds.
    """
    f = float(us)
    ev: List[tuple] = []
    bad: List[Tuple[int, str]] = []
    lows: List[float] = []
    highs: List[float] = []
    st = {"ph": "idle", "atn_t": None, "acked": None}
    cur: dict = {}
    atn, clk, data = 1, 1, 1

    def close_frame(t: int) -> None:
        if st["ph"] == "frame" and (t - cur["end"]) / f > TIMING["frame_ack_max"][0]:
            ev.append(("byte", cur["v"], cur["eoi"], cur["atn"], None, cur["lo"], cur["hi"]))
            st["ph"] = "idle"

    for t, a, c, d in edges[1:]:
        close_frame(t)
        if st["atn_t"] is not None and (t - st["atn_t"]) / f > TIMING["atn_response_max"][0]:
            ev.append(("atn", st["atn_t"], None))
            st["atn_t"] = None
        if a != atn:
            atn = a
            if st["ph"] == "bits":
                bad.append((t, "ATN changed inside a byte"))
            if st["atn_t"] is not None:                  # released unanswered
                ev.append(("atn", st["atn_t"], None))
                st["atn_t"] = None
            if a == 0 and data == 0:
                ev.append(("atn", t, 0.0))
            elif a == 0:
                st["atn_t"] = t
            else:
                ev.append(("atn_off", t))
            st["ph"], st["acked"] = "idle", None
        if c != clk:
            clk = c
            ph = st["ph"]
            if c == 1 and ph == "idle" and data == 0:
                if st["acked"] is not None and (t - st["acked"]) / f < TIMING["between_bytes_min"][0]:
                    bad.append((t, "ready to send %.1f us after the frame, under 100"
                                % ((t - st["acked"]) / f)))
                st["ph"], cur = "rts", {"rts": t}
            elif c == 0 and ph == "rts":
                ev.append(("turn", t, (t - cur["rts"]) / f))
                st["ph"], st["acked"] = "idle", None
            elif c == 0 and ph in ("rfd", "eoi_wait"):
                gap = (t - cur["mark"]) / f
                if ph == "rfd" and gap >= TIMING["non_eoi_max"][0]:
                    bad.append((t, "CLK %.1f us after ready for data, no EOI acknowledge" % gap))
                if ph == "eoi_wait" and gap > TIMING["eoi_resume_max"][0]:
                    bad.append((t, "CLK %.1f us after the EOI acknowledge, over 60" % gap))
                cur.update(ph=None, eoi=ph == "eoi_wait", n=0, v=0, fall=t, lo=1e9, hi=1e9,
                           atn=atn == 0)
                st["ph"] = "bits"
            elif c == 0 and ph == "eoi_ack":
                bad.append((t, "CLK pulled during the EOI acknowledge"))
            elif ph == "bits" and c == 1:
                w = (t - cur["fall"]) / f
                lows.append(w)
                cur["lo"] = min(cur["lo"], w)
                cur["rise"] = t
                cur["v"] |= data << cur["n"]
            elif ph == "bits" and c == 0:
                w = (t - cur["rise"]) / f
                highs.append(w)
                cur["hi"] = min(cur["hi"], w)
                cur["fall"] = t
                cur["n"] += 1
                if cur["n"] == 8:
                    st["ph"], cur["end"] = "frame", t
        if d != data:
            data = d
            ph = st["ph"]
            if d == 0 and st["atn_t"] is not None:
                ev.append(("atn", st["atn_t"], (t - st["atn_t"]) / f))
                st["atn_t"] = None
            if ph == "rts" and d == 1 and clk == 1:
                st["ph"], cur["mark"] = "rfd", t
            elif ph == "rfd" and d == 0:
                gap = (t - cur["mark"]) / f
                if gap < TIMING["eoi_min"][0]:
                    bad.append((t, "EOI acknowledged %.1f us after ready for data" % gap))
                st["ph"], cur["ack"] = "eoi_ack", t
            elif ph == "eoi_ack" and d == 1:
                w = (t - cur["ack"]) / f
                if w < TIMING["eoi_ack_min"][0]:
                    bad.append((t, "EOI acknowledge %.1f us, under 60" % w))
                st["ph"], cur["mark"] = "eoi_wait", t
            elif ph == "bits" and clk == 1:
                bad.append((t, "DATA changed while CLK says valid"))
            elif ph == "frame" and d == 0:
                ack = (t - cur["end"]) / f
                ev.append(("byte", cur["v"], cur["eoi"], cur["atn"], ack, cur["lo"], cur["hi"]))
                st["ph"], st["acked"] = "idle", (None if cur["eoi"] else t)
    for w in lows + highs:
        if w < TIMING["bit_min"][0]:
            bad.append((-1, "bit phase %.1f us, under 20" % w))
            break
    return {"events": ev, "violations": bad, "lows": lows, "highs": highs}


def frames(events: Sequence[tuple]) -> List[List[tuple]]:
    """The bytes grouped by ATN: a list per run of bytes with the same ATN
    level, a new one at every ATN, each byte ``(value, eoi)``."""
    out: List[List[tuple]] = []
    last = None
    for e in events:
        if e[0] == "atn":
            last = None
        if e[0] != "byte":
            continue
        if e[3] != last:
            out.append([])
            last = e[3]
        out[-1].append((e[1], e[2]))
    return out
