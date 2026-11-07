"""Centronics (IEEE 1284 compatibility mode) reference models: printer, host
and a timing checker.

Written from public descriptions of the interface, not from the firmware.
IEEE 1284-1994 itself is not free and was **not read**; every figure here is
from a secondary source, named with it, and marked so:

* Signals, direction and polarity (Wikipedia "Parallel port", section
  Pinouts; Tektronix wiki "Parallel port", section Pinout; secondary):
  host to printer nSTROBE (active low), D0..D7 (D0 the LSB), nAUTOFD
  (active low), nINIT (active low, resets the printer), nSELECTIN (active
  low, tied low by a minimal host); printer to host nACK (active low), BUSY
  (high = busy), PE (paper out / paper end, high = out), SELECT (high =
  on line), nERROR (active low).
* The handshake (Tektronix wiki, Introduction; Wikipedia, History >
  Centronics; secondary): the host waits for BUSY low, puts the byte on the
  data lines, pulses nSTROBE low; the printer raises BUSY, takes the byte,
  pulses nACK and drops BUSY when it can take the next.
* Timing (:data:`TIMING`, nanoseconds):
  A data set-up to nSTROBE low >= 500, B nSTROBE width >= 500, C data hold
  after nSTROBE rises >= 500: Tektronix wiki ("Minimum width of the strobe
  pulse is 0.5 us, data setup and hold times should also be >= 0.5 us") and
  Lexmark C760 Technical Reference, chapter 8, table 8-1 "Typical Parallel
  Interface Timings", times A, B, C (secondary: that table is the printer's
  "Fastbytes" protocol; whether it is also its compatibility-mode table is
  not stated).
  D nSTROBE low to BUSY high <= 500 (table 8-1, time D; a property of that
  printer, secondary). E nACK low width >= 500 (time E). G BUSY low and nACK
  high to nSTROBE low >= 500 (time G). Table 8-1 notes that the order of
  nACK and BUSY "is not consistent", so the checker accepts BUSY falling
  before, with or after nACK rises.
  The Solaris ``ecpp.conf`` driver configuration says the Centronics
  specification asks for at least 1 us of strobe (secondary, unchecked); the
  tests run the host at 0.64 us and 1 us.
  nINIT width >= 50 us: from memory, **unchecked**. nACK widths of printers
  (a few microseconds to about 12 us): from memory, **unchecked**; the model
  printer takes its widths as parameters.

What is here:

* :class:`CentronicsMonitor`, a bus analyser on D0..D7, nSTROBE, BUSY and
  nACK: one record per strobe, with A, B, C, D, E, G measured on the pads,
  and ``violations`` for the windows above;
* :class:`ModelPrinter`, a printer for the host program, with chosen BUSY
  delay, processing time and nACK width, paper out, offline and a hold;
* :class:`ModelHost`, a host for the printer program, with chosen set-up,
  strobe, hold and gap, an optional wait for nACK, and nINIT pulses.

Times are in core clocks; ``clock_hz`` converts the nanosecond figures.
"""

from __future__ import annotations

from typing import Dict, Iterator, List, Optional, Sequence

from .bench import Drive, Lines, Model, PinSpec, pad_of

TIMING: Dict[str, float] = dict(setup=500, width=500, hold=500, busy=500,
                                ack=500, gap=500, init=50_000)


def ns_to_clocks(ns: float, clock_hz: float) -> float:
    return round(ns * 1e-9 * clock_hz, 6)          # 500 ns at 50 MHz is 25.0


def _pads(pins: Sequence[PinSpec]):
    return [pad_of(p) for p in pins]


def _byte(lines: Lines, pads) -> int:
    v = 0
    for i, pad in enumerate(pads):
        v |= lines.get(pad) << i
    return v


class CentronicsMonitor(Model):
    """Decodes the compatibility-mode handshake off the pads.

    ``records`` holds one dict per strobe: ``data`` (the lines when nSTROBE
    fell), ``fall``, ``rise``, ``setup`` (A), ``width`` (B), ``hold`` (C,
    ``None`` while the data have not changed since), ``busy`` (D: clocks
    from the fall to BUSY high, ``None`` if BUSY was already high or never
    rose), ``gap`` (G), ``ack_fall``, ``ack_width`` (E), ``busy_fall``.
    ``check`` names the windows that go into ``violations``.
    """

    def __init__(self, data: Sequence[PinSpec], strobe: PinSpec, busy: PinSpec,
                 ack: PinSpec, *, clock_hz: float = 50e6,
                 check: Sequence[str] = ("setup", "width", "hold", "gap", "ack"),
                 timing: Optional[Dict[str, float]] = None) -> None:
        self.data = _pads(data)
        self.strobe, self.busy_pad, self.ack_pad = (pad_of(strobe), pad_of(busy),
                                                    pad_of(ack))
        t = dict(TIMING, **(timing or {}))
        self.need = {k: ns_to_clocks(v, clock_hz) for k, v in t.items()}
        self.check = set(check)
        self.records: List[Dict] = []
        self.ack_widths: List[int] = []
        self.violations: List[str] = []
        self._prev = None
        self._last_change = -10**9
        self._ready_since: Optional[int] = None
        self._hold_rec: Optional[Dict] = None
        self._ack_fall: Optional[int] = None

    @property
    def bytes(self) -> List[int]:
        return [r["data"] for r in self.records]

    def _bad(self, c: int, text: str) -> None:
        self.violations.append("cycle %d: %s" % (c, text))

    def _min(self, c: int, key: str, got: float, what: str) -> None:
        if key in self.check and got < self.need[key]:
            self._bad(c, "%s %d clocks, needs >= %.1f" % (what, got, self.need[key]))

    def observe(self, lines: Lines) -> None:
        c = lines.cycle
        d = _byte(lines, self.data)
        s, b, a = (lines.get(self.strobe), lines.get(self.busy_pad),
                   lines.get(self.ack_pad))
        if self._prev is None:
            self._prev = (d, s, b, a)
            self._last_change = c
            if b == 0 and a == 1:
                self._ready_since = c
            return
        pd, ps, pb, pa = self._prev
        self._prev = (d, s, b, a)
        rec = self.records[-1] if self.records else None
        if d != pd:
            self._last_change = c
            if rec is not None and rec["rise"] is None:
                if "hold" in self.check:
                    self._bad(c, "data changed while nSTROBE is low")
            hr = self._hold_rec
            if hr is not None:
                hr["hold"] = c - hr["rise"]
                self._min(c, "hold", hr["hold"], "data hold (C)")
                self._hold_rec = None
        if b == 0 and a == 1:
            if not (pb == 0 and pa == 1):
                self._ready_since = c
        else:
            self._ready_since = None
        if ps == 1 and s == 0:                          # nSTROBE falls
            if self._hold_rec is not None:              # data held to here
                self._hold_rec["hold"] = c - self._hold_rec["rise"]
                self._hold_rec = None
            gap = None if self._ready_since is None else c - self._ready_since
            rec = dict(data=d, fall=c, rise=None, setup=c - self._last_change,
                       width=None, hold=None, busy=None, gap=gap,
                       busy_at_fall=b, ack_fall=None, ack_width=None,
                       busy_fall=None)
            self.records.append(rec)
            self._min(c, "setup", rec["setup"], "data set-up (A)")
            if gap is None:
                if "gap" in self.check:
                    self._bad(c, "nSTROBE fell while BUSY high or nACK low")
            else:
                self._min(c, "gap", gap, "ready to nSTROBE (G)")
        elif ps == 0 and s == 1 and rec is not None and rec["rise"] is None:
            rec["rise"] = c
            rec["width"] = c - rec["fall"]
            self._min(c, "width", rec["width"], "nSTROBE width (B)")
            self._hold_rec = rec
        if rec is not None:
            if pb == 0 and b == 1 and rec["busy"] is None and not rec["busy_at_fall"]:
                rec["busy"] = c - rec["fall"]
                if "busy" in self.check and rec["busy"] > self.need["busy"]:
                    self._bad(c, "BUSY rose %d clocks after nSTROBE fell, "
                              "D <= %.1f" % (rec["busy"], self.need["busy"]))
            if pb == 1 and b == 0 and rec["busy_fall"] is None:
                rec["busy_fall"] = c
        if pa == 1 and a == 0:
            self._ack_fall = c
            if rec is not None and rec["ack_fall"] is None:
                rec["ack_fall"] = c
        elif pa == 0 and a == 1 and self._ack_fall is not None:
            w = c - self._ack_fall
            self.ack_widths.append(w)
            if rec is not None and rec["ack_fall"] == self._ack_fall:
                rec["ack_width"] = w
            self._min(c, "ack", w, "nACK width (E)")
            self._ack_fall = None


class ModelPrinter(Model):
    """A printer on the host program's pins.

    Data are latched when nSTROBE falls. BUSY rises ``busy_delay`` clocks
    after the fall; ``process`` clocks after nSTROBE rises nACK falls for
    ``ack_width`` clocks, and BUSY falls ``busy_offset`` clocks after nACK
    rises (negative: before it; table 8-1's order "is not consistent").
    :meth:`set_status` sets paper out, offline, fault and a hold (buffer
    full); any of them holds BUSY high, and nINIT low does too.
    ``received`` lists ``(byte, nAUTOFD, nSELECTIN, cycle)`` per strobe;
    ``inits`` the nINIT low widths; ``errors`` strobes that came while BUSY
    was high or nACK low.
    """

    def __init__(self, data: Sequence[PinSpec], strobe: PinSpec, init: PinSpec,
                 selin: PinSpec, autofd: PinSpec, busy: PinSpec, ack: PinSpec,
                 pe: PinSpec, select: PinSpec, error: PinSpec, *,
                 busy_delay: int = 10, process: int = 100, ack_width: int = 250,
                 busy_offset: int = 0) -> None:
        self.data = _pads(data)
        self.strobe, self.init, self.selin, self.autofd = _pads(
            (strobe, init, selin, autofd))
        self.outs = _pads((busy, ack, pe, select, error))
        self.busy_delay, self.process = busy_delay, process
        self.ack_width, self.busy_offset = ack_width, busy_offset
        self.paper_out = self.offline = self.fault = self.hold = False
        self.received: List[tuple] = []
        self.inits: List[int] = []
        self.errors: List[int] = []
        self._byte_busy = 0
        self._ack = 1
        self._init_low: Optional[int] = None
        self._events: List[tuple] = []
        self._prev = None

    def set_status(self, *, paper_out: Optional[bool] = None,
                   offline: Optional[bool] = None, fault: Optional[bool] = None,
                   hold: Optional[bool] = None) -> None:
        for name, v in (("paper_out", paper_out), ("offline", offline),
                        ("fault", fault), ("hold", hold)):
            if v is not None:
                setattr(self, name, bool(v))

    def levels(self) -> List[int]:
        """``[BUSY, nACK, PE, SELECT, nERROR]`` as driven now."""
        held = self.paper_out or self.offline or self.fault or self.hold
        busy = 1 if (self._byte_busy or held or self._init_low is not None) else 0
        err = 0 if (self.paper_out or self.offline or self.fault) else 1
        return [busy, self._ack, int(self.paper_out), int(not self.offline), err]

    def drive(self, drive: Drive, cycle: int) -> None:
        for pad, v in zip(self.outs, self.levels()):
            drive.set(pad, v)

    def _at(self, cycle: int, fn) -> None:
        self._events.append((cycle, fn))

    def _set(self, name: str, v: int):
        return lambda: setattr(self, name, v)

    def observe(self, lines: Lines) -> None:
        c = lines.cycle
        s, i = lines.get(self.strobe), lines.get(self.init)
        if self._events:
            due = [e for e in self._events if e[0] <= c]
            self._events = [e for e in self._events if e[0] > c]
            for _, fn in sorted(due, key=lambda e: e[0]):
                fn()
        if self._prev is None:
            self._prev = (s, i)
            return
        ps, pi = self._prev
        self._prev = (s, i)
        if pi == 1 and i == 0:
            self._init_low = c
        elif pi == 0 and i == 1 and self._init_low is not None:
            self.inits.append(c - self._init_low)
            self._init_low = None
        if ps == 1 and s == 0:
            if self.levels()[0] or not self._ack:
                self.errors.append(c)
            self.received.append((_byte(lines, self.data), lines.get(self.autofd),
                                  lines.get(self.selin), c))
            self._at(c + self.busy_delay, self._set("_byte_busy", 1))
        elif ps == 0 and s == 1 and self.received:
            fall = c + self.process
            rise = fall + self.ack_width
            self._at(fall, self._set("_ack", 0))
            self._at(rise, self._set("_ack", 1))
            self._at(max(rise + self.busy_offset, c + 1),
                     self._set("_byte_busy", 0))


class ModelHost(Model):
    """A host on the printer program's pins.

    For each byte: wait (at most ``timeout`` clocks) for BUSY low and nACK
    high, wait ``gap`` clocks (G), put the byte on D0..D7, ``setup`` clocks
    (A), nSTROBE low for ``width`` (B), high, hold the data ``hold`` clocks
    (C); with ``wait_ack`` it then waits for the nACK pulse before the next
    byte, as some hosts do. The data stay on the lines until the next byte.
    :meth:`init` pulses nINIT low. ``sent`` lists ``(byte, fall cycle)``,
    ``timeouts`` the bytes not sent. ``status`` is ``[BUSY, nACK, PE,
    SELECT, nERROR]`` as last seen.
    """

    def __init__(self, data: Sequence[PinSpec], strobe: PinSpec, init: PinSpec,
                 selin: PinSpec, autofd: PinSpec, busy: PinSpec, ack: PinSpec,
                 pe: PinSpec, select: PinSpec, error: PinSpec, *,
                 setup: int = 30, width: int = 30, hold: int = 30, gap: int = 30,
                 wait_ack: bool = False, timeout: Optional[int] = None,
                 autofd_level: int = 1, selin_level: int = 0) -> None:
        self.data = _pads(data)
        self.strobe, self.init_pad, self.selin, self.autofd = _pads(
            (strobe, init, selin, autofd))
        self.ins = _pads((busy, ack, pe, select, error))
        self.setup, self.width, self.hold, self.gap = setup, width, hold, gap
        self.wait_ack, self.timeout = wait_ack, timeout
        self.levels = dict(d=0, s=1, i=1, sel=selin_level, af=autofd_level)
        self.status = [1, 1, 0, 0, 1]
        self.sent: List[tuple] = []
        self.timeouts: List[int] = []
        self._jobs: List[Iterator[None]] = []
        self._cycle = 0

    @property
    def busy(self) -> bool:
        return bool(self._jobs)

    def send(self, data: Sequence[int]) -> None:
        for v in data:
            self._jobs.append(self._send(v & 0xFF))

    def init(self, width: int) -> None:
        self._jobs.append(self._init(width))

    def set_lines(self, *, autofd: Optional[int] = None,
                  selin: Optional[int] = None) -> None:
        if autofd is not None:
            self.levels["af"] = autofd
        if selin is not None:
            self.levels["sel"] = selin

    def drive(self, drive: Drive, cycle: int) -> None:
        lv = self.levels
        for k, pad in enumerate(self.data):
            drive.set(pad, (lv["d"] >> k) & 1)
        drive.set(self.strobe, lv["s"])
        drive.set(self.init_pad, lv["i"])
        drive.set(self.selin, lv["sel"])
        drive.set(self.autofd, lv["af"])

    def observe(self, lines: Lines) -> None:
        self._cycle = lines.cycle
        self.status = [lines.get(p) for p in self.ins]
        while self._jobs:
            try:
                next(self._jobs[0])
                return
            except StopIteration:
                self._jobs.pop(0)

    def _wait(self, n: int) -> Iterator[None]:
        for _ in range(max(n - 1, 0)):
            yield

    def _send(self, v: int) -> Iterator[None]:
        t = 0
        while not (self.status[0] == 0 and self.status[1] == 1):
            t += 1
            if self.timeout is not None and t > self.timeout:
                self.timeouts.append(v)
                return
            yield
        yield from self._wait(self.gap)
        self.levels["d"] = v
        yield
        yield from self._wait(self.setup)
        self.levels["s"] = 0
        self.sent.append((v, self._cycle + 1))
        yield
        yield from self._wait(self.width)
        self.levels["s"] = 1
        yield
        yield from self._wait(self.hold)
        if self.wait_ack:
            while self.status[1] == 1:
                yield
            while self.status[1] == 0:
                yield

    def _init(self, width: int) -> Iterator[None]:
        self.levels["i"] = 0
        yield
        yield from self._wait(width)
        self.levels["i"] = 1
        yield
