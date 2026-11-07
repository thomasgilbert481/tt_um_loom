"""EnDat 2.2 reference models: an absolute position encoder and its master.

Written from the protocol description, not from the firmware. The source is
HEIDENHAIN, *Technical Information EnDat 2.2 - Bidirectional Interface for
Position Encoders*, ID 383942-25, 9/2011 (the public 20-page PDF; read
2026-11-07 from the heidenhain.us copy, the heidenhain.de copy refused the
fetch). Page numbers are the document's own:

* p. 4 "Data Transfer": the clock idles high ("When not transmitting, the
  clock signal is on high level"); 100 kHz to 2 MHz without
  propagation-delay compensation, up to 16 MHz with it. Determining the
  propagation time: the master sends the mode command "Encoder send position
  values"; "after in total 10 clock periods" a counter starts at every rising
  clock edge and the master measures the time from the last rising edge to
  the edge of the start bit; it must sample at least eight times faster than
  the clock it will use.
* p. 5 "Selecting the Transmission Type": a mode command is three bits
  M2 M1 M0, each sent "redundantly (inverted or double)"; "Encoder send
  position values" is 000111 (EnDat 2.1 command set). An encoder that
  detects an incorrect mode transmission "transmits an error message".
* p. 6 "Position Values": the cycle begins with the first falling clock
  edge, which saves the position; "After two clock pulses (2T)" the master
  sends the mode command; after the calculation time tcal the start bit,
  then the error bit(s) (error 2 only with EnDat 2.2 commands, so one error
  bit here), the position LSB first, the CRC. "With the end of the data
  word, the clock must be set to HIGH. After 10 to 30 us or 1.25 to 3.75 us
  (with EnDat 2.2 parameterizable recovery time tm) the data line falls
  back to LOW. Then a new data transmission can be initiated". Table: fc
  100 kHz to 2 MHz (16 MHz with compensation), tcal typically 5 us,
  tm 10 to 30 us (EnDat 2.1), data delay tD = (0.2 + 0.01 x cable length in
  m) us, tHI and tLO 0.2 to 10 us. tR (max. 500 ns) and tST (2 to 10 us)
  are in the table but their meaning is not in the text read (**unchecked**;
  not used here).

**Unchecked** (not in that document; the polynomial comes from a
mikrocontroller.net thread, "CRC Polynom fuer EnDAT Encoder", 2021, which
reads it off a published shift-register drawing; the rest is the author's
reading and is listed in the W-090 QUESTIONS.md):

* the CRC: 5 bits, x^5 + x^3 + x + 1 (0x0B), start value 11111, over the
  error bit and the position in transmission order, sent inverted, MSB
  first;
* the edges: both ends change DATA after a rising clock edge and the
  receiver samples at the falling edge (without compensation); the mode bits
  go out after rises 2 .. 7 and are read at falls 3 .. 8; the master
  releases DATA at rise 9 and the encoder drives it (low) from rise 10; the
  start bit comes on rise 11 at the earliest (10 periods after the first
  falling edge), later by whole periods if tcal is longer; the error bit is
  active high; the frame ends with one more rising edge after the last CRC
  bit, from which tm runs with DATA high;
* the encoder answers a wrong mode with the error bit set.

On a real board CLOCK and DATA are RS-485; these models are the logic side.
The master has DATA out, a driver enable (DE) and DATA in; the encoder the
same. :class:`Line` resolves the bus. Times are in core clocks.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Sequence

from tools.protomodels.bench import Drive, Lines, Model, PinSpec, pad_of

CRC_POLY = 0x0B                     # x^5 + x^3 + x + 1 (unchecked)
CRC_INIT = 0x1F
CRC_BITS = 5
MODE_POS = (0, 0, 0, 1, 1, 1)       # "Encoder send position values" (p. 5)
FIRST_START_RISE = 11               # 10 periods after the first fall (p. 4)
US = 50                             # clocks per microsecond at 50 MHz


def crc5(bits: Sequence[int]) -> int:
    """CRC-5 (0x0B, start 11111) of ``bits`` in transmission order."""
    c = CRC_INIT
    for b in bits:
        fb = ((c >> 4) & 1) ^ (b & 1)
        c = (c << 1) & 0x1F
        if fb:
            c ^= CRC_POLY
    return c


def lsb_first(value: int, n: int) -> List[int]:
    return [(value >> i) & 1 for i in range(n)]


def msb_first(value: int, n: int) -> List[int]:
    return [(value >> (n - 1 - i)) & 1 for i in range(n)]


def endat_bits(pos: int, n: int, *, err: bool = False,
               bad_crc: bool = False) -> List[int]:
    """What the encoder sends after the start bit: F1, the n position bits
    LSB first, the CRC-5 inverted MSB first (not inverted: ``bad_crc``)."""
    data = [1 if err else 0] + lsb_first(pos, n)
    c = crc5(data)
    if not bad_crc:
        c ^= 0x1F
    return data + msb_first(c, CRC_BITS)


def decode_endat(bits: Sequence[int], n: int) -> Dict[str, int]:
    """F1, position and CRC check from the ``n + 6`` bits after the start bit."""
    if len(bits) < n + 6:
        raise ValueError("an EnDat frame of %d bits needs %d" % (len(bits), n + 6))
    data = list(bits[:n + 1])
    rx = 0
    for b in bits[n + 1:n + 6]:
        rx = (rx << 1) | b
    pos = sum(b << i for i, b in enumerate(data[1:]))
    return {"f1": data[0], "pos": pos, "crc_ok": int((rx ^ 0x1F) == crc5(data))}


IDLE_BUS = 1                        # an undriven RS-485 pair, fail-safe receiver


class Encoder(Model):
    """A model EnDat encoder answering "Encoder send position values".

    Pins: ``clk`` and the master's ``mdata`` / ``mde`` are chip outputs it
    reads; ``data`` is the chip input it drives with the bus level (its own
    output while its driver is on, the master's while the master's is, else
    :data:`IDLE_BUS`). Its output reaches the master ``delay`` clocks after
    the rising edge that causes it (output delay plus the cable both ways).
    ``tcal`` whole periods move the start bit past rise 11; ``err`` sets F1,
    ``bad_crc`` sends the CRC un-inverted; ``tm`` is the recovery time.

    ``frames`` records each frame: ``start``, ``edges`` (``(cycle, level)``),
    ``mode`` (the six bits read), ``pos``, ``complete``. ``violations``:
    frames started during tm, the master driving DATA while the encoder does,
    the master changing DATA too close to the falling edge that reads it.
    """

    def __init__(self, clk: PinSpec, mdata: PinSpec, mde: PinSpec,
                 data: PinSpec, *, n: int = 25, pos: int = 0, delay: int = 10,
                 tcal: int = 0, tm: int = 20 * US, err: bool = False,
                 bad_crc: bool = False, quiet: int = 15 * US) -> None:
        self.clk, self.mdata, self.mde = pad_of(clk), pad_of(mdata), pad_of(mde)
        self.data = pad_of(data)
        self.n, self.pos, self.delay, self.tcal, self.tm = n, pos, max(1, delay), tcal, tm
        self.err, self.bad_crc, self.quiet = err, bad_crc, quiet
        self.out, self.de = 0, 1             # idle: ready (DATA low)
        self.bus = 0
        self.pending: List[List[int]] = []
        self.prev_clk: Optional[int] = None
        self.prev_mdata = 0
        self.mdata_change = -10 ** 9
        self.state = "idle"
        self.rise = self.fall = 0
        self.last_edge = 0
        self.seq: List[int] = []
        self.frames: List[Dict] = []
        self.violations: List[str] = []

    def drive(self, drive: Drive, cycle: int) -> None:
        drive.set(self.data, self.bus)

    def _out(self, cycle: int, value: int) -> None:
        self.pending.append([cycle + self.delay, value])

    def _sequence(self, mode: List[int]) -> List[int]:
        bad = tuple(mode) != MODE_POS
        return [1] + endat_bits(self.frames[-1]["pos"], self.n,
                                err=self.err or bad, bad_crc=self.bad_crc)

    def observe(self, lines: Lines) -> None:
        c = lines.cycle
        clk = lines.get(self.clk)
        md, mde = lines.get(self.mdata), lines.get(self.mde)
        if md != self.prev_mdata:
            self.mdata_change = c
        self.prev_mdata = md
        edge = self.prev_clk is not None and clk != self.prev_clk
        self.prev_clk = clk
        if mde and self.de:
            self.violations.append("cycle %d: master and encoder both drive DATA" % c)
        if edge:
            self.last_edge = c
            if self.state == "frame":
                self._edge(c, clk, md)
            elif clk == 0 and self.state == "idle":
                if self.pending:
                    self.violations.append("cycle %d: frame started before DATA settled" % c)
                self.state, self.rise, self.fall = "frame", 0, 1
                self.de = 0                         # the master takes DATA
                self.frames.append({"start": c, "edges": [(c, 0)], "pos": self.pos,
                                    "mode": [], "complete": False})
            elif clk == 0 and self.state == "tm":
                self.violations.append("cycle %d: frame started during tm" % c)
        elif self.state == "frame" and clk == 1 and c - self.last_edge >= self.quiet:
            self._recover(c)                        # the master left the frame
        elif self.state == "tm" and c >= self.tm_end:
            self._out(c, 0)                         # ready
            self.state = "idle"
        while self.pending and self.pending[0][0] <= c + 1:
            self.out = self.pending.pop(0)[1]
        if self.de:
            self.bus = self.out
        else:
            self.bus = md if mde else IDLE_BUS

    def _recover(self, c: int) -> None:
        self._out(c, 1)
        self.state, self.tm_end = "tm", c + self.delay + self.tm

    def _edge(self, c: int, clk: int, md: int) -> None:
        f = self.frames[-1]
        f["edges"].append((c, clk))
        if clk == 0:
            self.fall += 1
            if 3 <= self.fall <= 8:
                if c - self.mdata_change < 2:
                    self.violations.append("cycle %d: mode bit changed at the falling edge" % c)
                f["mode"].append(md)
            return
        self.rise += 1
        r = self.rise
        if r == 10:
            self.de, self.out = 1, 0                # the encoder takes DATA, low
            self.seq = self._sequence(f["mode"])
        first = FIRST_START_RISE + self.tcal
        if r >= first:
            k = r - first
            if k < len(self.seq):
                self._out(c, self.seq[k])
            else:                                   # the closing rise: tm
                f["complete"] = True
                self._recover(c)


class Master(Model):
    """A model EnDat master: CLOCK, DATA out and its driver enable are chip
    inputs it drives (``clk``, ``mdata``); it reads the encoder's DATA
    (``edata``) and driver enable (``ede``), chip outputs.

    A frame: fall 1, then a square wave of ``period`` clocks (rise k at
    ``t0 + (k - 1) * period + period / 2``); the mode command after rises
    2 .. 7; DATA released at rise 9. From rise 11 it looks for the start bit
    every clock and measures ``delay``, the clocks from the last rising edge
    to the start bit (p. 4), then samples bit k in the middle of its cell,
    ``delay + period / 2`` after rise ``S + k``. The clock stops high after
    the closing rise S + n + 7. ``mode`` changes the command sent.

    ``results`` gets one dict per frame: ``decode_endat``'s fields, ``delay``,
    ``start_rise``, ``latency`` (most clocks from a rising edge to an encoder
    DATA change), ``tm`` (closing rise to DATA low), ``error``.
    """

    def __init__(self, clk: PinSpec, mdata: PinSpec, edata: PinSpec,
                 ede: PinSpec, *, period: int = 256, n: int = 25,
                 mode=MODE_POS, max_start: int = 16) -> None:
        self.clk, self.mdata = pad_of(clk), pad_of(mdata)
        self.edata, self.ede = pad_of(edata), pad_of(ede)
        self.period, self.half = period, period // 2
        self.n, self.mode, self.max_start = n, tuple(mode), max_start
        self.requests = 0
        self.level, self.dout, self.de = 1, 0, 0
        self.state = "idle"
        self.results: List[Dict] = []
        self.violations: List[str] = []
        self.prev_bus = 0

    def read(self, count: int = 1) -> None:
        self.requests += count

    def busy(self) -> bool:
        return self.requests > 0 or self.state != "idle"

    def drive(self, drive: Drive, cycle: int) -> None:
        drive.set(self.clk, self.level)
        drive.set(self.mdata, self.dout if self.de else IDLE_BUS)

    def _rise_at(self, k: int) -> int:
        return self.t0 + (k - 1) * self.period + self.half

    def observe(self, lines: Lines) -> None:
        c = lines.cycle
        ede = lines.get(self.ede)
        bus = lines.get(self.edata) if ede else IDLE_BUS
        if ede and self.de:
            self.violations.append("cycle %d: both drive DATA" % c)
        if self.state == "frame" and bus != self.prev_bus and self.last_rise is not None:
            self.latency = max(self.latency, c - self.last_rise)
        self.prev_bus = bus
        if self.state == "idle":
            if self.requests and ede and bus == 0:
                self.requests -= 1
                self.state, self.t0 = "frame", c + 1
                self.start, self.delay, self.bits = None, None, []
                self.error, self.latency, self.last_rise, self.end = None, 0, None, None
                self.level = 0
            return
        if self.state == "frame":
            self._frame(c, bus)
            return
        if self.state == "tm" and bus == 1:
            self.state = "tm_high"
        elif self.state == "tm_high" and bus == 0 or self.state == "tm" and self.error \
                and c - self.end > 40 * US:
            r = {"delay": self.delay, "start_rise": self.start, "latency": self.latency,
                 "tm": c - self.end, "error": self.error, "bits": list(self.bits)}
            if not self.error:
                r.update(decode_endat(self.bits, self.n))
            self.results.append(r)
            self.state = "idle"

    def _frame(self, c: int, bus: int) -> None:
        t = c + 1 - self.t0                         # the levels for cycle c + 1
        k = t // self.period + 1                    # the period cycle c + 1 is in
        ph = t % self.period
        if self.end is None or c + 1 < self.end:
            self.level = 0 if ph < self.half else 1
            if ph == self.half:
                self.last_rise = c + 1
        else:
            self.level = 1
        # the master's DATA: driven from rise 1 to rise 9, mode bits after rises 2 .. 7
        if ph == self.half + 1:
            if k == 1:
                self.de, self.dout = 1, 0
            elif 2 <= k <= 7:
                self.dout = self.mode[k - 2]
            elif k == 8:
                self.dout = 0
            elif k == 9:
                self.de = 0
        if self.start is None:
            if k >= FIRST_START_RISE and self.last_rise is not None and \
                    self.last_rise >= self._rise_at(FIRST_START_RISE) and bus == 1:
                self.delay = c - self.last_rise
                self.start = (self.last_rise - self.t0 - self.half) // self.period + 1
                self.end = self._rise_at(self.start + self.n + 7)
            elif k >= FIRST_START_RISE + self.max_start:
                self.error, self.end = "no start bit", c + 1
                self.level = 1
                self.state = "tm"
            return
        j = len(self.bits) + 1                      # the next bit's rise: S + j
        if c == self._rise_at(self.start + j) + self.delay + self.half:
            self.bits.append(bus)
        if len(self.bits) == self.n + 6 and c + 1 >= self.end:
            self.state = "tm"


class Line(Model):
    """The DATA pair between two Loom threads: each side's DATA out and
    driver enable are chip outputs; the resolved level goes to each side's
    DATA in, one clock late. ``clk_src`` / ``clk_dst`` also wire CLOCK, and
    ``cable`` delays everything the encoder drives (both directions of the
    cable folded into one, as tD is measured)."""

    def __init__(self, mdata: PinSpec, mde: PinSpec, min_: PinSpec,
                 edata: PinSpec, ede: PinSpec, ein: PinSpec,
                 clk_src: PinSpec, clk_dst: PinSpec, cable: int = 0) -> None:
        self.mdata, self.mde, self.min = pad_of(mdata), pad_of(mde), pad_of(min_)
        self.edata, self.ede, self.ein = pad_of(edata), pad_of(ede), pad_of(ein)
        self.csrc, self.cdst = pad_of(clk_src), pad_of(clk_dst)
        self.cable = cable
        self.hist: List[int] = []
        self.to_master = self.to_enc = IDLE_BUS
        self.clk = 1
        self.clk_edges: List[int] = []
        self.violations: List[str] = []

    def drive(self, drive: Drive, cycle: int) -> None:
        drive.set(self.min, self.to_master)
        drive.set(self.ein, self.to_enc)
        drive.set(self.cdst, self.clk)

    def observe(self, lines: Lines) -> None:
        c = lines.cycle
        md, mde = lines.get(self.mdata), lines.get(self.mde)
        ed, ede = lines.get(self.edata), lines.get(self.ede)
        if md is not None and mde and ede:
            self.violations.append("cycle %d: both drive DATA" % c)
        enc = ed if ede else None
        self.hist.append(enc)
        late = self.hist[-1 - self.cable] if len(self.hist) > self.cable else None
        if len(self.hist) > self.cable + 1:
            self.hist.pop(0)
        bus = md if mde else (late if late is not None else IDLE_BUS)
        self.to_master = bus
        self.to_enc = md if mde else IDLE_BUS
        clk = lines.get(self.csrc)
        if clk != self.clk:
            self.clk_edges.append(c)
        self.clk = clk


# ------------------------------------------------------------ timing checks
def half_periods(frame: Dict) -> List[int]:
    """Clocks between consecutive CLOCK edges of one :class:`Encoder` frame."""
    cycles = [c for c, _ in frame["edges"]]
    return [b - a for a, b in zip(cycles, cycles[1:])]


def clock_window(frames: Sequence[Dict], low: int, high: int) -> List[str]:
    """Every CLOCK half period outside ``low .. high`` clocks, as text."""
    bad = []
    for i, f in enumerate(frames):
        for j, h in enumerate(half_periods(f)):
            if not low <= h <= high:
                bad.append("frame %d half period %d: %d clocks, window %d .. %d"
                           % (i, j, h, low, high))
    return bad


def pulse_widths_ok(frames: Sequence[Dict], clk_hz: float = 50e6) -> List[str]:
    """tHI and tLO inside 0.2 .. 10 us (p. 6 table), at ``clk_hz``."""
    lo, hi = 0.2e-6 * clk_hz, 10e-6 * clk_hz
    return clock_window(frames, int(lo), int(hi))


def frequency_ok(period: int, clk_hz: float = 50e6) -> bool:
    """100 kHz .. 2 MHz without propagation-delay compensation (p. 4)."""
    return 100e3 <= clk_hz / period <= 2e6


def tm_ok(results: Sequence[Dict], low: int, high: int) -> List[str]:
    """Every :class:`Master` result whose tm is outside ``low .. high`` clocks."""
    return ["frame %d: tm %d clocks, expected %d .. %d" % (i, r["tm"], low, high)
            for i, r in enumerate(results) if not low <= r["tm"] <= high]
