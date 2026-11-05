"""BiSS-C and SSI reference models: absolute position encoders on a motor drive.

Written from the protocol descriptions, not from the firmware.

BiSS-C (point to point, single-cycle data only, no control frames):

* Beckhoff Information System, EL5042 "Basics of BiSS-C technology", fig. 5
  "BiSS-C communication process" (checked 2026-11-04): MA (the master's
  clock) and SLO (the slave's data) idle high; the slave pulls SLO low after
  the second rising MA edge (the Ack, which lasts while the slave works);
  then the start bit (1), always followed by a 0 (the CDS bit); then the
  position data, the error and warning bits and the CRC; the frame ends with
  the slave's timeout, during which the master holds MA high until the
  slave is ready (SLO high) again.
* iC-Haus BiSS application notes AN3 and AN6 (checked 2026-11-04, from the
  published summaries): the single-cycle data's CRC-6 has the polynomial
  0x43 (x^6 + x + 1) and is sent inverted.
* encoder.co.uk "BiSS C info" (checked 2026-11-04): the position, error and
  warning bits are covered by the CRC; the timeout is either constant,
  typically about 20 us, or adaptive (from the MA frequency).
* **Unchecked** (the iC-Haus "BiSS C Protocol Description" itself could not
  be fetched in this session; these are the author's reading of it): the
  slave changes SLO after rising MA edges (so a master samples before the
  next rising edge, or at the falling edge on a short line); nE and nW are
  active low (0 = error, warning); the CRC's start value is 0 and it covers
  the data, nE and nW in transmission order, MSB first; the CDM bit is sent
  inverted on MA during the timeout (MA held high is CDM = 0, which is all
  these models do); MA up to 10 MHz.

SSI (synchronous serial interface), as the absolute encoder datasheets of
Hengstler (ACURO series) and Kubler (Sendix series) draw it, **unchecked**
against a datasheet open in this session: the clock idles high; the first
falling edge latches the position (the monoflop is triggered); each rising
edge puts the next bit on the data line, MSB first, so n bits take n + 1
clock pulses and are read before the rising edges 2 .. n + 1; after the last
pulse the data line is low for the monoflop time tm (typically 15 to 25 us)
and then high; clock 100 kHz to 1 MHz; the code binary or Gray.

On a real board both lines are RS-422; these models are the logic side: MA
push-pull from the master, SLO push-pull from the slave.

What is here:

* :func:`crc6`, :func:`gray`, :func:`from_gray`;
* :func:`biss_bits` and :func:`ssi_bits`, the bits a slave sends, and
  :func:`decode_biss`, which checks a received single-cycle data word;
* :class:`Encoder`, a model slave (BiSS-C or SSI) with a position, a chosen
  output delay (the line delay), error injection (error and warning bits, a
  bad CRC, no start bit, a longer Ack) and a timing checker on the MA it
  sees (half periods, frames started before the slave was ready);
* :class:`Master`, a model master that clocks MA at a chosen period,
  samples SLO at a chosen point, decodes the frame and checks the slave's
  timing (each SLO change inside its bit cell, the timeout tTO);
* :class:`Wire`, the board wire between two Loom threads.

Times are in core clocks.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Sequence

from tools.protomodels.bench import Drive, Lines, Model, PinSpec, pad_of

CRC_POLY = 0x43                     # x^6 + x^1 + x^0 (AN6)
CRC_BITS = 6
US = 50                             # clocks per microsecond at 50 MHz
TTO_TYPICAL = 20 * US               # the usual constant BiSS timeout


def crc6(bits: Sequence[int]) -> int:
    """CRC-6 (0x43, start value 0) of ``bits`` in transmission order."""
    c = 0
    for b in bits:
        fb = ((c >> 5) & 1) ^ (b & 1)
        c = (c << 1) & 0x3F
        if fb:
            c ^= CRC_POLY & 0x3F
    return c


def gray(b: int) -> int:
    return b ^ (b >> 1)


def from_gray(g: int) -> int:
    b = 0
    while g:
        b ^= g
        g >>= 1
    return b


def msb_first(value: int, n: int) -> List[int]:
    return [(value >> (n - 1 - i)) & 1 for i in range(n)]


def biss_bits(pos: int, n: int, *, err: bool = False, warn: bool = False,
              bad_crc: bool = False, cds: int = 0) -> List[int]:
    """What a BiSS-C slave sends after the start bit: CDS, the n position
    bits MSB first, nE, nW and the CRC-6 inverted (not inverted with
    ``bad_crc``)."""
    data = msb_first(pos, n) + [0 if err else 1, 0 if warn else 1]
    c = crc6(data)
    if not bad_crc:
        c ^= 0x3F
    return [cds & 1] + data + msb_first(c, CRC_BITS)


def ssi_bits(pos: int, n: int) -> List[int]:
    """What an SSI encoder sends: the n position bits MSB first."""
    return msb_first(pos, n)


def decode_biss(bits: Sequence[int], n: int) -> Dict[str, int]:
    """Decode CDS, position, nE, nW and CRC from the bits after the start
    bit (``n + 9`` of them)."""
    if len(bits) < n + 9:
        raise ValueError("a BiSS-C frame of %d bits needs %d" % (len(bits), n + 9))
    data = list(bits[1:n + 3])
    rx_crc = 0
    for b in bits[n + 3:n + 9]:
        rx_crc = (rx_crc << 1) | b
    pos = 0
    for b in data[:n]:
        pos = (pos << 1) | b
    return {"cds": bits[0], "pos": pos, "nE": data[n], "nW": data[n + 1],
            "crc_ok": int((rx_crc ^ 0x3F) == crc6(data))}


class Encoder(Model):
    """A model BiSS-C or SSI slave on MA (a chip output) and SLO (a chip
    input).

    SLO changes ``delay`` clocks after the MA edge that causes it (the slave's
    output delay plus the line delay both ways). Error injection: ``err`` and
    ``warn`` send nE = 0 and nW = 0, ``bad_crc`` the CRC not inverted,
    ``no_start`` an Ack that never ends, ``ack`` the Ack's length in periods.

    ``frames`` records every frame: ``start`` (the cycle of its first
    falling edge), ``edges`` (``(cycle, level)`` of every MA edge in it),
    ``pos`` (the position latched) and ``complete`` (every bit went out).
    ``violations`` lists what the slave saw the master do wrong: a frame
    started while SLO was low (the timeout). MA edges during the timeout
    (a master on a line longer than a period clocks on after the last bit)
    are counted in ``timeout_edges`` and restart the timeout.
    """

    def __init__(self, ma: PinSpec, slo: PinSpec, *, n: int = 24, pos: int = 0,
                 ssi: bool = False, gray_code: bool = False, delay: int = 4,
                 tto: int = TTO_TYPICAL, ack: int = 1, err: bool = False,
                 warn: bool = False, bad_crc: bool = False,
                 no_start: bool = False) -> None:
        self.ma, self.slo = pad_of(ma), pad_of(slo)
        self.n, self.pos, self.ssi, self.gray_code = n, pos, ssi, gray_code
        self.delay, self.tto, self.ack = max(1, delay), tto, ack
        self.err, self.warn, self.bad_crc, self.no_start = err, warn, bad_crc, no_start
        self.level = 1
        self.pending: List[List[int]] = []
        self.prev_ma: Optional[int] = None  # the first cycle is no edge
        self.state = "idle"
        self.seq: List[Optional[int]] = []
        self.rise = 0
        self.last_edge = 0
        self.frames: List[Dict] = []
        self.violations: List[str] = []
        self.timeout_edges = 0

    def _sequence(self) -> List[Optional[int]]:
        """SLO after rises 1, 2, ... (None: unchanged); the timeout follows."""
        value = gray(self.pos) if self.gray_code else self.pos
        if self.ssi:
            return list(ssi_bits(value, self.n))
        if self.no_start:
            return [None] + [0] * 0x10000
        return ([None] + [0] * self.ack + [1]
                + biss_bits(value, self.n, err=self.err, warn=self.warn,
                            bad_crc=self.bad_crc))

    def _out(self, cycle: int, value: int) -> None:
        self.pending.append([cycle + self.delay, value])

    def drive(self, drive: Drive, cycle: int) -> None:
        drive.set(self.slo, self.level)

    def observe(self, lines: Lines) -> None:
        c = lines.cycle
        ma = lines.get(self.ma)
        edge = self.prev_ma is not None and ma != self.prev_ma
        self.prev_ma = ma
        if edge:
            if self.state == "frame":
                self.frames[-1]["edges"].append((c, ma))
            elif self.state == "timeout":
                self.timeout_edges += 1             # restarts the timeout
            elif ma == 0:                                   # idle: a frame
                if self.level == 0 or self.pending:
                    self.violations.append("cycle %d: frame started before SLO was ready" % c)
                self.state, self.rise, self.seq = "frame", 0, self._sequence()
                self.frames.append({"start": c, "edges": [(c, 0)], "pos": self.pos,
                                    "complete": False})
            if self.state == "frame" and ma == 1:
                self.rise += 1
                if self.rise <= len(self.seq):
                    v = self.seq[self.rise - 1]
                    if v is not None:
                        self._out(c, v)
                else:                                       # every bit is out
                    self.frames[-1]["complete"] = True
                    self._out(c, 0)
                    self.state = "timeout"
            self.last_edge = c
        elif ma == 1 and self.state in ("frame", "timeout") \
                and c - self.last_edge >= self.tto:
            if self.state == "frame":                       # the master left
                self._out(c, 0)
                self.last_edge = c
                self.state = "timeout"
            else:
                self._out(c, 1)
                self.state = "idle"
        while self.pending and self.pending[0][0] <= c + 1:
            self.level = self.pending.pop(0)[1]


class Master(Model):
    """A model BiSS-C or SSI master: MA on a chip input, SLO from a chip output.

    MA idles high; a frame is a falling edge and then a square wave of
    ``period`` clocks (rise k at ``start + (2k - 1) * period / 2``). SLO is
    sampled ``sample`` clocks after each rising edge (default ``period - 2``,
    the end of the bit cell, which is where a slave that changes SLO after
    the rising edge is surely valid). BiSS-C: the Ack is looked for from the
    cell of rise 2 on, then the start bit, then CDS and ``n + 8`` bits; SSI:
    ``n`` bits in the cells of rises 1 .. n. MA rises once more after the last
    sample and stays high; the frame is over when SLO is high again.

    Each :meth:`read` queues a frame; ``results`` gets one dict per frame
    (``decode_biss``'s fields, or ``pos`` for SSI, plus ``ack_rise``: the
    rise whose cell held the Ack, ``latency``: the most clocks from a rising
    edge to an SLO change in the frame, ``tto``: clocks from the last rising
    edge to SLO high, ``error``: why a frame was abandoned, or None).
    """

    def __init__(self, ma: PinSpec, slo: PinSpec, *, period: int = 64,
                 sample: Optional[int] = None, n: int = 24, ssi: bool = False,
                 max_ack: int = 16, max_start: int = 16) -> None:
        self.ma, self.slo = pad_of(ma), pad_of(slo)
        self.period, self.half = period, period // 2
        self.sample = period - 2 if sample is None else sample
        self.n, self.ssi, self.max_ack, self.max_start = n, ssi, max_ack, max_start
        self.requests = 0
        self.level = 1
        self.state = "idle"
        self.results: List[Dict] = []
        self.prev_slo = 1

    def read(self, count: int = 1) -> None:
        self.requests += count

    def busy(self) -> bool:
        return self.requests > 0 or self.state != "idle"

    def drive(self, drive: Drive, cycle: int) -> None:
        drive.set(self.ma, self.level)

    def _begin(self, c: int) -> None:
        self.requests -= 1
        self.state, self.t0, self.end = "frame", c + 1, None
        self.phase, self.bits, self.ack_rise, self.last_rise = "ack", [], None, None
        self.latency, self.error, self.seen_low = 0, None, False
        if self.ssi:
            self.phase = "data"

    def _sampled(self, k: int, v: int) -> None:
        """The sample of rise k's cell; may end the frame."""
        need = self.n if self.ssi else self.n + 9
        if self.phase == "ack":
            if k >= 2 and v == 0:
                self.phase, self.ack_rise = "start", k
            elif k >= 2 + self.max_ack:
                self.error = "no Ack"
        elif self.phase == "start":
            if v == 1:
                self.phase = "data"
            elif k >= self.ack_rise + self.max_start:
                self.error = "no start bit"
        else:
            self.bits.append(v)
        if self.error or len(self.bits) == need:
            self.end = self.t0 + (2 * (k + 1) - 1) * self.half   # one more rise

    def observe(self, lines: Lines) -> None:
        c = lines.cycle
        slo = lines.get(self.slo)
        if slo != self.prev_slo and self.state == "frame" and self.last_rise is not None:
            self.latency = max(self.latency, c - self.last_rise)
        self.prev_slo = slo
        if self.state == "idle":
            if self.requests and slo == 1:
                self._begin(c)
                self.level = 0
            return
        if self.state == "frame":
            t = c + 1 - self.t0                     # the level for cycle c + 1
            if self.end is not None and c + 1 >= self.end:
                self.level = 1
                if c + 1 == self.end:
                    self.last_rise = c + 1
                self.state = "done"
                self.seen_low = False
                return
            self.level = 0 if (t // self.half) % 2 == 0 else 1
            if self.level == 1 and t % self.half == 0:
                self.last_rise = c + 1
            k = (t + self.half) // self.period      # the rise this cell follows
            if self.end is None and k >= 1 and t - (2 * k - 1) * self.half == self.sample:
                self._sampled(k, slo)
            return
        if slo == 0:
            self.seen_low = True
        if slo == 1 and (self.seen_low or self.error):  # done: the timeout is over
            r = {"ack_rise": self.ack_rise, "latency": self.latency,
                 "tto": c - self.end, "error": self.error, "bits": list(self.bits)}
            if not self.error:
                if self.ssi:
                    pos = 0
                    for b in self.bits:
                        pos = (pos << 1) | b
                    r["pos"] = pos
                else:
                    r.update(decode_biss(self.bits, self.n))
            self.results.append(r)
            self.state = "idle"


class Wire(Model):
    """A board wire from a chip output to a chip input, one clock late."""

    def __init__(self, src: PinSpec, dst: PinSpec, level: int = 1) -> None:
        self.src, self.dst, self.level = pad_of(src), pad_of(dst), level

    def drive(self, drive: Drive, cycle: int) -> None:
        drive.set(self.dst, self.level)

    def observe(self, lines: Lines) -> None:
        self.level = lines.get(self.src)


# ------------------------------------------------------------ timing checks
def half_periods(frame: Dict) -> List[int]:
    """Clocks between consecutive MA edges of one :class:`Encoder` frame."""
    cycles = [c for c, _ in frame["edges"]]
    return [b - a for a, b in zip(cycles, cycles[1:])]


def clock_window(frames: Sequence[Dict], low: int, high: int) -> List[str]:
    """Every MA half period outside ``low .. high`` clocks (the master's
    clock window), as text; an empty list when all are inside."""
    bad = []
    for i, f in enumerate(frames):
        for j, h in enumerate(half_periods(f)):
            if not low <= h <= high:
                bad.append("frame %d half period %d: %d clocks, window %d .. %d"
                           % (i, j, h, low, high))
    return bad


def timeout_ok(results: Sequence[Dict], tto: int, slack: int) -> List[str]:
    """Every :class:`Master` result whose timeout (last rising MA edge to SLO
    high) is not ``tto`` to ``tto + slack`` clocks."""
    return ["frame %d: tTO %d clocks, expected %d .. %d" % (i, r["tto"], tto, tto + slack)
            for i, r in enumerate(results) if not tto <= r["tto"] <= tto + slack]
