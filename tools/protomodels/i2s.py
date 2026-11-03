"""I2S target receiver and PDM microphone reference models.

Written from the specifications, never from the firmware.

**I2S.** NXP UM11732 "I2S bus specification" Rev 3.0, 17 February 2022,
which republishes the Philips text of February 1986, revised 5 June 1996.

* 3.1: serial data is two's complement, MSB first; the receiver latches SD
  on the leading (LOW-to-HIGH) edge of SCK; the transmitter may change SD
  on either edge, within the timing below.
* 3.2: WS = 0 is channel 1 (left), WS = 1 channel 2 (right); WS is latched
  on the leading edge; it changes one clock period before the MSB, so the
  bit latched with the new WS is the previous word's LSB and the MSB of the
  new word is latched one SCK later.
* Table 3 (with notes 2 and 5): a controller's clock HIGH ``tHC`` and LOW
  ``tLC`` are at least 0.35 T, with a fixed mark/space ratio; the
  transmitter's delay ``tdtr`` is at most 0.8 T and its hold ``thtr`` at
  least 0, both measured from the RISING edge; the receiver needs set-up
  ``tsr`` >= 0.2 T and hold ``thr`` >= 0.

:class:`I2sRx` watches SCK, WS and SD every clock. It latches WS and SD on
each rising edge from the value in the cycle before it, measures every SCK
period, high and low time, and every change of SD and WS against the last
rising edge, and records anything outside Table 3 in
:attr:`I2sRx.violations`. A hold of 0 clocks (a change in the very cycle
SCK rises) is taken as a violation: on a sampled bench it is a race, which
is the safe reading of ``thtr >= 0``. Words are framed by the WS rule of
3.2 and paired into ``(left, right)`` frames.

**PDM.** ST MP34DT01-M, DocID026514 Rev 3, September 2014. Table 3 and
Table 5: clock 1 to 3.25 MHz in normal mode (period 308 to 1000 ns), duty
40 to 60 %, below 0.23 MHz the part powers down. Table 7: with L/R tied to
GND the data is valid while CLK is low and the output is high impedance
while CLK is high. Table 5: ``TL,EN`` 18 ns (data enabled after the
falling edge) and ``TL,DIS`` 16 ns (released after the rising edge), "from
design simulations"; the extracted table does not say whether they are
minima or maxima, and this model takes them as the maximum delays
(unchecked). The datasheet does not give the modulator, so
:class:`PdmMic` uses the textbook second-order single-bit sigma-delta
modulator (two integrators, a sign quantiser, the quantised value fed back
to both); what the test checks against is the bits it actually sent.

:func:`cic` is the reference decimator: the CIC filter of Hogenauer (IEEE
Trans. ASSP-29, 1981) written as its impulse response, a boxcar of length
R convolved with itself ``order`` times, applied directly to the bit
stream. It shares no structure with the firmware's integrators and combs.
"""

from __future__ import annotations

import dataclasses
import math
from typing import List, Optional, Sequence, Tuple

from .bench import Drive, Lines, Model, PinSpec, pad_of

#: Table 3 limits as fractions of T.
T_HIGH_MIN = T_LOW_MIN = 0.35
DELAY_MAX = 0.80
SETUP_MIN = 0.20

#: MP34DT01-M Table 5 and Table 3.
PDM_F_MIN_MHZ, PDM_F_MAX_MHZ = 1.0, 3.25
PDM_DUTY_MIN, PDM_DUTY_MAX = 0.40, 0.60
TL_EN_NS, TL_DIS_NS = 18.0, 16.0


@dataclasses.dataclass
class Violation:
    """One measurement the specification does not allow."""

    kind: str                       # "period" | "high" | "low" | "sd" | "ws" | ...
    cycle: int
    clocks: float
    low: float
    high: float
    detail: str = ""

    def __str__(self) -> str:                               # pragma: no cover
        return ("%s: %g clocks at cycle %d, outside %g .. %g%s"
                % (self.kind, self.clocks, self.cycle, self.low, self.high,
                   ": " + self.detail if self.detail else ""))


@dataclasses.dataclass
class I2sWord:
    """One word between two WS changes."""

    channel: int                    # 0 left, 1 right (the WS level)
    bits: int                       # how many bits it had
    raw: int                        # the bits, MSB first, as an unsigned int
    start: int                      # cycle of the rising edge of its MSB

    def value(self, width: int = 16) -> int:
        """The word as a ``width``-bit two's complement number (3.1: a longer
        word is truncated to its first bits, a shorter one zero-filled)."""
        if self.bits >= width:
            v = self.raw >> (self.bits - width)
        else:
            v = self.raw << (width - self.bits)
        return v - (1 << width) if v >> (width - 1) else v


class I2sRx(Model):
    """An I2S target receiver on ``sck``, ``ws`` and ``sd``.

    Args:
        period: the SCK period the controller promises, in clocks (may be
            fractional). Each measured period must be within ``period_tol``
            clocks of it (a gap in SCK is a violation), and the Table 3
            fractions are taken of it. ``None``: each measured period is its
            own T and no gap check is made.
        period_tol: clocks a period may differ from ``period`` (the 4-clock
            slot grid, or 1 clock for a fractional tick).
        strict: raise on the first violation.
    """

    def __init__(self, sck: PinSpec, ws: PinSpec, sd: PinSpec, *,
                 period: Optional[float] = None, period_tol: float = 1.0,
                 strict: bool = False) -> None:
        self.p_sck, self.p_ws, self.p_sd = pad_of(sck), pad_of(ws), pad_of(sd)
        self.period = period
        self.period_tol = period_tol
        self.strict = strict
        self.violations: List[Violation] = []
        self.words: List[I2sWord] = []
        self.rises: List[int] = []          # cycle of every rising SCK edge
        self.sd_delays: List[int] = []      # clocks from the last rise, per SD change
        self.ws_delays: List[int] = []      # the same for WS
        self.highs: List[int] = []
        self.lows: List[int] = []
        self._prev: Optional[Tuple[int, int, int]] = None
        self._fall: Optional[int] = None
        self._change: Optional[int] = None  # last SD or WS change since a rise
        self._ws_latched: Optional[int] = None
        self._bits: List[int] = []
        self._start = 0
        self._framed = False                # a WS change has been seen

    # ------------------------------------------------------------ checks
    def _bad(self, kind, cycle, clocks, low, high, detail=""):
        v = Violation(kind, cycle, clocks, low, high, detail)
        self.violations.append(v)
        if self.strict:
            raise AssertionError(str(v))

    def _t(self, measured: Optional[float] = None) -> Optional[float]:
        return self.period if self.period is not None else measured

    def observe(self, lines: Lines) -> None:
        c = lines.cycle
        s, w, d = lines.get(self.p_sck), lines.get(self.p_ws), lines.get(self.p_sd)
        prev = self._prev
        self._prev = (s, w, d)
        if prev is None:
            return
        ps, pw, pd = prev
        last = self.rises[-1] if self.rises else None
        if ps == 0 and s == 1:
            self._rise(c, last, pw, pd, w != pw or d != pd)
            return
        if ps == 1 and s == 0:
            self._fall = c
        for kind, now, before, store in (("sd", d, pd, self.sd_delays),
                                         ("ws", w, pw, self.ws_delays)):
            if now != before and last is not None:
                delay = c - last
                store.append(delay)
                self._change = c
                t = self._t()
                if t is not None and delay > DELAY_MAX * t:
                    self._bad(kind + " delay", c, delay, 1, DELAY_MAX * t,
                              "tdtr <= 0.8T from the rising edge")

    def _rise(self, c, last, ws_l, sd_l, changed_now):
        if changed_now:
            self._bad("hold", c, 0, 1, float("inf"),
                      "SD or WS changed in the cycle SCK rose (thtr >= 0)")
        if last is not None:
            per = c - last
            t = self._t(per)
            # The first period starts at the rise that set the idle level:
            # before it there was no clock, so it is not a gap.
            if (self.period is not None and len(self.rises) >= 2
                    and abs(per - self.period) > self.period_tol):
                self._bad("period", c, per, self.period - self.period_tol,
                          self.period + self.period_tol, "SCK not continuous")
            if self._fall is not None and last < self._fall < c:
                high, low = self._fall - last, c - self._fall
                self.highs.append(high)
                self.lows.append(low)
                if high < T_HIGH_MIN * t:
                    self._bad("high", c, high, T_HIGH_MIN * t, t, "tHC >= 0.35T")
                if low < T_LOW_MIN * t:
                    self._bad("low", c, low, T_LOW_MIN * t, t, "tLC >= 0.35T")
            if self._change is not None and last < self._change <= c:
                setup = c - self._change
                if setup < SETUP_MIN * t:
                    self._bad("setup", c, setup, SETUP_MIN * t, t, "tsr >= 0.2T")
        self._change = None
        self.rises.append(c)
        self._latch(c, ws_l, sd_l)

    # ----------------------------------------------------------- framing
    def _latch(self, c, ws_l, sd_l):
        if not self._bits:
            self._start = c
        self._bits.append(sd_l)
        if self._ws_latched is not None and ws_l != self._ws_latched:
            # 3.2: the bit latched with the new WS is the old word's LSB.
            if self._framed:
                raw = 0
                for b in self._bits:
                    raw = (raw << 1) | b
                self.words.append(I2sWord(self._ws_latched, len(self._bits),
                                          raw, self._start))
            self._framed = True
            self._bits = []
        self._ws_latched = ws_l

    @property
    def frames(self) -> List[Tuple[int, int]]:
        """``(left, right)`` 16-bit values: each left word with the right
        word that follows it."""
        out = []
        ws = self.words
        for a, b in zip(ws, ws[1:]):
            if a.channel == 0 and b.channel == 1:
                out.append((a.value(), b.value()))
        return out

    def report(self) -> str:
        return "%d words, %d rises, violations: %s" % (
            len(self.words), len(self.rises),
            "; ".join(str(v) for v in self.violations[:5]) or "none")


# ---------------------------------------------------------------------- PDM
class PdmMic(Model):
    """A PDM microphone with L/R tied to GND, clocked by ``clk``, driving
    ``dat`` (a ``ui`` pad).

    On every falling CLK edge it takes the next bit of a second-order
    sigma-delta modulation of ``dc + amplitude * sin(2 pi f t)``, ``t`` the
    bit index over ``bit_rate_hz``, and drives it from ``TL,EN`` after the
    edge until ``TL,DIS`` after the next rising edge (Table 7 and Table 5,
    rounded up to whole clocks). Outside that window the line would be high
    impedance; this model drives the complement of the bit there, the worst
    a second microphone or a floating line could do, so a receiver sampling
    in the wrong half reads garbage. Every period and duty cycle is checked
    against Table 3 and Table 5.
    """

    def __init__(self, clk: PinSpec, dat: PinSpec, *, clock_mhz: float = 50.0,
                 bit_rate_hz: float = 1.5625e6, tone_hz: float = 1000.0,
                 amplitude: float = 0.5, dc: float = 0.0,
                 strict: bool = False) -> None:
        self.p_clk, self.p_dat = pad_of(clk), pad_of(dat)
        self.clock_mhz = float(clock_mhz)
        self.bit_rate_hz = bit_rate_hz
        self.tone_hz, self.amplitude, self.dc = tone_hz, amplitude, dc
        self.strict = strict
        self.en = max(1, math.ceil(TL_EN_NS * clock_mhz / 1000.0))
        self.dis = max(1, math.ceil(TL_DIS_NS * clock_mhz / 1000.0))
        self.bits: List[int] = []           # every bit sent, in order
        self.inputs: List[float] = []       # the modulator input per bit
        self.violations: List[Violation] = []
        self.rises: List[int] = []
        self._v1 = self._v2 = 0.0
        self._y = -1.0
        self._prev: Optional[int] = None
        self._fall: Optional[int] = None
        self._valid_from = self._valid_to = None
        self._bit = 0

    def _next_bit(self) -> int:
        n = len(self.bits)
        x = self.dc + self.amplitude * math.sin(
            2 * math.pi * self.tone_hz * n / self.bit_rate_hz)
        self._v1 += x - self._y
        self._v2 += self._v1 - self._y
        self._y = 1.0 if self._v2 >= 0 else -1.0
        b = 1 if self._y > 0 else 0
        self.bits.append(b)
        self.inputs.append(x)
        return b

    def drive(self, drive: Drive, cycle: int) -> None:
        valid = (self._valid_from is not None and cycle >= self._valid_from
                 and (self._valid_to is None or cycle < self._valid_to))
        drive.set(self.p_dat, self._bit if valid else 1 - self._bit)

    def _bad(self, kind, cycle, clocks, low, high, detail=""):
        v = Violation(kind, cycle, clocks, low, high, detail)
        self.violations.append(v)
        if self.strict:
            raise AssertionError(str(v))

    def observe(self, lines: Lines) -> None:
        c = lines.cycle
        s = lines.get(self.p_clk)
        prev, self._prev = self._prev, s
        if prev is None or s == prev:
            return
        if prev == 1 and s == 0:                    # falling: the next bit
            self._fall = c
            self._bit = self._next_bit()
            self._valid_from, self._valid_to = c + self.en, None
            return
        self._valid_to = c + self.dis               # rising: released after TL,DIS
        if self.rises:
            per = c - self.rises[-1]
            f = self.clock_mhz / per
            if not PDM_F_MIN_MHZ <= f <= PDM_F_MAX_MHZ:
                self._bad("clock", c, per, self.clock_mhz / PDM_F_MAX_MHZ,
                          self.clock_mhz / PDM_F_MIN_MHZ, "1 to 3.25 MHz")
            if self._fall is not None and self.rises[-1] < self._fall < c:
                duty = (self._fall - self.rises[-1]) / per
                if not PDM_DUTY_MIN <= duty <= PDM_DUTY_MAX:
                    self._bad("duty", c, duty, PDM_DUTY_MIN, PDM_DUTY_MAX,
                              "40 to 60 %")
        self.rises.append(c)


def cic(bits: Sequence[int], r: int, order: int = 2, phase: int = 0) -> List[int]:
    """The CIC decimator's outputs over ``bits``, as its impulse response.

    ``h`` is a length-``r`` boxcar convolved with itself ``order`` times
    (length ``order * (r - 1) + 1``, sum ``r ** order``). Output ``m`` is
    ``sum_k h[k] * bits[m * r + phase - k]``; only outputs whose whole
    support lies in ``bits`` are returned, from the first such ``m``.
    """
    h = [1]
    for _ in range(order):
        g = [0] * (len(h) + r - 1)
        for i, a in enumerate(h):
            for j in range(r):
                g[i + j] += a
        h = g
    out = []
    m = 0
    while m * r + phase < len(bits):
        end = m * r + phase
        if end >= len(h) - 1:
            out.append(sum(h[k] * bits[end - k] for k in range(len(h))))
        m += 1
    return out


def fit_tone(samples: Sequence[float], freq: float, rate: float):
    """Least-squares fit of ``a sin + b cos + c`` at ``freq`` to ``samples``
    taken at ``rate``. Returns ``(amplitude, dc, rms residual)``."""
    n = len(samples)
    s = [math.sin(2 * math.pi * freq * k / rate) for k in range(n)]
    co = [math.cos(2 * math.pi * freq * k / rate) for k in range(n)]
    one = [1.0] * n
    cols = [s, co, one]
    ata = [[sum(x * y for x, y in zip(p, q)) for q in cols] for p in cols]
    atb = [sum(x * y for x, y in zip(p, samples)) for p in cols]
    sol = _solve3(ata, atb)
    fit = [sol[0] * a + sol[1] * b + sol[2] for a, b in zip(s, co)]
    rms = math.sqrt(sum((x - y) ** 2 for x, y in zip(samples, fit)) / n)
    return math.hypot(sol[0], sol[1]), sol[2], rms


def _solve3(a, b):
    m = [row[:] + [v] for row, v in zip(a, b)]
    for i in range(3):
        p = max(range(i, 3), key=lambda k: abs(m[k][i]))
        m[i], m[p] = m[p], m[i]
        for k in range(3):
            if k != i:
                f = m[k][i] / m[i][i]
                m[k] = [x - f * y for x, y in zip(m[k], m[i])]
    return [m[i][3] / m[i][i] for i in range(3)]
