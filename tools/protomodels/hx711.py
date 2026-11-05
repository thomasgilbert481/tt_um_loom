"""HX711 24-bit load-cell ADC: a device model, a host model and a timing checker.

Written from the Avia Semiconductor datasheet, "HX711 24-Bit
Analog-to-Digital Converter (ADC) for Weigh Scales" (the copy used carries
no revision date), never from ``firmware/hx711_*.loom``:

* "Serial Interface": DOUT is high while no data is ready and PD_SCK should
  then be low; DOUT low means data is ready. 25 to 27 positive PD_SCK
  pulses shift the data out, one bit a pulse, MSB first; the 25th pulse
  pulls DOUT back high (Fig. 2). Fewer than 25 or more than 27 pulses in
  one conversion period are a "serial communication error".
* Table 3, "Input Channel and Gain Selection": 25 pulses select channel A
  gain 128 for the next conversion, 26 channel B gain 32, 27 channel A gain
  64. Partly unchecked: the table's cells did not survive text extraction;
  the description (channel A gain 128 or 64, channel B 32) agrees, Fig. 2
  labels the 27-pulse case "CH.B, Gain:64", which is taken as a typo.
* The timing table under Fig. 2: T1 DOUT falling edge to PD_SCK rising
  edge, min 0.1 us; T2 PD_SCK rising edge to DOUT data ready, max 0.1 us;
  T3 PD_SCK high time, min 0.2, typ 1, max 50 us; T4 PD_SCK low time, min
  0.2, typ 1 us.
* "Reset and Power-Down", Fig. 3: PD_SCK high for longer than 60 us powers
  the chip down; when it returns low the chip resets and enters normal
  operation, and the input is channel A gain 128.
* Table 2 and "Output data coding": 24-bit two's complement, 800000h (min)
  to 7FFFFFh (max), saturating; output data rate 10 or 80 samples/s.

The conversion time is a parameter of :class:`Hx711Device` (``conv``, in
clocks from the falling edge of the read's 25th pulse, or from the start or
a wake, to DOUT low), not the chip's 100 or 12.5 ms: the RTL budget holds
milliseconds of nothing. The datasheet does not say what a high between 50
and 60 us does; the device model powers down only above ``pd`` (60 us).

Bench conventions (``tools/protomodels/bench.py``): a model drives from the
state registered at the previous edge, so its outputs follow what it sees
one clock later at the earliest. Times are kept in clocks; the windows are
converted with the bench's ``clock_hz``.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Sequence, Tuple

from tools.protomodels.bench import Drive, Lines, Model, PinSpec, pad_of

#: Pulses in all -> (channel, gain) of the next conversion (Table 3).
PULSES = {25: ("A", 128), 26: ("B", 32), 27: ("A", 64)}
#: The input codes the firmware uses: index = pulses - 25.
INPUTS = [("A", 128), ("B", 32), ("A", 64)]

T1_MIN = 0.1e-6         # DOUT fall to PD_SCK rise
T2_MAX = 0.1e-6         # PD_SCK rise to DOUT ready
T3_MIN, T3_TYP, T3_MAX = 0.2e-6, 1e-6, 50e-6     # PD_SCK high
T4_MIN, T4_TYP = 0.2e-6, 1e-6                     # PD_SCK low
PD_TIME = 60e-6         # PD_SCK high longer than this: power down

MIN_CODE, MAX_CODE = -0x800000, 0x7FFFFF


def to_code(value: int) -> int:
    """A signed reading, saturated as the chip does, as its 24-bit code."""
    return max(MIN_CODE, min(MAX_CODE, value)) & 0xFFFFFF


def from_code(code: int) -> int:
    """A 24-bit two's-complement code as a signed number."""
    code &= 0xFFFFFF
    return code - 0x1000000 if code & 0x800000 else code


def words32(value: int) -> Tuple[int, int]:
    """A signed reading as the two 16-bit words of a 32-bit integer."""
    v = from_code(to_code(value)) & 0xFFFFFFFF
    return v >> 16, v & 0xFFFF


def clocks(seconds: float, clock_hz: float) -> float:
    return seconds * clock_hz


# ----------------------------------------------------------------- timing
def pulses_of(edges: Sequence[Tuple[int, int]]) -> List[Tuple[int, int]]:
    """``[(rise, fall)]`` cycles of the high pulses in a PD_SCK edge list
    ``[(cycle, level)]``; a pulse still high at the end has fall ``-1``."""
    out: List[Tuple[int, int]] = []
    rise = None
    for cycle, level in edges:
        if level and rise is None:
            rise = cycle
        elif not level and rise is not None:
            out.append((rise, cycle))
            rise = None
    if rise is not None:
        out.append((rise, -1))
    return out


def check_pd_sck(edges: Sequence[Tuple[int, int]], clock_hz: float, *,
                 allow_power_down: bool = False) -> List[str]:
    """Every PD_SCK high and low time of ``edges`` against T3 and T4.

    Returns the violations as text (empty when every phase is inside its
    window). A high longer than T3's 50 us is a violation unless
    ``allow_power_down`` and it is longer than the 60 us of Fig. 3, the
    deliberate power-down. Lows between reads are not bounded above."""
    bad = []
    hi_min, hi_max = clocks(T3_MIN, clock_hz), clocks(T3_MAX, clock_hz)
    lo_min, pd = clocks(T4_MIN, clock_hz), clocks(PD_TIME, clock_hz)
    pulses = pulses_of(edges)
    for i, (rise, fall) in enumerate(pulses):
        if fall < 0:
            continue
        high = fall - rise
        if high < hi_min:
            bad.append("pulse at %d: high %d clocks < T3 min %.1f" % (rise, high, hi_min))
        if high > hi_max and not (allow_power_down and high > pd):
            bad.append("pulse at %d: high %d clocks > T3 max %.1f" % (rise, high, hi_max))
        if i + 1 < len(pulses):
            low = pulses[i + 1][0] - fall
            if low < lo_min:
                bad.append("low at %d: %d clocks < T4 min %.1f" % (fall, low, lo_min))
    return bad


class EdgeLog(Model):
    """Records every change of one pad: ``edges = [(cycle, level)]``."""

    def __init__(self, pin: PinSpec, initial: int = 0) -> None:
        self.pad = pad_of(pin)
        self.level = initial
        self.edges: List[Tuple[int, int]] = []

    def observe(self, lines: Lines) -> None:
        v = lines.get(self.pad)
        if v != self.level:
            self.level = v
            self.edges.append((lines.cycle, v))


# ----------------------------------------------------------------- device
class Hx711Device(Model):
    """The chip, clock by clock, on the bench.

    Args:
        sck: the pad PD_SCK arrives on (a chip output, ``OUT0`` for
            ``hx711_host.loom``); dout: the pad it drives (``IN0``).
        clock_hz: the bench clock, for the 60 us power-down.
        conv: the conversion time in clocks (module docstring).
        readings: ``{input: signed value}`` for inputs 0 (A/128), 1 (B/32),
            2 (A/64); saturated to 24 bits when served.
        t2: clocks from a PD_SCK rise (seen) to DOUT changing, at least 1;
            5 is the datasheet's 0.1 us at 50 MHz.
        never_ready: error injection, DOUT never goes low.

    Records: ``served`` ``[(cycle, input, value)]`` per DOUT low;
    ``periods`` ``[(pulses, next input, error)]`` per conversion that
    ended after pulses; ``power_downs`` and ``wakes`` (cycles); ``t1``
    ``[clocks]`` from each DOUT low to the first rise after it.
    """

    def __init__(self, sck: PinSpec = "OUT0", dout: PinSpec = "IN0", *,
                 clock_hz: float = 50e6, conv: int = 2000,
                 readings: Optional[Dict[int, int]] = None, t2: int = 1,
                 never_ready: bool = False) -> None:
        self.sck, self.dout = pad_of(sck), pad_of(dout)
        self.clock_hz = clock_hz
        self.conv = conv
        self.readings = dict(readings or {})
        self.t2 = max(1, t2)
        self.never_ready = never_ready
        self.pd = clocks(PD_TIME, clock_hz)
        self.level = 1                  # DOUT: busy
        self.pending: List[Tuple[int, int]] = []
        self.sck_level = 0
        self.rise = -1
        self.state = "conv"
        self.conv_end = conv
        self.input = 0                  # of the conversion in progress
        self.count = 0
        self.code = 0
        self.ready_at = -1
        self.served: List[Tuple[int, int, int]] = []
        self.periods: List[Tuple[int, int, bool]] = []
        self.power_downs: List[int] = []
        self.wakes: List[int] = []
        self.t1: List[int] = []

    def drive(self, drive: Drive, cycle: int) -> None:
        drive.set(self.dout, self.level)

    def _at(self, cycle: int, level: int) -> None:
        self.pending.append((cycle, level))

    def observe(self, lines: Lines) -> None:
        c = lines.cycle
        s = lines.get(self.sck)
        if s and not self.sck_level:
            self._rise(c)
        elif not s and self.sck_level:
            self._fall(c)
        self.sck_level = s
        if s and self.state != "down" and c - self.rise > self.pd:
            self.state = "down"             # Fig. 3
            self.pending = []
            self.level = 1
            self.power_downs.append(c)
        if self.state == "conv" and c >= self.conv_end and not self.never_ready:
            self._ready(c)
        if self.pending:
            keep = []
            for at, level in self.pending:
                if at <= c + 1:             # drives from the next cycle
                    self.level = level
                else:
                    keep.append((at, level))
            self.pending = keep

    def _rise(self, c: int) -> None:
        self.rise = c
        if self.state == "down":
            return
        self.count += 1
        if self.state == "ready" and self.count == 1 and self.ready_at >= 0:
            self.t1.append(c - self.ready_at)
        if self.state == "ready":
            n = self.count
            if n <= 24:
                self._at(c + self.t2, (self.code >> (24 - n)) & 1)
            if n == 25:
                self._at(c + self.t2, 1)    # the 25th pulse: DOUT high
                self.state = "tail"

    def _fall(self, c: int) -> None:
        if self.state == "down":
            self.state = "conv"             # reset: channel A, gain 128
            self.input, self.count = 0, 0
            self.conv_end = c + self.conv
            self.wakes.append(c)
        elif self.state == "tail" and self.count == 25:
            self.state = "conv"             # the next conversion runs
            self.conv_end = c + self.conv

    def _ready(self, c: int) -> None:
        if self.count:
            nxt = self.count - 25 if self.count in PULSES else 0
            error = self.count not in PULSES
            self.periods.append((self.count, nxt, error))
            self.input = nxt
        self.count = 0
        self.code = to_code(self.readings.get(self.input, 0))
        self.served.append((c, self.input, from_code(self.code)))
        self.state = "ready"
        self.ready_at = c + 1
        self._at(c + 1, 0)


# ------------------------------------------------------------------- host
class Hx711Host(Model):
    """A host that reads an HX711 the way the datasheet describes.

    It waits for DOUT low, waits ``t1`` clocks, then gives the read's
    pulses (25, 26 or 27; ``plan`` lists them per read, and the reads stop
    at its end), each ``high`` clocks high and ``low`` clocks low, and
    samples DOUT ``sample`` clocks after each of the first 24 rises
    (default: the last clock of the high time). After a read it waits for
    DOUT high before it looks for the next low.

    Error injection: ``long_high = (pulse, clocks)`` holds that pulse
    (1-based) high for ``clocks`` in the first read, the mid-read power-down
    of Fig. 3; the read is then abandoned. :meth:`hold_high` raises PD_SCK
    for a stretch at a chosen cycle, between reads.

    Results: ``reads`` ``[(cycle of the first rise, signed value,
    pulses)]``; ``edges`` the PD_SCK edges it made, for
    :func:`check_pd_sck`.
    """

    def __init__(self, sck: PinSpec = "IN1", dout: PinSpec = "OUT1", *,
                 high: int = 50, low: int = 50, sample: Optional[int] = None,
                 t1: int = 10, plan: Sequence[int] = (25,),
                 long_high: Optional[Tuple[int, int]] = None) -> None:
        self.sck, self.dout = pad_of(sck), pad_of(dout)
        self.high, self.low, self.t1 = high, low, t1
        self.sample = high - 1 if sample is None else sample
        self.plan = list(plan)
        self.long_high = long_high
        self.state = "wait"
        self.level = 0
        self.highs: List[Tuple[int, int]] = []   # [start, end) of each high
        self.samples: List[int] = []             # cycles to sample at
        self.bits: List[int] = []
        self.start = -1
        self.end = -1
        self.reads: List[Tuple[int, int, int]] = []
        self.abandoned: List[int] = []
        self.edges: List[Tuple[int, int]] = []

    def hold_high(self, start: int, clocks_high: int) -> None:
        """PD_SCK high over ``[start, start + clocks_high)``, outside a read."""
        self.highs.append((start, start + clocks_high))

    def drive(self, drive: Drive, cycle: int) -> None:
        level = 1 if any(a <= cycle < b for a, b in self.highs) else 0
        if level != self.level:
            self.level = level
            self.edges.append((cycle, level))
        drive.set(self.sck, level)

    def _schedule(self, c: int) -> None:
        pulses = self.plan.pop(0)
        period = self.high + self.low
        first = c + self.t1
        self.start = first
        self.highs = [h for h in self.highs if h[1] > c]
        self.samples, self.bits = [], []
        end = first
        for k in range(pulses):
            rise = end if k else first
            high = self.high
            if self.long_high and k + 1 == self.long_high[0]:
                high = self.long_high[1]
            self.highs.append((rise, rise + high))
            if k < 24:
                self.samples.append(rise + self.sample)
            end = rise + high + self.low if k + 1 < pulses else rise + high
            if high != self.high:           # the long high ends the read
                end = rise + high
                break
        self.end = end
        self.pulses = pulses
        self.state = "read"

    def observe(self, lines: Lines) -> None:
        c = lines.cycle
        d = lines.get(self.dout)
        if self.state == "wait":
            if not d and self.plan:
                self._schedule(c)
        elif self.state == "read":
            if self.samples and c == self.samples[0]:
                self.samples.pop(0)
                self.bits.append(d)
            if c >= self.end:
                if len(self.bits) == 24 and not self.long_high:
                    code = 0
                    for b in self.bits:
                        code = (code << 1) | b
                    self.reads.append((self.start, from_code(code), self.pulses))
                else:
                    self.abandoned.append(self.start)
                self.long_high = None
                self.state = "after"
        elif self.state == "after" and d:
            self.state = "wait"
