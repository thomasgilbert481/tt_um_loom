"""WS2812B reference model: a decoder that watches the data line of a strip.

:class:`Ws2812Rx` is the pad-level counterpart of
:class:`~tools.protomodels.uart.UartRx`: it observes one pad every cycle,
measures every high and every low pulse in core clocks and checks each one
against the WS2812B datasheet window before it decodes anything. A pulse
outside its window is recorded in :attr:`Ws2812Rx.violations` (and raises at
once with ``strict=True``), so a test asserts on the timing first and on the
colours second.

The datasheet, at 50 MHz (one clock is 20 ns), with ``clock_mhz`` scaling
every number:

=========  ===========  ============  ==============
pulse      nominal      tolerance     clocks at 50 MHz
=========  ===========  ============  ==============
T0H        0.40 us      +-0.15 us     20 (13 .. 27)
T1H        0.80 us      +-0.15 us     40 (33 .. 47)
T0L        0.85 us      +-0.15 us     42.5 (35 .. 50)
T1L        0.45 us      +-0.15 us     22.5 (15 .. 30)
period     1.25 us      +-0.60 us     62.5 (33 .. 92)
reset      > 50 us      -             > 2500
=========  ===========  ============  ==============

A bit is a high pulse followed by a low one. The high pulse decides the bit
value: it must be inside the T0H window or inside the T1H window and the two
do not overlap, so the value is never a guess. The low pulse that follows is
then checked against the window of *that* bit value, and the period (this
rise to the next rise) against the period window.

The last low pulse of a frame is different: nothing follows it, and the
strip latches when the line has been low for more than the reset time. So a
low pulse is judged as soon as it ends *or* as soon as it is longer than
every bit low window allows:

* longer than :attr:`reset_clocks`: the frame ends and the low is a reset;
* longer than the bit window but shorter than a reset: a violation, because
  the strip would neither take it as a bit nor latch cleanly. That is what
  makes "the firmware must not start the next frame before the reset time"
  a testable promise.

Bits are grouped into bytes MSB first and bytes into LEDs of three (G, R, B).
A frame whose bit count is not a multiple of 8, or whose byte count is not a
multiple of 3, is recorded as a violation and still delivered, so a test can
see what did come out.
"""

from __future__ import annotations

import dataclasses
from typing import List, Optional, Tuple

from .bench import Lines, Model, PinSpec, pad_of

#: Datasheet times in microseconds: nominal high and low of a 0 and a 1 bit.
T0H_US, T0L_US = 0.40, 0.85
T1H_US, T1L_US = 0.80, 0.45
PULSE_TOL_US = 0.15
PERIOD_US, PERIOD_TOL_US = 1.25, 0.60
RESET_US = 50.0


@dataclasses.dataclass
class Ws2812Frame:
    """One reset-delimited byte stream seen on the line."""

    data: bytes                     # the bytes, in the order they were sent
    start: int                      # cycle of the first rising edge
    end: int                        # cycle of the last falling edge
    bits: int                       # bits seen (data may be truncated)
    reset_clocks: Optional[int]     # the low that ended it, None if still low

    @property
    def leds(self) -> List[Tuple[int, int, int]]:
        """``(green, red, blue)`` per LED, the WS2812B wire order."""
        return [tuple(self.data[i:i + 3]) for i in range(0, len(self.data) - 2, 3)]

    @property
    def rgb(self) -> List[Tuple[int, int, int]]:
        """``(red, green, blue)`` per LED, for tests that think in RGB."""
        return [(r, g, b) for g, r, b in self.leds]


@dataclasses.dataclass
class Violation:
    """One pulse (or frame) the datasheet does not allow."""

    kind: str                       # "high" | "low" | "period" | "frame"
    cycle: int                      # where the offending pulse ended
    clocks: float                   # what was measured
    low: float                      # the window, inclusive
    high: float
    detail: str = ""

    def __str__(self) -> str:                               # pragma: no cover
        return ("%s pulse of %g clocks at cycle %d, outside %g .. %g%s"
                % (self.kind, self.clocks, self.cycle, self.low, self.high,
                   ": " + self.detail if self.detail else ""))


class Ws2812Rx(Model):
    """Decodes a WS2812B line on ``pin``; every pulse is checked as it ends.

    Args:
        pin: the pad to watch (``"OUT0"``, a pin index or a raw pad).
        clock_mhz: core clocks per microsecond (50 for this chip).
        tol_us: the high and low pulse tolerance (datasheet +-0.15 us).
        strict: raise :class:`AssertionError` on the first violation instead
            of only recording it.
    """

    def __init__(self, pin: PinSpec, *, clock_mhz: float = 50.0,
                 tol_us: float = PULSE_TOL_US,
                 period_tol_us: float = PERIOD_TOL_US,
                 reset_us: float = RESET_US, strict: bool = False) -> None:
        self.pad = pad_of(pin)
        self.clock_mhz = float(clock_mhz)
        self.strict = strict
        us = self.clock_mhz                                  # clocks per us
        tol = tol_us * us
        self.high_window = ((T0H_US * us - tol, T0H_US * us + tol),
                            (T1H_US * us - tol, T1H_US * us + tol))
        self.low_window = ((T0L_US * us - tol, T0L_US * us + tol),
                           (T1L_US * us - tol, T1L_US * us + tol))
        self.period_window = (PERIOD_US * us - period_tol_us * us,
                              PERIOD_US * us + period_tol_us * us)
        self.reset_clocks = reset_us * us
        #: the longest a low pulse may be and still be a bit's low phase
        self.low_limit = max(self.low_window[0][1], self.low_window[1][1])

        self.frames: List[Ws2812Frame] = []
        self.violations: List[Violation] = []
        self.bits: List[int] = []                # of the frame being decoded
        self.high_clocks: List[int] = []         # every high pulse, in clocks
        self.low_clocks: List[int] = []          # every low pulse inside a frame
        self.periods: List[int] = []             # rise to rise, inside a frame
        self.resets: List[int] = []              # every reset low, in clocks

        self.level = 0
        self.since = 0                           # cycle the level was entered
        self._start: Optional[int] = None        # first rise of the frame
        self._rise: Optional[int] = None         # the last rise
        self._last_fall: Optional[int] = None
        self._pending: Optional[int] = None      # bit whose low is not judged
        self._closed = True                      # the line has latched

    # ------------------------------------------------------------- helpers
    def _fail(self, kind: str, cycle: int, clocks: float,
              window: Tuple[float, float], detail: str = "") -> None:
        bad = Violation(kind, cycle, clocks, window[0], window[1], detail)
        self.violations.append(bad)
        if self.strict:
            raise AssertionError("ws2812: %s" % bad)

    def _close_frame(self, end: int, reset: Optional[int]) -> None:
        bits = self.bits
        data = bytes(sum(bit << (7 - i) for i, bit in enumerate(bits[k:k + 8]))
                     for k in range(0, len(bits) - 7, 8))
        frame = Ws2812Frame(data=data, start=self._start, end=end,
                            bits=len(bits), reset_clocks=reset)
        if len(bits) % 8:
            self._fail("frame", end, len(bits), (0, 0),
                       "%d bits is not a whole number of bytes" % len(bits))
        elif len(data) % 3:
            self._fail("frame", end, len(data), (0, 0),
                       "%d bytes is not a whole number of LEDs" % len(data))
        self.frames.append(frame)
        self.bits = []
        self._start = self._rise = self._pending = None
        self._closed = True

    def _judge_low(self, clocks: int, cycle: int) -> None:
        """A low pulse inside a frame ended after ``clocks`` clocks."""
        self.low_clocks.append(clocks)
        window = self.low_window[self._pending]
        if not window[0] <= clocks <= window[1]:
            self._fail("low", cycle, clocks, window,
                       "after a %d bit" % self._pending)
        self._pending = None

    # -------------------------------------------------------------- Model
    def observe(self, lines: Lines) -> None:
        v = lines.get(self.pad)
        cycle = lines.cycle
        if v == self.level:
            # A low that has outstayed every bit window ends the frame: the
            # strip latches once it is longer than the reset time.
            if (v == 0 and not self._closed
                    and cycle - self.since >= self.reset_clocks):
                self.resets.append(cycle - self.since)
                if self._pending is not None:
                    self._pending = None          # the frame's last low
                self._close_frame(self.since, cycle - self.since)
            return
        clocks = cycle - self.since
        if v == 1:                                           # rising edge
            if not self._closed and clocks > self.low_limit:
                # Too long for a bit, too short for a reset: the strip would
                # neither shift it in nor latch. Report and end the frame.
                self._fail("low", cycle, clocks,
                           (self.low_window[1][0], self.low_limit),
                           "a gap shorter than the %g-clock reset"
                           % self.reset_clocks)
                self._close_frame(self.since, None)
            elif not self._closed:
                self._judge_low(clocks, cycle)
                period = cycle - self._rise
                self.periods.append(period)
                if not self.period_window[0] <= period <= self.period_window[1]:
                    self._fail("period", cycle, period, self.period_window)
            if self._closed:
                self._closed = False
                self._start = cycle
            self._rise = cycle
        else:                                                # falling edge
            self.high_clocks.append(clocks)
            for bit, window in enumerate(self.high_window):
                if window[0] <= clocks <= window[1]:
                    self.bits.append(bit)
                    self._pending = bit
                    break
            else:
                self._fail("high", cycle, clocks,
                           (self.high_window[0][0], self.high_window[1][1]),
                           "neither a 0 (%g..%g) nor a 1 (%g..%g)"
                           % (self.high_window[0] + self.high_window[1]))
                self.bits.append(0)
                self._pending = 0
            self._last_fall = cycle
        self.level = v
        self.since = cycle

    # ---------------------------------------------------------------- view
    @property
    def data(self) -> List[bytes]:
        """The bytes of every completed frame."""
        return [f.data for f in self.frames]

    @property
    def leds(self) -> List[Tuple[int, int, int]]:
        """The LEDs of the last completed frame."""
        return self.frames[-1].leds if self.frames else []

    def report(self) -> str:
        """Every violation, one per line (empty when there is none)."""
        return "\n".join(str(v) for v in self.violations)
