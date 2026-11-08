"""A STEP/DIR stepper driver input, the reference for ``firmware/stepper_step_dir.loom``.

Written from the Allegro A4988 datasheet (A4988-DS, Rev. 8):

* Figure 1, "Logic Interface Timing Diagram", and its table (page 6):
  STEP minimum HIGH pulse width tA = 1 us, STEP minimum LOW pulse width
  tB = 1 us, setup time from an input change (DIR, MSx, RESET) to the STEP
  rising edge tC = 200 ns, hold time from the STEP rising edge to an input
  change tD = 200 ns. The table heads its column "Typ."; this model treats
  the four figures as the limits a controller must keep, as the figure's
  names (minimum, setup, hold) say.
* "Direction Input (DIR)" (page 9): a change of DIR takes effect at the
  next STEP rising edge; one pulse on STEP moves the motor one (micro)step
  (page 1, "Simply inputting one pulse on the STEP input drives the motor
  one microstep").

Which way DIR high turns the shaft depends on the wiring of the motor; the
datasheet gives no sign. This model counts a rising STEP edge with DIR high
as +1 and with DIR low as -1, the convention the firmware's header states.

:class:`StepDirDriver` sits on the two pins, counts the position, keeps
every pulse and lists every violation of tA, tB, tC and tD at the board
clock it is given, so a test can run at 50 MHz or at a slower clock and
check the same windows.
"""

from __future__ import annotations

from typing import List, Optional, Tuple

from tools.protomodels.bench import Lines, Model, PinSpec, pad_of

T_A = 1e-6          # STEP high, minimum (Figure 1, page 6)
T_B = 1e-6          # STEP low, minimum
T_C = 200e-9        # DIR change to STEP rising, minimum (setup)
T_D = 200e-9        # STEP rising to DIR change, minimum (hold)


class StepDirDriver(Model):
    """The driver's STEP and DIR inputs: position, pulses and violations."""

    def __init__(self, step: PinSpec, dir: PinSpec, *, clock_hz: float = 50e6) -> None:
        self.step_pad = pad_of(step)
        self.dir_pad = pad_of(dir)
        self.clock_hz = float(clock_hz)
        self.position = 0
        self.pulses: List[Tuple[int, int, int]] = []    # (rise, fall, dir)
        self.dir_changes: List[int] = []
        self.violations: List[str] = []
        self._step: Optional[int] = None
        self._dir: Optional[int] = None
        self._rise: Optional[int] = None
        self._fall: Optional[int] = None
        self._last_rise: Optional[int] = None

    def _secs(self, clocks: int) -> float:
        return clocks / self.clock_hz

    def observe(self, lines: Lines) -> None:
        c = lines.cycle
        s, d = lines.get(self.step_pad), lines.get(self.dir_pad)
        if self._dir is not None and d != self._dir:
            self.dir_changes.append(c)
            if self._last_rise is not None and self._secs(c - self._last_rise) < T_D:
                self.violations.append("cycle %d: DIR changed %d clocks after a "
                                       "STEP rise (tD)" % (c, c - self._last_rise))
        if self._step is not None and s != self._step:
            if s == 1:
                if self._fall is not None and self._secs(c - self._fall) < T_B:
                    self.violations.append("cycle %d: STEP low %d clocks (tB)"
                                           % (c, c - self._fall))
                last_dir = self.dir_changes[-1] if self.dir_changes else None
                if last_dir is not None and self._secs(c - last_dir) < T_C:
                    self.violations.append("cycle %d: DIR changed %d clocks before "
                                           "the STEP rise (tC)" % (c, c - last_dir))
                self._rise = self._last_rise = c
                self.position += 1 if d else -1
            else:
                if self._rise is not None:
                    if self._secs(c - self._rise) < T_A:
                        self.violations.append("cycle %d: STEP high %d clocks (tA)"
                                               % (c, c - self._rise))
                    self.pulses.append((self._rise, c, d))
                self._fall = c
        self._step, self._dir = s, d

    def highs(self, since: int = 0) -> List[int]:
        """The high time of every pulse that rose at or after ``since``, in clocks."""
        return [f - r for r, f, _ in self.pulses if r >= since]

    def lows(self, since: int = 0) -> List[int]:
        """The low time between consecutive pulses after ``since``, in clocks."""
        ps = [p for p in self.pulses if p[0] >= since]
        return [b[0] - a[1] for a, b in zip(ps, ps[1:])]
