"""Six-step BLDC commutation from Hall sensors: a motor model and a gate checker.

Written from Microchip AN857, "Brushless DC Motor Control Made Easy"
(DS00857B), read 2026-11-07, never from ``firmware/bldc_6step.loom``:

* Figure 4 ("Commutation code segment"): the drive words ``FwdTable`` and
  ``RevTable`` indexed by the sensor code (sensor A the least significant
  bit); codes 0 and 7 are invalid and drive nothing. Drive-word bits 5..0
  are C high, C low, B high, B low, A high, A low (Table 1, RC5..RC0).
  :data:`FWD` and :data:`REV` are those words. Table 3's rule, "swapping all
  the high and low drives of Table 2", is checked by :func:`swap_high_low`
  (the tests assert ``REV[c] == swap_high_low(FWD[c])``).
* Figure 2 / Tables 1 and 2: driving by ``FwdTable`` turns the motor
  clockwise through the phases 1..6, whose sensor codes are 5, 4, 6, 2, 3, 1
  (Figure 4's comments: phase 1 is code 5, phase 2 code 4, ...):
  :data:`CW_SEQUENCE`. Each step changes exactly one sensor.
* Page 3, dead-time control: a phase's low-side driver may only turn on
  once its high-side driver has had time to turn off, and the reverse; both
  must never be on at once. :class:`GateChecker` checks both against a
  dead time in clocks that the test gives (AN857 gives no figure; the
  program's default of 1 us is from memory, unchecked).
* Pages 6-7: the high-side driver is modulated and the low-side driver
  left on; ANDing the drive word with the low-side bits turns the active
  high side off (:data:`LOWS`).

The speed model of :class:`Motor` is not from AN857 (which gives only
``RPM = KV x Volts``): a first-order rotor whose acceleration is
``accel`` times the torque sign minus ``drag`` times the speed, with a
torque of +1 while the gates show exactly the forward drive word of the
sector the rotor is in, -1 for the reverse word, and 0 otherwise (the
high side off between PWM pulses, all off, a stale word). It is a
plausible plant, not a physical one: it makes the sector advance according
to the drive applied, which is all the tests need.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple

from tools.protomodels.bench import Drive, Lines, Model, pad_of

#: AN857 Figure 4, FwdTable (clockwise) and RevTable (counter-clockwise).
FWD = (0b000000, 0b010010, 0b001001, 0b011000,
       0b100100, 0b000110, 0b100001, 0b000000)
REV = (0b000000, 0b100001, 0b000110, 0b100100,
       0b011000, 0b001001, 0b010010, 0b000000)
#: Sensor codes in clockwise order, phases 1..6 (AN857 Figure 4 comments).
CW_SEQUENCE = (5, 4, 6, 2, 3, 1)
INVALID = (0, 7)
LOWS = 0b010101                     # AL, BL, CL
HIGHS = 0b101010                    # AH, BH, CH
GATE_NAMES = ("AL", "AH", "BL", "BH", "CL", "CH")   # drive-word bits 0..5
PHASES = ((1, 0), (3, 2), (5, 4))   # (high bit, low bit) of A, B, C


def swap_high_low(word: int) -> int:
    """Table 3's construction: every high drive becomes a low one and back."""
    return ((word & HIGHS) >> 1) | ((word & LOWS) << 1)


def drive_word(code: int, reverse: bool) -> int:
    """The drive word for a sensor code: AN857 FwdTable or RevTable."""
    return (REV if reverse else FWD)[code & 7]


def position(code: int) -> Optional[int]:
    """Clockwise position 0..5 of a valid code, None for 000 and 111."""
    return CW_SEQUENCE.index(code) if code in CW_SEQUENCE else None


def step(code: int, sectors: int) -> int:
    """The code ``sectors`` steps clockwise (negative: counter-clockwise)."""
    return CW_SEQUENCE[(CW_SEQUENCE.index(code) + sectors) % 6]


class HallSource(Model):
    """Scripted Hall sensors on IN0..IN2 (A, B, C): ``schedule`` takes
    ``[(cycle, code)]``; the code is driven from that cycle on."""

    def __init__(self, code: int = 5, pins=("IN0", "IN1", "IN2")) -> None:
        self.pads = [pad_of(p) for p in pins]
        self.code = code
        self.events: List[Tuple[int, int]] = []
        self.changes: List[Tuple[int, int]] = []    # (cycle, code) applied

    def schedule(self, events) -> None:
        self.events = sorted(self.events + list(events))

    def drive(self, drive: Drive, cycle: int) -> None:
        while self.events and self.events[0][0] <= cycle:
            _, code = self.events.pop(0)
            if code != self.code:
                self.code = code
                self.changes.append((cycle, code))
        for bit, pad in enumerate(self.pads):
            drive.set(pad, (self.code >> bit) & 1)


class Motor(Model):
    """A rotor whose sector advances according to the drive applied.

    ``theta`` is the electrical position in sectors (one sector is 60
    electrical degrees), ``omega`` its speed in sectors per clock. Every
    clock, after seeing the gates (``uo[5:0]``):

        torque = +1 if gates == FWD[code], -1 if gates == REV[code], else 0
        omega += accel * torque - drag * omega
        theta += omega

    and the Hall code driven on IN0..IN2 is ``CW_SEQUENCE[floor(theta) % 6]``
    (clockwise is increasing theta). The terminal speed at a duty fraction
    ``d`` is about ``accel * d / drag`` sectors per clock.
    """

    def __init__(self, *, sector: int = 0, theta: Optional[float] = None,
                 omega: float = 0.0, accel: float = 2e-8, drag: float = 5e-5,
                 pins=("IN0", "IN1", "IN2")) -> None:
        self.pads = [pad_of(p) for p in pins]
        self.theta = sector + 0.5 if theta is None else theta
        self.omega = omega
        self.accel, self.drag = accel, drag
        self.code = self._code()
        self.changes: List[Tuple[int, int]] = []    # (cycle, new code)
        self.torque_clocks = {1: 0, -1: 0, 0: 0}

    def _code(self) -> int:
        return CW_SEQUENCE[int(self.theta // 1) % 6]

    def drive(self, drive: Drive, cycle: int) -> None:
        for bit, pad in enumerate(self.pads):
            drive.set(pad, (self.code >> bit) & 1)

    def observe(self, lines: Lines) -> None:
        gates = lines.uo & 0x3F
        torque = 1 if gates == FWD[self.code] else (-1 if gates == REV[self.code] else 0)
        self.torque_clocks[torque] += 1
        self.omega += self.accel * torque - self.drag * self.omega
        self.theta += self.omega
        code = self._code()
        if code != self.code:
            self.code = code
            self.changes.append((lines.cycle + 1, code))

    def intervals(self) -> List[int]:
        """Clocks between consecutive Hall changes."""
        return [b[0] - a[0] for a, b in zip(self.changes, self.changes[1:])]


class GateChecker(Model):
    """Watches the six gates (``uo[5:0]``, AN857 drive-word order) and the
    Hall lines (``ui[2:0]``) every clock and records every rule broken:

    * ``shoot``: both switches of one phase on in the same clock;
    * ``dead``: a switch turning on less than ``dead`` clocks after the
      other switch of its phase turned off (AN857 page 3);
    * ``pair``: a switch on that the drive word of the Hall code on the
      lines does not have (FwdTable, or RevTable while ``reverse``), more
      than ``latency`` clocks after the last Hall or direction change; an
      invalid code allows nothing, so this is also the all-off check.

    ``edges[bit]`` lists ``(cycle, level)`` for every change of gate ``bit``.
    """

    def __init__(self, *, dead: int, latency: int, reverse: bool = False) -> None:
        self.dead, self.latency, self.reverse = dead, latency, reverse
        self.edges: Dict[int, List[Tuple[int, int]]] = {b: [] for b in range(6)}
        self.violations: List[Tuple[int, str, int]] = []
        self.hall_changes: List[Tuple[int, int]] = []
        self.gates = 0
        self.hall: Optional[int] = None
        self.quiet_from = 0             # last Hall or direction change
        self.last_off = [-(1 << 30)] * 6

    def set_reverse(self, reverse: bool, cycle: int, grace: int = 0) -> None:
        """The host asked for a direction at ``cycle``; the program may take
        up to ``grace`` clocks more than ``latency`` to read it."""
        self.reverse = reverse
        self.quiet_from = cycle + grace

    def observe(self, lines: Lines) -> None:
        c = lines.cycle
        gates, hall = lines.uo & 0x3F, lines.ui & 7
        if hall != self.hall:
            if self.hall is not None:
                self.hall_changes.append((c, hall))
            self.hall, self.quiet_from = hall, c
        changed = gates ^ self.gates
        for bit in range(6):
            if not changed >> bit & 1:
                continue
            level = gates >> bit & 1
            self.edges[bit].append((c, level))
            if level:
                other = bit ^ 1
                if c - self.last_off[other] < self.dead:
                    self.violations.append((c, "dead", bit))
            else:
                self.last_off[bit] = c
        for high, low in PHASES:
            if gates >> high & 1 and gates >> low & 1:
                self.violations.append((c, "shoot", high))
        extra = gates & ~drive_word(hall, self.reverse)
        if extra and c - self.quiet_from > self.latency:
            self.violations.append((c, "pair", extra))
        self.gates = gates

    def pulses(self, bit: int, since: int = 0) -> List[Tuple[int, int]]:
        """``(rise, fall)`` cycle pairs of gate ``bit`` from ``since`` on."""
        out, rise = [], None
        for cyc, level in self.edges[bit]:
            if cyc < since:
                continue
            if level:
                rise = cyc
            elif rise is not None:
                out.append((rise, cyc))
                rise = None
        return out
