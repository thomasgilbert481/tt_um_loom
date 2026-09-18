"""Pad-input stimulus plans for the co-simulation harness.

A plan says what the outside world drives onto the input pads on every clock
edge. Both the RTL and :mod:`tools.loomsim` are fed from the same plan, which
is what ``docs/SEMANTICS.md`` section 10 means by "the pad input values at
every edge".

Indexing: ``plan.at(k)`` is what the pads hold at the edge that **ends** cycle
``k`` (edge ``k + 1``), counting cycles from the reset edge as SEMANTICS 1
does. That is exactly the argument :meth:`tools.loomsim.Machine.set_pad_inputs`
takes before the model steps cycle ``k``, and what the harness drives onto the
DUT while the RTL is in cycle ``k``.

A plan is a list of per-pin waves rather than an event list: it is a few
hundred bytes of JSON, it answers any cycle without replaying history, and it
knows which pins actually move, so the generator can point untimed waits at
pins that will satisfy them. Three kinds of wave exist:

* ``const``: the pad never changes.
* ``square``: a square wave with half period ``period`` clocks. Half periods
  that are and are not multiples of the 4-clock slot grid are both used, so
  edges land in every phase relative to a thread's X cycle.
* ``noise``: a pseudo-random level that is redrawn every ``period`` clocks
  from a counter-based hash (no state, so any cycle can be evaluated
  directly). With ``period`` 1 or 2 this makes pulses shorter than a slot,
  which SEMANTICS 6.4 says ``WAITE`` may miss; both sides must miss the same
  ones.

Pin indices follow ``isa/isa.yaml``: 0..7 are the BIDIR pads (what the plan
drives is what the core reads whenever ``PIN_OE`` releases the pin; the pad
model in ``test/tb.v`` and ``Machine(loopback=True)`` loop driven bits back),
8..11 are ``ui_in[3:0]`` and 12 is ``ui_in[7]``.
"""

from __future__ import annotations

import dataclasses
from typing import Dict, FrozenSet, List, Sequence, Tuple

#: Pin indices the testbench can drive from outside.
INPUT_PINS: Tuple[int, ...] = tuple(range(0, 13))

#: Which ``ui_in`` bit carries pin index 8..12 (ARCHITECTURE 3.1).
UI_BIT: Dict[int, int] = {8: 0, 9: 1, 10: 2, 11: 3, 12: 7}

#: Bits of ``ui_in`` that are Loom inputs; bits 4..6 are the host SPI pins.
UI_MASK = 0x8F

WAVE_KINDS = ("const", "square", "noise")

_M64 = (1 << 64) - 1


def _mix64(value: int) -> int:
    """SplitMix64 finaliser: a portable, stateless integer hash."""
    value = (value + 0x9E3779B97F4A7C15) & _M64
    value = ((value ^ (value >> 30)) * 0xBF58476D1CE4E5B9) & _M64
    value = ((value ^ (value >> 27)) * 0x94D049BB133111EB) & _M64
    return value ^ (value >> 31)


@dataclasses.dataclass(frozen=True)
class PinWave:
    """One input pad.

    ``period`` is the half period of a square wave or the hold time of a
    noise level, in clocks; ``phase`` shifts the wave in time; ``init`` is the
    constant level (``const``) or the level of the first half period
    (``square``); ``key`` seeds the noise hash.
    """

    index: int
    kind: str
    period: int = 0
    phase: int = 0
    init: int = 0
    key: int = 0

    def level(self, cycle: int) -> int:
        if self.kind == "square":
            return (self.init ^ (((cycle + self.phase) // self.period) & 1)) & 1
        if self.kind == "noise":
            slot = (cycle + self.phase) // self.period
            return _mix64((self.key << 24) ^ (self.index << 56) ^ slot) & 1
        return self.init & 1

    @property
    def toggles(self) -> bool:
        return self.kind != "const"

    def to_obj(self) -> Dict:
        return {"index": self.index, "kind": self.kind, "period": self.period,
                "phase": self.phase, "init": self.init, "key": self.key}

    @staticmethod
    def from_obj(obj: Dict) -> "PinWave":
        return PinWave(index=int(obj["index"]), kind=str(obj["kind"]),
                       period=int(obj.get("period", 0)),
                       phase=int(obj.get("phase", 0)),
                       init=int(obj.get("init", 0)), key=int(obj.get("key", 0)))


class StimulusPlan:
    """What the input pads hold at every clock edge."""

    def __init__(self, waves: Sequence[PinWave], cycles: int = 0):
        self.waves: List[PinWave] = sorted(waves, key=lambda w: w.index)
        self.cycles = int(cycles)
        for wave in self.waves:
            if wave.index not in INPUT_PINS:
                raise ValueError("pin %d is not an input pad" % wave.index)
            if wave.kind not in WAVE_KINDS:
                raise ValueError("unknown wave kind %r" % wave.kind)
            if wave.kind != "const" and wave.period < 1:
                raise ValueError("pin %d: period must be >= 1" % wave.index)
        self._moving = [w for w in self.waves if w.toggles]
        ui = uio = 0
        for wave in self.waves:
            if wave.kind == "const" and wave.init & 1:
                if wave.index <= 7:
                    uio |= 1 << wave.index
                else:
                    ui |= 1 << UI_BIT[wave.index]
        self._const = (ui, uio)

    def at(self, cycle: int) -> Tuple[int, int]:
        """``(ui_in, uio_drv)`` held at the edge that ends cycle ``cycle``.

        ``ui_in`` carries only the Loom input bits (``UI_MASK``); the harness
        ORs in the host SPI pins before it drives the pad.
        """
        ui, uio = self._const
        for wave in self._moving:
            if wave.level(cycle):
                if wave.index <= 7:
                    uio |= 1 << wave.index
                else:
                    ui |= 1 << UI_BIT[wave.index]
        return ui, uio

    @property
    def toggling(self) -> FrozenSet[int]:
        """Pin indices that change during the run: safe for an untimed wait."""
        return frozenset(w.index for w in self._moving)

    @property
    def constant(self) -> FrozenSet[int]:
        return frozenset(w.index for w in self.waves if not w.toggles)

    def to_obj(self) -> Dict:
        return {"cycles": self.cycles,
                "waves": [w.to_obj() for w in self.waves]}

    @staticmethod
    def from_obj(obj: Dict) -> "StimulusPlan":
        return StimulusPlan([PinWave.from_obj(w) for w in obj["waves"]],
                            int(obj.get("cycles", 0)))

    def __eq__(self, other) -> bool:
        return isinstance(other, StimulusPlan) and self.to_obj() == other.to_obj()

    def __repr__(self) -> str:
        return "<StimulusPlan %d waves, %d moving>" % (
            len(self.waves), len(self._moving))


#: Half periods for square waves. They include multiples of the 4-clock slot
#: grid and values that are not, so edges land in every phase of a slot.
SQUARE_HALF_PERIODS: Tuple[int, ...] = (3, 5, 6, 7, 8, 11, 13, 16, 17, 19,
                                        23, 29, 31, 37, 48, 61)
#: Hold times for noise waves: 1 and 2 give pulses shorter than a slot.
NOISE_HOLDS: Tuple[int, ...] = (1, 2, 3, 4, 5, 7, 9, 12, 16, 24)


def build_plan(rng, cycles: int, busy: bool = False) -> StimulusPlan:
    """A plan in which most input pads move and a few are held constant.

    ``busy`` shortens the periods and raises the share of noise, which is
    what the ``pins`` profile wants: a pad that changes every few cycles works
    the synchroniser and the ``WAITE`` edge detector far harder than one that
    changes twice a run.
    """
    waves: List[PinWave] = []
    squares = SQUARE_HALF_PERIODS[:9] if busy else SQUARE_HALF_PERIODS
    for index in INPUT_PINS:
        roll = rng.random()
        if roll < (0.08 if busy else 0.15):
            waves.append(PinWave(index, "const", init=rng.randrange(2)))
        elif roll < (0.45 if busy else 0.35):
            hold = rng.choice(NOISE_HOLDS[:6] if busy else NOISE_HOLDS)
            waves.append(PinWave(index, "noise", period=hold,
                                 phase=rng.randrange(hold),
                                 key=rng.randrange(1 << 24)))
        else:
            half = rng.choice(squares)
            if not busy:
                half *= rng.choice((1, 1, 1, 2, 3))
            waves.append(PinWave(index, "square", period=half,
                                 phase=rng.randrange(2 * half),
                                 init=rng.randrange(2)))
    # Make sure at least two of the five dedicated inputs move, so an untimed
    # wait always has a pin that only the stimulus (never the chip) controls.
    moving_in = [w for w in waves if 8 <= w.index <= 12 and w.toggles]
    for position, wave in enumerate(waves):
        if len(moving_in) >= 2:
            break
        if 8 <= wave.index <= 12 and not wave.toggles:
            fixed = PinWave(wave.index, "square", period=7, phase=wave.index,
                            init=wave.init)
            waves[position] = fixed
            moving_in.append(fixed)
    return StimulusPlan(waves, cycles)
