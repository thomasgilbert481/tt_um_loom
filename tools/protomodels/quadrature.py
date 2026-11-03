"""Incremental (quadrature) encoder: a waveform source and a reference x4 decoder.

Written from the encoder and MCU documents below, never from the firmware
(``firmware/quad_decoder.loom``, ``firmware/quad_generator.loom``).

Waveform (Broadcom/Avago HEDS-5540, "Quick Assembly Two and Three Channel
Optical Encoders", the HEDS-55xx/56xx and HEDM-55xx/560x datasheet):

* two channels A and B in quadrature, four states per cycle, each state
  nominally 90 electrical degrees (90 degE) wide;
* the index P0 of the three-channel HEDS-5540 is "a one state width
  (nominally 90 electrical degrees), high true index pulse which is
  coincident with the low states of channels A and B", once per
  revolution (checked against the datasheet text 2026-11-03);
* the logic state width error is at most 45 degE, so the shortest state,
  the minimum separation of two edges (one on A, one on B), is 45 degE
  (checked against a search excerpt of the datasheet's encoding
  characteristics table; the column layout of that excerpt, typical
  against maximum, is UNCHECKED);
* "performance is guaranteed to 100 kHz", the count frequency of one
  channel (checked against the datasheet text). At 100 kHz a cycle is
  10 us and the shortest state 1.25 us.

Counting rules (x4):

* STMicroelectronics RM0090 (STM32F4 reference manual), general-purpose
  timers, "Encoder interface mode" and its table "Counting direction
  versus encoder signals": in mode 3 the counter counts on both edges of
  both inputs, up or down according to the level of the other input; the
  section number (18.3.12 in the revision recalled) is UNCHECKED.
* Microchip dsPIC33/PIC24 Family Reference Manual, "Quadrature Encoder
  Interface (QEI)" (DS70000601): x4 mode counts every edge of A and B, and
  the count goes up when phase A leads phase B; UNCHECKED against the
  document in this session (from the family manual as recalled).
* Neither document defines a counted step for both channels changing
  together: a Gray sequence never does that. This module follows the
  common QEI reading: a transition that changes both channels is invalid,
  the count is unchanged, an error is counted, and the decoder takes the
  new state.

With the state written AB and phase ``ph`` = position mod 4: ph 0 is 00,
1 is 10, 2 is 11, 3 is 01. A step +1 (A leads B) is 00 -> 10 -> 11 -> 01
-> 00, a step -1 the reverse.

Pins default to the firmware's (A = IN0, B = IN1, Z = IN2).
"""

from __future__ import annotations

import random
from collections import deque
from typing import Iterable, List, Optional, Sequence, Tuple

from .bench import Drive, Lines, Model, PinSpec, pad_of

#: (A, B) for each phase
PHASE_AB = ((0, 0), (1, 0), (1, 1), (0, 1))
AB_PHASE = {ab: ph for ph, ab in enumerate(PHASE_AB)}

#: HEDS-5540 figures (module docstring)
HEDS5540_MAX_HZ = 100_000
HEDS5540_MIN_STATE_DEGE = 45

Event = Tuple[int, int, int, int]          # (cycle, A, B, Z)


def to_signed(v: int) -> int:
    v &= 0xFFFF
    return v - 0x10000 if v & 0x8000 else v


def min_edge_clocks(count_hz: float, clock_hz: float,
                    state_dege: float = HEDS5540_MIN_STATE_DEGE) -> float:
    """The shortest edge separation, in clocks, of an encoder at
    ``count_hz`` whose shortest state is ``state_dege`` electrical degrees."""
    return clock_hz / count_hz * state_dege / 360.0


def steps(start: int, phase: int, n: int, spacing: int, *, pos: int = 0,
          index_every: int = 0, jitter: int = 0, seed: int = 0,
          invalid: Iterable[int] = ()) -> Tuple[List[Event], int, int]:
    """The events of ``n`` steps (``n < 0``: -1 steps) from ``phase`` and
    position ``pos``, the first at cycle ``start`` and then every
    ``spacing`` clocks, each moved by a uniform ``-jitter..+jitter``.

    ``index_every`` = N (a multiple of 4, 0 = none) raises Z while the
    position is a multiple of N (then phase 0, state 00). The steps whose
    numbers (0-based) are in ``invalid`` change both channels at once: the
    phase and position move by 2, as if a step had been skipped.

    Returns ``(events, end_phase, end_pos)``.
    """
    rng = random.Random(seed)
    bad = set(invalid)
    d = 1 if n >= 0 else -1
    events: List[Event] = []
    t = start
    for k in range(abs(n)):
        stride = 2 if k in bad else 1
        pos += d * stride
        phase = (phase + d * stride) % 4
        a, b = PHASE_AB[phase]
        z = int(bool(index_every) and pos % index_every == 0)
        cyc = t + (rng.randint(-jitter, jitter) if jitter else 0)
        events.append((cyc, a, b, z))
        t += spacing
    events.sort()
    return events, phase, pos


class QuadSource(Model):
    """An encoder on three input pads: drives A, B, Z from scheduled events.

    An event ``(cycle, a, b, z)`` sets the levels the source drives from
    ``cycle`` on (the pad then changes at the edge that ends ``cycle - 1``,
    a registered model's view)."""

    def __init__(self, a: PinSpec = "IN0", b: PinSpec = "IN1",
                 z: Optional[PinSpec] = "IN2", level=(0, 0, 0)) -> None:
        self.pads = [pad_of(a), pad_of(b), pad_of(z) if z is not None else None]
        self.level = list(level)
        self.events: deque = deque()

    def schedule(self, events: Sequence[Event]) -> None:
        merged = sorted(list(self.events) + list(events))
        self.events = deque(merged)

    @property
    def idle(self) -> bool:
        return not self.events

    def drive(self, drive: Drive, cycle: int) -> None:
        ev = self.events
        while ev and ev[0][0] <= cycle:
            _, a, b, z = ev.popleft()
            self.level = [a, b, z]
        for pad, v in zip(self.pads, self.level):
            if pad is not None:
                drive.set(pad, v)


class QuadDecoder(Model):
    """The reference x4 decoder of the module docstring, clock-exact: it
    looks at the resolved pads every cycle, so it is the decoder with no
    sampling limit. Also records every change with its cycle
    (``edges``: ``(cycle, a, b, z)``) for timing checks."""

    def __init__(self, a: PinSpec = "IN0", b: PinSpec = "IN1",
                 z: Optional[PinSpec] = "IN2") -> None:
        self.pads = (pad_of(a), pad_of(b), pad_of(z) if z is not None else None)
        self.state: Optional[Tuple[int, int]] = None
        self.zlev = 0
        self.count = 0
        self.dir = 0                        # 1: the last count was -1
        self.errors = 0
        self.index: Optional[int] = None    # count at the latest index
        self.indexes = 0
        self._latched = False
        self.edges: List[Event] = []

    def feed(self, a: int, b: int, z: int = 0, cycle: int = 0) -> None:
        """One sample of the lines (any rate: the clock-exact view)."""
        if self.state is None:
            self.state, self.zlev = (a, b), z
            self.edges.append((cycle, a, b, z))
            self._index(z)
            return
        if (a, b) == self.state and z == self.zlev:
            return
        self.edges.append((cycle, a, b, z))
        if (a, b) != self.state:
            old, new = AB_PHASE[self.state], AB_PHASE[(a, b)]
            delta = (new - old) % 4
            if delta == 1:
                self.count = to_signed(self.count + 1)
                self.dir = 0
            elif delta == 3:
                self.count = to_signed(self.count - 1)
                self.dir = 1
            else:                           # both changed: invalid
                self.errors += 1
            self.state = (a, b)
            self._latched = False
        self.zlev = z
        self._index(z)

    def _index(self, z: int) -> None:
        if self.state == (0, 0) and z and not self._latched:
            self.index = self.count
            self.indexes += 1
            self._latched = True

    def observe(self, lines: Lines) -> None:
        a = lines.get(self.pads[0])
        b = lines.get(self.pads[1])
        z = lines.get(self.pads[2]) if self.pads[2] is not None else 0
        self.feed(a, b, z, lines.cycle)


def decode(events: Sequence[Event], start=(0, 0, 0)) -> QuadDecoder:
    """The reference decoder run over a list of events."""
    dec = QuadDecoder.__new__(QuadDecoder)
    QuadDecoder.__init__(dec, "IN0", "IN1", "IN2")
    dec.feed(*start, cycle=-1)
    for cyc, a, b, z in sorted(events):
        dec.feed(a, b, z, cyc)
    return dec


class Wire(Model):
    """A board wire from one pad to another: ``dst`` follows ``src`` one
    clock later, as a registered model must (generator outputs to decoder
    inputs)."""

    def __init__(self, src: PinSpec, dst: PinSpec) -> None:
        self.src, self.dst = pad_of(src), pad_of(dst)
        self.level = 0

    def drive(self, drive: Drive, cycle: int) -> None:
        drive.set(self.dst, self.level)

    def observe(self, lines: Lines) -> None:
        self.level = lines.get(self.src)
