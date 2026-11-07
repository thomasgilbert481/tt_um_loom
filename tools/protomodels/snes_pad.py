"""SNES / Super Famicom game controller: a pad model, a console model, frame timing.

Written from public secondary sources (Nintendo published no specification),
never from ``firmware/snes_*.loom``. Every figure is secondary:

SOURCES
* [C] Jim Christy, "Super Nintendo Entertainment System: pinouts & protocol",
  as mirrored in the repairfaq.org SNES FAQ (``F_SNES.html``), section 5
  "SNES Controller Communication Protocol" (timing diagram credited to
  Declan Williams) and section 6 (button order). Read 2026-11-07:
  - every 16.67 ms (60 Hz) the CPU sends a 12 us wide, positive-going latch
    pulse; 6 us after its fall it sends 16 clock pulses, 50 % duty, 12 us
    a full cycle (so a 6 us half);
  - the controller shifts its data out on every clock rise, the CPU
    samples it on every fall; the clock idles high (implied by the fall
    coming first, not stated in words: unchecked);
  - clock cycles 1..12 are B, Y, Select, Start, Up, Down, Left, Right, A,
    X, L, R; cycles 13..16 are left high by controllers; the data line is
    active low (high = not pressed).
* [N] snes.nesdev.org wiki, "Standard controller". Read 2026-11-07: the same
  order, MSB first (B first); the last four bits are a signature, 0000 in
  the register's polarity (1 = pressed), so high on the wire, for a
  standard pad; reads past the first 16 bits "return 1s on official
  controllers" (register polarity: low on the wire; the polarity of that
  sentence is our reading, unchecked), third-party pads may return 0s.
* [S] wiki.superfamicom.org, "Schematics, ports and pinouts", section
  "Controller Ports": pin 2 clock, 3 latch, 4 data1; the data lines are
  pulled low inside the console, so an empty port reads all pressed. The
  mouse sub-sections show a 0.5 us clock pulse and a clock "going low for
  700 ns": the shortest clock pulse in any source read, taken here as the
  fastest clock half (``FASTEST_HALF``) to test against. Read 2026-11-07.
* Not found: any tolerance, the pad's own propagation delay (a 4021 CMOS
  shift register: tens of ns, from memory, unchecked), and an official
  minimum or maximum for any of the times above. "Timing has no formal
  standard."

Bench conventions (``tools/protomodels/bench.py``): a model drives from the
state registered at the previous edge. Times are kept in clocks.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Sequence, Tuple

from tools.protomodels.bench import Drive, Lines, Model, PinSpec, pad_of
from tools.protomodels.hx711 import EdgeLog          # noqa: F401  (re-exported)

LATCH = 12e-6           # [C] latch pulse width
GAP = 6e-6              # [C] latch fall to first clock fall
HALF = 6e-6             # [C] clock half (12 us a cycle, 50 % duty)
FASTEST_HALF = 0.5e-6   # [S] the mouse's 0.5 us clock pulse
FRAME = 1 / 60.0        # [C] one read every 16.67 ms
BITS = 16

#: Clock cycle order [C]; bit 15 of a word is the first bit sent.
BUTTONS = ("B", "Y", "Select", "Start", "Up", "Down", "Left", "Right",
           "A", "X", "L", "R")


def word(*pressed: str, signature: int = 0) -> int:
    """A 16-bit word, 1 = pressed, the first bit sent in bit 15; the last
    four bits (``signature``, 0 for a standard pad [N]) in bits 3:0."""
    w = signature & 0xF
    for name in pressed:
        w |= 1 << (15 - BUTTONS.index(name))
    return w


def clocks(seconds: float, clock_hz: float) -> int:
    return int(round(seconds * clock_hz))


# ------------------------------------------------------------------ frames
def frames(latch_edges: Sequence[Tuple[int, int]],
           clock_edges: Sequence[Tuple[int, int]]) -> List[Dict[str, object]]:
    """Split edge logs ``[(cycle, level)]`` into reads: for each latch pulse
    ``rise``, ``fall`` and the clock ``falls`` and ``rises`` after its rise
    and before the next latch rise."""
    rises = [c for c, v in latch_edges if v]
    falls = [c for c, v in latch_edges if not v]
    out = []
    for i, r in enumerate(rises):
        nxt = rises[i + 1] if i + 1 < len(rises) else 1 << 62
        f = next((c for c in falls if c > r), -1)
        out.append({
            "rise": r, "fall": f,
            "falls": [c for c, v in clock_edges if not v and r < c < nxt],
            "rises": [c for c, v in clock_edges if v and r < c < nxt],
        })
    return out


# --------------------------------------------------------------------- pad
class SnesPad(Model):
    """The pad, a 4021-type shift register, clock by clock [C][N].

    While LATCH is high the register loads the buttons (parallel mode) and
    DATA shows the first (B); each CLOCK rise with LATCH low shifts the next
    bit out; a 0 comes in behind the 16 bits [N]. DATA is active low.

    Args:
        latch, clock: the pads the console's lines arrive on (chip outputs
            ``OUT0``, ``OUT1`` for ``snes_host.loom``); data: the pad it
            drives (``IN0``).
        buttons: the word, 1 = pressed (:func:`word`); :meth:`press` changes it.
        delay: clocks from a seen edge to DATA changing, at least 1.

    Records ``latches`` (latch rise cycles) and ``shifts`` (per latch, the
    clock rises seen after its fall).
    """

    def __init__(self, latch: PinSpec = "OUT0", clock: PinSpec = "OUT1",
                 data: PinSpec = "IN0", *, buttons: int = 0,
                 delay: int = 1) -> None:
        self.latch, self.clock, self.data = pad_of(latch), pad_of(clock), pad_of(data)
        self.buttons = buttons & 0xFFFF
        self.delay = max(1, delay)
        self.reg = 0
        self.out = 0
        self.pending: List[Tuple[int, int]] = []
        self.prev_latch = 0
        self.prev_clock = 1
        self.latches: List[int] = []
        self.shifts: List[int] = []

    def press(self, buttons: int) -> None:
        self.buttons = buttons & 0xFFFF

    def drive(self, drive: Drive, cycle: int) -> None:
        while self.pending and self.pending[0][0] <= cycle:
            self.out = self.pending.pop(0)[1]
        drive.set(self.data, self.out)

    def observe(self, lines: Lines) -> None:
        lat, clk = lines.get(self.latch), lines.get(self.clock)
        changed = False
        if lat:
            if not self.prev_latch:
                self.latches.append(lines.cycle)
            self.reg = ~self.buttons & 0xFFFF
            changed = True
        else:
            if self.prev_latch:
                self.shifts.append(0)
            if clk and not self.prev_clock:
                self.reg = (self.reg << 1) & 0xFFFF
                if self.shifts:
                    self.shifts[-1] += 1
                changed = True
        if changed:
            self.pending.append((lines.cycle + self.delay, self.reg >> 15))
        self.prev_latch, self.prev_clock = lat, clk


# ----------------------------------------------------------------- console
class SnesHost(Model):
    """The console's reader [C]: LATCH high ``latch_t``, ``gap`` to the first
    CLOCK fall, ``nclocks`` pulses of ``low`` then ``high``, CLOCK idle high;
    DATA sampled in the cycle each fall is driven (the console samples on
    the fall). Reads start at ``start`` and repeat every ``period`` clocks,
    ``n`` of them.

    Records ``words`` (one per read, 1 = pressed, first bit in bit 15, and
    the bits past 16 in ``extra``).
    """

    def __init__(self, latch: PinSpec = "IN1", clock: PinSpec = "IN2",
                 data: PinSpec = "OUT2", *, clock_hz: float = 50e6,
                 start: int = 0, n: int = 1, period: Optional[int] = None,
                 latch_t: float = LATCH, gap: float = GAP, low: float = HALF,
                 high: float = HALF, nclocks: int = BITS) -> None:
        self.latch, self.clock, self.data = pad_of(latch), pad_of(clock), pad_of(data)
        self.L = clocks(latch_t, clock_hz)
        self.G = clocks(gap, clock_hz)
        self.lo, self.hi = clocks(low, clock_hz), clocks(high, clock_hz)
        self.nclocks = nclocks
        self.length = self.L + self.G + nclocks * (self.lo + self.hi)
        self.period = period if period is not None else self.length + 100
        self.start, self.n = start, n
        self.bits: List[int] = []
        self.words: List[int] = []
        self.extra: List[List[int]] = []

    def again(self, start: int, n: int = 1) -> None:
        """Schedule ``n`` more reads from ``start`` (after the last one)."""
        self.start, self.n = start, n

    def done(self, cycle: int) -> bool:
        """All scheduled reads are over by ``cycle``."""
        return cycle >= self.start + (self.n - 1) * self.period + self.length

    def _phase(self, cycle: int) -> Tuple[int, int, int]:
        """(latch, clock, index of the fall at this cycle or -1)."""
        k, t = divmod(cycle - self.start, self.period)
        if cycle < self.start or k >= self.n or t >= self.length:
            return 0, 1, -1
        if t < self.L:
            return 1, 1, -1
        t -= self.L + self.G
        if t < 0:
            return 0, 1, -1
        i, u = divmod(t, self.lo + self.hi)
        return 0, (0 if u < self.lo else 1), (i if u == 0 else -1)

    def drive(self, drive: Drive, cycle: int) -> None:
        lat, clk, _ = self._phase(cycle)
        drive.set(self.latch, lat)
        drive.set(self.clock, clk)

    def observe(self, lines: Lines) -> None:
        _, _, i = self._phase(lines.cycle)
        if i < 0:
            return
        if i == 0:
            self.bits = []
        self.bits.append(lines.get(self.data))
        if len(self.bits) == self.nclocks:
            w = 0
            for b in self.bits[:BITS]:
                w = (w << 1) | (1 - b)
            self.words.append(w)
            self.extra.append(self.bits[BITS:])
