"""Sega Mega Drive / Genesis control pad, three- and six-button: pad and console models.

Written from public secondary sources (Sega's manual, MK-1653-50, cited by
Genesis Plus GX, was not available), never from ``firmware/sega6_*.loom``.
Every figure is secondary:

SOURCES
* [G] Charles MacDonald, "Sega Genesis hardware notes" (gen-hw.txt), as
  mirrored at gendev.spritesmind.net, section 3.1 (I/O port: data bits 3:0
  RIGHT LEFT DOWN UP, bits 4 and 5 TL/TR, bit 6 TH; "0" pressed, "1"
  released) and 3.2 "Gamepad specifics". Read 2026-11-07:
      TH = 1 : ?1CBRLDU    TH = 0 : ?0SA00DU          (three-button pad)
  and the six-button sequence, "based on how many times TH is switched
  from 1 to 0". The copy read (through a summarising fetch) listed six
  rows with SA0000 on the second low; [P] quotes the same table from
  gen_hw.txt with eight rows, the first two pairs being three-button
  values, and [Q] and [B] agree with eight (three falls). Taken: eight,
      1 CBRLDU, 0 SA00DU, 1 CBRLDU, 0 SA00DU, 1 CBRLDU,
      0 SA0000 (third low: D3-0 forced 0), 1 CBMXYZ (extra buttons in
      D3-0), 0 SA1111 (fourth low: D3-0 forced 1),
  "from this point on, the standard 3-button pad values will be returned";
  the counter times out if TH is not changed in "about 8192 (probably less
  than that) 68000 CPU cycles" (1.07 ms at 7.67 MHz).
  Which of TL and TR is bit 4 and which bit 5: the copy as read was
  ambiguous; the bit order of the tables above is what this model uses.
* [P] Genesis Plus GX, ``core/input_hw/gamepad.c`` (ekeeke, GitHub master,
  read 2026-11-07): the same table (quoting gen-hw), the counter advanced
  on TH 0 -> 1 and frozen from 8, reset by a timeout of 25 refresh calls
  (per scanline: about 1.6 ms; the unit is our reading, unchecked).
* [M] the Moa emulator, commit 4ca0c9b6 "Fixed bug in genesis controllers
  with incorrect counter" (git.applefritter.com): a 1.5 ms timer resets
  the TH counter. Read 2026-11-07 through a search summary only: unchecked.
* [Q] plutiedev.com, "Controllers" (read 2026-11-07): the same seven
  steps; wait "a bit" ("a few nop") between the TH write and the read; the
  six-button pad is "a lot more picky" about that delay; no figure.
* [B] hackaday.io BlueRetro log 180790 (read 2026-11-07): the extra
  buttons "trigger if 3 falling edge arise on TH within ~1ms".
* The brief's figure, "about 1.5 ms", is the reset time this model uses
  (``RESET``); the sources span 1.07 [G] to about 1.6 ms [P].
* Not found: the pad's response time to a TH edge, any tolerance, and the
  DE-9 pin numbers (from memory: UP 1, DOWN 2, LEFT 3, RIGHT 4, TL 6,
  TH 7, TR 9; unchecked, and not used here).

Line words are D5..D0 in bits 5:0, as on the wire (1 = high = released).
The button word used by the tests and both programs (an interface choice,
not part of the protocol) is, 1 = pressed: bits 5:0 U D L R B C, 7:6 A
Start, 11:8 Z Y X Mode.
"""

from __future__ import annotations

from typing import List, Optional, Sequence, Tuple

from tools.protomodels.bench import Drive, Lines, Model, PinSpec, pad_of

RESET = 1.5e-3          # counter reset after this long with no TH edge
RESET_RANGE = (1.07e-3, 1.6e-3)     # [G] .. [P]

NAMES = ("U", "D", "L", "R", "B", "C", "A", "Start", "Z", "Y", "X", "Mode")


def buttons(*pressed: str) -> int:
    """A button word, 1 = pressed (layout in the module docstring)."""
    w = 0
    for name in pressed:
        w |= 1 << NAMES.index(name)
    return w


def lines_for(word: int, th: int, n: int, three: bool = False) -> int:
    """D5..D0 as the pad drives them [G], for TH level ``th`` and ``n`` TH
    falls since the counter reset (frozen at 5)."""
    b = ~word & 0xFFF                           # active low
    if th:
        if n == 3 and not three:
            return (b & 0x30) | ((b >> 8) & 0xF)        # C B M X Y Z
        return b & 0x3F                                 # C B R L D U
    sa = (b >> 2) & 0x30                                # S A in D5 D4
    if not three and n == 3:
        return sa                                       # S A 0 0 0 0
    if not three and n == 4:
        return sa | 0xF                                 # S A 1 1 1 1
    return sa | (b & 3)                                 # S A 0 0 D U


def report(word: int, *, six: bool = True, present: bool = True) -> int:
    """What ``sega6_host.loom`` reports for a pad holding ``word``."""
    if not present:
        return 0
    r = (word & 0xFF) | 0x1000
    if six:
        r |= (word & 0xF00) | 0x2000
    return r


def clocks(seconds: float, clock_hz: float) -> int:
    return int(round(seconds * clock_hz))


def _pads(first: PinSpec, given: Optional[Sequence[PinSpec]]) -> List[Tuple[str, int]]:
    if given is not None:
        return [pad_of(p) for p in given]
    port, bit = pad_of(first)
    return [(port, bit + i) for i in range(6)]


# --------------------------------------------------------------------- pad
class Sega6Pad(Model):
    """The pad, clock by clock [G][P]: a multiplexer on TH and a counter of
    TH falls that resets after ``reset`` seconds with no TH edge.

    Args:
        th: the pad TH arrives on (``BIDIR6`` from ``sega6_host.loom``);
        d0: the pad of D0, D1..D5 on the next five (``BIDIR0``), or
            ``data``, six pads.
        word: the buttons (:func:`buttons`); :meth:`press` changes them.
        three: a three-button pad (no counter).
        delay: clocks from a seen TH edge to the lines changing (>= 1).

    Records ``phases`` ``[(cycle, th, n)]`` per TH edge and ``resets``
    (cycles at which a counter above 0 went back to 0).
    """

    def __init__(self, th: PinSpec = "BIDIR6", d0: PinSpec = "BIDIR0", *,
                 data: Optional[Sequence[PinSpec]] = None, word: int = 0,
                 three: bool = False, clock_hz: float = 50e6,
                 reset: float = RESET, delay: int = 2) -> None:
        self.th, self.data = pad_of(th), _pads(d0, data)
        self.word, self.three = word, three
        self.reset = clocks(reset, clock_hz)
        self.delay = max(1, delay)
        self.prev, self.n, self.last = 1, 0, 0
        self.out = lines_for(word, 1, 0, three)
        self.pending: List[Tuple[int, int]] = []
        self.phases: List[Tuple[int, int, int]] = []
        self.resets: List[int] = []

    def press(self, word: int) -> None:
        self.word = word
        self.pending.append((0, lines_for(word, self.prev, self.n, self.three)))

    def drive(self, drive: Drive, cycle: int) -> None:
        while self.pending and self.pending[0][0] <= cycle:
            self.out = self.pending.pop(0)[1]
        for i, pad in enumerate(self.data):
            drive.set(pad, (self.out >> i) & 1)

    def observe(self, lines: Lines) -> None:
        th, cyc = lines.get(self.th), lines.cycle
        if th != self.prev:
            if not th and not self.three:
                self.n = min(self.n + 1, 5)
            self.last = cyc
            self.prev = th
            self.phases.append((cyc, th, self.n))
        elif self.n and cyc - self.last > self.reset:
            self.n = 0
            self.resets.append(cyc)
        else:
            return
        self.pending.append((cyc + self.delay, lines_for(self.word, th, self.n, self.three)))


# ----------------------------------------------------------------- console
SIX = (1, 0, 1, 0, 1, 0, 1, 0, 1)
THREE = (1, 0, 1)


class Sega6Host(Model):
    """The console's reader [G][Q]: for each ``(start, levels)`` of ``plan``
    it holds TH at ``levels[i]`` from ``start + i * phase`` and reads the
    six lines ``settle`` clocks into each phase; TH is high otherwise.

    Records ``reads``, per plan entry the list of line words (D5..D0)."""

    def __init__(self, th: PinSpec = "IN3", d0: PinSpec = "OUT0", *,
                 data: Optional[Sequence[PinSpec]] = None,
                 plan: Sequence[Tuple[int, Sequence[int]]] = (),
                 phase: int = 125, settle: int = 100) -> None:
        assert 0 < settle < phase
        self.th, self.data = pad_of(th), _pads(d0, data)
        self.plan = [(s, tuple(lv)) for s, lv in plan]
        self.phase, self.settle = phase, settle
        self.reads: List[List[int]] = [[] for _ in self.plan]

    def _at(self, cycle: int) -> Tuple[int, int, int]:
        """(TH level, plan index or -1, phase index)."""
        for k, (s, lv) in enumerate(self.plan):
            if s <= cycle < s + len(lv) * self.phase:
                i = (cycle - s) // self.phase
                return lv[i], k, i
        return 1, -1, -1

    def drive(self, drive: Drive, cycle: int) -> None:
        drive.set(self.th, self._at(cycle)[0])

    def observe(self, lines: Lines) -> None:
        _, k, i = self._at(lines.cycle)
        if k >= 0 and lines.cycle - self.plan[k][0] - i * self.phase == self.settle:
            self.reads[k].append(sum(lines.get(p) << b for b, p in enumerate(self.data)))
