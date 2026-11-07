"""ITA2 ("Baudot") teleprinter reference model: the code, the shift rules
and start-stop line timing with a 1.5-unit stop element.

Written from the specifications, not from the Loom firmware:

* **ITU-T S.1 (03/93), International Telegraph Alphabet No. 2**, read
  2026-11-07 from the ITU PDF. Table 1/S.1 gives the 32 combinations,
  their letter case and figure case; 3.2 makes condition A start polarity
  and binary 0, condition Z stop polarity and binary 1; the note under
  Table 1 says code element 1 is transmitted first. 4.4: combinations 1 ..
  26 print in the case of the last shift received, and the first
  combination transmitted shall be a shift. 4.1 (figure D, "who are
  you?"), 4.3 (figure J, audible signal), 4.2 (figure F, G, H not defined
  internationally) and 4.7.2 (combination 32 not used during a call).
  Table 2 lists carriage return, line feed, the shifts and space as
  "either" case.
* **CCITT R.103 (Melbourne 1988)**, read from the Blue Book text: section
  1, "50 bauds (7.5 bits including a stop element of 1.5 units)"; 2.2,
  characters at 50 bauds have nominally 1.5-unit stop elements; 3.3, the
  minimum stop element released at an output is 1.25 units; 2.6, a start
  element under 0.4 unit is rejected and one over 0.6 unit accepted. The
  rate tolerance of 2.1 is illegible in that copy: unchecked.
* 45.45 bauds (22 ms a unit, 165 ms a 7.5-unit character) is the amateur
  radioteletype rate: **secondary sources only** (electronics-notes.com,
  "What is RTTY"; the minimodem RTTY protocol page), not an ITU text.

In this module a combination is an int 0 .. 31 with code element 1 in bit 0
(so E = 0x01, T = 0x10, LTRS = 0x1F, FIGS = 0x1B); a line level is 1 for
condition Z (mark, idle) and 0 for condition A.
"""

from __future__ import annotations

import dataclasses
from collections import deque
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from .bench import Drive, Lines, Model, PinSpec, pad_of

# Table 1/S.1, code elements 1 .. 5 as written in the table (A or Z).
_PATTERNS = {
    1: "ZZAAA", 2: "ZAAZZ", 3: "AZZZA", 4: "ZAAZA", 5: "ZAAAA", 6: "ZAZZA",
    7: "AZAZZ", 8: "AAZAZ", 9: "AZZAA", 10: "ZZAZA", 11: "ZZZZA", 12: "AZAAZ",
    13: "AAZZZ", 14: "AAZZA", 15: "AAAZZ", 16: "AZZAZ", 17: "ZZZAZ",
    18: "AZAZA", 19: "ZAZAA", 20: "AAAAZ", 21: "ZZZAA", 22: "AZZZZ",
    23: "ZZAAZ", 24: "ZAZZZ", 25: "ZAZAZ", 26: "ZAAAZ", 27: "AAAZA",
    28: "AZAAA", 29: "ZZZZZ", 30: "ZZAZZ", 31: "AAZAA", 32: "AAAAA",
}
#: combination number (1 .. 32) -> code (element 1 in bit 0).
CODE_OF_NUMBER: Dict[int, int] = {
    n: sum(1 << i for i, c in enumerate(p) if c == "Z") for n, p in _PATTERNS.items()}

CR, LF, LTRS, FIGS, SPACE, NULL = (CODE_OF_NUMBER[n] for n in (27, 28, 29, 30, 31, 32))
WRU = "\x05"                # figure D, "who are you?" (4.1), as ASCII ENQ
BELL = "\x07"               # figure J, audible signal (4.3), as ASCII BEL

_LETTERS = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
# Figure case of combinations 1 .. 26 (Table 1); None: not defined (4.2).
_FIGURES = ["-", "?", ":", WRU, "3", None, None, None, "8", BELL, "(", ")",
            ".", ",", "9", "0", "1", "4", "'", "5", "7", "=", "2", "/", "6", "+"]

#: code -> letter, code -> figure (combinations 1 .. 26 only).
LETTER: Dict[int, str] = {CODE_OF_NUMBER[i + 1]: ch for i, ch in enumerate(_LETTERS)}
FIGURE: Dict[int, str] = {CODE_OF_NUMBER[i + 1]: ch for i, ch in enumerate(_FIGURES) if ch}
#: Either-case combinations (Table 2) as the ASCII they stand for.
EITHER: Dict[int, str] = {CR: "\r", LF: "\n", SPACE: " "}

LETTERS_CASE, FIGURES_CASE, UNKNOWN = "L", "F", None


def lookup(ch: str) -> Optional[Tuple[int, Optional[str]]]:
    """``(code, case)`` for one ASCII character, lower case as upper case
    (S.1 2.3), case None for an either-case character; None when ITA2 has
    no code for it."""
    up = ch.upper() if "a" <= ch <= "z" else ch
    for code, c in EITHER.items():
        if c == up:
            return code, None
    for code, c in LETTER.items():
        if c == up:
            return code, LETTERS_CASE
    for code, c in FIGURE.items():
        if c == up:
            return code, FIGURES_CASE
    return None


def encode(text: str, *, send_wru: bool = False, usos_safe: bool = True,
           state: Optional[str] = UNKNOWN) -> Tuple[List[int], List[str]]:
    """Codes for ``text`` with the shifts S.1 4.4 needs, and the characters
    dropped (no ITA2 code; ENQ too unless ``send_wru``).

    The case starts unknown, so the first character is preceded by a shift
    (4.4: the first combination transmitted shall be a shift; LTRS before
    an either-case character). With ``usos_safe`` a space sent in figure
    case makes the case unknown again, so the next letter or figure gets
    its shift although an S.1 receiver would not need it: a receiver set to
    "unshift on space" (not in S.1) then prints the same."""
    codes: List[int] = []
    dropped: List[str] = []
    for ch in text:
        found = lookup(ch)
        if found is None or (ch == WRU and not send_wru):
            dropped.append(ch)
            continue
        code, case = found
        if case is None:
            if state is UNKNOWN:
                codes.append(LTRS)
                state = LETTERS_CASE
            codes.append(code)
            if usos_safe and code == SPACE and state == FIGURES_CASE:
                state = UNKNOWN
            continue
        if state != case:
            codes.append(LTRS if case == LETTERS_CASE else FIGS)
            state = case
        codes.append(code)
    return codes, dropped


def decode(codes: Iterable[int], *, usos: bool = False,
           state: str = LETTERS_CASE, undefined: str = "\x7f") -> str:
    """Text for a stream of codes, tracking the shift (S.1 4.4). With
    ``usos`` a space returns the receiver to letters (the amateur
    convention, not S.1). Shifts and the null print nothing; a figure-case
    combination with no definition (F, G, H: 4.2) gives ``undefined``."""
    out = []
    for code in codes:
        code &= 0x1F
        if code == LTRS:
            state = LETTERS_CASE
        elif code == FIGS:
            state = FIGURES_CASE
        elif code == NULL:
            pass
        elif code in EITHER:
            out.append(EITHER[code])
            if usos and code == SPACE:
                state = LETTERS_CASE
        elif state == LETTERS_CASE:
            out.append(LETTER[code])
        else:
            out.append(FIGURE.get(code, undefined))
    return "".join(out)


# ------------------------------------------------------------ line timing
UNITS_CODE = 5              # code elements a character
STOP_UNITS = 1.5            # R.103 1, 2.2
MIN_STOP_UNITS = 1.25       # R.103 3.3: the least an output may release
START_REJECT, START_ACCEPT = 0.4, 0.6   # R.103 2.6, units


def unit_clocks(baud: float, f_clk: float = 50e6) -> float:
    """Clocks a unit: 1 / baud seconds (45.45 bauds: 22 ms, 1.1 M clocks)."""
    return f_clk / baud


def frame_levels(code: int, stop: int = 1) -> List[int]:
    """Start (A), code elements 1 .. 5, stop (Z, or A for a broken frame)."""
    return [0] + [(code >> i) & 1 for i in range(UNITS_CODE)] + [stop]


class BaudotTx(Model):
    """Drives start-stop ITA2 characters onto a pad; idles at Z (1).

    ``unit`` is the element length in clocks (fractional allowed) and
    ``stop_units`` the stop element; ``send_frame(code, stop=0)`` holds the
    stop element at A, a framing error. Edges are placed at
    ``start + round(k * unit)``, so a stream never drifts; ``unit`` may be
    changed between frames to model a sender off rate."""

    def __init__(self, pin: PinSpec, unit: float, *, stop_units: float = STOP_UNITS,
                 gap_units: float = 0.0) -> None:
        self.pad = pad_of(pin)
        self.unit = float(unit)
        self.stop_units = stop_units
        self.gap_units = gap_units
        self.level = 1
        self.now = 0
        self._changes: deque = deque()
        self._free_at = 0.0
        self.frames_sent: List[Tuple[int, int, int]] = []   # (start, code, stop)

    def send(self, codes: Iterable[int]) -> None:
        for code in codes:
            self.send_frame(code)

    def send_frame(self, code: int, *, stop: int = 1, start: Optional[int] = None) -> int:
        t0 = max(self._free_at, float(self.now + 1))
        if start is not None:
            t0 = max(t0, float(start))
        for k, level in enumerate(frame_levels(code, stop)):
            self._changes.append((int(round(t0 + k * self.unit)), level))
        end = t0 + (1 + UNITS_CODE + self.stop_units) * self.unit
        self._changes.append((int(round(end)), 1))
        # After a stop held at A the line idles a unit, so the next start
        # element has an edge to be found by.
        gap = max(self.gap_units, 0.0 if stop else 1.0)
        self._free_at = end + gap * self.unit
        self.frames_sent.append((int(round(t0)), code & 0x1F, stop))
        return int(round(t0))

    @property
    def busy(self) -> bool:
        return bool(self._changes)

    def drive(self, drive: Drive, cycle: int) -> None:
        self.now = cycle
        while self._changes and self._changes[0][0] <= cycle:
            self.level = self._changes.popleft()[1]
        drive.set(self.pad, self.level)


@dataclasses.dataclass
class BaudotFrame:
    code: int
    start: int                      # cycle of the start edge (Z to A)
    framing_error: bool             # the stop element was A at its middle
    edges: List[Tuple[int, int]]    # (cycle, level) of the frame's edges


class BaudotRx(Model):
    """Decodes start-stop ITA2 from a pad by sampling each element at its
    middle (the stop element at the middle of its first unit). A start
    element that is back at Z at its middle is a glitch. Not armed until
    the line has been seen at Z."""

    def __init__(self, pin: PinSpec, unit: float) -> None:
        self.pad = pad_of(pin)
        self.unit = float(unit)
        self.prev = 0
        self.armed = False
        self.frames: List[BaudotFrame] = []
        self.edges: List[Tuple[int, int]] = []
        self._start: Optional[int] = None
        self._samples: List[int] = []
        self._next = 0
        self._frame_edges: List[Tuple[int, int]] = []

    def observe(self, lines: Lines) -> None:
        v = lines.get(self.pad)
        cycle = lines.cycle
        if v != self.prev:
            self.edges.append((cycle, v))
            if self._start is not None:
                self._frame_edges.append((cycle, v))
        if not self.armed:
            self.armed = v == 1
        elif self._start is None:
            if self.prev == 1 and v == 0:
                self._start, self._samples = cycle, []
                self._frame_edges = [(cycle, 0)]
                self._next = cycle + int(round(0.5 * self.unit))
        elif cycle >= self._next:
            self._samples.append(v)
            k = len(self._samples)
            if k == 1 and v != 0:
                self._start = None
            elif k == UNITS_CODE + 2:
                code = sum(b << i for i, b in enumerate(self._samples[1:-1]))
                self.frames.append(BaudotFrame(code, self._start, v == 0,
                                               list(self._frame_edges)))
                self._start = None
            else:
                self._next = self._start + int(round((k + 0.5) * self.unit))
        self.prev = v

    @property
    def codes(self) -> List[int]:
        """Every correctly framed code, in order."""
        return [f.code for f in self.frames if not f.framing_error]


def check_timing(edges: Sequence[Tuple[int, int]], unit: float, *, tol: float = 0.0,
                 back_to_back: bool = True) -> List[str]:
    """Check a transmitter's line edges (``[(cycle, level)]``, starting
    with the first start edge) against the start-stop format: every element
    a whole number of units within ``tol`` clocks, each character's run
    from its start edge to the end of element 5 exactly 6 units, and every
    stop element at least 1.5 units; with ``back_to_back`` a stop element
    followed by another character is 1.5 units within ``tol`` (R.103 1,
    2.2). The last stop element (which runs into idle) is not bounded
    above. Returns the problems found, empty when the edges are good."""
    bad: List[str] = []
    starts: List[int] = []
    i = 0
    # Walk characters: a start edge (to 0), then levels for 6 units.
    while i < len(edges):
        c0, lev = edges[i]
        if lev != 0:
            bad.append("edge %d at %d: expected a start edge" % (i, c0))
            return bad
        starts.append(c0)
        end6 = c0 + 6 * unit
        j = i + 1
        while j < len(edges) and edges[j][0] < end6 - tol - 0.5:
            off = (edges[j][0] - c0) / unit
            if abs(edges[j][0] - c0 - round(off) * unit) > tol + 0.5:
                bad.append("char at %d: edge at +%d clocks is off the unit grid"
                           % (c0, edges[j][0] - c0))
            j += 1
        # The stop element starts at end6 (an edge there if element 5 was A).
        if j < len(edges) and abs(edges[j][0] - end6) <= tol + 0.5 and edges[j][1] == 1:
            j += 1
        if j < len(edges):
            nxt = edges[j][0]
            stop = (nxt - end6) / unit
            if stop < STOP_UNITS - (tol + 0.5) / unit:
                bad.append("char at %d: stop element %.3f units" % (c0, stop))
            elif back_to_back and abs(nxt - end6 - STOP_UNITS * unit) > tol + 0.5:
                bad.append("char at %d: stop element %.3f units, not 1.5 back to back"
                           % (c0, stop))
        i = j
    return bad


def decode_edges(edges: Sequence[Tuple[int, int]], unit: float) -> List[int]:
    """Codes from a recorded line (``[(cycle, level)]``, idle at Z before
    the first edge), sampled at mid-element from each start edge: the
    reference decoder for a transmitter's output."""
    def level(t: float) -> int:
        lev = 1
        for c, v in edges:
            if c > t:
                break
            lev = v
        return lev
    codes, t_free = [], -1.0
    for c, v in edges:
        if v != 0 or c < t_free:
            continue
        if level(c + 0.5 * unit) != 0:
            continue
        codes.append(sum(level(c + (k + 1.5) * unit) << k for k in range(UNITS_CODE)))
        t_free = c + 6.5 * unit
    return codes
