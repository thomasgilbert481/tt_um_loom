"""International Morse code reference models, for firmware/morse_tx.loom and
morse_rx.loom.

Written from the specification, not from the firmware: ITU-R
Recommendation M.1677-1 (10/2009), "International Morse code", Annex 1
Part I, read on 2026-11-07 (the PDF from itu.int):

* 1.1.1 Letters (a .. z, and accented e ..-.., which is not ASCII and is
  left out here), 1.1.2 Figures (0 .. 9), 1.1.3 Punctuation marks and
  miscellaneous signs: full stop .-.-.-, comma --..--, colon ---...,
  question mark ..--.., apostrophe .----., hyphen -....-, fraction bar
  -..-., left bracket -.--., right bracket -.--.-, inverted commas
  .-..-., double hyphen -...- (written =), cross or addition sign .-.-.
  (+), commercial at .--.-. (@); multiplication sign -..-, which is the
  letter X (3.2.1). The signs with no written character (understood ...-.,
  error ........, invitation to transmit -.-, which is K, wait .-...,
  end of work ...-.-, starting signal -.-.-) have no ASCII code here: a
  decoder reports them as patterns (:data:`SIGNS` names them), except the
  error signal, which :func:`decode_units` reports as such.
* 2.1 a dash is equal to three dots; 2.2 the space between the signals of
  one letter is one dot; 2.3 between two letters three dots; 2.4 between
  two words seven dots.

The recommendation sets no speed. Words per minute are counted, by the
amateur and military convention, on the word PARIS, which is 50 dot units
with its word space: one dot lasts 1.2 s / WPM (:func:`dot_us`). That
convention is not in M.1677-1 (the word "PARIS" does not occur in it,
checked 2026-11-07); it is UNCHECKED against a primary document here.

Representations:

* a code is a string of ``.`` and ``-``;
* a code byte (:func:`code_byte`) is the code with its first element in
  bit 0 (1 = dash) and a 1 just above its last element, so ``.-`` (A) is
  0b110 = 6; it is what the firmware tables hold;
* pulses are ``[(level, duration), ...]`` with ``MARK`` (key down) and
  ``SPACE``, starting with a mark, the duration in dot units or in
  microseconds (``units_to_us``), as in :mod:`tools.protomodels.ir_nec`,
  whose edge recorder and pulse source this module reuses: ``IrSource``
  drives a pin low for a mark (the key or a tone decoder pulls the line
  low) and high otherwise.

Pieces: :func:`encode_units` and :func:`encode_us` (with a speed and a
timing jitter), :func:`decode_units` and :func:`decode_us` (the reference
decoder), :func:`check_timing` (a transmitter against 2.1 .. 2.4 with a
stated tolerance), :func:`speed_change`.
"""

from __future__ import annotations

import random
from typing import Dict, List, Optional, Sequence, Tuple

from .bench import Drive, Lines, Model, PinSpec, pad_of
from .ir_nec import MARK, SPACE, EdgeRecorder, IrSource  # noqa: F401

DOT, DASH = 1, 3                    # 2.1
ELEMENT_GAP, LETTER_GAP, WORD_GAP = 1, 3, 7     # 2.2, 2.3, 2.4
PARIS_UNITS = 50                    # the convention, not M.1677-1

LETTERS = {
    "A": ".-", "B": "-...", "C": "-.-.", "D": "-..", "E": ".", "F": "..-.",
    "G": "--.", "H": "....", "I": "..", "J": ".---", "K": "-.-", "L": ".-..",
    "M": "--", "N": "-.", "O": "---", "P": ".--.", "Q": "--.-", "R": ".-.",
    "S": "...", "T": "-", "U": "..-", "V": "...-", "W": ".--", "X": "-..-",
    "Y": "-.--", "Z": "--..",
}                                   # 1.1.1
FIGURES = {
    "1": ".----", "2": "..---", "3": "...--", "4": "....-", "5": ".....",
    "6": "-....", "7": "--...", "8": "---..", "9": "----.", "0": "-----",
}                                   # 1.1.2
PUNCTUATION = {
    ".": ".-.-.-", ",": "--..--", ":": "---...", "?": "..--..", "'": ".----.",
    "-": "-....-", "/": "-..-.", "(": "-.--.", ")": "-.--.-", '"': ".-..-.",
    "=": "-...-", "+": ".-.-.", "@": ".--.-.",
}                                   # 1.1.3
SIGNS = {
    "understood": "...-.", "error": "........", "invitation to transmit": "-.-",
    "wait": ".-...", "end of work": "...-.-", "starting signal": "-.-.-",
}                                   # 1.1.3, no written character
ERROR_SIGNAL = SIGNS["error"]

TABLE: Dict[str, str] = {}
TABLE.update(LETTERS)
TABLE.update(FIGURES)
TABLE.update(PUNCTUATION)
DECODE: Dict[str, str] = {v: k for k, v in TABLE.items()}

Pulses = List[Tuple[int, float]]


def dot_us(wpm: float) -> float:
    """One dot at ``wpm`` words per minute (PARIS, 50 units a word)."""
    return 60e6 / (PARIS_UNITS * wpm)


def code_of(ch: str) -> Optional[str]:
    """The code of an ASCII character (lower case as upper), or None."""
    return TABLE.get(ch.upper())


def code_byte(code: str) -> int:
    """The marker form: first element in bit 0, 1 = dash, a 1 above the last."""
    v = 0
    for k, e in enumerate(code):
        if e == "-":
            v |= 1 << k
    return v | (1 << len(code))


def code_from_byte(v: int) -> str:
    """Inverse of :func:`code_byte` (v >= 1)."""
    assert v >= 1
    n = v.bit_length() - 1
    return "".join("-" if (v >> k) & 1 else "." for k in range(n))


# ------------------------------------------------------------------ encoder
def encode_units(text: str) -> Tuple[Pulses, List[str]]:
    """Pulses in dot units for ``text``, and the characters skipped.

    Each character with a code (lower case taken as upper) is its elements,
    a dot 1 and a dash 3 (2.1), with a 1-unit space between them (2.2) and
    a 3-unit space after the character (2.3). A space character makes the
    space after the previous character a word space, 7 units (2.4); each
    further space adds 7 more. Characters without a code are skipped and
    cost no time. The list starts with a mark and ends with one (leading
    and trailing spaces are idle line, not pulses)."""
    out: Pulses = []
    skipped: List[str] = []
    for ch in text:
        if ch == " ":
            if out and out[-1][0] == SPACE:
                lev, u = out[-1]
                out[-1] = (SPACE, WORD_GAP if u == LETTER_GAP else u + WORD_GAP)
            continue
        code = code_of(ch)
        if code is None:
            skipped.append(ch)
            continue
        for k, e in enumerate(code):
            out.append((MARK, DASH if e == "-" else DOT))
            out.append((SPACE, ELEMENT_GAP if k < len(code) - 1 else LETTER_GAP))
    if out and out[-1][0] == SPACE:
        out.pop()
    return out, skipped


def units_to_us(pulses: Sequence[Tuple[int, float]], dot: float) -> Pulses:
    return [(lev, u * dot) for lev, u in pulses]


def encode_us(text: str, dot: float, *, jitter: float = 0.0,
              seed: int = 1) -> Pulses:
    """:func:`encode_units` at a dot of ``dot`` us, each duration multiplied
    by an independent factor uniform in 1 +- ``jitter``."""
    rng = random.Random(seed)
    pulses, _ = encode_units(text)
    return [(lev, u * dot * (1.0 + rng.uniform(-jitter, jitter)))
            for lev, u in pulses]


def join(a: Sequence[Tuple[int, float]], gap_us: float,
         b: Sequence[Tuple[int, float]]) -> Pulses:
    """Two pulse lists with a space of ``gap_us`` between them."""
    return list(a) + [(SPACE, gap_us)] + list(b)


def speed_change(first: str, dot1: float, second: str, dot2: float, *,
                 jitter: float = 0.0, seed: int = 1) -> Pulses:
    """``first`` at ``dot1``, a word space at the new speed (7 ``dot2``),
    then ``second`` at ``dot2``: a sender changing speed between words."""
    return join(encode_us(first, dot1, jitter=jitter, seed=seed), WORD_GAP * dot2,
                encode_us(second, dot2, jitter=jitter, seed=seed + 1))


# ------------------------------------------------------------------ decoder
def decode_units(pulses: Sequence[Tuple[int, float]]) -> List[object]:
    """Tokens from pulses in dot units: a character (str), ``" "`` for a word
    space, ``("pattern", code)`` for a code with no character, ``("error",
    n)`` for the error signal (n dots, n >= 8). A mark under 2 units is a
    dot, else a dash; a space under 2 units separates elements, under 5
    letters, else words (the midpoints of 2.1 .. 2.4)."""
    out: List[object] = []
    code = ""

    def flush():
        if not code:
            return
        if code in DECODE:
            out.append(DECODE[code])
        elif len(code) >= len(ERROR_SIGNAL) and set(code) == {"."}:
            out.append(("error", len(code)))
        else:
            out.append(("pattern", code))

    for lev, u in pulses:
        if lev == MARK:
            code += "." if u < 2 else "-"
        elif u >= 2:
            flush()
            code = ""
            if u >= 5:
                out.append(" ")
    flush()
    return out


def decode_us(pulses: Sequence[Tuple[int, float]], dot: float) -> List[object]:
    """:func:`decode_units` for durations in us at a known dot length."""
    return decode_units([(lev, us / dot) for lev, us in pulses])


def text_of(tokens: Sequence[object]) -> str:
    """The characters and word spaces of a token list (others dropped)."""
    return "".join(t for t in tokens if isinstance(t, str))


# ----------------------------------------------------------- timing checker
def check_timing(got: Sequence[Tuple[int, float]], nominal_units: Sequence[Tuple[int, float]],
                 dot: float, tol_us: float) -> List[Tuple[int, float, float]]:
    """A transmitter's pulses (us) against the nominal ones (units) at a dot
    of ``dot`` us: ``(index, us, nominal us)`` of every pulse further than
    ``tol_us`` from nominal; a length or level mismatch is reported at the
    first index where they differ (nominal -1)."""
    bad: List[Tuple[int, float, float]] = []
    for k, (g, n) in enumerate(zip(got, nominal_units)):
        if g[0] != n[0]:
            return bad + [(k, g[1], -1.0)]
        if abs(g[1] - n[1] * dot) > tol_us:
            bad.append((k, g[1], n[1] * dot))
    if len(got) != len(nominal_units):
        bad.append((min(len(got), len(nominal_units)), -1.0, -1.0))
    return bad


def _units(level: int, ratio: float) -> int:
    """The nearest length 2.1 .. 2.4 allow: marks 1 or 3, spaces 1, 3 or 7k."""
    if level == MARK:
        return DOT if ratio < 2 else DASH
    if ratio < 2:
        return ELEMENT_GAP
    if ratio < 5:
        return LETTER_GAP
    return WORD_GAP * max(1, int(round(ratio / WORD_GAP)))


def check_ratios(got: Sequence[Tuple[int, float]], tol: float) -> Tuple[float, List[Tuple[int, float, int]]]:
    """2.1 .. 2.4 on a pulse list (us) of unknown speed. The dot is first
    taken as the shortest pulse (the list must hold a dot or an element
    space), every pulse is given the nearest allowed length in units, and
    the dot is then re-estimated over the whole list. Returns that dot and
    ``(index, ratio, units)`` of every pulse whose length in dots is more
    than ``tol`` (a fraction) from its units."""
    d0 = min(us for _, us in got)
    units = [_units(lev, us / d0) for lev, us in got]
    dot = sum(us for _, us in got) / float(sum(units))
    bad = []
    for k, ((lev, us), u) in enumerate(zip(got, units)):
        if abs(us / dot - u) > tol * u:
            bad.append((k, us / dot, u))
    return dot, bad


# ------------------------------------------------------------- the board
class KeyLine(Model):
    """The board wire from a keyer's output (high = key down) to a decoder's
    input (low = key down), as a keying transistor pulling the decoder's
    line low against its pull-up: ``rx`` is the inverse of ``tx`` one clock
    later. Idle (before the first cycle) the line is high."""

    def __init__(self, tx: PinSpec, rx: PinSpec) -> None:
        self.tx, self.rx = pad_of(tx), pad_of(rx)
        self.level = 1

    def drive(self, drive: Drive, cycle: int) -> None:
        drive.set(self.rx, self.level)

    def observe(self, lines: Lines) -> None:
        self.level = 0 if lines.get(self.tx) else 1
