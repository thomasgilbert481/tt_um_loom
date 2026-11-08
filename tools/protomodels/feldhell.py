"""Feld-Hell (Hellschreiber) reference models, for firmware/feldhell_tx.loom.

Written from published descriptions of the mode, not from the firmware.
Feld-Hell has no formal standard; both sources below are SECONDARY
(amateur-radio descriptions of the 1930s Siemens machine and of its
software revivals), read on 2026-11-07 through the web fetch tool:

* M. Greenman ZL1BPU, "Feld-Hell", https://www.qsl.net/zl1bpu/HELL/Feld.htm:
  - "How Characters are Transmitted": the scan goes up each column from
    bottom to top, then up each following column from left to right;
    five columns a character with two blank columns between letters
    (7 in all); typically 14 dot positions a column;
  - "Transmission Timing": each character takes 400 ms, each pixel is
    8.163 ms long, 122.5 baud, 2.5 characters a second (about 25 WPM);
    the receiver prints the text twice, it is not transmitted twice;
  - "The Two Pixel Rule": each dot is sent as two half-height pixels, so
    a pulse to the transmitter is never shorter than 8 ms;
  - "Dot Shaping": raised-cosine keying confines the bandwidth to about
    245 Hz (300 Hz at 30 dB down); hard keying gives sidebands for some
    500 Hz; 17.5 columns a second;
  - "Character Generation": the wartime machine keyed a 900 Hz audio
    tone, amplitude keyed like Morse.
* Wikipedia, "Hellschreiber", section "Operation": a 7 by 7 pixel grid,
  sent column by column as on-off signals; each received column printed
  twice, one below the other. It gives "112.5 baud", which disagrees with
  Greenman's 122.5; 122.5 is the one consistent with 400 ms a character
  (49 pixels / 0.4 s) and is used here (QUESTIONS.md).

Not read in a source, so UNCHECKED: the 980 Hz carrier (the brief's
example; chosen because it is exactly 8 carrier cycles a pixel, so every
pixel holds whole cycles), and the font. The classic Hell glyphs were
not copied: :data:`FONT` is a plain 5 x 7 dot-matrix font drawn for this
model (rows top to bottom, ``#`` on), in the 5 glyph columns of the 7-
column cell, with the two blank columns after it.

Representations:

* a column is an int, bit 0 the bottom pixel, bit 6 the top;
* a character is 7 columns (:func:`columns_of`), a text the columns of
  its characters in order;
* the pixel stream (:func:`pixels_of`) is one value a pixel slot of
  1 / 122.5 s, column after column, bottom pixel first; each pixel is two
  half-pixel slots of equal value (:func:`half_pixels_of`).
"""

from __future__ import annotations

from typing import Dict, List, Optional, Sequence, Tuple

ROWS = 7                    # pixels a column (14 half-pixels)
COLUMNS = 7                 # columns a character: 5 of glyph, 2 blank
GLYPH_COLUMNS = 5
PIXEL_RATE = 122.5          # pixels a second (Greenman, Transmission Timing)
PIXEL_S = 1.0 / PIXEL_RATE  # 8.163 ms
HALF_S = PIXEL_S / 2        # 4.082 ms, a half-pixel
COLUMN_S = ROWS * PIXEL_S   # 57.1 ms
CHAR_S = COLUMNS * COLUMN_S  # 400 ms
CHAR_RATE = 1.0 / CHAR_S    # 2.5 a second
TONE_HZ = 980.0             # unchecked: 8 carrier cycles a pixel
FIRST, LAST = 0x20, 0x5F    # the drawn range; 0x60 .. 0x7F fold onto 0x40 ..

# The font, ASCII 0x20 .. 0x5F, 7 rows of 5, top row first.
_F = """
20 ..... ..... ..... ..... ..... ..... .....
21 ..#.. ..#.. ..#.. ..#.. ..#.. ..... ..#..
22 .#.#. .#.#. ..... ..... ..... ..... .....
23 .#.#. ##### .#.#. .#.#. .#.#. ##### .#.#.
24 ..#.. .#### #.#.. .###. ..#.# ####. ..#..
25 ##... ##..# ...#. ..#.. .#... #..## ...##
26 .##.. #..#. #.#.. .#... #.#.# #..#. .##.#
27 ..#.. ..#.. ..... ..... ..... ..... .....
28 ...#. ..#.. .#... .#... .#... ..#.. ...#.
29 .#... ..#.. ...#. ...#. ...#. ..#.. .#...
2A ..... ..#.. #.#.# .###. #.#.# ..#.. .....
2B ..... ..#.. ..#.. ##### ..#.. ..#.. .....
2C ..... ..... ..... ..... .##.. ..#.. .#...
2D ..... ..... ..... ##### ..... ..... .....
2E ..... ..... ..... ..... ..... .##.. .##..
2F ....# ...#. ...#. ..#.. .#... .#... #....
30 .###. #...# #..## #.#.# ##..# #...# .###.
31 ..#.. .##.. ..#.. ..#.. ..#.. ..#.. .###.
32 .###. #...# ....# ...#. ..#.. .#... #####
33 ##### ...#. ..#.. ...#. ....# #...# .###.
34 ...#. ..##. .#.#. #..#. ##### ...#. ...#.
35 ##### #.... ####. ....# ....# #...# .###.
36 ..##. .#... #.... ####. #...# #...# .###.
37 ##### ....# ...#. ..#.. .#... .#... .#...
38 .###. #...# #...# .###. #...# #...# .###.
39 .###. #...# #...# .#### ....# ...#. .##..
3A ..... .##.. .##.. ..... .##.. .##.. .....
3B ..... .##.. .##.. ..... .##.. ..#.. .#...
3C ...#. ..#.. .#... #.... .#... ..#.. ...#.
3D ..... ..... ##### ..... ##### ..... .....
3E .#... ..#.. ...#. ....# ...#. ..#.. .#...
3F .###. #...# ....# ...#. ..#.. ..... ..#..
40 .###. #...# ....# .##.# #.#.# #.#.# .###.
41 .###. #...# #...# ##### #...# #...# #...#
42 ####. #...# #...# ####. #...# #...# ####.
43 .###. #...# #.... #.... #.... #...# .###.
44 ####. #...# #...# #...# #...# #...# ####.
45 ##### #.... #.... ####. #.... #.... #####
46 ##### #.... #.... ####. #.... #.... #....
47 .###. #...# #.... #.### #...# #...# .####
48 #...# #...# #...# ##### #...# #...# #...#
49 .###. ..#.. ..#.. ..#.. ..#.. ..#.. .###.
4A ..### ...#. ...#. ...#. ...#. #..#. .##..
4B #...# #..#. #.#.. ##... #.#.. #..#. #...#
4C #.... #.... #.... #.... #.... #.... #####
4D #...# ##.## #.#.# #.#.# #...# #...# #...#
4E #...# #...# ##..# #.#.# #..## #...# #...#
4F .###. #...# #...# #...# #...# #...# .###.
50 ####. #...# #...# ####. #.... #.... #....
51 .###. #...# #...# #...# #.#.# #..#. .##.#
52 ####. #...# #...# ####. #.#.. #..#. #...#
53 .#### #.... #.... .###. ....# ....# ####.
54 ##### ..#.. ..#.. ..#.. ..#.. ..#.. ..#..
55 #...# #...# #...# #...# #...# #...# .###.
56 #...# #...# #...# #...# #...# .#.#. ..#..
57 #...# #...# #...# #.#.# #.#.# #.#.# .#.#.
58 #...# #...# .#.#. ..#.. .#.#. #...# #...#
59 #...# #...# .#.#. ..#.. ..#.. ..#.. ..#..
5A ##### ....# ...#. ..#.. .#... #.... #####
5B .###. .#... .#... .#... .#... .#... .###.
5C #.... .#... .#... ..#.. ...#. ...#. ....#
5D .###. ...#. ...#. ...#. ...#. ...#. .###.
5E ..#.. .#.#. #...# ..... ..... ..... .....
5F ..... ..... ..... ..... ..... ..... #####
"""


def _parse(text: str) -> Dict[str, Tuple[str, ...]]:
    font = {}
    for line in text.strip().splitlines():
        code, *rows = line.split()
        assert len(rows) == ROWS and all(len(r) == GLYPH_COLUMNS for r in rows), line
        font[chr(int(code, 16))] = tuple(rows)
    assert sorted(font) == [chr(c) for c in range(FIRST, LAST + 1)]
    return font


#: ASCII 0x20 .. 0x5F -> 7 rows of 5 (``#`` on), top row first.
FONT: Dict[str, Tuple[str, ...]] = _parse(_F)


# ------------------------------------------------------------- the encoder
def glyph_char(byte: int) -> Optional[str]:
    """The font character drawn for ``byte``: itself for 0x20 .. 0x5F,
    0x60 .. 0x7F folded onto 0x40 .. 0x5F (lower case as upper), None for
    any other byte (the transmitter sends a space for it and reports it)."""
    if FIRST <= byte <= LAST:
        return chr(byte)
    if 0x60 <= byte <= 0x7F:
        return chr(byte - 0x20)
    return None


def glyph_columns(ch: str) -> List[int]:
    """The 7 columns of one font character, left to right, bit 0 the
    bottom pixel; the last two are the blank columns between letters."""
    rows = FONT[ch]
    cols = []
    for x in range(GLYPH_COLUMNS):
        v = 0
        for y in range(ROWS):                   # y = 0 the top row
            if rows[y][x] == "#":
                v |= 1 << (ROWS - 1 - y)
        cols.append(v)
    return cols + [0] * (COLUMNS - GLYPH_COLUMNS)


def columns_of(data: Sequence[int]) -> List[int]:
    """The columns of a byte string as sent, a byte with no glyph as a space."""
    out: List[int] = []
    for b in data:
        out += glyph_columns(glyph_char(b) or " ")
    return out


def pixels_of(columns: Sequence[int]) -> List[int]:
    """One value a pixel slot: each column bottom pixel first."""
    return [(c >> y) & 1 for c in columns for y in range(ROWS)]


def half_pixels_of(columns: Sequence[int]) -> List[int]:
    """One value a half-pixel slot: each pixel twice (the two-pixel rule)."""
    return [p for p in pixels_of(columns) for _ in range(2)]


def font_words(ch: str) -> List[int]:
    """The firmware's three table words for one glyph (header of
    feldhell_tx.loom): columns 0 | 1 << 7, 2 | 3 << 7, each with a marker
    at bit 14, and column 4 with its marker at bit 7."""
    c = glyph_columns(ch)
    return [c[0] | c[1] << 7 | 1 << 14, c[2] | c[3] << 7 | 1 << 14, c[4] | 1 << 7]


def font_table() -> List[int]:
    """All 64 glyphs' words, ASCII 0x20 first: 192 words."""
    return [w for code in range(FIRST, LAST + 1) for w in font_words(chr(code))]


# ------------------------------------------------------------ the display
def render(columns: Sequence[int], on: str = "#", off: str = ".") -> List[str]:
    """The columns as the receiver draws them: 7 text rows, top first.
    (A real receiver prints two copies, one above the other, so that a
    slant from a speed error still shows one whole line; one is enough
    here, where the grid is known.)"""
    return ["".join(on if (c >> y) & 1 else off for c in columns)
            for y in range(ROWS - 1, -1, -1)]


# ----------------------------------------------------------- the receiver
def tone_slots(edges: Sequence[Tuple[int, int]], origin: int, slot: float,
               count: int) -> List[float]:
    """The fraction of each of ``count`` slots of ``slot`` clocks from
    ``origin`` in which the tone pin was high: the receiver's detector
    (tone energy a slot). ``edges`` are ``(cycle, level)`` changes of the
    pin, low before the first."""
    out = []
    for k in range(count):
        a, b = origin + k * slot, origin + (k + 1) * slot
        high, level, t = 0.0, 0, a
        for c, v in edges:
            if c <= a:
                level = v
                continue
            if c >= b:
                break
            if level:
                high += c - t
            level, t = v, c
        if level:
            high += b - t
        out.append(high / slot)
    return out


def receive(edges: Sequence[Tuple[int, int]], origin: int, half: float,
            n_columns: int, threshold: float = 0.25) -> Tuple[List[int], List[int]]:
    """Rebuild ``n_columns`` columns from the tone pin: a half-pixel slot is
    black when the tone is high for more than ``threshold`` of it (a square
    carrier is high half the time). Returns ``(columns, mismatched)``: the
    columns (a pixel black if either of its two half-pixels is) and the
    indices of pixels whose two half-pixels disagree (the two-pixel rule)."""
    fr = tone_slots(edges, origin, half, n_columns * ROWS * 2)
    bits = [1 if f > threshold else 0 for f in fr]
    cols, bad = [], []
    for x in range(n_columns):
        v = 0
        for y in range(ROWS):
            k = (x * ROWS + y) * 2
            if bits[k] != bits[k + 1]:
                bad.append(x * ROWS + y)
            if bits[k] or bits[k + 1]:
                v |= 1 << y
        cols.append(v)
    return cols, bad


# ------------------------------------------------------------ the timing
def runs_of(pixels: Sequence[int]) -> List[Tuple[int, int]]:
    """``[(level, pixels)]``: the key's marks and spaces in pixel units."""
    out: List[Tuple[int, int]] = []
    for p in pixels:
        if out and out[-1][0] == p:
            out[-1] = (p, out[-1][1] + 1)
        else:
            out.append((p, 1))
    return out


def check_key(edges: Sequence[Tuple[int, int]], origin: int, pixel: int,
              pixels: Sequence[int]) -> List[Tuple[int, int, int]]:
    """Hold the key line's edges to the pixel grid: from ``origin`` (the
    first pixel slot's start) every rise and fall must be exactly where
    the expected pixel stream changes level, to the clock. Returns the
    mismatches ``(index, got cycle, expected cycle)``."""
    want, t, level = [], origin, 0
    for p in pixels:
        if p != level:
            want.append((t, p))
            level = p
        t += pixel
    if level:
        want.append((t, 0))
    bad = []
    for k in range(max(len(want), len(edges))):
        g = edges[k] if k < len(edges) else (-1, -1)
        w = want[k] if k < len(want) else (-1, -1)
        if g != w:
            bad.append((k, g[0], w[0]))
    return bad
