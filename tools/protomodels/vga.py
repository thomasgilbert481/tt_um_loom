"""VGA 640x480 at 60 Hz: a monitor model that times the sync pins against
VESA DMT and samples the colour pins at every pixel to build the frame.

Written from the standard and the Pmod's documentation, not from the
firmware, for ``firmware/vga_sync.loom`` and ``firmware/vga_bars.loom``
(W-068, D-035).

Timing: VESA Display Monitor Timing Standard, Version 1.0 Rev. 13 (2013),
page 21, the 640 x 480 at 60 Hz entry (DMT ID 04h; the page marks it "For
Reference Only - Not a VESA Standard"). Read from the PDF on 2026-11-07:

=====================  ===========================================  =======
figure                 DMT page 21                                  here
=====================  ===========================================  =======
pixel clock            25.175 MHz +-0.5 %                           checked
line                   31.778 us = 800 px, 31.469 kHz               checked
active (H addr)        640 px                                       checked
right border, front    8 px, 8 px (HSYNC starts at 656 px)          checked
HSYNC                  96 px, NEGATIVE                              checked
back porch, left bdr   40 px, 8 px                                  checked
frame                  525 lines, 59.940 Hz                         checked
active (V addr)        480 lines                                    checked
bottom border, front   8 lines, 2 lines (VSYNC starts at line 490)  checked
VSYNC                  2 lines, NEGATIVE                            checked
back porch, top bdr    25 lines, 8 lines                            checked
=====================  ===========================================  =======

DMT's borders are part of the blanking for a source that draws none, so
the model folds them into the porches, the usual 16 / 96 / 48 pixels and
10 / 2 / 33 lines (:data:`H_TIMING`, :data:`V_TIMING`). The +-0.5 % is the
only tolerance DMT gives; the porches are whole pixels of that clock. The
monitor here locks its pixel clock to the measured line (800 pixels a
line, as a real monitor's PLL does), so it judges every width in pixels of
the clock the source actually ran: :data:`H_SLACK_PX`, the slack it allows
on a sync width, is this model's choice, not DMT's.

Pinout: the TinyVGA Pmod. Tiny Tapeout's pinout page (tinytapeout.com/
specs/pinouts, section "VGA Output", read 2026-11-07) gives uo_out[0..7] =
R1, G1, B1, VSYNC, R0, G0, B0, HSYNC; the Pmod's own README (github.com/
mole99/tiny-vga, "Tiny VGA Pinout", read 2026-11-07) gives the same order on
Pmod pins 1-4 and 7-10, R1/G1/B1 the most significant bits. Loom's uo_out[6]
and [7] are the host port, so on Loom the Pmod sits on the uio header in
the same order (:data:`TINYVGA_ON_UIO`).

A channel's level is ``2 * X1 + X0`` (0 .. 3).
"""

from __future__ import annotations

import dataclasses
from typing import Dict, List, Optional, Sequence, Tuple

from .bench import Lines, Model, PinSpec, pad_of

CLOCK = 50e6                        # the bench's core clock, Hz
PIXEL_HZ = 25.175e6                 # DMT p. 21
PIXEL_TOL = 0.005                   # +-0.5 %, DMT p. 21
#: (active, front porch, sync, back porch) in pixels, borders folded in
H_TIMING = (640, 16, 96, 48)
#: (active, front porch, sync, back porch) in lines, borders folded in
V_TIMING = (480, 10, 2, 33)
H_TOTAL = sum(H_TIMING)             # 800
V_TOTAL = sum(V_TIMING)             # 525
LINE_S = H_TOTAL / PIXEL_HZ         # 31.778 us
H_SLACK_PX = 1.0                    # the model's slack on a sync width

#: The TinyVGA Pmod's signals on Loom's uio header (bit i = BIDIRi).
TINYVGA_ON_UIO = {"R1": "BIDIR0", "G1": "BIDIR1", "B1": "BIDIR2",
                  "VSYNC": "BIDIR3", "R0": "BIDIR4", "G0": "BIDIR5",
                  "B0": "BIDIR6", "HSYNC": "BIDIR7"}

Colour = Tuple[int, int, int]       # (r, g, b) levels 0 .. 3
BLACK: Colour = (0, 0, 0)


@dataclasses.dataclass
class Frame:
    """One frame: 480 rows of 640 sampled pixels."""

    number: int
    rows: List[List[Colour]]

    def runs(self, row: int) -> List[Tuple[int, int, Colour]]:
        """``[(first pixel, length, colour)]`` of one row."""
        out: List[Tuple[int, int, Colour]] = []
        for x, c in enumerate(self.rows[row]):
            if out and out[-1][2] == c:
                out[-1] = (out[-1][0], out[-1][1] + 1, c)
            else:
                out.append((x, 1, c))
        return out

    def ppm(self) -> bytes:
        """The frame as a binary PPM (P6), levels scaled to 0 .. 255."""
        body = bytearray()
        for row in self.rows:
            for r, g, b in row:
                body += bytes((r * 85, g * 85, b * 85))
        head = b"P6\n%d %d\n255\n" % (len(self.rows[0]), len(self.rows))
        return head + bytes(body)


def levels(word: int, bits: Dict[str, int]) -> Colour:
    """The (r, g, b) levels of a pad word, ``bits`` mapping R1 .. B0 to a bit."""
    def ch(x: str) -> int:
        return 2 * ((word >> bits[x + "1"]) & 1) + ((word >> bits[x + "0"]) & 1)
    return ch("R"), ch("G"), ch("B")


class VgaMonitor(Model):
    """A monitor on the sync and colour pins: times every edge against DMT,
    locks a pixel clock of 800 pixels a line to HSYNC, and samples the colour
    at the centre of every active pixel.

    Line ``i`` is the one that starts ``H_TIMING[3] + H_TIMING[2]`` (144)
    pixels after the ``i``-th HSYNC fall, its pixel period the period of
    the line before over 800 (the nominal one for the first). Active row
    ``k`` of a frame is line ``v + 35 + k`` (2 sync + 33 back porch lines),
    ``v`` the line at whose start VSYNC fell. Colour anywhere outside the
    640-pixel window of an active row, on any clock, is a blanking
    violation.
    """

    MAX_RECORDED = 40

    def __init__(self, pins: Optional[Dict[str, PinSpec]] = None,
                 clock: float = CLOCK) -> None:
        pins = dict(TINYVGA_ON_UIO if pins is None else pins)
        self.pads = {name: pad_of(pin) for name, pin in pins.items()}
        ports = {p for p, _ in self.pads.values()}
        if len(ports) != 1:
            raise ValueError("the model wants the eight pins on one port")
        self.port = ports.pop()
        self.bits = {name: bit for name, (_, bit) in self.pads.items()}
        self.clock = clock
        self.h_falls: List[int] = []
        self.h_low: List[int] = []              # clocks, per HSYNC pulse
        self.starts: List[float] = []           # line i's start, in clocks
        self.tpix: List[float] = []             # line i's pixel period
        self.v_falls: List[int] = []            # line index of each VSYNC fall
        self.v_low_lines: List[int] = []
        self.vs_offset: List[float] = []        # VSYNC edge - line start, clocks
        self.frames: List[Frame] = []
        self.violations: List[str] = []
        self.blank_clocks = 0                   # clocks of colour in blanking
        self.cycles = 0
        self.colour_edges: List[Tuple[int, Colour]] = []   # (cycle, new colour)
        self._colour = BLACK
        self._hs = self._vs = None
        self._cur = -1
        self._next_px = 0
        self._rows: Optional[List[List[Colour]]] = None
        self._row: List[Colour] = []
        self._vs_fall_line: Optional[int] = None

    # ---------------------------------------------------------- helpers
    def _bad(self, cycle: int, text: str) -> None:
        if len(self.violations) < self.MAX_RECORDED:
            self.violations.append("cycle %d: %s" % (cycle, text))

    def _nearest_line(self, cycle: int) -> Tuple[int, float]:
        best = min(range(max(0, len(self.starts) - 3), len(self.starts)),
                   key=lambda i: abs(self.starts[i] - cycle))
        return best, cycle - self.starts[best]

    def active_row(self, line: int) -> Optional[int]:
        """The frame row line ``line`` carries, or None in the blanking."""
        if self._vs_fall_line is None:
            return None
        k = line - self._vs_fall_line - V_TIMING[2] - V_TIMING[3]
        return k if 0 <= k < V_TIMING[0] else None

    # ---------------------------------------------------------- edges
    def _hsync_fall(self, c: int) -> None:
        nominal = self.clock / PIXEL_HZ
        tp = nominal
        if self.h_falls:
            period = c - self.h_falls[-1]
            tp = period / H_TOTAL
            err = period / self.clock / LINE_S - 1
            if abs(err) > PIXEL_TOL:
                self._bad(c, "line %.3f us, %+.3f %% from DMT's %.3f us (+-0.5 %%)"
                          % (period / self.clock * 1e6, err * 100, LINE_S * 1e6))
        self.h_falls.append(c)
        self.starts.append(c + (H_TIMING[2] + H_TIMING[3]) * tp)
        self.tpix.append(tp)

    def _hsync_rise(self, c: int) -> None:
        if not self.h_falls:
            return
        low = c - self.h_falls[-1]
        self.h_low.append(low)
        px = low / self.tpix[-1]
        if abs(px - H_TIMING[2]) > H_SLACK_PX:
            self._bad(c, "HSYNC low %d clocks = %.2f px, DMT %d (negative)"
                      % (low, px, H_TIMING[2]))

    def _vsync_fall(self, c: int) -> None:
        if not self.starts:
            return
        line, off = self._nearest_line(c)
        self.vs_offset.append(off)
        if self.v_falls and line - self.v_falls[-1] != V_TOTAL:
            self._bad(c, "frame of %d lines, DMT %d" % (line - self.v_falls[-1], V_TOTAL))
        self.v_falls.append(line)
        self._vs_fall_line = line
        self._rows = []

    def _vsync_rise(self, c: int) -> None:
        if not self.starts or self._vs_fall_line is None:
            return
        line, off = self._nearest_line(c)
        self.vs_offset.append(off)
        lines = line - self._vs_fall_line
        self.v_low_lines.append(lines)
        if lines != V_TIMING[2]:
            self._bad(c, "VSYNC low %d lines, DMT %d (negative)" % (lines, V_TIMING[2]))

    # ---------------------------------------------------------- per clock
    def observe(self, lines: Lines) -> None:
        c = lines.cycle
        self.cycles += 1
        word = getattr(lines, self.port)
        hs = (word >> self.bits["HSYNC"]) & 1
        vs = (word >> self.bits["VSYNC"]) & 1
        if self._hs is not None:
            if self._hs and not hs:
                self._hsync_fall(c)
            elif hs and not self._hs:
                self._hsync_rise(c)
            if self._vs and not vs:
                self._vsync_fall(c)
            elif vs and not self._vs:
                self._vsync_rise(c)
        self._hs, self._vs = hs, vs
        colour = levels(word, self.bits)
        if colour != self._colour:
            self.colour_edges.append((c, colour))
            self._colour = colour
        while self._cur + 1 < len(self.starts) and c >= self.starts[self._cur + 1]:
            self._end_line()
            self._cur += 1
            self._next_px = 0
            self._row = []
        if self._cur < 0:
            return
        start, tp = self.starts[self._cur], self.tpix[self._cur]
        row = self.active_row(self._cur)
        locked = self._vs_fall_line is not None
        in_window = c < start + H_TIMING[0] * tp and (row is not None or not locked)
        if colour != BLACK and not in_window:
            self.blank_clocks += 1
            self._bad(c, "colour %r in the blanking (line %d, %.1f px into it)"
                      % (colour, self._cur, (c - start) / tp))
        if row is None or self._rows is None:
            return
        while (self._next_px < H_TIMING[0]
               and round(start + (self._next_px + 0.5) * tp) <= c):
            self._row.append(colour)
            self._next_px += 1

    def _end_line(self) -> None:
        """Close line ``_cur``: file its row, and the frame after row 479."""
        if self._cur < 0 or self._rows is None:
            return
        row = self.active_row(self._cur)
        if row is None:
            return
        if len(self._row) != H_TIMING[0]:
            self._bad(self.starts[self._cur], "row %d has %d pixels"
                      % (row, len(self._row)))
        self._rows.append(self._row)
        if row == V_TIMING[0] - 1:
            self.frames.append(Frame(len(self.frames), self._rows))
            self._rows = None

    # ---------------------------------------------------------- results
    def rows(self) -> List[List[Colour]]:
        """The finished rows of the frame being drawn (or the last frame)."""
        if self._rows:
            return list(self._rows)
        return list(self.frames[-1].rows) if self.frames else []

    def line_us(self) -> List[float]:
        """Every measured line period, in microseconds."""
        f = self.h_falls
        return [(b - a) / self.clock * 1e6 for a, b in zip(f, f[1:])]

    def report(self) -> str:
        periods = sorted(set(b - a for a, b in zip(self.h_falls, self.h_falls[1:])))
        return ("%d clocks, %d HSYNC falls (line periods %s clocks), HSYNC low "
                "%s clocks, VSYNC at lines %s (low %s lines, edges %s clocks "
                "from a line start), %d frames, %d clocks of colour in the "
                "blanking, violations: %s"
                % (self.cycles, len(self.h_falls), periods,
                   sorted(set(self.h_low)), self.v_falls, self.v_low_lines,
                   sorted(set(round(o, 2) for o in self.vs_offset)),
                   len(self.frames), self.blank_clocks,
                   self.violations or "none"))
