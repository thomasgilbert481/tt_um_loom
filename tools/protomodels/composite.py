"""Monochrome composite video, NTSC-M (SMPTE 170M, the RS-170A timing): a
television model that decodes two Loom pins into the signal's levels,
recovers horizontal and vertical sync the way a set does, checks every
interval against the standard and rebuilds the picture as a small
greyscale frame.

Written from the standard, not from the firmware, for
``firmware/composite_ntsc.loom`` (W-103, D-035).

Source: SMPTE 170M-2004, "Composite Analog Video Signal - NTSC for Studio
Applications" (pub.smpte.org/pub/st170/st0170-2004_stable2010.pdf, read on
2026-11-08; the text pages extracted, Figure 7 read as an image):

======================  =================================================  =======
figure                  SMPTE 170M-2004                                    here
======================  =================================================  =======
subcarrier fsc          11.1: 315/88 MHz = 3.579545 MHz +-10 Hz            derived
line frequency fH       11.2: fH = 2/455 fsc = 15,734.265 Hz               checked
fH tolerance            none stated; +-10 Hz on fsc is +-2.79 ppm on fH    checked
line period             Table 2: 63.556 us (derived)                       checked
field frequency         11.3: 59.94 Hz, 525 lines a frame, 2:1 interlace   checked
H sync                  Table 2: 4.70 +-0.10 us (50 % points)              checked
front porch             Table 2: blanking start to H ref 1.5 +-0.1 us      checked
H blanking end          Table 2: H ref to blanking end 9.20 +0.20 -0.10    checked
rise times              Table 2: 140 +-20 ns                               not modelled
levels                  Table 1: white 100, setup 7.5, blank 0, sync -40   see below
IRE to volts            Annex B: 140 IRE = 1.0 V p-p                       see below
equalising pulse        Figure 7: 2.3 us (no tolerance printed)            checked
serration               Figure 7: 4.7 us (no tolerance printed)            checked
vertical interval       Figure 7: pre-equalising 3H, vertical sync 3H,     checked
                        post-equalising 3H, pulses every 0.5H (6 each)
field 1, field 2        Figure 7: field 1 starts at line 1's H ref,        checked
                        field 2 at 0.5H into line 263
vertical blanking       Figure 7: to the end of line 20 (field 1) and of   checked
                        line 282 (field 2); see LINE_283 below
======================  =================================================  =======

The H reference point is the 50 % point of the sync's falling edge
(Table 2); on two logic pins that is the clock at which SYNC falls.

Tolerances that are this model's choice, not 170M's: the equalising pulse
and the serration, which Figure 7 gives with no tolerance, are held to
+-0.10 us, as Table 2 holds the H sync (:data:`PULSE_TOL_S`); and every
sync edge must sit within :data:`JITTER_S` of the straight line fitted
through all of them (170M bounds only the subcarrier's jitter, 11.1, and
there is no subcarrier here). A real set's horizontal AFC and vertical
integrator tolerate far more of both.

Line 283 (field 2) and line 263 (field 1) are half lines: 170M's Figure 7
puts field 2's vertical blanking interval to line 282, and field 2 starts
half way through 263. This model allows picture only from half line 40 of
either field (line 21, and the middle of line 283), and up to
:data:`FP_MIN_S` before the next sync pulse of any kind, so the first half
of line 263 may carry picture; it rebuilds whole lines only (21 .. 262 and
284 .. 525).

Levels. Two pins and a resistor DAC (the firmware header gives the
resistors): SYNC low is the sync tip; SYNC high and VIDEO low the blanking
level; both high white. SYNC low with VIDEO high is no level of the
standard and is a violation. Black equals blanking (no 7.5 IRE setup):
the model cannot see the analogue levels, so it checks only that the
picture uses the two picture levels.

Spatial order (derived from Figure 7, not stated by it): field 1's line 21
starts 20 lines after its field's start and field 2's line 284 starts 20.5
lines after its own, so line 284 lies half a line spacing below line 21:
the frame interleaves 21, 284, 22, 285, ... from the top.
"""

from __future__ import annotations

import bisect
import dataclasses
from typing import Dict, List, Optional, Tuple

from .bench import Lines, Model, PinSpec, pad_of

CLOCK = 50e6                        # the bench's core clock, Hz
FSC = 315e6 / 88                    # 170M 11.1
FSC_TOL_HZ = 10.0                   # 170M 11.1
FH = 2 * FSC / 455                  # 170M 11.2: 15734.2657 Hz
FH_TOL = FSC_TOL_HZ / FSC           # 2.79e-6, the same fraction as fsc's
LINE_S = 1 / FH                     # 63.5556 us
HALF_S = LINE_S / 2
HSYNC_S, HSYNC_TOL_S = 4.70e-6, 0.10e-6         # Table 2
FP_S, FP_TOL_S = 1.5e-6, 0.1e-6                 # Table 2
BLANK_END_S = 9.20e-6                           # Table 2, +0.20 / -0.10
BLANK_END_MIN_S, BLANK_END_MAX_S = 9.10e-6, 9.40e-6
EQ_S, SERR_S = 2.3e-6, 4.7e-6                   # Figure 7
PULSE_TOL_S = 0.10e-6               # the model's choice for EQ and serration
FP_MIN_S = FP_S - FP_TOL_S          # white must end this long before a pulse
JITTER_S = 25e-9                    # the model's choice, see the docstring
HALVES = 525                        # half lines a field (262.5 lines)
VI_PULSES = 18                      # 6 pre-equalising, 6 broad, 6 post
PICTURE_HALF = 40                   # half line 40: line 21, or mid 283
WINDOW_S = LINE_S - BLANK_END_S - FP_S          # 52.856 us of picture
COLS = 64                           # samples a rebuilt line

SYNC_TIP, BLANK, WHITE, BAD = "sync", "blank", "white", "bad"

#: The firmware's pins: SYNC on uo_out[0], VIDEO on uo_out[1].
DEFAULT_PINS = {"SYNC": "OUT0", "VIDEO": "OUT1"}


def level(sync: int, video: int) -> str:
    """The signal level two pins make through the resistor DAC."""
    if not sync:
        return BAD if video else SYNC_TIP
    return WHITE if video else BLANK


@dataclasses.dataclass
class Pulse:
    """One sync pulse: falls and rise in clocks, its kind ("H" line sync,
    "E" equalising, "B" broad), its field and half line in that field."""

    fall: int
    rise: int
    kind: str
    field: Optional[int] = None
    half: Optional[int] = None

    @property
    def width(self) -> int:
        return self.rise - self.fall


@dataclasses.dataclass
class Field:
    """A field as the set locks to it: the fall of its first equalising
    pulse, and 1 or 2 (None until the model can tell)."""

    number: int
    start: int
    parity: Optional[int] = None


def line_of(parity: int, half: int) -> int:
    """The 170M line number whose H ref is half line ``half`` of a field."""
    return 1 + half // 2 if parity == 1 else 263 + (half + 1) // 2


def kind_expected(parity: int, half: int) -> Optional[str]:
    """The pulse 170M Figure 7 puts at half line ``half`` of a field, or None."""
    if half < 6 or 12 <= half < VI_PULSES:
        return "E"
    if half < 12:
        return "B"
    return "H" if (half - (parity - 1)) % 2 == 0 else None


class CompositeMonitor(Model):
    """A television on the two pins: separates sync, sorts each pulse by
    width, finds each field from its equalising run, checks every pulse's
    place and width and every stretch of white against SMPTE 170M, and
    keeps what it needs to rebuild the picture (:meth:`picture`)."""

    MAX_RECORDED = 40
    EQ_MAX_S, H_MAX_S = 3.5e-6, 20e-6           # the sync separator's bins

    def __init__(self, pins: Optional[Dict[str, PinSpec]] = None,
                 clock: float = CLOCK) -> None:
        pins = dict(DEFAULT_PINS if pins is None else pins)
        self.sync_pad = pad_of(pins["SYNC"])
        self.video_pad = pad_of(pins["VIDEO"])
        self.clock = clock
        self.half_clk = HALF_S * clock
        self.pulses: List[Pulse] = []
        self.fields: List[Field] = []
        self.violations: List[str] = []
        self.white_s: List[int] = []            # white stretches, [start, end)
        self.white_e: List[int] = []
        self.white_runs: List[Tuple[int, int, float, float]] = []
        self.serrations: List[int] = []         # clocks high after a broad pulse
        self.bad_level = 0
        self.cycles = 0
        self._sync = self._video = None
        self._fall: Optional[int] = None
        self._white_from: Optional[int] = None
        self._queue: List[Tuple[int, int]] = []

    def _bad(self, cycle: float, text: str) -> None:
        if len(self.violations) < self.MAX_RECORDED:
            self.violations.append("cycle %d: %s" % (cycle, text))

    def _us(self, clocks: float) -> float:
        return clocks / self.clock * 1e6

    def _within(self, clocks: float, nominal_s: float, lo_s: float,
                hi_s: float) -> bool:
        s = clocks / self.clock
        return nominal_s - lo_s - 1e-12 <= s <= nominal_s + hi_s + 1e-12

    # ---------------------------------------------------------- per clock
    def observe(self, lines: Lines) -> None:
        c = lines.cycle
        self.cycles += 1
        sync, video = lines.get(self.sync_pad), lines.get(self.video_pad)
        if self._sync is not None:
            if self._sync and not sync:
                self._on_fall(c)
            elif sync and not self._sync:
                self._on_rise(c)
        lv = level(sync, video)
        if lv == BAD and level(self._sync or 0, self._video or 0) != BAD:
            self.bad_level += 1
            self._bad(c, "VIDEO high while SYNC is low: no level of the signal")
        if lv == WHITE and self._white_from is None:
            self._white_from = c
        elif lv != WHITE and self._white_from is not None:
            self._queue.append((self._white_from, c))
            self.white_s.append(self._white_from)
            self.white_e.append(c)
            self._white_from = None
        self._sync, self._video = sync, video

    # ---------------------------------------------------------- sync edges
    def _on_fall(self, c: int) -> None:
        last = self.pulses[-1] if self.pulses else None
        if last is not None and last.kind == "B":
            high = c - last.rise
            self.serrations.append(high)
            if not self._within(high, SERR_S, PULSE_TOL_S, PULSE_TOL_S):
                self._bad(c, "serration %.3f us, 170M Figure 7 %.1f us"
                          % (self._us(high), SERR_S * 1e6))
        while self._queue and self._queue[0][1] <= c:
            self._check_white(*self._queue.pop(0), nxt=c)
        self._fall = c

    def _on_rise(self, c: int) -> None:
        if self._fall is None:
            return
        w = (c - self._fall) / self.clock
        kind = "E" if w < self.EQ_MAX_S else "H" if w < self.H_MAX_S else "B"
        p = Pulse(self._fall, c, kind)
        if kind == "H" and not self._within(p.width, HSYNC_S, HSYNC_TOL_S, HSYNC_TOL_S):
            self._bad(c, "H sync %.3f us, 170M Table 2 4.70 +-0.10 us" % self._us(p.width))
        if kind == "E" and not self._within(p.width, EQ_S, PULSE_TOL_S, PULSE_TOL_S):
            self._bad(c, "equalising pulse %.3f us, 170M Figure 7 2.3 us"
                      % self._us(p.width))
        self._place(p)
        self.pulses.append(p)

    def _halves(self, a: float, b: float) -> int:
        return round((b - a) / self.half_clk)

    def _place(self, p: Pulse) -> None:
        """Give the pulse its field and half line, and check it is the pulse
        170M Figure 7 puts there."""
        prev = self.pulses[-1] if self.pulses else None
        cur = self.fields[-1] if self.fields else None
        if p.kind == "E" and (prev is None or prev.kind == "H"):
            gap = None if prev is None else self._halves(prev.fall, p.fall)
            parity = {2: 1, 1: 2}.get(gap) if gap is not None else None
            if gap is not None and parity is None:
                self._bad(p.fall, "%d half lines from the last H sync to the "
                          "vertical interval (1 or 2)" % gap)
            if cur is not None:
                n = self._halves(cur.start, p.fall)
                if n != HALVES:
                    self._bad(p.fall, "field of %d half lines, 170M 525" % n)
                if cur.parity is not None and parity == cur.parity:
                    self._bad(p.fall, "two fields %d in a row: no interlace" % parity)
            cur = Field(len(self.fields), p.fall, parity)
            self.fields.append(cur)
        if cur is None:
            return                              # not yet locked
        p.field, p.half = cur.number, self._halves(cur.start, p.fall)
        if cur.parity is None and p.half >= VI_PULSES:
            cur.parity = 1 if p.half % 2 == 0 else 2
        want = kind_expected(cur.parity or 1, p.half)
        known = p.half < VI_PULSES or cur.parity is not None
        if p.half > HALVES or (known and want != p.kind):
            self._bad(p.fall, "%s pulse at half line %d of field %d, 170M "
                      "Figure 7 has %s" % (p.kind, p.half, cur.number, want))
        if prev is not None and prev.field == cur.number:
            step = 1 if prev.half < VI_PULSES - 1 else 2
            if prev.half == VI_PULSES - 1:
                step = 1 if cur.parity == 1 else 2
            if p.half - prev.half != step:
                self._bad(p.fall, "half line %d follows %d in field %d"
                          % (p.half, prev.half, cur.number))

    # ---------------------------------------------------------- picture
    def _check_white(self, s: int, e: int, nxt: int) -> None:
        """A stretch of white [s, e), the next pulse falling at ``nxt``."""
        i = bisect.bisect_right([q.fall for q in self.pulses[-4:]], s) - 1
        ref = self.pulses[-4:][i] if i >= 0 else None
        if ref is None or ref.field is None:
            self._bad(s, "white before the set has locked to a field")
            return
        start = self.fields[ref.field].start
        after, before = (s - ref.fall) / self.clock, (nxt - e) / self.clock
        self.white_runs.append((s, e, after * 1e6, before * 1e6))
        if (s - start) / self.half_clk < PICTURE_HALF:
            self._bad(s, "white in the vertical blanking (half line %.1f of "
                      "field %d)" % ((s - start) / self.half_clk, ref.field))
        if ref.kind != "H" or after < BLANK_END_MIN_S - 1e-12:
            self._bad(s, "white %.3f us after a %s pulse, 170M Table 2: none "
                      "before 9.10 us" % (after * 1e6, ref.kind))
        if before < FP_MIN_S - 1e-12:
            self._bad(e, "white ends %.3f us before the next sync, 170M "
                      "Table 2: 1.5 +-0.1 us" % (before * 1e6))

    def is_white(self, cycle: float) -> bool:
        i = bisect.bisect_right(self.white_s, cycle) - 1
        return i >= 0 and cycle < self.white_e[i]

    def line_refs(self, field: int) -> Dict[int, int]:
        """``{line: H ref clock}`` of the whole lines of a field."""
        f = self.fields[field]
        return {line_of(f.parity, p.half): p.fall for p in self.pulses
                if p.field == field and p.kind == "H" and f.parity is not None}

    def picture(self, field: int, cols: int = COLS) -> Dict[int, List[int]]:
        """The picture lines of a field, ``{line: [0 black / 1 white] * cols}``,
        each sampled at the centres of ``cols`` columns of the 52.856 us
        window from 9.20 us after its H ref; only lines whose next H ref
        has been seen (so the whole line has run)."""
        refs = self.line_refs(field)
        first = 21 if self.fields[field].parity == 1 else 284
        out: Dict[int, List[int]] = {}
        for line in range(first, first + 242):
            if line not in refs or line + 1 not in refs and line != first + 241:
                continue
            t0 = refs[line] + BLANK_END_S * self.clock
            step = WINDOW_S * self.clock / cols
            out[line] = [int(self.is_white(t0 + (i + 0.5) * step)) for i in range(cols)]
        return out

    def frame(self, field: int, cols: int = COLS) -> List[List[int]]:
        """Fields ``field`` (a field 1) and the next interleaved from the top:
        21, 284, 22, 285, ... (module docstring)."""
        a, b = self.picture(field, cols), self.picture(field + 1, cols)
        rows: List[List[int]] = []
        for i in range(242):
            rows += [a[21 + i], b[284 + i]]
        return rows

    # ---------------------------------------------------------- results
    def timing(self) -> Dict[str, float]:
        """Fit a straight line through every placed pulse's fall against its
        half line counted from the first field: fH, its error from 170M's
        2/455 fsc in ppm, and the largest residual in ns. Records a
        violation when either is outside its limit; the fH limit is the
        +-2.79 ppm of :data:`FH_TOL` plus the fit's own resolution (two
        clocks over the span), which is wider than FH_TOL for a run much
        shorter than a field."""
        pts = [(self.fields[p.field].start, p) for p in self.pulses if p.field is not None]
        if len(self.fields) < 1 or len(pts) < 3:
            return {}
        base = self.fields[0].start
        xs = [self._halves(base, s) + p.half for s, p in pts]
        ys = [p.fall for _, p in pts]
        n = len(xs)
        mx, my = sum(xs) / n, sum(ys) / n
        sxx = sum((x - mx) ** 2 for x in xs)
        slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
        res = max(abs(y - (my + slope * (x - mx))) for x, y in zip(xs, ys))
        fh = self.clock / (2 * slope)
        ppm = (fh / FH - 1) * 1e6
        # the fit's own resolution: two clocks of edge quantisation over the span
        lim = FH_TOL * 1e6 + 2 / max(ys[-1] - ys[0], 1) * 1e6
        out = {"fh": fh, "ppm": ppm, "ppm_limit": lim,
               "jitter_ns": res / self.clock * 1e9,
               "line_us": 2 * slope / self.clock * 1e6}
        if abs(ppm) > lim:
            self._bad(ys[-1], "fH %.3f Hz, %+.2f ppm from 170M's %.3f (+-%.2f "
                      "with the fit's resolution)" % (fh, ppm, FH, lim))
        if res > JITTER_S * self.clock:
            self._bad(ys[-1], "a sync edge %.1f ns off the fitted grid" % out["jitter_ns"])
        return out

    def widths_us(self, kind: str) -> List[float]:
        """Every distinct low width of pulses of one kind, microseconds."""
        return sorted({round(self._us(p.width), 3) for p in self.pulses if p.kind == kind})

    def report(self) -> str:
        t = self.timing()
        return ("%d clocks, %d pulses (H %s, E %s, B %s us; serrations %s us), "
                "fields %s, timing %s, %d white runs, violations: %s"
                % (self.cycles, len(self.pulses), self.widths_us("H"),
                   self.widths_us("E"), self.widths_us("B"),
                   sorted({round(self._us(s), 3) for s in self.serrations}),
                   [(f.number, f.start, f.parity) for f in self.fields],
                   {k: round(v, 3) for k, v in t.items()},
                   len(self.white_runs), self.violations or "none"))
