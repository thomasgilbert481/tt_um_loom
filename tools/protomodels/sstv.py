"""Reference receiver for SSTV Martin M1 on one pin: the VIS header, the
line timing and the picture, measured from the pin's edges.

Written from the mode specifications, never from ``firmware/sstv_m1.loom``.

Sources (every figure below names one):

* [W] Wikipedia, "Slow-scan television", section "Header" (read
  2026-11-07; SECONDARY): "a 300-millisecond leader tone at 1,900 Hz", "a
  10 ms break at 1,200 Hz", a second 300 ms leader, "a start bit at 1,200
  Hz", "bits of 30 milliseconds", "7 data bits (LSB first; 1,100 Hz for 1,
  1,300 Hz for 0)", "An even parity bit follows, then a stop bit at 1,200
  Hz"; decimal 44 for Martin M1. Section "Scanlines": the frequency runs
  "between 1,500 and 2,300 Hz" with the brightness. The stop bit's length
  is not stated; it is read as one 30 ms bit like the others.
* [B] J. L. Barber, "Proposal for SSTV mode specifications", Dayton SSTV
  forum, 2000, Martin M1 table (SECONDARY; the document could not be
  fetched on 2026-11-07, barberdsp.com refused the connection, so every
  [B] figure is FROM MEMORY, UNCHECKED): sync 1200 Hz 4.862 ms, sync porch
  1500 Hz 0.572 ms, green, blue, red scans of 146.432 ms (320 pixels), a
  1500 Hz separator of 0.572 ms after each scan, line 446.446 ms, 256
  lines; 1500 Hz black, 2300 Hz white (also [W] for the range).

Method. A square wave's edges are its fundamental's zero crossings, and a
phase-continuous FM source makes the edge index ``n`` (one per half cycle)
a continuous, piecewise-linear function of time: ``n = a + 2 f t`` over a
segment of frequency ``f``. The receiver fits that line over each segment
(least squares on the edge times), which gives the segment's frequency to a
fraction of the edge jitter, and finds a boundary between two segments as
the intersection of their two lines, with a known short segment between
them (a porch) carried as its half-cycle count. Coarse positions come from
the half-period series (``1 / (2 * gap)``) against frequency thresholds.

Time scale: a test may compress every duration by ``scale`` (the
firmware's T: scale = 2^-T) and leave the frequencies alone; every
duration below is then multiplied by ``scale``.
"""

from __future__ import annotations

import dataclasses
from typing import List, Optional, Sequence, Tuple

from tools.protomodels.dtmf import EdgeRecorder  # noqa: F401  (re-exported)

# ------------------------------------------------------------- the mode
LEADER_HZ, LEADER_S = 1900.0, 0.300          # [W]
BREAK_HZ, BREAK_S = 1200.0, 0.010            # [W]
BIT_S = 0.030                                # [W]
START_HZ = STOP_HZ = 1200.0                  # [W]
ONE_HZ, ZERO_HZ = 1100.0, 1300.0             # [W]
VIS_MARTIN_M1 = 44                           # [W]
SYNC_HZ, SYNC_S = 1200.0, 4.862e-3           # [B] unchecked
PORCH_HZ, PORCH_S = 1500.0, 0.572e-3         # [B] unchecked
SEP_HZ, SEP_S = 1500.0, 0.572e-3             # [B] unchecked
SCAN_S = 146.432e-3                          # [B] unchecked
PIXELS, LINES = 320, 256                     # [B] unchecked
LINE_S = SYNC_S + PORCH_S + 3 * (SCAN_S + SEP_S)   # 446.446 ms [B]
BLACK_HZ, WHITE_HZ = 1500.0, 2300.0          # [B], [W]
ORDER = ("G", "B", "R")                      # [B] unchecked


def level_of(freq: float) -> float:
    """Brightness 0..255 of a scan frequency (linear, [B])."""
    return (freq - BLACK_HZ) * 255.0 / (WHITE_HZ - BLACK_HZ)


def freq_of(level: float) -> float:
    return BLACK_HZ + level * (WHITE_HZ - BLACK_HZ) / 255.0


def scan_offset(channel: int) -> float:
    """Start of channel ``channel``'s scan after the line's sync start."""
    return SYNC_S + PORCH_S + channel * (SCAN_S + SEP_S)


# ------------------------------------------------------------- the edges
def edge_times(edges: Sequence[Tuple[int, int]], clock: float,
               c0: int = 0) -> List[float]:
    """Edge times in seconds from cycle ``c0``."""
    return [(c - c0) / clock for c, _ in edges if c >= c0]


def half_periods(times: Sequence[float]) -> List[Tuple[float, float, float]]:
    """``(start, end, frequency)`` of each half cycle."""
    return [(a, b, 0.5 / (b - a)) for a, b in zip(times, times[1:]) if b > a]


def coarse(times: Sequence[float], t0: float, lo: float, hi: float,
           run: int = 1, width: int = 6) -> Optional[float]:
    """First edge at or after ``t0`` from which ``run`` windows in a row
    (each the mean frequency over ``width`` half cycles, so the sample
    grid's jitter averages out) all lie in ``[lo, hi]``."""
    import bisect
    i = bisect.bisect_left(times, t0)
    good = 0
    while i + width < len(times):
        f = width / (2.0 * (times[i + width] - times[i]))
        good = good + 1 if lo <= f <= hi else 0
        if good == run:
            return times[i - run + 1]
        i += 1
    return None


@dataclasses.dataclass
class Line:
    """``n = a + b * t`` over a window: b / 2 is the frequency."""
    a: float
    b: float
    count: int

    @property
    def freq(self) -> float:
        return self.b / 2.0


def fit(times: Sequence[float], t0: float, t1: float) -> Line:
    """Least-squares line through (time, edge index) for edges in [t0, t1]."""
    pts = [(t, n) for n, t in enumerate(times) if t0 <= t <= t1]
    if len(pts) < 3:
        raise ValueError("too few edges in %.6f .. %.6f s" % (t0, t1))
    k = len(pts)
    mt = sum(t for t, _ in pts) / k
    mn = sum(n for _, n in pts) / k
    stt = sum((t - mt) ** 2 for t, _ in pts)
    b = sum((t - mt) * (n - mn) for t, n in pts) / stt
    return Line(mn - b * mt, b, k)


def meet(first: Line, second: Line, gap_s: float = 0.0,
         gap_hz: float = 0.0) -> float:
    """The start of ``second``'s segment, when ``first``'s ends ``gap_s``
    before it and a segment of ``gap_hz`` fills the gap (phase continuous)."""
    num = first.a - first.b * gap_s + 2.0 * gap_hz * gap_s - second.a
    return num / (second.b - first.b)


def count_at(times: Sequence[float], t: float) -> float:
    """The edge index at time ``t``, linear between edges."""
    import bisect
    i = bisect.bisect_right(times, t) - 1
    if i < 0 or i + 1 >= len(times):
        raise ValueError("time %.6f s outside the capture" % t)
    return i + (t - times[i]) / (times[i + 1] - times[i])


def mean_freq(times: Sequence[float], t0: float, t1: float) -> float:
    """Mean frequency over [t0, t1] from the interpolated edge count."""
    return (count_at(times, t1) - count_at(times, t0)) / (2.0 * (t1 - t0))


def _inner(t0: float, t1: float, frac: float = 0.15) -> Tuple[float, float]:
    m = (t1 - t0) * frac
    return t0 + m, t1 - m


# ------------------------------------------------------------- the VIS
@dataclasses.dataclass
class Vis:
    code: int                   # the seven data bits, LSB first
    parity: int                 # the eighth bit
    start_s: float              # start of the start bit (precise)
    freqs: List[float]          # start, 7 data, parity, stop (Hz)
    leader_hz: float

    @property
    def parity_ok(self) -> bool:            # even parity [W]
        return (bin(self.code).count("1") + self.parity) % 2 == 0

    @property
    def stop_end_s(self) -> float:
        return self.start_s + 10 * BIT_S * self.scale

    scale: float = 1.0


def decode_vis(times: Sequence[float], scale: float = 1.0) -> Vis:
    """Find the two leaders, the break and the start bit, then read the
    ten 30 ms bits (each from a fit over its middle 70 %)."""
    run = max(4, int(LEADER_HZ * LEADER_S * scale))    # half the leader
    l1 = coarse(times, 0.0, 1750.0, 2050.0, run)
    if l1 is None:
        raise ValueError("no leader")
    b = coarse(times, l1, 900.0, 1600.0)
    l2 = coarse(times, b, 1750.0, 2050.0, run) if b is not None else None
    sb = coarse(times, l2, 900.0, 1600.0) if l2 is not None else None
    if sb is None:
        raise ValueError("no break, second leader or start bit")
    bit = BIT_S * scale
    m = min(0.002, 0.3 * bit)               # the coarse step's error
    lead = fit(times, sb - 0.5 * LEADER_S * scale, sb - m)
    first = fit(times, sb + m, sb + bit - 0.1 * bit)
    start = meet(lead, first)
    freqs = [fit(times, *_inner(start + i * bit, start + (i + 1) * bit)).freq
             for i in range(10)]
    bits = [1 if abs(f - ONE_HZ) < abs(f - ZERO_HZ) else 0 for f in freqs[1:9]]
    code = sum(v << i for i, v in enumerate(bits[:7]))
    return Vis(code, bits[7], start, freqs, lead.freq, scale)


# ------------------------------------------------------------- the lines
@dataclasses.dataclass
class LineResult:
    sync_start_s: Optional[float]   # None on line 0 (the stop bit is 1200 Hz too)
    scan_start_s: List[float]       # G, B, R scan starts (precise)
    bars: List[List[float]]         # [channel][bar] brightness 0..255


def decode_line(times: Sequence[float], sync_nominal: float, bars: int = 8,
                scale: float = 1.0, first: bool = False) -> LineResult:
    """One line whose sync nominally starts at ``sync_nominal``. Each scan
    must open with a bright bar (above mid-grey) for its start to be found;
    the scan's end is not observable when its last bar is black (1500 Hz,
    the separator's frequency), so the timing is read from the starts."""
    s = scale
    sync_s, porch_s = SYNC_S * s, PORCH_S * s
    bar_s = SCAN_S * s / bars
    starts: List[float] = []
    sync_start = None
    if not first:
        before = fit(times, *_inner(sync_nominal - bar_s, sync_nominal - SEP_S * s))
        sync = fit(times, *_inner(sync_nominal, sync_nominal + sync_s))
        sync_start = meet(before, sync)
    else:       # the stop bit and the sync are one 1200 Hz segment
        sync = fit(times, *_inner(sync_nominal - BIT_S * s, sync_nominal + sync_s))
    for c in range(3):
        g = sync_nominal + scan_offset(c) * s
        white = fit(times, *_inner(g, g + bar_s))
        if c == 0:
            starts.append(meet(sync, white, porch_s, PORCH_HZ))
        else:
            black = fit(times, *_inner(g - bar_s, g - SEP_S * s))
            starts.append(meet(black, white))
    levels = []
    for c in range(3):
        row = []
        for j in range(bars):
            b0 = starts[c] + j * bar_s
            row.append(level_of(fit(times, *_inner(b0, b0 + bar_s)).freq))
        levels.append(row)
    return LineResult(sync_start, starts, levels)


def rebuild(times: Sequence[float], line_starts: Sequence[float],
            scale: float = 1.0) -> List[List[Tuple[int, int, int]]]:
    """Every pixel's (R, G, B) from the mean frequency over its window, for
    lines whose G scans start at ``line_starts``."""
    px = SCAN_S * scale / PIXELS
    img = []
    for g in line_starts:
        row = []
        for p in range(PIXELS):
            rgb = []
            for c in (2, 0, 1):                 # R, G, B from G, B, R scans
                t0 = g + c * (SCAN_S + SEP_S) * scale + p * px
                v = level_of(mean_freq(times, t0, t0 + px))
                rgb.append(max(0, min(255, int(round(v)))))
            row.append(tuple(rgb))
        img.append(row)
    return img


def write_ppm(path, img) -> None:
    with open(path, "wb") as f:
        f.write(b"P6\n%d %d\n255\n" % (len(img[0]), len(img)))
        for row in img:
            f.write(bytes(v for px in row for v in px))
