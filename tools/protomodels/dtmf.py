"""DTMF (multifrequency push-button) reference models: a pin recorder, a
decimating filter that turns a one-bit pulse-density stream into audio
samples, a two-tone measurement (frequencies, levels, twist, distortion)
and a tone/pause timer, checked against ITU-T Q.23 and Q.24; and a one-bit
DTMF source (a comparator on the line) with a single-bin DFT, which backs
the finding that a one-bit Goertzel detector cannot meet Q.24.

Written from the specifications, not from ``firmware/dtmf_gen.loom``. Pure
Python: the test Python of this repo has no numpy, so the spectral
analysis is a direct DFT at chosen frequencies and a least-squares fit.

Sources (both read on 2026-11-07, the ITU-T Blue Book extracts from
itu.int, T-REC-Q.23-198811-I and T-REC-Q.24-198811-I):

* ITU-T Q.23 (11/1988), "Technical features of push-button telephone sets":
  section 5, two frequencies sent at once, one from each of two groups of
  four (the "2 (1/4) code"); section 6, low group 697, 770, 852, 941 Hz and
  high group 1209, 1336, 1477, 1633 Hz, the keys as Figure 1/Q.23 (rows the
  low group, columns the high group: 1 2 3 A / 4 5 6 B / 7 8 9 C / * 0 # D);
  section 7.1, each transmitted frequency within +-1.8 % of nominal;
  section 7.2, the total distortion products (harmonics or
  intermodulation) at least 20 dB below the fundamentals; section 8, the
  sending levels are national matters (not standardised).
* ITU-T Q.24 (11/1988), "Multifrequency push-button signal reception":
  section 2.2, the receiver checks for one and only one frequency of each
  group; sections 2.3 to 2.5 name the parameters, and Annex A, Table
  A-1/Q.24 gives the values adopted by five Administrations. This module
  uses the AT&T column: frequency deviation operate <= 1.5 %, non-operate
  >= 3.5 %; level difference between the two frequencies +4 dB to -8 dB
  (note b: the high group may be up to 4 dB above or 8 dB below the low
  group); signal duration operate >= 40 ms, non-operate <= 23 ms; pause
  >= 40 ms; signal interruption <= 10 ms; signalling velocity >= 93 ms a
  digit. (The other columns: NTT 1.8 % / 3.0 %, Danish 1.5 % + 2 Hz,
  Australian 1.5 % + 4 Hz / 7 %, Brazilian 1.8 % / 3 %; every column has
  40 ms operate.)
* The Goertzel algorithm: G. Goertzel, "An algorithm for the evaluation of
  finite trigonometric series", American Mathematical Monthly 65 (1958);
  any DSP textbook (Oppenheim and Schafer, "Discrete-Time Signal
  Processing", the section on the Goertzel algorithm).
"""

from __future__ import annotations

import cmath
import math
from typing import List, Optional, Sequence, Tuple

from tools.protomodels.bench import Lines, Model, PinSpec, pad_of

# ----------------------------------------------------- Q.23 and Q.24 values
LOW_HZ = (697.0, 770.0, 852.0, 941.0)           # Q.23 section 6
HIGH_HZ = (1209.0, 1336.0, 1477.0, 1633.0)      # Q.23 section 6
KEYS = "123A456B789C*0#D"                       # Figure 1/Q.23, row by row
SEND_TOLERANCE = 0.018                          # Q.23 section 7.1
DISTORTION_DB = 20.0                            # Q.23 section 7.2
OPERATE_TOLERANCE = 0.015                       # Table A-1/Q.24, AT&T
NON_OPERATE_TOLERANCE = 0.035                   # Table A-1/Q.24, AT&T
TWIST_DB = (-8.0, 4.0)                          # high minus low, AT&T note b
MIN_TONE_S = 0.040                              # Table A-1/Q.24: operate
MAX_REJECT_S = 0.023                            # Table A-1/Q.24: non-operate
MIN_PAUSE_S = 0.040                             # Table A-1/Q.24, AT&T
VOICE_BAND_HZ = (300.0, 3400.0)                 # where distortion is summed


def key_index(key: str) -> int:
    """4 * row + column of Figure 1/Q.23 for a key character."""
    return KEYS.index(key)


def key_freqs(index: int) -> Tuple[float, float]:
    """The nominal (low, high) frequencies of key ``index`` (Q.23 s. 6)."""
    return LOW_HZ[index >> 2], HIGH_HZ[index & 3]


# ------------------------------------------------------------- recording
class EdgeRecorder(Model):
    """Every change of one pad: ``edges`` holds ``(cycle, level)``, the cycle
    being the first with the new level; the level before is ``initial``."""

    def __init__(self, pin: PinSpec, initial: int = 0) -> None:
        self.pad = pad_of(pin)
        self.initial = self.level = initial
        self.edges: List[Tuple[int, int]] = []

    def observe(self, lines: Lines) -> None:
        level = lines.get(self.pad)
        if level != self.level:
            self.edges.append((lines.cycle, level))
            self.level = level


# ------------------------------------------------------------- filtering
def window_means(edges: Sequence[Tuple[int, int]], initial: int, start: float,
                 width: float, count: int) -> List[float]:
    """The pin's mean level over ``count`` back-to-back windows of ``width``
    clocks from cycle ``start``: the exact integral of the waveform the
    edges describe (a boxcar, the first stage of a decimating filter)."""
    out: List[float] = []
    level, i = initial, 0
    while i < len(edges) and edges[i][0] <= start:
        level = edges[i][1]
        i += 1
    t = start
    for w in range(count):
        end = start + (w + 1) * width
        acc = 0.0
        while i < len(edges) and edges[i][0] < end:
            acc += level * (edges[i][0] - t)
            t, level = edges[i][0], edges[i][1]
            i += 1
        acc += level * (end - t)
        t = end
        out.append(acc / width)
    return out


def boxcar(xs: Sequence[float], n: int) -> List[float]:
    """Moving average of length ``n`` (output ``i`` averages ``xs[i..i+n-1]``)."""
    s = sum(xs[:n])
    out = [s / n]
    for i in range(n, len(xs)):
        s += xs[i] - xs[i - n]
        out.append(s / n)
    return out


#: The decimating filter: windows of 1/62 500 s, two moving averages of 4
#: windows, every 4th output kept. Its nulls fall on the multiples of
#: 15 625 Hz and 62 500 Hz, where a first-order modulator's noise and a
#: square wave's harmonics would otherwise fold into the voice band.
STAGE_HZ = 62_500.0
DECIMATE = 4
FS_AUDIO = STAGE_HZ / DECIMATE                  # 15 625 Hz


def filter_gain(f: float) -> float:
    """The amplitude response of :func:`decimate` at ``f`` Hz."""
    if f == 0:
        return 1.0
    x = math.pi * f / STAGE_HZ
    box = math.sin(DECIMATE * x) / (DECIMATE * math.sin(x))
    return (math.sin(x) / x) * box * box


def decimate(edges: Sequence[Tuple[int, int]], initial: int, start: float,
             seconds: float, clock_hz: float) -> List[float]:
    """Audio samples at :data:`FS_AUDIO` of the pin waveform from cycle
    ``start`` for ``seconds``: the mean level, 0 .. 1."""
    width = clock_hz / STAGE_HZ
    n = int(seconds * STAGE_HZ)
    xs = window_means(edges, initial, start, width, n)
    ys = boxcar(boxcar(xs, DECIMATE), DECIMATE)
    return ys[::DECIMATE]


# -------------------------------------------------------------- analysis
def dft(xs: Sequence[float], f: float, fs: float, hann: bool = False) -> complex:
    """The DFT of ``xs`` at ``f`` Hz, scaled so a sine of amplitude A gives
    about A/2 in magnitude (exactly A/2 on a whole number of cycles)."""
    n = len(xs)
    w = cmath.exp(-2j * math.pi * f / fs)
    z, acc, norm = 1.0 + 0j, 0j, 0.0
    for i, x in enumerate(xs):
        g = 0.5 - 0.5 * math.cos(2 * math.pi * (i + 0.5) / n) if hann else 1.0
        acc += g * x * z
        norm += g
        z *= w
    return acc / norm


def peak_freq(xs: Sequence[float], fs: float, lo: float, hi: float) -> float:
    """The frequency in ``[lo, hi]`` where the Hann-windowed DFT of ``xs``
    peaks: a 1 Hz scan, then a golden-section search to 0.01 Hz."""
    mag = lambda f: abs(dft(xs, f, fs, hann=True))
    best = max((lo + k for k in range(int(hi - lo) + 1)), key=mag)
    a, b = max(lo, best - 1.0), min(hi, best + 1.0)
    g = (math.sqrt(5) - 1) / 2
    c, d = b - g * (b - a), a + g * (b - a)
    while b - a > 0.01:
        if mag(c) > mag(d):
            b, d = d, c
            c = b - g * (b - a)
        else:
            a, c = c, d
            d = a + g * (b - a)
    return (a + b) / 2


def _solve(m: List[List[float]], v: List[float]) -> List[float]:
    """Gaussian elimination with partial pivoting."""
    n = len(v)
    a = [row[:] + [v[i]] for i, row in enumerate(m)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(a[r][c]))
        a[c], a[p] = a[p], a[c]
        for r in range(c + 1, n):
            k = a[r][c] / a[c][c]
            for j in range(c, n + 1):
                a[r][j] -= k * a[c][j]
    out = [0.0] * n
    for r in range(n - 1, -1, -1):
        out[r] = (a[r][n] - sum(a[r][j] * out[j] for j in range(r + 1, n))) / a[r][r]
    return out


def fit_coeffs(xs: Sequence[float], fs: float, freqs: Sequence[float],
               i0: int = 0) -> List[float]:
    """Least-squares fit of a constant plus a sine at each of ``freqs`` to
    ``xs``, whose first sample has index ``i0``: ``[c, a1, b1, a2, b2 ...]``
    for ``c + sum(a cos(w i) + b sin(w i))`` (see :func:`tone_model`)."""
    basis = [[1.0] * len(xs)]
    for f in freqs:
        w = 2 * math.pi * f / fs
        basis.append([math.cos(w * (i0 + i)) for i in range(len(xs))])
        basis.append([math.sin(w * (i0 + i)) for i in range(len(xs))])
    m = [[sum(p * q for p, q in zip(bi, bj)) for bj in basis] for bi in basis]
    v = [sum(p * x for p, x in zip(bi, xs)) for bi in basis]
    return _solve(m, v)


def tone_model(c: Sequence[float], fs: float, freqs: Sequence[float], i: int,
               dc: bool = True) -> float:
    """The fitted signal of :func:`fit_coeffs` at sample index ``i``."""
    y = c[0] if dc else 0.0
    for j, f in enumerate(freqs):
        w = 2 * math.pi * f / fs
        y += c[1 + 2 * j] * math.cos(w * i) + c[2 + 2 * j] * math.sin(w * i)
    return y


def fit_tones(xs: Sequence[float], fs: float, freqs: Sequence[float]
              ) -> Tuple[List[float], List[float]]:
    """(the amplitudes of the :func:`fit_coeffs` fit, the residual samples)."""
    c = fit_coeffs(xs, fs, freqs)
    amps = [math.hypot(c[1 + 2 * j], c[2 + 2 * j]) for j in range(len(freqs))]
    return amps, [x - tone_model(c, fs, freqs, i) for i, x in enumerate(xs)]


def band_power(xs: Sequence[float], fs: float, band: Tuple[float, float]) -> float:
    """The power of ``xs`` in ``band``: the sum of A^2/2 over the DFT bins
    (spacing fs / len) inside it."""
    n = len(xs)
    k0, k1 = int(math.ceil(band[0] * n / fs)), int(band[1] * n / fs)
    return sum(2 * abs(dft(xs, k * fs / n, fs)) ** 2 for k in range(k0, k1 + 1))


def _golden_min(cost, a: float, b: float, tol: float = 0.005) -> float:
    g = (math.sqrt(5) - 1) / 2
    c, d = b - g * (b - a), a + g * (b - a)
    fc, fd = cost(c), cost(d)
    while b - a > tol:
        if fc < fd:
            b, d, fd = d, c, fc
            c = b - g * (b - a)
            fc = cost(c)
        else:
            a, c, fc = c, d, fd
            d = a + g * (b - a)
            fd = cost(d)
    return (a + b) / 2


def refine_pair(xs: Sequence[float], fs: float, lo: float, hi: float,
                span: float = 3.0, rounds: int = 3) -> Tuple[float, float]:
    """The two frequencies that minimise the residual of :func:`fit_tones`
    near ``lo`` and ``hi`` (each searched in +-``span`` Hz, in turn, on a
    21-point grid and then by golden section): a joint
    fit, so neither tone's estimate is pulled by the other's leakage, as a
    windowed peak is on a short tone."""
    def resid(f1, f2):
        return sum(r * r for r in fit_tones(xs, fs, (f1, f2))[1])

    def best(cost, f, span):
        grid = [f + span * (k - 10) / 10.0 for k in range(21)]
        g = min(grid, key=cost)                 # a grid first: the cost of a
        return _golden_min(cost, g - span / 10, g + span / 10)  # short tone has side minima
    for _ in range(rounds):
        lo = best(lambda f: resid(f, hi), lo, span)
        hi = best(lambda f: resid(lo, f), hi, span)
        span /= 5
    return lo, hi


# -------------------------------------------------- tones and their timing
class Tone:
    """One DTMF signal found in the audio: where it starts and ends (the
    half-amplitude points of both tones' envelopes), the two frequencies,
    their amplitudes (corrected for :func:`filter_gain`, in units of the
    logic swing) and the voice-band distortion relative to the weaker one."""

    def __init__(self, start_s, end_s, low_hz, high_hz, low_amp, high_amp,
                 distortion_db):
        self.start_s, self.end_s = start_s, end_s
        self.low_hz, self.high_hz = low_hz, high_hz
        self.low_amp, self.high_amp = low_amp, high_amp
        self.distortion_db = distortion_db

    @property
    def duration_s(self) -> float:
        return self.end_s - self.start_s

    @property
    def twist_db(self) -> float:
        """High group level minus low group level (Table A-1/Q.24 note b)."""
        return 20 * math.log10(self.high_amp / self.low_amp)

    def key(self) -> Optional[int]:
        """The key whose nominal pair is nearest, if both are within Q.23's
        1.8 % (section 7.1); else None."""
        lo = min(range(4), key=lambda r: abs(self.low_hz / LOW_HZ[r] - 1))
        hi = min(range(4), key=lambda c: abs(self.high_hz / HIGH_HZ[c] - 1))
        if (abs(self.low_hz / LOW_HZ[lo] - 1) <= SEND_TOLERANCE
                and abs(self.high_hz / HIGH_HZ[hi] - 1) <= SEND_TOLERANCE):
            return 4 * lo + hi
        return None

    def __repr__(self) -> str:
        return ("Tone(%.2f..%.2f ms, %.2f Hz + %.2f Hz, twist %+.2f dB, "
                "distortion %.1f dB)" % (self.start_s * 1e3, self.end_s * 1e3,
                                         self.low_hz, self.high_hz,
                                         self.twist_db, self.distortion_db))


def _runs(flags: Sequence[bool]) -> List[Tuple[int, int]]:
    out, start = [], None
    for i, f in enumerate(list(flags) + [False]):
        if f and start is None:
            start = i
        elif not f and start is not None:
            out.append((start, i))
            start = None
    return out


def _edge(ac: Sequence[float], c: Sequence[float], fs: float,
          freqs: Sequence[float], near: int, span: int, rising: bool) -> float:
    """Where the fitted tone switches on (``rising``) or off near sample
    ``near``: the step position, in fractional samples, that makes the
    gated model match the audio best over +-``span`` samples (a parabola
    through the best three). The filter is symmetric, so the matched step
    sits on the edge itself."""
    lo, hi = max(0, near - span), min(len(ac), near + span)
    model = [tone_model(c, fs, freqs, i, dc=False) for i in range(lo, hi)]

    def cost(s):
        return sum((ac[i] - (model[i - lo] if (i >= s) == rising else 0.0)) ** 2
                   for i in range(lo, hi))
    costs = [cost(s) for s in range(lo, hi + 1)]
    k = min(range(1, len(costs) - 1), key=lambda j: costs[j])
    a, b, d = costs[k - 1], costs[k], costs[k + 1]
    frac = 0.5 * (a - d) / (a - 2 * b + d) if a - 2 * b + d > 0 else 0.0
    return lo + k + frac - 0.5                  # between samples s-1 and s


#: Where a :func:`decimate` output sample sits: sample ``k`` is the average
#: centred on ``k / FS_AUDIO + DELAY_S`` (half a window, then 1.5 windows
#: for each moving average of 4).
DELAY_S = 3.5 / STAGE_HZ
#: Audio samples at each end of a tone that the filter (9 windows, 144 us)
#: mixes with silence, left out of the fit.
SMEAR = 3
#: The quietest signal looked for: mean square 1e-4 of the logic swing
#: (a tone of amplitude 0.014); below it the audio is called silence.
MIN_POWER = 1e-4


def find_tones(xs: Sequence[float], fs: float = FS_AUDIO,
               t0: float = 0.0) -> List[Tone]:
    """Every DTMF signal in the audio ``xs``, which :func:`decimate` made
    from the pin starting at time ``t0``. The capture should start and end
    in silence, and pauses be 2 ms or longer (the rough search averages
    the power over 2 ms). Frequencies, levels and distortion come from a joint fit
    over the tone's body (2 ms in from each end); the start and end from
    matching the gated fit to the audio, within about one audio sample
    (64 us). A second fit then takes the whole body but ``SMEAR`` samples
    at each end, where the filter mixes tone and silence."""
    mean = sum(xs) / len(xs)
    win = int(0.002 * fs)                   # under the shortest pause looked for
    ms = boxcar([(x - mean) ** 2 for x in xs], win)
    top = max(ms)
    floor = max(0.1 * top, MIN_POWER)
    rough = [(a + win // 2, b + win // 2)
             for a, b in _runs([m > floor for m in ms]) if b - a > 0.003 * fs]
    quiet = [x for i, x in enumerate(xs)
             if not any(a - win <= i < b + win for a, b in rough)]
    level = sum(quiet) / len(quiet) if quiet else mean
    ac = [x - level for x in xs]
    guard, span = int(0.002 * fs), int(0.003 * fs)
    tones = []
    for a, b in rough:
        i0 = a + guard
        body = ac[i0:b - guard]
        lo = peak_freq(body, fs, LOW_HZ[0] * 0.95, LOW_HZ[-1] * 1.05)
        hi = peak_freq(body, fs, HIGH_HZ[0] * 0.95, HIGH_HZ[-1] * 1.05)
        lo, hi = refine_pair(body, fs, lo, hi, span=0.03 * lo)
        c = fit_coeffs(body, fs, (lo, hi), i0)
        start = _edge(ac, c, fs, (lo, hi), a, span, rising=True)
        end = _edge(ac, c, fs, (lo, hi), b, span, rising=False)
        i0 = int(start) + SMEAR                 # second pass: the whole body
        body = ac[i0:int(end) - SMEAR]
        lo, hi = refine_pair(body, fs, lo, hi, span=0.03 * lo)
        c = fit_coeffs(body, fs, (lo, hi), i0)
        amps = [math.hypot(c[1], c[2]) / filter_gain(lo),
                math.hypot(c[3], c[4]) / filter_gain(hi)]
        resid = [x - tone_model(c, fs, (lo, hi), i0 + i) for i, x in enumerate(body)]
        dist = band_power(resid, fs, VOICE_BAND_HZ)
        weaker = min(x * x / 2 for x in amps)
        tones.append(Tone(t0 + DELAY_S + start / fs, t0 + DELAY_S + end / fs,
                          lo, hi, amps[0], amps[1], 10 * math.log10(dist / weaker)))
    return tones


# ------------------------------------------- the one-bit receiver finding
def one_bit_samples(index: int, fs: float, n: int, dev: float = 0.0,
                    twist_db: float = 0.0, phases: Tuple[float, float] = (0.0, 0.0),
                    t0: float = 0.0) -> List[int]:
    """What a comparator on the line gives when key ``index`` is sent: the
    sign of the two-tone sum, sampled ``n`` times at ``fs``. ``dev`` is the
    relative frequency deviation of both tones, ``twist_db`` the high group
    level minus the low group level (Table A-1/Q.24 note b)."""
    lo, hi = key_freqs(index)
    a_hi = 10 ** (twist_db / 20)
    out = []
    for i in range(n):
        t = t0 + i / fs
        v = (math.sin(2 * math.pi * lo * (1 + dev) * t + phases[0])
             + a_hi * math.sin(2 * math.pi * hi * (1 + dev) * t + phases[1]))
        out.append(1 if v > 0 else 0)
    return out


def goertzel_power(bits: Sequence[int], f: float, fs: float) -> float:
    """The Goertzel recursion (s = x + 2 cos(w) s1 - s2, power = s1^2 + s2^2
    - 2 cos(w) s1 s2) over the one-bit samples read as +-1, scaled to the
    squared amplitude of a sine at ``f`` (1.0 for a sine of amplitude 1)."""
    c = 2 * math.cos(2 * math.pi * f / fs)
    s1 = s2 = 0.0
    for b in bits:
        s1, s2 = (1.0 if b else -1.0) + c * s1 - s2, s1
    p = s1 * s1 + s2 * s2 - c * s1 * s2
    return p * 4 / (len(bits) ** 2)


def goertzel_lengths(fs: float, cycles: float = 20.0) -> List[int]:
    """A window of ``cycles`` periods for each of the eight frequencies: the
    length at which +-1.5 % costs 1.3 dB and +-3.5 % costs 8.7 dB (20
    cycles: sinc(0.3)^2 and sinc(0.7)^2), so one threshold serves every
    filter."""
    return [int(round(cycles * fs / f)) for f in LOW_HZ + HIGH_HZ]


def one_bit_bins(index: int, fs: float = 8000.0, dev: float = 0.0,
                 twist_db: float = 0.0, phases: Tuple[float, float] = (0.0, 0.0),
                 cycles: float = 20.0) -> List[float]:
    """The eight Goertzel powers, in dB, of a one-bit capture of key
    ``index``: each filter over its own :func:`goertzel_lengths` window,
    all windows ending on the same sample."""
    ns = goertzel_lengths(fs, cycles)
    bits = one_bit_samples(index, fs, max(ns), dev, twist_db, phases)
    return [10 * math.log10(max(goertzel_power(bits[len(bits) - n:], f, fs), 1e-12))
            for f, n in zip(LOW_HZ + HIGH_HZ, ns)]
