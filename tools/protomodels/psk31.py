"""PSK31 reference: the varicode, a reference demodulator that works from the
pin stream alone, a symbol-timing checker and a spectrum measurement.

Written from the protocol's descriptions, never from firmware/psk31_tx.loom.

Sources
-------
* [ARRL] "PSK31 Spec", ARRL web page (www.arrl.org/psk31-spec), prepared by
  Steven L. Karty N5SK from Peter Martinez G3PLX's description; read
  2026-11-07. It gives: 31.25 baud; a 0 is a phase reversal (BPSK), a 1 a
  steady carrier; the output is cosine-filtered before the transmitter;
  the preamble is an idle of continuous zeros (continuous reversals), with
  no duration given; the postamble is continuous unmodulated carrier (a
  run of ones); at least two zeros between characters; the varicode table.
* [G3PLX] P. Martinez G3PLX, "PSK31: A New Radio-Teletype Mode", Radio
  Communication (RSGB), December 1998 and January 1999: the original
  description and varicode table as he published it. Not fetched (no copy
  reachable on 2026-11-07): every statement here attributed to it is from
  memory, unchecked, and the director should check it. From memory: the
  shaped reversal is the sum of two tones at fc +- baud / 2, and the
  varicode was chosen so that no character contains "00".
* [FLDIGI] fldigi src/psk/pskvaricode.cxx (W1HKJ, GPL, from gmfsk): the
  table below, codes 0..127, read 2026-11-07. Cross-checked entry by entry
  against [ARRL]: 0..89 and 97..127 agree; [ARRL]'s 90..96 are shifted one
  row (its Z repeats Y's 101111011, its '[' is Z's 1010101101), so
  [ARRL] has a transcription error there and [FLDIGI]'s rows are taken.
  :func:`check_table` proves the property the scheme rests on (no "00"
  inside, start and end with 1, all distinct).

Timing figures
--------------
* Symbol rate 31.25 baud ([ARRL]): 32 ms a symbol. A receiver tracks the
  symbol clock, so the tests take a tolerance of 0.1 % on the rate (the
  arithmetic of the tick makes it exact to the board clock) - a tolerance
  chosen here, not read from a source.
* Carrier: PSK31 has no fixed audio frequency (the operator tunes it);
  1000 Hz is the example the brief gives. The tests take the measured
  carrier within 1 % of the baud rate of the configured one (0.31 Hz at
  31.25 baud): a phase drift under 3.6 degrees a symbol, negligible to a
  differential receiver. A tolerance chosen here, not read from a source;
  the arithmetic makes the carrier exact to the board clock.
* Preamble and postamble lengths are not given by [ARRL]; the transmitter
  takes both from the host. (fldigi sends 32 symbols of each: from memory,
  unchecked.)
"""

from __future__ import annotations

import cmath
import dataclasses
import math
from typing import Dict, List, Optional, Sequence, Tuple

# ------------------------------------------------------------- varicode
# [FLDIGI] rows 0..127 (checked against [ARRL] as the docstring says).
VARICODE: Tuple[str, ...] = (
    "1010101011", "1011011011", "1011101101", "1101110111",     # 0
    "1011101011", "1101011111", "1011101111", "1011111101",
    "1011111111", "11101111", "11101", "1101101111",            # 8 BS TAB LF VT
    "1011011101", "11111", "1101110101", "1110101011",          # 12 FF CR
    "1011110111", "1011110101", "1110101101", "1110101111",     # 16
    "1101011011", "1101101011", "1101101101", "1101010111",
    "1101111011", "1101111101", "1110110111", "1101010101",     # 24
    "1101011101", "1110111011", "1011111011", "1101111111",
    "1", "111111111", "101011111", "111110101",                 # 32 SP ! " #
    "111011011", "1011010101", "1010111011", "101111111",       # $ % & '
    "11111011", "11110111", "101101111", "111011111",           # 40 ( ) * +
    "1110101", "110101", "1010111", "110101111",                # , - . /
    "10110111", "10111101", "11101101", "11111111",             # 48 0 1 2 3
    "101110111", "101011011", "101101011", "110101101",         # 4 5 6 7
    "110101011", "110110111", "11110101", "110111101",          # 56 8 9 : ;
    "111101101", "1010101", "111010111", "1010101111",          # < = > ?
    "1010111101", "1111101", "11101011", "10101101",            # 64 @ A B C
    "10110101", "1110111", "11011011", "11111101",              # D E F G
    "101010101", "1111111", "111111101", "101111101",           # 72 H I J K
    "11010111", "10111011", "11011101", "10101011",             # L M N O
    "11010101", "111011101", "10101111", "1101111",             # 80 P Q R S
    "1101101", "101010111", "110110101", "101011101",           # T U V W
    "101110101", "101111011", "1010101101", "111110111",        # 88 X Y Z [
    "111101111", "111111011", "1010111111", "101101101",        # \ ] ^ _
    "1011011111", "1011", "1011111", "101111",                  # 96 ` a b c
    "101101", "11", "111101", "1011011",                        # d e f g
    "101011", "1101", "111101011", "10111111",                  # 104 h i j k
    "11011", "111011", "1111", "111",                           # l m n o
    "111111", "110111111", "10101", "10111",                    # 112 p q r s
    "101", "110111", "1111011", "1101011",                      # t u v w
    "11011111", "1011101", "111010101", "1010110111",           # 120 x y z {
    "110111011", "1010110101", "1011010111", "1110110101",      # | } ~ DEL
)
DECODE: Dict[str, int] = {c: i for i, c in enumerate(VARICODE)}
SEPARATOR = "00"
BAUD = 31.25                                   # [ARRL]


def check_table() -> None:
    """The properties [ARRL] and [G3PLX] state for the code: 128 distinct
    codes, each starting and ending with 1 and holding no "00" (so "00"
    can only be a separator). The code need not be prefix-free: the
    separator ends every character."""
    assert len(VARICODE) == 128 and len(DECODE) == 128
    for c in VARICODE:
        assert set(c) <= {"0", "1"} and c[0] == c[-1] == "1", c
        assert SEPARATOR not in c, c


def encode(text: str) -> str:
    """The bit string of ``text``: each character's code, then "00"."""
    return "".join(VARICODE[ord(ch) & 0x7F] + SEPARATOR for ch in text)


def decode(bits: str) -> str:
    """Characters from a bit stream: split at runs of two or more zeros,
    as a receiver does; an unknown code becomes U+FFFD, never silently
    dropped."""
    out: List[str] = []
    for word in bits.split(SEPARATOR):
        word = word.strip("0")
        if word:
            out.append(chr(DECODE[word]) if word in DECODE else "�")
    return "".join(out)


def left_aligned(code: str) -> int:
    """A code with its separator and a stop marker, left-aligned in 16 bits:
    the transmitter's table word (code + "00" + "1", then zeros)."""
    s = code + SEPARATOR + "1"
    assert len(s) <= 16
    return int(s, 2) << (16 - len(s))


def decode_terminated(bits: str) -> str:
    """As :func:`decode`, but only characters a "00" has ended: a receiver
    cannot print a character before its separator, so the postamble's run
    of ones (never followed by "00") prints nothing."""
    end = bits.rfind(SEPARATOR)
    return decode(bits[:end + 2]) if end >= 0 else ""


# ------------------------------------------------------ the pin stream
def pin_samples(edges: Sequence[Tuple[int, int]], initial: int, first: float,
                period: float, count: int) -> List[int]:
    """The pin's level at ``first + (k + 1/2) * period`` clocks, k = 0 ..
    count - 1: one value a transmitter sample when ``period`` is the tick
    and ``first`` a tick of the grid (the edges are on it)."""
    out: List[int] = []
    level, i = initial, 0
    for k in range(count):
        t = first + (k + 0.5) * period
        while i < len(edges) and edges[i][0] <= t:
            level = edges[i][1]
            i += 1
        out.append(level)
    return out


def _boxcar(zs: Sequence[complex], width: int) -> List[complex]:
    """A centred moving sum of ``width`` samples (zero outside)."""
    acc, out, h = 0j, [], width // 2
    pre = [0j] * (len(zs) + 1)
    for k, z in enumerate(zs):
        acc += z
        pre[k + 1] = acc
    n = len(zs)
    for k in range(n):
        lo, hi = max(0, k - h), min(n, k - h + width)
        out.append((pre[hi] - pre[lo]) / width)
    return out


def baseband(levels: Sequence[int], cycles_per_sample: float) -> List[complex]:
    """The complex envelope of the pin about the nominal carrier: the
    level as +-1, mixed down, then two boxcars of one carrier period
    each (they null the 2 fc product, the square wave's harmonics and the
    sigma-delta's 1, 0 pattern at fs / 2 when the period is a whole number
    of samples). A pure carrier of amplitude A (as +-A) gives |z| = A / 2."""
    period = round(1 / cycles_per_sample)
    w = 2 * math.pi * cycles_per_sample
    mixed = [(2 * v - 1) * cmath.exp(-1j * w * k) for k, v in enumerate(levels)]
    return _boxcar(_boxcar(mixed, period), period)


# ------------------------------------------------- reference demodulator
@dataclasses.dataclass
class Demod:
    """What the reference receiver made of a transmission (sample units:
    one transmitter sample; positions count from the first sample given)."""
    start: int                    # first and last sample with the carrier
    end: int                      # above half its peak envelope
    reversals: List[float]        # zero crossings of the real envelope
    t0: float                     # the fitted grid: boundaries at t0 + m T
    period: float                 # T, samples a symbol
    grid_error: float             # largest |reversal - grid|, samples
    bits: str                     # one a symbol after the first (reference)
    text: str                     # the varicode-decoded characters
    offset: float                 # carrier measured - nominal, cycles a sample


def demodulate(levels: Sequence[int], cycles_per_sample: float,
               samples_per_symbol: int) -> Demod:
    """Differentially coherent BPSK from the pin alone. The carrier's
    frequency offset and phase come from the squared envelope (squaring
    removes the BPSK sign); a reversal is a sign change of the derotated
    real envelope; the symbol grid is a least-squares fit of the
    reversals; a bit is 0 when the sign at a symbol centre differs from
    the centre before ([ARRL]: 0 = reversal)."""
    n = samples_per_symbol
    z = baseband(levels, cycles_per_sample)
    mags = [abs(v) for v in z]
    peak = max(mags)
    act = [k for k, m in enumerate(mags) if m > 0.5 * peak]
    start, end = act[0], act[-1]
    lag = n // 2
    z2 = [v * v for v in z]
    acc = sum(z2[k] * z2[k - lag].conjugate() for k in range(start + lag, end + 1))
    offset = cmath.phase(acc) / (4 * math.pi * lag)
    zr = [v * cmath.exp(-2j * math.pi * offset * k) for k, v in enumerate(z)]
    phi = cmath.phase(sum(v * v for v in zr[start:end + 1])) / 2
    b = [(v * cmath.exp(-1j * phi)).real for v in zr]
    rev: List[float] = []
    for k in range(start + 1, end + 1):
        if (b[k - 1] < 0) != (b[k] < 0):
            t = k - 1 + b[k - 1] / (b[k - 1] - b[k])
            if not rev or t - rev[-1] > n / 4:
                rev.append(_refine(b, t, max(2, n // 8)))
    if len(rev) >= 2:
        ms = [round((r - rev[0]) / n) for r in rev]
        mm = sum(ms) / len(ms)
        mr = sum(rev) / len(rev)
        sxx = sum((m - mm) ** 2 for m in ms)
        period = sum((m - mm) * (r - mr) for m, r in zip(ms, rev)) / sxx if sxx else n
        t0 = mr - period * mm
        err = max(abs(r - (t0 + period * m)) for m, r in zip(ms, rev))
    else:
        period, t0, err = float(n), (rev[0] if rev else float(start)), 0.0
    j = math.ceil((start - t0) / period - 0.5)
    centres = []
    while t0 + (j + 0.5) * period <= end:
        centres.append(int(round(t0 + (j + 0.5) * period)))
        j += 1
    signs = [b[c] < 0 for c in centres]
    bits = "".join("0" if s != p else "1" for p, s in zip(signs, signs[1:]))
    return Demod(start, end, rev, t0, period, err, bits,
                 decode_terminated(bits), offset)


def _refine(b: Sequence[float], t: float, w: int) -> float:
    """A zero crossing near ``t`` as the root of a least-squares line
    through b over +- ``w`` samples: the envelope is symmetric about a
    reversal in both modes, so the root is unbiased and the noise of the
    two neighbouring samples averages out."""
    ks = [k for k in range(int(t) - w + 1, int(t) + w + 1) if 0 <= k < len(b)]
    km = sum(ks) / len(ks)
    bm = sum(b[k] for k in ks) / len(ks)
    slope = sum((k - km) * (b[k] - bm) for k in ks) / sum((k - km) ** 2 for k in ks)
    return km - bm / slope if slope else t


# ------------------------------------------------------------- spectrum
def line_levels(levels: Sequence[int], first: int, count: int,
                cycles_per_sample: float, samples_per_symbol: int,
                orders: Sequence[int] = (1, 3, 5, 7)) -> Dict[int, float]:
    """During idle (continuous reversals) the signal repeats every two
    symbols, so its spectrum is lines at fc +- m baud / 2, m odd. For each
    m: the power of the two lines, in dB relative to the m = 1 pair, from
    a DFT over ``count`` samples from ``first`` (a whole number of two-
    symbol periods, so the lines fall on bins: no leakage)."""
    assert count % (2 * samples_per_symbol) == 0
    xs = [2 * v - 1 for v in levels[first:first + count]]

    def power(f: float) -> float:
        w = -2j * math.pi * f
        return abs(sum(x * cmath.exp(w * k) for k, x in enumerate(xs))) ** 2

    half = 1 / (2 * samples_per_symbol)
    p = {m: power(cycles_per_sample + m * half) + power(cycles_per_sample - m * half)
         for m in orders}
    return {m: 10 * math.log10(max(p[m], 1e-30) / p[orders[0]]) for m in orders}


def ideal_square(bits: str, cycles_per_sample: float,
                 samples_per_symbol: int, lead: int = 0) -> List[int]:
    """A textbook square-wave BPSK stream from the description alone (for
    checking the demodulator, not the firmware): ``lead`` samples of
    silence (low), a reference symbol, then one symbol a bit, a 0 turning
    the phase by half a cycle at the symbol's start ([ARRL])."""
    out, sign = [0] * lead, 0
    for j, bit in enumerate("1" + bits):
        if bit == "0":
            sign ^= 1
        for k in range(samples_per_symbol):
            s = j * samples_per_symbol + k
            out.append(int((s * cycles_per_sample) % 1.0 >= 0.5) ^ sign)
    return out
