"""DCF77 time code reference models, for firmware/dcf77_gen.loom and
dcf77_rx.loom.

Written from PTB's descriptions of the time code, not from the firmware.
Sources, read 2026-11-07:

* [PTB-AM] PTB, "DCF77 amplitude modulation" (ptb.de, Fachbereich 4.4,
  dissemination of legal time): at the beginning of each second the carrier
  is lowered to about 15 % for 0.1 s or 0.2 s; the last second of each
  minute has no reduction, which marks the beginning of the next minute.
* [PTB-TC] PTB, "DCF77 time code" (same site): a 0.1 s mark is a binary 0,
  0.2 s a 1; the first mark of the minute (M) is always 0; second 15 the
  call bit R, 16 A1 (CET/CEST change announced), 17/18 Z1/Z2 (CEST: 1 0,
  CET: 0 1), 19 A2 (leap second announced), time information from second 20;
  minute 7 bits, hour 6 bits, date 22 bits including the weekday, all BCD;
  P1, P2, P3 make each preceding word an even number of ones; each code
  carries the time of the following minute; Monday is weekday 1 (ISO 8601).
  Leap second: A2 is 1 for the hour before; the 59th mark before 01:00:00
  CET (02:00:00 CEST) is sent 0.1 s long, and the inserted 60th mark has no
  carrier reduction.
* [PTBM-2004] D. Piester, P. Hetzel, A. Bauch, "Zeit- und
  Normalfrequenzverbreitung mit DCF77", PTB-Mitteilungen 114 (2004), Heft
  4, section 4.2 and Bild 5 (the coding scheme): the same rules, S (second
  20) a start bit sent as 0.2 s, R at 15, A1 at 16, Z1 Z2 at 17 18, A2 at 19;
  "die drei Pruefbits P1, P2 und P3 ergaenzen jeweils die vorhergehenden
  Informationswoerter (7 Bits fuer die Minute, 6 Bits fuer die Stunde und
  22 Bits fuer das Datum ...) auf eine gerade Zahl von Einsen". (That article
  gives the reduction as 25 %; the current page says 15 %. The logic level
  a receiver module gives is the same.)

The second numbers follow from [PTBM-2004]'s order and counts: S at 20,
minute 21..27, P1 28, hour 29..34, P2 35, date 36..57 (day 6, weekday 3,
month 5, year 8 bits), P3 58. The weights inside each field (1, 2, 4, 8,
10, 20, 40, 80, least significant first) are drawn in Bild 5, a graphic
whose text did not extract: they are taken from Wikipedia, "DCF77" (read
2026-11-07), a secondary source, and are UNCHECKED against PTB's figure.

No source gives a receiver module's pulse width error. The decoder's
default windows here (:data:`WINDOWS`) are this model's choice, stated, not
a specification.

Signals are lists ``[(level, ms), ...]`` of the receiver module's logic
output: ``PULSE`` while the carrier is reduced, ``IDLE`` otherwise. A
minute's list starts at its second 0 and ends with its marker second.

* :class:`DcfTime`, :func:`encode`, :func:`decode_bits`: the 59 bits.
* :func:`minute_pulses`: the encoder to pulses; :func:`decode_pulses`: a
  decoder with windows; :func:`check_timing`: a generator's edges against
  the second grid.
* :class:`DcfSource`, :class:`EdgeRecorder`: bench models; :func:`glitch`,
  :func:`with_width`, :func:`drop_pulse`, :func:`flip`: faults;
  :func:`deglitch`: a glitch filter for the decoder's input.
"""

from __future__ import annotations

import dataclasses
from typing import List, Optional, Sequence, Tuple

from .bench import Drive, Lines, Model, PinSpec, pad_of

PULSE, IDLE = 1, 0
SECOND_MS = 1000.0
WIDTH_MS = {0: 100.0, 1: 200.0}             # [PTB-TC]
BITS = 59                                   # seconds 0..58 carry bits
P1, P2, P3 = 28, 35, 58
PARITY_SPANS = {P1: range(21, 28), P2: range(29, 35), P3: range(36, 58)}
FIELDS = {                                  # name: (first second, weights)
    "minute": (21, (1, 2, 4, 8, 10, 20, 40)),
    "hour": (29, (1, 2, 4, 8, 10, 20)),
    "day": (36, (1, 2, 4, 8, 10, 20)),
    "weekday": (42, (1, 2, 4)),
    "month": (45, (1, 2, 4, 8, 10)),
    "year": (50, (1, 2, 4, 8, 10, 20, 40, 80)),
}
R_BIT, A1_BIT, Z1_BIT, Z2_BIT, A2_BIT, S_BIT = 15, 16, 17, 18, 19, 20
# The decoder's windows in ms from a pulse's start (the model's choice):
WINDOWS = {"zero": (40.0, 150.0), "one": (150.0, 260.0),
           "next": (900.0, 1100.0), "marker": (1900.0, 2100.0)}

Pulses = List[Tuple[int, float]]


@dataclasses.dataclass
class DcfTime:
    """One minute's content. ``year`` is the year in the century (0..99).
    ``cest``: Z1 = 1, Z2 = 0 (else CET: Z1 = 0, Z2 = 1)."""

    minute: int = 0
    hour: int = 0
    day: int = 1
    weekday: int = 1
    month: int = 1
    year: int = 0
    cest: bool = False
    a1: bool = False                        # CET/CEST change announced
    a2: bool = False                        # leap second announced
    call: bool = False                      # R
    weather: int = 0                        # seconds 1..14, bit 0 = second 1


def _bcd(value: int, weights: Sequence[int]) -> List[int]:
    """``value`` in BCD on ``weights``: the units digit on the weights below
    10, the tens digit on 10, 20, 40, 80."""
    tens, units = divmod(value, 10)
    out = []
    for w in weights:
        digit, w1 = (units, w) if w < 10 else (tens, w // 10)
        out.append(1 if digit & w1 else 0)
    if sum(w * b for w, b in zip(weights, out)) != value or value < 0:
        raise ValueError("%d does not fit weights %r" % (value, tuple(weights)))
    return out


def encode(t: DcfTime) -> List[int]:
    """The 59 bits of seconds 0..58, parity included."""
    bits = [0] * BITS
    for s in range(1, 15):
        bits[s] = (t.weather >> (s - 1)) & 1
    bits[R_BIT], bits[A1_BIT] = int(t.call), int(t.a1)
    bits[Z1_BIT], bits[Z2_BIT] = (1, 0) if t.cest else (0, 1)
    bits[A2_BIT], bits[S_BIT] = int(t.a2), 1
    for name, (first, weights) in FIELDS.items():
        bits[first:first + len(weights)] = _bcd(getattr(t, name), weights)
    for p, span in PARITY_SPANS.items():
        bits[p] = sum(bits[i] for i in span) & 1
    return bits


def decode_bits(bits: Sequence[int]) -> Tuple[Optional[DcfTime], str]:
    """``(time, "")`` for 59 good bits, else ``(None, reason)``: M not 0,
    S not 1, a parity, Z1 = Z2, or a BCD digit out of its range."""
    if len(bits) != BITS:
        return None, "%d bits" % len(bits)
    if bits[0] != 0:
        return None, "M"
    if bits[S_BIT] != 1:
        return None, "S"
    for p, span in PARITY_SPANS.items():
        if (sum(bits[i] for i in span) + bits[p]) & 1:
            return None, "P%d" % (1 + sorted(PARITY_SPANS).index(p))
    if bits[Z1_BIT] == bits[Z2_BIT]:
        return None, "Z1 Z2"
    vals = {}
    for name, (first, weights) in FIELDS.items():
        b = bits[first:first + len(weights)]
        units = sum(w * x for w, x in zip(weights, b) if w < 10)
        if units > 9:
            return None, "BCD " + name
        vals[name] = sum(w * x for w, x in zip(weights, b))
    t = DcfTime(cest=bool(bits[Z1_BIT]), a1=bool(bits[A1_BIT]), a2=bool(bits[A2_BIT]),
                call=bool(bits[R_BIT]),
                weather=sum(bits[s] << (s - 1) for s in range(1, 15)), **vals)
    return t, ""


def minute_pulses(bits: Sequence[int], leap: bool = False) -> Pulses:
    """One minute from its second 0 to the end of its marker second: each
    bit a pulse of 100 or 200 ms and the rest of its second idle; with
    ``leap``, second 59 a 100 ms pulse and second 60 the marker [PTB-TC]."""
    out: Pulses = []
    for b in list(bits) + ([0] if leap else []):
        w = WIDTH_MS[b]
        out += [(PULSE, w), (IDLE, SECOND_MS - w)]
    out[-1] = (IDLE, out[-1][1] + SECOND_MS)    # the marker second joins the idle
    return out


@dataclasses.dataclass
class DcfMessage:
    kind: str                       # "minute" or "error"
    bits: List[int] = dataclasses.field(default_factory=list)
    leap: bool = False
    time: Optional[DcfTime] = None
    second: int = 0                 # where an error was found
    reason: str = ""


def decode_pulses(pulses: Sequence[Tuple[int, float]], *, windows=None) -> List[DcfMessage]:
    """Decode a signal: hunt for a pulse followed by an idle stretch longer
    than 1.5 s (the marker; the start of the list is not one, since the
    second a first pulse belongs to is unknown), then one pulse a second,
    each classified by its width and starting within ``next`` of the one
    before; a minute ends at a missing pulse in second 59 (60 with A2 and a
    leap 0 in 59). A minute is reported whole or as an error, never both."""
    win = dict(WINDOWS, **(windows or {}))
    starts, t, idle_before = [], 0.0, 0.0
    for level, ms in pulses:                    # (start ms, width ms, idle before)
        if level == PULSE:
            starts.append((t, ms, idle_before))
        else:
            idle_before = ms
        t += ms
    out: List[DcfMessage] = []
    i = 0
    while i < len(starts):
        if starts[i][2] < 1500.0:
            i += 1
            continue
        msg, i = _minute(starts, i, t, win)
        out.append(msg)
    return out


def _width_bit(ms, win):
    lo0, hi0 = win["zero"]
    lo1, hi1 = win["one"]
    if lo0 <= ms < hi0:
        return 0
    if lo1 <= ms <= hi1:
        return 1
    return None


def _minute(starts, i, end, win) -> Tuple[DcfMessage, int]:
    bits: List[int] = []
    s = 0
    while True:
        t0, w, _ = starts[i]
        b = _width_bit(w, win)
        if b is None:
            return DcfMessage("error", second=s, reason="width %.1f ms" % w), i + 1
        leap_second = s == 59
        if leap_second and b != 0:
            return DcfMessage("error", second=s, reason="leap second not 0"), i + 1
        if not leap_second:
            bits.append(b)
        s += 1
        nxt = starts[i + 1][0] - t0 if i + 1 < len(starts) else end - t0 + 1e9
        lo, hi = win["next"]
        if lo <= nxt <= hi:
            if s == 59 and bits[A2_BIT] and not leap_second:
                i += 1
                continue                        # the leap 0 of second 59
            if s >= 59:
                return DcfMessage("error", second=s, reason="no marker"), i + 1
            i += 1
            continue
        if nxt < lo:
            return DcfMessage("error", second=s, reason="early pulse"), i + 1
        if s < 59:
            return DcfMessage("error", second=s, reason="missing pulse"), i + 1
        time, why = decode_bits(bits)
        if time is None:
            return DcfMessage("error", bits=bits, second=s, reason=why), i + 1
        return DcfMessage("minute", bits=bits, leap=leap_second, time=time), i + 1


# ------------------------------------------------------- timing checker
def edges_to_pulses(edges: Sequence[Tuple[int, int]], clock_hz: float) -> Pulses:
    """``[(cycle, level)]`` (an :class:`EdgeRecorder`'s) to a pulse list
    from the first rise to the last edge; times in ms."""
    out: Pulses = []
    k = next((j for j, (_, lv) in enumerate(edges) if lv == PULSE), len(edges))
    for (c0, lv), (c1, _) in zip(edges[k:], edges[k + 1:]):
        out.append((lv, (c1 - c0) * 1e3 / clock_hz))
    return out


def check_timing(edges: Sequence[Tuple[int, int]], clock_hz: float,
                 bits: Sequence[Sequence[int]], leaps: Sequence[bool] = (),
                 tol_ms: float = 0.0) -> List[str]:
    """A generator's pin against the code: every pulse starts on the second
    grid of the first one (exactly, or within ``tol_ms``), lasts 100 or 200
    ms as its bit says, and no pulse is sent in a marker second. ``bits``
    is one 59-bit list per minute, sent back to back. Empty when good."""
    rises = [c for c, lv in edges if lv == PULSE]
    falls = [c for c, lv in edges if lv == IDLE and c > (rises[0] if rises else 0)]
    want: List[Tuple[float, float]] = []    # (start ms, width ms) from the first rise
    t = 0.0
    for k, minute in enumerate(bits):
        leap = bool(leaps[k]) if k < len(leaps) else False
        for b in list(minute) + ([0] if leap else []):
            want.append((t, WIDTH_MS[b]))
            t += SECOND_MS
        t += SECOND_MS                      # the marker second
    bad: List[str] = []
    if len(rises) != len(want) or len(falls) < len(want):
        return ["%d rises and %d falls for %d pulses" % (len(rises), len(falls), len(want))]
    ms = lambda c: (c - rises[0]) * 1e3 / clock_hz
    for j, ((st, w), r, f) in enumerate(zip(want, rises, falls)):
        if abs(ms(r) - st) > tol_ms:
            bad.append("pulse %d starts at %.3f ms, not %.0f" % (j, ms(r), st))
        if abs((f - r) * 1e3 / clock_hz - w) > tol_ms:
            bad.append("pulse %d lasts %.3f ms, not %.0f" % (j, (f - r) * 1e3 / clock_hz, w))
    return bad


# ------------------------------------------------------- bench models
class EdgeRecorder(Model):
    """Every change of one pad: ``edges`` = ``[(cycle, level)]``; the level
    before the first edge is 0."""

    def __init__(self, pin: PinSpec) -> None:
        self.pad, self.level, self.edges = pad_of(pin), 0, []

    def observe(self, lines: Lines) -> None:
        level = lines.get(self.pad)
        if level != self.level:
            self.edges.append((lines.cycle, level))
            self.level = level


class DcfSource(Model):
    """Drives a receiver pin as a DCF77 module's output: ``send`` queues a
    pulse list at cycle ``at`` (default: where the previous list ended) and
    returns its start. ``ms_clocks`` is clocks per millisecond of the
    signal: the board clock / 1000 in real time, or the scaled figure."""

    def __init__(self, pin: PinSpec, ms_clocks: float) -> None:
        self.pad, self.ms_clocks = pad_of(pin), float(ms_clocks)
        self.level, self.free_at = IDLE, 0
        self._changes: List[Tuple[int, int]] = []

    def cycles(self, ms: float) -> int:
        return int(round(ms * self.ms_clocks))

    def send(self, pulses: Sequence[Tuple[int, float]], at: Optional[int] = None) -> int:
        start = self.free_at if at is None else int(at)
        t = 0.0
        for level, ms in pulses:
            self._changes.append((start + self.cycles(t), level))
            t += ms
        self.free_at = start + self.cycles(t)
        self._changes.append((self.free_at, IDLE))
        self._changes.sort(key=lambda c: c[0])
        return start

    def drive(self, drive: Drive, cycle: int) -> None:
        while self._changes and self._changes[0][0] <= cycle:
            self.level = self._changes.pop(0)[1]
        drive.set(self.pad, self.level)


class Receiver(Model):
    """A receiver module between a generator pin and a receiver pin: the
    output follows the input ``delay_ms`` later, each pulse ``stretch_ms``
    longer (negative: shorter). A stand-in for a real module, whose
    figures no source here gives."""

    def __init__(self, tx: PinSpec, rx: PinSpec, ms_clocks: float, *,
                 delay_ms: float = 0.0, stretch_ms: float = 0.0) -> None:
        self.tx, self.rx = pad_of(tx), pad_of(rx)
        self.delay = int(round(delay_ms * ms_clocks))
        self.stretch = int(round(stretch_ms * ms_clocks))
        self.level, self.seen = IDLE, IDLE
        self._changes: List[Tuple[int, int]] = []

    def drive(self, drive: Drive, cycle: int) -> None:
        while self._changes and self._changes[0][0] <= cycle:
            self.level = self._changes.pop(0)[1]
        drive.set(self.rx, self.level)

    def observe(self, lines: Lines) -> None:
        v = lines.get(self.tx)
        if v != self.seen:
            self.seen = v
            at = lines.cycle + 1 + self.delay + (self.stretch if v == IDLE else 0)
            self._changes.append((at, v))
            self._changes.sort(key=lambda c: c[0])


# ------------------------------------------------------- faults
def pulse_index(second: int) -> int:
    """Index in a minute's pulse list of second ``second``'s pulse."""
    return 2 * second


def with_width(pulses: Sequence[Tuple[int, float]], second: int, ms: float) -> Pulses:
    """Second ``second``'s pulse lasting ``ms``; the second keeps its length."""
    out = list(pulses)
    k = pulse_index(second)
    total = out[k][1] + out[k + 1][1]
    out[k], out[k + 1] = (PULSE, ms), (IDLE, total - ms)
    return out


def drop_pulse(pulses: Sequence[Tuple[int, float]], second: int) -> Pulses:
    """Second ``second`` with no pulse: its whole second idle."""
    out = list(pulses)
    k = pulse_index(second)
    out[k] = (IDLE, out[k][1])
    merged: Pulses = []
    for level, ms in out:                   # join neighbouring idle stretches
        if merged and merged[-1][0] == level:
            merged[-1] = (level, merged[-1][1] + ms)
        else:
            merged.append((level, ms))
    return merged


def flip(bits: Sequence[int], second: int) -> List[int]:
    """``bits`` with one bit inverted (a parity error, unless ``second`` is a
    parity bit's own partner too)."""
    out = list(bits)
    out[second] ^= 1
    return out


def glitch(pulses: Sequence[Tuple[int, float]], at_ms: float, width_ms: float) -> Pulses:
    """A spike of the opposite level, ``width_ms`` long, ``at_ms`` from the
    start of the list (it must fall inside one pulse or idle stretch)."""
    out: Pulses = []
    t = 0.0
    for level, ms in pulses:
        if t < at_ms and at_ms + width_ms < t + ms:
            out += [(level, at_ms - t), (1 - level, width_ms),
                    (level, t + ms - at_ms - width_ms)]
        else:
            out.append((level, ms))
        t += ms
    return out


def deglitch(pulses: Sequence[Tuple[int, float]], min_ms: float) -> Pulses:
    """Drop every stretch shorter than ``min_ms`` (a spike), joining it and
    its neighbours into one: what a glitch filter makes of a signal. Not
    from any source; the decoder above has no filter of its own."""
    out: Pulses = []
    for level, ms in pulses:
        if out and (ms < min_ms or out[-1][0] == level):
            out[-1] = (out[-1][0], out[-1][1] + ms)
        else:
            out.append((level, ms))
    return out


def start_of(pulses: Sequence[Tuple[int, float]], index: int) -> float:
    """ms from the start of the list to the start of pulse ``index``."""
    return sum(ms for _, ms in pulses[:index])
