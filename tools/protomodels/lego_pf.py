"""LEGO Power Functions infrared reference models, for
firmware/lego_pf_tx.loom and lego_pf_rx.loom.

Written from LEGO's protocol document, not from the firmware: "LEGO Power
Functions RC", version 1.20, 26.02.2010 (a PDF LEGO published; the copy read
on 2026-11-07 is the one attached to IRremoteESP8266 on GitHub, files/2974525;
philohome.com's copy refused the fetch). Its text was extracted from the PDF
streams; a few glyphs were lost in extraction, noted below. Sections:

* "LEGO Power Functions RC Protocol": 1 toggle bit, 1 escape bit, 2 channel
  bits (channel 1..4 = CC + 1), 1 address bit, 3 mode bits, 4 data bits, then
  a 4-bit LRC; "start T E C C a M M M D D D D L L L L stop", so nibble 1 is
  ``T E C C``; LRC = 0xF xor nibble 1 xor nibble 2 xor nibble 3. Escape 0 is
  the mode field (000 extended, 001 combo direct, 01x reserved, 1xx single
  output); escape 1 is combo PWM.
* "Extended mode", "Combo direct mode", "Single output mode", "Combo PWM
  mode": the field layouts (single output: ``a 1 M O D D D D``, M = 0 PWM,
  1 clear/set/toggle/inc/dec, O the output; combo PWM: nibble 1 is
  ``a 1 C C`` (the address where the toggle was, no toggle bit), nibble 2
  output B's PWM step, nibble 3 output A's). PWM data: 0 float, 1..7 forward
  steps, 8 brake then float, 9..15 backward steps 7..1.
* "LEGO Power Functions RC Encoding": a mark is six 38 kHz cycles; low bit
  16 cycles (421 us), high bit 27 (711 us), start and stop bit 45 (1184 us),
  each from the start of its mark to the start of the next.
* "Transmitting Messages": five matching messages, with tm the maximum
  message length (16 ms) and Ch the channel: the first after (4 - Ch) tm
  (the operator was lost in extraction: "(4 Ch)*tm" is what was read; 4 - Ch
  is the reading open implementations use, from memory, UNCHECKED), the
  next two 5 tm start to start, the
  following (6 + 2 Ch) tm start to start; a held key repeats at the last
  spacing. Ch is taken as 1..4 (the channel switch's labels); QUESTIONS.md.
* "LEGO Power Functions RC Decoding" / "Receiving Messages": bits measured
  from mark start to mark start; windows low 316 .. 526 us, high 526 ..
  947 us, start and stop 947 .. 1579 us; a bit too long ends the message.

Not in the document: the carrier's duty (the transmitter uses 1/3, as
ir_nec_tx does); how a receiver drops the five copies (the toggle "toggling
for every new command" is the document's rule; the dark time that forgets
the last message is :data:`FORGET_US`, derived below from the spacing).

Pulse lists are ``[(level, microseconds), ...]`` with ``MARK`` and ``SPACE``
as in :mod:`tools.protomodels.ir_nec`, whose edge recorder, carrier
analyser, receiver-module model, demodulated source and variations this
module reuses at 38 kHz.

* :func:`lrc`, :func:`message`, :func:`combo_pwm`, :func:`single_pwm`,
  :func:`combo_direct`, :func:`extended`: the message word;
* :func:`mark_starts`, :func:`message_pulses`, :func:`burst_cycles`: the
  encoder;
* :func:`decode_starts`, :func:`decode_pulses`: a decoder with the
  document's windows; :class:`RepeatFilter`: copies dropped;
* :func:`schedule_tm`, :func:`check_schedule`: the retransmission timing;
* :func:`check_marks`: a transmitter's mark starts against nominal.
"""

from __future__ import annotations

import dataclasses
from typing import List, Optional, Sequence, Tuple

from .ir_nec import (MARK, SPACE, CarrierAnalysis, EdgeRecorder, IrSource,  # noqa: F401
                     TsopModel, glitch, tsop_distort, vary)

CARRIER_HZ = 38_000.0
CYCLE_US = 1e6 / CARRIER_HZ          # 26.316 us
MARK_CYCLES = 6
BIT_CYCLES = {0: 16, 1: 27}
START_CYCLES = STOP_CYCLES = 45
TM_US = 16_000.0                     # tm, the maximum message length
TM_CYCLES = 608                      # 16 ms of 38 kHz, exactly
LOW_US = (316.0, 526.0)              # "Receiving Messages"
HIGH_US = (526.0, 947.0)
STARTSTOP_US = (947.0, 1579.0)
MARKS = 18                           # start, 16 data, stop
#: The longest message, start bit to the stop bit's end: 522 cycles,
#: 13.7 ms, inside tm.
MAX_MESSAGE_US = (START_CYCLES + 16 * BIT_CYCLES[1] + STOP_CYCLES) * CYCLE_US
#: The shortest message's start mark to the end of its stop mark (the line
#: is dark after that).
MIN_MESSAGE_US = (START_CYCLES + 16 * BIT_CYCLES[0] + MARK_CYCLES) * CYCLE_US
#: The longest dark time inside one sequence: the longest start-to-start
#: spacing, (6 + 2 * 4) tm, less the shortest message. A receiver that
#: forgets the last message after a longer dark time never delivers a copy
#: twice.
MAX_DARK_US = (6 + 2 * 4) * TM_US - MIN_MESSAGE_US
FORGET_US = 36 * 255 * CYCLE_US      # the receiver's choice, 241.6 ms

Pulses = List[Tuple[int, float]]


# ------------------------------------------------------------- the message
def lrc(word: int) -> int:
    """The LRC of a message's first three nibbles (bits 15:4 of ``word``)."""
    n1, n2, n3 = (word >> 12) & 0xF, (word >> 8) & 0xF, (word >> 4) & 0xF
    return 0xF ^ n1 ^ n2 ^ n3


def message(word: int) -> int:
    """``word`` with its LRC in bits 3:0."""
    return (word & 0xFFF0) | lrc(word)


def lrc_ok(word: int) -> bool:
    return (word & 0xF) == lrc(word)


def _nibbles(n1: int, n2: int, n3: int) -> int:
    return message(((n1 & 0xF) << 12) | ((n2 & 0xF) << 8) | ((n3 & 0xF) << 4))


def _check_channel(channel: int) -> int:
    if not 1 <= channel <= 4:
        raise ValueError("channel is 1..4")
    return channel - 1


def combo_direct(channel: int, b: int, a: int, toggle: int = 0, address: int = 0) -> int:
    """Combo direct mode: ``b`` and ``a`` 0 float, 1 forward, 2 backward,
    3 brake then float."""
    cc = _check_channel(channel)
    return _nibbles((toggle << 3) | cc, (address << 3) | 0b001, ((b & 3) << 2) | (a & 3))


def single_pwm(channel: int, output: int, data: int, toggle: int = 0,
               address: int = 0) -> int:
    """Single output mode, PWM (M = 0): ``output`` 0 = A, 1 = B."""
    cc = _check_channel(channel)
    return _nibbles((toggle << 3) | cc, (address << 3) | 0b100 | (output & 1), data)


def single_cstid(channel: int, output: int, data: int, toggle: int = 0,
                 address: int = 0) -> int:
    """Single output mode, clear/set/toggle/inc/dec (M = 1)."""
    cc = _check_channel(channel)
    return _nibbles((toggle << 3) | cc, (address << 3) | 0b110 | (output & 1), data)


def extended(channel: int, function: int, toggle: int = 0, address: int = 0) -> int:
    """Extended mode (mode 000)."""
    cc = _check_channel(channel)
    return _nibbles((toggle << 3) | cc, (address << 3), function)


def combo_pwm(channel: int, b: int, a: int, address: int = 0) -> int:
    """Combo PWM mode (escape 1): nibble 1 is ``a 1 C C``."""
    cc = _check_channel(channel)
    return _nibbles((address << 3) | 0b0100 | cc, b, a)


def channel_of(word: int) -> int:
    """The channel, 1..4: bits 13:12 + 1 in every mode."""
    return ((word >> 12) & 3) + 1


def bits_of(word: int) -> List[int]:
    """The 16 bits in wire order, most significant first."""
    return [(word >> (15 - k)) & 1 for k in range(16)]


# ------------------------------------------------------------- the encoder
def interval_cycles(word: int) -> List[int]:
    """Carrier cycles from each mark's start to the next: the start bit, the
    16 data bits (17 intervals before the stop mark)."""
    return [START_CYCLES] + [BIT_CYCLES[b] for b in bits_of(word)]


def mark_starts(word: int) -> List[float]:
    """Microseconds from the start mark's start to each of the 18 marks'."""
    out, t = [0.0], 0
    for n in interval_cycles(word):
        t += n
        out.append(t * CYCLE_US)
    return out


def message_pulses(word: int, mark_us: float = MARK_CYCLES * CYCLE_US) -> Pulses:
    """The demodulated message: 18 marks of ``mark_us`` at their starts, the
    spaces between (the stop bit's pause is the idle line after the list)."""
    starts = mark_starts(word)
    out: Pulses = []
    for a, b in zip(starts, starts[1:]):
        out += [(MARK, mark_us), (SPACE, b - a - mark_us)]
    out.append((MARK, mark_us))
    return out


def burst_cycles(word: int) -> List[int]:
    """Carrier cycles in each of the 18 bursts."""
    return [MARK_CYCLES] * MARKS


def message_us(word: int) -> float:
    """Start mark's start to the stop bit's end."""
    return (sum(interval_cycles(word)) + STOP_CYCLES) * CYCLE_US


# ------------------------------------------------------------- the decoder
@dataclasses.dataclass
class PfMessage:
    """A decoded message: ``word`` as received, ``start_us`` its start mark,
    ``ok`` its LRC right."""

    word: int
    start_us: float
    ok: bool


def classify(us: float) -> Optional[str]:
    """One mark-to-mark interval against the document's windows."""
    if LOW_US[0] <= us < LOW_US[1]:
        return "low"
    if HIGH_US[0] <= us < HIGH_US[1]:
        return "high"
    if STARTSTOP_US[0] <= us <= STARTSTOP_US[1]:
        return "startstop"
    return None


def decode_starts(starts_us: Sequence[float]) -> List[PfMessage]:
    """Messages from a list of mark start times, as "Receiving Messages"
    describes: a start bit, then 16 bits each in its window; a bit out of
    the windows ends the attempt and the search for a start bit goes on from
    that mark. The stop mark must not be followed by a mark within the stop
    bit's lower bound (947 us). Every message is returned, with its LRC
    verdict."""
    out: List[PfMessage] = []
    k = 0
    n = len(starts_us)
    while k + MARKS - 1 < n:
        if classify(starts_us[k + 1] - starts_us[k]) != "startstop":
            k += 1
            continue
        word, j = 0, k + 1
        while j < k + MARKS - 1:
            c = classify(starts_us[j + 1] - starts_us[j])
            if c not in ("low", "high"):
                break
            word = (word << 1) | (c == "high")
            j += 1
        if j < k + MARKS - 1:
            k = j
            continue
        stop = starts_us[j]
        if j + 1 < n and starts_us[j + 1] - stop < STARTSTOP_US[0]:
            k = j + 1
            continue
        out.append(PfMessage(word, starts_us[k], lrc_ok(word)))
        k = j + 1
    return out


def decode_pulses(pulses: Sequence[Tuple[int, float]]) -> List[PfMessage]:
    """:func:`decode_starts` on a pulse list's mark starts."""
    starts, t = [], 0.0
    for level, us in pulses:
        if level == MARK:
            starts.append(t)
        t += us
    return decode_starts(starts)


class RepeatFilter:
    """Copies dropped: a message equal, toggle included, to the last one
    delivered is not delivered again unless the line was dark for more than
    ``forget_us`` before it (the toggle changes for every new command,
    "LEGO Power Functions RC Protocol"). ``forget_us`` must exceed
    :data:`MAX_DARK_US` so no copy of one sequence is delivered twice."""

    def __init__(self, forget_us: float = FORGET_US) -> None:
        assert forget_us > MAX_DARK_US
        self.forget_us = forget_us
        self.last: Optional[int] = None
        self.last_end_us: Optional[float] = None

    def feed(self, msgs: Sequence[PfMessage]) -> List[int]:
        out = []
        for m in msgs:
            if not m.ok:
                continue
            dark = (None if self.last_end_us is None
                    else m.start_us - self.last_end_us)
            if dark is not None and dark > self.forget_us:
                self.last = None
            if m.word != self.last:
                out.append(m.word)
                self.last = m.word
            self.last_end_us = m.start_us + sum(interval_cycles(m.word)) * CYCLE_US
        return out


# ------------------------------------------------------- the retransmission
def schedule_tm(channel: int, copies: int = 5) -> List[int]:
    """In tm: the delay before the first copy, then the start-to-start
    spacing of each later one ("Transmitting Messages"). Copies past the
    fifth (a held key) keep the last spacing."""
    _check_channel(channel)
    later = 6 + 2 * channel
    gaps = [5, 5] + [later] * max(0, copies - 3)
    return [4 - channel] + gaps[:copies - 1]


def check_schedule(starts_us: Sequence[float], channel: int, *,
                   tol_us: float, first_from_us: Optional[float] = None,
                   first_extra_us: float = 0.0) -> List[Tuple[int, float, float]]:
    """``(index, got, nominal)`` for every start-to-start spacing off by
    more than ``tol_us``; with ``first_from_us`` (the time the command was
    given) the first delay too, which may be up to ``first_extra_us`` late
    (a transmitter's set-up time)."""
    sched = schedule_tm(channel, len(starts_us))
    bad = []
    if first_from_us is not None:
        got = starts_us[0] - first_from_us
        nom = sched[0] * TM_US
        if not (nom - tol_us <= got <= nom + first_extra_us + tol_us):
            bad.append((0, got, nom))
    for k in range(1, len(starts_us)):
        got = starts_us[k] - starts_us[k - 1]
        nom = sched[k] * TM_US
        if abs(got - nom) > tol_us:
            bad.append((k, got, nom))
    return bad


def check_marks(starts_us: Sequence[float], word: int,
                tol_us: float) -> List[Tuple[int, float, float]]:
    """``(index, got, nominal)`` for every mark start (from the first) more
    than ``tol_us`` off the encoding, and for every interval outside the
    receiver windows."""
    nominal = mark_starts(word)
    bad = []
    if len(starts_us) != len(nominal):
        return [(-1, float(len(starts_us)), float(len(nominal)))]
    t0 = starts_us[0]
    for k, (g, n) in enumerate(zip(starts_us, nominal)):
        if abs((g - t0) - n) > tol_us:
            bad.append((k, g - t0, n))
    want = ["startstop"] + ["high" if b else "low" for b in bits_of(word)]
    for k, (a, b) in enumerate(zip(starts_us, starts_us[1:])):
        if classify(b - a) != want[k]:
            bad.append((k, b - a, nominal[k + 1] - nominal[k]))
    return bad


def group_bursts(starts_us: Sequence[float], gap_us: float = 2000.0) -> List[List[float]]:
    """Burst starts grouped into messages: a gap longer than ``gap_us``
    (more than any bit, 1184 us) starts a new one."""
    groups: List[List[float]] = []
    for t in starts_us:
        if not groups or t - groups[-1][-1] > gap_us:
            groups.append([])
        groups[-1].append(t)
    return groups
