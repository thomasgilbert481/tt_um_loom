"""Wiegand reference model: a card reader that sends frames, a panel that
checks every pulse, the 26-bit and 37-bit card formats, and error injection.

Written from the protocol's description, not from the firmware
(``firmware/wiegand_reader.loom``, ``firmware/wiegand_panel.loom``).

The wire. Two open-collector lines, D0 and D1, idle high through the panel's
pull-ups. A 0 is a low pulse on D0, a 1 a low pulse on D1, and the two are
never low at once. The bits go out first bit first; a frame ends when no
pulse comes for longer than the widest interval.

Sources and what was checked:

* SIA AC-01 (1996), "Access Control Standard Protocol for the 26-Bit
  Wiegand Reader Interface", Security Industry Association. The primary
  document was **not reachable** (it is sold by SIA). Its timing is taken as
  quoted by two secondary sources that agree: US patent 8,943,562 (the
  background section, which cites the standard) and Suprema's support
  article "BioStar 2: Configuring pulse width and interval for Wiegand
  input" (article 24000028006, which gives the same "SIA standard" ranges
  for its devices' inputs):

  - pulse width ``PW`` (``Tpw``): 20 us minimum, 100 us maximum;
  - pulse interval ``PI`` (``Tpi``, fall to fall): 200 us minimum, 20 ms
    maximum.

  So these four figures are **checked against two secondary sources, not
  against the standard itself**. The "typical" 50 us pulse every 1 ms
  (:data:`TYPICAL_PW_US`, :data:`TYPICAL_PI_US`) is from the same patent
  text and is only a convenient default, not a limit.
* The inter-frame time is not a figure the secondary sources give. A
  panel must wait longer than the widest legal interval before it ends a
  frame, so :data:`FRAME_GAP_US` is 25 ms, 5 ms more than ``PI`` max:
  **unchecked**, a choice made here.
* The 26-bit format (HID H10301, the format SIA AC-01 is named for): bit 1
  even parity over bits 2 to 13, bits 2 to 9 the facility code (8 bits),
  bits 10 to 25 the card number (16 bits), bit 26 odd parity over bits 14
  to 25. Bits are numbered from 1 in transmission order. This is the
  layout the task brief states and the one HID's "Understanding Card Data
  Formats" white paper gives for H10301; that paper was not opened in this
  session, so the layout rests on the brief and on how widely it is
  published, not on a document read here.
* The 37-bit format, HID H10304: bit 1 even parity over bits 2 to 19,
  bits 2 to 17 the facility code (16 bits), bits 18 to 36 the card number
  (19 bits), bit 37 odd parity over bits 19 to 36. Vendor listings confirm
  37 bits and a 19-bit card field (numbers up to 524,287); the field
  split and the parity spans are from HID's white paper as remembered and
  were **not checked** against a copy of it in this session (a web search
  found no page giving them), so :func:`h10304` is unchecked.

Time is in core clocks, as everywhere in ``tools/protomodels``: the models
take ``clock_hz`` and convert the microsecond windows.
"""

from __future__ import annotations

import random
from collections import deque
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from .bench import UI, UIO, Drive, Lines, Model, PinSpec, pad_of

# ------------------------------------------------------------- the windows
PW_MIN_US, PW_MAX_US = 20.0, 100.0          # SIA AC-01 via two secondary sources
PI_MIN_US, PI_MAX_US = 200.0, 20000.0       # fall to fall, the same sources
TYPICAL_PW_US, TYPICAL_PI_US = 50.0, 1000.0  # "typical", US 8,943,562
FRAME_GAP_US = 25000.0                      # unchecked: PI max + 5 ms (above)


def us_to_clocks(us: float, clock_hz: float) -> int:
    return int(round(us * clock_hz / 1e6))


# ------------------------------------------------------------- the formats
def parity(value: int) -> int:
    """XOR of the bits of ``value``."""
    return bin(value).count("1") & 1


def bit_list(value: int, n: int) -> List[int]:
    """``value`` as ``n`` bits in transmission order (most significant first)."""
    return [(value >> (n - 1 - i)) & 1 for i in range(n)]


def from_bits(bits: Sequence[int]) -> int:
    value = 0
    for bit in bits:
        value = (value << 1) | (bit & 1)
    return value


def _with_parities(data: int, n_data: int, even_span: Tuple[int, int],
                   odd_span: Tuple[int, int]) -> int:
    """A frame of ``n_data + 2`` bits: an even parity bit, the data, an odd
    parity bit. Spans are 1-based bit numbers of the whole frame, inclusive."""
    n = n_data + 2
    bits = [0] + bit_list(data, n_data) + [0]
    lo, hi = even_span
    bits[0] = parity(from_bits(bits[lo - 1:hi]))
    lo, hi = odd_span
    bits[-1] = 1 - parity(from_bits(bits[lo - 1:hi]))
    assert len(bits) == n
    return from_bits(bits)


def h10301(facility: int, card: int) -> int:
    """The 26-bit frame for an 8-bit facility code and a 16-bit card number."""
    if not (0 <= facility < 1 << 8 and 0 <= card < 1 << 16):
        raise ValueError("26-bit format: facility 0..255, card 0..65535")
    return _with_parities((facility << 16) | card, 24, (2, 13), (14, 25))


def h10304(facility: int, card: int) -> int:
    """The 37-bit frame for a 16-bit facility code and a 19-bit card number
    (parity spans unchecked: module docstring)."""
    if not (0 <= facility < 1 << 16 and 0 <= card < 1 << 19):
        raise ValueError("37-bit format: facility 0..65535, card 0..524287")
    return _with_parities((facility << 19) | card, 35, (2, 19), (19, 36))


def check_parities(value: int, n: int) -> bool:
    """True when the frame's two parity bits are right (26 or 37 bits)."""
    spans = {26: ((2, 13), (14, 25)), 37: ((2, 19), (19, 36))}[n]
    bits = bit_list(value, n)
    (elo, ehi), (olo, ohi) = spans
    even_ok = parity(from_bits(bits[elo - 1:ehi])) == bits[0]
    odd_ok = 1 - parity(from_bits(bits[olo - 1:ohi])) == bits[-1]
    return even_ok and odd_ok


def decode26(value: int) -> Tuple[int, int]:
    """``(facility, card)`` of a 26-bit frame (parities not checked)."""
    return (value >> 17) & 0xFF, (value >> 1) & 0xFFFF


def to_words(value: int, n: int) -> List[int]:
    """The frame as ceil(n / 16) words, most significant first, right-aligned."""
    count = (n + 15) // 16
    return [(value >> (16 * (count - 1 - i))) & 0xFFFF for i in range(count)]


def from_words(words: Sequence[int]) -> int:
    value = 0
    for word in words:
        value = (value << 16) | (word & 0xFFFF)
    return value


# --------------------------------------------------------------- the reader
class WiegandReader(Model):
    """A card reader that sends queued frames on D0 and D1.

    A ``ui`` pad (the chip's IN pins) is driven push-pull, high when idle;
    a ``uio`` pad is pulled low and released (open collector), so the bench's
    pull-ups make the high level, as on a real panel.

    Args:
        d0, d1: the two pads.
        clock_hz: core clock, for the microsecond defaults.
        pw, pi: pulse width and fall-to-fall interval in clocks (default the
            typical 50 us and 1 ms).
        jitter: each fall moves by a uniform whole number of clocks in
            ``-jitter .. +jitter`` from its place on the ``pi`` grid (no
            accumulation), from ``random.Random(seed)``.
        gap: idle clocks after a frame's last pulse before the next frame
            may start (default :data:`FRAME_GAP_US` plus 1 ms).
    """

    def __init__(self, d0: PinSpec, d1: PinSpec, *, clock_hz: float = 50e6,
                 pw: Optional[int] = None, pi: Optional[int] = None,
                 jitter: int = 0, gap: Optional[int] = None, seed: int = 1) -> None:
        self.pads = (pad_of(d0), pad_of(d1))
        self.clock_hz = float(clock_hz)
        self.pw = us_to_clocks(TYPICAL_PW_US, clock_hz) if pw is None else int(pw)
        self.pi = us_to_clocks(TYPICAL_PI_US, clock_hz) if pi is None else int(pi)
        self.jitter = int(jitter)
        self.gap = (us_to_clocks(FRAME_GAP_US + 1000, clock_hz) if gap is None
                    else int(gap))
        self.rng = random.Random(seed)
        self._events: deque = deque()           # (cycle, line, +1 low / -1)
        self._low = [0, 0]
        self._free_at = 0
        self.now = 0
        #: per frame: ``{"start", "n", "value", "pulses": [(line, fall, rise)]}``
        self.frames_sent: List[Dict] = []

    def send(self, value: int, n: int, *, start: Optional[int] = None,
             drop: Iterable[int] = (), both: Iterable[int] = (),
             glitches: Iterable[Tuple[int, int, int]] = (),
             pw: Optional[int] = None, pi: Optional[int] = None) -> int:
        """Queue an ``n``-bit frame; returns the cycle of its first fall.

        Faults, by bit index (0 = the first bit sent): ``drop`` leaves that
        bit's pulse out (its slot stays empty: a missing bit); ``both``
        pulls both lines for that bit; ``glitches`` holds ``(k, line,
        width)``: a low pulse of ``width`` clocks on ``line`` halfway through
        the interval after bit ``k``.
        """
        pw = self.pw if pw is None else int(pw)
        pi = self.pi if pi is None else int(pi)
        t0 = max(self._free_at, self.now + 1)
        if start is not None:
            t0 = max(t0, int(start))
        drop, both = set(drop), set(both)
        pulses = []
        last_end = t0
        for k, bit in enumerate(bit_list(value, n)):
            fall = t0 + k * pi
            if self.jitter and k:
                fall += self.rng.randint(-self.jitter, self.jitter)
            if k in drop:
                continue
            lines = (0, 1) if k in both else (bit,)
            for line in lines:
                pulses.append((line, fall, fall + pw))
            last_end = max(last_end, fall + pw)
        for k, line, width in glitches:
            at = t0 + k * pi + pi // 2
            pulses.append((line, at, at + int(width)))
            last_end = max(last_end, at + int(width))
        events = []
        for line, fall, rise in pulses:
            events.append((fall, line, 1))
            events.append((rise, line, -1))
        events.sort()
        self._events.extend(events)
        self._free_at = last_end + self.gap
        self.frames_sent.append({"start": t0, "n": n, "value": value,
                                 "pulses": sorted(pulses, key=lambda p: p[1])})
        return t0

    @property
    def busy(self) -> bool:
        return bool(self._events)

    def idle_at(self) -> int:
        return self._free_at

    def drive(self, drive: Drive, cycle: int) -> None:
        self.now = cycle
        events = self._events
        while events and events[0][0] <= cycle:
            _, line, delta = events.popleft()
            self._low[line] += delta
        for line in (0, 1):
            pad = self.pads[line]
            low = self._low[line] > 0
            if pad[0] == UIO:
                if low:
                    drive.pull_low(pad)
            else:
                drive.set(pad, 0 if low else 1)


# ---------------------------------------------------------------- the panel
class WiegandPanel(Model):
    """A panel that watches D0 and D1, checks every pulse against SIA AC-01's
    windows and decodes the frames. It never drives the lines.

    A pulse is a low level on one line, from its fall to its rise. Each one
    is checked: its width against ``PW_MIN_US .. PW_MAX_US``, the interval
    from the previous pulse's fall against ``PI_MIN_US .. PI_MAX_US`` (inside
    a frame), and the other line, which must be high for the whole pulse.
    A frame ends when no fall comes for :data:`FRAME_GAP_US` after the last
    rise; :meth:`flush` ends the one in progress.

    ``violations`` holds ``(cycle, kind, detail)`` for each failed check:
    kinds ``"width"``, ``"interval"`` and ``"both"``. ``frames`` holds
    ``{"n", "value", "bits", "pulses"}`` per frame, with a pulse on both
    lines read as no bit at all.
    """

    def __init__(self, d0: PinSpec, d1: PinSpec, *, clock_hz: float = 50e6,
                 frame_gap_us: float = FRAME_GAP_US) -> None:
        self.pads = (pad_of(d0), pad_of(d1))
        c = float(clock_hz)
        self.pw_window = (us_to_clocks(PW_MIN_US, c), us_to_clocks(PW_MAX_US, c))
        self.pi_window = (us_to_clocks(PI_MIN_US, c), us_to_clocks(PI_MAX_US, c))
        self.gap = us_to_clocks(frame_gap_us, c)
        self.prev = [1, 1]
        self.fall_at: List[Optional[int]] = [None, None]
        self.overlap = [False, False]
        self.last_fall: Optional[int] = None
        self.last_rise: Optional[int] = None
        self.cycle = -1
        self._pulses: List[Tuple[int, int, int]] = []
        #: every pulse ever seen: ``(line, fall, rise)``
        self.pulses: List[Tuple[int, int, int]] = []
        self.frames: List[Dict] = []
        self.violations: List[Tuple[int, str, str]] = []

    def observe(self, lines: Lines) -> None:
        cycle = self.cycle = lines.cycle
        level = [lines.get(self.pads[0]), lines.get(self.pads[1])]
        if (self._pulses and self.last_rise is not None and level == [1, 1]
                and cycle - self.last_rise > self.gap):
            self.flush()
        for line in (0, 1):
            if self.prev[line] and not level[line]:                 # a fall
                self.fall_at[line] = cycle
                self.overlap[line] = not level[1 - line]
                if (self.last_fall is not None and self._pulses):
                    gap = cycle - self.last_fall
                    lo, hi = self.pi_window
                    if not lo <= gap <= hi:
                        self.violations.append(
                            (cycle, "interval", "%d clocks, window %d..%d" % (gap, lo, hi)))
                self.last_fall = cycle
            elif not level[line] and not level[1 - line]:
                self.overlap[line] = True
            elif not self.prev[line] and level[line] and self.fall_at[line] is not None:
                fall = self.fall_at[line]                           # a rise
                width = cycle - fall
                lo, hi = self.pw_window
                if not lo <= width <= hi:
                    self.violations.append(
                        (cycle, "width", "D%d %d clocks, window %d..%d" % (line, width, lo, hi)))
                if self.overlap[line]:
                    self.violations.append((cycle, "both", "D%d low with D%d" % (line, 1 - line)))
                pulse = (line, fall, cycle)
                self._pulses.append(pulse)
                self.pulses.append(pulse)
                self.fall_at[line] = None
                self.last_rise = cycle
        self.prev = level

    def flush(self) -> None:
        """End the frame in progress, if any."""
        if not self._pulses:
            return
        pulses = sorted(self._pulses, key=lambda p: p[1])
        bits = []
        i = 0
        while i < len(pulses):
            line, fall, rise = pulses[i]
            if (i + 1 < len(pulses) and pulses[i + 1][1] < rise
                    and pulses[i + 1][0] != line):
                i += 2                              # both lines: no bit
                continue
            bits.append(line)
            i += 1
        self.frames.append({"n": len(bits), "value": from_bits(bits),
                            "bits": bits, "pulses": pulses})
        self._pulses = []
        self.last_fall = None


class Wire(Model):
    """A board wire from one pad to another: ``dst`` follows ``src`` one
    clock later, as a registered model must. Used to join a reader thread's
    open-drain BIDIR pins to a panel thread's IN pins."""

    def __init__(self, src: PinSpec, dst: PinSpec) -> None:
        self.src, self.dst = pad_of(src), pad_of(dst)
        self.level = 1

    def drive(self, drive: Drive, cycle: int) -> None:
        drive.set(self.dst, self.level)

    def observe(self, lines: Lines) -> None:
        self.level = lines.get(self.src)
