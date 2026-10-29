"""DShot reference model: frame encoder, pulse-width decoder, telemetry ESC.

DShot has no formal standard. Every figure below names the source it was
taken from; nothing here was taken from ``firmware/dshot_*.loom``.

Sources (fetched 2026-10-28):

* **BF-DPWM**: Betaflight ``src/platform/common/stm32/dshot_dpwm.h`` (master
  81e82a6): ``MOTOR_DSHOT600_HZ 12 MHz`` (300: 6, 150: 3), ``MOTOR_BIT_0 7``,
  ``MOTOR_BIT_1 14``, ``MOTOR_BITLENGTH 20`` (a bit is 20 timer units, high
  for 7 of them for a 0 and 14 for a 1), ``DSHOT_TELEMETRY_DEADTIME_US
  (30 + 5)``, ``DSHOT_DMA_BUFFER_SIZE 18 /* resolution + frame reset (2us) */``
  (16 bits then two bit periods of idle line).
* **BF-DSHOT**: Betaflight ``src/main/drivers/dshot.c``
  ``prepareDshotPacket``: ``packet = value << 1 | requestTelemetry``; the
  checksum is the XOR of the packet's three nibbles, inverted when
  bidirectional telemetry is on; ``frame = packet << 4 | csum``. Also
  ``dshot.h``: ``DSHOT_MIN_THROTTLE 48``, ``DSHOT_MAX_THROTTLE 2047``.
* **BF-TLM**: Betaflight ``src/platform/common/stm32/
  pwm_output_dshot_shared.c`` ``decodeTelemetryPacket``: the reply is 21
  line bits, the first edge always falling, each edge a GCR 1; the 5b/4b
  table ``decode[32]``; the 16 decoded bits are valid iff their four
  nibbles XOR to 0xF; the value is ``decoded >> 4`` (eeem mmmm mmmm).
* **BLUEJAY**: Bluejay ``src/Modules/DShot.asm`` (0368d11): "Compute
  inverted xor checksum (4-bit)" for the telemetry word; ``src/Bluejay.asm``:
  telemetry at ``5/4 * 600000`` = 750 kbit/s for DShot600, and the frame
  length criterion ``DShot_Frame_Length_Thr`` (40 for DShot600, 80 for
  DShot300) in Timer2 ticks of SYSCLK/12 at 24 MHz, so 2 us a tick: a frame
  is accepted when it lasts between Thr and 2 Thr, 20 to 40 us at DShot600.
  The 2 us tick is read from the source's comments, not from a datasheet:
  **unchecked**.
* **HANDBOOK**: "DSHOT - the missing Handbook", brushlesswhoop.com,
  sections "Frame Structure", "Calculating the CRC", "Bidirectional DSHOT",
  "eRPM Telemetry Frame", "eRPM Transmission": DShot600 bit 1.67 us, T1H
  1.25 us, T0H 0.625 us; the inverted signal and checksum of bidirectional
  DShot; the reply about 30 us after the frame, at 5/4 of the bit rate; the
  GCR nibble table; the 21-bit line code (a GCR 1 toggles the level).

Where the sources disagree (``QUESTIONS.md`` in the agent's scratch
directory has the full notes):

* high times: BF-DPWM 7/20 and 14/20 of a bit (0.583 / 1.167 us at
  DShot600), HANDBOOK 3/8 and 3/4 (0.625 / 1.25 us). No source states a
  tolerance. The decoder's windows span both nominal values and are widened
  by one Betaflight timer unit (1/20 bit, :data:`TOL_BITS`) each way, the
  resolution Betaflight itself states the times in: T0H 0.30 .. 0.425 bit,
  T1H 0.65 .. 0.80 bit, period 0.95 .. 1.05 bit. These are the model's own
  choice, made narrow on purpose: a transmitter inside them is inside every
  source's nominal plus one unit of the reference implementation;
* the telemetry checksum: HANDBOOK says "sent uninverted"; BF-TLM accepts
  only nibbles that XOR to 0xF and BLUEJAY computes an inverted XOR, which
  agree with each other. The model follows BF-TLM and BLUEJAY (the decoder
  that is actually flown).

The models are :class:`~tools.protomodels.bench.Model` s, registered
circuits clocked by the core clock, and count time in core clocks:
``clock_mhz`` (50 by default) and ``rate`` (bit/s, 600000 by default)
turn every figure into clocks.
"""

from __future__ import annotations

import dataclasses
from typing import Callable, List, Optional, Sequence

from .bench import Drive, Lines, Model, PinSpec, pad_of

#: BF-DPWM: timer units per bit, and the high time of a 0 and a 1 in them.
BITLENGTH, BIT_0, BIT_1 = 20, 7, 14
#: HANDBOOK: the high times as fractions of a bit (0.625 / 1.67, 1.25 / 1.67).
HB_T0H, HB_T1H = 3 / 8, 3 / 4
#: The decoder's tolerance beyond both nominals, in bits (one BF timer unit).
TOL_BITS = 1 / BITLENGTH
T0H_WINDOW = (BIT_0 / BITLENGTH - TOL_BITS, HB_T0H + TOL_BITS)    # 0.30 .. 0.425
T1H_WINDOW = (BIT_1 / BITLENGTH - TOL_BITS, HB_T1H + TOL_BITS)    # 0.65 .. 0.80
PERIOD_WINDOW = (1 - TOL_BITS, 1 + TOL_BITS)                      # 0.95 .. 1.05
#: BF-DPWM: the idle line after a frame, in bits ("frame reset (2us)").
FRAME_RESET_BITS = 2
#: The decoder ends a frame 1.5 bits after its last leading edge: later than
#: any period the window allows, so a long period is judged as a period,
#: and sooner than the next frame may start (1 + FRAME_RESET_BITS).
CLOSE_BITS = 1.5
#: BLUEJAY: a frame lasts between Thr and 2 * Thr: 20 .. 40 us at DShot600,
#: 12 bit periods to 24 (unchecked, see above).
FRAME_LEN_BITS = (12.0, 24.0)
#: BF-DSHOT: the throttle range; 1 .. 47 are special commands, 0 disarms.
MIN_THROTTLE, MAX_THROTTLE = 48, 2047
#: HANDBOOK, BLUEJAY: the reply's bit rate is 5/4 of the frame's.
TLM_RATE_FACTOR = 5 / 4
#: HANDBOOK, BF-DPWM: the reply comes about 30 us after the frame.
TLM_DELAY_US = 30.0
#: HANDBOOK / BF-TLM: nibble -> 5-bit GCR code.
GCR_ENCODE = (0x19, 0x1B, 0x12, 0x13, 0x1D, 0x15, 0x16, 0x17,
              0x1A, 0x09, 0x0A, 0x0B, 0x1E, 0x0D, 0x0E, 0x0F)
#: BF-TLM decode[32], with the codes GCR does not use as None.
GCR_DECODE = {code: nibble for nibble, code in enumerate(GCR_ENCODE)}


def checksum(packet: int, inverted: bool = False) -> int:
    """BF-DSHOT: XOR of the 12-bit packet's three nibbles, inverted if bidir."""
    packet &= 0xFFF
    csum = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0xF
    return csum ^ 0xF if inverted else csum


def packet_of(value: int, telemetry: bool = False) -> int:
    """BF-DSHOT: ``value << 1 | requestTelemetry`` (value 0 .. 2047)."""
    if not 0 <= value <= 2047:
        raise ValueError("DShot value %d outside 0..2047" % value)
    return (value << 1) | (1 if telemetry else 0)


def encode_frame(value: int, telemetry: bool = False, bidir: bool = False) -> int:
    """The 16-bit frame, MSB first on the wire."""
    packet = packet_of(value, telemetry)
    return (packet << 4) | checksum(packet, bidir)


def frame_ok(frame: int, bidir: bool = False) -> bool:
    """Is the checksum of a received 16-bit frame right?"""
    return (frame & 0xF) == checksum(frame >> 4, bidir)


def telemetry_word(value: int) -> int:
    """The ESC's 16-bit word for a 12-bit telemetry value, checksum inverted
    so that the four nibbles XOR to 0xF (BF-TLM, BLUEJAY)."""
    value &= 0xFFF
    return (value << 4) | checksum(value, inverted=True)


def erpm_value(period_us: int) -> int:
    """BF-TLM's eeem mmmm mmmm for an eRPM period in us: the smallest
    exponent whose 9-bit mantissa holds it (the inverse of Betaflight's
    ``(value & 0x1ff) << (value >> 9)``, exact when the period allows)."""
    if period_us <= 0:
        raise ValueError("period must be positive")
    exp = 0
    while (period_us >> exp) > 0x1FF:
        exp += 1
    if exp > 7:
        raise ValueError("period %d us does not fit eeem mmmm mmmm" % period_us)
    return (exp << 9) | (period_us >> exp)


def gcr_bits(word: int) -> int:
    """HANDBOOK / BF-TLM: the 20 GCR bits of a 16-bit word, nibbles MSB first."""
    out = 0
    for shift in (12, 8, 4, 0):
        out = (out << 5) | GCR_ENCODE[(word >> shift) & 0xF]
    return out


def line_levels(gcr20: int) -> List[int]:
    """The 21 line levels of a reply: a 0, then each GCR 1 toggles the level
    (HANDBOOK "eRPM Transmission"; BF-TLM decodes each edge as a 1)."""
    levels = [0]
    for i in range(19, -1, -1):
        levels.append(levels[-1] ^ ((gcr20 >> i) & 1))
    return levels


def decode_levels(levels: Sequence[int]) -> Optional[int]:
    """The inverse, BF-TLM's way: the 16-bit word, or None if a 5-bit code is
    not GCR. The checksum is the caller's (:func:`telemetry_ok`)."""
    if len(levels) != 21 or levels[0] != 0:
        return None
    gcr = 0
    for prev, cur in zip(levels, levels[1:]):
        gcr = (gcr << 1) | (prev ^ cur)
    word = 0
    for shift in (15, 10, 5, 0):
        nibble = GCR_DECODE.get((gcr >> shift) & 0x1F)
        if nibble is None:
            return None
        word = (word << 4) | nibble
    return word


def telemetry_ok(word: int) -> bool:
    """BF-TLM: the four nibbles XOR to 0xF."""
    x = word ^ (word >> 8)
    return ((x ^ (x >> 4)) & 0xF) == 0xF


@dataclasses.dataclass
class DshotFrame:
    """One frame seen on the line."""

    frame: int                      # the 16 bits, MSB first
    start: int                      # cycle of the first bit's leading edge
    edges: List[int]                # every edge of the frame, in cycles
    highs: List[int]                # the active (pulse) time of each bit
    periods: List[int]              # leading edge to leading edge, 15 of them
    bidir: bool

    @property
    def packet(self) -> int:
        return self.frame >> 4

    @property
    def value(self) -> int:
        return self.frame >> 5

    @property
    def telemetry(self) -> bool:
        return bool((self.frame >> 4) & 1)

    @property
    def crc_ok(self) -> bool:
        return frame_ok(self.frame, self.bidir)


@dataclasses.dataclass
class Violation:
    """One pulse, period, gap or frame no source allows."""

    kind: str                       # "high" | "period" | "gap" | "frame" | "crc"
    cycle: int
    clocks: float
    low: float
    high: float
    detail: str = ""

    def __str__(self) -> str:                               # pragma: no cover
        return ("%s of %g clocks at cycle %d, outside %g .. %g%s"
                % (self.kind, self.clocks, self.cycle, self.low, self.high,
                   ": " + self.detail if self.detail else ""))


class DshotRx(Model):
    """Decodes DShot frames on ``pin`` and checks every pulse as it ends.

    The pulse is the line at its active level: high for normal DShot, low
    for bidirectional (``bidir=True``, HANDBOOK). Each leading edge starts a
    bit; the pulse's length decides it, and must be inside
    :data:`T0H_WINDOW` or :data:`T1H_WINDOW` (in bits, which do not
    overlap); each leading edge to the next must be inside
    :data:`PERIOD_WINDOW`. A frame ends when no leading edge comes within
    :data:`CLOSE_BITS`; it must have 16 bits, last :data:`FRAME_LEN_BITS`
    from its first leading edge to its last edge, have a good checksum, and
    start at least :data:`FRAME_RESET_BITS` after the end of the previous
    frame's last period. Anything else is a :class:`Violation` (raised at
    once with ``strict=True``); the frame is still recorded.

    ``listening`` can be cleared by a model that drives the same line (the
    ESC while it answers), so its own edges are not taken for a frame.
    """

    def __init__(self, pin: PinSpec, *, clock_mhz: float = 50.0,
                 rate: float = 600000.0, bidir: bool = False,
                 strict: bool = False,
                 on_frame: Optional[Callable[[DshotFrame], None]] = None) -> None:
        self.pad = pad_of(pin)
        self.bit = clock_mhz * 1e6 / rate          # clocks per bit
        self.bidir = bidir
        self.active = 0 if bidir else 1
        self.strict = strict
        self.on_frame = on_frame
        self.listening = True
        self.frames: List[DshotFrame] = []
        self.violations: List[Violation] = []
        self.level = 1 - self.active               # the line idles inactive
        self._starts: List[int] = []
        self._edges: List[int] = []
        self._highs: List[int] = []
        self._bits: List[int] = []
        self._prev_end: Optional[float] = None     # end of the last frame's period
        self.last_cycle = -1

    # ---------------------------------------------------------- helpers
    def _window(self, frac):
        return frac[0] * self.bit, frac[1] * self.bit

    def _check(self, kind, cycle, clocks, window, detail=""):
        lo, hi = window
        if lo <= clocks <= hi:
            return True
        v = Violation(kind, cycle, clocks, lo, hi, detail)
        self.violations.append(v)
        if self.strict:
            raise AssertionError(str(v))
        return False

    @property
    def in_frame(self) -> bool:
        return bool(self._starts)

    # ------------------------------------------------------------ clock
    def observe(self, lines: Lines) -> None:
        c = lines.cycle
        self.last_cycle = c
        v = lines.get(self.pad)
        if not self.listening:
            self.level = v
            return
        if v != self.level:
            self.level = v
            self._edges.append(c)
            if v == self.active:
                self._leading(c)
            elif self._starts:
                self._trailing(c)
        elif (self._starts and v != self.active
              and c - self._starts[-1] > CLOSE_BITS * self.bit):
            self._close(c)

    def _leading(self, c: int) -> None:
        if self._starts:
            self._check("period", c, c - self._starts[-1],
                        self._window(PERIOD_WINDOW), "bit %d" % len(self._starts))
        elif self._prev_end is not None:
            self._check("gap", c, c - self._prev_end,
                        (FRAME_RESET_BITS * self.bit, float("inf")),
                        "idle line before the frame")
        self._starts.append(c)

    def _trailing(self, c: int) -> None:
        high = c - self._starts[-1]
        self._highs.append(high)
        lo0, hi0 = self._window(T0H_WINDOW)
        lo1, hi1 = self._window(T1H_WINDOW)
        if lo0 <= high <= hi0:
            self._bits.append(0)
        elif lo1 <= high <= hi1:
            self._bits.append(1)
        else:
            # (lo0, hi1) spans both windows, so judge against the empty gap
            # window: a pulse here is always a violation.
            self._check("high", c, high, (float("inf"), float("-inf")),
                        "in neither T0H %g..%g nor T1H %g..%g clocks"
                        % (lo0, hi0, lo1, hi1))
            self._bits.append(1 if high > self.bit / 2 else 0)

    def _close(self, c: int) -> None:
        starts, bits = self._starts, self._bits
        frame = 0
        for b in bits:
            frame = (frame << 1) | b
        edges = [e for e in self._edges if e >= starts[0]]
        rec = DshotFrame(frame=frame & 0xFFFF, start=starts[0], edges=edges,
                         highs=list(self._highs),
                         periods=[b - a for a, b in zip(starts, starts[1:])],
                         bidir=self.bidir)
        if len(bits) != 16:
            self._check("frame", c, len(bits), (16, 16), "bits in the frame")
        else:
            self._check("frame", c, edges[-1] - starts[0],
                        self._window(FRAME_LEN_BITS), "frame length (BLUEJAY)")
            if not rec.crc_ok:
                self._check("crc", c, frame & 0xF, (-1, -1),
                            "checksum of frame 0x%04X" % frame)
        self.frames.append(rec)
        self._prev_end = starts[-1] + self.bit
        self._starts, self._bits, self._highs, self._edges = [], [], [], []
        if self.on_frame is not None:
            self.on_frame(rec)

    def report(self) -> str:                                # pragma: no cover
        return "; ".join(str(v) for v in self.violations[:8]) or "no violations"


@dataclasses.dataclass
class Reply:
    """One telemetry reply the ESC model sent (or chose not to)."""

    frame: DshotFrame               # the frame it answers
    start: Optional[int]            # cycle of the reply's first fall, None if none
    word: Optional[int]             # the 16-bit word GCR-encoded, None if none
    fault: str                      # "" | "none" | "crc" | "gcr"


class DshotEsc(Model):
    """A bidirectional-DShot ESC on a ``uio`` line with a pull-up.

    It decodes the FC's inverted frames with a :class:`DshotRx` (so every
    pulse is checked against the windows) and answers each frame whose
    checksum is right with a telemetry reply: ``delay_us`` (30, HANDBOOK)
    after the frame's last edge, 21 line bits at 5/4 of the frame bit rate
    (scaled by ``1 + skew`` to model the ESC's own clock error), driven
    push-pull, then the line released. ``values`` supplies the 12-bit
    telemetry value per frame (a list, used in turn and then repeated from
    its last entry, or a callable taking the frame). Error injection, one
    entry per reply in ``faults`` (then ""): ``"none"`` sends nothing,
    ``"crc"`` flips the checksum's low bit, ``"gcr"`` replaces the first
    code with ``bad_code`` (0x00 by default, a code GCR never uses).
    """

    def __init__(self, pin: PinSpec, *, clock_mhz: float = 50.0,
                 rate: float = 600000.0, delay_us: float = TLM_DELAY_US,
                 skew: float = 0.0, values=(0x0FFF,),
                 faults: Sequence[str] = (), bad_code: int = 0x00) -> None:
        self.pad = pad_of(pin)
        self.clock_mhz = clock_mhz
        self.rx = DshotRx(pin, clock_mhz=clock_mhz, rate=rate, bidir=True,
                          on_frame=self._answer)
        self.tbit = clock_mhz * 1e6 / (rate * TLM_RATE_FACTOR) * (1 + skew)
        self.delay = delay_us * clock_mhz
        self.values = values
        self.faults = list(faults)
        self.bad_code = bad_code & 0x1F
        self.replies: List[Reply] = []
        self._levels: Optional[List[int]] = None
        self._start = 0

    def _value(self, frame: DshotFrame) -> int:
        if callable(self.values):
            return self.values(frame) & 0xFFF
        index = min(len(self.replies), len(self.values) - 1)
        return self.values[index] & 0xFFF

    def _answer(self, frame: DshotFrame) -> None:
        if not frame.crc_ok:
            return                          # an ESC ignores a bad frame
        fault = self.faults.pop(0) if self.faults else ""
        if fault == "none":
            self.replies.append(Reply(frame, None, None, fault))
            return
        word = telemetry_word(self._value(frame))
        if fault == "crc":
            word ^= 1
        gcr = gcr_bits(word)
        if fault == "gcr":
            gcr = (gcr & 0x7FFF) | (self.bad_code << 15)
        start = int(frame.edges[-1] + self.delay)
        if start <= self.rx.last_cycle:
            raise AssertionError("reply delay shorter than the frame-end detection")
        self._levels = line_levels(gcr)
        self._start = start
        self.replies.append(Reply(frame, start, word, fault))

    def drive(self, drive: Drive, cycle: int) -> None:
        if self._levels is None or cycle < self._start:
            return
        index = int((cycle - self._start) / self.tbit)
        if index >= len(self._levels):
            self._levels = None             # done: release the line
            return
        self.rx.listening = False
        drive.set(self.pad, self._levels[index])

    def observe(self, lines: Lines) -> None:
        if self._levels is None:
            self.rx.listening = True
        self.rx.observe(lines)

    # the decoder's results, for tests
    @property
    def frames(self) -> List[DshotFrame]:
        return self.rx.frames

    @property
    def violations(self) -> List[Violation]:
        return self.rx.violations
