"""Futaba S.BUS reference models: frame packing, the inverted 8E2 line, and
hobby-servo pulses.

S.BUS has no public formal standard. Every figure below is taken from public
decoder and encoder source, which is a **secondary** source; files and lines
as fetched from each project's default branch on 2026-11-04:

* Betaflight ``src/main/rx/sbus.c``: line 59 ``SBUS_BAUDRATE 100000``;
  lines 81-83 ``SERIAL_STOPBITS_2 | SERIAL_PARITY_EVEN``; lines 205-208
  "On SBUS, SERIAL_INVERTED is default"; line 71 ``SBUS_FRAME_BEGIN_BYTE
  0x0F``; lines 98-99 the end byte is 0x00; lines 73 and 79
  ``SBUS_INTERBYTE_TIMEOUT_US`` = 5 byte times (about 600 us) and lines
  133-135, which restart a frame when it passes between two bytes.
* Betaflight ``src/main/rx/sbus_channels.h``: lines 30-49, 16 channels of
  11 bits packed LSB first, then the flags byte; lines 27-28 signal loss
  ``1 << 2``, failsafe ``1 << 3``. ``sbus_channels.c``: lines 38-39 channel
  17 ``1 << 0``, channel 18 ``1 << 1``; lines 41-42 the digital channel
  range 173 .. 1812; line 94 the scaling ``5 * x / 8 + 880`` microseconds.
* EdgeTX ``radio/src/pulses/sbus.cpp`` (an encoder): line 38 the begin byte
  0x0F, line 40 ``SBUS_CHAN_CENTER 992``, lines 67-78 the packing, lines
  89-90 the end byte 0x00, line 94 ``SBUS_BAUDRATE 100000``, line 101
  ``ETX_Encoding_8E2``, line 103 ``ETX_Pol_Inverted``.
* PX4 ``src/lib/rc/sbus.cpp``: lines 294-298 "we detect frame boundaries
  by the inter-frame delay", "the minimum frame spacing is 7ms; with 25
  bytes at 100000bps frame transmission time is 3ms"; lines 91-95 the
  range 200 .. 1800 maps to 1000 .. 2000 us; lines 592-614 end bytes 0x00
  (S.BUS) and 0x04, 0x14, 0x24, 0x34 (S.BUS2, not modelled here).
* ArduPilot ``libraries/AP_RCProtocol/AP_RCProtocol_SBUS.cpp``: line 60
  ``SBUS_FRAME_SIZE 25``, lines 78-79 a 2000 us frame gap, lines 67-72
  200 .. 1800 to 1000 .. 2000 us.

**Unchecked** (in none of the sources above): the 14 ms frame period of
the normal (not high-speed) mode, and the 20 ms (50 Hz) servo frame. Both
are the figures RC equipment is commonly described with; the tests use
them as the programs' defaults, not as limits.

Line coding (from the above): each byte is a start bit, 8 data bits LSB
first, an even-parity bit and two stop bits, at 100000 baud; the line is
inverted, so it idles **low**, a start bit is **high** and a stop bit low.
A frame is 25 bytes: 0x0F, 22 bytes holding channels 1 .. 16 (channel k is
bits ``11k .. 11k + 10`` of the 176-bit little-endian field), the flags
byte, 0x00.

As in ``uart.py``, bit periods are in core clocks and may be fractional;
edges are placed at ``start + round(k * bit_clocks)`` so a stream never
drifts.
"""

from __future__ import annotations

import dataclasses
import math
from collections import deque
from typing import Iterable, List, Optional, Sequence, Tuple

from .bench import Drive, Lines, Model, PinSpec, pad_of

BAUD = 100000
FRAME_BYTES = 25
BITS_PER_BYTE = 12                  # start, 8 data, parity, 2 stop
FRAME_BITS = FRAME_BYTES * BITS_PER_BYTE    # 300: 3 ms at 100000 baud
BEGIN = 0x0F
END = 0x00
CH17, CH18, FRAME_LOST, FAILSAFE = 0x01, 0x02, 0x04, 0x08
CHANNELS = 16
CH_BITS = 11
CENTRE = 992                        # EdgeTX SBUS_CHAN_CENTER
RAW_MIN, RAW_MAX = 173, 1812        # Betaflight's digital channel range
PERIOD_US = 14000                   # normal mode (unchecked, see above)
FAST_PERIOD_US = 7000               # high-speed mode (PX4 line 297)
INTERBYTE_TIMEOUT_US = 600          # Betaflight: 5 byte times between bytes
SERVO_FRAME_US = 20000              # 50 Hz (unchecked, see above)


# ------------------------------------------------------------------ frames
def pack(channels: Sequence[int], flags: int = 0) -> List[int]:
    """The 25 bytes of a frame: channels 1 .. 16 (11 bits each, LSB first)
    and the flags byte."""
    if len(channels) != CHANNELS:
        raise ValueError("a frame holds %d channels" % CHANNELS)
    field = 0
    for k, value in enumerate(channels):
        if not 0 <= value < 1 << CH_BITS:
            raise ValueError("channel %d: %d is not 11 bits" % (k + 1, value))
        field |= value << (CH_BITS * k)
    data = [(field >> (8 * i)) & 0xFF for i in range(22)]
    return [BEGIN] + data + [flags & 0xFF, END]


def unpack(frame: Sequence[int]) -> Tuple[List[int], int]:
    """``(channels, flags)`` of a 25-byte frame; ValueError if the begin or
    end byte is wrong or the length is not 25."""
    if len(frame) != FRAME_BYTES:
        raise ValueError("a frame is %d bytes, not %d" % (FRAME_BYTES, len(frame)))
    if frame[0] != BEGIN or frame[-1] != END:
        raise ValueError("begin 0x%02X, end 0x%02X" % (frame[0], frame[-1]))
    field = sum(b << (8 * i) for i, b in enumerate(frame[1:23]))
    mask = (1 << CH_BITS) - 1
    return [(field >> (CH_BITS * k)) & mask for k in range(CHANNELS)], frame[23]


def parity(value: int) -> int:
    """The even-parity bit of a byte: 1 when the byte has an odd number of 1s."""
    return bin(value & 0xFF).count("1") & 1


def byte_bits(value: int, *, bad_parity: bool = False,
              stop: Tuple[int, int] = (1, 1)) -> List[int]:
    """The 12 logical bits of one byte in line order (before the inversion):
    start 0, data LSB first, the parity bit, the two stop bits 1."""
    p = parity(value) ^ (1 if bad_parity else 0)
    return [0] + [(value >> i) & 1 for i in range(8)] + [p] + list(stop)


def servo_us(raw: int) -> float:
    """Betaflight's pulse width for a channel value, in microseconds, with
    the value clamped to the digital range first: 988 .. 2012 us."""
    raw = min(max(raw, RAW_MIN), RAW_MAX)
    return 5 * raw / 8 + 880


# ------------------------------------------------------------ line encoder
class SbusTx(Model):
    """Drives inverted 8E2 S.BUS onto a chip input: the line idles low, a
    start bit is high. ``send`` queues a whole frame from its channels and
    flags; ``send_bytes`` any byte list, with errors injected on purpose:

    * ``bad_parity``: byte indices whose parity bit is flipped;
    * ``stop_low``: byte indices whose first stop bit is a 0 (line high);
    * a short frame is just fewer bytes (``send_bytes(frame[:20])``);
    * the failsafe and frame-lost flags are ordinary flag bits (``send``).

    ``gap_bits`` is the idle time between bytes of a frame (0: back to
    back, as receivers send them); ``bit_clocks`` may be changed between
    frames to model a sender with a rate error. ``frames_sent`` holds
    ``(start, end)`` cycles: the first start edge, and the end of the last
    stop bit.
    """

    def __init__(self, pin: PinSpec, bit_clocks: float, *, gap_bits: float = 0.0) -> None:
        self.pad = pad_of(pin)
        self.bit_clocks = float(bit_clocks)
        self.gap_bits = gap_bits
        self.level = 0                      # idle: an inverted 1
        self.now = 0
        self._changes: deque = deque()
        self._free_at = 0.0
        self.frames_sent: List[Tuple[int, int]] = []

    def send(self, channels: Sequence[int], flags: int = 0, **kwargs) -> int:
        return self.send_bytes(pack(channels, flags), **kwargs)

    def send_bytes(self, data: Iterable[int], *, start: Optional[float] = None,
                   bad_parity: Iterable[int] = (), stop_low: Iterable[int] = ()) -> int:
        """Queue bytes back to back (``gap_bits`` apart); returns the first
        start edge's cycle. ``start`` delays it to at least that cycle."""
        bad, low = set(bad_parity), set(stop_low)
        t = max(self._free_at, float(self.now + 1))
        if start is not None:
            t = max(t, float(start))
        first = int(round(t))
        for i, value in enumerate(data):
            bits = byte_bits(value, bad_parity=i in bad,
                             stop=(0, 1) if i in low else (1, 1))
            for k, bit in enumerate(bits):
                self._changes.append((int(round(t + k * self.bit_clocks)), 1 - bit))
            t += BITS_PER_BYTE * self.bit_clocks
            self._changes.append((int(round(t)), 0))
            t += self.gap_bits * self.bit_clocks
        end = t - self.gap_bits * self.bit_clocks
        self._free_at = end
        self.frames_sent.append((first, int(round(end))))
        return first

    def idle_until(self, cycle: float) -> None:
        """No byte starts before ``cycle``: the gap before the next frame."""
        self._free_at = max(self._free_at, float(cycle))

    @property
    def busy(self) -> bool:
        return bool(self._changes)

    def drive(self, drive: Drive, cycle: int) -> None:
        self.now = cycle
        changes = self._changes
        while changes and changes[0][0] <= cycle:
            self.level = changes.popleft()[1]
        drive.set(self.pad, self.level)


# ------------------------------------------------------------ line decoder
@dataclasses.dataclass
class SbusByte:
    value: int
    start: int                      # cycle of the start bit's rising edge
    parity_ok: bool
    stop_ok: bool                   # both stop bits low on the line
    edges: List[Tuple[int, int]]    # (cycle, level) of every edge in the byte


@dataclasses.dataclass
class SbusFrame:
    start: int                      # the first byte's start edge
    bytes: List[SbusByte]

    @property
    def data(self) -> List[int]:
        return [b.value for b in self.bytes]

    @property
    def ok(self) -> bool:
        """25 bytes, each with good parity and stop bits, 0x0F ... 0x00."""
        return (len(self.bytes) == FRAME_BYTES and self.data[0] == BEGIN
                and self.data[-1] == END
                and all(b.parity_ok and b.stop_ok for b in self.bytes))


class SbusRx(Model):
    """Decodes inverted 8E2 S.BUS from a chip output by sampling each bit at
    its middle, and groups the bytes into frames by the gap between them.

    A byte starts on a rising edge (the line idles low) and its start bit
    must still be high at its middle. ``gap_bits`` is the idle time after a
    byte's last stop bit beyond which the next byte begins a new frame
    (48: Betaflight's 5-byte inter-byte timeout, less the byte itself).
    :meth:`edge_offsets` measures every edge of a byte against the nominal
    bit grid from its start edge.
    """

    def __init__(self, pin: PinSpec, bit_clocks: float, *, gap_bits: float = 48) -> None:
        self.pad = pad_of(pin)
        self.bit_clocks = float(bit_clocks)
        self.gap_clocks = gap_bits * self.bit_clocks
        self.prev = 1                       # armed once the line is seen low
        self.bytes: List[SbusByte] = []
        self.frames: List[SbusFrame] = []
        self._start: Optional[int] = None
        self._samples: List[int] = []
        self._next = 0
        self._edges: List[Tuple[int, int]] = []
        self._last_end: Optional[float] = None

    def _sample_at(self, k: int) -> int:
        return self._start + int(round((k + 0.5) * self.bit_clocks))

    def observe(self, lines: Lines) -> None:
        v = lines.get(self.pad)
        cycle = lines.cycle
        if v != self.prev and self._start is not None:
            self._edges.append((cycle, v))
        if self._start is None:
            if self.prev == 0 and v == 1:
                self._start = cycle
                self._samples = []
                self._edges = [(cycle, 1)]
                self._next = self._sample_at(0)
        elif cycle >= self._next:
            self._samples.append(1 - v)     # logical bit: the line inverted
            k = len(self._samples)
            if k == 1 and self._samples[0] != 0:
                self._start = None          # a glitch, not a start bit
            elif k == BITS_PER_BYTE:
                self._finish()
            else:
                self._next = self._sample_at(k)
        self.prev = v

    def _finish(self) -> None:
        s = self._samples
        value = sum(bit << i for i, bit in enumerate(s[1:9]))
        b = SbusByte(value, self._start, parity(value) == s[9],
                     s[10] == 1 and s[11] == 1, list(self._edges))
        self.bytes.append(b)
        if self._last_end is None or b.start - self._last_end > self.gap_clocks:
            self.frames.append(SbusFrame(b.start, []))
        self.frames[-1].bytes.append(b)
        self._last_end = b.start + BITS_PER_BYTE * self.bit_clocks
        self._start = None

    def complete_frames(self) -> List[SbusFrame]:
        """Frames of 25 bytes or more (a frame still arriving is left out)."""
        return [f for f in self.frames if len(f.bytes) >= FRAME_BYTES]

    def edge_offsets(self, b: SbusByte) -> List[Tuple[int, float]]:
        """For each edge of ``b``: ``(bit index, clocks off the nominal
        grid)``, measured from its start edge."""
        out = []
        for cycle, _ in b.edges:
            pos = (cycle - b.start) / self.bit_clocks
            k = int(round(pos))
            out.append((k, (cycle - b.start) - k * self.bit_clocks))
        return out


# ------------------------------------------------------------------ servos
class ServoMeter(Model):
    """Records every high pulse on one chip output as ``(rise, fall)``
    cycles, and every edge as ``(cycle, level)``."""

    def __init__(self, pin: PinSpec) -> None:
        self.pad = pad_of(pin)
        self.prev: Optional[int] = None
        self.pulses: List[Tuple[int, int]] = []
        self.edges: List[Tuple[int, int]] = []
        self._rise: Optional[int] = None

    def observe(self, lines: Lines) -> None:
        v = lines.get(self.pad)
        if self.prev is not None and v != self.prev:
            self.edges.append((lines.cycle, v))
            if v:
                self._rise = lines.cycle
            elif self._rise is not None:
                self.pulses.append((self._rise, lines.cycle))
                self._rise = None
        self.prev = v


def servo_violations(pulses: Sequence[Tuple[int, int]], raw: int, us_clocks: float,
                     *, slack_clocks: float = 3) -> List[str]:
    """Each pulse against :func:`servo_us` of ``raw``: an output in whole
    microseconds rounds the width one way or the other, so a pulse must be
    within ``[floor(us), ceil(us)]`` microseconds, plus the slot grid's
    ``slack_clocks`` either side."""
    us = servo_us(raw)
    lo = math.floor(us) * us_clocks - slack_clocks
    hi = math.ceil(us) * us_clocks + slack_clocks
    return ["pulse at %d: %d clocks, %.1f us expected (%.0f .. %.0f)"
            % (rise, fall - rise, us, lo, hi)
            for rise, fall in pulses if not lo <= fall - rise <= hi]
