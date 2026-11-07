"""Sony LANC (Control-L): camera and controller models, the wire, a decoder
and a timing checker, written from the protocol's description, not from
``firmware/lanc_ctrl.loom`` or ``firmware/lanc_cam.loom``.

Sources. Sony has published no LANC standard; every figure here is
secondary and is marked with where it comes from.

* [B] U. Boehmel, "LANC(tm)", http://www.boehmel.de/lanc.htm (page dated
  2020-11-08), section "How SONY's LANC(tm) protocol works": "One bit has a
  duration of 104us" and "The timing conforms to RS232 at 9600 Baud"; the
  master (the camera) "generates the telegram frame, creating 8 startbits,
  followed each by 8 bits (1 byte) and a (long) stopbit"; "The distance
  between two startbits can vary between 1200us and 1400us"; "The distance
  between two telegrams is 20ms for PAL/625 and 16.6ms for NTSC/525"; "A
  command is valid after 3...4 telegrams"; open collector. Command tables:
  byte 0 = 0x18 "normal command to VTR or video camera", byte 1 = 0x34
  play, 0x30 stop, 0x33 start/stop, 0x3A rec; bytes 4 to 7 are camera
  status (byte 5 bit 2 "battery low").
* [W] Wikipedia, "LANC": 8 bytes a frame, "usually clocked by the camera
  at 9600 bit/s", one frame per video frame (field).
* [M] From memory, unchecked (no readable source states it; the timing
  diagram of [B] is an image that could not be read): the line idles high;
  a start bit is low; a data bit read as 1 is a low line (the "inverted"
  of the brief); LSB first; bytes 0 and 1 are the controller's, written in
  the slots the camera's start bits open, bytes 2 to 7 the camera's.
* [O] This model's own windows, which no source gives: a receiver samples
  at the middle of a bit, so an edge may sit up to half a bit off the
  start bit's grid before a bit is misread; the checker allows a quarter
  (``EDGE_TOL``). The frame gap that tells byte 0 from the others is any
  idle longer than ``GAP_US``: [B]'s 1400 us start to start leaves at most
  1400 - 9 x 104 = 463 us between bytes, and 8 slots of 1400 us in a
  16.6 ms field leave at least 5.9 ms before byte 0.

NTSC: [B] says 16.6 ms; the field rate of 59.94 Hz is 16.683 ms, which is
what ``FIELD_US["NTSC"]`` holds.

Everything is in clocks of the bench; ``us`` is clocks per microsecond and
a bit is ``BIT_US * us`` clocks. The tests run the real rate (50 clocks a
us) and a scaled one (``scale_for``), as ``cec.py`` does.
"""

from __future__ import annotations

import bisect
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from tools.protomodels.bench import Drive, Lines, Model, PinSpec, pad_of

BAUD = 9600                              # [B]
BIT_US = 1e6 / BAUD                      # 104.17 us; [B] "104us"
BYTES = 8                                # [B], [W]
START_TO_START_US = (1200.0, 1400.0)     # [B]
FIELD_US = {"PAL": 20000.0, "NTSC": 1e6 / 59.94}    # [B] 20 ms, 16.6 ms
VALID_AFTER = (3, 4)                     # [B] telegrams
CONTROLLER_BYTES = (0, 1)                # [M]
STATUS_BYTES = (4, 5, 6, 7)              # [B]
EDGE_TOL = 0.25                          # [O] bits
GAP_US = 2000.0                          # [O] longer idle: byte 0 is next
STOP_BITS = 9                            # start + 8 data bits, then the stop

NORMAL = 0x18                            # [B] byte 0
PLAY, STOP, START_STOP, REC = 0x34, 0x30, 0x33, 0x3A   # [B] byte 1


def scale_for(tick_clocks: float) -> float:
    """Clocks per microsecond when one tick of ``tick_clocks`` is an
    eighth of a bit (the programs' tick)."""
    return 8 * tick_clocks / BIT_US


def tick_for(us: float) -> Tuple[int, int]:
    """``(TICK_INT, TICK_FRAC)`` for a tick of one eighth of a bit."""
    clocks = BIT_US * us / 8
    whole = int(clocks)
    return whole, int(round((clocks - whole) * 256))


def byte_bits(value: int) -> List[int]:
    """The 8 data bits in line order, LSB first [M]."""
    return [(value >> j) & 1 for j in range(8)]


def level_of(bit: int) -> int:
    """A data bit's line level: a 1 is a low line [M]."""
    return 0 if bit else 1


class Camera:
    """A model camcorder: the LANC master [B]. Frame ``f`` starts at
    ``start + f * field``; byte ``k`` of it at ``k * slot`` later, with a
    start bit of one bit (low). Bytes 0 and 1 are left released and read
    at the middle of each bit; bytes 2 to 7 are :attr:`status`, sent LSB
    first, then the stop (high) [M].

    ``rate`` above 1 makes every time shorter by that factor (a camera
    whose clock is fast). :attr:`received` holds ``(b0, b1)`` per frame.
    Clocked by :class:`LancWire` with the line level of every cycle."""

    def __init__(self, us: float, field: str = "PAL", status: Optional[Dict[int, int]] = None,
                 slot_us: float = 12.5 * BIT_US, start: int = 0, rate: float = 1.0,
                 field_us: Optional[float] = None, frames: Optional[int] = None) -> None:
        self.bit = BIT_US * us / rate
        self.slot = slot_us * us / rate
        self.field = (field_us or FIELD_US[field]) * us / rate
        self.start = start
        self.frames = frames                 # stop after this many, if given
        self.status = {k: 0 for k in range(2, BYTES)}
        self.status.update(status or {})
        self.received: List[Tuple[int, int]] = []
        self.frame_starts: List[float] = []
        self.level = 1
        self._got = [0, 0]

    def _where(self, cycle: float):
        """``(frame, byte, offset in clocks)`` of ``cycle``, or None."""
        t = cycle - self.start
        if t < 0:
            return None
        f = int(t // self.field)
        if self.frames is not None and f >= self.frames:
            return None
        o = t - f * self.field
        k = int(o // self.slot)
        return (f, k, o - k * self.slot) if k < BYTES else None

    def _drive(self, cycle: int) -> int:
        w = self._where(cycle)
        if w is None:
            return 1
        f, k, o = w
        if o < self.bit:
            return 0                          # start bit
        j = int(o // self.bit) - 1
        if k in CONTROLLER_BYTES or j >= 8:
            return 1                          # released / the stop
        return level_of((self.status[k] >> j) & 1)

    def clock(self, level: int, cycle: int) -> None:
        w = self._where(cycle)
        if w is not None:
            f, k, o = w
            if k == 0 and o < 1:
                self.frame_starts.append(self.start + f * self.field)
            if k in CONTROLLER_BYTES:
                for j in range(8):
                    s = (1.5 + j) * self.bit
                    if o - 1 < s <= o:
                        if j == 0:
                            self._got[k] = 0
                        self._got[k] |= (1 - level) << j
                        if k == 1 and j == 7:
                            self.received.append((self._got[0], self._got[1]))
        self.level = self._drive(cycle + 1)


class Controller:
    """A model LANC controller [B], [M]: it finds byte 0 after an idle of
    more than ``GAP_US``, takes each later start bit as the next byte, and
    while a command is pending writes it into bytes 0 and 1, bit ``j`` from
    ``(1 + j)`` bits after the start bit's fall (plus ``delay`` clocks) for
    one bit, each for :attr:`repeat` frames. It reads bytes 4 to 7 at the
    middle of each bit, into :attr:`statuses` (one tuple per frame)."""

    def __init__(self, us: float, repeat: int = 5, delay: float = 0.0) -> None:
        self.us = us
        self.bit = BIT_US * us
        self.repeat = repeat
        self.delay = delay
        self.queue: List[Tuple[int, int]] = []
        self.statuses: List[Tuple[int, int, int, int]] = []
        self.written: List[Tuple[int, int]] = []    # (b0, b1) of each frame written
        self.level = 1
        self._prev = 1
        self._rise = 0                     # the line idles high from the start
        self._s: Optional[int] = None      # the current byte's start-bit fall
        self._k: Optional[int] = None      # its place in the frame
        self._cmd: Optional[Tuple[int, int]] = None
        self._left = 0
        self._st = [0, 0, 0, 0]

    def send(self, b0: int, b1: int) -> None:
        self.queue.append((b0, b1))

    def clock(self, level: int, cycle: int) -> None:
        if self._prev == 0 and level == 1:
            self._rise = cycle
        if (self._prev == 1 and level == 0
                and (self._s is None or cycle >= self._s + 9.5 * self.bit)):
            if self._k == 1 and self._left:          # byte 1 has been written
                self.written.append(self._cmd)
                self._left -= 1
            if cycle - self._rise >= GAP_US * self.us:
                self._k = 0
                if self._left == 0 and self.queue:
                    self._cmd, self._left = self.queue.pop(0), self.repeat
            elif self._k is not None:
                self._k = self._k + 1 if self._k < BYTES - 1 else None
            self._s = cycle
        self._prev = level
        self._sample(level, cycle)
        self.level = self._drive(cycle + 1)

    def _sample(self, level: int, cycle: int) -> None:
        if self._s is None or self._k not in STATUS_BYTES:
            return
        o = cycle - self._s
        for j in range(8):
            s = (1.5 + j) * self.bit
            if o - 1 < s <= o:
                i = self._k - STATUS_BYTES[0]
                if j == 0:
                    self._st[i] = 0
                self._st[i] |= (1 - level) << j
                if self._k == BYTES - 1 and j == 7:
                    self.statuses.append(tuple(self._st))

    def _drive(self, cycle: int) -> int:
        if self._s is None or self._k not in CONTROLLER_BYTES or self._left == 0:
            return 1
        o = cycle - self._s - self.delay
        j = int(o // self.bit) - 1 if o >= 0 else -1
        if 0 <= j < 8:
            return level_of((self._cmd[self._k] >> j) & 1)
        return 1


class LancWire(Model):
    """The LANC wire on the bench: a wired AND of open-drain drivers.

    Each chip pin in ``pins`` is a BIDIR pin in open-drain mode with its
    own pad (two Loom threads never share a pin; PORTING 4); the board joins
    them. The line is low in cycle ``c`` if a chip pin pulled low in cycle
    ``c - 1`` (the board wire's one clock) or a model node is low; while it
    is low the wire pulls every chip pad low, and the bench's pull-ups on
    the pads (``pullups``) make the high. :attr:`edges` holds every change
    of the line as ``(cycle, level)``; :attr:`pin_lows` each chip pin's own
    lows as ``(fall, length)`` in clocks, measured on the pin."""

    def __init__(self, nodes: Iterable[object] = (),
                 pins: Sequence[PinSpec] = ("BIDIR0",)) -> None:
        self.nodes = list(nodes)
        self.pads = [pad_of(p) for p in pins]
        self.level = 1
        self.edges: List[Tuple[int, int]] = []
        self.pin_lows: Dict[Tuple[str, int], List[Tuple[int, int]]] = {p: [] for p in self.pads}
        self._chip = 0
        self._fall = {p: None for p in self.pads}

    @property
    def pullups(self) -> int:
        return sum(1 << bit for _, bit in self.pads)

    def add(self, node):
        self.nodes.append(node)
        return node

    def drive(self, drive: Drive, cycle: int) -> None:
        level = 0 if self._chip else 1
        for node in self.nodes:
            level &= node.level
        if level != self.level:
            self.edges.append((cycle, level))
        self.level = level
        if not level:
            for pad in self.pads:
                drive.pull_low(pad)

    def observe(self, lines: Lines) -> None:
        chip = 0
        for pad in self.pads:
            bit = 1 << pad[1]
            low = bool(lines.uio_oe & bit) and not lines.uio_out & bit
            chip |= low
            if low and self._fall[pad] is None:
                self._fall[pad] = lines.cycle
            elif not low and self._fall[pad] is not None:
                self.pin_lows[pad].append((self._fall[pad], lines.cycle - self._fall[pad]))
                self._fall[pad] = None
        self._chip = chip
        for node in self.nodes:
            node.clock(self.level, lines.cycle)

    def lows_of(self, pin: PinSpec) -> List[Tuple[int, int]]:
        return self.pin_lows[pad_of(pin)]


class Frame:
    """One frame decoded from the line: the start-bit falls of its bytes
    and the bytes read at the middle of each bit."""

    def __init__(self) -> None:
        self.starts: List[int] = []
        self.data: List[int] = []

    def complete(self) -> bool:
        return len(self.starts) == BYTES

    def command(self) -> Tuple[int, int]:
        return self.data[0], self.data[1]

    def status(self) -> Tuple[int, ...]:
        return tuple(self.data[k] for k in STATUS_BYTES)


def level_at(edges: Sequence[Tuple[int, int]], cycle: float) -> int:
    """The line level at ``cycle`` from :attr:`LancWire.edges` (high before
    the first edge)."""
    i = bisect.bisect_right([c for c, _ in edges], cycle) - 1
    return 1 if i < 0 else edges[i][1]


def decode(edges: Sequence[Tuple[int, int]], us: float, rate: float = 1.0,
           end: Optional[int] = None) -> List[Frame]:
    """Frames on the line. A fall is a start bit when it comes at least 9.5
    bits after the last one (data lows end within 9 bits); a start more
    than ``GAP_US`` after the last begins a frame. Bits are read at
    ``(1.5 + j)`` bits of a camera ``rate`` times fast; a byte whose middle
    of bit 7 is past ``end`` is left out."""
    bit = BIT_US * us / rate
    cycles = [c for c, _ in edges]
    starts: List[int] = []
    for c, level in edges:
        if level == 0 and (not starts or c - starts[-1] >= 9.5 * bit):
            starts.append(c)
    frames: List[Frame] = []
    for s in starts:
        if end is not None and s + 8.5 * bit > end:
            break
        if not frames or s - frames[-1].starts[-1] > GAP_US * us:
            frames.append(Frame())
        value = 0
        for j in range(8):
            i = bisect.bisect_right(cycles, s + (1.5 + j) * bit) - 1
            value |= (1 - (1 if i < 0 else edges[i][1])) << j
        frames[-1].starts.append(s)
        frames[-1].data.append(value)
    return frames


def _near(x: float, lo: int, hi: int, tol: float) -> bool:
    n = round(x)
    return lo <= n <= hi and abs(x - n) <= tol


def check_frames(frames: Sequence[Frame], us: float, rate: float = 1.0) -> List[str]:
    """[B]'s start-to-start window (1200 to 1400 us, a camera ``rate``
    times fast scaled by it) for every pair of bytes in each frame, and 8
    bytes in every frame but the last. Returns the faults found."""
    lo, hi = (t * us / rate for t in START_TO_START_US)
    faults = []
    for n, f in enumerate(frames):
        if n < len(frames) - 1 and not f.complete():
            faults.append("frame %d has %d bytes" % (n, len(f.starts)))
        for a, b in zip(f.starts, f.starts[1:]):
            if not lo <= b - a <= hi:
                faults.append("frame %d: start to start %.1f us" % (n, (b - a) / us))
    return faults


def check_pin(lows: Sequence[Tuple[int, int]], frames: Sequence[Frame], us: float,
              allowed: Sequence[int], first: int = 1, tol: float = EDGE_TOL) -> List[str]:
    """Every low of one pin against the grid of the byte slot it is in
    [O]: both edges within ``tol`` bits of ``S + n`` bits, ``S`` the byte's
    start-bit fall on the line and ``n`` a whole number from ``first`` to 9
    (the stop); the byte must be one of ``allowed``. A controller's pin
    has ``first = 1`` (its bits follow the camera's start bit), a camera's
    ``first = 0``. The grid is the nominal 104.17 us bit: a controller
    times its bits with its own clock from the camera's edge."""
    bit = BIT_US * us
    slots = [(s, k) for f in frames for k, s in enumerate(f.starts)]
    begins = [s for s, _ in slots]
    faults = []
    for fall, length in lows:
        i = bisect.bisect_right(begins, fall + tol * bit) - 1
        if i < 0:
            faults.append("low at %d before any start bit" % fall)
            continue
        s, k = slots[i]
        a, b = (fall - s) / bit, (fall + length - s) / bit
        if k not in allowed:
            faults.append("low at %d in byte %d" % (fall, k))
        elif not (_near(a, first, 8, tol) and _near(b, first + 1, STOP_BITS, tol)):
            faults.append("byte %d: low from %.3f to %.3f bits" % (k, a, b))
    return faults
