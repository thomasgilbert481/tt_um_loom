"""XY2-100 galvanometer scanner interface: frame codec, receiver, sender.

Written from public descriptions of the interface, never from the firmware.

There is no formal standard: XY2-100 is a de-facto interface of the galvo
scanner makers (SCANLAB, Raylase and others), whose interface manuals are
not public. The sources below are **secondary** and every figure taken from
them is marked with where it came from.

* [L] Lutz Lisseck, ``XY2_100`` library for Teensy 3.x, ``XY2_100.cpp``
  (MIT licence, 2018, github.com/Tuet/XY2_100), read 2026-11-07:
  ``setXY()`` builds each channel as a 20-bit word sent MSB first:
  bits 19..17 the control bits ``001`` (``| 0x20000``), bits 16..1 the
  16-bit position (``X << 1``), bit 0 a parity bit set when the count of
  ones in the other 19 bits is odd, i.e. **even parity over all 20 bits**.
  Its output tables (``Sync1``/``Sync0``, comment "Clock cycle: 1-0,
  Sync 111...0") put CLOCK high in the first half of every bit and low in
  the second, change SYNC and the data lines together with the rising
  CLOCK edge, and hold SYNC high for the first 19 bits and low for the
  last (the parity bit). The data are therefore stable around the falling
  edge, where a receiver samples them. Its bit rate is 2 MHz (a 4 MHz half
  bit DMA), 10 us a frame.
* [T] techoverflow.net, "What is the typical & maximum clock frequency for
  XY2-100?" (2021-11-01): typically 2 MHz (20 bits at a 100 kHz frame
  rate); Raylase lists 10 MHz as a maximum. Unchecked against Raylase.
* [O] Omron CK3M series manual, XY2-100 section (the page could not be
  fetched, 2026-11-07; quoted by the search result only): a continuous
  2 MHz clock and a frame every 20 clocks, 10 us. Unchecked.

No source gives set-up, hold or jitter limits in nanoseconds. The checker
here takes the protocol's own shape: data change on the rising edge and
are sampled on the falling one, so a change must not come within
``setup_min`` clocks before a falling edge or ``hold_min`` clocks after it.
Both are parameters; their defaults (2 clocks, 40 ns at 50 MHz) are a
choice of this model, **not** a figure from any source.

The Z axis (a third data line on some heads) and the 18- and 20-bit
extended modes [O] are not modelled.
"""

from __future__ import annotations

import dataclasses
from typing import List, Optional, Sequence, Tuple

from tools.protomodels.bench import Drive, Lines, Model, PinSpec, pad_of

FRAME_BITS = 20                 # [L] 3 control + 16 data + 1 parity
CTRL_DEFAULT = 0b001            # [L] the control bits of a position frame
BIT_RATE_HZ = 2_000_000         # [L], [T], [O]
FRAME_RATE_HZ = BIT_RATE_HZ // FRAME_BITS   # 100 kHz, [T]
SETUP_MIN = 2                   # clocks; this model's choice, see above
HOLD_MIN = 2                    # clocks; this model's choice, see above


def parity_bit(word19: int) -> int:
    """Even parity [L]: the bit that makes the 20-bit frame's ones even."""
    return bin(word19 & 0x7FFFF).count("1") & 1


def frame_word(pos: int, ctrl: int = CTRL_DEFAULT, *, bad_parity=False) -> int:
    """The 20-bit frame as an integer, bit 19 sent first [L]."""
    w = ((ctrl & 7) << 17) | ((pos & 0xFFFF) << 1)
    return w | (parity_bit(w >> 1) ^ (1 if bad_parity else 0))


def frame_bits(pos: int, ctrl: int = CTRL_DEFAULT, *, bad_parity=False) -> List[int]:
    """The 20 bits in the order sent: c2 c1 c0, D15..D0, parity."""
    w = frame_word(pos, ctrl, bad_parity=bad_parity)
    return [(w >> (FRAME_BITS - 1 - i)) & 1 for i in range(FRAME_BITS)]


def sync_bits() -> List[int]:
    """SYNC per bit [L]: high for bits 0..18, low for the parity bit."""
    return [1] * (FRAME_BITS - 1) + [0]


@dataclasses.dataclass
class Channel:
    """One decoded 20-bit channel word."""

    ctrl: int
    pos: int
    parity: int

    @property
    def parity_ok(self) -> bool:
        w = (self.ctrl << 18) | (self.pos << 2) | (self.parity << 1)
        return bin(w).count("1") % 2 == 0


def parse_bits(bits: Sequence[int]) -> Channel:
    """Decode 20 bits in the order sent."""
    assert len(bits) == FRAME_BITS, len(bits)
    w = 0
    for b in bits:
        w = (w << 1) | (b & 1)
    return Channel(ctrl=w >> 17, pos=(w >> 1) & 0xFFFF, parity=w & 1)


@dataclasses.dataclass
class Frame:
    """One frame: the channels and the cycle of its first falling edge."""

    start: int
    x: Channel
    y: Channel


@dataclasses.dataclass
class Violation:
    """A measurement outside the checker's windows."""

    kind: str
    cycle: int
    clocks: float
    detail: str = ""


class Xy2Rx(Model):
    """A galvo driver's XY2-100 input on ``clk``, ``sync``, ``x``, ``y``.

    Samples SYNC, X and Y on every falling CLOCK edge [L], from their
    values in the cycle before the edge, and frames them: 19 bits with
    SYNC high then one with SYNC low make a frame; anything else is a
    "framing" violation and the bits are dropped. Every change of SYNC, X
    or Y is measured against the falling edges around it (``setup_min``
    before, ``hold_min`` after), and against the last rising edge
    (:attr:`delays`, for the report). :attr:`rises` and :attr:`falls` keep
    the cycle of every edge for the clock checks of the tests.
    """

    def __init__(self, clk: PinSpec, sync: PinSpec, x: PinSpec, y: PinSpec,
                 *, setup_min: int = SETUP_MIN, hold_min: int = HOLD_MIN,
                 strict: bool = False) -> None:
        self.pads = [pad_of(p) for p in (clk, sync, x, y)]
        self.setup_min, self.hold_min, self.strict = setup_min, hold_min, strict
        self.frames: List[Frame] = []
        self.violations: List[Violation] = []
        self.rises: List[int] = []
        self.falls: List[int] = []
        self.delays: List[int] = []        # clocks from the last rise, per change
        self.setups: List[int] = []        # clocks from the last change to a fall
        self.holds: List[int] = []         # clocks from a fall to the next change
        self._prev: Optional[Tuple[int, int, int, int]] = None
        self._change: Optional[int] = None
        self._held = True                  # the hold after the last fall is measured
        self._bits: List[Tuple[int, int, int]] = []
        self._start = 0

    def _bad(self, kind, cycle, clocks, detail=""):
        v = Violation(kind, cycle, clocks, detail)
        self.violations.append(v)
        if self.strict:
            raise AssertionError(str(v))

    def observe(self, lines: Lines) -> None:
        c = lines.cycle
        now = tuple(lines.get(p) for p in self.pads)
        prev, self._prev = self._prev, now
        if prev is None:
            return
        if now[1:] != prev[1:]:
            if self.rises:
                self.delays.append(c - self.rises[-1])
            if self.falls and not self._held:
                hold = c - self.falls[-1]
                self.holds.append(hold)
                self._held = True
                if hold < self.hold_min:
                    self._bad("hold", c, hold, "data changed %d clocks after "
                              "the falling edge" % hold)
            self._change = c
        if prev[0] == 0 and now[0] == 1:
            self.rises.append(c)
        elif prev[0] == 1 and now[0] == 0:
            self._fall(c, prev)

    def _fall(self, c, prev):
        self.falls.append(c)
        self._held = False
        if self._change is not None:
            setup = c - self._change
            self.setups.append(setup)
            if setup < self.setup_min:
                self._bad("setup", c, setup, "data changed %d clocks before "
                          "the falling edge" % setup)
        _, s, x, y = prev
        if not self._bits:
            self._start = c
        self._bits.append((s, x, y))
        n = len(self._bits)
        if s == 1 and n < FRAME_BITS:
            return
        if s == 0 and n == FRAME_BITS:
            xs = [b[1] for b in self._bits]
            ys = [b[2] for b in self._bits]
            self.frames.append(Frame(self._start, parse_bits(xs), parse_bits(ys)))
        elif self.frames or n > 1:
            # A SYNC low before bit 20, or 20 bits with SYNC still high. The
            # bits before the first SYNC low after power-up are not a frame
            # and are not an error (the receiver has not locked yet).
            self._bad("framing", c, n, "SYNC low after %d bits" % n
                      if s == 0 else "SYNC high for %d bits" % n)
        self._bits = []

    def report(self) -> str:                                # pragma: no cover
        return "%d frames, violations %s" % (len(self.frames), self.violations[:5])


class Xy2Tx(Model):
    """An XY2-100 controller driving ``clk``, ``sync``, ``x``, ``y``.

    Sends ``frames`` (a list of ``(x, y)`` positions, or of ``(x_bits,
    y_bits)`` 20-bit lists for an injected error), back to back with a
    continuous clock, then repeats the last one. A bit is ``high`` clocks
    with CLOCK high then ``low`` clocks low; SYNC and the data change in
    the cycle CLOCK rises [L]. ``start`` is the cycle of the first rise.
    """

    def __init__(self, clk: PinSpec, sync: PinSpec, x: PinSpec, y: PinSpec,
                 frames, *, high: int = 12, low: int = 13, start: int = 0) -> None:
        self.pads = [pad_of(p) for p in (clk, sync, x, y)]
        self.high, self.low, self.start = high, low, start
        self.seq: List[Tuple[List[int], List[int]]] = []
        for f in frames:
            a, b = f
            self.seq.append((list(a) if not isinstance(a, int) else frame_bits(a),
                             list(b) if not isinstance(b, int) else frame_bits(b)))
        self.sent = 0                     # frames whose last bit has started

    def drive(self, drive: Drive, cycle: int) -> None:
        clk = sync = x = y = 0
        t = cycle - self.start
        if t >= 0:
            per = self.high + self.low
            bit, phase = divmod(t, per)
            n, k = divmod(bit, FRAME_BITS)
            xs, ys = self.seq[min(n, len(self.seq) - 1)]
            clk = 1 if phase < self.high else 0
            sync, x, y = sync_bits()[k], xs[k], ys[k]
            if k == FRAME_BITS - 1:
                self.sent = max(self.sent, n + 1)
        for pad, v in zip(self.pads, (clk, sync, x, y)):
            drive.set(pad, v)


class Wire(Model):
    """A board wire: ``dst`` follows ``src`` one clock later (a registered
    model), joining the controller's OUT pins to the receiver's IN pins."""

    def __init__(self, src: PinSpec, dst: PinSpec) -> None:
        self.src, self.dst = pad_of(src), pad_of(dst)
        self.level = 0

    def drive(self, drive: Drive, cycle: int) -> None:
        drive.set(self.dst, self.level)

    def observe(self, lines: Lines) -> None:
        self.level = lines.get(self.src)
