"""EM4100 read-only RFID reference models: the frame, a tag and a reader.

Written from the EM4100 data sheet (EM Microelectronic-Marin SA, "EM4100
Read Only Contactless Identification Device", copyright 2004, nine pages;
read from the copy at static.chipdip.ru/lib/500/DOC032500449.pdf on
2026-11-07), not from the firmware:

* **Frame** (p. 5, "Memory Array for Manchester & Bi-Phase encoding ICs"):
  64 bits, "9 bits are used for the header, 10 row parity bits (P0-P9), 4
  column parity bits (PC0-PC3), 40 data bits (D00-D93), and 1 stop bit set
  to logic 0". The header bits are all 1; each of the 10 rows is four data
  bits and an even row parity; the last group is four even column parity
  bits and S0. The table is sent row by row, D00 first (the table's order;
  the text says only "outputted serially"). Bits D00..D13 are the
  "version bits or customer ID".
* **Repetition** (p. 5): "the output sequence is repeated continuously
  until power goes off"; p. 4, General: "When the last bit is sent, the chip
  will continue with the first bit".
* **Code** (p. 5, "Code Description, Manchester", and Fig. 6): "always a
  transition ... in the middle of bit period"; on Fig. 6's "Modulator
  control" trace a 1 is low then high and a 0 high then low, and
  "Modulation control 'low' means high current", i.e. the load switch on.
* **Rate** (p. 1, Description; p. 4, Sequencer; p. 8, Ordering
  Information): 64 or 32 periods of the field per bit for Manchester
  (versions A6 and A5); 16 for PSK, not modelled. The clock is the field
  itself, taken from COIL1 by the clock extractor (p. 4).
* **Field** (p. 1, Features): "Operating frequency 100 - 150 kHz", and
  Fig. 1 gives the coil for f0 = 125 kHz.

Not in the data sheet, so chosen here and marked as such: which field edge
the sequencer counts (here: a half-bit is N rising edges of RF), where the
first bit starts after power-up, and any lag from field edge to modulator
edge. Biphase and PSK are not modelled.

Logic-level pins, as in ``firmware/em4100_tag.loom``: ``RF`` is the field
as a square wave; ``MOD`` is Fig. 6's "Modulator control" (1 = load off);
a reader's ``DEMOD`` is the envelope, high while unloaded, so a bench wire
from MOD to DEMOD is the whole air interface.

Bench models (``tools.protomodels.bench.Model``): :class:`ModelReader`
drives a field on a chip input and records the chip's MOD; :class:`TagModel`
counts the chip's field and drives DEMOD; :class:`Wire` joins a chip output
to a chip input. :func:`decode` turns MOD edges and field edges back into
half-bits, bits and frames, counting time in field cycles, so a test can
hold a tag to the field and not to any clock of its own.
"""

from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

from tools.protomodels.bench import Drive, Lines, Model, PinSpec, pad_of

CLOCK = 50_000_000          # the chip's clock in the tests
FIELD = 125_000             # the nominal field (p. 1, Fig. 1)
FRAME_BITS = 64
HEADER_BITS = 9
ROWS = 10
#: field periods per bit -> field periods per half-bit
HALF = {64: 32, 32: 16}


# ------------------------------------------------------------------ frame
def nibbles(ident: int) -> List[int]:
    """The ten data nibbles of a 40-bit ID, D00..D03 first (D00 = bit 39)."""
    if not 0 <= ident < 1 << 40:
        raise ValueError("an EM4100 ID is 40 bits")
    return [(ident >> (36 - 4 * i)) & 0xF for i in range(ROWS)]


def _bits4(n: int) -> List[int]:
    return [(n >> (3 - i)) & 1 for i in range(4)]


def frame_bits(ident: int, flip: Optional[int] = None) -> List[int]:
    """The 64 bits of the frame for ``ident``, in transmission order.

    ``flip`` is the index (0..63) of one bit to invert, to make a frame
    with a parity or stop-bit error."""
    bits = [1] * HEADER_BITS
    column = 0
    for n in nibbles(ident):
        row = _bits4(n)
        bits += row + [sum(row) & 1]            # even row parity
        column ^= n
    bits += _bits4(column) + [0]                # even column parity, S0
    assert len(bits) == FRAME_BITS
    if flip is not None:
        bits[flip] ^= 1
    return bits


@dataclass
class FrameCheck:
    """What a reader finds in 64 bits that start with the header."""

    ident: Optional[int]            # the ID if every check passed
    header_ok: bool
    bad_rows: List[int]             # rows whose parity is odd, in order
    column_ok: bool                 # PC0..PC3 match the data
    stop_ok: bool                   # S0 == 0

    @property
    def ok(self) -> bool:
        return self.ident is not None


def check_frame(bits: Sequence[int]) -> FrameCheck:
    """Check 64 bits against the p. 5 structure; the ID only if all pass."""
    if len(bits) != FRAME_BITS:
        raise ValueError("a frame is 64 bits")
    header_ok = all(b == 1 for b in bits[:HEADER_BITS])
    bad_rows, column, ident = [], [0, 0, 0, 0], 0
    for r in range(ROWS):
        row = bits[HEADER_BITS + 5 * r:HEADER_BITS + 5 * r + 5]
        if sum(row) & 1:
            bad_rows.append(r)
        for c in range(4):
            column[c] ^= row[c]
            ident = (ident << 1) | row[c]
    tail = bits[HEADER_BITS + 5 * ROWS:]
    column_ok = list(tail[:4]) == column
    stop_ok = tail[4] == 0
    good = header_ok and not bad_rows and column_ok and stop_ok
    return FrameCheck(ident if good else None, header_ok, bad_rows,
                      column_ok, stop_ok)


def halves(bits: Sequence[int]) -> List[int]:
    """Modulator-control levels, two per bit (Fig. 6): 1 -> 0, 1; 0 -> 1, 0."""
    out: List[int] = []
    for b in bits:
        out += [0, 1] if b else [1, 0]
    return out


# ----------------------------------------------------------------- decode
@dataclass
class Decoded:
    """MOD read back on the field's own time base (:func:`decode`)."""

    lags: List[int] = field(default_factory=list)       # clocks, field rise to MOD edge
    offsets: List[int] = field(default_factory=list)    # field cycles off the half-bit grid
    halves: List[int] = field(default_factory=list)     # one level per half-bit
    bits: List[int] = field(default_factory=list)
    violations: List[int] = field(default_factory=list)  # bits with no mid-bit transition
    frames: List[Tuple[int, FrameCheck]] = field(default_factory=list)

    def ids(self) -> List[Optional[int]]:
        return [check.ident for _, check in self.frames]


def decode(edges: Sequence[Tuple[int, int]], rises: Sequence[int], n: int) -> Decoded:
    """Read a tag's frames from its MOD ``edges`` (``(cycle, level)``, the
    first cycle the new level is on the pad) and the field's ``rises`` (the
    first cycle of each high half), at ``n`` field periods per half-bit.

    Leading edges up to the first fall are dropped: the line is 1 (no load)
    before the first frame, whose first half-bit is low (a header 1). Each
    edge is placed on the field: the last rise at or before it gives its lag
    in clocks and its field-cycle index, and ``offsets`` is that index's
    distance from the half-bit grid that the first edge starts. Half-bits
    are sampled at the field rise in their middle; the Manchester phase is
    the one with fewer violations; a frame is nine 1s after a 0 (or at the
    start) with 64 bits to read."""
    out = Decoded()
    edges = list(edges)
    while edges and edges[0][1] != 0:
        edges.pop(0)
    if not edges:
        return out
    index = []
    for cycle, _ in edges:
        k = bisect_right(rises, cycle) - 1
        if k < 0:
            raise ValueError("a MOD edge at cycle %d before any field rise" % cycle)
        index.append(k)
        out.lags.append(cycle - rises[k])
    base = index[0]
    out.offsets = [(k - base + n // 2) % n - n // 2 for k in index]
    cycles = [c for c, _ in edges]

    def level_at(cycle: int) -> int:
        i = bisect_right(cycles, cycle) - 1
        return 1 if i < 0 else edges[i][1]

    j = 0
    while base + j * n + n // 2 < len(rises):
        out.halves.append(level_at(rises[base + j * n + n // 2]))
        j += 1
    best = None
    for phase in (0, 1):
        h = out.halves[phase:]
        pairs = [(h[i], h[i + 1]) for i in range(0, len(h) - 1, 2)]
        viol = [i for i, (a, b) in enumerate(pairs) if a == b]
        if best is None or len(viol) < len(best[1]):
            best = ([b for _, b in pairs], viol)
    out.bits, out.violations = best
    bits = out.bits
    for i in range(len(bits) - FRAME_BITS + 1):
        if all(bits[i:i + HEADER_BITS]) and (i == 0 or bits[i - 1] == 0):
            out.frames.append((i, check_frame(bits[i:i + FRAME_BITS])))
    return out


# ------------------------------------------------------------ bench models
class Wire(Model):
    """A board wire from chip output ``src`` to chip input ``dst``, one clock
    late (what the input sees in cycle c is ``src`` in cycle c - 1)."""

    def __init__(self, src: PinSpec, dst: PinSpec, initial: int = 0) -> None:
        self.src, self.dst = pad_of(src), pad_of(dst)
        self.value = initial & 1

    def drive(self, drive: Drive, cycle: int) -> None:
        drive.set(self.dst, self.value)

    def observe(self, lines: Lines) -> None:
        self.value = lines.get(self.src)


class EdgeLog(Model):
    """Records every change of one pad as ``(cycle, new level)``."""

    def __init__(self, pin: PinSpec) -> None:
        self.pad = pad_of(pin)
        self.level: Optional[int] = None
        self.edges: List[Tuple[int, int]] = []

    def observe(self, lines: Lines) -> None:
        v = lines.get(self.pad)
        if self.level is not None and v != self.level:
            self.edges.append((lines.cycle, v))
        self.level = v

    def rises(self) -> List[int]:
        return [c for c, v in self.edges if v]

    def falls(self) -> List[int]:
        return [c for c, v in self.edges if not v]


class ModelReader(Model):
    """A reader: drives the field on chip input ``rf`` and reads the tag's
    MOD on chip output ``mod``.

    The field is a 50 % square wave of ``period`` clocks (any float: a
    period off nominal is how a test checks that the tag follows the field),
    low before ``start``. ``rises`` holds the first cycle of every high half
    and ``mod`` the MOD edges; :meth:`decoded` reads them back."""

    def __init__(self, rf: PinSpec, mod: PinSpec, period: float,
                 rate: int = 64, start: int = 0) -> None:
        self.rf_pad = pad_of(rf)
        self.mod = EdgeLog(mod)
        self.period = float(period)
        self.n = HALF[rate]
        self.start = start
        self.rises: List[int] = []
        self._level = 0

    def field_level(self, cycle: int) -> int:
        if cycle < self.start:
            return 0
        return 1 if ((cycle - self.start) / self.period) % 1.0 < 0.5 else 0

    def drive(self, drive: Drive, cycle: int) -> None:
        level = self.field_level(cycle)
        if level and not self._level:
            self.rises.append(cycle)
        self._level = level
        drive.set(self.rf_pad, level)

    def observe(self, lines: Lines) -> None:
        self.mod.observe(lines)

    def decoded(self) -> Decoded:
        return decode(self.mod.edges, self.rises, self.n)


class TagModel(Model):
    """An EM4100 tag: counts the rising edges of the field on chip output
    ``rf`` and drives its modulator control on chip input ``demod``.

    The line is 1 (no load) until the first ``n``-th rise; from then on
    every ``n`` rises (``n`` = 32 for RF/64, 16 for RF/32) it moves to the
    next half-bit of :func:`halves` of the frame, frame after frame. The
    frame is rebuilt from ``ident`` and ``flip`` (a bit to invert, or None)
    at the start of every repetition, so a test can change them while it
    runs. ``start_bit`` begins the first frame part way through, to save
    simulated time. ``violate`` is the index of a bit whose second half
    repeats its first (no mid-bit transition: a Manchester code violation),
    in every frame while it is set. Records the field's rises and falls (to
    check the reader's clock) and the cycle each frame starts at."""

    def __init__(self, rf: PinSpec, demod: PinSpec, ident: int, rate: int = 64,
                 flip: Optional[int] = None, start_bit: int = 0,
                 violate: Optional[int] = None) -> None:
        self.rf_pad, self.demod_pad = pad_of(rf), pad_of(demod)
        self.ident, self.flip, self.violate = ident, flip, violate
        self.n = HALF[rate]
        self.half = 2 * start_bit - 1           # -1: not started
        self.level = 1
        self.count = 0
        self.levels = halves(frame_bits(ident, flip))
        self.frame_starts: List[int] = []
        self.field = EdgeLog(rf)

    def drive(self, drive: Drive, cycle: int) -> None:
        drive.set(self.demod_pad, self.level)

    def observe(self, lines: Lines) -> None:
        before = self.field.level
        self.field.observe(lines)
        if before == 0 and self.field.level == 1:
            self.count += 1
            if self.count == self.n:
                self.count = 0
                self.half = (self.half + 1) % (2 * FRAME_BITS)
                if self.half == 0:
                    self.levels = halves(frame_bits(self.ident, self.flip))
                    self.frame_starts.append(lines.cycle)
                self.level = self.levels[self.half]
                if self.violate is not None and self.half == 2 * self.violate + 1:
                    self.level = self.levels[self.half - 1]
