"""IBM System/34 double-density MFM reference models, for
firmware/mfm_write.loom and mfm_read.loom.

Written from the sources below, never from the firmware (METH-1). Read
2026-11-24:

* [WP] English Wikipedia, "Modified frequency modulation": the rule
  "(x, y, z, ...) encodes to (x, x NOR y, y, y NOR z, z, ...)", so a 1 is
  written 01, a 0 after a 0 is 10 and a 0 after a 1 is 00; at least one and
  at most three 0 cells between two 1 cells (an RLL(1,3) code); floppies at
  250-500 kbit/s data, 500-1000 kbit/s encoded; the A1 sync mark has "the
  fifth clock bit" different from the normal encoding of A1, a deliberate
  violation of the rule.
* [FD] Western Digital FD179X-01 Floppy Disk Formatter/Controller data
  sheet, October 1979 (bitsavers):
  - "CRC Logic" (p. 5): G(x) = x^16 + x^12 + x^5 + 1, "the CRC register is
    preset to ones prior to data being shifted through the circuit", the
    CRC covers everything from the address mark up to the CRC characters.
  - Sector length table (p. 9): length code 00 = 128 bytes, 01 = 256,
    02 = 512, 03 = 1024.
  - Write Track control bytes (p. 13): in MFM, F5 writes "A1* ... preset
    CRC"; F7 writes the two CRC bytes. Figure "IBM track format" (p. 16):
    the A1* has its "missing clock transition between bits 4 and 5".
  - "IBM System 34 format, 256 bytes/sector" (p. 17): ... 4E gap, 12 x 00,
    3 x F5 (A1*), FE (ID address mark), track, side, sector, length, F7
    (2 CRCs), 22 x 4E, 12 x 00, 3 x F5, FB (data address mark), data, F7;
    "Non-IBM formats" table: MFM gap I 32 x 4E, gap II 22 x 4E (exact),
    12 x 00, 3 x A1 (exactly 3), gap III 24 x 4E (minimum), 8 x 00, 3 x A1.
  - Timing (p. 19), "all times double when CLK = 1 MHz" (5.25-inch double
    density, 250 kbit/s): write data cycle time Tbc 2, 3 or 4 us at 2 MHz,
    so 4, 6, 8 us here; write data pulse width Twp MFM 150/200/250 ns
    (min/typ/max) at 2 MHz, so 300/400/500 ns here; write gate to write
    data Twg 1 us MFM, 2 us here; input data timing, 5" MFM at 1 MHz:
    RCLK Ta = Tb = 2 us, one window a half-cell; note 1: RAW READ pulses
    normally 100-300 ns, doubled at 1 MHz.

Derived (not stated in so many words, worked from the above):

* The A1* cell pattern is 0x4489 (cells MSB first): A1 encodes normally as
  0x44A9, and dropping the clock cell between data bits 4 and 5 (bits
  counted from 0 at the MSB, the two 0s that would carry a clock 1) gives
  0100 0100 1000 1001 = 0x4489, with a flux gap of 4 cells that ordinary
  data cannot place there (QUESTIONS.md: the hex value itself is from
  memory, unchecked against a printed source; it follows from [FD]'s
  figure and [WP]'s rule).
* The CRC of a field covers A1 A1 A1, the mark and the field (preset to
  ones at the first F5, [FD] p. 13 and p. 5); over the field and its own
  two CRC bytes it leaves 0 (CRC without a final inversion).
* A flux transition is written at every 1 cell; intervals are 2, 3 or 4
  half-cells (4, 6, 8 us at 250 kbit/s).
* Read window: a transition is read in the half-cell window it falls in,
  each window one half-cell wide ([FD] p. 19), so a transition may move by
  up to half a half-cell (1 us) from its nominal place.
"""

from __future__ import annotations

import dataclasses
import random
from typing import Dict, List, Optional, Sequence, Tuple

from .bench import Drive, Lines, Model, PinSpec, pad_of

HALF_CELL_S = 2e-6                   # [FD] p. 19, 250 kbit/s: 4 us bit cell
WRITE_PULSE_S = (300e-9, 400e-9, 500e-9)   # [FD] p. 19 Twp MFM, doubled
WRITE_GATE_LEAD_S = 2e-6             # [FD] p. 19 Twg MFM, doubled
WINDOW = 0.5                         # half-cells each side ([FD] p. 19)
SYNC_A1 = 0x4489                     # derived (module docstring)
MARK_ID, MARK_DATA = 0xFE, 0xFB      # [FD] p. 17
GAP1, GAP2, GAP3, SYNC00 = 32, 22, 24, 12   # [FD] p. 17, non-IBM table
SIZES = {0: 128, 1: 256, 2: 512, 3: 1024}    # [FD] p. 9


def crc16(data: Sequence[int], crc: int = 0xFFFF) -> int:
    """CRC with G(x) = x^16 + x^12 + x^5 + 1, MSB first ([FD] p. 5)."""
    for byte in data:
        for i in range(7, -1, -1):
            fb = ((crc >> 15) ^ (byte >> i)) & 1
            crc = (crc << 1) & 0xFFFF
            if fb:
                crc ^= 0x1021
    return crc


def with_crc(body: Sequence[int]) -> List[int]:
    """``body`` (mark first) and its CRC, preset over A1 A1 A1."""
    crc = crc16([0xA1, 0xA1, 0xA1] + list(body))
    return list(body) + [crc >> 8, crc & 0xFF]


def id_field(c: int, h: int, r: int, n: int) -> List[int]:
    """The ID field after the three A1*: FE C H R N CRC CRC."""
    return with_crc([MARK_ID, c, h, r, n])


def data_field(payload: Sequence[int], mark: int = MARK_DATA) -> List[int]:
    return with_crc([mark] + list(payload))


def mfm_cells(data: Sequence[int], prev: int = 0) -> Tuple[List[int], int]:
    """[WP]'s rule: two cells a bit, clock = NOR of the data bits either
    side, data bit. Returns the cells and the last data bit."""
    cells = []
    for byte in data:
        for i in range(7, -1, -1):
            bit = (byte >> i) & 1
            cells += [1 if (prev == 0 and bit == 0) else 0, bit]
            prev = bit
    return cells, prev


def word_cells(word: int) -> List[int]:
    return [(word >> i) & 1 for i in range(15, -1, -1)]


def sync_cells() -> List[int]:
    """Three A1*, each with its missing clock."""
    return word_cells(SYNC_A1) * 3


def field_cells(field: Sequence[int], prev: int) -> Tuple[List[int], int]:
    """12 x 00, A1* A1* A1*, then ``field`` (mark first)."""
    cells, prev = mfm_cells([0] * SYNC00, prev)
    cells += sync_cells()
    more, prev = mfm_cells(field, 1)          # A1 ends with data bit 1
    return cells + more, prev


def sector_cells(c: int, h: int, r: int, n: int, payload: Sequence[int],
                 gap1: int = GAP1, gap2: int = GAP2, gap3: int = GAP3,
                 prev: int = 1) -> List[int]:
    """One sector as [FD] p. 17 lays it out: gap I, ID field, gap II, data
    field, gap III. ``prev`` is the data bit before the first cell."""
    cells, prev = mfm_cells([0x4E] * gap1, prev)
    more, prev = field_cells(id_field(c, h, r, n), prev)
    cells += more
    more, prev = mfm_cells([0x4E] * gap2, prev)
    cells += more
    more, prev = field_cells(data_field(payload), prev)
    cells += more
    more, prev = mfm_cells([0x4E] * gap3, prev)
    return cells + more


# ------------------------------------------------------------------ decoder
@dataclasses.dataclass
class Field:
    """One field found after three A1*: the mark, the bytes after it (its
    CRC included), whether the CRC checks, and the clock cells inside it that
    break [WP]'s rule."""

    mark: int
    data: List[int]
    crc_ok: bool
    clock_errors: int
    start: int                       # the cell where the mark begins


def _word(cells: Sequence[int], at: int) -> int:
    w = 0
    for c in cells[at:at + 16]:
        w = (w << 1) | c
    return w


def _byte(cells: Sequence[int], at: int, prev: int) -> Tuple[int, int, int]:
    """The data byte in cells ``at .. at+15``, the clock errors, the last bit."""
    byte = errors = 0
    for j in range(8):
        clock, bit = cells[at + 2 * j], cells[at + 2 * j + 1]
        if clock != (1 if prev == 0 and bit == 0 else 0):
            errors += 1
        byte = (byte << 1) | bit
        prev = bit
    return byte, errors, prev


def decode_cells(cells: Sequence[int], data_len: int = 128) -> List[Field]:
    """Every field in a cell stream: three A1* in a row, then a mark; an ID
    field (FE) carries 6 more bytes, any other mark ``data_len`` + 2."""
    out: List[Field] = []
    i = 0
    while i + 64 <= len(cells):
        if not all(_word(cells, i + 16 * j) == SYNC_A1 for j in range(3)):
            i += 1
            continue
        at = i + 48
        mark, errors, prev = _byte(cells, at, 1)
        count = 6 if mark == MARK_ID else data_len + 2
        data: List[int] = []
        p = at + 16
        while len(data) < count and p + 16 <= len(cells):
            b, e, prev = _byte(cells, p, prev)
            data.append(b)
            errors += e
            p += 16
        ok = len(data) == count and crc16([0xA1] * 3 + [mark] + data) == 0
        out.append(Field(mark, data, ok, errors, at))
        i = p
    return out


def intervals(cells: Sequence[int]) -> List[int]:
    """The flux intervals, in half-cells, between successive 1 cells."""
    ones = [i for i, c in enumerate(cells) if c]
    return [b - a for a, b in zip(ones, ones[1:])]


def cells_from_intervals(gaps: Sequence[int]) -> List[int]:
    cells = [1]
    for g in gaps:
        cells += [0] * (g - 1) + [1]
    return cells


# ------------------------------------------------------------- bench models
class MfmDrive(Model):
    """The drive end of the writer: WRITE DATA (active low, one pulse a flux
    transition, [FD] p. 19) and WRITE GATE (active low). Records every
    falling edge of WRITE DATA and every pulse's width, in clocks."""

    def __init__(self, wd: PinSpec, wg: Optional[PinSpec] = None) -> None:
        self.wd = pad_of(wd)
        self.wg = pad_of(wg) if wg is not None else None
        self.level: Optional[int] = None      # until the line is first high
        self.falls: List[int] = []
        self.widths: List[int] = []
        self.gate_at_fall: List[int] = []
        self.gate_edges: List[Tuple[int, int]] = []
        self._gate = 1

    def observe(self, lines: Lines) -> None:
        if self.wg is not None:
            g = lines.get(self.wg)
            if self.level is not None and g != self._gate:
                self.gate_edges.append((lines.cycle, g))
                self._gate = g
        level = lines.get(self.wd)
        if self.level is None:
            # before the program drives the line (it resets low) nothing counts
            self.level = 1 if level == 1 else None
            return
        if level == self.level:
            return
        if level == 0:
            self.falls.append(lines.cycle)
            self.gate_at_fall.append(self._gate)
        elif self.falls:
            self.widths.append(lines.cycle - self.falls[-1])
        self.level = level

    def check(self, clock_hz: float, tol: float = WINDOW) -> List[str]:
        """Every breach of [FD] p. 19: pulse width outside 300-500 ns, an
        interval not within ``tol`` half-cells of 2, 3 or 4, a pulse while
        WRITE GATE is off, WRITE GATE on less than 2 us before the first."""
        half = HALF_CELL_S * clock_hz
        lo, _, hi = (w * clock_hz for w in WRITE_PULSE_S)
        bad = []
        for k, w in enumerate(self.widths):
            if not lo <= w <= hi:
                bad.append("pulse %d is %d clocks wide" % (k, w))
        for k, (a, b) in enumerate(zip(self.falls, self.falls[1:])):
            n = (b - a) / half
            if round(n) not in (2, 3, 4) or abs(n - round(n)) > tol:
                bad.append("interval %d is %.3f half-cells" % (k, n))
        if self.wg is not None:
            if any(g != 0 for g in self.gate_at_fall):
                bad.append("a pulse with WRITE GATE off")
            on = [c for c, g in self.gate_edges if g == 0]
            if self.falls and (not on or self.falls[0] - on[0]
                               < WRITE_GATE_LEAD_S * clock_hz):
                bad.append("WRITE GATE on too late before the first pulse")
        return bad

    def cells(self, clock_hz: float) -> List[int]:
        """The cell stream the pulses write, from the first pulse."""
        half = HALF_CELL_S * clock_hz
        return cells_from_intervals(
            [int(round((b - a) / half)) for a, b in zip(self.falls, self.falls[1:])])


class FluxSource(Model):
    """The drive end of the reader: READ DATA, idle high, one active-low
    pulse a flux transition ([FD] p. 19 note 1: 100-300 ns at CLK 2 MHz,
    doubled here, so 200-600 ns). :meth:`send` places a pulse at every 1
    cell of ``cells``, ``half`` clocks a cell (a float; placed from ``at``
    without drift), each moved by a uniform jitter of up to ``jitter``
    half-cells either way and by ``shift[k]`` half-cells for the k-th 1 cell
    (a bit shift: adjacent transitions pushed apart on the medium)."""

    def __init__(self, pin: PinSpec, pulse: int = 20) -> None:
        self.pad = pad_of(pin)
        self.pulse = pulse
        self.level = 1
        self.falls: List[int] = []
        self._changes: List[Tuple[int, int]] = []

    def send(self, cells: Sequence[int], half: float, at: int,
             jitter: float = 0.0, seed: int = 1,
             shift: Optional[Dict[int, float]] = None) -> List[int]:
        rng = random.Random(seed)
        shift = shift or {}
        times = []
        k = 0
        for i, c in enumerate(cells):
            if not c:
                continue
            move = rng.uniform(-jitter, jitter) + shift.get(k, 0.0)
            times.append(int(round(at + (i + move) * half)))
            k += 1
        for t in times:
            self._changes += [(t, 0), (t + self.pulse, 1)]
        self._changes.sort()
        self.falls += times
        return times

    def drive(self, drive: Drive, cycle: int) -> None:
        changes = self._changes
        while changes and changes[0][0] <= cycle:
            self.level = changes.pop(0)[1]
        drive.set(self.pad, self.level)


class PulseWire(Model):
    """The board wire from the writer's WRITE DATA pin to the reader's READ
    DATA pin, as a drive in loopback would pass it (one clock of delay)."""

    def __init__(self, src: PinSpec, dst: PinSpec) -> None:
        self.src, self.dst = pad_of(src), pad_of(dst)
        self.level = 1

    def observe(self, lines: Lines) -> None:
        self.level = lines.get(self.src)

    def drive(self, drive: Drive, cycle: int) -> None:
        drive.set(self.dst, self.level)
