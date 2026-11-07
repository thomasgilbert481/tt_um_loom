"""L3 for firmware/vga_sync.loom and firmware/vga_bars.loom (W-068).

VGA 640x480 at 60 Hz from firmware: HSYNC and VSYNC with VESA DMT's porches,
and a picture of 2-bit-per-channel colour on a TinyVGA Pmod (on Loom's uio
header: the TT pinout's uo_out[6] and [7] are Loom's host port).
``tools.protomodels.vga.VgaMonitor`` is written from DMT page 21 and the
Pmod's pinout: it times every sync edge, locks an 800-pixel line to HSYNC,
samples the colour at every pixel centre and flags any colour in the
blanking.

On the RTL (about 6,400 clocks a second) the scenarios are short: the sync
over the first 16 lines (VSYNC falls on line 10), and the bars' first two
active lines, 47 lines in. The whole frame, the other patterns and a
pattern change are model only.

Expected values come from DMT (the line within +-0.5 % of 31.778 us, HSYNC
96 px, VSYNC 2 lines, 525 lines, 35 lines from VSYNC to the first active
row), from the patterns as the program header defines them (eight bars of
80 px; an 8 x 8 board of 80 px x 60 lines) and from SEMANTICS: a
deadline-latched write lands on the exact clock (6.10), so every line has
the same length, and an OUT lands on the 4-clock slot grid (2), so a bar
edge is within 2 px of its place.
"""

import os
from pathlib import Path

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels.bench import Lines, Model, pad_of
from tools.protomodels.vga import (BLACK, CLOCK, H_TIMING, LINE_S, PIXEL_TOL,
                                   V_TIMING, VgaMonitor)
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
SYNC = FIRMWARE / "vga_sync.loom"
BARS = FIRMWARE / "vga_bars.loom"
LINE = 1588                         # the programs' line, clocks (their header)
BOOT = 6000                         # clocks of host load before RUN, at most
FIRST_ROW_LINE = 10 + V_TIMING[2] + V_TIMING[3]   # 45: VSYNC falls on line 10
TABLES = 0x0C0

W, K = (3, 3, 3), (0, 0, 0)
#: The pattern words' colours, as the program header names them
BARS_ROW = [W, (3, 3, 0), (0, 3, 3), (0, 3, 0), (3, 0, 3), (3, 0, 0), (0, 0, 3), K]
CHECK_A = [W, K] * 4
CHECK_B = [K, W] * 4
LEVELS_A = [(3, 3, 3), (2, 2, 2), (1, 1, 1), K, (3, 0, 0), (2, 0, 0), (1, 0, 0), K]
LEVELS_B = [(0, 3, 0), (0, 2, 0), (0, 1, 0), K, (0, 0, 3), (0, 0, 2), (0, 0, 1), K]
EDGE_PX = 1                         # the 4-clock grid is within 0.8 px of 80 i
HSW, HBP = 191, 95                  # the programs' HSYNC and back porch, clocks
#: Where each bar starts, clocks after S, and the window end (the header)
BAR_EDGES = [0, 160, 316, 476, 636, 792, 952, 1112]
COLOUR_END = 1268


class EdgeLog(Model):
    """Every change of one pad, as ``(cycle, new value)``."""

    def __init__(self, pin) -> None:
        self.pad = pad_of(pin)
        self.edges = []
        self._last = None

    def observe(self, lines: Lines) -> None:
        v = lines.get(self.pad)
        if self._last is not None and v != self._last:
            self.edges.append((lines.cycle, v))
        self._last = v


def vga_bench(backend):
    """A bench whose chip has the latch (6.10), LD (6.11) and the FIFOs."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "SETPD", "DMEM"), imem_words=512)
    return backend.bench()


def boot(backend, path, pattern=None):
    bench = vga_bench(backend)
    mon = bench.add(VgaMonitor())
    program = assemble_file(path, isa=ISA, strict=True)
    loom = backend.loom(bench, isa=ISA)
    loom.load(program)
    if pattern is not None:
        loom.push(0, [pattern])
    loom.run(0)
    return bench, mon, loom


def run_lines(bench, mon, lines, extra=4 * LINE):
    """Run until ``lines`` HSYNC falls have been seen."""
    budget = lines * LINE + extra
    assert bench.run_until(lambda: len(mon.h_falls) >= lines, budget, every=256), \
        "only %d lines in %d clocks: %s" % (len(mon.h_falls), budget, mon.report())


def check_sync(mon):
    """Every measured figure inside DMT's window."""
    assert mon.violations == [], mon.report()
    for us in mon.line_us():
        assert abs(us / (LINE_S * 1e6) - 1) <= PIXEL_TOL, us
    assert len(set(b - a for a, b in zip(mon.h_falls, mon.h_falls[1:]))) == 1, \
        "latched edges (SEMANTICS 6.10): every line the same: " + mon.report()
    for low, tp in zip(mon.h_low, mon.tpix):
        assert abs(low / tp - H_TIMING[2]) <= 1.0, (low, tp)


def test_vga_programs_assemble_strict_with_no_diagnostic():
    """Both programs: no diagnostic, every pair proved, thread 0 only."""
    sync = assemble_file(SYNC, isa=ISA, strict=True)
    bars = assemble_file(BARS, isa=ISA, strict=True)
    assert sync.diagnostics == [] and bars.diagnostics == []
    assert sorted(sync.threads) == [0] and sorted(bars.threads) == [0]
    assert max(sync.words) < 0x080
    code = [a for a in bars.words if a < TABLES]
    assert max(code) < 0x080
    assert sorted(a for a in bars.words if a >= TABLES) == list(range(TABLES, 0x100))
    for prog in (sync, bars):
        rep = prog.deadlines[0]
        assert rep.pairs and not rep.unbounded and not rep.infeasible
        assert rep.worst_slack >= 0


def line_starts(mon, first, count):
    """The line start S after HSYNC falls ``first .. first + count - 1``:
    the fall + HSYNC low + back porch, the programs' 191 + 95 clocks."""
    return [mon.h_falls[i] + HSW + HBP for i in range(first, first + count)]


def check_row(row, expected):
    """A row shows ``expected``, eight bars of 80 px, within the 4-clock
    grid: each bar's colour at its centre, each change within EDGE_PX of a
    bar boundary."""
    for i, colour in enumerate(expected):
        assert row[80 * i + 40] == colour, (i, row[80 * i + 40], colour)
    for x in range(1, 640):
        if row[x] != row[x - 1]:
            near = round(x / 80) * 80
            assert abs(x - near) <= EDGE_PX, (x, row[x - 1], row[x])
    assert all(row[x] == expected[0] for x in range(EDGE_PX, 80 - EDGE_PX))


# ---------------------------------------------------------------- the RTL
def test_vga_sync_vsync_and_display_enable(backend):
    """vga_sync: the 45 blanking lines and two active ones against DMT, VSYNC
    two lines low, DE exactly from S to S + 1268, colour pins black."""
    bench, mon, loom = boot(backend, SYNC)
    de = bench.add(EdgeLog("OUT0"))
    run_lines(bench, mon, FIRST_ROW_LINE + 2)
    check_sync(mon)
    assert mon.v_falls and mon.v_low_lines == [V_TIMING[2]], mon.report()
    assert all(abs(o) <= mon.tpix[-1] for o in mon.vs_offset), mon.vs_offset
    assert mon.blank_clocks == 0 and mon.colour_edges == []
    # the fall before active row 0 is fall FIRST_ROW_LINE - 1 (line 0 starts
    # before the first fall), and DE stays low until then
    starts = line_starts(mon, FIRST_ROW_LINE - 1, 2)
    want = [e for s in starts for e in ((s, 1), (s + COLOUR_END, 0))]
    assert de.edges[:4] == want, (de.edges[:4], want)
    assert loom.badop() == 0


def test_vga_bars_first_two_active_rows(backend):
    """vga_bars: the 45 blanking lines, then rows 0 and 1 of the bars, every
    colour edge on its clock and nothing in the blanking."""
    bench, mon, loom = boot(backend, BARS)
    run_lines(bench, mon, FIRST_ROW_LINE + 3)      # a row is filed at the next start
    check_sync(mon)
    assert mon.v_low_lines == [V_TIMING[2]], mon.report()
    rows = mon.rows()
    assert len(rows) >= 2, mon.report()
    for row in rows[:2]:
        check_row(row, BARS_ROW)
    starts = line_starts(mon, FIRST_ROW_LINE - 1, 2)
    # bar 7 is black, so the end of the window at S + 1268 is no edge
    want = [(s + x, c) for s in starts for x, c in zip(BAR_EDGES, BARS_ROW)]
    got = [e for e in mon.colour_edges if e[0] >= starts[0]][:len(want)]
    assert got == want, (got, want)
    assert mon.blank_clocks == 0
    assert loom.badop() == 0


# ---------------------------------------------------------------- the model
@model_only("two frames, 1 M clocks: nearly three minutes of Icarus alone")
def test_vga_bars_whole_frame_then_a_pattern_change(backend):
    """Frame 0: all 480 rows of bars, the 525-line frame and its PPM image.
    A word pushed in the middle of it (row 200) changes frame 1 only."""
    bench, mon, loom = boot(backend, BARS)
    run_lines(bench, mon, FIRST_ROW_LINE + 200)
    loom.push(0, [1])
    run_lines(bench, mon, FIRST_ROW_LINE + sum(V_TIMING) + 2 * 60 + 2)
    check_sync(mon)
    assert len(mon.frames) == 1 and len(mon.v_falls) == 2, mon.report()
    assert mon.v_falls[1] - mon.v_falls[0] == sum(V_TIMING)
    assert mon.v_low_lines == [V_TIMING[2]] * 2
    frame = mon.frames[0]
    assert len(frame.rows) == V_TIMING[0]
    assert all(row == frame.rows[0] for row in frame.rows)
    check_row(frame.rows[0], BARS_ROW)
    nxt = mon.rows()
    assert len(nxt) >= 2 * 60, mon.report()
    check_row(nxt[0], CHECK_A)
    check_row(nxt[59], CHECK_A)
    check_row(nxt[60], CHECK_B)
    check_row(nxt[119], CHECK_B)
    assert mon.blank_clocks == 0
    assert loom.badop() == 0
    out = os.environ.get("LOOM_VGA_PPM")
    if out:
        Path(out).write_bytes(frame.ppm())


@model_only("140 k clocks each, the bars' row test covers the RTL")
@pytest.mark.parametrize("pattern", [1, 2, 3])
def test_vga_bars_other_patterns(backend, pattern):
    """Patterns 1 to 3, picked by a word pushed before RUN: the first two
    checkerboard rows (120 lines), so both halves of each pattern."""
    bench, mon, loom = boot(backend, BARS, pattern=pattern)
    run_lines(bench, mon, FIRST_ROW_LINE + 2 * 60 + 1)
    check_sync(mon)
    rows = mon.rows()
    assert len(rows) >= 2 * 60, mon.report()
    halves = {1: (CHECK_A, CHECK_B), 2: (LEVELS_A, LEVELS_B), 3: ([W] * 8, [W] * 8)}
    a, b = halves[pattern]
    for k, row in enumerate(rows[:120]):
        check_row(row, a if k < 60 else b)
    assert mon.blank_clocks == 0
    assert loom.badop() == 0
