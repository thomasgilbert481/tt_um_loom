"""L3 for firmware/composite_ntsc.loom (W-103).

Monochrome composite video with NTSC-M / RS-170A timing from two pins and a
resistor DAC: 525 lines, 2:1 interlace, the pre- and post-equalising
pulses and the serrated vertical sync, a picture of eight vertical bars
and a white line at a row the host sends. ``tools.protomodels.composite.
CompositeMonitor`` is written from SMPTE 170M-2004 (Table 2, Figure 7,
clause 11): it sorts each sync pulse by width, locks to each field by its
equalising run, checks that every pulse is the one Figure 7 puts at its
half line with its width, that no white falls in the blanking, fits fH
against 2/455 fsc, and rebuilds the picture as 64 columns a line.

On the RTL one scenario runs: field 1's vertical interval and its first
three picture lines, 33 sync pulses, 2.13 ms of simulated time (106 k
clocks with the host load; 64 s of Icarus on 2026-11-07, about 1,700
clocks a second with six agents on the laptop), with the host's white
line on row 1, so it covers every pulse kind, the end of blanking, the
bars and the host path. A whole frame is 1.67 M clocks, some 16 minutes
of Icarus at that rate, so the frame, the interlace (field 2 is half a
line later) and a moving line are model only.

Expected values come from SMPTE 170M (the monitor's constants) and from
what the program header promises (eight bars, white first, across the
picture window; the host's row white in both fields), never from what the
model or the RTL happens to do.
"""

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels.composite import (BLANK_END_MAX_S, BLANK_END_MIN_S,
                                         CLOCK, COLS, FH_TOL, FP_MIN_S, HALF_S, HALVES,
                                         LINE_S, VI_PULSES, WINDOW_S,
                                         BLANK_END_S, CompositeMonitor)
from tools.tests.fw_backend import model_only

ISA = load()
PROGRAM = REPO / "firmware" / "composite_ntsc.loom"
BOOT = 6000                         # clocks of host load before RUN, at most
NBLANK = 11                         # lines 10 .. 20 (field 1), 273 .. 283 (field 2)
#: one picture line as 64 columns: eight bars of 8, white first
BARS = ([1] * 8 + [0] * 8) * 4
WHITE = [1] * COLS
#: where each white bar should start and end, us after the H ref: the
#: picture window cut in eight
BAR_US = [(BLANK_END_S + i * WINDOW_S / 8) * 1e6 for i in range(9)]
EDGE_US = 0.06                      # one tick (39.9 ns) plus one clock (20 ns)


def composite_bench(backend):
    """A bench whose chip has the latch (6.10) and the FIFOs."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "SETPD"), imem_words=512)
    return backend.bench()


def boot(backend, row=None):
    bench = composite_bench(backend)
    mon = bench.add(CompositeMonitor())
    program = assemble_file(PROGRAM, isa=ISA, strict=True)
    loom = backend.loom(bench, isa=ISA)
    loom.load(program)
    if row is not None:
        loom.push(0, [row])
    loom.run(0)
    return bench, mon, loom


def run_pulses(bench, mon, count):
    """Run until ``count`` sync pulses have been seen."""
    budget = BOOT + int((count + 4) * LINE_S * CLOCK)
    assert bench.run_until(lambda: len(mon.pulses) >= count, budget, every=256), \
        "only %d pulses in %d clocks: %s" % (len(mon.pulses), budget, mon.report())


def run_fields(bench, mon, fields, lines=0):
    """Run until field index ``fields`` has started and ``lines`` H syncs of
    it have followed its vertical interval."""
    budget = BOOT + int((fields * HALVES / 2 + lines + 12) * LINE_S * CLOCK)

    def done():
        if len(mon.fields) <= fields:
            return False
        n = sum(1 for p in mon.pulses[-(lines + VI_PULSES + 4):]
                if p.field == fields and p.kind == "H")
        return n >= lines
    assert bench.run_until(done, budget, every=1024), \
        "%d fields in %d clocks: %s" % (len(mon.fields), budget, mon.report())


def test_composite_program_assembles_strict_with_no_diagnostic():
    """No diagnostic, every deadline pair proved, thread 0's quarter only."""
    prog = assemble_file(PROGRAM, isa=ISA, strict=True)
    assert prog.diagnostics == []
    assert sorted(prog.threads) == [0]
    assert max(prog.words) < 0x080
    rep = prog.deadlines[0]
    assert rep.pairs and not rep.unbounded and not rep.infeasible
    assert rep.worst_slack >= 0


def check_vertical_interval(mon, field):
    """Figure 7: six equalising, six broad, six equalising pulses at half
    line spacing from the field's start, then H syncs a line apart, field 1
    from half line 18 and field 2 from 19; every width inside 170M's."""
    ps = [p for p in mon.pulses if p.field == field]
    assert [p.kind for p in ps[:VI_PULSES]] == ["E"] * 6 + ["B"] * 6 + ["E"] * 6
    assert [p.half for p in ps[:VI_PULSES]] == list(range(VI_PULSES))
    first = VI_PULSES if mon.fields[field].parity == 1 else VI_PULSES + 1
    assert [p.half for p in ps[VI_PULSES:]] == list(range(first, first + 2 * len(ps[VI_PULSES:]), 2))
    assert all(p.kind == "H" for p in ps[VI_PULSES:])
    us = lambda clocks: clocks / CLOCK * 1e6                       # noqa: E731
    for p in ps:
        lo, hi = {"H": (4.60, 4.80), "E": (2.20, 2.40), "B": (0, 99)}[p.kind]
        assert lo <= us(p.width) <= hi, (p, us(p.width))
    assert all(4.60 <= us(s) <= 4.80 for s in mon.serrations), mon.serrations


def runs_in_line(mon, field, line):
    """The white runs of one line, as (start, end) in us after its H ref."""
    refs = mon.line_refs(field)
    a, b = refs[line], refs.get(line + 1, refs[line] + LINE_S * CLOCK)
    return [(after, after + (e - s) / CLOCK * 1e6)
            for s, e, after, _ in mon.white_runs if a <= s < b]


def check_bars_line(mon, field, line):
    """Four white bars, each edge within EDGE_US of an eighth of the window."""
    runs = runs_in_line(mon, field, line)
    want = [(BAR_US[2 * i], BAR_US[2 * i + 1]) for i in range(4)]
    assert len(runs) == 4, runs
    for (s, e), (ws, we) in zip(runs, want):
        assert abs(s - ws) <= EDGE_US and abs(e - we) <= EDGE_US, (runs, want)


def check_white_line(mon, field, line):
    """The window white, from inside 170M's blanking end (9.10 .. 9.40 us)
    to at least 1.4 us (front porch 1.5 -0.1) before the next H ref."""
    runs = runs_in_line(mon, field, line)
    assert len(runs) == 1, runs
    s, e = runs[0]
    assert BLANK_END_MIN_S * 1e6 <= s <= BLANK_END_MAX_S * 1e6, runs
    assert e <= (LINE_S - FP_MIN_S) * 1e6 + 1e-9, runs
    assert abs(e - BAR_US[8]) <= EDGE_US, runs


# ---------------------------------------------------------------- the RTL
def test_composite_vertical_interval_and_first_lines(backend):
    """Field 1 from its first pulse: the 9-line vertical interval, lines 10
    to 20 with H sync only, then picture lines 21 to 23: bars, the host's
    row 1 white, bars; no white in the blanking, nothing illegal."""
    bench, mon, loom = boot(backend, row=1)
    run_pulses(bench, mon, VI_PULSES + NBLANK + 4)
    assert mon.violations == [], mon.report()
    assert len(mon.fields) == 1 and mon.fields[0].parity == 1, mon.report()
    check_vertical_interval(mon, 0)
    pic = mon.picture(0)
    assert sorted(pic) == [21, 22, 23], mon.report()
    assert pic[21] == BARS and pic[22] == WHITE and pic[23] == BARS, pic
    check_bars_line(mon, 0, 21)
    check_white_line(mon, 0, 22)
    check_bars_line(mon, 0, 23)
    assert mon.white_runs[0][0] > mon.line_refs(0)[21], "white before line 21"
    assert mon.bad_level == 0
    t = mon.timing()
    assert abs(t["ppm"]) <= t["ppm_limit"] and t["jitter_ns"] <= 25, t
    assert loom.badop() == 0


# ---------------------------------------------------------------- the model
@model_only("1.75 M clocks: four to sixteen minutes of Icarus alone")
def test_composite_frame_interlace_and_moving_line(backend):
    """A whole frame and the start of the next: field 1, field 2 starting
    half a line after line 263's H ref, field 1 again a whole line after
    line 525's; 525 half lines a field; fH inside 2/455 fsc +-2.8 ppm; the
    rebuilt frame all bars but rows 100 of both fields (frame rows 200 and
    201), and a new row sent during frame 1 moves the line in frame 2."""
    bench, mon, loom = boot(backend, row=100)
    run_pulses(bench, mon, VI_PULSES + NBLANK + 4)      # row 100 taken at line 1
    loom.push(0, [5])
    run_fields(bench, mon, 2, lines=NBLANK + 8)         # frame 2, lines 21 .. 27
    assert mon.violations == [], mon.report()
    assert [f.parity for f in mon.fields] == [1, 2, 1], mon.report()
    half, line = HALF_S * CLOCK, LINE_S * CLOCK
    for a, b in zip(mon.fields, mon.fields[1:]):
        assert abs(b.start - a.start - HALVES * half) <= 2, (a, b)
    for f, gap in ((1, half), (2, line)):              # the interlace
        last_h = [p for p in mon.pulses if p.field == f - 1][-1]
        assert last_h.kind == "H"
        assert abs(mon.fields[f].start - last_h.fall - gap) <= 2, (f, last_h)
    for f, count in ((0, 254), (1, 253)):              # lines 10 .. 263, 273 .. 525
        check_vertical_interval(mon, f)
        assert sum(p.kind == "H" for p in mon.pulses if p.field == f) == count
    t = mon.timing()
    assert abs(t["ppm"]) <= t["ppm_limit"] and t["jitter_ns"] <= 25, t
    # over a frame the fit resolves well under a ppm: hold fH to 170M's own
    # +-2.79 ppm (fsc +-10 Hz) with no allowance for the fit
    assert abs(t["ppm"]) <= FH_TOL * 1e6, t
    rows = mon.frame(0)
    assert len(rows) == 2 * 242
    for k, row in enumerate(rows):
        assert row == (WHITE if k in (200, 201) else BARS), k
    check_white_line(mon, 0, 121)
    check_white_line(mon, 1, 384)
    check_bars_line(mon, 1, 284)
    nxt = mon.picture(2)
    assert sorted(nxt) == list(range(21, 28)), sorted(nxt)
    for ln, row in nxt.items():
        assert row == (WHITE if ln == 26 else BARS), ln
    assert mon.bad_level == 0
    assert loom.badop() == 0


@model_only("three runs of 80 k clocks; the RTL scenario covers the same lines and the host path")
def test_composite_no_line_before_a_row_and_out_of_range(backend):
    """With no word sent, or a row of 242 or more, no line is white."""
    for row in (None, 242, 0xFFFF):
        bench, mon, loom = boot(backend, row=row)
        run_pulses(bench, mon, VI_PULSES + NBLANK + 4)
        assert mon.violations == [], mon.report()
        pic = mon.picture(0)
        assert sorted(pic) == [21, 22, 23] and all(r == BARS for r in pic.values())
        assert loom.badop() == 0


@model_only("the RTL scenario has row 1; row 0 and row 241 are its two ends")
def test_composite_first_row_white(backend):
    """Row 0 is line 21, the first picture line of field 1."""
    bench, mon, loom = boot(backend, row=0)
    run_pulses(bench, mon, VI_PULSES + NBLANK + 3)
    assert mon.violations == [], mon.report()
    pic = mon.picture(0)
    assert pic[21] == WHITE and pic[22] == BARS, pic
    check_white_line(mon, 0, 21)
