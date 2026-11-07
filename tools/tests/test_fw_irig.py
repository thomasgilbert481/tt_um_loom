"""L3 for the IRIG-B (B00x, dc level shift) programs, on the model and the RTL.

``firmware/irig_b_gen.loom`` (TX = OUT0) sends IRIG-B frames from a time the
host gives and counts the seconds itself; ``firmware/irig_b_rx.loom`` (RX =
IN0, PPS = OUT0) reads them, reports the time of every whole frame and
raises a 1 PPS pulse on Pr's leading edge. The reference is
``tools.protomodels.irig``, written from IRIG 200-16: ``frame_symbols``
encodes a time, ``elements`` and ``frames`` hold a recorded pin to the
widths of Table 5-6 and the 10 ms grid of 3.3, ``IrigSource`` feeds the
reader frames with faults, and ``Wire`` joins the two programs' pins when
they run together (generator thread 0, reader thread 2, its PPS moved to
OUT1, because the generator spills into thread 1's quarter).

Expected values come from 200-16 and SEMANTICS: a deadline-latched edge
lands on the exact clock edge of its tick (6.10), so with a whole number of
clocks a millisecond every element edge of the generator is exactly on the
grid, every width exactly 2, 5 or 8 ms, and Pr to Pr exactly 1000 ticks;
the reader's PPS follows the RX pad by 8 to 11 clocks (SEMANTICS 3: the
synchroniser and the 4-clock slot grid, then one slot for the SETP).

Scaled time. IRIG-B at 50 MHz is 500,000 clocks an element, 50 million a
frame, out of reach of either simulator. The RTL and most model scenarios
run the same programs with the same host words on a board clocked at
40 kHz: 1 ms is 40 clocks (TICK_INT 40, the shortest tick the generator's
schedule is proved for), the reader's 0.2 ms unit 8 clocks, a frame 40,000
clocks. Only the number of clocks a tick changes; every count of ticks,
every width and every frame is the one a 50 MHz board sends. The
``model_only`` real-time scenario runs a 200 kHz board in real seconds
(1 ms = 200 clocks, the reader's unit 40) for three frames.
"""

import pytest

from tools.loomasm import assemble_file, assemble_text
from tools.loomisa import REPO, load
from tools.protomodels import irig as I
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
IMEM = 512


class Board:
    """A clock: the generator's tick (1 ms) and the reader's unit (0.2 ms)."""

    def __init__(self, name, ms):
        self.name, self.ms = name, ms

    @property
    def unit(self):
        return self.ms // 5

    @property
    def element(self):
        return 10 * self.ms

    @property
    def frame(self):
        return 100 * self.element

    def __repr__(self):
        return self.name


SCALED = Board("40kHz", 40)
REAL = Board("200kHz", 200)
SLOW = Board("80kHz", 80)                   # the reader's unit 16 clocks
PPS_LATENCY = range(8, 12)                  # clocks, RX pad to PPS pad (header)


def program(name, thread=0, pps="OUT0"):
    """A program from ``firmware/``, moved to ``thread`` and its PPS pin to
    ``pps`` if asked."""
    path = FIRMWARE / (name + ".loom")
    if thread == 0 and pps == "OUT0":
        return assemble_file(path, isa=ISA, strict=True)
    text = path.read_text(encoding="utf-8")
    assert text.count(".thread 0") == 1
    text = text.replace(".thread 0", ".thread %d" % thread)
    text = text.replace("PPS = OUT0", "PPS = %s" % pps)
    return assemble_text(text, name + ".loom", isa=ISA, imem_words=IMEM, strict=True)


def irig_bench(backend):
    """The FIFOs, the deadline latch, the data memory and the 512-word
    memory; the RTL has them all and ``RtlBench`` takes neither keyword."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "SETPD", "DMEM"), imem_words=IMEM)
    return backend.bench()


def start(backend, bench, loads):
    """Load ``[(program, thread, host words)]``, run the threads, then push
    each thread its host words (program headers)."""
    loom = backend.loom(bench, isa=ISA)
    words = {}
    for prog, _, _ in loads:
        words.update(prog.words)
    loom.load(words)
    loom.run([thread for _, thread, _ in loads])
    for _, thread, config in loads:
        loom.push(thread, config)
    return loom


def gen_words(board, t, z):
    """irig_b_gen.loom's host words for start time ``t``, signal B00``z``."""
    return [board.ms, z | ((t.sbs >> 16) << 5), t.sbs & 0xFFFF,
            I.bcd(t.second), I.bcd(t.minute), I.bcd(t.hour), I.bcd(t.day), I.bcd(t.year)]


def rx_words(board):
    return [board.unit, 0]


def report(t, z=0):
    """The four words irig_b_rx.loom reports for a frame carrying ``t``."""
    sbs = t.sbs if I.has_sbs(z) else 0
    year = I.bcd(t.year) if I.has_year(z) else 0
    return [I.bcd(t.minute) << 8 | I.bcd(t.second), I.bcd(t.hour) << 10 | I.bcd(t.day),
            year | (sbs >> 16) << 8, sbs & 0xFFFF]


# -------------------------------------------------------------- generator
def gen_run(backend, board, t, z, frames):
    """Run the generator from ``t`` until ``frames`` whole frames and the
    next Pr are on OUT0; returns the pulses and the frames decoded."""
    bench = irig_bench(backend)
    rec = bench.add(I.EdgeRecorder("OUT0"))
    loom = start(backend, bench, [(program("irig_b_gen"), 0, gen_words(board, t, z))])
    want = frames * I.ELEMENTS + 1
    budget = (frames + 1) * board.frame + 10 * board.element
    assert bench.run_until(lambda: len(rec.rises()) >= want, budget, every=board.element), (
        "%d of %d leading edges" % (len(rec.rises()), want))
    bench.step(board.element)
    pulses = I.elements(rec.edges, board.element)
    got = I.frames(pulses, board.element, z, first_is_pr=True)
    assert loom.badop() == 0
    return pulses, got


def check_gen(pulses, got, board, t, z, frames):
    """200-16 to the clock: every pulse exactly 2, 5 or 8 ms (Table 5-6),
    every leading edge on the 10 ms grid from Pr (3.3), Pr to Pr exactly
    1 s, and the frames carrying ``t``, ``t + 1 s``, ... (3.6)."""
    assert all(p.symbol is not None for p in pulses), \
        [p for p in pulses if p.symbol is None][:3]
    assert len(got) >= frames
    for k, f in enumerate(got[:frames]):
        assert f.grid_errors == [], f.grid_errors
        assert f.decoded.errors == [], f.decoded.errors
        want = t.later(k)
        assert f.decoded.time == (want if I.has_year(z) else
                                  I.Time(0, want.day, want.hour, want.minute, want.second))
        assert f.decoded.sbs == (want.sbs if I.has_sbs(z) else 0)
        assert f.decoded.cf == 0
    prs = [f.pr for f in got]
    assert [b - a for a, b in zip(prs, prs[1:])] == [board.frame] * (len(prs) - 1)
    assert prs[0] == pulses[0].lead                 # the first pulse is Pr


def test_irig_gen_year_end(backend):
    """B004 from 2024 (a leap year) day 366 23:59:59: two frames, the second
    2025 day 001 00:00:00 with SBS 0, every edge on its clock. The
    PHY-GL-SDF candidate (two frames, 80,000 clocks)."""
    t = I.Time(24, 366, 23, 59, 59)
    pulses, got = gen_run(backend, SCALED, t, 4, 2)
    check_gen(pulses, got, SCALED, t, 4, 2)
    assert got[1].decoded.time == I.Time(25, 1, 0, 0, 0) and got[1].decoded.sbs == 0


@model_only("one frame per signal, 320,000 clocks; B004 runs on the RTL")
@pytest.mark.parametrize("z", [0, 1, 2, 3, 5, 6, 7])
def test_irig_gen_signals(backend, z):
    """Every coded expression of Table 4-1: year, CF and SBS present or
    index markers, as B00z says."""
    t = I.Time(26, 123, 12, 34, 56)
    pulses, got = gen_run(backend, SCALED, t, z, 1)
    check_gen(pulses, got, SCALED, t, z, 1)
    assert got[0].decoded.sbs == (45296 if I.has_sbs(z) else 0)


ROLLS = [
    # (what, start) - the frame after it is start.next() (3.6)
    ("minute", I.Time(26, 45, 10, 14, 59)),
    ("hour, SBS past 2^16", I.Time(26, 45, 18, 12, 15)),
    ("day 365 of a common year", I.Time(25, 365, 23, 59, 59)),
    ("day 365 of a leap year", I.Time(28, 365, 23, 59, 59)),
    ("2099 to 2000", I.Time(99, 365, 23, 59, 59)),
    ("day 99 to 100", I.Time(26, 99, 23, 59, 59)),
]


@model_only("two frames each, 480,000 clocks; the year end runs on the RTL")
@pytest.mark.parametrize("what,t", ROLLS)
def test_irig_gen_counting(backend, what, t):
    """The program's own clock across every carry of the BCD time of year,
    the SBS bit 16, and the leap-year rule for 2000..2099."""
    pulses, got = gen_run(backend, SCALED, t, 7, 2)
    check_gen(pulses, got, SCALED, t, 7, 2)


# ----------------------------------------------------------------- reader
def rx_setup(backend, board):
    bench = irig_bench(backend)
    src = bench.add(I.IrigSource("IN0"))
    pps = bench.add(I.EdgeRecorder("OUT0"))
    loom = start(backend, bench, [(program("irig_b_rx"), 0, rx_words(board))])
    return bench, loom, src, pps


def stream(t, count, z=0, faults=None, tail=10):
    """The last ``tail`` elements of the frame before ``t`` (P0 last), then
    ``count`` frames from ``t``, then the next Pr; ``faults`` maps (frame,
    index) to a replacement element."""
    before = I.frame_symbols(t.later(0), z)[-tail:] if tail else []
    out = list(before)
    for k in range(count):
        sym = I.frame_symbols(t.later(k), z)
        for (f, i), s in (faults or {}).items():
            if f == k:
                sym[i] = s
        out += sym
    return out + ["P"]


def rx_run(bench, loom, src, board, symbols, element=None):
    """Send ``symbols`` at ``element`` clocks each and read OUTQ as it goes;
    returns the reports as lists of four words."""
    element = element or board.element
    src.send(symbols, element, at=bench.cycle + 4 * board.element)
    end = src.free_at + 2 * board.element
    words = []
    while bench.cycle < end:
        bench.step(min(10 * board.element, end - bench.cycle))
        words += loom.pop_available(0)
    assert len(words) % 4 == 0, words
    return [words[i:i + 4] for i in range(0, len(words), 4)]


def pps_rises(pps):
    return pps.rises()


def check_pps(pps, src, prs):
    """A PPS rise 8 to 11 clocks after each Pr leading edge in ``prs``
    (cycles on the RX pad), and none elsewhere; each PPS pulse ends after
    Pr's does (it is Pr's, delayed)."""
    rises = pps_rises(pps)
    assert len(rises) == len(prs), (rises, prs)
    for r, p in zip(rises, prs):
        assert r - p in PPS_LATENCY, (r, p)
    return [r - p for r, p in zip(rises, prs)]


def pr_cycles(src, symbols, frames):
    """The RX-pad cycles of the leading edges of the Pr of frames
    ``frames`` (0 = the first whole frame of :func:`stream`)."""
    return [src.leads[10 + 100 * k] for k in frames]


def test_irig_rx_reads_frames(backend):
    """B000 into the reader: the tail of a frame (P0 last), two frames, the
    next Pr. The reader finds P0 Pr, reports both frames, and raises the
    PPS on the Pr after each, 8 to 11 clocks after the RX pad."""
    t = I.Time(0, 200, 7, 59, 59)
    bench, loom, src, pps = rx_setup(backend, SCALED)
    sym = stream(t, 2)
    got = rx_run(bench, loom, src, SCALED, sym)
    assert got == [report(t), report(t.next())]
    check_pps(pps, src, pr_cycles(src, sym, [1, 2]))
    assert loom.badop() == 0


@model_only("three frames each, 120,000 clocks; the nominal widths run on the RTL")
@pytest.mark.parametrize("element,widths", [
    (SCALED.element * 1.03, None),                  # a generator 3 % slow
    (SCALED.element * 0.97, None),                  # 3 % fast
    (SCALED.element, {"0": 0.26, "1": 0.44, "P": 0.74}),    # widths off by 0.6 ms
    (SCALED.element, {"0": 0.14, "1": 0.56, "P": 0.86}),
])
def test_irig_rx_tolerance(backend, element, widths):
    """B007 at an element 3 % long or short, and with every width 0.6 ms
    from nominal (inside the reader's midpoint thresholds): read clean."""
    t = I.Time(26, 31, 23, 59, 58)
    bench, loom, src, pps = rx_setup(backend, SCALED)
    sym = stream(t, 3, z=7)
    if widths:
        sym = [widths[s] for s in sym]
    got = rx_run(bench, loom, src, SCALED, sym, element)
    assert got == [report(t.later(k), 7) for k in range(3)]
    assert len(pps_rises(pps)) == 3


def test_irig_rx_corrupted_frame(backend):
    """B004: four frames, the second with P3 read as a 1 (a misplaced
    position identifier). That frame is not reported and no PPS follows it;
    the reader hunts, finds P0 Pr at its end, and reads the next one (whose
    own Pr has no PPS: it was only known as Pr after it ended)."""
    t = I.Time(26, 77, 1, 2, 3)
    bench, loom, src, pps = rx_setup(backend, SCALED)
    sym = stream(t, 4, z=4, faults={(1, 29): "1"})
    got = rx_run(bench, loom, src, SCALED, sym)
    assert got == [report(t, 4), report(t.later(2), 4), report(t.later(3), 4)]
    check_pps(pps, src, pr_cycles(src, sym, [1, 3, 4]))


@model_only("three frames each at 80 kHz, 240,000 clocks; the misplaced marker runs on the RTL")
@pytest.mark.parametrize("what,fault", [
    ("a 0.2 ms glitch for a 0", 0.02),
    ("a stuck line, 9.7 ms", 0.97),
    ("a missing pulse", 0),
    ("a marker for a bit", "P"),
])
def test_irig_rx_faults(backend, what, fault):
    """One fault in the middle of the first frame (index 45): that frame is
    lost; the reader hunts, finds the P0 Pr at its end, and reads the next
    two frames (no PPS at the first of them). At an 80 kHz board: the
    reader's shortest measurable pulse is 7 slots (28 clocks, from one NOW
    read to the next), which is under its 3-unit glitch limit only for a
    unit of 10 clocks or more (irig_b_rx.loom header)."""
    t = I.Time(26, 77, 1, 2, 3)
    bench, loom, src, pps = rx_setup(backend, SLOW)
    sym = stream(t, 3, z=0, faults={(0, 45): fault})
    got = rx_run(bench, loom, src, SLOW, sym)
    assert got == [report(t.later(1)), report(t.later(2))]
    check_pps(pps, src, pr_cycles(src, sym, [2, 3]))


# ---------------------------------------------------------------- together
def both_run(backend, board, t, z, frames):
    """The generator on thread 0 from ``t``, the board wire OUT0 -> IN0, the
    reader on thread 2 with its PPS on OUT1; run until ``frames`` frames
    and the next Pr are on the wire. Returns the generator's pulses, the
    wire's recorder, the PPS recorder and the reports."""
    bench = irig_bench(backend)
    tx = bench.add(I.EdgeRecorder("OUT0"))
    bench.add(I.Wire("OUT0", "IN0"))
    pps = bench.add(I.EdgeRecorder("OUT1"))
    loom = start(backend, bench, [
        (program("irig_b_rx", 2, pps="OUT1"), 2, rx_words(board)),
        (program("irig_b_gen"), 0, gen_words(board, t, z))])
    words = []
    want = frames * I.ELEMENTS + 1
    for _ in range(4 * frames + 8):
        if len(tx.rises()) >= want:
            break
        bench.step(25 * board.element)
        words += loom.pop_available(2)
    assert len(tx.rises()) >= want
    bench.step(5 * board.element)
    words += loom.pop_available(2)
    assert loom.badop() == 0
    return tx, pps, [words[i:i + 4] for i in range(0, len(words), 4)]


def check_together(board, t, z, frames, tx, pps, got):
    """The generator's pin is 200-16 to the clock; the reader, which starts
    hunting at the first frame (it has no P0 before it), reads every frame
    from the second on, and its PPS follows each Pr after the first read
    frame by 8 to 11 clocks of the RX pad (one more than the generator's
    pad: the wire)."""
    pulses = I.elements(tx.edges, board.element)
    fr = I.frames(pulses, board.element, z, first_is_pr=True)
    check_gen(pulses, fr, board, t, z, frames)
    assert got == [report(t.later(k), z) for k in range(1, frames)]
    # The Pr of frames 2 .. frames (the last is the Pr after the last frame),
    # on the RX pad: the wire adds a clock.
    prs = [fr[0].pr + 1 + k * board.frame for k in range(2, frames + 1)]
    return check_pps(pps, None, prs)


def test_irig_together(backend):
    """B007 from 2026 day 59 23:59:58 on a common year: thread 0 sends three
    frames and the next Pr, thread 2 reads frames two and three (day 60
    00:00:00 is 1 March), with PPS on the Pr after each."""
    t = I.Time(26, 59, 23, 59, 58)
    tx, pps, got = both_run(backend, SCALED, t, 7, 3)
    lat = check_together(SCALED, t, 7, 3, tx, pps, got)
    assert len(lat) == 2


@model_only("real seconds at a 200 kHz board, 800,000 clocks (about a minute "
            "of model time); the same counts run scaled on the RTL")
def test_irig_together_real_time(backend):
    """B004 in real time: 1 ms = 200 clocks, the reader's unit 40. Three
    frames and the next Pr, the reader's reports and PPS as above, and Pr
    to Pr exactly 200,000 clocks: one second of the board clock."""
    t = I.Time(26, 311, 17, 30, 0)
    tx, pps, got = both_run(backend, REAL, t, 4, 3)
    check_together(REAL, t, 4, 3, tx, pps, got)
    assert REAL.frame == 200_000


# -------------------------------------------------------------- the programs
def test_irig_programs_assemble_strict_with_no_diagnostic():
    """L2-DEADLINE: the generator's 22 pairs proved at a 40-clock tick with
    4 clocks of slack and nothing declared bounded; the reader has no WAITD
    and no pair. The generator takes 160 words (32 into the next quarter,
    header), the reader 118, and the two on threads 0 and 2 do not overlap."""
    gen, rx = program("irig_b_gen"), program("irig_b_rx")
    assert gen.diagnostics == [] and rx.diagnostics == []
    assert (len(gen.words), len(rx.words)) == (160, 118)
    g = gen.deadlines[0]
    assert g.period == 40 and not g.infeasible and not g.unbounded
    assert len(g.pairs) == 22 and g.worst_slack == 4 and g.declarations == []
    r = rx.deadlines[0]
    assert r.pairs == [] and not r.unbounded
    moved = program("irig_b_rx", 2, pps="OUT1")
    assert moved.diagnostics == []
    assert max(gen.words) < 0x100 <= min(moved.words) and max(moved.words) < 0x180


# --------------------------------------------------------------- the model
def test_irig_model_round_trip():
    """Every coded expression encodes and decodes clean; the known bits of
    Table 5-4 (seconds 59: 1 0 0 1 at 1..4, 1 0 1 at 6..8) and the SBS."""
    t = I.Time(24, 366, 23, 59, 59)
    for z in range(8):
        d = I.decode_symbols(I.frame_symbols(t, z, cf=0x2A5A5), z)
        assert d.errors == [] and (d.day, d.hour, d.minute, d.second) == (366, 23, 59, 59)
        assert d.sbs == (86399 if I.has_sbs(z) else 0)
        assert d.cf == (0x2A5A5 if I.has_cf(z) else 0)
    sym = I.frame_symbols(t, 0)
    assert "".join(sym[:10]) == "P10010101P"
    assert [i for i, s in enumerate(sym) if s == "P"] == list(I.MARKERS)


def test_irig_model_counting():
    """3.6: day 001 after 365, or 366 in a leap year; the year to 2099."""
    assert I.Time(25, 365, 23, 59, 59).next() == I.Time(26, 1, 0, 0, 0)
    assert I.Time(24, 365, 23, 59, 59).next() == I.Time(24, 366, 0, 0, 0)
    assert I.Time(99, 365, 23, 59, 59).next() == I.Time(0, 1, 0, 0, 0)
    assert I.Time(0, 1, 18, 12, 16).sbs == 1 << 16


def test_irig_model_checker_finds_faults():
    """The timing checker names a width off nominal and a leading edge off
    the grid; the decoder names a misplaced marker."""
    sym = I.frame_symbols(I.Time(0, 1, 0, 0, 0)) + ["P"]
    edges = []
    for k, s in enumerate(sym):
        lead = k * 400 + (3 if k == 50 else 0)
        width = int(I.WIDTH_FRACTION[s] * 400) + (1 if k == 20 else 0)
        edges += [(lead, 1), (lead + width, 0)]
    pulses = I.elements(edges, 400)
    assert [i for i, p in enumerate(pulses) if p.symbol is None] == [20]
    fr = I.frames(pulses, 400, first_is_pr=True)
    assert fr[0].grid_errors == [50]
    bad = list(sym[:100])
    bad[29] = "1"
    assert I.decode_symbols(bad).errors == ["marker 29"]
