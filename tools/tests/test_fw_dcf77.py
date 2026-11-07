"""L3 for the DCF77 time code programs, on the model and the RTL.

``firmware/dcf77_gen.loom`` (thread 0, DCF = OUT0) sends the minutes the
host queues as a receiver module's logic output; ``firmware/dcf77_rx.loom``
(RX = IN0) decodes that output. The references are
``tools.protomodels.dcf77``: the encoder makes the minutes, the timing
checker holds the generator's pin to the second grid and the pulse widths,
the decoder reads the pin back, ``DcfSource`` feeds the receiver with
timing variations and faults, and ``Receiver`` stands between the two
programs' pins when they run together (generator on thread 0, receiver on
thread 1).

Expected values come from PTB's description of the code (the model's
docstring) and from the program headers' stated windows, never from what
the model or the RTL did:

* generator: every pulse starts on the 1000 ms grid and lasts 100 ms (a 0)
  or 200 ms (a 1) exactly (its edges are deadline-latched writes on exact
  ticks), second 59 has no pulse, P1 P2 P3 even, the minute decoded;
* receiver: a 0 is 40..150 ms, a 1 150..255 ms, the next pulse 900..1100 ms
  after the one before (1900..2100 after a minute's last), spikes shorter
  than 10 ms ignored, faults reported as the header says.

Time. Both programs count 1 ms ticks. In real time that is TICK_INT 50000
at 50 MHz, and a minute is 3 G clocks: no simulator here runs that. So the
programs run on scaled time: TICK_INT 2, a "millisecond" in 2 clocks, a
minute in 120 k clocks (about 19 s of RTL). The waveform in ticks is the
same, and the programs never see the clock (it is real time on a 2 kHz
clock). Real seconds at a realistic resolution run on the model only, on
an 8 kHz board: TICK_INT 8, a tick a real millisecond and two slots, a
minute 480 k clocks (the model runs about 14 k clocks a second; 50 MHz
would be 3 G clocks a minute).

Every body here is named ``test_dcf77_*``: test/test_fw.py collects the
bodies of all modules into one namespace.
"""

import pytest

from tools.loomasm import assemble_file, assemble_text
from tools.loomisa import REPO, load
from tools.protomodels import dcf77 as D
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
IMEM = 512


class Board:
    """A clock and the tick (TICK_INT) that makes one tick a millisecond of
    the signal: ``ms`` clocks a millisecond."""

    def __init__(self, name, clock_hz, tick):
        self.name, self.clock_hz, self.tick = name, float(clock_hz), tick
        self.ms = tick

    def cycles(self, ms):
        return int(round(ms * self.ms))

    def __repr__(self):
        return self.name


SCALED = Board("scaled", 2_000, 2)          # 1 "ms" = 2 clocks (any clock)
REAL = Board("8kHz", 8_000, 8)              # real seconds, on a slow board


def program(name, thread=0):
    """A program from ``firmware/``, moved to ``thread`` if asked."""
    path = FIRMWARE / (name + ".loom")
    if thread == 0:
        return assemble_file(path, isa=ISA, strict=True)
    text = path.read_text(encoding="utf-8")
    assert text.count(".thread 0") == 1
    text = text.replace(".thread 0", ".thread %d" % thread)
    return assemble_text(text, name + ".loom", isa=ISA, imem_words=IMEM, strict=True)


def dcf_bench(backend):
    """The FIFOs, the deadline latch and the 512-word memory."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "SETPD"), imem_words=IMEM)
    return backend.bench()


def start(backend, bench, loads):
    """Load ``[(program, thread, TICK_INT)]`` and run the threads."""
    loom = backend.loom(bench, isa=ISA)
    words = {}
    for prog, _, _ in loads:
        words.update(prog.words)
    loom.load(words)
    for _, thread, tint in loads:
        loom.write_csr(thread, "TICK_INT", tint)
        loom.write_csr(thread, "TICK_FRAC", 0)
    loom.run([thread for _, thread, _ in loads])
    return loom


def host_words(bits, leap=False):
    """The four INQ words of a minute (dcf77_gen.loom's header): seconds
    0..15, 16..31, 32..47, then 48..58 with L at bit 11."""
    v = sum(b << i for i, b in enumerate(bits))
    return [v & 0xFFFF, (v >> 16) & 0xFFFF, (v >> 32) & 0xFFFF,
            ((v >> 48) & 0x7FF) | (0x800 if leap else 0)]


def report_minute(bits, leap=False):
    """What the receiver pushes for a minute (dcf77_rx.loom's header)."""
    w = host_words(bits)
    return ("minute", (w[3] & 0x7FF) | 0x4000 | (0x800 if leap else 0), w[0], w[1], w[2])


def report_error(fault, second):
    return ("error", 0xC000 | (fault << 8) | second)


SHORT, LONG, EARLY, MISSING, PARITY, MS, MARKER = 1, 2, 3, 4, 5, 6, 7


def reports(loom, thread=0):
    """Everything in OUTQ[thread], by kind (bits 15:14 of the first word)."""
    out, words = [], loom.pop_available(thread)
    while words:
        w = words.pop(0)
        if w >> 14 == 1:
            while len(words) < 3:
                words += loom.pop(thread, 3 - len(words))
            out.append(("minute", w, words.pop(0), words.pop(0), words.pop(0)))
        elif w >> 14 == 3:
            out.append(("error", w))
        else:
            out.append(("bad", w))
        if not words:
            words = loom.pop_available(thread)
    return out


# The minutes. Each code carries the time of the minute that follows it.
T1 = D.DcfTime(minute=5, hour=13, day=7, weekday=6, month=11, year=26)
T2 = D.DcfTime(minute=6, hour=13, day=7, weekday=6, month=11, year=26)
T3 = D.DcfTime(minute=59, hour=23, day=31, weekday=7, month=12, year=99,
               cest=True, a1=True, call=True, weather=0x2A55)
# The leap second at the end of 2016: the minute before 01:00:00 CET on
# 1 January 2017 (a Sunday) had 61 seconds and carries the time 01:00.
TLEAP = D.DcfTime(minute=0, hour=1, day=1, weekday=7, month=1, year=17, a2=True)


def wrong_parity(bits):
    """``bits`` with all three parity bits inverted: the generator must
    send its own."""
    return [b ^ (s in (D.P1, D.P2, D.P3)) for s, b in enumerate(bits)]


# ------------------------------------------------------------- generator
def gen_setup(backend, board):
    bench = dcf_bench(backend)
    rec = bench.add(D.EdgeRecorder("OUT0"))
    loom = start(backend, bench, [(program("dcf77_gen"), 0, board.tick)])
    return bench, loom, rec


def run_ms(bench, board, ms):
    run_to(bench, board, bench.cycle, ms)


def run_to(bench, board, t0, ms):
    """Run to ``ms`` milliseconds after cycle ``t0``."""
    end = t0 + board.cycles(ms)
    if end > bench.cycle:
        bench.run_until(lambda: bench.cycle >= end, end - bench.cycle + 1, every=256)


def gen_run(bench, loom, board, minutes):
    """Queue ``[(bits, leap)]``. The generator pops the first at once and
    sends second 0 1.8 s later; it pops each next minute 200 ms into the
    marker before it, so minute k (k >= 1) is queued 3 s after minute k-1
    started (INQ is empty again then). Runs to 1 s past the last marker."""
    t0, t = bench.cycle, 0.0
    for k, (bits, leap) in enumerate(minutes):
        run_to(bench, board, t0, t)
        loom.push(0, host_words(wrong_parity(bits), leap))
        t = 3_000.0 if k == 0 else t + 60_000.0 + 1_000.0 * minutes[k - 1][1]
    end = 1_800.0 + sum(60_000.0 + 1_000.0 * leap for _, leap in minutes) + 1_000.0
    run_to(bench, board, t0, end)


def check_gen(rec, board, minutes):
    """The pin against the code, exactly, and decoded back."""
    bits = [D.encode(t) for t, _ in minutes]
    leaps = [leap for _, leap in minutes]
    clock = board.ms * 1000.0
    assert D.check_timing(rec.edges, clock, bits, leaps) == []
    got = D.decode_pulses(LEAD_IN + D.edges_to_pulses(rec.edges, clock)
                          + [(D.IDLE, 2000.0)])
    assert [(m.kind, m.leap, m.time) for m in got] == \
        [("minute", leap, t) for t, leap in minutes]


@model_only("two minutes, 0.25 M clocks, 40 s of RTL; the together scenario "
            "runs the generator's two minutes on the RTL")
def test_dcf77_gen_two_minutes(backend):
    """Two minutes back to back, their parity bits sent wrong by the host:
    every edge on its exact tick, P1 P2 P3 corrected, the first pulse at
    least 1.8 s after the words came (a marker before second 0), no gap in
    the grid at the minute boundary."""
    bench, loom, rec = gen_setup(backend, SCALED)
    gen_run(bench, loom, SCALED, [(D.encode(T1), False), (D.encode(T2), False)])
    check_gen(rec, SCALED, [(T1, False), (T2, False)])
    assert rec.edges[0][0] >= SCALED.cycles(1_800)
    assert loom.badop() == 0


@model_only("a 61-second minute, 0.13 M clocks")
def test_dcf77_gen_leap_second(backend):
    """With L, second 59 carries a 0 pulse and 60 is the marker; then the
    next minute follows on the grid."""
    bench, loom, rec = gen_setup(backend, SCALED)
    gen_run(bench, loom, SCALED, [(D.encode(TLEAP), True), (D.encode(T1), False)])
    check_gen(rec, SCALED, [(TLEAP, True), (T1, False)])


@model_only("an outage and a restart, 0.3 M clocks")
def test_dcf77_gen_outage_and_restart(backend):
    """No next minute by the marker: the line stays idle (no pulse at all)
    until the words come, then a second with no pulse and second 0."""
    bench, loom, rec = gen_setup(backend, SCALED)
    gen_run(bench, loom, SCALED, [(D.encode(T1), False)])
    run_ms(bench, SCALED, 5_000)
    first = len(rec.edges)
    assert first == 2 * 59
    before = bench.cycle
    loom.push(0, host_words(D.encode(T2)))
    after = bench.cycle                         # the words land in between
    run_ms(bench, SCALED, 63_000)
    rise = rec.edges[first][0]
    assert SCALED.cycles(1_800) <= rise - before and rise - after <= SCALED.cycles(1_802)
    clock = SCALED.ms * 1000.0
    assert D.check_timing(rec.edges[first:], clock, [D.encode(T2)]) == []


@model_only("real seconds: 0.5 M clocks at 8 kHz")
def test_dcf77_gen_real_seconds(backend):
    """The same program at a 1 ms tick in real time (8 kHz board, TICK_INT
    8): one minute with flags and weather bits set, exact."""
    bench, loom, rec = gen_setup(backend, REAL)
    gen_run(bench, loom, REAL, [(D.encode(T3), False)])
    check_gen(rec, REAL, [(T3, False)])


# -------------------------------------------------------------- receiver
LEAD_IN = [(D.PULSE, 100.0), (D.IDLE, 1900.0)]   # second 58's pulse and the marker


def rx_setup(backend, board):
    bench = dcf_bench(backend)
    src = bench.add(D.DcfSource("IN0", board.ms))
    loom = start(backend, bench, [(program("dcf77_rx"), 0, board.tick)])
    return bench, loom, src


def rx_send(bench, loom, src, board, signal, tail_ms=1_500.0):
    """:func:`rx_queue` of one signal."""
    return rx_queue(bench, loom, src, board, [(signal, tail_ms)])


def rx_queue(bench, loom, src, board, items, poll_ms=5_000.0):
    """Queue ``[(signal, tail_ms)]``, each signal after a lead-in, the first
    50 ms from now and each next one ``tail_ms`` (+ 50 ms) after the end of
    the one before, at absolute cycles (the host's reads take bench cycles
    too); run to ``tail_ms`` past the last, reading OUTQ every ``poll_ms``
    so it never fills."""
    t = bench.cycle + board.cycles(50)
    for signal, tail_ms in items:
        src.send(LEAD_IN + list(signal), at=t)
        end = src.free_at + board.cycles(tail_ms)
        t = end + board.cycles(50)
    out = []
    while bench.cycle < end:
        step = min(end, bench.cycle + board.cycles(poll_ms))
        bench.run_until(lambda: bench.cycle >= step, step - bench.cycle + 1, every=256)
        out += reports(loom)
    return out


def minutes_signal(*minutes):
    """``(bits, leap)`` minutes back to back as pulses."""
    out = []
    for bits, leap in minutes:
        out += D.minute_pulses(bits, leap)
    return out


B1, B2 = D.encode(T1), D.encode(T2)


def test_dcf77_rx_minute(backend):
    """A lead-in pulse and a marker, then one minute at nominal timing:
    reported at its marker, its bits as sent."""
    bench, loom, src = rx_setup(backend, SCALED)
    got = rx_send(bench, loom, src, SCALED, D.minute_pulses(B1), tail_ms=-700.0)
    assert got == [report_minute(B1)]
    assert loom.badop() == 0


def test_dcf77_rx_parity_error(backend):
    """A minute bit inverted (second 23): P1 fails at second 28, one error
    word, nothing else. (The model's decoder rejects it too.)"""
    bad = D.flip(B1, 23)
    assert D.decode_bits(bad) == (None, "P1")
    bench, loom, src = rx_setup(backend, SCALED)
    sig = D.minute_pulses(bad)[:2 * 30]
    assert rx_send(bench, loom, src, SCALED, sig, tail_ms=0.0) == \
        [report_error(PARITY, D.P1)]


def cut(signal, second):
    """``signal`` up to the end of second ``second``'s pulse."""
    return list(signal[:2 * second + 1])


def early(signal, second, ms):
    """Second ``second``'s pulse starting ``ms`` early (the second before it
    shorter); the rest follows it."""
    out = list(signal)
    k = 2 * second - 1
    out[k] = (D.IDLE, out[k][1] - ms)
    return out


S1 = D.minute_pulses(B1)
LEAP_BITS = D.encode(TLEAP)
# Each faulty signal stops at the end of the pulse that shows the fault
# (for a missing pulse or a marker check, at the end of second 58's or
# 59's), so the receiver, hunting again, sees the next lead-in pulse within
# 1.5 s of the last one and does not take it for second 0.
RX_ERRORS = [
    # (what, signal, fault, second)
    ("0 pulse 35 ms", cut(D.with_width(S1, 3, 35.0), 3), SHORT, 3),
    ("1 pulse 265 ms", cut(D.with_width(S1, 20, 265.0), 20), LONG, 20),
    ("pulse missing", cut(D.drop_pulse(S1, 10), 9), MISSING, 10),
    ("pulse 150 ms early", cut(early(S1, 5, 150.0), 5), EARLY, 5),
    ("P2: an hour bit", cut(D.minute_pulses(D.flip(B1, 30)), D.P2), PARITY, D.P2),
    ("P3: a day bit", cut(D.minute_pulses(D.flip(B1, 40)), D.P3), PARITY, D.P3),
    ("M is 1", cut(D.minute_pulses(D.flip(B1, 0)), 58), MS, 59),
    ("S is 0", cut(D.minute_pulses(D.flip(B1, 20)), 58), MS, 59),
    ("second 59 pulse, no A2", cut(D.minute_pulses(B1, leap=True), 59), MARKER, 59),
    ("leap pulse a 1",
     cut(D.with_width(D.minute_pulses(LEAP_BITS, True), 59, 200.0), 59), MARKER, 59),
]
ERROR_TAIL_MS = 1_100.0


@model_only("ten faulty minutes, 0.8 M clocks, two minutes of RTL; the "
            "parity error runs there")
def test_dcf77_rx_errors_and_recovery(backend):
    """Each fault gives one error word naming it and its second, and the
    receiver hunts for the next marker; a good minute after them all is
    reported. The model's decoder rejects every faulty signal too."""
    for what, sig, _, _ in RX_ERRORS:
        kinds = [m.kind for m in D.decode_pulses(LEAD_IN + sig + [(D.IDLE, 3000.0)])]
        assert kinds[:1] == ["error"] and "minute" not in kinds, what
    bench, loom, src = rx_setup(backend, SCALED)
    items = [(sig, ERROR_TAIL_MS) for _, sig, _, _ in RX_ERRORS] + [(S1, -700.0)]
    got = rx_queue(bench, loom, src, SCALED, items)
    assert got == [report_error(f, s) for _, _, f, s in RX_ERRORS] + [report_minute(B1)]


@model_only("two minutes with a leap second, 0.25 M clocks")
def test_dcf77_rx_leap_second(backend):
    """A2 set and a 0 pulse in second 59: a 61-second minute, reported with
    L; the next minute after it on the grid."""
    bench, loom, src = rx_setup(backend, SCALED)
    sig = minutes_signal((LEAP_BITS, True), (B1, False))
    got = rx_send(bench, loom, src, SCALED, sig, tail_ms=-700.0)
    assert got == [report_minute(LEAP_BITS, leap=True), report_minute(B1)]


def glitched(signal, glitches):
    """``glitches`` = [(second, offset ms from its start, width ms)]."""
    for second, offset, width in glitches:
        signal = D.glitch(signal, second * D.SECOND_MS + offset, width)
    return signal


@model_only("two minutes, 0.25 M clocks; the nominal minute runs on the RTL")
def test_dcf77_rx_glitches(backend):
    """Spikes of 5 and 8 ms (shorter than the 10 ms filter) inside a 1
    pulse, a 0 pulse, the idle part of a second and the marker change
    nothing. A 20 ms drop inside second 21's 1 pulse is an edge: the pulse
    reads as a 0 ending at 100 ms and the line coming back at 120 ms is a
    pulse 880 ms early, an error in second 22 (that minute follows the good
    one on its grid)."""
    good = glitched(S1, [(20, 120, 5), (3, 50, 8), (10, 500, 5), (30, 999 - 8, 8),
                         (58, 1_400, 5), (21, 60, 8)])
    bad = cut(glitched(S1, [(21, 100, 20)]), 22)
    assert D.decode_pulses(LEAD_IN + good)[0].kind == "error"      # unfiltered
    assert D.decode_pulses(D.deglitch(LEAD_IN + good, 10.0))[0].time == T1
    assert D.decode_pulses(D.deglitch(LEAD_IN + bad, 10.0))[0].reason == "early pulse"
    bench, loom, src = rx_setup(backend, SCALED)
    got = rx_queue(bench, loom, src, SCALED, [(good + bad, ERROR_TAIL_MS), (S1, -700.0)])
    assert got == [report_minute(B1), report_error(EARLY, 22), report_minute(B1)]


def built(bits, widths, periods):
    """A minute with each 0 / 1 pulse ``widths[bit]`` ms long and second s
    starting ``periods(s)`` ms after second s-1 (the marker gap included:
    ``periods(59)`` is from second 58 to the next minute's second 0)."""
    out = []
    for s, b in enumerate(bits):
        out += [(D.PULSE, widths[b]), (D.IDLE, periods(s + 1 if s < 58 else 59) - widths[b])]
    return out


# How close to a window's edge a test goes. An edge is timed late by one to
# two slots (WAITP sees it, CSRR reads NOW a slot later) and compared with a
# deadline on the slot grid: up to 6 "ms" on the scaled board, where a
# slot is 2 ticks, 1.5 ms at 8 kHz. A pulse that starts just after
# a gap window's first edge (900 or 1900 ms) is also seen late by up to 7
# slots, the code that sets the next deadline: 28 clocks, 14 "ms" on the
# scaled board, 3.5 ms at 8 kHz, 0.6 us at 50 MHz (FW_README.md).
WIDTH_MARGIN = {SCALED: 8.0, REAL: 2.0}
PERIOD_MARGIN = {SCALED: 20.0, REAL: 6.0}


def edges_minutes(board):
    """Two minutes at the window edges (dcf77_rx.loom's header): every
    pulse at the short then the long edge of its window, every period
    alternately at the short and the long edge of its window, the marker
    gap at the short then the long edge."""
    m, p = WIDTH_MARGIN[board], PERIOD_MARGIN[board]
    short = built(B1, {0: 40 + m, 1: 150 + m},
                  lambda s: (1_900 + p) if s == 59 else (900 + p if s % 2 else 1_100 - p))
    long_ = built(B1, {0: 150 - m, 1: 255 - m},
                  lambda s: (2_100 - p) if s == 59 else (1_100 - p if s % 2 else 900 + p))
    return short, long_


@model_only("two minutes, 0.25 M clocks")
def test_dcf77_rx_window_edges(backend):
    """Pulse widths and periods at both edges of the receiver's windows,
    less the measurement margin: both minutes read."""
    bench, loom, src = rx_setup(backend, SCALED)
    a, b = edges_minutes(SCALED)
    assert rx_send(bench, loom, src, SCALED, a + b, tail_ms=-700.0) == \
        [report_minute(B1)] * 2


@model_only("real seconds: one minute, 0.5 M clocks at 8 kHz")
def test_dcf77_rx_real_seconds(backend):
    """At a real 1 ms tick (8 kHz board): one minute at the short edges of
    its windows, 2 ms inside for widths, 6 ms for periods (a slot is half
    a tick here)."""
    bench, loom, src = rx_setup(backend, REAL)
    a, _ = edges_minutes(REAL)
    assert rx_send(bench, loom, src, REAL, a, tail_ms=-700.0) == [report_minute(B1)]


# ---------------------------------------------------------------- together
def both_setup(backend, board, **receiver):
    """The generator on thread 0, a receiver module from OUT0 to IN0, the
    receiver on thread 1 (its source with `.thread 1`, at 0x080)."""
    bench = dcf_bench(backend)
    rec = bench.add(D.EdgeRecorder("OUT0"))
    bench.add(D.Receiver("OUT0", "IN0", board.ms, **receiver))
    loom = start(backend, bench, [(program("dcf77_gen"), 0, board.tick),
                                  (program("dcf77_rx", 1), 1, board.tick)])
    return bench, loom, rec


def together(backend, board, minutes, **receiver):
    """Run ``minutes`` through both programs; the generator's pin judged as
    in its own tests. Returns the receiver's reports. The first minute is
    never reported: no pulse comes before its marker, and the receiver
    needs one to know the marker for what it is (dcf77_rx.loom)."""
    bench, loom, rec = both_setup(backend, board, **receiver)
    gen_run(bench, loom, board, [(D.encode(t), leap) for t, leap in minutes])
    check_gen(rec, board, minutes)
    assert loom.badop() == 0
    return reports(loom, 1)


def test_dcf77_together_two_minutes(backend):
    """Two minutes from thread 0 to thread 1 on the scaled board: the
    generator exact on the pin, the second minute reported by the
    receiver. The host queues no third minute, so the generator falls
    silent after the second marker and the receiver reports second 0
    missing."""
    got = together(backend, SCALED, [(T1, False), (T2, False)])
    assert got == [report_minute(B2), report_error(MISSING, 0)]


@model_only("three minutes per receiver module, a minute of RTL each")
@pytest.mark.parametrize("delay,stretch", [(40.0, 40.0), (25.0, -40.0)])
def test_dcf77_together_receiver_module(backend, delay, stretch):
    """A receiver module that delays the signal and stretches or shortens
    every pulse by 40 ms (no source gives a module's figures; 40 ms is a
    guess at a poor one): both later minutes read, flags and the leap
    second included."""
    got = together(backend, SCALED, [(T1, False), (TLEAP, True), (T3, False)],
                   delay_ms=delay, stretch_ms=stretch)
    assert got == [report_minute(LEAP_BITS, leap=True), report_minute(D.encode(T3)),
                   report_error(MISSING, 0)]


# -------------------------------------------------------------- the programs
def test_dcf77_programs_assemble_strict_with_no_diagnostic():
    """L2-DEADLINE: the generator's 19 pairs proved at a 2-clock tick with
    180 clocks of slack, its four INQ tests' POPs declared bounded; the
    receiver has no WAITD and no pair. Each fills at most a quarter of the
    512-word memory (the receiver exactly), and the receiver moved to
    thread 1 stays in that quarter, so both run together."""
    gen, rx = program("dcf77_gen"), program("dcf77_rx")
    assert gen.diagnostics == [] and rx.diagnostics == []
    assert (len(gen.words), len(rx.words)) == (72, 128)
    t = gen.deadlines[0]
    assert t.period == 2 and not t.infeasible and not t.unbounded
    assert len(t.pairs) == 19 and t.worst_slack == 180
    assert [d.name for d in t.declarations] == ["POP"] * 4
    assert all("only this thread pops INQ" in d.reason for d in t.declarations)
    r = rx.deadlines[0]
    assert r.period == 2 and r.pairs == [] and not r.unbounded
    assert max(gen.words) < 0x80 and max(rx.words) < 0x80
    moved = program("dcf77_rx", 1)
    assert moved.diagnostics == []
    assert min(moved.words) == 0x80 and max(moved.words) < 0x100


# --------------------------------------------------------------- the model
def test_dcf77_model_encoder_and_decoder():
    """Round trips; the field layout; July coded as PTB-Mitteilungen 114
    (2004) section 4.2 describes Bild 6 (units bits 1, 2, 4 set, bit 8 and
    the tens bit clear); every parity even; the faults."""
    for t in (T1, T2, T3, TLEAP):
        bits = D.encode(t)
        assert len(bits) == 59 and bits[0] == 0 and bits[D.S_BIT] == 1
        for p, span in D.PARITY_SPANS.items():
            assert (sum(bits[i] for i in span) + bits[p]) % 2 == 0
        assert D.decode_bits(bits) == (t, "")
    july = D.encode(D.DcfTime(month=7))
    assert july[45:50] == [1, 1, 1, 0, 0]
    assert B1[21:28] == [1, 0, 1, 0, 0, 0, 0]            # minute 05
    assert B1[29:35] == [1, 1, 0, 0, 1, 0]               # hour 13
    assert B1[17:19] == [0, 1] and D.encode(T3)[17:19] == [1, 0]   # CET, CEST
    assert D.decode_bits(D.flip(B1, 23)) == (None, "P1")
    assert D.decode_bits(D.flip(B1, 30)) == (None, "P2")
    assert D.decode_bits(D.flip(B1, 50)) == (None, "P3")
    assert D.decode_bits(D.flip(B1, 0))[1] == "M"
    with pytest.raises(ValueError):
        D.encode(D.DcfTime(minute=80))
    p = D.minute_pulses(B1)
    assert len(p) == 2 * 59 and sum(ms for _, ms in p) == 60_000
    assert sum(ms for _, ms in D.minute_pulses(B1, leap=True)) == 61_000
    assert [w for lv, w in p if lv == D.PULSE] == [100.0 + 100 * b for b in B1]


def test_dcf77_model_timing_checker_and_filter():
    """check_timing passes exact edges and flags a pulse 1 ms long or a
    start 1 ms off; deglitch removes spikes and keeps pulses."""
    clock = 2000.0
    edges, t = [], 1000
    for b in B1:
        edges += [(t, 1), (t + 200 * (1 + b), 0)]
        t += 2000
    assert D.check_timing(edges, clock, [B1]) == []
    bad = list(edges)
    bad[1] = (bad[1][0] + 2, 0)
    assert D.check_timing(bad, clock, [B1])[0].startswith("pulse 0 lasts")
    bad = list(edges)
    bad[4] = (bad[4][0] + 2, 1)
    assert any(m.startswith("pulse 2 starts") for m in D.check_timing(bad, clock, [B1]))
    assert D.check_timing(edges[:-2], clock, [B1]) != []
    spiky = D.glitch(D.minute_pulses(B1), 50.0, 5.0)
    assert len(spiky) == len(D.minute_pulses(B1)) + 2
    assert D.deglitch(spiky, 10.0) == D.minute_pulses(B1)
