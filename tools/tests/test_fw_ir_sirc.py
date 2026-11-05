"""L3 for the Sony SIRC infrared programs, on the model and the RTL.

``firmware/ir_sirc_tx.loom`` (thread 0, LED = OUT0) makes the 40 kHz carrier
and the SIRC envelope itself; ``firmware/ir_sirc_rx.loom`` (RX = IN0)
decodes a receiver module's output. The references are
``tools.protomodels.ir_sirc``: the carrier analyser, the timing checker and
the decoder hold the transmitter to the SIRC timing, ``IrSource`` feeds the
receiver demodulated frames with timing variations and faults, and
``TsopModel`` sits between the two programs' pins when they run together
(transmitter on thread 0, receiver on thread 1).

Expected values come from the SIRC descriptions and the TSOP datasheet (the
model's docstring) and from the program headers' stated windows, never from
what the model or the RTL did:

* transmitter: the carrier within +-1 % of 40 kHz over every whole burst, a
  1/3 duty, 24 carrier cycles a unit in every mark, every mark and space and
  every mark's start within one carrier period (25 us) of nominal, the frame
  decoded, repeats 45 ms apart (+-0.5 %);
* receiver: the windows of its header (44 .. 84 ticks of 37.5 us for the
  header mark, 8 .. 24 for a T mark or space, 24 .. 40 for a 2T mark, a
  frame ending at a 64-tick space), glitches shorter than 75 us ignored.

Two boards. Both programs take their tick from the host: at 50 MHz the
transmitter's tick is 416 + 171/256 clocks (a third of a carrier period) and
the receiver's 1875 (T/16). A frame and its repeat are then 4.5 M clocks,
about 12 minutes of RTL simulation, so the RTL runs the same programs on a
board clocked at 1.92 MHz, where the ticks are 16 and 72 clocks: the same
waveform in microseconds, 26 times fewer clocks. The 50 MHz bodies, and the
longer 1.92 MHz ones, are ``model_only``.

Every body here is named ``test_sirc_*``: test/test_fw.py collects the
bodies of all modules into one namespace.
"""

import pytest

from tools.loomasm import assemble_file, assemble_text
from tools.loomisa import REPO, load
from tools.protomodels import ir_sirc as S
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
IMEM = 512


class Board:
    """A clock and the tick values the program headers give for it."""

    def __init__(self, name, clock_hz, tx_tick, rx_tick):
        self.name, self.clock_hz = name, float(clock_hz)
        self.tx_tick, self.rx_tick = tx_tick, rx_tick

    def cycles(self, us):
        return int(round(us * self.clock_hz / 1e6))

    def __repr__(self):
        return self.name


B192 = Board("1920kHz", 1_920_000, (16, 0), (72, 0))
B50 = Board("50MHz", 50_000_000, (416, 171), (1875, 0))
CODE = {12: 0, 15: 1, 20: 2}                # the length field (program headers)


def program(name, thread=0):
    """A program from ``firmware/``, moved to ``thread`` if asked."""
    path = FIRMWARE / (name + ".loom")
    if thread == 0:
        return assemble_file(path, isa=ISA, strict=True)
    text = path.read_text(encoding="utf-8")
    assert text.count(".thread 0") == 1
    text = text.replace(".thread 0", ".thread %d" % thread)
    return assemble_text(text, name + ".loom", isa=ISA, imem_words=IMEM, strict=True)


def ir_bench(backend):
    """The FIFOs, the deadline latch and the 512-word memory; the RTL has
    them all and ``RtlBench`` takes neither keyword."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "SETPD"), imem_words=IMEM)
    return backend.bench()


def start(backend, bench, loads):
    """Load ``[(program, thread, (TICK_INT, TICK_FRAC))]`` and run the threads."""
    loom = backend.loom(bench, isa=ISA)
    words = {}
    for prog, _, _ in loads:
        words.update(prog.words)
    loom.load(words)
    for _, thread, (tint, tfrac) in loads:
        loom.write_csr(thread, "TICK_INT", tint)
        loom.write_csr(thread, "TICK_FRAC", tfrac)
    loom.run([thread for _, thread, _ in loads])
    return loom


def tx_setup(backend, board):
    bench = ir_bench(backend)
    rec = bench.add(S.EdgeRecorder("OUT0"))
    loom = start(backend, bench, [(program("ir_sirc_tx"), 0, board.tx_tick)])
    return bench, loom, rec


def rx_setup(backend, board, gap_us=10_000.0):
    bench = ir_bench(backend)
    src = bench.add(S.IrSource("IN0", board.clock_hz, gap_us=gap_us))
    loom = start(backend, bench, [(program("ir_sirc_rx"), 0, board.rx_tick)])
    return bench, loom, src


# ------------------------------------------------------------ transmitter
CARRIER_TOL = 0.01
PERIOD_TOL = 0.005
GROUP_GAP_US = 3_000.0              # longer than any space inside a frame (600 us)


def tx_words(length, command, address, extended=0, repeats=0):
    """The two INQ words of a command (program header)."""
    return [(command & 0x7F) | (CODE[length] << 8) | ((repeats & 0xF) << 12),
            (address & 0xFF) | ((extended & 0xFF) << 8)]


def edges_of(frames):
    """Pin edges of a list of ``(length, command, address, extended)``."""
    return sum(2 * sum(S.burst_cycles(*f)) for f in frames)


def tx_run(bench, rec, board, edges, budget_ms):
    """Run until ``edges`` edges have gone out and the line has been low 3 ms."""
    quiet = board.cycles(3000)
    done = lambda: (len(rec.edges) >= edges
                    and bench.cycle - rec.edges[-1][0] > quiet)
    assert bench.run_until(done, board.cycles(budget_ms * 1000), every=256), (
        "%d of %d edges after %d ms" % (len(rec.edges), edges, budget_ms))


def messages(ca):
    """The bursts grouped into frames: index lists into ``ca.bursts``."""
    groups = [[0]]
    for k in range(1, len(ca.bursts)):
        if ca.us(ca.bursts[k].start - ca.bursts[k - 1].end) > GROUP_GAP_US:
            groups.append([])
        groups[-1].append(k)
    return groups


def envelope(ca, group):
    """The pulse list of one group of bursts."""
    out = []
    for a, b in zip(group, group[1:] + [None]):
        out.append((S.MARK, ca.us(ca.bursts[a].end - ca.bursts[a].start)))
        if b is not None:
            out.append((S.SPACE, ca.us(ca.bursts[b].start - ca.bursts[a].end)))
    return out


def check_tx(rec, board, frames):
    """The carrier and the envelope of every frame against the
    specification; ``frames`` is ``[(length, command, address, extended)]``."""
    ca = S.CarrierAnalysis(rec.edges, board.clock_hz)
    for b in ca.bursts:
        assert abs(ca.frequency(b) / S.CARRIER_HZ - 1) <= CARRIER_TOL, ca.frequency(b)
        assert abs(ca.duty(b) - 1 / 3.0) <= 0.01, ca.duty(b)
    groups = messages(ca)
    assert len(groups) == len(frames), [len(g) for g in groups]
    for g, f in zip(groups, frames):
        assert [ca.bursts[k].cycles for k in g] == S.burst_cycles(*f)
        got = envelope(ca, g)
        bad = S.check_timing(got, S.frame_pulses(*f))
        assert not bad, "pulses off nominal (index, us, nominal): %r" % bad[:6]
        m = S.decode_pulses(got)
        assert [(x.kind, x.length, x.command, x.address, x.extended) for x in m] == \
            [("frame",) + tuple(f)]
    return ca, groups


def starts_us(ca, groups):
    return [ca.us(ca.bursts[g[0]].start) for g in groups]


FRAMES = {12: (12, 0x15, 0x01, 0), 15: (15, 0x2A, 0xA7, 0), 20: (20, 0x7F, 0x1A, 0xC9)}


@pytest.mark.parametrize("length", [12, 15, 20])
def test_sirc_tx_frame(backend, length):
    """One frame of each length at the 1.92 MHz board: carrier, duty, the
    cycle count of every mark, every mark and space, the decoded fields.
    (The 20-bit frame's command is all ones: its longest case.)"""
    bench, loom, rec = tx_setup(backend, B192)
    f = FRAMES[length]
    loom.push(0, tx_words(*f))
    tx_run(bench, rec, B192, edges_of([f]), 60)
    ca, _ = check_tx(rec, B192, [f])
    assert {h for b in ca.bursts for h in b.highs} == {16}
    assert {p for b in ca.bursts for p in b.periods} == {48}
    assert loom.badop() == 0


@model_only("three frames 45 ms apart, 0.2 M clocks, 35 s of RTL; the "
            "together scenario sends a repeat there")
def test_sirc_tx_repeats_on_the_45ms_grid(backend):
    """A 15-bit command with two repeats: three identical frames, each
    starting 45 ms after the one before (+-0.5 %)."""
    bench, loom, rec = tx_setup(backend, B192)
    f = FRAMES[15]
    loom.push(0, tx_words(*f, repeats=2))
    tx_run(bench, rec, B192, edges_of([f] * 3), 160)
    ca, groups = check_tx(rec, B192, [f] * 3)
    s = starts_us(ca, groups)
    for a, b in zip(s, s[1:]):
        assert abs((b - a) / S.PERIOD_US - 1) <= PERIOD_TOL, b - a
    assert loom.badop() == 0


@model_only("three commands over 140 ms, 0.27 M clocks, 45 s of RTL")
def test_sirc_tx_chain_rules(backend):
    """A command with the reserved length code 3 is dropped; a command
    queued behind a frame goes out on the frame's next 45 ms slot; once a
    slot has found INQ empty the chain is over, and the next command starts
    when it is pushed; a command whose second word is late starts when
    that word arrives."""
    bench, loom, rec = tx_setup(backend, B192)
    loom.push(0, [0x0300 | 0x11, 0x05])         # length code 3
    bench.step(B192.cycles(5_000))
    assert rec.edges == []
    a, b = FRAMES[12], FRAMES[20]
    loom.push(0, tx_words(*a) + tx_words(*b))
    tx_run(bench, rec, B192, edges_of([a, b]), 110)
    ca, groups = check_tx(rec, B192, [a, b])
    s = starts_us(ca, groups)
    assert abs((s[1] - s[0]) / S.PERIOD_US - 1) <= PERIOD_TOL
    c = FRAMES[15]
    words = tx_words(*c)
    bench.step(B192.cycles(30_000))             # past b's next slot: the chain ends
    loom.push(0, words[:1])
    bench.step(B192.cycles(5_000))
    assert len(rec.edges) == edges_of([a, b])   # waiting for the second word
    late = bench.cycle
    loom.push(0, words[1:])
    tx_run(bench, rec, B192, edges_of([a, b, c]), 60)
    ca, groups = check_tx(rec, B192, [a, b, c])
    assert ca.bursts[groups[2][0]].start > late
    assert ca.us(ca.bursts[groups[2][0]].start - late) < 1_000.0   # at once
    assert loom.badop() == 0


def header_and_four_bits(backend, board):
    """Send command 0x05 (bits 1, 0, 1, 0 first) and check the carrier and
    the envelope up to bit 4's mark; returns the analysis."""
    bench, loom, rec = tx_setup(backend, board)
    f = (12, 0x05, 0x00, 0)
    loom.push(0, tx_words(*f))
    cycles = S.burst_cycles(*f)[:6]             # header, bits 0..4
    edges = 2 * sum(cycles)
    assert bench.run_until(lambda: len(rec.edges) >= edges, board.cycles(15_000),
                           every=1024)
    ca = S.CarrierAnalysis(rec.edges[:edges], board.clock_hz)
    assert [b.cycles for b in ca.bursts] == cycles == [96, 48, 24, 48, 24, 24]
    for b in ca.bursts:
        assert abs(ca.frequency(b) / S.CARRIER_HZ - 1) <= CARRIER_TOL
        assert abs(ca.duty(b) - 1 / 3.0) <= 0.01
    got = envelope(ca, list(range(6)))
    assert S.check_timing(got, S.frame_pulses(*f)[:11]) == []
    assert loom.badop() == 0
    return ca


def test_sirc_tx_header_and_four_bits(backend):
    """The PHY-GL-SDF candidate (8.4 ms on the pin, 44,900 clocks on the
    RTL with the program load, at the 1.92 MHz board): the header, its
    space and four bits, judged on the pin only."""
    ca = header_and_four_bits(backend, B192)
    assert {h for b in ca.bursts for h in b.highs} == {16}
    assert {p for b in ca.bursts for p in b.periods} == {48}


@model_only("0.42 M clocks at 50 MHz, a minute of RTL; the 1.92 MHz twin "
            "runs there")
def test_sirc_tx_carrier_at_50mhz(backend):
    """The fractional tick at 50 MHz over the header and four bits: the
    carrier within 1 % of 40 kHz (39,999.9 Hz), a 1/3 duty, every carrier
    edge on a clock edge, so a high lasts 416 or 417 clocks and a period
    1250 or 1251 (one clock, 20 ns, of jitter)."""
    ca = header_and_four_bits(backend, B50)
    assert {h for b in ca.bursts for h in b.highs} <= {416, 417}
    assert {p for b in ca.bursts for p in b.periods} <= {1250, 1251}
    assert abs(ca.frequency(ca.bursts[0]) - 39_999.9) < 0.5, ca.frequency(ca.bursts[0])


# --------------------------------------------------------------- receiver
TICK_US = S.UNIT_US / 16            # the receiver's tick, 37.5 us
HDR, SPACE = 8, 4                   # error word item bits (program header)
WIN, LONG, LEN = 1, 2, 3            # fault bits
#: the windows of the program header, in ticks: nominal units -> (min, max)
MARK_WIN = {4: (44, 84), 1: (8, 24), 2: (24, 40)}
SPACE_WIN = (8, 24)


def report_frame(length, command, address, extended=0):
    """What the receiver reports for a frame (program header)."""
    abits = S.ADDRESS_BITS[length]
    w2 = address & ((1 << abits) - 1)
    if length == 20:
        w2 |= (extended & 0xFF) << 8
    return ("frame", 0x4000 | (CODE[length] << 8) | (command & 0x7F), w2)


def report_error(n, item, fault):
    return ("error", (n << 8) | item | fault)


def reports(loom, thread=0):
    """Everything in OUTQ[thread], decoded by kind (bits 15:14)."""
    out, words = [], loom.pop_available(thread)
    while words:
        w = words.pop(0)
        kind = w >> 14
        if kind == 1:
            if not words:
                words += loom.pop(thread, 1)
            out.append(("frame", w, words.pop(0)))
        elif kind == 3:
            out.append(("error", w & 0x3FFF))
        else:
            out.append(("bad", w))
        if not words:
            words = loom.pop_available(thread)
    return out


def rx_send(bench, loom, src, board, lists, gap_us=10_000.0, tail_us=3_500.0):
    """Send pulse lists ``gap_us`` apart (the first after 2 ms of idle
    line); after each, run ``tail_us`` (past the 2.4 ms frame end) and
    collect the reports, so OUTQ never fills."""
    out = []
    for i, pulses in enumerate(lists):
        at = bench.cycle + board.cycles(2000) if i == 0 else None
        src.send(pulses, at=at, gap_us=gap_us)
        end = src.free_at + board.cycles(tail_us)
        bench.run_until(lambda: bench.cycle >= end, end - bench.cycle + 1, every=256)
        out += reports(loom)
    return out


def margin_us(board):
    """How close to a window edge a test goes: the receiver measures a pulse
    to within 30 clocks (program header), and never closer than 10 us."""
    return max(10.0, 30 * 1e6 / board.clock_hz)


def edge(pulses, sign, board):
    """Every mark and space at the short (-1) or long (+1) edge of its
    window, less the measurement margin."""
    out = []
    for lv, us in pulses:
        lo, hi = MARK_WIN[int(round(us / S.UNIT_US))] if lv == S.MARK else SPACE_WIN
        t = (hi if sign > 0 else lo) * TICK_US - sign * margin_us(board)
        out.append((lv, t))
    return out


def test_sirc_rx_frame(backend):
    """A 12-bit frame at nominal timing, reported 2.4 ms after its last mark."""
    bench, loom, src = rx_setup(backend, B192)
    f = FRAMES[12]
    assert rx_send(bench, loom, src, B192, [S.frame_pulses(*f)]) == [report_frame(*f)]
    assert loom.badop() == 0


@model_only("six frames, 0.25 M clocks, 40 s of RTL; the nominal 12-bit "
            "frame and the together scenario run there")
def test_sirc_rx_window_edges(backend):
    """A frame of each length with every mark and space at the short and at
    the long edge of its window: the 0/1 threshold (24 ticks, 900 us) from
    both sides. (A receiver module's +-5/f0 is the together tests'
    business.)"""
    bench, loom, src = rx_setup(backend, B192)
    lists, want = [], []
    for length in (12, 15, 20):
        f = FRAMES[length]
        for sign in (-1, +1):
            lists.append(edge(S.frame_pulses(*f), sign, B192))
            want.append(report_frame(*f))
    assert rx_send(bench, loom, src, B192, lists) == want


F = FRAMES[15]                              # 0x2A: bits 0..6 = 0 1 0 1 0 1 0
P = S.frame_pulses(*F)
ERRORS = [
    # (what, pulses, the error word's n, item and fault)
    ("header 3.4 ms", S.short_header(P, 3_400), (0, HDR, LONG)),
    ("header space 200 us", S.with_pulse(P, 1, 200), (0, HDR | SPACE, WIN)),
    ("header space 1.2 ms", S.with_pulse(P, 1, 1_200), (0, HDR | SPACE, LONG)),
    ("bit 3 mark 200 us", S.bad_width(P, 3, 200), (3, 0, WIN)),
    ("bit 3 mark 1.7 ms", S.bad_width(P, 3, 1_700), (3, 0, LONG)),
    ("bit 5 space 200 us", S.bad_width(P, 5, 200, S.SPACE), (6, SPACE, WIN)),
    ("bit 5 space 1.5 ms", S.bad_width(P, 5, 1_500, S.SPACE), (6, SPACE, WIN)),
    ("cut after 10 bits", S.truncate(P, S.bit_index(9) + 1), (10, SPACE, LEN)),
    ("13 bits", S.bits_pulses(S.frame_bits(*F)[:13]), (13, SPACE, LEN)),
    ("21 bits", S.bits_pulses(S.frame_bits(*FRAMES[20]) + [1]), (20, SPACE, LEN)),
]


@model_only("eleven faulty frames, 0.4 M clocks, a minute of RTL")
def test_sirc_rx_errors_and_recovery(backend):
    """A header too short for one (1.5 ms) is noise and gives nothing; each
    other fault is one error word saying where it was and what; a good frame
    after them all is received. The model's decoder rejects every faulty
    list too."""
    bench, loom, src = rx_setup(backend, B192)
    noise = S.short_header(P, 1_500)
    assert [m.kind for m in S.decode_pulses(noise)] == []
    for what, pulses, _ in ERRORS:
        assert all(m.kind == "error" for m in S.decode_pulses(pulses)), what
    lists = [noise] + [pulses for _, pulses, _ in ERRORS] + [P]
    got = rx_send(bench, loom, src, B192, lists)
    assert got == [report_error(*e) for _, _, e in ERRORS] + [report_frame(*F)]


def glitched(pulses, glitches):
    """``glitches`` = [(index, offset us, width us)], applied last first so
    the indices stay those of ``pulses``."""
    for index, offset, width in sorted(glitches, reverse=True):
        pulses = S.glitch(pulses, S.start_of(pulses, index) + offset, width)
    return pulses


@model_only("three frames, 0.15 M clocks; the nominal frame runs on the RTL")
def test_sirc_rx_glitches_are_rejected(backend):
    """50 us glitches of both levels, in the header, a 1 and a 0 mark and a
    space, and a 40 us and a 1 ms pulse on the idle line, change nothing; a
    150 us one (longer than the 75 us filter) inside a 2T mark is an error
    (the mark ends too short)."""
    bench, loom, src = rx_setup(backend, B192)
    good = glitched(P, [(0, 1_000, 50), (1, 300, 50), (S.bit_index(1), 500, 50),
                        (S.bit_index(2), 250, 50), (S.bit_index(4, S.SPACE), 250, 50)])
    bad = glitched(P, [(S.bit_index(1), 200, 150)])
    lists = [[(S.MARK, 40.0)], [(S.MARK, 1_000.0)], good, bad]
    got = rx_send(bench, loom, src, B192, lists)
    assert got == [report_frame(*F), report_error(1, 0, WIN)]


@model_only("0.6 M clocks at 50 MHz, a minute and a half of RTL; the 1.92 MHz "
            "frame runs there")
def test_sirc_rx_at_50mhz(backend):
    """The receiver at its 50 MHz tick (1875 clocks; a glitch filter of
    933 + 4 slots): a header and three bits at the short edges of their
    windows, then the frame ends: a length error after 3 bits."""
    bench, loom, src = rx_setup(backend, B50)
    cut = S.truncate(P, S.bit_index(2) + 1)
    assert rx_send(bench, loom, src, B50, [edge(cut, -1, B50)]) == \
        [report_error(3, SPACE, LEN)]


# ---------------------------------------------------------------- together
def both_setup(backend, board, **tsop):
    """The transmitter on thread 0, a receiver module from OUT0 to IN0, the
    receiver on thread 1 (its source with `.thread 1`, at 0x080)."""
    bench = ir_bench(backend)
    rec = bench.add(S.EdgeRecorder("OUT0"))
    module = bench.add(S.TsopModel("OUT0", "IN0", board.clock_hz, **tsop))
    loom = start(backend, bench, [(program("ir_sirc_tx"), 0, board.tx_tick),
                                  (program("ir_sirc_rx", 1), 1, board.rx_tick)])
    return bench, loom, rec, module


def test_sirc_together_frame_and_repeat(backend):
    """A 20-bit command with one repeat from thread 0 through a receiver
    module (td 10 carrier periods, output pulses 3 periods long, 5 us of
    jitter on every edge) to thread 1, at the 1.92 MHz board: the pin
    judged as in the transmitter tests, two frames 45 ms apart, two
    reports."""
    bench, loom, rec, module = both_setup(backend, B192, td=10, delta=3,
                                          jitter_us=5, seed=1)
    f = FRAMES[20]
    loom.push(0, tx_words(*f, repeats=1))
    tx_run(bench, rec, B192, edges_of([f, f]), 100)
    ca, groups = check_tx(rec, B192, [f, f])
    s = starts_us(ca, groups)
    assert abs((s[1] - s[0]) / S.PERIOD_US - 1) <= PERIOD_TOL
    assert len(module.events) == 2 * 2 * 21          # one output pulse a mark
    assert reports(loom, 1) == [report_frame(*f)] * 2
    assert loom.badop() == 0


@model_only("two frames per receiver module, 30 s of RTL each")
@pytest.mark.parametrize("td,delta", [(7, -5), (13, 5)])
def test_sirc_together_receiver_module_limits(backend, td, delta):
    """A receiver module at the TSOP382.. datasheet's limits (td 7 or 13
    carrier periods, output pulses 5 periods short or long, +-125 us), 10 us
    of jitter; a 12-bit and a 15-bit command."""
    bench, loom, rec, module = both_setup(backend, B192, td=td, delta=delta,
                                          jitter_us=10, seed=td)
    a, b = FRAMES[12], FRAMES[15]
    loom.push(0, tx_words(*a) + tx_words(*b))
    tx_run(bench, rec, B192, edges_of([a, b]), 100)
    assert reports(loom, 1) == [report_frame(*a), report_frame(*b)]


# -------------------------------------------------------------- the programs
def test_sirc_programs_assemble_strict_with_no_diagnostic():
    """L2-DEADLINE: the transmitter's 10 pairs proved at a 16-clock tick with
    8 clocks of slack, its two INQ tests' POPs declared bounded; the receiver
    has no WAITD and no pair. Both fit a quarter of the 512-word memory, and
    the receiver moved to thread 1 stays in that quarter."""
    tx, rx = program("ir_sirc_tx"), program("ir_sirc_rx")
    assert tx.diagnostics == [] and rx.diagnostics == []
    assert (len(tx.words), len(rx.words)) == (76, 109)
    t = tx.deadlines[0]
    assert t.period == 16 and not t.infeasible and not t.unbounded
    assert len(t.pairs) == 10 and t.worst_slack == 8
    assert [d.name for d in t.declarations] == ["POP", "POP"]
    assert all("only this thread pops INQ" in d.reason for d in t.declarations)
    r = rx.deadlines[0]
    assert r.period == 64 and r.pairs == [] and not r.unbounded
    assert max(tx.words) < 0x80 and max(rx.words) < 0x80
    moved = program("ir_sirc_rx", 1)
    assert moved.diagnostics == []
    assert min(moved.words) == 0x80 and max(moved.words) < 0x100


# --------------------------------------------------------------- the model
def test_sirc_model_encoder_and_decoder():
    """Round trips at all three lengths; the field layout; the faults."""
    for length, f in FRAMES.items():
        p = S.frame_pulses(*f)
        assert len(p) == 2 + 2 * length - 1
        m = S.decode_pulses(p)[0]
        assert (m.kind, m.length, m.command, m.address, m.extended) == ("frame",) + f
        assert S.decode_pulses(S.vary(p, mark=1.1, space=1.1))[0].kind == "frame"
    assert S.frame_bits(12, 0x01, 0x10) == [1] + [0] * 10 + [1]
    assert S.frame_bits(20, 0, 0, 0x80)[19] == 1
    assert S.frame_units(20, 0x7F, 0x1F, 0xFF) == 5 + 20 * 3 - 1      # 64 T
    assert S.burst_cycles(12, 0x01, 0) == [96, 48] + [24] * 11
    with pytest.raises(ValueError):
        S.frame_bits(13, 0, 0)
    assert S.decode_pulses(S.bad_width(P, 3, 1_700))[0].reason == "mark of bit 3"
    assert S.decode_pulses(S.truncate(P, S.bit_index(9) + 1))[0].reason == "length 10"
    assert S.decode_pulses(S.short_header(P, 1_500)) == []


def test_sirc_model_timing_checker():
    """check_timing passes a nominal envelope and the 25 us a real burst
    loses at its end, and flags a mark 30 us off and a shifted start."""
    p = S.frame_pulses(*FRAMES[12])
    assert S.check_timing(p, p) == []
    real = S.tsop_distort(p, -50 / 3.0)         # the last cycle's dark 2/3
    assert S.check_timing(real, p) == []
    assert S.check_timing(S.with_pulse(p, 2, p[2][1] + 30), p)[0][0] == 2
    shifted = S.with_pulse(p, 1, p[1][1] + 30)
    assert S.check_timing(shifted, p) != []
    assert S.check_timing(p[:-2], p)[0][0] == -1
