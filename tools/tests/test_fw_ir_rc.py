"""L3 for the RC-5 / RC-6 mode 0 infrared programs, on the model and the RTL.

``firmware/ir_rc_tx.loom`` (thread 0, LED = OUT0) makes the 36 kHz carrier
and both envelopes itself; ``firmware/ir_rc_rx.loom`` (RX = IN0) decodes a
receiver module's output and tells the two protocols apart. The references
are ``tools.protomodels.ir_rc``: its encoder gives every burst's start and
length, its decoder reads the transmitter's envelope, ``IrSource`` feeds the
receiver demodulated frames with timing variations and faults, and
``TsopModel`` (36 kHz) sits between the two programs' pins when they run
together (transmitter on thread 0, receiver on thread 1).

Expected values come from the protocol descriptions and the TSOP382..
datasheet (the model's docstring) and the program headers' stated formats
and windows, never from what the model or the RTL did:

* transmitter: the carrier within 1 % of 36 kHz over every burst, a 1/3
  duty, every burst exactly 16 carrier periods per lit unit and starting
  within 2 us of its nominal place from the frame's start, the repeat
  period 256 units (RC-5) or 240 (RC-6) within 0.1 %, the toggle kept on
  repeats and flipped on a new frame;
* receiver: pulses accepted within [n - 1/2, n + 1/2) halves, glitches up
  to 55 us ignored, every fault one error word saying where and what.

Two boards. Both programs take their tick from the host: at 50 MHz the
transmitter's tick is 462 + 247/256 clocks (a third of a carrier period)
and the receiver's 1388 + 228/256 (one carrier period). An RC-5 frame and
its repeat are then 7 M clocks, so the RTL runs the same programs on a
board clocked at 1.728 MHz, where the ticks are exactly 16 and 48 clocks
and the waveform in microseconds is the same (the analogue of
``test_fw_ir_nec``'s 1.824 MHz board, which suits 38 kHz). The 50 MHz bodies
are ``model_only``.
"""

import pytest

from tools.loomasm import assemble_file, assemble_text
from tools.loomisa import REPO, load
from tools.protomodels import ir_rc as R
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

    def us(self, cycles):
        return cycles * 1e6 / self.clock_hz

    def __repr__(self):
        return self.name


B1728 = Board("1728kHz", 1_728_000, (16, 0), (48, 0))
B50 = Board("50MHz", 50_000_000, (462, 247), (1388, 228))


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
    """The FIFOs, the deadline latch and the 512-word memory."""
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
    rec = bench.add(R.EdgeRecorder("OUT0"))
    loom = start(backend, bench, [(program("ir_rc_tx"), 0, board.tx_tick)])
    return bench, loom, rec


def rx_setup(backend, board):
    bench = ir_bench(backend)
    src = bench.add(R.IrSource("IN0", board.clock_hz, gap_us=10_000.0))
    loom = start(backend, bench, [(program("ir_rc_rx"), 0, board.rx_tick)])
    return bench, loom, src


# ------------------------------------------------------------ transmitter
REPEAT = 0x8000
CARRIER_TOL = 0.01
PLACE_US = 2.0                      # a burst's start against its nominal place
PERIOD_TOL = 0.001


def frame_words(proto, address, command):
    """The two INQ words of a frame (program header)."""
    if proto == R.RC6:
        return [0x4000 | (command & 0xFF), address & 0xFF]
    return [command & 0x7F, address & 0x1F]


def levels_of(proto, address, command, toggle):
    f = R.rc6_levels if proto == R.RC6 else R.rc5_levels
    return f(address, command, toggle)


def edges_of(levels):
    return 32 * sum(levels)          # two edges a period, 16 periods a unit


def tx_run(bench, rec, board, edges, budget_ms):
    """Run until ``edges`` edges have gone out and the line has been low 5 ms."""
    quiet = board.cycles(5000)
    done = lambda: (len(rec.edges) >= edges
                    and bench.cycle - rec.edges[-1][0] > quiet)
    assert bench.run_until(done, board.cycles(budget_ms * 1000), every=256), (
        "%d of %d edges after %d ms" % (len(rec.edges), edges, budget_ms))


def groups(ca):
    """The bursts grouped into frames: a space over 10 ms separates two (the
    longest inside a frame is 1.8 ms)."""
    out = []
    for b in ca.bursts:
        if out and ca.us(b.start - out[-1][-1].end) < 10_000:
            out[-1].append(b)
        else:
            out.append([b])
    return out


def check_frame(ca, group, levels):
    """One frame on the pin against the encoder: carrier, duty, every
    burst's length in periods and its start from the frame's start S.
    Returns S in cycles and the decoded message."""
    for b in group:
        assert abs(ca.frequency(b) / R.CARRIER_HZ - 1) <= CARRIER_TOL, ca.frequency(b)
        assert abs(ca.duty(b) - 1 / 3.0) <= 0.01, ca.duty(b)
    nominal = R.bursts(levels)
    assert [b.cycles for b in group] == [n for _, n in nominal]
    unit = R.UNIT_US * ca.clock_hz / 1e6
    s = group[0].start - nominal[0][0] * unit
    late = [(i, ca.us(b.start - s - u * unit)) for i, (b, (u, _)) in
            enumerate(zip(group, nominal)) if abs(ca.us(b.start - s - u * unit)) > PLACE_US]
    assert not late, "bursts off their places (index, us): %r" % late[:6]
    pulses = []
    for a, b in zip(group, group[1:] + [None]):
        pulses.append((R.MARK, ca.us(a.end - a.start)))
        if b is not None:
            pulses.append((R.SPACE, ca.us(b.start - a.end)))
    msgs = R.decode_pulses(pulses, tol=0.1)
    assert len(msgs) == 1, msgs
    return s, msgs[0]


def check_tx(rec, board, frames):
    """``frames`` = [(proto, address, command, toggle)], one per frame on
    the pin; returns the frames' starts in cycles."""
    ca = R.CarrierAnalysis(rec.edges, board.clock_hz)
    got = groups(ca)
    assert len(got) == len(frames), [len(g) for g in got]
    starts = []
    for g, (proto, a, c, t) in zip(got, frames):
        s, msg = check_frame(ca, g, levels_of(proto, a, c, t))
        assert (msg.kind, msg.address, msg.command, msg.toggle) == (proto, a, c, t)
        starts.append(s)
    return starts


def check_period(board, starts, i, units):
    us = board.us(starts[i + 1] - starts[i])
    assert abs(us / (units * R.UNIT_US) - 1) <= PERIOD_TOL, us


def test_tx_rc6_frame(backend):
    """The PHY-GL-SDF candidate for the transmitter (23 ms of frame, 48,000
    clocks at the 1.728 MHz board): one RC-6 frame, judged on the pin only:
    exactly 16 clocks high and 48 a period, every burst on its place."""
    bench, loom, rec = tx_setup(backend, B1728)
    loom.push(0, frame_words(R.RC6, 0xA5, 0x3C))
    tx_run(bench, rec, B1728, edges_of(levels_of(R.RC6, 0xA5, 0x3C, 0)), 40)
    check_tx(rec, B1728, [(R.RC6, 0xA5, 0x3C, 0)])
    ca = R.CarrierAnalysis(rec.edges, B1728.clock_hz)
    assert {h for b in ca.bursts for h in b.highs} == {16}
    assert {p for b in ca.bursts for p in b.periods} == {48}
    assert loom.badop() == 0


@model_only("three RC-5 frames, 250 ms: two minutes of RTL; the together "
            "scenario runs an RC-5 frame and its repeat there")
def test_tx_rc5_repeats_and_toggle(backend):
    """An RC-5 frame, a repeat word and a new frame of extended RC-5
    (command bit 6 set, S2 = 0): the repeat goes out 256 units (113.8 ms)
    after the frame with the same toggle, the new frame 256 units after that
    with the toggle flipped."""
    bench, loom, rec = tx_setup(backend, B1728)
    loom.push(0, frame_words(R.RC5, 0x05, 0x35) + [REPEAT]
              + frame_words(R.RC5, 0x1F, 0x41))
    frames = [(R.RC5, 0x05, 0x35, 0), (R.RC5, 0x05, 0x35, 0), (R.RC5, 0x1F, 0x41, 1)]
    tx_run(bench, rec, B1728, sum(edges_of(levels_of(*f)) for f in frames), 300)
    starts = check_tx(rec, B1728, frames)
    check_period(B1728, starts, 0, R.RC5_PERIOD_UNITS)
    check_period(B1728, starts, 1, R.RC5_PERIOD_UNITS)
    assert loom.badop() == 0


@model_only("two RC-6 frames, 130 ms: a minute of RTL; the RTL runs one RC-6 frame")
def test_tx_rc6_repeat(backend):
    """An RC-6 frame and a repeat word: the repeat 240 units (106.7 ms) on,
    the trailer bit (toggle) the same."""
    bench, loom, rec = tx_setup(backend, B1728)
    loom.push(0, frame_words(R.RC6, 0x00, 0xFF) + [REPEAT])
    frames = [(R.RC6, 0x00, 0xFF, 0)] * 2
    tx_run(bench, rec, B1728, sum(edges_of(levels_of(*f)) for f in frames), 160)
    starts = check_tx(rec, B1728, frames)
    check_period(B1728, starts, 0, R.RC6_PERIOD_UNITS)


@model_only("four frames, 400 ms: three minutes of RTL")
def test_tx_chain_rules(backend):
    """A repeat word with no frame before it is dropped; an RC-6 frame
    queued behind an RC-5 frame goes out on the RC-5 period's slot with the
    toggle flipped; a frame whose address word comes after its slot starts
    when the word does; once a slot has found INQ empty the chain is over
    and a repeat word pushed after it is dropped too."""
    bench, loom, rec = tx_setup(backend, B1728)
    loom.push(0, [REPEAT])
    bench.step(B1728.cycles(5_000))
    assert rec.edges == []
    loom.push(0, frame_words(R.RC5, 0x01, 0x02) + frame_words(R.RC6, 0x12, 0x34))
    two = [(R.RC5, 0x01, 0x02, 0), (R.RC6, 0x12, 0x34, 1)]
    tx_run(bench, rec, B1728, sum(edges_of(levels_of(*f)) for f in two), 200)
    starts = check_tx(rec, B1728, two)
    check_period(B1728, starts, 0, R.RC5_PERIOD_UNITS)
    third = frame_words(R.RC5, 0x0A, 0x0B)
    loom.push(0, third[:1])                      # before the RC-6 frame's slot
    bench.step(B1728.cycles(110_000))            # past it: the address is late
    edges = len(rec.edges)
    late = bench.cycle
    loom.push(0, third[1:])
    three = two + [(R.RC5, 0x0A, 0x0B, 0)]
    tx_run(bench, rec, B1728, sum(edges_of(levels_of(*f)) for f in three), 60)
    starts = check_tx(rec, B1728, three)
    assert starts[2] > late and len(rec.edges) > edges
    edges = len(rec.edges)
    bench.step(B1728.cycles(120_000))            # past the third frame's slot
    loom.push(0, [REPEAT])
    bench.step(B1728.cycles(30_000))
    assert len(rec.edges) == edges
    assert loom.badop() == 0


@model_only("1.2 M clocks at 50 MHz, three minutes of RTL; the 1.728 MHz "
            "twin of the frame runs there")
def test_tx_carrier_at_50mhz(backend):
    """The fractional tick at 50 MHz over the RC-6 leader and the start and
    mode bits: the carrier within 1 % of 36 kHz over every burst (35,999.85
    Hz over the leader), a 1/3 duty, every edge on a clock edge, so a high
    lasts 462 or 463 clocks and a period 1388 or 1389; every burst on its
    place."""
    bench, loom, rec = tx_setup(backend, B50)
    loom.push(0, frame_words(R.RC6, 0x00, 0x00))
    levels = levels_of(R.RC6, 0x00, 0x00, 0)[:16]  # leader, start, mode
    edges = edges_of(levels)
    assert bench.run_until(lambda: len(rec.edges) >= edges, B50.cycles(9_000),
                           every=1024)
    ca = R.CarrierAnalysis(rec.edges[:edges], B50.clock_hz)
    check_frame(ca, ca.bursts, levels)
    assert {h for b in ca.bursts for h in b.highs} == {462, 463}
    assert {p for b in ca.bursts for p in b.periods} == {1388, 1389}
    assert abs(ca.frequency(ca.bursts[0]) - 35_999.85) < 0.5


# --------------------------------------------------------------- receiver
TICK_US = R.UNIT_US / 16            # the receiver's tick, one carrier period
WIN, TMO, BIPH, HDR = 1, 2, 3, 4    # fault codes (program header)


def rc5_word(address, command, toggle):
    return sum(b << (13 - i) for i, b in enumerate(R.rc5_bits(address, command, toggle)))


def report_frame(proto, address, command, toggle):
    """What the receiver reports for a frame (program header)."""
    if proto == R.RC6:
        return ("frame", 1, toggle, ((address & 0xFF) << 8) | (command & 0xFF))
    return ("frame", 0, toggle, rc5_word(address, command, toggle))


def report_error(h, rc6, fault):
    return ("error", h, rc6, fault)


def reports(loom, thread=0):
    """Everything in OUTQ[thread], decoded by kind (bits 15:14)."""
    out, words = [], loom.pop_available(thread)
    while words:
        w = words.pop(0)
        if w >> 14 == 1:
            if not words:
                words += loom.pop(thread, 1)
            assert w & 0x0FFF == 0, hex(w)
            out.append(("frame", (w >> 13) & 1, (w >> 12) & 1, words.pop(0)))
        elif w >> 14 == 3:
            assert w & 0xB8 == 0, hex(w)
            out.append(report_error((w >> 8) & 0x3F, (w >> 6) & 1, w & 7))
        else:
            out.append(("bad", w))
        if not words:
            words = loom.pop_available(thread)
    return out


def pulses_of(proto, address, command, toggle):
    return R.frame_pulses(levels_of(proto, address, command, toggle))


def rx_send(bench, loom, src, board, lists, gap_us=10_000.0, tail_us=4_000.0):
    """Send pulse lists ``gap_us`` apart, or ``tail_us`` when that is longer
    (the first after 2 ms of idle line); after each, run ``tail_us`` (past the 2.2 ms end-of-frame timeout) and
    collect the reports."""
    out = []
    for i, pulses in enumerate(lists):
        at = bench.cycle + board.cycles(2000) if i == 0 else bench.cycle
        src.send(pulses, at=at, gap_us=gap_us)  # never in the past
        end = src.free_at + board.cycles(tail_us)
        bench.run_until(lambda: bench.cycle >= end, end - bench.cycle + 1, every=256)
        out += reports(loom)
    return out


def glitched(pulses, glitches):
    """``glitches`` = [(index, offset us, width us)], applied last first so
    the indices stay those of ``pulses``."""
    for index, offset, width in sorted(glitches, reverse=True):
        pulses = R.glitch(pulses, R.start_of(pulses, index) + offset, width)
    return pulses


def test_rx_rc6_glitches_are_rejected(backend):
    """The PHY-GL-SDF candidate for the receiver (an RC-6 frame, 23 ms,
    40,000 clocks at the 1.728 MHz board): 45 us pulses of the other level
    in the leader mark, the leader space, a one-unit mark, the trailer's
    space and a data space, and a 45 us pulse on the idle line before it,
    change nothing (the filter is 2 ticks, 55.6 us)."""
    bench, loom, src = rx_setup(backend, B1728)
    frame = pulses_of(R.RC6, 0x5A, 0xC3, 1)
    good = glitched(frame, [(0, 1_500, 45), (1, 400, 45), (2, 200, 45),
                            (6, 500, 45), (12, 150, 45)])
    got = rx_send(bench, loom, src, B1728, [[(R.MARK, 45.0)], good], gap_us=3_000.0)
    assert got == [report_frame(R.RC6, 0x5A, 0xC3, 1)]
    assert loom.badop() == 0


@model_only("four frames, 120 ms: a minute and a half of RTL; RC-5 runs "
            "through the together scenario there")
def test_rx_both_protocols(backend):
    """RC-5, extended RC-5 (command bit 6: S2 = 0, the first mark two
    halves long), RC-6 and RC-6 with the other toggle, back to back: the
    protocol told from the first mark each time."""
    bench, loom, src = rx_setup(backend, B1728)
    frames = [(R.RC5, 0x05, 0x35, 0), (R.RC5, 0x1F, 0x7F, 1),
              (R.RC6, 0x00, 0xFF, 0), (R.RC6, 0xFF, 0x00, 1)]
    got = rx_send(bench, loom, src, B1728, [pulses_of(*f) for f in frames])
    assert got == [report_frame(*f) for f in frames]
    assert loom.badop() == 0


MARGIN_TICKS = 2                    # a pulse is measured to a tick or so


def edge(proto, pulses, sign):
    """Every pulse at the short (sign -1) or long (+1) edge of its window:
    n halves +- (half a half less 2 ticks); the RC-6 leader +- (16 - 2)
    ticks of its 80 .. 111 (program header)."""
    q = 16 if proto == R.RC6 else 32
    out = []
    for i, (lv, us) in enumerate(pulses):
        half = 16 if proto == R.RC6 and i == 0 else q // 2
        out.append((lv, us + sign * (half - MARGIN_TICKS) * TICK_US))
    return out


def module_and_sender(pulses, factor, delta_us):
    """A sender whose clock is ``factor`` off (every pulse scaled) seen
    through a receiver module that lengthens marks by ``delta_us``."""
    return R.tsop_distort(R.vary(pulses, mark=factor, space=factor), delta_us)


F5 = (R.RC5, 0x05, 0x35, 0)
F6 = (R.RC6, 0x5A, 0xC3, 1)
F5X = (R.RC5, 0x1A, 0x6E, 1)        # extended: S2 = 0


@model_only("eight frames, 220 ms: three minutes of RTL; the nominal and "
            "glitched frames run there")
def test_rx_tolerance_limits(backend):
    """Every pulse of an RC-5 and an RC-6 frame at the short and at the long
    edge of its window; then the TSOP38236's pulse error at its limits
    (+-5/f0, 139 us) on top of a sender clock 3 % (RC-6) or 10 % (RC-5)
    off, the error budgets the program header states."""
    bench, loom, src = rx_setup(backend, B1728)
    p5, p5x, p6 = pulses_of(*F5), pulses_of(*F5X), pulses_of(*F6)
    d = 5e6 / R.CARRIER_HZ
    lists = [edge(R.RC5, p5, -1), edge(R.RC5, p5x, +1),
             edge(R.RC6, p6, -1), edge(R.RC6, p6, +1),
             module_and_sender(p5, 1.10, d), module_and_sender(p5x, 0.90, -d),
             module_and_sender(p6, 1.03, d), module_and_sender(p6, 0.97, -d)]
    got = rx_send(bench, loom, src, B1728, lists)
    assert got == [report_frame(*f) for f in (F5, F5X, F6, F6, F5, F5X, F6, F6)]


def cut_after(pulses, j):
    """``pulses`` up to pulse ``j``; a space is closed by a 300 us mark,
    which an idle receiver ignores as noise (under 444 us)."""
    out = list(pulses[:j + 1])
    if out[-1][0] == R.SPACE:
        out.append((R.MARK, 300.0))
    return out


def h_before(proto, pulses, j):
    """Halves taken before pulse ``j`` (program header: RC-5 counts S1's
    dark first half; RC-6 counts from the leader space)."""
    q = R.UNIT_US if proto == R.RC6 else 2 * R.UNIT_US
    first = 1 if proto == R.RC6 else 0
    return sum(int(round(us / q)) for _, us in pulses[first:j]) + (proto == R.RC5)


def index(pulses, level, units, after=1):
    return next(i for i, (lv, us) in enumerate(pulses)
                if i >= after and lv == level and round(us / R.UNIT_US) == units)


def at_half(proto, pulses, h):
    """The index of the pulse that brings the halves taken to ``h``."""
    return next(j for j in range(len(pulses)) if h_before(proto, pulses, j + 1) >= h)


def errors():
    """(what, pulses, (h, RC-6, fault)) from the header's format."""
    p5, p6 = pulses_of(*F5), pulses_of(*F6)
    m3 = index(p6, R.MARK, 3)
    h1, s2 = index(p5, R.MARK, 2), index(p5, R.SPACE, 4)
    mode1 = R.frame_pulses(R.rc6_levels(0, 0, 0, data=[1, 0, 0, 1, 0] + [0] * 16))
    normal = [1] * 6 + [0, 0] + [v for b in [1, 0, 0, 0, 0, 1, 1] for v in (b, 1 - b)]
    normal = R.frame_pulses(normal)
    out = [
        ("RC-6 leader mark 3.3 ms", [(R.MARK, 3_300.0)], (0, 1, TMO)),
        ("RC-6 leader space 1 unit", cut_after(R.with_pulse(p6, 1, R.UNIT_US), 2),
         (2, 1, BIPH)),
        ("RC-6 3-unit mark 1.65 ms", cut_after(R.with_pulse(p6, m3, 1_650), m3),
         (h_before(R.RC6, p6, m3), 1, WIN)),
        ("RC-6 mode 001", cut_after(mode1, at_half(R.RC6, mode1, 10)), (10, 1, HDR)),
        ("RC-6 trailer one unit", cut_after(normal, at_half(R.RC6, normal, 12)),
         (12, 1, BIPH)),
        ("RC-6 data bit missing", R.frame_pulses(R.drop_bit(R.rc6_levels(
            0x5A, 0xC3, 1), 3, R.RC6)), None),
        ("RC-5 half-bit mark 380 us", cut_after(R.with_pulse(p5, h1, 380), h1),
         (h_before(R.RC5, p5, h1), 0, WIN)),
        ("RC-5 two-half space 2.4 ms", cut_after(R.with_pulse(p5, s2, 2_400), s2),
         (h_before(R.RC5, p5, s2), 0, TMO)),
        ("RC-5 bit missing", R.frame_pulses(R.drop_bit(R.rc5_levels(
            0x05, 0x35, 0), 5, R.RC5)), None),
    ]
    fixed = []
    for what, pulses, e in out:
        if e is None:                   # a frame cut short: the timeout
            proto = R.RC6 if what.startswith("RC-6") else R.RC5
            e = (h_before(proto, pulses, len(pulses)), proto == R.RC6, TMO)
        fixed.append((what, pulses, (e[0], int(e[1]), e[2])))
    return fixed


@model_only("nine faulty frames, a minute and a half of RTL")
def test_rx_errors_and_recovery(backend):
    """Each fault is one error word saying where it was and what; a good
    RC-5 and a good RC-6 frame after them all are received."""
    bench, loom, src = rx_setup(backend, B1728)
    cases = errors()
    lists = [p for _, p, _ in cases] + [pulses_of(*F5), pulses_of(*F6)]
    got = rx_send(bench, loom, src, B1728, lists, gap_us=5_000.0)
    want = [report_error(*e) for _, _, e in cases] + [report_frame(*F5), report_frame(*F6)]
    bad = [(c[0], g, w) for c, g, w in zip(cases, got, want) if g != w]
    assert got == want, bad or (got, want)


@model_only("1.5 M clocks at 50 MHz, four minutes of RTL; the 1.728 MHz "
            "glitch frame runs there")
def test_rx_short_pulses_at_50mhz(backend):
    """At 50 MHz (tick 1388 + 228/256 clocks, a dead time of 62 us after an
    edge, program header) a 150 us space in an RC-6 frame and a 150 us
    glitch inside an RC-5 two-half space are too short (fault 1), where the
    1.728 MHz board's 243 us dead time would swallow them; an RC-5 frame
    with every pulse at the short edge of its window is received."""
    bench, loom, src = rx_setup(backend, B50)
    p5, p6 = pulses_of(*F5), pulses_of(*F6)
    s1, s2 = index(p6, R.SPACE, 1, after=2), index(p5, R.SPACE, 4)
    g5 = R.glitch(p5, R.start_of(p5, s2) + 800, 150)
    lists = [cut_after(R.with_pulse(p6, s1, 150), s1), cut_after(g5, s2 + 1),
             edge(R.RC5, p5, -1)]
    got = rx_send(bench, loom, src, B50, lists, gap_us=3_000.0)
    assert got == [report_error(h_before(R.RC6, p6, s1), 1, WIN),
                   report_error(h_before(R.RC5, p5, s2) + 1, 0, WIN),
                   report_frame(*F5)]


# ---------------------------------------------------------------- together
def both_setup(backend, board, **tsop):
    """The transmitter on thread 0, a receiver module from OUT0 to IN0, the
    receiver on thread 1 (its source with `.thread 1`, at 0x080)."""
    bench = ir_bench(backend)
    rec = bench.add(R.EdgeRecorder("OUT0"))
    module = bench.add(R.TsopModel("OUT0", "IN0", board.clock_hz, **tsop))
    loom = start(backend, bench, [(program("ir_rc_tx"), 0, board.tx_tick),
                                  (program("ir_rc_rx", 1), 1, board.rx_tick)])
    return bench, loom, rec, module


def test_together_rc5_frame_and_repeat(backend):
    """An RC-5 frame and its repeat from thread 0 through a receiver module
    (td 10 carrier periods, output pulses 3 periods long, 5 us of jitter on
    every edge) to thread 1, at the 1.728 MHz board: the pin judged as in
    the transmitter tests, both frames received with the same toggle.
    About 240,000 clocks."""
    bench, loom, rec, module = both_setup(backend, B1728, td=10, delta=3,
                                          jitter_us=5, seed=1)
    loom.push(0, frame_words(R.RC5, 0x05, 0x35) + [REPEAT])
    frames = [F5, F5]
    tx_run(bench, rec, B1728, sum(edges_of(levels_of(*f)) for f in frames), 160)
    starts = check_tx(rec, B1728, frames)
    check_period(B1728, starts, 0, R.RC5_PERIOD_UNITS)
    assert reports(loom, 1) == [report_frame(*F5)] * 2
    assert loom.badop() == 0


@model_only("three frames per receiver module, two minutes of RTL each")
@pytest.mark.parametrize("td,delta", [(7, -5), (13, 5)])
def test_together_receiver_module_limits(backend, td, delta):
    """A receiver module at the TSOP382.. datasheet's limits (td 7 or 13
    carrier periods, output pulses 5 periods short or long), 10 us of
    jitter: an RC-6 frame, its repeat, and an extended RC-5 frame with the
    toggle flipped."""
    bench, loom, rec, module = both_setup(backend, B1728, td=td, delta=delta,
                                          jitter_us=10, seed=td)
    loom.push(0, frame_words(R.RC6, 0x5A, 0xC3) + [REPEAT]
              + frame_words(R.RC5, 0x1A, 0x6E))
    frames = [(R.RC6, 0x5A, 0xC3, 0)] * 2 + [(R.RC5, 0x1A, 0x6E, 1)]
    tx_run(bench, rec, B1728, sum(edges_of(levels_of(*f)) for f in frames), 260)
    assert reports(loom, 1) == [report_frame(*f) for f in frames]


# -------------------------------------------------------------- the programs
def test_programs_assemble_strict_with_no_diagnostic():
    """L2-DEADLINE: the transmitter's 55 pairs proved at a 16-clock tick
    with 4 clocks of slack, its two INQ tests' POPs declared bounded; the
    receiver has no WAITD and no pair. Each fills a quarter of the 512-word
    memory (128 and 127 words), and the receiver moved to thread 1 stays in
    that quarter, so the two run together."""
    tx, rx = program("ir_rc_tx"), program("ir_rc_rx")
    assert tx.diagnostics == [] and rx.diagnostics == []
    assert (len(tx.words), len(rx.words)) == (128, 127)
    t = tx.deadlines[0]
    assert t.period == 16 and not t.infeasible and not t.unbounded
    assert len(t.pairs) == 55 and t.worst_slack == 4
    assert [d.name for d in t.declarations] == ["POP", "POP"]
    assert all("only this thread pops INQ" in d.reason for d in t.declarations)
    r = rx.deadlines[0]
    assert r.period == 48 and r.pairs == [] and not r.unbounded
    assert max(tx.words) < 0x80 and max(rx.words) < 0x80
    moved = program("ir_rc_rx", 1)
    assert moved.diagnostics == []
    assert min(moved.words) == 0x80 and max(moved.words) < 0x100


# --------------------------------------------------------------- the model
def test_model_encoder_and_decoder():
    """The encoder against the descriptions' arithmetic: RC-5 is 14 bits of
    4 units (56), RC-6 a 6 + 2 unit leader, 4 bits of 2 units, a 4-unit
    trailer and 16 bits of 2 (52); the decoder reads both back, flags
    extended RC-5, and rejects a frame with a bit missing."""
    lv5, lv6 = R.rc5_levels(0x05, 0x35, 1), R.rc6_levels(0xA5, 0x3C, 1)
    assert (len(lv5), len(lv6)) == (56, 52)
    assert lv6[:8] == [1] * 6 + [0] * 2 and lv6[16:20] == [1, 1, 0, 0]
    assert lv5[:4] == [0, 0, 1, 1] and R.first_mark_unit(lv5) == 2
    m5 = R.decode_pulses(R.frame_pulses(lv5))[0]
    assert (m5.kind, m5.address, m5.command, m5.toggle) == (R.RC5, 5, 0x35, 1)
    assert not m5.extended
    assert R.decode_pulses(pulses_of(*F5X))[0].extended
    m6 = R.decode_pulses(R.frame_pulses(lv6))[0]
    assert (m6.kind, m6.address, m6.command, m6.toggle) == (R.RC6, 0xA5, 0x3C, 1)
    for proto, lv in ((R.RC5, lv5), (R.RC6, lv6)):
        cut = R.decode_pulses(R.frame_pulses(R.drop_bit(lv, 3, proto)))[0]
        assert cut.kind == "error" and cut.protocol == proto
    bad = R.with_pulse(R.frame_pulses(lv6), 2, 0.3 * R.UNIT_US)
    assert R.decode_pulses(bad)[0].kind == "error"


def test_model_full_carrier():
    """The full-carrier encoder through the carrier analyser: 36 kHz, duty
    1/3, the bursts of the encoder, and its envelope decodes."""
    lv = R.rc6_levels(0x12, 0x34, 0)
    ca = R.CarrierAnalysis(R.carrier_edges(lv, 1_728_000), 1_728_000)
    assert all(abs(ca.frequency(b) - 36_000) < 1 for b in ca.bursts)
    assert all(abs(ca.duty(b) - 1 / 3.0) < 0.01 for b in ca.bursts)
    assert [b.cycles for b in ca.bursts] == [n for _, n in R.bursts(lv)]
    assert R.decode_pulses(ca.pulses(), tol=0.1)[0].command == 0x34
