"""L3 for the NEC infrared programs, on the model and the RTL.

``firmware/ir_nec_tx.loom`` (thread 0, LED = OUT0) makes the 38 kHz carrier
and the NEC envelope itself; ``firmware/ir_nec_rx.loom`` (RX = IN0) decodes a
receiver module's output. The references are ``tools.protomodels.ir_nec``:
the carrier analyser and the decoder hold the transmitter to the NEC
timing, ``IrSource`` feeds the receiver demodulated frames with timing
variations and faults, and ``TsopModel`` sits between the two programs'
pins when they run together (transmitter on thread 0, receiver on thread 1).

Expected values come from the NEC description and the TSOP datasheet (the
model's docstring) and from the program headers' stated windows, never from
what the model or the RTL did:

* transmitter: the carrier within +-1 % of 38 kHz over every whole burst, a
  1/3 duty, every burst and space within +-0.5 % of nominal (the program's
  unit is 0.19 % short), frames and repeats decoded, a repeat 108 ms after
  the frame (+-0.5 %);
* receiver: each mark and space accepted within +-25 % of nominal plus
  140.6 us (4 ticks of 35.156 us), glitches shorter than 70 us ignored.

Two boards. Both programs take their tick from the host (the clock is the
board's): at 50 MHz the transmitter's tick is 438 + 153/256 clocks (a third
of a carrier period) and the receiver's 1757 + 208/256 (1/16 of the NEC
unit). A frame and its repeat are then 6 M clocks, about 15 minutes of RTL
simulation at 6,600 clocks a second, so the RTL runs the same programs on a
board clocked at 1.824 MHz, where the ticks are 16 and 64 + 32/256 clocks:
the same waveform in microseconds, 27 times fewer clocks. The 50 MHz bodies
are ``model_only``.
"""

import pytest

from tools.loomasm import assemble_file, assemble_text
from tools.loomisa import REPO, load
from tools.protomodels import ir_nec as N
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


B1824 = Board("1824kHz", 1_824_000, (16, 0), (64, 32))
B50 = Board("50MHz", 50_000_000, (438, 153), (1757, 208))



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
    rec = bench.add(N.EdgeRecorder("OUT0"))
    loom = start(backend, bench, [(program("ir_nec_tx"), 0, board.tx_tick)])
    return bench, loom, rec


def rx_setup(backend, board, gap_us=40_000.0):
    bench = ir_bench(backend)
    src = bench.add(N.IrSource("IN0", board.clock_hz, gap_us=gap_us))
    loom = start(backend, bench, [(program("ir_nec_rx"), 0, board.rx_tick)])
    return bench, loom, src


# ------------------------------------------------------------ transmitter
TX_TOL = 0.005                      # bursts, spaces, start to start
CARRIER_TOL = 0.01
EDGES_FRAME = 2 * (342 + 33 * 22)   # 22 cycles a unit burst, 342 the leader
EDGES_REPEAT = 2 * (342 + 22)


def frame_words(address, command, extended=False):
    """The two INQ words of a frame (program header)."""
    return [(command & 0xFF) | (0x100 if extended else 0), address & 0xFFFF]


def tx_run(bench, rec, board, edges, budget_ms):
    """Run until ``edges`` edges have gone out and the line has been low 2 ms."""
    quiet = board.cycles(2000)
    done = lambda: (len(rec.edges) >= edges
                    and bench.cycle - rec.edges[-1][0] > quiet)
    assert bench.run_until(done, board.cycles(budget_ms * 1000), every=256), (
        "%d of %d edges after %d ms" % (len(rec.edges), edges, budget_ms))


def messages(ca):
    """The bursts grouped into frames and repeat codes: a space longer than
    10 ms (NEC's longest inside a message is 4.5 ms) separates two."""
    groups, gap = [], 10_000.0
    pulses = ca.pulses()
    cur = [pulses[0]]
    for level, us in pulses[1:]:
        if level == N.SPACE and us > gap:
            groups.append(cur)
            cur = []
        else:
            cur.append((level, us))
    groups.append(cur)
    return groups


def check_tx(rec, board, expected):
    """The carrier and the envelope against the specification; ``expected``
    is the list of nominal pulse lists, one per message."""
    ca = N.CarrierAnalysis(rec.edges, board.clock_hz)
    for b in ca.bursts:
        assert abs(ca.frequency(b) / N.CARRIER_HZ - 1) <= CARRIER_TOL, ca.frequency(b)
        assert abs(ca.duty(b) - 1 / 3.0) <= 0.01, ca.duty(b)
    got = messages(ca)
    assert len(got) == len(expected), [len(g) for g in got]
    for g, e in zip(got, expected):
        assert [lv for lv, _ in g] == [lv for lv, _ in e]
        bad = [(i, us, eu) for i, ((_, us), (_, eu)) in enumerate(zip(g, e))
               if abs(us / eu - 1) > TX_TOL]
        assert not bad, "pulses off nominal (index, us, nominal): %r" % bad[:6]
        assert N.decode_pulses(g, tol=TX_TOL)[0].kind in ("frame", "repeat")
    return ca


def check_period(ca, board, first, second):
    """Message ``second`` starts 108 ms after message ``first`` (+-0.5 %)."""
    starts = [b.start for b in ca.bursts if b.cycles >= 300]      # leaders
    us = (starts[second] - starts[first]) * 1e6 / board.clock_hz
    assert abs(us / N.PERIOD_US - 1) <= TX_TOL, us


@model_only("the together scenario runs this frame and repeat through both programs on the RTL and judges the transmitter's pin there too")
def test_tx_frame_and_repeat(backend):
    """A standard frame and one repeat code at the 1.824 MHz board: carrier,
    duty, every burst and space, the decoded frame, the 108 ms period."""
    bench, loom, rec = tx_setup(backend, B1824)
    loom.push(0, frame_words(0x5A, 0xC3) + [0x8000])
    tx_run(bench, rec, B1824, EDGES_FRAME + EDGES_REPEAT, 130)
    ca = check_tx(rec, B1824, [N.frame_pulses(0x5A, 0xC3), N.repeat_pulses()])
    got = N.decode_pulses(messages(ca)[0], tol=TX_TOL)[0]
    assert (got.kind, got.address, got.command, got.extended) == \
        ("frame", 0x5A | (0xA5 << 8), 0xC3, False)
    check_period(ca, B1824, 0, 1)
    assert loom.badop() == 0


@model_only("two frames and two dropped repeat words, 280 ms: a minute of RTL")
def test_tx_chain_rules(backend):
    """A repeat word with no frame before it is dropped; a frame queued
    behind a frame goes out on the next slot, 192 units after the first;
    a frame whose address word comes after its slot starts when the word
    does; once a slot has found INQ empty the chain is over, and a repeat
    word pushed after it is dropped too."""
    bench, loom, rec = tx_setup(backend, B1824)
    loom.push(0, [0x8000])
    bench.step(B1824.cycles(5_000))
    assert rec.edges == []
    loom.push(0, frame_words(0x01, 0x02) + frame_words(0x1234, 0x56, extended=True))
    tx_run(bench, rec, B1824, 2 * EDGES_FRAME, 240)
    ca = check_tx(rec, B1824, [N.frame_pulses(0x01, 0x02),
                               N.frame_pulses(0x1234, 0x56, extended=True)])
    check_period(ca, B1824, 0, 1)
    third = frame_words(0x77, 0x88)
    loom.push(0, third[:1])                      # before the second frame's slot
    bench.step(B1824.cycles(60_000))             # past it: the address is late
    assert len(rec.edges) == 2 * EDGES_FRAME
    late = bench.cycle
    loom.push(0, third[1:])
    tx_run(bench, rec, B1824, 3 * EDGES_FRAME, 100)
    ca = N.CarrierAnalysis(rec.edges, B1824.clock_hz)
    assert N.decode_pulses(messages(ca)[2], tol=TX_TOL)[0].command == 0x88
    assert ca.bursts[2 * 34].start > late        # it waited for the word
    edges = len(rec.edges)
    bench.step(B1824.cycles(60_000))             # past the third frame's slot
    loom.push(0, [0x8000])
    bench.step(B1824.cycles(20_000))
    assert len(rec.edges) == edges
    assert loom.badop() == 0


@model_only("1 M clocks at 50 MHz, two and a half minutes of RTL; the "
            "1.824 MHz twin of the whole frame runs there")
def test_tx_carrier_at_50mhz(backend):
    """The fractional tick at 50 MHz, over the leader, its space and four
    bits (1, 0, 1, 0): the carrier within 1 % of 38 kHz over every whole
    burst (37,999.9 Hz), a 1/3 duty, and every carrier edge on a clock edge,
    so a high lasts 438 or 439 clocks and a period 1315 or 1316 (one clock,
    20 ns, of jitter); every burst and space within 0.5 % of nominal. The
    program's schedule is in ticks, so the rest of the frame and the repeat
    are the 1.824 MHz runs again."""
    ca = leader_and_four_bits(backend, B50)
    assert {h for b in ca.bursts for h in b.highs} == {438, 439}
    assert {p for b in ca.bursts for p in b.periods} == {1315, 1316}
    lead = ca.bursts[0]
    assert abs(ca.frequency(lead) - 37_999.9) < 0.5, ca.frequency(lead)


def leader_and_four_bits(backend, board):
    """Send address 0x05 (bits 1, 0, 1, 0 first) and check the carrier and
    the envelope up to bit 4's burst; returns the analysis."""
    bench, loom, rec = tx_setup(backend, board)
    loom.push(0, frame_words(0x05, 0x00))
    edges = 2 * (342 + 5 * 22)                   # leader, four bits, bit 4's burst
    assert bench.run_until(lambda: len(rec.edges) >= edges, board.cycles(25_000),
                           every=1024)
    ca = N.CarrierAnalysis(rec.edges[:edges], board.clock_hz)
    assert [b.cycles for b in ca.bursts] == [342] + [22] * 5
    for b in ca.bursts:
        assert abs(ca.frequency(b) / N.CARRIER_HZ - 1) <= CARRIER_TOL
        assert abs(ca.duty(b) - 1 / 3.0) <= 0.01
    nominal = N.frame_pulses(0x05, 0x00)[:len(ca.pulses())]
    for (lv, us), (nlv, nus) in zip(ca.pulses(), nominal):
        assert lv == nlv and abs(us / nus - 1) <= TX_TOL, (us, nus)
    assert loom.badop() == 0
    return ca


def test_tx_leader_and_four_bits(backend):
    """The PHY-GL-SDF candidate (20 ms, 37,000 clocks at the 1.824 MHz
    board): the leader, its space and four bits, judged on the pin only."""
    ca = leader_and_four_bits(backend, B1824)
    assert {h for b in ca.bursts for h in b.highs} == {16}
    assert {p for b in ca.bursts for p in b.periods} == {48}


# --------------------------------------------------------------- receiver
TICK_US = N.UNIT_US / 16            # the receiver's tick
ALLOW_US = 4 * TICK_US              # the windows' 4 ticks beyond +-25 %
REP, LEAD, SPACE = 16, 8, 4         # error word item bits (program header)
WIN, LONG, INV = 1, 2, 3            # fault bits


def report_frame(address, command, extended=False):
    """What the receiver reports for a frame (program header)."""
    if not extended:
        address = (address & 0xFF) | ((~address & 0xFF) << 8)
    return ("frame", (command & 0xFF) | (0x100 if extended else 0), address)


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
            out.append(("frame", w & 0x1FF, words.pop(0)))
        elif kind == 2:
            out.append(("repeat",) if w == 0x8000 else ("bad", w))
        else:
            out.append(("error", w & 0x3FFF) if kind == 3 else ("bad", w))
        if not words:
            words = loom.pop_available(thread)
    return out


def rx_send(bench, loom, src, board, lists, gap_us=40_000.0, tail_us=1000.0):
    """Send pulse lists ``gap_us`` apart (the first after 2 ms of idle line);
    after each, run ``tail_us`` and collect the reports, so OUTQ never fills."""
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
    """Every mark and space at the edge of its window: +-25 % and the 4
    ticks, less the measurement margin."""
    return [(lv, us * (1 + 0.25 * sign) + sign * (ALLOW_US - margin_us(board)))
            for lv, us in pulses]


@model_only("the together scenario runs this frame and repeat through both programs on the RTL and judges the transmitter's pin there too")
def test_rx_frame_and_repeat(backend):
    """A frame and its repeat code 108 ms later, at nominal timing."""
    bench, loom, src = rx_setup(backend, B1824)
    frame = N.frame_pulses(0x5A, 0xC3)
    got = rx_send(bench, loom, src, B1824, [frame, N.repeat_pulses()],
                  gap_us=N.PERIOD_US - sum(us for _, us in frame))
    assert got == [report_frame(0x5A, 0xC3), ("repeat",)]
    assert loom.badop() == 0


def test_rx_repeat_code(backend):
    """The PHY-GL-SDF candidate (14 ms, 26,000 clocks at the 1.824 MHz
    board): one repeat code at nominal timing."""
    bench, loom, src = rx_setup(backend, B1824)
    assert rx_send(bench, loom, src, B1824, [N.repeat_pulses()]) == [("repeat",)]
    assert loom.badop() == 0


@model_only("four messages, a minute of RTL; the nominal frame runs there")
def test_rx_tolerance_edges(backend):
    """Every mark and space of a frame and of a repeat code at the short and
    at the long edge of its window. (A receiver module's -5/f0 and +5/f0
    pulse errors are the together tests' business.)"""
    bench, loom, src = rx_setup(backend, B1824)
    lists = [edge(N.frame_pulses(0x10, 0xEF), -1, B1824),
             edge(N.frame_pulses(0x81, 0x7E), +1, B1824),
             edge(N.repeat_pulses(), -1, B1824), edge(N.repeat_pulses(), +1, B1824)]
    assert rx_send(bench, loom, src, B1824, lists) == [
                             report_frame(0x10, 0xEF), report_frame(0x81, 0x7E),
                             ("repeat",), ("repeat",)]


def with_item(pulses, index, us):
    """``pulses`` with item ``index`` lasting ``us``, cut after the item that
    follows it (the receiver has given up by then)."""
    out = list(pulses[:index + 2])
    out[index] = (out[index][0], us)
    return out


def bad_inverse(address, command):
    bits = N.frame_bits(address, command)
    bits[31] ^= 1                           # the inverse's top bit
    return N.bits_pulses(bits)


BIT3, BIT7 = N.bit_index(3), N.bit_index(7, N.SPACE)
F = N.frame_pulses(0x21, 0x43)              # bit 3 of 0x21 is 0, bit 7 is 0
ERRORS = [
    # (what, pulses, the error word's n, item and fault)
    ("leader mark 11.6 ms", with_item(F, 0, 11_600), (0, LEAD, LONG)),
    ("leader space 1.4 ms", with_item(F, 1, 1_400), (0, LEAD | SPACE, WIN)),
    ("leader space 3.1 ms", with_item(F, 1, 3_100), (0, LEAD | SPACE, WIN)),
    ("leader space 6 ms", with_item(F, 1, 6_000), (0, LEAD | SPACE, LONG)),
    ("bit 3 mark 250 us", with_item(F, BIT3, 250), (3, 0, WIN)),
    ("bit 3 mark 900 us", with_item(F, BIT3, 900), (3, 0, LONG)),
    ("bit 7 space 250 us", with_item(F, BIT7, 250), (7, SPACE, WIN)),
    ("bit 7 space 960 us", with_item(F, BIT7, 960), (7, SPACE, WIN)),
    ("bit 7 space 2.4 ms", with_item(F, BIT7, 2_400), (7, SPACE, LONG)),
    ("final burst 900 us", with_item(F, 66, 900), (32, 0, LONG)),
    ("repeat burst 900 us", with_item(N.repeat_pulses(), 2, 900), (32, REP, LONG)),
    ("command inverse", bad_inverse(0x21, 0x43), (32, 0, INV)),
    ("cut after bit 10's mark", N.truncate(F, N.bit_index(10) + 1), (10, SPACE, LONG)),
]


@model_only("thirteen faulty frames, a minute of RTL")
def test_rx_errors_and_recovery(backend):
    """Each fault is one error word saying where it was and what; a good
    frame after them all is received."""
    bench, loom, src = rx_setup(backend, B1824)
    lists = [pulses for _, pulses, _ in ERRORS] + [F]
    got = rx_send(bench, loom, src, B1824, lists, gap_us=5_000.0)
    assert got == [report_error(*e) for _, _, e in ERRORS] + [report_frame(0x21, 0x43)]


def glitched(pulses, glitches):
    """``glitches`` = [(index, offset us, width us)], applied last first so
    the indices stay those of ``pulses``."""
    for index, offset, width in sorted(glitches, reverse=True):
        pulses = N.glitch(pulses, N.start_of(pulses, index) + offset, width)
    return pulses


def test_rx_glitches_are_rejected(backend):
    """60 us glitches of both levels, in the leader, a mark and the spaces
    of a 0 and a 1, and a 40 us and a 1 ms pulse on the idle line, change
    nothing; a 150 us one (longer than the 70 us filter) inside a space is
    an error."""
    bench, loom, src = rx_setup(backend, B1824)
    frame = N.frame_pulses(0x96, 0x69)          # bits 0 and 1 of 0x96: 0, 1
    good = glitched(frame, [(0, 4_000, 60), (1, 2_000, 60), (N.bit_index(4), 250, 60),
                            (N.bit_index(0, N.SPACE), 250, 60),
                            (N.bit_index(1, N.SPACE), 1_000, 60)])
    bad = glitched(frame, [(N.bit_index(1, N.SPACE), 900, 150)])[:N.bit_index(2) + 2]
    lists = [[(N.MARK, 40.0)], [(N.MARK, 1_000.0)], good, bad]
    got = rx_send(bench, loom, src, B1824, lists, gap_us=5_000.0)
    assert got == [report_frame(0x96, 0x69), report_error(1, SPACE, WIN)]


# ---------------------------------------------------------------- together
def both_setup(backend, board, **tsop):
    """The transmitter on thread 0, a receiver module from OUT0 to IN0, the
    receiver on thread 1 (its source with `.thread 1`, at 0x080)."""
    bench = ir_bench(backend)
    rec = bench.add(N.EdgeRecorder("OUT0"))
    module = bench.add(N.TsopModel("OUT0", "IN0", board.clock_hz, **tsop))
    loom = start(backend, bench, [(program("ir_nec_tx"), 0, board.tx_tick),
                                  (program("ir_nec_rx", 1), 1, board.rx_tick)])
    return bench, loom, rec, module


def test_together_frame_and_repeat(backend):
    """A frame and its repeat code from thread 0 through a receiver module
    (td 10 carrier periods, output pulses 3 periods long, 5 us of jitter on
    every edge) to thread 1, at the 1.824 MHz board."""
    bench, loom, rec, module = both_setup(backend, B1824, td=10, delta=3,
                                          jitter_us=5, seed=1)
    loom.push(0, frame_words(0xE7, 0x18) + [0x8000])
    tx_run(bench, rec, B1824, EDGES_FRAME + EDGES_REPEAT, 130)
    ca = check_tx(rec, B1824, [N.frame_pulses(0xE7, 0x18), N.repeat_pulses()])
    check_period(ca, B1824, 0, 1)
    assert len(module.events) == 2 * (34 + 2)       # one output pulse a burst
    assert reports(loom, 1) == [report_frame(0xE7, 0x18), ("repeat",)]
    assert loom.badop() == 0


@model_only("a frame and a repeat code per receiver module, 30 s of RTL each")
@pytest.mark.parametrize("td,delta", [(7, -5), (15, 6)])
def test_together_receiver_module_limits(backend, td, delta):
    """A receiver module at the TSOP382.. datasheet's short limits (td 7
    carrier periods, output pulses 5 periods short) and beyond its long
    ones (td 15 and pulses 6 periods long, the older TSOP18238 sheet's
    figures; TSOP382.. allows 13 and 5), 10 us of jitter; an extended
    frame."""
    bench, loom, rec, module = both_setup(backend, B1824, td=td, delta=delta,
                                          jitter_us=10, seed=td)
    loom.push(0, frame_words(0x1234, 0x56, extended=True) + [0x8000])
    tx_run(bench, rec, B1824, EDGES_FRAME + EDGES_REPEAT, 130)
    assert reports(loom, 1) == [report_frame(0x1234, 0x56, extended=True), ("repeat",)]


# ------------------------------------------------------- at 50 MHz, the rest
@model_only("0.7 M clocks at 50 MHz, two minutes of RTL; the 1.824 MHz "
            "frame runs there")
def test_rx_at_50mhz(backend):
    """The receiver at its 50 MHz tick (1757 + 208/256 clocks; a glitch
    filter of 874 + 4 slots): a frame with every mark and space at the short
    edge of its window, cut after bit 3's mark, gives the timeout at bit 3."""
    bench, loom, src = rx_setup(backend, B50)
    cut = N.truncate(N.frame_pulses(0x05, 0x00), N.bit_index(3) + 1)
    got = rx_send(bench, loom, src, B50, [edge(cut, -1, B50)], tail_us=3000.0)
    assert got == [report_error(3, SPACE, LONG)]


# -------------------------------------------------------------- the programs
def test_programs_assemble_strict_with_no_diagnostic():
    """L2-DEADLINE: the transmitter's 19 pairs proved at a 16-clock tick with
    8 clocks of slack, its two INQ tests' POPs declared bounded; the receiver
    has no WAITD and no pair. Both fit a quarter of the 512-word memory, and
    the receiver moved to thread 1 stays in that quarter."""
    tx, rx = program("ir_nec_tx"), program("ir_nec_rx")
    assert tx.diagnostics == [] and rx.diagnostics == []
    assert (len(tx.words), len(rx.words)) == (71, 112)
    t = tx.deadlines[0]
    assert t.period == 16 and not t.infeasible and not t.unbounded
    assert len(t.pairs) == 19 and t.worst_slack == 8
    assert [(d.name, d.addr) for d in t.declarations] == [("POP", 0x037), ("POP", 0x03C)]
    assert all("only this thread pops INQ" in d.reason for d in t.declarations)
    r = rx.deadlines[0]
    assert r.period == 64 and r.pairs == [] and not r.unbounded
    assert max(tx.words) < 0x80 and max(rx.words) < 0x80
    moved = program("ir_nec_rx", 1)
    assert moved.diagnostics == []
    assert min(moved.words) == 0x80 and max(moved.words) < 0x100


# --------------------------------------------------------------- the model
def test_model_encoder_and_decoder():
    p = N.frame_pulses(0x04, 0x08)
    assert len(p) == 67 and sum(us for _, us in p) == 121 * N.UNIT_US
    m = N.decode_pulses(p)[0]
    assert (m.kind, m.address, m.command, m.extended) == ("frame", 0xFB04, 0x08, False)
    assert N.decode_pulses(N.frame_pulses(0x1234, 0x56, True))[0].extended
    assert N.decode_pulses(N.repeat_pulses())[0].kind == "repeat"
    assert N.decode_pulses(N.vary(p, mark=1.3, space=1.3)) == []   # no leader
    assert N.decode_pulses(bad_inverse(4, 8))[0].reason == "command inverse"
    assert N.decode_pulses(edge(p, -1, B50), extra_us=ALLOW_US)[0].kind == "frame"


def test_model_receiver_module_timing():
    """A 22-cycle burst through TsopModel: the output falls td after the
    first rise and rises td + delta after the last fall."""
    clock, period = 1_000_000, 1_000_000 / N.CARRIER_HZ          # 26.3 clocks
    module = N.TsopModel("OUT0", "IN0", clock, td=10, delta=-3)
    lines, drive = N.Lines(), N.Drive()
    rises = [int(round(100 + k * period)) for k in range(22)]
    falls = [r + 9 for r in rises]
    level = 0
    for cycle in range(2000):
        drive.clear()
        module.drive(drive, cycle)
        level = 1 if cycle in rises else 0 if cycle in falls else level
        lines.cycle, lines.uo = cycle, level
        module.observe(lines)
    (fall, lo), (rise, hi) = module.events
    assert (lo, hi) == (0, 1)
    assert abs(fall - (rises[0] + 10 * period)) <= 2
    assert abs(rise - (falls[-1] + 7 * period)) <= 2
