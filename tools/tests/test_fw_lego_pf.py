"""L3 for the LEGO Power Functions infrared programs, on the model and the RTL.

``firmware/lego_pf_tx.loom`` (thread 0, LED = OUT0) makes the 38 kHz carrier,
the LRC and the five channel-spaced copies itself; ``firmware/lego_pf_rx.loom``
(RX = IN0) decodes a receiver module's output, checks the LRC and drops the
copies. The references are ``tools.protomodels.lego_pf``, written from LEGO's
"Power Functions RC" document v1.20: the encoder, the decoder with the
document's windows, the schedule checker, and ``IrSource`` / ``TsopModel``
from ``ir_nec`` for the demodulated line.

Expected values come from the document and the program headers' stated
choices, never from what the model or the RTL did:

* transmitter: the carrier within 1 % of 38 kHz in every burst, duty 1/3,
  six cycles a mark, 18 marks a message, every mark start within one clock
  of the document's 16 / 27 / 45-cycle grid (and so inside the receiver
  windows), the LRC, the first delay (4 - Ch) tm and the spacings 5 tm and
  (6 + 2 Ch) tm with tm = 16 ms, five copies, HOLD, pre-emption;
* receiver: the windows 316 .. 526, 526 .. 947 and 947 .. 1579 us from the
  document, a bad LRC dropped, a copy dropped, a receiver module's output
  pulse from 1 to 11 carrier periods accepted (only mark starts are timed).

Two boards. Both programs take their tick from the host: at 50 MHz the
transmitter's tick is 438 + 153/256 clocks (a third of a carrier period),
the receiver's 1315 + 202/256 (a carrier period). The RTL runs the programs
on a board clocked at 1.824 MHz, where the ticks are 16 and 48 clocks: the
same waveform in microseconds, 27 times fewer clocks. One message is about
22 k clocks there; a full five-copy sequence is 0.5 to 0.7 s, about 1 M
clocks, so the sequences, and the 50 MHz bodies, are ``model_only``.

Every body here is named ``test_lego_*``: test/test_fw.py collects the
bodies of all modules into one namespace.
"""

import pytest

from tools.loomasm import assemble_file, assemble_text
from tools.loomisa import REPO, load
from tools.protomodels import lego_pf as L
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

    def us(self, clocks):
        return clocks * 1e6 / self.clock_hz

    def __repr__(self):
        return self.name


B1824 = Board("1824kHz", 1_824_000, (16, 0), (48, 0))
B50 = Board("50MHz", 50_000_000, (438, 153), (1315, 202))


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
    rec = bench.add(L.EdgeRecorder("OUT0"))
    loom = start(backend, bench, [(program("lego_pf_tx"), 0, board.tx_tick)])
    return bench, loom, rec


def rx_setup(backend, board, gap_us=5_000.0):
    bench = ir_bench(backend)
    src = bench.add(L.IrSource("IN0", board.clock_hz, gap_us=gap_us))
    loom = start(backend, bench, [(program("lego_pf_rx"), 0, board.rx_tick)])
    return bench, loom, src


# ------------------------------------------------------------ transmitter
CARRIER_TOL = 0.01
EDGES_PER_MESSAGE = 2 * L.MARK_CYCLES * L.MARKS          # 216
SETUP_US = 300.0          # the first delay's set-up: K = 4 cycles plus the code


def host_word(word, hold=False, junk=0b1010):
    """The INQ word for a message (program header): bits 15:4 as sent,
    bit 0 HOLD, bits 3:1 ignored (``junk`` shows it)."""
    return (word & 0xFFF0) | (junk & 0b1110) | int(hold)


def tx_run(bench, rec, board, messages, budget_ms, quiet_ms=3.0):
    """Run until ``messages`` messages have gone out and the line has been
    dark ``quiet_ms``."""
    edges = messages * EDGES_PER_MESSAGE
    quiet = board.cycles(quiet_ms * 1000)
    done = lambda: (len(rec.edges) >= edges
                    and bench.cycle - rec.edges[-1][0] > quiet)
    assert bench.run_until(done, board.cycles(budget_ms * 1000), every=256), (
        "%d of %d edges after %d ms" % (len(rec.edges), edges, budget_ms))


def check_tx(rec, board, words, tol_us=None):
    """Every burst's carrier, every message's marks against the encoding;
    ``words`` the expected messages in order. Returns their start times."""
    tol = 1e6 / board.clock_hz if tol_us is None else tol_us    # one clock
    ca = L.CarrierAnalysis(rec.edges, board.clock_hz)
    for b in ca.bursts:
        assert b.cycles == L.MARK_CYCLES
        assert abs(ca.frequency(b) / L.CARRIER_HZ - 1) <= CARRIER_TOL, ca.frequency(b)
        assert abs(ca.duty(b) - 1 / 3.0) <= 0.01, ca.duty(b)
    groups = L.group_bursts(ca.starts_us())
    assert len(groups) == len(words), [len(g) for g in groups]
    for g, w in zip(groups, words):
        bad = L.check_marks(g, w, tol)
        assert not bad, "marks off (index, got, nominal): %r" % bad[:6]
        got = L.decode_starts(g)
        assert [(m.word, m.ok) for m in got] == [(w, True)]
    return [g[0] for g in groups]


#: Messages with many ones (the longest) and many zeros, in four modes.
WORDS = {
    "combo_pwm": L.combo_pwm(4, 0xF, 0xE),
    "single_pwm": L.single_pwm(4, 1, 0x9, toggle=1),
    "combo_direct": L.combo_direct(4, 0, 1),
    "extended": L.extended(4, 0b0111, toggle=1),
}


@pytest.mark.parametrize("mode", ["combo_pwm", "combo_direct"])
def test_lego_tx_message(backend, mode):
    """The first copy of a channel-4 message (no first delay) at the
    1.824 MHz board: 18 bursts of six 38 kHz cycles at duty 1/3, every mark
    start on the document's grid to one clock, the LRC computed by the
    program (the host's bits 3:1 ignored), the decoded word."""
    bench, loom, rec = tx_setup(backend, B1824)
    w = WORDS[mode]
    loom.push(0, [host_word(w)])
    tx_run(bench, rec, B1824, 1, 40)
    check_tx(rec, B1824, [w])
    ca = L.CarrierAnalysis(rec.edges, B1824.clock_hz)
    assert {h for b in ca.bursts for h in b.highs} == {16}
    assert {p for b in ca.bursts for p in b.periods} == {48}
    assert loom.badop() == 0


@model_only("five copies over 0.46 to 0.61 s, 0.85 to 1.1 M clocks, three "
            "minutes of RTL per channel; the first copy runs on the RTL")
@pytest.mark.parametrize("channel", [1, 4])
def test_lego_tx_five_copies_spaced_by_channel(backend, channel):
    """Five copies, then silence (channels 1 and 4, the two ends of each
    formula; a minute of model time each): the first (4 - Ch) tm after the word
    (plus the set-up), then 5 tm, 5 tm, (6 + 2 Ch) tm, (6 + 2 Ch) tm start
    to start, to one clock."""
    bench, loom, rec = tx_setup(backend, B1824)
    w = L.single_pwm(channel, 0, 3, toggle=channel & 1)
    loom.push(0, [host_word(w)])
    t0 = B1824.us(bench.cycle)
    budget = sum(L.schedule_tm(channel)) * 16 + 360
    tx_run(bench, rec, B1824, 5, budget, quiet_ms=300)  # no sixth copy
    assert len(rec.edges) == 5 * EDGES_PER_MESSAGE
    starts = check_tx(rec, B1824, [w] * 5)
    bad = L.check_schedule(starts, channel, tol_us=1.0, first_from_us=t0,
                           first_extra_us=SETUP_US)
    assert not bad, bad
    assert loom.badop() == 0


@model_only("0.2 M clocks (100 s of RTL, measured 2026-11-07, where it "
            "passed); off the RTL set to keep it near 770 k clocks")
def test_lego_tx_new_word_preempts_the_copies(backend):
    """A word queued after the first copy of another ends that sequence at
    the second copy's slot (5 tm after the first): the new word's first copy
    goes out there instead, after its own first delay (0 on channel 4)."""
    bench, loom, rec = tx_setup(backend, B1824)
    a, b = WORDS["combo_pwm"], WORDS["combo_direct"]
    loom.push(0, [host_word(a)])
    tx_run(bench, rec, B1824, 1, 40)
    loom.push(0, [host_word(b)])
    tx_run(bench, rec, B1824, 2, 120)
    s = check_tx(rec, B1824, [a, b])
    slot = s[0] + 5 * L.TM_US
    assert slot <= s[1] <= slot + SETUP_US, (s[1] - s[0]) / L.TM_US
    assert loom.badop() == 0


@model_only("eight copies and a release over 0.77 s, 1.4 M clocks; the "
            "pre-emption runs on the RTL")
def test_lego_tx_hold_stream_until_the_next_word(backend):
    """HOLD on channel 1: after the five copies the message goes on at the
    fourth-to-fifth spacing, (6 + 2 Ch) tm = 8 tm, until a word is queued;
    the release word then takes the next slot and its own first delay,
    (4 - Ch) tm = 3 tm."""
    bench, loom, rec = tx_setup(backend, B1824)
    a = L.combo_direct(1, 1, 2, toggle=1)
    b = L.combo_direct(1, 0, 0, toggle=0)
    loom.push(0, [host_word(a, hold=True)])
    tx_run(bench, rec, B1824, 6, 37 * 16 + 40)
    loom.push(0, [host_word(b)])
    tx_run(bench, rec, B1824, 7, 12 * 16 + 40)
    s = check_tx(rec, B1824, [a] * 6 + [b])
    assert not L.check_schedule(s[:6], 1, tol_us=1.0)
    first = s[5] + 8 * L.TM_US + 3 * L.TM_US
    assert first <= s[6] <= first + SETUP_US, (s[6] - s[5]) / L.TM_US
    assert loom.badop() == 0


@model_only("0.1 M clocks at 50 MHz; the 1.824 MHz twin runs on the RTL")
def test_lego_tx_carrier_at_50mhz(backend):
    """At 50 MHz with TICK 438 + 153/256 (program header): the start mark
    and two data marks; the carrier within 10 ppm of 38 kHz over each burst,
    mark starts within 0.05 us of the document's grid."""
    bench, loom, rec = tx_setup(backend, B50)
    w = WORDS["single_pwm"]                             # bits 15, 14: 1, 0
    loom.push(0, [host_word(w)])
    edges = 3 * 2 * L.MARK_CYCLES
    assert bench.run_until(lambda: len(rec.edges) >= edges, B50.cycles(5_000),
                           every=256)
    ca = L.CarrierAnalysis(rec.edges[:edges], B50.clock_hz)
    assert [b.cycles for b in ca.bursts] == [6, 6, 6]
    for b in ca.bursts:
        assert abs(ca.frequency(b) / L.CARRIER_HZ - 1) < 1e-5
    s = ca.starts_us()
    nominal = L.mark_starts(w)[:3]
    assert all(abs((g - s[0]) - n) < 0.05 for g, n in zip(s, nominal)), s
    assert loom.badop() == 0


# ---------------------------------------------------------------- receiver
MARK_US = L.MARK_CYCLES * L.CYCLE_US                    # 157.9 us


def rx_send(bench, loom, src, board, lists, gap_us=5_000.0, tail_us=3_000.0):
    """Send pulse lists ``gap_us`` apart, which must be more than
    ``tail_us`` (the first after 2 ms of idle line); after each, run
    ``tail_us`` (past the receiver's 1.6 ms stop-bit wait) and collect what
    it pushed, so OUTQ never fills."""
    out = []
    for i, pulses in enumerate(lists):
        at = bench.cycle + board.cycles(2000) if i == 0 else None
        src.send(pulses, at=at, gap_us=gap_us)
        end = src.free_at + board.cycles(tail_us)
        bench.run_until(lambda: bench.cycle >= end, end - bench.cycle + 1, every=256)
        out += loom.pop_available(0)
    return out


def from_intervals(intervals_us, mark_us=MARK_US):
    """A demodulated message from its 17 mark-to-mark intervals."""
    out = []
    for us in intervals_us:
        out += [(L.MARK, mark_us), (L.SPACE, us - mark_us)]
    return out + [(L.MARK, mark_us)]


def nominal_intervals(word):
    return [n * L.CYCLE_US for n in L.interval_cycles(word)]


def test_lego_rx_messages(backend):
    """Two messages at nominal timing (combo PWM with most bits high,
    extended mode with most low), each pushed as its 16 bits."""
    bench, loom, src = rx_setup(backend, B1824)
    a, b = WORDS["combo_pwm"], WORDS["extended"]
    got = rx_send(bench, loom, src, B1824, [L.message_pulses(a), L.message_pulses(b)])
    assert got == [a, b]
    assert loom.badop() == 0


def test_lego_rx_bad_lrc_dropped(backend):
    """A message with one LRC bit wrong, and one with a data bit flipped
    (its LRC no longer matches), are dropped; the good one after them is
    delivered."""
    bench, loom, src = rx_setup(backend, B1824)
    w = WORDS["single_pwm"]
    lists = [L.message_pulses(w ^ 0x0001), L.message_pulses(w ^ 0x0100),
             L.message_pulses(w)]
    assert [L.lrc_ok(x) for x in (w ^ 1, w ^ 0x100, w)] == [False, False, True]
    assert rx_send(bench, loom, src, B1824, lists) == [w]
    assert loom.badop() == 0


def test_lego_rx_copy_with_the_same_toggle_dropped(backend):
    """The same message twice (a copy: the same toggle) is delivered once;
    the next command, with the toggle changed, is delivered."""
    bench, loom, src = rx_setup(backend, B1824)
    a = L.single_pwm(2, 0, 5, toggle=0)
    b = L.single_pwm(2, 0, 5, toggle=1)
    lists = [L.message_pulses(a), L.message_pulses(a), L.message_pulses(b)]
    assert rx_send(bench, loom, src, B1824, lists) == [a, b]
    assert loom.badop() == 0


def test_lego_rx_receiver_module_pulse_width(backend):
    """A receiver module's output pulse is the burst length -5/f0 .. +5/f0
    (TSOP382.. figures, as protomodels/ir_nec reads them): a six-cycle mark
    comes out 1 to 11 cycles long. Only mark starts are timed, so both ends
    decode: marks 5 cycles longer (290 us) and 4.5 shorter (39 us)."""
    bench, loom, src = rx_setup(backend, B1824)
    a, b = WORDS["combo_pwm"], WORDS["combo_direct"]
    lists = [L.tsop_distort(L.message_pulses(a), 5 * L.CYCLE_US),
             L.tsop_distort(L.message_pulses(b), -4.5 * L.CYCLE_US)]
    assert [m.word for m in L.decode_pulses(lists[0])] == [a]
    assert rx_send(bench, loom, src, B1824, lists) == [a, b]
    assert loom.badop() == 0


def margin_us(board):
    """How close to a window edge a test goes: the receiver sees an edge
    within about 30 clocks (both ends of an interval), and its window edges
    are whole ticks, within 0.5 us of the document's figures."""
    return max(10.0, 30 * 1e6 / board.clock_hz) + 0.5


def at_edges(word, sign, board):
    """``word``'s intervals moved to the short (-1) or long (+1) edge of
    their windows, less the margin."""
    m = margin_us(board)
    want = ["startstop"] + ["high" if b else "low" for b in L.bits_of(word)]
    win = {"low": L.LOW_US, "high": L.HIGH_US, "startstop": L.STARTSTOP_US}
    return [(win[c][1] - m) if sign > 0 else (win[c][0] + m) for c in want]


@model_only("six messages, 0.15 M clocks; the nominal ones run on the RTL")
def test_lego_rx_window_edges(backend):
    """Every interval at the short and at the long edge of the document's
    window decodes; one interval just outside loses the message: a data
    bit under 316 us, a start bit under 947 us, a data bit over 947 us
    (taken as a new start bit, so the message is lost), and a mark 947 us
    too early after the stop mark."""
    bench, loom, src = rx_setup(backend, B1824)
    w = WORDS["single_pwm"]
    m = margin_us(B1824)
    short, long_ = at_edges(w, -1, B1824), at_edges(w, +1, B1824)
    bad_short = list(short)
    bad_short[5] = L.LOW_US[0] - m
    bad_start = list(long_)
    bad_start[0] = L.STARTSTOP_US[0] - m
    bad_long = list(short)
    bad_long[9] = L.HIGH_US[1] + m
    lists = [from_intervals(i) for i in (short, long_, bad_short, bad_start, bad_long)]
    for p, want in zip(lists, [[w], [w], [], [], []]):
        assert [x.word for x in L.decode_pulses(p) if x.ok] == want
    got = rx_send(bench, loom, src, B1824, lists[:2])
    assert got == [w]                               # the second is a copy
    c = L.message(w ^ 0x8000)                       # toggled: delivered
    lists = [from_intervals(at_edges(c, -1, B1824))] + lists[2:]
    d = L.message(w ^ 0x0010)                       # another valid message
    early = L.message_pulses(d) + [(L.SPACE, 900.0 - MARK_US), (L.MARK, MARK_US)]
    assert L.decode_pulses(early) == [] and L.decode_pulses(L.message_pulses(d))
    got = rx_send(bench, loom, src, B1824, lists + [early])
    assert got == [c]
    assert loom.badop() == 0


@model_only("three messages, 0.07 M clocks; the RTL runs the nominal ones")
def test_lego_rx_glitches_are_rejected(backend):
    """Pulses shorter than the one-tick (26 us) filter, a 12 us dark gap in
    a mark and a 12 us mark in a space, change nothing."""
    bench, loom, src = rx_setup(backend, B1824)
    w = WORDS["combo_pwm"]
    p = L.message_pulses(w)
    g = L.glitch(L.glitch(p, 60.0, 12.0), L.mark_starts(w)[3] + 300.0, 12.0)
    g = L.glitch(g, L.mark_starts(w)[10] + 80.0, 12.0)
    assert len(g) == len(p) + 6                     # all three landed
    assert rx_send(bench, loom, src, B1824, [g]) == [w]
    assert loom.badop() == 0


@model_only("0.5 s of line, 0.9 M clocks")
def test_lego_rx_forgets_the_last_message_after_dark(backend):
    """A copy after 230 ms of dark line (more than the longest dark time in
    a sequence, 216 ms) is still dropped; after 260 ms (more than the
    program's 241.6 ms) it is delivered again."""
    assert L.MAX_DARK_US < 230_000 < L.FORGET_US < 260_000 - 2_000
    bench, loom, src = rx_setup(backend, B1824)
    w = WORDS["combo_direct"]
    p = L.message_pulses(w)
    got = rx_send(bench, loom, src, B1824, [p])
    got += rx_send(bench, loom, src, B1824, [p], gap_us=230_000)
    got += rx_send(bench, loom, src, B1824, [p], gap_us=260_000)
    assert got == [w, w]
    assert loom.badop() == 0


@model_only("0.75 M clocks at 50 MHz; the 1.824 MHz twin runs on the RTL")
def test_lego_rx_at_50mhz(backend):
    """At 50 MHz with TICK 1315 + 202/256 (program header): a message."""
    bench, loom, src = rx_setup(backend, B50)
    w = WORDS["extended"]
    assert rx_send(bench, loom, src, B50, [L.message_pulses(w)]) == [w]
    assert loom.badop() == 0


# ---------------------------------------------------------------- together
def both_setup(backend, board, **tsop):
    """The transmitter on thread 0, a receiver module from OUT0 to IN0, the
    receiver on thread 1 (its source with `.thread 1`, at 0x080)."""
    bench = ir_bench(backend)
    rec = bench.add(L.EdgeRecorder("OUT0"))
    module = bench.add(L.TsopModel("OUT0", "IN0", board.clock_hz, **tsop))
    loom = start(backend, bench, [(program("lego_pf_tx"), 0, board.tx_tick),
                                  (program("lego_pf_rx", 1), 1, board.rx_tick)])
    return bench, loom, rec, module


@pytest.mark.parametrize("td,delta", [(3, 0), (1, -4), (5, 5)])
def test_lego_together_through_a_receiver_module(backend, td, delta):
    """A channel-4 command from thread 0 through a receiver module to
    thread 1, at the 1.824 MHz board. The module (protomodels/ir_nec
    TsopModel) must answer a six-cycle burst, so its delay td is 1 to 5
    carrier periods, below the TSOP382.. figure of 7 or more (QUESTIONS.md);
    its output pulse is the burst plus delta periods (-4 .. +5), with 5 us
    of jitter on every edge. The pin judged as in the transmitter tests, one
    output pulse a mark, the message pushed on thread 1."""
    bench, loom, rec, module = both_setup(backend, B1824, td=td, delta=delta,
                                          jitter_us=5, seed=td)
    w = WORDS["single_pwm"]
    loom.push(0, [host_word(w)])
    tx_run(bench, rec, B1824, 1, 40)
    check_tx(rec, B1824, [w])
    assert len(module.events) == 2 * L.MARKS
    assert loom.pop_available(1) == [w]
    assert loom.badop() == 0


@model_only("three copies over 0.17 s, 0.3 M clocks; one copy runs on the RTL")
def test_lego_together_copies_delivered_once(backend):
    """The first three copies of a channel-4 command, 5 tm apart, reach the
    receiver; it pushes the message once."""
    bench, loom, rec, module = both_setup(backend, B1824, td=3, delta=1)
    w = WORDS["combo_pwm"]
    loom.push(0, [host_word(w)])
    tx_run(bench, rec, B1824, 3, 200)
    check_tx(rec, B1824, [w] * 3)
    assert loom.pop_available(1) == [w]
    assert loom.badop() == 0


# -------------------------------------------------------------- the programs
def test_lego_programs_assemble_strict_with_no_diagnostic():
    """L2-DEADLINE: the transmitter's 23 pairs proved at a 16-clock tick
    with 8 clocks of slack, its pre-emption POP declared bounded; the
    receiver has no WAITD and no pair. Both fit a quarter of the 512-word
    memory, and the receiver moved to thread 1 stays in that quarter."""
    tx, rx = program("lego_pf_tx"), program("lego_pf_rx")
    assert tx.diagnostics == [] and rx.diagnostics == []
    assert (len(tx.words), len(rx.words)) == (92, 82)
    t = tx.deadlines[0]
    assert t.period == 16 and not t.infeasible and not t.unbounded
    assert len(t.pairs) == 23 and t.worst_slack == 8
    assert [d.name for d in t.declarations] == ["POP"]
    assert "only this thread pops INQ" in t.declarations[0].reason
    r = rx.deadlines[0]
    assert r.period == 32 and r.pairs == [] and not r.unbounded
    assert max(tx.words) < 0x80 and max(rx.words) < 0x80
    moved = program("lego_pf_rx", 1)
    assert moved.diagnostics == []
    assert min(moved.words) == 0x80 and max(moved.words) < 0x100


# --------------------------------------------------------------- the model
def test_lego_model_message_fields_and_lrc():
    """The field layouts of the document's mode sections and its LRC."""
    assert L.lrc(0x0000) == 0xF and L.message(0x1230) == 0x1230 | (0xF ^ 1 ^ 2 ^ 3)
    assert L.combo_pwm(1, 7, 9) == 0x4795                     # a 1 C C = 0100
    assert L.combo_direct(3, 1, 2, toggle=1) == L.message(0xA160)
    assert L.single_pwm(2, 1, 5) == L.message(0x1550)         # a 1 0 O = 0101
    assert L.single_cstid(1, 0, 0xB) == L.message(0x06B0)     # a 1 1 O = 0110
    assert L.extended(4, 6, address=1) == L.message(0x3860)
    assert [L.channel_of(L.combo_pwm(c, 0, 0)) for c in (1, 2, 3, 4)] == [1, 2, 3, 4]
    with pytest.raises(ValueError):
        L.combo_pwm(5, 0, 0)
    assert all(L.lrc_ok(L.message(w)) for w in range(0, 0x10000, 0x10))
    assert not L.lrc_ok(L.message(0x4790) ^ 0x0001)


def test_lego_model_encoder_and_decoder():
    """Round trips; 18 marks; the windows; faults; message lengths."""
    for w in WORDS.values():
        p = L.message_pulses(w)
        assert sum(1 for lv, _ in p if lv == L.MARK) == L.MARKS
        assert [(m.word, m.ok) for m in L.decode_pulses(p)] == [(w, True)]
        assert L.decode_starts(L.mark_starts(w)) == L.decode_pulses(p)
    assert L.interval_cycles(0x8000)[:3] == [45, 27, 16]
    assert (L.classify(316), L.classify(525.9), L.classify(526),
            L.classify(946.9), L.classify(947), L.classify(1579),
            L.classify(315.9), L.classify(1579.1)) == \
        ("low", "low", "high", "high", "startstop", "startstop", None, None)
    assert abs(L.MAX_MESSAGE_US - 522 * L.CYCLE_US) < 1e-9 < L.TM_US - L.MAX_MESSAGE_US
    bad = L.decode_pulses(L.message_pulses(L.message(0x1230) ^ 1))
    assert len(bad) == 1 and not bad[0].ok
    two = L.message_pulses(0x4795) + [(L.SPACE, 5000.0)] + L.message_pulses(0x4795)
    assert [m.word for m in L.decode_pulses(two)] == [0x4795] * 2


def test_lego_model_schedule_and_repeat_filter():
    """The retransmission table of "Transmitting Messages" in tm; the
    checker flags a spacing one clock-ish off; the repeat filter."""
    assert [L.schedule_tm(c) for c in (1, 2, 3, 4)] == [
        [3, 5, 5, 8, 8], [2, 5, 5, 10, 10], [1, 5, 5, 12, 12], [0, 5, 5, 14, 14]]
    assert L.schedule_tm(2, 7) == [2, 5, 5, 10, 10, 10, 10]
    assert L.TM_CYCLES * L.CYCLE_US == pytest.approx(L.TM_US)
    starts = [0.0, 80_000.0, 160_000.0, 384_000.0, 608_000.0]
    assert L.check_schedule(starts, 4, tol_us=1.0) == []
    assert L.check_schedule(starts[:4] + [608_002.0], 4, tol_us=1.0)[0][0] == 4
    assert L.check_schedule([48_100.0], 1, tol_us=1.0, first_from_us=0.0,
                            first_extra_us=300.0) == []
    f = L.RepeatFilter()
    w = L.PfMessage
    msgs = [w(0x4795, 0.0, True), w(0x4795, 80_000.0, True),
            w(0x4795 ^ 1, 90_000.0, False), w(0x5795 ^ 0x0001, 100_000.0, True),
            w(0x5794, 400_000.0, True)]
    assert f.feed(msgs) == [0x4795, 0x5794, 0x5794]          # dark 290 ms: again
    with pytest.raises(AssertionError):
        L.RepeatFilter(200_000.0)
