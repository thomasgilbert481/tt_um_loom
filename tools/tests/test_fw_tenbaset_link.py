"""L3-TENBASET: 10BASE-T link pulses (NLP), FLP bursts and the link
integrity test, on the model and the RTL.

``firmware/tenbaset_link.loom`` (thread 0: NLPs or FLP bursts on TD+/TD-,
OUT0/OUT1) and ``firmware/tenbaset_link_rx.loom`` (thread 1: link pass /
fail and FLP decoding on RD, IN0) are assembled, loaded and fed through
``tools.loomhost.Loom``. ``tools.protomodels.tenbaset_link.PulseMonitor``
cuts the pins into pulses, NLPs and bursts and checks each against the
windows of the clauses; ``LinkPulseSource`` plays a link partner into RD,
and ``LinkIntegrity`` is the reference for when the link passes and fails.
Every body that takes a ``backend`` runs here on the golden model and
under cocotb on the RTL (``test/test_fw.py``).

Expected numbers come from the clauses (as the model cites them) and from
``docs/SEMANTICS.md``, never from what the model or the RTL did. A pulse is
100 ns, 5 clocks at 50 MHz, exactly: the rise is an ordinary pin write and
the fall a deadline-latched one 5 ticks of 1 clock later (SEMANTICS 6.10).
Burst positions are nominally 62.5 us (3125 clocks) apart; an ordinary pin
write lands on the thread's 4-clock slot grid (SEMANTICS 2), so the
program's spacing is 3124 clocks and clock pulses 6248 apart, every one,
inside the 55.5-69.5 us and 111-139 us windows. The period is the host's
N * 4 us exactly.

Scaling on the RTL (Icarus runs about 6,400 clocks a second): the 16 ms
period is a host parameter, so the RTL runs NLPs every 240 us (N = 60) and
bursts every 2.2 ms (N = 550; the program allows N = 501, but then the
gap after a burst is 4.6 us and a receiver sees one long group; a
burst itself lasts 2.0 ms and is not scaled). The receiver's link timers
are host parameters too (T_MIN, T_MAX in 4 us ticks), scaled the same
way. The real 16 ms and 4 ms / 100 ms timers run on the model only.
"""

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels import tenbaset_link as L
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
IMEM = 512
CLOCK_HZ = 50e6
CPU = CLOCK_HZ / 1e6                         # clocks per microsecond
WIDTH = 5                                    # 100 ns at 50 MHz
POS = 3124                                   # 62.5 us on the 4-clock grid
UNIT = 200                                   # the host's period unit, 4 us
RX_TICK = 200                                # the receiver's tick, 4 us


def program(name):
    prog = assemble_file(FIRMWARE / (name + ".loom"), isa=ISA, strict=True)
    assert prog.diagnostics == [], prog.diagnostics
    return prog


TX, RX = program("tenbaset_link"), program("tenbaset_link_rx")


def image(*progs):
    words = {}
    for prog in progs:
        assert not set(words) & set(prog.words)
        words.update(prog.words)
    return words


def config(n, flp=False):
    return 0x8000 | (0x4000 if flp else 0) | n


def make_bench(backend):
    """FIFOs, the deadline latch and the bit engine's shifter, 512 words."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "SETPD", "BE"), imem_words=IMEM)
    return backend.bench()


def boot(backend, bench, threads, progs):
    loom = backend.loom(bench, isa=ISA)
    loom.load(image(*progs))
    loom.run(threads)
    return loom


def tx_setup(backend, check_period=False):
    bench = make_bench(backend)
    mon = bench.add(L.PulseMonitor("OUT0", "OUT1", clock_hz=CLOCK_HZ,
                                   check_period=check_period))
    return bench, mon, boot(backend, bench, [0], [TX])


def run_events(bench, mon, n, budget):
    """Step until ``mon`` has closed ``n`` events."""
    ok = bench.run_until(lambda: len(mon.flush()) >= n, budget, every=64)
    assert ok, "%d of %d events after %d clocks: %s" % (
        len(mon.events), n, budget, mon.report())
    return mon.events


def check_pulses(mon):
    assert mon.violations == [], mon.report()
    assert mon.tdn_high == 0                 # TD- is held low
    assert {p.width for p in mon.pulses} == {WIDTH}


def check_burst(ev, word):
    """17 clocks 2 * POS apart, data pulses POS after their clock."""
    assert ev.kind == "flp" and ev.word == word, (ev.kind, ev.word, word)
    assert len(ev.clocks) == L.FLP_CLOCKS
    clocks = [p.rise for p in ev.clocks]
    assert {b - a for a, b in zip(clocks, clocks[1:])} == {2 * POS}
    data = [p.rise for p in ev.pulses if p not in ev.clocks]
    assert len(data) == bin(word).count("1")
    assert all((t - ev.start) % (2 * POS) == POS for t in data)


# ---------------------------------------------------------------- NLPs
def test_tenbaset_nlp_width_and_period_exact(backend):
    """NLPs every N * 4 us (N = 60, scaled): every pulse exactly 5 clocks
    (100 ns), every interval exactly 12,000 clocks, TD- low."""
    n = 60
    bench, mon, loom = tx_setup(backend)
    loom.push(0, [config(n)])
    events = run_events(bench, mon, 4, 5 * n * UNIT + 4000)
    check_pulses(mon)
    assert [e.kind for e in events] == ["nlp"] * len(events)
    starts = [e.start for e in events]
    assert {b - a for a, b in zip(starts, starts[1:])} == {n * UNIT}
    assert loom.badop() == 0


@model_only("16 ms a pulse: 800,000 clocks each, two minutes of Icarus")
def test_tenbaset_nlp_16ms_inside_the_window(backend):
    """N = 4000: NLPs 16 ms apart, exactly 800,000 clocks, inside 16 +- 8 ms."""
    bench, mon, loom = tx_setup(backend, check_period=True)
    loom.push(0, [config(4000)])
    events = run_events(bench, mon, 3, 3 * 800_000 + 4000)
    check_pulses(mon)
    starts = [e.start for e in events]
    assert {b - a for a, b in zip(starts, starts[1:])} == {800_000}


@model_only("the clamp and the stop are control flow the other bodies cover")
def test_tenbaset_nlp_short_period_clamps_and_zero_stops(backend):
    """N = 1 is taken as 2 (400 clocks); N = 0 stops after the pulse."""
    bench, mon, loom = tx_setup(backend)
    loom.push(0, [config(1)])
    bench.step(6 * 400 + 1000)
    starts = [p.rise for p in mon.pulses]
    assert len(starts) >= 5
    assert {b - a for a, b in zip(starts, starts[1:])} == {400}
    loom.push(0, [config(0)])
    bench.step(2000)
    count = len(mon.pulses)
    bench.step(4000)
    assert len(mon.pulses) == count and bench.lines.uo & 3 == 0
    check_pulses(mon)


# ---------------------------------------------------------------- FLP
def test_tenbaset_flp_burst_carries_the_code_word(backend):
    """FLP at the period N = 550 (2.2 ms, scaled): a burst for 10BASE-T half and
    full duplex, 17 clocks and the data pulses on their positions, then the
    next burst exactly 110,000 clocks after the first."""
    word = L.base_page(half=True, full=True)
    bench, mon, loom = tx_setup(backend)
    loom.push(0, [word, config(550, flp=True)])
    events = run_events(bench, mon, 1, 550 * UNIT + 4000)
    check_pulses(mon)
    check_burst(events[0], word)
    first = events[0].start
    assert bench.run_until(lambda: len(mon.pulses) > len(events[0].pulses),
                           550 * UNIT + 4000, every=64)
    assert mon.pulses[-1].rise - first == 550 * UNIT
    assert loom.badop() == 0


@model_only("two bursts of 2 ms each and a mode switch: 400 k clocks")
def test_tenbaset_flp_word_and_mode_change_between_bursts(backend):
    """A new code word (with Ack) is taken after a burst and sent in the
    next; a configuration word switches FLP to NLP (N = 60 after a burst is
    taken as the shortest, 550, so the line is quiet for 205 us after the
    burst), then N = 0 stops."""
    w1 = L.base_page()
    w2 = L.base_page(full=True, ack=True)
    bench, mon, loom = tx_setup(backend)
    loom.push(0, [w1, config(550, flp=True)])
    run_events(bench, mon, 1, 550 * UNIT + 4000)
    loom.push(0, [w2])                       # taken after the next burst
    run_events(bench, mon, 3, 2 * 550 * UNIT + 4000)
    loom.push(0, [config(60)])
    run_events(bench, mon, 7, 2 * 550 * UNIT + 4 * 60 * UNIT + 4000)
    loom.push(0, [config(0)])
    bench.step(3 * 60 * UNIT)
    events = mon.flush()
    check_pulses(mon)
    flps = [e for e in events if e.kind == "flp"]
    nlps = [e for e in events if e.kind == "nlp"]
    assert [e.kind for e in events] == ["flp"] * len(flps) + ["nlp"] * len(nlps)
    for e in flps:
        check_burst(e, e.word)
    words = [e.word for e in flps]
    assert words[:2] == [w1, w1] and set(words[2:]) == {w2}, words
    starts = [e.start for e in flps]
    assert {b - a for a, b in zip(starts, starts[1:])} == {550 * UNIT}
    assert len(nlps) >= 3
    assert nlps[0].start - flps[-1].start == 550 * UNIT
    starts = [e.start for e in nlps]
    assert {b - a for a, b in zip(starts, starts[1:])} == {60 * UNIT}
    count = len(mon.pulses)
    bench.step(3 * 60 * UNIT)
    assert len(mon.pulses) == count          # stopped
    assert loom.badop() == 0


# ------------------------------------------------------------ receiver
PASS_W, FAIL_W, FLP_W, BAD_W = 0x8001, 0x8000, 0xC000, 0x8002
#: How late a word may be seen: the polling step (2048 clocks), three ticks
#: of the receiver's 4 us timer resolution, and the host's FIFO reads.
LAG = 2048 + 3 * RX_TICK + 2000


def rx_setup(backend, t_min, t_max, tx=False):
    """The receiver on thread 1 with T_MIN, T_MAX in 4 us ticks; RD driven
    by a ``LinkPulseSource`` or, with ``tx``, wired from the transmitter."""
    bench = make_bench(backend)
    if tx:
        bench.add(L.Wire("OUT0", "IN0"))
        src = None
    else:
        src = bench.add(L.LinkPulseSource("IN0", clock_hz=CLOCK_HZ))
    loom = boot(backend, bench, [0, 1] if tx else [1], [TX, RX] if tx else [RX])
    loom.push(1, [t_min, t_max])
    return bench, src, loom


def collect(bench, loom, until, every=2048):
    """Step to clock ``until``, taking OUTQ[1]'s words with the clock they
    were seen at (``every`` clocks apart at most)."""
    out = []
    while bench.cycle < until:
        bench.step(min(every, until - bench.cycle))
        out += [(bench.cycle, w) for w in loom.pop_available(1)]
    return out


def expected(src, t_min, t_max, until):
    """``LinkIntegrity`` on what the source sent, grouped as a receiver
    groups it (``GROUP_US``)."""
    groups = []
    for rise, _ in src.spans:
        t = rise / CPU
        if groups and t - groups[-1][2] <= L.GROUP_US:
            groups[-1][1], groups[-1][2] = "flp", t
        else:
            groups.append([t, "nlp", t])
    ref = L.LinkIntegrity(t_min * RX_TICK / CPU, t_max * RX_TICK / CPU, 3)
    return ref.run([tuple(g) for g in groups], until / CPU)


def test_tenbaset_rx_link_pass_then_loss(backend):
    """NLPs 250 us apart with T_MIN 100 us and T_MAX 400 us (scaled):
    link pass on the third, link fail 400 us after the fourth."""
    t_min, t_max = 25, 100
    bench, src, loom = rx_setup(backend, t_min, t_max)
    t0 = bench.cycle + 2000
    times = [src.nlp(t0 + k * src.clocks(250)) for k in range(4)]
    until = times[-1] + src.clocks(400 + 300)
    got = collect(bench, loom, until)
    assert [w for _, w in got] == [PASS_W, FAIL_W]
    want = expected(src, t_min, t_max, until)
    assert [s for _, s in want] == ["pass", "fail"]
    # pass: once the third pulse's group has ended (190 us of quiet line);
    # fail: when T_MAX after the fourth pulse has run out
    assert times[2] < got[0][0] - src.clocks(L.GROUP_US) < times[3]
    lag = got[1][0] - src.clocks(want[1][0])
    assert 0 <= lag <= LAG, lag
    assert loom.badop() == 0


@model_only("the count rules in detail; the RTL runs pass and loss above")
def test_tenbaset_rx_early_pulse_and_burst_reset_the_count(backend):
    """T_MIN 300 us, T_MAX 1 ms: an NLP 250 us after the last sets the count
    to 0, a burst sets it to 0, a gap over T_MAX starts again at 1; the
    link passes only on three good NLPs in a row, as ``LinkIntegrity``."""
    t_min, t_max = 75, 250
    bench, src, loom = rx_setup(backend, t_min, t_max)
    t, us = bench.cycle + 2000, src.clocks
    for gap in (0, 400, 250, 400, 400):          # 1, 2, early 0, 1, 2
        t = src.nlp(t + us(gap))
    t = src.burst(t + us(400), L.base_page())    # count 0, a code word
    for gap in (400, 400, 1300, 400, 400):       # 1, 2, late: 1, 2, 3 pass
        t = src.nlp(t + us(gap))
    t = src.nlp(t + us(250))                     # early, but passing: kept
    until = t + us(1000 + 500)
    got = [w for _, w in collect(bench, loom, until)]
    want = expected(src, t_min, t_max, until)
    assert [s for _, s in want] == ["pass", "fail"]
    assert got == [FLP_W, L.base_page(), PASS_W, FAIL_W]
    assert loom.badop() == 0


@model_only("bursts of 2 ms each; the RTL decodes one in the loopback")
def test_tenbaset_rx_decodes_bursts_across_the_windows(backend):
    """Bursts at the edges of the clause's windows (clocks 112 and 138 us
    apart, data 56 and 69 us after a clock) decode to their code words; a
    burst for 0 with its 9th clock pulse missing leaves 250 us of quiet
    line, so the receiver sees two groups of 8 clocks, both reported bad."""
    bench, src, loom = rx_setup(backend, 25, 30000)
    words = [0x4061, 0xFFFF & ~(1 << 15), 0x0001, 0x2AAA]
    t = bench.cycle + 2000
    for k, w in enumerate(words):
        pos = 56.0 if k % 2 == 0 else 69.0
        t = src.burst(t, w, position_us=pos, data_us=pos) + src.clocks(1000)
    # a burst for 0 with its 9th clock pulse removed
    start = t
    src.burst(start, 0)
    gone = start + src.clocks(8 * 125.0)
    src.spans = [s for s in src.spans if s[0] != gone]
    until = start + src.clocks(2200)
    got = [w for _, w in collect(bench, loom, until)]
    want = []
    for w in words:
        want += [FLP_W, w]
    assert got == want + [BAD_W, BAD_W]
    assert loom.badop() == 0


# ------------------------------------------------------------ loopback
def test_tenbaset_loopback_flp_into_rx(backend):
    """Transmitter into receiver, TD+ wired to RD: the receiver decodes the
    transmitter's burst (10BASE-T half and full duplex, Ack) exactly."""
    word = L.base_page(half=True, full=True, ack=True)
    bench, _, loom = rx_setup(backend, 25, 30000, tx=True)
    loom.push(0, [word, config(550, flp=True)])
    got = collect(bench, loom, bench.cycle + 550 * UNIT, every=8192)
    assert [w for _, w in got] == [FLP_W, word]
    assert loom.badop() == 0


def test_tenbaset_loopback_nlp_link_pass(backend):
    """NLPs every 252 us (N = 63, scaled) into the receiver with T_MIN
    100 us, T_MAX 400 us: link pass on the third; N = 0 stops the
    transmitter and the link fails T_MAX later."""
    bench, _, loom = rx_setup(backend, 25, 100, tx=True)
    loom.push(0, [config(63)])
    got = collect(bench, loom, bench.cycle + 4 * 63 * UNIT, every=4096)
    assert [w for _, w in got] == [PASS_W]
    loom.push(0, [config(0)])
    got = collect(bench, loom, bench.cycle + 63 * UNIT + 100 * RX_TICK + 8000,
                  every=4096)
    assert [w for _, w in got] == [FAIL_W]
    assert loom.badop() == 0
