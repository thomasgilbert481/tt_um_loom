"""L3 for the game-controller programs (SNES and Sega six-button), model and RTL.

``firmware/snes_host.loom`` (thread 0; LATCH = OUT0, CLOCK = OUT1, DATA =
IN0) reads an SNES pad; ``firmware/snes_pad.loom`` (thread 1; LATCH = IN1,
CLOCK = IN2, DATA = OUT2) is one. ``firmware/sega6_host.loom`` (thread 2;
D0..D5 = BIDIR0..5, TH = BIDIR6) reads a Mega Drive pad;
``firmware/sega6_pad.loom`` (thread 3; TH = IN3, D0..D5 = OUT0..5) is one.
The references are ``tools.protomodels.snes_pad`` and ``sega6_pad``,
written from secondary sources (no standard exists). Expected times come
from those sources (12 us latch, 6 us gap, 6 us clock halves, the fastest
0.5 us pulse; the 1.5 ms counter reset) and from the program headers'
stated choices (ticks of 0.5 us, H, S, GUARD), never from what the model
or the RTL did.

PHY-GL-SDF candidate: ``test_snes_host_fastest_clock``: two reads at H = 1
(0.5 us halves), every LATCH and CLOCK edge checked to the clock.
"""

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels import sega6_pad as G
from tools.protomodels import snes_pad as N
from tools.protomodels.hx711 import EdgeLog
from tools.protomodels.quadrature import Wire
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
CLOCK = 50e6
TICK = 25                       # the hosts' tick: 0.5 us at 50 MHz
STARTUP = 300
PROGRAMS = {"snes_host": (0, 66), "snes_pad": (1, 32),
            "sega6_host": (2, 98), "sega6_pad": (3, 101)}


def image(name):
    return assemble_file(FIRMWARE / (name + ".loom"), isa=ISA, strict=True)


def bench_for(backend, **kw):
    if backend.name == "model":
        return backend.bench(features=("FIFO", "SETPD", "BE"), imem_words=512, **kw)
    return backend.bench(**kw)


def start(backend, bench, *names):
    loom = backend.loom(bench, isa=ISA)
    for name in names:
        loom.load(image(name))
    loom.run([PROGRAMS[n][0] for n in names])
    bench.step(STARTUP)
    return loom


def test_padctl_programs_assemble_strict_with_no_diagnostic():
    """L2-DEADLINE: both hosts' every pair proved at TK = 25 with one
    .bounded PUSH each; the pads have no pair (their deadlines are
    timeouts) and one .bounded POP each (sega6_pad: two). Each program in
    its own thread's quarter; all four fit the 512 words together."""
    used = set()
    for name, (thread, words) in PROGRAMS.items():
        prog = image(name)
        assert prog.diagnostics == [] and list(prog.threads) == [thread], name
        assert len(prog.words) == words, name
        assert min(prog.words) >= thread * 128 and max(prog.words) < (thread + 1) * 128
        rep = prog.deadlines[thread]
        if name.endswith("host"):
            assert rep.period == TICK and rep.pairs and not rep.infeasible
            assert not rep.unbounded and rep.worst_slack >= 0
        else:
            assert rep.pairs == []
        assert not used & set(prog.words)
        used |= set(prog.words)
    assert len(used) == 297


# ============================================================ SNES host
def snes_host_setup(backend, buttons=0, pad=True):
    bench = bench_for(backend)
    model = bench.add(N.SnesPad("OUT0", "OUT1", "IN0", buttons=buttons)) if pad else None
    lat = bench.add(EdgeLog("OUT0"))
    clk = bench.add(EdgeLog("OUT1"))
    loom = start(backend, bench, "snes_host")
    return bench, model, lat, clk, loom


def exact_frames(lat, clk, since, half, n):
    """``n`` reads after ``since``, each edge exact to the clock: the 12 us
    latch, the 6 us gap to the first fall, 16 pulses of ``half`` ticks
    low and high [C]."""
    frs = [f for f in N.frames(lat.edges, clk.edges) if f["rise"] >= since]
    assert len(frs) == n
    for f in frs:
        assert f["fall"] - f["rise"] == N.clocks(N.LATCH, CLOCK) == 24 * TICK
        falls, rises = f["falls"], f["rises"]
        assert len(falls) == len(rises) == 16
        assert falls[0] - f["fall"] == N.clocks(N.GAP, CLOCK) == 12 * TICK
        assert all(r - fa == half * TICK for fa, r in zip(falls, rises))
        assert all(fa - r == half * TICK for r, fa in zip(rises, falls[1:]))
    return frs


def test_snes_host_reads_model_pad(backend):
    """The sources' timing (H = 12: 6 us halves): two reads of a pad whose
    buttons change between them; each word arrives as pressed = 1, B in
    bit 15; the pad sees 16 clock rises after each latch."""
    w1 = N.word("B", "Start", "Left", "R")
    w2 = N.word("Y", "Select", "Up", "A", "X", "L")
    bench, pad, lat, clk, loom = snes_host_setup(backend, w1)
    since = bench.cycle
    loom.push(0, [0])
    assert loom.pop(0, 1, max_polls=4000) == [w1]
    pad.press(w2)
    loom.push(0, [0])
    assert loom.pop(0, 1, max_polls=4000) == [w2]
    exact_frames(lat, clk, since, 12, 2)
    assert pad.shifts == [16, 16]


def test_snes_host_fastest_clock(backend):
    """H = 1: halves of 0.5 us, the fastest clock pulse in the sources [S];
    two reads, every edge exact."""
    w1, w2 = N.word("A", "Down"), N.word("B", "Y", "Right", "L", signature=0)
    bench, pad, lat, clk, loom = snes_host_setup(backend, w1)
    loom.push(0, [0x4000 | 1])
    since = bench.cycle
    loom.push(0, [0])
    assert loom.pop(0, 1) == [w1]
    pad.press(w2)
    loom.push(0, [0])
    assert loom.pop(0, 1) == [w2]
    frs = exact_frames(lat, clk, since, 1, 2)
    assert frs[0]["falls"][0] - frs[0]["fall"] == N.clocks(N.GAP, CLOCK)


def test_snes_host_polls_on_a_grid(backend):
    """POLL N = 2 at H = 1: a read every 36 + 32 H + 128 N = 324 ticks
    (header), latch to latch exact to the clock (no drift); the third word
    follows the pad; a READ stops polling."""
    w1, w2 = N.word("Start"), N.word("Select", "R")
    bench, pad, lat, clk, loom = snes_host_setup(backend, w1)
    loom.push(0, [0x4000 | 1])
    since = bench.cycle
    loom.push(0, [0x8000 | 2])
    assert loom.pop(0, 2) == [w1, w1]
    pad.press(w2)
    words = loom.pop(0, 2)
    assert words[-1] == w2
    loom.push(0, [0])
    loom.pop(0, 1)
    frs = [f for f in N.frames(lat.edges, clk.edges) if f["rise"] >= since]
    gaps = [b["rise"] - a["rise"] for a, b in zip(frs, frs[1:])]
    assert gaps[:3] == [324 * TICK] * 3
    edges = len(lat.edges)
    bench.step(2 * 324 * TICK)
    assert len(lat.edges) == edges and not loom.pop_available(0)


def test_snes_host_empty_port(backend):
    """No pad: DATA low (the console pulls its data lines low [S]), every
    bit reads pressed: 0xFFFF."""
    bench, _, lat, clk, loom = snes_host_setup(backend, pad=False)
    loom.push(0, [0x4000 | 1, 0])
    assert loom.pop(0, 1) == [0xFFFF]


# ============================================================= SNES pad
def snes_pad_setup(backend, buttons, **host):
    bench = bench_for(backend)
    loom = start(backend, bench, "snes_pad")
    loom.push(1, [buttons])
    bench.step(2000)                    # a 16 us piece picks it up
    data = bench.add(EdgeLog("OUT2"))
    ref = bench.add(N.SnesHost("IN1", "IN2", "OUT2", clock_hz=CLOCK,
                               start=bench.cycle + 50, **host))
    return bench, ref, data, loom


def run_reads(bench, ref):
    assert bench.run_until(lambda: ref.done(bench.cycle), 200000, every=100)


@pytest.mark.parametrize("speed", ["slowest", "fastest"])
def test_snes_pad_serves_model_host(backend, speed):
    """A console at the sources' 6 us halves [C] and at the fastest 0.5 us
    pulse [S], sampling at each fall: every bit right, the buttons from
    the host changing between reads, and a 17th clock reads low (the
    grounded serial input of an official pad [N])."""
    half = N.HALF if speed == "slowest" else N.FASTEST_HALF
    w1 = N.word("B", "Up", "A", "R")
    w2 = N.word("Y", "Start", "Down", "Right", "X")
    bench, ref, data, loom = snes_pad_setup(backend, w1, low=half, high=half,
                                            nclocks=17)
    run_reads(bench, ref)
    loom.push(1, [w2])
    bench.step(2000)
    ref.again(bench.cycle + 50)
    run_reads(bench, ref)
    assert ref.words == [w1, w2]
    assert ref.extra == [[0], [0]]


def test_snes_pad_data_lag_on_the_pins(backend):
    """Measured on the pins at the sources' timing: each DATA change comes
    7 to 15 clocks after the CLOCK rise that shifts it (the header's
    figure), well inside the 6 us half before the console samples."""
    w = 0xA5A0                          # a change on most rises
    bench, ref, data, loom = snes_pad_setup(backend, w)
    run_reads(bench, ref)
    assert ref.words == [w]
    s = ref.start + ref.L + ref.G
    rises = [s + i * (ref.lo + ref.hi) + ref.lo for i in range(16)]
    lags = []
    for cyc, _ in data.edges:
        before = [r for r in rises if r <= cyc]
        if before and cyc - before[-1] < ref.hi:
            lags.append(cyc - before[-1])
    assert len(lags) >= 8 and min(lags) >= 7 and max(lags) <= 15


def test_snes_pad_recovers_from_a_cut_short_read(backend):
    """A console that stops after 5 clocks: the pad gives up after its
    100 us clock timeout, and the next full read is right."""
    w = N.word("L", "R", "Select")
    bench, ref, data, loom = snes_pad_setup(backend, w, nclocks=5,
                                            low=N.FASTEST_HALF, high=N.FASTEST_HALF)
    run_reads(bench, ref)
    assert ref.words == [w >> 11]      # the first five bits only
    ref.nclocks = 16
    ref.length = ref.L + ref.G + 16 * (ref.lo + ref.hi)
    ref.period = ref.length + 100
    ref.again(bench.cycle + N.clocks(150e-6, CLOCK))
    run_reads(bench, ref)
    assert ref.words[-1] == w


# ============================================================ SNES pair
@pytest.mark.parametrize("half", [12, 1])
def test_snes_host_and_pad_on_two_threads(backend, half):
    """snes_host on thread 0 reads snes_pad on thread 1 over three board
    wires (OUT0 to IN1, OUT1 to IN2, OUT2 to IN0), at the sources' 6 us
    halves and at 0.5 us: the host reports what the pad was given."""
    bench = bench_for(backend)
    bench.add(Wire("OUT0", "IN1"))
    bench.add(Wire("OUT1", "IN2"))
    bench.add(Wire("OUT2", "IN0"))
    loom = start(backend, bench, "snes_host", "snes_pad")
    loom.push(0, [0x4000 | half])
    for w in (N.word("B", "A", "Start"), N.word("Up", "Left", "X", "L", "R")):
        loom.push(1, [w])
        bench.step(2000)
        loom.push(0, [0])
        assert loom.pop(0, 1, max_polls=4000) == [w]


# =========================================================== Sega host
S_PHASE = 5 * TICK              # 1 + S ticks: 2.5 us from edge to edge
GUARD_CLOCKS = 32 * 128 * TICK  # 2.05 ms
SIXW = G.buttons("U", "R", "B", "Start", "X", "Mode")


def sega_host_setup(backend, word=SIXW, pad=True, **pad_kw):
    bench = bench_for(backend, pullups=0x7F)        # lines high, and TH before it is driven
    model = bench.add(G.Sega6Pad("BIDIR6", "BIDIR0", word=word, **pad_kw)) if pad else None
    th = bench.add(EdgeLog("BIDIR6"))
    loom = start(backend, bench, "sega6_host")
    return bench, model, th, loom


def exact_th(th, since, count):
    """``count`` TH edges after ``since``, alternating from a fall, each
    one phase (1 + S ticks) after the last, exact to the clock."""
    edges = [e for e in th.edges if e[0] >= since]
    assert [v for _, v in edges] == [0, 1] * (count // 2)
    assert all(b[0] - a[0] == S_PHASE for a, b in zip(edges, edges[1:]))
    return edges


def test_sega6_host_reads_six_button_pad(backend):
    """READ6 of a six-button pad: four TH pulses (the third low is the ID,
    the fourth high X Y Z Mode [G][P][Q]); the report holds every button
    and says pad and six buttons; the pad saw n = 1, 1, 2, 2, 3, 3, 4, 4."""
    bench, pad, th, loom = sega_host_setup(backend)
    since = bench.cycle
    loom.push(2, [0])
    assert loom.pop(2, 1) == [G.report(SIXW)]
    exact_th(th, since, 8)
    assert [n for _, _, n in pad.phases] == [1, 1, 2, 2, 3, 3, 4, 4]


def test_sega6_host_reads_three_button_pad(backend):
    """READ6 of a three-button pad: the third low is S A 0 0 D U, not
    0000, so the report says three buttons and carries no X Y Z Mode."""
    bench, pad, th, loom = sega_host_setup(backend, three=True)
    loom.push(2, [0])
    assert loom.pop(2, 1) == [G.report(SIXW, six=False)]


def test_sega6_host_read3_command(backend):
    """READ3: one TH pulse only, what a three-button game does."""
    bench, pad, th, loom = sega_host_setup(backend)
    since = bench.cycle
    loom.push(2, [1])
    assert loom.pop(2, 1) == [G.report(SIXW, six=False)]
    exact_th(th, since, 2)


def test_sega6_host_empty_port(backend):
    """No pad (lines pulled high): D3:D2 not low with TH low, report 0."""
    bench, _, th, loom = sega_host_setup(backend, pad=False)
    loom.push(2, [0])
    assert loom.pop(2, 1) == [0]


@model_only("2 ms guards: the pair test runs one guard on the RTL")
def test_sega6_host_polls_past_the_pad_reset(backend):
    """POLL with N = 0x10, taken as 32: a read every 37 + 32 * 128 ticks
    (header), exact; a pad with the longest reset in the sources (1.6 ms
    [P]) has reset before every read, so every report is six-button; the
    buttons follow the pad; a READ stops polling."""
    bench, pad, th, loom = sega_host_setup(backend, reset=G.RESET_RANGE[1])
    since = bench.cycle
    loom.push(2, [0x8000 | 0x10])
    assert loom.pop(2, 2, max_polls=20000) == [G.report(SIXW)] * 2
    w2 = G.buttons("D", "L", "C", "A", "Z", "Y")
    pad.press(w2)
    assert loom.pop(2, 1, max_polls=20000) == [G.report(w2)]
    falls = [c for c, v in th.edges if c >= since and not v][::4]
    assert [b - a for a, b in zip(falls, falls[1:])][:2] == [(37 + 32 * 128) * TICK] * 2
    assert len(pad.resets) >= 2


# ============================================================ Sega pad
def sega_pad_setup(backend, word, three=False):
    bench = bench_for(backend)
    loom = start(backend, bench, "sega6_pad")
    loom.push(3, [word | (0x8000 if three else 0)])
    bench.step(1500)                    # a 15 us piece picks it up
    return bench, loom


def expected(word, levels, n=0, three=False):
    """What the pad's lines must read in each phase [G][P]: ``n`` TH falls
    so far, one more at each low."""
    out = []
    for i, lv in enumerate(levels):
        if i and not lv and not three:
            n = min(n + 1, 5)
        out.append(G.lines_for(word, lv, n, three))
    return out


def sega_host_model(bench, plan, **kw):
    return bench.add(G.Sega6Host("IN3", "OUT0", plan=plan, **kw))


def test_sega6_pad_six_button_cycle(backend):
    """A console doing the six-button cycle with 2 us before each read:
    every phase right, the ID (0000) in the third low, X Y Z Mode in the
    fourth high, 1111 in the fourth low."""
    bench, loom = sega_pad_setup(backend, SIXW)
    ref = sega_host_model(bench, [(bench.cycle + 50, G.SIX)])
    bench.step(50 + len(G.SIX) * ref.phase + 10)
    assert ref.reads == [expected(SIXW, G.SIX)]
    assert ref.reads[0][5] & 0xF == 0 and ref.reads[0][7] & 0xF == 0xF


def test_sega6_pad_answers_within_a_microsecond(backend):
    """The header's limit, measured: lines right 46 clocks (0.92 us) after
    each TH edge, phases of 2 us."""
    w = G.buttons("D", "L", "C", "A", "Y", "Z")
    bench, loom = sega_pad_setup(backend, w)
    ref = sega_host_model(bench, [(bench.cycle + 50, G.SIX)], phase=100, settle=46)
    bench.step(50 + len(G.SIX) * 100 + 10)
    assert ref.reads == [expected(w, G.SIX)]


def test_sega6_pad_counter_resets_after_a_pause(backend):
    """After a six-button cycle (n = 4), a pause of 1.45 ms keeps the
    counter (the next cycle reads three-button values throughout, n = 5),
    and a pause of 1.55 ms resets it (the cycle after reads X Y Z Mode
    again), against the 1.5 ms reset [M]. New buttons from the host are
    taken during the pause."""
    bench, loom = sega_pad_setup(backend, SIXW)
    span = len(G.SIX) * 125
    c0 = bench.cycle + 50
    c1 = c0 + span + G.clocks(1.45e-3, CLOCK)
    c2 = c1 + span + G.clocks(1.55e-3, CLOCK)
    ref = sega_host_model(bench, [(c0, G.SIX), (c1, G.SIX), (c2, G.SIX)])
    bench.step(c1 + span - bench.cycle)
    w2 = G.buttons("U", "C", "Start", "Mode", "Z")
    loom.push(3, [w2])
    bench.step(c2 + span + 10 - bench.cycle)
    assert ref.reads[0] == expected(SIXW, G.SIX)
    assert ref.reads[1] == expected(SIXW, G.SIX, n=4)
    assert ref.reads[2] == expected(w2, G.SIX)


def test_sega6_pad_three_button_mode(backend):
    """Bit 15 of the host word: a three-button pad; the six-button cycle
    reads three-button values in every phase, so a console sees no ID."""
    bench, loom = sega_pad_setup(backend, SIXW, three=True)
    ref = sega_host_model(bench, [(bench.cycle + 50, G.SIX)])
    bench.step(50 + len(G.SIX) * ref.phase + 10)
    assert ref.reads == [expected(SIXW, G.SIX, three=True)]
    assert ref.reads[0][5] & 0xF != 0             # U pressed, D not: no ID


# ============================================================ Sega pair
def sega_pair(backend):
    """sega6_host on thread 2 and sega6_pad on thread 3 over seven board
    wires (BIDIR6 to IN3, OUT0..5 to BIDIR0..5), the pad holding SIXW."""
    bench = bench_for(backend)
    bench.add(Wire("BIDIR6", "IN3"))
    for i in range(6):
        bench.add(Wire("OUT%d" % i, "BIDIR%d" % i))
    loom = start(backend, bench, "sega6_host", "sega6_pad")
    loom.push(3, [SIXW])
    bench.step(1500)
    return bench, loom


def test_sega6_host_and_pad_on_two_threads(backend):
    """The pair: READ6 then (no guard needed before the first) the report
    is six-button with every button the pad was given."""
    bench, loom = sega_pair(backend)
    loom.push(2, [0])
    assert loom.pop(2, 1) == [G.report(SIXW)]


@model_only("two 2 ms guards; the RTL runs the one-read pair and the pad's reset")
def test_sega6_host_and_pad_guard_lets_the_pad_reset(backend):
    """READ6 twice back to back: the second waits out the host's 2.05 ms
    guard, by which the pad's counter has reset, so both are six-button;
    then READ3, three-button bits only."""
    bench, loom = sega_pair(backend)
    w2 = G.buttons("D", "B", "C", "A", "X", "Y")
    loom.push(2, [0])
    assert loom.pop(2, 1) == [G.report(SIXW)]
    loom.push(3, [w2])
    loom.push(2, [0, 1])
    assert loom.pop(2, 2, max_polls=20000) == [G.report(w2), G.report(w2, six=False)]
