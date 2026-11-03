"""L3 for the quadrature encoder programs, on the model and the RTL.

``firmware/quad_decoder.loom`` (threads 0 and 1; A = IN0, B = IN1, Z = IN2)
is the x4 decoder with its host side; ``firmware/quad_generator.loom``
(thread 0, moved to thread 2 to run beside the decoder; A = OUT0,
B = OUT1, Z = OUT2) is the encoder simulator. The references are
``tools.protomodels.quadrature``: ``steps`` builds encoder waveforms with
chosen edge spacing, jitter, invalid transitions and an index, and
``QuadDecoder`` is the clock-exact x4 decoder from the cited counting
rules. Expected counts come from that reference and from the rules
themselves; expected timings from the program headers' stated limits
(32 clocks per edge for the decoder, one edge per period P, the first three
periods after the command, for the generator), never from what the model
or the RTL did.

The PHY-GL-SDF candidate is ``test_generator_edges_are_exact``: a
40-clock period (the shortest), 24 steps out and back over the index,
every A/B edge checked to the clock: about 2,200 clocks of edges in a
37,500-clock RTL run (the rest is the host loading the image).
"""

import pytest

from tools.loomasm import assemble_file, assemble_text
from tools.loomisa import REPO, load
from tools.protomodels import quadrature as Q
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
IMEM = 512
DECODER = FIRMWARE / "quad_decoder.loom"
GENERATOR = FIRMWARE / "quad_generator.loom"

READ, ZERO, PERIOD = 0x0000, 0x4000, 0x8000     # decoder commands
INDEX = 0x8000                                  # generator: index every N
DIR, IDX = 1, 2                                 # decoder status bits
LIMIT = 32                                      # decoder: clocks per edge
PMIN = 40                                       # generator: shortest period
STARTUP = 200                                   # clocks for set-up after RUN


def program(path, thread=0):
    """A program from ``firmware/`` with ``--strict``, moved to ``thread``."""
    if thread == 0:
        return assemble_file(path, isa=ISA, strict=True)
    text = path.read_text(encoding="utf-8")
    assert text.count(".thread 0") == 1
    text = text.replace(".thread 0", ".thread %d" % thread)
    return assemble_text(text, path.name, isa=ISA, imem_words=IMEM, strict=True)


def qbench(backend):
    """FIFOs, the deadline latch, LD/ST and the 512-word memory; the RTL
    has them all and ``RtlBench`` takes neither keyword."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "SETPD", "DMEM"), imem_words=IMEM)
    return backend.bench()


# ------------------------------------------------------------ the decoder
def decoder_setup(backend):
    """The decoder on threads 0 and 1, an encoder model on IN0..IN2 and the
    reference decoder watching the same pads."""
    bench = qbench(backend)
    src = bench.add(Q.QuadSource())
    ref = bench.add(Q.QuadDecoder())
    loom = backend.loom(bench, isa=ISA)
    loom.load(program(DECODER))
    loom.run([0, 1])
    bench.step(STARTUP)
    return bench, src, ref, loom


def play(bench, src, events, tail=400):
    """Schedule ``events`` from now (cycles relative to the bench's) and run
    until they are all out, plus ``tail`` clocks."""
    now = bench.cycle
    src.schedule([(now + c, a, b, z) for c, a, b, z in events])
    last = max(c for c, _, _, _ in events) if events else 0
    bench.step(last + tail)


def report(bench, loom):
    """READ: (count, status, index), the two signed."""
    loom.push(1, [READ])
    count, status, index = loom.pop(1, 3)
    return Q.to_signed(count), status, Q.to_signed(index)


def expect(loom, bench, ref, *, idx=None):
    """The decoder's report agrees with the reference decoder."""
    count, status, index = report(bench, loom)
    assert count == ref.count
    assert status & DIR == ref.dir
    assert status >> 2 == ref.errors & 0x3FFF
    if idx is not None:
        assert bool(status & IDX) == idx
    if ref.index is not None:
        assert index == ref.index
    return count, status, index


def test_quadrature_programs_assemble_strict_with_no_diagnostic():
    """L2-DEADLINE: the decoder has no deadline pair by design (it keeps no
    schedule; thread 1's deadline is a timeout); the generator's every pair
    is proved at its .tick of 40 clocks, nothing unbounded, no
    ``.bounded``. Word counts as the headers give them, and the decoder
    (threads 0, 1) and the generator moved to thread 2 do not overlap."""
    dec = program(DECODER)
    assert dec.diagnostics == [] and sorted(dec.threads) == [0, 1]
    assert all(r.pairs == [] for r in dec.deadlines.values())
    assert len(dec.words) == 126
    assert max(a for a in dec.words if a < 0x80) == 0x7F
    assert min(a for a in dec.words if a >= 0x80) == 0x80
    assert max(dec.words) < 0x100
    gen = program(GENERATOR)
    assert gen.diagnostics == [] and list(gen.threads) == [0]
    rep = gen.deadlines[0]
    assert rep.period == PMIN and rep.pairs and not rep.infeasible
    assert not rep.unbounded and rep.worst_slack >= 0
    assert len(gen.words) == 106 and max(gen.words) < 0x80
    moved = program(GENERATOR, thread=2)
    assert min(moved.words) == 0x100 and max(moved.words) < 0x180
    assert not set(dec.words) & set(moved.words)


def test_decoder_counts_forward_and_backward(backend):
    """x4: every edge counts, +1 with A leading, -1 with B leading; the
    direction bit is that of the last count; no errors. 100 steps out,
    177 back."""
    bench, src, ref, loom = decoder_setup(backend)
    phase = pos = 0
    for n, at in ((100, 100), (-177, -77)):
        events, phase, pos = Q.steps(10, phase, n, 120, pos=pos)
        play(bench, src, events)
        count, status, _ = expect(loom, bench, ref, idx=False)
        assert count == at and status == (DIR if n < 0 else 0)


def test_decoder_follows_reversals(backend):
    """Direction reversals at every phase, with jitter: the count follows
    the reference decoder at every report."""
    bench, src, ref, loom = decoder_setup(backend)
    phase, pos = 0, 0
    for n, seed in ((7, 1), (-3, 2), (5, 3), (-9, 4), (2, 5), (-1, 6), (6, 7)):
        events, phase, pos = Q.steps(10, phase, n, 90, pos=pos, jitter=20, seed=seed)
        play(bench, src, events, tail=200)
        expect(loom, bench, ref)
    assert ref.count == pos == 7


def test_decoder_latches_the_index(backend):
    """Z high in state 00 every 40 counts (a 10-line encoder): the count at
    the latest index is latched, IDX is set once and cleared by the report
    that carries it; going back over the index latches it again."""
    bench, src, ref, loom = decoder_setup(backend)
    events, phase, pos = Q.steps(10, 0, 90, 100, index_every=40)
    play(bench, src, events)
    count, status, index = expect(loom, bench, ref, idx=True)
    assert (count, index) == (90, 80)
    assert report(bench, loom)[1] & IDX == 0
    events, phase, pos = Q.steps(10, phase, -55, 100, pos=pos, index_every=40)
    play(bench, src, events)
    count, status, index = expect(loom, bench, ref, idx=True)
    assert (count, index) == (35, 40) and status & DIR


def test_decoder_counts_invalid_transitions(backend):
    """Three steps change both channels at once: each is counted as an
    error and not as a count, and the decoder carries on from the new
    state."""
    bench, src, ref, loom = decoder_setup(backend)
    events, _, _ = Q.steps(10, 0, 60, 100, invalid=(5, 22, 41))
    play(bench, src, events)
    count, status, _ = expect(loom, bench, ref)
    assert ref.errors == 3 and status >> 2 == 3
    assert count == 57


def test_decoder_zero_and_periodic_reports(backend):
    """ZERO makes the count read 0 there; PERIOD 3000 pushes a report every
    3000 clocks (tick 1 clock) while the encoder turns; PERIOD 0 stops."""
    bench, src, ref, loom = decoder_setup(backend)
    events, phase, pos = Q.steps(10, 0, 25, 100)
    play(bench, src, events)
    loom.push(1, [ZERO])
    bench.step(100)
    assert report(bench, loom)[0] == 0
    loom.push(1, [PERIOD | 3000])
    start = bench.cycle
    events, phase, pos = Q.steps(10, phase, 80, 100, pos=pos)
    src.schedule([(start + c, a, b, z) for c, a, b, z in events])
    got = []
    while len(got) < 3:
        got.append(loom.pop(1, 3))
    counts = [Q.to_signed(w[0]) for w in got]
    assert counts == sorted(counts) and 0 < counts[0] and counts[-1] <= 80
    loom.push(1, [PERIOD | 0])
    bench.step(9000)
    loom.pop_available(1)
    bench.step(7000)
    assert loom.pop_available(1) == []
    assert report(bench, loom)[0] == ref.count - 25


def run_at(backend, spacing, n, offset, index_every=0):
    """``n`` steps at ``spacing`` clocks, the first ``offset`` clocks off the
    decoder's slot grid; returns (decoder report, reference)."""
    bench, src, ref, loom = decoder_setup(backend)
    bench.step(offset)
    events, _, _ = Q.steps(10, 0, n, spacing, index_every=index_every)
    play(bench, src, events)
    return report(bench, loom), ref


def test_decoder_edge_rate_limit(backend):
    """The header's limit, measured: at LIMIT = 32 clocks per edge, the
    shortest spacing the slot arithmetic guarantees in either direction,
    every edge is counted with an index every 8 counts (the costliest
    path), in bursts of 48 steps each way at four offsets against the slot
    grid. One clock faster, the +1 direction (6 or 7 slots a step) still
    holds, and the -1 direction (8 slots) falls behind and reports invalid
    transitions. One decoder run throughout, compared with the reference
    after every burst."""
    bench, src, ref, loom = decoder_setup(backend)
    phase = pos = 0
    for offset in range(4):
        for n in (48, -48):
            bench.step(offset)
            events, phase, pos = Q.steps(10, phase, n, LIMIT, pos=pos,
                                         index_every=8)
            play(bench, src, events, tail=100)
            count, status, index = expect(loom, bench, ref, idx=True)
            assert ref.errors == 0 and count == pos and index == ref.index
    events, phase, pos = Q.steps(10, phase, 48, LIMIT - 1, pos=pos)
    play(bench, src, events, tail=100)
    count, status, _ = expect(loom, bench, ref)
    assert count == pos == 48 and status >> 2 == 0
    events, phase, pos = Q.steps(10, phase, -48, LIMIT - 1, pos=pos)
    play(bench, src, events, tail=100)
    count, status, _ = report(bench, loom)
    assert ref.errors == 0 and ref.count == pos == 0     # a clean waveform
    assert status >> 2 > 0 and count != 0


@model_only("the sweep below the limit repeats test_decoder_edge_rate_limit "
            "at more spacings; about 40,000 clocks each")
def test_decoder_edge_rate_sweep(backend):
    """Where each direction breaks: +1 steps hold down to 25 clocks per
    edge (their slot costs 24, 24, 24, 28 average 25) and fail at 24; -1
    steps hold at 32 and fail at 31, as the header's arithmetic says."""
    for n, good, bad in ((120, 25, 24), (-120, 32, 31)):
        for offset in range(4):
            (count, status, _), _ = run_at(backend, good, n, offset)
            assert count == n and status >> 2 == 0
        (count, status, _), _ = run_at(backend, bad, n, 0)
        assert count != n and status >> 2 > 0


@model_only("a HEDS-5540 at 100 kHz is 500 clocks a cycle at 50 MHz; "
            "2,000 counts are 250,000 clocks")
def test_decoder_heds5540_at_100_khz_with_worst_states(backend):
    """A 500-line HEDS-5540 (2,000 counts a turn) at its 100 kHz limit on a
    50 MHz clock, every state between 45 and 135 degE (the datasheet's
    45 degE state width error) by jitter: a full turn, index latched."""
    shortest = Q.min_edge_clocks(Q.HEDS5540_MAX_HZ, 50e6)
    assert shortest == 62.5 and shortest > LIMIT
    bench, src, ref, loom = decoder_setup(backend)
    events, _, _ = Q.steps(10, 0, 2001, 125, index_every=2000, jitter=31, seed=9)
    gaps = [b[0] - a[0] for a, b in zip(events, events[1:])]
    assert min(gaps) >= shortest
    play(bench, src, events)
    count, status, index = expect(loom, bench, ref, idx=True)
    assert (count, index) == (2001, 2000)


# ---------------------------------------------------------- the generator
def generator_setup(backend, thread=0):
    """The generator alone, the reference decoder on its pads."""
    bench = qbench(backend)
    mon = bench.add(Q.QuadDecoder("OUT0", "OUT1", "OUT2"))
    loom = backend.loom(bench, isa=ISA)
    loom.load(program(GENERATOR, thread))
    loom.run(thread)
    bench.step(STARTUP)
    return bench, mon, loom


def move(bench, mon, loom, period, n, thread=0):
    """One move; returns (A/B edge cycles, Z edges as (cycle, level), the
    reported position, the cycle the command was in INQ)."""
    seen = len(mon.edges)
    loom.push(thread, [period, n & 0xFFFF])
    pushed = bench.cycle
    pos = Q.to_signed(loom.pop(thread, 1)[0])
    bench.step(50)
    ab, z = [], []
    prev = mon.edges[seen - 1]
    for e in mon.edges[seen:]:
        if e[1:3] != prev[1:3]:
            ab.append(e[0])
        if e[3] != prev[3]:
            z.append((e[0], e[3]))
        prev = e
    return ab, z, pos, pushed


def check_move(ab, z, period, n, pushed):
    """|n| A/B edges exactly ``period`` apart, the first no later than two
    periods and the command's handling after the push; each Z edge 6 to 9
    clocks after an A/B edge."""
    p = max(period, PMIN)
    assert len(ab) == abs(n)
    assert all(b - a == p for a, b in zip(ab, ab[1:]))
    assert 0 < ab[0] - pushed <= 3 * p + 80
    for cyc, _ in z:
        lag = min(cyc - e for e in ab if e <= cyc)
        assert 6 <= lag <= 9


def test_generator_edges_are_exact(backend):
    """PHY-GL-SDF candidate. The shortest period, 40 clocks, 24 steps out
    and 24 back with the index every 8 steps: every A/B edge exactly 40
    clocks after the one before, the Gray sequence and the index as the
    reference decoder reads them, the position reported after each move."""
    bench, mon, loom = generator_setup(backend)
    loom.push(0, [INDEX | 8])
    ab, z, pos, pushed = move(bench, mon, loom, PMIN, 24)
    check_move(ab, z, PMIN, 24, pushed)
    assert pos == mon.count == 24 and mon.errors == 0 and mon.dir == 0
    assert mon.index == 24 and mon.indexes == 3
    assert [lvl for _, lvl in z] == [1, 0, 1, 0, 1]
    ab, z, pos, pushed = move(bench, mon, loom, PMIN, -24)
    check_move(ab, z, PMIN, -24, pushed)
    assert pos == mon.count == 0 and mon.errors == 0 and mon.dir == 1
    assert mon.index == 0 and mon.indexes == 6
    assert bench.contention_free() and loom.badop() == 0


def test_generator_periods_and_resume(backend):
    """Any period, a number of clocks that is not a multiple of the 4-clock
    slot, is exact (the deadline latch); a period below 40 is raised to 40;
    each move resumes from the phase the one before ended in. Moves of 9,
    -6, 5 and -11 steps at 57, 10, 203 and 41 clocks."""
    bench, mon, loom = generator_setup(backend)
    at = 0
    for period, n in ((57, 9), (10, -6), (203, 5), (41, -11)):
        ab, z, pos, pushed = move(bench, mon, loom, period, n)
        check_move(ab, z, period, n, pushed)
        at += n
        assert pos == mon.count == at and z == [] and mon.errors == 0


# ---------------------------------------------------------------- together
def test_generator_and_decoder_together(backend):
    """The decoder on threads 0 and 1, the generator on thread 2, the bench
    wiring OUT0..OUT2 to IN0..IN2 as three board wires: moves out, back
    past the index and out again at 40 and 61 clocks per edge, and the
    decoder's reports agree with the generator's positions."""
    bench = qbench(backend)
    for k in range(3):
        bench.add(Q.Wire("OUT%d" % k, "IN%d" % k))
    ref = bench.add(Q.QuadDecoder())
    loom = backend.loom(bench, isa=ISA)
    words = dict(program(DECODER).words)
    gen = program(GENERATOR, thread=2).words
    assert not set(words) & set(gen)
    words.update(gen)
    loom.load(words)
    loom.run([0, 1, 2])
    bench.step(STARTUP)
    loom.push(2, [INDEX | 16])
    for period, n, at in ((PMIN, 30, 30), (61, -41, -11), (PMIN, 20, 9)):
        loom.push(2, [period, n & 0xFFFF])
        assert Q.to_signed(loom.pop(2, 1)[0]) == at
        bench.step(100)
        count, status, index = expect(loom, bench, ref)
        assert count == at and status & DIR == (n < 0) and status >> 2 == 0
    assert ref.indexes == 4 and index == 0
    assert bench.contention_free() and loom.badop() == 0


# ------------------------------------------------- the reference model alone
def test_model_counting_rules():
    """The reference decoder from the cited rules: +1 along 00 10 11 01,
    -1 back, a double change an error, Z latched in 00 once a visit."""
    ev, ph, pos = Q.steps(0, 0, 8, 10)
    assert [e[1:3] for e in ev[:4]] == [(1, 0), (1, 1), (0, 1), (0, 0)]
    assert Q.decode(ev).count == 8 and (ph, pos) == (0, 8)
    back, _, _ = Q.steps(100, ph, -3, 10, pos=pos)
    dec = Q.decode(ev + back)
    assert dec.count == 5 and dec.dir == 1 and dec.errors == 0
    bad, _, pos = Q.steps(0, 0, 4, 10, invalid=(1,))
    dec = Q.decode(bad)
    assert pos == 5 and dec.count == 3 and dec.errors == 1
    idx, _, _ = Q.steps(0, 0, 12, 10, index_every=4)
    dec = Q.decode(idx)
    assert dec.indexes == 3 and dec.index == 12
    assert Q.min_edge_clocks(100_000, 50e6) == 62.5
    assert Q.to_signed(0xFFFF) == -1 and Q.to_signed(-32768) == -32768
