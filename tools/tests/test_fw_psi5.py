"""L3 for the PSI5 programs, on the model and the RTL.

``firmware/psi5_sensor.loom`` (DATA = OUT0, SYNC = IN1) sends 10-bit PSI5
frames in a slot after each sync pulse or at a fixed period;
``firmware/psi5_ecu.loom`` (SYNC = OUT1 from thread 0, DATA = IN0 to thread
1) sends the sync pulses and decodes up to two slots. The reference is
``tools.protomodels.psi5``: ``decode`` reads a recorded line and checks the
start bits, the Manchester code and the parity or CRC; ``Sensor`` is a model
sensor with faults; ``Bus`` is the line, the OR of every sender (current adds).

Expected values come from the PSI5 descriptions in the model's docstring
(vendor documents citing the specification; slot offsets and sync widths are
the tests' own, unchecked) and from SEMANTICS: half-bit edges one slot after
their tick on a non-drifting grid (2, 6.9.1), so with a tick a multiple of 4
clocks every bit is exactly two ticks; the sync pulse's edges on the exact
clock edge of their tick (6.10).

The board. The RTL runs everything on a board clocked at 16 MHz: a 125
kbit/s bit is 128 clocks, a half-bit (the sensor's tick) 64, the ECU's 1/8
bit and 1 us tick 16, and a 500 us sync period 8,000 clocks. The 50 MHz and
189 kbit/s bodies are ``model_only``.
"""

import pytest

from tools.loomasm import assemble_text
from tools.loomisa import REPO, load
from tools.protomodels import psi5 as P
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
IMEM = 512
CLOCK = 16_000_000
BIT = 128                       # clocks per bit, 125 kbit/s at 16 MHz
HALF = 64
US = 16                         # clocks per us
PERIOD_US = 500                 # the P10P-500 sync period
WIDTH_US = 16                   # our sync pulse width (unchecked)

# sensor configuration word bits (psi5_sensor.loom header)
ASYNC, CRC, HOST = 1, 2, 4


def program(name, thread=0, subs=()):
    """A program from ``firmware/``, its threads moved by ``thread`` and the
    text substitutions ``subs`` applied (another pin)."""
    text = (FIRMWARE / (name + ".loom")).read_text(encoding="utf-8")
    for old, new in subs:
        assert text.count(old) == 1, old
        text = text.replace(old, new)
    if thread:
        for t in (1, 0):
            if (".thread %d" % t) in text:
                text = text.replace(".thread %d" % t, ".thread %d" % (t + thread))
    return assemble_text(text, name + ".loom", isa=ISA, imem_words=IMEM, strict=True)


#: The second sensor's data pin, for two Loom sensors on one bus.
SENSOR_B = (("        .pins   DATA = OUT0", "        .pins   DATA = OUT2"),
            ("DATA_IX = 16", "DATA_IX = 18"))


def psi5_bench(backend):
    """The FIFOs, the deadline latch, the bit engine with its encoders and the
    512-word memory; the RTL has them all and ``RtlBench`` takes neither."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "SETPD", "BE", "BEENC"), imem_words=IMEM)
    return backend.bench()


def start(backend, bench, loads):
    """Load ``[(program, {thread: (TICK_INT, TICK_FRAC)}, {thread: words})]``,
    set the ticks, run every thread, then push the configuration words."""
    loom = backend.loom(bench, isa=ISA)
    words = {}
    for prog, _, _ in loads:
        words.update(prog.words)
    loom.load(words)
    threads = []
    for _, ticks, _ in loads:
        for t, (tint, tfrac) in ticks.items():
            loom.write_csr(t, "TICK_INT", tint)
            loom.write_csr(t, "TICK_FRAC", tfrac)
            threads.append(t)
    loom.run(sorted(threads))
    for _, _, config in loads:
        for t, ws in config.items():
            loom.push(t, ws)
    return loom


def sensor_load(cfg, word2, thread=0, half=(HALF, 0), subs=()):
    return (program("psi5_sensor", thread, subs), {thread: half}, {thread: [cfg, word2]})


def ecu_load(cfg=0, win1=(2, 10), win2=(0, 0), period=PERIOD_US, width=WIDTH_US,
             tick=(US, 0), rx_tick=(US, 0)):
    """The ECU on threads 0 and 1: sync period and width in us (thread 0's
    1 us tick), windows (earliest, latest) in bits after the sync."""
    w = [win1[0] | (win1[1] << 8), win2[0] | (win2[1] << 8)]
    return (program("psi5_ecu"), {0: tick, 1: rx_tick},
            {1: [cfg] + w, 0: [period, width]})    # the receiver first


def edges_of(rec, since=0):
    return [(c, lv) for c, lv in rec.edges if c >= since]


def check_frames(got, datas, bit=BIT, exact=True):
    """Every frame clean, the data in order, and (``exact``) every bit
    exactly ``bit`` clocks with its boundary edge exactly half way."""
    assert [g.data for g in got] == list(datas), [hex(g.data) for g in got]
    for g in got:
        assert g.errors == [], g.errors
        assert P.check_timing(g, bit) == []
        if exact:
            assert {b - a for a, b in zip(g.mids, g.mids[1:])} == {bit}
            for e in g.bounds:
                assert e - max(m for m in g.mids if m < e) == bit // 2


# ------------------------------------------------- the sensor, model ECU
OFFSET = 10                     # slot offset, half-bit ticks: 40 us


def sensor_setup(backend, cfg, word2, half=(HALF, 0), data=()):
    bench = psi5_bench(backend)
    rec = bench.add(P.EdgeRecorder("OUT0"))
    prog, ticks, config = sensor_load(cfg, word2, half=half)
    config[0] += list(data)
    loom = start(backend, bench, [(prog, ticks, config)])
    return bench, loom, rec


def run_syncs(bench, n, period=PERIOD_US * US, width=WIDTH_US * US, lead=1000):
    """``n`` sync pulses on IN1 from ``lead`` clocks on; run past the last
    period. Returns the rising edges' cycles (first cycle high on the pad)."""
    src = bench.add(P.SyncSource("IN1", period, width, bench.cycle + lead, count=n))
    rises = src.rises(bench.cycle + lead + n * period)
    bench.step(lead + n * period)
    return rises


def slot_lags(got, rises, half=HALF):
    """For each frame, clocks from its sync's rising edge to S1's start
    (half a bit before the first edge)."""
    return [g.first - half - max(r for r in rises if r < g.first) for g in got]


@pytest.mark.parametrize("crc", [False, True])
def test_sensor_sync_counter(backend, crc):
    """Synchronous mode, the counter as data: three syncs, three frames 0, 1,
    2, each in its slot (offset 10 half-bits, S1 starting 640 + 1..16
    clocks after the sync edge: the synchroniser, the slot grid and the
    tick restart, header), every bit exactly 128 clocks. The PHY-GL-SDF
    candidate is the parity case (three periods, 26,000 clocks)."""
    bench, loom, rec = sensor_setup(backend, CRC if crc else 0, OFFSET)
    rises = run_syncs(bench, 3)
    got = P.decode(rec.edges, BIT, crc=crc)
    check_frames(got, [0, 1, 2])
    for lag in slot_lags(got, rises):
        assert OFFSET * HALF + 1 <= lag <= OFFSET * HALF + 16, lag
    assert loom.badop() == 0


def test_sensor_host_words_and_repeat(backend):
    """Host data, parity: one word with the configuration, then (INQ empty)
    the same word again; two words pushed during the second period reach
    the frames after the one already prepared (header: a frame is prepared
    as soon as the previous one ends)."""
    bench, loom, rec = sensor_setup(backend, HOST, OFFSET, data=[0x2A5])
    run_syncs(bench, 2)
    loom.push(0, [0x3FF, 0x001])
    run_syncs(bench, 3)
    check_frames(P.decode(rec.edges, BIT), [0x2A5, 0x2A5, 0x2A5, 0x3FF, 0x001])


def test_sensor_async_period(backend):
    """Asynchronous mode with the CRC, period 57 half-bits (228 us): the
    S1 starts of successive frames exactly 57 * 64 clocks apart."""
    bench, loom, rec = sensor_setup(backend, ASYNC | CRC | HOST, 57,
                                    data=[0x155, 0x0AA, 0x3C3])
    bench.step(4 * 57 * HALF + 2000)
    got = P.decode(rec.edges, BIT, crc=True)
    check_frames(got[:4], [0x155, 0x0AA, 0x3C3, 0x3C3])
    assert {b.first - a.first for a, b in zip(got, got[1:])} == {57 * HALF}


@model_only("189 kbit/s at 50 MHz, a fractional tick: 132.28 clocks a half-"
            "bit; the program's schedule is in ticks, proved on the RTL at "
            "125 kbit/s")
def test_sensor_189k_at_50mhz(backend):
    """TICK_INT 132, TICK_FRAC 72: every bit 264 or 268 clocks (the slot
    grid, SEMANTICS 2), the frame's length 15 bits of 264.55 within a slot."""
    bit = 50e6 / 189e3
    bench, loom, rec = sensor_setup(backend, CRC, OFFSET, half=(132, 72))
    run_syncs(bench, 2, period=25_000, width=800)
    got = P.decode(rec.edges, bit, crc=True)
    check_frames(got, [0, 1], bit=bit, exact=False)
    for g in got:
        assert {b - a for a, b in zip(g.mids, g.mids[1:])} <= {264, 268}
        assert abs((g.mids[-1] - g.mids[0]) - 14 * bit) <= 4


# --------------------------------------------------- the ECU, model sensors
TWO = 2                         # ECU configuration: two slots (bit 1)
WIN1, WIN2 = (2, 10), (20, 30)  # slot windows, bits after the sync
OFF1_US, OFF2_US = 40, 200      # the model sensors' slot offsets


def ecu_setup(backend, cfg, sensors=(), chips=(), before=(), win2=(0, 0), **kw):
    """The ECU on threads 0 and 1, any Loom sensors in ``before`` (loaded
    and configured first), the bus on IN0 from ``chips`` and the model
    ``sensors``, and recorders on SYNC (OUT1) and the bus."""
    bench = psi5_bench(backend)
    sync = bench.add(P.EdgeRecorder("OUT1"))
    bus = bench.add(P.EdgeRecorder("IN0"))
    bench.add(P.Bus("IN0", chips, sensors))
    loom = start(backend, bench, list(before) + [ecu_load(cfg, WIN1, win2, **kw)])
    return bench, loom, sync, bus


def collect(bench, loom, periods, period=PERIOD_US * US):
    """Run ``periods`` sync periods (and a little), reading OUTQ[1]."""
    end = bench.cycle + periods * period + 1000
    words = []
    while bench.cycle < end:
        bench.step(min(2000, end - bench.cycle))
        words += loom.pop_available(1)
    return words


def check_syncs(rec, count, period=PERIOD_US * US, width=WIDTH_US * US):
    """At least ``count`` sync pulses, every one exactly ``width`` clocks and
    every period exactly ``period`` (SEMANTICS 6.10)."""
    got = P.pulses(rec.edges)
    assert len(got) >= count, got
    assert {f - r for r, f in got} == {width}
    assert {b[0] - a[0] for a, b in zip(got, got[1:])} == {period}
    return got


@pytest.mark.parametrize("crc", [False, True])
def test_ecu_with_a_model_sensor(backend, crc):
    """Three sync periods of 500 us with 16 us pulses, exact; a model sensor
    in slot 1 (40 us); every frame reported with its data and no flag."""
    datas = [0x155, 0x2AA, 0x3FF]
    s = P.Sensor(CLOCK, crc=crc, sync="OUT1", offset_us=OFF1_US)
    s.queue_frames([(d, P.Fault()) for d in datas])
    bench, loom, sync, _ = ecu_setup(backend, 1 if crc else 0, [s])
    words = collect(bench, loom, 3)
    check_syncs(sync, 3)
    assert words[:3] == [P.report(d) for d in datas], [hex(w) for w in words]
    assert loom.badop() == 0


def test_ecu_reports_errors(backend):
    """CRC frames from a model sensor, one fault each: a data bit flipped (a
    CRC error, the data as received), a wrong CRC, S2 sent as 1, a bit with
    no mid-bit edge (reading stops), a frame 190 us late (none in the
    window), then a good frame."""
    faults = [(0x0F0, P.Fault(flip=3)), (0x0F0, P.Fault(check=P.crc3(P.data_bits(0x0F0)) ^ 5)),
              (0x123, P.Fault(flip=-1)), (0x123, P.Fault(violate=6)),
              (0x321, P.Fault(late=190 * US)), (0x321, P.Fault())]
    s = P.Sensor(CLOCK, crc=True, sync="OUT1", offset_us=OFF1_US)
    s.queue_frames(faults)
    bench, loom, _, _ = ecu_setup(backend, 1, [s])
    words = collect(bench, loom, 6)
    assert words[:6] == [P.report(0x0F8, 0, P.E_CHECK), P.report(0x0F0, 0, P.E_CHECK),
                         P.report(0x123, 0, P.E_START), P.report(0, 0, P.E_MANCH),
                         P.report(0, 0, P.E_NONE), P.report(0x321)], [hex(w) for w in words]


@model_only("sensor clocks 10 % slow and fast, four periods; the windows "
            "are the same instructions at every rate")
@pytest.mark.parametrize("scale", [0.9, 1.1])
def test_ecu_follows_the_sensor_clock(backend, scale):
    """A sensor bit time 10 % long or short: the ECU resynchronises on every
    mid-bit edge, so every frame reads clean (header: 12 % / 20 %)."""
    datas = [0x000, 0x3FF, 0x2A5, 0x15A]
    s = P.Sensor(CLOCK, crc=True, sync="OUT1", offset_us=OFF1_US, scale=scale)
    s.queue_frames([(d, P.Fault()) for d in datas])
    bench, loom, _, _ = ecu_setup(backend, 1, [s])
    assert collect(bench, loom, 4)[:4] == [P.report(d) for d in datas]


# ------------------------------------------------------ programs together
def loom_sensor(cfg, offset, thread, subs=(), data=()):
    """A Loom sensor on ``thread`` for ``ecu_setup``'s ``before``."""
    prog, ticks, config = sensor_load(cfg, offset, thread, subs=subs)
    config[thread] += list(data)
    return prog, ticks, config


def test_together_sensor_and_model_two_slots(backend):
    """The ECU (threads 0, 1) with two slots; the Loom sensor on thread 2
    (counter, parity, slot 1 at 10 half-bits) on SYNC through a board wire
    OUT1 -> IN1, and a model sensor in slot 2 (200 us). Three periods, six
    reports in order; the Loom sensor's bits exact on the bus."""
    m = P.Sensor(CLOCK, sync="OUT1", offset_us=OFF2_US)
    m.queue_frames([(d, P.Fault()) for d in (0x3E1, 0x01F, 0x200)])
    bench = psi5_bench(backend)
    bench.add(P.Wire("OUT1", "IN1"))
    rec = bench.add(P.EdgeRecorder("OUT0"))
    sensor = loom_sensor(0, OFFSET, 2)
    sync = bench.add(P.EdgeRecorder("OUT1"))
    bench.add(P.Bus("IN0", ["OUT0"], [m]))
    loom = start(backend, bench, [sensor, ecu_load(TWO, WIN1, WIN2)])
    words = collect(bench, loom, 3)
    assert words[:6] == [P.report(0), P.report(0x3E1, 1), P.report(1), P.report(0x01F, 1),
                         P.report(2), P.report(0x200, 1)], [hex(w) for w in words]
    check_frames(P.decode(rec.edges, BIT)[:3], [0, 1, 2])
    rises = [r for r, _ in check_syncs(sync, 3)]
    for lag in slot_lags(P.decode(rec.edges, BIT)[:3], rises):
        assert OFFSET * HALF + 2 <= lag <= OFFSET * HALF + 17, lag     # + the wire


def test_together_two_loom_sensors(backend):
    """All four threads: the ECU (CRC, two slots) and two Loom sensors with
    the CRC, A on thread 2 (DATA OUT0, counter, slot 1) and B on thread 3
    (DATA OUT2, host words, slot 2 at 50 half-bits = 200 us); the bus is the
    OR of OUT0 and OUT2. Every frame on the bus is clean and exact, and the
    ECU reports both slots each period."""
    bench = psi5_bench(backend)
    bench.add(P.Wire("OUT1", "IN1"))
    bus = bench.add(P.EdgeRecorder("IN0"))
    bench.add(P.Bus("IN0", ["OUT0", "OUT2"]))
    a = loom_sensor(CRC, OFFSET, 2)
    b = loom_sensor(CRC | HOST, 50, 3, SENSOR_B, data=[0x0C3, 0x3C0, 0x111])
    loom = start(backend, bench, [a, b, ecu_load(1 | TWO, WIN1, WIN2)])
    words = collect(bench, loom, 3)
    assert words[:6] == [P.report(0), P.report(0x0C3, 1), P.report(1), P.report(0x3C0, 1),
                         P.report(2), P.report(0x111, 1)], [hex(w) for w in words]
    check_frames(P.decode(bus.edges, BIT, crc=True)[:6],
                 [0, 0x0C3, 1, 0x3C0, 2, 0x111])
    assert bench.contention_free() and loom.badop() == 0


@model_only("189 kbit/s at 50 MHz, two periods of 25,000 clocks; the 16 MHz "
            "runs prove the same instructions")
def test_together_189k_at_50mhz(backend):
    """The Loom sensor (thread 2, TICK 132 + 72/256, CRC, slot at 16 half-
    bits) and the ECU at 189 kbit/s: thread 0's tick 1 us (50 clocks), the
    receiver's 1/8 bit (33 + 18/256 clocks), windows in 5.29 us bits."""
    bench = psi5_bench(backend)
    bench.add(P.Wire("OUT1", "IN1"))
    bench.add(P.Bus("IN0", ["OUT0"]))
    sensor = (program("psi5_sensor", 2), {2: (132, 72)}, {2: [CRC, 16]})
    loom = start(backend, bench, [sensor, ecu_load(1, (2, 20), tick=(50, 0),
                                                   rx_tick=(33, 18))])
    words = collect(bench, loom, 2, period=PERIOD_US * 50)
    assert words[:2] == [P.report(0), P.report(1)], [hex(w) for w in words]


# --------------------------------------------- the programs and the model
#: program -> (words, threads, deadline pairs per thread or None for none)
PROGRAMS = {"psi5_sensor": (96, [0], [12]), "psi5_ecu": (125, [0, 1], [7, 0])}


@pytest.mark.parametrize("name", sorted(PROGRAMS))
def test_programs_assemble_strict_with_no_diagnostic(name):
    """--strict, no diagnostic, every pair proved; the sizes the README
    quotes; each thread inside its 128-word quarter. The ECU's receiver has
    no deadline pair: its windows are SETD and timed waits (header)."""
    words, threads, pairs = PROGRAMS[name]
    prog = program(name)
    assert prog.diagnostics == []
    assert len(prog.words) == words
    assert sorted(prog.threads) == threads
    for t, n in zip(threads, pairs):
        rep = prog.deadlines[t]
        assert len(rep.pairs) == n and not rep.infeasible and not rep.unbounded
        assert all(t * 128 <= a < (t + 1) * 128 for a in prog.words if a // 128 == t)
    assert max(prog.words) < 128 * len(threads)


def test_all_four_threads_fit_together():
    """The ECU on threads 0-1 and two sensors on 2-3 share no word."""
    ws = [program("psi5_ecu").words, program("psi5_sensor", 2).words,
          program("psi5_sensor", 3, SENSOR_B).words]
    addrs = [set(w) for w in ws]
    assert not (addrs[0] & addrs[1]) and not (addrs[1] & addrs[2]) and not (addrs[0] & addrs[2])


def test_model_parity_and_crc():
    """Even parity; the CRC x^3 + x + 1 with seed 111: appending it (C2
    first) leaves a zero register, the property the ECU checks with; a
    one-bit change in the data always changes it (a CRC-3 detects every
    single-bit error)."""
    assert P.parity(0) == 0 and P.parity(1) == 1 and P.parity(0x3FF) == 0
    assert P.crc3([]) == P.SEED
    for d in range(1024):
        bits = P.data_bits(d)
        c = P.crc3(bits)
        assert P.crc3(bits + [(c >> 2) & 1, (c >> 1) & 1, c & 1]) == 0
        for i in range(10):
            assert P.crc3(P.data_bits(d ^ (1 << i))) != c
    assert len(P.frame_bits(0x155)) == 13 and len(P.frame_bits(0x155, True)) == 15
    assert P.frame_bits(0x001)[:3] == [0, 0, 1]                 # S1 S2 D0, LSB first
    assert P.half_levels([0, 1]) == [0, 1, 1, 0]                # 0 rises, 1 falls


def _line(sensor, upto):
    edges, lv = [], 0
    for c in range(upto):
        x = sensor.level(c)
        if x != lv:
            edges.append((c, x))
            lv = x
    return edges


@pytest.mark.parametrize("crc", [False, True])
def test_model_decoder(crc):
    """The model sensor and decoder agree, faults are named, and the timing
    checker passes an exact sensor and fails one 20 % slow."""
    s = P.Sensor(CLOCK, crc=crc)
    t = 100
    plan = [(0x2A5, P.Fault(), []), (0x2A5, P.Fault(flip=4), ["check"]),
            (0x111, P.Fault(flip=-1), ["start"]), (0x3FF, P.Fault(violate=8), ["manchester"])]
    for d, f, _ in plan:
        t = s.send(d, t, f) + 3 * BIT
    got = P.decode(_line(s, t), BIT, crc=crc)
    assert [g.errors for g in got] == [e for _, _, e in plan]
    assert got[0].data == 0x2A5 and P.check_timing(got[0], BIT) == []
    slow = P.Sensor(CLOCK, crc=crc, scale=1.2)
    slow.send(0x2A5, 100)
    g = P.decode(_line(slow, 3000), BIT * 1.2, crc=crc)[0]
    assert g.ok and P.check_timing(g, BIT) != []
