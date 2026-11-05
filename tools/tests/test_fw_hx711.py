"""L3 for the HX711 programs, on the model and the RTL.

``firmware/hx711_host.loom`` (thread 0; PD_SCK = OUT0, DOUT = IN0) reads
an HX711; ``firmware/hx711_device.loom`` (thread 1; PD_SCK = IN1, DOUT =
OUT1) emulates one. The references are ``tools.protomodels.hx711``: the
chip (``Hx711Device``) and a host (``Hx711Host``) written from the Avia
datasheet, and ``check_pd_sck``, the T3/T4 windows. Expected values come
from the datasheet (Table 3's pulse counts, T1 to T4, the 60 us power-down,
24-bit two's complement) and from the program headers' stated limits (the
device's DOUT 9 to 16 clocks after a rise, a period of 24 clocks), never
from what the model or the RTL did.

The PHY-GL-SDF candidate is ``test_hx711_host_reads_each_gain``: three
reads at TICK 25, every PD_SCK edge checked to the clock against the tick
and against T3/T4, the pulse counts checked by the reference chip.
"""

import pytest

from tools.loomasm import assemble_file, assemble_text
from tools.loomisa import REPO, load
from tools.protomodels import hx711 as H
from tools.protomodels.bench import Drive, Model, pad_of
from tools.protomodels.quadrature import Wire
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
IMEM = 512
HOST = FIRMWARE / "hx711_host.loom"
DEVICE = FIRMWARE / "hx711_device.loom"
CLOCK = 50e6

TIMEOUT, POWER_DOWN, WAKE = 0x4000, 0x8000, 0x8001     # host commands
E_OK, E_TIMEOUT, E_DOWN, E_STUCK = 0, 1, 2, 3          # host status codes
CONV, SETR = 0x4000, 0x8000                            # device commands
PD_REPORT, ERR = 0x4000, 0x8000                        # device reports
HOST_TICK = 25                  # half an SCK period on most host tests
DEVICE_TICK = 50                # 1 us at 50 MHz: PD = 60 us
TBL = 0x0FA                     # the device's readings

VALUES = (0x123456, -0x234567, 0x7FFFFF)        # positive, negative, full
STARTUP = 300


def bench_for(backend):
    if backend.name == "model":
        return backend.bench(features=("FIFO", "SETPD", "BE", "DMEM"),
                             imem_words=IMEM)
    return backend.bench()


def device_program(readings=(0, 0, 0)):
    """The device with its six reading words set in the image."""
    text = DEVICE.read_text(encoding="utf-8")
    words = []
    for value in readings:
        code = H.to_code(value)
        words += [(code >> 16) << 8, code & 0xFFFF]
    line = "        .word   0, 0, 0, 0, 0, 0"
    assert text.count(line) == 1
    text = text.replace(line, "        .word   " + ", ".join("0x%04X" % w for w in words))
    return assemble_text(text, DEVICE.name, isa=ISA, imem_words=IMEM, strict=True)


def reply(loom, polls=None):
    """One READ reply: (error, input, signed value)."""
    status, high, low = loom.pop(0, 3, max_polls=polls)
    value = (high << 16) | low
    if value & 0x80000000:
        value -= 1 << 32
    return status & 0xF, (status >> 8) & 3, value


def test_hx711_programs_assemble_strict_with_no_diagnostic():
    """L2-DEADLINE: the host's every pair is proved at its .tick of 24
    clocks, nothing unbounded, no ``.bounded``; the device has no pair (its
    deadlines are timeouts). Word counts as the headers give them; the two
    fit threads 0 and 1 together."""
    host = assemble_file(HOST, isa=ISA, strict=True)
    assert host.diagnostics == [] and list(host.threads) == [0]
    rep = host.deadlines[0]
    assert rep.period == 24 and rep.pairs and not rep.infeasible
    assert not rep.unbounded and rep.worst_slack >= 0
    assert len(host.words) == 91 and max(host.words) < 0x80
    dev = device_program()
    assert dev.diagnostics == [] and list(dev.threads) == [1]
    assert all(r.pairs == [] for r in dev.deadlines.values())
    code = [a for a in dev.words if a < TBL]
    assert len(code) == 118 and min(code) == 0x80 and max(code) == 0xF5
    assert sorted(a for a in dev.words if a >= TBL) == list(range(TBL, 0x100))
    assert not set(host.words) & set(dev.words)


# ------------------------------------------------------------- the host
def host_setup(backend, tick=HOST_TICK, clock=CLOCK, **chip):
    """The host on thread 0, the reference chip on OUT0/IN0, a log of
    PD_SCK."""
    bench = bench_for(backend)
    chip.setdefault("conv", 600)
    dev = bench.add(H.Hx711Device("OUT0", "IN0", clock_hz=clock, **chip))
    log = bench.add(H.EdgeLog("OUT0"))
    loom = backend.loom(bench, isa=ISA)
    loom.load(assemble_file(HOST, isa=ISA, strict=True))
    loom.write_csr(0, "TICK_INT", tick)
    loom.run(0)
    bench.step(STARTUP)
    return bench, dev, log, loom


def exact_pulses(log, since, tick, count, clock=CLOCK):
    """The PD_SCK pulses after ``since``: ``count`` of them, each high one
    tick exactly and low one tick exactly between them (the staged edges
    land on their ticks), and inside T3 and T4."""
    edges = [e for e in log.edges if e[0] >= since]
    pulses = H.pulses_of(edges)
    assert len(pulses) == count
    assert all(f - r == tick for r, f in pulses)
    assert all(b[0] - a[1] == tick for a, b in zip(pulses, pulses[1:]))
    assert H.check_pd_sck(edges, clock) == []
    return pulses


def test_hx711_host_reads_each_gain(backend):
    """Three READs asking for B/32, A/64, A/128 next: the chip sees 26, 27
    and 25 pulses (Table 3), so the readings are of A/128 (after reset),
    B/32 and A/64, and each reply says so; positive, negative and full
    scale come back sign-extended to 32 bits. Every edge is exact to the
    tick; T1 (0.1 us) is kept many times over."""
    bench, dev, log, loom = host_setup(backend, readings=dict(enumerate(VALUES)))
    for sel, expect_input in ((1, 0), (2, 1), (0, 2)):
        since = bench.cycle
        loom.push(0, [sel])
        err, inp, value = reply(loom)
        assert (err, inp, value) == (E_OK, expect_input, VALUES[expect_input])
        exact_pulses(log, since, HOST_TICK, 25 + sel)
    assert [p[:2] for p in dev.periods] == [(26, 1), (27, 2), (25, 0)]
    assert not any(p[2] for p in dev.periods)
    assert all(t >= H.clocks(H.T1_MIN, CLOCK) for t in dev.t1)


def tick_window(backend, tick, clock, t2):
    """Two reads at ``tick``: every high and low exactly one tick, inside
    T3 and T4 at ``clock``, the readings right, no power-down."""
    bench, dev, log, loom = host_setup(backend, tick=tick, clock=clock, t2=t2,
                                       readings={0: -1, 1: 0x400001})
    for sel, expect in ((1, (E_OK, 0, -1)), (0, (E_OK, 1, 0x400001))):
        since = bench.cycle
        loom.push(0, [sel])
        assert reply(loom, polls=20000) == expect
        exact_pulses(log, since, tick, 25 + sel, clock=clock)
    assert not dev.power_downs


@model_only("the tick sweep: the 25-clock case runs on the RTL")
@pytest.mark.parametrize("tick", [24, 50])
def test_hx711_host_tick_window(backend, tick):
    """The shortest proved tick (24 clocks, 0.48 us at 50 MHz) and the
    datasheet's typical 1 us, with a chip whose DOUT lags each rise by
    T2's full 0.1 us (5 clocks)."""
    tick_window(backend, tick, CLOCK, 5)


@model_only("T3 max: 50 us highs; on a 5 MHz clock to keep it short")
def test_hx711_host_t3_max(backend):
    """T3's maximum, 50 us high and low, on a 5 MHz board clock (tick 250):
    exact, inside the windows, and the chip does not power down."""
    tick_window(backend, 250, 5e6, 1)


def test_hx711_host_timeout(backend):
    """A chip that never goes ready: the READ times out after its N units of
    128 ticks and sends no pulse; status 1 and a zero reading."""
    bench, dev, log, loom = host_setup(backend, never_ready=True)
    loom.push(0, [TIMEOUT | 2])
    since = bench.cycle
    loom.push(0, [1])
    assert reply(loom) == (E_TIMEOUT, 0, 0)
    assert bench.cycle - since >= 2 * 128 * HOST_TICK
    assert [e for e in log.edges if e[0] >= since] == []


def test_hx711_host_power_down_and_wake(backend):
    """0x8000 holds PD_SCK high: the chip powers down after 60 us; a READ
    then sends nothing and says so (status 2); 0x8001 lets it go, the chip
    resets to A/128, and the next reading is of A/128 whatever the read
    before it asked for."""
    bench, dev, log, loom = host_setup(backend, readings=dict(enumerate(VALUES)))
    loom.push(0, [2])
    assert reply(loom) == (E_OK, 0, VALUES[0])          # asks for A/64 next
    loom.push(0, [POWER_DOWN])
    bench.step(int(H.clocks(H.PD_TIME, CLOCK)) + 200)
    assert len(dev.power_downs) == 1
    loom.push(0, [0])
    assert reply(loom) == (E_DOWN, 2, 0)
    loom.push(0, [WAKE])
    loom.push(0, [0])
    assert reply(loom) == (E_OK, 0, VALUES[0])
    assert len(dev.wakes) == 1


class StuckLow(Model):
    """DOUT held low: no chip on the line, or a shorted one."""

    def __init__(self, pin):
        self.pad = pad_of(pin)

    def drive(self, drive: Drive, cycle: int) -> None:
        drive.set(self.pad, 0)


def test_hx711_host_sees_dout_stuck_low(backend):
    """DOUT never goes back high after the 25th pulse: the read completes
    (24 zero bits) and the status says the reading is not to be trusted."""
    bench = bench_for(backend)
    bench.add(StuckLow("IN0"))
    log = bench.add(H.EdgeLog("OUT0"))
    loom = backend.loom(bench, isa=ISA)
    loom.load(assemble_file(HOST, isa=ISA, strict=True))
    loom.write_csr(0, "TICK_INT", HOST_TICK)
    loom.run(0)
    since = bench.cycle
    loom.push(0, [0])
    assert reply(loom) == (E_STUCK, 0, 0)
    exact_pulses(log, since, HOST_TICK, 25)


# ----------------------------------------------------------- the device
DEV_CONV = 20                   # ticks: 20 us at 50 MHz


def device_setup(backend, readings=VALUES, tick=DEVICE_TICK, **host):
    """The device on thread 1 with ``readings`` in its image and a
    20-tick conversion; after its start-up, a reference host on IN1/OUT1
    and a log of DOUT."""
    bench = bench_for(backend)
    loom = backend.loom(bench, isa=ISA)
    loom.load(device_program(readings))
    loom.write_csr(1, "TICK_INT", tick)
    loom.run(1)
    loom.push(1, [CONV | DEV_CONV])
    bench.step(STARTUP)
    log = bench.add(H.EdgeLog("OUT1", initial=1))
    ref = bench.add(H.Hx711Host("IN1", "OUT1", **host))
    return bench, ref, log, loom


def reads_done(bench, ref, n, limit=400000):
    assert bench.run_until(lambda: len(ref.reads) + len(ref.abandoned) >= n,
                           limit, every=200)


def reports(loom, n):
    return loom.pop(1, n)


def test_hx711_device_serves_each_gain(backend):
    """A host at the datasheet's typical 1 us high and low gives 26, 27, 25
    and 25 pulses: the readings are of A/128 (after start), B/32, A/64 and
    A/128, and the reports name each input and the count that chose it."""
    bench, ref, log, loom = device_setup(backend, plan=[26, 27, 25, 25])
    reads_done(bench, ref, 4)
    assert [r[1] for r in ref.reads] == [VALUES[0], VALUES[1], VALUES[2], VALUES[0]]
    assert reports(loom, 5) == [25 << 8, 26 << 8 | 1, 27 << 8 | 2, 25 << 8 | 0,
                                25 << 8 | 0]
    assert H.check_pd_sck(ref.edges, CLOCK) == []


def test_hx711_device_readings_from_the_host(backend):
    """Readings set through INQ (0x8000 | I << 12 | H, then L) are served
    from the conversion after the one that takes them."""
    bench, ref, log, loom = device_setup(backend, readings=(0, 0, 0),
                                         plan=[26, 25, 25])
    hi = lambda v: (H.to_code(v) >> 16) & 0xFF
    lo = lambda v: H.to_code(v) & 0xFFFF
    loom.push(1, [SETR | hi(-2), lo(-2), SETR | 1 << 12 | hi(-0x800000),
                  lo(-0x800000)])
    reads_done(bench, ref, 3)
    assert [r[1] for r in ref.reads][-2:] == [-0x800000, -2]


def window_reads(backend, high, low, sample, clock=CLOCK):
    """Three reads by a host with the given times: every reading right,
    every count honoured, the host inside T3 and T4, no power-down."""
    bench, ref, log, loom = device_setup(backend, plan=[26, 27, 25],
                                         tick=int(DEVICE_TICK * clock / CLOCK),
                                         high=high, low=low, sample=sample)
    reads_done(bench, ref, 3)
    assert [r[1] for r in ref.reads] == [VALUES[0], VALUES[1], VALUES[2]]
    assert reports(loom, 4)[1:] == [26 << 8 | 1, 27 << 8 | 2, 25 << 8 | 0]
    assert H.check_pd_sck(ref.edges, clock) == []
    assert PD_REPORT not in loom.pop_available(1)


@pytest.mark.parametrize("high, low", [(10, 14), (14, 10)])
def test_hx711_device_timing_window_edges(backend, high, low):
    """A host at the edges of T3 and T4 at 50 MHz (0.2 us, 10 clocks) with
    the device's shortest period (24 clocks), sampling DOUT 16 clocks after
    each rise (the header's limit)."""
    window_reads(backend, high, low, 16)


@model_only("T3 max: 50 us highs; on a 5 MHz clock to keep it short")
def test_hx711_device_t3_max(backend):
    """A host holding every high for T3's 50 us maximum (250 clocks at a
    5 MHz board clock, the device's tick 5, 1 us): read correctly, and a
    50 us high never powers the device down."""
    window_reads(backend, 250, 50, 249, clock=5e6)


def test_hx711_device_dout_lags_each_rise(backend):
    """Measured on the pins: each DOUT change comes 9 to 16 clocks after the
    PD_SCK rise that shifts it (the header's limit; the datasheet's T2 is
    5 clocks at 50 MHz, which this device does not meet), and the 25th
    pulse's return high too."""
    bench, ref, log, loom = device_setup(backend, plan=[25, 25])
    reads_done(bench, ref, 2)
    rises = [r for r, _ in H.pulses_of(ref.edges)]
    start = ref.reads[0][0]
    lags = []
    for cycle, level in log.edges:
        if cycle < start:
            continue
        before = [r for r in rises if r <= cycle]
        if not before or cycle - before[-1] > 40:
            continue                    # DOUT low at the end of a conversion
        lags.append(cycle - before[-1])
    assert lags and min(lags) >= 9 and max(lags) <= 16


def test_hx711_device_powers_down_mid_read(backend):
    """A host that holds the 10th pulse high for 62 us: the device powers
    down (report 0x4000) and drops the read; when PD_SCK falls it resets,
    so the next reading is of A/128 (reported with a count of 25), and a held high of
    62 us between reads powers it down again."""
    hold = int(H.clocks(62e-6, CLOCK))
    bench, ref, log, loom = device_setup(backend, plan=[26, 26],
                                         long_high=(10, hold))
    reads_done(bench, ref, 2)
    assert len(ref.abandoned) == 1
    assert ref.reads[0][1] == VALUES[0]
    assert reports(loom, 3) == [25 << 8, PD_REPORT, 25 << 8]
    ref.hold_high(bench.cycle + 100, hold)
    bench.step(hold + 400)
    assert PD_REPORT in loom.pop_available(1)


def test_hx711_device_flags_a_wrong_count(backend):
    """28 pulses (Table 3 has 25 to 27): the report carries the error bit
    and the count, and the next conversion is of A/128."""
    bench, ref, log, loom = device_setup(backend, plan=[26, 28, 25])
    reads_done(bench, ref, 3)
    assert [r[1] for r in ref.reads] == [VALUES[0], VALUES[1], VALUES[0]]
    assert reports(loom, 4) == [25 << 8, 26 << 8 | 1, ERR | 28 << 8 | 0, 25 << 8 | 0]


# ------------------------------------------------- host and device together
def test_hx711_host_and_device_on_two_threads(backend):
    """hx711_host on thread 0 reads hx711_device on thread 1 through two
    board wires (OUT0 to IN1, OUT1 to IN0), at the host's fastest proved
    tick (24 clocks): each gain in turn, the device's reports agreeing with
    the host's replies, then a power-down and wake through the host."""
    bench = bench_for(backend)
    bench.add(Wire("OUT0", "IN1"))
    bench.add(Wire("OUT1", "IN0"))
    log = bench.add(H.EdgeLog("OUT0"))
    loom = backend.loom(bench, isa=ISA)
    image = assemble_file(HOST, isa=ISA, strict=True)
    loom.load(image)
    loom.load(device_program(VALUES))
    loom.write_csr(0, "TICK_INT", 24)
    loom.write_csr(1, "TICK_INT", DEVICE_TICK)
    loom.run([0, 1])
    loom.push(1, [CONV | DEV_CONV])
    for sel, expect_input in ((1, 0), (2, 1), (0, 2)):
        loom.push(0, [sel])
        assert reply(loom) == (E_OK, expect_input, VALUES[expect_input])
    assert reports(loom, 3) == [25 << 8, 26 << 8 | 1, 27 << 8 | 2]
    assert H.check_pd_sck(log.edges, CLOCK) == []
    loom.push(0, [POWER_DOWN])
    bench.step(int(H.clocks(H.PD_TIME, CLOCK)) + 400)
    loom.push(0, [WAKE, 1])
    assert reply(loom) == (E_OK, 0, VALUES[0])
    # the power-down came during the conversion after the third read, so
    # that conversion never reported; then the wake's, then the read's
    assert reports(loom, 3) == [PD_REPORT, 25 << 8, 26 << 8 | 1]
