"""L3-DHT on the model and the RTL: firmware/dht22_host.loom and
firmware/dht22_sensor.loom, the AM2302 (DHT22) programs of D-035.

The host (thread 0, OW = BIDIR0) is fed through ``tools.loomhost.Loom`` and
read against ``tools.protomodels.dht22.Dht22Sensor`` on a ``OneWireBus``
line (the open-drain wire with its pull-up; the AM2302's protocol is not
1-Wire, but the wire is the same); the sensor (thread 1, OW = BIDIR1) is
read by ``Dht22Host``. Together, the host on BIDIR0 and the sensor on
BIDIR1 are one line: the bench joins the two pads, as two Loom threads
never share a pin, and they are started at every phase of the slot grid
and in both orders (BUGS 14: both set their OD_MASK bit first).

What is expected comes from the AM2302 datasheet (section 6(2): the start
low of at least 1 ms, the 20..40 us release, the 80 us response halves,
the 50 us bit low, a 0's 26..28 us high and a 1's 70 us high, the frame
and its checksum, sign-magnitude temperatures) as ``protomodels.dht22``
cites it, and from the program headers (the host words, the error codes,
the 49 us threshold), never from what the model or the RTL did. Every
pulse on the chip's pad is measured: the host's start low to the clock,
the sensor's reply against the windows of ``dht22.WINDOWS`` (the ones the
datasheet does not give are marked unchecked there).

A read is about 5 ms, 260000 clocks at 50 MHz, so:

* at the real rate (``US = 50``) the RTL runs one host read and one
  sensor reply, about 530000 clocks;
* the error cases, the window edges, the value sweeps and the start-phase
  sweep run on a time-scaled build, ``US = 10``: the same source with that
  one ``.equ`` changed, so every interval is the same number of ticks and
  one fifth as many clocks (the deadline checker proves both programs at
  10), against models clocked at 10 MHz. The RTL runs one phase of the two
  together on it.

Every body takes a ``backend`` (``tools/tests/fw_backend.py``): pytest runs
them on the golden model, ``test/test_fw.py`` on the RTL.
"""

import functools

import pytest

from tools.loomasm import assemble_text
from tools.loomisa import REPO, load
from tools.protomodels.bench import Model, pad_of
from tools.protomodels.dht22 import (AM2302, WINDOWS, Dht22Host, Dht22Sensor,
                                     OneWireBus, check_reply, check_start,
                                     checksum, decode, frame, frame_bits,
                                     from_wire_temp, to_wire_temp)
from tools.tests.fw_backend import model_only

ISA = load()
HOST = REPO / "firmware" / "dht22_host.loom"
SENSOR = REPO / "firmware" / "dht22_sensor.loom"
US = 50                           # clocks per microsecond: 50 MHz, the programs' rate
FAST = 10                         # the time-scaled build (module doc)
STARTUP = 400                     # clocks for a program's set-up after RUN
E_UP, E_NORESP, E_RLOW, E_RHIGH, E_BLOW, E_BHIGH, E_END, E_SUM = range(1, 9)
SCALED = ("a time-scaled build (US = 10, module doc): %s")


@functools.lru_cache(maxsize=None)
def program(path, us=US):
    """The program assembled with ``--strict``, for ``us`` clocks a microsecond."""
    text = path.read_text(encoding="utf-8")
    line = ".equ    US = 50 "
    assert text.count(line) == 1
    return assemble_text(text.replace(line, ".equ    US = %d " % us), path.name,
                         isa=ISA, strict=True)


def dht_bench(backend, bus):
    """The chip with the FIFOs, the bit engine, the deadline latch and the
    512-word memory; the RTL has all of them and ``RtlBench`` takes none of
    those keywords."""
    kwargs = dict(pullups=bus.pullups)
    if backend.name == "model":
        kwargs.update(features=("FIFO", "BE", "SETPD"), imem_words=512)
    bench = backend.bench(**kwargs)
    bench.add(bus)
    return bench


def status(csum, err=0):
    return err << 8 | csum


class Stuck(Model):
    """A short to ground on the line."""

    def __init__(self, pin):
        self.pad = pad_of(pin)

    def drive(self, drive, cycle):
        drive.pull_low(self.pad)


# ------------------------------------------------------------- the programs
@pytest.mark.parametrize("path,thread,words", [(HOST, 0, 117), (SENSOR, 1, 73)])
def test_dht22_programs_assemble_strict_with_no_diagnostic(path, thread, words):
    """L2-DEADLINE: every pair proved at the tick each program sets itself
    (TICK_INT 50, 1 us) and in the time-scaled build at 10; nothing
    unbounded, no ``.bounded``. Each program fits its thread's quarter, so
    the two load together."""
    for us in (US, FAST):
        prog = program(path, us)
        assert prog.diagnostics == []
        report = prog.deadlines[thread]
        assert report.period == us
        assert report.pairs and not report.infeasible and not report.unbounded
        assert report.worst_slack >= 0
    assert list(prog.threads) == [thread] and len(prog.words) == words
    assert all(thread * 128 <= a < (thread + 1) * 128 for a in prog.words)
    assert not set(program(HOST).words) & set(program(SENSOR).words)


# ------------------------------------------------------------------ host
def host_setup(backend, sensor=None, us=US):
    """The host program on thread 0 (BIDIR0), a model sensor on the line."""
    bus = OneWireBus(["BIDIR0"], clock=us * 1_000_000)
    if sensor is not None:
        bus.add(sensor)
    bench = dht_bench(backend, bus)
    loom = backend.loom(bench, isa=ISA)
    loom.load(program(HOST, us))
    loom.run(0)
    bench.step(STARTUP)
    return bench, bus, loom


def h_read(loom, k=10):
    """One read with a start low of ``k`` steps: (humidity, temperature, status)."""
    loom.push(0, [k])
    return tuple(loom.pop(0, 3, max_polls=100_000))   # a read is ~5 ms: more than the default


def starts(bus, pin="BIDIR0"):
    """The chip's pulls on its pad, in us: [(start, length)]."""
    return [(s / bus.us, (e - s) / bus.us) for s, e in bus.intervals(bus.chip_name(pin))]


def signed(word):
    return word - 0x10000 if word & 0x8000 else word


def test_host_reads_a_sensor(backend):
    """One read of the datasheet's example (65.2 %RH, 35.1 C, checksum
    0xEE): the start low is exactly 10 steps of 100 us, the datasheet's 1 ms
    minimum, and the chip pulls the line nowhere else; the report is the
    humidity, the temperature and status 0x00EE. The model sensor saw a good
    start and no violation. About 265000 clocks."""
    sensor = Dht22Sensor(652, 351, clock=US * 1_000_000)
    bench, bus, loom = host_setup(backend, sensor)
    assert sensor.frame() == bytes([0x02, 0x8C, 0x01, 0x5F, 0xEE])   # datasheet p.3
    assert h_read(loom) == (652, 351, status(0xEE))
    pulls = starts(bus)
    assert len(pulls) == 1 and pulls[0][1] == AM2302["start_min"] and check_start(pulls[0][1]) == []
    assert len(sensor.starts) == 1 and sensor.violations == []
    assert bench.contention_free() and loom.badop() == 0


@model_only(SCALED % "four reads, 210000 clocks")
@pytest.mark.parametrize("humidity,temp", [(0, 0), (1000, -400), (999, 800), (652, -101)])
def test_host_reads_signed_temperatures(backend, humidity, temp):
    """The datasheet's range (0..100 %RH, -40..80 C) and its -10.1 C example
    (0x8065 on the wire): the temperature reaches the host in two's
    complement, the checksum byte in status bits 7:0, error 0."""
    sensor = Dht22Sensor(humidity, temp, clock=FAST * 1_000_000)
    bench, bus, loom = host_setup(backend, sensor, FAST)
    h, t, st = h_read(loom)
    assert (h, signed(t), st) == (humidity, temp, status(sensor.frame()[4]))
    assert sensor.violations == [] and bench.contention_free()


@model_only(SCALED % "three reads")
@pytest.mark.parametrize("k,low", [(0, 1000), (9, 1000), (10, 1000), (25, 2500),
                                   (0x1F0A, 1000)])
def test_host_start_low_is_k_steps_at_least_1_ms(backend, k, low):
    """K below 10 is taken as 10 (1 ms) and bits 15:8 are ignored: the start
    low is max(K, 10) x 100 us, exact to the clock (latched edges)."""
    sensor = Dht22Sensor(652, 351, clock=FAST * 1_000_000)
    bench, bus, loom = host_setup(backend, sensor, FAST)
    assert h_read(loom, k) == (652, 351, status(0xEE))
    assert [length for _, length in starts(bus)] == [low] and sensor.violations == []


ERRORS = {
    "bad_checksum":   (dict(csum=0x5A), E_SUM, True),
    "no_response":    (dict(respond=False), E_NORESP, False),
    "late_response":  (dict(go_us=105), E_NORESP, False),
    "short_rel":      (dict(rel_us=35), E_RLOW, False),
    "long_rel":       (dict(rel_us=125), E_RLOW, False),
    "short_reh":      (dict(reh_us=35), E_RHIGH, False),
    "long_reh":       (dict(reh_us=125), E_RHIGH, False),
    "long_bit_low":   (dict(low_us=105), E_BLOW, False),
    "stops_at_20":    (dict(stop_after=20), E_BHIGH, False),
    "long_high_3":    (dict(high_us={3: 105}), E_BHIGH, False),
    "long_end_low":   (dict(end_us=105), E_END, True),
}


@model_only(SCALED % "one read a case")
@pytest.mark.parametrize("case", sorted(ERRORS))
def test_host_reports_each_error(backend, case):
    """Each fault the header lists gives its error code in status bits 11:8
    and the thread goes on: a good read follows with error 0. A wrong
    checksum still reports the values; a fault before the 40 bits reports
    the words that had arrived whole (the humidity after 16 bits) and 0 for
    the rest. Every timeout is a window from the header, past the
    datasheet's figure: 105 us for a 30 us answer, 35 and 125 for an 80 us
    half, 105 for a 50 us low and for a 70 us high."""
    kwargs, err, values = ERRORS[case]
    sensor = Dht22Sensor(652, -101, clock=FAST * 1_000_000, **kwargs)
    bench, bus, loom = host_setup(backend, sensor, FAST)
    h, t, st = h_read(loom)
    assert st >> 8 == err, (case, hex(st))
    if values:
        assert (h, signed(t)) == (652, -101)
        assert st & 0xFF == sensor.frame()[4]
    elif case == "stops_at_20":
        assert (h, t, st & 0xFF) == (652, 0, 0)
    else:
        assert (h, t, st & 0xFF) == (0, 0, 0)
    bench.step(8000 * FAST)                         # the sensor finishes its reply
    sensor.__init__(652, 351, clock=FAST * 1_000_000)
    assert h_read(loom) == (652, 351, status(0xEE))


@model_only("the same timeout as no response, on a stuck line")
def test_host_reports_a_line_held_low(backend):
    """A short to ground: the line is not high 10 us after the release,
    E_UP, and the values are 0."""
    bench, bus, loom = host_setup(backend, None, FAST)
    bench.add(Stuck("BIDIR0"))
    assert h_read(loom) == (0, 0, status(0, E_UP))


CORNERS = {
    "fast": dict(go_us=20, rel_us=75, reh_us=75, low_us=45, h0_us=26, h1_us=68, end_us=45),
    "slow": dict(go_us=40, rel_us=85, reh_us=85, low_us=55, h0_us=28, h1_us=75, end_us=55),
    "near": dict(h0_us=28, h1_us=68),
}


@model_only(SCALED % "one read a corner")
@pytest.mark.parametrize("corner", sorted(CORNERS))
def test_host_reads_a_sensor_at_the_window_edges(backend, corner):
    """A sensor with every width at the short ends of the windows, at the
    long ends, and with a 0's high at its longest (28 us, the datasheet's
    end) and a 1's at the window's shortest (68): every bit is decided
    right, the 49 us threshold 21 us from 28 and 19 from 68."""
    sensor = Dht22Sensor(0x5AA5 & 0x3FF, -0x2A5, clock=FAST * 1_000_000, **CORNERS[corner])
    bench, bus, loom = host_setup(backend, sensor, FAST)
    h, t, st = h_read(loom)
    assert (h, signed(t), st) == (sensor.humidity, sensor.temp, status(sensor.frame()[4]))


# ---------------------------------------------------------------- sensor
def sensor_setup(backend, pairs=((652, 351),), us=US):
    """The sensor program on thread 1 (BIDIR1), given its values, with a
    model host on the line."""
    bus = OneWireBus(["BIDIR1"], clock=us * 1_000_000)
    host = bus.add(Dht22Host(clock=us * 1_000_000))
    bench = dht_bench(backend, bus)
    loom = backend.loom(bench, isa=ISA)
    loom.load(program(SENSOR, us))
    loom.run(1)
    for h, t in pairs:
        loom.push(1, [h, t & 0xFFFF])
    bench.step(STARTUP)
    return bench, bus, host, loom


def finish(bench, rd, budget):
    assert bench.run_until(lambda: rd.done, budget, every=500), "not done: %r" % (rd,)
    return rd


def check_exact(rd):
    """The reply inside the windows, and every width the program makes from
    the release as it saw it exact to the clock: the 80 us halves, the 50 us
    lows, 27 or 70 us highs (the datasheet's values), the 50 us end low."""
    assert rd.violations == [], rd.violations
    for name, _, width in rd.result["widths"]:
        want = {"rel": 80, "reh": 80, "low": 50, "end": 50}.get(name)
        if name == "high":
            assert width in (27, 70), width
        elif want is not None:
            assert width == want, (name, width)
    lo, hi, _ = WINDOWS["go"]
    assert lo <= rd.result["widths"][0][2] <= hi


def test_sensor_answers_a_host(backend):
    """A model host's 1 ms start: the sensor answers 20..40 us after the
    release with the datasheet's example frame (02 8C 01 5F EE), every
    pulse at the datasheet's width to the clock. About 265000 clocks."""
    bench, bus, host, loom = sensor_setup(backend)
    rd = finish(bench, host.read(1000), 400_000)
    assert rd.data == bytes([0x02, 0x8C, 0x01, 0x5F, 0xEE]) and rd.decoded() == (652, 351, True)
    check_exact(rd)
    assert bench.contention_free() and loom.badop() == 0


@model_only(SCALED % "three reads")
def test_sensor_takes_new_values_between_reads(backend):
    """Negative temperatures go out as sign and magnitude (-10.1 C as
    0x8065, datasheet p.3) with the checksum over those bytes; a pair pushed
    while the sensor waits is taken after its next reply, so it shows from
    the read after that one."""
    bench, bus, host, loom = sensor_setup(backend, [(652, -101)], FAST)
    rd = finish(bench, host.read(1000), 100_000)
    assert rd.data == frame(652, 0x8065) and rd.decoded() == (652, -101, True)
    check_exact(rd)
    loom.push(1, [1000, -400 & 0xFFFF])
    rd = finish(bench, host.read(1000), 100_000)
    assert rd.decoded() == (652, -101, True)
    rd = finish(bench, host.read(1000), 100_000)
    assert rd.decoded() == (1000, -400, True) and rd.data[2:4] == bytes([0x81, 0x90])
    check_exact(rd)


@model_only(SCALED % "a glitch and a read")
def test_sensor_ignores_a_short_low(backend):
    """A 100 us low is not a start (the program takes 250 us and more): the
    sensor stays silent; the next 1 ms start is answered."""
    bench, bus, host, loom = sensor_setup(backend, us=FAST)
    rd = finish(bench, host.read(100), 50_000)
    assert rd.edges == [] and rd.violations[0][0] == "missing"
    rd = finish(bench, host.read(1000), 100_000)
    assert rd.decoded() == (652, 351, True)
    check_exact(rd)


# --------------------------------------------------------------- together
def together(backend, start, us=FAST):
    """Host (thread 0, BIDIR0) and sensor (thread 1, BIDIR1), the pads joined
    by the bench as one line, started by ``start(bench, loom)``."""
    bus = OneWireBus(["BIDIR0", "BIDIR1"], clock=us * 1_000_000)
    bench = dht_bench(backend, bus)
    loom = backend.loom(bench, isa=ISA)
    words = dict(program(HOST, us).words)
    sensor = program(SENSOR, us).words
    assert not set(words) & set(sensor)
    words.update(sensor)
    loom.load(words)
    start(bench, loom)
    bench.step(STARTUP)
    return bench, bus, loom


def read_together(bench, bus, loom, humidity, temp):
    """The sensor given its values, one read by the host: the report, the
    host's start low to the clock and the reply held to the windows."""
    loom.push(1, [humidity, temp & 0xFFFF])
    h, t, st = h_read(loom)
    assert (h, signed(t), st) == (humidity, temp,
                                  status(frame(humidity, to_wire_temp(temp))[4]))
    pulls = starts(bus, "BIDIR0")
    assert [length for _, length in pulls] == [AM2302["start_min"]]
    let_go = bus.intervals(bus.chip_name("BIDIR0"))[0][1]
    release = [c for c, level in bus.edges if c >= let_go and level][0]     # the line's rise
    assert release - let_go <= 1                    # the board wire: one clock
    edges = [e for e in bus.edges if e[0] > release]
    reply = check_reply(edges, release, clock=bus.clock_hz)
    assert reply["violations"] == [] and reply["data"] == frame(humidity, to_wire_temp(temp))
    assert bench.contention_free() and loom.badop() == 0


def test_host_and_sensor_together(backend):
    """The two programs under one RUN on the time-scaled build: the host
    reads the sensor's -10.1 C and 65.2 %RH, its start low exactly 1 ms and
    the sensor's reply inside every window, with no contention on the line.
    (A thread whose OD_MASK bit is lost drives nothing at all, as its
    PIN_OE bit stays 0: see the race test below.) The PHY-GL-SDF candidate:
    about 70000 clocks."""
    bench, bus, loom = together(backend, lambda bench, loom: loom.run([0, 1]))
    read_together(bench, bus, loom, 652, -101)


def _phase(p, order):
    def start(bench, loom):
        bench.step(p)
        if order == "one_run":
            loom.run([0, 1])
        else:
            first, second = (0, 1) if order == "host_first" else (1, 0)
            loom.run(first)
            bench.step(p)
            loom.run(second)
    return start


@model_only(SCALED % "twelve starts, 70000 clocks each")
@pytest.mark.parametrize("order", ["one_run", "host_first", "sensor_first"])
@pytest.mark.parametrize("p", [0, 1, 2, 3])
def test_together_at_every_start_phase(backend, p, order):
    """BUGS 14: both programs set their OD_MASK bit first. Under one RUN at
    each of the four phases of the slot grid, and each started p clocks
    after the other in both orders, the read is right: the read-back and
    retry in both headers keep both bits."""
    bench, bus, loom = together(backend, _phase(p, order))
    read_together(bench, bus, loom, 999, 800)


def _without_read_back(path, us):
    text = path.read_text(encoding="utf-8").replace(".equ    US = 50 ", ".equ    US = %d " % us)
    line = "        BZ      start                   ; lost to the other thread's write"
    assert text.count(line) == 1
    return assemble_text(text.replace(line, "        NOP" + " " * 29 + "; (removed)"),
                         path.name, isa=ISA, strict=True)


@model_only(SCALED % "the race itself, without the fix")
@pytest.mark.parametrize("p", [0, 1, 2, 3])
def test_without_the_read_back_one_run_loses_a_bit(backend, p, monkeypatch):
    """Why the read-back is there (BUGS 14): the same two programs with the
    retry branch replaced by a NOP, under one RUN. At every phase the two
    plain read-modify-writes overlap, the host's write lands last, the
    sensor's OD_MASK bit is lost, so its pad is never driven (PIN_OE stays
    0) and the host reports no response."""
    monkeypatch.setattr(__import__(__name__), "program",
                        functools.lru_cache(maxsize=None)(lambda path, us=US:
                                                          _without_read_back(path, us)))
    bench, bus, loom = together(backend, _phase(p, "one_run"))
    loom.push(1, [999, 800])
    assert h_read(loom) == (0, 0, status(0, E_NORESP))
    assert bus.intervals(bus.chip_name("BIDIR1")) == []


# ------------------------------------------------- the reference model alone
def test_model_frame_and_helpers():
    """The datasheet's examples: 65.2 %RH and 35.1 C are 02 8C 01 5F with
    checksum 0xEE (p.3), -10.1 C is 0x8065 (sign and magnitude), and the
    bits go MSB first."""
    data = frame(652, to_wire_temp(351))
    assert data == bytes([0x02, 0x8C, 0x01, 0x5F, 0xEE]) and checksum(data[:4]) == 0xEE
    assert to_wire_temp(-101) == 0x8065 and from_wire_temp(0x8065) == -101
    assert from_wire_temp(0x8000) == 0 and decode(data) == (652, 351, True)
    assert frame_bits(data)[:16] == [int(c) for c in "0000001010001100"]
    assert decode(frame(652, 0x8065, csum=0))[2] is False
    assert all(src in ("datasheet", "unchecked") for *_, src in WINDOWS.values())


@pytest.mark.parametrize("corner", sorted(CORNERS))
def test_model_host_reads_model_sensor(corner):
    """The reference host against the reference sensor, no chip, at each
    corner of the windows: the frame decodes and no width is outside its
    window; a sensor past a window is reported."""
    bus = OneWireBus(["BIDIR0"])
    sensor = bus.add(Dht22Sensor(431, -55, **CORNERS[corner]))
    host = bus.add(Dht22Host())
    rd = host.read(1000)
    assert bus.run_alone(lambda: rd.done, 2_000_000)
    assert rd.decoded() == (431, -55, True) and rd.violations == [] and sensor.violations == []
    sensor.h1_us = 80
    rd = host.read(900)
    assert bus.run_alone(lambda: rd.done, 2_000_000)
    assert rd.decoded() == (431, -55, True)
    assert rd.violations and {v[0] for v in rd.violations} == {"h1"}
    assert sensor.violations == [("start", 0, 900.0, (1000, None))]
