"""L3-JOYBUS on the model and the RTL: firmware/joybus_host.loom and
firmware/joybus_device.loom, the N64 / GameCube controller bus of D-035.

The host (thread 0, JB = BIDIR0) is fed through ``tools.loomhost.Loom`` and
read against ``tools.protomodels.joybus.JoybusDevice`` on a ``OneWireBus``
line (the open-drain wire and its pull-up); the device (thread 1, JB =
BIDIR1) is read by ``JoybusHost``. Together, the host on BIDIR0 and the
device on BIDIR1 are one line: the bench joins the two pads, as two Loom
threads never share a pin.

What is expected comes from the secondary sources ``protomodels.joybus``
cites (4 us cells; a 0 is 3 us low and 1 us high, a 1 is 1 us low and 3 us
high; the stop bits; the reply within about 62.5 us; the command set and
reply lengths) and from the program headers (the host words, the error
codes, the table), never from what the model or the RTL did. Every low on
the chip's pad is measured: the firmware's edges are latched, so its lows
must be 1 and 3 us and its cells 4 us exactly to the clock, inside the
model's (unchecked) windows.

A transaction is about 200 us, 10000 clocks at 50 MHz, so everything runs
at the real rate; the error cases, the sweeps and the start-phase sweep
run on the model only.

Every body takes a ``backend`` (``tools/tests/fw_backend.py``): pytest runs
them on the golden model, ``test/test_fw.py`` on the RTL.
"""

import functools

import pytest

from tools.loomasm import assemble_text
from tools.loomisa import REPO, load
from tools.protomodels.bench import Model, pad_of
from tools.protomodels.joybus import (JOYBUS, WINDOWS, JoybusDevice, JoybusHost,
                                      OneWireBus, bits_of, bytes_of, check_frame,
                                      frames_of, lows_of, plan_frame)
from tools.tests.fw_backend import model_only

ISA = load()
HOST = REPO / "firmware" / "joybus_host.loom"
DEVICE = REPO / "firmware" / "joybus_device.loom"
CLOCK = 50_000_000                # the programs' rate: TK = 10 clocks, 0.2 us
US = 50                           # clocks per microsecond
STARTUP = 400                     # clocks for a program's set-up after RUN
E_LINE, E_NORESP, E_SHORT, E_LOW, E_HIGH, E_STOP = range(1, 7)
MODEL_ONLY = "a case its RTL siblings already cover: %s"

N64_INFO = b"\x05\x00\x02"
N64_POLL = b"\x12\x34\x80\x7F"            # any four bytes: the bus does not care
GC_INFO = b"\x09\x00\x20"
GC_POLL = bytes([0x01, 0x80, 0x80, 0x80, 0x80, 0x80, 0x20, 0x20])
REPLIES = {0x00: N64_INFO, 0xFF: N64_INFO, 0x01: N64_POLL, 0x40: GC_POLL}


@functools.lru_cache(maxsize=None)
def program(path):
    """The program assembled with ``--strict``."""
    return assemble_text(path.read_text(encoding="utf-8"), path.name, isa=ISA,
                         strict=True)


def jb_bench(backend, bus):
    """The chip with the FIFOs, the bit engine, the deadline latch, LD/ST
    and the 512-word memory; the RTL has all of them and ``RtlBench``
    takes none of those keywords."""
    kwargs = dict(pullups=bus.pullups)
    if backend.name == "model":
        kwargs.update(features=("FIFO", "BE", "SETPD", "DMEM"), imem_words=512)
    bench = backend.bench(**kwargs)
    bench.add(bus)
    return bench


class Stuck(Model):
    """A short to ground on the line."""

    def __init__(self, pin):
        self.pad = pad_of(pin)

    def drive(self, drive, cycle):
        drive.pull_low(self.pad)


def words_of(data, nbytes):
    """The host's reply words for ``data`` received of ``nbytes`` due."""
    data = bytes(data) + bytes(nbytes - len(data))
    if nbytes % 2:
        data += b"\x00"
    return [data[i] << 8 | data[i + 1] for i in range(0, len(data), 2)]


def status(nbytes, err=0):
    return err << 8 | nbytes


def chip_lows(bus, pin):
    """The chip's pulls on its pad, ``[(start, end)]`` cycles."""
    return [(s, e) for s, e in bus.intervals(bus.chip_name(pin)) if e is not None]


def assert_exact(lows, data, stop=True):
    """A firmware frame: every low 1 or 3 us and every cell 4 us exactly
    to the clock (latched edges), the stop bit 1 us, inside the windows."""
    result = check_frame(lows, clock=CLOCK, nbytes=len(data))
    assert result["violations"] == [] and result["data"] == bytes(data), result
    for (f, r), bit in zip(lows, bits_of(data)):
        assert r - f == (1 if bit else 3) * US, (f, r, bit)
    for (f, _), (g, _) in zip(lows, lows[1:]):
        assert g - f == 4 * US
    if stop:
        assert lows[-1][1] - lows[-1][0] == US


# ------------------------------------------------------------- the programs
@pytest.mark.parametrize("path,thread,words", [(HOST, 0, 166), (DEVICE, 1, 134)])
def test_joybus_programs_assemble_strict_with_no_diagnostic(path, thread, words):
    """L2-DEADLINE: every pair proved at the 0.2 us tick each program sets
    (TICK_INT 10); nothing unbounded, no ``.bounded``. The host spills into
    thread 2's quarter and the device's table sits in thread 3's (their
    headers): the two load together, with threads 2 and 3 not started."""
    prog = program(path)
    assert prog.diagnostics == []
    report = prog.deadlines[thread]
    assert report.period == 10
    assert report.pairs and not report.infeasible and not report.unbounded
    assert report.worst_slack >= 0
    assert list(prog.threads) == [thread] and len(prog.words) == words
    assert not set(program(HOST).words) & set(program(DEVICE).words)


# ------------------------------------------------------------------ host
def host_setup(backend, device=None, extra=()):
    """The host program on thread 0 (BIDIR0), a model device on the line."""
    bus = OneWireBus(["BIDIR0"], clock=CLOCK)
    if device is not None:
        bus.add(device)
    bench = jb_bench(backend, bus)
    for model in extra:
        bench.add(model)
    loom = backend.loom(bench, isa=ISA)
    loom.load(program(HOST))
    loom.run(0)
    bench.step(STARTUP)
    return bench, bus, loom


def transact(loom, words, nbytes):
    """One transaction: the report, ``ceil(nbytes / 2) + 1`` words."""
    loom.push(0, words)
    return loom.pop(0, (nbytes + 1) // 2 + 1, max_polls=10_000)


def check_host_frame(bus, command, n=1):
    """The chip's ``n``-th frame on BIDIR0 is ``command`` and the console
    stop bit, exact to the clock."""
    frames = frames_of(chip_lows(bus, "BIDIR0"), clock=CLOCK)
    assert len(frames) == n
    assert_exact(frames[-1], command)


@pytest.mark.parametrize("words,command,nbytes,reply", [
    ([0x0030], b"\x00", 3, N64_INFO),
    ([0x0141], b"\x01", 4, N64_POLL),
    ([0x4083, 0x0300], b"\x40\x03\x00", 8, GC_POLL),
])
def test_host_reads_a_device(backend, words, command, nbytes, reply):
    """N64 info (0x00, 3 bytes back), the N64 poll (0x01, 4) and the
    GameCube poll (0x40 0x03 0x00, 8): the command goes out MSB first with
    lows of exactly 1 and 3 us, 4 us cells and the 1 us console stop bit,
    the model device decodes it with no violation, and the report is the
    reply two bytes a word and status B = R, error 0. About 60000 clocks
    with the program load."""
    device = JoybusDevice(REPLIES, clock=CLOCK)
    bench, bus, loom = host_setup(backend, device)
    assert transact(loom, words, nbytes) == words_of(reply, nbytes) + [status(nbytes)]
    check_host_frame(bus, command)
    assert [r["data"] for _, r in device.frames] == [command]
    assert all(r["violations"] == [] for _, r in device.frames)
    assert bench.contention_free() and loom.badop() == 0


def test_host_times_out_with_no_device(backend):
    """Nothing on the line: after the stop bit the host waits RESP_TO (64 us,
    past N64brew's "about 62.5 us") and reports E_NORESP with nothing
    received; the pads are released and the next command goes out."""
    bench, bus, loom = host_setup(backend)
    assert transact(loom, [0x0030], 3) == [0, 0, status(0, E_NORESP)]
    assert transact(loom, [0x0141], 4) == [0, 0, status(0, E_NORESP)]
    check_host_frame(bus, b"\x01", 2)
    assert bus.level == 1 and loom.badop() == 0


@model_only(MODEL_ONLY % "the reply length 0")
def test_host_with_no_reply_due(backend):
    """R = 0: the command and its stop bit, then status 0 at once."""
    bench, bus, loom = host_setup(backend, JoybusDevice(REPLIES, clock=CLOCK))
    assert transact(loom, [0xFF01], 0) == [status(0)]
    check_host_frame(bus, b"\xFF")


ERRORS = {
    "silent":     (dict(respond=False), [0, 0], status(0, E_NORESP)),
    "late":       (dict(delay_us=70), [0, 0], status(0, E_NORESP)),
    "no_stop":    (dict(stop=False), words_of(N64_POLL, 4), status(4, E_STOP)),
    "long_stop":  (dict(stop_low_us=5), words_of(N64_POLL, 4), status(4, E_STOP)),
    "short_low":  (dict(lows_us={19: 0.3}), [0x1234, 0], status(2, E_SHORT)),
    "truncated":  (dict(truncate=2), [0x1234, 0], status(2, E_HIGH)),
}


@model_only(MODEL_ONLY % "error injection")
@pytest.mark.parametrize("case", sorted(ERRORS))
def test_host_reports_a_bad_reply(backend, case):
    """A device that is silent or too late (past 64 us), leaves out its stop
    bit or holds it, makes a glitch-short low, or stops
    after two bytes: the error code, the words received whole (an error
    inside a word drops that word) and B; the next poll is read whole."""
    kwargs, words, st = ERRORS[case]
    device = JoybusDevice(REPLIES, clock=CLOCK, **kwargs)
    bench, bus, loom = host_setup(backend, device)
    assert transact(loom, [0x0141], 4) == words + [st]
    for key in kwargs:
        setattr(device, key, JoybusDevice().__dict__[key])
    bench.step(100 * US)
    assert transact(loom, [0x0141], 4) == words_of(N64_POLL, 4) + [status(4)]


@model_only(MODEL_ONLY % "the edges of the decision")
@pytest.mark.parametrize("low1,low0", [(0.8, 2.5), (1.6, 2.4), (1.0, 3.0)])
def test_host_decides_by_the_low(backend, low1, low0):
    """A device whose lows are off: 1-lows of 0.8 to 1.6 us and 0-lows down to
    2.4 us still read right (every low to 1.84 us is a 1 and from 2.08 us a 0,
    measured over ten phases, the host header)."""
    lows = {i: (low1 if b else low0) for i, b in enumerate(bits_of(N64_POLL))}
    device = JoybusDevice(REPLIES, clock=CLOCK, lows_us=lows)
    bench, bus, loom = host_setup(backend, device)
    assert transact(loom, [0x0141], 4) == words_of(N64_POLL, 4) + [status(4)]


@model_only(MODEL_ONLY % "a stuck line")
def test_host_reports_a_stuck_line(backend):
    """The line held low: E_LINE before anything is sent, no pull by the chip."""
    bench, bus, loom = host_setup(backend, extra=[Stuck("BIDIR0")])
    assert transact(loom, [0x0141], 4) == [0, 0, status(0, E_LINE)]
    assert chip_lows(bus, "BIDIR0") == []


# ---------------------------------------------------------------- device
def table_words(entries):
    """INQ words for the device's table: (index, value) pairs."""
    return [w for i, value in entries for w in (i, value)]


N64_TABLE = [(0, 0x0500), (1, 0x0200), (2, 0x1234), (3, 0x807F)]
GC_TABLE = [(0, 0x0900), (1, 0x2000)] + [(2 + i, GC_POLL[2 * i] << 8 | GC_POLL[2 * i + 1])
                                         for i in range(4)]


def device_setup(backend, entries=N64_TABLE):
    """The device program on thread 1 (BIDIR1) with its table set, a model
    console on the line."""
    bus = OneWireBus(["BIDIR1"], clock=CLOCK)
    host = bus.add(JoybusHost(clock=CLOCK))
    bench = jb_bench(backend, bus)
    loom = backend.loom(bench, isa=ISA)
    loom.load(program(DEVICE))
    loom.run(1)
    bench.step(STARTUP)
    if entries:
        loom.push(1, table_words(entries))
        bench.step(60 * US)                 # the device takes INQ between frames
    return bench, bus, host, loom


def finish(bench, tx, max_cycles=40_000):
    assert bench.run_until(lambda: tx.done, max_cycles, every=50), tx
    return tx


def check_reply(bus, tx, reply):
    """The reply decoded with no violation, its first fall inside the
    turnaround window, and the chip's lows on BIDIR1 exact to the clock."""
    assert tx.reply == reply and tx.violations == [], tx.result
    lo, hi, _ = WINDOWS["reply"]
    assert lo <= tx.turnaround_us <= hi
    frames = frames_of(chip_lows(bus, "BIDIR1"), clock=CLOCK)
    assert_exact(frames[-1], reply)


@pytest.mark.parametrize("command,nbytes,reply", [
    (b"\x00", 3, N64_INFO), (b"\x01", 4, N64_POLL)])
def test_device_answers_an_n64_console(backend, command, nbytes, reply):
    """Info (0x00: 05 00 and the status byte the host set) and the poll
    (0x01: the four bytes the host set), each after the console stop bit,
    the turnaround about 2 us, every low 1 or 3 us and every cell 4 us to
    the clock, then the 1 us device stop bit. About 55000 clocks with the
    program load."""
    bench, bus, host, loom = device_setup(backend)
    tx = finish(bench, host.send(command, nbytes))
    check_reply(bus, tx, reply)
    assert 2.0 <= tx.turnaround_us <= 2.5
    assert bench.contention_free() and loom.badop() == 0


def test_device_answers_a_gamecube_poll(backend):
    """0x40 0x03 0x00 (three bytes out, A; the arguments are read and
    ignored): the eight bytes the host set, MSB first, with the stop bit;
    then GameCube info (0x00: 09 00 20)."""
    bench, bus, host, loom = device_setup(backend, GC_TABLE)
    check_reply(bus, finish(bench, host.send(b"\x40\x03\x00", 8)), GC_POLL)
    bench.step(20 * US)
    check_reply(bus, finish(bench, host.send(b"\x00", 3)), GC_INFO)
    assert bench.contention_free() and loom.badop() == 0


@model_only(MODEL_ONLY % "commands the device does not answer")
@pytest.mark.parametrize("command", [b"\x02", b"\x41", b"\x13"])
def test_device_ignores_an_unknown_command(backend, command):
    """A command not among 0x00, 0xFF, 0x01, 0x40 gets no answer (official
    devices do the same, A); the next poll is answered."""
    bench, bus, host, loom = device_setup(backend)
    tx = finish(bench, host.send(command, 3))
    assert tx.reply == b"" and ("noreply",) in tx.violations
    assert chip_lows(bus, "BIDIR1") == []
    bench.step(20 * US)
    check_reply(bus, finish(bench, host.send(b"\x01", 4)), N64_POLL)


@model_only(MODEL_ONLY % "console error injection")
@pytest.mark.parametrize("case", ["no_stop", "short_low"])
def test_device_survives_a_bad_command(backend, case):
    """A console that leaves out its stop bit, or whose first bit's low is
    a 0.3 us glitch (read as a 1: 0x80, unknown): no answer, no hang; the
    next poll is answered."""
    bench, bus, host, loom = device_setup(backend)
    kwargs = dict(stop=False) if case == "no_stop" else dict(lows_us={0: 0.3})
    tx = finish(bench, host.send(b"\x01", 4, **kwargs))
    assert tx.reply == b""
    bench.step(20 * US)
    check_reply(bus, finish(bench, host.send(b"\x01", 4)), N64_POLL)


@model_only(MODEL_ONLY % "a table change")
def test_device_takes_a_new_table_between_frames(backend):
    """FF (reset/info) answers like 0x00; a poll value pushed between two
    polls shows in the second."""
    bench, bus, host, loom = device_setup(backend)
    check_reply(bus, finish(bench, host.send(b"\xFF", 3)), N64_INFO)
    loom.push(1, table_words([(3, 0x0102)]))
    bench.step(60 * US)
    check_reply(bus, finish(bench, host.send(b"\x01", 4)), b"\x12\x34\x01\x02")


# --------------------------------------------------------------- together
def together(backend, start, entries=N64_TABLE):
    """Host (thread 0, BIDIR0) and device (thread 1, BIDIR1), the pads joined
    by the bench as one line, started by ``start(bench, loom)``; the
    device's table set."""
    bus = OneWireBus(["BIDIR0", "BIDIR1"], clock=CLOCK)
    bench = jb_bench(backend, bus)
    loom = backend.loom(bench, isa=ISA)
    words = dict(program(HOST).words)
    device = program(DEVICE).words
    assert not set(words) & set(device)
    words.update(device)
    loom.load(words)
    start(bench, loom)
    bench.step(STARTUP)
    loom.push(1, table_words(entries))
    bench.step(60 * US)
    return bench, bus, loom


def check_together(bench, bus, loom, words, command, nbytes, reply, n):
    """One transaction between the two programs: the host's report, its
    ``n``-th frame and the device's reply each exact to the clock, the
    device's first fall 2 to 2.5 us after the console stop bit's rise."""
    assert transact(loom, words, nbytes) == words_of(reply, nbytes) + [status(nbytes)]
    host = frames_of(chip_lows(bus, "BIDIR0"), clock=CLOCK)
    device = frames_of(chip_lows(bus, "BIDIR1"), clock=CLOCK)
    assert len(host) == len(device) == n
    assert_exact(host[-1], command)
    assert_exact(device[-1], reply)
    turn = (device[-1][0][0] - host[-1][-1][1]) / US
    assert 2.0 <= turn <= 2.5, turn
    assert bench.contention_free() and loom.badop() == 0


def test_host_and_device_together(backend):
    """The two programs under one RUN: N64 info and the N64 poll, every
    edge of both directions exact to the clock and the turnaround about
    2 us. The PHY-GL-SDF candidate: about 109000 clocks with the program
    load (65 s of RTL)."""
    bench, bus, loom = together(backend, lambda bench, loom: loom.run([0, 1]))
    check_together(bench, bus, loom, [0x0030], b"\x00", 3, N64_INFO, 1)
    check_together(bench, bus, loom, [0x0141], b"\x01", 4, N64_POLL, 2)


def test_host_and_device_together_gamecube(backend):
    """The GameCube poll between the two programs: 0x40 0x03 0x00 out,
    the eight table bytes back."""
    bench, bus, loom = together(backend, lambda bench, loom: loom.run([0, 1]),
                                GC_TABLE)
    check_together(bench, bus, loom, [0x4083, 0x0300], b"\x40\x03\x00", 8, GC_POLL, 1)


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


@model_only(MODEL_ONLY % "twelve starts")
@pytest.mark.parametrize("order", ["one_run", "host_first", "device_first"])
@pytest.mark.parametrize("p", [0, 1, 2, 3])
def test_together_at_every_start_phase(backend, p, order):
    """BUGS 14: both programs set their OD_MASK bit first. Under one RUN at
    each phase of the slot grid, and each started p clocks after the other
    in both orders, the poll is read right."""
    bench, bus, loom = together(backend, _phase(p, order))
    check_together(bench, bus, loom, [0x0141], b"\x01", 4, N64_POLL, 1)


# ------------------------------------------------- the reference model alone
def test_model_frame_helpers():
    """MSB first; a planned frame is 4 us cells with 1 or 3 us lows and the
    stop bit, and the checker passes it and decodes it; the windows say
    where they come from."""
    assert bits_of(b"\x40") == [0, 1, 0, 0, 0, 0, 0, 0]
    assert bytes_of(bits_of(b"\x40\x03")) == b"\x40\x03"
    plan = plan_frame(b"\x01", 100, clock=CLOCK)
    edges = [(c, 0 if pull else 1) for c, pull in plan]
    lows = lows_of(edges)
    assert len(lows) == 9 and [r - f for f, r in lows] == [150] * 7 + [50, 50]
    result = check_frame(lows, clock=CLOCK, nbytes=1)
    assert result["data"] == b"\x01" and result["violations"] == [] and result["stop"] == 1.0
    assert JOYBUS["cell"] == 4.0 and all(len(w) == 3 for w in WINDOWS.values())


def test_model_checker_catches_bad_shapes():
    """The checker flags a short low, a missing stop bit and a long cell."""
    lows = lows_of([(c, 0 if p else 1) for c, p in plan_frame(b"\x01", 0, clock=CLOCK,
                                                               lows_us={2: 0.3})])
    assert ("low", 2, 0.3) in check_frame(lows, clock=CLOCK, nbytes=1)["violations"]
    lows = lows_of([(c, 0 if p else 1) for c, p in plan_frame(b"\x01", 0, clock=CLOCK,
                                                               stop=False)])
    assert ("nostop",) in check_frame(lows, clock=CLOCK, nbytes=1)["violations"]
    lows = [(0, 50), (250, 300)] + [(400 + 200 * i, 450 + 200 * i) for i in range(7)]
    assert ("cell", 0, 5.0) in check_frame(lows, clock=CLOCK, nbytes=1)["violations"]


@pytest.mark.parametrize("command,nbytes", [(b"\x00", 3), (b"\x01", 4), (b"\x40\x03\x00", 8)])
def test_model_host_reads_model_device(command, nbytes):
    """The reference console against the reference controller, no chip:
    the reply decodes with no violation; a silent device is reported."""
    bus = OneWireBus(["BIDIR0"], clock=CLOCK)
    device = bus.add(JoybusDevice(REPLIES, clock=CLOCK))
    host = bus.add(JoybusHost(clock=CLOCK))
    tx = host.send(command, nbytes)
    assert bus.run_alone(lambda: tx.done, 200_000)
    assert tx.reply == REPLIES[command[0]] and tx.violations == []
    assert tx.turnaround_us == JOYBUS["delay"]
    assert device.frames[-1][1]["data"] == command
    device.respond = False
    tx = host.send(command, nbytes)
    assert bus.run_alone(lambda: tx.done, 200_000)
    assert ("noreply",) in tx.violations
