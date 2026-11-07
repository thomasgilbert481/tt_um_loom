"""L3-ADB on the model and the RTL: firmware/adb_host.loom and
firmware/adb_device.loom, the Apple Desktop Bus programs of D-035.

The host (thread 0, AD = BIDIR0) is fed through ``tools.loomhost.Loom`` and
talks to ``tools.protomodels.adb.AdbDevice`` models on a ``OneWireBus``
line (ADB's wire is the same open-drain line with a pull-up); the device
(thread 1, AD = BIDIR1) is driven by ``AdbHost``. Together, the host on
BIDIR0 and the device on BIDIR1 are one line, the bench joining the pads,
as two Loom threads never share a pin; a model device on the same line is
the second device (the two programs fill threads 0 to 3's memory, so a
second Loom device does not fit beside them: the program headers).

What is expected comes from the ADB documents as ``protomodels.adb`` cites
them (AN591B, Apple HW01, TMK's adb.c): the 800 us attention, the 65 us
sync, the 100 us cells with a 0 65 us low and a 1 35 us low, the stop bit,
Tlt 140..260 us, the 300 us SRQ, the 3 ms reset, the command byte, register
3 and the collision rule; and from the program headers (the host words,
the status word, the device's configuration words), never from what the
model or the RTL did. Every cell the chip makes is held to the windows by
the checker (``check_command`` for the host's commands at +-3 %,
``check_reply`` for the device's replies), and the host's widths are
exact to the clock as well.

A Talk is about 4 ms, 200000 clocks at 50 MHz, so the RTL runs one Talk at
the real rate (``US = 50``) and the rest on a time-scaled build, ``US =
10``: the same source with that one ``.equ`` changed, so every interval is
the same number of ticks and one fifth as many clocks (the deadline checker
proves both programs at 10), against models clocked at 10 MHz.

Every body takes a ``backend`` (``tools/tests/fw_backend.py``): pytest runs
them on the golden model, ``test/test_fw.py`` on the RTL.
"""

import functools

import pytest

from tools.loomasm import assemble_text
from tools.loomisa import REPO, load
from tools.protomodels.adb import (ADB, WINDOWS, AdbDevice, AdbHost, OneWireBus,
                                   bits_of, bytes_of, check_cells, check_command,
                                   check_reply, flush, listen, talk)
from tools.tests.fw_backend import model_only

ISA = load()
HOST = REPO / "firmware" / "adb_host.loom"
DEVICE = REPO / "firmware" / "adb_device.loom"
US = 50                           # clocks per microsecond: 50 MHz, the programs' rate
FAST = 10                         # the time-scaled build (module doc)
STARTUP = 400                     # clocks for a program's set-up after RUN
S_SRQ, S_ERR, S_RST = 1, 2, 4     # host status bits (adb_host.loom header)
SCALED = "a time-scaled build (US = 10, module doc): %s"


@functools.lru_cache(maxsize=None)
def program(path, us=US):
    """The program assembled with ``--strict``, for ``us`` clocks a microsecond."""
    text = path.read_text(encoding="utf-8")
    line = ".equ    US = 50 "
    assert text.count(line) == 1
    return assemble_text(text.replace(line, ".equ    US = %d " % us), path.name,
                         isa=ISA, strict=True)


def adb_bench(backend, bus):
    """The chip with the FIFOs, the bit engine, the deadline latch and the
    512-word memory; the RTL has all of them and ``RtlBench`` takes none of
    those keywords."""
    kwargs = dict(pullups=bus.pullups)
    if backend.name == "model":
        kwargs.update(features=("FIFO", "BE", "SETPD"), imem_words=512)
    bench = backend.bench(**kwargs)
    bench.add(bus)
    return bench


def status(cmd, n=0, flags=0):
    """The host's status word: the command byte, N, the flags (its header)."""
    return (cmd & 0xFF) << 8 | n << 4 | flags


def words_of(data):
    """A reply's bytes as the host reports them: pairs MSB first, an odd last
    byte in the high half."""
    data = bytes(data) + (b"\0" if len(data) % 2 else b"")
    return [data[i] << 8 | data[i + 1] for i in range(0, len(data), 2)]


# ------------------------------------------------------------- the programs
@pytest.mark.parametrize("path,thread,words", [(HOST, 0, 184), (DEVICE, 1, 223)])
def test_adb_programs_assemble_strict_with_no_diagnostic(path, thread, words):
    """L2-DEADLINE: every pair proved at the tick each program sets itself
    (TICK_INT 5 x US, 5 us) at US = 50 and in the time-scaled build at 10;
    nothing unbounded. The host spills into thread 3's quarter and the
    device into thread 2's (their headers), and the two load together."""
    for us in (US, FAST):
        prog = program(path, us)
        assert prog.diagnostics == []
        report = prog.deadlines[thread]
        assert report.period == 5 * us
        assert report.pairs and not report.infeasible and not report.unbounded
        assert report.worst_slack >= 0
    assert list(prog.threads) == [thread] and len(prog.words) == words
    assert not set(program(HOST).words) & set(program(DEVICE).words)
    assert max(program(HOST).words) < 512 and max(program(DEVICE).words) < 0x180


# ------------------------------------------------------------------ host
def host_setup(backend, devices=(), us=US):
    """The host program on thread 0 (BIDIR0), model devices on the line."""
    bus = OneWireBus(["BIDIR0"], clock=us * 1_000_000)
    for dev in devices:
        bus.add(dev)
    bench = adb_bench(backend, bus)
    loom = backend.loom(bench, isa=ISA)
    loom.load(program(HOST, us))
    loom.run(0)
    bench.step(STARTUP)
    return bench, bus, loom


def h_do(loom, word, data=()):
    """One transaction: (status, data words)."""
    loom.push(0, [word] + list(data), max_polls=100_000)
    st = loom.pop(0, 1, max_polls=100_000)[0]
    n = st >> 4 & 15
    return st, (loom.pop(0, (n + 1) // 2, max_polls=100_000) if n else [])


def host_commands(bus, us):
    """Every command the chip sent, from the line's edges: held to the
    host's windows by ``check_command``, plus each width exact to the
    clock (the latched edges): 800, 65, then 65/35 or 35/65 cells. Where
    the bench joins two chip pads, a low is one clock longer and the high
    after it one shorter (the board wire, ``OneWireBus``): ``slack``."""
    edges, out = bus.edges, []
    slack = 1 if len(bus.pads) > 1 else 0
    for i, (c, lvl) in enumerate(edges):
        if lvl == 0 and i + 1 < len(edges) and 700 * us <= edges[i + 1][0] - c <= 900 * us:
            res = check_command(edges[i:i + 20], clock=us * 1_000_000)
            w = [edges[i + k + 1][0] - edges[i + k][0] for k in range(18)]
            assert w[0] - slack == 800 * us and w[1] + slack == 65 * us, w[:2]
            for k in range(8):
                low, high = w[2 + 2 * k] - slack, w[3 + 2 * k] + slack
                assert (low, high) in ((35 * us, 65 * us), (65 * us, 35 * us)), w
            out.append(res)
    return out


def test_host_talks_to_a_device(backend):
    """Talk register 0 of a mouse at its default address 3 with a report
    queued: the command 0x3C on the wire inside every host window and exact
    to the clock, the reply's two bytes in one word, status N = 2, no SRQ.
    At the real rate, about 245000 clocks (no reply, N = 0, is tested on
    the scaled build by test_host_srq_flush_sendreset_and_reset)."""
    mouse = AdbDevice(3, 1, clock=US * 1_000_000)
    mouse.report(b"\x81\x7F")
    bench, bus, loom = host_setup(backend, [mouse])
    assert h_do(loom, talk(3, 0)) == (status(0x3C, 2), [0x817F])
    cmds = host_commands(bus, US)
    assert [c["command"] for c in cmds] == [0x3C]
    assert all(c["violations"] == [] for c in cmds) and mouse.violations == []
    assert mouse.sent == [b"\x81\x7F"] and bench.contention_free() and loom.badop() == 0


def fast_dev(*args, **kwargs):
    return AdbDevice(*args, clock=FAST * 1_000_000, **kwargs)


@model_only(SCALED % "five Talks")
@pytest.mark.parametrize("data", [b"\x12\x34", b"\x01\x02\x03", b"\xA5" * 5,
                                  bytes(range(1, 8)), bytes(range(0xF0, 0xF8))])
def test_host_captures_2_to_8_bytes(backend, data):
    """A reply of 2 to 8 bytes (HW01: registers hold 2 to 8): N in the
    status and (N + 1) / 2 words, an odd last byte in the high half."""
    dev = fast_dev(3, 1)
    dev.report(data)
    bench, bus, loom = host_setup(backend, [dev], FAST)
    assert h_do(loom, talk(3, 0)) == (status(0x3C, len(data)), words_of(data))
    assert dev.violations == [] and bench.contention_free()


@model_only(SCALED % "a Listen and two Talks")
def test_host_listen_writes_a_register(backend):
    """Listen register 2 of a keyboard (address 2) with W = 1 word: the
    device receives the two bytes, the packet inside the host's windows
    (Tlt 200 us, +-3 % cells) as the device model checks it; Talk register
    2 then reads them back. A Listen of W = 2 writes four bytes."""
    kbd = fast_dev(2, 2)
    bench, bus, loom = host_setup(backend, [kbd], FAST)
    assert h_do(loom, listen(2, 2), [0xBEEF]) == (status(0x2A), [])
    bench.step(300 * FAST)                          # the model's end-of-packet timeout
    assert kbd.listens == [(2, b"\xBE\xEF")] and kbd.violations == []
    assert h_do(loom, talk(2, 2)) == (status(0x2E, 2), [0xBEEF])
    assert h_do(loom, listen(2, 1) | 1 << 8, [0x0102, 0x0304]) == (status(0x29), [])
    bench.step(300 * FAST)
    assert kbd.regs[1] == b"\x01\x02\x03\x04" and kbd.violations == []
    assert all(c["violations"] == [] for c in host_commands(bus, FAST))


@model_only(SCALED % "the address dance of HW01")
def test_host_resolves_two_devices_at_one_address(backend):
    """Two mice at address 3 (HW01's walk-through): Talk register 3, both
    answer with random addresses, one sees the collision and stops, the
    host gets the other's two bytes whole; Listen register 3 with 0xFE
    moves only the winner (to 8). Talk register 3 at 3 again: the other
    answers alone and moves to 9. Then each answers at its own address."""
    a, b = fast_dev(3, 1, name="a", seed=5), fast_dev(3, 1, name="b", seed=11)
    bench, bus, loom = host_setup(backend, [a, b], FAST)
    st, words = h_do(loom, talk(3, 3))
    assert st == status(0x3F, 2) and a.collisions + b.collisions == 1
    winner, loser = (a, b) if b.collided else (b, a)
    assert words[0] & 0xFF == 1 and winner.sent[-1] == bytes([words[0] >> 8, 1])
    assert h_do(loom, listen(3, 3), [0x08FE]) == (status(0x3B), [])
    bench.step(300 * FAST)
    assert (winner.address, loser.address) == (8, 3)
    st, words = h_do(loom, talk(3, 3))
    assert st == status(0x3F, 2) and loser.collided is False
    assert h_do(loom, listen(3, 3), [0x09FE]) == (status(0x3B), [])
    bench.step(300 * FAST)
    assert loser.address == 9
    for addr in (8, 9):
        st, words = h_do(loom, talk(addr, 3))
        assert st == status(talk(addr, 3), 2) and words[0] & 0x0F00 != 0x0F00
    assert bench.contention_free()


@model_only(SCALED % "SRQ, Flush, SendReset and the reset")
def test_host_srq_flush_sendreset_and_reset(backend):
    """A keyboard at 2 with a report asks for service in the stop bit of a
    Talk to the mouse at 3: status bit 0, and the mouse's reply still
    arrives (Tlt counted from the end of the SRQ). Flush of the keyboard
    drops its report, so the next Talk to 3 has no SRQ. SendReset puts a
    moved mouse back at 3; the global reset is 3 ms low, exact."""
    mouse, kbd = fast_dev(3, 1, name="mouse"), fast_dev(2, 2, name="kbd")
    bench, bus, loom = host_setup(backend, [mouse, kbd], FAST)
    mouse.report(b"\x00\x01")
    kbd.report(b"\x1E\x9E")
    assert h_do(loom, talk(3, 0)) == (status(0x3C, 2, S_SRQ), [0x0001])
    assert len(kbd.srqs) == 1 and mouse.violations == [] and kbd.violations == []
    assert h_do(loom, flush(2)) == (status(0x21), [])
    assert kbd.queue == [] and h_do(loom, talk(3, 0)) == (status(0x3C), [])
    assert h_do(loom, listen(3, 3), [0x0AFE]) == (status(0x3B), [])
    bench.step(300 * FAST)
    assert mouse.address == 10
    assert h_do(loom, 0x00) == (status(0x00), []) and mouse.address == 3
    since = bus.edges[-1][0]
    assert h_do(loom, 0x8000) == (S_RST, [])
    lows = [(e[0] - s[0]) / FAST for s, e in zip(bus.edges, bus.edges[1:])
            if s[0] > since and s[1] == 0]
    assert lows == [3000] and mouse.resets == kbd.resets == 1


ERRORS = {
    "long_low": (dict(l0_us=95), status(0x3C, 0, S_ERR)),
    "no_stop": (dict(stop=False), status(0x3C, 0, S_ERR)),
}


@model_only(SCALED % "one bad reply a case")
@pytest.mark.parametrize("case", sorted(ERRORS))
def test_host_reports_a_bad_reply(backend, case):
    """A device whose 0 cells are 95 us low (past any device's 84.5 us) and
    one that leaves out its stop bit (8 N + 1 cells): error, N = 0. The
    thread goes on: a good Talk follows."""
    kwargs, want = ERRORS[case]
    dev = fast_dev(3, 1, **kwargs)
    dev.report(b"\x40\x00")
    bench, bus, loom = host_setup(backend, [dev], FAST)
    assert h_do(loom, talk(3, 0)) == (want, [])
    bench.step(4000 * FAST)
    dev.__init__(3, 1, clock=FAST * 1_000_000)
    dev.report(b"\x40\x00")
    assert h_do(loom, talk(3, 0)) == (status(0x3C, 2), [0x4000])


@model_only(SCALED % "cells at the edges of the device windows")
@pytest.mark.parametrize("l0,l1,cell", [(52, 26, 80), (84, 45, 120), (55, 25, 85),
                                         (78, 44, 115)])
def test_host_reads_cells_across_the_device_tolerance(backend, l0, l1, cell):
    """Cells from 80 to 120 us with their lows inside TMK's ratios (a 0 60
    to 70 % of the cell, a 1 30 to 40 %): every bit decided right by the
    50 us sample. The limit of a fixed sample (AN591's receiver has the
    same): it reads a cell only while a 0's low is over 50 us and a 1's
    under, so of the devices' +-30 % (70 to 130 us cells) it covers about
    77 to 125 us; a 70 us cell's 0 (45.5 us low) reads as a 1 (header,
    FW_README)."""
    dev = fast_dev(3, 1, l0_us=l0, l1_us=l1, cell_us=cell)
    dev.report(b"\x5A\xC3\x0F")
    bench, bus, loom = host_setup(backend, [dev], FAST)
    assert h_do(loom, talk(3, 0)) == (status(0x3C, 3), words_of(b"\x5A\xC3\x0F"))


# ---------------------------------------------------------------- device
MOUSE = (0x0301, 0x1234)          # register 3 after a reset (address 3, handler 1), seed


def device_setup(backend, config=MOUSE, us=FAST, others=(), **host_kwargs):
    """The device program on thread 1 (BIDIR1), configured, with a model
    host (and any model devices) on the line."""
    bus = OneWireBus(["BIDIR1"], clock=us * 1_000_000)
    host = bus.add(AdbHost(clock=us * 1_000_000, **host_kwargs))
    for dev in others:
        bus.add(dev)
    bench = adb_bench(backend, bus)
    loom = backend.loom(bench, isa=ISA)
    loom.load(program(DEVICE, us))
    loom.run(1)
    loom.push(1, list(config))
    bench.step(STARTUP)
    return bench, bus, host, loom


def run(bench, tx, budget=200_000):
    assert bench.run_until(lambda: tx.done, budget, every=200), "not done: %r" % (tx,)
    return tx


def check_exact(tx, us):
    """The reply inside every window, and exact where the program makes it
    so: Tlt 200 us after the release as the device saw it (2 to 6 clocks
    late) and restarted its tick on it (one slot more): 200 to 202 us at
    10 MHz, 12 to 15 clocks measured; every cell 100 us with a 35 or 65 us
    low (latched edges), the stop bit 65 us."""
    assert tx.violations == [], tx.violations
    assert 200 <= tx.reply["tlt"] <= 202, tx.reply["tlt"]
    cells = check_cells(tx.reply_edges, clock=us * 1_000_000)["cells"]
    assert all(low in (35, 65) and period in (100, None) for low, period in cells), cells
    assert cells[-1][0] == 65


def test_device_answers_a_host(backend):
    """A model host's Talk register 0 to the mouse at 3 holding a report:
    the two bytes Tlt after the stop bit, every cell exact; a second Talk
    finds nothing held and nothing in INQ: no reply (HW01). Talk register 3
    answers the SRQ enable bit, a random address and handler 1. The
    PHY-GL-SDF candidate: about 160000 clocks."""
    bench, bus, host, loom = device_setup(backend)
    loom.push(1, [0x8102])
    tx = run(bench, host.talk(3, 0))
    assert tx.answer == b"\x81\x02" and not tx.srq
    check_exact(tx, FAST)
    assert run(bench, host.talk(3, 0)).answer is None
    tx = run(bench, host.talk(3, 3))
    assert tx.answer[1] == 0x01 and tx.answer[0] & 0xF0 == 0x20
    check_exact(tx, FAST)
    assert bench.contention_free() and loom.badop() == 0


def test_device_asks_for_service(backend):
    """Holding a report, the device stretches the stop bit of a Talk to
    address 2 (not its own) to 300 us from the stop bit's fall as it saw it
    (2 to 6 clocks and a slot late; the end is latched); the host sees an
    SRQ, then polls address 3 and gets the report, and the SRQ stops."""
    bench, bus, host, loom = device_setup(backend)
    loom.push(1, [0x0A0B])
    tx = run(bench, host.talk(2, 0))
    assert tx.srq and tx.answer is None
    srq = check_command(tx.edges, clock=FAST * 1_000_000)["srq"]
    assert srq is not None and 300 <= srq <= 302, srq   # the fall is seen 2 to 6 clocks late
    assert run(bench, host.talk(3, 0)).answer == b"\x0A\x0B"
    assert not run(bench, host.talk(2, 0)).srq


@model_only(SCALED % "the register 3 rules")
def test_device_listen_register_3(backend):
    """Listen register 3: 0xFE moves the device (to 8: Talk at 3 is then
    silent, at 8 answers); 0x00 sets the address and the SRQ enable bit
    (to 5 with SRQ off: no SRQ while it holds a report); 0xFD, 0xFF and a
    handler change are ignored; SendReset and the 3 ms reset put it back
    at 3 with SRQ on."""
    bench, bus, host, loom = device_setup(backend)
    run(bench, host.listen(3, 3, b"\x08\xFE"))
    loom.push(1, [0x1111])
    assert run(bench, host.talk(3, 0)).answer is None
    assert run(bench, host.talk(8, 0)).answer == b"\x11\x11"
    run(bench, host.listen(8, 3, b"\x05\x00"))
    for bad in (b"\x09\xFD", b"\x09\xFF", b"\x09\x02"):
        run(bench, host.listen(5, 3, bad))
    loom.push(1, [0x2222])
    tx = run(bench, host.talk(2, 0))
    assert not tx.srq                               # SRQ disabled
    assert run(bench, host.talk(5, 3)).answer[0] & 0x20 == 0
    assert run(bench, host.talk(5, 0)).answer == b"\x22\x22"
    run(bench, host.command(0x00))                  # SendReset
    assert run(bench, host.talk(3, 3)).answer[0] & 0x20 == 0x20
    run(bench, host.listen(3, 3, b"\x0C\xFE"))
    run(bench, host.reset())
    tx = run(bench, host.talk(3, 3))
    assert tx.answer is not None and tx.answer[1] == 0x01
    assert bench.contention_free()


@model_only(SCALED % "Listen 2, Flush, the reserved codes")
def test_device_listen_flush_and_silence(backend):
    """Listen register 2 (a keyboard's LEDs) reaches OUTQ as one word;
    Flush drops the report held; Talk registers 1 and 2 and the reserved
    codes get no reply."""
    bench, bus, host, loom = device_setup(backend, (0x0205, 7))
    run(bench, host.listen(2, 2, b"\xFF\xF8"))
    assert loom.pop(1, 1, max_polls=10_000) == [0xFFF8]
    loom.push(1, [0x3333])
    run(bench, host.command(flush(2)))
    assert run(bench, host.talk(2, 0)).answer is None
    for byte in (talk(2, 1), talk(2, 2), 0x24, 0x22):
        assert run(bench, host.command(byte)).answer is None
    assert bench.contention_free() and loom.badop() == 0


@model_only(SCALED % "what is not a command")
@pytest.mark.parametrize("case", ["short_low", "long_low", "no_stop", "short_attention"])
def test_device_ignores_a_bad_command(backend, case):
    """A 300 us low is no attention (under 560), a host whose 0 cells are
    95 us low is cut off (no low past about 90 us), a command without its
    stop bit and a 500 us attention are dropped: the device stays silent
    and answers the next good Talk."""
    kwargs = dict(long_low=dict(l0_us=95), no_stop=dict(stop=False),
                  short_attention=dict(attention_us=500)).get(case, {})
    bench, bus, host, loom = device_setup(backend, **kwargs)
    loom.push(1, [0x4444])
    if case == "short_low":
        run(bench, host.reset(300))
    else:
        assert run(bench, host.talk(3, 0)).answer is None
    host.__init__(clock=FAST * 1_000_000)
    bench.step(2000 * FAST)
    assert run(bench, host.talk(3, 0)).answer == b"\x44\x44"


@model_only(SCALED % "a model device at the same address")
def test_device_collides_with_a_model_device(backend):
    """The Loom device and a model mouse both at 3: Talk register 3, one
    detects the collision 50 us into a 1 and stops, the host reads the
    other whole. Listen register 3 with 0xFE moves only the winner; the
    loser answers alone at 3 and then moves too."""
    other = AdbDevice(3, 1, clock=FAST * 1_000_000, name="other", seed=3)
    bench, bus, host, loom = device_setup(backend, others=[other])
    tx = run(bench, host.talk(3, 3))
    assert tx.violations == [] and tx.answer[1] == 1
    run(bench, host.listen(3, 3, b"\x08\xFE"))
    loom_won = other.collided
    assert other.address == (3 if loom_won else 8)
    tx = run(bench, host.talk(3, 3))
    assert tx.violations == [] and tx.answer is not None
    run(bench, host.listen(3, 3, b"\x09\xFE"))
    loom.push(1, [0x5555])
    assert run(bench, host.talk(8 if loom_won else 9, 0)).answer == b"\x55\x55"
    assert other.address == (9 if loom_won else 8)


# --------------------------------------------------------------- together
def together(backend, start, us=FAST, others=()):
    """Host (thread 0, BIDIR0) and device (thread 1, BIDIR1), the pads joined
    by the bench as one line, with any model devices; started by
    ``start(bench, loom)``. The device is configured as the mouse at 3."""
    bus = OneWireBus(["BIDIR0", "BIDIR1"], clock=us * 1_000_000)
    for dev in others:
        bus.add(dev)
    bench = adb_bench(backend, bus)
    loom = backend.loom(bench, isa=ISA)
    words = dict(program(HOST, us).words)
    device = program(DEVICE, us).words
    assert not set(words) & set(device)
    words.update(device)
    loom.load(words)
    start(bench, loom)
    loom.push(1, list(MOUSE))
    bench.step(STARTUP)
    return bench, bus, loom


def replies(bus, us):
    """Every data packet on the line after a command, checked: (tlt, data,
    violations), from the line's own edges."""
    edges, out, i = bus.edges, [], 0
    clock = us * 1_000_000
    while i < len(edges):
        c, lvl = edges[i]
        if lvl == 0 and i + 1 < len(edges) and 700 * us <= edges[i + 1][0] - c <= 900 * us:
            j = i + 19                              # the stop bit's rise (or the SRQ's)
            if j + 1 < len(edges) and edges[j + 1][0] - edges[j][0] < 300 * us:
                k = j + 1
                while k + 1 < len(edges) and edges[k + 1][0] - edges[k][0] < 150 * us:
                    k += 1
                res = check_reply(edges[j + 1:k + 1], edges[j][0], clock=clock)
                out.append((res["tlt"], res["data"], res["violations"]))
            i = j
        i += 1
    return out


def test_host_and_device_together_with_srq(backend):
    """The two programs under one RUN on the time-scaled build, with a model
    keyboard at 2 on the same line. The Loom host Talks register 0 of the
    Loom mouse, which holds a report; then the keyboard gets a report and
    asks for service in the stop bit of the next Talk to 3 (the mouse holds
    another), the host reports the SRQ, polls 2 and gets the key. Every
    command inside the host windows, every reply inside the device's and
    Tlt in 140..260 us; no contention. About 235000 clocks."""
    kbd = AdbDevice(2, 2, clock=FAST * 1_000_000, name="kbd")
    bench, bus, loom = together(backend, lambda bench, loom: loom.run([0, 1]), others=[kbd])
    loom.push(1, [0x0102])
    assert h_do(loom, talk(3, 0)) == (status(0x3C, 2), [0x0102])
    kbd.report(b"\x0E\x8E")
    loom.push(1, [0x0304])
    assert h_do(loom, talk(3, 0)) == (status(0x3C, 2, S_SRQ), [0x0304])
    assert h_do(loom, talk(2, 0)) == (status(0x2C, 2), [0x0E8E])
    cmds = host_commands(bus, FAST)
    assert [c["command"] for c in cmds] == [0x3C, 0x3C, 0x2C]
    assert all(c["violations"] == [] for c in cmds) and kbd.violations == []
    reps = replies(bus, FAST)
    assert [r[1] for r in reps] == [b"\x01\x02", b"\x03\x04", b"\x0E\x8E"]
    assert all(r[2] == [] for r in reps)
    assert bench.contention_free() and loom.badop() == 0


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


@model_only(SCALED % "twelve starts")
@pytest.mark.parametrize("order", ["one_run", "host_first", "device_first"])
@pytest.mark.parametrize("p", [0, 1, 2, 3])
def test_together_at_every_start_phase(backend, p, order):
    """BUGS 14: both programs set their OD_MASK bit first. Under one RUN at
    each phase of the slot grid, and each started p clocks after the other
    in both orders, a Talk register 3 and a Talk register 0 come back
    whole: the read-back and retry in both headers keep both bits."""
    bench, bus, loom = together(backend, _phase(p, order))
    st, words = h_do(loom, talk(3, 3))
    assert st == status(0x3F, 2) and words[0] & 0xF0FF == 0x2001
    loom.push(1, [0xCAFE])
    assert h_do(loom, talk(3, 0)) == (status(0x3C, 2), [0xCAFE])
    assert all(r[2] == [] for r in replies(bus, FAST)) and bench.contention_free()


# ------------------------------------------------- the reference model alone
def test_model_command_bytes_and_bits():
    """AN591 p.3: 0010 1100 is "Device 2, Talk Register 0"; Listen, Flush
    and SendReset have the codes 10, 0001 and 0000; bits go MSB first."""
    assert talk(2, 0) == 0b0010_1100 and listen(3, 3) == 0x3B and flush(2) == 0x21
    assert bits_of(b"\xA0") == [1, 0, 1, 0, 0, 0, 0, 0] and bytes_of(bits_of(b"\x5A\x01")) == b"\x5A\x01"
    assert ADB["attention"] == 800 and ADB["srq"] == 300 and ADB["tlt"] == 200
    assert all(len(w) == 3 for w in WINDOWS.values())


def test_model_host_and_devices_alone():
    """The reference host against two reference devices, no chip: a Talk
    with a reply inside every window, an SRQ from the other device, no reply
    from an empty register, the collision of two devices at one address
    with one backing off, and a device with a bad ratio flagged by the
    checker while still read."""
    bus = OneWireBus(["BIDIR0"])
    mouse = bus.add(AdbDevice(3, 1, name="mouse"))
    kbd = bus.add(AdbDevice(2, 2, name="kbd", seed=7))
    host = bus.add(AdbHost())

    def go(tx):
        assert bus.run_alone(lambda: tx.done, 10_000_000)
        return tx

    mouse.report(b"\x81\x02")
    tx = go(host.talk(3, 0))
    assert tx.answer == b"\x81\x02" and tx.violations == [] and tx.reply["tlt"] == 200
    assert host.command_check(tx)["violations"] == [] and mouse.violations == []
    assert go(host.talk(3, 0)).answer is None
    kbd.report(b"\x12\x34")
    assert go(host.talk(3, 0)).srq and len(kbd.srqs) == 1
    twin = bus.add(AdbDevice(3, 1, name="twin", seed=99))
    tx = go(host.talk(3, 3))
    assert tx.violations == [] and mouse.collisions + twin.collisions == 1
    odd = bus.add(AdbDevice(5, 1, name="odd", l1_us=45))
    odd.report(b"\xFF\xFF")
    tx = go(host.talk(5, 0))
    assert tx.answer == b"\xFF\xFF" and {v[0] for v in tx.violations} == {"ratio"}
