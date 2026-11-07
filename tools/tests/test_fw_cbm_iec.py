"""L3-CBM-IEC on the model and the RTL: firmware/cbm_iec_host.loom and
firmware/cbm_iec_device.loom, the Commodore serial (IEC) bus controller and
device (D-035).

The host program (thread 0, ATN = BIDIR0, CLK = BIDIR1, DATA = BIDIR2) is
fed through ``tools.loomhost.Loom`` and talks to
``tools.protomodels.cbm_iec.IecDevice`` on an ``IecBus``; the device program
(thread 1, ATN = BIDIR3, CLK = BIDIR4, DATA = BIDIR5) is driven by
``IecController``. Together, the bench joins 0-3, 1-4 and 2-5 (two Loom
threads never share a pin).

What is expected comes from the secondary sources ``protomodels.cbm_iec``
cites (Butterfield, COMPUTE! 38; Steil, pagetable.com parts 2 and 4): the
command bytes, the order LISTEN, SECOND, data, UNLISTEN and TALK, SECOND,
turnaround, data, UNTALK, EOI on the last byte, and the windows of
``TIMING`` (1 ms ATN response and frame handshake, 200 us EOI, 60 us EOI
acknowledge, 100 us between bytes, 20 and 60 us bit holds); and from the
program headers (the host words, the 64 us bit phases, the error words),
never from what the model or the RTL did. Every edge goes through
``cbm_iec.check``.

Bus speed: the RTL scenarios run the bus with one "microsecond" of 10
clocks (TICK_INT 10, five times the real speed at 50 MHz), every window
scaled with it, to keep the RTL time in budget; the model also runs it at
the real 50 clocks per microsecond.
"""

import functools

import pytest

from tools.loomasm import assemble_text
from tools.loomisa import REPO, load
from tools.protomodels.cbm_iec import (CLOSE, LISTEN, OPEN, SECOND, TALK, TIMING,
                                       UNLISTEN, UNTALK, IecBus, IecController,
                                       IecDevice, check, frames)
from tools.tests.fw_backend import model_only

ISA = load()
HOST = REPO / "firmware" / "cbm_iec_host.loom"
DEVICE = REPO / "firmware" / "cbm_iec_device.loom"
FAST = 10                          # clocks per bus microsecond on the RTL
STARTUP = 400
OP_SEND, OP_RECEIVE = 1, 2
DONE = 0x4000
E_NODEV, E_FRAME, E_TURN = 0x8100, 0x8200, 0x8300
EOI = 0x100
T_BIT = 64                         # the programs' bit phases (headers)
POLLS = 100_000


@functools.lru_cache(maxsize=None)
def program(path, text_patch=None):
    """The program assembled with ``--strict``."""
    text = path.read_text(encoding="utf-8")
    if text_patch is not None:
        old, new = text_patch
        assert text.count(old) == 1
        text = text.replace(old, new)
    return assemble_text(text, path.name, isa=ISA, strict=True)


def iec_bench(backend, bus):
    """The chip with the FIFOs, the bit engine and the 512-word memory; the
    RTL has all of them and ``RtlBench`` takes none of those keywords."""
    kwargs = dict(pullups=bus.pullups)
    if backend.name == "model":
        kwargs.update(features=("FIFO", "BE"), imem_words=512)
    bench = backend.bench(**kwargs)
    bench.add(bus)
    return bench


def op(code, pa, sa):
    return code << 12 | sa << 5 | pa


def message(data):
    """The data words of a send: bit 8 on the last."""
    return [b for b in data[:-1]] + [data[-1] | EOI]


def bus_ok(bus):
    """Everything on the wire inside the sources' windows: the events."""
    result = check(bus.edges, bus.us)
    assert result["violations"] == []
    return result


def bytes_of(result, *, atn=None):
    return [(e[1], e[2]) for e in result["events"]
            if e[0] == "byte" and (atn is None or e[3] == atn)]


# ------------------------------------------------------------- the programs
@pytest.mark.parametrize("path,thread,words", [(HOST, 0, 182), (DEVICE, 1, 154)])
def test_cbm_iec_programs_assemble_strict_with_no_diagnostic(path, thread, words):
    """L2-DEADLINE: every pair proved at TICK_INT 10, the fastest bus the
    tests run (a slower tick only adds slack). The host's main loop is in
    thread 0's quarter and its subroutines in thread 2's; the device's in
    thread 1's and thread 3's; the two load together."""
    prog = program(path)
    assert prog.diagnostics == []
    report = prog.deadlines[thread]
    assert report.period == FAST
    assert report.pairs and not report.infeasible and not report.unbounded
    assert report.worst_slack >= 0
    lo = thread * 128
    assert all(lo <= a < lo + 128 or lo + 256 <= a < lo + 384 for a in prog.words)
    assert list(prog.threads) == [thread] and len(prog.words) == words
    assert not set(program(HOST).words) & set(program(DEVICE).words)


def near(value, nominal, slack=1.0):
    """A width on the thread's slot grid: within a few slots of nominal."""
    assert abs(value - nominal) <= slack, (value, nominal)


# ------------------------------------------------------- host, model device
def host_setup(backend, device, us=FAST):
    """The host program on thread 0 (BIDIR0..2), a model device on the bus."""
    bus = IecBus(["BIDIR0"], ["BIDIR1"], ["BIDIR2"], us=us)
    bus.add(device)
    bench = iec_bench(backend, bus)
    loom = backend.loom(bench, isa=ISA)
    loom.load(program(HOST))
    loom.push(0, [us])
    loom.run(0)
    bench.step(STARTUP)
    return bench, bus, loom


def do(loom, words, count):
    """One operation: its words pushed, ``count`` result words popped."""
    loom.push(0, words, max_polls=POLLS)
    return loom.pop(0, count, max_polls=POLLS)


def host_bits_ok(result, sent_by_host):
    """Every byte the host sent: both bit phases T_BIT (64 us, over [S4]'s
    60 for devices and [B]'s 20), within a slot or two; frame handshake in
    1 ms."""
    got = [e for e in result["events"] if e[0] == "byte"]
    for e in got:
        assert e[4] is not None and e[4] <= TIMING["frame_ack_max"][0]
    for e in [e for e in got if (e[1], e[2]) in sent_by_host]:
        near(e[5], T_BIT)
        near(e[6], T_BIT)


def test_cbm_iec_host_sends_and_receives_with_a_model_device(backend):
    """LISTEN 8, SECOND 2, two bytes (the last with EOI), UNLISTEN; then
    TALK 8, SECOND 15, the turnaround, the device's two bytes up to EOI,
    UNTALK. On the wire exactly those bytes in that order and ATN on the
    commands only ([S2]), every window of [B] and [S4] kept."""
    dev = IecDevice(8, reply=[0x41, 0x42], us=FAST)
    bench, bus, loom = host_setup(backend, dev)
    assert do(loom, [op(OP_SEND, 8, 2)] + message([0x12, 0xC5]), 1) == [DONE | OP_SEND]
    assert do(loom, [op(OP_RECEIVE, 8, 15)], 3) == [0x41, 0x42 | EOI, DONE | OP_RECEIVE]
    assert dev.log[:2] == [("cmd", LISTEN | 8), ("cmd", SECOND | 2)]
    assert dev.data == [0x12, 0xC5]
    result = bus_ok(bus)
    assert frames(result["events"]) == [
        [(LISTEN | 8, False), (SECOND | 2, False)],
        [(0x12, False), (0xC5, True)],
        [(UNLISTEN, False)],
        [(TALK | 8, False), (SECOND | 15, False)],
        [(0x41, False), (0x42, True)],
        [(UNTALK, False)]]
    host_bits_ok(result, {(LISTEN | 8, False), (SECOND | 2, False), (0x12, False),
                          (0xC5, True), (UNLISTEN, False),
                          (TALK | 8, False), (SECOND | 15, False), (UNTALK, False)})
    atn = [e for e in result["events"] if e[0] == "atn"]
    assert len(atn) == 4 and all(e[2] is not None for e in atn)
    turns = [e for e in result["events"] if e[0] == "turn"]
    assert len(turns) == 1 and turns[0][2] <= TIMING["turnaround_max"][0]
    assert bench.contention_free() and loom.badop() == 0


def test_cbm_iec_eoi_on_a_one_byte_message(backend):
    """A one-byte message is its own last byte: the host holds CLK back
    after ready for data until the listener's EOI acknowledge ([B] step 2,
    [S4] 'End of Stream'), and as listener it acknowledges a one-byte reply
    the same way. Measured on the wire: the acknowledge comes at least
    200 us after ready for data and lasts at least 60 us (``check``), the
    host resumes within 60 us, and the two bytes carry EOI."""
    dev = IecDevice(8, reply=[0x99], us=FAST)
    bench, bus, loom = host_setup(backend, dev)
    assert do(loom, [op(OP_SEND, 8, 0)] + message([0xAA]), 1) == [DONE | OP_SEND]
    assert do(loom, [op(OP_RECEIVE, 8, 0)], 2) == [0x99 | EOI, DONE | OP_RECEIVE]
    assert dev.log[2] == ("data", 0xAA, True)
    result = bus_ok(bus)
    assert [b for b in bytes_of(result, atn=False)] == [(0xAA, True), (0x99, True)]
    assert all(not eoi for _, eoi in bytes_of(result, atn=True))
    # The host's own EOI acknowledge (as listener) is DATA pulled T_EI = 80 us.
    pulses = [(s, e) for s, e in bus.intervals("chip:uio2", "data") if e is not None]
    widths = [(e - s) / FAST for s, e in pulses]
    assert any(abs(w - 80) <= 1 for w in widths), widths
    assert bench.contention_free() and loom.badop() == 0


def test_cbm_iec_host_reports_an_absent_device(backend):
    """No device on the bus: nobody pulls DATA after ATN, and after 1 ms
    ([B]: 'within 1 millisecond') the host lets the bus go and reports
    0x81 with the LISTEN byte, then drops the send's data word. The next
    operation, a receive, fails the same way with the TALK byte. ATN is
    held at least 1 ms each time, and not much more."""
    dev = IecDevice(8, us=FAST, present=False)
    bench, bus, loom = host_setup(backend, dev)
    assert do(loom, [op(OP_SEND, 8, 1)] + message([0x01]), 1) == [E_NODEV | LISTEN | 8]
    assert do(loom, [op(OP_RECEIVE, 8, 1)], 1) == [E_NODEV | TALK | 8]
    atn = [(s, e) for s, e in bus.intervals("chip:uio0", "atn")]
    assert len(atn) == 2
    for s, e in atn:
        assert e is not None and 1000 <= (e - s) / FAST <= 1010
    assert bus.level == (1, 1, 1)
    assert check(bus.edges, FAST)["violations"] == []
    assert bench.contention_free() and loom.badop() == 0


# ------------------------------------------------------- model controller
def device_setup(backend, ctl, pa, reply, us=FAST):
    """The device program on thread 1 (BIDIR3..5), a model controller."""
    bus = IecBus(["BIDIR3"], ["BIDIR4"], ["BIDIR5"], us=us)
    bus.add(ctl)
    bench = iec_bench(backend, bus)
    loom = backend.loom(bench, isa=ISA)
    loom.load(program(DEVICE))
    loom.push(1, [us, pa] + message(reply))
    loom.run(1)
    bench.step(STARTUP)
    return bench, bus, loom


def device_bits_ok(result, sent_by_device):
    """The bytes the device talked: both phases T_BIT, over [S4]'s 60 us."""
    for e in result["events"]:
        if e[0] == "byte" and (e[1], e[2]) in sent_by_device:
            assert e[5] >= TIMING["bit_device_min"][0] and e[6] >= TIMING["bit_device_min"][0]
            near(e[5], T_BIT)
            near(e[6], T_BIT)


def test_cbm_iec_device_listens_and_talks_to_a_model_controller(backend):
    """A model C64 at [S4]'s fastest controller bits (20 us) sends device 9
    two bytes on secondary 3, then reads it: the device reports SECOND 3
    and the bytes with the EOI flag, and talks back its host-given reply,
    the last byte with EOI, after the turnaround ([B], [S4] C and D)."""
    ctl = IecController(us=FAST, hold_us=20)
    bench, bus, loom = device_setup(backend, ctl, 9, [0x50, 0x0F])
    ctl.send(9, 3, [0xA5, 0x5A])
    ctl.receive(9, 3)
    assert bench.run_until(lambda: len(ctl.results) == 2, 400_000, every=64)
    assert ctl.results == [("send", None), ("receive", [0x50, 0x0F], None)]
    assert loom.pop(1, 4, max_polls=POLLS) == [0x200 | SECOND | 3, 0xA5, 0x5A | EOI,
                                               0x200 | SECOND | 3]
    result = bus_ok(bus)
    assert frames(result["events"])[4] == [(0x50, False), (0x0F, True)], frames(result["events"])
    device_bits_ok(result, {(0x50, False), (0x0F, True)})
    assert bench.contention_free() and loom.badop() == 0


# --------------------------------------------------------------- together
def together(backend, start, pa=8, reply=(0x31, 0x32), host=HOST, device=DEVICE, us=FAST):
    """Host (thread 0, BIDIR0..2) and device (thread 1, BIDIR3..5), the pads
    joined by the bench as one bus, started by ``start(bench, loom)``."""
    bus = IecBus(["BIDIR0", "BIDIR3"], ["BIDIR1", "BIDIR4"], ["BIDIR2", "BIDIR5"], us=us)
    bench = iec_bench(backend, bus)
    loom = backend.loom(bench, isa=ISA)
    words = dict(program(*host).words if isinstance(host, tuple) else program(host).words)
    dev = program(*device).words if isinstance(device, tuple) else program(device).words
    assert not set(words) & set(dev)
    words.update(dev)
    loom.load(words)
    loom.push(0, [us])
    loom.push(1, [us, pa] + message(list(reply)))
    start(bench, loom)
    bench.step(STARTUP)
    return bench, bus, loom


def test_cbm_iec_host_and_device_together(backend):
    """The two programs under one RUN on one bus: the host sends device 8
    two bytes on secondary 1 and reads its two-byte reply. The
    PHY-GL-SDF candidate."""
    bench, bus, loom = together(backend, lambda bench, loom: loom.run([0, 1]))
    assert do(loom, [op(OP_SEND, 8, 1)] + message([0x0D, 0xF0]), 1) == [DONE | OP_SEND]
    assert loom.pop(1, 3, max_polls=POLLS) == [0x200 | SECOND | 1, 0x0D, 0xF0 | EOI]
    assert do(loom, [op(OP_RECEIVE, 8, 1)], 3) == [0x31, 0x32 | EOI, DONE | OP_RECEIVE]
    assert loom.pop(1, 1, max_polls=POLLS) == [0x200 | SECOND | 1]
    result = bus_ok(bus)
    assert [len(f) for f in frames(result["events"])] == [2, 2, 1, 2, 2, 1]
    device_bits_ok(result, {(0x31, False), (0x32, True)})
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


@model_only("six starts of a full exchange, about 60000 clocks each")
@pytest.mark.parametrize("p,order", [(0, "one_run"), (1, "one_run"), (2, "one_run"),
                                     (3, "one_run"), (1, "host_first"), (2, "device_first")])
def test_cbm_iec_together_at_every_start_phase(backend, p, order):
    """BUGS 14: both programs set their OD_MASK bits first and read them
    back. Under one RUN at each phase of the slot grid, and started apart
    in both orders, OD_MASK ends with all five bits and a one-byte message
    gets through with EOI."""
    bench, bus, loom = together(backend, _phase(p, order), reply=(0x77,))
    assert do(loom, [op(OP_SEND, 8, 0)] + message([0x42]), 1) == [DONE | OP_SEND]
    assert loom.pop(1, 2, max_polls=POLLS) == [0x200 | SECOND, 0x42 | EOI]
    assert bench.machine.od_mask & 0x3F == 0b110111
    bus_ok(bus)
    assert bench.contention_free() and loom.badop() == 0


@model_only("the real-time bus: five times the clocks of the RTL scenarios")
def test_cbm_iec_together_at_the_real_speed(backend):
    """TICK_INT 50: one tick is a real microsecond at 50 MHz. The same
    exchange as on the RTL, every window of the sources kept unscaled."""
    bench, bus, loom = together(backend, lambda bench, loom: loom.run([0, 1]), us=50)
    assert do(loom, [op(OP_SEND, 8, 1)] + message([0x0D, 0xF0]), 1) == [DONE | OP_SEND]
    assert loom.pop(1, 3, max_polls=POLLS) == [0x200 | SECOND | 1, 0x0D, 0xF0 | EOI]
    assert do(loom, [op(OP_RECEIVE, 8, 1)], 3) == [0x31, 0x32 | EOI, DONE | OP_RECEIVE]
    result = bus_ok(bus)
    device_bits_ok(result, {(0x31, False), (0x32, True)})
    assert bench.contention_free() and loom.badop() == 0


@model_only("one injected error per error word; the RTL runs the absent device")
def test_cbm_iec_host_reports_a_missing_frame_handshake_and_recovers(backend):
    """A device that does not acknowledge the first data byte ([B] step 4:
    the talker 'knows something is wrong' after 1 ms): 0x82 with the byte,
    the rest of the message dropped, the bus released; the next operation
    works."""
    dev = IecDevice(8, reply=[0x5E], us=FAST, nack=[2])
    bench, bus, loom = host_setup(backend, dev)
    assert do(loom, [op(OP_SEND, 8, 0)] + message([0x11, 0x22, 0x33]), 1) == [E_FRAME | 0x11]
    assert bus.level == (1, 1, 1)
    assert do(loom, [op(OP_RECEIVE, 8, 0)], 2) == [0x5E | EOI, DONE | OP_RECEIVE]
    late = [e for e in check(bus.edges, FAST)["events"] if e[0] == "byte" and e[4] is None]
    assert [e[1] for e in late] == [0x11]
    assert bench.contention_free() and loom.badop() == 0


@model_only("one injected error per error word; the RTL runs the absent device")
def test_cbm_iec_host_reports_a_device_that_does_not_turn_round(backend):
    """A device that answers the TALK but never takes CLK: 0x83 with the
    SECOND byte after 1 ms, and the bus released."""
    dev = IecDevice(8, us=FAST, no_turn=True)
    bench, bus, loom = host_setup(backend, dev)
    assert do(loom, [op(OP_RECEIVE, 8, 4)], 1) == [E_TURN | SECOND | 4]
    assert bus.level == (1, 1, 1)
    assert bench.contention_free() and loom.badop() == 0


@model_only("addressing; the RTL scenarios cover the matching case")
def test_cbm_iec_device_ignores_another_device_number(backend):
    """LISTEN 9 to the device program at 8: it acknowledges ATN like every
    device ([B]) but stores nothing and lets DATA go, so no listener holds
    DATA before the data byte ([S4] "Errors": device not present); then a
    send to 8 is taken as usual."""
    ctl = IecController(us=FAST)
    bench, bus, loom = device_setup(backend, ctl, 8, [0x01])
    ctl.send(9, 0, [0x66])
    ctl.send(8, 0, [0x67])
    assert bench.run_until(lambda: len(ctl.results) == 2, 300_000, every=64)
    assert ctl.results == [("send", "nodev"), ("send", None)]
    assert loom.pop(1, 2, max_polls=POLLS) == [0x200 | SECOND, 0x67 | EOI]
    assert loom.pop_available(1) == []


@model_only("addressing; the RTL scenarios cover the matching case")
def test_cbm_iec_host_reports_a_number_no_device_answers(backend):
    """A send to device 8 with only device 9 on the bus: ATN is answered
    (every device acknowledges it, [B]) but nobody listens, so no one holds
    DATA when the data byte should start ([S4] "Errors": device not
    present): 0x81 with the data byte, and the bus released."""
    dev = IecDevice(9, us=FAST)
    bench, bus, loom = host_setup(backend, dev)
    assert do(loom, [op(OP_SEND, 8, 0)] + message([0x66]), 1) == [E_NODEV | 0x66]
    assert bus.level == (1, 1, 1) and dev.data == []
    assert bench.contention_free() and loom.badop() == 0
