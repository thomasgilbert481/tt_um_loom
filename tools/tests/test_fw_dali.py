"""L3-DALI on the model and the RTL: firmware/dali_controller.loom and dali_gear.loom.

The controller (thread 0, TX = OUT0, RX = IN0) sends the forward frames the
host asks for and reads the backward frames; the gear (thread 0 alone, or
thread 1 beside the controller on OUT1 and IN1) answers at its short
address. ``tools.protomodels.dali.DaliBus`` makes the bus the wired AND of
the chip's TX pins and the model nodes, seen by the chip's RX pins.

Expected values come from IEC 62386-101 and -102 through the reference
model (frame coding, the gear's commands, the timing limits, with the
sources the model names) and from the program headers (command and report
formats), never from what the model or the RTL did. Pin timing is measured
on the bus: every half-bit the chip sends against the transmitter's limits
(``check_tx``), the gear's answer against its settling window, the
controller's forward frames against the settling time between frames.

Rate: 1200 bit/s, the only one DALI has, is a half-bit of 20,833 clocks at
50 MHz (TICK 2604 + 43/256, eight ticks a half-bit) and a query about 2.5
million clocks: that runs on the model only. The other bodies run the whole
bus at a test rate, TICK 25, a half-bit of 200 clocks, with every time in
the model nodes scaled with it (``scale_for``); the programs count every
time in ticks, so only the speed changes.

Every body takes a ``backend`` (``tools/tests/fw_backend.py``): pytest runs
them on the golden model, ``test/test_fw.py`` on the RTL.
"""

import pytest

from tools.loomasm import assemble_file, assemble_text
from tools.loomisa import REPO, load
from tools.protomodels import dali as D
from tools.tests.fw_backend import model_only

ISA = load()
CONTROLLER = REPO / "firmware" / "dali_controller.loom"
GEAR = REPO / "firmware" / "dali_gear.loom"
IMEM = 512

# Te/8 a tick (program headers): the standard rate and the test rate
STD = (2604, 43)
FAST = (25, 0)

# the controller's status bits and the gear's word B flags (program headers)
BACK, FRAMING, NONE = 0x100, 0x200, 0x400
REPLIED, ERROR, DROPPED, UNKNOWN = 0x100, 0x200, 0x400, 0x800
# the ends of the receiver's half-bit range, 333.3 and 500 us (the first a
# hair inside 1000/3, so that its test id stays short)
RX_LIMITS = [333.34, D.RX_HALF[1]]


def tick(rate):
    ti, tf = rate
    return ti + tf / 256


def us_of(rate):
    """Clocks per microsecond with the bus scaled to ``rate``."""
    return D.scale_for(8 * tick(rate))


def dali_bench(backend):
    """FIFOs, the bit engine with slice A's encoders and the 512-word
    memory on the model; the RTL has all of them and takes no keyword."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "BE", "BEENC"), imem_words=IMEM)
    return backend.bench()


def start(backend, bench, image, rate, threads=(0,)):
    loom = backend.loom(bench, isa=ISA)
    loom.load(image)
    for t in threads:
        loom.write_csr(t, "TICK_INT", rate[0])
        loom.write_csr(t, "TICK_FRAC", rate[1])
    loom.run(list(threads))
    return loom


def run_for(bench, us, micro):
    bench.step(int(micro * us))


def forward_frames(bus):
    return [f for f in bus.decoder.frames if f.is_forward]


def assert_sent_well(frames, us):
    """Every half-bit of every frame inside the transmitter's limits."""
    for f in frames:
        assert f.bits and not f.violation, f
        assert D.check_tx(f, us) == [], (hex(f.value), D.check_tx(f, us))


# -------------------------------------------------------------- controller
def controller_setup(backend, rate, gears=()):
    """The chip controller at ``rate`` with model gears on the bus, each
    ``(address, groups, keyword arguments)``; returns ``(bench, loom, bus,
    nodes)``."""
    us = us_of(rate)
    bench = dali_bench(backend)
    nodes = [D.DaliGearNode(us, a, g, **kw) for a, g, kw in gears]
    bus = bench.add(D.DaliBus(us, nodes))
    loom = start(backend, bench, assemble_file(CONTROLLER, isa=ISA, strict=True).words, rate)
    return bench, loom, bus, nodes


def ask(loom, rate, reply, frame, thread=0):
    """One forward frame through the chip controller: ``(status, settle)``."""
    loom.push(thread, [int(reply), frame])
    polls = int(60_000 * us_of(rate)) // 40 + 2000
    return tuple(loom.pop(thread, 2, max_polls=polls))


def settle_us(settle, rate):
    """The controller's settling ticks in microseconds."""
    return settle * tick(rate) / us_of(rate)


def test_controller_sends_and_reads_a_gear(backend):
    """At the test rate: a direct arc power frame (no answer) and a QUERY
    ACTUAL LEVEL to a model gear. Both forward frames are inside the
    transmitter's limits, the second starts 13.5 to 14.7 ms after the
    first, the answer is read and its settling time reported to within
    two ticks of what the bus shows."""
    us = us_of(FAST)
    bench, loom, bus, (gear,) = controller_setup(backend, FAST, [(3, 0, {})])
    assert ask(loom, FAST, 0, (D.short(3) << 8) | 128) == (0, 0)
    status, settle = ask(loom, FAST, 1, (D.short(3, True) << 8) | D.QUERY_LEVEL)
    assert status == BACK | 128, hex(status)
    run_for(bench, us, 3000)                    # the backward frame closes
    fw = forward_frames(bus)
    assert [f.value for f in fw] == [0x0680, 0x07A0]
    assert_sent_well(fw, us)
    assert D.FORWARD_SETTLE[0] <= D.settling_us(fw[0], fw[1], us) <= D.FORWARD_SETTLE[1]
    back = bus.decoder.frames[-1]
    assert back.is_backward and back.value == 128
    measured = D.settling_us(fw[1], back, us)
    assert abs(settle_us(settle, FAST) - measured) <= 2 * tick(FAST) / us, (settle, measured)
    assert [f.value for f, _ in gear.acted] == [0x0680, 0x07A0]
    assert loom.badop() == 0


@model_only("three more answers inside the window the late-answer case runs on the RTL")
@pytest.mark.parametrize("answer_us", [D.GEAR_ANSWER[0], D.GEAR_ANSWER[1], 12000.0])
def test_controller_takes_answers_across_the_window(backend, answer_us):
    """A gear may answer 5.5 to 10.5 ms after the frame; the controller
    listens to 12.4 ms, so 12 ms is read too."""
    bench, loom, bus, _ = controller_setup(backend, FAST, [(9, 0, {"answer_us": answer_us})])
    status, settle = ask(loom, FAST, 1, (D.short(9, True) << 8) | D.QUERY_PRESENT)
    assert status == BACK | D.YES, hex(status)
    assert abs(settle_us(settle, FAST) - answer_us) <= 3 * tick(FAST) / us_of(FAST)


def test_controller_late_answer_is_no_answer(backend):
    """An answer 13 ms after the frame is outside the window: NONE. The
    controller's next frame waits until that backward frame is over and
    the bus has settled again, so it never talks over the gear."""
    us = us_of(FAST)
    bench, loom, bus, _ = controller_setup(backend, FAST, [(9, 0, {"answer_us": 13000.0})])
    assert ask(loom, FAST, 1, (D.short(9, True) << 8) | D.QUERY_LEVEL) == (NONE, 0)
    assert ask(loom, FAST, 0, (D.short(9, True) << 8) | D.OFF) == (0, 0)
    run_for(bench, us, 3000)
    frames = bus.decoder.frames
    assert [(f.is_forward, f.is_backward) for f in frames] == \
        [(True, False), (False, True), (True, False)]
    assert D.settling_us(frames[0], frames[1], us) > D.BACKWARD_WINDOW
    assert D.settling_us(frames[1], frames[2], us) >= D.FORWARD_SETTLE[0]
    assert_sent_well([frames[0], frames[2]], us)


@pytest.mark.parametrize("half_us", RX_LIMITS)
def test_controller_reads_a_gear_at_the_receive_limits(backend, half_us):
    """A gear whose half-bits are 333.3 or 500 us, the ends of the
    receiver's range (62386-101 tables 18 and 19), with every double
    half-bit twice that: the controller reads its answer."""
    bench, loom, bus, _ = controller_setup(
        backend, FAST, [(12, 0x0081, {"half_us": half_us})])
    status, _ = ask(loom, FAST, 1, (D.short(12, True) << 8) | D.QUERY_GROUPS_LO)
    assert status == BACK | 0x81, hex(status)
    run_for(bench, us_of(FAST), 3000)
    back = bus.decoder.frames[-1]
    assert back.is_backward and D.check_rx(back, us_of(FAST)) == []


@model_only("the corrupted answer is read by the same decoder the RTL runs in the other cases")
def test_controller_two_gears_at_once_is_a_framing_error(backend):
    """Two gears in group 2 answer QUERY GROUPS 0-7 together with different
    bytes; the wired AND is no backward frame, so the status is FRAMING
    (62386-102 reads a corrupted answer as "more than one answered")."""
    bench, loom, bus, gears = controller_setup(
        backend, FAST, [(1, 0x0004, {}), (2, 0x00F4, {})])
    status, settle = ask(loom, FAST, 1, (D.group(2, True) << 8) | D.QUERY_GROUPS_LO)
    assert status == FRAMING, hex(status)
    assert settle > 0
    assert all(len(g.acted) == 1 for g in gears)


@model_only("1200 bit/s: about 2.5 million clocks a query")
def test_controller_at_the_standard_rate(backend):
    """At 1200 bit/s on a 50 MHz chip (TICK 2604 + 43/256): a query and its
    answer, every half-bit and settling time in microseconds against the
    standard."""
    us = us_of(STD)
    assert abs(us - 50.0) < 0.01
    bench, loom, bus, (gear,) = controller_setup(backend, STD, [(40, 0, {})])
    status, settle = ask(loom, STD, 1, (D.short(40, True) << 8) | D.QUERY_STATUS)
    assert status == BACK | 0x04, hex(status)          # lamp on at power-up
    run_for(bench, us, 3000)
    fw = forward_frames(bus)
    back = bus.decoder.frames[-1]
    assert_sent_well(fw, us)
    for d, n in fw[0].intervals:                       # 416.7 us, to a slot
        assert abs(d / us - n * D.TE_US) < 0.2, d
    assert D.GEAR_ANSWER[0] <= D.settling_us(fw[0], back, us) <= D.GEAR_ANSWER[1]
    assert abs(settle_us(settle, STD) - D.settling_us(fw[0], back, us)) <= 110


# -------------------------------------------------------------------- gear
def gear_setup(backend, rate, address, groups=0, half_us=D.TE_US):
    """The chip gear (thread 0) at ``address`` with a model controller whose
    half-bits are ``half_us``; returns ``(bench, loom, bus, controller)``."""
    us = us_of(rate)
    bench = dali_bench(backend)
    ctl = D.DaliControllerNode(us, half_us=half_us)
    bus = bench.add(D.DaliBus(us, [ctl]))
    loom = start(backend, bench, assemble_file(GEAR, isa=ISA, strict=True).words, rate)
    loom.push(0, [address, groups])
    bench.step(200)
    return bench, loom, bus, ctl


def send(bench, bus, ctl, address, data, **kw):
    """The model controller sends one frame, a priority-1 settling time after
    the last frame on the bus, and the bench runs until any answer is over;
    returns the :class:`Transfer`."""
    us = ctl.us
    at = ctl.cycle + 2
    if bus.decoder.frames:
        at = max(at, bus.decoder.frames[-1].end + int(D.FORWARD_SETTLE[0] * us))
    tr = ctl.send(address, data, at=at, **kw)
    bench.step(tr.end - ctl.cycle + int(21_000 * us))
    return tr


def reports(loom):
    words = loom.pop_available(0)
    assert len(words) % 2 == 0, words
    return [tuple(words[i:i + 2]) for i in range(0, len(words), 2)]


def expected_report(state, a, d):
    """The gear's report for frame ``(a, d)`` from the 62386-102 model and
    the program header, or ``None`` when it is not addressed."""
    if not state.addressed(a):
        return None, None
    if not state.known(a, d):
        return ((a << 8) | d, UNKNOWN | state.level), None
    answer = state.apply(a, d)
    return ((a << 8) | d, state.level | (REPLIED if answer is not None else 0)), answer


def assert_answer(bus, tr, answer, us):
    """The backward frame after ``tr``: ``answer``, sent inside the
    transmitter's limits and the gear's settling window."""
    fwd = [f for f in bus.decoder.frames if abs(f.start - tr.start) <= 2]
    assert len(fwd) == 1 and fwd[0].is_forward, fwd
    after = [f for f in bus.decoder.frames if f.start > fwd[0].end]
    if answer is None:
        assert after == [], after
        return
    back = after[0]
    assert back.is_backward and back.value == answer, (back.value, answer)
    assert_sent_well([back], us)
    assert D.GEAR_ANSWER[0] <= D.settling_us(fwd[0], back, us) <= D.GEAR_ANSWER[1]


@pytest.mark.parametrize("half_us", RX_LIMITS)
def test_gear_follows_a_controller_at_the_receive_limits(backend, half_us):
    """A controller whose half-bits are 333.3 or 500 us (the receiver's
    range): the gear takes a direct arc power level and answers QUERY
    ACTUAL LEVEL with it, inside the 5.5 to 10.5 ms window, and reports
    both frames."""
    us = us_of(FAST)
    bench, loom, bus, ctl = gear_setup(backend, FAST, 5, half_us=half_us)
    send(bench, bus, ctl, D.short(5), 77)
    tr = send(bench, bus, ctl, D.short(5, True), D.QUERY_LEVEL)
    assert_answer(bus, tr, 77, us)
    assert reports(loom) == [(0x0A4D, 77), (0x0BA0, REPLIED | 77)]
    assert loom.badop() == 0


ADDRESSING = [
    (D.broadcast(), 100),                       # direct arc power, broadcast
    (D.group(4), 50),                           # a member of group 4
    (D.group(2), 10),                           # not a member: ignored
    (D.short(6), 20),                           # another gear: ignored
    (D.short(5, True), D.OFF),
    (D.short(5, True), D.QUERY_STATUS),         # lamp off: 0x00
    (D.broadcast(True), D.RECALL_MAX),
    (D.broadcast(True, unaddressed=True), D.QUERY_PRESENT),   # has an address
    (D.group(1, True), D.QUERY_GROUPS_LO),
    (D.short(5, True), D.QUERY_GROUPS_HI),
    (D.short(5, True), D.QUERY_STATUS),         # lamp on: 0x04
    (D.short(5, True), D.RECALL_MIN),
    (D.short(5), D.MASK),                       # no change
    (D.short(5, True), 0x20),                   # RESET: not implemented
    (0xA1, 0x00),                               # TERMINATE, a special command
    (D.short(5, True), D.QUERY_PRESENT),
]


@model_only("sixteen frames, about 240,000 clocks; the RTL runs short, group and broadcast in the together test")
def test_gear_addressing_and_commands(backend):
    """Short, group and broadcast addressing and every implemented command,
    against the 62386-102 model: the reports, the levels and every answer
    with its timing; frames for others, special commands and the
    unaddressed broadcast (this gear has an address) are not reported."""
    us = us_of(FAST)
    bench, loom, bus, ctl = gear_setup(backend, FAST, 5, groups=0x0012)
    state = D.GearState(5, 0x0012)
    for a, d in ADDRESSING:
        report, answer = expected_report(state, a, d)
        tr = send(bench, bus, ctl, a, d)
        assert_answer(bus, tr, answer, us)
        want = [] if report is None else [report]
        assert reports(loom) == want, (hex(a), hex(d))
    assert loom.badop() == 0


@model_only("one frame; the addressed cases are in the test above")
def test_gear_without_an_address_takes_the_unaddressed_broadcast(backend):
    """A gear with no short address (64) answers the broadcast to gear
    without one, and QUERY STATUS says so (bit 6)."""
    us = us_of(FAST)
    bench, loom, bus, ctl = gear_setup(backend, FAST, 64)
    tr = send(bench, bus, ctl, D.broadcast(True, unaddressed=True), D.QUERY_STATUS)
    assert_answer(bus, tr, 0x44, us)
    assert reports(loom) == [(0xFD90, REPLIED | 254)]


def assert_error_then_recovers(bench, loom, bus, ctl, level, violation):
    """One ERROR report with the level unchanged and nothing answered, then
    a good QUERY ACTUAL LEVEL is answered as usual."""
    (a_word, b_word), = reports(loom)
    assert b_word & (ERROR | REPLIED | 0xFF) == ERROR | (REPLIED if violation else 0) | level, hex(b_word)
    assert [f for f in bus.decoder.frames if f.is_backward] == []
    tr = send(bench, bus, ctl, D.short(5, True), D.QUERY_LEVEL)
    assert_answer(bus, tr, level, ctl.us)
    assert reports(loom) == [(0x0BA0, REPLIED | level)]


def test_gear_rejects_a_half_bit_outside_the_limits(backend):
    """A forward frame with one half-bit of 1300 us, inside table 19's bit
    timing violation (1200 to 1400 us): not a frame. The gear reports the
    violation (word B bit 8), does not act or answer, and takes the next
    frame."""
    bench, loom, bus, ctl = gear_setup(backend, FAST, 5)
    send(bench, bus, ctl, D.short(5), 0x33, bad_half=(10, 1300.0))
    assert D.check_rx(bus.decoder.frames[0], ctl.us) != []
    assert_error_then_recovers(bench, loom, bus, ctl, D.POWER_ON_LEVEL, True)


@model_only("the error path the bad half-bit case runs on the RTL")
@pytest.mark.parametrize("address, data", [(D.short(5, True), D.QUERY_LEVEL),
                                           (D.short(5), 0x81)])
def test_gear_rejects_a_missing_stop_condition(backend, address, data):
    """The frame is sent again 1 ms after its end, so it has no stop
    condition (2.4 ms high): a frame ending on a 0 runs into the next start
    bit with a violation, one ending on a 1 is followed by an edge before the
    stop condition. Either way the gear neither acts nor answers."""
    bench, loom, bus, ctl = gear_setup(backend, FAST, 5)
    send(bench, bus, ctl, address, data, stop_gap_us=1000.0)
    assert_error_then_recovers(bench, loom, bus, ctl, D.POWER_ON_LEVEL, data == D.QUERY_LEVEL)


@model_only("1200 bit/s: about 2 million clocks a query")
def test_gear_at_the_standard_rate(backend):
    """At 1200 bit/s on a 50 MHz chip: QUERY ACTUAL LEVEL from a nominal
    controller, answered 5.5 to 10.5 ms after the frame with every half-bit
    within a slot of 416.7 us."""
    us = us_of(STD)
    bench, loom, bus, ctl = gear_setup(backend, STD, 33)
    tr = send(bench, bus, ctl, D.short(33, True), D.QUERY_LEVEL)
    assert_answer(bus, tr, D.POWER_ON_LEVEL, us)
    back = bus.decoder.frames[-1]
    for d, n in back.intervals:
        assert abs(d / us - n * D.TE_US) < 0.2, d
    assert reports(loom) == [(0x43A0, REPLIED | D.POWER_ON_LEVEL)]


# ---------------------------------------------------------------- together
def together_image():
    """The controller on thread 0 (OUT0, IN0) and the gear on thread 1
    (OUT1, IN1): the gear's file with only its ``.thread`` and ``.pins``
    lines changed, both assembled for the 512-word memory, as one image."""
    words = {}
    for path, edits in ((CONTROLLER, ()),
                        (GEAR, ((".thread 0", ".thread 1"),
                                (".pins   TX = OUT0, RX = IN0",
                                 ".pins   TX = OUT1, RX = IN1")))):
        text = path.read_text(encoding="utf-8")
        for old, new in edits:
            assert text.count(old) == 1, old
            text = text.replace(old, new)
        prog = assemble_text(text, path.name, isa=ISA, imem_words=IMEM, strict=True)
        assert prog.diagnostics == []
        overlap = set(words) & set(prog.words)
        assert not overlap, "the programs overlap at %s" % sorted(overlap)[:4]
        words.update(prog.words)
    return words


def test_controller_and_gear_together(backend):
    """The controller on thread 0 and the gear on thread 1, each with its
    own TX and RX pins, the bench wiring both TX pins into one bus seen by
    both RX pins. Short, group and broadcast frames, two of them queries:
    both programs' reports agree with the 62386-102 model, and on the bus
    every half-bit is inside the transmitter's limits, every answer inside
    the gear's window and every forward frame a priority-1 settling time
    after the frame before it."""
    us = us_of(FAST)
    bench = dali_bench(backend)
    bus = bench.add(D.DaliBus(us, tx=("OUT0", "OUT1"), rx=("IN0", "IN1")))
    loom = start(backend, bench, together_image(), FAST, threads=(0, 1))
    loom.push(1, [7, 0x0008])
    state = D.GearState(7, 0x0008)
    for a, d in [(D.short(7), 200), (D.short(7, True), D.QUERY_LEVEL),
                 (D.group(3, True), D.OFF), (D.broadcast(True), D.QUERY_STATUS)]:
        report, answer = expected_report(state, a, d)
        status, settle = ask(loom, FAST, answer is not None, (a << 8) | d)
        assert status == (0 if answer is None else BACK | answer), hex(status)
        assert (settle == 0) == (answer is None)
        assert tuple(loom.pop(1, 2, max_polls=4000)) == report
    run_for(bench, us, 3000)
    frames = bus.decoder.frames
    assert [f.is_forward for f in frames] == [True, True, False, True, True, False]
    assert_sent_well(frames, us)
    for before, after in zip(frames, frames[1:]):
        gap = D.settling_us(before, after, us)
        if after.is_backward:
            assert D.GEAR_ANSWER[0] <= gap <= D.GEAR_ANSWER[1], gap
        else:
            assert D.FORWARD_SETTLE[0] <= gap <= D.FORWARD_SETTLE[1], gap
    assert loom.badop() == 0
