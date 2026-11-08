"""L3-I3C on the model and the RTL: firmware/i3c_controller.loom and i3c_target.loom.

The controller runs on thread 0 (SCL = BIDIR0, SDA = BIDIR1), the target on
thread 1 (SCL = BIDIR2, SDA = BIDIR3); ``tools.protomodels.i3c.I3cBus`` joins
BIDIR0/2 and BIDIR1/3 into the two board wires, with the pull-up, and holds
the model targets and the model controller. Both programs are loaded as one
image in every scenario (the controller spills into thread 2's quarter, the
target's data and code into thread 3's) and only the threads a scenario
needs are run.

Expected values come from the I3C frame rules through the reference model
(``i3c.I3cTarget``, ``i3c.I3cController``; the sources are in its
docstring) and from the program headers (host words and report formats),
never from what the model or the RTL did. Pin timing and the open-drain
phases are checked on the wires by ``i3c.check``: every bit's tLOW and
tHIGH against the SDR table for its phase, the START, Sr and STOP set-up
and hold, the bus free time, the parity of every write and CCC T-bit, and
nobody driving SDA high in an open-drain bit.

Rates (50 MHz): the controller's bit is 16 ticks. With model targets it
runs at TICK_INT 4 (64 clocks a bit, 781 kHz, the fastest its schedule is
proved for); with the Loom target on the bus at TICK_INT 8 (390 kHz), and
the model controller then uses the same bit shape in clocks.
"""

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels import i3c as I
from tools.tests.fw_backend import model_only

ISA = load()
CONTROLLER = REPO / "firmware" / "i3c_controller.loom"
TARGET = REPO / "firmware" / "i3c_target.loom"
IMEM = 512

FAST = 4                        # TICK_INT with model targets: 781 kHz
SLOW = 8                        # TICK_INT with the Loom target: 390 kHz
BIT_TICKS = 16                  # i3c_controller.loom: LOA 6 + LOB 4 + HA 2 + HB 4

# i3c_controller.loom host words
OP_RSTDAA, OP_ENTDAA, OP_WRITE, OP_READ = 0x1000, 0x2000, 0x3000, 0x4000
BAD_T = 0x0080
# i3c_target.loom: identity in data memory, reports
LOOM_PID, LOOM_BCR, LOOM_DCR = 0x0123_4567_89AB, 0x01, 0x00
DATA = 0x180

_IMAGE = None


def image():
    """Both programs, assembled with --strict, as one image."""
    global _IMAGE
    if _IMAGE is None:
        words = {}
        for path in (CONTROLLER, TARGET):
            prog = assemble_file(path, isa=ISA, strict=True)
            assert prog.diagnostics == []
            overlap = set(words) & set(prog.words)
            assert not overlap, "the programs overlap at %s" % sorted(overlap)[:4]
            words.update(prog.words)
        _IMAGE = words
    return _IMAGE


def model_controller(tick):
    """A model controller with the Loom controller's bit shape at ``tick``."""
    return I.I3cController(la=6 * tick, lb=4 * tick, ha=2 * tick, hb=4 * tick,
                           buf=16 * tick)


def setup(backend, *, tick=FAST, controller=True, target=False, agents=()):
    """The chip with ``agents`` on the bus; thread 0 runs the controller and
    thread 1 the target, as asked. Returns ``(bench, loom, bus)``."""
    bus = I.I3cBus(scl=("BIDIR0", "BIDIR2"), sda=("BIDIR1", "BIDIR3"))
    for agent in agents:
        bus.add(agent)
    if backend.name == "model":
        bench = backend.bench(features=("FIFO", "DMEM"), imem_words=IMEM,
                              pullups=bus.pullups)
    else:
        bench = backend.bench(pullups=bus.pullups)
    bench.add(bus)
    loom = backend.loom(bench, isa=ISA)
    loom.load(image())
    threads = ([0] if controller else []) + ([1] if target else [])
    if controller:
        loom.write_csr(0, "TICK_INT", tick)
    loom.run(threads)
    bench.step(200)
    return bench, loom, bus


def polls(tick, bits):
    return bits * BIT_TICKS * tick // 40 + 2000


def entdaa_words(rounds, first):
    """The controller's report of an ENTDAA that assigns to ``rounds``
    (``(pid, bcr, dcr)`` in arbitration order) from address ``first``."""
    out = []
    for k, (pid, bcr, dcr) in enumerate(rounds):
        out += [pid >> 32 & 0xFFFF, pid >> 16 & 0xFFFF, pid & 0xFFFF,
                bcr << 8 | dcr, 0x2000 | (first + k)]
    return out + [0x2E00 | len(rounds)]


def assert_clean(bench, bus):
    report = I.check(bus)
    assert report.violations == []
    assert bus.contentions == []
    assert bench.contention_free()
    return report


# ------------------------------------------------- controller, model targets
def test_i3c_controller_assigns_two_model_targets(backend):
    """RSTDAA, then ENTDAA to two model targets: the lower 64-bit value
    (PID, BCR, DCR) wins the first round and gets the first address; then
    a private write to the second and a read back of it. 781 kHz."""
    a = I.I3cTarget(0x0000_1234_5678, 0x01, 0x22, name="A")
    b = I.I3cTarget(0x0000_1234_5600, 0x01, 0x33, name="B")
    bench, loom, bus = setup(backend, agents=(a, b))
    loom.push(0, [OP_RSTDAA])
    assert loom.pop(0, 1, max_polls=polls(FAST, 40)) == [0x1000]
    loom.push(0, [OP_ENTDAA | 0x08])
    order = sorted([(a.pid, a.bcr, a.dcr), (b.pid, b.bcr, b.dcr)],
                   key=lambda r: I.daa_value(*r))
    assert order[0][0] == b.pid
    assert loom.pop(0, 11, max_polls=polls(FAST, 300)) == entdaa_words(order, 0x08)
    assert (b.da, a.da) == (0x08, 0x09)
    loom.push(0, [OP_WRITE | 3 << 8 | 0x09, 0xC3, 0x00, 0x7F])
    assert loom.pop(0, 1, max_polls=polls(FAST, 60)) == [0x3003]
    assert a.buffer == [0xC3, 0x00, 0x7F] and b.buffer == []
    loom.push(0, [OP_READ | 3 << 8 | 0x09])
    assert loom.pop(0, 3, max_polls=polls(FAST, 60)) == [0x41C3, 0x4100, 0x407F]
    report = assert_clean(bench, bus)
    assert report.cccs == [I.CCC_RSTDAA, I.CCC_ENTDAA]
    assert [d["pid"] for d in report.daa] == [b.pid, a.pid]
    assert all(d["par_ok"] and d["ack"] for d in report.daa)
    assert report.writes == [{"da": 0x09, "t_errors": [],
                              "data": [(x, I.t_bit(x)) for x in (0xC3, 0x00, 0x7F)]}]
    # the bit period on the wire: 16 ticks of 4 clocks, each edge 0 to 3 late
    seg = report.segments[-1]
    periods = [y.rise - x.rise for x, y in zip(seg.bits[9:16], seg.bits[10:17])]
    assert all(abs(p - BIT_TICKS * FAST) <= 3 for p in periods), periods


def test_i3c_controller_bad_t_bit_is_refused(backend):
    """A write whose first T-bit is forced wrong: the model target refuses
    the transfer and keeps nothing, so the read that follows is NACKed. The
    checker sees the bad parity on the wire."""
    t = I.I3cTarget(0x0000_00AB_CDEF, 0x01, 0x00)
    bench, loom, bus = setup(backend, agents=(t,))
    loom.push(0, [OP_ENTDAA | 0x20])
    assert loom.pop(0, 6, max_polls=polls(FAST, 200)) == entdaa_words([(t.pid, 1, 0)], 0x20)
    loom.push(0, [OP_WRITE | BAD_T | 2 << 8 | 0x20, 0x55, 0x66])
    assert loom.pop(0, 1, max_polls=polls(FAST, 60)) == [0x3002]
    assert t.refused == 1 and t.buffer == []
    loom.push(0, [OP_READ | 1 << 8 | 0x20])
    assert loom.pop(0, 1, max_polls=polls(FAST, 40)) == [0xC000]
    report = I.check(bus)
    assert report.violations == [] and bus.contentions == []
    assert report.writes[0]["t_errors"] == [0]


def test_i3c_controller_entdaa_on_an_empty_bus(backend):
    """No target: the 0x7E header is NACKed and the controller reports it."""
    bench, loom, bus = setup(backend)
    loom.push(0, [OP_ENTDAA | 0x08])
    assert loom.pop(0, 1, max_polls=polls(FAST, 40)) == [0xAE00]
    assert_clean(bench, bus)


# ------------------------------------------------- Loom target, model controller
def test_i3c_target_with_model_controller(backend):
    """The Loom target under a model controller at 390 kHz: RSTDAA, ENTDAA
    (its PID, BCR and DCR from data memory), a private write, a read of all
    of it (T = 0 on the last byte), a read the controller ends after one
    byte with an Sr (the target had more), then a write with a bad T-bit,
    refused, after which the target has nothing to read and NACKs."""
    ctl = model_controller(SLOW)
    bench, loom, bus = setup(backend, controller=False, target=True, agents=(ctl,))
    ctl.rstdaa()
    ctl.entdaa(0x30)
    ctl.write(0x30, [0xA5, 0x5A, 0x00])
    ctl.read(0x30, 3)
    ctl.read(0x30, 1)
    ctl.write(0x30, [0x11], bad_t=True)
    ctl.read(0x30, 1)
    assert bench.run_until(lambda: ctl.idle, 400 * BIT_TICKS * SLOW, every=256)
    assert loom.pop(1, 4, max_polls=200) == [0x1000, 0x2030, 0x3003, 0xB000]
    r = ctl.results
    assert [x["op"] for x in r] == ["rstdaa", "entdaa", "write", "read", "read", "write", "read"]
    assert r[1]["rounds"] == [{"pid": LOOM_PID, "bcr": LOOM_BCR, "dcr": LOOM_DCR,
                               "da": 0x30, "ack": True}]
    assert r[3]["data"] == [(0xA5, 1), (0x5A, 1), (0x00, 0)]
    assert r[4]["data"] == [(0xA5, 1)]
    assert r[6]["ack"] is False
    report = assert_clean(bench, bus)
    assert [w["t_errors"] for w in report.writes] == [[], [0]]


def test_i3c_target_loses_arbitration_to_a_lower_pid(backend):
    """ENTDAA from a model controller with the Loom target and a model target
    whose PID is lower: the model wins round one, the Loom target round two."""
    ctl = model_controller(SLOW)
    low = I.I3cTarget(0x0123_4567_0000, 0x01, 0x00, name="low")
    bench, loom, bus = setup(backend, controller=False, target=True, agents=(ctl, low))
    ctl.entdaa(0x10)
    assert bench.run_until(lambda: ctl.idle, 300 * BIT_TICKS * SLOW, every=256)
    rounds = ctl.results[0]["rounds"]
    assert [(x["pid"], x["da"], x["ack"]) for x in rounds] == [
        (low.pid, 0x10, True), (LOOM_PID, 0x11, True)]
    assert low.da == 0x10
    assert loom.pop(1, 1, max_polls=200) == [0x2011]
    assert_clean(bench, bus)


# ------------------------------------------------- both on one chip
def test_i3c_controller_and_target_on_one_chip(backend):
    """Thread 0's controller assigns addresses to thread 1's target and to a
    model target with a higher PID, across the bench's wires, then writes
    to and reads back from the Loom target. 390 kHz."""
    high = I.I3cTarget(0x7FFF_0000_0001, 0x00, 0x00, name="high")
    bench, loom, bus = setup(backend, tick=SLOW, target=True, agents=(high,))
    loom.push(0, [OP_ENTDAA | 0x08])
    rounds = [(LOOM_PID, LOOM_BCR, LOOM_DCR), (high.pid, high.bcr, high.dcr)]
    assert loom.pop(0, 11, max_polls=polls(SLOW, 300)) == entdaa_words(rounds, 0x08)
    assert high.da == 0x09
    assert loom.pop(1, 1, max_polls=200) == [0x2008]
    loom.push(0, [OP_WRITE | 2 << 8 | 0x08, 0xBE, 0xEF])
    assert loom.pop(0, 1, max_polls=polls(SLOW, 40)) == [0x3002]
    assert loom.pop(1, 1, max_polls=polls(SLOW, 20)) == [0x3002]
    loom.push(0, [OP_READ | 4 << 8 | 0x08])
    assert loom.pop(0, 2, max_polls=polls(SLOW, 40)) == [0x41BE, 0x40EF]
    assert_clean(bench, bus)


# ------------------------------------------------- the checker itself
class _PushPullArbiter(I.I3cTarget):
    """A broken target that drives its 1s high in the ENTDAA arbitration."""

    def _set(self, cycle, value):
        if self.mode == "arb" and value is None and self.tx:
            value = 1
        super()._set(cycle, value)


def test_i3c_checker_flags_push_pull_in_open_drain():
    """The checker is not vacuous: a target driving SDA high during the
    arbitration is reported, by name, and the wire stays readable."""
    bus = I.I3cBus(scl=(), sda=())
    ctl = bus.add(model_controller(4))
    bus.add(_PushPullArbiter(0x0000_0000_00F0, name="bad"))
    ctl.entdaa(0x08)
    assert bus.run_alone(lambda: ctl.idle, 100000)
    report = I.check(bus)
    assert report.violations
    assert all("open-drain bit by bad" in v for v in report.violations)
