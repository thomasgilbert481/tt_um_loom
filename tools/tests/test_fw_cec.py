"""L3-CEC on the model and the RTL: firmware/cec_initiator.loom and cec_follower.loom.

The initiator runs on thread 0 with CEC = BIDIR0, the follower on thread 1
with CEC = BIDIR1, both open drain; ``tools.protomodels.cec.CecBus`` joins
the two pads and the model nodes into one wired-AND line (the board wire),
with the bench's pull-ups on both pads. Both programs are loaded as one
image in every scenario and only the threads a scenario needs are run.

Expected values come from the CEC frame layout and timing through the
reference model (and the sources its docstring names) and from the program
headers (host words and report formats), never from what the model or the
RTL did. Pin timing is measured on the line: every frame's start bit, bit
lows and bit periods against the specification's windows
(``cec.check_frame``), the follower's ACK low on its own pin, and the
signal free time between frames.

Rate: the programs count ticks of 50 us, TICK_INT 2500 at 50 MHz, where one
short frame is millions of clocks; that runs on the model only
(``test_real_rate_polling_message``). The other bodies run at a test tick
of 32 clocks (a 1536-clock bit) with every time on the bus scaled with it
(``cec.scale_for``); the programs count in ticks, so only the speed changes.
"""

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels import cec as C
from tools.tests.fw_backend import model_only

ISA = load()
INITIATOR = REPO / "firmware" / "cec_initiator.loom"
FOLLOWER = REPO / "firmware" / "cec_follower.loom"
IMEM = 512

REAL = (2500, 0)                    # 50 us at 50 MHz
FAST = (32, 0)                      # the test tick

# initiator report (cec_initiator.loom header)
R_ACK, R_NACK, R_LOST = 1, 2, 3
# follower status bits (cec_follower.loom header)
BCAST, DIRECT, SHORT, LONG, NOEOM, OVERFLOW = 0x20, 0x40, 0x80, 0x100, 0x200, 0x400

CHIP_INIT = 4                       # the chip initiator's logical address
CHIP_FOLLOW = 0                     # the chip follower's (a TV)

_IMAGE = None


def image():
    """Both programs, assembled with --strict, as one image."""
    global _IMAGE
    if _IMAGE is None:
        words = {}
        for path in (INITIATOR, FOLLOWER):
            prog = assemble_file(path, isa=ISA, strict=True)
            assert prog.diagnostics == []
            overlap = set(words) & set(prog.words)
            assert not overlap, "the programs overlap at %s" % sorted(overlap)[:4]
            words.update(prog.words)
        _IMAGE = words
    return _IMAGE


def us_of(rate):
    return C.scale_for(rate[0] + rate[1] / 256)


def bit_clocks(rate):
    return C.BIT_US * us_of(rate)


def setup(backend, rate, nodes=(), initiator=True, follower=False,
          init_addr=CHIP_INIT, follow_addr=CHIP_FOLLOW):
    """The chip at ``rate`` on a bus with model ``nodes``; thread 0 runs
    the initiator, thread 1 the follower, as asked. Returns
    ``(bench, loom, bus)``."""
    us = us_of(rate)
    bus = C.CecBus(us, nodes, pins=("BIDIR0", "BIDIR1"))
    if backend.name == "model":
        bench = backend.bench(features=("FIFO", "DMEM"), imem_words=IMEM,
                              pullups=bus.pullups)
    else:
        bench = backend.bench(pullups=bus.pullups)
    bench.add(bus)
    loom = backend.loom(bench, isa=ISA)
    loom.load(image())
    threads = ([0] if initiator else []) + ([1] if follower else [])
    for t in threads:
        loom.write_csr(t, "TICK_INT", rate[0])
        loom.write_csr(t, "TICK_FRAC", rate[1])
    if initiator:
        loom.push(0, [init_addr])
    if follower:
        loom.push(1, [follow_addr])
    loom.run(threads)
    bench.step(int(2 * bit_clocks(rate)))   # the threads reach their idle loops
    return bench, loom, bus


def message(loom, destination, data):
    """Queue one message on the chip initiator (header word, then data)."""
    words = [(destination << 8) | len(data)]
    for k in range(0, len(data), 2):
        hi = data[k] << 8
        lo = data[k + 1] if k + 1 < len(data) else 0
        words.append(hi | lo)
    loom.push(0, words)


def polls(rate, bits):
    return int(bits * bit_clocks(rate)) // 40 + 2000


def initiator_report(loom, rate, bits=200):
    """``(result, block, retransmissions)`` of the next initiator report."""
    w = loom.pop(0, 1, max_polls=polls(rate, bits))[0]
    assert w & ~0x7F3 == 0, hex(w)
    return w & 3, (w >> 4) & 15, (w >> 8) & 7


def follower_report(loom, rate, bits=200):
    """``(status flags, bytes)`` of the next follower report."""
    status = loom.pop(1, 1, max_polls=polls(rate, bits))[0]
    n = status & 0x1F
    words = loom.pop(1, (n + 1) // 2, max_polls=polls(rate, 20)) if n else []
    data = []
    for w in words:
        data += [w >> 8, w & 0xFF]
    if n % 2:
        assert data[-1] == 0
    return status & ~0x1F, data[:n]


def complete_frames(bus):
    return [f for f in bus.frames() if f.bits]


def assert_timing(bus, us):
    for f in complete_frames(bus):
        assert C.check_frame(f, us) == [], (f.data, C.check_frame(f, us))


# ------------------------------------------------------------ the initiator
def test_initiator_to_model_follower(backend):
    """The chip initiator sends <Give Device Power Status> (0x8F) to a model
    TV at address 0: the line carries exactly the header 0x40 and 0x8F with
    EOM in the last block, every timing inside the specification's
    windows, both blocks ACKed by the model; the report is ACKED, block 1,
    no retransmission; the frame starts at least 7 bit periods after the
    initiator first saw the line idle."""
    us = us_of(FAST)
    tv = C.CecNode(us, address=0)
    bench, loom, bus = setup(backend, FAST, [tv])
    message(loom, 0, [0x8F])
    assert initiator_report(loom, FAST, bits=40) == (R_ACK, 1, 0)
    frames = complete_frames(bus)
    assert len(frames) == 1
    f = frames[0]
    assert f.complete() and f.data == C.blocks_of(CHIP_INIT, 0, [0x8F])
    assert [b[2] for b in f.blocks] == [0, 0]           # ACKed: line low
    assert len(tv.ack_falls) == 2
    assert_timing(bus, us)
    assert f.start >= C.SFT_NEXT * bit_clocks(FAST)
    assert loom.badop() == 0


def test_nack_and_retry(backend):
    """A polling message (header only) to address 3, where nothing
    answers: the initiator sends it 6 times, each NACKed (the ACK bit reads
    1), every retransmission at least 3 bit periods after the end of the
    attempt before (and under 3.25: it waits no longer than it must), then
    reports NACKED in block 0 after 5 retransmissions."""
    us = us_of(FAST)
    bench, loom, bus = setup(backend, FAST, [C.CecNode(us, address=0)])
    message(loom, 3, [])
    assert initiator_report(loom, FAST, bits=150) == (R_NACK, 0, C.MAX_RETRIES)
    frames = complete_frames(bus)
    assert len(frames) == 1 + C.MAX_RETRIES
    for f in frames:
        assert f.data == [C.header(CHIP_INIT, 3)] and f.blocks[0][1:] == (1, 1)
    for a, b in zip(frames, frames[1:]):
        assert C.SFT_RETRY <= C.free_time(a, b, us) < C.SFT_RETRY + 0.25
    assert_timing(bus, us)


def test_arbitration_lost(backend):
    """A model initiator at address 1 starts its start bit in the same clock
    as the chip (address 4). They agree on the first header bit; in the
    second the chip sends a 1 and reads the model's 0: it stops driving and
    reports LOST in block 0. The line carries the model's frame, intact and
    ACKed by the model TV; the chip's pin made only the start bit and the
    first two bits' lows."""
    us = us_of(FAST)
    tv = C.CecNode(us, address=0)
    other = C.CecNode(us, address=1)
    s = other.send(0, [0x8F], with_chip=True)
    bench, loom, bus = setup(backend, FAST, [tv, other])
    message(loom, 0, [0x36])
    assert initiator_report(loom, FAST, bits=40) == (R_LOST, 0, 0)
    bench.step(int(25 * bit_clocks(FAST)))
    assert s.result == "ack" and s.starts[0] == complete_frames(bus)[0].start
    f = complete_frames(bus)[0]
    assert f.complete() and f.data == C.blocks_of(1, 0, [0x8F])
    assert len(bus.lows_of("BIDIR0")) == 3
    assert_timing(bus, us)


@model_only("the same transmit path as test_initiator_to_model_follower with the ACK sense inverted")
def test_initiator_broadcast(backend):
    """<Standby> (0x36) broadcast: no follower pulls the ACK bits low, which
    for a broadcast is the ACK; a model follower that rejects broadcasts
    makes every attempt a NACK and the initiator retransmits."""
    us = us_of(FAST)
    bench, loom, bus = setup(backend, FAST, [C.CecNode(us, address=0)])
    message(loom, C.BROADCAST, [0x36])
    assert initiator_report(loom, FAST, bits=40) == (R_ACK, 1, 0)
    f = complete_frames(bus)[0]
    assert f.data == C.blocks_of(CHIP_INIT, 15, [0x36]) and [b[2] for b in f.blocks] == [1, 1]
    assert_timing(bus, us)
    bench2, loom2, bus2 = setup(backend, FAST, [C.CecNode(us, address=0, nack_broadcast=True)])
    message(loom2, C.BROADCAST, [0x36])
    assert initiator_report(loom2, FAST, bits=200) == (R_NACK, 0, C.MAX_RETRIES)


@model_only("a longer frame through the same block loop as the RTL scenarios")
def test_initiator_long_message(backend):
    """<Set OSD Name> with 14 operand bytes, the longest frame (16 blocks):
    every byte on the line in order, EOM only in the last block."""
    us = us_of(FAST)
    tv = C.CecNode(us, address=0)
    data = [0x47] + [ord(c) for c in "Loom CEC test!"]
    bench, loom, bus = setup(backend, FAST, [tv])
    message(loom, 0, data)
    assert initiator_report(loom, FAST, bits=200) == (R_ACK, 15, 0)
    f = complete_frames(bus)[0]
    assert f.complete() and f.data == C.blocks_of(CHIP_INIT, 0, data)
    assert_timing(bus, us)


# ------------------------------------------------------------ the follower
def test_model_initiator_to_follower(backend):
    """A model player at address 4 sends <Active Source> 0x82 with its
    physical address 1.0.0.0 to the chip follower (0): the follower
    reports DIRECT with the four bytes; it held every ACK bit low (the
    model initiator saw all four ACKed), each ACK low on its own pin ends
    1.5 ms after the bit's fall (within a 0's window from the fall on the
    line)."""
    us = us_of(FAST)
    player = C.CecNode(us, address=4)
    bench, loom, bus = setup(backend, FAST, [player], initiator=False, follower=True)
    s = player.send(0, [0x82, 0x10, 0x00])
    status, data = follower_report(loom, FAST, bits=60)
    assert status == DIRECT, hex(status)
    assert data == C.blocks_of(4, 0, [0x82, 0x10, 0x00])
    assert s.result == "ack" and s.acks == [0, 0, 0, 0]
    f = complete_frames(bus)[0]
    acks = [f.bits[10 * k + 9] for k in range(4)]
    lows = bus.lows_of("BIDIR1")
    assert len(lows) == 4
    for bit, (fall, length) in zip(acks, lows):
        assert fall >= bit.fall
        end = (fall + length - bit.fall) / us
        assert C.ZERO_LOW_WIN[0] <= end <= C.ZERO_LOW_WIN[1], end
    assert_timing(bus, us)
    assert loom.badop() == 0


@model_only("the address filter is the same compare as the DIRECT case")
def test_follower_filters(backend):
    """A frame to another device (5) is followed and not reported nor
    ACKed; a broadcast is reported BCAST and not ACKed; a polling message
    to the follower is reported DIRECT with the header alone."""
    us = us_of(FAST)
    player = C.CecNode(us, address=4)
    bench, loom, bus = setup(backend, FAST, [player], initiator=False, follower=True)
    player.send(5, [0x8F])
    player.send(C.BROADCAST, [0x36])
    player.send(0, [])
    assert follower_report(loom, FAST, bits=120) == (BCAST, C.blocks_of(4, 15, [0x36]))
    assert follower_report(loom, FAST, bits=60) == (DIRECT, [C.header(4, 0)])
    assert len(bus.lows_of("BIDIR1")) == 1
    assert [s.result for s in player.sent] == ["nack", "ack", "ack"]


@model_only("error injection: three more frames than the RTL budget needs")
@pytest.mark.parametrize("fault, flag, blocks", [
    ({"short_at": 13}, SHORT, 1), ({"long_at": 12}, LONG, 1),
    ({"no_eom": True}, NOEOM, 3)])
def test_follower_errors(backend, fault, flag, blocks):
    """A model initiator breaks the frame: a bit period of 1.6 ms (SHORT),
    a low of 2.0 ms (LONG), or no EOM in any block (NOEOM). The follower
    reports the flag with the blocks it kept before the error, then
    receives the next, correct, frame."""
    us = us_of(FAST)
    player = C.CecNode(us, address=4)
    bench, loom, bus = setup(backend, FAST, [player], initiator=False, follower=True)
    player.send(0, [0x82, 0x10], **fault)
    player.send(0, [0x8F])
    status, data = follower_report(loom, FAST, bits=80)
    assert status & flag and status & ~(DIRECT | BCAST) == flag, hex(status)
    assert data == C.blocks_of(4, 0, [0x82, 0x10])[:blocks]
    assert follower_report(loom, FAST, bits=80) == (DIRECT, C.blocks_of(4, 0, [0x8F]))


# ------------------------------------------------------------ together
def test_initiator_and_follower_together(backend):
    """Thread 0's initiator (4) sends <Report Power Status> 0x90 0x00 to
    thread 1's follower (0) across the bench's wire: the initiator reports
    ACKED in block 2, the follower DIRECT with the three bytes, and the
    line's timing is inside every window."""
    us = us_of(FAST)
    bench, loom, bus = setup(backend, FAST, follower=True)
    message(loom, 0, [0x90, 0x00])
    assert initiator_report(loom, FAST, bits=50) == (R_ACK, 2, 0)
    assert follower_report(loom, FAST, bits=10) == (DIRECT, C.blocks_of(CHIP_INIT, 0, [0x90, 0x00]))
    assert_timing(bus, us)


@model_only("50 us ticks at 50 MHz: 2.3 million clocks")
def test_real_rate_polling_message(backend):
    """At the real rate (TICK_INT 2500, 50 MHz) the initiator polls address
    0 and the follower on thread 1 answers: both reports as at the test
    rate, and the line's times inside the specification's windows in
    microseconds."""
    us = us_of(REAL)
    assert us == 50.0
    bench, loom, bus = setup(backend, REAL, follower=True)
    message(loom, 0, [])
    assert initiator_report(loom, REAL, bits=30) == (R_ACK, 0, 0)
    assert follower_report(loom, REAL, bits=5) == (DIRECT, [C.header(CHIP_INIT, 0)])
    assert_timing(bus, us)
