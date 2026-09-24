"""L3-CAN on the model and the RTL: firmware/can_loopback.loom as a CAN 2.0A node.

Thread 0 sends the frames the host pushes to INQ[0]; thread 1 receives every
frame on the bus, the chip's own included, ACKs the good frames of other
nodes, and reports each frame on OUTQ[1] as a status word and the words it
announces (the program header has the formats). TX is OUT0 and RX is IN0;
``tools.protomodels.can.CanBus`` makes RX the wired AND of TX and the
``CanNode``'s driver, and the node receives, ACKs, sends and spoils frames.

Expected values come from ISO 11898-1 through the reference model (frame
layout, CRC15, stuffing, ACK, fixed-form fields) and from the program header
(word formats, status bits, which errors end a frame), never from what the
model or the RTL did. The node hard-synchronises on each SOF and records how
far every edge lies from its nominal bit boundary; a transmitter's edges must
fall within one time quantum of it, taken as a tenth of the bit (the
resynchronisation jump width of a ten-quantum bit time), or a receiver
without resynchronisation would drift out of the bit.

Rates: 500 kbit/s (thread 0 TICK 100, thread 1 TICK 12 + 128/256) and
125 kbit/s (TICK 400 and TICK 50), at 50 MHz.

Every body takes a ``backend`` (``tools/tests/fw_backend.py``): pytest runs
them on the golden model, ``test/test_fw.py`` on the RTL.
"""

import random

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels.can import CanBus, CanNode, Frame, IDLE_BITS, encode

ISA = load()
PROGRAM = REPO / "firmware" / "can_loopback.loom"
RATES = [500_000, 125_000]
TICKS = {500_000: ((100, 0), (12, 128)), 125_000: ((400, 0), (50, 0))}

# The status word (program header).
CRC_ERR, STUFF_ERR, FORM_ERR, NO_ACK, OWN, EXT = 1, 2, 4, 8, 16, 32
FRAME_BITS = 1 + 11 + 3 + 4 + 64 + 15      # SOF .. CRC, eight data bytes
MAX_BITS = FRAME_BITS + (FRAME_BITS - 1) // 4 + 10   # stuff bits, CRC delim .. EOF


def can_bench(backend):
    """Slice A (``BE``, ``BEENC``), the FIFOs, ``SETP ... D`` and the
    512-word memory; the RTL has all of them and ``RtlBench`` takes neither
    keyword."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "BE", "BEENC", "SETPD"),
                             imem_words=512)
    return backend.bench()


def setup(backend, rate=500_000, *, ack=True):
    """Load and run the program at ``rate`` with a node on the bus, and wait
    until both the node and thread 1 have seen the bus idle."""
    bench = can_bench(backend)
    node = CanNode(rate, ack=ack)
    bench.add(CanBus(node))
    loom = backend.loom(bench, isa=ISA)
    loom.load(assemble_file(PROGRAM, isa=ISA, strict=True).words)
    (tx_int, tx_frac), (rx_int, rx_frac) = TICKS[rate]
    loom.write_csr(0, "TICK_INT", tx_int)
    loom.write_csr(0, "TICK_FRAC", tx_frac)
    loom.write_csr(1, "TICK_INT", rx_int)
    loom.write_csr(1, "TICK_FRAC", rx_frac)
    loom.run([0, 1])
    bench.step((IDLE_BITS + 2) * node.period)
    return bench, loom, node


def report(loom):
    """One frame from OUTQ[1]: ``(status, words)``."""
    status = loom.pop(1, 1)[0]
    return status, loom.pop(1, (status >> 8) & 7)


def expected_words(frame):
    return [frame.header()] + frame.words()


def wait_frame(bench, node, count):
    """Run until the node has recorded ``count`` frames in all."""
    budget = 2 * (MAX_BITS + IDLE_BITS) * node.period + 20_000
    assert bench.run_until(lambda: len(node.frames) >= count, budget, every=64), \
        "no frame on the bus after %d clocks" % budget
    return node.frames[count - 1]


def chip_sends(bench, loom, node, frame):
    """The host has the chip send ``frame``: returns the node's record of it
    and the chip's own report of it (thread 1 reading it back)."""
    count = len(node.frames) + 1
    loom.push(0, expected_words(frame))
    rec = wait_frame(bench, node, count)
    return rec, report(loom)


def node_sends(bench, loom, node, frame, **spoil):
    """The node sends ``frame`` (``crc_error=`` / ``stuff_error=``): returns
    its :class:`Sent` record and the chip's report."""
    sent = node.send(frame, **spoil)
    budget = 2 * (MAX_BITS + IDLE_BITS) * node.period + 20_000
    assert bench.run_until(lambda: sent.done, budget, every=64), \
        "the node's frame did not go out in %d clocks" % budget
    return sent, report(loom)


def assert_on_time(rec, node):
    """Every edge of the frame within a tenth of a bit of its boundary."""
    tol = node.period // 10
    assert len(rec.edges) >= 4, rec.edges             # SOF, ACK, and some data
    late = [(bit, off) for bit, off in rec.edges if abs(off) > tol]
    assert not late, "edges off their bit boundary (bit, clocks): %r" % late[:8]


def good(frame, status, words, *, own):
    """The chip's report of a clean, ACKed frame."""
    n = len(expected_words(frame))
    assert status == ((OWN if own else 0) | (n << 8)), hex(status)
    assert words == expected_words(frame)


def random_frame(rng, length):
    return Frame(rng.randrange(1 << 11), bytes(rng.randrange(256) for _ in range(length)))


# ------------------------------------------------------------- the program
def test_can_loopback_assembles_strict_with_no_diagnostic():
    """L2-DEADLINE for can_loopback.loom: every pair proved at 500 kbit/s
    (thread 0 at 100 clocks a tick, thread 1 at 12, the 12.5 of TICK 12 +
    128/256 rounded down), nothing unbounded, no ``.bounded`` declaration.
    Thread 1's tightest intervals are the two around the ACK slot: 8 slots
    against 3 ticks from the CRC delimiter's sample to the slot's first tick,
    and 14 against 5 ticks from there to the slot's sample point."""
    program = assemble_file(PROGRAM, isa=ISA, strict=True)
    assert program.diagnostics == []
    tx, rx = program.deadlines[0], program.deadlines[1]
    assert (tx.period, rx.period) == (100, 12)
    for report in (tx, rx):
        assert not report.infeasible and not report.unbounded
        assert not report.declarations
    assert (tx.worst_slack, rx.worst_slack) == (28, 4)
    assert sorted(program.threads) == [0, 1]
    assert max(a for a in program.words if a < 0x080) < 0x080
    assert max(program.words) < 0x180          # nothing in thread 3's quarter


# ------------------------------------------------------------ chip sends
@pytest.mark.parametrize("rate", RATES)
def test_chip_sends_frames_of_every_length(backend, rate):
    """0 to 8 data bytes, random identifiers and data."""
    rng = random.Random(rate)
    bench, loom, node = setup(backend, rate)
    for length in range(9):
        frame = random_frame(rng, length)
        rec, (status, words) = chip_sends(bench, loom, node, frame)
        assert rec.clean and rec.frame == frame and rec.acked, rec
        assert_on_time(rec, node)
        good(frame, status, words, own=True)


def test_chip_sends_stuff_heavy_frames(backend):
    """Runs of 0s and 1s: the identifier, the data and the CRC all need
    stuff bits, and the node's destuffing must agree with the engine's."""
    frames = [Frame(0x000, b"\x00" * 8), Frame(0x7FF, b"\xFF" * 8),
              Frame(0x000, b"\xFF\x00\xFF\x00\xF8\x07\xC1\xF0"),
              Frame(0x7C1, b"\x0F\xF0"), Frame(0x07F, b"")]
    bench, loom, node = setup(backend)
    for frame in frames:
        rec, (status, words) = chip_sends(bench, loom, node, frame)
        assert rec.clean and rec.frame == frame and rec.acked, rec
        assert_on_time(rec, node)
        good(frame, status, words, own=True)


def test_chip_sends_remote_frames_and_long_dlc(backend):
    """A remote frame has no data field whatever its DLC; DLC 9..15 carries
    eight bytes and is sent and reported as given (ISO 11898-1)."""
    frames = [Frame(0x123, rtr=True, dlc=5), Frame(0x456, rtr=True, dlc=0),
              Frame(0x2AA, bytes(range(1, 9)), dlc=12)]
    bench, loom, node = setup(backend)
    for frame in frames:
        rec, (status, words) = chip_sends(bench, loom, node, frame)
        assert rec.clean and rec.frame == frame and rec.acked, rec
        good(frame, status, words, own=True)


def test_no_ack_when_the_node_is_silent(backend):
    """Nobody ACKs: the ACK slot stays recessive (the chip does not ACK its
    own frame), and thread 1 says so."""
    bench, loom, node = setup(backend, ack=False)
    frame = Frame(0x321, b"\xCA\xFE")
    rec, (status, words) = chip_sends(bench, loom, node, frame)
    assert rec.clean and not rec.acked, rec
    assert status == OWN | NO_ACK | (2 << 8), hex(status)
    assert words == expected_words(frame)


def test_form_errors_are_reported_and_the_next_frame_is_clean(backend):
    """The node drives a dominant bit into the chip's own frame, first on
    the ACK delimiter, then on the third EOF bit: each is a form error (ISO
    11898-1), reported with the header and data received; thread 1 then
    waits for the bus to go idle, and the next frame is clean."""
    bench, loom, node = setup(backend)
    frame = Frame(0x0A5, b"\x11\x22\x33")
    ack = encode(frame)[1]
    for bit in (ack + 1, ack + 4):              # ACK delimiter, EOF bit 3
        node.disturb(bit)
        rec, (status, words) = chip_sends(bench, loom, node, frame)
        assert rec.form_error and rec.acked, rec
        assert status == FORM_ERR | OWN | (3 << 8), hex(status)
        assert words == expected_words(frame)
    rec, (status, words) = chip_sends(bench, loom, node, frame)
    assert rec.clean and rec.acked, rec
    good(frame, status, words, own=True)


# ------------------------------------------------------------ node sends
@pytest.mark.parametrize("rate", RATES)
def test_chip_receives_and_acks_the_nodes_frames(backend, rate):
    rng = random.Random(rate + 1)
    lengths = (0, 1, 2, 5, 7, 8) if rate == 500_000 else (1, 8)
    bench, loom, node = setup(backend, rate)
    for length in lengths:
        frame = random_frame(rng, length)
        sent, (status, words) = node_sends(bench, loom, node, frame)
        assert sent.acked, "the chip did not ACK %r" % (frame,)
        good(frame, status, words, own=False)


def test_crc_error_is_reported_and_the_next_frame_is_clean(backend):
    """A CRC error: the frame is there in full, not ACKed, and flagged; the
    next frame is received and ACKed as usual."""
    bench, loom, node = setup(backend)
    frame = Frame(0x5A5, b"\x12\x34\x56")
    sent, (status, words) = node_sends(bench, loom, node, frame, crc_error=True)
    assert sent.acked is False
    assert status == CRC_ERR | NO_ACK | (3 << 8), hex(status)
    assert words == expected_words(frame)
    frame = Frame(0x5A6, b"\x9A\xBC")
    sent, (status, words) = node_sends(bench, loom, node, frame)
    assert sent.acked
    good(frame, status, words, own=False)


def test_stuff_error_is_reported_and_the_next_frame_is_clean(backend):
    """Six equal bits where a stuff bit belongs: the frame ends there with a
    stuff error and nothing after the status word; thread 1 waits for the
    bus to go idle and takes the next frame."""
    bench, loom, node = setup(backend)
    frame = Frame(0x100, b"\x00\x00\x42")         # stuff bits from the ID on
    sent, (status, words) = node_sends(bench, loom, node, frame, stuff_error=True)
    assert sent.acked is False
    assert status == STUFF_ERR and words == [], hex(status)
    frame = Frame(0x101, b"\x00\x00\x43")
    sent, (status, words) = node_sends(bench, loom, node, frame)
    assert sent.acked
    good(frame, status, words, own=False)
