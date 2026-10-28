"""L3-HDLC on the model and the RTL: firmware/hdlc_loopback.loom (D-035).

Thread 0 sends the frames the host pushes to INQ[0] as HDLC (ISO/IEC 13239)
on OUT0, NRZI coded, with idle flags between frames; thread 1 receives every
frame on IN0 and reports it on OUTQ[1] as a status word and the octets it
announces (the program header has the formats). ``tools.protomodels.hdlc``'s
``HdlcLine`` is either a wire from OUT0 to IN0 (the loopback) or carries an
``HdlcNode``'s line to IN0, and it records OUT0, so the transmitted frames
are decoded by the reference model independently of the chip's receiver.

Expected values come from ISO/IEC 13239 and RFC 1662 through the reference
model (FCS-16, zero-bit insertion, flags, abort, NRZI) and from the program
header (word formats, status bits, N = 127), never from what the golden model
or the RTL did. The transmitter's edges are held to a grid of one bit time
from the frame's first edge: exactly on it when the bit time is a multiple of
the 4-clock slot (SEMANTICS 2), within 3 clocks otherwise.

Rates: 329 kbit/s (thread 0 TICK 152, thread 1 TICK 19: the fastest the
deadline checker proves for thread 1, eight ticks a bit) and 64 kbit/s
(TICK 781 + 64/256 and TICK 97 + 168/256), at 50 MHz.

Every body takes a ``backend`` (``tools/tests/fw_backend.py``): pytest runs
them on the golden model, ``test/test_fw.py`` on the RTL. RTL time is about
6,400 clocks a second, so the RTL runs short frames at the fast rate; the
64 kbit/s frames, the long frames and the clock-offset sweep are
``model_only``.
"""

import random

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels import hdlc as H
from tools.tests.fw_backend import model_only

ISA = load()
PROGRAM = REPO / "firmware" / "hdlc_loopback.loom"
N_MAX = 127                                 # information octets (program header)

#: rate name -> (bit time in clocks, thread 0 TICK_INT/FRAC, thread 1 TICK_INT/FRAC)
RATES = {"329k": (152.0, (152, 0), (19, 0)),
         "64k": (781.25, (781, 64), (97, 168))}

# The status word (program header).
BAD_FCS, ABORT, TOO_LONG, NOT_OCTETS = 0x8000, 0x4000, 0x2000, 0x1000


def hdlc_bench(backend):
    """Slice A (``BE``, ``BEENC``), slice B (``DMEM``), the FIFOs and the
    512-word memory; the RTL has all of them and ``RtlBench`` takes neither
    keyword."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "BE", "BEENC", "DMEM"),
                             imem_words=512)
    return backend.bench()


def setup(backend, rate="329k", node=None):
    """Load and run the program at ``rate`` with the line a wire (no
    ``node``) or the node's, and let both threads run through a few idle
    flags (thread 1 needs one whole flag to start)."""
    bit, (t0_int, t0_frac), (t1_int, t1_frac) = RATES[rate]
    bench = hdlc_bench(backend)
    line = bench.add(H.HdlcLine(node))
    loom = backend.loom(bench, isa=ISA, max_polls=3000)
    # No read-back: it would double the load of the 277-word image (35 k
    # clocks a scenario on the RTL); the host-library tests check that path.
    loom.load(assemble_file(PROGRAM, isa=ISA, strict=True).words, verify=False)
    loom.write_csr(0, "TICK_INT", t0_int)
    loom.write_csr(0, "TICK_FRAC", t0_frac)
    loom.write_csr(1, "TICK_INT", t1_int)
    loom.write_csr(1, "TICK_FRAC", t1_frac)
    loom.run([0, 1])
    bench.step(int(40 * bit))
    return bench, loom, line


def command(address, control, info=b"", abort=False):
    """The INQ[0] words for one frame (program header)."""
    info = bytes(info)
    words = [len(info) | (0x8000 if abort else 0), (control << 8) | address]
    for i in range(0, len(info), 2):
        words.append(info[i] | ((info[i + 1] << 8) if i + 1 < len(info) else 0))
    return words


def report(loom):
    """One report from OUTQ[1]: ``(status, octets)``."""
    status = loom.pop(1, 1)[0]
    m = status & 0x1FF
    words = loom.pop(1, (m + 1) // 2)
    octets = bytearray()
    for w in words:
        octets += bytes([w & 0xFF, w >> 8])
    if m % 2:
        assert octets[-1] == 0, "15:8 of an odd last word must be 0: %r" % words
    return status, bytes(octets[:m])


def raw_between_flags(bits, frame):
    """The bits on the line between the frame's opening flag and its closing
    flag, stuff bits included (``frame`` from ``H.deframe(bits)``)."""
    return bits[frame.start:frame.end - 7]


def on_grid(offsets, bit):
    """Every edge of the transmitter on its bit grid (SEMANTICS 2)."""
    tol = 0 if bit % 4 == 0 else 4
    off = [o for o in offsets if abs(o) > tol]
    assert not off, "edges off the bit grid (clocks): %r" % off[:8]


def chip_sends(bench, loom, line, rate, address, control, info=b"", abort=False):
    """The host has thread 0 send one frame and pops thread 1's report of
    it. Returns the frames the reference model finds on OUT0 (with the bits
    it recovered), and the report."""
    bit = RATES[rate][0]
    since = bench.cycle
    loom.push(0, command(address, control, info, abort))
    rep = report(loom)
    bench.step(int(12 * bit))                    # the closing or abort bits
    bits, offsets, _ = H.recover(line.tx_log, bit, since, bench.cycle)
    on_grid(offsets, bit)
    return bits, H.deframe(bits), rep


def node_sends(bench, node, stream):
    """The node puts ``stream`` (bits, flags included) on the line."""
    sent = node.send(stream)
    budget = int((len(stream) + 40) * node.period) + 20_000
    assert bench.run_until(lambda: sent.done, budget, every=64), (
        "the node's stream did not go out in %d clocks" % budget)
    return sent


def stuff_heavy(rng):
    """Information fields that need many stuff bits or contain the flag."""
    return [b"\xFF" * 9, b"\x7E" * 5, b"\xF8\x1F\xF0\x0F\x3F\xFC",
            b"\x7E\xFF\x7E", bytes(rng.randrange(256) | 0x81 for _ in range(7))]


# ------------------------------------------------------------- the program
def test_hdlc_assembles_strict_with_no_diagnostic():
    """L2-DEADLINE for hdlc_loopback.loom: every pair proved, thread 0 at 64
    clocks a tick (781 kbit/s, one tick a bit) and thread 1 at 19 (eight
    ticks a bit, 152 clocks: 329 kbit/s), nothing unbounded. Thread 0's two
    POPs are declared bounded: each follows a WAITB INQ_NE, T that completed
    by condition, and only thread 0 pops INQ[0]. The worst slack is 0 in both
    threads: thread 0's FCS step (16 slots in one tick) and thread 1's word
    store (14 slots in the 3 ticks from a sample to the next edge window)."""
    program = assemble_file(PROGRAM, isa=ISA, strict=True)
    assert program.diagnostics == []
    tx, rx = program.deadlines[0], program.deadlines[1]
    assert (tx.period, rx.period) == (64, 19)
    for rep in (tx, rx):
        assert rep.pairs and not rep.infeasible and not rep.unbounded
    assert (tx.worst_slack, rx.worst_slack) == (0, 0)
    assert len(tx.declarations) == 2 and not rx.declarations
    assert sorted(program.threads) == [0, 1]
    t0 = [a for a in program.words if a < 0x080]
    t1 = [a for a in program.words if a >= 0x080]
    assert len(t0) == program.threads[0].size and max(t0) < 0x080
    # thread 1: its quarter, and the report block between the buffers
    assert all(0x080 <= a < 0x100 or 0x141 <= a < 0x180 for a in t1)


def test_fcs16_check_value():
    """RFC 1662's FCS-16 (CRC-16/X-25) of "123456789" is 0x906E, and a good
    frame leaves the residue 0xF0B8: the reference the tests rely on."""
    assert H.fcs16(H.CHECK_INPUT) ^ 0xFFFF == H.CHECK_FCS
    assert H.fcs_octets(H.CHECK_INPUT) == bytes([0x6E, 0x90])
    assert H.frame_ok(H.CHECK_INPUT + H.fcs_octets(H.CHECK_INPUT))


# ---------------------------------------------------------------- loopback
def loop_frames(bench, loom, line, rate, frames):
    """Each ``(address, control, info)`` goes out on OUT0 and comes back on
    IN0: the reference model finds it on OUT0, good, with every stuff bit
    where ISO/IEC 13239 puts it, and thread 1 reports it good with the same
    octets."""
    for address, control, info in frames:
        data = bytes([address, control]) + bytes(info)
        bits, found, (status, octets) = chip_sends(bench, loom, line, rate,
                                                   address, control, info)
        assert len(found) == 1, found
        f = found[0]
        assert f.good and f.data == data, (f, data)
        assert raw_between_flags(bits, f) == H.frame_bits(address, control, info)[0]
        assert status == len(data) and octets == data, (hex(status), octets, data)


def test_loopback_fcs_check_vector(backend):
    """The frame "123456789" (address '1', control '2', information
    "3456789") carries the FCS octets 0x6E 0x90 on the line: RFC 1662's check
    value 0x906E, low octet first; thread 1 finds it good. One frame at 329
    kbit/s, about 40 k clocks: the PHY-GL-SDF candidate."""
    bench, loom, line = setup(backend)
    bits, found, (status, octets) = chip_sends(bench, loom, line, "329k",
                                               0x31, 0x32, b"3456789")
    assert [f.octets for f in found] == [b"123456789" + bytes([0x6E, 0x90])]
    assert raw_between_flags(bits, found[0]) == H.frame_bits(0x31, 0x32, b"3456789")[0]
    assert status == 9 and octets == b"123456789"


def test_loopback_short_and_stuff_heavy_frames(backend):
    """No information, nine 0xFF (a stuff bit after every five 1s, across
    the octet boundaries), five 0x7E (the flag inside the data), runs of 1s
    across octet boundaries, and flags around 0xFF, at 329 kbit/s."""
    rng = random.Random(5)
    bench, loom, line = setup(backend)
    frames = [(0xFF, 0x03, b"")] + [(0x01, 0x10 + i, info)
                                    for i, info in enumerate(stuff_heavy(rng)[:4])]
    loop_frames(bench, loom, line, "329k", frames)


@model_only("random frames of every length up to N: about 3 M clocks")
def test_loopback_random_lengths(backend):
    """Random address, control and information of random lengths, N = 127
    included, and random stuff-heavy fields, at 329 kbit/s."""
    rng = random.Random(11)
    bench, loom, line = setup(backend)
    lengths = [N_MAX, 1, 2, 3, 64] + [rng.randrange(N_MAX + 1) for _ in range(6)]
    frames = [(rng.randrange(256), rng.randrange(256),
               bytes(rng.randrange(256) for _ in range(n))) for n in lengths]
    frames += [(0xFF, 0xFF, info) for info in stuff_heavy(rng)]
    loop_frames(bench, loom, line, "329k", frames)


@model_only("64 kbit/s is 781 clocks a bit: about 2 M clocks for these frames")
def test_loopback_at_64k(backend):
    """The same at 64 kbit/s, with a fractional tick in both threads."""
    rng = random.Random(64)
    bench, loom, line = setup(backend, "64k")
    frames = [(0x03, 0x3F, b""), (0xFF, 0x13, bytes(rng.randrange(256) for _ in range(20)))]
    frames += [(0x7E, 0xFF, info) for info in stuff_heavy(rng)[:3]]
    loop_frames(bench, loom, line, "64k", frames)


# ------------------------------------------------------------- node sends
def iso_reports(stream):
    """What thread 1 must report for ``stream``, from the reference model's
    receiver (``H.deframe``) and the status format of the program header:
    an abort as ABORT, a frame that is not whole octets as NOT_OCTETS, and
    otherwise the octets before the FCS, with BAD_FCS when it is wrong.
    Frames under 32 bits (idle flags) are not frames and give no report."""
    out = []
    for f in H.deframe(stream):
        if f.aborted:
            out.append((ABORT, b""))
        elif not f.whole:
            out.append((NOT_OCTETS, b""))
        else:
            out.append(((0 if f.good else BAD_FCS) | len(f.data), f.data))
    return out


def node_frames(bench, loom, node, streams):
    """The node sends each stream; thread 1's reports are those an ISO/IEC
    13239 receiver gives for it (:func:`iso_reports`)."""
    for stream in streams:
        node_sends(bench, node, stream)
        for want in iso_reports(H.FLAG_BITS + stream + H.FLAG_BITS):
            assert report(loom) == want, want


def not_octets(address, control, info, extra):
    """A frame with ``extra`` bits after the FCS: not a whole number of
    octets between the flags."""
    octets = bytes([address, control]) + bytes(info)
    body = H.octet_bits(octets + H.fcs_octets(octets)) + list(extra)
    return H.FLAG_BITS + H.stuff(body)[0] + H.FLAG_BITS


def test_node_frames_with_errors(backend):
    """From the node at 329 kbit/s: a good frame; one bit flipped (a bad
    FCS, the octets still delivered); an abort after the information; a
    missing stuff bit (six 1s and then a 1: an abort as the receiver sees
    it); three bits too many before the closing flag (NOT_OCTETS); and a
    good frame again."""
    node = H.HdlcNode(H.CLOCK / RATES["329k"][0])
    bench, loom, line = setup(backend, node=node)
    good = H.frame_stream(0x05, 0x10, b"\x12\x34\x56")
    flipped = H.frame_stream(0x05, 0x10, b"\x12\x34\x56", flip=20)
    aborted = H.frame_stream(0x05, 0x10, b"\x9A", abort=True)
    skipped = H.frame_stream(0x05, 0x10, b"\xFF\xFF\x0F", skip_stuff=0)
    ragged = not_octets(0x01, 0x02, b"\x33\x44", [1, 0, 1])
    assert iso_reports(H.FLAG_BITS + flipped + H.FLAG_BITS)[0][0] & BAD_FCS
    assert iso_reports(H.FLAG_BITS + skipped + H.FLAG_BITS)[0][0] == ABORT
    assert iso_reports(H.FLAG_BITS + ragged + H.FLAG_BITS) == [(NOT_OCTETS, b"")]
    node_frames(bench, loom, node, [good, flipped, aborted, skipped, ragged, good])


def test_chip_abort_then_a_good_frame(backend):
    """ABORT in the command: the frame goes out to the end of its
    information and then eight 1s, no FCS (ISO/IEC 13239: seven or more 1s);
    the reference model sees an aborted frame, thread 1 reports ABORT, and
    the next frame is good."""
    bench, loom, line = setup(backend)
    bits, found, rep = chip_sends(bench, loom, line, "329k", 0x0F, 0xF0,
                                  b"\x01\x02\x03", abort=True)
    assert [f.aborted for f in found] == [True]
    sent = H.frame_bits(0x0F, 0xF0, b"\x01\x02\x03", abort=True)[0] + [1] * 8
    start = found[0].start
    assert bits[start:start + len(sent)] == sent
    assert rep == (ABORT, b"")
    loop_frames(bench, loom, line, "329k", [(0x0F, 0xF0, b"\x04")])


@model_only("a 260-octet frame is 2,200 bits: 340 k clocks")
def test_node_frame_too_long(backend):
    """More than 256 octets between the flags: TOO_LONG as soon as the
    buffer is full, and the next frame is good (thread 1 hunts for a flag)."""
    node = H.HdlcNode(H.CLOCK / RATES["329k"][0])
    bench, loom, line = setup(backend, node=node)
    long_frame = H.frame_stream(0x01, 0x02, bytes(range(256)) + b"\x00\x01")
    node_sends(bench, node, long_frame)
    assert report(loom) == (TOO_LONG, b"")
    node_frames(bench, loom, node, [H.frame_stream(0x01, 0x02, b"\x66\x77")])


@model_only("idle marks and 64 kbit/s: about 1 M clocks")
def test_node_idle_marks_at_64k(backend):
    """A sender that idles with marks (the line still) instead of flags, at
    64 kbit/s: each frame's opening flag is found by the hunt."""
    rng = random.Random(3)
    node = H.HdlcNode(H.CLOCK / RATES["64k"][0], idle="marks")
    bench, loom, line = setup(backend, "64k", node=node)
    streams = [H.frame_stream(rng.randrange(256), rng.randrange(256),
                              bytes(rng.randrange(256) for _ in range(n)))
               for n in (0, 5, 16)]
    node_frames(bench, loom, node, streams)


# ------------------------------------------------------- clock offset
# Measured on the golden model (three frames, 12 random octets and eight
# 0xFF): thread 1 receives a sender whose bit rate is off by -4 % .. +1.75 %
# at 329 kbit/s (fails at -5 % and +2 %) and by -3 % .. +3 % at 64 kbit/s
# (fails at -4 % and +4 %). A faster sender (positive offset) is the tighter
# side at 329 kbit/s: the edge window opens 3 ticks after the sample, and the
# detection latency (the edge seen in a slot, then the TICK_INT write, about
# 13 clocks) is 9 % of a 152-clock bit; at 64 kbit/s it is under 2 %.
# The tests below stay inside those limits.
def test_node_frames_at_a_clock_offset(backend):
    """The node's clock 1 % fast at 329 kbit/s (the tighter side): thread 1
    follows it, re-phasing on every transition, through a stuff-heavy
    frame. The slow side and the limits are in the model-only test below."""
    node = H.HdlcNode(H.CLOCK / RATES["329k"][0], offset=0.01)
    bench, loom, line = setup(backend, node=node)
    node_frames(bench, loom, node, [H.frame_stream(0xFF, 0x3F, b"\xFF\x7E\x81")])


@model_only("the offset sweep: about 2 M clocks")
@pytest.mark.parametrize("rate,offset", [("329k", 0.015), ("329k", -0.03),
                                         ("64k", 0.025), ("64k", -0.025)])
def test_node_frames_at_the_offset_limits(backend, rate, offset):
    """Near the measured limits: +1.5 % and -3 % at 329 kbit/s, +-2.5 % at
    64 kbit/s, with random and stuff-heavy frames."""
    rng = random.Random(int(offset * 1000) + len(rate))
    node = H.HdlcNode(H.CLOCK / RATES[rate][0], offset=offset)
    bench, loom, line = setup(backend, rate, node=node)
    streams = [H.frame_stream(rng.randrange(256), rng.randrange(256),
                              bytes(rng.randrange(256) for _ in range(12))),
               H.frame_stream(0xFF, 0xFF, b"\xFF" * 8)]
    node_frames(bench, loom, node, streams)


def test_loopback_back_to_back(backend):
    """Two frames queued at once go out with one idle flag between the
    closing flag and the next opening flag (the program header), and thread
    1, which pushes the first report and hunts again inside that idle flag,
    receives both."""
    bench, loom, line = setup(backend)
    since = bench.cycle
    first, second = (0x01, 0x02, b"\x11\x22"), (0x03, 0x04, b"\x33\x44\x55")
    loom.push(0, command(*first) + command(*second))
    reports = [report(loom), report(loom)]
    bench.step(int(12 * RATES["329k"][0]))
    bits, offsets, _ = H.recover(line.tx_log, RATES["329k"][0], since, bench.cycle)
    on_grid(offsets, RATES["329k"][0])
    found = H.deframe(bits)
    assert [f.data for f in found] == [bytes([a, c]) + i for a, c, i in (first, second)]
    assert all(f.good for f in found)
    # closing flag, one idle flag, opening flag: 16 bits after the closing 0
    assert found[1].start - found[0].end == 1 + 2 * 8
    assert reports == [(4, b"\x01\x02\x11\x22"), (5, b"\x03\x04\x33\x44\x55")]
