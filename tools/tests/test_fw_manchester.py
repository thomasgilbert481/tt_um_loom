"""L3-MANCH on the model and the RTL: firmware/manchester_loopback.loom.

Thread 0 sends the frames the host pushes to INQ[0] as Manchester on OUT0;
thread 1 receives every frame on IN0 and reports it on OUTQ[1] as a status
word and the data words it announces (the program header has the formats).
``tools.protomodels.manchester.ManchesterLine`` is either a wire from OUT0 to
IN0 (the loopback) or carries a ``ManchesterNode``'s frames to IN0, and it
records OUT0 so that the transmitted frames are decoded independently of the
chip's own receiver.

Expected values come from the frame format in the program header (the sync
word 0xAAAB, 0 to 4 data words, the end marker, 16 bit times of idle between
frames, the status bits) and from SEMANTICS, never from what the model or the
RTL did. The transmitter's edges are held to a grid of one tick per half-bit
from the frame's first edge: exactly on it at TICK 12, which is a multiple of
the 4-clock slot, and within 3 clocks of it at TICK 25 (SEMANTICS 2: each edge
is 0 to 3 clocks late of its deadline).

Rates: TICK 12 (2.083 Mbit/s at 50 MHz, the fastest the deadline checker
proves) and TICK 25 (1 Mbit/s), one tick per half-bit in both threads.

Every body takes a ``backend`` (``tools/tests/fw_backend.py``): pytest runs
them on the golden model, ``test/test_fw.py`` on the RTL.
"""

import random

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels.manchester import (CLOCK, GAP_BITS, ManchesterLine,
                                           ManchesterNode, data_bit, decode)

ISA = load()
PROGRAM = REPO / "firmware" / "manchester_loopback.loom"
TICKS = [12, 25]                    # clocks per half-bit
VIOL, BAD_SYNC, TOO_LONG = 8, 16, 32   # status bits (program header)


def rate(tick):
    return CLOCK / (2 * tick)


def manchester_bench(backend):
    """Slice A (``BE``, ``BEENC``), the FIFOs and the 512-word memory; the RTL
    has all of them and ``RtlBench`` takes neither keyword."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "BE", "BEENC"), imem_words=512)
    return backend.bench()


def setup(backend, tick, node=None):
    """Load and run the program at ``tick`` clocks per half-bit, with the line
    a wire (no ``node``) or the node's, and let thread 1 see the line idle."""
    bench = manchester_bench(backend)
    line = bench.add(ManchesterLine(node))
    loom = backend.loom(bench, isa=ISA)
    loom.load(assemble_file(PROGRAM, isa=ISA, strict=True).words)
    for thread in (0, 1):
        loom.write_csr(thread, "TICK_INT", tick)
    loom.run([0, 1])
    bench.step(8 * 2 * tick)
    return bench, loom, line


def report(loom):
    """One frame from OUTQ[1]: ``(status, words)``."""
    status = loom.pop(1, 1)[0]
    return status, loom.pop(1, status & 7)


def on_grid(frame, tick):
    """Every edge of a transmitted frame on its half-bit grid."""
    tol = 0 if tick % 4 == 0 else 3
    assert len(frame.edges) >= 16, frame.edges
    off = [(j, o) for j, o in frame.edges if abs(o) > tol]
    assert not off, "edges off the grid (half-bit, clocks): %r" % off[:8]


def chip_sends(bench, loom, line, tick, frames):
    """The host has thread 0 send each word list of ``frames``, all pushed at
    once, and pops thread 1's report of each; returns the frames decoded on
    OUT0 and the reports. More than one frame only while every report fits in
    OUTQ[1] (four words): thread 1 does not look for the next frame until its
    report is pushed."""
    since = bench.cycle
    for words in frames:
        loom.push(0, [len(words)] + list(words))
    reports = [report(loom) for _ in frames]
    return decode(line.tx_log, tick, since=since), reports


def node_sends(bench, loom, node, words, **kw):
    """The node sends ``words`` (``violate=``); returns thread 1's report."""
    sent = node.send(words, **kw)
    budget = (len(words) + 4) * 32 * 2 * int(node.half + 1) + 20_000
    assert bench.run_until(lambda: sent.done, budget, every=64), (
        "the node's frame did not go out in %d clocks" % budget)
    return report(loom)


def patterns(rng):
    """Frames of every length, 0 to 4 words, with the four data patterns:
    all 0s, all 1s, alternating bits, random."""
    out = [[]]
    for n in range(1, 5):
        out += [[0x0000] * n, [0xFFFF] * n,
                [(0x5555, 0xAAAA)[i % 2] for i in range(n)],
                [rng.randrange(1 << 16) for _ in range(n)]]
    return out


# ------------------------------------------------------------- the program
def test_manchester_assembles_strict_with_no_diagnostic():
    """L2-DEADLINE for manchester_loopback.loom: every pair proved at TICK 12
    in both threads, nothing unbounded, no ``.bounded`` declaration. The
    worst slack is 0: the 6.9.1 loop's WAITD, SHx, BNZ interval is three slots
    against one 12-clock tick, and so is every interval of the unrolled
    word-boundary blocks, which carry one instruction each."""
    program = assemble_file(PROGRAM, isa=ISA, strict=True)
    assert program.diagnostics == []
    tx, rx = program.deadlines[0], program.deadlines[1]
    assert (tx.period, rx.period) == (12, 12)
    for rep in (tx, rx):
        assert not rep.infeasible and not rep.unbounded
        assert not rep.declarations
    assert (tx.worst_slack, rx.worst_slack) == (0, 0)
    assert sorted(program.threads) == [0, 1]
    assert max(a for a in program.words if a < 0x080) < 0x080
    assert max(program.words) < 0x100          # thread 1 stays in its quarter


# ---------------------------------------------------------------- loopback
@pytest.mark.parametrize("tick", TICKS)
def test_loopback_frames_of_every_length_and_pattern(backend, tick):
    """0 to 4 words, all 0s, all 1s, alternating and random, through the wire:
    the frame on OUT0 is well formed and on its grid, and thread 1 reports it
    clean with the same words."""
    rng = random.Random(tick)
    bench, loom, line = setup(backend, tick)
    for words in patterns(rng):
        frames, [(status, got)] = chip_sends(bench, loom, line, tick, [words])
        assert len(frames) == 1, frames
        f = frames[0]
        assert f.well_formed and f.words == words, (f, words)
        on_grid(f, tick)
        assert status == len(words) and got == words, (hex(status), got, words)


@pytest.mark.parametrize("tick", TICKS)
def test_loopback_back_to_back(backend, tick):
    """Two frames queued at once go out 16 bit times apart (the program
    header's gap), and thread 1 receives both."""
    bench, loom, line = setup(backend, tick)
    sent = [[0x0F0F], [0xC3A5]]
    frames, reports = chip_sends(bench, loom, line, tick, sent)
    assert [f.words for f in frames] == sent and all(f.well_formed for f in frames)
    last_bit_end = frames[0].start + (2 * frames[0].end - 1) * tick
    assert frames[1].start - last_bit_end >= (2 * GAP_BITS + 1) * tick
    assert reports == [(1, sent[0]), (1, sent[1])]


# ------------------------------------------------------------- node sends
OFFSETS = {12: (0.001, -0.001), 25: (0.002, -0.002)}


@pytest.mark.parametrize("tick,offset", [(t, o) for t in TICKS for o in OFFSETS[t]])
def test_node_frames_at_a_rate_offset(backend, tick, offset):
    """The node's own clock off by +-0.1 % (TICK 12) or +-0.2 % (TICK 25):
    thread 1 synchronises once, on the first edge, and still reads frames of
    1 to 4 words right to the end marker."""
    rng = random.Random(int(offset * 1e6) + tick)
    node = ManchesterNode(rate(tick), offset=offset)
    bench, loom, line = setup(backend, tick, node)
    for n in (1, 4, 2, 3):
        words = [rng.randrange(1 << 16) for _ in range(n)]
        assert node_sends(bench, loom, node, words) == (n, words)


@pytest.mark.parametrize("tick", TICKS)
def test_violation_is_reported_and_the_next_frame_is_clean(backend, tick):
    """A bit without its mid-bit transition: in the middle of word 2, at the
    first bit of word 1 (where the end marker would be; the edges that
    follow tell them apart) and in the sync word. Each is reported with the
    words received whole before it, and the next frame is received clean."""
    node = ManchesterNode(rate(tick))
    bench, loom, line = setup(backend, tick, node)
    words = [0x1234, 0xBEEF, 0x0F0F]
    cases = [(data_bit(2, 7), VIOL | 2, words[:2]),
             (data_bit(1, 0), VIOL | 1, words[:1]),
             (5, VIOL | BAD_SYNC, [])]
    for bit, status, got in cases:
        assert node_sends(bench, loom, node, words, violate=bit) == (status, got)
        assert node_sends(bench, loom, node, words) == (3, words)


def test_frame_too_long(backend):
    """Six data words where four is the most: reported as too long, with the
    last four; the next frame is clean."""
    node = ManchesterNode(rate(12))
    bench, loom, line = setup(backend, 12, node)
    words = [0x1111 * (i + 1) for i in range(6)]
    assert node_sends(bench, loom, node, words) == (TOO_LONG | 4, words[2:])
    assert node_sends(bench, loom, node, words[:2]) == (2, words[:2])
