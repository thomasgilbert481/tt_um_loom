"""L3 tests: XY2-100 galvo controller and receiver (D-035).

``firmware/xy2_ctrl.loom`` (threads 0 and 1: CLK OUT0, SYNC OUT1, X OUT2,
Y OUT3) takes X positions through ``INQ[0]`` and Y through ``INQ[1]`` and
is watched by :class:`tools.protomodels.xy2.Xy2Rx`.
``firmware/xy2_rx.loom`` (threads 2 and 3: CLK IN0, SYNC IN1, X IN2, Y
IN3) is driven by :class:`tools.protomodels.xy2.Xy2Tx` and reports the
positions through ``OUTQ[2]`` and ``OUTQ[3]``. Together, board wires join
OUT0..3 to IN0..3.

**Rate reached: the standard's 2 MHz, exactly on average.** The tick is
12 + 128/256 clocks, half a bit at 50 MHz; a frame is 40 ticks = 500
clocks = 10.000 us, so the frame rate is exactly 100 kHz and, 500 clocks
being 125 slots, every frame has the same edge pattern. Each CLOCK edge is
within a band of 3.5 clocks (70 ns) around one ideal 12.5-clock grid
(SEMANTICS 4 and 6.10 rule 2 below a 17-clock tick, as ``i2s_tx``;
measured: 3 clocks). The fastest tick, 12 clocks
(2.083 MHz), gives edges exactly periodic on the slot grid. Expected
values come from the sources cited in ``tools/protomodels/xy2.py`` and
from SEMANTICS 2 and 6.10, not from the firmware.
"""

from __future__ import annotations

import pytest

from tools.loomasm import assemble_file, assemble_text
from tools.loomisa import REPO, load
from tools.protomodels import xy2 as XY
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
IMEM = 512
FRAME = 20


class Board:
    """A tick for thread 0 of the controller (half a bit)."""

    def __init__(self, name, tick):
        self.name, self.tick = name, tick
        self.p = tick[0] + tick[1] / 256.0          # clocks per tick
        self.bit = 2 * self.p                       # clocks per bit
        self.frame = FRAME * self.bit               # clocks per frame

    def __repr__(self):
        return self.name


STD = Board("xy2_2mhz", (12, 128))       # 2 MHz: 25 clocks a bit, 500 a frame
FAST = Board("xy2_tick12", (12, 0))      # 2.083 MHz: 24 a bit, the fastest


def grid_band(board):
    """Clocks the CLOCK edges may spread around the ideal half-bit grid.
    A whole tick: 0 (SEMANTICS 2, constant lateness). A fractional tick
    below 17 clocks: each tick edge is on a whole clock, up to half a clock
    after the ideal time (SEMANTICS 4), and a staged edge lands on its tick
    or, when the WAITD of its half issued late, at that WAITD's commit,
    up to 3 clocks after the tick (SEMANTICS 6.10 rule 2: a half bit is
    3 slots, 12 clocks, against 12.5): a band of 3.5 clocks."""
    return 0.0 if board.tick[1] == 0 else 3.5


def program(name, thread=None):
    path = FIRMWARE / (name + ".loom")
    return assemble_file(path, isa=ISA, strict=True)


def make_bench(backend):
    """FIFOs, the deadline latch, the bit engine and the 512-word memory."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "SETPD", "BE", "BEENC", "DMEM"),
                             imem_words=IMEM)
    return backend.bench()


POSITIONS = [(0x0000, 0xFFFF), (0x8000, 0x7FFF), (0x1234, 0xEDCB),
             (0x5555, 0xAAAA), (0x0001, 0x8001), (0xFFFE, 0x0100)]


PRELOAD = 3                 # pairs queued before RUN (INQ holds 4)


# ------------------------------------------------------------ controller
def ctrl_setup(backend, board, first):
    """Load the controller and queue ``first`` (up to 3 pairs) before RUN:
    the host port is slower than a 10 us frame, so the test keeps the
    INQs ahead of the POPs."""
    bench = make_bench(backend)
    rx = bench.add(XY.Xy2Rx("OUT0", "OUT1", "OUT2", "OUT3"))
    loom = backend.loom(bench, isa=ISA)
    loom.load(program("xy2_ctrl"))
    loom.write_csr(0, "TICK_INT", board.tick[0])
    loom.write_csr(0, "TICK_FRAC", board.tick[1])
    loom.push(0, [x for x, _ in first])
    loom.push(1, [y for _, y in first])
    loom.run([0, 1])
    return bench, rx, loom


def feed(bench, rx, loom, positions, board, extra=1):
    """Queue one position per axis per frame, ahead of the POPs (bit 7 of
    the frame before), never more than three ahead; run until every
    position plus ``extra`` repeats has been received."""
    pushed = PRELOAD
    budget = int((len(positions) + extra + 3) * board.frame) + 4000
    start = bench.cycle
    while len(rx.frames) < len(positions) + extra:
        assert bench.cycle - start < budget, rx.report()
        if pushed < len(positions) and pushed - len(rx.frames) <= 3:
            x, y = positions[pushed]
            loom.push(0, [x], check=False)
            loom.push(1, [y], check=False)
            pushed += 1
        else:
            bench.step(64)
    return rx.frames


def check_clock(rx, board, skip=2):
    """Every CLOCK edge after the first ``skip`` bits within
    :func:`grid_band` of one ideal half-bit grid, and every frame exactly
    ``board.frame`` clocks after the one before (deadlines are absolute)."""
    r0 = rx.rises[skip]
    edges = sorted([r for r in rx.rises if r >= r0] + [f for f in rx.falls if f > r0])
    devs = [e - r0 - j * board.p for j, e in enumerate(edges)]
    assert max(devs) - min(devs) <= grid_band(board), (min(devs), max(devs))
    starts = [f.start for f in rx.frames[1:]]
    for a, b in zip(starts, starts[1:]):
        assert b - a == board.frame, (a, b)


@pytest.mark.parametrize("board", [STD, FAST])
def test_xy2_ctrl_frames_reach_the_driver(backend, board):
    """Six X/Y pairs in order, control bits 001 and even parity in every
    frame [L], SYNC low only in bit 20, the clock on its grid and every
    data change at least 2 clocks before and after a falling edge."""
    bench, rx, loom = ctrl_setup(backend, board, POSITIONS[:PRELOAD])
    frames = feed(bench, rx, loom, POSITIONS, board)
    got = [(f.x.pos, f.y.pos) for f in frames[:len(POSITIONS)]]
    assert got == POSITIONS
    for f in frames:
        assert f.x.ctrl == XY.CTRL_DEFAULT and f.y.ctrl == XY.CTRL_DEFAULT
        assert f.x.parity_ok and f.y.parity_ok
    assert rx.violations == [], rx.report()
    check_clock(rx, board)
    assert min(rx.setups) >= 2 and min(rx.holds) >= 12, (min(rx.setups), min(rx.holds))
    assert loom.badop() == 0


def test_xy2_ctrl_underrun_repeats_the_last_position(backend):
    """Two pairs, then nothing: the last pair is sent again in every later
    frame, the clock keeps its grid; a pair queued later comes out next."""
    board = STD
    bench, rx, loom = ctrl_setup(backend, board, POSITIONS[:2])
    assert bench.run_until(lambda: len(rx.frames) >= 5, int(7 * board.frame), every=64)
    got = [(f.x.pos, f.y.pos) for f in rx.frames]
    assert got[:2] == POSITIONS[:2] and set(got[2:]) == {POSITIONS[1]}
    loom.push(0, [0x4321], check=False)
    loom.push(1, [0x8765], check=False)
    n = len(rx.frames)
    assert bench.run_until(lambda: len(rx.frames) >= n + 3, int(5 * board.frame), every=64)
    got = [(f.x.pos, f.y.pos) for f in rx.frames]
    k = got.index((0x4321, 0x8765))
    assert set(got[2:k]) == {POSITIONS[1]} and set(got[k:]) == {(0x4321, 0x8765)}
    assert all(f.x.parity_ok and f.y.parity_ok for f in rx.frames)
    assert rx.violations == [], rx.report()
    check_clock(rx, board)


# ------------------------------------------------------------ receiver
RX_THREADS = (2, 3)
ERRP_X, ERRC_X, ERRS_X, ERRP_Y, ERRC_Y, ERRS_Y = 2, 3, 4, 5, 6, 7
HOLD = 3                    # frames each position is held (reports are on change)


def rx_setup(backend, frames, high=12, low=13):
    bench = make_bench(backend)
    tx = bench.add(XY.Xy2Tx("IN0", "IN1", "IN2", "IN3", frames,
                            high=high, low=low, start=1 << 40))
    loom = backend.loom(bench, isa=ISA)
    loom.load(program("xy2_rx"))
    loom.run(list(RX_THREADS))
    tx.start = bench.cycle + 100        # the sender starts after the load
    return bench, tx, loom


def collect(bench, loom, until_frames, tx, frame_clocks):
    """Pop OUTQ[2] and OUTQ[3] while the sender runs ``until_frames``."""
    xs, ys = [], []
    budget = int((until_frames + 2) * frame_clocks) + 2000
    start = bench.cycle
    while tx.sent < until_frames:
        assert bench.cycle - start < budget
        bench.step(int(frame_clocks))
        xs += loom.pop_available(2)
        ys += loom.pop_available(3)
    bench.step(int(frame_clocks))
    return xs + loom.pop_available(2), ys + loom.pop_available(3)


def held(pairs, n=HOLD):
    return [p for p in pairs for _ in range(n)]


RX_POS = [(0x8000, 0x8000), (0x1234, 0xFEDC), (0xFFFF, 0x0001), (0x5A5A, 0xA5A5)]


def test_xy2_rx_reports_each_new_position(backend):
    """A standard 2 MHz sender (12/13-clock halves, data on the rising
    edge [L]): frame 0 is used to lock on SYNC, then every new position
    of each axis reaches its OUTQ once, in order, with no error flag."""
    bench, tx, loom = rx_setup(backend, held(RX_POS))
    xs, ys = collect(bench, loom, len(RX_POS) * HOLD, tx, 25 * FRAME)
    assert xs == [x for x, _ in RX_POS]
    assert ys == [y for _, y in RX_POS]
    assert loom.sflags() & 0xFC == 0
    assert loom.badop() == 0


def test_xy2_rx_flags_a_parity_error(backend):
    """An X frame with its parity bit flipped sets ERRP of thread 2 and is
    not reported; Y in the same frame is reported; the next good frames
    are reported again."""
    bad = (XY.frame_bits(0x1111, bad_parity=True), XY.frame_bits(0x2222))
    frames = held([(0x8000, 0x8000)]) + [bad] + held([(0x3333, 0x4444)])
    bench, tx, loom = rx_setup(backend, frames)
    xs, ys = collect(bench, loom, len(frames), tx, 25 * FRAME)
    assert xs == [0x8000, 0x3333]
    assert ys == [0x8000, 0x2222, 0x4444]
    flags = loom.sflags()
    assert flags & (1 << ERRP_X) and not flags & (1 << ERRP_Y), hex(flags)
    assert not flags & ((1 << ERRC_X) | (1 << ERRS_X) | (1 << ERRC_Y) | (1 << ERRS_Y))


@model_only("framing loss repeats on the model what the RTL parity case proves of the error path")
def test_xy2_rx_flags_sync_lost_and_relocks(backend):
    """A frame whose SYNC stays low into bit 1 (SYNC must be high from bit
    0 to 18 [L]): the receiver, which checks it in bit 1, sets ERRS, drops
    the frame, locks again on the next SYNC low and reports the positions
    after it."""
    short = held([(0x8000, 0x8000)])
    frames = short + [(0x1111, 0x2222)] + held([(0x3333, 0x4444)])
    bench, tx, loom = rx_setup(backend, frames)
    # Bits 0 and 1 of frame HOLD carry SYNC low.
    sync = XY.sync_bits()
    orig = tx.drive

    def drive(d, cycle, _orig=orig):
        _orig(d, cycle)
        t = cycle - tx.start
        if t >= 0:
            bit = t // (tx.high + tx.low)
            n, k = divmod(bit, FRAME)
            if n == HOLD and k <= 1:
                d.set(tx.pads[1], 0)
    tx.drive = drive
    xs, ys = collect(bench, loom, len(frames), tx, 25 * FRAME)
    flags = loom.sflags()
    assert flags & (1 << ERRS_X) and flags & (1 << ERRS_Y), hex(flags)
    assert xs[0] == 0x8000 and xs[-1] == 0x3333 and 0x1111 not in xs
    assert ys[0] == 0x8000 and ys[-1] == 0x4444 and 0x2222 not in ys
    assert sync[1] == 1


@model_only("one control-bit pattern on the model; the check is the same code as parity")
def test_xy2_rx_flags_wrong_control_bits(backend):
    """Control bits 000 on Y (a status or extended-mode frame [O]) set
    ERRC of thread 3 and are not reported as a position."""
    odd = (XY.frame_bits(0x5555), XY.frame_bits(0x6666, ctrl=0))
    frames = held([(0x8000, 0x8000)]) + [odd] + held([(0x7777, 0x7777)])
    bench, tx, loom = rx_setup(backend, frames)
    xs, ys = collect(bench, loom, len(frames), tx, 25 * FRAME)
    assert xs == [0x8000, 0x5555, 0x7777] and ys == [0x8000, 0x7777]
    flags = loom.sflags()
    assert flags & (1 << ERRC_Y) and not flags & ((1 << ERRC_X) | (1 << ERRP_Y)), hex(flags)


# ------------------------------------------------------------ together
def test_xy2_ctrl_to_rx_together(backend):
    """xy2_ctrl on threads 0 and 1, wired OUT0..3 -> IN0..3 to xy2_rx on
    threads 2 and 3, at 2 MHz: each pair pushed (one at a time, the
    controller repeating it until the next) arrives at OUTQ[2] and
    OUTQ[3], with no error flag, and the wire meets the driver's checks."""
    board = STD
    bench = make_bench(backend)
    mon = bench.add(XY.Xy2Rx("OUT0", "OUT1", "OUT2", "OUT3"))
    for k in range(4):
        bench.add(XY.Wire("OUT%d" % k, "IN%d" % k))
    loom = backend.loom(bench, isa=ISA)
    words = dict(program("xy2_ctrl").words)
    rxw = program("xy2_rx").words
    assert not set(words) & set(rxw)
    words.update(rxw)
    loom.load(words)
    loom.write_csr(0, "TICK_INT", board.tick[0])
    loom.write_csr(0, "TICK_FRAC", board.tick[1])
    pairs = [(0x8000, 0x8000), (0x0123, 0xFEDC), (0xFFFF, 0x0000), (0x2468, 0x1357)]
    loom.push(0, [pairs[0][0]])
    loom.push(1, [pairs[0][1]])
    loom.run([0, 1, 2, 3])
    xs, ys = [], []
    for i, (x, y) in enumerate(pairs):
        if i:
            loom.push(0, [x], check=False)
            loom.push(1, [y], check=False)
        assert bench.run_until(lambda: (xs.extend(loom.pop_available(2)) or True)
                               and (ys.extend(loom.pop_available(3)) or True)
                               and len(xs) > i and len(ys) > i,
                               int(8 * board.frame), every=int(board.frame))
    assert xs == [x for x, _ in pairs] and ys == [y for _, y in pairs]
    assert loom.sflags() & 0xFC == 0
    assert mon.violations == [], mon.report()
    assert all(f.x.parity_ok and f.y.parity_ok for f in mon.frames)
    check_clock(mon, board)
    assert loom.badop() == 0
