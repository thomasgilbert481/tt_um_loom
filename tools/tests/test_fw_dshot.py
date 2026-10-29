"""L3-DSHOT: DShot600/300/150 and bidirectional DShot, on the model and the RTL.

``firmware/dshot_tx.loom`` (normal DShot on OUT0) and
``firmware/dshot_bidir.loom`` (bidirectional DShot on BIDIR0, with the
eRPM telemetry reply) are assembled, loaded and fed through
``tools.loomhost.Loom``. ``tools.protomodels.dshot.DshotRx`` measures every
pulse and period on the pad against the windows of the cited sources
before it decodes a bit, and ``DshotEsc`` answers bidirectional frames with
GCR telemetry. Every body that takes a ``backend`` runs here on the golden
model and under cocotb on the RTL (``test/test_fw.py``).

The expected numbers come from the sources the model cites and from
``docs/SEMANTICS.md``, never from what the model or the RTL happened to do:
a bit is 20 ticks, high (low, bidirectional) for 7 for a 0 and 14 for a 1
(Betaflight MOTOR_BIT_0/1, MOTOR_BITLENGTH); the host sets the tick to
1/20 of a bit; and SEMANTICS 6.10 puts each deadline-latched edge on the
exact clock edge at which its tick lands (SEMANTICS 4 places the ticks).
So on a 48 MHz board, where DShot600's tick is exactly 4 clocks, every
pulse is exactly 28 or 56 clocks and every period exactly 80; on a 50 MHz
board (4 + 43/256 clocks a tick) every edge is within one clock of the
ideal tick grid and the tick counts between edges are exactly 7, 14 and 20.
"""

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels import dshot as D
from tools.protomodels.bench import Lines
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
IMEM = 512
BIT, T0H, T1H = 20, 7, 14                     # ticks (Betaflight dshot_dpwm.h)


class Board:
    """A board clock, a DShot rate and the tick the program header gives."""

    def __init__(self, name, clock_mhz, rate, tick):
        self.name, self.clock_mhz, self.rate, self.tick = name, clock_mhz, rate, tick
        self.p = tick[0] + tick[1] / 256.0        # clocks per tick (SEMANTICS 4)
        self.bit = BIT * self.p                   # clocks per bit, as driven

    def cycles(self, us):
        return int(round(us * self.clock_mhz))

    def __repr__(self):
        return self.name


B600 = Board("600_50MHz", 50.0, 600000, (4, 43))
B300 = Board("300_50MHz", 50.0, 300000, (8, 85))
B150 = Board("150_50MHz", 50.0, 150000, (16, 171))
B600_48 = Board("600_48MHz", 48.0, 600000, (4, 0))
B300_48 = Board("300_48MHz", 48.0, 300000, (8, 0))


def program(name):
    return assemble_file(FIRMWARE / (name + ".loom"), isa=ISA, strict=True)


def make_bench(backend, pullups=0):
    """FIFOs, the deadline latch, the bit engine with its encoders, data
    memory and the 512-word memory; the RTL has them all."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "SETPD", "BE", "BEENC", "DMEM"),
                             imem_words=IMEM, pullups=pullups)
    return backend.bench(pullups=pullups)


def boot(backend, bench, name, board):
    loom = backend.loom(bench, isa=ISA)
    loom.load(program(name))
    loom.write_csr(0, "TICK_INT", board.tick[0])
    loom.write_csr(0, "TICK_FRAC", board.tick[1])
    loom.run(0)
    return loom


def tx_setup(backend, board, strict=False):
    bench = make_bench(backend)
    rx = bench.add(D.DshotRx("OUT0", clock_mhz=board.clock_mhz, rate=board.rate,
                             strict=strict))
    return bench, rx, boot(backend, bench, "dshot_tx", board)


def bidir_setup(backend, board, **esc):
    bench = make_bench(backend, pullups=0b1)
    model = bench.add(D.DshotEsc("BIDIR0", clock_mhz=board.clock_mhz,
                                 rate=board.rate, **esc))
    return bench, model, boot(backend, bench, "dshot_bidir", board)


def word(value, telemetry=False):
    """A frame word for the host: the packet, as Betaflight packs it."""
    return D.packet_of(value, telemetry)


def interval(bits):
    return 0x8000 | bits


def run_frames(bench, rx, n, board, budget_bits=None):
    """Step until ``rx`` has ``n`` frames (or fail with its report)."""
    budget = int((budget_bits or 200 * n) * board.bit) + 4000
    assert bench.run_until(lambda: len(rx.frames) >= n, budget, every=32), \
        "%d of %d frames after %d clocks: %s" % (len(rx.frames), n, budget,
                                                  rx.report())
    return rx.frames


# ------------------------------------------------------------ the timing
def expected_ticks(frame):
    """The tick offset of every edge of a frame from its first, from the
    frame's bits and the source's 20 / 7 / 14."""
    out = []
    for j in range(16):
        bit = (frame.frame >> (15 - j)) & 1
        out += [BIT * j, BIT * j + (T1H if bit else T0H)]
    return out


def on_grid(clocks, ticks, board):
    """SEMANTICS 4 and 6.10: an edge ``ticks`` ticks after another lands
    within one clock of ``ticks * p`` (ticks fall on clock edges), and
    exactly on it when ``p`` is a whole number of clocks."""
    ideal = ticks * board.p
    if board.tick[1] == 0:
        return clocks == ideal
    return abs(clocks - ideal) < 1


def check_frames(rx, board):
    """Every window of the model, then every edge on its exact tick."""
    assert rx.violations == [], rx.report()
    for f in rx.frames:
        want = expected_ticks(f)
        assert len(f.edges) == len(want), (f, want)
        for edge, ticks in zip(f.edges, want):
            assert on_grid(edge - f.edges[0], ticks, board), \
                "frame 0x%04X: edge %d clocks after the first, %d ticks = %g" \
                % (f.frame, edge - f.edges[0], ticks, ticks * board.p)
        assert f.crc_ok


def check_starts(frames, bits, board):
    """Frame starts exactly ``bits`` bit periods apart, on the tick grid."""
    for a, b in zip(frames, frames[1:]):
        assert on_grid(b.start - a.start, bits * BIT, board), \
            (b.start - a.start, bits * BIT * board.p)


# ---------------------------------------------------------- DShot600 TX
#: (value, telemetry): disarm, special commands, both ends of the throttle,
#: the handbook's worked example (1046), every telemetry bit both ways.
VALUES = [(0, False), (1, True), (47, True), (48, False), (1046, False),
          (1046, True), (1500, False), (2047, True)]


@pytest.mark.parametrize("board", [B600_48, B600])
def test_tx_one_shot_frames_across_the_range(backend, board):
    """Interval 0: each frame word sends one frame; every edge checked."""
    bench, rx, loom = tx_setup(backend, board)
    loom.push(0, [word(v, t) for v, t in VALUES])
    frames = run_frames(bench, rx, len(VALUES), board)
    assert [(f.value, f.telemetry) for f in frames] == VALUES
    assert [f.frame for f in frames] == [D.encode_frame(v, t) for v, t in VALUES]
    check_frames(rx, board)
    if board.tick[1] == 0:                       # 48 MHz: whole clocks
        assert set(h for f in frames for h in f.highs) == {28, 56}
        assert set(p for f in frames for p in f.periods) == {80}
    bench.step(int(40 * board.bit))
    assert len(rx.frames) == len(VALUES)         # one-shot: nothing repeats
    assert bench.lines.uo & 1 == 0               # and the line idles low
    assert loom.badop() == 0


@pytest.mark.parametrize("board", [B600_48])
def test_tx_repeat_stream_keeps_the_interval(backend, board):
    """``0x8000 | I`` then a frame: the frame repeats every I bits exactly,
    the telemetry bit only on the first; a new word replaces it at the next
    start; ``0x8000`` stops the stream."""
    i = 40
    bench, rx, loom = tx_setup(backend, board)
    loom.push(0, [interval(i), word(1000, True)])
    run_frames(bench, rx, 4, board, budget_bits=6 * i)
    loom.push(0, [word(1200)])
    run_frames(bench, rx, 7, board, budget_bits=4 * i)
    loom.push(0, [interval(0)])
    bench.step(int(3 * i * board.bit))
    n = len(rx.frames)
    bench.step(int(3 * i * board.bit))
    assert len(rx.frames) == n                   # stopped
    frames = rx.frames
    check_frames(rx, board)
    check_starts(frames, i, board)               # one unbroken grid
    assert frames[0].value == 1000 and frames[0].telemetry
    assert all(not f.telemetry for f in frames[1:])
    values = [f.value for f in frames]
    k = values.index(1200)
    assert 4 <= k <= 5 and set(values[:k]) == {1000} and set(values[k:]) == {1200}
    assert loom.badop() == 0


def test_tx_short_interval_is_the_shortest(backend):
    """An interval below 19 bits is taken as 19: a gap of three idle bits
    after the 16th period, more than Betaflight's two (frame reset)."""
    board = B600_48
    bench, rx, loom = tx_setup(backend, board)
    loom.push(0, [interval(5), word(300)])
    run_frames(bench, rx, 4, board)
    check_frames(rx, board)
    check_starts(rx.frames, 19, board)
    assert loom.badop() == 0


@pytest.mark.parametrize("board", [B300, B150, B300_48])
def test_tx_dshot300_and_dshot150_are_the_tick(backend, board):
    """The same program at another tick: the frame and a 30-bit stream."""
    bench, rx, loom = tx_setup(backend, board)
    loom.push(0, [interval(30), word(1046, True)])
    run_frames(bench, rx, 3, board, budget_bits=120)
    check_frames(rx, board)
    check_starts(rx.frames, 30, board)
    assert [f.frame for f in rx.frames] == [D.encode_frame(1046, True)] + \
        [D.encode_frame(1046)] * 2


# ------------------------------------------------------ bidirectional
NO_REPLY, BAD_GCR, BAD_SUM = 0x8001, 0x8002, 0x8003
#: Telemetry values: stopped (0xFFF), eRPM periods with exponents 0 .. 3,
#: an Extended DShot Telemetry temperature frame (type 0x2, 50 degrees).
TLM = [0x0FFF, D.erpm_value(300), D.erpm_value(1234), D.erpm_value(3000),
       0x0232]


def bidir_frames(backend, board, values, faults=(), **esc):
    """One-shot: one frame word per entry of ``values``; returns the bench,
    the ESC model and the words the program pushed, one per frame."""
    bench, esc_model, loom = bidir_setup(backend, board, values=values,
                                         faults=faults, **esc)
    n = len(values)
    loom.push(0, [word(1000 + 10 * k, k == 0) for k in range(n)])
    out = []
    for k in range(n):
        assert bench.run_until(lambda: loom.fifo_status(0)["outq"] > 0,
                               int(150 * board.bit) + 4000, every=256), \
            "no telemetry word for frame %d: %s" % (k, esc_model.rx.report())
        out += loom.pop_available(0)
    return bench, esc_model, loom, out


def check_bidir(bench, esc_model, board):
    """The FC's inverted frames inside every window and on the tick grid,
    and the line never driven both ways."""
    check_frames(esc_model.rx, board)
    assert all(f.bidir for f in esc_model.frames)
    assert bench.contention_free(), bench.contentions[:4]


@pytest.mark.parametrize("board", [B600])
def test_bidir_telemetry_is_decoded(backend, board):
    """Inverted frame, inverted checksum, turnaround, GCR reply decoded."""
    bench, esc_model, loom, out = bidir_frames(backend, board, TLM)
    assert out == TLM
    check_bidir(bench, esc_model, board)
    assert [f.frame for f in esc_model.frames] == \
        [D.encode_frame(1000 + 10 * k, k == 0, bidir=True) for k in range(len(TLM))]
    assert [r.word for r in esc_model.replies] == [D.telemetry_word(v) for v in TLM]
    assert loom.badop() == 0


@pytest.mark.parametrize("bad_code", [0x00, 0x1C])
def test_bidir_reply_errors(backend, bad_code):
    """A bad checksum, a code GCR never uses (below the table and in it),
    no reply: each reported, and the next frame decodes again."""
    board = B600_48
    values = [0x0123, 0x0456, 0x0789, 0x0ABC, 0x0DEF]
    bench, esc_model, loom, out = bidir_frames(
        backend, board, values, faults=["", "crc", "gcr", "none", ""],
        bad_code=bad_code)
    assert out == [0x0123, BAD_SUM, BAD_GCR, NO_REPLY, 0x0DEF]
    check_bidir(bench, esc_model, board)


@pytest.mark.parametrize("board", [B600_48])
def test_bidir_stream_keeps_its_grid_across_the_replies(backend, board):
    """A 100-bit stream (6 kHz at DShot600): frames exactly I bits apart
    while every reply is received in between, one word per frame."""
    i = 100
    bench, esc_model, loom = bidir_setup(backend, board, values=TLM)
    loom.push(0, [interval(i), word(500, True)])
    out = []
    while len(esc_model.frames) < 5:
        assert bench.run_until(lambda: loom.fifo_status(0)["outq"] > 0,
                               int(2 * i * board.bit), every=256)
        out += loom.pop_available(0)
    bench.step(int(60 * board.bit))
    out += loom.pop_available(0)
    check_bidir(bench, esc_model, board)
    check_starts(esc_model.frames, i, board)
    assert out[:5] == TLM
    assert esc_model.frames[0].telemetry
    assert not any(f.telemetry for f in esc_model.frames[1:])


@model_only("the delay and clock-error sweep; the nominal reply runs on the RTL")
@pytest.mark.parametrize("delay_us, skew", [(20.0, 0.0), (40.0, 0.0),
                                            (30.0, -0.01), (30.0, 0.02)])
def test_bidir_reply_delay_and_esc_clock_error(backend, delay_us, skew):
    """The reply 20 to 40 us after the frame, the ESC's clock -1 % to +2 %."""
    board = B600
    bench, esc_model, loom, out = bidir_frames(
        backend, board, TLM, delay_us=delay_us, skew=skew)
    assert out == TLM
    check_bidir(bench, esc_model, board)


def test_bidir_dshot300(backend):
    """DShot300: the tick doubled, the reply at 375 kbit/s."""
    board = B300
    bench, esc_model, loom, out = bidir_frames(backend, board, TLM[:3])
    assert out == TLM[:3]
    check_bidir(bench, esc_model, board)


# ------------------------------------------------------------ the programs
@pytest.mark.parametrize("name, words, pairs, bounded", [
    ("dshot_tx", 65, 15, ["POP"]),
    ("dshot_bidir", 154, 17, ["POP", "PUSH"]),
])
def test_assembles_strict_with_no_diagnostic(name, words, pairs, bounded):
    """L2-DEADLINE: every pair proved at the declared 4-clock tick, nothing
    unbounded; the declarations are the guarded FIFO accesses only."""
    prog = program(name)
    assert prog.diagnostics == []
    report = prog.deadlines[0]
    assert report.period == 4
    assert not report.infeasible and not report.unbounded
    assert len(report.pairs) == pairs
    assert report.worst_slack == 8                   # two slots
    assert [d.name for d in report.declarations] == bounded
    assert all("WAITB above completed with T = 0" in d.reason
               for d in report.declarations)
    assert list(prog.threads) == [0]
    assert len(prog.words) == words


def test_tx_fits_thread_0_and_bidir_spills_into_thread_1():
    """D-020: thread 0 owns 0..127. dshot_tx fits; dshot_bidir runs to
    0x099 and says thread 1 must not be started with its image."""
    assert max(program("dshot_tx").words) < 128
    bidir = program("dshot_bidir")
    assert max(bidir.words) == 0x099
    assert 1 in bidir.unstartable and 2 not in bidir.unstartable


# ------------------------------------------------------ the model itself
def test_checksum_matches_the_handbook_example():
    """HANDBOOK "Calculating the CRC": value 1046, no telemetry, gives
    1000001011000110; bidirectional, 1000001011001001."""
    assert D.encode_frame(1046) == 0b1000001011000110
    assert D.encode_frame(1046, bidir=True) == 0b1000001011001001
    assert D.frame_ok(D.encode_frame(48, True)) and not D.frame_ok(0x0001)


def test_gcr_tables_and_line_code_round_trip():
    """Every nibble's code is in Betaflight's decode table, the 21 line
    levels start with a 0, and every word survives the round trip."""
    assert sorted(D.GCR_DECODE) == [0x09, 0x0A, 0x0B, 0x0D, 0x0E, 0x0F, 0x12,
                                    0x13, 0x15, 0x16, 0x17, 0x19, 0x1A, 0x1B,
                                    0x1D, 0x1E]
    for value in (0, 0x0FFF, 0x0123, 0x0ABC):
        word = D.telemetry_word(value)
        assert D.telemetry_ok(word) and not D.telemetry_ok(word ^ 1)
        levels = D.line_levels(D.gcr_bits(word))
        assert len(levels) == 21 and levels[0] == 0
        assert D.decode_levels(levels) == word
    # Betaflight's eRPM value: period = mantissa << exponent
    v = D.erpm_value(3000)
    assert (v & 0x1FF) << (v >> 9) == 3000


def feed(rx, pattern, pad_bit=0):
    """Drive ``(level, clocks)`` through a decoder on ``uo[0]``."""
    lines = Lines()
    cycle = 0
    for level, clocks in pattern:
        for _ in range(clocks):
            lines.cycle = cycle
            lines.uo = level << pad_bit
            rx.observe(lines)
            cycle += 1


def wire(frame, high=None, period=80, idle=400):
    """A frame on a 48 MHz DShot600 line (80 clocks a bit), active high."""
    out = [(0, idle)]
    for j in range(16):
        h = high if high is not None else (56 if (frame >> (15 - j)) & 1 else 28)
        out += [(1, h), (0, period - h)]
    return out + [(0, idle)]


def test_decoder_accepts_a_clean_frame():
    rx = D.DshotRx("OUT0", clock_mhz=48.0)
    feed(rx, wire(D.encode_frame(1046, True)))
    assert rx.violations == [] and [f.value for f in rx.frames] == [1046]


@pytest.mark.parametrize("high", [22, 42, 66])
def test_decoder_rejects_a_pulse_outside_both_windows(high):
    """0.275, 0.525 and 0.825 of a bit: short of T0H, between, past T1H."""
    rx = D.DshotRx("OUT0", clock_mhz=48.0)
    feed(rx, wire(0xFFFF, high=high))
    assert any(v.kind == "high" for v in rx.violations), rx.report()


@pytest.mark.parametrize("period", [74, 86])
def test_decoder_rejects_a_period_outside_its_window(period):
    """7.5 % short and long, against +-5 % (one Betaflight timer unit)."""
    rx = D.DshotRx("OUT0", clock_mhz=48.0)
    feed(rx, wire(D.encode_frame(300), period=period))
    assert any(v.kind == "period" for v in rx.violations), rx.report()


def test_decoder_rejects_a_bad_checksum_and_a_short_gap():
    rx = D.DshotRx("OUT0", clock_mhz=48.0)
    good = D.encode_frame(300)
    feed(rx, wire(good ^ 1)[:-1] + [(0, 100)] + wire(good, idle=0)[1:])
    kinds = [v.kind for v in rx.violations]
    assert "crc" in kinds and "gap" in kinds, rx.report()


# ----------------------------------------- the same bodies, other boards
# The RTL runs each of these at one board clock; the other runs here only.
# Together the RTL scenarios already put the transmitter at both clocks
# (one-shot), the bidirectional frames and replies at both (decoded at 50
# MHz, errors and the stream at 48 MHz), so these add no new hardware path.
@model_only("the 48 MHz stream runs on the RTL (exact edges); the 50 MHz one-shot covers the fractional tick there")
def test_tx_repeat_stream_at_50mhz(backend):
    test_tx_repeat_stream_keeps_the_interval(backend, B600)


@model_only("the 50 MHz decode runs on the RTL; 48 MHz bidirectional edges run there in the error and stream scenarios")
def test_bidir_telemetry_at_48mhz(backend):
    test_bidir_telemetry_is_decoded(backend, B600_48)


@model_only("the 48 MHz stream runs on the RTL; the 50 MHz reply runs there in the decode scenario")
def test_bidir_stream_at_50mhz(backend):
    test_bidir_stream_keeps_its_grid_across_the_replies(backend, B600)
