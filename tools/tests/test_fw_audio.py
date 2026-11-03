"""L3-AUDIO: I2S controller transmitter and PDM microphone receiver.

``firmware/i2s_tx.loom`` (SCK OUT0, SD OUT1, WS OUT2) is fed 16-bit samples
through ``INQ[0]`` and watched by ``tools.protomodels.i2s.I2sRx``, which
checks every SCK period, high and low time and every SD and WS change
against Table 3 of the I2S bus specification (NXP UM11732) before it frames
words by the WS rule of its section 3.2. ``firmware/pdm_rx.loom`` clocks
``tools.protomodels.i2s.PdmMic`` (CLK OUT3, DAT IN0; an MP34DT01-M with
L/R = GND) and its words are compared with the reference CIC of the bits
the microphone actually sent, then fitted to the tone. Every body that
takes a ``backend`` runs here on the golden model and under cocotb on the
RTL (``test/test_fw.py``).

Expected numbers come from the specifications and ``docs/SEMANTICS.md``:
one tick is half an SCK (CLK) period, so an SCK period is ``2p`` clocks for
a tick of ``p``; with ``p`` a whole number of slots the edges are exactly
periodic (SEMANTICS 2), and with a fractional ``p`` and slack in the
interval each staged edge lands on the clock edge at which its tick lands
(SEMANTICS 4 and 6.10), within one clock of the ideal.
"""

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels import i2s as I
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
IMEM = 512
CLOCK_HZ = 50e6


class Board:
    """A tick for the I2S program (half an SCK period)."""

    def __init__(self, name, tick):
        self.name, self.tick = name, tick
        self.p = tick[0] + tick[1] / 256.0          # clocks per tick
        self.period = 2 * self.p                    # clocks per SCK period
        self.fs = CLOCK_HZ / (32 * self.period)     # 16-bit stereo frames/s

    def __repr__(self):
        return self.name


FASTEST = Board("tick12", (12, 0))       # SCK 2.083 MHz, fs 65.1 kHz
FS48K = Board("fs48k", (16, 71))         # fs 47.998 kHz
FS44K = Board("fs44k1", (17, 183))       # fs 44.100 kHz


def grid_tol(board):
    """How far a rising edge may sit from the ideal grid, in clocks. Whole
    ticks: 0 (SEMANTICS 2, constant lateness). A fractional tick of 17
    clocks or more: under 1, each staged edge on its own tick (SEMANTICS
    6.10 rule 1). Below 17 the WAITD of an interval can issue in the X
    cycle of the tick itself when the WAITD before it completed 3 clocks
    late (12 clocks of code after a completion up to 3 clocks after the
    tick), and then rule 2 lands the edge at its commit, one clock after
    the tick edge: under 2."""
    if board.tick[1] == 0:
        return 0.0
    return 1.0 if board.p >= 17 else 2.0


def program(name):
    return assemble_file(FIRMWARE / (name + ".loom"), isa=ISA, strict=True)


def make_bench(backend):
    """FIFOs, the deadline latch, the bit engine, data memory and the
    512-word memory; the RTL has them all."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "SETPD", "BE", "BEENC", "DMEM"),
                             imem_words=IMEM)
    return backend.bench()


# ------------------------------------------------------------------ I2S
def i2s_setup(backend, board):
    bench = make_bench(backend)
    rx = bench.add(I.I2sRx("OUT0", "OUT2", "OUT1", period=board.period,
                           period_tol=grid_tol(board)))
    loom = backend.loom(bench, isa=ISA)
    loom.load(program("i2s_tx"))
    loom.write_csr(0, "TICK_INT", board.tick[0])
    loom.write_csr(0, "TICK_FRAC", board.tick[1])
    loom.run(0)
    return bench, rx, loom


def stream(bench, rx, loom, words, board, extra_words=2):
    """Feed ``words`` to INQ as the program consumes them, two per host
    transaction, never more than the INQ holds: a decoded word means its
    successor has been popped (the POP is in bit 12 of the word before),
    so at most ``pushed - decoded - 1`` words are queued. Then run until
    every word, plus ``extra_words`` more, has been decoded."""
    pushed = 0
    word_clocks = 16 * board.period
    budget = int((len(words) + extra_words + 4) * word_clocks) + 6000
    start = bench.cycle
    while len(rx.words) < len(words) + extra_words:
        assert bench.cycle - start < budget, rx.report()
        queued = pushed - len(rx.words) - 1
        if pushed < len(words) and queued <= 2:
            loom.push(0, words[pushed:pushed + 2], check=False)
            pushed += len(words[pushed:pushed + 2])
        else:
            bench.step(32)
    return rx.words


def s16(v):
    return v - 0x10000 if v & 0x8000 else v


#: Both ends of the range, one LSB either side of 0, and patterns whose
#: every bit position toggles.
SAMPLES = [0x7FFF, 0x8000, 0x0001, 0xFFFF, 0x0000, 0x5555, 0xAAAA, 0x1234,
           0x8001, 0x7FFE, 0x00FF, 0xFF00]


def check_timing(rx, board):
    """Table 3 (the model's windows), then the clock grid: every rising
    edge within one clock of ``r0 + k * 2p`` (exactly on it for a whole
    tick), so SCK never pauses, and every high and low time ``p``."""
    assert rx.violations == [], rx.report()
    rises = rx.rises[1:]            # the first is SCK going to its idle level
    r0 = rises[0]
    for k, r in enumerate(rises):
        ideal = k * board.period
        if board.tick[1] == 0:
            assert r - r0 == ideal, (k, r - r0, ideal)
        else:
            assert abs(r - r0 - ideal) < grid_tol(board), (k, r - r0, ideal)
    tol = max(grid_tol(board), 1.0) if board.tick[1] else 0.0
    for h, lo in zip(rx.highs[1:], rx.lows[1:]):
        assert abs(h - board.p) <= tol and abs(lo - board.p) <= tol, (h, lo)
    assert max(rx.sd_delays) <= I.DELAY_MAX * board.period
    assert min(rx.sd_delays + rx.ws_delays) >= 1


@pytest.mark.parametrize("board", [FASTEST, FS48K])
def test_i2s_frames_are_decoded_and_timed(backend, board):
    """Twelve words: the preamble is not a word, then left/right frames in
    the order pushed, every SCK edge on its grid and inside Table 3."""
    bench, rx, loom = i2s_setup(backend, board)
    words = stream(bench, rx, loom, SAMPLES, board, extra_words=0)
    assert [w.bits for w in words] == [16] * len(SAMPLES)
    assert [w.channel for w in words] == [0, 1] * (len(SAMPLES) // 2)
    assert [w.value() for w in words] == [s16(v) for v in SAMPLES]
    assert rx.frames == [(s16(a), s16(b)) for a, b in
                         zip(SAMPLES[0::2], SAMPLES[1::2])]
    check_timing(rx, board)
    # 3.2: one WS change per word, so WS changed exactly once per word
    # (the preamble's change included) and the words are back to back.
    assert len(rx.ws_delays) in (len(words), len(words) + 1)
    for a, b in zip(words, words[1:]):
        assert abs(b.start - a.start - 16 * board.period) < max(1.0, grid_tol(board))
    assert loom.badop() == 0


def test_i2s_frame_rate_is_the_tick(backend):
    """At the 48 kHz tick, word starts are 16 SCK periods apart within one
    clock, so the frame rate is 50 MHz / (64 * 16.277) = 47.998 kHz."""
    board = FS48K
    bench, rx, loom = i2s_setup(backend, board)
    words = stream(bench, rx, loom, SAMPLES[:8], board, extra_words=0)
    starts = [w.start for w in words]
    for k, s in enumerate(starts):
        assert abs(s - starts[0] - k * 16 * board.period) < grid_tol(board)
    fs = CLOCK_HZ * (len(starts) - 1) / 2 / (starts[-1] - starts[0])
    assert abs(fs - 47997.6) < 30, fs
    check_timing(rx, board)


@model_only("a third rate repeats what the two RTL rates prove")
def test_i2s_at_44k1(backend):
    board = FS44K
    bench, rx, loom = i2s_setup(backend, board)
    words = stream(bench, rx, loom, SAMPLES[:4], board, extra_words=0)
    assert [w.value() for w in words] == [s16(v) for v in SAMPLES[:4]]
    check_timing(rx, board)


def test_i2s_underrun_sends_zero_and_keeps_the_clock(backend):
    """Two frames, then nothing: every later word is 0 and SCK keeps its
    grid; a word pushed again comes out whole, in order, after zeros."""
    board = FASTEST
    bench, rx, loom = i2s_setup(backend, board)
    first = [0x1111, 0x2222, 0x3333, 0x4444]
    stream(bench, rx, loom, first, board, extra_words=4)
    assert [w.value() for w in rx.words[:4]] == [s16(v) for v in first]
    assert [w.raw for w in rx.words[4:]] == [0] * (len(rx.words) - 4)
    loom.push(0, [0x7ABC, 0x0DEF], check=False)
    n = len(rx.words)
    assert bench.run_until(lambda: len(rx.words) >= n + 5,
                           int(8 * 16 * board.period), every=64)
    raws = [w.raw for w in rx.words]
    k = raws.index(0x7ABC, 4)
    assert raws[k + 1] == 0x0DEF
    assert set(raws[4:k]) == {0} and set(raws[k + 2:]) == {0}
    check_timing(rx, board)
    assert loom.badop() == 0


# ------------------------------------------------------------------ PDM
R = 64                                    # CIC decimation (pdm_rx.loom)
PDM_BIT = 32                              # clocks per PDM bit: tick 16 x 2
FS_OUT = CLOCK_HZ / (PDM_BIT * R)         # 24414.0625 Hz
FULL = R * R                              # CIC2 gain: 4096


def pdm_setup(backend, **mic):
    bench = make_bench(backend)
    m = bench.add(I.PdmMic("OUT3", "IN0", bit_rate_hz=CLOCK_HZ / PDM_BIT, **mic))
    loom = backend.loom(bench, isa=ISA)
    loom.load(program("pdm_rx"))
    loom.run(0)
    return bench, m, loom


def collect(bench, loom, n, poll=4 * PDM_BIT * R // 2):
    """Pop ``n`` words, polling OUTQ about every two output periods."""
    out = []
    budget = (n + 4) * PDM_BIT * R + 8000
    start = bench.cycle
    while len(out) < n:
        assert bench.cycle - start < budget, "%d of %d words" % (len(out), n)
        bench.step(poll)
        out += loom.pop_available(0)
    return out[:n]


def align(words, bits):
    """The block phase at which the reference CIC of ``bits`` equals the
    program's words, skipping the two filling the filter. Returns the
    phase and the matching reference, or fails."""
    got = words[2:]
    for phase in range(R):
        ref = I.cic(bits, R, 2, phase)
        for off in range(0, 4):
            seg = ref[off:off + len(got)]
            if len(seg) == len(got) and seg == got:
                return phase, seg
    raise AssertionError("no block phase makes the reference CIC match: %r"
                         % (got[:6],))


def check_clock(mic):
    """MP34DT01-M Table 5 / Table 3 (the model's windows), then exactly
    32 clocks a period and 16 high: tick 16 is four slots (SEMANTICS 2)."""
    assert mic.violations == [], mic.violations[:3]
    rises = mic.rises[1:]
    assert {b - a for a, b in zip(rises, rises[1:])} == {PDM_BIT}


def test_pdm_tone_is_recovered(backend):
    """A 1 kHz tone at half scale through a second-order modulator: every
    word equals the reference CIC2 (R = 64) of the bits the microphone
    sent, and the fitted tone is within 2 % of the input amplitude times
    the CIC's droop at 1 kHz (sinc^2: 0.9945), DC within 0.5 % of scale."""
    bench, mic, loom = pdm_setup(backend, tone_hz=1000.0, amplitude=0.5)
    words = collect(bench, loom, 30)
    check_clock(mic)
    phase, ref = align(words, mic.bits)
    x = [(w - FULL / 2) / (FULL / 2) for w in words[2:]]
    amp, dc, rms = I.fit_tone(x, 1000.0, FS_OUT)
    droop = (I.math.sin(I.math.pi * 1000 / FS_OUT) /
             (R * I.math.sin(I.math.pi * 1000 / (FS_OUT * R)))) ** 2
    assert abs(amp - 0.5 * droop) < 0.02 * 0.5, (amp, dc, rms)
    assert abs(dc) < 0.005, dc
    assert rms < 0.02, rms
    assert loom.badop() == 0


def test_pdm_silence_is_midscale(backend):
    """No tone: the words sit at 2048 (half the bits are ones) within the
    modulator's idle noise, and still match the reference exactly."""
    bench, mic, loom = pdm_setup(backend, amplitude=0.0)
    words = collect(bench, loom, 8)
    check_clock(mic)
    align(words, mic.bits)
    assert all(abs(w - FULL // 2) <= 64 for w in words[2:]), words


@model_only("waits out a full OUTQ: about 40 000 clocks with nothing new on the pins")
def test_pdm_full_outq_drops_words_and_keeps_the_filter(backend):
    """Nobody pops for ten output periods: the four oldest words wait in
    OUTQ, the rest are dropped, and the words after that still equal the
    reference CIC at their own block (the filter state was kept)."""
    bench, mic, loom = pdm_setup(backend, tone_hz=700.0, amplitude=0.4)
    bench.step(12 * PDM_BIT * R)
    held = loom.pop_available(0)
    assert len(held) == 4
    later = collect(bench, loom, 6)
    # held[0:2] are the filter filling; find the block phase of held[2:4]
    # and of the later words in the one reference.
    ref = j = k = None
    for phase in range(R):
        cand = I.cic(mic.bits, R, 2, phase)
        js = [i for i in range(len(cand)) if cand[i:i + 2] == held[2:4]]
        ks = [i for i in range(len(cand)) if cand[i:i + len(later)] == later]
        if js and ks:
            ref, j, k = cand, js[0], ks[0]
            break
    assert ref is not None, (held, later)
    assert k > j + 2                     # words in between were dropped
    assert loom.badop() == 0
