"""L3 for the SMPTE LTC programs, on the model and the RTL.

``firmware/ltc_gen.loom`` (TX = OUT0) sends linear timecode, biphase mark,
from a time the host gives and counts the frames itself;
``firmware/ltc_rx.loom`` (RX = IN0, DIR = OUT0) decodes biphase mark by edge
spacing, finds the sync word in either direction, follows the tape's speed
and reports every whole frame. The reference is ``tools.protomodels.ltc``,
written from the public descriptions of SMPTE ST 12-1: ``frame_bits``
encodes a time, ``bits_from_edges`` holds a recorded pin to the half-bit
grid, ``LtcSource`` plays bits at any speed and either way, ``Wire`` joins
the two programs when they run together (generator thread 0, reader thread
1, its DIR moved to OUT1).

Expected values come from the frame layout and SEMANTICS: a deadline-latched
edge lands on the exact clock edge of its tick (6.10), so with a whole
number of clocks a half-bit every generator edge is exactly on the grid
``start + n * half`` and a frame is exactly 160 half-bits.

Scaled time. LTC at 50 MHz is 10,417 clocks a half-bit at 30 fps, 1.67
million clocks a frame. The RTL and most model scenarios run the same
programs on a board whose half-bit is 256 clocks (the generator's schedule
is proved down to 184, the reader stamps every edge on time down to 216):
only the clocks a tick change, every count of half-bits is the one a 50 MHz
board sends. The reader's unit is 8 clocks, so the nominal bit is 64 units.
"""

import pytest

from tools.loomasm import assemble_file, assemble_text
from tools.loomisa import REPO, load
from tools.protomodels import ltc as L
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
IMEM = 512
HALF = 256                                  # clocks a half-bit (header)
UNIT = 8                                    # the reader's unit, clocks
FRAME = L.BITS * 2 * HALF


def program(name, thread=0, dir_pin="OUT0"):
    """A program from ``firmware/``, moved to ``thread`` and its DIR pin to
    ``dir_pin`` if asked."""
    path = FIRMWARE / (name + ".loom")
    if thread == 0 and dir_pin == "OUT0":
        return assemble_file(path, isa=ISA, strict=True)
    text = path.read_text(encoding="utf-8")
    assert text.count(".thread 0") == 1
    text = text.replace(".thread 0", ".thread %d" % thread)
    text = text.replace("DIR = OUT0", "DIR = %s" % dir_pin)
    return assemble_text(text, name + ".loom", isa=ISA, imem_words=IMEM, strict=True)


def ltc_bench(backend):
    """The FIFOs, the deadline latch, the data memory and the 512-word
    memory; the RTL has them all and ``RtlBench`` takes neither keyword."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "SETPD", "DMEM"), imem_words=IMEM)
    return backend.bench()


def start(backend, bench, loads):
    """Load ``[(program, thread, host words)]``, run the threads, then push
    each thread its host words (program headers)."""
    loom = backend.loom(bench, isa=ISA)
    words = {}
    for prog, _, _ in loads:
        words.update(prog.words)
    loom.load(words)
    loom.run([thread for _, thread, _ in loads])
    for _, thread, config in loads:
        loom.push(thread, config)
    return loom


def gen_words(t, rate, half=HALF, frac=0):
    """ltc_gen.loom's host words for a first frame ``t`` at ``rate``."""
    return [half, frac, L.bcd(rate.fps), 0x400 if rate.drop else 0,
            L.bcd(t.frame), L.bcd(t.second), L.bcd(t.minute), L.bcd(t.hour)]


def rx_words(half=HALF):
    return [UNIT, 0, 2 * half // UNIT]


def report(t, rate, user=0):
    """The four words ltc_rx.loom reports for the frame carrying ``t``."""
    return L.words(L.frame_bits(t, rate, user))


def run_reading(bench, loom, cycles, thread=0, chunk=FRAME // 4):
    """Step ``cycles`` clocks, draining OUTQ as a host would; the reports,
    in fours."""
    got = []
    end = bench.cycle + cycles
    while bench.cycle < end:
        bench.step(min(chunk, end - bench.cycle))
        got += loom.pop_available(thread)
    assert len(got) % 4 == 0, got
    return [got[i:i + 4] for i in range(0, len(got), 4)]


# -------------------------------------------------------------- generator
def gen_run(backend, t, rate, frames, half=HALF):
    """Run the generator from ``t`` for a sync word and ``frames`` frames
    (and the edge that starts the next); returns the bits, read off the
    half-bit grid, the recorded edges and the first edge's cycle."""
    bench = ltc_bench(backend)
    rec = bench.add(L.EdgeRecorder("OUT0"))
    loom = start(backend, bench, [(program("ltc_gen"), 0, gen_words(t, rate, half))])
    assert bench.run_until(lambda: rec.edges, 40 * half, every=half // 4), "no first edge"
    first = rec.edges[0][0]
    nbits = 16 + L.BITS * frames
    bench.step(first + 2 * half * nbits + half // 2 - bench.cycle)
    assert loom.badop() == 0
    bits = L.bits_from_edges(rec.edges, first, half, nbits)
    assert rec.edges[0][1] == 1                             # the first edge rises
    return bits, rec.edges, first


def check_stream(bits, t, rate, frames):
    """A sync word, then ``frames`` frames counting from ``t``, each with
    even 0s, the flags of its rate and its sync word."""
    assert bits[:16] == L.SYNC
    for k in range(frames):
        fb = bits[16 + L.BITS * k:16 + L.BITS * (k + 1)]
        assert fb == L.frame_bits(t, rate), (k, str(t))
        d = L.decode_frame(fb)
        assert d.time == t and d.drop == rate.drop and d.polarity_ok
        t = t.next(rate)


def test_ltc_gen_to_model(backend):
    """30 fps from 23:59:59:28 across midnight: every edge on the half-bit
    grid, the frames as the model encodes them, polarity even."""
    t = L.Time(23, 59, 59, 28)
    bits, edges, first = gen_run(backend, t, L.R30, 2)
    check_stream(bits, t, L.R30, 2)
    # bit 0 of every frame starts with the same edge direction (polarity bit)
    starts = {first + 2 * HALF * (16 + L.BITS * k) for k in range(3)}
    levels = [lv for c, lv in edges if c in starts]
    assert len(levels) == 3 and len(set(levels)) == 1


@model_only("two frames each, 90,000 clocks; 30 fps runs on the RTL")
@pytest.mark.parametrize("rate", [L.R24, L.R25, L.R2997])
def test_ltc_gen_rates(backend, rate):
    """24, 25 and 29.97 DF: the frame wraps at the rate, the drop-frame flag,
    the polarity bit at 27 (59 at 25 fps)."""
    t = L.Time(10, 20, 31, rate.fps - 1)
    bits, _, _ = gen_run(backend, t, rate, 2)
    check_stream(bits, t, rate, 2)


@model_only("three frames each, 130,000 clocks; the counting runs on the RTL at 30 fps")
@pytest.mark.parametrize("t,expect", [
    (L.Time(1, 8, 59, 28), L.Time(1, 9, 0, 2)),         # frames 0, 1 dropped
    (L.Time(1, 9, 59, 28), L.Time(1, 10, 0, 0)),        # a tenth minute: none
    (L.Time(23, 59, 59, 28), L.Time(0, 0, 0, 0)),       # midnight is a tenth minute
])
def test_ltc_gen_drop_frame(backend, t, expect):
    """29.97 drop-frame across a minute boundary (SMPTE timecode)."""
    bits, _, _ = gen_run(backend, t, L.R2997, 3)
    check_stream(bits, t, L.R2997, 3)
    assert L.decode_frame(bits[16 + 2 * L.BITS:16 + 3 * L.BITS]).time == expect


# ----------------------------------------------------------------- reader
def rx_run(backend, bits, half=HALF, speed=None, lead=None):
    """Play ``bits`` (biphase mark, from the model) into the reader at
    ``half`` clocks a half-bit and ``speed``; returns the reports, the DIR
    pin's edges and the source."""
    bench = ltc_bench(backend)
    src = bench.add(L.LtcSource("IN0"))
    dirp = bench.add(L.EdgeRecorder("OUT0"))
    loom = start(backend, bench, [(program("ltc_rx"), 0, rx_words(half))])
    bench.step(64 * UNIT)
    # two bits more: the edge that ends the last bit of the last sync word
    src.send(list(bits) + [0, 0], half, at=bench.cycle + (lead or 4 * half), speed=speed)
    got = run_reading(bench, loom, src.free_at + 4 * half - bench.cycle)
    assert loom.badop() == 0
    return got, dirp, src


def frames_from(t, rate, count, user=0):
    out, times = [], []
    for _ in range(count):
        out += L.frame_bits(t, rate, user)
        times.append(t)
        t = t.next(rate)
    return out, times


def test_ltc_rx_reads_model(backend):
    """Three frames forward, user bits set: the first sync word locks, the
    next two frames are reported whole, in frame order, DIR low."""
    user = 0x8421FEDC
    bits, times = frames_from(L.Time(12, 34, 56, 7), L.R30, 3, user)
    got, dirp, _ = rx_run(backend, bits)
    assert got == [report(t, L.R30, user) for t in times[1:]]
    assert all(lv == 0 for _, lv in dirp.edges)


def test_ltc_rx_reverse(backend):
    """Three frames played backwards: the reversed sync word is found and
    the frames come out in frame order, last first, with DIR high."""
    bits, times = frames_from(L.Time(1, 2, 3, 4), L.R25, 3)
    got, dirp, _ = rx_run(backend, L.reverse_play(bits))
    assert got == [report(t, L.R25) for t in (times[2], times[1])]
    assert [lv for _, lv in dirp.edges] == [1]


def test_ltc_rx_speed_10_percent_fast(backend):
    """A tape 10 % fast against the host's nominal bit period."""
    bits, times = frames_from(L.Time(0, 0, 9, 29), L.R30, 3)
    got, _, _ = rx_run(backend, bits, speed=lambda n: 1.1)
    assert got == [report(t, L.R30) for t in times[1:]]


#: name -> (half-bit in clocks, speed of half-bit n)
PROFILES = {
    "slow10": (HALF, lambda n: 0.9),
    "step_up20": (HALF, lambda n: 1.0 if n < 200 else 1.2),
    "step_down20": (HALF, lambda n: 1.0 if n < 200 else 0.8),
    "shuttle": (2 * HALF, lambda n: 1.0 + n / 640.0),  # 1x to 2x in four frames
}


@model_only("four frames each, 160,000 to 330,000 clocks; 10 % fast runs on the RTL")
@pytest.mark.parametrize("profile", sorted(PROFILES))
def test_ltc_rx_speed_profiles(backend, profile):
    """10 % slow, steps of +-20 % in the middle of a frame and a shuttle
    that doubles the speed (the deck's bit rate changing as it winds)."""
    bits, times = frames_from(L.Time(9, 59, 59, 28), L.R30, 4)
    half, speed = PROFILES[profile]
    got, _, _ = rx_run(backend, bits, half, speed=speed)
    assert got == [report(t, L.R30) for t in times[1:]]


def test_ltc_rx_corrupted_sync(backend):
    """Frame 1's sync word with one of its twelve 1s read as a 0: frames 1
    and 2 are lost (neither is framed by two good sync words 80 bits
    apart), frame 3 is reported."""
    bits, times = frames_from(L.Time(5, 6, 7, 8), L.R30, 4)
    bits[L.BITS + 64 + 8] = 0
    got, _, _ = rx_run(backend, bits)
    assert got == [report(times[3], L.R30)]


# ----------------------------------------------------------------- together
def both_run(backend, t, rate, frames):
    """The generator on thread 0 wired to the reader on thread 1 (DIR on
    OUT1); the generator's bits off the grid and the reports."""
    bench = ltc_bench(backend)
    tx = bench.add(L.EdgeRecorder("OUT0"))
    bench.add(L.Wire("OUT0", "IN0"))
    dirp = bench.add(L.EdgeRecorder("OUT1"))
    loom = start(backend, bench, [(program("ltc_rx", 1, "OUT1"), 1, rx_words()),
                                  (program("ltc_gen"), 0, gen_words(t, rate))])
    assert bench.run_until(lambda: tx.edges, 40 * HALF, every=HALF // 4)
    first = tx.edges[0][0]
    nbits = 16 + L.BITS * frames
    got = run_reading(bench, loom, first + 2 * HALF * (nbits + 1) - bench.cycle, thread=1)
    assert loom.badop() == 0
    assert not dirp.edges                                   # forward all along
    return L.bits_from_edges(tx.edges, first, HALF, nbits), got


def test_ltc_together(backend):
    """ltc_gen.loom into ltc_rx.loom across a tenth minute at 30 fps: the
    reader locks on the generator's first sync word and reports every frame
    it sends."""
    t = L.Time(0, 9, 59, 28)
    bits, got = both_run(backend, t, L.R30, 2)
    check_stream(bits, t, L.R30, 2)
    assert got == [report(t, L.R30), report(t.next(L.R30), L.R30)]


@model_only("four frames, 180,000 clocks; 30 fps together runs on the RTL")
def test_ltc_together_drop_frame(backend):
    """29.97 drop-frame through both programs: 01:08:59:28, :29, then
    01:09:00:02 and :03 as the reader reports them."""
    t = L.Time(1, 8, 59, 28)
    bits, got = both_run(backend, t, L.R2997, 4)
    check_stream(bits, t, L.R2997, 4)
    want = [t, L.Time(1, 8, 59, 29), L.Time(1, 9, 0, 2), L.Time(1, 9, 0, 3)]
    assert got == [report(x, L.R2997) for x in want]


# ------------------------------------------------- assembly and the model
def test_ltc_programs_assemble_strict_with_no_diagnostic():
    """Both programs, --strict, no diagnostic; the generator's 20 pairs are
    proved at its .tick of 184 clocks; the two fit threads 0 and 1."""
    gen, rx = program("ltc_gen"), program("ltc_rx")
    for prog in (gen, rx):
        assert prog.diagnostics == []
        assert max(prog.words) < IMEM // 4
    assert len(gen.words) == 128 and len(rx.words) == 107
    rep = gen.deadlines[0]
    assert rep.period == 184 and rep.pairs and not rep.infeasible and not rep.unbounded
    assert rep.worst_slack >= 0
    assert not rx.deadlines[0].pairs                        # no WAITD (header)
    moved = program("ltc_rx", 1, "OUT1")
    assert set(moved.words).isdisjoint(gen.words)


def test_ltc_model_round_trip():
    for rate in L.RATES:
        for t in (L.Time(0, 0, 0, 0), L.Time(23, 59, 59, rate.fps - 1),
                  L.Time(12, 34, 56, rate.fps // 2)):
            b = L.frame_bits(t, rate, user=0x12345678)
            d = L.decode_frame(b)
            assert d.time == t and d.drop == rate.drop and d.user == 0x12345678
            assert d.polarity_ok and b.count(0) % 2 == 0
            assert L.bits_of_words(L.words(b)) == b[:64]
    assert L.SYNC_WORD == 0xBFFC
    assert L.polarity_bit(25) == 59 and L.polarity_bit(30) == 27


def test_ltc_model_drop_frame_counting():
    """Ten minutes of 29.97 DF hold 17,982 labels (18,000 less two in nine
    of the minutes), and next and prev undo each other."""
    t, n = L.Time(0, 0, 0, 0), 0
    while True:
        u = t.next(L.R2997)
        assert u.valid(L.R2997) and u.prev(L.R2997) == t
        t, n = u, n + 1
        if t == L.Time(0, 10, 0, 0):
            break
    assert n == 17982


def test_ltc_model_decoder_both_ways_and_checker():
    bits = L.stream(L.Time(3, 4, 5, 6), L.R24, 3) + [0, 0]
    times = []
    lv = L.bmc_levels(bits)
    for play, want in ((bits, [L.Time(3, 4, 5, 7), L.Time(3, 4, 5, 8)]),
                       (L.reverse_play(bits), [L.Time(3, 4, 5, 8), L.Time(3, 4, 5, 7)])):
        lv = L.bmc_levels(play)
        times = [0] + [10 * i for i in range(1, len(lv)) if lv[i] != lv[i - 1]]
        got = L.find_frames(L.intervals_to_bits(times, 20))
        assert [L.decode_frame(d).time for _, _, d in got] == want
    edges = [(10 * i, 0) for i in range(0, 20, 2)]
    assert L.bits_from_edges(edges, 0, 10, 10) == [0] * 10
    with pytest.raises(AssertionError):
        L.bits_from_edges(edges + [(31, 1)], 0, 10, 10)
