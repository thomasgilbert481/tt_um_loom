"""L3 for the SENT (SAE J2716) programs, on the model and the RTL.

``firmware/sent_tx.loom`` (TX = OUT0) sends a SENT frame stream, computing
the CRC; ``firmware/sent_rx.loom`` (RX = IN0) learns the transmitter's tick
from each sync and reports every frame with a status word. The reference is
``tools.protomodels.sent``: ``decode`` holds the transmitter's pin to the
J2716 timing (the sync against the nominal tick +-20 %, low times of 4 ticks
or more, nibbles of 12..27 ticks, the 1/64 rule, the CRC), ``SentSource``
feeds the receiver frames at any tick with faults, and ``Wire`` joins the
two programs' pins when they run together (TX thread 0, RX thread 1).

Expected values come from the SENT description in the model's docstring
(unchecked against J2716 itself, see there) and from SEMANTICS 6.10: a
deadline-latched edge lands on the exact clock edge of its tick, so with a
whole number of clocks a tick every pulse of the transmitter is exactly its
tick count times the tick and every low time exactly 5 ticks.

The board. SENT's tick is 3 us nominal; at 50 MHz that is 150 clocks and a
6-nibble frame about 40,000 clocks. The RTL runs the programs on a board
clocked at 16 MHz, where the tick is 48 clocks (the same waveform in
microseconds, 3.1 times fewer clocks) and the receiver's measuring tick,
1/16 of it, is 3 clocks. The 50 MHz bodies are ``model_only``.
"""

import pytest

from tools.loomasm import assemble_file, assemble_text
from tools.loomisa import REPO, load
from tools.protomodels import sent as S
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
IMEM = 512


class Board:
    """A clock and the host words the program headers give for it: the
    transmitter's tick (one SENT tick) and the receiver's (1/16 of one)."""

    def __init__(self, name, clock_hz, tx_tick, rx_tick):
        self.name, self.clock_hz = name, float(clock_hz)
        self.tx_tick, self.rx_tick = tx_tick, rx_tick

    @property
    def tick(self):
        """The nominal SENT tick, 3 us, in clocks."""
        return S.TICK_NOMINAL_US * self.clock_hz / 1e6

    def cycles(self, us):
        return int(round(us * self.clock_hz / 1e6))

    def __repr__(self):
        return self.name


B16 = Board("16MHz", 16_000_000, (48, 0), (3, 0))
B50 = Board("50MHz", 50_000_000, (150, 0), (9, 96))

E_CRC, E_RANGE, E_SYNC, E_FIRST = 1, 2, 4, 8       # sent_rx status word bits
PAUSE, LEGACY = 0x08, 0x10                          # layout word bits


def program(name, thread=0):
    """A program from ``firmware/``, moved to ``thread`` if asked."""
    path = FIRMWARE / (name + ".loom")
    if thread == 0:
        return assemble_file(path, isa=ISA, strict=True)
    text = path.read_text(encoding="utf-8")
    assert text.count(".thread 0") == 1
    text = text.replace(".thread 0", ".thread %d" % thread)
    return assemble_text(text, name + ".loom", isa=ISA, imem_words=IMEM, strict=True)


def sent_bench(backend):
    """The FIFOs, the deadline latch, the bit engine and the 512-word memory;
    the RTL has them all and ``RtlBench`` takes neither keyword."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "SETPD", "BE"), imem_words=IMEM)
    return backend.bench()


def start(backend, bench, loads):
    """Load ``[(program, thread, host words)]``, run the threads, then push
    each thread its configuration words (program headers)."""
    loom = backend.loom(bench, isa=ISA)
    words = {}
    for prog, _, _ in loads:
        words.update(prog.words)
    loom.load(words)
    loom.run([thread for _, thread, _ in loads])
    for _, thread, config in loads:
        loom.push(thread, config)
    return loom


def layout(n, tick, pause=None, legacy=False):
    """The configuration words: TICK_INT, the layout with TICK_FRAC, and the
    pause pulse if any (sent_tx.loom header; sent_rx.loom takes two)."""
    tint, tfrac = tick
    word = n | (PAUSE if pause else 0) | (LEGACY if legacy else 0) | (tfrac << 8)
    return [tint, word] + ([pause] if pause else [])


def frame_words(frame):
    """A frame's nibbles, status first, four to a word, zero-filled."""
    nib = [frame.status] + list(frame.data)
    nib += [0] * (-len(nib) % 4)
    return [(nib[i] << 12) | (nib[i + 1] << 8) | (nib[i + 2] << 4) | nib[i + 3]
            for i in range(0, len(nib), 4)]


# ------------------------------------------------------------ transmitter
def tx_setup(backend, board, n, pause=None, legacy=False, tick=None):
    bench = sent_bench(backend)
    rec = bench.add(S.EdgeRecorder("OUT0", idle=0))     # OUT0 is low until RUN
    loom = start(backend, bench, [(program("sent_tx"), 0,
                                   layout(n, tick or board.tx_tick, pause, legacy))])
    return bench, loom, rec


def falls(rec):
    return sum(1 for _, lv in rec.edges if lv == 0)


def tx_run(bench, rec, board, frames, n, pause=None):
    """Run until ``frames`` whole frames (and the falling edge that ends the
    last) are on the pin."""
    want = frames * (n + 3 + (1 if pause else 0)) + 1
    budget = (frames + 1) * (56 + (n + 2) * 27 + (pause or 0) + 20) * int(board.tick)
    assert bench.run_until(lambda: falls(rec) >= want, budget, every=64), (
        "%d of %d falling edges" % (falls(rec), want))


def check_tx(rec, n, expected, tick, pause=None, legacy=False):
    """Decode the pin and hold it to J2716 and SEMANTICS 6.10: the frames
    ``expected`` in order, no check failing, and every pulse exactly its
    tick count of ``tick`` clocks with a low time of exactly 5 ticks."""
    got = S.decode(rec.edges, n, tick, pause=bool(pause), legacy=legacy)
    assert len(got) >= len(expected), len(got)
    for g, e in zip(got, expected):
        assert g.nibbles == e.nibbles(), (g.nibbles, e.nibbles())
        assert g.errors == []
        assert g.sync_clocks == S.SYNC_TICKS * tick
        assert g.pulses == [t * tick for t in S.frame_ticks(e)[1:]]
        assert set(g.lows) == {S.LOW_TYPICAL * tick}
    assert [g.first for g in got] == [True] + [False] * (len(got) - 1)
    return got


F6A = S.Frame(0x0, [1, 2, 3, 4, 5, 6])
F6B = S.Frame(0xC, [0xF, 0x0, 0xA, 0x5, 0xE, 0x9])


def test_tx_frames_and_repeat(backend):
    """Six data nibbles, the recommended CRC, no pause, at the 16 MHz board:
    a frame, the next frame queued during it, then (INQ empty) the second
    frame again. The PHY-GL-SDF candidate (three frames, 30,000 clocks)."""
    bench, loom, rec = tx_setup(backend, B16, 6)
    loom.push(0, frame_words(F6A))
    assert bench.run_until(lambda: falls(rec) >= 3, 4000, every=16)
    loom.push(0, frame_words(F6B))
    tx_run(bench, rec, B16, 3, 6)
    check_tx(rec, 6, [F6A, F6B, F6B], 48)
    assert loom.badop() == 0


def test_tx_pause_and_legacy_crc(backend):
    """Three data nibbles (one word a frame), a 20-tick pause pulse, the
    legacy CRC: two frames, the second queued during the first."""
    a = S.Frame(0x3, [0x7, 0x0, 0xD], legacy=True, pause=20)
    b = S.Frame(0x8, [0xF, 0xF, 0x1], legacy=True, pause=20)
    bench, loom, rec = tx_setup(backend, B16, 3, pause=20, legacy=True)
    loom.push(0, frame_words(a))
    assert bench.run_until(lambda: falls(rec) >= 3, 4000, every=16)
    loom.push(0, frame_words(b))
    tx_run(bench, rec, B16, 2, 3, pause=20)
    check_tx(rec, 3, [a, b], 48, pause=20, legacy=True)
    assert a.crc_nibble != S.crc4(a.data)           # the variants differ here


@model_only("four frames of five nibbles, 40,000 clocks; the frame stream "
            "with a two-word frame runs there in test_tx_frames_and_repeat")
def test_tx_half_frame_is_held(backend):
    """Five data nibbles: a new frame whose second word comes late is held
    and the old frame goes out again meanwhile (twice: word B comes just
    after the second frame boundary); then the new one."""
    a, b = S.Frame(0x1, [2, 3, 4, 5, 6]), S.Frame(0x9, [8, 7, 6, 5, 4])
    bench, loom, rec = tx_setup(backend, B16, 5)
    loom.push(0, frame_words(a))
    assert bench.run_until(lambda: falls(rec) >= 3, 4000, every=16)
    wa, wb = frame_words(b)
    loom.push(0, [wa])                              # word B not yet
    tx_run(bench, rec, B16, 2, 5)
    loom.push(0, [wb])
    tx_run(bench, rec, B16, 4, 5)
    check_tx(rec, 5, [a, a, a, b], 48)          # B missed the second boundary


@model_only("one frame at 50 MHz, 40,000 clocks; the program's schedule is "
            "in ticks, so the 16 MHz runs prove the same schedule")
def test_tx_at_50mhz(backend):
    """The 3 us tick at 50 MHz, 150 clocks: one frame, every edge exact."""
    bench, loom, rec = tx_setup(backend, B50, 6)
    loom.push(0, frame_words(F6B))
    tx_run(bench, rec, B50, 1, 6)
    check_tx(rec, 6, [F6B], 150)


@model_only("a 22 % fast and a 22 % slow tick on the transmitter: the "
            "receiver's tolerance tests run on the RTL")
@pytest.mark.parametrize("tint", [37, 59])
def test_tx_any_tick(backend, tint):
    """The tick is the host's: 37 and 59 clocks (3 us -23 % and +23 % at
    16 MHz) give the same frame in those ticks."""
    bench, loom, rec = tx_setup(backend, B16, 2, tick=(tint, 0))
    f = S.Frame(0x5, [0xA, 0x3])
    loom.push(0, frame_words(f))
    tx_run(bench, rec, B16, 1, 2)
    got = S.decode(rec.edges, 2, tint)
    assert got[0].nibbles == f.nibbles() and got[0].errors == []
    assert got[0].pulses == [t * tint for t in S.frame_ticks(f)[1:]]


# --------------------------------------------------------------- receiver
def rx_setup(backend, board, n, pause=False, legacy=False, low=S.LOW_TYPICAL):
    bench = sent_bench(backend)
    src = bench.add(S.SentSource("IN0", board.clock_hz, low=low))
    loom = start(backend, bench, [(program("sent_rx"), 0,
                                   layout(n, board.rx_tick, pause, legacy)[:2])])
    return bench, loom, src


def reports(words, n):
    """OUTQ words as ``[(data words, status word)]``, ceil((N + 1) / 4)
    data words a frame (program header)."""
    per = (n + 1 + 3) // 4 + 1
    assert len(words) % per == 0, words
    return [(words[i:i + per - 1], words[i + per - 1]) for i in range(0, len(words), per)]


def report(frame, n, flags=0, pause=False, legacy=False, crc=None):
    """What the receiver reports for ``frame`` (program header): its words,
    then the flags, the layout in bits 8:4 and the CRC received."""
    lay = n | (PAUSE if pause else 0) | (LEGACY if legacy else 0)
    got = frame.crc_nibble if crc is None else crc
    return (frame_words(frame), flags | (lay << 4) | (got << 12))


def rx_send(bench, loom, src, board, frames, n, tail_us=300.0):
    """Send ``[(frame, tick_us)]`` as one stream, run past its end reading
    OUTQ every 2,000 clocks (a frame is 7,000 or more), and decode the reports."""
    src.send(frames, at=bench.cycle + board.cycles(200))
    end = src.free_at + board.cycles(tail_us)
    words = []
    while bench.cycle < end:
        bench.step(min(2000, end - bench.cycle))
        words += loom.pop_available(0)
    return reports(words, n)


T = S.TICK_NOMINAL_US
FRAMES6 = [S.Frame(0x0, [1, 2, 3, 4, 5, 6]), S.Frame(0xF, [0xF] * 6),
           S.Frame(0x6, [0x0, 0x9, 0x0, 0xA, 0xB, 0xC])]


@pytest.mark.parametrize("scale", [0.8, 1.0, 1.2])
def test_rx_follows_the_tick(backend, scale):
    """Three frames of six nibbles from a transmitter at -20 %, nominal and
    +20 % of the 3 us tick: every frame decoded, the first flagged as having
    no previous sync, the others clean."""
    bench, loom, src = rx_setup(backend, B16, 6)
    got = rx_send(bench, loom, src, B16, [(f, T * scale) for f in FRAMES6], 6)
    assert got == [report(FRAMES6[0], 6, E_FIRST)] + [report(f, 6) for f in FRAMES6[1:]]
    assert loom.badop() == 0


@model_only("six frames, 60,000 clocks; the fixed ticks run on the RTL")
def test_rx_drift_and_the_1_64_rule(backend):
    """The tick drifts by 1/100 a frame (within 1/64 of the previous sync):
    clean; then a step of 1/32: a sync ratio error, and one back."""
    bench, loom, src = rx_setup(backend, B16, 2)
    f = S.Frame(0x2, [0x4, 0x8])
    ticks = [T * 1.01 ** k for k in range(4)]
    ticks += [ticks[-1] * (1 + 1 / 32.0), ticks[-1]]
    got = rx_send(bench, loom, src, B16, [(f, t) for t in ticks], 2)
    assert [s & 0xF for _, s in got] == [E_FIRST, 0, 0, 0, E_SYNC, E_SYNC]
    assert all(w == frame_words(f) for w, _ in got)


N2 = [S.Frame(0x1, [0x2, 0x3]), S.Frame(0x4, [0xF, 0x6])]
ERRORS = [
    # (what, frame, the status word's flags, the CRC nibble received)
    ("good", N2[0], E_FIRST, None),
    ("bad CRC", S.Frame(0x1, [0x2, 0x3], crc=S.crc4([2, 3]) ^ 1), E_CRC, None),
    ("a nibble of 28 ticks", S.Frame(0x4, [0xF, 0x6], ticks={1: 28}), E_RANGE, None),
    ("a nibble of 11 ticks", S.Frame(0x4, [0x0, 0x6], ticks={1: 11}), E_RANGE, None),
    ("good", N2[1], 0, None),
    # The tick learnt from a sync 1/32 long is 1/32 long, so the 20-tick CRC
    # nibble (8) reads 20 / (1 + 1/32) = 19.4 ticks, 7, which fails the CRC.
    ("sync 1/32 long", S.Frame(0x1, [0x2, 0x3], sync_scale=1 + 1 / 32.0), E_SYNC | E_CRC, 7),
    ("good, 1/33 short of that", N2[0], E_SYNC, None),
    ("good", N2[1], 0, None),
]


def test_rx_errors(backend):
    """Each fault of the model's injection is flagged on its own frame (a
    nibble out of range reads 0 or 15), and the stream goes on."""
    bench, loom, src = rx_setup(backend, B16, 2)
    got = rx_send(bench, loom, src, B16, [(f, T) for _, f, _, _ in ERRORS], 2)
    assert got == [report(f, 2, flags, crc=c) for _, f, flags, c in ERRORS]


@model_only("three frames, 30,000 clocks; the faults run on the RTL")
def test_rx_lost_sync(backend):
    """A sync of 40 ticks is not one: that frame is lost, the receiver hunts,
    and the next frame comes with no previous sync."""
    bench, loom, src = rx_setup(backend, B16, 2)
    lost = S.Frame(0x7, [0x7, 0x7], sync_scale=40 / 56.0)
    got = rx_send(bench, loom, src, B16, [(N2[0], T), (lost, T), (N2[1], T)], 2)
    assert got == [report(N2[0], 2, E_FIRST), report(N2[1], 2, E_FIRST)]


def test_rx_pause_and_legacy_crc(backend):
    """Three data nibbles, the legacy CRC and a pause pulse (20, then 300
    ticks): decoded clean; the recommended CRC on them would be wrong."""
    frames = [S.Frame(0x3, [0x7, 0x0, 0xD], legacy=True, pause=20),
              S.Frame(0x8, [0xF, 0xF, 0x1], legacy=True, pause=300),
              S.Frame(0x0, [0x1, 0x2, 0x3], legacy=True, pause=20)]
    bench, loom, src = rx_setup(backend, B16, 3, pause=True, legacy=True)
    got = rx_send(bench, loom, src, B16, [(f, T) for f in frames], 3)
    assert got == [report(frames[0], 3, E_FIRST, True, True)] + \
                  [report(f, 3, 0, True, True) for f in frames[1:]]
    assert any(f.crc_nibble != S.crc4(f.data) for f in frames)


@model_only("low times are the transmitter's business; 20,000 clocks")
@pytest.mark.parametrize("low", [4, 10])
def test_rx_low_time(backend, low):
    """Pulses with a low time of 4 ticks (J2716's least) and of 10: the
    receiver measures falling edge to falling edge and does not care."""
    bench, loom, src = rx_setup(backend, B16, 2, low=low)
    got = rx_send(bench, loom, src, B16, [(f, T) for f in N2], 2)
    assert got == [report(N2[0], 2, E_FIRST), report(N2[1], 2)]


@model_only("two frames at 50 MHz, 80,000 clocks; the 16 MHz runs prove the "
            "same decoding in units")
def test_rx_at_50mhz(backend):
    """The 50 MHz measuring tick, 9 + 96/256 clocks, at a +20 % and a -20 %
    transmitter."""
    bench, loom, src = rx_setup(backend, B50, 6)
    got = rx_send(bench, loom, src, B50, [(FRAMES6[0], T * 1.2), (FRAMES6[1], T * 1.2)], 6)
    assert got == [report(FRAMES6[0], 6, E_FIRST), report(FRAMES6[1], 6)]


# ---------------------------------------------------------------- together
def both_setup(backend, n, tx_tick, pause=None, legacy=False):
    """The transmitter on thread 0, the board wire from OUT0 to IN0, the
    receiver on thread 1 (its source with `.thread 1`, at 0x080)."""
    bench = sent_bench(backend)
    rec = bench.add(S.EdgeRecorder("OUT0", idle=0))
    bench.add(S.Wire("OUT0", "IN0"))
    loom = start(backend, bench, [
        (program("sent_tx"), 0, layout(n, tx_tick, pause, legacy)),
        (program("sent_rx", 1), 1, layout(n, B16.rx_tick, pause, legacy)[:2])])
    return bench, loom, rec


def run_both(bench, loom, rec, frames, n, pause=None):
    """Run until ``frames`` frames are on the pin, reading OUTQ[1] as it goes;
    returns the receiver's reports."""
    want = frames * (n + 3 + (1 if pause else 0)) + 1
    words = []
    for _ in range(200):
        if falls(rec) >= want:
            break
        bench.step(2000)
        words += loom.pop_available(1)
    assert falls(rec) >= want
    bench.step(500)
    return reports(words + loom.pop_available(1), n)


def test_together_frames(backend):
    """Thread 0 sends two frames of six nibbles at a 10 % slow tick (53
    clocks for 48) and the second again; thread 1 receives all three; the
    pin is J2716 to the clock."""
    bench, loom, rec = both_setup(backend, 6, (53, 0))
    loom.push(0, frame_words(F6A))
    assert bench.run_until(lambda: falls(rec) >= 3, 4000, every=16)
    loom.push(0, frame_words(F6B))
    got = run_both(bench, loom, rec, 3, 6)
    check_tx(rec, 6, [F6A, F6B, F6B], 53)
    assert got == [report(F6A, 6, E_FIRST), report(F6B, 6), report(F6B, 6)]
    assert loom.badop() == 0


@model_only("three frames with a pause, 25,000 clocks; the six-nibble "
            "stream runs on the RTL")
def test_together_pause_legacy_fast(backend):
    """Three data nibbles, the legacy CRC, a 40-tick pause, at a 19 % fast
    tick (39 clocks): the receiver follows and checks the legacy CRC."""
    a = S.Frame(0xA, [0x1, 0xB, 0x2], legacy=True, pause=40)
    bench, loom, rec = both_setup(backend, 3, (39, 0), pause=40, legacy=True)
    loom.push(0, frame_words(a))
    got = run_both(bench, loom, rec, 3, 3, pause=40)
    check_tx(rec, 3, [a, a, a], 39, pause=40, legacy=True)
    assert got == [report(a, 3, E_FIRST, True, True)] + [report(a, 3, 0, True, True)] * 2


# -------------------------------------------------------------- the programs
def test_programs_assemble_strict_with_no_diagnostic():
    """L2-DEADLINE: the transmitter's 8 pairs proved at a 32-clock tick with
    16 clocks of slack, its two INQ tests' POPs declared bounded; the
    receiver has no WAITD and no pair. The transmitter fits a quarter of the
    512-word memory; the receiver takes 146 words, 18 into the next quarter
    (its header), and the two together on threads 0 and 1 do not overlap."""
    tx, rx = program("sent_tx"), program("sent_rx")
    assert tx.diagnostics == [] and rx.diagnostics == []
    assert (len(tx.words), len(rx.words)) == (108, 146)
    t = tx.deadlines[0]
    assert t.period == 32 and not t.infeasible and not t.unbounded
    assert len(t.pairs) == 8 and t.worst_slack == 16
    assert [d.name for d in t.declarations] == ["POP", "POP"]
    assert all("only this thread pops INQ" in d.reason for d in t.declarations)
    r = rx.deadlines[0]
    assert r.period == 32 and r.pairs == [] and not r.unbounded
    assert max(tx.words) < 0x80
    moved = program("sent_rx", 1)
    assert moved.diagnostics == []
    assert (min(moved.words), max(moved.words)) == (0x80, 0x111)


# --------------------------------------------------------------- the model
def test_model_crc():
    """The CRC table of J2716 (as published with it) and the two variants."""
    assert S.crc_table() == [0, 13, 7, 10, 14, 3, 9, 4, 1, 12, 6, 11, 15, 2, 8, 5]
    assert S.crc4([1, 2, 3, 4, 5, 6], legacy=True) == 13
    assert S.crc4([1, 2, 3, 4, 5, 6]) == S.crc_table()[13]


def test_model_decoder():
    """frame_ticks through edges at 48 clocks a tick decode clean; the
    injected faults are what the decoder reports."""
    def edges(frames, tick):
        out, t = [], 0
        for f in frames:
            for ticks in S.frame_ticks(f):
                out += [(int(t), 0), (int(t + 5 * tick), 1)]
                t += ticks * tick
        return out + [(int(t), 0), (int(t + 5 * tick), 1)]
    good = S.decode(edges(N2, 48), 2, 48)
    assert [g.nibbles for g in good] == [f.nibbles() for f in N2]
    assert [g.errors for g in good] == [[], []] and good[0].first and not good[1].first
    bad = S.decode(edges([f for _, f, _, _ in ERRORS], 48), 2, 48)
    assert [g.errors for g in bad] == [[], ["crc"], ["range"], ["range"], [],
                                      ["crc", "sync ratio"], ["sync ratio"], []]
    assert bad[5].crc == 7
