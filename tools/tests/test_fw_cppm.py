"""L3-CPPM on the model and the RTL: firmware/cppm_tx.loom (thread 0, PPM =
OUT0) and firmware/cppm_rx.loom (thread 1, RX = IN0).

CPPM has no formal standard; ``tools.protomodels.cppm`` is the reference
(encoder with error injection, Betaflight's decoder rules, a timing
checker), written from the secondary sources its docstring cites
(EdgeTX, Betaflight, ArduPilot). Expected values come from those sources
through the model and from the programs' headers (host words, clamps,
the frame-boundary rule), never from what the model or the RTL did.

Rates. One tick is the protocol's microsecond. At 50 MHz that is TICK_INT
50; the RTL scenarios, and most model ones, run TICK_INT 4 (the shortest
tick cppm_tx.loom's pairs are proved for), which scales every time by
4/50 and keeps the protocol's ratios: a 300 us separator is 1200 clocks.
Frames are kept short the way the sources allow: four channels (Betaflight's
minimum) and the shortest sync gap EdgeTX sends (3 ms), about 8,700 ticks a
frame. The model also runs the real rate: 8 channels, 300 us, 22.5 ms.

The PHY-GL-SDF candidate is ``test_cppm_rx_decodes_model_frames``: the
encoder model drives IN0 and only the receiver runs, three frames.
"""

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels import cppm as C
from tools.protomodels.quadrature import Wire
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
IMEM = 512
TX = assemble_file(FIRMWARE / "cppm_tx.loom", isa=ISA, strict=True)
RX = assemble_file(FIRMWARE / "cppm_rx.loom", isa=ISA, strict=True)

T = 4                           # clocks a tick on the RTL (the scaled us)
SEP = C.SEP_DEFAULT             # 300 us [EdgeTX]
SHORT = 3000                    # a FRAME the sync minimum overrides: frames of sum + 3 ms
SET_A = [1000, 1500, 2000, 1250]
SET_B = [1900, 1100, 1600, 800]
SET_8 = [988, 2012, 1500, 1234, 1766, 1100, 1900, 1500]    # [EdgeTX] 1500 +- 512
LOAD = 0x8000


def bench_for(backend):
    """FIFOs, the deadline latch, LD/ST and the 512-word memory."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "SETPD", "DMEM"), imem_words=IMEM)
    return backend.bench()


def start(backend, bench, images, tick, threads):
    """Load ``images`` (merged), set every thread's tick, run ``threads``."""
    words = {}
    for image in images:
        words.update(image)
    loom = backend.loom(bench, isa=ISA)
    loom.load(words)
    for thread in threads:
        loom.write_csr(thread, "TICK_INT", tick)
        loom.write_csr(thread, "TICK_FRAC", 0)
    loom.run(list(threads), exclusive=True)
    return loom


def load_set(loom, values):
    """cppm_tx's load: 0x8000 | N, then the N values."""
    loom.push(0, [LOAD | len(values)] + list(values))


def collect(bench, loom, until, every=2000):
    """Read OUTQ[1] as cppm_rx's host would, until cycle ``until``."""
    words = []
    while bench.cycle < until:
        words += loom.pop_available(1)
        bench.step(every)
    return words + loom.pop_available(1)


def close(got, want, tol=1):
    """Channel widths within ``tol`` ticks (one tick: the timestamp's
    resolution, cppm_rx's header)."""
    return len(got) == len(want) and all(abs(g - w) <= tol for g, w in zip(got, want))


def test_cppm_programs_assemble_strict_with_no_diagnostic():
    """L2-DEADLINE: cppm_tx's every pair is proved at its .tick of 4 clocks,
    nothing unbounded, one ``.bounded`` POP; cppm_rx has no pair (its only
    deadlines are timeouts). Words as the headers give them; the two fit
    side by side (thread 0's and thread 1's quarters)."""
    assert TX.diagnostics == [] and list(TX.threads) == [0]
    rep = TX.deadlines[0]
    assert rep.period == T and rep.pairs and not rep.infeasible
    assert not rep.unbounded and rep.worst_slack >= 0
    assert len(TX.words) == 128 and max(TX.words) < 0x80
    assert RX.diagnostics == [] and list(RX.threads) == [1]
    assert all(r.pairs == [] for r in RX.deadlines.values())
    assert len(RX.words) == 93 and 0x80 <= min(RX.words) and max(RX.words) == 0xFF
    assert not set(TX.words) & set(RX.words)


# ============================================================== cppm_tx
def boot_tx(backend, *, sep=SEP, frame=SHORT, idle=0, tick=T):
    """cppm_tx on thread 0, configured, with an edge recorder on OUT0 that
    starts once the line is at ``idle`` (OUT0 resets low; the program sets
    the idle level as it starts)."""
    bench = bench_for(backend)
    watch = bench.add(C.EdgeWatch("OUT0"))
    loom = start(backend, bench, [TX.words], tick, [0])
    loom.push(0, [sep, frame, idle])
    assert bench.run_until(lambda: watch.level == idle, 2000)
    watch.edges.clear()
    return bench, loom, watch


def frames_of(watch, tick=T):
    return C.decode(watch.edges, tick)


@model_only("both polarities and a frame period that is not overridden; "
            "the RTL checks the same timing in the update scenario")
@pytest.mark.parametrize("idle", [0, 1])
def test_cppm_tx_frames_meet_the_sources(backend, idle):
    """TX to the model decoder, eight channels from EdgeTX's range (1500 +-
    512 us) in its default 22.5 ms frame: every frame carries the loaded
    channels to the tick, every separator is 300 us to the clock (latched
    edges on a tick of 4 clocks), the frame period is exactly FRAME (the
    sync gap is FRAME less the channels), and the line idles at ``idle``."""
    frame = C.FRAME_DEFAULT
    bench, loom, watch = boot_tx(backend, frame=frame, idle=idle)
    load_set(loom, SET_8)
    assert bench.run_until(lambda: len(frames_of(watch)) >= 2, 400_000, every=4000)
    assert watch.edges[0][1] == idle ^ 1                # idle until the first pulse
    for f in frames_of(watch):
        assert f.channels == SET_8 and f.out_of_range == []
        assert f.sync == frame - sum(SET_8)
    starts = C.frame_starts(watch.edges, T, idle)
    assert all(b - a == frame * T for a, b in zip(starts, starts[1:]))
    assert C.check_timing(watch.edges, T, sep=SEP, idle=idle) == []


def test_cppm_tx_update_mid_frame_applies_next_frame(backend):
    """A load sent while a frame is going out changes nothing in that frame
    and is the whole of the next (cppm_tx's frame-boundary rule): no frame
    mixes the two sets. FRAME 3 ms is overridden by the 3 ms sync minimum
    ([EdgeTX]), so each sync gap is exactly 3000 ticks; every edge meets
    the sources' timing to the clock."""
    bench, loom, watch = boot_tx(backend)
    load_set(loom, SET_A)
    assert bench.run_until(lambda: len(C.frame_starts(watch.edges, T, 0)) >= 1,
                           120_000, every=1000)          # frame 1 has begun
    load_set(loom, SET_B)
    assert bench.run_until(lambda: len(frames_of(watch)) >= 2, 120_000, every=2000)
    first, second = frames_of(watch)[:2]
    assert first.channels == SET_A and second.channels == SET_B
    assert first.sync == second.sync == 3000
    assert C.check_timing(watch.edges, T, sep=SEP, idle=0) == []


@model_only("clamps and the stop: host-interface corners, no new timing")
def test_cppm_tx_clamps_and_stops(backend):
    """Channel values outside Betaflight's 750 .. 2250 are clamped to it, a
    separator under 250 to 250, N over 8 to 8 (the extra value ignored, and
    the next load taken); a load of N = 0 stops the pulses: the line idles."""
    bench, loom, watch = boot_tx(backend, sep=100)
    load_set(loom, [100, 3000, 1500, 2250, 750])
    assert bench.run_until(lambda: len(frames_of(watch)) >= 1, 150_000, every=2000)
    assert frames_of(watch)[0].channels == [750, 2250, 1500, 2250, 750]
    assert C.check_timing(watch.edges, T, sep=250, idle=0) == []
    loom.push(0, [LOAD | 9] + [1000 + 50 * k for k in range(9)])
    assert bench.run_until(lambda: any(len(f.channels) == 8 for f in frames_of(watch)),
                           200_000, every=2000)
    assert [1000 + 50 * k for k in range(8)] in [f.channels for f in frames_of(watch)]
    loom.push(0, [LOAD | 0])
    bench.step(4 * 12_000 * T)
    last = watch.edges[-1]
    bench.step(2 * 9_000 * T)
    assert watch.edges[-1] == last and last[1] == 0      # stopped, idle low


# ============================================================== cppm_rx
LOSS = 6000                     # ticks: longer than any sync gap sent here


def boot_rx(backend, *, n=4, loss=LOSS, idle=0, tick=T):
    """cppm_rx on thread 1, configured, with the encoder model on IN0."""
    bench = bench_for(backend)
    src = bench.add(C.CppmSource("IN0", idle))
    loom = start(backend, bench, [RX.words], tick, [1])
    loom.push(1, [n, loss])
    bench.step(200)
    return bench, loom, src


def train(frames, start, *, idle=0, syncs=None, tick=T, tail=True):
    """The model encoder's edges for ``frames`` from ``start``, and one more
    separator after the last sync gap, which ends it (a decoder reports a
    frame at the edge that ends its sync gap)."""
    e, end = C.edges(frames, start=start, tick_clocks=tick, idle=idle, sep=SEP,
                     frame=SHORT, syncs=syncs)
    if tail:
        e += [(end, idle ^ 1), (end + SEP * tick, idle)]
    return e, end + SEP * tick


def expected(edge_list, n, tick=T):
    """``[(widths, status)]`` that cppm_rx's header promises for these edges,
    from the reference decoder: kind 0 for a frame of N channels, 1 for
    another count, 2 for a missed sync gap; the [BF] out-of-range mask."""
    out = []
    for f in C.decode(edge_list, tick):
        mask = sum(1 << i for i in f.out_of_range)
        kind = 2 if f.nosync else (0 if n in (0, len(f.channels)) else 1)
        out.append((f.channels, C.status_word(kind, len(f.channels), mask)))
    return out


def check_reports(words, want):
    """The channel words carry indexes 0, 1, ... and widths within a tick
    of the reference's; the status words are exactly the promised ones."""
    got = C.split_reports(words)
    assert [s for _, s in got] == [s for _, s in want], [hex(w) for w in words]
    for (chans, _), (widths, _) in zip(got, want):
        assert [i for i, _ in chans] == list(range(len(chans)))
        assert close([w for _, w in chans], widths)


def test_cppm_rx_decodes_model_frames(backend):
    """Model encoder to RX: three frames of four channels (300 us
    separators, 3 ms sync gaps); the first is partial to the receiver, which
    starts hunting, so frames 2 and 3 are reported, each as four channel
    words within a tick of the wire and a good status (0x8400)."""
    bench, loom, src = boot_rx(backend)
    e, end = train([SET_A, SET_B, SET_A], bench.cycle + 2000)
    src.schedule(e)
    words = collect(bench, loom, end + 2000)
    want = expected(e, 4)
    assert [s for _, s in want] == [0x8400, 0x8400]
    check_reports(words, want)


def test_cppm_rx_rejects_malformed_frames(backend):
    """A glitch (a 20 us pulse 500 us into channel 2) and a short frame
    (three channels where four are expected) are both rejected: the
    glitched frame reports five channels with channel 2 out of range and
    the wrong count; the short one the wrong count. The good frame after
    them is good again."""
    bench, loom, src = boot_rx(backend)
    t0 = bench.cycle + 2000
    e0, t1 = C.edges([SET_A], start=t0, tick_clocks=T, sep=SEP, frame=SHORT)
    e1, t2 = C.edges([SET_A], start=t1, tick_clocks=T, sep=SEP, frame=SHORT)
    e1 = C.glitch(e1, t1 + (SET_A[0] + 500) * T, 20 * T)
    e2, end = train([SET_A[:3], SET_B], t2)
    e = e0 + e1 + e2
    src.schedule(e)
    words = collect(bench, loom, end + 2000)
    want = expected(e, 4)
    assert [s for _, s in want] == [C.status_word(1, 5, 0b10), C.status_word(1, 3),
                                    C.status_word(0, 4)]
    check_reports(words, want)


@model_only("error corners of the receiver; the RTL runs the glitch and the "
            "short frame")
def test_cppm_rx_missing_sync_lost_signal_and_drops(backend):
    """A missed sync gap (two frames run together with a 1 ms gap, under
    2700 us) voids the frame at its ninth channel (kind 2, eight channels)
    and the receiver hunts until the next real sync gap; a line that stops
    is lost after LOSS ticks (kind 3, once); a host that does not read
    OUTQ loses words, and the next status it gets says so (bit 14)."""
    bench, loom, src = boot_rx(backend)
    e, end = train([SET_A, SET_A, SET_B, SET_A], bench.cycle + 2000,
                   syncs=[None, 1000, None, None])
    src.schedule(e)
    words = collect(bench, loom, end + 2000)
    want = expected(e, 4)
    assert [s for _, s in want] == [C.status_word(2, 8), C.status_word(0, 4)]
    check_reports(words, want)
    bench.step(LOSS * T + 4000)                     # the line stops
    assert loom.pop_available(1) == [C.status_word(3, 0)]
    bench.step(4 * LOSS * T)
    assert loom.pop_available(1) == []              # reported once
    e, end = train([SET_A, SET_B, SET_A, SET_B], bench.cycle + 2000)
    src.schedule(e)
    bench.step(end + 2000 - bench.cycle)            # nobody reads OUTQ
    got = loom.pop_available(1)
    assert len(got) == 4                            # FIFO depth: the rest dropped
    e2, end2 = train([SET_A, SET_A], end + 4000)
    src.schedule(e2)
    words = collect(bench, loom, end2 + 2000)
    statuses = [w for w in words if w & 0x8000]
    assert statuses[0] & 0x4000                     # the first one says so
    assert statuses[-1] == C.status_word(0, 4)      # and it clears


@model_only("the other polarity and N = 0; the TX and RX scenario runs the "
            "inverted line on the RTL")
@pytest.mark.parametrize("idle, n", [(1, 4), (0, 0)])
def test_cppm_rx_polarity_and_any_count(backend, idle, n):
    """An idle-high line (low separators) decodes with no setting (the
    rising edges end the separators, all of one width); N = 0 accepts any
    count from 1 to 8 as a good frame."""
    bench, loom, src = boot_rx(backend, n=n, idle=idle)
    e, end = train([SET_A, SET_A[:2] + SET_B, SET_B[:1]], bench.cycle + 2000, idle=idle)
    src.schedule(e)
    words = collect(bench, loom, end + 2000)
    want = expected(e, n)
    if n == 0:
        assert [s for _, s in want] == [C.status_word(0, 6), C.status_word(0, 1)]
    else:
        assert [s for _, s in want] == [C.status_word(1, 6), C.status_word(1, 1)]
    check_reports(words, want)


# ===================================================== cppm_tx and cppm_rx
def test_cppm_tx_and_rx_together(backend):
    """cppm_tx on thread 0 and cppm_rx on thread 1, OUT0 wired to IN0 by the
    bench, the line idle high: the receiver reports the loaded channels to
    the tick (the latched edges are on exact ticks of both threads), two
    good frames."""
    bench = bench_for(backend)
    watch = bench.add(C.EdgeWatch("OUT0"))
    bench.add(Wire("OUT0", "IN0"))
    loom = start(backend, bench, [TX.words, RX.words], T, [0, 1])
    loom.push(1, [4, LOSS])
    loom.push(0, [SEP, SHORT, 1])
    load_set(loom, SET_B)
    words = []
    assert bench.run_until(lambda: words.extend(loom.pop_available(1)) or
                           sum(w >> 15 for w in words) >= 2, 200_000, every=2000)
    reports = C.split_reports(words)
    for chans, status in reports[:2]:
        assert status == C.status_word(0, 4)
        assert close([w for _, w in chans], SET_B)
    assert C.check_timing(watch.edges[1:], T, sep=SEP, idle=1) == []


@model_only("the real rate: about 800 k clocks (about a minute on the model), "
            "the RTL budget's whole")
def test_cppm_real_rate(backend):
    """At 50 MHz with 1 us ticks (TICK_INT 50), TX and RX wired: four
    channels, 300 us separators and EdgeTX's 3 ms sync minimum. Every
    separator is 15000 clocks and every channel its value times 50 clocks
    exactly, and the receiver reports the frame good, each channel within a
    microsecond. (The receiver is started 3 ms before the first pulse, so
    the silence before it is a sync gap and the first frame is reported.)"""
    us = 50
    bench = bench_for(backend)
    watch = bench.add(C.EdgeWatch("OUT0"))
    bench.add(Wire("OUT0", "IN0"))
    loom = start(backend, bench, [TX.words, RX.words], us, [0, 1])
    loom.push(1, [4, 30000])
    bench.step(3000 * us)
    loom.push(0, [SEP, SHORT, 0])
    load_set(loom, SET_A)
    words = []
    assert bench.run_until(lambda: words.extend(loom.pop_available(1)) or
                           any(w >> 15 for w in words), 1_000_000, every=20_000)
    (got, status), = C.split_reports(words)[:1]
    assert status == C.status_word(0, 4)
    assert close([w for _, w in got], SET_A)
    assert C.check_timing(watch.edges, us, sep=SEP, idle=0) == []
    starts = [c for c, v in watch.edges if v == 1]
    assert [(b - a) for a, b in zip(starts, starts[1:])][:5] == [c * us for c in SET_A + [3000]]
