"""L3 for the WSPR beacon source, on the model and the RTL.

``firmware/wspr_tx.loom`` (TONE = OUT0, SYM = OUT1, PTT = OUT2) encodes
each queued message and sends its 162 symbols as a phase-continuous 4-FSK
square wave from a 16-bit phase accumulator, one sample a tick. The
reference is ``tools.protomodels.wspr``: an encoder written from G4JNT's
description of K1JT's encoder (checked here against two published symbol
lists), the accumulator's pin pattern, and a reader that recovers the
symbols from the sampled pin.

Expected values come from the description and the program's documented
contract, never from what the model or the RTL did:

* the symbols equal the reference encoder's (all 162 on the model, a
  prefix on the RTL), read back from the pin by matching every sample;
* every sample of TONE equals the accumulator's top bit for the
  reference symbols (the header's contract: sample n shows the phase after
  the steps of samples 0 .. n-1);
* each tone's frequency, measured from the edges, within the stated error
  of (B + k S) fs / 65536: one sample of edge quantisation over the span
  measured;
* symbol timing exact: PTT high for exactly 162 L ticks, SYM changing at
  every multiple of L ticks after PTT rises, at a constant lag of at most
  two slots, and never anywhere else.

Settings. The program needs a tick of 152 clocks (any board of 14.6 MHz
or more at 96 kHz; the header says why); every body runs at that tick,
with a few samples a symbol and tone steps far apart (the header's scaled timing), so
the whole message is 162 L x 152 clocks. Real WSPR settings (B = 1024,
S = 1, L = 65 536) are checked as arithmetic in
``test_wspr_real_tone_plan`` and, on the pin, for a short prefix with the
real steps in ``test_wspr_real_steps_on_pin``. Every body is named
``test_wspr_*``: test/test_fw.py collects all modules into one namespace.
"""

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels import wspr as W
from tools.protomodels.dtmf import EdgeRecorder
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
TICK = 152                              # clocks: the header's minimum
ENCODE_CLOCKS = 60_000                  # allowance for packing and coding
#: Published symbol lists: the Rust crate ``wspr`` (test_encode_wspr), an
#: implementation of the same G4JNT description (tools.protomodels.wspr).
VECTORS = {
    ("G1ABC", "IO83", 37):
        "330002001020113222322101131222000030010302220232132233010001323222"
        "011230301030012232330023121012221020223201003112332211212001332002"
        "212320112222202332121302233220",
    ("KA1BCD", "AA00", 33):
        "332202023202111002102321111002022032232322202030310231032201321202"
        "033032121030230030332021103032203200203201221312132011230021332022"
        "230122110200002312123322231202",
}


def message_words(call, loc, dbm):
    """The eleven INQ words of the header: the callsign as written,
    left-justified in six characters, the locator, the power."""
    return [ord(c) for c in call.ljust(6)] + [ord(c) for c in loc] + [dbm]


def start(backend, base, spacing, length):
    """Load and start the program with its three settings: (bench, loom,
    TONE, SYM and PTT recorders)."""
    if backend.name == "model":
        bench = backend.bench(features=("FIFO", "SETPD", "DMEM"), imem_words=512)
    else:
        bench = backend.bench()
    recs = [bench.add(EdgeRecorder(p)) for p in ("OUT0", "OUT1", "OUT2")]
    loom = backend.loom(bench, isa=ISA)
    prog = assemble_file(FIRMWARE / "wspr_tx.loom", isa=ISA, strict=True)
    loom.load(prog.words)
    loom.write_csr(0, "TICK_INT", TICK)
    loom.run([0])
    loom.push(0, [base, spacing, length])
    return bench, loom, recs


def ptt_rise(bench, rec, limit):
    """Step until PTT rises; its cycle."""
    assert bench.run_until(lambda: any(v for _, v in rec.edges), limit, 500), \
        "PTT never rose"
    return next(c for c, v in rec.edges if v)


def tone_levels(rec, c0, count):
    """TONE in the middle of each of ``count`` samples from ``c0``."""
    return W.pin_samples(rec.edges, rec.initial, c0, TICK, count, TICK / 2)


def check_sym_timing(sym, c0, length, symbols):
    """SYM changes exactly at every symbol boundary, at one constant lag
    of at most two slots after the tick (header: one slot after it), high
    in even symbols."""
    edges = [(c, v) for c, v in sym.edges if c >= c0]
    assert len(edges) >= symbols, edges[:4]
    lags = []
    for j, (c, v) in enumerate(edges[:symbols]):
        assert v == (1 if j % 2 == 0 else 0), (j, c, v)
        lags.append(c - (c0 + j * length * TICK))
    assert 0 <= min(lags) and max(lags) - min(lags) <= 3 and max(lags) < 8, lags


def run_message(backend, msg, base, spacing, length, symbols):
    """Queue one message and capture ``symbols`` of it: (bench, loom, recs,
    PTT rise cycle, the reference symbols, the TONE samples)."""
    bench, loom, recs = start(backend, base, spacing, length)
    tone, sym, ptt = recs
    loom.push(0, message_words(*msg))
    c0 = ptt_rise(bench, ptt, ENCODE_CLOCKS)
    end = c0 + symbols * length * TICK
    bench.step(end - bench.cycle + TICK)
    ref = W.symbols(*msg)
    return bench, loom, recs, c0, ref, tone_levels(tone, c0, symbols * length)


# ---------------------------------------------------------- the encoder
@pytest.mark.parametrize("msg", sorted(VECTORS))
def test_wspr_reference_matches_published_symbols(msg):
    """The reference encoder against the published lists, all 162."""
    assert "".join(map(str, W.symbols(*msg))) == VECTORS[msg]


def test_wspr_real_tone_plan():
    """At fs = 96 kHz, step 1 is 12000 / 8192 Hz exactly and a symbol is
    65 536 samples exactly; B = 1024 is 1500 Hz; at 50 MHz the tick
    520 + 213/256 clocks is within 3 ppm of 50e6 / 96000."""
    fs = 96_000.0
    for k in range(4):
        assert W.tone_hz(1024, 1, k, fs) == 1500.0 + k * W.TONE_SPACING_HZ
    assert W.SYMBOL_S * fs == 65536.0
    period = 520 + 213 / 256
    assert abs((50e6 / period) / fs - 1) < 3e-6
    assert abs(W.tone_hz(1024, 1, 0, 50e6 / period) - 1500.0) < 0.005


# ----------------------------------------------------------- on the pin
def test_wspr_rtl_prefix(backend):
    """G1ABC IO83 37 at B = 4096, S = 2048, L = 16: the first 24 symbols
    read back from TONE equal the published list, every TONE sample is
    the accumulator's, SYM marks every boundary exactly. The PHY-GL-SDF
    candidate."""
    msg, n, L = ("G1ABC", "IO83", 37), 24, 16
    bench, loom, recs, c0, ref, levels = run_message(backend, msg, 4096, 2048, L, n)
    assert W.recover_symbols(levels, 4096, 2048, L) == ref[:n]
    assert "".join(map(str, ref[:n])) == VECTORS[msg][:n]
    assert levels == W.tone_pin(ref[:n], 4096, 2048, L)
    check_sym_timing(recs[1], c0, L, n)
    assert recs[2].level == 1                           # PTT still high
    assert loom.badop() == 0


def test_wspr_callsign_alignment(backend):
    """K1ABC FN42 37 sent as written ("K1ABC "): the program puts the
    space in front (" K1ABC"); the first 8 symbols are the reference's."""
    msg, n, L = ("K1ABC", "FN42", 37), 8, 16
    bench, loom, recs, c0, ref, levels = run_message(backend, msg, 4096, 2048, L, n)
    assert W.normalise_call("K1ABC") == " K1ABC"
    assert W.recover_symbols(levels, 4096, 2048, L) == ref[:n]
    assert levels == W.tone_pin(ref[:n], 4096, 2048, L)
    assert loom.badop() == 0


@model_only("162 symbols x 16 samples x 152 clocks, 0.4 M clocks each, "
            "about 1 min of RTL each; the RTL runs a 24-symbol prefix")
@pytest.mark.parametrize("msg", sorted(VECTORS))
def test_wspr_whole_message(backend, msg):
    """All 162 symbols of each published message, read back from TONE,
    equal the published list; every sample is the accumulator's; SYM
    marks all 162 boundaries; PTT is high for exactly 162 L ticks, then
    TONE and SYM are low."""
    L = 16
    bench, loom, recs, c0, ref, levels = run_message(backend, msg, 4096, 2048, L, 162)
    assert "".join(map(str, W.recover_symbols(levels, 4096, 2048, L))) == VECTORS[msg]
    assert levels == W.tone_pin(ref, 4096, 2048, L)
    check_sym_timing(recs[1], c0, L, 162)
    rises = [c for c, v in recs[2].edges if v]
    falls = [c for c, v in recs[2].edges if not v and c > c0]
    assert rises == [c0] and falls == [c0 + 162 * L * TICK], (rises, falls)
    bench.step(4 * TICK)
    assert recs[0].level == 0 and recs[1].level == 0
    assert loom.badop() == 0


@model_only("4 symbols of 512 samples, 0.35 M clocks, about 1 min of RTL; "
            "the RTL checks every TONE sample of its prefixes instead")
def test_wspr_tone_frequencies(backend):
    """K1ABC IO83 20 starts 3, 1, 0, 2, so its first four symbols are the
    four tones. At B = 4096, S = 256, L = 512 each is 32 + 2k cycles a
    symbol; the frequency measured from TONE's rising edges is within one
    sample of edge quantisation (plus the 0..3-clock slot lag) over the
    span measured of (B + k S) / 65536 cycles a sample, and the four are
    the four tones the reference symbols name. At fs = 96 kHz the same
    figures are (B + k S) x 1.46484375 Hz."""
    msg, n, L, B, S = ("K1ABC", "IO83", 20), 4, 512, 4096, 256
    bench, loom, recs, c0, ref, levels = run_message(backend, msg, B, S, L, n)
    assert sorted(ref[:n]) == [0, 1, 2, 3]
    tone = recs[0]
    for j in range(n):
        t0, t1 = c0 + j * L * TICK, c0 + (j + 1) * L * TICK
        rises = [c for c, v in tone.edges if v == 1 and t0 <= c < t1]
        span = (rises[-1] - rises[0]) / TICK            # samples
        got = W.measured_hz(tone.edges, t0, t1, 1.0) * TICK   # cycles/sample
        want = W.tone_hz(B, S, ref[j], 1.0)
        tol = want * (1 + 3 / TICK) / span
        assert abs(got - want) <= tol, (j, got, want, tol)
        for k in range(4):                              # and no other tone
            if k != ref[j]:
                assert abs(got - W.tone_hz(B, S, k, 1.0)) > tol, (j, k)
    assert levels == W.tone_pin(ref[:n], B, S, L)
    assert loom.badop() == 0


def test_wspr_real_steps_on_pin(backend):
    """With the real steps (B = 1024: 1500 Hz at 96 kHz; S = 1: 12000 /
    8192 Hz) and L = 64 samples, the first six symbols' TONE samples are
    exactly the accumulator's for the reference symbols (a phase match,
    not a symbol read-back: tones 1.46 Hz apart differ by under 0.003 of a
    cycle in 64 samples)."""
    msg, n, L = ("G1ABC", "IO83", 37), 6, 64
    bench, loom, recs, c0, ref, levels = run_message(backend, msg, 1024, 1, L, n)
    assert levels == W.tone_pin(ref[:n], 1024, 1, L)
    check_sym_timing(recs[1], c0, L, n)
    assert loom.badop() == 0


@model_only("two whole messages at L = 2, 0.2 M clocks; one message's "
            "framing is checked on the RTL by test_wspr_rtl_prefix")
def test_wspr_messages_queue(backend):
    """Two messages queued at once go out one after the other: each its
    own 162 symbols in a PTT window of exactly 162 L ticks, the second
    after the first has ended."""
    L = 2
    msgs = sorted(VECTORS)
    bench, loom, recs = start(backend, 4096, 8192, L)
    tone, sym, ptt = recs
    loom.push(0, message_words(*msgs[0]) + message_words(*msgs[1]))
    c0 = ptt_rise(bench, ptt, ENCODE_CLOCKS)
    bench.step(c0 + 162 * L * TICK - bench.cycle + 2 * ENCODE_CLOCKS)
    rises = [c for c, v in ptt.edges if v]
    falls = [c for c, v in ptt.edges if not v]
    assert len(rises) == 2 and len(falls) >= 1, ptt.edges
    assert rises[0] == c0 and falls[0] == c0 + 162 * L * TICK
    assert rises[1] > falls[0]
    bench.step(rises[1] + 162 * L * TICK - bench.cycle + TICK)
    assert [c - rises[1] for c, v in ptt.edges if not v][1] == 162 * L * TICK
    for c, msg in zip(rises, msgs):
        levels = tone_levels(tone, c, 162 * L)
        assert levels == W.tone_pin(W.symbols(*msg), 4096, 8192, L), msg
    assert loom.badop() == 0
