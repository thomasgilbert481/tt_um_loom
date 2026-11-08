"""L3 for the magnetic stripe programs, on the model and the RTL.

``firmware/magstripe_emu.loom`` (TX = OUT0) plays a host-given track 2 as
one swipe in F2F at a set speed; ``firmware/magstripe_reader.loom`` (RX =
IN0) follows a hand swipe's speed from the clocking zeros, frames the
characters by sentinels, parity and LRC, in either direction, and reports
them. The reference is ``tools.protomodels.magstripe``, written from the
MagTek TTL reader manual and the other public descriptions it cites:
``encode`` builds a track, ``SwipeSource`` plays it with a speed profile
and jitter, ``decode`` is an independent decoder, ``report`` the words the
reader's header promises.

Expected values come from the track format and SEMANTICS: a
deadline-latched transition lands on the exact clock edge of its tick
(6.10), so with a whole number of clocks a half cell every emulator
transition is exactly on the grid ``start + n * half``.

Scaled time. A track 2 swipe at 20 in/s and 50 MHz is 16,667 clocks a half
cell. The tests run 320 clocks a half cell (256 at the fastest point of a
swipe that doubles its speed): only the clocks change, every count of bits
is the card's. The reader's unit is 8 clocks, so a nominal cell is 80
units. Test data are obviously fake numbers, never a card number.
"""

import pytest

from tools.loomasm import assemble_file, assemble_text
from tools.loomisa import REPO, load
from tools.protomodels import magstripe as M
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
IMEM = 512
HALF = 320                                  # clocks a half cell (header)
UNIT = 8                                    # the reader's unit, clocks
TMAX = 400                                  # units: 5 nominal cells
BUF = 192                                   # the reader's bit buffer (header)
ZEROS = 10                                  # clocking zeros each side
TEXT = "99999=0000"                         # obviously fake
STATUS = 0x8000


def program(name, thread=0):
    """A program from ``firmware/``, moved to ``thread`` if asked."""
    path = FIRMWARE / (name + ".loom")
    if thread == 0:
        return assemble_file(path, isa=ISA, strict=True)
    text = path.read_text(encoding="utf-8")
    assert text.count(".thread 0") == 1
    text = text.replace(".thread 0", ".thread %d" % thread)
    return assemble_text(text, name + ".loom", isa=ISA, imem_words=IMEM, strict=True)


def ms_bench(backend):
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


def rx_words(track=M.TRACK2):
    return [UNIT, 0, TMAX, track.data_bits, track.ss, track.es, track.offset, BUF]


def emu_words(text, zeros=ZEROS):
    return [zeros, len(text)] + [ord(c) for c in text]


def want(text, reverse=False, errors=0):
    """The reader's report: the characters, then the status word."""
    return [ord(c) for c in text] + [STATUS | (int(reverse) << 8) | errors]


def read_reports(bench, loom, until, thread=0, chunk=2048):
    """Step to cycle ``until`` at least, draining OUTQ as a host would, and
    on until a status word has come or 200,000 clocks more have passed;
    the reports, each a list ending with its status word."""
    got, limit = [], until + 200000
    while bench.cycle < limit:
        bench.step(chunk)
        got += loom.pop_available(thread)
        if bench.cycle >= until and got and got[-1] & STATUS:
            break
    out, cur = [], []
    for w in got:
        cur.append(w)
        if w & STATUS:
            out.append(cur)
            cur = []
    assert not cur, got
    return out


# ----------------------------------------------------------------- reader
def rx_run(backend, swipes, track=M.TRACK2):
    """Play ``swipes`` (``[(bits, half, speed, jitter)]``, from the model)
    into the reader one after another; returns the reports."""
    bench = ms_bench(backend)
    src = bench.add(M.SwipeSource("IN0"))
    loom = start(backend, bench, [(program("magstripe_reader"), 0, rx_words(track))])
    bench.step(64 * UNIT)
    got = []
    for bits, half, speed, jitter in swipes:
        src.send(bits, half, at=bench.cycle + 4 * half, speed=speed, jitter=jitter)
        if len(bits) < 2:                                   # a stray pulse: nothing to wait for
            bench.step(src.free_at + 64 * half - bench.cycle)
            continue
        got += read_reports(bench, loom, src.free_at + 16 * half)
    assert loom.badop() == 0
    return got


def test_magstripe_reader_reads_model(backend):
    """A track 2 swipe at constant speed: the characters between the
    sentinels as ASCII, then status 0x8000 (forward, no error)."""
    bits = M.encode(TEXT, zeros=ZEROS)
    got = rx_run(backend, [(bits, HALF, None, 0)])
    assert got == [want(TEXT)] == [M.report(M.decode(bits))]


def test_magstripe_reader_speed_doubles(backend):
    """A swipe that speeds up to twice its starting speed end to end (half
    cell 512 clocks to 256), with 8 clocks of jitter on every transition."""
    bits = M.encode(TEXT, zeros=ZEROS)
    got = rx_run(backend, [(bits, 512, M.ramp(1.0, 2.0), 8)])
    assert got == [want(TEXT)]


def test_magstripe_reader_parity_error(backend):
    """The fourth data character written with even parity: reported with
    status bit 0; the LRC (parity bits ignored) still checks."""
    bits = M.encode(TEXT, zeros=ZEROS, bad_parity=[4])
    got = rx_run(backend, [(bits, HALF, None, 0)])
    assert got == [want(TEXT, errors=M.E_PARITY)] == [M.report(M.decode(bits))]


def test_magstripe_reader_lrc_error(backend):
    """An LRC with one column wrong (its own parity odd): status bit 1."""
    bits = M.encode(TEXT, zeros=ZEROS, bad_lrc=True)
    got = rx_run(backend, [(bits, HALF, None, 0)])
    assert got == [want(TEXT, errors=M.E_LRC)] == [M.report(M.decode(bits))]


@model_only("one swipe, 75,000 clocks; forward swipes run on the RTL")
def test_magstripe_reader_reverse(backend):
    """The card swiped the other way: found from the last stored 1, read
    backwards, reported in card order with dir = 1."""
    bits = M.reverse_swipe(M.encode(TEXT, zeros=ZEROS))
    got = rx_run(backend, [(bits, HALF, None, 0)])
    assert got == [want(TEXT, reverse=True)]


def lrc_colon_text():
    """A text whose LRC is 0xA, ':': read backwards from the first 1 the
    LRC and the end sentinel begin 1 1 0 1 0, a start sentinel (MT p. 34)."""
    for n in range(10000):
        t = "%04d=1" % n
        if M.record(t)[-1] == 0xA:
            return t
    raise AssertionError("no such text")


#: name -> (bits, half, speed, jitter)
PROFILES = {
    "slowing_half": (M.encode(TEXT, zeros=ZEROS), 256, M.ramp(1.0, 0.5), 8),
    "hump": (M.encode(TEXT, zeros=22), 400, M.hump(1.0, 1.6), 12),
    "reverse_fast": (M.reverse_swipe(M.encode(TEXT, zeros=ZEROS)), 512, M.ramp(1.0, 2.0), 8),
    "reverse_lrc_colon": (M.reverse_swipe(M.encode(lrc_colon_text(), zeros=ZEROS)), HALF, None, 0),
}


@model_only("80,000 to 150,000 clocks each; doubling speed runs on the RTL")
@pytest.mark.parametrize("profile", sorted(PROFILES))
def test_magstripe_reader_profiles(backend, profile):
    """Slowing to half speed, a push that speeds up 60 % in the middle with
    MagTek's 22 clocking zeros, a fast reverse swipe, and a reverse swipe
    whose LRC passes for a start sentinel: every one reported whole."""
    bits, half, speed, jitter = PROFILES[profile]
    got = rx_run(backend, [(bits, half, speed, jitter)])
    assert got == [M.report(M.decode(bits))]
    assert got[0][-1] & 0xFF == 0


@model_only("track 1, 230 bits, 160,000 clocks")
def test_magstripe_reader_track1(backend):
    """Track 1 (7-bit characters, '%' and '?', ASCII + 20h) by the host's
    settings, reverse, with 62 clocking zeros."""
    text = "B0000000000^TEST/CARD^9912"
    bits = M.encode(text, M.TRACK1, zeros=M.TRACK1.zeros)
    got = rx_run(backend, [(M.reverse_swipe(bits), 256, None, 0)], M.TRACK1)
    assert got == [want(text, reverse=True)]


@model_only("two swipes and a stray transition, 170,000 clocks")
def test_magstripe_reader_two_swipes_and_noise(backend):
    """A stray pulse (two transitions) before a card says nothing; two
    swipes, the second a different card and reversed, give two reports."""
    a, b = "12=34", "5678=90"
    got = rx_run(backend, [([0], HALF, None, 0),
                           (M.encode(a, zeros=ZEROS), HALF, None, 0),
                           (M.reverse_swipe(M.encode(b, zeros=ZEROS)), HALF, None, 0)])
    assert got == [want(a), want(b, reverse=True)]


# --------------------------------------------------------------- emulator
def emu_check(edges, first, text, half=HALF, zeros=ZEROS):
    """Every transition on the half-cell grid from ``first``; the bits read
    off it are the model's track, which the model decodes to ``text``."""
    bits = M.encode(text, zeros=zeros)
    got = M.bits_from_edges(edges, first, half, len(bits))
    assert got == bits
    assert edges[-1][0] == first + 2 * half * len(bits)    # the closing transition
    assert M.decode(got).text == text and M.decode(got).errors == 0


def test_magstripe_emu_to_model(backend):
    """magstripe_emu.loom plays the track: clocking zeros, ';', the
    characters LSB first with odd parity, '?', the LRC, zeros, a closing
    transition, every one on the exact clock of its half-cell tick."""
    bench = ms_bench(backend)
    rec = bench.add(M.EdgeRecorder("OUT0"))
    loom = start(backend, bench, [(program("magstripe_emu"), 0, [HALF, 0] + emu_words(TEXT))])
    assert bench.run_until(lambda: rec.edges, 40 * HALF, every=HALF // 4), "no first edge"
    first = rec.edges[0][0]
    nbits = len(M.encode(TEXT, zeros=ZEROS))
    bench.step(first + 2 * HALF * (nbits + 4) - bench.cycle)
    assert loom.badop() == 0
    assert rec.edges[0][1] == 1                             # the first transition rises
    emu_check(rec.edges, first, TEXT)


@model_only("two swipes, 120,000 clocks; one swipe runs on the RTL")
def test_magstripe_emu_two_swipes(backend):
    """A second swipe from new host words, a card of 40 characters (the
    buffer's limit) after a short one, 3 clocking zeros."""
    bench = ms_bench(backend)
    rec = bench.add(M.EdgeRecorder("OUT0"))
    long_text = ("0123456789" * 4)[:40]
    loom = start(backend, bench, [(program("magstripe_emu"), 0,
                                   [HALF, 0] + emu_words("1=2", 3) + emu_words(long_text, 3))])
    assert bench.run_until(lambda: rec.edges, 40 * HALF, every=HALF // 4)
    n1 = len(M.encode("1=2", zeros=3))
    first = rec.edges[0][0]
    one = [e for e in rec.edges if e[0] <= first + 2 * HALF * n1]
    emu_check(one, first, "1=2", zeros=3)
    bench.run_until(lambda: len(rec.edges) > len(one), 200 * HALF, every=HALF // 4)
    second = rec.edges[len(one)][0]
    n2 = len(M.encode(long_text, zeros=3))
    bench.step(second + 2 * HALF * (n2 + 4) - bench.cycle)
    assert loom.badop() == 0
    emu_check(rec.edges[len(one):], second, long_text, zeros=3)


# ----------------------------------------------------------------- together
def test_magstripe_together(backend):
    """The emulator on thread 2 wired to the reader on thread 0: the reader
    reports the card the host gave the emulator."""
    bench = ms_bench(backend)
    tx = bench.add(M.EdgeRecorder("OUT0"))
    bench.add(M.Wire("OUT0", "IN0"))
    loom = start(backend, bench, [(program("magstripe_reader"), 0, rx_words()),
                                  (program("magstripe_emu", 2), 2, [HALF, 0] + emu_words(TEXT))])
    assert bench.run_until(lambda: tx.edges, 40 * HALF, every=HALF // 4)
    first = tx.edges[0][0]
    nbits = len(M.encode(TEXT, zeros=ZEROS))
    got = read_reports(bench, loom, first + 2 * HALF * (nbits + 8))
    assert loom.badop() == 0
    emu_check(tx.edges, first, TEXT)
    assert got == [want(TEXT)]


# ------------------------------------------------- assembly and the model
def test_magstripe_programs_assemble_strict_with_no_diagnostic():
    """Both programs, --strict, no diagnostic; the emulator's pairs proved
    at its .tick of 80 clocks; reader (0..191, spilling into thread 1's
    quarter), its buffer (192..231) and the emulator on thread 2 fit
    together."""
    rx, emu = program("magstripe_reader"), program("magstripe_emu")
    for prog in (rx, emu):
        assert prog.diagnostics == []
    assert len(rx.words) == 192 and max(rx.words) == 191
    assert len(emu.words) == 107 and max(emu.words) < IMEM // 4
    assert not rx.deadlines[0].pairs                        # no WAITD (header)
    rep = emu.deadlines[0]
    assert rep.period == 80 and rep.pairs and not rep.infeasible and not rep.unbounded
    assert rep.worst_slack >= 0
    moved = program("magstripe_emu", 2)
    assert moved.diagnostics == [] and min(moved.words) == 256
    assert set(moved.words).isdisjoint(rx.words)
    assert BUF > max(rx.words) and BUF + 40 <= min(moved.words)


def test_magstripe_model_format():
    """The track 2 and track 1 records against MagTek's rules: odd parity
    on every character, every data column even over SS..LRC, LSB first."""
    for track, text in ((M.TRACK2, TEXT), (M.TRACK1, "B1234^FAKE/CARD^99")):
        vals = M.record(text, track)
        assert vals[0] == track.ss and vals[-2] == track.es
        acc = 0
        for v in vals:
            acc ^= v
            bits = track.char_bits(v)
            assert sum(bits) % 2 == 1 and len(bits) == track.data_bits + 1
            assert sum(b << i for i, b in enumerate(bits[:-1])) == v
        assert acc == 0
    assert M.TRACK2.value(";") == 0x0B and M.TRACK2.value("?") == 0x0F
    assert M.TRACK1.value("%") == 0x05 and M.TRACK1.value("?") == 0x1F
    # ';' on the card: 1 1 0 1 0 (1, 2, 4, 8, P), MT p. 8's order
    assert M.TRACK2.char_bits(0x0B) == [1, 1, 0, 1, 0]


def test_magstripe_model_decoder_both_ways():
    """The model's own decoder: edges to bits (last-window reference) to
    characters, both directions, through a speed ramp."""
    for track, text in ((M.TRACK2, "0123=456"), (M.TRACK1, "B99^X/Y^1")):
        bits = M.encode(text, track, zeros=8)
        for play, rev in ((bits, False), (M.reverse_swipe(bits), True)):
            src = M.SwipeSource("IN0")
            src.send(play, 100.0, 0, speed=M.ramp(1.0, 2.0))
            got = M.intervals_to_bits(src.edges)
            d = M.decode(got, track)
            assert (d.text, d.reverse, d.errors) == (text, rev, 0)
    d = M.decode(M.encode("12", zeros=4, bad_parity=[1]))
    assert d.errors == M.E_PARITY
    assert M.decode([0] * 30).errors == M.E_NO_SS
    assert M.decode(M.encode("12", zeros=4)[:-12]).errors & M.E_NO_ES
