"""L3 for the Bell 202 FSK modem programs, on the model and the RTL.

``firmware/bell202_mod.loom`` (TX = OUT0) synthesises the two tones as a
phase-continuous square wave and frames host bytes as 8N1 or 8O1;
``firmware/bell202_demod.loom`` (RX = IN0) times the zero crossings and
recovers the characters. The references are ``tools.protomodels.bell202``:
its demodulator, phase-continuity checker and half-period analysis judge
the modulator, and ``FskSource`` (frequency offset and edge jitter) feeds
the demodulator. Together, the modulator runs on thread 0 and the
demodulator on thread 1, wired OUT0 to IN0 by the bench.

Expected values come from the specifications (the model's docstring) and
the program headers, never from what the model or the RTL did:

* modulator: every character decoded by the reference receiver; every
  steady half-period within +-1 % of 1/2400 s (mark) or 1/4400 s (space)
  (HCF_SPEC-54); continuous phase: every edge within two clocks of where
  an ideal continuous-phase modulator puts it (the header: the crossings
  all fall on ticks, so only the fractional tick's dither is left);
  back-to-back characters 10 or 11 bits apart on the fitted bit grid;
* demodulator: the bytes, the framing and parity error bits of its header,
  for a source at +1 % and -1 % (HART's tolerance), with edge jitter, and
  after a HART preamble of 0xFF characters.

Boards. At 50 MHz a tick is 315 + 168/256 clocks and a bit 41 667 clocks,
so one 11-bit character is 0.46 M clocks, 70 s of RTL simulation. The RTL
runs on a 16.47 MHz board (158 400 Hz x 104), where a tick is 104 clocks
(the modulator is proved for 100): the same waveform in microseconds, three
times fewer clocks. The 50 MHz bodies and the longer ones are
``model_only``. Every body here is named ``test_bell202_*``: test/test_fw.py
collects the bodies of all modules into one namespace.
"""

import pytest

from tools.loomasm import assemble_file, assemble_text
from tools.loomisa import REPO, load
from tools.protomodels import bell202 as F
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
IMEM = 512
TICK_HZ = 132 * 1200                    # the program headers: 1/132 of a bit
TICK_S = 1.0 / TICK_HZ


class Board:
    """A clock and the tick the program headers give for it."""

    def __init__(self, name, clock_hz, tick):
        self.name, self.clock_hz, self.tick = name, float(clock_hz), tick

    def cycles(self, seconds):
        return int(round(seconds * self.clock_hz))

    def __repr__(self):
        return self.name


B50 = Board("50MHz", 50_000_000, (315, 168))
BRTL = Board("16MHz", TICK_HZ * 104, (104, 0))


def program(name, thread=0):
    """A program from ``firmware/``, moved to ``thread`` if asked."""
    path = FIRMWARE / (name + ".loom")
    if thread == 0:
        return assemble_file(path, isa=ISA, strict=True)
    text = path.read_text(encoding="utf-8")
    assert text.count(".thread 0") == 1
    text = text.replace(".thread 0", ".thread %d" % thread)
    return assemble_text(text, name + ".loom", isa=ISA, imem_words=IMEM, strict=True)


def fsk_bench(backend):
    """The FIFOs, the deadline latch and the 512-word memory; the RTL has
    them all and ``RtlBench`` takes neither keyword."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "SETPD"), imem_words=IMEM)
    return backend.bench()


def start(backend, bench, board, loads):
    """Load ``[(program, thread)]``, give each thread the board's tick, run."""
    loom = backend.loom(bench, isa=ISA)
    words = {}
    for prog, _ in loads:
        words.update(prog.words)
    loom.load(words)
    for _, thread in loads:
        loom.write_csr(thread, "TICK_INT", board.tick[0])
        loom.write_csr(thread, "TICK_FRAC", board.tick[1])
    loom.run([thread for _, thread in loads])
    return loom


def words_of(data, framing):
    """The modulator's INQ words (its header: bit 8 selects 8O1)."""
    return [b | (0x100 if framing == F.ODD else 0) for b in data]


def nbits(framing):
    return 11 if framing == F.ODD else 10


# -------------------------------------------------------------- modulator
def mod_setup(backend, board):
    bench = fsk_bench(backend)
    rec = bench.add(F.EdgeRecorder("OUT0"))
    loom = start(backend, bench, board, [(program("bell202_mod"), 0)])
    return bench, loom, rec


def mod_times(rec, board):
    """The edge times after the program's first ``SETP TX, 1`` (the level
    the oscillator starts from, not a crossing it made)."""
    assert rec.edges and rec.edges[0][1] == 1
    return rec.times(board.clock_hz)[1:]


def send(bench, loom, board, data, framing, tail_bits=2.5):
    """Queue the characters and run until the last has gone out: at most
    one bit of mark before the first (the header: an empty queue is
    looked at again at the next bit boundary), then ``tail_bits``."""
    loom.push(0, words_of(data, framing))
    bench.step(board.cycles((len(data) * nbits(framing) + 1 + tail_bits) * F.BIT_S))


def check_mod(times, board, data, framing, back_to_back=True):
    """The modulator's output against HCF_SPEC-54 and the header."""
    chars = F.demodulate(times, framing)
    assert [c.byte for c in chars] == list(data), [hex(c.byte) for c in chars]
    assert not any(c.framing_error or c.parity_error for c in chars)
    # steady tones: within +-1 %, and (the header) exact on the fractional
    # tick grid, so within one clock of the exact length
    mark, space = F.steady_half_periods(times)
    assert mark and space
    clock = 1.0 / board.clock_hz
    for hps, hz in ((mark, F.MARK_HZ), (space, F.SPACE_HZ)):
        nominal = 0.5 / hz
        assert all(abs(h / nominal - 1) <= F.TOLERANCE for h in hps)
        worst = max(abs(h - nominal) for h in hps)
        assert worst <= 1.01 * clock, (hz, worst * 1e6)
    # continuous phase: the header says every ideal crossing is on a tick,
    # so only the fractional tick's one-clock dither is left
    bits = F.bits_from_chars(chars, framing)
    t0, err = F.check_phase_continuity(times, bits, chars[0].start_s)
    assert err <= 2 * clock, "phase error %.2f us" % (err * 1e6)
    if back_to_back:                    # the fitted grid: one stop bit between
        k = [int(round((c.start_s - chars[0].start_s) / F.BIT_S)) for c in chars]
        assert k == [i * nbits(framing) for i in range(len(chars))], k
    return chars, err


@model_only("one 8O1 character at 50 MHz, 0.58 M clocks, 90 s of RTL; the "
            "RTL sends two on the 16 MHz board in the together scenario")
def test_bell202_mod_50mhz(backend):
    """At the 50 MHz tick (315 + 168/256): 0x55 in 8O1 (alternating bits, a
    tone change at every boundary), decoded by the reference receiver, each
    steady tone within one clock of nominal, every edge within two clocks
    of the ideal continuous-phase wave."""
    bench, loom, rec = mod_setup(backend, B50)
    send(bench, loom, B50, [0x55], F.ODD)
    check_mod(mod_times(rec, B50), B50, [0x55], F.ODD)
    assert loom.badop() == 0


@model_only("three 8N1 characters, 0.45 M clocks, 70 s of RTL; the RTL "
            "runs the modulator in the together scenario")
def test_bell202_mod_8n1_back_to_back(backend):
    """0x00, 0xFF and 0xA7 in 8N1, queued at once: back to back with one
    stop bit, each checked as above (0x00 is nine space bits in a row, 0xFF
    a start bit alone in mark)."""
    bench, loom, rec = mod_setup(backend, BRTL)
    data = [0x00, 0xFF, 0xA7]
    send(bench, loom, BRTL, data, F.NONE)
    check_mod(mod_times(rec, BRTL), BRTL, data, F.NONE)
    assert loom.badop() == 0


@model_only("idle mark only; 0.1 M clocks")
def test_bell202_mod_idles_on_mark(backend):
    """With nothing queued the line carries a steady 1200 Hz tone: every
    half-period within one clock of 1/2400 s."""
    bench, loom, rec = mod_setup(backend, BRTL)
    bench.step(BRTL.cycles(6 * F.BIT_S))
    times = mod_times(rec, BRTL)
    hps = [n for _, n in F.half_periods(times)]
    assert len(hps) >= 10
    assert all(abs(h - 0.5 / F.MARK_HZ) <= 1.01 / BRTL.clock_hz for h in hps)
    assert F.demodulate(times) == []


# ------------------------------------------------------------ demodulator
FE, PE = 0x100, 0x200                   # the demodulator's error bits (header)


def config(framing):
    """The demodulator's configuration word (header: bit 0 = 8O1)."""
    return [1 if framing == F.ODD else 0]


def collect(bench, loom, board, thread, seconds):
    """Run ``seconds`` and drain OUTQ[thread] every two characters' time
    (FIFO_DEPTH 4: the header asks the host to drain within four)."""
    got = []
    left = board.cycles(seconds)
    chunk = board.cycles(20 * F.BIT_S)
    while left > 0:
        n = min(chunk, left)
        bench.step(n)
        left -= n
        got += loom.pop_available(thread)
    return got


def expected(data, errors=None):
    """The OUTQ words for ``data``; ``errors`` maps an index to its bits."""
    errors = errors or {}
    return [b | errors.get(i, 0) for i, b in enumerate(data)]


def demod_run(backend, board, bits, framing, *, offset=0.0, jitter_us=0.0,
              seed=202):
    """Run the demodulator on thread 0 with an ``FskSource`` on IN0 that
    starts once the program runs; returns ``(words, loom)``."""
    bench = fsk_bench(backend)
    loom = start(backend, bench, board, [(program("bell202_demod"), 0)])
    loom.push(0, config(framing))
    begin_us = bench.cycle / board.clock_hz * 1e6 + 50.0
    src = bench.add(F.FskSource("IN0", board.clock_hz, bits, offset=offset,
                                jitter_us=jitter_us, seed=seed, start_us=begin_us))
    length = len(bits) * F.BIT_S / (1 + offset)
    words = collect(bench, loom, board, 0, length + 100e-6)
    assert src.done
    return words, loom


def test_bell202_demod_slow_source(backend):
    """A source 1 % slow (HART's limit) sends 0xA7 in 8N1 after three bits
    of mark: one word, the byte, no error bits."""
    bits = F.stream_bits([0xA7], F.NONE, lead=3, tail=2)
    words, loom = demod_run(backend, BRTL, bits, F.NONE, offset=-0.01)
    assert words == expected([0xA7])
    assert loom.badop() == 0


@model_only("two characters a case, 0.37 M clocks each; the RTL runs the "
            "-1 % source in test_bell202_demod_slow_source")
@pytest.mark.parametrize("offset, jitter_us, framing", [
    (+0.01, 0.0, F.ODD), (-0.01, 0.0, F.ODD), (+0.01, 10.0, F.NONE),
    (-0.01, 10.0, F.ODD)])
def test_bell202_demod_tolerance(backend, offset, jitter_us, framing):
    """+-1 % on the tones and the bit rate together, with and without +-10
    us of edge jitter (4 % of a space half-period): 0x00 then 0x5A, decoded
    with no error bits."""
    data = [0x00, 0x5A]
    bits = F.stream_bits(data, framing, lead=3, tail=2)
    words, loom = demod_run(backend, BRTL, bits, framing, offset=offset,
                            jitter_us=jitter_us)
    assert words == expected(data)
    assert loom.badop() == 0


@model_only("three characters, 0.56 M clocks")
def test_bell202_demod_reports_errors(backend):
    """8O1: a character whose stop bit is 0 sets bit 8, one with even
    parity sets bit 9, and the good character after them is clean."""
    bits = [1] * 3 + F.char_bits(0x3C, F.ODD, stop=0) + [1] * 2
    bits += F.char_bits(0x81, F.ODD, parity=0) + [1]     # two 1s: odd wants 1
    bits += F.char_bits(0x7E, F.ODD) + [1] * 2
    words, loom = demod_run(backend, BRTL, bits, F.ODD)
    assert words == [0x3C | FE, 0x81 | PE, 0x7E]
    assert loom.badop() == 0


# --------------------------------------------------------------- together
def together(backend, board):
    """The modulator on thread 0 (0x000), the demodulator on thread 1
    (0x080), the bench's wire from OUT0 to IN0, and a recorder on OUT0."""
    bench = fsk_bench(backend)
    rec = bench.add(F.EdgeRecorder("OUT0"))
    bench.add(F.Wire("OUT0", "IN0"))
    mod, dem = program("bell202_mod"), program("bell202_demod", thread=1)
    assert not set(mod.words) & set(dem.words)
    loom = start(backend, bench, board, [(mod, 0), (dem, 1)])
    return bench, loom, rec


def test_bell202_together(backend):
    """Two 8O1 characters, 0x55 and 0xA7, from the modulator to the
    demodulator on one chip: the demodulator's two words are the bytes with
    no error bits, and the modulator's waveform passes every check of
    check_mod (reference receiver, tones, phase, one stop bit)."""
    bench, loom, rec = together(backend, BRTL)
    loom.push(1, config(F.ODD))
    data = [0x55, 0xA7]
    loom.push(0, words_of(data, F.ODD))
    got = collect(bench, loom, BRTL, 1, (len(data) * 11 + 2.5) * F.BIT_S)
    assert got == expected(data)
    check_mod(mod_times(rec, BRTL), BRTL, data, F.ODD)
    assert loom.badop() == 0


@model_only("six 8O1 characters, 0.95 M clocks, 2.5 minutes of RTL")
def test_bell202_hart_preamble(backend):
    """A HART-shaped message: the minimum preamble of five 0xFF characters
    (HCF_SPEC-81), then a short-frame start delimiter 0x02 (the rest of a
    message is ordinary data), 8O1, through both programs. A 0xFF is a start
    bit alone in a sea of mark (the parity bit is 1, so is the stop bit),
    which is what the preamble is for: the receiver must pick out each one."""
    bench, loom, rec = together(backend, BRTL)
    loom.push(1, config(F.ODD))
    data = [0xFF] * 5 + [0x02]
    loom.push(0, words_of(data, F.ODD))
    got = collect(bench, loom, BRTL, 1, (len(data) * 11 + 2.5) * F.BIT_S)
    assert got == expected(data)
    check_mod(mod_times(rec, BRTL), BRTL, data, F.ODD)
    assert loom.badop() == 0


# ---------------------------------------------------- the reference itself
def test_bell202_reference_round_trip():
    """The reference modulator and receiver agree at 0 and +-1 %, and the
    checker finds an ideal stream continuous to within rounding."""
    data = [0x00, 0xFF, 0x55, 0xA7]
    for framing in (F.NONE, F.ODD):
        bits = F.stream_bits(data, framing, lead=3, tail=3)
        for offset in (0.0, 0.01, -0.01):
            times, _ = F.fsk_crossings(bits, offset=offset)
            chars = F.demodulate(times, framing)
            assert [c.byte for c in chars] == data
            assert not any(c.framing_error or c.parity_error for c in chars)
            t0 = 3 * F.BIT_S / (1 + offset)
            _, err = F.check_phase_continuity(times, bits[3:], t0, offset=offset)
            assert err < 1e-7


def test_bell202_checker_catches_a_phase_jump():
    """A modulator that restarts its square wave at every bit boundary (so
    the phase jumps there) still decodes, but the checker rejects it by far
    more than the firmware's one-tick allowance."""
    bits = F.stream_bits([0x55, 0x3C], F.NONE, lead=3, tail=3)
    times, start = [], 0.0
    for bit in bits:
        half = 0.5 / F.tone_hz(bit)
        times.append(start)                     # a new half-cycle starts here
        x = half
        while x < F.BIT_S - 1e-9:
            times.append(start + x)
            x += half
        start += F.BIT_S
    clean = [t for i, t in enumerate(times) if i == 0 or t - times[i - 1] > 1e-9]
    assert [c.byte for c in F.demodulate(clean)] == [0x55, 0x3C]
    _, err = F.check_phase_continuity(clean, bits[3:], 3 * F.BIT_S)
    assert err > 10 * TICK_S
