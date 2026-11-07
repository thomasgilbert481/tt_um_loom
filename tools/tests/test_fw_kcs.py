"""L3 for the Kansas City Standard cassette programs, on the model and the RTL.

``firmware/kcs_enc.loom`` (TX = OUT0) records host bytes as whole cycles
of 1200 Hz (a 0) and 2400 Hz (a 1), 300 baud or CUTS's 1200, with two stop
bits and a leader; ``firmware/kcs_dec.loom`` (RX = IN0) counts the
crossings of a played-back recording and frames the characters. The
references are ``tools.protomodels.kcs``: its decoder and ideal recording
judge the encoder, and ``KcsSource`` (tape speed error, edge jitter, a
dropout) feeds the decoder. Together, the encoder runs on thread 0 and the
decoder on thread 1, wired OUT0 to IN0 by the bench.

Expected values come from the specification (the model's docstring: BYTE,
February 1976) and the program headers, never from what the model or the
RTL did:

* encoder: every character decoded by the reference receiver with both
  stop bits; every edge within two clocks of the ideal recording (each bit
  whole cycles of its tone, the header's one-clock fractional-tick dither
  at most, twice for a difference of two edges), so every bit holds BYTE's
  cycle count; every half-period within one clock of 1/4800 s or 1/2400 s;
  back-to-back characters 11 bits apart; the leader as long as asked;
* decoder: the bytes and the framing and dropout bits of its header, at
  nominal speed, at 5 % slow and fast, at +-25 % with jitter (the header's
  claim), and across a dropout.

Boards. At 50 MHz one 300-baud bit is 166 667 clocks: a character is 1.8 M
clocks, far beyond the RTL budget. The RTL runs on a 2.4576 MHz board,
where the encoder's tick is 512 clocks and the decoder's 32 (the same
waveform in microseconds, twenty times fewer clocks), and mostly at CUTS's
1200 baud (2048 clocks a bit). The 50 MHz bodies and the long ones are
``model_only``. Every body is named ``test_kcs_*``: test/test_fw.py
collects the bodies of all modules into one namespace.
"""

import pytest

from tools.loomasm import assemble_file, assemble_text
from tools.loomisa import REPO, load
from tools.protomodels import kcs as K
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
IMEM = 512
ENC_HZ = 4800                           # kcs_enc.loom: a 2400 Hz half-period
DEC_HZ = 76800                          # kcs_dec.loom: 1/32 of a 1200 Hz half
FE, DROP = 0x100, 0x200                 # the decoder's error bits (header)
LEADER = 0x8000                         # the encoder's leader command (header)


class Board:
    """A clock and the ticks the program headers give for it."""

    def __init__(self, name, clock_hz, enc_tick, dec_tick):
        self.name, self.clock_hz = name, float(clock_hz)
        self.tick = {"kcs_enc": enc_tick, "kcs_dec": dec_tick}

    def cycles(self, seconds):
        return int(round(seconds * self.clock_hz))

    def __repr__(self):
        return self.name


B50 = Board("50MHz", 50_000_000, (10416, 171), (651, 11))
BRTL = Board("2MHz", DEC_HZ * 32, (512, 0), (32, 0))


def program(name, thread=0):
    """A program from ``firmware/``, moved to ``thread`` if asked."""
    path = FIRMWARE / (name + ".loom")
    if thread == 0:
        return assemble_file(path, isa=ISA, strict=True)
    text = path.read_text(encoding="utf-8")
    assert text.count(".thread 0") == 1
    text = text.replace(".thread 0", ".thread %d" % thread)
    return assemble_text(text, name + ".loom", isa=ISA, imem_words=IMEM, strict=True)


def kcs_bench(backend):
    """The FIFOs, the deadline latch and the 512-word memory; the RTL has
    them all and ``RtlBench`` takes neither keyword."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "SETPD"), imem_words=IMEM)
    return backend.bench()


def start(backend, bench, board, loads):
    """Load ``[(name, program, thread)]``, give each thread its program's
    tick on this board, run."""
    loom = backend.loom(bench, isa=ISA)
    words = {}
    for _, prog, _ in loads:
        words.update(prog.words)
    loom.load(words)
    for name, _, thread in loads:
        loom.write_csr(thread, "TICK_INT", board.tick[name][0])
        loom.write_csr(thread, "TICK_FRAC", board.tick[name][1])
    loom.run([thread for _, _, thread in loads])
    return loom


def config(baud):
    """Both programs' configuration word (headers: bit 0 = CUTS)."""
    return [1 if baud == K.CUTS else 0]


def commands(data, lead):
    """The encoder's INQ words: a leader of ``lead`` marks, the bytes."""
    return ([LEADER | lead] if lead else []) + list(data)


# ---------------------------------------------------------------- encoder
def enc_setup(backend, board):
    bench = kcs_bench(backend)
    rec = bench.add(K.EdgeRecorder("OUT0"))
    loom = start(backend, bench, board, [("kcs_enc", program("kcs_enc"), 0)])
    return bench, loom, rec


def send(bench, loom, board, baud, data, lead, tail_bits=2):
    """Configure, queue the leader and the characters, and run until the
    last has gone out: one mark bit before the leader (the header: the
    first bit is a mark, then the queue is looked at), then ``tail_bits``."""
    loom.push(0, config(baud) + commands(data, lead))
    bits = 1 + lead + len(data) * 11 + tail_bits
    bench.step(board.cycles(bits * K.bit_s(baud)))


def check_enc(rec, board, baud, data, lead):
    """The encoder's recording against BYTE and the header."""
    assert rec.edges and rec.edges[0][1] == 1           # the first is a rise
    times = rec.times(board.clock_hz)
    chars = K.decode(times, baud)
    assert [c.byte for c in chars] == list(data), [hex(c.byte) for c in chars]
    assert not any(c.framing_error for c in chars)
    clock = 1.0 / board.clock_hz
    # every bit whole cycles of its tone, every edge where the ideal
    # recording puts it (two clocks: the fractional tick's dither, twice)
    bits = K.bits_from_chars(chars, baud) + [1]
    err = K.edge_error(times, bits, chars[0].start_s, baud)
    assert err <= 2.01 * clock, "edge error %.2f us" % (err * 1e6)
    short, long_ = K.steady_half_periods(times)
    assert short and (long_ or all(b for b in bits))
    for hps, hz in ((short, K.MARK_HZ), (long_, K.SPACE_HZ)):
        assert all(abs(h - 0.5 / hz) <= 1.01 * clock for h in hps), hz
    b = K.bit_s(baud)
    k = [int(round((c.start_s - chars[0].start_s) / b)) for c in chars]
    assert k == [11 * i for i in range(len(chars))], k     # two stop bits
    # the leader: one mark bit, then ``lead`` (one more bit at most if the
    # host's words land after the first bit boundary)
    first = (chars[0].start_s - times[0]) / b
    assert abs(first - round(first)) < 0.01 and 1 + lead <= round(first) <= 2 + lead, first
    return chars


def test_kcs_enc_cuts(backend):
    """CUTS, 1200 baud: a leader of 4 marks, then 0xA5, 0x00 (eight 0 bits
    in a row) and 0xFF (a start bit alone among marks), back to back."""
    bench, loom, rec = enc_setup(backend, BRTL)
    data = [0xA5, 0x00, 0xFF]
    send(bench, loom, BRTL, K.CUTS, data, 4)
    check_enc(rec, BRTL, K.CUTS, data, 4)
    assert loom.badop() == 0


@model_only("two 300-baud characters, 0.25 M clocks, 40 s of RTL; the RTL "
            "sends 300 baud in test_kcs_together_300")
def test_kcs_enc_300(backend):
    """300 baud: a leader of 3 marks, then 0x5A and 0x81, each bit four
    cycles of 1200 Hz or eight of 2400 Hz."""
    bench, loom, rec = enc_setup(backend, BRTL)
    data = [0x5A, 0x81]
    send(bench, loom, BRTL, K.KCS, data, 3)
    check_enc(rec, BRTL, K.KCS, data, 3)
    assert loom.badop() == 0


@model_only("one CUTS character at 50 MHz, 0.6 M clocks, 95 s of RTL")
def test_kcs_enc_50mhz(backend):
    """At the 50 MHz tick (10416 + 171/256 clocks): 0x3C at CUTS's 1200
    baud, every edge within two clocks of the ideal recording."""
    bench, loom, rec = enc_setup(backend, B50)
    send(bench, loom, B50, K.CUTS, [0x3C], 1)
    check_enc(rec, B50, K.CUTS, [0x3C], 1)
    assert loom.badop() == 0


@model_only("idle marks only, 0.1 M clocks")
def test_kcs_enc_idles_on_mark(backend):
    """Configured with nothing queued, the line carries 2400 Hz: every
    half-period within one clock of 1/4800 s, and nothing decodes."""
    bench, loom, rec = enc_setup(backend, BRTL)
    loom.push(0, config(K.KCS))
    bench.step(BRTL.cycles(3 * K.bit_s(K.KCS)))
    times = rec.times(BRTL.clock_hz)
    hps = [n for _, n in K.half_periods(times)]
    assert len(hps) >= 40
    assert all(abs(h - 0.5 / K.MARK_HZ) <= 1.01 / BRTL.clock_hz for h in hps)
    assert K.decode(times) == []


# ---------------------------------------------------------------- decoder
def collect(bench, loom, board, thread, seconds, baud):
    """Run ``seconds`` and drain OUTQ[thread] every two characters' time
    (FIFO_DEPTH 4: the header asks the host to drain within four)."""
    got = []
    left = board.cycles(seconds)
    chunk = board.cycles(22 * K.bit_s(baud))
    while left > 0:
        n = min(chunk, left)
        bench.step(n)
        left -= n
        got += loom.pop_available(thread)
    return got


def dec_run(backend, board, bits, baud, **source):
    """The decoder on thread 0 with a ``KcsSource`` on IN0 that starts once
    the program runs; returns ``(words, loom)``."""
    bench = kcs_bench(backend)
    loom = start(backend, bench, board, [("kcs_dec", program("kcs_dec"), 0)])
    loom.push(0, config(baud))
    begin_us = bench.cycle / board.clock_hz * 1e6 + 50.0
    src = bench.add(K.KcsSource("IN0", board.clock_hz, bits, baud,
                                start_us=begin_us, **source))
    length = len(bits) * K.bit_s(baud) / (1 + source.get("speed", 0.0))
    words = collect(bench, loom, board, 0, length + 1e-3, baud)
    assert src.done
    return words, loom


def test_kcs_dec_slow_tape(backend):
    """CUTS from a tape 5 % slow: 0x3C and 0xC3 after a leader of 8 marks,
    two words, the bytes, no error bits."""
    bits = K.stream_bits([0x3C, 0xC3], lead=8, tail=2)
    words, loom = dec_run(backend, BRTL, bits, K.CUTS, speed=-0.05)
    assert words == [0x3C, 0xC3]
    assert loom.badop() == 0


@model_only("two characters a case, up to 0.5 M clocks each; the RTL runs "
            "CUTS 5 % slow in test_kcs_dec_slow_tape")
@pytest.mark.parametrize("baud, speed, jitter_us", [
    (K.KCS, +0.05, 0.0), (K.KCS, -0.05, 0.0), (K.CUTS, +0.05, 10.0),
    (K.KCS, +0.25, 10.0), (K.KCS, -0.25, 10.0), (K.CUTS, +0.25, 10.0),
    (K.CUTS, -0.25, 10.0)])
def test_kcs_dec_speed(backend, baud, speed, jitter_us):
    """Tape speed error alone (+-5 %) and at the header's claimed +-25 %
    with +-10 us of edge jitter: 0x00 and 0xA7 decoded clean."""
    bits = K.stream_bits([0x00, 0xA7], lead=8, tail=2)
    words, loom = dec_run(backend, BRTL, bits, baud, speed=speed,
                          jitter_us=jitter_us)
    assert words == [0x00, 0xA7]
    assert loom.badop() == 0


def test_kcs_dec_dropout(backend):
    """CUTS: 0xFF with two bits of its data missing from the tape (the
    signal gone for 1.67 ms, more than the header's 48 ticks): a word with
    the dropout bit; then 12 marks and 0x96, decoded clean. 0xFF leaves only
    marks after the gap, so no false start follows (the header's limit)."""
    b = K.bit_s(K.CUTS)
    lead = 8
    bits = [1] * lead + K.char_bits(0xFF) + [1] * 12 + K.char_bits(0x96) + [1] * 2
    words, loom = dec_run(backend, BRTL, bits, K.CUTS,
                          dropout=((lead + 3) * b, 2 * b))
    assert len(words) == 2 and words[0] & (FE | DROP) == DROP, [hex(w) for w in words]
    assert words[1] == 0x96
    assert loom.badop() == 0


@model_only("three 300-baud characters, 0.6 M clocks")
def test_kcs_dec_framing_error(backend):
    """300 baud: a 0 in the first stop bit, then a 0 in the second, each
    sets bit 8; the good character after them is clean."""
    bits = [1] * 8 + K.char_bits(0x3C, stop=(0, 1)) + [1] * 2
    bits += K.char_bits(0x81, stop=(1, 0)) + [1] * 2
    bits += K.char_bits(0x7E) + [1] * 2
    words, loom = dec_run(backend, BRTL, bits, K.KCS)
    assert words == [0x3C | FE, 0x81 | FE, 0x7E]
    assert loom.badop() == 0


@model_only("one CUTS character at 50 MHz, 0.9 M clocks")
def test_kcs_dec_50mhz(backend):
    """At the 50 MHz tick (651 + 11/256 clocks): 0x5A at 1200 baud."""
    bits = K.stream_bits([0x5A], lead=8, tail=2)
    words, loom = dec_run(backend, B50, bits, K.CUTS)
    assert words == [0x5A]


# --------------------------------------------------------------- together
def together(backend, board):
    """The encoder on thread 0 (0x000), the decoder on thread 1 (0x080),
    the bench's wire from OUT0 to IN0, and a recorder on OUT0."""
    bench = kcs_bench(backend)
    rec = bench.add(K.EdgeRecorder("OUT0"))
    bench.add(K.Wire("OUT0", "IN0"))
    enc, dec = program("kcs_enc"), program("kcs_dec", thread=1)
    assert not set(enc.words) & set(dec.words)
    loom = start(backend, bench, board, [("kcs_enc", enc, 0), ("kcs_dec", dec, 1)])
    return bench, loom, rec


def run_together(backend, baud, data, lead):
    bench, loom, rec = together(backend, BRTL)
    loom.push(1, config(baud))
    loom.push(0, config(baud) + commands(data, lead))
    bits = 2 + lead + len(data) * 11 + 2
    got = collect(bench, loom, BRTL, 1, bits * K.bit_s(baud), baud)
    assert got == list(data), [hex(w) for w in got]
    check_enc(rec, BRTL, baud, data, lead)
    assert loom.badop() == 0


def test_kcs_together_300(backend):
    """300 baud on one chip: a leader of 3 marks and 0x5A from the encoder
    to the decoder, one word with no error bits, and the recording passes
    every check of check_enc."""
    run_together(backend, K.KCS, [0x5A], 3)


def test_kcs_together_cuts(backend):
    """CUTS on one chip: a leader of 6 marks, then 0x00, 0xFF and 0x96."""
    run_together(backend, K.CUTS, [0x00, 0xFF, 0x96], 6)


# ---------------------------------------------------- the reference itself
def test_kcs_reference_cycle_counts():
    """BYTE's figures: 8 and 4 cycles a bit at 300 baud, 2 and 1 at CUTS's
    1200, so every bit is 16 or 4 ticks of 1/4800 s."""
    assert (K.cycles_per_bit(1), K.cycles_per_bit(0)) == (8, 4)
    assert (K.cycles_per_bit(1, K.CUTS), K.cycles_per_bit(0, K.CUTS)) == (2, 1)
    times, length = K.crossings(K.char_bits(0x00), K.KCS)
    assert len(times) == 8 * 9 + 16 * 2 and abs(length - 11 / 300) < 1e-12


def test_kcs_reference_round_trip():
    """The ideal recording decodes at both rates and at +-5 % speed (the
    reference clock knows the speed), and edge_error finds it exact."""
    data = [0x00, 0xFF, 0x55, 0xA7]
    for baud in (K.KCS, K.CUTS):
        bits = K.stream_bits(data, lead=4, tail=3)
        for speed in (0.0, 0.05, -0.05):
            times, _ = K.crossings(bits, baud, speed=speed)
            chars = K.decode(times, baud, speed=speed)
            assert [c.byte for c in chars] == data
            assert not any(c.framing_error for c in chars)
        times, _ = K.crossings(bits, baud)
        t0 = 4 * K.bit_s(baud)
        assert K.edge_error(times, bits[4:], t0, baud) < 1e-12


def test_kcs_checker_catches_a_missing_cycle():
    """A recording with one 2400 Hz cycle dropped from a stop bit (seven
    cycles, not BYTE's eight) still decodes, but edge_error rejects it by
    far more than the firmware's two-clock allowance."""
    bits = K.stream_bits([0x55], lead=4, tail=3)
    times, _ = K.crossings(bits, K.KCS)
    t0 = 4 * K.bit_s(K.KCS)
    i = K.bisect.bisect_left(times, t0 + 9.5 * K.bit_s(K.KCS))
    bad = times[:i] + [t - 1.0 / K.MARK_HZ for t in times[i + 2:]]
    assert [c.byte for c in K.decode(bad)] == [0x55]
    assert K.edge_error(bad, bits[4:], t0) > 50e-6
