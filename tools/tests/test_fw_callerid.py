"""L3 for the on-hook caller ID programs, on the model and the RTL.

``firmware/callerid_gen.loom`` (TX = OUT0) buffers a message from the host
and sends the channel seizure, the mark signal and the message as one
Bell 202 FSK burst; ``firmware/callerid_rx.loom`` (RX = IN0) demodulates a
burst, finds the seizure and the mark signal, parses the message and checks
its checksum. The references are ``tools.protomodels.callerid`` (message
builder and parser, burst bits, the reference burst decoder, the host
words of the program headers) and ``tools.protomodels.bell202`` (the FSK
source, the zero-crossing receiver, the tone and phase checks).

Expected values come from ETSI EN 300 659-1 and -3 (the model's docstring)
and the program headers, never from what the model or the RTL did:

* the generator: the reference decoder finds the seizure and mark signal
  of the configured lengths (to a quarter of a bit), the octets that
  ``build_message`` makes, every octet 10 bits after the one before (clause
  5.3: continuous), one to ten mark bits after the checksum, every steady
  half-period within +-1 % of its tone, and every edge within two clocks of
  the ideal continuous-phase wave;
* the receiver: the OUTQ words of its header for a good message, a bad
  checksum, a mark signal too short, and a carrier that stops early.

Boards. A full-length burst is 800 to 900 bits. The generator's schedule is
proved for a tick of 112 clocks, so a bit is 14 784 clocks at least and a
full burst 13 M clocks: 15 minutes on the model, half an hour on the RTL. So the
generator runs on a 17.74 MHz board (158 400 Hz x 112) with the shortened
seizure and mark signal of its configuration word (a test option of the
header, not a standard length); the receiver, which has no deadline, runs
on a 2.53 MHz board (tick 16 clocks: the same waveform in microseconds) and
takes the standard 300 and 180 bits there. Every body is named
``test_callerid_*``.
"""

import pytest

from tools.loomasm import assemble_file, assemble_text
from tools.loomisa import REPO, load
from tools.protomodels import bell202 as F
from tools.protomodels import callerid as C
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
IMEM = 512
TICK_HZ = 132 * 1200                    # the program headers: 1/132 of a bit
TAIL = 4                                # callerid_gen.loom's .equ TAIL


class Board:
    """A clock and the tick the program headers give for it."""

    def __init__(self, name, clock_hz, tick):
        self.name, self.clock_hz, self.tick = name, float(clock_hz), tick

    def cycles(self, seconds):
        return int(round(seconds * self.clock_hz))

    def __repr__(self):
        return self.name


B50 = Board("50MHz", 50_000_000, (315, 168))
BGEN = Board("17MHz", TICK_HZ * 112, (112, 0))      # the generator's .tick
BRX = Board("2MHz", TICK_HZ * 16, (16, 0))           # the receiver alone

# A message with only the number, one digit: the shortest MDMF there is
# (a parameter holds at least one octet, EN 300 659-1 clause 5.1).
NUMBER_ONLY = [(C.NUMBER, b"5")]
FULL = [(C.DATE_TIME, C.date_time(11, 7, 13, 45)),
        (C.NUMBER, b"9195551234"),
        (C.NAME, b"GILBERT THOMAS")]


def program(name, thread=0):
    """A program from ``firmware/``, moved to ``thread`` if asked."""
    path = FIRMWARE / (name + ".loom")
    if thread == 0:
        return assemble_file(path, isa=ISA, strict=True)
    text = path.read_text(encoding="utf-8")
    assert text.count(".thread 0") == 1
    text = text.replace(".thread 0", ".thread %d" % thread)
    return assemble_text(text, name + ".loom", isa=ISA, imem_words=IMEM, strict=True)


def cid_bench(backend):
    """The FIFOs, the deadline latch, the data memory and 512 words; the
    RTL has them all and ``RtlBench`` takes neither keyword."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "SETPD", "DMEM"), imem_words=IMEM)
    return backend.bench()


def start(backend, bench, board, loads):
    """Load ``[(program, thread)]``, give each thread the board's tick, run."""
    loom = backend.loom(bench, isa=ISA)
    words = {}
    for prog, _ in loads:
        assert not set(words) & set(prog.words)
        words.update(prog.words)
    loom.load(words)
    for _, thread in loads:
        loom.write_csr(thread, "TICK_INT", board.tick[0])
        loom.write_csr(thread, "TICK_FRAC", board.tick[1])
    loom.run([thread for _, thread in loads])
    return loom


def collect(bench, loom, board, thread, seconds):
    """Run ``seconds`` and drain OUTQ[thread] every two characters' time
    (the receiver's header: drain within four)."""
    got = []
    left = board.cycles(seconds)
    chunk = board.cycles(20 * F.BIT_S)
    while left > 0:
        n = min(chunk, left)
        bench.step(n)
        left -= n
        got += loom.pop_available(thread)
    return got


def burst_seconds(message, seizure_bits, mark_bits, tail=TAIL):
    return (seizure_bits + mark_bits + 10 * len(message) + tail) * F.BIT_S


# -------------------------------------------------------------- generator
def check_gen(times, board, message, seizure_bits, mark_bits):
    """A recorded burst against EN 300 659-1 clauses 5.2 and 5.3, the tone
    tolerances of HCF_SPEC-54 (bell202.py) and continuous phase."""
    burst = C.decode_burst(times)
    assert burst.octets == message, burst.octets.hex()
    assert burst.framing_errors == 0
    assert burst.continuous                         # clause 5.3
    assert abs(burst.seizure_bits - seizure_bits) < 0.5, burst.seizure_bits
    assert abs(burst.mark_bits - mark_bits) < 0.5, burst.mark_bits
    tail = burst.tail_s / F.BIT_S                   # the last edge, in bits
    assert C.TAIL_MIN - 0.5 <= tail <= C.TAIL_MAX, tail
    mark, space = F.steady_half_periods(times)
    clock = 1.0 / board.clock_hz
    for hps, hz in ((mark, F.MARK_HZ), (space, F.SPACE_HZ)):
        nominal = 0.5 / hz
        assert hps and all(abs(h / nominal - 1) <= F.TOLERANCE for h in hps)
        assert max(abs(h - nominal) for h in hps) <= 1.01 * clock
    # continuous phase over the whole burst; the first edge is the carrier
    # switching on (an ordinary SETP, a fraction of a tick before the grid)
    bits = C.burst_bits(message, seizure_bits=seizure_bits,
                        mark_bits=mark_bits, tail=TAIL)
    _, err = F.check_phase_continuity(times[1:], bits, times[0])
    assert err <= 2 * clock, "phase error %.2f us" % (err * 1e6)
    return burst


def gen_run(backend, params, seizure_blocks, mark_blocks):
    """The generator alone on thread 0, a recorder on OUT0."""
    bench = cid_bench(backend)
    rec = bench.add(F.EdgeRecorder("OUT0"))
    loom = start(backend, bench, BGEN, [(program("callerid_gen"), 0)])
    loom.push(0, [C.gen_config(seizure_blocks, mark_blocks)])
    loom.push(0, C.gen_words(params))
    message = C.build_message(params)
    secs = burst_seconds(message, 10 * seizure_blocks, 10 * mark_blocks) + 1e-3
    got = collect(bench, loom, BGEN, 0, secs)
    times = rec.times(BGEN.clock_hz)
    check_gen(times, BGEN, message, 10 * seizure_blocks, 10 * mark_blocks)
    assert got == [message[1]]                      # the header: the length sent
    assert loom.badop() == 0
    return times


@model_only("19 octets after a 70-bit preamble, 3.9 M clocks, 4 minutes of "
            "model and 10 of RTL; the RTL runs the generator in "
            "test_callerid_together_number_only")
def test_callerid_gen_mdmf(backend):
    """Date and time, number and name, the generator to the reference
    decoder: a 40-bit seizure and a 30-bit mark signal (four and three
    blocks: the counting the standard's 30 and 18 blocks use), the octets
    of build_message, continuous, the tail, tones and phase."""
    gen_run(backend, [(C.DATE_TIME, C.date_time(11, 7, 9, 5)),
                      (C.NUMBER, b"5"), (C.NAME, b"A")], 4, 3)


# --------------------------------------------------------------- receiver
def rx_run(backend, board, bits, config, *, offset=0.0, jitter_us=0.0, seed=30):
    """The receiver on thread 0, an FskSource on IN0 sending ``bits`` once
    the configuration word is in; returns ``(words, loom)``."""
    bench = cid_bench(backend)
    loom = start(backend, bench, board, [(program("callerid_rx"), 0)])
    loom.push(0, [config])
    begin_us = bench.cycle / board.clock_hz * 1e6 + 200.0
    src = bench.add(F.FskSource("IN0", board.clock_hz, bits, offset=offset,
                                jitter_us=jitter_us, seed=seed, start_us=begin_us))
    length = len(bits) * F.BIT_S / (1 + offset)
    words = collect(bench, loom, board, 0, length + 3e-3)  # and the silence after
    assert src.done
    return words, loom


def test_callerid_rx_number_only(backend):
    """Model to receiver: a message with only the number, after a 20-bit
    seizure and a 20-bit mark signal (SMIN 2, MMIN 15): the header's words,
    checksum good."""
    msg = C.build_message(NUMBER_ONLY)
    bits = C.burst_bits(msg, seizure_bits=20, mark_bits=20)
    words, loom = rx_run(backend, BRX, bits, C.rx_config(2, 15))
    assert words == C.rx_words(msg)
    assert words[-1] == C.END | C.GOOD
    assert loom.badop() == 0


@model_only("a full-length burst, 0.9 k bits, 1.9 M clocks: 2 minutes of "
            "model, 5 of RTL")
@pytest.mark.parametrize("marks, offset, jitter_us", [
    (180, -0.01, 10.0), (80, +0.01, 0.0), (155, 0.0, 10.0)])
def test_callerid_rx_full_mdmf(backend, marks, offset, jitter_us):
    """The standard burst: 300 seizure bits, a mark signal of 180, 80 or
    155 bits (clause 5.2: 180 +- 25 or 80 +- 25), the date, a 10-digit
    number and a 14-letter name, at +-1 % with and without 10 us of jitter,
    with the suggested configuration (SMIN 10, MMIN 50)."""
    msg = C.build_message(FULL)
    bits = C.burst_bits(msg, mark_bits=marks)
    words, loom = rx_run(backend, BRX, bits, C.rx_config(10, 50),
                         offset=offset, jitter_us=jitter_us)
    assert words == C.rx_words(msg)
    assert C.parse_message(msg).params == FULL
    assert loom.badop() == 0


SHORT = dict(seizure_bits=20, mark_bits=20)         # with rx_config(2, 15)


@model_only("the RTL runs the receiver in test_callerid_rx_number_only and "
            "the together scenario; 0.2 M clocks")
def test_callerid_rx_bad_checksum(backend):
    """The number-only message with one checksum bit flipped: every field
    is still reported, and the end word says the checksum is bad (EN 300
    659-1 clause 5.2: the terminal discards it, here the host)."""
    msg = C.build_message(NUMBER_ONLY, corrupt=0x01)
    assert not C.parse_message(msg).checksum_ok
    words, loom = rx_run(backend, BRX, C.burst_bits(msg, **SHORT), C.rx_config(2, 15))
    assert words == C.rx_words(msg)
    assert words[-1] == C.END | C.BAD_CHECKSUM
    assert loom.badop() == 0


@model_only("0.2 M clocks")
def test_callerid_rx_mark_signal_too_short(backend):
    """A seizure, then only 10 mark bits where the receiver asks for 15:
    not a message, so nothing at all reaches the host."""
    msg = C.build_message(NUMBER_ONLY)
    bits = C.burst_bits(msg, seizure_bits=20, mark_bits=10)
    words, loom = rx_run(backend, BRX, bits, C.rx_config(2, 15))
    assert words == []
    assert loom.badop() == 0


@model_only("0.15 M clocks")
def test_callerid_rx_carrier_lost(backend):
    """The burst stops after the message type, the length and the
    parameter type (two mark bits, then silence): those three words, then
    the end word 'carrier lost before the checksum'."""
    msg = C.build_message(NUMBER_ONLY)
    bits = C.seizure(20) + [1] * 20
    for octet in msg[:3]:
        bits += F.char_bits(octet, F.NONE)
    words, loom = rx_run(backend, BRX, bits + [1, 1], C.rx_config(2, 15))
    assert words == [C.TYPE | msg[0], C.LENGTH | msg[1], C.PTYPE | msg[2],
                     C.END | C.CARRIER_LOST]
    assert loom.badop() == 0


# --------------------------------------------------------------- together
def test_callerid_together_number_only(backend):
    """The generator on thread 0 (0x000), the receiver on thread 1 (0x080),
    the bench's wire from OUT0 to IN0: a number-only message after a
    10-bit seizure and a 10-bit mark signal (the configuration's shortest;
    receiver SMIN 1, MMIN 8). The receiver's words are the header's for the
    message, checksum good; the generator's burst passes check_gen and its
    OUTQ word is the length sent."""
    bench = cid_bench(backend)
    rec = bench.add(F.EdgeRecorder("OUT0"))
    bench.add(F.Wire("OUT0", "IN0"))
    gen, rx = program("callerid_gen"), program("callerid_rx", thread=1)
    loom = start(backend, bench, BGEN, [(gen, 0), (rx, 1)])
    loom.push(1, [C.rx_config(1, 8)])
    loom.push(0, [C.gen_config(1, 1)] + C.gen_words(NUMBER_ONLY))
    msg = C.build_message(NUMBER_ONLY)
    got = collect(bench, loom, BGEN, 1, burst_seconds(msg, 10, 10) + 1.5e-3)
    assert got == C.rx_words(msg)
    assert loom.pop_available(0) == [msg[1]]
    check_gen(rec.times(BGEN.clock_hz), BGEN, msg, 10, 10)
    assert loom.badop() == 0


# ---------------------------------------------------- the reference itself
def test_callerid_reference_message():
    """build_message and parse_message invert each other, the checksum
    makes the sum 0 mod 256, and the length counts the parameters only."""
    msg = C.build_message(FULL)
    assert msg[0] == C.MDMF and msg[1] == len(msg) - 3
    assert sum(msg) & 0xFF == 0
    parsed = C.parse_message(msg)
    assert parsed.params == FULL and parsed.checksum_ok
    assert msg[2:4] == bytes([C.DATE_TIME, 8]) and msg[4:12] == b"11071345"


def test_callerid_reference_burst():
    """The reference decoder on ideal continuous-phase FSK at the standard
    lengths (bell202.fsk_crossings): 300 seizure bits, the mark signal,
    the octets, continuous, at 0 and +-1 %."""
    msg = C.build_message(FULL)
    for marks in (180, 80):
        for offset in (0.0, 0.01, -0.01):
            bits = C.burst_bits(msg, mark_bits=marks)
            times, _ = F.fsk_crossings(bits, offset=offset)
            b = C.decode_burst([0.0] + times)
            scale = 1 + offset
            assert b.octets == msg and b.continuous and b.framing_errors == 0
            assert abs(b.seizure_bits * scale - 300) < 0.5, b.seizure_bits
            assert abs(b.mark_bits * scale - marks) < 0.5, b.mark_bits


def test_callerid_reference_words():
    """The host words of the headers: a parameter header carries bit 9,
    the end word bit 8 and the type; the receiver tags every octet."""
    assert C.gen_words(NUMBER_ONLY) == [0x202, ord("5"), 0x180]
    msg = C.build_message(NUMBER_ONLY)
    assert C.rx_words(msg) == [0x180, 0x203, 0x302, 0x401, 0x500 | ord("5"), 0x600]
