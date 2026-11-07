"""L3 for the AX.25 over Bell 202 AFSK programs, on the model and the RTL.

``firmware/ax25_tx.loom`` (threads 0 and 1; TX = OUT0, the NRZI baseband
on LINE = OUT1) builds a UI frame from host words and sends it as phase-
continuous AFSK; ``firmware/ax25_rx.loom`` (thread 3; RX = IN0) receives
AFSK and reports frames on OUTQ[3]. The references are
``tools.protomodels.ax25`` (frame builder, FCS-16, the reference decoder
chain and the timing checker) and ``tools.protomodels.bell202`` (the AFSK
source with frequency offset and jitter, the edge recorder, the wire).

Expected values come from the specifications (the model's docstring) and
the program headers, never from what the model or the RTL did:

* transmitter: the reference receiver finds exactly one frame, good, whose
  octets are the AX.25 2.2 UI frame the model builds (address encoding
  3.12, control 0x03, PID 0xF0) and whose FCS is ISO 3309's (the model's
  CRC is checked against the catalogue check value); the bits between the
  flags are exactly the zero-bit insertion of 3.6; every steady half-
  period within +-1 % (and, the header, within one clock) of 1/2400 s or
  1/4400 s; every edge within two clocks of the ideal continuous-phase
  wave on one bit grid (the header: all crossings fall on ticks);
* receiver: the octets of every frame, the FCS octets with them, then the
  report its header defines (bit 15, 14 bad FCS, 13 not whole, 12 abort,
  11:0 the count), for a source at +-1 % with jitter, a bad FCS, an abort
  and a flag shared by two frames.

Boards. A bit is 132 ticks. At 50 MHz (tick 315 + 168/256) a 20-octet
frame is about 8 M clocks, so the RTL runs on a 7.6 MHz board (158 400 Hz
x 48, tick 48 clocks: ax25_tx.loom is proved for 44, and its staged edges
land exactly on their ticks from 46 on, SEMANTICS 6.10 rule 1, see its header):
the same waveform in microseconds. Every body here is named
``test_ax25_*``: test/test_fw.py collects all modules into one namespace.
"""

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels import ax25 as A
from tools.protomodels import bell202 as B
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
TICK_HZ = 132 * 1200                    # the program headers: 1/132 of a bit


class Board:
    """A clock and the tick the program headers give for it."""

    def __init__(self, name, clock_hz, tick):
        self.name, self.clock_hz, self.tick = name, float(clock_hz), tick

    def cycles(self, seconds):
        return int(round(seconds * self.clock_hz))

    def __repr__(self):
        return self.name


B50 = Board("50MHz", 50_000_000, (315, 168))
BRTL = Board("7.6MHz", TICK_HZ * 48, (48, 0))

TX = assemble_file(FIRMWARE / "ax25_tx.loom", isa=ISA, strict=True)
RX = assemble_file(FIRMWARE / "ax25_rx.loom", isa=ISA, strict=True)


def ax_bench(backend):
    """FIFOs, the bit engine with its encoders, the deadline latch, LD/ST
    and the 512-word memory; the RTL has them all."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "BE", "BEENC", "SETPD", "DMEM"),
                             imem_words=512)
    return backend.bench()


def start(backend, bench, board, threads):
    """Load both images, give ``threads`` the board's tick, run them."""
    loom = backend.loom(bench, isa=ISA)
    words = dict(TX.words)
    words.update(RX.words)
    loom.load(words)
    for t in threads:
        loom.write_csr(t, "TICK_INT", board.tick[0])
        loom.write_csr(t, "TICK_FRAC", board.tick[1])
    loom.run(threads)
    return loom


def run_drain(bench, loom, board, seconds, thread=3):
    """Run ``seconds``, draining OUTQ[thread] every 16 bits (the RX header:
    within four octets)."""
    got = []
    left = board.cycles(seconds)
    chunk = board.cycles(16 * A.BIT_S)
    while left > 0:
        n = min(chunk, left)
        bench.step(n)
        left -= n
        got += loom.pop_available(thread)
    return got


def report(octets, bad=False, partial=False, abort=False):
    """The RX report word (its header)."""
    return (0x8000 | (0x4000 if bad else 0) | (0x2000 if partial else 0)
            | (0x1000 if abort else 0) | octets)


def expected_rx(frame):
    """A good frame on OUTQ[3]: its octets, the FCS octets, the report."""
    full = A.with_fcs(frame)
    return list(full) + [report(len(full))]


# ------------------------------------------------------------ transmitter
def tx_run(backend, board, dest, src, info, *, rx=True, tail_bits=24):
    """The transmitter on threads 0 and 1 (and the receiver on thread 3,
    wired OUT0 to IN0, if ``rx``) sending one UI frame. Returns ``(times,
    rx words, loom)``; ``times`` are the edges after the program's first
    ``SETP TX, 1`` (the level the oscillator starts from)."""
    bench = ax_bench(backend)
    rec = bench.add(B.EdgeRecorder("OUT0"))
    if rx:
        bench.add(B.Wire("OUT0", "IN0"))
    loom = start(backend, bench, board, [0, 1, 3] if rx else [0, 1])
    words = A.host_words(dest[0], dest[1], src[0], src[1], info)
    loom.push(0, words)
    frame = A.ui_frame(dest[0], dest[1], src[0], src[1], info)
    nbits = len(A.stuff(A.octet_bits(A.with_fcs(frame))))
    # the header: one word a bit while idle flags go out, the frame two
    # flags after the last word; then the closing flag and a margin
    lead = len(words) + 8 + 16
    got = run_drain(bench, loom, board, (lead + nbits + 8 + tail_bits) * A.BIT_S)
    assert rec.edges and rec.edges[0][1] == 1
    return rec.times(board.clock_hz)[1:], got, loom, frame


def check_tx(times, board, frame):
    """The audio against AX.25 2.2 and Bell 202 (module docstring)."""
    t0, bits, frames = A.decode(times)
    good = [f for f in frames if f.good]
    assert len(frames) == 1 and len(good) == 1, [(f.good, f.octets) for f in frames]
    f = good[0]
    full = A.with_fcs(frame)
    assert f.octets == full, (f.octets.hex(), full.hex())
    stuffed = A.FLAG_BITS + A.stuff(A.octet_bits(full)) + A.FLAG_BITS
    text = "".join(map(str, bits))
    assert text.count("".join(map(str, stuffed))) == 1, "stuff bits misplaced"
    _, tones = A.decode_tones(times, t0)
    tm = A.check_timing(times, tones, t0, board.clock_hz)
    clock = 1.0 / board.clock_hz
    for name in ("mark", "space"):
        assert tm[name] is not None and tm[name] <= B.TOLERANCE, tm
        assert tm[name + "_s"] <= 1.01 * clock, (name, tm[name + "_s"] * 1e6)
    assert tm["phase_s"] <= 2 * clock, "phase error %.2f us" % (tm["phase_s"] * 1e6)
    return tm


def test_ax25_loopback_short_frame(backend):
    """The RTL scenario and the PHY-GL-SDF candidate: a UI frame from N0CALL
    to APRS with no information (16 octets and the FCS, the shortest UI
    frame, 144 bits between the flags plus stuffing), sent by threads 0 and
    1 and received by thread 3 over the wire OUT0 to IN0; about 1.4 M
    clocks on the 7.6 MHz board. The audio decodes to the frame, its stuff
    bits where 3.6 puts them, its tones and phase as the header says; thread
    3 delivers the octets, the FCS and a good report."""
    times, got, loom, frame = tx_run(backend, BRTL, ("APRS", 0), ("N0CALL", 7), b"")
    check_tx(times, BRTL, frame)
    assert got == expected_rx(frame), [hex(w) for w in got]
    assert loom.badop() == 0


@model_only("a 34-octet frame, 2 M clocks on the 7.6 MHz board")
def test_ax25_tx_stuffing(backend):
    """Information with long runs of 1s: 0xFF 0xFF (sixteen 1s: three stuff
    bits), 0x7E (a flag's pattern, which must not appear: one stuff bit),
    0x3E 0x1F and text; the reference receiver finds the frame good with
    every stuff bit where 3.6 puts it, and thread 3 agrees."""
    info = bytes([0xFF, 0xFF, 0x7E, 0x3E, 0x1F, 0xF8]) + b">Loom AX.25"
    times, got, loom, frame = tx_run(backend, BRTL, ("CQ", 0), ("KB1XYZ", 15), info)
    check_tx(times, BRTL, frame)
    assert got == expected_rx(frame), [hex(w) for w in got]
    assert loom.badop() == 0


@model_only("idle flags only, 24 bits at 50 MHz, 1 M clocks")
def test_ax25_tx_idle_flags_50mhz(backend):
    """At the 50 MHz tick (315 + 168/256), with nothing queued: flags back
    to back (3.11 time fill), so the decoded bits are 01111110 repeated,
    each steady tone within one clock of nominal and every edge within two
    clocks of the ideal wave (the fractional tick's dither)."""
    bench = ax_bench(backend)
    rec = bench.add(B.EdgeRecorder("OUT0"))
    start(backend, bench, B50, [0, 1])
    bench.step(B50.cycles(26 * A.BIT_S))
    times = rec.times(B50.clock_hz)[1:]
    t0, tones = A.decode_tones(times, A.fit_grid(times, fit_bits=16)[0])
    bits = "".join(map(str, A.un_nrzi(tones, tones[0])))[1:]
    assert len(bits) >= 16 and "01111110" * 2 in bits, bits
    s = bits[bits.index("01111110"):]
    assert len(s) >= 16 and s[:len(s) // 8 * 8] == "01111110" * (len(s) // 8), bits
    tm = A.check_timing(times, tones, t0, B50.clock_hz)
    clock = 1.0 / B50.clock_hz
    assert tm["mark_s"] <= 1.01 * clock and tm["space_s"] <= 1.01 * clock, tm
    assert tm["phase_s"] <= 2 * clock, tm


# --------------------------------------------------------------- receiver
def rx_run(backend, board, frames_bits, *, offset=0.0, jitter_us=0.0, seed=25):
    """The receiver alone on thread 3, an ``FskSource`` on IN0 sending the
    tones of ``frames_bits`` (bits before NRZI). Returns ``(words, loom)``."""
    bench = ax_bench(backend)
    loom = start(backend, bench, board, [3])
    tones = A.nrzi(frames_bits)
    begin_us = bench.cycle / board.clock_hz * 1e6 + 50.0
    src = bench.add(B.FskSource("IN0", board.clock_hz, tones, offset=offset,
                                jitter_us=jitter_us, seed=seed, start_us=begin_us))
    words = run_drain(bench, loom, board, len(tones) * A.BIT_S / (1 + offset) + 2e-3)
    assert src.done
    return words, loom


FRAME = A.ui_frame("APRS", 0, "N0CALL", 7, b"!")


@model_only("one frame a case, 0.9 M clocks each; the RTL receives in "
            "test_ax25_loopback_short_frame")
@pytest.mark.parametrize("offset,jitter_us", [(0.01, 20.0), (-0.01, 20.0)])
def test_ax25_rx_tolerance(backend, offset, jitter_us):
    """A source 1 % fast or slow with +-20 us of edge jitter: thread 3
    delivers the frame's octets, its FCS and a good report."""
    bits = A.frame_line_bits(A.with_fcs(FRAME), lead_flags=4, tail_flags=2)
    words, loom = rx_run(backend, BRTL, bits, offset=offset, jitter_us=jitter_us)
    assert words == expected_rx(FRAME), [hex(w) for w in words]
    assert loom.badop() == 0


@model_only("two frames, 1.7 M clocks")
def test_ax25_rx_bad_fcs_and_shared_flag(backend):
    """Two frames sharing one flag (3.1): the first with one information
    bit flipped after its FCS was computed, so its report says bad FCS;
    the second, opened by the first's closing flag, good."""
    full = bytearray(A.with_fcs(FRAME))
    full[16] ^= 0x04
    second = A.ui_frame("CQ", 0, "KB1XYZ", 1, b"ok")
    bits = (A.FLAG_BITS * 4 + A.stuff(A.octet_bits(full)) + A.FLAG_BITS
            + A.stuff(A.octet_bits(A.with_fcs(second))) + A.FLAG_BITS * 2)
    words, loom = rx_run(backend, BRTL, bits)
    assert words == list(full) + [report(len(full), bad=True)] + expected_rx(second), \
        [hex(w) for w in words]
    assert loom.badop() == 0


@model_only("an aborted frame and a good one, 1.6 M clocks")
def test_ax25_rx_abort(backend):
    """A frame cut after its address field by fifteen 1s (3.10), then flags
    and a good frame: the 14 address octets, an abort report counting
    them, then the good frame."""
    head = A.with_fcs(FRAME)[:14]
    bits = (A.FLAG_BITS * 4 + A.stuff(A.octet_bits(head)) + [1] * 15
            + A.FLAG_BITS * 3 + A.stuff(A.octet_bits(A.with_fcs(FRAME)))
            + A.FLAG_BITS * 2)
    words, loom = rx_run(backend, BRTL, bits)
    assert words == list(head) + [report(14, abort=True)] + expected_rx(FRAME), \
        [hex(w) for w in words]
    assert loom.badop() == 0


# ---------------------------------------------------- the references alone
def test_ax25_reference_fcs_and_address():
    """The model's FCS gives the catalogue check value of CRC-16/X-25 and
    the good residue; its address encoding gives AX.25 2.2 figure 3.5's
    destination octets for NJ7P (0x94 0x6E 0xA0 0x40 0x40 for J 7 P and the
    spaces, SSID octet 0xE0 with C = 1, SSID 0). The figure gives N as
    0x98, which is L (0x4C) shifted: N (0x4E) shifted is 0x9C, the text's
    rule; the figure's A1 is a misprint (QUESTIONS.md)."""
    assert A.fcs16(b"123456789") == A.CHECK == 0x906E
    assert A.fcs_good(A.with_fcs(b"123456789"))
    field = A.address_field("NJ7P", 0, "N7LEM", 0)
    assert field[:7] == bytes([0x9C, 0x94, 0x6E, 0xA0, 0x40, 0x40, 0xE0])
    assert field[13] & 1 and not any(o & 1 for o in field[:13])
    assert field[13] >> 7 == 0                  # the source's C bit (6.1.2)
    words = A.host_words("NJ7P", 0, "N7LEM", 0, b"abc")
    raw = bytes(b for w in words[1:8] for b in (w & 0xFF, w >> 8))
    assert bytes(((o << 1) & 0xFF) for o in raw[:13]) == field[:13]
    assert words[0] == 3 and words[8:] == [ord("a") | ord("b") << 8, ord("c")]


def test_ax25_reference_round_trip():
    """The reference chain on an ideal signal: frame, stuffing, NRZI,
    tones, crossings, then the decoder finds the frame good, and the
    timing checker reports no error."""
    full = A.with_fcs(A.ui_frame("CQ", 0, "KB1XYZ", 15, bytes([0xFF, 0x7E, 0x3F])))
    tones = A.nrzi(A.frame_line_bits(full, lead_flags=3, tail_flags=2))
    times, _ = B.fsk_crossings(tones)
    times = [t + 1e-3 for t in times]
    t0, bits, frames = A.decode(times)
    assert [f.octets for f in frames if f.good] == [full]
    _, tn = A.decode_tones(times, t0)
    tm = A.check_timing(times, tn, t0, 1e9)
    assert tm["phase_s"] < 1e-9 and tm["mark"] < 1e-9 and tm["space"] < 1e-9


def test_ax25_reference_catches_errors():
    """The checks can fail: a flipped bit fails the FCS, a missing stuff
    bit makes the deframer see a flag or an abort, and edges moved 30 us
    off the ideal continuous-phase wave show in the phase check."""
    full = bytearray(A.with_fcs(A.ui_frame("CQ", 0, "KB1XYZ", 1, b"ok")))
    full[3] ^= 1
    assert not A.fcs_good(full)
    good = A.with_fcs(A.ui_frame("CQ", 0, "KB1XYZ", 1, bytes([0xFF])))
    bits = A.octet_bits(good)
    frames = A.deframe(A.FLAG_BITS * 2 + bits + A.FLAG_BITS * 2)   # unstuffed
    assert not any(f.good for f in frames)
    tones = A.nrzi(A.frame_line_bits(good, lead_flags=3, tail_flags=2))
    times, _ = B.fsk_crossings(tones)
    jumped = [t + (30e-6 if i % 7 == 3 else 0.0) for i, t in enumerate(times)]
    _, tn = A.decode_tones(times, 0.0)
    assert A.check_timing(jumped, tn, 0.0, 1e9)["phase_s"] > 10e-6
