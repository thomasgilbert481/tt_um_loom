"""L3 for the PSK31 transmitter, on the model and the RTL.

``firmware/psk31_tx.loom`` (TX = OUT0) sends host text as varicode BPSK of
an audio carrier: square mode (the pin is the carrier's sign, reversals
abrupt) or shaped mode (cosine-shaped reversals, sigma-delta). The
reference is ``tools.protomodels.psk31``: a receiver that works from the
pin alone (carrier offset and phase from the squared envelope, reversals
as sign changes, a fitted symbol grid, differential decisions, varicode).

Expected values come from the description ([ARRL] in the model's
docstring) and the program's interface, never from what the model or the
RTL did:

* the text decodes to itself, and the bit stream is exactly the preamble
  (PRE zeros), each character's varicode followed by "00", the postamble
  (POST ones) ([ARRL]: 0 = reversal, idle = zeros, postamble = ones);
* every reversal on one symbol grid of N samples (the header: a symbol is
  N samples, a tick a sample); in square mode, exactly: the pin's edges
  are the carrier's half-period grid, and each reversal is a missing edge
  exactly at key-up + m N samples;
* the symbol rate fs / N and the carrier STEP fs / 65536 within the
  tolerances below; at the real rate, 31.25 baud ([ARRL]) and 1000 Hz;
* every pin edge on the tick grid (the program writes the pin one slot
  after each WAITD; WAITD deadlines are absolute, SEMANTICS 2, 6.4);
* spectrum during idle: shaped, the lines at fc +- 3/2 baud far below the
  pair at +- 1/2 baud; square, the rectangular-pulse values (-9.5 dB).

Rates. The schedule is proved for a tick of 300 clocks, the tick used
everywhere here. Real PSK31 is 32 ms a symbol (9.6 M clocks at 50 MHz),
so the RTL runs scaled symbols: the waveform in samples is the program's
for any N, and N and STEP are host words. The real-rate body (fs 8 kHz,
N 256, STEP 8192: 31.25 baud and 1000 Hz on a 2.4 MHz board) and the
spectrum bodies are ``model_only``. Every body is named ``test_psk31_*``.
"""

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels import psk31 as P
from tools.protomodels.dtmf import EdgeRecorder
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
TICK = 300                              # the proved minimum, clocks a sample
END = 0x100                             # host word: end of message
RATE_TOL = 1e-3                         # symbol rate, relative (the model docstring)
CARRIER_TOL = 0.01                      # carrier offset, in baud (the model docstring)
GRID_TOL = 1 / 16                       # a reversal off the fitted grid, in symbols


def tx_bench(backend):
    """The FIFOs and the data memory; the RTL has both."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "DMEM"), imem_words=512)
    return backend.bench()


def send(backend, step, n, shaped, pre, post, text, end=True):
    """Configure, queue ``text`` (and the end word), and run until the
    transmission is over: (bench, loom, recorder, levels, first cycle)."""
    bench = tx_bench(backend)
    rec = bench.add(EdgeRecorder("OUT0"))
    loom = backend.loom(bench, isa=ISA)
    prog = assemble_file(FIRMWARE / "psk31_tx.loom", isa=ISA, strict=True)
    loom.load(prog.words)
    loom.write_csr(0, "TICK_INT", TICK)
    loom.run([0])
    words = [step, n, int(shaped), pre, post] + [ord(c) for c in text]
    loom.push(0, words + ([END] if end else []))
    symbols = 2 + pre + post + len(P.encode(text))
    bench.step((symbols + 1) * n * TICK)   # one symbol of margin: the RTL budget
    assert rec.edges, "the pin never moved"
    first = rec.edges[0][0]             # a sample boundary (the tick grid)
    count = (bench.cycle - first) // TICK
    levels = P.pin_samples(rec.edges, rec.initial, first, TICK, count)
    return bench, loom, rec, levels, first


def check_ticks(rec):
    """Every edge on the tick grid of the first one."""
    cycles = [c for c, _ in rec.edges]
    assert cycles and all((c - cycles[0]) % TICK == 0 for c in cycles)


def check_stream(d, text, pre, post, n, cps):
    """The decoded text and bits, the grid, the rate and the carrier."""
    assert d.text == text, d
    assert d.bits == "0" * pre + P.encode(text) + "1" * post, d.bits
    assert len(d.reversals) == d.bits.count("0"), d
    assert d.grid_error <= GRID_TOL * n, d
    assert abs(d.period / n - 1) <= RATE_TOL, d
    assert abs(d.offset) <= CARRIER_TOL / n, d


# ------------------------------------------------------------ varicode
def test_psk31_varicode_table():
    """[ARRL]'s rules for the code: 128 codes, none holding "00", each
    starting and ending with 1; every character round-trips; a stream
    split at "00" decodes; the postamble's ones print nothing."""
    P.check_table()
    text = "".join(chr(c) for c in range(128))
    assert P.decode(P.encode(text)) == text
    assert P.decode_terminated("0000" + P.encode("PSK31 de G3PLX") + "1111") \
        == "PSK31 de G3PLX"
    assert P.VARICODE[ord(" ")] == "1" and P.VARICODE[ord("e")] == "11"


# ------------------------------------------------------------ square mode
def test_psk31_square_text(backend):
    """Square mode, "te" with a 2-symbol preamble and postamble, 8 samples
    a carrier cycle and 64 a symbol: the text decodes to itself with
    exactly the expected bits; every reversal on the fitted grid, the rate
    and carrier exact; on the pin, every edge on the carrier's half-period
    grid from key-up and every reversal a missing edge exactly at key-up +
    m N, nowhere else. The PHY-GL-SDF candidate."""
    n, half = 64, 4
    bench, loom, rec, lv, first = send(backend, 8192, n, False, 2, 2, "te")
    d = P.demodulate(lv, 1 / 8, n)
    check_stream(d, "te", 2, 2, n, 1 / 8)
    check_ticks(rec)
    # key-up starts with a low half-cycle (sin > 0), indistinguishable from
    # the pin before it: the first edge is half a carrier period in
    ups = [(c - first) // TICK + half for c, _ in rec.edges]
    assert all(u % half == 0 for u in ups), ups
    gaps = [(a, b) for a, b in zip(ups, ups[1:]) if b - a != half]
    assert all(b - a == 2 * half for a, b in gaps), gaps
    assert [a + half for a, _ in gaps] == \
        [m * n for m, bit in enumerate(d.bits, 1) if bit == "0"], gaps
    assert rec.level == 0                       # keyed down: pin low
    assert loom.badop() == 0


# ------------------------------------------------------------ shaped mode
def test_psk31_shaped_text(backend):
    """Shaped mode, "e" with a 2-symbol preamble and postamble, 32 samples
    a carrier cycle (the sigma-delta needs that oversampling: at 8 the
    envelope's zeros fill in to 20 % of the peak) and 128 a symbol: the text decodes to itself with
    exactly the expected bits, every reversal (an envelope zero) on the
    fitted grid, the rate and carrier exact; after the key-down ramp the
    line is silent (the pin alternating 1, 0: the filter's mid level).
    The words are queued at once, so key-up follows the configuration."""
    n = 128
    bench, loom, rec, lv, first = send(backend, 2048, n, True, 2, 2, "e")
    d = P.demodulate(lv, 1 / 32, n)
    check_stream(d, "e", 2, 2, n, 1 / 32)
    envelope_on_grid(lv, d, n, 1 / 32)
    check_ticks(rec)
    tail = lv[-n:]
    assert all(a != b for a, b in zip(tail, tail[1:])), tail
    assert loom.badop() == 0


def envelope_on_grid(lv, d, n, cps):
    """Shaped: |envelope| at each fitted boundary with a reversal is near
    zero and at each symbol centre near its peak (a cosine, [ARRL])."""
    z = P.baseband(lv, cps)
    peak = max(abs(v) for v in z)
    for r in d.reversals:
        assert abs(z[int(round(r))]) <= 0.1 * peak, r
    for j in range(len(d.bits)):
        c = int(round(d.t0 + (j + 0.5) * d.period))
        if d.start + n <= c <= d.end - n:
            assert abs(z[c]) >= 0.6 * peak, c


# ------------------------------------------------------- the real rate
@model_only("31.25 baud is 32 ms a symbol: 3 M clocks here, 8 min of RTL; "
            "the RTL runs the same program at scaled N in the bodies above")
def test_psk31_real_rate(backend):
    """The header's 32 kHz configuration (N 1024, STEP 2048: 1000 Hz and
    31.25 baud) sample for sample, on a 9.6 MHz board (tick 300): "e"
    decodes, the symbol rate is 31.25 baud ([ARRL]) within 0.1 %, the
    carrier 1000 Hz within 1 % of a baud, every reversal on the grid."""
    n, step, clock = 1024, 2048, 9_600_000
    fs = clock / TICK
    bench, loom, rec, lv, first = send(backend, step, n, True, 2, 2, "e")
    d = P.demodulate(lv, step / 65536, n)
    check_stream(d, "e", 2, 2, n, step / 65536)
    assert abs(fs / d.period / P.BAUD - 1) <= RATE_TOL, fs / d.period
    carrier = (step / 65536 + d.offset) * fs
    assert abs(carrier - 1000.0) <= CARRIER_TOL * P.BAUD, carrier
    assert loom.badop() == 0


# ------------------------------------------------------------- spectrum
@model_only("measurement only: twelve symbols of idle in each mode, "
            "0.6 to 2.5 M clocks each")
@pytest.mark.parametrize("shaped,step,n,floor", [
    (False, 2048, 128, None), (True, 2048, 128, -20.0), (True, 512, 512, -40.0)],
    ids=["square", "shaped_32", "shaped_128"])
def test_psk31_idle_spectrum(backend, shaped, step, n, floor):
    """During the preamble (continuous reversals) the spectrum is lines at
    fc +- m baud / 2, m odd ("IMD" in PSK31 programs is the m = 3 pair).
    Square: the values of textbook square-wave BPSK
    (``psk31.ideal_square``, from the description) within 0.5 dB: about
    -9 dB at m = 3 and -12 dB at m = 5 here (1/m^2 for the reversals,
    plus the carrier's harmonics' sidebands, which land on these lines at
    4 carrier cycles a symbol): as wide as BPSK gets. Shaped: ideally
    only m = 1 (G3PLX's two tones); what is left is the first-order
    sigma-delta's idle tones, which fall with oversampling: m = 3, 5, 7
    at most -20 dB at 32 samples a carrier cycle (the 32 kHz setting) and
    -40 dB at 128 (the 128 kHz setting); the usual PSK31 yardstick is
    -25 dB (from memory, unchecked)."""
    cps = step / 65536
    bench, loom, rec, lv, first = send(backend, step, n, shaped, 12, 1, "")
    d = P.demodulate(lv, cps, n)
    assert d.bits.startswith("0" * 12), d.bits
    lo = int(round(d.t0 + 3 * n))       # whole symbols inside the preamble
    lines = P.line_levels(lv, lo, 8 * n, cps, n)
    print("PSK31 idle lines (dB re m = 1), shaped=%s step=%d n=%d: %s"
          % (shaped, step, n, {m: round(v, 1) for m, v in lines.items()}))
    if shaped:
        assert all(lines[m] <= floor for m in (3, 5, 7)), lines
    else:
        ideal = P.line_levels(P.ideal_square("0" * 12, cps, n), n, 8 * n, cps, n)
        for m in (3, 5, 7):
            assert abs(lines[m] - ideal[m]) <= 0.5, (lines, ideal)
        assert lines[3] >= -10.0, lines     # 1/m^2 alone would be -9.5
    assert loom.badop() == 0
