"""L3 for the DTMF generator, on the model and the RTL, and the one-bit
receiver finding, on the model.

``firmware/dtmf_gen.loom`` (TX = OUT0) sends each queued key as the sum of
its two tones in a one-bit pulse-density stream at 125 000 samples a
second. The reference is ``tools.protomodels.dtmf``: a decimating filter
turns the pin into audio, a joint least-squares fit measures both
frequencies, their levels and the voice-band distortion, and a matched step
finds where each signal starts and ends. No dedicated detector program
exists: a one-bit Goertzel detector cannot meet Q.24 (the module docstring
of ``tools.protomodels.dtmf`` and FW_README); the ``test_dtmf_one_bit_*``
bodies hold the numbers that say why.

Expected values come from the standards and the program's interface, never
from what the model or the RTL did:

* every frequency within +-1.8 % of nominal (Q.23 section 7.1) and inside
  AT&T's +-1.5 % operate band (Table A-1/Q.24), the key the nominal pair
  says (Figure 1/Q.23), the level difference inside +4 .. -8 dB (Table
  A-1/Q.24 note b), distortion at least 20 dB down (Q.23 section 7.2);
* a tone lasts its host-set count of samples (the header: 125 per ms), a
  pause at least its count; at Q.24's 40 ms both are at least 40 ms;
* every pin edge on the tick grid: the program writes the pin one slot
  after each WAITD, and WAITD deadlines are absolute (SEMANTICS 2, 6.4).

Boards. The program is proved for a tick of 144 clocks, so it runs on any
clock of 18 MHz or more with the tick set to clock / 125 000. The model
runs about 25 000 clocks a second, so every body here uses the 18 MHz board
(a 10 ms tone is 180 k clocks); at 50 MHz the stream is the same, sample for
sample. The 40 ms timing body and the sixteen-key sweep are ``model_only``.
Every body is named ``test_dtmf_*``: test/test_fw.py collects the bodies of
all modules into one namespace.
"""

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels import dtmf as D
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
FS = 125_000.0                          # the header: samples a second
CLOCK = 18_000_000.0                    # the 18 MHz board
TICK = int(CLOCK / FS)                  # 144 clocks, the proved minimum
EDGE_TOL_S = 0.25e-3                    # the reference's timing accuracy


def gen_bench(backend):
    """The FIFOs and the data memory; the RTL has both and ``RtlBench``
    takes neither keyword."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "DMEM"), imem_words=512)
    return backend.bench()


def start(backend, tone, pause):
    """Load and start the generator with its two durations (samples), and
    let 1 ms of silence go out: (bench, loom, recorder, capture start)."""
    bench = gen_bench(backend)
    rec = bench.add(D.EdgeRecorder("OUT0"))
    loom = backend.loom(bench, isa=ISA)
    prog = assemble_file(FIRMWARE / "dtmf_gen.loom", isa=ISA, strict=True)
    loom.load(prog.words)
    loom.write_csr(0, "TICK_INT", TICK)
    loom.run([0])
    loom.push(0, [tone, pause])
    bench.step(int(0.0005 * CLOCK))     # the silence has started
    c0 = bench.cycle
    bench.step(int(0.0010 * CLOCK))
    return bench, loom, rec, c0


def capture(bench, rec, c0):
    """Every signal in the pin's audio from cycle ``c0`` to now."""
    xs = D.decimate(rec.edges, rec.initial, c0, (bench.cycle - c0) / CLOCK, CLOCK)
    return D.find_tones(xs, D.FS_AUDIO, c0 / CLOCK)


def check_tone(t, index):
    """One signal against Q.23 and Q.24 (AT&T)."""
    lo, hi = D.key_freqs(index)
    for got, nominal in ((t.low_hz, lo), (t.high_hz, hi)):
        assert abs(got / nominal - 1) <= D.SEND_TOLERANCE, (t, nominal)
        assert abs(got / nominal - 1) <= D.OPERATE_TOLERANCE, (t, nominal)
    assert t.key() == index, t
    assert D.TWIST_DB[0] <= t.twist_db <= D.TWIST_DB[1], t
    assert t.distortion_db <= -D.DISTORTION_DB, t


def check_grid(rec, c0):
    """Every edge after ``c0`` on the tick grid of the first one."""
    cycles = [c for c, _ in rec.edges if c >= c0]
    assert cycles and all((c - cycles[0]) % TICK == 0 for c in cycles)


# -------------------------------------------------------------- generator
def test_dtmf_short_tone(backend):
    """Key D (941 + 1633 Hz, the highest pair) for 1250 samples, then a
    250-sample pause: both frequencies inside Q.23's 1.8 % and AT&T's
    1.5 %, level difference and distortion inside Q.23/Q.24, the tone
    10 ms long, every edge on the tick grid. The PHY-GL-SDF candidate."""
    bench, loom, rec, c0 = start(backend, 1250, 250)
    loom.push(0, [D.key_index("D")])
    bench.step(int(0.0125 * CLOCK))
    tones = capture(bench, rec, c0)
    assert len(tones) == 1, tones
    check_tone(tones[0], D.key_index("D"))
    assert abs(tones[0].duration_s - 1250 / FS) <= EDGE_TOL_S, tones[0]
    check_grid(rec, c0)
    assert loom.badop() == 0


def test_dtmf_idle_is_silent(backend):
    """With the durations given and no key queued the line carries no
    signal: no tone found, the audio steady within 1 % of the swing."""
    bench, loom, rec, c0 = start(backend, 1250, 250)
    bench.step(int(0.0020 * CLOCK))
    xs = D.decimate(rec.edges, rec.initial, c0, (bench.cycle - c0) / CLOCK, CLOCK)
    assert D.find_tones(xs, D.FS_AUDIO, c0 / CLOCK) == []
    mean = sum(xs) / len(xs)
    assert max(abs(x - mean) for x in xs) <= 0.01
    check_grid(rec, c0)
    assert loom.badop() == 0


@model_only("four 8 ms keys a row, 0.81 M clocks each row, 2 min of RTL "
            "each; the RTL sends key D in test_dtmf_short_tone")
@pytest.mark.parametrize("row", [0, 1, 2, 3])
def test_dtmf_every_key(backend, row):
    """The four keys of one row of Figure 1/Q.23, queued at once, 1000
    samples each with 375-sample pauses: each signal checked as in
    test_dtmf_short_tone, in order, 8 ms long, and at least 3 ms apart."""
    bench, loom, rec, c0 = start(backend, 1000, 375)
    keys = [4 * row + col for col in range(4)]
    loom.push(0, keys)
    bench.step(int((4 * 0.011 + 0.001) * CLOCK))
    tones = capture(bench, rec, c0)
    assert [t.key() for t in tones] == keys, tones
    for t, k in zip(tones, keys):
        check_tone(t, k)
        assert abs(t.duration_s - 1000 / FS) <= EDGE_TOL_S, t
    for a, b in zip(tones, tones[1:]):
        assert b.start_s - a.end_s >= 375 / FS - EDGE_TOL_S, (a, b)
    assert loom.badop() == 0


@model_only("two 40 ms tones and a 40 ms pause, 2.2 M clocks, 6 min of RTL")
def test_dtmf_q24_timing(backend):
    """At Q.24's figures (AT&T, Table A-1/Q.24): tones of 5000 samples and
    a pause of 5000, keys 1 and #: each tone at least 40 ms (the operate
    minimum) and within the reference's accuracy of 40 ms, so far above the
    23 ms non-operate maximum; the pause at least 40 ms."""
    bench, loom, rec, c0 = start(backend, 5000, 5000)
    keys = [D.key_index("1"), D.key_index("#")]
    loom.push(0, keys)
    bench.step(int((0.040 * 3 + 0.001) * CLOCK))
    tones = capture(bench, rec, c0)
    assert [t.key() for t in tones] == keys, tones
    for t, k in zip(tones, keys):
        check_tone(t, k)
        assert t.duration_s >= D.MIN_TONE_S - EDGE_TOL_S, t
        assert abs(t.duration_s - 0.040) <= EDGE_TOL_S, t
        assert t.duration_s > D.MAX_REJECT_S, t
    assert tones[1].start_s - tones[0].end_s >= D.MIN_PAUSE_S - EDGE_TOL_S, tones
    assert loom.badop() == 0


# ------------------------------------------------- the one-bit receiver
# Why there is no dtmf_detect.loom: a comparator on the line (a one-bit
# input, as classic one-bit decoders use) feeding Goertzel filters on the
# eight frequencies. These bodies use the reference's one-bit source and
# Goertzel recursion at 8 kHz, each filter over 20 cycles of its frequency
# (the window at which +-1.5 % costs 1.3 dB and +-3.5 % costs 8.7 dB, so one
# threshold could serve every filter), at three starting phases per key.
PHASES = ((0.0, 0.0), (1.0, 2.5), (2.0, 4.0))


def one_bit_extremes(dev, twist_db):
    """Over all keys and PHASES: (the weakest bin of a sent frequency, the
    strongest bin of an unsent one), in dB."""
    sent, unsent = 99.0, -99.0
    for k in range(16):
        for ph in PHASES:
            p = D.one_bit_bins(k, dev=dev, twist_db=twist_db, phases=ph)
            mine = (k >> 2, 4 + (k & 3))
            sent = min(sent, min(p[i] for i in mine))
            unsent = max(unsent, max(v for i, v in enumerate(p) if i not in mine))
    return sent, unsent


def test_dtmf_one_bit_margin_at_zero_twist():
    """With equal levels a threshold exists, but a thin one: a tone at the
    edge of AT&T's operate band (+-1.5 %) gives a bin only 2 to 4 dB above
    the strongest bin of a frequency that was not sent (the limiter's
    intermodulation), and a tone 3.5 % off falls below that bin."""
    ok_sent = min(one_bit_extremes(d, 0.0)[0] for d in (0.015, -0.015))
    unsent = max(one_bit_extremes(d, 0.0)[1] for d in (0.0, 0.015, -0.015))
    assert 0.0 < ok_sent - unsent < 5.0, (ok_sent, unsent)
    far_sent, _ = one_bit_extremes(0.035, 0.0)
    assert far_sent < unsent, (far_sent, unsent)


def test_dtmf_one_bit_capture_at_minus_8_db():
    """At AT&T's -8 dB level difference (Table A-1/Q.24 note b) the limiter
    suppresses the weaker tone below a frequency that was not sent, even
    on frequency: no threshold separates them, so a one-bit receiver cannot
    meet Q.24's level-difference range."""
    sent, unsent = one_bit_extremes(0.0, -8.0)
    assert sent < unsent, (sent, unsent)


def test_dtmf_one_bit_star_intermodulation():
    """Key * (941 + 1209 Hz): 2 x 1209 - 941 = 1477 Hz, the third column's
    frequency, so the limiter's third-order product lands on a DTMF bin, 8
    dB or less under the 1209 Hz bin that was sent."""
    p = D.one_bit_bins(D.key_index("*"))
    assert 2 * D.HIGH_HZ[0] - D.LOW_HZ[3] == D.HIGH_HZ[2]
    assert p[4] - p[6] <= 8.0, p
