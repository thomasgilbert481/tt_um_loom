"""L3-ENDAT on the model and the RTL: firmware/endat_master.loom and endat_encoder.loom.

The master (thread 0: CLK = OUT0, DOUT = OUT1, DE = OUT2, DIN = IN0) reads a
model encoder, ``tools.protomodels.endat.Encoder``, which answers "Encoder
send position values" with a chosen cable delay and injected errors and
records every CLOCK edge; the tests decode the master's three result words
and time CLOCK against HEIDENHAIN's windows. The encoder (thread 0: CLK =
IN0, DIN = IN1, DOUT = OUT0, DE = OUT1) answers a model master,
``endat.Master``. Together, the master on thread 0 reads the encoder on
thread 1 through ``endat.Line``.

Expected values come from the protocol through the reference model (the
frame, the CRC-5) and from the program headers (the command and result
formats, eight ticks per period, the encoder's setting words), never from
what the model or the RTL did.

Every body takes a ``backend`` (``tools/tests/fw_backend.py``): pytest runs
them on the golden model, ``test/test_fw.py`` on the RTL.
"""

import pytest

from tools.loomasm import assemble_text
from tools.loomisa import REPO, load
from tools.protomodels.endat import (MODE_POS, Encoder, Line, Master, US,
                                     clock_window, crc5, endat_bits,
                                     frequency_ok, pulse_widths_ok, tm_ok)
from tools.protomodels.biss import Wire
from tools.tests.fw_backend import model_only

ISA = load()
MASTER = REPO / "firmware" / "endat_master.loom"
ENCODER = REPO / "firmware" / "endat_encoder.loom"
IMEM = 512

# The master's formats (program header).
CONFIG = 0x8000
ERR, CRCF, NOSTART, BUSY = 1, 4, 8, 0x10
TICK = 60                           # the master's .tick: an eighth of a period
PERIOD = 8 * TICK                   # 104 kHz at 50 MHz


def endat_bench(backend):
    """FIFOs, the deadline latch, the bit engine, LD/ST and the 512-word memory on the model."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "SETPD", "BE", "BEENC", "DMEM"),
                             imem_words=IMEM)
    return backend.bench()


def program(path, thread=0, pins=None):
    """A program for the 512-word memory, moved to ``thread`` and to other
    pins by changing only its ``.thread`` and ``.pins`` lines."""
    text = path.read_text(encoding="utf-8")
    if thread:
        assert text.count(".thread 0") == 1
        text = text.replace(".thread 0", ".thread %d" % thread)
    if pins:
        old, new = pins
        assert text.count(old) == 1, old
        text = text.replace(old, new)
    prog = assemble_text(text, path.name, isa=ISA, imem_words=IMEM, strict=True)
    assert prog.diagnostics == []
    return prog


def boot(backend, bench, programs):
    """Load ``[(program, thread, tick)]`` as one image, set each thread's
    tick, run them all."""
    words = {}
    for prog, _, _ in programs:
        assert not set(words) & set(prog.words), "the programs overlap"
        words.update(prog.words)
    loom = backend.loom(bench, isa=ISA)
    loom.load(words)
    for _, thread, tick in programs:
        loom.write_csr(thread, "TICK_INT", tick)
        loom.write_csr(thread, "TICK_FRAC", 0)
    loom.run([thread for _, thread, _ in programs])
    return loom


def read_frame(loom, thread=0):
    """One READ: the master's three result words (status, pos low, pos high)."""
    loom.push(thread, [0x0000])
    return tuple(loom.pop(thread, 3))


def position(words):
    return words[1] | (words[2] << 16)


def delay_phase(status):
    """Status bits 11:8 (the tick, 1 .. 8, that first saw the start bit) and
    15:12 (whole periods after rise 11)."""
    return (status >> 8) & 0xF, status >> 12


def test_endat_crc5_reference():
    """The model's CRC-5 is the shift register of the source (0x0B, start
    11111); an inverted CRC appended leaves the CRC of 11111 from 0, the
    residue the master checks for (its RESIDUE, 01000 left-aligned)."""
    assert crc5([]) == 0x1F
    for pos, n in ((0, 1), (0x1ABCDEF, 25), (0xFFFFFFFF, 32)):
        bits = endat_bits(pos, n)
        c = 0x1F
        for b in bits:
            fb = ((c >> 4) & 1) ^ b
            c = ((c << 1) & 0x1F) ^ (0x0B if fb else 0)
        assert c == 0x08


# ------------------------------------------------------------------ master
def master_on(backend, **encoder):
    """The master on thread 0 against a model encoder on OUT0 .. OUT2 / IN0."""
    bench = endat_bench(backend)
    enc = bench.add(Encoder("OUT0", "OUT1", "OUT2", "IN0", **encoder))
    loom = boot(backend, bench, [(program(MASTER), 0, TICK)])
    return bench, enc, loom


def configure(loom, n):
    loom.push(0, [CONFIG | n])


def check_clock(enc):
    """Every CLOCK half period is exactly four ticks (every edge is a
    deadline-latched write), inside tHI/tLO 0.2 .. 10 us and 100 kHz ..
    2 MHz (p. 6, p. 4); every frame reached its closing rise; the master
    read the mode command 000111 and never drove DATA with the encoder."""
    assert enc.frames and all(f["complete"] for f in enc.frames)
    assert clock_window(enc.frames, 4 * TICK, 4 * TICK) == []
    assert pulse_widths_ok(enc.frames) == []
    assert frequency_ok(PERIOD)
    assert all(tuple(f["mode"]) == MODE_POS for f in enc.frames)
    assert enc.violations == []


def check_delay(status, delay, tcal=0):
    """The measured phase brackets the cable delay to a tick, allowing the
    master's 2-clock synchroniser and its 4-clock slot."""
    d, periods = delay_phase(status)
    assert periods == tcal, hex(status)
    assert (d - 1) * TICK <= delay + 8 and delay < d * TICK + 8, (d, delay)


@pytest.mark.parametrize("delay", [10, 330])
def test_endat_master_cable(backend, delay):
    """Two cables: 0.2 us (a few metres) and 6.6 us, more than half a
    period (330 of 480 clocks), where a master sampling at the falling edge
    would read the wrong bit. Position, a clean status, the delay
    measured, CLOCK exact; a CONFIG to 13 bits between the reads."""
    bench, enc, loom = master_on(backend, n=25, pos=0x1ABCDEF, delay=delay,
                                 tm=15 * US)
    words = read_frame(loom)
    assert words[0] & 0xFF == 0 and position(words) == 0x1ABCDEF, [hex(w) for w in words]
    check_delay(words[0], delay)
    enc.n, enc.pos = 13, 0x1555
    configure(loom, 13)
    words = read_frame(loom)
    assert words[0] & 0xFF == 0 and position(words) == 0x1555, [hex(w) for w in words]
    check_clock(enc)
    # Rises: 10 to the turnaround, the start bit on 11, F1, n bits, the
    # CRC and the closing rise: n + 18.
    rises = [sum(1 for _, v in f["edges"] if v == 1) for f in enc.frames]
    assert rises == [25 + 18, 13 + 18]


@pytest.mark.parametrize("fault, status", [("err", ERR), ("bad_crc", CRCF)])
def test_endat_master_errors(backend, fault, status):
    """The error bit F1 set and a CRC sent un-inverted: reported, the
    position still read; then a clean frame."""
    bench, enc, loom = master_on(backend, n=16, pos=0xBEEF, delay=40,
                                 tm=15 * US, **{fault: True})
    configure(loom, 16)
    words = read_frame(loom)
    assert words[0] & 0xFF == status and position(words) == 0xBEEF, [hex(w) for w in words]
    setattr(enc, fault, False)
    words = read_frame(loom)
    assert words[0] & 0xFF == 0 and position(words) == 0xBEEF


@model_only("the same code paths as test_endat_master_cable; only the "
            "lengths (one, two, three loads) and the late start bit differ")
@pytest.mark.parametrize("n, tcal", [(1, 0), (10, 2), (26, 0), (32, 1)])
def test_endat_master_lengths(backend, n, tcal):
    """Position lengths at the load boundaries (m = n + 6 = 7, 16, 32, 38)
    and a calculation time of whole periods (status bits 15:12)."""
    pos = 0xDEADBEEF & ((1 << n) - 1)
    bench, enc, loom = master_on(backend, n=n, pos=pos, delay=100, tcal=tcal,
                                 tm=15 * US)
    configure(loom, n)
    words = read_frame(loom)
    assert words[0] & 0xFF == 0 and position(words) == pos, [hex(w) for w in words]
    check_delay(words[0], 100, tcal)
    check_clock(enc)


@model_only("two frames of fixed code (busy, no start bit) with long timeouts")
def test_endat_master_busy_nostart(backend):
    """DATA held high (an encoder still in tm, or a cut cable on a fail-safe
    receiver): BUSY after 255 ticks. An encoder that never sends a start
    bit: NOSTART, CLOCK left high."""
    bench = endat_bench(backend)
    bench.add(Wire("OUT0", "IN0", level=1))         # DATA follows CLOCK: high
    loom = boot(backend, bench, [(program(MASTER), 0, TICK)])
    start = bench.cycle
    assert read_frame(loom) == (BUSY, 0, 0)
    assert bench.cycle - start >= 255 * TICK
    bench, enc, loom = master_on(backend, n=16, pos=0x1234, delay=40,
                                 tcal=100, tm=15 * US)
    configure(loom, 16)
    assert read_frame(loom) == (NOSTART, 0, 0)
    assert enc.frames[-1]["edges"][-1][1] == 1      # CLOCK left high
    enc.tcal = 0                                    # the encoder recovers
    words = read_frame(loom)
    assert words[0] & 0xFF == 0 and position(words) == 0x1234


# ----------------------------------------------------------------- encoder
ENC_TICK = 4                        # 80 ns at 50 MHz: TM = 250 ticks = 20 us
E_ERR, E_BADCRC = 1 << 8, 1 << 9    # the encoder's setting word (header)


def react(delay):
    """Clocks from a rising edge to the encoder's DATA change (program
    header): the edge is seen 2 to 5 clocks late (synchroniser, slot grid),
    NOW is read a slot later and may have ticked up to 3 clocks before, and
    the latched write reaches the pad register and this model a clock
    after its tick; never before the CSRW TD commits (DELAY 0 to 2)."""
    return max(4 * delay + 3, 14), max(4 * delay + 14, 24)


def encoder_on(backend, period, n, **master):
    """The encoder on thread 0 (CLK = IN0, DIN = IN1, DOUT = OUT0, DE =
    OUT1) and a model master."""
    bench = endat_bench(backend)
    mst = bench.add(Master("IN0", "IN1", "OUT0", "OUT1", period=period, n=n, **master))
    loom = boot(backend, bench, [(program(ENCODER), 0, ENC_TICK)])
    return bench, mst, loom


def set_encoder(bench, loom, setting, pos, delay, thread=0):
    loom.push(thread, [setting, pos & 0xFFFF, pos >> 16, delay])
    bench.step(600)                 # taken between frames, and prepared


def frames(bench, mst, count, budget=60000):
    mst.read(count)
    assert bench.run_until(lambda: not mst.busy(), budget * count, every=32)
    return mst.results[-count:]


def check_answer(r, pos, delay, f1=0, crc_ok=1, tcal=0):
    """A decoded frame: the start bit on rise 11 + TCAL, DATA DELAY ticks
    plus the reaction after each rising edge, tm inside 10 .. 30 us."""
    assert (r["error"], r["start_rise"], r["f1"], r["pos"], r["crc_ok"]) == \
        (None, 11 + tcal, f1, pos, crc_ok), r
    lo, hi = react(delay)
    assert lo <= r["delay"] <= hi, (delay, r["delay"])
    assert tm_ok([r], 10 * US, 30 * US) == []


@pytest.mark.parametrize("period, delay", [(480, 0), (256, 30)])
def test_endat_encoder_answers(backend, period, delay):
    """The encoder at 104 kHz (the master's rate) with no cable and at
    195 kHz with DELAY 30 (2.4 us plus the reaction, beyond a half period
    of 128 clocks): position, F1 = 0, a good CRC, the measured delay; then
    a 32-bit and a 3-bit position."""
    bench, mst, loom = encoder_on(backend, period, 25)
    set_encoder(bench, loom, 25, 0x1ABCDEF, delay)
    for r in frames(bench, mst, 2):
        check_answer(r, 0x1ABCDEF, delay)
    assert mst.violations == []
    for n, pos in ((32, 0xDEADBEEF), (3, 0x5)):
        mst.n = n
        set_encoder(bench, loom, n, pos, delay)
        check_answer(frames(bench, mst, 1)[0], pos, delay)


@pytest.mark.parametrize("setting, mode, expect", [
    (E_ERR, MODE_POS, {"f1": 1, "crc_ok": 1}),
    (E_BADCRC, MODE_POS, {"f1": 0, "crc_ok": 0}),
    (0, (0, 0, 1, 1, 1, 0), {"f1": 1, "crc_ok": 1})])
def test_endat_encoder_errors(backend, setting, mode, expect):
    """Error injection: F1 set, the CRC sent un-inverted, and a mode
    command the encoder does not support (001 110, selection of memory
    area), answered with F1 = 1 (p. 5); then a clean frame."""
    bench, mst, loom = encoder_on(backend, 480, 16, mode=mode)
    set_encoder(bench, loom, 16 | setting, 0xBEEF, 5)
    r = frames(bench, mst, 1)[0]
    assert {k: r.get(k) for k in expect} == expect and r["pos"] == 0xBEEF, r
    mst.mode = MODE_POS
    set_encoder(bench, loom, 16, 0x1234, 5)
    check_answer(frames(bench, mst, 1)[0], 0x1234, 5)


@model_only("the same code paths as test_endat_encoder_answers with a "
            "longer tcal loop; one frame of 18 bits")
def test_endat_encoder_tcal(backend):
    """TCAL 3: the start bit on rise 14 (a longer calculation time)."""
    bench, mst, loom = encoder_on(backend, 256, 18)
    set_encoder(bench, loom, 18 | (3 << 12), 0x2A5A5, 10)
    check_answer(frames(bench, mst, 1)[0], 0x2A5A5, 10, tcal=3)


# --------------------------------------------------- master and encoder
ENC_PINS = ("CLK = IN0, DIN = IN1, DOUT = OUT0, DE = OUT1, SCR = OUT2",
            "CLK = IN1, DIN = IN2, DOUT = OUT3, DE = OUT4, SCR = OUT5")


@pytest.mark.parametrize("delay", [2, 70])
def test_endat_master_reads_encoder(backend, delay):
    """The master on thread 0 reads the encoder on thread 1 through the
    bench's DATA pair (OUT1/OUT2 -> IN2, OUT3/OUT4 -> IN0) and CLOCK wire
    (OUT0 -> IN1), with the encoder playing a short cable (DELAY 2) and a
    long one (DELAY 70: about 290 clocks, beyond the 240-clock half
    period): positions, F1 set by the encoder, the delay measured, CLOCK
    exact, no contention on DATA."""
    bench = endat_bench(backend)
    line = bench.add(Line("OUT1", "OUT2", "IN0", "OUT3", "OUT4", "IN2", "OUT0", "IN1"))
    enc = program(ENCODER, thread=1, pins=ENC_PINS)
    loom = boot(backend, bench, [(program(MASTER), 0, TICK), (enc, 1, ENC_TICK)])
    configure(loom, 25)
    lo, hi = react(delay)
    for pos, setting, status in ((0x1ABCDEF, 25, 0), (0x0012345, 25 | E_ERR, ERR)):
        set_encoder(bench, loom, setting, pos, delay, thread=1)
        start = len(line.clk_edges)
        words = read_frame(loom)
        assert words[0] & 0xFF == status and position(words) == pos, [hex(w) for w in words]
        d, periods = delay_phase(words[0])
        assert periods == 0 and (d - 1) * TICK <= hi + 8 and lo < d * TICK + 8, (d, lo, hi)
        edges = line.clk_edges[start:]
        assert {b - a for a, b in zip(edges, edges[1:])} == {4 * TICK}
        assert len(edges) == 2 * (25 + 18)
    assert line.violations == [] and bench.contention_free()
