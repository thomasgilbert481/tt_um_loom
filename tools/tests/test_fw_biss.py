"""L3-BISS on the model and the RTL: firmware/biss_master.loom and biss_slave.loom.

The master (thread 0, MA = OUT0, SLO = IN0) reads a model encoder,
``tools.protomodels.biss.Encoder``, which answers BiSS-C or SSI frames with
a chosen delay and injected errors and records every MA edge; the tests
decode the master's three result words and time MA against the protocol.
The slave (thread 0, MA = IN0, SLO = OUT0) answers a model master,
``biss.Master``, at two MA rates. Together, the master on thread 0 reads
the slave on thread 1 (MA = IN1, SLO = OUT1) through two bench wires.

Expected values come from the protocol through the reference model (the
frame, the CRC-6, the Gray code) and from the program headers (the command
and result formats, the tick per half period, TTO = 40 ticks), never from
what the model or the RTL did.

Every body takes a ``backend`` (``tools/tests/fw_backend.py``): pytest runs
them on the golden model, ``test/test_fw.py`` on the RTL.
"""

import pytest

from tools.loomasm import assemble_text
from tools.loomisa import REPO, load
from tools.protomodels.biss import (Encoder, Master, Wire, clock_window, crc6,
                                    biss_bits, from_gray, gray, half_periods,
                                    timeout_ok)
from tools.tests.fw_backend import model_only

ISA = load()
MASTER = REPO / "firmware" / "biss_master.loom"
SLAVE = REPO / "firmware" / "biss_slave.loom"
IMEM = 512

# The master's formats (program header).
CONFIG, SSI, GRAY = 0x8000, 1 << 8, 1 << 9
ERR, WARN, CRCF, NOACK, BUSY, CDS, SSIF = 1, 2, 4, 8, 0x10, 0x20, 0x8000
# The slave's setting word (program header).
S_SSI, S_GRAY, S_WARN, S_ERR, S_BADCRC, S_NOSTART = (1 << 8, 1 << 9, 1 << 10,
                                                     1 << 11, 1 << 12, 1 << 15)
TTO_TICKS = 40                      # the slave's timeout, in its ticks


def biss_bench(backend):
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
    """One READ on the master: its three result words (status, pos low, pos
    high); ``Loom.pop`` polls, which runs the bench meanwhile."""
    loom.push(thread, [0x0000])
    return tuple(loom.pop(thread, 3))


def position(words):
    return words[1] | (words[2] << 16)


# ------------------------------------------------------------------ master
def master_on(backend, tick, **encoder):
    """The master on thread 0 against a model encoder on OUT0 / IN0."""
    bench = biss_bench(backend)
    enc = bench.add(Encoder("OUT0", "IN0", **encoder))
    loom = boot(backend, bench, [(program(MASTER), 0, tick)])
    return bench, enc, loom


def configure(loom, n, ssi=False, gray_code=False):
    loom.push(0, [CONFIG | n | (SSI if ssi else 0) | (GRAY if gray_code else 0)])


def check_ma(enc, tick):
    """Every MA half period of every frame is exactly one tick (the header:
    every edge is a deadline-latched write), every frame went to its end,
    and no frame started before the encoder was ready."""
    assert enc.frames and all(f["complete"] for f in enc.frames)
    assert clock_window(enc.frames, tick, tick) == []
    assert enc.violations == []


@pytest.mark.parametrize("tick, n, pos", [(25, 24, 0xA5C3E1), (32, 13, 0x1ABC),
                                           (25, 32, 0xDEADBEEF)])
def test_master_biss_read(backend, tick, n, pos):
    """BiSS-C reads at MA 1 MHz (the program's tick) and 781 kHz: position,
    a clean status (CDS 0, no line delay), MA exact to the clock."""
    bench, enc, loom = master_on(backend, tick, n=n, pos=pos, delay=4, tto=200)
    configure(loom, n)
    for p in (pos, pos ^ 0x5, 0):
        enc.pos = p
        words = read_frame(loom)
        assert words[0] == 0, hex(words[0])
        assert position(words) == p
    check_ma(enc, tick)
    # Rising edges: rise 1, the Ack (1 period), the start bit, CDS, the n + 8
    # bits after it, and the closing rise that starts the timeout: n + 13.
    rises = [sum(1 for _, v in f["edges"] if v == 1) for f in enc.frames]
    assert rises == [n + 13] * 3


def test_master_line_delay(backend):
    """A line a period and 6 clocks long: the frame moves by a period, the
    master counts it (status bits 11:8 = 1) and reads the position."""
    tick = 25
    bench, enc, loom = master_on(backend, tick, n=24, pos=0x123456,
                                 delay=2 * tick + 6, tto=200)
    words = read_frame(loom)
    assert words[0] == 1 << 8, hex(words[0])
    assert position(words) == 0x123456
    assert clock_window(enc.frames, tick, tick) == []


@model_only("five frames of the code paths the RTL runs in test_master_biss_read "
            "and test_master_reads_slave (ERR and WARN there); only the status bits differ")
@pytest.mark.parametrize("fault, status", [("err", ERR), ("warn", WARN),
                                           ("bad_crc", CRCF), ("ack3", 0),
                                           ("no_start", NOACK)])
def test_master_errors(backend, fault, status):
    """Injected faults: the error and warning bits, a CRC sent un-inverted,
    a long Ack (three periods, legal) and an Ack that never ends."""
    kw = {"ack": 3} if fault == "ack3" else {fault: True}
    bench, enc, loom = master_on(backend, 25, n=16, pos=0xBEEF, delay=4,
                                 tto=200, **kw)
    configure(loom, 16)
    words = read_frame(loom)
    assert words[0] == status, hex(words[0])
    if not status & NOACK:
        assert position(words) == 0xBEEF
    # A frame the master left: the encoder times out, then a clean read.
    enc.err = enc.warn = enc.bad_crc = enc.no_start = False
    enc.ack = 1
    words = read_frame(loom)
    assert words[0] == 0 and position(words) == 0xBEEF


def test_master_busy(backend):
    """SLO held low (an encoder that never comes out of its timeout, or a
    cut cable on an RS-422 receiver that fails low): BUSY after 255 ticks."""
    bench = biss_bench(backend)
    loom = boot(backend, bench, [(program(MASTER), 0, 25)])
    start = bench.cycle
    assert read_frame(loom) == (BUSY, 0, 0)
    assert bench.cycle - start >= 255 * 25


@model_only("SSI on the RTL is test_master_reads_slave[ssi=True] (25 bits, Gray)")
@pytest.mark.parametrize("n, gray_code", [(13, True), (25, False), (25, True)])
def test_master_ssi(backend, n, gray_code):
    """SSI at 781 kHz (inside the usual 100 kHz to 1 MHz): n + 1 clock
    pulses, the position MSB first, binary or Gray."""
    tick = 32
    pos = 0x1234567 & ((1 << n) - 1)
    bench, enc, loom = master_on(backend, tick, n=n, pos=pos, ssi=True,
                                 gray_code=gray_code, delay=4, tto=300)
    configure(loom, n, ssi=True, gray_code=gray_code)
    for p in (pos, pos >> 1):
        enc.pos = p
        words = read_frame(loom)
        assert words[0] == SSIF and position(words) == p
    check_ma(enc, tick)
    rises = [sum(1 for _, v in f["edges"] if v == 1) for f in enc.frames]
    assert rises == [n + 1] * 2


# ------------------------------------------------------------------- slave
SLAVE_TICK = 25                     # 0.5 us: TTO = 40 ticks = 20 us


def slave_on(backend, period, n, **master):
    """The slave on thread 0 (MA = IN0, SLO = OUT0) and a model master."""
    bench = biss_bench(backend)
    mst = bench.add(Master("IN0", "OUT0", period=period, n=n, **master))
    loom = boot(backend, bench, [(program(SLAVE), 0, SLAVE_TICK)])
    return bench, mst, loom


def set_slave(bench, loom, setting, pos, thread=0):
    loom.push(thread, [setting, pos & 0xFFFF, pos >> 16])
    bench.step(400)                 # taken between frames, and prepared


def frames(bench, mst, count, budget=40000):
    mst.read(count)
    assert bench.run_until(lambda: not mst.busy(), budget, every=32)
    return mst.results[-count:]


def check_slave_timing(results, period, sample):
    """Every SLO change inside its bit cell, before the master's sample
    point; the timeout TTO ticks from the closing rising edge, plus the
    slave's reaction (at most 5 slots and the 2-clock synchroniser)."""
    for r in results:
        assert r["latency"] < sample, r
    assert timeout_ok(results, TTO_TICKS * SLAVE_TICK, 24) == []


@pytest.mark.parametrize("period", [64, 128])
def test_slave_biss(backend, period):
    """A BiSS-C frame at MA 781 kHz and 391 kHz: Ack on rise 2, start, CDS 0,
    the position, nE = nW = 1, a good CRC; then a 32-bit and an 8-bit one."""
    bench, mst, loom = slave_on(backend, period, 24)
    set_slave(bench, loom, 24, 0xA5C3E1)
    res = frames(bench, mst, 2)
    for r in res:
        assert (r["error"], r["ack_rise"], r["cds"], r["pos"], r["nE"], r["nW"],
                r["crc_ok"]) == (None, 2, 0, 0xA5C3E1, 1, 1, 1), r
    check_slave_timing(res, period, period - 2)
    for n, pos in ((32, 0xDEADBEEF), (8, 0x5A)):
        mst.n = n
        set_slave(bench, loom, n, pos)
        r = frames(bench, mst, 1)[0]
        assert (r["error"], r["pos"], r["crc_ok"]) == (None, pos, 1), (n, r)


@model_only("SSI on the RTL is test_master_reads_slave[ssi=True] (25 bits, Gray)")
@pytest.mark.parametrize("n, gray_code", [(13, True), (25, False)])
def test_slave_ssi(backend, n, gray_code):
    """SSI at 781 kHz: the position MSB first on rises 1 .. n, binary or
    Gray, then tm (TTO ticks) low."""
    period = 64
    pos = 0x1234567 & ((1 << n) - 1)
    bench, mst, loom = slave_on(backend, period, n, ssi=True)
    set_slave(bench, loom, n | S_SSI | (S_GRAY if gray_code else 0), pos)
    res = frames(bench, mst, 2)
    want = gray(pos) if gray_code else pos
    assert [r["pos"] for r in res] == [want, want]
    assert from_gray(res[0]["pos"]) == pos if gray_code else True
    check_slave_timing(res, period, period - 2)


@model_only("ten frames with 20 us timeouts; ERR and WARN reach the RTL in "
            "test_master_reads_slave, the rest only change the bits sent")
@pytest.mark.parametrize("setting, expect", [
    (S_ERR, {"nE": 0, "nW": 1, "crc_ok": 1}),
    (S_WARN, {"nE": 1, "nW": 0, "crc_ok": 1}),
    (S_BADCRC, {"nE": 1, "nW": 1, "crc_ok": 0}),
    (2 << 13, {"nE": 1, "nW": 1, "crc_ok": 1}),
    (S_NOSTART, {"error": "no start bit"})])
def test_slave_errors(backend, setting, expect):
    """Error injection: nE, nW, the CRC sent un-inverted, a three-period Ack
    (still a good frame) and no start bit (the master gives up; the slave
    times out and answers the next frame)."""
    bench, mst, loom = slave_on(backend, 64, 16)
    set_slave(bench, loom, 16 | setting, 0xBEEF)
    r = frames(bench, mst, 1)[0]
    assert {k: r.get(k) for k in expect} == expect, r
    if "error" not in expect:
        assert r["pos"] == 0xBEEF
    set_slave(bench, loom, 16, 0x1234)
    r = frames(bench, mst, 1)[0]
    assert (r["error"], r["pos"], r["crc_ok"]) == (None, 0x1234, 1), r


# ----------------------------------------------------- master and slave
class EdgeLog(Wire):
    """The MA wire, also logging its edges (the cycle of each change)."""

    def __init__(self, src, dst):
        super().__init__(src, dst)
        self.edges = []

    def observe(self, lines):
        level = lines.get(self.src)
        if level != self.level:
            self.edges.append(lines.cycle)
        super().observe(lines)


@pytest.mark.parametrize("ssi", [False, True])
def test_master_reads_slave(backend, ssi):
    """The master on thread 0 reads the slave on thread 1 at MA 781 kHz
    (master TICK 32), the bench wiring OUT0 -> IN1 (MA) and OUT1 -> IN0
    (SLO): positions, Gray for SSI, the error bits for BiSS."""
    bench = biss_bench(backend)
    ma = bench.add(EdgeLog("OUT0", "IN1"))
    bench.add(Wire("OUT1", "IN0"))
    slave = program(SLAVE, thread=1, pins=("MA = IN0, SLO = OUT0", "MA = IN1, SLO = OUT1"))
    loom = boot(backend, bench, [(program(MASTER), 0, 32), (slave, 1, SLAVE_TICK)])
    n = 25 if ssi else 26
    kind = S_SSI | S_GRAY if ssi else 0
    configure(loom, n, ssi=ssi, gray_code=ssi)
    want_status = SSIF if ssi else 0
    for pos, flags, status in ((0x1ABCDEF, 0, want_status),
                               (0x0012345, 0 if ssi else S_ERR | S_WARN,
                                want_status if ssi else ERR | WARN)):
        set_slave(bench, loom, n | kind | flags, pos, thread=1)
        start = len(ma.edges)
        words = read_frame(loom)
        assert words[0] == status and position(words) == pos, [hex(w) for w in words]
        edges = ma.edges[start:]
        halves = {b - a for a, b in zip(edges, edges[1:])}
        assert halves == {32}, halves           # one frame, exact half periods
        assert len(edges) == 2 * ((n + 1) if ssi else (n + 13))
