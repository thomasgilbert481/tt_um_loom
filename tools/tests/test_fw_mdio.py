"""L3 for the MDIO programs, on the model and the RTL.

``firmware/mdio_manager.loom`` (MDC = OUT0, MDIO = BIDIR0, open drain) is
the station manager; ``firmware/mdio_phy.loom`` (MDC = IN0, MDIO = BIDIR1,
open drain) is a PHY with 32 registers in data memory. The references are
``tools.protomodels.mdio``: ``MdioMonitor`` decodes the pads and checks
every edge against IEEE 802.3 clause 22.3.4 (MDC period >= 400 ns, high
and low >= 160 ns, manager set-up and hold >= 10 ns, PHY clock-to-output
0 to 300 ns), ``MdioPhy`` answers the manager program, and ``MdioManager``
drives the PHY program. Expected timings come from those figures and the
program headers' stated windows, never from what the model or RTL did.

Rate: 50 MHz core clock (``CLOCK_HZ``), the manager at TICK_INT 20, one MDC
period of 400 ns: 2.5 MHz, the fastest 802.3 allows. A frame is 64 MDC
cycles, 1280 clocks, so every body is cheap enough for the RTL except the
slow-rate sweep, which is ``model_only``.

The PHY-GL-SDF candidate is ``test_manager_and_phy_together``: two threads,
one open-drain line between two pads, a write and a read across at 2.5 MHz,
every MDC edge and every MDIO change checked against 22.3.4, in about
77,000 clocks with the host transfers (the frames are about 6,000).
"""

import pytest

from tools.loomasm import assemble_file, assemble_text
from tools.loomisa import REPO, load
from tools.protomodels import mdio as M
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
IMEM = 512
MANAGER, PHY = FIRMWARE / "mdio_manager.loom", FIRMWARE / "mdio_phy.loom"
CLOCK_HZ = 50e6
TICK = 20                       # one MDC period: 400 ns, 2.5 MHz
NO_PHY, EARLY_TA = 0x1, 0x2     # manager status bits (its header)
REGS = 0x180                    # the PHY's register file (its header)
STARTUP = 300                   # clocks for a program's set-up after RUN


def program(path, thread=0):
    """A program from ``firmware/`` with ``--strict``, moved to ``thread``."""
    if thread == 0:
        return assemble_file(path, isa=ISA, strict=True)
    text = path.read_text(encoding="utf-8")
    assert text.count(".thread 0") == 1
    text = text.replace(".thread 0", ".thread %d" % thread)
    return assemble_text(text, path.name, isa=ISA, imem_words=IMEM, strict=True)


def mbench(backend, pullups):
    """FIFOs, the bit engine and data memory, 512 words; the RTL has them
    all and ``RtlBench`` takes neither keyword."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "BE", "BEENC", "DMEM"),
                             imem_words=IMEM, pullups=pullups)
    return backend.bench(pullups=pullups)


def c22(op, phy, reg):
    return M.header(M.ST_C22, op, phy, reg)


def c45(op, prtad, devad):
    return M.header(M.ST_C45, op, prtad, devad)


def mgr_read(loom, word):
    """One read frame: ``(data, status)``."""
    loom.push(0, [word])
    data, status = loom.pop(0, 2)
    return data, status


def mgr_write(loom, word, data):
    """One frame the manager drives (a write, a clause 45 address): status."""
    loom.push(0, [word, data])
    return loom.pop(0, 1)[0]


def edges(mon):
    """``(highs, lows, periods)`` of MDC in clocks, from the monitor."""
    rises, falls = mon.rises, mon.falls
    highs = [f - r for r, f in zip(rises, falls) if f > r]
    lows = [r - f for f, r in zip(falls, rises[1:]) if r > f]
    periods = [b - a for a, b in zip(rises, rises[1:])]
    return highs, lows, periods


def check_c22_timing(mon, tick):
    """22.3.4 at 50 MHz: period >= 20 clocks, high and low >= 8 clocks; the
    manager's own schedule (its header): high exactly 8 at TICK 20, low 12,
    and every period between two bits of one part of a frame exactly one
    tick."""
    assert mon.violations == []
    highs, lows, periods = edges(mon)
    assert min(highs) >= 8 and min(lows) >= 8 and min(periods) >= 20
    assert min(periods) == tick
    assert min(lows) == 12


# ------------------------------------------------------------- assembly
def test_mdio_programs_assemble_strict_with_no_diagnostic():
    """L2-DEADLINE: the manager's every pair is proved at its .tick of 20
    clocks (2.5 MHz), nothing unbounded and no ``.bounded``; the PHY has no
    deadline pair by design (it follows MDC). Each fits its quarter, the
    PHY's register file is 0x180..0x19F, and with the PHY moved to thread 1
    the two images do not overlap."""
    mgr = program(MANAGER)
    assert mgr.diagnostics == [] and list(mgr.threads) == [0]
    report = mgr.deadlines[0]
    assert report.period == 20
    assert report.pairs and not report.infeasible and not report.unbounded
    assert report.worst_slack >= 0
    assert len(mgr.words) == 66 and max(mgr.words) < 0x80
    phy = program(PHY)
    assert phy.diagnostics == [] and phy.deadlines[0].pairs == []
    code = [a for a in phy.words if a < 0x180]
    assert len(code) == 123 and max(code) < 0x80
    assert sorted(a for a in phy.words if a >= 0x180) == list(range(0x180, 0x1A0))
    moved = program(PHY, thread=1)
    moved_code = [a for a in moved.words if a < 0x180]
    assert min(moved_code) == 0x80 and max(moved_code) < 0x100
    assert not set(mgr.words) & set(moved.words)


# ------------------------------------------------- the manager, model PHY
def manager_setup(backend, tick=TICK, **phy):
    bench = mbench(backend, pullups=0b1)
    phy.setdefault("phyad", 3)
    port = bench.add(M.MdioPhy("OUT0", "BIDIR0", **phy))
    mon = bench.add(M.MdioMonitor("OUT0", "BIDIR0", clock_hz=CLOCK_HZ))
    loom = backend.loom(bench, isa=ISA)
    loom.load(program(MANAGER))
    loom.write_csr(0, "TICK_INT", tick)
    loom.run(0)
    return bench, port, mon, loom


def test_manager_writes_and_reads_clause_22(backend):
    """A write then a read of the same register at 2.5 MHz. The model PHY
    answers 240 ns after each rise (the limit the manager header states
    for TICK_INT 20). Frames decoded off the pads: 32-bit preambles, ST 01,
    the right OP, addresses, TA and data; every edge inside 22.3.4."""
    bench, port, mon, loom = manager_setup(backend, delay=12, regs={2: 0x0022})
    assert mgr_write(loom, c22(M.OP_WRITE, 3, 9), 0xBEEF) == 0
    assert port.regs[9] == 0xBEEF
    assert mgr_read(loom, c22(M.OP_READ, 3, 9)) == (0xBEEF, 0)
    assert mgr_read(loom, c22(M.OP_READ, 3, 2)) == (0x0022, 0)
    w, r = mon.frames[0], mon.frames[1]
    assert w["preamble"] == r["preamble"] == 32
    assert (w["st"], w["op"], w["phyad"], w["regad"], w["ta"], w["data"]) == \
        (1, M.OP_WRITE, 3, 9, 0b10, 0xBEEF)
    assert (r["st"], r["op"], r["phyad"], r["regad"], r["ta"], r["data"]) == \
        (1, M.OP_READ, 3, 9, 0b10, 0xBEEF)
    check_c22_timing(mon, TICK)
    assert bench.contention_free() and loom.badop() == 0


def test_manager_reads_the_slowest_phy_from_tick_24(backend):
    """A PHY at the full 300 ns clock-to-output (15 clocks): the manager
    reads it right from TICK_INT 24 (2.08 MHz), as its header says (23
    would dither to 20-clock periods, SEMANTICS 2)."""
    bench, port, mon, loom = manager_setup(backend, tick=24, delay=15,
                                           regs={1: 0x7909})
    assert mgr_read(loom, c22(M.OP_READ, 3, 1)) == (0x7909, 0)
    assert mon.violations == [] and min(edges(mon)[2]) == 24


@model_only("documents the stated limit; the RTL proves the passing side")
def test_manager_at_2_5_mhz_misses_a_phy_slower_than_240_ns(backend):
    """The weakness the header states: at TICK_INT 20 the sample is 8
    clocks before the rise, so a PHY that takes 13 clocks (260 ns, inside
    802.3's 300) is read one bit late."""
    bench, port, mon, loom = manager_setup(backend, delay=13, regs={1: 0x7909})
    data, status = mgr_read(loom, c22(M.OP_READ, 3, 1))
    assert data != 0x7909
    assert mon.frames[-1]["data"] == 0x7909    # the bus itself was right


def test_manager_reports_no_phy(backend):
    """Nobody answers: TA[0] stays high (the pull-up), status bit 0, data
    0xFFFF; a write to nobody completes (a write has no answer)."""
    bench, port, mon, loom = manager_setup(backend, absent=True)
    assert mgr_read(loom, c22(M.OP_READ, 3, 0)) == (0xFFFF, NO_PHY)
    assert mgr_write(loom, c22(M.OP_WRITE, 3, 0), 0x1234) == 0
    assert port.regs == {} and bench.contention_free()


def test_manager_reports_a_phy_driving_the_first_turnaround_bit(backend):
    """Error injection: the PHY pulls TA[1] low too, which nobody may: the
    status says so (bit 1); the data still arrives."""
    bench, port, mon, loom = manager_setup(backend, early_ta=True, regs={4: 0x5A5A})
    assert mgr_read(loom, c22(M.OP_READ, 3, 4)) == (0x5A5A, EARLY_TA)


def test_manager_makes_clause_45_frames(backend):
    """Clause 45 (ST = 00): an address frame, a write, a read and two
    read-increments to MMD 1 of port 3. The program never looks at ST; OP[1]
    alone says who drives the data, which is true of both clauses."""
    bench, port, mon, loom = manager_setup(backend, delay=12)
    port.mmd[(1, 0x0008)] = 0x0001
    assert mgr_write(loom, c45(M.OP45_ADDR, 3, 1), 0x0007) == 0
    assert mgr_write(loom, c45(M.OP45_WRITE, 3, 1), 0xC0DE) == 0
    assert port.mmd[(1, 0x0007)] == 0xC0DE
    assert mgr_read(loom, c45(M.OP45_READ, 3, 1)) == (0xC0DE, 0)
    assert mgr_read(loom, c45(M.OP45_RINC, 3, 1)) == (0xC0DE, 0)
    assert mgr_read(loom, c45(M.OP45_RINC, 3, 1)) == (0x0001, 0)
    assert port.mmd_addr[1] == 0x0009
    assert [f["st"] for f in mon.frames] == [0] * 5
    assert [f["op"] for f in mon.frames] == [0, 1, 3, 2, 2]
    assert all(f["preamble"] == 32 for f in mon.frames)
    assert mon.violations == [] and bench.contention_free()


@model_only("400 kHz and 1 MHz: 8,000 clocks a frame; 2.5 MHz covers the RTL")
@pytest.mark.parametrize("tick", [50, 125])
def test_manager_at_slower_rates(backend, tick):
    """MDC at 1 MHz and 400 kHz (50 and 125 clocks, not multiples of the
    4-clock slot, so the periods dither between the multiples of 4 either
    side, SEMANTICS 2): the low phase stays 12 clocks and the high phase
    takes the rest (the header: the WAITD sits in the high phase); 22.3.4
    holds."""
    bench, port, mon, loom = manager_setup(backend, tick=tick, delay=15,
                                           regs={7: 0x0F0F})
    assert mgr_write(loom, c22(M.OP_WRITE, 3, 8), 0xA5A5) == 0
    assert mgr_read(loom, c22(M.OP_READ, 3, 7)) == (0x0F0F, 0)
    highs, lows, periods = edges(mon)
    grid = 4 * (tick // 4)
    assert min(periods) == grid and min(lows) == 12
    assert min(highs) == grid - 12
    assert mon.violations == []


# ------------------------------------------------- the PHY, model manager
class Edges(M.Model):
    """MDC rise cycles and MDIO change cycles, to measure the PHY's output
    delay on the pins."""

    def __init__(self, mdc, mdio):
        self.mdc, self.io = M.pad_of(mdc), M.pad_of(mdio)
        self.prev = None
        self.rises, self.changes = [], []

    def observe(self, lines):
        now = (lines.get(self.mdc), lines.get(self.io))
        if self.prev is not None:
            if now[0] and not self.prev[0]:
                self.rises.append(lines.cycle)
            if now[1] != self.prev[1]:
                self.changes.append(lines.cycle)
        self.prev = now

    def delays(self, first, last):
        """Clocks from the latest rise before each change, for changes in
        cycles first..last."""
        out = []
        for c in self.changes:
            if first <= c <= last:
                out.append(c - max(r for r in self.rises if r <= c))
        return out


def phy_setup(backend, phyad=5, period=TICK, gap=40):
    bench = mbench(backend, pullups=0b10)
    mgr = bench.add(M.MdioManager("IN0", "BIDIR1", period=period, gap=gap))
    mon = bench.add(M.MdioMonitor("IN0", "BIDIR1", clock_hz=CLOCK_HZ))
    rec = bench.add(Edges("IN0", "BIDIR1"))
    loom = backend.loom(bench, isa=ISA)
    loom.load(program(PHY))
    loom.run(0)
    loom.push(0, [phyad])
    bench.step(STARTUP)
    return bench, mgr, mon, rec, loom


def run_frames(bench, mgr, frames):
    assert bench.run_until(lambda: not mgr.busy, frames * 64 * TICK * 2 + 2000,
                           every=32)


def test_phy_answers_reads_and_takes_writes_at_2_5_mhz(backend):
    """The model manager at 2.5 MHz (200 ns high, 200 ns low, MDIO changed at
    the fall) reads the reset values and the identifier, writes a register
    and reads it back. The PHY reports the write, the word lands in data
    memory, and every bit it drives changes 8 to 11 clocks (160 to 220 ns)
    after the rise, as its header says: inside 22.3.4's 0 to 300 ns."""
    bench, mgr, mon, rec, loom = phy_setup(backend)
    for reg in (0, 1, 2, 3):
        mgr.read(5, reg)
    mgr.write(5, 9, 0x1234)
    mgr.read(5, 9)
    run_frames(bench, mgr, 6)
    got = [(r["regad"], r["ta"], r["data"]) for r in mgr.results]
    assert got[:4] == [(0, 0b10, 0x1140), (1, 0b10, 0x7909),
                       (2, 0b10, 0x4C4F), (3, 0b10, 0x4F4D)]
    assert got[5] == (9, 0b10, 0x1234)
    assert loom.pop(0, 2) == [9, 0x1234]
    assert mon.violations == [] and bench.contention_free()
    for f in mon.frames:
        if f["op"] == M.OP_READ:
            d = rec.delays(f["rises"][14], f["end"])
            assert d and min(d) >= 8 and max(d) <= 11, d
    loom.halt(0)
    assert loom.read_imem(REGS + 9) == [0x1234]
    assert loom.badop() == 0


def test_phy_follows_frames_with_no_gap(backend):
    """Back to back, no idle cycle between frames: the PHY counts the next
    preamble while it stores and reports a write, and answers every frame
    (its header: 'frames may follow each other with no gap')."""
    bench, mgr, mon, rec, loom = phy_setup(backend, gap=0)
    mgr.write(5, 4, 0xAAAA)
    mgr.read(5, 4)
    mgr.write(5, 31, 0x8001)
    mgr.read(5, 31)
    mgr.read(5, 0)
    run_frames(bench, mgr, 5)
    assert [r["data"] for r in mgr.results if r["op"] == M.OP_READ] == \
        [0xAAAA, 0x8001, 0x1140]
    assert loom.pop(0, 4) == [4, 0xAAAA, 31, 0x8001]
    assert mon.violations == [] and bench.contention_free()


def test_phy_ignores_other_frames(backend):
    """Frames it must not answer: another PHY address (read and write), a
    31-bit preamble (22.2.4.5.1: the PHY shall see 32 ones), and clause 45
    (ST = 00). Nobody answers (TA and data are the pull-up), nothing is
    stored or reported; the next proper frame is answered. A PHY that
    ignores a frame hunts through the rest of it, so a 1 at the end of
    that frame counts towards the next preamble (they are contiguous ones,
    each with its rise): the frames before the short preambles end in 0."""
    bench, mgr, mon, rec, loom = phy_setup(backend)
    mgr.read(6, 0)
    mgr.write(4, 9, 0x5554)            # ends in 0: see the docstring
    mgr.write(5, 9, 0x6666, preamble=31)   # ends in 0 too
    mgr.read(5, 0, preamble=31)
    mgr.read(5, 0, st=M.ST_C45, op=M.OP45_READ)
    mgr.read(5, 9)
    run_frames(bench, mgr, 6)
    got = [(r["ta"], r["data"]) for r in mgr.results]
    # a write's TA and data are the manager's own; a read nobody answers
    # reads the pull-up
    assert got == [(0b11, 0xFFFF), (0b10, 0x5554), (0b10, 0x6666),
                   (0b11, 0xFFFF), (0b11, 0xFFFF), (0b10, 0x0000)]
    assert loom.pop_available(0) == []
    assert mon.violations == [] and bench.contention_free()


def test_phy_stores_a_write_with_a_bad_turnaround(backend):
    """Error injection on the manager's side: TA = 00 on a write. The
    header says TA is not checked on a write, so the word is stored and
    reported like any other (a dedicated PHY might drop it)."""
    bench, mgr, mon, rec, loom = phy_setup(backend)
    mgr.write(5, 12, 0x0BAD, ta=(0, 0))
    mgr.read(5, 12)
    run_frames(bench, mgr, 2)
    assert mgr.results[1]["data"] == 0x0BAD
    assert loom.pop(0, 2) == [12, 0x0BAD]


# ------------------------------------------------------------- together
def test_manager_and_phy_together(backend):
    """The manager on thread 0 (MDC = OUT0, MDIO = BIDIR0) and the PHY on
    thread 1 (MDC = IN0, MDIO = BIDIR1), the bench wiring OUT0 to IN0 and
    BIDIR0 to BIDIR1 as one open-drain board line, pulled up at both pads.
    At 2.5 MHz the manager writes a register and reads it and the
    identifier back; the PHY reports the write; every edge on the manager's
    pads is inside 22.3.4. The PHY-GL-SDF candidate."""
    bench = mbench(backend, pullups=0b11)
    bench.add(M.Wire("OUT0", "IN0"))
    bench.add(M.OpenDrainWire("BIDIR0", "BIDIR1"))
    mon = bench.add(M.MdioMonitor("OUT0", "BIDIR0", clock_hz=CLOCK_HZ))
    loom = backend.loom(bench, isa=ISA)
    words = dict(program(MANAGER).words)
    phy = program(PHY, thread=1).words
    assert not set(words) & set(phy)
    words.update(phy)
    loom.load(words)
    loom.write_csr(0, "TICK_INT", TICK)
    loom.push(1, [5])
    loom.run([0, 1])
    bench.step(STARTUP)
    assert mgr_write(loom, c22(M.OP_WRITE, 5, 17), 0x2026) == 0
    assert loom.pop(1, 2) == [17, 0x2026]
    assert mgr_read(loom, c22(M.OP_READ, 5, 17)) == (0x2026, 0)
    assert mgr_read(loom, c22(M.OP_READ, 5, 2)) == (0x4C4F, 0)
    assert mgr_read(loom, c22(M.OP_READ, 4, 2)) == (0xFFFF, NO_PHY)
    check_c22_timing(mon, TICK)
    assert [f["phyad"] for f in mon.frames] == [5, 5, 5, 4]
    assert bench.contention_free() and loom.badop() == 0


# ------------------------------------------------- the reference model alone
def test_model_frame_format():
    """22.2.4.5: 32 ones, ST 01, OP, PHYAD, REGAD MSB first, TA 10 on a
    write, released (None) from TA on a read; the decoder inverts it."""
    bits = M.frame_bits(M.ST_C22, M.OP_WRITE, 0b10101, 0b00011, 0x8001)
    assert len(bits) == 64 and bits[:32] == [1] * 32
    assert bits[32:46] == [0, 1, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 1]
    assert bits[46:48] == [1, 0] and bits[48] == 1 and bits[63] == 1
    assert M.decode_bits(bits[32:]) == dict(st=1, op=1, phyad=21, regad=3,
                                             ta=2, data=0x8001)
    read = M.frame_bits(M.ST_C22, M.OP_READ, 1, 2)
    assert read[46:] == [None] * 18
    assert M.header(M.ST_C45, M.OP45_ADDR, 3, 1) >> 14 == 0


def test_model_manager_and_phy_agree_and_the_monitor_catches_faults():
    """The two models on one bench, at 2.5 MHz; then the monitor flags a
    manager that breaks 22.3.4 (MDC high 140 ns)
    and a PHY slower than 300 ns."""
    from tools.protomodels.bench import Bench
    bench = Bench(features=("FIFO",), pullups=0b1)
    mgr = bench.add(M.MdioManager(("ui", 0), "BIDIR0", period=20))
    phy = bench.add(M.MdioPhy(("ui", 0), "BIDIR0", phyad=2, regs={5: 0x1357}))
    mon = bench.add(M.MdioMonitor(("ui", 0), "BIDIR0"))
    mgr.write(2, 6, 0x2468)
    mgr.read(2, 5)
    mgr.read(2, 6, preamble=30)                 # ignored: nobody answers
    assert bench.run_until(lambda: not mgr.busy, 10000, every=16)
    assert [r["data"] for r in mgr.results] == [0x2468, 0x1357, 0xFFFF]
    assert phy.regs[6] == 0x2468 and mon.violations == []
    assert [f["preamble"] for f in mon.frames] == [32, 32, 30]
    bad = Bench(features=("FIFO",), pullups=0b1)
    mgr = bad.add(M.MdioManager(("ui", 0), "BIDIR0", period=20, high=7))
    bad.add(M.MdioPhy(("ui", 0), "BIDIR0", phyad=2, delay=16))
    mon = bad.add(M.MdioMonitor(("ui", 0), "BIDIR0"))
    mgr.read(2, 5)
    assert bad.run_until(lambda: not mgr.busy, 10000, every=16)
    kinds = {text.split(" ")[0] + " " + text.split(" ")[1] for _, text in mon.violations}
    assert "MDC high" in kinds
    assert any("PHY output" in text for _, text in mon.violations)
