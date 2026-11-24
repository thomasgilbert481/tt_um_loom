"""L3 for ``firmware/hc595_seg7.loom``, on the golden model and the RTL.

A 74HC595 driving a common-cathode 7-segment digit. The reference is
``tools.protomodels.hc595_seg7.HC595``, written from the SN74HC595 function
table and timing requirements. Expected segments come from the usual hex
font drawn segment by segment below (a top, b upper right, c lower right,
d bottom, e lower left, f upper left, g middle), never from the firmware's
table; expected timing comes from the program header (three ticks a bit,
SER T before the SRCLK rise, SRCLK high T, RCLK 2T after the last rise).

RTL: ``bash scripts/try_rtl.sh test_fw_hc595_seg7``; in CI through
``MODULES`` in ``test/test_fw.py`` (W-126). Written on 2026-11-19 by a fresh
agent playing a judge; W-126 changed the word count and slack in the
assembly test and added the 2 V column, fclock and 200 MHz checks.
"""

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels import hc595_seg7 as M
from tools.tests.fw_backend import model_only

ISA = load()
SOURCE = REPO / "firmware" / "hc595_seg7.loom"
IMEM = 512
TICK = 25                       # clocks a tick: SRCLK at 50 MHz / 75
TMIN = 24                       # the header's shortest proved tick
STARTUP = 200

FONT = ["abcdef", "bc", "abdeg", "abcdg", "bcfg", "acdfg", "acdefg", "abc",
        "abcdefg", "abcdfg", "abcefg", "cdefg", "adef", "bcdeg", "adefg", "aefg"]


def expected(word):
    """The segments the header says a host word lights."""
    lit = set(FONT[word & 15])
    if word & 0x10:
        lit.add("dp")
    return lit


def bench_for(backend):
    if backend.name == "model":
        return backend.bench(features=("FIFO", "SETPD", "DMEM"), imem_words=IMEM)
    return backend.bench()


def setup(backend, tick=TICK, clock_hz=50e6):
    bench = bench_for(backend)
    chip = bench.add(M.HC595("OUT0", "OUT1", "OUT2", clock_hz=clock_hz))
    loom = backend.loom(bench, isa=ISA)
    loom.load(assemble_file(SOURCE, isa=ISA, strict=True))
    loom.write_csr(0, "TICK_INT", tick)
    loom.run(0)
    bench.step(STARTUP)
    return bench, chip, loom


def show(bench, chip, loom, word, tick):
    """Send one digit; check the latch, the report and every edge."""
    since = bench.cycle
    n = len(chip.latches)
    loom.push(0, [word])
    (rep,) = loom.pop(0, 1)
    assert len(chip.latches) == n + 1
    cycle, byte = chip.latches[-1]
    assert M.segments(byte) == expected(word)
    assert rep == byte
    rises = [r for r in chip.srclk_rises if r >= since]
    assert len(rises) == 8
    assert [b - a for a, b in zip(rises, rises[1:])] == [3 * tick] * 7
    for c in (c for c in chip.ser_changes if c >= since):
        assert c + tick in rises                         # tsu: exactly one tick
    assert cycle - rises[-1] == 2 * tick                 # RCLK after the last rise
    assert chip.violations == []


def test_hc595_seg7_assembles_strict_with_no_diagnostic():
    """Every deadline pair proved at the 24-clock tick, nothing unbounded;
    60 words (44 code, 16 font), inside thread 0's quarter. Slack >= 5,
    not just >= 0: below 5 a staged edge can land late (program header,
    loomasm --staged-edges), and the staged check is clean."""
    prog = assemble_file(SOURCE, isa=ISA, strict=True, staged_check=True)
    assert prog.diagnostics == [] and list(prog.threads) == [0]
    assert [d for d in prog.warnings if "staged edge" in d.message] == []
    rep = prog.deadlines[0]
    assert rep.period == TMIN and rep.pairs
    assert not rep.unbounded and not rep.infeasible and rep.worst_slack >= 5
    assert len(prog.words) == 60 and max(prog.words) < IMEM // 4


def test_hc595_seg7_three_digits(backend):
    """3, 4, 8 with the decimal point, F: the bytes latched light the font's
    segments, the reports match, every edge on its tick. 4 (segment a dark)
    ends on a 0 bit, the path whose latch tail W-126 changed."""
    bench, chip, loom = setup(backend)
    for word in (0x03, 0x04, 0x18, 0x0F):
        show(bench, chip, loom, word, TICK)
    assert chip.storage & 0x7F == 0x71                    # F still shown


@model_only("all 32 combinations; three run on the RTL")
def test_hc595_seg7_every_digit_with_and_without_dp(backend):
    bench, chip, loom = setup(backend, tick=TMIN)
    for word in list(range(16)) + [0x10 | d for d in range(16)]:
        show(bench, chip, loom, word, TMIN)


@model_only("the same edges as the 4.5 V run, checked against another column")
def test_hc595_seg7_meets_the_2v_column_at_the_shortest_tick(backend):
    """Loom's pads are 3.3 V, between the datasheet's 2 V and 4.5 V columns.
    At T = 24 and 50 MHz the slower 2 V column (tw 100 ns, tsu 125 ns, RCLK
    94 ns, fclock 5 MHz) is met too: every window is 480 ns or more."""
    bench = bench_for(backend)
    chip = bench.add(M.HC595("OUT0", "OUT1", "OUT2", vcc=2.0))
    loom = backend.loom(bench, isa=ISA)
    loom.load(assemble_file(SOURCE, isa=ISA, strict=True))
    loom.write_csr(0, "TICK_INT", TMIN)
    loom.run(0)
    bench.step(STARTUP)
    for word in (0x08, 0x11, 0x1F):
        show(bench, chip, loom, word, TMIN)


@model_only("a host-word decode check; the model is enough")
def test_hc595_seg7_ignores_the_high_bits(backend):
    """0xFFE3: digit 3 (bits 3:0), bit 4 clear so no decimal point."""
    bench, chip, loom = setup(backend)
    show(bench, chip, loom, 0xFFE3, TICK)
    assert M.segments(chip.storage) == set("abcdg")


@model_only("a check of the checker; the model is enough")
@pytest.mark.parametrize("clock_hz, ok", [(50e6, True), (2e9, False)])
def test_hc595_seg7_checker_flags_a_short_pulse(backend, clock_hz, ok):
    """24 clocks are 480 ns at 50 MHz, inside every window; on a 2 GHz
    board they are 12 ns, under tw and tsu, and the reference says so."""
    bench, chip, loom = setup(backend, tick=TMIN, clock_hz=clock_hz)
    loom.push(0, [0x08])
    loom.pop(0, 1)
    assert (chip.violations == []) == ok
    if not ok:
        assert any("(tw)" in v for v in chip.violations)
        assert any("(tsu)" in v for v in chip.violations)
        assert any("(fclock)" in v for v in chip.violations)   # 36 ns < 40 ns


@model_only("a check of the checker; the model is enough")
@pytest.mark.parametrize("vcc, ok", [(4.5, True), (2.0, False)])
def test_hc595_seg7_checker_uses_the_vcc_column(backend, vcc, ok):
    """At 200 MHz a 24-clock tick is 120 ns: SER is set up 120 ns before
    the rise, inside 25 ns (4.5 V) but under 125 ns (2 V)."""
    bench = bench_for(backend)
    chip = bench.add(M.HC595("OUT0", "OUT1", "OUT2", clock_hz=200e6, vcc=vcc))
    loom = backend.loom(bench, isa=ISA)
    loom.load(assemble_file(SOURCE, isa=ISA, strict=True))
    loom.write_csr(0, "TICK_INT", TMIN)
    loom.run(0)
    bench.step(STARTUP)
    loom.push(0, [0x08])
    loom.pop(0, 1)
    assert (chip.violations == []) == ok
    if not ok:
        assert all("(tsu)" in v and "SER" in v for v in chip.violations)
