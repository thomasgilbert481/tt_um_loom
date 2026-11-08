"""L3 for ``firmware/stepper_step_dir.loom``, on the golden model and the RTL.

The worked example of ``docs/TRY_IT.md``: a STEP/DIR stepper controller,
grown from the square wave ``python -m tools.loomnew`` generates. The
reference is ``tools.protomodels.stepper.StepDirDriver``, written from the
Allegro A4988 datasheet (Figure 1 and its table: tA, tB, tC, tD). Expected
values come from the datasheet and the program header (every edge P clocks
after the last, P raised to 50, the first rise 3 ticks after the command),
never from what the model or the RTL did.

RTL: ``bash scripts/try_rtl.sh test_fw_stepper``. The PHY-GL-SDF candidate
would be ``test_stepper_moves_forward_then_back``.
"""

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels import stepper as S
from tools.tests.fw_backend import model_only

ISA = load()
SOURCE = REPO / "firmware" / "stepper_step_dir.loom"
IMEM = 512
PMIN = 50                       # the clamp: tA = tB = 1 us at 50 MHz
STARTUP = 200


def bench_for(backend):
    if backend.name == "model":
        return backend.bench(features=("FIFO", "SETPD"), imem_words=IMEM)
    return backend.bench()


def setup(backend, clock_hz=50e6):
    bench = bench_for(backend)
    drv = bench.add(S.StepDirDriver("OUT0", "OUT1", clock_hz=clock_hz))
    loom = backend.loom(bench, isa=ISA)
    loom.load(assemble_file(SOURCE, isa=ISA, strict=True))
    loom.run(0)
    bench.step(STARTUP)
    return bench, drv, loom


def move(loom, period, steps):
    """One move; returns the reported position, signed."""
    loom.push(0, [period, steps & 0xFFFF])
    (pos,) = loom.pop(0, 1)
    return pos - 0x10000 if pos & 0x8000 else pos


def exact(drv, since, half, count, direction):
    """``count`` pulses after ``since``: high exactly ``half`` clocks, low
    exactly ``half`` between them, DIR as asked, no datasheet violation."""
    pulses = [p for p in drv.pulses if p[0] >= since]
    assert len(pulses) == count
    assert drv.highs(since) == [half] * count
    assert drv.lows(since) == [half] * (count - 1)
    assert all(p[2] == direction for p in pulses)
    assert drv.violations == []


def test_stepper_assembles_strict_with_no_diagnostic():
    """Every deadline pair proved at the 50-clock clamp, nothing unbounded;
    27 words, inside thread 0's quarter."""
    prog = assemble_file(SOURCE, isa=ISA, strict=True)
    assert prog.diagnostics == [] and list(prog.threads) == [0]
    rep = prog.deadlines[0]
    assert rep.period == PMIN and rep.pairs
    assert not rep.unbounded and not rep.infeasible and rep.worst_slack >= 0
    assert len(prog.words) == 27 and max(prog.words) < IMEM // 4


def test_stepper_moves_forward_then_back(backend):
    """Five steps with DIR high, then three with DIR low, at the clamp
    (500 k steps a second): the driver counts to 5 and back to 2, as the
    reports say; every edge exact; tC and tD kept."""
    bench, drv, loom = setup(backend)
    since = bench.cycle
    assert move(loom, PMIN, 5) == 5
    exact(drv, since, PMIN, 5, 1)
    since = bench.cycle
    assert move(loom, PMIN, -3) == 2
    exact(drv, since, PMIN, 3, 0)
    assert drv.position == 2
    rise = [p[0] for p in drv.pulses if p[0] >= since][0]
    assert rise - drv.dir_changes[-1] >= 3 * PMIN - 8     # tC, header: 3 ticks


def test_stepper_raises_a_short_period_to_the_clamp(backend):
    """P = 10 would give 200 ns pulses, under tA: the program sends 50."""
    bench, drv, loom = setup(backend)
    since = bench.cycle
    assert move(loom, 10, 2) == 2
    exact(drv, since, PMIN, 2, 1)


def test_stepper_zero_steps_sends_nothing(backend):
    bench, drv, loom = setup(backend)
    assert move(loom, PMIN, 0) == 0
    assert drv.pulses == [] and drv.dir_changes == []


@model_only("10 k steps a second: 100 k clocks; the clamp case runs on the RTL")
def test_stepper_slow_move(backend):
    bench, drv, loom = setup(backend)
    since = bench.cycle
    assert move(loom, 2500, -20) == -20
    exact(drv, since, 2500, 20, 0)
    assert drv.position == -20


@model_only("a check of the checker; the model is enough")
@pytest.mark.parametrize("clock_hz, ok", [(50e6, True), (60e6, False)])
def test_stepper_checker_flags_a_short_pulse(backend, clock_hz, ok):
    """The clamp's 50-clock pulse is exactly 1 us at 50 MHz, so it keeps
    tA and tB; on a 60 MHz board the same 50 clocks are 0.83 us and the
    reference flags every pulse. The windows are in seconds, the board
    clock is a parameter: that is how a test scales time."""
    bench, drv, loom = setup(backend, clock_hz=clock_hz)
    assert move(loom, PMIN, 2) == 2
    assert (drv.violations == []) == ok
    if not ok:
        assert any("(tA)" in v for v in drv.violations)
