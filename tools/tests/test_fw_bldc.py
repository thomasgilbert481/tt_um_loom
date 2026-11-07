"""L3 for ``firmware/bldc_6step.loom``, six-step BLDC commutation, model and RTL.

Thread 0 drives the six gates (AL, AH, BL, BH, CL, CH = OUT0..OUT5) from
the Hall code on IN0..IN2 by AN857's FwdTable or RevTable, with high-side
PWM of 250 ticks a period; thread 1 measures the time between sector
changes. The references are ``tools.protomodels.bldc``: the tables and the
clockwise sequence from AN857 Figure 4, a scripted Hall source, a motor
whose sector advances by the drive applied, and a gate checker (never both
switches of a phase, the dead time, the right pair for the Hall code).

Expected timings come from the program's stated contract and AN857, never
from what the model or the RTL did: a PWM period is exactly 250 ticks and
a pulse exactly ``duty`` ticks (deadline-latched edges); the dead time is
at least the dead-time word in ticks; every gate is off within ``TRIP``
clocks of an invalid code (the header's bound); a duty takes effect at the
next period; a sector's reported length is its length in thread 1 ticks to
within one tick and the detection jitter the header states.

The PHY-GL-SDF candidate is ``test_bldc_dead_time_on_a_reversal``: one
command, one H-to-L switch of a phase, every gate edge checked to the clock.
"""

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels import bldc as M
from tools.tests.fw_backend import model_only

ISA = load()
IMEM = 512
SOURCE = REPO / "firmware" / "bldc_6step.loom"

TICK0 = 10                  # thread 0: 250 ticks = 2500 clocks, 20 kHz at 50 MHz
TICK1 = 50                  # thread 1: 1 us at 50 MHz
PERIOD = 250                # ticks (header)
DMIN, DMAX = 12, 222        # duty clamp (header)
DEADT = 5                   # the default dead-time word, ticks
TRIP = 210                  # header: every gate off within 206 clocks, + pad
LATE = 60                   # header: the first rise of a sector, after its tick
JITTER1 = 64                # header: a Hall change is seen by thread 1 within 63 clocks
REV_BIT = 0x8000
NONE = 0xFFFF
AH, BH, CH = 1, 3, 5        # gate bits of the high sides
STARTUP = 400


def program():
    prog = assemble_file(SOURCE, isa=ISA, strict=True)
    assert prog.diagnostics == []
    return prog


def bench_for(backend):
    if backend.name == "model":
        return backend.bench(features=("FIFO", "SETPD", "DMEM"), imem_words=IMEM)
    return backend.bench()


def boot(backend, hall, *, deadt=DEADT, tick0=TICK0, tick1=TICK1, dead=None):
    """Load and run both threads with ``hall`` (a model) on the Hall pins and
    a gate checker; the dead-time word patched to ``deadt``."""
    bench = bench_for(backend)
    bench.add(hall)
    checker = bench.add(M.GateChecker(dead=(deadt * tick0) if dead is None else dead,
                                      latency=TRIP))
    prog = program()
    words = dict(prog.words)
    words[prog.symbols["DEADT"]] = deadt
    loom = backend.loom(bench, isa=ISA)
    loom.load(words)
    for thread, tick in ((0, tick0), (1, tick1)):
        loom.write_csr(thread, "TICK_INT", tick)
        loom.write_csr(thread, "TICK_FRAC", 0)
    loom.run([0, 1])
    bench.step(STARTUP)
    return bench, checker, loom


def command(bench, loom, checker, duty, reverse=False):
    loom.push(0, [duty | (REV_BIT if reverse else 0)])
    if reverse != checker.reverse:
        # read once a period, after the falling edge (header)
        checker.set_reverse(reverse, bench.cycle, grace=PERIOD * TICK0)


def report(loom):
    """Thread 1's reply: (length in ticks, status word)."""
    loom.push(1, [0])
    return tuple(loom.pop(1, 2))


def gates_between(checker, bench_cycle_from, bench_cycle_to):
    """Every gate bit that was on at some clock in [from, to)."""
    seen = 0
    for bit, edges in checker.edges.items():
        level = 0
        for cyc, lvl in edges:
            if cyc >= bench_cycle_to:
                break
            if cyc < bench_cycle_from:
                level = lvl
                continue
            seen |= lvl << bit
            level = lvl
        seen |= level << bit
    return seen


def test_bldc_program_assembles_strict_with_no_diagnostic():
    """--strict with no diagnostic; no deadline pair (thread 0 schedules its
    edges through CSRW TD and the latch, thread 1 only tests); 207 words in
    threads 0 and 1's quarters; the tables in the image are AN857's, and
    RevTable is FwdTable with high and low swapped (Table 3's rule)."""
    prog = program()
    assert sorted(prog.threads) == [0, 1]
    assert all(r.pairs == [] for r in prog.deadlines.values())
    assert len(prog.words) == 207 and max(prog.words) < 0x100
    assert len([a for a in prog.words if a < 0x80]) == 121
    table = prog.symbols["TABLE"]
    assert tuple(prog.words[table + i] for i in range(8)) == M.FWD
    assert tuple(prog.words[table + 8 + i] for i in range(8)) == M.REV
    assert all(M.REV[c] == M.swap_high_low(M.FWD[c]) for c in range(8))
    pos = prog.symbols["POS"]
    assert all(prog.words[pos + c] == (6 if M.position(c) is None else M.position(c))
               for c in range(8))
    assert prog.words[prog.symbols["DEADT"]] == DEADT


def level_through(checker, bit, a, b):
    """The level of gate ``bit`` if it is constant over [a, b), else None."""
    level = 0
    for cyc, lvl in checker.edges[bit]:
        if cyc <= a:
            level = lvl
        elif cyc < b:
            return None
    return level


def wait_edge(bench, checker, bit, level, limit=6000):
    """Step until gate ``bit`` next changes to ``level``; return that cycle."""
    count = sum(1 for _, lvl in checker.edges[bit] if lvl == level)
    ok = bench.run_until(
        lambda: sum(1 for _, lvl in checker.edges[bit] if lvl == level) > count,
        limit, every=4)
    assert ok, "gate %s never went %d" % (M.GATE_NAMES[bit], level)
    return [c for c, lvl in checker.edges[bit] if lvl == level][-1]


@pytest.mark.parametrize("reverse", [False, True])
def test_bldc_table_in_both_directions(backend, reverse):
    """The six valid codes in the order the motor would turn (clockwise for
    FwdTable, counter-clockwise for RevTable), each held two PWM periods:
    once the dead time is over, the gates are exactly the AN857 word for
    the code, its low side on throughout, its high side pulsing; no gate
    rule is broken."""
    hall = M.HallSource(code=5)
    bench, checker, loom = boot(backend, hall)
    command(bench, loom, checker, 100, reverse)
    bench.step(3000)
    hold = 2 * PERIOD * TICK0 + 200
    order = [M.step(5, -k if reverse else k) for k in range(1, 7)]
    start = bench.cycle
    hall.schedule([(start + i * hold, code) for i, code in enumerate(order)])
    bench.step(len(order) * hold)
    for i, code in enumerate(order):
        a = start + i * hold + TRIP + (DEADT + 1) * TICK0 + 100
        b = start + (i + 1) * hold
        word = M.drive_word(code, reverse)
        assert gates_between(checker, a, b) == word, "code %d" % code
        for bit in range(6):
            if word >> bit & 1 and not bit & 1:
                assert level_through(checker, bit, a, b) == 1
        high = [bit for bit in (AH, BH, CH) if word >> bit & 1][0]
        assert level_through(checker, high, a, b) is None      # pulsing
    assert checker.violations == []


def test_bldc_pwm_edges_are_exact(backend):
    """Duty 100 of 250 at TICK 10: after the first pulse of the sector every
    high-side pulse is exactly 1000 clocks and the rises exactly 2500 apart
    (the deadline latch); the low side stays on."""
    hall = M.HallSource(code=5)                      # FwdTable: AH and BL
    bench, checker, loom = boot(backend, hall)
    command(bench, loom, checker, 100)
    bench.step(7 * PERIOD * TICK0)
    pulses = checker.pulses(AH)
    assert len(pulses) >= 6
    # The sector's first rise is the ordinary OUT after the dead time, up
    # to LATE clocks after the tick that starts the period; its fall and
    # every later edge are latched on their ticks.
    first = pulses[0]
    late = PERIOD * TICK0 - (pulses[1][0] - first[0])
    assert 0 <= late <= LATE
    assert first[1] - first[0] == 100 * TICK0 - late
    steady = pulses[1:]
    assert all(f - r == 100 * TICK0 for r, f in steady)
    assert all(b[0] - a[0] == PERIOD * TICK0 for a, b in zip(steady, steady[1:]))
    assert level_through(checker, 2, pulses[1][0], bench.cycle) == 1   # BL
    assert checker.violations == []


def test_bldc_dead_time_on_a_reversal(backend):
    """Code 5 forward (AH pulsing, BL on), then a reversal: RevTable's word
    for 5 is BH and AL, so phase A goes high to low and phase B low to high
    directly. Each switch turns on at least the dead time (5 ticks, 50
    clocks) after its partner turned off; the all-off gap of the
    commutation is at most the header's bound."""
    hall = M.HallSource(code=5)
    bench, checker, loom = boot(backend, hall)
    command(bench, loom, checker, 100)
    bench.step(2 * PERIOD * TICK0)
    wait_edge(bench, checker, AH, 1)
    command(bench, loom, checker, 100, reverse=True)
    al_on = wait_edge(bench, checker, 0, 1)
    bh_on = [c for c, lvl in checker.edges[BH] if lvl == 1][-1]
    ah_off = [c for c, lvl in checker.edges[AH] if lvl == 0][-1]
    bl_off = [c for c, lvl in checker.edges[2] if lvl == 0][-1]
    assert ah_off < al_on and bl_off < bh_on
    # The command is read after a falling edge, so AH is already off by
    # PWM when the commutation writes all six off (BL's fall); both new
    # switches come on in one OUT, the dead time after that.
    assert al_on == bh_on
    assert al_on - ah_off >= DEADT * TICK0
    assert DEADT * TICK0 <= bh_on - bl_off <= (DEADT + 1) * TICK0 + 80
    bench.step(PERIOD * TICK0)
    assert checker.violations == []


def all_off_at(checker, after):
    """The first cycle at or after ``after`` with every gate off, and stays."""
    on = 0
    first = None
    events = sorted((c, bit, lvl) for bit, e in checker.edges.items() for c, lvl in e)
    for bit in range(6):
        on |= (level_through(checker, bit, after - 1, after) or 0) << bit
    if not on:
        first = after
    for c, bit, lvl in events:
        if c < after:
            continue
        on = (on | (1 << bit)) if lvl else (on & ~(1 << bit))
        if on and first is not None:
            first = None
        if not on and first is None:
            first = c
    return first


def test_bldc_invalid_code_turns_everything_off(backend):
    """000 and 111, each entered at a different point of the PWM period (a
    pulse just begun, half way through the off time): every gate is off
    within TRIP clocks and stays off while the code is invalid; a valid
    code brings its word back after the dead time."""
    hall = M.HallSource(code=4)                      # FwdTable: CH and BL
    bench, checker, loom = boot(backend, hall)
    command(bench, loom, checker, 150)
    bench.step(2 * PERIOD * TICK0)
    for bad, offset in ((0, 8), (7, 2100)):
        rise = wait_edge(bench, checker, CH, 1)
        at = rise + offset
        bench.step(max(0, at - bench.cycle))
        hall.schedule([(at, bad)])
        bench.step(3000)
        changed = hall.changes[-1][0]
        off = all_off_at(checker, changed)
        assert off is not None and off - changed <= TRIP
        assert gates_between(checker, off, bench.cycle) == 0
        hall.schedule([(bench.cycle, 4)])
        bench.step(1200)
        back = hall.changes[-1][0]
        a = back + TRIP + (DEADT + 1) * TICK0 + 100
        assert gates_between(checker, a, bench.cycle) == M.FWD[4]
    assert checker.violations == []


def test_bldc_duty_change_takes_effect_at_the_next_period(backend):
    """A new duty pushed just after a rise: that period keeps the old width,
    the next ones have the new one, the period itself unchanged. Then duty
    0 coasts (all off) and a duty again drives."""
    hall = M.HallSource(code=6)                      # FwdTable: CH and AL
    bench, checker, loom = boot(backend, hall)
    command(bench, loom, checker, 200)
    bench.step(2 * PERIOD * TICK0)
    rise = wait_edge(bench, checker, CH, 1)
    command(bench, loom, checker, 40)
    assert bench.cycle < rise + 200 * TICK0         # pushed before the fall
    bench.step(3 * PERIOD * TICK0)
    widths = [f - r for r, f in checker.pulses(CH, since=rise)]
    assert widths[0] == 200 * TICK0
    assert widths[1:3] == [40 * TICK0, 40 * TICK0]
    rises = [r for r, _ in checker.pulses(CH, since=rise)]
    assert all(b - a == PERIOD * TICK0 for a, b in zip(rises, rises[1:]))
    command(bench, loom, checker, 0)
    bench.step(2 * PERIOD * TICK0)
    t = bench.cycle
    bench.step(PERIOD * TICK0)
    assert gates_between(checker, t, bench.cycle) == 0
    command(bench, loom, checker, 100)
    bench.step(2 * PERIOD * TICK0)
    t = bench.cycle
    bench.step(PERIOD * TICK0)
    assert gates_between(checker, t, bench.cycle) == M.FWD[6]
    assert checker.violations == []


@model_only("the clamp is two branches the other duty tests cover on the RTL")
def test_bldc_duty_is_clamped(backend):
    """1..11 raise to DMIN, 223..255 lower to DMAX, bits 14:8 ignored."""
    hall = M.HallSource(code=5)
    bench, checker, loom = boot(backend, hall)
    for duty, want in ((5, DMIN), (255, DMAX), (0x7F00 | 77, 77)):
        command(bench, loom, checker, duty)
        bench.step(4 * PERIOD * TICK0)
        widths = [f - r for r, f in checker.pulses(AH, since=bench.cycle - 2 * PERIOD * TICK0)]
        assert widths and all(w == want * TICK0 for w in widths)
    assert checker.violations == []


def until(bench, cycle):
    """Step to bench cycle ``cycle`` (a host transfer takes clocks too)."""
    assert bench.cycle <= cycle, "the host was slower than the schedule"
    bench.step(cycle - bench.cycle)


def near(ticks, clocks, tick=TICK1):
    """A reported length against a true one: one tick of quantisation plus
    the detection jitter of the header, both ways."""
    slack = 1 + -(-JITTER1 // tick)
    return abs(ticks - clocks / tick) <= slack


def test_bldc_speed_report(backend):
    """Thread 1 against scripted sectors: nothing to report at first; then
    clockwise sectors of 4000 clocks (80 ticks), counter-clockwise ones of
    6100; an invalid code and the first change after it give no length;
    the count and the direction bit follow every change."""
    hall = M.HallSource(code=5)
    bench, checker, loom = boot(backend, hall)
    assert report(loom) == (NONE, 5)
    t = bench.cycle + 100
    cw = [M.step(5, k) for k in range(1, 5)]
    hall.schedule([(t + i * 4000, c) for i, c in enumerate(cw)])
    bench.step(3 * 4000 + 1000)
    length, status = report(loom)
    assert near(length, 4000)
    assert status == (4 << 4) | cw[-1]
    t = bench.cycle + 100
    ccw = [M.step(cw[-1], -k) for k in range(1, 4)]
    hall.schedule([(t + i * 6100, c) for i, c in enumerate(ccw)])
    bench.step(2 * 6100 + 1000)
    length, status = report(loom)
    assert near(length, 6100)
    assert status == (7 << 4) | 8 | ccw[-1]
    t = bench.cycle + 100
    hall.schedule([(t, 7), (t + 4000, ccw[-1]), (t + 7000, M.step(ccw[-1], 1))])
    until(bench, t + 500)
    assert report(loom) == (NONE, (8 << 4) | 8 | 7)
    until(bench, t + 4500)
    assert report(loom)[0] == NONE
    until(bench, t + 7500)
    length, status = report(loom)
    assert near(length, 3000) and status & 8 == 0 and status >> 4 == 10


@model_only("2^15 ticks of silence: 65,536 clocks even at the shortest tick")
def test_bldc_speed_report_times_out(backend):
    """No change for 2^15 thread 1 ticks: the length becomes 0xFFFF, and
    the first change after it measures nothing."""
    hall = M.HallSource(code=5)
    bench, checker, loom = boot(backend, hall, tick1=2)
    hall.schedule([(bench.cycle + 100, 4), (bench.cycle + 1100, 6)])
    bench.step(1500)
    assert near(report(loom)[0], 1000, tick=2)
    bench.step(2 * 32768 + 500)
    assert report(loom)[0] == NONE
    hall.schedule([(bench.cycle, 2)])
    bench.step(500)
    assert report(loom)[0] == NONE


@model_only("a motor spin-up is 300,000 clocks, about 50 s on Icarus")
@pytest.mark.parametrize("reverse", [False, True])
def test_bldc_motor_turns_the_commanded_way(backend, reverse):
    """The firmware closed around the motor model: the rotor accelerates in
    the commanded direction (AN857: FwdTable turns clockwise), thread 1's
    sector length agrees with the motor's own, the measured direction bit
    matches, and no gate rule is broken."""
    motor = M.Motor(sector=0)
    bench, checker, loom = boot(backend, motor)
    command(bench, loom, checker, 200, reverse)
    bench.step(300_000)
    assert len(motor.changes) > 30
    assert (motor.theta < 0) == reverse and (motor.omega < 0) == reverse
    length, status = report(loom)
    recent = motor.intervals()[-3:]
    assert any(near(length, clocks) for clocks in recent), (length, recent)
    assert bool(status & 8) == reverse
    # the rotor may move on while the reply crosses the host port
    assert status & 7 in [code for _, code in motor.changes[-2:]]
    assert checker.violations == []
