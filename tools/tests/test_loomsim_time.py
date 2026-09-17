"""Time: the tick generator, WAITD/DLY/SETD, timeouts, WAITE and WAITS.

Check IDs: L1-TIMER-DIV (mean period and jitter), L1-TIMER-REACH (wrap-safe
comparison in all four quadrants), L2-DEADLINE (a deadline loop keeps its
schedule whatever path the code takes).

The deadline property is the reason the architecture exists, so it is tested
twice: once on the register (TD advances by exactly the programmed amount, and
a re-issue does not advance it again) and once on the pads (the pin edges of a
loop land on the same cycles whether the loop takes its short or its long path).
"""

import pytest

from tools.loomisa import load
from tools.loomsim import Machine, alu
from tools.loomsim.harness import PadTrace, assemble, run_thread

ISA = load()

CSR_TICK_INT = 0x00
CSR_TICK_FRAC = 0x01
CSR_TD = 0x0A


def idle_machine(**kwargs):
    """A machine with no program running; the tick generators still run."""
    kwargs.setdefault("isa", ISA)
    return Machine({}, **kwargs)


def tick_edges(machine, count, thread=0, max_cycles=400000):
    """The edge numbers at which NOW incremented, for ``count`` ticks."""
    out = []
    previous = machine.threads[thread].now
    for _ in range(max_cycles):
        machine.step_cycle()
        now = machine.threads[thread].now
        if now != previous:
            out.append(machine.cycle)
            previous = now
            if len(out) == count:
                return out
    raise AssertionError("only saw %d of %d ticks" % (len(out), count))


# ------------------------------------------------------------ tick generator
def test_the_default_tick_is_one_per_clock():
    machine = idle_machine()
    edges = tick_edges(machine, 16)
    assert edges == list(range(1, 17))
    assert machine.threads[0].now == 16


@pytest.mark.parametrize("tick_int,tick_frac,ticks", [
    (1, 0, 64),
    (4, 0, 64),
    (434, 0, 32),          # 115200 baud at 50 MHz
    (3, 85, 4096),         # 3 + 85/256 = 3.332
    (1, 128, 4096),        # 1.5
    (33, 85, 256),         # 1.5 Mbit USB at 50 MHz: 33.332 clocks
])
def test_mean_tick_period_and_jitter(tick_int, tick_frac, ticks):
    """L1-TIMER-DIV: mean period is TICK_INT + TICK_FRAC/256, jitter one clock."""
    machine = idle_machine()
    machine.host_write_debug(0, "TICK_INT", tick_int)
    machine.host_write_debug(0, "TICK_FRAC", tick_frac)
    machine.step_cycle()                       # let the host writes commit
    edges = tick_edges(machine, ticks + 1)
    intervals = [b - a for a, b in zip(edges, edges[1:])]
    expected = tick_int + tick_frac / 256.0
    mean = sum(intervals) / len(intervals)
    assert abs(mean - expected) < 1.0
    assert abs(mean - expected) < 0.01         # much tighter than "within a clock"
    assert max(intervals) - min(intervals) <= 1
    assert set(intervals) <= {int(expected), int(expected) + 1}


def test_tick_int_zero_behaves_as_one():
    """``period = max(TICK_INT, 1) * 256 + TICK_FRAC``."""
    machine = idle_machine()
    machine.host_write_debug(0, "TICK_INT", 0)
    machine.step_cycle()
    edges = tick_edges(machine, 8)
    assert [b - a for a, b in zip(edges, edges[1:])] == [1] * 7


def test_the_tick_generator_runs_while_the_thread_is_halted():
    machine = idle_machine()
    machine.run_cycles(100)
    assert machine.threads[0].now == 100
    assert machine.run == 0


def test_csrw_tick_int_clears_the_accumulator():
    """SEMANTICS 4: the clear wins over the accumulate at that edge."""
    program = [("LDI", dict(rd=1, imm=5)),
               ("CSRW", dict(csr=CSR_TICK_INT, ra=1)),
               ("HALT", {})]
    machine = Machine(assemble(ISA, program), isa=ISA)
    machine.host_write_debug(0, "TICK_INT", 7)
    machine.host_write_debug(0, "TICK_FRAC", 128)      # period 1920/256 clocks
    machine.host_set_run(0b0001)
    seen_nonzero = False
    cleared = False
    for _ in range(60):
        record = machine.step_cycle()
        if machine.threads[0].acc != 0:
            seen_nonzero = True
        if record is not None and record.mnemonic == "CSRW":
            # step_cycle has just applied the commit edge (x + 2).
            assert machine.threads[0].acc == 0
            assert machine.threads[0].tick_int == 5
            cleared = True
            break
    assert seen_nonzero and cleared


@pytest.mark.parametrize("register", ["TICK_INT", "TICK_FRAC"])
def test_a_host_write_of_the_divider_also_clears_the_accumulator(register):
    """SEMANTICS 4: *any* write to TICK_INT or TICK_FRAC clears ACC."""
    machine = idle_machine()
    machine.host_write_debug(0, "TICK_INT", 7)
    machine.host_write_debug(0, "TICK_FRAC", 128)
    machine.step_cycle()
    machine.run_cycles(3)
    assert machine.threads[0].acc != 0                # it has been accumulating
    now_before = machine.threads[0].now
    machine.host_write_debug(0, register, 5)
    machine.step_cycle()
    assert machine.threads[0].acc == 0
    assert machine.threads[0].now == now_before       # no tick at that edge


def test_a_host_write_of_the_divider_restarts_the_tick_phase():
    """The first tick after the write is a whole period later, not sooner."""
    machine = idle_machine()
    machine.host_write_debug(0, "TICK_INT", 10)
    machine.step_cycle()
    machine.run_cycles(7)                             # most of the way to a tick
    machine.host_write_debug(0, "TICK_INT", 10)       # same value, fresh phase
    machine.step_cycle()
    written_at = machine.cycle
    edges = tick_edges(machine, 3)
    assert [edge - written_at for edge in edges] == [10, 20, 30]


def test_csrw_tick_int_stops_now_from_ticking_at_that_edge():
    """White-box: line up a tick with the CSRW commit and check NOW holds."""
    program = [("LDI", dict(rd=1, imm=3)),
               ("CSRW", dict(csr=CSR_TICK_INT, ra=1)),
               ("HALT", {})]
    machine = Machine(assemble(ISA, program), isa=ISA)
    machine.host_write_debug(0, "TICK_INT", 4)
    machine.host_set_run(0b0001)
    # LDI has X cycle 6 and CSRW X cycle 10, so the CSRW commits at edge 12.
    while machine.cycle < 11:
        machine.step_cycle()
    # Arrange for the accumulator to be one step short of a tick at that edge.
    machine.threads[0].acc = machine.threads[0].tick_period - 256
    now_before = machine.threads[0].now
    record = machine.step_cycle()                    # runs cycle 11, edge 12
    assert record is not None and record.mnemonic == "CSRW"
    assert machine.threads[0].now == now_before      # the tick was suppressed
    assert machine.threads[0].acc == 0
    assert machine.threads[0].tick_int == 3


# --------------------------------------------------------------- reached()
@pytest.mark.parametrize("now,deadline,expected", [
    (0, 0, True), (1, 0, True), (0, 1, False),
    (0x7FFF, 0x0000, True), (0x8000, 0x0000, False), (0x8001, 0x0000, False),
    (0x0000, 0x8000, False), (0x0000, 0x8001, True), (0x0000, 0x7FFF, False),
    (0xFFFF, 0xFFFF, True), (0x0000, 0xFFFF, True), (0xFFFE, 0xFFFF, False),
    (0x7FFF, 0x8000, False), (0x8000, 0x7FFF, True),
    (0x0010, 0xFFF0, True), (0xFFF0, 0x0010, False),
])
def test_reached_is_wrap_safe_in_every_quadrant(now, deadline, expected):
    assert alu.reached(now, deadline) is expected


def test_reached_is_monotone_over_half_the_range():
    """TIMER-1: once true it stays true until the deadline moves."""
    deadline = 0x8000
    values = [alu.reached((deadline + k) & 0xFFFF, deadline) for k in range(32768)]
    assert all(values)
    assert not alu.reached((deadline - 1) & 0xFFFF, deadline)


# ------------------------------------------------------------------- WAITD
def test_waitd_adds_on_first_issue_only():
    """TD advances by exactly imm8 however many slots the wait takes."""
    program = [("WAITD", dict(imm=5)), ("HALT", {})]
    machine = Machine(assemble(ISA, program), isa=ISA)
    machine.host_write_debug(0, "TICK_INT", 40)
    machine.host_set_run(0b0001)
    records = []
    for _ in range(400):
        record = machine.step_cycle()
        if record is not None:
            records.append(record)
        if machine.halted & 1:
            break
    waits = [r for r in records if r.mnemonic == "WAITD"]
    assert len(waits) > 3
    assert [r.done for r in waits] == [False] * (len(waits) - 1) + [True]
    assert all(r.pc == 0 and r.next_pc == 0 for r in waits[:-1])
    assert waits[-1].next_pc == 1
    assert machine.threads[0].td == 5          # 0 + 5, added once
    assert machine.threads[0].wait_active == 0


def test_waitd_deadline_is_absolute_not_relative():
    """Two WAITD 4 in a row put TD at 8, whatever happened in between."""
    program = [("WAITD", dict(imm=4)),
               ("NOP", {}), ("NOP", {}), ("NOP", {}),
               ("WAITD", dict(imm=4)),
               ("HALT", {})]

    def setup(machine):
        machine.host_write_debug(0, "TICK_INT", 20)

    machine, records = run_thread(ISA, program, setup=setup, max_cycles=4000)
    assert machine.threads[0].td == 8


def test_waitd_wraps_with_now():
    """A deadline that wraps past 0xFFFF still ends the wait at the right tick."""
    program = [("WAITD", dict(imm=0x20)), ("HALT", {})]
    machine = Machine(assemble(ISA, program), isa=ISA)
    while machine.threads[0].now != 0xFFF0:          # one tick per clock
        machine.step_cycle()
    machine.host_write_debug(0, "TD", 0xFFF0)
    machine.host_set_run(0b0001)
    start = machine.cycle
    records = []
    for _ in range(200):
        record = machine.step_cycle()
        if record is not None:
            records.append(record)
        if machine.halted & 1:
            break
    waits = [r for r in records if r.mnemonic == "WAITD"]
    assert machine.threads[0].td == 0x0010           # 0xFFF0 + 0x20, wrapped
    assert waits[-1].done is True
    assert all(not r.done for r in waits[:-1])
    # It waited about 0x20 ticks, i.e. 0x20 clocks, not 65536 of them.
    assert 0x20 <= machine.cycle - start <= 0x20 + 12


def test_a_stalled_wait_keeps_wait_active_set():
    program = [("WAITD", dict(imm=50)), ("HALT", {})]
    machine = Machine(assemble(ISA, program), isa=ISA)
    machine.host_write_debug(0, "TICK_INT", 100)
    machine.host_set_run(0b0001)
    machine.run_cycles(30)
    assert machine.threads[0].wait_active == 1
    assert machine.threads[0].pc == 0


# --------------------------------------------------------------- DLY / SETD
def test_dly_zero_is_a_single_slot():
    program = [("DLY", dict(imm=0)), ("HALT", {})]

    def setup(machine):
        machine.host_write_debug(0, "TICK_INT", 100)

    machine, records = run_thread(ISA, program, setup=setup)
    assert len(records) == 2
    assert records[0].done is True
    assert machine.threads[0].td == 0            # DLY never touches TD


def test_dly_waits_from_now_and_leaves_td_alone():
    program = [("DLY", dict(imm=3)), ("HALT", {})]
    machine = Machine(assemble(ISA, program), isa=ISA)
    machine.host_write_debug(0, "TICK_INT", 30)
    machine.host_write_debug(0, "TD", 0x1234)
    machine.host_set_run(0b0001)
    records = []
    for _ in range(400):
        record = machine.step_cycle()
        if record is not None:
            records.append(record)
        if machine.halted & 1:
            break
    delays = [r for r in records if r.mnemonic == "DLY"]
    assert len(delays) > 2
    assert delays[-1].done is True
    assert machine.threads[0].td == 0x1234
    assert machine.threads[0].dt == machine.threads[0].now


def test_setd_anchors_the_deadline_to_now():
    program = [("SETD", dict(imm=7)), ("HALT", {})]
    machine = Machine(assemble(ISA, program), isa=ISA)
    machine.host_write_debug(0, "TICK_INT", 25)
    machine.host_write_debug(0, "TD", 0xBEEF)
    machine.host_set_run(0b0001)
    now_by_cycle = {}
    records = []
    for _ in range(20):
        now_by_cycle[machine.cycle] = machine.threads[0].now
        record = machine.step_cycle()
        if record is not None:
            records.append(record)
    setd = [r for r in records if r.mnemonic == "SETD"][0]
    assert machine.threads[0].td == now_by_cycle[setd.x_cycle] + 7


# ------------------------------------------------------------- timed waits
def timed_wait_program(name, operands):
    return [(name, operands), ("HALT", {})]


def test_timed_wait_sets_t_when_it_ends_at_the_deadline():
    program = timed_wait_program("WAITP", dict(pin=8, val=1, tmo=1))
    machine = Machine(assemble(ISA, program), isa=ISA)
    machine.host_write_debug(0, "TICK_INT", 8)
    machine.host_write_debug(0, "TD", 6)             # six ticks away
    machine.host_set_run(0b0001)
    records = []
    for _ in range(400):
        record = machine.step_cycle()
        if record is not None:
            records.append(record)
        if machine.halted & 1:
            break
    waits = [r for r in records if r.mnemonic == "WAITP"]
    assert len(waits) > 2
    assert waits[-1].done is True
    assert waits[-1].t == 1
    assert all(r.t == 0 for r in waits[:-1])
    assert machine.threads[0].t == 1


def test_timed_wait_clears_t_when_the_condition_ends_it():
    program = timed_wait_program("WAITP", dict(pin=8, val=0, tmo=1))

    def setup(machine):
        machine.host_write_debug(0, "FLAGS", 0b100)      # T set beforehand

    machine, records = run_thread(ISA, program, setup=setup)
    assert records[0].done is True
    assert records[0].t == 0
    assert machine.threads[0].t == 0


def test_an_untimed_wait_never_touches_t():
    program = timed_wait_program("WAITP", dict(pin=8, val=0))

    def setup(machine):
        machine.host_write_debug(0, "FLAGS", 0b100)

    machine, records = run_thread(ISA, program, setup=setup)
    assert records[0].done is True
    assert records[0].t == 1                      # left alone
    assert machine.threads[0].t == 1


def test_a_timed_wait_that_ends_by_condition_wins_over_the_deadline():
    """The condition is tested first, so T is cleared even at the deadline."""
    program = timed_wait_program("WAITP", dict(pin=8, val=0, tmo=1))

    def setup(machine):
        machine.host_write_debug(0, "FLAGS", 0b100)
        machine.host_write_debug(0, "TD", 0)          # already reached

    machine, records = run_thread(ISA, program, setup=setup)
    assert records[0].done is True and records[0].t == 0


def test_a_timed_waits_timeout_does_not_clear_the_shared_flag():
    program = timed_wait_program("WAITS", dict(flag=1, tmo=1))

    def setup(machine):
        machine.host_write_sflags_set(0b0000_0100)    # a different flag

    machine, records = run_thread(ISA, program, setup=setup)
    assert records[0].done is True
    assert records[0].t == 1
    assert machine.sflags == 0b0000_0100              # untouched


# -------------------------------------------------------------------- WAITE
def run_with_stimulus(program, stimulus, cycles=400, thread=0, **kwargs):
    """Run a program while ``stimulus(machine)`` drives the pads each cycle."""
    kwargs.setdefault("isa", ISA)
    machine = Machine(assemble(ISA, program, thread * 0x100), **kwargs)
    machine.host_set_run(1 << thread)
    records = []
    for _ in range(cycles):
        stimulus(machine)
        record = machine.step_cycle()
        if record is not None:
            records.append(record)
        if (machine.halted >> thread) & 1:
            break
    return machine, records


@pytest.mark.parametrize("edge,start,flip_to,should_fire", [
    (0, 0, 1, True),          # rise
    (0, 1, 0, False),         # rise, falling stimulus
    (1, 1, 0, True),          # fall
    (1, 0, 1, False),         # fall, rising stimulus
    (2, 0, 1, True),          # any
    (2, 1, 0, True),          # any
])
def test_waite_edges_use_prev_pins(edge, start, flip_to, should_fire):
    # Two NOPs first so PREV_PINS holds the starting level before the WAITE:
    # PREV_PINS resets to 0, so a pin that is already high looks like a rise.
    program = [("NOP", {}), ("NOP", {}),
               ("WAITE", dict(pin=8, edge=edge)), ("HALT", {})]

    def stimulus(machine):
        machine.set_pad_inputs(ui_in=start if machine.cycle < 40 else flip_to)

    machine, records = run_with_stimulus(program, stimulus, cycles=120)
    fired = bool(machine.halted & 1)
    assert fired is should_fire
    waits = [r for r in records if r.mnemonic == "WAITE"]
    if should_fire:
        assert waits[-1].done is True
        assert all(not r.done for r in waits[:-1])
        # The edge is seen at the first slot after the synchroniser passes it.
        assert 40 <= waits[-1].x_cycle <= 48
    else:
        assert all(not r.done for r in waits)


def test_waite_rise_fires_at_once_when_the_pin_is_already_high():
    """PREV_PINS resets to 0, so the thread's first slot sees a rise.

    This is a consequence of the reset values in SEMANTICS 5, not a special
    case: firmware that cares re-arms with one slot before the WAITE.
    """
    program = [("WAITE", dict(pin=8, edge=0)), ("HALT", {})]

    def stimulus(machine):
        machine.set_pad_inputs(ui_in=0x01)

    machine, records = run_with_stimulus(program, stimulus, cycles=40)
    assert records[0].done is True
    assert records[0].x_cycle == 6


def test_waite_on_a_reserved_pin_never_fires():
    program = [("WAITE", dict(pin=13, edge=2)), ("HALT", {})]

    def stimulus(machine):
        machine.set_pad_inputs(ui_in=0xFF if machine.cycle < 20 else 0x00)

    machine, records = run_with_stimulus(program, stimulus, cycles=80)
    assert machine.halted == 0
    assert all(not r.done for r in records)


def test_prev_pins_follows_the_last_valid_slot():
    program = [("NOP", {}), ("NOP", {}), ("NOP", {}), ("HALT", {})]

    def stimulus(machine):
        machine.set_pad_inputs(ui_in=0x03)

    machine, records = run_with_stimulus(program, stimulus, cycles=60)
    assert machine.threads[0].prev_pins == 0x0300      # ui_in[1:0] at bits 9:8


# -------------------------------------------------------------------- WAITS
def test_waits_is_an_atomic_test_and_clear_between_adjacent_threads():
    """Thread 0's X cycle comes one before thread 1's, so thread 0 wins the flag."""
    image = {}
    image.update(assemble(ISA, [("WAITS", dict(flag=0)), ("HALT", {})], 0x000))
    image.update(assemble(ISA, [("WAITS", dict(flag=0)), ("HALT", {})], 0x100))
    machine = Machine(image, isa=ISA)
    machine.host_set_run(0b0011)
    machine.run_cycles(20)
    assert machine.halted == 0                    # both threads are waiting
    machine.host_write_sflags_set(0b0000_0001)
    machine.run_cycles(40)
    assert machine.halted == 0b0001               # exactly one thread proceeded
    assert machine.sflags == 0
    machine.host_write_sflags_set(0b0000_0001)
    machine.run_cycles(40)
    assert machine.halted == 0b0011               # now the other one gets it


def test_waits_forwarding_is_visible_one_cycle_before_the_commit():
    """The loser's X cycle is in the winner's W cycle: it must see the clear."""
    image = {}
    image.update(assemble(ISA, [("WAITS", dict(flag=3)), ("HALT", {})], 0x000))
    image.update(assemble(ISA, [("WAITS", dict(flag=3)), ("HALT", {})], 0x100))
    machine = Machine(image, isa=ISA)
    machine.host_set_run(0b0011)
    machine.run_cycles(12)
    machine.host_write_sflags_set(0b0000_1000)
    done = {}
    for _ in range(12):
        record = machine.step_cycle()
        if record is not None and record.mnemonic == "WAITS" and record.done:
            done.setdefault(record.thread, record.x_cycle)
    assert list(done) == [0]
    # Thread 1's X cycle is the very next one, and it stalled.
    assert machine.threads[1].wait_active == 1


def test_sig_and_clr_move_the_shared_flags():
    program = [("SIG", dict(flag=5)), ("NOP", {}), ("CLR", dict(flag=5)),
               ("HALT", {})]
    machine = Machine(assemble(ISA, program), isa=ISA)
    machine.host_set_run(0b0001)
    seen = []
    for _ in range(40):
        machine.step_cycle()
        seen.append(machine.sflags)
        if machine.halted & 1:
            break
    assert 0b0010_0000 in seen
    assert machine.sflags == 0


def test_the_thread_wins_a_same_edge_conflict_on_sflags():
    """SEMANTICS 6.5: a host clear loses to a thread SIG on the bits it touches."""
    program = [("SIG", dict(flag=0)), ("HALT", {})]
    machine = Machine(assemble(ISA, program), isa=ISA)
    machine.host_set_run(0b0001)
    # SIG has X cycle 6 and is in W during cycle 7, so its commit and a host
    # write issued during cycle 7 land at the same edge.
    while machine.cycle < 7:
        machine.step_cycle()
    machine.host_write_sflags_clr(0xFF)
    record = machine.step_cycle()
    assert record is not None and record.mnemonic == "SIG"
    assert machine.sflags == 0b0000_0001          # the thread won that bit


def test_a_host_clear_one_cycle_later_does_take_effect():
    program = [("SIG", dict(flag=0)), ("HALT", {})]
    machine = Machine(assemble(ISA, program), isa=ISA)
    machine.host_set_run(0b0001)
    while machine.cycle < 8:
        machine.step_cycle()
    assert machine.sflags == 0b0000_0001
    machine.host_write_sflags_clr(0xFF)
    machine.step_cycle()
    assert machine.sflags == 0


# ---------------------------------------------------- the deadline property
DEADLINE_K = 64


def deadline_loop(long_path):
    """A loop that toggles OUT0 every K ticks, with two paths of different length."""
    program = [
        ("LDI", dict(rd=1, imm=3)),                  # 0: iterations
        ("LDI", dict(rd=3, imm=1 if long_path else 0)),   # 1: path selector
        ("SETD", dict(imm=DEADLINE_K)),              # 2: anchor TD
        ("WAITD", dict(imm=DEADLINE_K)),             # 3
        ("SETP", dict(pin=16, val=1)),               # 4
        ("WAITD", dict(imm=DEADLINE_K)),             # 5
        ("SETP", dict(pin=16, val=0)),               # 6
        ("TEST", dict(rd=3, ra=3)),                  # 7: Z = (r3 == 0)
        ("BZ", dict(rel=3)),                         # 8: short path skips 9..11
        ("NOP", {}),                                 # 9
        ("NOP", {}),                                 # 10
        ("NOP", {}),                                 # 11
        ("DJNZ", dict(rd=1, rel=-10)),               # 12: back to 3
        ("HALT", {}),                                # 13
    ]
    return program


@pytest.mark.parametrize("long_path", [False, True])
def test_a_deadline_loop_keeps_its_schedule(long_path):
    trace = PadTrace()
    machine, records = run_thread(ISA, deadline_loop(long_path), on_cycle=trace,
                                  max_cycles=4000)
    edges = trace.edges(trace.uo_bit(0))
    assert len(edges) == 6                       # three high, three low
    spacing = [b[0] - a[0] for a, b in zip(edges, edges[1:])]
    assert spacing == [DEADLINE_K] * 5


# ------------------------------------------------- the slot grid, SEMANTICS 2
GRID_EDGES = 101


def measure_grid(tick_int, ticks_per_edge, edge_count=GRID_EDGES):
    """Toggle a pin every ``ticks_per_edge`` ticks and time every edge.

    Returns ``(lateness, edges, period_clocks)`` where ``lateness[n]`` is how
    many clocks after its deadline the n-th ``WAITD`` completed, and ``edges``
    are the cycles at which the pad actually changed.
    """
    program = [
        ("SETD", dict(imm=2)),                    # 0: anchor the schedule
        ("WAITD", dict(imm=ticks_per_edge)),      # 1
        ("SETP", dict(pin=16, val=1)),            # 2
        ("WAITD", dict(imm=ticks_per_edge)),      # 3
        ("SETP", dict(pin=16, val=0)),            # 4
        ("JMP", dict(abs=1)),                     # 5
    ]
    trace = PadTrace()
    machine = Machine(assemble(ISA, program), isa=ISA, on_cycle=trace)
    machine.host_write_debug(0, "TICK_INT", tick_int)
    machine.host_set_run(0b0001)

    deadlines = []                 # (TD after the wait, X cycle of that wait)
    tick_cycle = {}                # NOW value -> the first cycle it held it
    previous_now = machine.threads[0].now
    limit = (edge_count + 4) * ticks_per_edge * tick_int + 400
    while len(deadlines) < edge_count and machine.cycle < limit:
        record = machine.step_cycle()
        now = machine.threads[0].now
        if now != previous_now:
            tick_cycle.setdefault(now, machine.cycle)
            previous_now = now
        if record is not None and record.mnemonic == "WAITD" and record.done:
            deadlines.append((machine.threads[0].td, record.x_cycle))
    assert len(deadlines) == edge_count, "not enough edges within the limit"
    machine.run_cycles(8)          # flush the last pad write into the trace

    lateness = [x_cycle - tick_cycle[td] for td, x_cycle in deadlines]
    edges = [cycle for cycle, _ in trace.edges(trace.uo_bit(0))][:edge_count]
    assert len(edges) == edge_count
    return lateness, edges, ticks_per_edge * tick_int


@pytest.mark.parametrize("tick_int,ticks_per_edge", [
    (64, 1),         # 64 clocks between edges: a multiple of 4
    (8, 2),          # 16 clocks
    (33, 1),         # 33 clocks: not a multiple of 4
    (434, 1),        # 115200 baud, the UART bit time
])
def test_every_edge_is_zero_to_three_clocks_after_its_deadline(tick_int, ticks_per_edge):
    """SEMANTICS 2: a wait completes in the first X cycle at or after its deadline."""
    lateness, _, _ = measure_grid(tick_int, ticks_per_edge)
    assert min(lateness) >= 0
    assert max(lateness) <= 3
    # Zero drift: the lateness of the hundredth edge is no worse than the first.
    assert abs(lateness[-1] - lateness[0]) <= 3


@pytest.mark.parametrize("tick_int,ticks_per_edge", [(64, 1), (8, 2), (100, 1)])
def test_edges_are_exactly_periodic_when_the_period_is_a_multiple_of_four(
        tick_int, ticks_per_edge):
    _, edges, period = measure_grid(tick_int, ticks_per_edge)
    assert period % 4 == 0
    spacing = {b - a for a, b in zip(edges, edges[1:])}
    assert spacing == {period}


@pytest.mark.parametrize("tick_int,ticks_per_edge", [(33, 1), (434, 1), (7, 3)])
def test_edges_dither_but_never_drift_when_the_period_is_not_a_multiple_of_four(
        tick_int, ticks_per_edge):
    _, edges, period = measure_grid(tick_int, ticks_per_edge)
    assert period % 4 != 0
    spacing = [b - a for a, b in zip(edges, edges[1:])]
    assert max(spacing) - min(spacing) <= 4
    assert all(abs(gap - period) <= 3 for gap in spacing)
    # Zero accumulated drift over a hundred edges: the span is the ideal span
    # to within the 4-clock slot quantum, and the mean is the exact period.
    span = edges[-1] - edges[0]
    ideal = (len(edges) - 1) * period
    assert abs(span - ideal) <= 3
    assert abs(span / (len(edges) - 1) - period) < 0.05


def test_the_uart_bit_time_dithers_by_two_clocks_either_way():
    """The worked example in SEMANTICS 2: 434 clocks alternates 432 and 436."""
    _, edges, period = measure_grid(434, 1)
    spacing = [b - a for a, b in zip(edges, edges[1:])]
    assert set(spacing) == {432, 436}
    assert sum(spacing) == (len(edges) - 1) * period or abs(
        sum(spacing) - (len(edges) - 1) * period) <= 3
    # Ten bit times is a multiple of 4, so frame boundaries are exact.
    tenth = [edges[i + 10] - edges[i] for i in range(len(edges) - 10)]
    assert set(tenth) == {10 * period}


def test_the_pin_schedule_is_identical_on_both_paths():
    """The key property: the branch path does not move a single pad edge."""
    traces = []
    work = []
    for long_path in (False, True):
        trace = PadTrace()
        _, records = run_thread(ISA, deadline_loop(long_path), on_cycle=trace,
                                max_cycles=4000)
        traces.append(trace.edges(trace.uo_bit(0)))
        work.append(len([r for r in records if r.done]))
    assert traces[0] == traces[1]
    # ...even though the long path really did execute nine more instructions.
    assert work[1] - work[0] == 9
