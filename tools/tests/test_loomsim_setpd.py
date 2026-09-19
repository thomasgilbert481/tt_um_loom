"""Deadline-latched pin write, SEMANTICS 6.10, on the golden model (feature ``"SETPD"``).

``SETP pin, v, D`` stages the write in the thread's latch at its commit edge,
replacing anything staged.  The staged write lands, and ``LAT_VALID`` clears,
at the first *later* edge at which either (rule 1) ``NOW`` ticks to exactly
``TD`` with ``TD`` not written at that edge, or (rule 2) ``TD`` is written at
that edge (``WAITD`` first issue, ``SETD``, ``CSRW TD``, the host) and
``reached(NOW', TD')`` holds after it.  It follows 6.3; an ordinary pin write
to the same pin at the same edge wins; ``CTRL.RESET`` clears ``LAT_VALID``;
debug 0x25 reads ``{LAT_VALID, LAT_VAL, LAT_PIN[4:0]}`` in bits 6:0
(HOST_PROTOCOL space 4).  Without the feature the ``D`` bit is ignored.

Expected edges are computed from SEMANTICS 2 and 4 in this file.  Unless a
test writes the divider, ``NOW`` ticks at every edge, so ``NOW`` in cycle
``c`` is ``c``; with ``RUN`` written in cycle 0 thread 0's X cycles are 6, 10,
14, ..., a slot commits at ``x + 2``, and thread 1's X cycles are 3, 7, 11, ...
Tests that pin a reading the text leaves open say so and name the item in
``docs/spec-questions/loomsim.md`` (M2 update).
"""

import pytest

from tools.loomisa import load
from tools.loomsim import Machine
from tools.loomsim.harness import PadTrace, assemble

ISA = load()
CSR = ISA.csr_by_name

OUT0 = 16                    # pin index of pad uo_out[0]
LAT = 0x25                   # debug register of the latch
EDGES = 101                  # pad edges checked by the headline tests (100 spacings)


def lat_word(valid, val, pin):
    """Debug 0x25: ``{LAT_VALID, LAT_VAL, LAT_PIN[4:0]}`` in bits 6:0."""
    return (valid << 6) | (val << 5) | pin


def setp(pin, val, lat=0):
    return ("SETP", dict(pin=pin, val=val, lat=lat))


def machine_for(program, features=("SETPD",), extra=None, **kwargs):
    kwargs.setdefault("isa", ISA)
    image = assemble(ISA, program)
    image.update(extra or {})
    return Machine(image, features=features, **kwargs)


class Watch:
    """What the pads and the host see in each cycle (list index = cycle)."""

    def __init__(self, thread=0):
        self.thread = thread
        self.uo, self.now, self.td, self.lat = [], [], [], []
        self.records = []

    def step(self, machine, host=None):
        """Sample the current cycle, let ``host(machine)`` act in it, step."""
        assert machine.cycle == len(self.uo)
        th = machine.threads[self.thread]
        self.uo.append(machine.uo_out)
        self.now.append(th.now)
        self.td.append(th.td)
        self.lat.append(machine.host_read_debug(self.thread, LAT))
        if host is not None:
            host(machine)
        record = machine.step_cycle()
        if record is not None:
            self.records.append(record)

    def run(self, machine, cycles, host=None):
        for _ in range(cycles):
            self.step(machine, host)
        return self

    def bit(self, index):
        return [(value >> index) & 1 for value in self.uo]

    def edges(self, index=0):
        """Cycles in which ``uo_out[index]`` differs from the cycle before."""
        values = self.bit(index)
        return [c for c in range(1, len(values)) if values[c] != values[c - 1]]

    def valid(self):
        return [(word >> 6) & 1 for word in self.lat]


def started(program, run=0b0001, **kwargs):
    machine = machine_for(program, **kwargs)
    machine.host_set_run(run)
    return machine


# ------------------------------------------------ the headline property
def square_wave(latched):
    """OUT0 toggles once per tick: ``SETP OUT0, v, D; WAITD 1`` (6.10) or the
    M1 idiom ``WAITD 1; SETP OUT0, v``."""
    if latched:
        body = [setp(OUT0, 1, 1), ("WAITD", dict(imm=1)),
                setp(OUT0, 0, 1), ("WAITD", dict(imm=1))]
    else:
        body = [("WAITD", dict(imm=1)), setp(OUT0, 1),
                ("WAITD", dict(imm=1)), setp(OUT0, 0)]
    return [("SETD", dict(imm=2))] + body + [("JMP", dict(abs=1))]


def tick_edges(tick_int, tick_frac, count):
    """SEMANTICS 4 for a divider the host writes in cycle 0: the write clears
    ``ACC`` at edge 1 (no tick there), then the j-th tick is at the first edge
    ``1 + m`` with ``256 m >= j * period``."""
    period = max(tick_int, 1) * 256 + tick_frac
    return [1 + -(-j * period // 256) for j in range(1, count + 1)]


def run_square_wave(latched, tick_int, tick_frac):
    ticks = tick_edges(tick_int, tick_frac, EDGES + 3)
    machine = machine_for(square_wave(latched))
    machine.host_write_debug(0, "TICK_INT", tick_int)
    machine.host_write_debug(0, "TICK_FRAC", tick_frac)
    machine.host_set_run(0b0001)
    pad, now, td = [], [], []
    th = machine.threads[0]
    for _ in range(ticks[-1] + 12):
        pad.append(machine.uo_out & 1)
        now.append(th.now)
        td.append(th.td)
        machine.step_cycle()
    edges = [c for c in range(1, len(pad)) if pad[c] != pad[c - 1]]
    now_edges = [c for c in range(1, len(now)) if now[c] != now[c - 1]]
    assert now_edges == ticks                        # the time base, from section 4
    assert machine.badop == 0
    return ticks, edges, pad, now, td


@pytest.mark.parametrize("tick_int,tick_frac,spacings", [
    (433, 128, {433, 434}),          # 433.5 clocks per tick
    (434, 0, {434}),                 # the 115200-baud bit time of section 2
])
def test_latched_edges_land_exactly_on_tick_edges(tick_int, tick_frac, spacings):
    ticks, edges, pad, now, td = run_square_wave(True, tick_int, tick_frac)
    # SETD 2 and the first WAITD 1 make the first deadline tick 3.
    assert edges[:EDGES] == ticks[2:2 + EDGES]
    for e in edges[:EDGES]:
        assert now[e] == (now[e - 1] + 1) & 0xFFFF   # the cycle NOW changes ...
        assert now[e] == td[e]                       # ... to the thread's deadline
    assert [pad[e] for e in edges[:EDGES]] == [(k + 1) % 2 for k in range(EDGES)]
    assert set(b - a for a, b in zip(edges[:EDGES], edges[1:EDGES])) == spacings


@pytest.mark.parametrize("tick_int,tick_frac", [(433, 128), (434, 0)])
def test_the_m1_idiom_dithers_on_the_slot_grid_instead(tick_int, tick_frac):
    """Section 2's contrast: the WAITD ends in the first X cycle (2 mod 4 for
    thread 0) at or after the tick and the ordinary SETP after it writes at
    that X + 4 + 2, so edges move in steps of 4 clocks: 432 and 436."""
    ticks, edges, _, _, _ = run_square_wave(False, tick_int, tick_frac)
    assert edges[:EDGES] == [e + (2 - e) % 4 + 6 for e in ticks[2:2 + EDGES]]
    assert set(b - a for a, b in zip(edges[:EDGES], edges[1:EDGES])) == {432, 436}


# ----------------------------------------------- rule 2 and who writes TD
@pytest.mark.parametrize("tick_int,nops", [(1, 0), (3, 4)])
def test_a_thread_already_late_writes_at_its_waitd_commit_edge(tick_int, nops):
    """Rule 2: the WAITD's first issue writes a TD that NOW has already passed,
    so the staged write lands at the WAITD's commit edge x + 2, the edge an
    ordinary SETP in that slot would have written at."""
    program = ([("SETD", dict(imm=1)), setp(OUT0, 1, 1)] + [("NOP", {})] * nops
               + [("WAITD", dict(imm=1)), ("HALT", {})])
    machine = machine_for(program)
    machine.host_write_debug(0, "TICK_INT", tick_int)
    machine.host_set_run(0b0001)
    watch = Watch().run(machine, 60)
    waitd = [r for r in watch.records if r.mnemonic == "WAITD"]
    assert len(waitd) == 1 and waitd[0].done         # late: done on first issue
    assert watch.edges(0) == [waitd[0].x_cycle + 2]
    assert watch.valid()[waitd[0].x_cycle + 2:] == [0] * (60 - waitd[0].x_cycle - 2)


TD_WRITERS = {
    # Staged at edge 8 (X 6); TD is written by the third slot (X 14, edge 16)
    # or, for "host", by a debug write in cycle 20 after the HALT.
    "SETD": lambda v: [setp(OUT0, 1, 1), ("NOP", {}), ("SETD", dict(imm=v)), ("HALT", {})],
    "CSRW": lambda v: [setp(OUT0, 1, 1), ("LDI", dict(rd=1, imm=v)),
                       ("CSRW", dict(csr=CSR["TD"], ra=1)), ("HALT", {})],
    "WAITD": lambda v: [setp(OUT0, 1, 1), ("NOP", {}), ("WAITD", dict(imm=v)), ("HALT", {})],
    "host": lambda v: [setp(OUT0, 1, 1), ("HALT", {})],
}


@pytest.mark.parametrize("writer,value,fire", [
    ("SETD", 0, 16),     # TD = NOW(14) = 14, reached at edge 16: rule 2
    ("SETD", 9, 23),     # TD = 23 is ahead: rule 1 when NOW ticks to 23
    ("CSRW", 3, 16),
    ("CSRW", 30, 30),
    ("WAITD", 1, 16),    # TD = 0 + 1, already passed: done on first issue
    ("WAITD", 30, 30),   # lands at the tick, while the WAITD is still waiting
    ("host", 5, 21),     # debug write in cycle 20 commits at edge 21
    ("host", 40, 40),
])
def test_the_write_lands_on_the_next_deadline_whoever_sets_it(writer, value, fire):
    def host(machine):
        if writer == "host" and machine.cycle == 20:
            machine.host_write_debug(0, "TD", value)

    watch = Watch().run(started(TD_WRITERS[writer](value)), 60, host)
    assert watch.edges(0) == [fire]
    assert watch.valid()[8:] == [1] * (fire - 8) + [0] * (60 - fire)


@pytest.mark.parametrize("fourth,fire", [
    (("SETD", dict(imm=5)), 23),     # TD rewritten to 23 at edge 20
    (("NOP", {}), 20),               # control: TD stays 20
])
def test_rule_1_ignores_a_td_that_is_rewritten_at_that_edge(fourth, fire):
    """At edge 20 NOW ticks to the old TD (20) while a SETD commits TD = 23:
    rule 1 needs TD not written at the edge and rule 2 needs the new TD
    reached, so the write waits for NOW to tick to 23."""
    program = [("SETD", dict(imm=14)), setp(OUT0, 1, 1), ("NOP", {}), fourth, ("HALT", {})]
    watch = Watch().run(started(program), 40)
    assert watch.td[19] == 20 and watch.now[20] == 20
    assert watch.edges(0) == [fire]


# ------------------------------------------------------- the arming edge
def test_the_arming_edge_never_applies_the_new_write():
    """NOW ticks to exactly TD at the very edge the SETP ... D commits.  The
    write may only land at a *later* edge, and NOW is past TD from then on, so
    it waits, still staged, for the next TD write (a SETD 0 at edge 24)."""
    program = [("SETD", dict(imm=6)),                # X 6: TD = 12
               setp(OUT0, 1, 1),                     # X 10: staged at edge 12
               ("NOP", {}), ("NOP", {}),
               ("SETD", dict(imm=0)),                # X 22: TD = 22 at edge 24
               ("HALT", {})]
    watch = Watch().run(started(program), 40)
    assert (watch.now[11], watch.now[12], watch.td[12]) == (11, 12, 12)
    assert watch.edges(0) == [24]
    assert watch.valid()[8:30] == [0] * 4 + [1] * 12 + [0] * 6


def test_a_latch_loaded_by_the_host_cannot_fire_at_its_loading_edge():
    """The model's reading (M2 update item 9): a host write of debug 0x25 loads
    the latch as a SETP ... D commit would, so a TD write by the host at the
    same edge does not apply it; the next one does."""
    def host(machine):
        if machine.cycle in (0, 5):
            machine.host_write_debug(0, "TD", 0)     # reached at once: rule 2
        if machine.cycle == 0:
            machine.host_write_debug(0, LAT, lat_word(1, 1, OUT0))

    watch = Watch().run(machine_for([("HALT", {})]), 12, host)
    assert watch.edges(0) == [6]
    assert watch.lat[1:7] == [lat_word(1, 1, OUT0)] * 5 + [lat_word(0, 1, OUT0)]


def test_an_older_write_still_lands_at_the_edge_a_newer_one_is_staged():
    """The model's reading (M2 update item 8): at edge 16 NOW ticks to TD while
    a second SETP ... D commits.  The write staged before the edge lands; the
    new one replaces it in the latch and, staged at that edge, waits."""
    program = [("SETD", dict(imm=10)),               # TD = 16
               setp(OUT0, 1, 1),                     # staged at edge 12
               setp(OUT0 + 1, 1, 1),                 # staged at edge 16
               ("HALT", {})]
    watch = Watch().run(started(program), 30)
    assert watch.edges(0) == [16] and watch.edges(1) == []
    assert watch.lat[15:17] == [lat_word(1, 1, OUT0), lat_word(1, 1, OUT0 + 1)]
    assert watch.lat[-1] == lat_word(1, 1, OUT0 + 1)


# --------------------------------------------- ordinary writes at that edge
@pytest.mark.parametrize("preset,pin,val,uo_out", [
    (1, OUT0, 0, 0b00),          # same pin: the ordinary write wins
    (0, OUT0 + 1, 1, 0b11),      # another pin: both land
])
def test_an_ordinary_setp_at_the_same_edge_wins_on_its_pin(preset, pin, val, uo_out):
    """Staged OUT0 = 1 lands at edge 16 (NOW ticks to TD) while the same
    thread's next SETP commits.  OUT0 starts at ``preset``, so the same-pin
    case tells "ordinary wins" (0) from "staged wins" and from "neither" (1)."""
    program = [("SETD", dict(imm=10)), setp(OUT0, 1, 1), setp(pin, val), ("HALT", {})]
    machine = started(program)
    machine.host_write_pin_out(preset << 8)
    watch = Watch().run(machine, 30)
    assert watch.uo[15] == preset
    assert watch.uo[16:] == [uo_out] * 14
    assert watch.valid()[15:17] == [1, 0]            # the staged write was used up


def test_another_threads_ordinary_setp_at_that_edge_wins_too():
    """Thread 0 stages OUT0 = 1 for edge 17 and halts; thread 1's SETP OUT0, 0
    (X 15) commits at edge 17.  The latch lands while its thread is halted."""
    thread1 = assemble(ISA, [("NOP", {})] * 3 + [setp(OUT0, 0), ("HALT", {})], 0x100)
    program = [("SETD", dict(imm=11)), setp(OUT0, 1, 1), ("HALT", {})]   # TD = 17
    machine = started(program, run=0b0011, extra=thread1)
    machine.host_write_pin_out(1 << 8)
    watch = Watch().run(machine, 30)
    assert [(r.thread, r.x_cycle) for r in watch.records if r.mnemonic == "SETP"] == \
        [(0, 10), (1, 15)]
    assert watch.bit(0)[16:] == [1] + [0] * 13
    assert watch.valid()[16:18] == [1, 0]


# ------------------------------------------------------------ CTRL.RESET
@pytest.mark.parametrize("reset", [True, False])
def test_ctrl_reset_discards_the_staged_write(reset):
    """Staged at edge 12 for TD = 106, thread halted at edge 16.  With a reset
    in cycle 30 the latch is gone: neither the reset's own TD write (TD <=
    NOW, reached) nor a host TD write in cycle 40 applies it; without the
    reset that host write does."""
    program = [("SETD", dict(imm=100)), setp(OUT0, 1, 1), ("HALT", {})]

    def host(machine):
        if reset and machine.cycle == 30:
            machine.host_write_ctrl("RESET", 0b0001)
        if machine.cycle == 40:
            machine.host_write_debug(0, "TD", 40)

    watch = Watch().run(started(program), 120, host)
    assert watch.lat[30] == lat_word(1, 1, OUT0)
    if reset:
        assert watch.edges(0) == []
        assert watch.td[31] == 30                    # the reset wrote TD <= NOW
        # Only LAT_VALID is cleared; the pin and level stay readable.
        assert watch.lat[31:] == [lat_word(0, 1, OUT0)] * 89
    else:
        assert watch.edges(0) == [41]


# ------------------------------------------------------------- debug 0x25
def test_debug_0x25_reads_back_the_latch():
    program = [setp(OUT0 + 3, 1, 1),                 # staged at edge 8
               setp(5, 0, 1),                        # replaces it at edge 12
               ("HALT", {})]

    def host(machine):
        if machine.cycle == 20:
            machine.host_write_debug(0, "TD", 0)     # lands at edge 21

    machine = started(program)
    machine.host_write_pin_out(1 << 5)
    watch = Watch().run(machine, 24, host)
    assert watch.lat[:8] == [0] * 8
    assert watch.lat[8:12] == [lat_word(1, 1, OUT0 + 3)] * 4 == [0x73] * 4
    assert watch.lat[12:21] == [lat_word(1, 0, 5)] * 9 == [0x45] * 9
    assert watch.lat[21:] == [lat_word(0, 0, 5)] * 3
    assert machine.host_read_debug(0, "LAT") == 0x05
    assert (machine.pin_out, machine.uo_out) == (0, 0)   # BIDIR5 written 0, OUT3 never


# ------------------------------------------------------ 6.3 on a staged write
@pytest.mark.parametrize("pin,val,od,pin_out,pin_oe", [
    (2, 1, 1, 0x0000, 0x00),     # open drain, 1: PIN_OUT <= 0, released
    (2, 0, 1, 0x0000, 0x04),     # open drain, 0: PIN_OUT <= 0, driven low
    (2, 0, 0, 0x0000, 0x04),     # push-pull: PIN_OUT only, OE kept
    (3, 1, 0, 0x000C, 0x04),
    (OUT0 + 5, 1, 0, 0x2004, 0x04),
    (8, 0, 0, 0x0004, 0x04),     # read-only and reserved indices: ignored
    (31, 0, 0, 0x0004, 0x04),
])
def test_a_staged_write_follows_the_pin_write_rules(pin, val, od, pin_out, pin_oe):
    """6.3 with OD_MASK = ``od`` on BIDIR2, from PIN_OUT = PIN_OE = bit 2."""
    program = [("SETD", dict(imm=10)), setp(pin, val, 1), ("HALT", {})]    # TD = 16
    machine = started(program)
    machine.host_write_od_mask(od << 2)
    machine.host_write_pin_out(0x0004)
    machine.host_write_pin_oe(0x04)
    machine.run_cycles(15)
    assert (machine.pin_out, machine.pin_oe) == (0x0004, 0x04)   # cycle 15: not yet
    machine.step_cycle()                                         # edge 16
    assert (machine.pin_out, machine.pin_oe) == (pin_out, pin_oe)
    assert machine.threads[0].lat_valid == 0


# ---------------------------------------------------- without the feature
def square_wave_d(lat):
    """``SETP ... D; WAITD 1`` twice per period, with the D bit as given."""
    return [("SETD", dict(imm=2)), setp(OUT0, 1, lat), ("WAITD", dict(imm=1)),
            setp(OUT0, 0, lat), ("WAITD", dict(imm=1)), ("JMP", dict(abs=1))]


@pytest.mark.parametrize("features", [(), ("FIFO", "BE")])
def test_without_the_feature_setp_d_is_an_ordinary_setp(features):
    def run(lat):
        trace = PadTrace()
        machine = machine_for(square_wave_d(lat), features=features, on_cycle=trace)
        machine.host_write_debug(0, "TICK_INT", 30)
        machine.host_set_run(0b0001)
        records = machine.run_cycles(800)
        return machine, trace, records

    machine, trace, records = run(1)
    _, plain_trace, plain_records = run(0)
    assert trace.samples == plain_trace.samples
    strip = [(r.x_cycle, r.pc, r.done, r.we, r.val, r.flags, r.next_pc) for r in records]
    assert strip == [(r.x_cycle, r.pc, r.done, r.we, r.val, r.flags, r.next_pc)
                     for r in plain_records]
    setps = [r.x_cycle for r in records if r.mnemonic == "SETP"]
    assert len(setps) > 10
    # Every SETP toggles OUT0 at its own edge x + 2 (within the traced cycles).
    assert [c for c, _ in trace.changes()] == [x + 2 for x in setps if x + 2 < 800]
    assert machine.badop == 0 and machine.caps & (1 << 7) == 0
    assert machine.host_read_debug(0, LAT) == 0 and machine.threads[0].lat_valid == 0
    assert machine_for([], features=features + ("SETPD",)).caps & (1 << 7)
