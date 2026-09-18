"""The host-visible registers of HOST_PROTOCOL 0.2 on the golden model.

Built in every configuration, ``features=()`` included (the M2 model update,
item 1): ``ID`` and ``VERSION``, the CTRL map by address, the host interrupt of
SEMANTICS 6.8 (``IRQ_EN``, ``IRQ_STAT``, ``IRQ_STAT2``, ``IRQ_EN2``, ``SWIRQ``
and the registered ``HOST_IRQ`` output), debug writes gated on "halted" as
SEMANTICS 7 defines it for every debug register, ``STEPS`` writes, the debug
CSR window 0x10..0x1F by number and debug 0x21..0x26.

The last section pins what the M1 co-simulation depends on: with
``features=()`` the pads, ``dump_thread`` and ``PadState`` look exactly as
they did before these registers existed.
"""

import dataclasses

import pytest

from tools.loomisa import load
from tools.loomsim import (DEBUG_REGS, CycleTrace, LoomsimError, Machine,
                           PadState)
from tools.loomsim.__main__ import main
from tools.loomsim.harness import assemble, run_thread

ISA = load()

CSR = ISA.csr_by_name
DEBUG_CSR = 0x10                     # debug 0x10 + n is the thread's CSR n


def machine_with(program, thread=0, **kwargs):
    kwargs.setdefault("isa", ISA)
    return Machine(assemble(ISA, program, thread * 0x100), **kwargs)


def spinner(**kwargs):
    """Thread 0 runs ``JMP 0`` forever: it touches no register but PC and STEPS."""
    return machine_with([("JMP", dict(abs=0))], **kwargs)


def irq_by_cycle(machine, cycles, probe=None):
    """``{cycle: (host_irq, probe(machine))}`` for the next ``cycles`` cycles."""
    out = {}
    for _ in range(cycles):
        out[machine.cycle] = (machine.host_irq, probe(machine) if probe else None)
        machine.step_cycle()
    return out


def first_cycle(series, predicate):
    for cycle in sorted(series):
        if predicate(series[cycle]):
            return cycle
    return None


# ------------------------------------------------------------ ID and VERSION
def test_id_reads_lm():
    machine = Machine({}, isa=ISA)
    assert machine.host_read_ctrl("ID") == 0x4C4D
    assert machine.host_read_ctrl(0x0000) == 0x4C4D


def test_version_defaults_to_the_host_protocol_version_and_can_be_set():
    assert Machine({}, isa=ISA).host_read_ctrl("VERSION") == 0x0002
    assert Machine({}, isa=ISA, version=0x0001).host_read_ctrl(0x0001) == 0x0001


def test_read_only_ctrl_registers_ignore_writes_by_address_and_refuse_by_name():
    machine = Machine({}, isa=ISA)
    for addr in (0x00, 0x01, 0x03, 0x11, 0x12, 0x18, 0x19):
        before = machine.host_read_ctrl(addr)
        machine.host_write_ctrl(addr, 0xFFFF)
        machine.step_cycle()
        assert machine.host_read_ctrl(addr) == before, hex(addr)
    for name in ("ID", "VERSION", "HALTED", "IRQ_STAT", "IRQ_STAT2", "PIN_IN", "CAPS"):
        with pytest.raises(LoomsimError):
            machine.host_write_ctrl(name, 1)
    with pytest.raises(LoomsimError):
        machine.host_read_ctrl("NOSUCH")


def test_the_ctrl_map_by_address():
    """Every register of HOST_PROTOCOL space 0 at its address."""
    machine = Machine({}, isa=ISA, imem_words=256)
    machine.host_write_ctrl(0x0B, 0x0155)            # RESET_PC3
    machine.host_write_ctrl(0x10, 0xA5A5)            # IRQ_EN
    machine.host_write_ctrl(0x13, 0x81)              # SFLAGS (set)
    machine.host_write_ctrl(0x15, 0x0F)              # OD_MASK
    machine.host_write_ctrl(0x16, 0x2A05)            # PIN_OUT
    machine.host_write_ctrl(0x17, 0x33)              # PIN_OE
    machine.host_write_ctrl(0x1C, 0xFFFF)            # IRQ_EN2 keeps 4 bits
    machine.step_cycle()
    assert machine.host_read_ctrl(0x08) == 0x000       # RESET_PC0 default
    assert machine.host_read_ctrl(0x09) == 0x040       # t * IMEM_WORDS / 4
    assert machine.host_read_ctrl(0x0B) == 0x155
    assert machine.host_read_ctrl(0x10) == 0xA5A5
    assert machine.host_read_ctrl(0x13) == 0x81
    assert machine.host_read_ctrl(0x15) == 0x0F
    assert machine.host_read_ctrl(0x16) == 0x2A05
    assert machine.host_read_ctrl(0x17) == 0x33
    assert machine.host_read_ctrl(0x19) == machine.caps == 0x8000
    assert machine.host_read_ctrl(0x1C) == 0x000F
    machine.host_write_ctrl(0x14, 0x01)              # SFLAGS_CLR
    machine.step_cycle()
    assert machine.host_read_ctrl(0x13) == 0x80
    # Write-only registers and unused addresses read 0.
    for addr in (0x04, 0x14, 0x05, 0x0C, 0x1D, 0xFF):
        assert machine.host_read_ctrl(addr) == 0, hex(addr)


def test_ctrl_run_reset_and_halted_through_the_map():
    machine = machine_with([("LDI", dict(rd=1, imm=3)), ("HALT", {})])
    machine.host_write_ctrl("RUN", 0b0001)
    assert machine.host_read_ctrl("RUN") == 0           # commits at the edge
    machine.step_cycle()
    assert machine.host_read_ctrl("RUN") == 0b0001
    machine.run_until(lambda m: m.halted & 1, max_cycles=40)
    assert machine.host_read_ctrl("HALTED") == 0b0001
    assert machine.host_read_ctrl("IRQ_STAT2") == 0b0001
    assert machine.host_read_ctrl("RUN") == 0
    machine.host_write_ctrl("RESET", 0b0001)
    machine.step_cycle()
    assert machine.threads[0].pc == 0
    assert machine.threads[0].regs[1] == 3            # registers untouched


def test_badop_is_write_one_to_clear_through_the_ctrl_map():
    machine = machine_with([("JMP", dict(abs=0))])
    machine.host_set_run(0b0001)
    machine.run_cycles(8)
    machine.host_write_imem(0, 0)                     # while running: bit 15
    machine.step_cycle()
    assert machine.host_read_ctrl("BADOP") == 1 << 15
    machine.host_write_ctrl(0x1A, 0x0001)             # clearing another bit
    machine.step_cycle()
    assert machine.host_read_ctrl("BADOP") == 1 << 15
    machine.host_write_ctrl(0x1A, 1 << 15)
    machine.step_cycle()
    assert machine.host_read_ctrl("BADOP") == 0


# ------------------------------------------------------------ host interrupt
def test_host_irq_is_low_out_of_reset_in_every_build():
    for features in ((), ("FIFO",), ("FIFO", "BE", "SETPD")):
        machine = Machine({}, features=features, isa=ISA)
        for _ in range(20):
            assert machine.host_irq == 0
            machine.step_cycle()


def test_swirq_raises_host_irq_one_edge_after_it_is_visible():
    """SEMANTICS 6.8: HOST_IRQ is registered from the values of the cycle before."""
    machine = machine_with([("CSRW", dict(csr=CSR["HOST_IRQ"], ra=0)), ("HALT", {})],
                           thread=2)
    machine.host_set_run(0b0100)
    series = {}
    records = []
    for _ in range(30):
        series[machine.cycle] = (machine.host_irq, machine.swirq)
        record = machine.step_cycle()
        if record is not None:
            records.append(record)
    csrw = [r for r in records if r.mnemonic == "CSRW"][0]
    visible = first_cycle(series, lambda v: v[1] == 0b0100)
    raised = first_cycle(series, lambda v: v[0] == 1)
    assert visible == csrw.x_cycle + 2                  # the commit edge
    assert raised == visible + 1                        # one edge later
    assert machine.host_read_ctrl("SWIRQ") == 0b0100


def test_swirq_is_write_one_to_clear_and_the_pin_follows_one_edge_later():
    machine = machine_with([("CSRW", dict(csr=CSR["HOST_IRQ"], ra=0)), ("HALT", {})])
    machine.host_set_run(0b0001)
    machine.run_cycles(20)
    assert (machine.swirq, machine.host_irq) == (0b0001, 1)
    machine.host_clear_swirq(0b0010)                  # another thread's bit
    machine.run_cycles(3)
    assert (machine.swirq, machine.host_irq) == (0b0001, 1)
    cleared_at = machine.cycle
    machine.host_write_ctrl("SWIRQ", 0b0001)
    series = irq_by_cycle(machine, 4, lambda m: m.swirq)
    assert series[cleared_at] == (1, 0b0001)
    assert series[cleared_at + 1] == (1, 0)           # SWIRQ gone, pin not yet
    assert series[cleared_at + 2] == (0, 0)


def test_a_thread_setting_swirq_wins_over_a_host_clear_at_the_same_edge():
    program = [("CSRW", dict(csr=CSR["HOST_IRQ"], ra=0)), ("NOP", {}),
               ("CSRW", dict(csr=CSR["HOST_IRQ"], ra=0)), ("HALT", {})]
    machine = machine_with(program)
    machine.host_set_run(0b0001)
    # Second CSRW: X cycle 14, in W during cycle 15, commits at edge 16.
    while machine.cycle < 15:
        machine.step_cycle()
    assert machine.swirq == 0b0001                    # from the first CSRW
    machine.host_clear_swirq(0b0001)
    record = machine.step_cycle()
    assert record is not None and record.mnemonic == "CSRW" and record.x_cycle == 14
    assert machine.swirq == 0b0001                    # the thread won


def test_irq_en_masks_irq_stat():
    machine = Machine({}, isa=ISA)
    machine.host_write_sflags_set(0x04)
    machine.run_cycles(5)
    assert machine.irq_stat == 0x0400 and machine.host_irq == 0   # not enabled
    enabled_at = machine.cycle
    machine.host_write_irq_en(0x0400)
    series = irq_by_cycle(machine, 4)
    assert series[enabled_at + 1][0] == 0             # IRQ_EN visible, pin not yet
    assert series[enabled_at + 2][0] == 1
    machine.host_write_irq_en(0x0800)                 # a different flag
    machine.run_cycles(3)
    assert machine.host_irq == 0


def test_irq_stat_uses_the_sflags_register_not_the_forwarded_value():
    """A SIG with X cycle x is visible in SFLAGS from x + 2; HOST_IRQ at x + 3."""
    machine = machine_with([("SIG", dict(flag=3)), ("HALT", {})])
    machine.host_write_irq_en(1 << (8 + 3))
    machine.host_set_run(0b0001)
    records = []
    series = {}
    for _ in range(20):
        series[machine.cycle] = (machine.host_irq, machine.sflags)
        record = machine.step_cycle()
        if record is not None:
            records.append(record)
    sig = [r for r in records if r.mnemonic == "SIG"][0]
    assert series[sig.x_cycle + 1][1] == 0            # forwarded to X only
    assert series[sig.x_cycle + 2] == (0, 0b1000)
    assert series[sig.x_cycle + 3] == (1, 0b1000)


def test_irq_en2_masks_halted_and_keeps_four_bits():
    machine = machine_with([("HALT", {})], thread=1)
    machine.host_write_irq_en2(0xFFF2)
    machine.host_set_run(0b0010)
    records = []
    series = {}
    for _ in range(20):
        series[machine.cycle] = (machine.host_irq, machine.halted)
        record = machine.step_cycle()
        if record is not None:
            records.append(record)
    assert machine.irq_en2 == 0x2 and machine.host_read_ctrl("IRQ_EN2") == 0x2
    halt = records[0]
    assert series[halt.x_cycle + 2] == (0, 0b0010)     # HALTED visible
    assert series[halt.x_cycle + 3] == (1, 0b0010)     # the pin, one edge later
    assert machine.host_read_ctrl("IRQ_STAT2") == 0b0010


def test_halted_needs_its_enable_bit():
    machine = machine_with([("HALT", {})])
    machine.host_write_irq_en2(0b1110)                 # not thread 0
    machine.host_set_run(0b0001)
    machine.run_cycles(20)
    assert machine.halted == 1 and machine.host_irq == 0


def test_irq_stat_fifo_fields_read_zero_without_fifos():
    machine = Machine({}, isa=ISA)
    machine.host_write_irq_en(0x00FF)
    machine.run_cycles(6)
    assert machine.irq_stat & 0xFF == 0
    assert machine.host_irq == 0


def test_irq_stat_fifo_fields_with_fifos():
    """INQ_NOT_FULL is 1 for an empty queue, so it is a cause from reset on."""
    machine = machine_with([("PUSH", dict(ra=0)), ("HALT", {})], features={"FIFO"},
                           fifo_depth=2)
    assert machine.irq_stat == 0x00F0
    machine.host_write_irq_en(0x0010)                  # INQ_NOT_FULL[0]
    machine.run_cycles(2)
    assert machine.host_irq == 1
    machine.host_fifo_push(0, 1)
    machine.step_cycle()
    machine.host_fifo_push(0, 2)
    machine.step_cycle()
    assert machine.irq_stat & 0x00F0 == 0x00E0          # INQ[0] full
    assert machine.host_irq == 1                        # one edge behind
    machine.step_cycle()
    assert machine.host_irq == 0
    # OUTQ_NOT_EMPTY[0] after the thread's PUSH.
    machine.host_write_irq_en(0x0001)
    machine.host_set_run(0b0001)
    records = []
    series = {}
    for _ in range(20):
        series[machine.cycle] = (machine.host_irq, machine.irq_stat & 0xF)
        record = machine.step_cycle()
        if record is not None:
            records.append(record)
    push = [r for r in records if r.mnemonic == "PUSH"][0]
    assert series[push.x_cycle + 2] == (0, 1)
    assert series[push.x_cycle + 3] == (1, 1)


def test_the_cycle_trace_carries_host_irq_and_uo_out_stays_six_bits():
    seen = []
    machine = machine_with([("CSRW", dict(csr=CSR["HOST_IRQ"], ra=0)),
                            ("SETP", dict(pin=21, val=1)), ("HALT", {})],
                           on_cycle=seen.append)
    machine.host_set_run(0b0001)
    machine.run_cycles(30)
    assert all(isinstance(t, CycleTrace) for t in seen)
    assert [t.host_irq for t in seen][-1] == 1
    assert max(t.uo_out for t in seen) == 0x20          # OUT5 only, bit 6 is not in it
    assert machine.uo_out == 0x20 and machine.host_irq == 1
    assert dataclasses.astuple(machine.pads) == (0x20, 0, 0)


# ------------------------------------------------------- debug write gating
GATED_NAMES = [
    ("PC", 5), ("FLAGS", 7), ("TD", 0x1234), ("DT", 0x4321), ("TICK_INT", 9),
    ("TICK_FRAC", 0x80), ("OUTGRP", 0x155), ("INGRP", 0x0AA), ("RS0", 0x111),
    ("RS1", 0x222), ("DEPTH", 2), ("WAIT_ACTIVE", 1), ("r3", 0xBEEF),
]


def _halted_value(machine, name):
    return machine.host_read_debug(0, name)


@pytest.mark.parametrize("name,value", GATED_NAMES)
def test_every_debug_write_is_dropped_while_the_thread_runs(name, value):
    machine = spinner()
    before = {n: machine.host_read_debug(0, n) for n, _ in GATED_NAMES}
    machine.host_set_run(0b0001)
    machine.run_cycles(9)
    machine.host_write_debug(0, name, value)
    machine.run_cycles(3)
    machine.host_set_run(0)
    machine.run_cycles(8)
    assert machine.thread_halted_for_debug(0)
    assert _halted_value(machine, name) == before[name]
    # ...and the same write while halted lands.
    machine.host_write_debug(0, name, value)
    machine.step_cycle()
    expected = value if name != "PC" else value & 0x3FF
    assert _halted_value(machine, name) == expected


@pytest.mark.parametrize("number,value", [
    (0x08, 5), (0x09, 3), (0x0A, 0x77), (0x0F, 0x12), (0x10, 12), (0x11, 3),
    (0x12, 0x21), (0x13, 0x42), (0x1A, 0x99), (0x1B, 2), (0x20, 500),
    (0x21, 0x0801), (0x22, 1), (0x23, 0x55), (0x24, 1),
])
def test_debug_writes_by_number_are_gated_the_same_way(number, value):
    machine = spinner()
    machine.host_write_debug(0, "TICK_INT", 1000)     # TICK_SEEN stays quiet
    machine.step_cycle()
    machine.host_set_run(0b0001)
    machine.run_cycles(9)
    before = machine.host_read_debug(0, number)
    machine.host_write_debug(0, number, value)
    machine.step_cycle()
    after = machine.host_read_debug(0, number)
    if number == 0x20:
        assert after in (before, before + 1)          # STEPS counts on, unwritten
    else:
        assert after == before, hex(number)


def test_debug_writes_are_dropped_while_a_step_request_is_pending_or_in_flight():
    machine = machine_with([("NOP", {}), ("HALT", {})])
    machine.host_step(0)
    accepted = {}
    for _ in range(10):
        cycle = machine.cycle
        machine.host_write_debug(0, "TD", cycle)
        machine.step_cycle()
        accepted[cycle] = machine.threads[0].td == cycle
    # The write of the STEP cycle itself is accepted (STEP_REQ is not yet
    # visible); then closed until the stepped slot has committed.
    closed = sorted(c for c, ok in accepted.items() if not ok)
    assert closed == list(range(1, closed[-1] + 1))
    assert accepted[0] is True and accepted[closed[-1] + 1] is True
    assert machine.threads[0].steps == 1


def test_debug_writes_are_accepted_before_a_first_run_and_after_halt():
    machine, _ = run_thread(ISA, [("HALT", {})])
    machine.host_write_debug(0, "OUTGRP", 0x3FF)
    machine.step_cycle()
    assert machine.threads[0].outgrp == 0x3FF
    fresh = Machine({}, isa=ISA)
    fresh.host_write_debug(3, "TD", 0xABCD)
    fresh.step_cycle()
    assert fresh.threads[3].td == 0xABCD


def test_misuse_of_debug_names_still_raises_whatever_the_run_state():
    machine = spinner()
    machine.host_set_run(0b0001)
    machine.run_cycles(8)
    for name in ("NOW", "TID", "FIFO_CNT", "ACC", "PREV_PINS", "INQ_CNT", "NOSUCH", "r8"):
        with pytest.raises(LoomsimError):
            machine.host_write_debug(0, name, 1)
    with pytest.raises(LoomsimError):
        machine.host_read_debug(4, "PC")


# ------------------------------------------------------------------ STEPS
def test_steps_is_writable_while_halted():
    machine = machine_with([("NOP", {}), ("NOP", {}), ("HALT", {})])
    machine.host_write_debug(0, "STEPS", 100)
    machine.step_cycle()
    assert machine.host_read_debug(0, 0x20) == 100
    machine.host_step(0)
    machine.run_cycles(8)
    assert machine.threads[0].steps == 101
    machine.host_write_debug(0, 0x20, 0xFFFF)
    machine.step_cycle()
    machine.host_step(0)
    machine.run_cycles(8)
    assert machine.threads[0].steps == 0              # wraps at 16 bits


# ------------------------------------------------------ the debug CSR window
def test_the_csr_window_reads_every_thread_csr_by_number():
    machine = Machine({}, isa=ISA)
    machine.run_cycles(5)
    th = machine.threads[2]
    values = {n: machine.host_read_debug(2, DEBUG_CSR + n) for n in range(16)}
    assert values[CSR["TICK_INT"]] == 1
    assert values[CSR["NOW"]] == th.now == machine.host_read_debug(2, "NOW")
    assert values[CSR["TID"]] == 2
    for name in ("BE_CFG", "BE_PINS", "BE_RELOAD", "CRC_POLY", "CRC_INIT", "SR",
                 "CNT", "CRC"):
        assert values[CSR[name]] == 0, name            # no bit engine in this build


def test_the_csr_window_writes_by_number_with_the_csr_widths():
    machine = Machine({}, isa=ISA)
    writes = {"TICK_INT": 0xFFFF, "TICK_FRAC": 0x1FF, "OUTGRP": 0xFFFF,
              "INGRP": 0x1234, "TD": 0x5555, "FLAGS": 0xFF}
    for name, value in writes.items():
        machine.host_write_debug(1, DEBUG_CSR + CSR[name], value)
    machine.host_write_debug(1, DEBUG_CSR + CSR["NOW"], 0x7777)    # read-only
    machine.host_write_debug(1, DEBUG_CSR + CSR["TID"], 3)         # read-only
    machine.host_write_debug(1, DEBUG_CSR + CSR["SR"], 0x7777)     # not built
    machine.step_cycle()
    th = machine.threads[1]
    assert (th.tick_int, th.tick_frac, th.outgrp, th.ingrp, th.td, th.flags) == \
        (0xFFFF, 0xFF, 0x3FF, 0x234, 0x5555, 7)
    # NOW did not take 0x7777; it did not even tick, because the divider
    # writes cleared the accumulator at that edge (SEMANTICS 4).
    assert th.now == 0 and th.tid == 1 and th.sr == 0
    assert machine.threads[0].now == 1
    assert machine.host_read_debug(1, DEBUG_CSR + CSR["TD"]) == 0x5555


def test_a_divider_write_through_the_window_clears_the_accumulator():
    machine = Machine({}, isa=ISA)
    machine.host_write_debug(0, DEBUG_CSR + CSR["TICK_INT"], 7)
    machine.host_write_debug(0, DEBUG_CSR + CSR["TICK_FRAC"], 128)
    machine.step_cycle()
    machine.run_cycles(3)
    assert machine.threads[0].acc != 0
    now = machine.threads[0].now
    machine.host_write_debug(0, DEBUG_CSR + CSR["TICK_FRAC"], 64)
    machine.step_cycle()
    assert machine.threads[0].acc == 0 and machine.threads[0].now == now


# --------------------------------------------------------- debug 0x21..0x26
def test_debug_0x21_packs_depth_and_rs1():
    machine = Machine({}, isa=ISA)
    machine.host_write_debug(0, 0x21, (2 << 10) | 0x2AA)
    machine.step_cycle()
    assert (machine.threads[0].rs1, machine.threads[0].depth) == (0x2AA, 2)
    assert machine.host_read_debug(0, 0x21) == 0x0AAA
    assert machine.host_read_debug(0, "RS1_DEPTH") == 0x0AAA


def test_debug_0x24_tick_seen_is_writable_while_halted():
    machine = Machine({}, isa=ISA)
    machine.run_cycles(2)                              # a tick at every edge so far
    machine.host_write_debug(0, "TICK_INT", 1000)      # then ticks become rare
    machine.step_cycle()
    assert machine.host_read_debug(0, 0x24) == 1
    machine.host_write_debug(0, 0x24, 0)
    machine.step_cycle()
    assert machine.host_read_debug(0, "TICK_SEEN") == 0
    machine.host_write_debug(0, "TICK_SEEN", 1)
    machine.step_cycle()
    assert machine.threads[0].tick_seen == 1


def test_a_tick_wins_over_a_host_clear_of_tick_seen_at_the_same_edge():
    machine = Machine({}, isa=ISA)                     # a tick at every edge
    machine.run_cycles(3)
    machine.host_write_debug(0, 0x24, 0)
    machine.step_cycle()
    assert machine.threads[0].tick_seen == 1


def test_debug_0x25_and_0x26_read_zero_without_their_features():
    machine = Machine({}, isa=ISA)
    machine.host_write_debug(0, 0x25, 0x7F)
    machine.host_write_debug(0, "LAT", 0x7F)
    machine.step_cycle()
    assert machine.host_read_debug(0, 0x25) == 0
    assert machine.host_read_debug(0, "LAT_VALID") == 0
    assert machine.threads[0].lat_valid == 0
    assert machine.host_read_debug(0, 0x26) == 0
    assert machine.host_read_debug(0, "INQ_CNT") == 0


def test_unused_debug_numbers_read_zero_and_ignore_writes():
    machine = Machine({}, isa=ISA)
    for number in (0x27, 0x30, 0x80, 0xFF):
        machine.host_write_debug(0, number, 0xFFFF)
        assert machine.host_read_debug(0, number) == 0
    machine.step_cycle()
    assert machine.dump_thread(0) == Machine({}, isa=ISA).dump_thread(0) | {
        "NOW": 1, "TICK_SEEN": 1, "ACC": 0}


def test_dump_debug_space_covers_0x00_to_0x26():
    machine, _ = run_thread(ISA, [("LDI", dict(rd=2, imm=9)), ("HALT", {})])
    space = machine.dump_debug_space(0)
    assert sorted(space) == list(range(0x27))
    assert space[0x02] == 9 and space[0x08] == 2 and space[0x20] == 2
    assert space[0x1C] == 0 and machine.dump_debug_space(3)[0x1C] == 3


# ------------------------------------------------ what the M1 co-sim relies on
def test_dump_thread_keeps_its_m1_keys():
    machine = Machine({}, isa=ISA)
    assert list(machine.dump_thread(0)) == list(DEBUG_REGS) + ["RUN", "HALTED", "BADOP"]
    assert DEBUG_REGS == ("r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "PC",
                          "FLAGS", "TD", "NOW", "STEPS", "RS0", "RS1", "DEPTH",
                          "WAIT_ACTIVE", "TICK_INT", "TICK_FRAC", "OUTGRP", "INGRP",
                          "DT", "ACC", "TICK_SEEN", "PREV_PINS")


def test_pad_state_keeps_three_fields():
    assert [f.name for f in dataclasses.fields(PadState)] == ["uo_out", "uio_out", "uio_oe"]


def test_unknown_features_and_bad_fifo_depths_are_refused():
    with pytest.raises(LoomsimError):
        Machine({}, features={"SETP_D"}, isa=ISA)
    for depth in (1, 3, 16):
        with pytest.raises(LoomsimError):
            Machine({}, features={"FIFO"}, fifo_depth=depth, isa=ISA)
    Machine({}, features={"fifo", "be", "setpd"}, fifo_depth=2, isa=ISA)
    Machine({}, fifo_depth=3, isa=ISA)                 # no FIFOs: not checked


# ------------------------------------------------------------------- CLI
def _image(tmp_path, program):
    import json
    words = assemble(ISA, program)
    path = tmp_path / "image.json"
    path.write_text(json.dumps({"words": {str(a): w for a, w in words.items()}}),
                    encoding="utf-8")
    return path


def test_cli_accepts_the_three_features_in_any_case(tmp_path, capsys):
    path = _image(tmp_path, [("SETP", dict(pin=16, val=1, lat=1)), ("HALT", {})])
    main(["run", str(path), "--cycles", "40"])
    assert "uo_out=01" in capsys.readouterr().out       # plain SETP without SETPD
    main(["run", str(path), "--cycles", "40", "--feature", "setpd"])
    assert "uo_out=01" not in capsys.readouterr().out   # staged, no deadline reached
    main(["run", str(path), "--cycles", "40", "--feature", "FIFO", "--feature", "Be",
          "--fifo-depth", "8"])
    assert "badop=0000" in capsys.readouterr().out


def test_cli_rejects_other_features_and_depths(tmp_path, capsys):
    path = _image(tmp_path, [("HALT", {})])
    with pytest.raises(SystemExit):
        main(["run", str(path), "--feature", "DMEM"])
    with pytest.raises(SystemExit):
        main(["run", str(path), "--feature", "FIFO", "--fifo-depth", "5"])
    capsys.readouterr()


def test_cli_pad_lines_show_the_interrupt(tmp_path, capsys):
    path = _image(tmp_path, [("CSRW", dict(csr=CSR["HOST_IRQ"], ra=0)), ("HALT", {})])
    main(["run", str(path), "--cycles", "40"])
    assert "irq=1" in capsys.readouterr().out
