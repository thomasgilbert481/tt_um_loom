# SPDX-License-Identifier: Apache-2.0
"""The point of the chip: when a pin edge happens.

A wait completes in the first X cycle at or after its deadline, and a thread
has an X cycle only every 4 clocks (docs/SEMANTICS.md 2 and 6.4). So a pin
edge driven by firmware lands on that thread's 4-clock slot grid: it is exact
when k * period is a multiple of 4 clocks, and otherwise late by 0..3 clocks
with no accumulation. These tests check both, that a taken branch costs
exactly one slot, and that another running thread moves nothing.
"""

import cocotb
from cocotb.triggers import ClockCycles, RisingEdge

from spi_host import (
    LoomHost, asm, resolve, SP_CTRL, CTRL_PIN_OUT,
    CSR_TICK_INT, CSR_TICK_FRAC,
)

OUT0 = 16                      # pin index of uo_out[0]


async def watch_pad(dut, events, bit=0):
    """Record (clock number, level) for every change of uo_out[bit]."""
    cycle = 0
    last = (resolve(dut.uo_out) >> bit) & 1
    while True:
        await RisingEdge(dut.clk)
        cycle += 1
        value = (resolve(dut.uo_out) >> bit) & 1
        if value != last:
            events.append((cycle, value))
            last = value


def toggle_program(k, pairs, start=0):
    """Toggle OUT0 every `k` deadlines, `pairs` times, then halt."""
    return {
        start + 0: asm("CSRW", csr=CSR_TICK_INT, ra=1),
        start + 1: asm("CSRW", csr=CSR_TICK_FRAC, ra=2),
        start + 2: asm("SETD", imm=0),          # TD = NOW: anchor the schedule
        start + 3: asm("WAITD", imm=k),
        start + 4: asm("SETP", pin=OUT0, val=1),
        start + 5: asm("WAITD", imm=k),
        start + 6: asm("SETP", pin=OUT0, val=0),
        start + 7: asm("DJNZ", rd=3, rel=-5),
        start + 8: asm("HALT"),
    }


async def run_toggles(host, dut, tick_int, tick_frac, k, pairs, extra=None):
    """Run the toggle program on thread 0 and return the edge clock numbers."""
    await host.write(SP_CTRL, CTRL_PIN_OUT, 0)
    prog = dict(toggle_program(k, pairs))
    if extra:
        prog.update(extra)
    await host.load_program(prog, verify=False)
    await host.set_reset_pc(0, 0)
    await host.reset_thread(0)
    await host.write_reg(0, 1, tick_int)
    await host.write_reg(0, 2, tick_frac)
    await host.write_reg(0, 3, pairs)
    events = []
    monitor = cocotb.start_soon(watch_pad(dut, events))
    mask = 0b0001 if not extra else 0b0011
    await host.run(mask)
    await host.wait_halted(0b0001)
    await ClockCycles(dut.clk, 4)
    monitor.cancel()
    await host.halt()
    await ClockCycles(dut.clk, 4)
    return [c for c, _ in events]


@cocotb.test()
async def test_exact_when_period_is_a_multiple_of_four(dut):
    """k * period a multiple of 4 clocks: the spacing is exact, every time."""
    host = LoomHost(dut)
    await host.start()
    for tick_int, k in ((32, 1), (8, 4), (16, 3)):
        want = tick_int * k
        assert want % 4 == 0
        edges = await run_toggles(host, dut, tick_int, 0, k, 6)
        assert len(edges) == 12, f"expected 12 edges, got {len(edges)}"
        gaps = [b - a for a, b in zip(edges, edges[1:])]
        assert all(g == want for g in gaps), \
            f"TICK_INT={tick_int}, WAITD {k}: gaps {gaps}, expected {want}"


@cocotb.test()
async def test_bounded_jitter_and_no_drift(dut):
    """Other periods: each edge is 0..3 clocks late, and nothing accumulates."""
    host = LoomHost(dut)
    await host.start()
    cases = [(33, 0, 1), (30, 0, 1), (32, 64, 1), (17, 128, 2)]
    for tick_int, tick_frac, k in cases:
        period = tick_int + tick_frac / 256.0
        want = period * k
        edges = await run_toggles(host, dut, tick_int, tick_frac, k, 8)
        assert len(edges) == 16
        gaps = [b - a for a, b in zip(edges, edges[1:])]
        for g in gaps:
            assert abs(g - want) < 4, \
                f"TICK_INT={tick_int}.{tick_frac} k={k}: gap {g} vs {want}"
        # No drift: the whole span matches the ideal one within one slot.
        span = edges[-1] - edges[0]
        ideal = want * (len(edges) - 1)
        assert abs(span - ideal) < 4, \
            f"drift: span {span} vs {ideal} over {len(edges)} edges"
        # Every gap sits on the slot grid.
        assert all(g % 4 == 0 for g in gaps), f"gaps off the slot grid: {gaps}"


@cocotb.test()
async def test_taken_branch_costs_one_slot(dut):
    """A taken branch costs exactly one slot (4 clocks), like every other
    instruction, whichever way the branch goes."""
    host = LoomHost(dut)
    await host.start()

    async def measure(program, flags):
        await host.write(SP_CTRL, CTRL_PIN_OUT, 0)
        await host.load_program(program, verify=False)
        await host.set_reset_pc(0, 0)
        await host.reset_thread(0)
        await host.write_reg(0, 1, flags)
        events = []
        monitor = cocotb.start_soon(watch_pad(dut, events))
        await host.run(0b0001)
        await host.wait_halted(0b0001)
        await ClockCycles(dut.clk, 4)
        monitor.cancel()
        assert len(events) >= 2, f"only {len(events)} edges"
        return events[1][0] - events[0][0]

    # Z = 1. BZ is taken and jumps over the NOP: two slots between the pin
    # writes (the SETP itself and the branch).
    taken = await measure({
        0x00: asm("CSRW", csr=0x0B, ra=1),
        0x01: asm("SETP", pin=OUT0, val=1),
        0x02: asm("BZ", rel=1),
        0x03: asm("NOP"),
        0x04: asm("SETP", pin=OUT0, val=0),
        0x05: asm("HALT"),
    }, 0x1)
    assert taken == 8, f"taken branch path took {taken} clocks, expected 8"

    # Same program with the opposite condition: the branch falls through and
    # the NOP runs, so the distance is one slot more.
    not_taken = await measure({
        0x00: asm("CSRW", csr=0x0B, ra=1),
        0x01: asm("SETP", pin=OUT0, val=1),
        0x02: asm("BNZ", rel=1),
        0x03: asm("NOP"),
        0x04: asm("SETP", pin=OUT0, val=0),
        0x05: asm("HALT"),
    }, 0x1)
    assert not_taken == 12, f"fall-through path took {not_taken}, expected 12"
    assert not_taken - taken == 4, "a skipped instruction is exactly one slot"

    # A backward taken branch is also one slot: CALL and RET cost one each.
    call_ret = await measure({
        0x00: asm("SETP", pin=OUT0, val=1),
        0x01: asm("CALL", abs=0x10),
        0x02: asm("SETP", pin=OUT0, val=0),
        0x03: asm("HALT"),
        0x10: asm("RET"),
    }, 0x0)
    assert call_ret == 12, f"SETP, CALL, RET took {call_ret}, expected 12"


@cocotb.test()
async def test_other_threads_do_not_move_the_edges(dut):
    """A second thread in a busy loop must not move thread 0 by one clock."""
    host = LoomHost(dut)
    await host.start()
    alone = await run_toggles(host, dut, 32, 0, 1, 8)
    gaps_alone = [b - a for a, b in zip(alone, alone[1:])]

    # Thread 1 spins on a three-instruction loop with an ALU operation and a
    # taken branch in it, at a different program address.
    busy = {0x40: asm("ADDI", rd=7, imm=1),
            0x41: asm("XOR", rd=6, ra=7, rb=7),
            0x42: asm("JMP", abs=0x40)}
    # RESET_PC reaches the PC only through a thread reset. Without one,
    # thread 1 starts at its power-on PC, which is 0x80 in the 512-word build
    # and holds unwritten (X) memory: the RTL decoded that quietly and passed,
    # the gate-level netlist spread the X into the pin register (BUGS 4).
    await host.set_reset_pc(1, 0x40)
    await host.reset_thread(1)
    await host.write_reg(1, 6, 0xFFFF)
    await host.write_reg(1, 7, 0)
    with_busy = await run_toggles(host, dut, 32, 0, 1, 8, extra=busy)
    gaps_busy = [b - a for a, b in zip(with_busy, with_busy[1:])]

    assert gaps_alone == gaps_busy, \
        f"thread 1 moved the edges: {gaps_alone} vs {gaps_busy}"
    assert all(g == 32 for g in gaps_busy), gaps_busy
    # ...and thread 1 really ran its loop the whole time.
    loops = await host.read_reg(1, 7)
    assert loops > 40 and await host.read_reg(1, 6) == 0, \
        f"thread 1 did not run the busy loop (r7 = {loops})"
    await host.halt()
    await ClockCycles(dut.clk, 8)


@cocotb.test()
async def test_tick_int_zero_is_one(dut):
    """TICK_INT = 0 is stored as 0 and the divider treats it as 1 (SEMANTICS 4),
    with and without a fraction. Nothing set it to 0 until a mutant that read
    it as 2 survived."""
    host = LoomHost(dut)
    await host.start()
    for frac in (0, 128):
        one = await run_toggles(host, dut, 1, frac, 16, 3)
        zero = await run_toggles(host, dut, 0, frac, 16, 3)
        gaps_one = [b - a for a, b in zip(one, one[1:])]
        gaps_zero = [b - a for a, b in zip(zero, zero[1:])]
        assert gaps_zero == gaps_one, (frac, gaps_zero, gaps_one)
    await host.halt()
    await ClockCycles(dut.clk, 8)


@cocotb.test()
async def test_long_tick_period(dut):
    """A 32,768-clock tick keeps its period. The accumulator's top bit only
    matters for TICK_INT >= 0x8000; a mutant that dropped it made the timer
    tick on every clock after the first tick, and nothing used a tick that
    long."""
    host = LoomHost(dut)
    await host.start()
    edges = await run_toggles(host, dut, 0x8000, 0, 1, 1)
    gaps = [b - a for a, b in zip(edges, edges[1:])]
    assert gaps == [0x8000], gaps
    await host.halt()
    await ClockCycles(dut.clk, 8)
