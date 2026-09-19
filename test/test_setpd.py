# SPDX-License-Identifier: Apache-2.0
"""Deadline-latched pin writes, `SETP pin, v, D` (docs/SEMANTICS.md 6.10).

`SETP ... D` writes no pin; it stages {LAT_VALID, LAT_PIN, LAT_VAL} at its
commit edge, replacing what was staged. A staged write is applied, and
LAT_VALID cleared, at the first LATER edge e at which
  rule 1: NOW ticks at e to exactly TD (TD not written at e), or
  rule 2: TD is written at e (WAITD first issue, SETD, CSRW TD, the host)
          and reached(NOW', TD') holds for the values after e.
It follows 6.3 (open drain included); an ordinary pin write of a slot wins
at the same edge; CTRL.RESET discards it; debug 0x25 reads and writes it.

Timing is read on the pads (PadMonitor): a pad register loaded at edge e
shows in entry e. An ordinary `SETP OUT1, 1` marker with X cycle x_m shows
in entry m = x_m + 2, and the thread's next slots have X cycles x_m + 4,
x_m + 8, ... With TICK_INT = 1 (the reset value) NOW advances at every
edge, so `SETD k` in the slot after the marker (X cycle x_m + 4, NOW = N)
sets TD = N + k, and NOW ticks to TD at edge x_m + 4 + k = entry m + 2 + k.
The fractional-tick test computes tick edges from section 4 instead: after
the TICK_FRAC write clears ACC at edge w (PadMonitor.host_commit), the n-th
tick lands at w + ceil(n * period), with period in clocks.

Expected values come from the text, not from the golden model.
"""

import cocotb
from cocotb.triggers import ClockCycles

from spi_host import (
    LoomHost, PadMonitor, asm, SP_CTRL, SP_DEBUG, CTRL_PIN_OUT, CTRL_CAPS,
    DBG_LATCH, DBG_TD, DBG_NOW, CSR_TICK_INT, CSR_TICK_FRAC, CAPS_LAT,
)

OUT0, OUT1, OUT2 = 16, 17, 18
LAT_VALID, LAT_VAL = 1 << 6, 1 << 5


def setpd(pin, val):
    """The D form: stage the write for the thread's next deadline."""
    return asm("SETP", pin=pin, val=val, lat=1)


def setp(pin, val):
    return asm("SETP", pin=pin, val=val)


def rises(mon, bit, start):
    return [c for c, v in mon.changes(mon.uo, bit, start) if v]


async def run_at(host, prog, t, base, timeout_slots=4000):
    await host.load_program(prog, verify=False)
    await host.set_reset_pc(t, base)
    await host.reset_thread(t)
    await host.run(1 << t)
    await host.wait_halted(1 << t, timeout_slots)


async def fresh(dut):
    host = LoomHost(dut)
    await host.start()
    assert await host.read1(SP_CTRL, CTRL_CAPS) & CAPS_LAT, \
        "the build must report the deadline-latched SETP (CAPS[7])"
    return host, PadMonitor(dut).start()


# --------------------------------------------------------------------- tests
def sweep_expect(k):
    """Entry of OUT0's rise relative to the marker m, for the program of
    test_setpd_rules_and_arming_edge (arm at m + 8, WAITD 0 first issue
    commits TD at m + 12, NOW ticks to TD at m + 2 + k)."""
    if k <= 6:
        # NOW reached TD at or before the arming edge: the arming edge never
        # applies (k = 6 is exactly the arming edge), rule 1 cannot fire any
        # more, so the WAITD's TD write applies it by rule 2: already late.
        return 12
    if k <= 9:
        return 2 + k            # rule 1, between the slots, to the clock
    if k == 10:
        return 12               # TD written at that edge: rule 2, on time
    return 2 + k                # rule 1 while the WAITD stalls (re-issues write no TD)


@cocotb.test()
async def test_setpd_rules_and_arming_edge(dut):
    """Sweep the deadline around a SETP D: rule 1 applies the write on the
    exact tick edge (off the 4-clock slot grid), the arming edge never
    applies it, a deadline already passed applies it at the WAITD's first
    issue (rule 2), and nothing else writes the pin."""
    host, mon = await fresh(dut)
    t = 0
    for k in (3, 5, 6, 7, 8, 9, 10, 11, 12, 13, 17):
        await host.write(SP_CTRL, CTRL_PIN_OUT, 0)
        prog = {0x00: setp(OUT1, 1),          # marker: entry m
                0x01: asm("SETD", imm=k),       # TD = NOW + k
                0x02: setpd(OUT0, 1),           # arms at m + 8
                0x03: asm("WAITD", imm=0),      # first issue writes TD at m + 12
                0x04: setp(OUT1, 0),
                0x05: asm("HALT")}
        start = mon.now
        await run_at(host, prog, t, 0)
        m = rises(mon, 1, start)[0]
        got = [c - m for c in rises(mon, 0, start)]
        assert got == [sweep_expect(k)], f"k={k}: OUT0 rose at m+{got}"
        assert mon.changes(mon.uo, 0, start)[-1][1] == 1, "exactly one pin write"
        lat = await host.read_debug(t, DBG_LATCH)
        assert lat == LAT_VAL | OUT0, f"k={k}: latch {lat:#04x} after the write"
    mon.stop()
    await ClockCycles(dut.clk, 2)


@cocotb.test()
async def test_setpd_fractional_tick_is_clock_exact(dut):
    """Headline (D-016): at 433.5 clocks per tick, `SETP OUT0, v, D; WAITD 1`
    puts every edge on a tick edge, so the spacings are the tick spacings
    (433 or 434) to the clock. The same loop with plain SETP lands on the
    thread's 4-clock grid and dithers 432/436."""
    host, mon = await fresh(dut)
    t = 0
    period = 433 * 256 + 128                     # 1/256 clocks
    iters = 12
    results = {}
    base = {}                                    # tick origin per run
    for lat in (1, 0):
        await host.write(SP_CTRL, CTRL_PIN_OUT, 0)
        await host.write_csr(t, CSR_TICK_INT, 433)
        start = mon.now
        await host.write_csr(t, CSR_TICK_FRAC, 128)
        w = PadMonitor.host_commit(mon.sck_rises(start)[-1])   # ACC = 0 from edge w
        base[lat] = w
        on = setpd(OUT0, 1) if lat else setp(OUT0, 1)
        off = setpd(OUT0, 0) if lat else setp(OUT0, 0)
        prog = {0x00: asm("SETD", imm=2),
                0x01: on,
                0x02: asm("WAITD", imm=1),
                0x03: off,
                0x04: asm("WAITD", imm=1),
                0x05: asm("DJNZ", rd=1, rel=-5),
                0x06: asm("HALT")}
        await host.load_program(prog, verify=False)
        await host.set_reset_pc(t, 0)
        await host.reset_thread(t)
        await host.write_reg(t, 1, iters)
        start = mon.now
        await host.run(1 << t)
        await host.wait_halted(1 << t, timeout_slots=20000)
        edges = mon.changes(mon.uo, 0, start)
        assert len(edges) == 2 * iters, f"lat={lat}: {len(edges)} edges"
        assert [v for _, v in edges] == [1, 0] * iters
        results[lat] = [c for c, _ in edges]
    # Tick n of the SETP D run lands on edge base[1] + ceil(n * period / 256).
    w = base[1]
    ticks = {w + (n * period + 255) // 256: n for n in range(1, 400)}
    exact = results[1]
    ns = [ticks.get(c) for c in exact]
    assert None not in ns, f"edges off the tick edges: {[c - w for c in exact]}"
    assert ns == list(range(ns[0], ns[0] + len(ns))), f"not consecutive ticks: {ns}"
    gaps = [b - a for a, b in zip(exact, exact[1:])]
    assert set(gaps) == {433, 434}, gaps
    # Plain SETP, same loop: the first edge is written straight after SETD (not
    # at a deadline), and the loop's two paths differ by one slot (DJNZ sits on
    # the "on" path only), so its edges sit on the thread's 4-clock slot grid,
    # not on the tick edges (SEMANTICS 2, "Slot grid").
    ticks0 = {base[0] + (n * period + 255) // 256 for n in range(1, 400)}
    later = results[0][1:]
    plain = [b - a for a, b in zip(later, later[1:])]
    assert all(g % 4 == 0 for g in plain), plain
    assert any(c not in ticks0 for c in later), "plain SETP landed on every tick edge"
    dut._log.info("SETP D gaps %s; plain SETP gaps %s", gaps, plain)
    mon.stop()
    await ClockCycles(dut.clk, 2)


@cocotb.test()
async def test_setpd_late_applies_at_waitd_commit(dut):
    """A thread already past its deadline: `SETP OUT0, 1, D; WAITD 1` writes
    the pin at the WAITD's commit edge, exactly where an ordinary SETP in
    that slot would (rule 2), here with a fractional tick."""
    host, mon = await fresh(dut)
    t = 1
    await host.write_csr(t, CSR_TICK_INT, 5)
    await host.write_csr(t, CSR_TICK_FRAC, 77)
    await host.write(SP_CTRL, CTRL_PIN_OUT, 0)
    prog = {0x40: asm("SETD", imm=0),
            0x41: asm("DLY", imm=4),              # now 4 ticks behind TD
            0x42: setp(OUT1, 1),                  # marker: entry m
            0x43: setpd(OUT0, 1),                 # arms at m + 4
            0x44: asm("WAITD", imm=1),            # TD + 1 is long past: m + 8
            0x45: setp(OUT2, 1),                  # the next slot: m + 12
            0x46: asm("HALT")}
    start = mon.now
    await run_at(host, prog, t, 0x40)
    m = rises(mon, 1, start)[0]
    assert [c - m for c in rises(mon, 0, start)] == [8]
    assert [c - m for c in rises(mon, 2, start)] == [12]
    assert await host.read_debug(t, DBG_LATCH) == LAT_VAL | OUT0
    mon.stop()
    await ClockCycles(dut.clk, 2)


@cocotb.test()
async def test_setpd_ordinary_write_wins(dut):
    """At an edge where the staged write and a slot's ordinary pin write both
    touch a pin, the ordinary write wins; on different pins both apply."""
    host, mon = await fresh(dut)
    t = 2
    base = 0x80
    # SETD k at the marker's next slot; the staged write fires at m + 2 + k;
    # the ordinary SETP in slot 3 commits at m + 12.
    cases = [
        # (k, ordinary pin, value): expected OUT0 changes, rel. to m
        (9, OUT0, 0, [(11, 1), (12, 0)]),   # staged first, then overwritten
        (10, OUT0, 0, []),                  # same edge: the ordinary 0 wins
        (11, OUT0, 0, [(13, 1)]),           # ordinary first, staged after
        (10, OUT2, 1, [(12, 1)]),           # same edge, other pin: both
    ]
    for k, pin, val, want in cases:
        await host.write(SP_CTRL, CTRL_PIN_OUT, 0)
        prog = {base + 0: setp(OUT1, 1),
                base + 1: asm("SETD", imm=k),
                base + 2: setpd(OUT0, 1),            # arms at m + 8
                base + 3: setp(pin, val),            # commits at m + 12
                base + 4: asm("WAITD", imm=0),
                base + 5: asm("HALT")}
        start = mon.now
        await run_at(host, prog, t, base)
        m = rises(mon, 1, start)[0]
        got = [(c - m, v) for c, v in mon.changes(mon.uo, 0, start)]
        assert got == want, f"k={k} pin={pin}: OUT0 {got}, want {want}"
        if pin == OUT2:
            assert [c - m for c in rises(mon, 2, start)] == [12]
        lat = await host.read_debug(t, DBG_LATCH)
        assert not lat & LAT_VALID, f"k={k}: the staged write was consumed"
    mon.stop()
    await ClockCycles(dut.clk, 2)


@cocotb.test()
async def test_setpd_replaced_write_applies_at_its_deadline(dut):
    """A new SETP D replaces the staged write at its commit edge; if that edge
    is the old write's deadline (rule 1), the old write is applied there
    (it was staged before that edge) and the new one waits for its own."""
    host, mon = await fresh(dut)
    t = 3
    base = 0xC0
    await host.write(SP_CTRL, CTRL_PIN_OUT, 0)
    prog = {base + 0: setp(OUT1, 1),              # marker: m
            base + 1: asm("SETD", imm=10),        # NOW ticks to TD at m + 12
            base + 2: setpd(OUT0, 1),             # arms at m + 8
            base + 3: setpd(OUT2, 1),             # replaces it at m + 12
            base + 4: asm("WAITD", imm=0),        # first issue, TD written: m + 16
            base + 5: asm("HALT")}
    start = mon.now
    await run_at(host, prog, t, base)
    m = rises(mon, 1, start)[0]
    assert [c - m for c in rises(mon, 0, start)] == [12], "old write at its deadline"
    assert [c - m for c in rises(mon, 2, start)] == [16], "new write by rule 2"
    mon.stop()
    await ClockCycles(dut.clk, 2)


@cocotb.test()
async def test_setpd_ctrl_reset_discards(dut):
    """CTRL.RESET clears LAT_VALID without a pin write (although it also sets
    TD := NOW); without the reset, a halted thread's staged write still lands
    on its deadline, because the tick generator never stops."""
    host, mon = await fresh(dut)
    t = 1
    base = 0x40
    await host.write_csr(t, CSR_TICK_INT, 40)
    await host.write(SP_CTRL, CTRL_PIN_OUT, 0)
    prog = {base + 0: asm("SETD", imm=150),   # 6000 clocks ahead
            base + 1: setpd(OUT0, 1),
            base + 2: asm("HALT")}
    start = mon.now
    await run_at(host, prog, t, base)
    assert await host.read_debug(t, DBG_LATCH) == LAT_VALID | LAT_VAL | OUT0
    await host.reset_thread(t)
    assert await host.read_debug(t, DBG_LATCH) == LAT_VAL | OUT0
    await ClockCycles(dut.clk, 7000)
    assert mon.changes(mon.uo, 0, start) == [], "a discarded write never lands"

    # The same without the reset: the write lands while the thread is halted.
    start = mon.now
    await run_at(host, prog, t, base)
    assert await host.read_debug(t, DBG_LATCH) == LAT_VALID | LAT_VAL | OUT0
    await ClockCycles(dut.clk, 7000)
    assert [v for _, v in mon.changes(mon.uo, 0, start)] == [1]
    assert await host.read_debug(t, DBG_LATCH) == LAT_VAL | OUT0
    mon.stop()
    await ClockCycles(dut.clk, 2)


@cocotb.test()
async def test_setpd_debug_latch_and_host_td(dut):
    """Debug 0x25 is {LAT_VALID, LAT_VAL, LAT_PIN} in bits 6:0, writable while
    halted. A host TD write is a rule-2 TD write: with a staged write and a
    deadline already reached, the pin changes at that write's commit edge;
    with a future deadline nothing happens until then."""
    host, mon = await fresh(dut)
    t = 2
    await host.write_debug(t, DBG_LATCH, 0xFFFF)
    assert await host.read_debug(t, DBG_LATCH) == 0x7F, "bits 15:7 read 0"
    await host.write_debug(t, DBG_LATCH, 0)
    assert await host.read_debug(t, DBG_LATCH) == 0
    await host.write(SP_CTRL, CTRL_PIN_OUT, 0)

    # Armed by the host, then TD written in the past: applied at the commit
    # edge of the TD write (docs/spec-questions/rtl-m2.md 7).
    await host.write_debug(t, DBG_LATCH, LAT_VALID | LAT_VAL | OUT0)
    now = await host.read_debug(t, DBG_NOW)
    start = mon.now
    await host.write_debug(t, DBG_TD, (now - 100) & 0xFFFF)
    visible = PadMonitor.host_commit(mon.sck_rises(start)[-1])
    await ClockCycles(dut.clk, 10)
    assert mon.changes(mon.uo, 0, start) == [(visible, 1)]
    assert await host.read_debug(t, DBG_LATCH) == LAT_VAL | OUT0

    # A future deadline: nothing at the TD write; the staged write stays.
    await host.write_debug(t, DBG_LATCH, LAT_VALID | OUT0)      # value 0
    now = await host.read_debug(t, DBG_NOW)
    start = mon.now
    await host.write_debug(t, DBG_TD, (now + 20000) & 0xFFFF)
    await ClockCycles(dut.clk, 100)
    assert mon.changes(mon.uo, 0, start) == []
    assert await host.read_debug(t, DBG_LATCH) == LAT_VALID | OUT0
    await host.reset_thread(t)
    assert await host.read_debug(t, DBG_LATCH) == OUT0
    mon.stop()
    await ClockCycles(dut.clk, 2)


@cocotb.test()
async def test_setpd_two_threads_one_edge(dut):
    """Two threads' staged writes to one pin at one edge: the higher thread
    wins (docs/spec-questions/rtl-m2.md 8). Threads 0 and 3 share NOW (both
    tick every clock since reset), so equal TDs fire at the same edge."""
    host, mon = await fresh(dut)
    for pin_out, v0, v3 in ((0x0000, 0, 1), (0x0100, 1, 0)):
        await host.write(SP_CTRL, CTRL_PIN_OUT, pin_out)
        await host.write_debug(0, DBG_LATCH, LAT_VALID | (LAT_VAL if v0 else 0) | OUT0)
        await host.write_debug(3, DBG_LATCH, LAT_VALID | (LAT_VAL if v3 else 0) | OUT0)
        now0 = await host.read_debug(0, DBG_NOW)
        now3 = await host.read_debug(3, DBG_NOW)
        assert (now3 - now0) & 0xFFFF < 2000, (now0, now3)      # same clock
        target = (now3 + 3000) & 0xFFFF
        start = mon.now
        await host.write_debug(0, DBG_TD, target)
        await host.write_debug(3, DBG_TD, target)
        await ClockCycles(dut.clk, 3000)
        changes = mon.changes(mon.uo, 0, start)
        assert [v for _, v in changes] == [v3], f"thread 3 must win: {changes}"
        assert not await host.read_debug(0, DBG_LATCH) & LAT_VALID
        assert not await host.read_debug(3, DBG_LATCH) & LAT_VALID
    mon.stop()
    await ClockCycles(dut.clk, 2)
