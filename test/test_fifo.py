# SPDX-License-Identifier: Apache-2.0
"""FIFOs (docs/SEMANTICS.md 6.7, docs/HOST_PROTOCOL.md SPACE 3).

Everything goes through the pads: the host side with the SPI port, the
thread side with PUSH/POP/WAITB programs, and timing is measured on the pads
with ``PadMonitor``. Expected values come from the specification:

* a host word takes effect at the end of the word, so the push or pop is
  visible from ``PadMonitor.host_commit(<cycle of its last SCK rise>)``;
* a blocked PUSH/POP/WAITB completes in the first X cycle of its thread at or
  after the FIFO state it waits for is visible, and a SETP in the next slot
  changes the pad two cycles after its own X cycle, so the blocked slot's X
  cycle is ``pad edge - 6``;
* counts, status words and BADOP[14] follow the text literally.
"""

import cocotb
from cocotb.triggers import ClockCycles

from spi_host import (
    LoomHost, PadMonitor, asm, run_snippet, SP_CTRL, SP_FIFO,
    CTRL_BADOP, CTRL_PIN_OUT, FIFO_QUEUE, FIFO_STATUS,
    DBG_PC, DBG_WAIT_ACTIVE, DBG_STEPS, DBG_FLAGS, DBG_TICK_SEEN,
    DBG_FIFO_COUNTS, CSR_TICK_INT, FLAG_T,
)

OUT0 = 16                       # pin index of uo_out[0]
BADOP14 = 1 << 14


# ----------------------------------------------------------------- helpers
async def fifo_depth(host):
    caps = await host.caps()
    assert caps & 0x08, "this test needs a build with FIFOs (CAPS[3])"
    return 1 << (caps & 0x7)


async def status(host, t):
    """Decoded status word of thread t (HOST_PROTOCOL SPACE 3, 0x0100 + t)."""
    w = await host.read1(SP_FIFO, FIFO_STATUS + t)
    assert w >> 12 == 0, f"status word {w:#06x}: bits 15:12 must read 0"
    return {"ocnt": (w >> 8) & 0xF, "icnt": (w >> 4) & 0xF,
            "oempty": (w >> 3) & 1, "ofull": (w >> 2) & 1,
            "iempty": (w >> 1) & 1, "ifull": w & 1}


def expect_status(icnt, ocnt, depth):
    return {"ocnt": ocnt, "icnt": icnt,
            "oempty": int(ocnt == 0), "ofull": int(ocnt == depth),
            "iempty": int(icnt == 0), "ifull": int(icnt == depth)}


async def counts(host, t):
    """Debug 0x26: {INQ_CNT, OUTQ_CNT} as {byte, byte}."""
    w = await host.read_debug(t, DBG_FIFO_COUNTS)
    return w >> 8, w & 0xFF


async def host_push(host, t, words):
    await host.write(SP_FIFO, FIFO_QUEUE + t, words)


async def host_pop(host, t, n=1):
    return await host.read(SP_FIFO, FIFO_QUEUE + t, n)


async def load_and_start(host, prog, thread, start):
    await host.load_program(prog, verify=False)
    await host.set_reset_pc(thread, start)
    await host.reset_thread(thread)
    await host.run(1 << thread)


def pad_edges(mon, start, bit=0):
    return mon.changes(mon.uo, bit, start)


async def run_here(host, instrs, thread, start):
    """run_snippet without CTRL.RESET, which would empty the thread's FIFOs:
    the thread is pointed at the code with a debug PC write instead."""
    words = list(instrs) + [asm("HALT")]
    await host.load_program({start + i: w for i, w in enumerate(words)},
                            verify=False)
    await host.write_debug(thread, DBG_PC, start)
    await host.run(1 << thread)
    await host.wait_halted(1 << thread)


# ------------------------------------------------------------------- tests
@cocotb.test()
async def test_host_push_thread_pop(dut):
    """Host pushes into INQ[t] (one multi-word transaction into the same
    FIFO); the thread POPs in order; status word and debug 0x26 track it."""
    host = LoomHost(dut)
    await host.start()
    depth = await fifo_depth(host)
    for t in range(4):
        assert await status(host, t) == expect_status(0, 0, depth)
        assert await counts(host, t) == (0, 0)

    t = 1
    words = [0x1234, 0xBEEF, 0x0001, 0xFFFF, 0x8000, 0x7FFF, 0x5555, 0xAAAA][:depth]
    await host_push(host, t, words)
    assert await status(host, t) == expect_status(depth, 0, depth)
    assert await counts(host, t) == (depth, 0)
    for other in (0, 2, 3):
        assert await status(host, other) == expect_status(0, 0, depth), \
            "a push must not touch another thread's FIFO"
    assert await host.badop() == 0

    pops = [asm("POP", rd=1 + i) for i in range(depth)]
    await run_here(host, pops, t, 0x40)
    for i, want in enumerate(words):
        assert await host.read_reg(t, 1 + i) == want, f"POP #{i}"
    assert await status(host, t) == expect_status(0, 0, depth)
    assert await counts(host, t) == (0, 0)

    # Pushing, popping part, and pushing again wraps the storage.
    await host_push(host, t, [0x0101, 0x0202])
    await run_here(host, [asm("POP", rd=1)], t, 0x40)
    assert await host.read_reg(t, 1) == 0x0101
    await host_push(host, t, [0x0303, 0x0404])
    assert await counts(host, t) == (3, 0)
    await run_here(host, [asm("POP", rd=2), asm("POP", rd=3), asm("POP", rd=4)],
                   t, 0x40)
    assert [await host.read_reg(t, r) for r in (2, 3, 4)] == [0x0202, 0x0303, 0x0404]
    assert await host.badop() == 0


@cocotb.test()
async def test_thread_push_host_pop(dut):
    """The thread PUSHes into OUTQ[t]; the host pops exactly the words it
    clocks out: a one-word read pops one, a word cut by CS_n pops nothing,
    and a pop from an empty OUTQ reads 0 and sets BADOP[14]."""
    host = LoomHost(dut)
    await host.start()
    depth = await fifo_depth(host)
    t = 2
    vals = [0x0011, 0x2200, 0x3333, 0xC0DE][:depth]
    prog = []
    for v in vals:
        prog += [asm("LDI", rd=1, imm=v & 0xFF), asm("LDIH", rd=1, imm=v >> 8),
                 asm("PUSH", ra=1)]
    await run_snippet(host, prog, thread=t, start=0x80)
    n = len(vals)
    assert await status(host, t) == expect_status(0, n, depth)
    assert await counts(host, t) == (0, n)

    # One word read: exactly one entry leaves (no prefetch pops the next).
    assert await host_pop(host, t, 1) == [vals[0]]
    assert await counts(host, t) == (0, n - 1)

    # A transaction cut in the middle of its second word: the first word is
    # popped, the half-sent one is not.
    cmd = bytes([SP_FIFO << 4, 0x00, t, 0x00])
    rx = await host.transfer(cmd + bytes(3))
    assert (rx[4] << 8) | rx[5] == vals[1]
    assert await counts(host, t) == (0, n - 2)

    # The rest in one multi-word read, plus one word too many: 0 + BADOP[14].
    rest = await host_pop(host, t, n - 2 + 1)
    assert rest[:-1] == vals[2:], rest
    assert rest[-1] == 0
    assert await host.badop() == BADOP14
    assert await status(host, t) == expect_status(0, 0, depth)
    await host.clear_badop(BADOP14)
    assert await host.badop() == 0

    # A pop of an empty OUTQ: 0, nothing removed, BADOP[14].
    assert await host_pop(host, t, 1) == [0]
    assert await host.badop() == BADOP14
    await ClockCycles(dut.clk, 2)


@cocotb.test()
async def test_pop_blocks_and_resumes(dut):
    """POP and WAITB 2 stall on an empty INQ (WAIT_ACTIVE, PC held, STEPS
    counts) and complete in the first X cycle at or after the host's push is
    visible; stepping a blocked POP is one slot."""
    host = LoomHost(dut)
    await host.start()
    await fifo_depth(host)
    mon = PadMonitor(dut).start()

    for name, instr, t, left in (("POP", asm("POP", rd=1), 0, 0),
                                 ("WAITB 2", asm("WAITB", cond=2), 1, 1)):
        base = 0x40 * t
        await host.write(SP_CTRL, CTRL_PIN_OUT, 0)
        await load_and_start(host, {base: instr, base + 1: asm("SETP", pin=OUT0, val=1),
                                    base + 2: asm("HALT")}, t, base)
        await ClockCycles(dut.clk, 200)
        assert await host.read_debug(t, DBG_WAIT_ACTIVE) == 1, name
        assert await host.read_debug(t, DBG_PC) == base, name
        steps0 = await host.read_debug(t, DBG_STEPS)
        await ClockCycles(dut.clk, 40)
        steps1 = await host.read_debug(t, DBG_STEPS)
        assert steps1 > steps0, f"{name}: STEPS must count stalled slots"

        start = mon.now
        await host_push(host, t, [0xA5A5])
        rises = mon.sck_rises(start)
        assert len(rises) == 40, f"{len(rises)} SCK rises in a one-word write"
        visible = PadMonitor.host_commit(rises[-1])
        await host.wait_halted(1 << t)
        edges = pad_edges(mon, start)
        assert edges and edges[0][1] == 1, f"{name}: OUT0 never rose"
        x = edges[0][0] - 6
        assert 0 <= x - visible <= 3, \
            f"{name}: completed in X cycle {x}, push visible from {visible}"
        if name == "POP":
            assert await host.read_reg(t, 1) == 0xA5A5
        assert await host.read_debug(t, DBG_WAIT_ACTIVE) == 0
        assert (await counts(host, t))[0] == left, f"{name}: INQ count after"

    # Single-stepping a blocked POP: one slot, PC held, WAIT_ACTIVE set.
    t = 2
    await host.load_program({0x80: asm("POP", rd=3), 0x81: asm("HALT")}, verify=False)
    await host.set_reset_pc(t, 0x80)
    await host.reset_thread(t)
    await host.write_debug(t, DBG_STEPS, 0)
    await host.step(t)
    assert await host.read_debug(t, DBG_PC) == 0x80
    assert await host.read_debug(t, DBG_WAIT_ACTIVE) == 1
    assert await host.read_debug(t, DBG_STEPS) == 1
    await host_push(host, t, [0x5A5A])
    await host.step(t)
    assert await host.read_debug(t, DBG_PC) == 0x81
    assert await host.read_debug(t, DBG_WAIT_ACTIVE) == 0
    assert await host.read_debug(t, DBG_STEPS) == 2
    assert await host.read_reg(t, 3) == 0x5A5A
    mon.stop()
    await ClockCycles(dut.clk, 2)


@cocotb.test()
async def test_push_blocks_and_resumes(dut):
    """PUSH and WAITB 1 stall on a full OUTQ and complete in the first X
    cycle at or after the host's pop is visible; nothing is lost."""
    host = LoomHost(dut)
    await host.start()
    depth = await fifo_depth(host)
    mon = PadMonitor(dut).start()

    for name, t in (("PUSH", 0), ("WAITB 1", 3)):
        base = 0x40 * t
        prog = {}
        a = base
        for v in range(1, depth + 1):
            prog[a] = asm("LDI", rd=1, imm=v)
            prog[a + 1] = asm("PUSH", ra=1)
            a += 2
        prog[a] = asm("LDI", rd=1, imm=depth + 1)
        prog[a + 1] = asm("PUSH", ra=1) if name == "PUSH" else asm("WAITB", cond=1)
        prog[a + 2] = asm("SETP", pin=OUT0, val=1)
        prog[a + 3] = asm("HALT")
        await host.write(SP_CTRL, CTRL_PIN_OUT, 0)
        await load_and_start(host, prog, t, base)
        await ClockCycles(dut.clk, 200)
        assert await host.read_debug(t, DBG_PC) == a + 1, f"{name} did not block"
        assert await host.read_debug(t, DBG_WAIT_ACTIVE) == 1
        assert await counts(host, t) == (0, depth)

        start = mon.now
        assert await host_pop(host, t, 1) == [1]
        rises = mon.sck_rises(start)
        assert len(rises) == 48, f"{len(rises)} SCK rises in a one-word read"
        visible = PadMonitor.host_commit(rises[-1])
        await host.wait_halted(1 << t)
        edges = pad_edges(mon, start)
        assert edges and edges[0][1] == 1, f"{name}: OUT0 never rose"
        x = edges[0][0] - 6
        assert 0 <= x - visible <= 3, \
            f"{name}: completed in X cycle {x}, pop visible from {visible}"
        if name == "PUSH":
            assert await host_pop(host, t, depth) == list(range(2, depth + 2))
        else:
            assert await host_pop(host, t, depth - 1) == list(range(2, depth + 1))
        assert await counts(host, t) == (0, 0)
        assert await host.badop() == 0
    mon.stop()
    await ClockCycles(dut.clk, 2)


def waitb3_model(period, loops):
    """Pad edges of the WAITB 3 loop in test_waitb_tick_seen, from SEMANTICS
    4 and 6.4, in cycles relative to the X cycle (0) of its CSRW TICK_INT.

    The CSRW commits at edge 2, which clears ACC and eats that edge's tick,
    so the ticks are at edges 2 + k * period. TICK_SEEN is set by a tick; at
    the commit edge (X + 2) of every slot of the thread it keeps only what
    that slot did not see in its X cycle (TICK_SEEN &= ~seen), the tick
    winning at the same edge. The run starts with TICK_INT 1, so the CSRW saw
    TICK_SEEN = 1 and leaves it 0 after edge 2.
    """
    ticks = set(2 + k * period for k in range(1, 8 * loops + 8))
    prog = ["CSRW", "WAITB", "SETP1", "WAITB", "SETP0", "DJNZ", "HALT"]
    commit_seen = {2: 1}          # commit edge -> TICK_SEEN as that slot saw it
    seen, edge, pc, x, left, edges = 0, 2, 1, 4, loops, []
    while prog[pc] != "HALT":
        for e in range(edge + 1, x + 1):          # TICK_SEEN during cycle x
            if e in ticks:
                seen = 1
            elif e in commit_seen:
                seen &= ~commit_seen[e] & 1
        edge = x
        ins, nxt = prog[pc], pc + 1
        commit_seen[x + 2] = seen
        if ins == "WAITB" and not seen:
            nxt = pc
        elif ins == "SETP1":
            edges.append((x + 2, 1))
        elif ins == "SETP0":
            edges.append((x + 2, 0))
        elif ins == "DJNZ":
            left -= 1
            nxt = 1 if left else 6
        pc, x = nxt, x + 4
    return edges


@cocotb.test()
async def test_waitb_tick_seen(dut):
    """WAITB 3 waits for TICK_SEEN (SEMANTICS 4, 6.4). With a period that is
    a multiple of 4 clocks the edges are exactly one period apart; with 41
    the pattern matches the rule edge for edge, and no tick is lost: a tick
    that lands between a slot's X cycle and its commit survives the commit
    (the rule was fixed on 2026-09-18; before, WAITB 3 at period 41 showed
    gaps of 84). Also WAITB with T and a past deadline, and debug 0x24."""
    host = LoomHost(dut)
    await host.start()
    await fifo_depth(host)
    mon = PadMonitor(dut).start()
    loops = 10
    for period in (40, 41):
        want = waitb3_model(period, loops)
        prog = {0x00: asm("CSRW", csr=CSR_TICK_INT, ra=5),
                0x01: asm("WAITB", cond=3),
                0x02: asm("SETP", pin=OUT0, val=1),
                0x03: asm("WAITB", cond=3),
                0x04: asm("SETP", pin=OUT0, val=0),
                0x05: asm("DJNZ", rd=3, rel=-5),
                0x06: asm("HALT")}
        await host.write(SP_CTRL, CTRL_PIN_OUT, 0)
        await host.write_csr(0, CSR_TICK_INT, 1)     # known TICK_SEEN at the start
        await host.load_program(prog, verify=False)
        await host.set_reset_pc(0, 0)
        await host.reset_thread(0)
        await host.write_reg(0, 5, period)
        await host.write_reg(0, 3, loops)
        start = mon.now
        await host.run(0b0001)
        await host.wait_halted(0b0001, timeout_slots=20000)
        got = pad_edges(mon, start)
        assert len(got) == len(want) == 2 * loops, (got, want)
        gaps = [b[0] - a[0] for a, b in zip(got, got[1:])]
        want_gaps = [b[0] - a[0] for a, b in zip(want, want[1:])]
        assert gaps == want_gaps, f"period {period}: gaps {gaps}, spec {want_gaps}"
        dut._log.info("WAITB 3, period %d: gaps %s" % (period, gaps))
        if period == 40:
            assert all(g == 40 for g in gaps), gaps
        assert max(gaps) <= period + 4, f"period {period}: a tick was lost: {gaps}"

    # WAITB with T: a condition that holds wins (T <- 0); otherwise a
    # deadline already reached ends it with T <- 1. INQ[0] is empty here.
    await run_snippet(host, [asm("SETD", imm=0), asm("WAITB", cond=2, tmo=1)],
                      thread=0, start=0x10)
    assert await host.read_debug(0, DBG_FLAGS) & FLAG_T, "timeout must set T"
    await host_push(host, 0, [7])
    await run_here(host, [asm("SETD", imm=0), asm("WAITB", cond=2, tmo=1)], 0, 0x10)
    assert not await host.read_debug(0, DBG_FLAGS) & FLAG_T, "condition clears T"
    assert await counts(host, 0) == (1, 0), "WAITB does not pop"

    # Debug 0x24 reads TICK_SEEN; it is set by the tick generator, which
    # runs while the thread is halted (TICK_INT = 1: a tick every clock).
    await host.write_csr(0, CSR_TICK_INT, 1)
    assert await host.read_debug(0, DBG_TICK_SEEN) == 1
    mon.stop()
    await ClockCycles(dut.clk, 2)


@cocotb.test()
async def test_badop14(dut):
    """A push into a full INQ is dropped and sets BADOP[14]; so does a pop
    of an empty OUTQ (which reads 0). Write 1 to clear."""
    host = LoomHost(dut)
    await host.start()
    depth = await fifo_depth(host)
    t = 3
    await host_push(host, t, list(range(100, 100 + depth)))
    assert await host.badop() == 0
    await host_push(host, t, [0xDEAD])
    assert await host.badop() == BADOP14
    assert await counts(host, t) == (depth, 0)
    pops = [asm("POP", rd=1 + i) for i in range(depth)]
    await run_here(host, pops, t, 0xC0)
    got = [await host.read_reg(t, 1 + i) for i in range(depth)]
    assert got == list(range(100, 100 + depth)), f"the dropped word got in: {got}"
    await host.clear_badop(BADOP14)
    assert await host.badop() == 0
    assert await host_pop(host, t, 1) == [0]
    assert await host.badop() == BADOP14
    await host.write(SP_CTRL, CTRL_BADOP, 0x0001)      # other bits: no effect
    assert await host.badop() == BADOP14
    await host.clear_badop(BADOP14)
    assert await host.badop() == 0
    # Unmapped FIFO-space addresses read 0 and do nothing.
    assert await host.read1(SP_FIFO, 0x0200) == 0
    await host.write(SP_FIFO, 0x0004, 0x1111)
    assert await counts(host, 0) == (0, 0)
    assert await host.badop() == 0


@cocotb.test()
async def test_ctrl_reset_empties_fifos(dut):
    """CTRL.RESET of thread t empties INQ[t] and OUTQ[t], and only those."""
    host = LoomHost(dut)
    await host.start()
    depth = await fifo_depth(host)
    t, other = 3, 2
    await host_push(host, t, [1, 2, 3])
    await host_push(host, other, [9])
    await run_here(host, [asm("LDI", rd=1, imm=0x42), asm("PUSH", ra=1),
                          asm("PUSH", ra=1)], t, 0xC0)
    assert await counts(host, t) == (3, 2)
    await host.reset_thread(t)
    assert await counts(host, t) == (0, 0)
    assert await status(host, t) == expect_status(0, 0, depth)
    assert await counts(host, other) == (1, 0), "another thread's FIFO changed"
    # The emptied FIFOs work normally afterwards.
    await host_push(host, t, [0x0A0A, 0x0B0B])
    await run_here(host, [asm("POP", rd=1), asm("POP", rd=2), asm("PUSH", ra=2)],
                   t, 0xC0)
    assert [await host.read_reg(t, 1), await host.read_reg(t, 2)] == [0x0A0A, 0x0B0B]
    assert await host_pop(host, t, 1) == [0x0B0B]
    assert await host.badop() == 0


SLOTS = 31                     # thread loop period: 124 clocks, 4 less than a word


def stream_program(pop):
    """Thread 0: after a trigger on IN0, one FIFO operation every SLOTS slots,
    immediately followed by a rising edge on OUT0, so the operation's commit
    edge is that pad edge minus 4. PUSH sends 1, 2, 3, ...; POP adds every
    popped word into r1. r2 is the iteration count."""
    if pop:
        words = [asm("WAITP", pin=8, val=1)]
    else:
        words = [asm("LDI", rd=1, imm=0), asm("WAITP", pin=8, val=1)]
    head = len(words)
    if pop:
        body = [asm("POP", rd=4), asm("SETP", pin=OUT0, val=1),
                asm("SETP", pin=OUT0, val=0), asm("ADD", rd=1, ra=1, rb=4)]
    else:
        body = [asm("ADDI", rd=1, imm=1), asm("PUSH", ra=1),
                asm("SETP", pin=OUT0, val=1), asm("SETP", pin=OUT0, val=0)]
    body += [asm("NOP")] * (SLOTS - len(body) - 1)
    body.append(asm("DJNZ", rd=2, rel=-SLOTS))
    words += body + [asm("HALT")]
    return {a: w for a, w in enumerate(words)}, head


@cocotb.test()
async def test_same_edge_push_and_pop(dut):
    """A thread PUSH (POP) and a host pop (push) of the same FIFO committing
    at the same edge both apply: count + pushes - pops (SEMANTICS 6.7). The
    thread works every 124 clocks, the host every 128 (one word), so their
    commit edges slide past each other by 4 clocks per word; over 34 words
    and four start phases of the transfer they are bound to meet. The test
    checks from the pads that they did meet, that the thread never stalled,
    and that every word arrived exactly once."""
    host = LoomHost(dut)
    await host.start()
    await fifo_depth(host)
    mon = PadMonitor(dut).start()
    words = 34
    met = {"OUTQ": 0, "INQ": 0}
    for which in ("OUTQ", "INQ"):
        pop = which == "INQ"
        for phase in range(4):
            await host.halt()
            await host.reset_thread(0)                  # also empties the FIFOs
            await host.write(SP_CTRL, CTRL_PIN_OUT, 0)
            host.set_in(8, 0)
            prog, _ = stream_program(pop)
            await host.load_program(prog, verify=False)
            await host.set_reset_pc(0, 0)
            await host.reset_thread(0)
            await host.write_reg(0, 1, 0)
            if pop:
                prefill = [1, 2, 3]
                await host_push(host, 0, prefill)
                await host.write_reg(0, 2, len(prefill) + words)
            else:
                await host.write_reg(0, 2, 60)
            await host.run(0b0001)
            await ClockCycles(dut.clk, 40)
            await mon.align(phase)
            start = mon.now
            sent = list(range(4, 4 + words))
            if pop:
                task = cocotb.start_soon(host_push(host, 0, sent))
                header = 3
            else:
                task = cocotb.start_soon(host_pop(host, 0, words))
                header = 4
            # Trigger the thread when the fourth byte starts: the first data
            # word of a push, the dummy byte of a pop (the first word to pop
            # is taken from OUTQ when the dummy byte ends, so it must be there).
            while len(mon.sck_rises(start)) < 8 * 3 + 1:
                await ClockCycles(dut.clk, 1)
            host.set_in(8, 1)
            got = await task
            rises = mon.sck_rises(start)
            host_edges = [PadMonitor.host_commit(rises[8 * (header + 2 * j + 2) - 1])
                          for j in range(words)]
            thread_edges = [c - 4 for c, v in pad_edges(mon, start) if v]
            during = [e for e in thread_edges if e <= host_edges[-1]]
            gaps = [b - a for a, b in zip(during, during[1:])]
            assert len(during) >= words - 2 and all(g == 4 * SLOTS for g in gaps),                 f"{which} phase {phase}: the thread stalled: {gaps}"
            met[which] += len(set(host_edges) & set(thread_edges))
            assert await host.badop() == 0, f"{which} phase {phase}: BADOP set"
            if pop:
                await host.wait_halted(0b0001)
                assert await counts(host, 0) == (0, 0)
                assert await host.read_reg(0, 1) == sum(prefill + sent) & 0xFFFF
                assert await host.read_reg(0, 4) == sent[-1]
            else:
                await host.halt()
                await ClockCycles(dut.clk, 8)
                rest = await host_pop(host, 0, (await counts(host, 0))[1])
                seq = got + rest
                assert seq == list(range(1, len(seq) + 1)), seq
                assert await host.badop() == 0
    dut._log.info("same-edge commits seen: %s" % met)
    assert met["OUTQ"] > 0, "no host pop met a thread PUSH at one edge"
    assert met["INQ"] > 0, "no host push met a thread POP at one edge"
    host.set_in(8, 0)
    mon.stop()
    await ClockCycles(dut.clk, 2)
