# SPDX-License-Identifier: Apache-2.0
"""Host interrupt (docs/SEMANTICS.md 6.8, docs/HOST_PROTOCOL.md CTRL).

HOST_IRQ (uo_out[6]) is a register loaded at every edge with
any(IRQ_STAT & IRQ_EN) | any(IRQ_STAT2 & IRQ_EN2) | any(SWIRQ), computed from
the values visible in the cycle before the edge; IRQ_STAT = {SFLAGS[7:0],
INQ_NOT_FULL[3:0], OUTQ_NOT_EMPTY[3:0]}, IRQ_STAT2 = {12'b0, HALTED[3:0]}.
So a cause that becomes visible in cycle c shows on the pin from cycle c + 1
(one edge later), and the pin follows the level: the host clears the cause.
Timing is measured on the pads (PadMonitor): a thread's effect is visible at
the commit edge x + 2 of its slot, which a SETP in the slot before marks on
OUT0 at its own x + 2 = the next slot's x - 2; a host word's effect is
visible from PadMonitor.host_commit(<its last SCK rise>).
"""

import cocotb
from cocotb.triggers import ClockCycles

from spi_host import (
    LoomHost, PadMonitor, asm, SP_CTRL, SP_FIFO, CTRL_IRQ_EN, CTRL_IRQ_STAT,
    CTRL_IRQ_STAT2, CTRL_IRQ_EN2, CTRL_SWIRQ, CTRL_SFLAGS, CTRL_SFLAGS_CLR,
    CTRL_PIN_OUT, CTRL_RUN, FIFO_QUEUE, CSR_HOST_IRQ,
)

OUT0 = 16
IRQ_BIT = 6


def irq_level(host):
    return (int(host.dut.uo_out.value) >> IRQ_BIT) & 1


def irq_edges(mon, start):
    return mon.changes(mon.uo, IRQ_BIT, start)


def out0_edges(mon, start):
    return mon.changes(mon.uo, 0, start)


async def start_thread(host, prog, t, base):
    await host.load_program(prog, verify=False)
    await host.set_reset_pc(t, base)
    await host.reset_thread(t)
    await host.run(1 << t)


async def host_write_timed(host, mon, space, addr, value):
    """A one-word host write; returns the first cycle its effect is visible."""
    start = mon.now
    await host.write(space, addr, value)
    rises = mon.sck_rises(start)
    assert len(rises) == 40, len(rises)
    return PadMonitor.host_commit(rises[-1])


@cocotb.test()
async def test_irq_reset_and_masks(dut):
    """After reset the pin is low; IRQ_STAT shows the FIFO fields (INQ not
    full everywhere, OUTQ empty) and IRQ_STAT2 shows HALTED; IRQ_EN2 keeps
    its four bits; nothing reaches the pin while the masks are 0."""
    host = LoomHost(dut)
    await host.start()
    assert irq_level(host) == 0
    assert await host.read1(SP_CTRL, CTRL_IRQ_STAT) == 0x00F0
    assert await host.read1(SP_CTRL, CTRL_IRQ_STAT2) == 0
    assert await host.read1(SP_CTRL, CTRL_IRQ_EN) == 0
    assert await host.read1(SP_CTRL, CTRL_IRQ_EN2) == 0
    await host.write(SP_CTRL, CTRL_IRQ_EN2, 0xFFFF)
    assert await host.read1(SP_CTRL, CTRL_IRQ_EN2) == 0x000F
    await host.write(SP_CTRL, CTRL_IRQ_EN2, 0)
    await host.write(SP_CTRL, CTRL_SFLAGS, 0xFF)
    await ClockCycles(dut.clk, 4)
    assert irq_level(host) == 0, "IRQ_EN = 0 masks every IRQ_STAT bit"
    assert await host.read1(SP_CTRL, CTRL_IRQ_STAT) == 0xFFF0
    await host.write(SP_CTRL, CTRL_SFLAGS_CLR, 0xFF)
    await ClockCycles(dut.clk, 2)


@cocotb.test()
async def test_irq_sflags_and_timing(dut):
    """A SIG with its IRQ_EN bit set raises the pin exactly one edge after
    SFLAGS changes; the host clearing the flag drops it one edge after its
    write takes effect; the host setting it raises it the same way."""
    host = LoomHost(dut)
    await host.start()
    mon = PadMonitor(dut).start()
    n = 5
    await host.write(SP_CTRL, CTRL_IRQ_EN, 1 << (8 + n))
    assert irq_level(host) == 0
    await host.write(SP_CTRL, CTRL_PIN_OUT, 0)
    start = mon.now
    await start_thread(host, {0x00: asm("SETP", pin=OUT0, val=1),
                              0x01: asm("SIG", flag=n),
                              0x02: asm("HALT")}, 0, 0)
    await host.wait_halted(0b0001)
    mark = out0_edges(mon, start)[0][0]           # SETP's x + 2 = SIG's x - 2
    sflags_visible = mark + 4                     # SIG's commit edge x + 2
    edges = irq_edges(mon, start)
    assert edges and edges[0] == (sflags_visible + 1, 1), (mark, edges)
    await ClockCycles(dut.clk, 20)
    assert irq_level(host) == 1, "level-sensitive: stays while the cause does"

    start = mon.now
    visible = await host_write_timed(host, mon, SP_CTRL, CTRL_SFLAGS_CLR, 1 << n)
    assert irq_edges(mon, start) == [(visible + 1, 0)]
    start = mon.now
    visible = await host_write_timed(host, mon, SP_CTRL, CTRL_SFLAGS, 1 << n)
    assert irq_edges(mon, start) == [(visible + 1, 1)]
    # Masking the enable drops it the same way.
    start = mon.now
    visible = await host_write_timed(host, mon, SP_CTRL, CTRL_IRQ_EN, 0)
    assert irq_edges(mon, start) == [(visible + 1, 0)]
    await host.write(SP_CTRL, CTRL_SFLAGS_CLR, 0xFF)
    mon.stop()
    await ClockCycles(dut.clk, 2)


@cocotb.test()
async def test_irq_swirq(dut):
    """CSRW HOST_IRQ sets SWIRQ[t], which reaches the pin without any mask,
    one edge after its commit; the host clears it by writing 1."""
    host = LoomHost(dut)
    await host.start()
    mon = PadMonitor(dut).start()
    await host.write(SP_CTRL, CTRL_PIN_OUT, 0)
    t = 2
    start = mon.now
    await start_thread(host, {0x80: asm("SETP", pin=OUT0, val=1),
                              0x81: asm("CSRW", csr=CSR_HOST_IRQ, ra=0),
                              0x82: asm("HALT")}, t, 0x80)
    await host.wait_halted(1 << t)
    mark = out0_edges(mon, start)[0][0]
    assert irq_edges(mon, start) == [(mark + 4 + 1, 1)]
    assert await host.read1(SP_CTRL, CTRL_SWIRQ) == 1 << t
    start = mon.now
    visible = await host_write_timed(host, mon, SP_CTRL, CTRL_SWIRQ, 1 << t)
    assert irq_edges(mon, start) == [(visible + 1, 0)]
    assert await host.read1(SP_CTRL, CTRL_SWIRQ) == 0
    mon.stop()
    await ClockCycles(dut.clk, 2)


@cocotb.test()
async def test_irq_fifo_fields(dut):
    """IRQ_STAT[3:0] OUTQ_NOT_EMPTY and [7:4] INQ_NOT_FULL follow the FIFOs:
    a PUSH raises the OUTQ bit one edge after its commit, the host's pop
    drops it; filling INQ drops the INQ bit, a POP raises it again."""
    host = LoomHost(dut)
    await host.start()
    caps = await host.caps()
    depth = 1 << (caps & 7)
    mon = PadMonitor(dut).start()
    t = 1
    await host.write(SP_CTRL, CTRL_IRQ_EN, 1 << t)            # OUTQ_NOT_EMPTY[1]
    await host.write(SP_CTRL, CTRL_PIN_OUT, 0)
    start = mon.now
    await start_thread(host, {0x40: asm("LDI", rd=1, imm=0x5A),
                              0x41: asm("SETP", pin=OUT0, val=1),
                              0x42: asm("PUSH", ra=1),
                              0x43: asm("HALT")}, t, 0x40)
    await host.wait_halted(1 << t)
    mark = out0_edges(mon, start)[0][0]
    assert irq_edges(mon, start) == [(mark + 4 + 1, 1)]
    assert await host.read1(SP_CTRL, CTRL_IRQ_STAT) == 0x00F0 | (1 << t)
    start = mon.now
    assert await host.read1(SP_FIFO, FIFO_QUEUE + t) == 0x5A
    assert await host.badop() == 0
    rises = mon.sck_rises(start)
    visible = PadMonitor.host_commit(rises[47])
    assert irq_edges(mon, start) == [(visible + 1, 0)]

    # INQ_NOT_FULL[1]: set now; the pin rises with the enable.
    start = mon.now
    visible = await host_write_timed(host, mon, SP_CTRL, CTRL_IRQ_EN, 1 << (4 + t))
    assert irq_edges(mon, start) == [(visible + 1, 1)]
    await host.write(SP_FIFO, FIFO_QUEUE + t, list(range(depth - 1)))
    assert irq_level(host) == 1
    start = mon.now
    await host.write(SP_FIFO, FIFO_QUEUE + t, 0x77)           # now full
    rises = mon.sck_rises(start)
    visible = PadMonitor.host_commit(rises[-1])
    assert irq_edges(mon, start) == [(visible + 1, 0)]
    assert await host.read1(SP_CTRL, CTRL_IRQ_STAT) & (1 << (4 + t)) == 0
    await host.write_debug(t, 0x08, 0x50)                      # PC, no RESET
    await host.load_program({0x50: asm("SETP", pin=OUT0, val=0),
                             0x51: asm("POP", rd=1), 0x52: asm("HALT")}, verify=False)
    start = mon.now
    await host.run(1 << t)
    await host.wait_halted(1 << t)
    mark = out0_edges(mon, start)[0][0]
    assert irq_edges(mon, start) == [(mark + 4 + 1, 1)]
    await host.write(SP_CTRL, CTRL_IRQ_EN, 0)
    mon.stop()
    await ClockCycles(dut.clk, 2)


@cocotb.test()
async def test_irq_halted(dut):
    """IRQ_STAT2 = {12'b0, HALTED} masked by IRQ_EN2: a HALT raises the pin
    one edge after it commits; the RUN write that clears HALTED drops it."""
    host = LoomHost(dut)
    await host.start()
    mon = PadMonitor(dut).start()
    t = 3
    await host.write(SP_CTRL, CTRL_IRQ_EN2, 1 << t)
    await host.write(SP_CTRL, CTRL_PIN_OUT, 0)
    start = mon.now
    await start_thread(host, {0xC0: asm("SETP", pin=OUT0, val=1),
                              0xC1: asm("HALT")}, t, 0xC0)
    await host.wait_halted(1 << t)
    mark = out0_edges(mon, start)[0][0]
    assert irq_edges(mon, start) == [(mark + 4 + 1, 1)]
    assert await host.read1(SP_CTRL, CTRL_IRQ_STAT2) == 1 << t
    # Another thread's HALT is masked.
    await start_thread(host, {0x80: asm("HALT")}, 2, 0x80)
    await host.wait_halted(1 << 2)
    assert irq_level(host) == 1
    await host.write(SP_CTRL, CTRL_IRQ_EN2, 1 << 2)
    await ClockCycles(dut.clk, 2)
    assert irq_level(host) == 1                 # thread 2 is halted too
    await host.write(SP_CTRL, CTRL_IRQ_EN2, 1 << t)
    # RUN 0 -> 1 clears HALTED[t] (the pin falls); the thread then runs a
    # NOP and halts again (the pin rises).
    await host.load_program({0xC2: asm("NOP"), 0xC3: asm("HALT")}, verify=False)
    start = mon.now
    visible = await host_write_timed(host, mon, SP_CTRL, CTRL_RUN, 1 << t)
    await host.wait_halted(1 << t)
    edges = irq_edges(mon, start)
    assert edges[0] == (visible + 1, 0), edges
    assert len(edges) == 2 and edges[1][1] == 1, edges
    await host.write(SP_CTRL, CTRL_IRQ_EN2, 0)
    await ClockCycles(dut.clk, 2)
    assert irq_level(host) == 0
    mon.stop()
    await ClockCycles(dut.clk, 2)
