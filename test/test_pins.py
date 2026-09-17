# SPDX-License-Identifier: Apache-2.0
"""Pin unit: SETP, OEP, OUT, IN, open drain, and the pin-conditional waits
(docs/SEMANTICS.md 6.3 and 6.4). Everything is checked on the real pads."""

import cocotb
from cocotb.triggers import ClockCycles, Timer


async def tick(dut, n=1):
    """Advance n clock edges and settle, so reads see the new register values."""
    await ClockCycles(dut.clk, n)
    await Timer(1, unit="ns")

from spi_host import (
    LoomHost, asm, run_snippet, resolve, SP_CTRL,
    CTRL_OD_MASK, CTRL_PIN_OUT, CTRL_PIN_OE, CTRL_PIN_IN,
    DBG_FLAGS, DBG_WAIT_ACTIVE, CSR_OUTGRP, CSR_INGRP, CSR_PIN_IN,
    CSR_PIN_OUT, CSR_PIN_OE, CSR_OD_MASK,
)


def outs(dut):
    return resolve(dut.uo_out) & 0x3F          # OUT0..OUT5 = pin index 16..21


@cocotb.test()
async def test_setp_out_pins(dut):
    """SETP drives OUT0..OUT5 (pin index 16..21) on uo_out[5:0]."""
    host = LoomHost(dut)
    await host.start()
    assert outs(dut) == 0
    for i in range(6):
        await run_snippet(host, [asm("SETP", pin=16 + i, val=1)])
        assert outs(dut) == (1 << i), f"SETP {16+i} -> {outs(dut):#04x}"
        await run_snippet(host, [asm("SETP", pin=16 + i, val=0)])
        assert outs(dut) == 0
    # Writes are bit-masked: only the written pin changes.
    await run_snippet(host, [asm("SETP", pin=16, val=1),
                             asm("SETP", pin=18, val=1),
                             asm("SETP", pin=21, val=1)])
    assert outs(dut) == 0b100101
    await run_snippet(host, [asm("SETP", pin=18, val=0)])
    assert outs(dut) == 0b100001
    # The register view matches (PIN_OUT[13:8] = uo_out[5:0]).
    assert (await host.read1(SP_CTRL, CTRL_PIN_OUT)) >> 8 == 0b100001
    await host.write(SP_CTRL, CTRL_PIN_OUT, 0)


@cocotb.test()
async def test_setp_reserved_and_readonly(dut):
    """Writes to read-only and reserved pin indices are ignored."""
    host = LoomHost(dut)
    await host.start()
    await host.write(SP_CTRL, CTRL_PIN_OUT, 0)
    before = await host.read1(SP_CTRL, CTRL_PIN_OUT)
    for pin in (8, 11, 12, 13, 15, 22, 27, 31):
        await run_snippet(host, [asm("SETP", pin=pin, val=1)])
        assert await host.read1(SP_CTRL, CTRL_PIN_OUT) == before, \
            f"SETP on index {pin} must be ignored"
        assert resolve(dut.uio_out) == 0 and outs(dut) == 0
    # OEP only accepts 0..7.
    for pin in (8, 16, 20, 31):
        await run_snippet(host, [asm("OEP", pin=pin, val=1)])
        assert resolve(dut.uio_oe) == 0, f"OEP on index {pin} must be ignored"


@cocotb.test()
async def test_bidir_and_oep(dut):
    """BIDIR pins: SETP drives the value, OEP the enable."""
    host = LoomHost(dut)
    await host.start()
    await run_snippet(host, [asm("SETP", pin=0, val=1),
                             asm("SETP", pin=3, val=1)])
    assert resolve(dut.uio_oe) == 0, "no BIDIR pin is enabled yet"
    assert resolve(dut.uio_out) == 0b1001
    await run_snippet(host, [asm("OEP", pin=0, val=1),
                             asm("OEP", pin=3, val=1)])
    assert resolve(dut.uio_oe) == 0b1001
    assert resolve(dut.uio_out) == 0b1001
    # A driven BIDIR pin reads back through the synchroniser (pad loopback).
    await ClockCycles(dut.clk, 8)
    pin_in = await host.read1(SP_CTRL, CTRL_PIN_IN)
    assert pin_in & 0xFF == 0b1001, f"PIN_IN {pin_in:#06x}"
    await run_snippet(host, [asm("OEP", pin=0, val=0),
                             asm("OEP", pin=3, val=0)])
    assert resolve(dut.uio_oe) == 0
    await host.write(SP_CTRL, CTRL_PIN_OUT, 0)


@cocotb.test()
async def test_open_drain(dut):
    """OD_MASK: writing 1 releases the pin, writing 0 drives it low."""
    host = LoomHost(dut)
    await host.start()
    await host.write(SP_CTRL, CTRL_PIN_OUT, 0)
    await host.write(SP_CTRL, CTRL_PIN_OE, 0)
    await host.write(SP_CTRL, CTRL_OD_MASK, 0b0000_0011)   # BIDIR0, BIDIR1
    host.set_uio_ext(0xFF, pull=0xFF)                      # external pull-ups

    await run_snippet(host, [asm("SETP", pin=0, val=0)])   # drive low
    assert resolve(dut.uio_oe) & 1 == 1
    assert resolve(dut.uio_out) & 1 == 0
    await run_snippet(host, [asm("SETP", pin=0, val=1)])   # release
    assert resolve(dut.uio_oe) & 1 == 0
    assert resolve(dut.uio_out) & 1 == 0, "PIN_OUT stays 0 in open drain"
    await ClockCycles(dut.clk, 8)
    assert (await host.read1(SP_CTRL, CTRL_PIN_IN)) & 1 == 1, "pulled up"

    # A pin outside OD_MASK is unaffected by the rule.
    await run_snippet(host, [asm("OEP", pin=4, val=1), asm("SETP", pin=4, val=1)])
    assert resolve(dut.uio_out) & 0x10 == 0x10
    assert resolve(dut.uio_oe) & 0x10 == 0x10
    await host.write(SP_CTRL, CTRL_OD_MASK, 0)
    await host.write(SP_CTRL, CTRL_PIN_OE, 0)
    await host.write(SP_CTRL, CTRL_PIN_OUT, 0)
    host.set_uio_ext(0, pull=0)
    await ClockCycles(dut.clk, 4)


@cocotb.test()
async def test_out_and_in_groups(dut):
    """OUT and IN use OUTGRP/INGRP: base[4:0], cnt[9:5], modulo 32."""
    host = LoomHost(dut)
    await host.start()
    await host.write(SP_CTRL, CTRL_PIN_OUT, 0)
    # base 16, count 6: ra[5:0] -> OUT0..OUT5
    outgrp = (6 << 5) | 16
    for value in (0x00, 0x3F, 0x15, 0x2A, 0xFFC1):
        await run_snippet(host, [asm("CSRW", csr=CSR_OUTGRP, ra=1),
                                 asm("OUT", ra=2)], regs={1: outgrp, 2: value})
        assert outs(dut) == value & 0x3F, f"OUT {value:#06x} -> {outs(dut):#04x}"
    # count 0 writes nothing.
    await run_snippet(host, [asm("CSRW", csr=CSR_OUTGRP, ra=1), asm("OUT", ra=2)],
                      regs={1: (0 << 5) | 16, 2: 0xFFFF})
    assert outs(dut) == 0x01

    # A group that runs off the end of the writable range: base 20, count 4
    # covers 20, 21, 22, 23 and only the first two exist.
    await host.write(SP_CTRL, CTRL_PIN_OUT, 0)
    await run_snippet(host, [asm("CSRW", csr=CSR_OUTGRP, ra=1), asm("OUT", ra=2)],
                      regs={1: (4 << 5) | 20, 2: 0xF})
    assert outs(dut) == 0b110000

    # IN: base 8, count 5 reads IN0..IN4.
    await host.write(SP_CTRL, CTRL_PIN_OUT, 0)
    for value in (0b00000, 0b11111, 0b10101, 0b01010):
        for i in range(5):
            host.set_in(8 + i, (value >> i) & 1)
        await ClockCycles(dut.clk, 8)
        await run_snippet(host, [asm("CSRW", csr=CSR_INGRP, ra=1),
                                 asm("IN", rd=3)],
                          regs={1: (5 << 5) | 8, 3: 0xFFFF})
        assert await host.read_reg(0, 3) == value, f"IN {value:#07b}"
    for i in range(5):
        host.set_in(8 + i, 0)
    # Reading the OUT indices returns the driven value.
    await run_snippet(host, [asm("SETP", pin=16, val=1),
                             asm("SETP", pin=19, val=1),
                             asm("CSRW", csr=CSR_INGRP, ra=1),
                             asm("IN", rd=3)],
                      regs={1: (6 << 5) | 16, 3: 0})
    assert await host.read_reg(0, 3) == 0b001001
    await host.write(SP_CTRL, CTRL_PIN_OUT, 0)


@cocotb.test()
async def test_pin_csrs(dut):
    """CSRW PIN_OUT/PIN_OE/OD_MASK write the whole register; PIN_IN reads."""
    host = LoomHost(dut)
    await host.start()
    await run_snippet(host, [asm("CSRW", csr=CSR_PIN_OUT, ra=1),
                             asm("CSRW", csr=CSR_PIN_OE, ra=2),
                             asm("CSRW", csr=CSR_OD_MASK, ra=3),
                             asm("CSRR", rd=4, csr=CSR_PIN_OUT),
                             asm("CSRR", rd=5, csr=CSR_PIN_OE),
                             asm("CSRR", rd=6, csr=CSR_PIN_IN)],
                      regs={1: 0x1234, 2: 0x00F0, 3: 0x0000})
    assert await host.read_reg(0, 4) == 0x1234
    assert await host.read_reg(0, 5) == 0x00F0
    assert resolve(dut.uio_out) == 0x34
    assert resolve(dut.uio_oe) == 0xF0
    assert outs(dut) == 0x12
    await host.write(SP_CTRL, CTRL_PIN_OUT, 0)
    await host.write(SP_CTRL, CTRL_PIN_OE, 0)


@cocotb.test()
async def test_input_synchroniser_latency(dut):
    """Two flop synchroniser: the pad sampled at edge x-1 is what X sees.

    This is the one structural check in the suite; it skips itself when the
    hierarchy is not present (gate level).
    """
    host = LoomHost(dut)
    await host.start()
    try:
        sync = dut.user_project.u_loom.u_pins.pad_sync
        resolve(sync)
    except AttributeError:
        dut._log.info("no RTL hierarchy (gate level): skipping")
        return
    # pad_sync[12:0] = {IN4, IN3..IN0, BIDIR7..0}, so pin index 8 is bit 8.
    def in0():
        return (resolve(sync) >> 8) & 1

    host.set_in(8, 0)
    await tick(dut, 6)
    assert in0() == 0
    host.set_in(8, 1)
    await tick(dut, 1)
    assert in0() == 0, "the first edge only loads FF1"
    await tick(dut, 1)
    assert in0() == 1, "visible to logic after the second edge"
    host.set_in(8, 0)
    await tick(dut, 1)
    assert in0() == 1
    await tick(dut, 1)
    assert in0() == 0


@cocotb.test()
async def test_waitp(dut):
    """WAITP stalls until the pin has the requested level."""
    host = LoomHost(dut)
    await host.start()
    host.set_in(8, 0)
    await host.load_program({0x00: asm("WAITP", pin=8, val=1),
                             0x01: asm("SETP", pin=16, val=1),
                             0x02: asm("HALT")}, verify=False)
    await host.set_reset_pc(0, 0)
    await host.reset_thread(0)
    await host.write(SP_CTRL, CTRL_PIN_OUT, 0)
    await host.run(0b0001)
    await ClockCycles(dut.clk, 60)
    assert await host.halted() == 0
    assert await host.read_debug(0, DBG_WAIT_ACTIVE) == 1
    assert outs(dut) == 0
    host.set_in(8, 1)
    await host.wait_halted(0b0001)
    assert outs(dut) == 1
    host.set_in(8, 0)
    await host.write(SP_CTRL, CTRL_PIN_OUT, 0)


@cocotb.test()
async def test_waite_edges(dut):
    """WAITE rise, fall and any, against real pad transitions."""
    host = LoomHost(dut)
    await host.start()
    for edge, start_level, move_to in ((0, 0, 1), (1, 1, 0), (2, 0, 1), (2, 1, 0)):
        host.set_in(9, start_level)                # IN1 = pin index 9
        await ClockCycles(dut.clk, 10)
        await host.load_program({0x00: asm("WAITE", pin=9, edge=edge),
                                 0x01: asm("ADDI", rd=1, imm=1),
                                 0x02: asm("HALT")}, verify=False)
        await host.set_reset_pc(0, 0)
        await host.reset_thread(0)
        await host.write_reg(0, 1, 0)
        await host.run(0b0001)
        await ClockCycles(dut.clk, 80)
        assert await host.halted() == 0, f"edge {edge}: fired without a change"
        host.set_in(9, move_to)
        await host.wait_halted(0b0001)
        assert await host.read_reg(0, 1) == 1
    # The wrong edge never fires: wait for a fall while the pin only rises.
    host.set_in(9, 0)
    await ClockCycles(dut.clk, 10)
    await host.load_program({0x00: asm("WAITE", pin=9, edge=1),
                             0x01: asm("HALT")}, verify=False)
    await host.reset_thread(0)
    await host.run(0b0001)
    host.set_in(9, 1)
    await ClockCycles(dut.clk, 200)
    assert await host.halted() == 0, "a rise must not satisfy WAITE fall"
    host.set_in(9, 0)
    await host.wait_halted(0b0001)
    host.set_in(9, 0)
    await ClockCycles(dut.clk, 4)


@cocotb.test()
async def test_timed_wait_t_flag(dut):
    """A wait with the T bit ends at the deadline and sets T, else clears it."""
    host = LoomHost(dut)
    await host.start()
    host.set_in(10, 0)                              # IN2 = pin index 10
    await ClockCycles(dut.clk, 8)
    # The pin never arrives: the deadline ends the wait and sets T.
    await run_snippet(host, [asm("CSRW", csr=CSR_OUTGRP, ra=0),   # harmless
                             asm("SETD", imm=20),
                             asm("WAITP", pin=10, val=1, tmo=1),
                             asm("CSRR", rd=2, csr=0x0B)],
                      regs={0: 0, 2: 0})
    assert await host.read_reg(0, 2) & 0x4, "T must be set on a timeout"
    assert await host.read_debug(0, DBG_FLAGS) & 0x4

    # The pin is already there: the wait completes and clears T.
    host.set_in(10, 1)
    await ClockCycles(dut.clk, 8)
    await run_snippet(host, [asm("CSRW", csr=0x0B, ra=1),
                             asm("SETD", imm=200),
                             asm("WAITP", pin=10, val=1, tmo=1),
                             asm("CSRR", rd=2, csr=0x0B)],
                      regs={1: 0x7, 2: 0})
    assert await host.read_reg(0, 2) & 0x4 == 0, "T must be cleared on success"

    # Without the T bit the flag is untouched, even past the deadline.
    await run_snippet(host, [asm("CSRW", csr=0x0B, ra=1),
                             asm("SETD", imm=1),
                             asm("WAITP", pin=10, val=1, tmo=0),
                             asm("CSRR", rd=2, csr=0x0B)],
                      regs={1: 0x4, 2: 0})
    assert await host.read_reg(0, 2) & 0x4 == 0x4, "T must be left alone"
    host.set_in(10, 0)
    await ClockCycles(dut.clk, 4)
