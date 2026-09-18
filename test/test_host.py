# SPDX-License-Identifier: Apache-2.0
"""Host port tests: CTRL identification, IMEM, run control, STEP, DEBUG.

Everything is driven through the pads by the SPI master in spi_host.py, so
these tests also run against the gate-level netlist.
"""

import cocotb
from cocotb.triggers import ClockCycles

from spi_host import (
    LoomHost, SP_CTRL, SP_IMEM, SP_DEBUG, SP_FIFO, SP_DMEM,
    CTRL_ID, CTRL_VERSION, CTRL_RUN, CTRL_HALTED, CTRL_CAPS, CTRL_BADOP,
    CTRL_RESET_PC0, CTRL_SFLAGS, CTRL_SFLAGS_CLR,
    DBG_PC, DBG_FLAGS, DBG_TD, DBG_NOW, DBG_RS0, DBG_STEPS, DBG_RS1_DEPTH,
    DBG_WAIT_ACTIVE, DBG_DT, DBG_CSR0, CSR_TICK_INT, CSR_OUTGRP,
)
from tools.loomisa import load

ISA = load()
IMEM_WORDS = 256
FIFO_DEPTH = 4
VERSION = 0x0001
#: What this build reports: FIFOs with depth 4 (log2 = 2).
EXPECT_CAPS = 0x8000 | 0x08 | 0x02


@cocotb.test()
async def test_id_version_caps(dut):
    """CTRL 0x00/0x01/0x19 identify the build."""
    host = LoomHost(dut)
    await host.start()
    assert await host.read1(SP_CTRL, CTRL_ID) == 0x4C4D
    assert await host.read1(SP_CTRL, CTRL_VERSION) == VERSION
    # CAPS (docs/SEMANTICS.md 5): [15:12] log2(IMEM_WORDS), [11:9] zero,
    # [8] BE auto mode, [7] deadline-latched SETP, [6] ROM, [5] DMEM,
    # [4] BE manual mode, [3] FIFO, [2:0] log2(FIFO_DEPTH).
    caps = await host.read1(SP_CTRL, CTRL_CAPS)
    assert caps >> 12 == IMEM_WORDS.bit_length() - 1, f"CAPS {caps:#06x}"
    assert (caps >> 9) & 0x7 == 0, "CAPS bits 11:9 are reserved"
    assert caps & 0x100 == 0, "no bit-engine auto mode before M3"
    assert caps & 0x40 == 0, "no boot ROM"
    assert caps & 0x20 == 0, "DMEM must report absent"
    assert caps & 0x08, "the M2 build has FIFOs"
    assert caps & 0x07 == FIFO_DEPTH.bit_length() - 1, "log2(FIFO_DEPTH)"
    assert caps == EXPECT_CAPS, f"256-word build must read {EXPECT_CAPS:#06x}, got {caps:#06x}"
    # The unbuilt data-memory space reads 0 and ignores writes.
    assert await host.read1(SP_DMEM, 0) == 0
    await host.write(SP_DMEM, 0, 0xBEEF)
    assert await host.read1(SP_DMEM, 0) == 0


@cocotb.test()
async def test_imem_write_readback(dut):
    """Multi-word IMEM writes with address auto-increment, then readback."""
    host = LoomHost(dut)
    await host.start()
    words = [0x0000, 0x1234, 0xFFFF, 0xAAAA, 0x5555]
    await host.write(SP_IMEM, 0x10, words)
    assert await host.read(SP_IMEM, 0x10, len(words)) == words
    # A single word somewhere else, and the neighbours are untouched.
    await host.write(SP_IMEM, 0x7F, 0xC0DE)
    assert await host.read1(SP_IMEM, 0x7F) == 0xC0DE
    assert await host.read(SP_IMEM, 0x10, 2) == words[:2]


@cocotb.test()
async def test_imem_refused_while_running(dut):
    """IMEM access while RUN != 0 is dropped, reads 0 and sets BADOP[15]."""
    host = LoomHost(dut)
    await host.start()
    # An endless loop at 0 so the thread never halts, plus a known word at
    # 0x20 so that the refused write has something to fail to change.
    await host.load_program({0: ISA.encode("JMP", abs=0), 0x20: 0xABCD})
    await host.set_reset_pc(0, 0)
    await host.reset_thread(0)
    assert await host.badop() == 0
    await host.run(0b0001)
    await ClockCycles(dut.clk, 40)

    await host.write(SP_IMEM, 0x20, 0x1111)
    assert await host.badop() & 0x8000, "BADOP[15] not set by a refused write"
    await host.clear_badop(0xFFFF)
    assert await host.badop() == 0

    assert await host.read1(SP_IMEM, 0) == 0, "read while running must return 0"
    assert await host.badop() & 0x8000, "BADOP[15] not set by a refused read"

    await host.halt()
    await ClockCycles(dut.clk, 40)
    await host.clear_badop(0xFFFF)
    assert await host.read1(SP_IMEM, 0x20) == 0xABCD, \
        "the write must have been dropped, not applied"
    assert await host.read1(SP_IMEM, 0) == ISA.encode("JMP", abs=0)
    assert await host.badop() == 0


@cocotb.test()
async def test_reset_pc_and_run_halted(dut):
    """RESET_PC is readable and writable; RUN starts a thread, HALT stops it."""
    host = LoomHost(dut)
    await host.start()
    # Defaults are t * (IMEM_WORDS / 4): 0, 64, 128, 192 for 256 words.
    for t in range(4):
        expect = t * (IMEM_WORDS // 4)
        assert await host.read1(SP_CTRL, CTRL_RESET_PC0 + t) == expect
    for t in range(4):
        await host.set_reset_pc(t, 0x40 + t)
    for t in range(4):
        assert await host.read1(SP_CTRL, CTRL_RESET_PC0 + t) == 0x40 + t

    await host.load_program({0x20: ISA.encode("NOP"),
                             0x21: ISA.encode("HALT")})
    await host.set_reset_pc(1, 0x20)
    await host.reset_thread(1)
    assert await host.read_debug(1, DBG_PC) == 0x20
    assert await host.read1(SP_CTRL, CTRL_RUN) == 0
    assert await host.halted() == 0

    await host.run(0b0010)
    await host.wait_halted(0b0010)
    assert await host.halted() == 0b0010
    assert await host.read1(SP_CTRL, CTRL_RUN) == 0, "HALT clears RUN[t]"
    assert await host.read_debug(1, DBG_PC) == 0x22

    # Writing the RUN bit again clears HALTED.
    await host.run(0b0010)
    await ClockCycles(dut.clk, 20)
    await host.halt()
    await ClockCycles(dut.clk, 20)
    assert await host.halted() == 0, "a 0->1 RUN write must clear HALTED"


@cocotb.test()
async def test_step_executes_one_slot(dut):
    """Each STEP retires exactly one slot: STEPS +1, PC +1."""
    host = LoomHost(dut)
    await host.start()
    prog = {0x30 + i: ISA.encode("NOP") for i in range(8)}
    prog[0x38] = ISA.encode("HALT")
    await host.load_program(prog)
    await host.set_reset_pc(2, 0x30)
    await host.reset_thread(2)
    assert await host.read_debug(2, DBG_STEPS) == 0
    for i in range(8):
        await host.step(2)
        await ClockCycles(dut.clk, 12)
        assert await host.read_debug(2, DBG_STEPS) == i + 1, f"after step {i}"
        assert await host.read_debug(2, DBG_PC) == 0x31 + i, f"after step {i}"
        assert await host.halted() == 0
    # The ninth step runs HALT.
    await host.step(2)
    await ClockCycles(dut.clk, 12)
    assert await host.halted() == 0b0100
    assert await host.read_debug(2, DBG_STEPS) == 9

    # STEP is ignored while the thread runs.
    await host.load_program({0x40: ISA.encode("JMP", abs=0x40)})
    await host.set_reset_pc(2, 0x40)
    await host.reset_thread(2)
    await host.run(0b0100)
    await ClockCycles(dut.clk, 20)
    before = await host.read_debug(2, DBG_STEPS)
    await host.step(2)
    await host.halt()
    await ClockCycles(dut.clk, 40)
    after = await host.read_debug(2, DBG_STEPS)
    assert after >= before                       # free running kept going
    await host.step(2)                           # now it is halted: one slot
    await ClockCycles(dut.clk, 12)
    assert await host.read_debug(2, DBG_STEPS) == after + 1


@cocotb.test()
async def test_debug_registers(dut):
    """Every debug register is readable, and writable while halted."""
    host = LoomHost(dut)
    await host.start()
    t = 3
    pairs = [
        (DBG_PC, 0x123, 0x123),
        (DBG_FLAGS, 0x7, 0x7),
        (DBG_TD, 0xBEEF, 0xBEEF),
        (DBG_RS0, 0x2AA, 0x2AA),
        (DBG_STEPS, 0x1234, 0x1234),
        (DBG_RS1_DEPTH, (2 << 10) | 0x155, (2 << 10) | 0x155),
        (DBG_WAIT_ACTIVE, 1, 1),
        (DBG_DT, 0xF00D, 0xF00D),
        (DBG_CSR0 + CSR_TICK_INT, 434, 434),
        (DBG_CSR0 + CSR_OUTGRP, 0x2F0, 0x2F0),
    ]
    for reg, write, expect in pairs:
        await host.write_debug(t, reg, write)
        got = await host.read_debug(t, reg)
        assert got == expect, f"debug reg {reg:#04x}: {got:#06x} != {expect:#06x}"
    # TID is read only and reports the thread number.
    assert await host.read_csr(t, 0x0C) == t
    # NOW is read only.
    now = await host.read_debug(t, DBG_NOW)
    await host.write_debug(t, DBG_NOW, 0)
    assert await host.read_debug(t, DBG_NOW) >= now
    # r0..r7 round trip while halted.
    for n in range(8):
        await host.write_reg(t, n, 0x1000 + n)
    for n in range(8):
        assert await host.read_reg(t, n) == 0x1000 + n


@cocotb.test()
async def test_regs_read_zero_while_running(dut):
    """r0..r7 read 0 and ignore writes while the thread runs (SEMANTICS 7)."""
    host = LoomHost(dut)
    await host.start()
    await host.load_program({0x50: ISA.encode("JMP", abs=0x50)})
    await host.set_reset_pc(0, 0x50)
    await host.reset_thread(0)
    for n in range(8):
        await host.write_reg(0, n, 0x7000 + n)
    await host.run(0b0001)
    await ClockCycles(dut.clk, 40)
    for n in range(8):
        assert await host.read_reg(0, n) == 0, f"r{n} must read 0 while running"
    await host.write_reg(0, 3, 0xDEAD)           # dropped
    # PC is readable at any time and is inside the loop.
    assert await host.read_debug(0, DBG_PC) == 0x50
    await host.halt()
    await ClockCycles(dut.clk, 40)
    for n in range(8):
        assert await host.read_reg(0, n) == 0x7000 + n


@cocotb.test()
async def test_sflags_host_access(dut):
    """CTRL SFLAGS sets bits, SFLAGS_CLR clears them."""
    host = LoomHost(dut)
    await host.start()
    assert await host.read1(SP_CTRL, CTRL_SFLAGS) == 0
    await host.write(SP_CTRL, CTRL_SFLAGS, 0b1010_0101)
    assert await host.read1(SP_CTRL, CTRL_SFLAGS) == 0b1010_0101
    await host.write(SP_CTRL, CTRL_SFLAGS, 0b0000_0010)
    assert await host.read1(SP_CTRL, CTRL_SFLAGS) == 0b1010_0111, "writes set"
    await host.write(SP_CTRL, CTRL_SFLAGS_CLR, 0b0000_0111)
    assert await host.read1(SP_CTRL, CTRL_SFLAGS) == 0b1010_0000


@cocotb.test()
async def test_sck_at_the_limit(dut):
    """The protocol minimum SCK period of 8 core clocks still works."""
    host = LoomHost(dut, sck_clocks=8)
    await host.start()
    assert await host.read1(SP_CTRL, CTRL_ID) == 0x4C4D
    await host.write(SP_IMEM, 0, [0x1357, 0x2468])
    assert await host.read(SP_IMEM, 0, 2) == [0x1357, 0x2468]

    slow = LoomHost(dut, sck_clocks=20)
    slow.ui_ext = host.ui_ext
    assert await slow.read1(SP_CTRL, CTRL_ID) == 0x4C4D
    assert await slow.read(SP_IMEM, 0, 2) == [0x1357, 0x2468]
