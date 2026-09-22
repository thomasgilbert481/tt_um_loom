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
IMEM_WORDS = 512    # the SRAM macro (D-020); test_flops.py covers the 256-word flops
FIFO_DEPTH = 4
VERSION = 0x0004    # M3 slice B: LD/ST on the instruction memory (6.11); slice A was 3
#: What this build reports: log2(IMEM_WORDS) = 9 in [15:12], the slice-A
#: encoders [9], data memory [5] (slice B), FIFOs with depth 4 (log2 = 2),
#: the bit engine in manual mode and the deadline-latched SETP.
EXPECT_CAPS = 0x9000 | 0x200 | 0x80 | 0x20 | 0x10 | 0x08 | 0x02


@cocotb.test()
async def test_id_version_caps(dut):
    """CTRL 0x00/0x01/0x19 identify the build."""
    host = LoomHost(dut)
    await host.start()
    assert await host.read1(SP_CTRL, CTRL_ID) == 0x4C4D
    assert await host.read1(SP_CTRL, CTRL_VERSION) == VERSION
    # CAPS (docs/SEMANTICS.md 5): [15:12] log2(IMEM_WORDS), [11:10] zero,
    # [9] slice-A encoders, [8] BE auto mode, [7] deadline-latched SETP,
    # [6] ROM, [5] DMEM, [4] BE manual mode, [3] FIFO, [2:0] log2(FIFO_DEPTH).
    caps = await host.read1(SP_CTRL, CTRL_CAPS)
    assert caps >> 12 == IMEM_WORDS.bit_length() - 1, f"CAPS {caps:#06x}"
    assert (caps >> 10) & 0x3 == 0, "CAPS bits 11:10 are reserved"
    assert caps & 0x200, "slice A (encoders, stuffing, DIFF) must report present"
    assert caps & 0x100 == 0, "no bit-engine auto mode before M3"
    assert caps & 0x40 == 0, "no boot ROM"
    assert caps & 0x20, "slice B (LD/ST on the instruction memory) must report present"
    assert caps & 0x08, "the M2 build has FIFOs"
    assert caps & 0x07 == FIFO_DEPTH.bit_length() - 1, "log2(FIFO_DEPTH)"
    assert caps == EXPECT_CAPS, \
        f"{IMEM_WORDS}-word build must read {EXPECT_CAPS:#06x}, got {caps:#06x}"
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
    # Every address bit selects its own word: address 0 and the walking-one
    # addresses 1, 2, 4 .. IMEM_WORDS/2 get distinct values, so an address bit
    # that is missing or stuck on the way to the array (the macro's A_ADDR[8]
    # is new with 512 words) would make two of them alias.
    addrs = [0] + [1 << b for b in range(IMEM_WORDS.bit_length() - 1)]
    for i, a in enumerate(addrs):
        await host.write(SP_IMEM, a, 0x5A00 + i)
    for i, a in enumerate(addrs):
        got = await host.read1(SP_IMEM, a)
        assert got == 0x5A00 + i, f"IMEM[{a:#05x}] = {got:#06x}, aliased?"
    # The last word (every address bit set) is a word of its own too.
    await host.write(SP_IMEM, IMEM_WORDS - 1, 0x1DEA)
    assert await host.read1(SP_IMEM, IMEM_WORDS - 1) == 0x1DEA
    assert await host.read1(SP_IMEM, IMEM_WORDS // 2) == 0x5A00 + len(addrs) - 1


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
    # Defaults are t * (IMEM_WORDS / 4): 0, 128, 256, 384 for 512 words.
    for t in range(4):
        expect = t * (IMEM_WORDS // 4)
        assert await host.read1(SP_CTRL, CTRL_RESET_PC0 + t) == expect
    for t in range(4):
        await host.set_reset_pc(t, 0x40 + t)
    for t in range(4):
        assert await host.read1(SP_CTRL, CTRL_RESET_PC0 + t) == 0x40 + t

    # 0x22 is where the second RUN resumes: a self-loop, so the thread keeps
    # running until the host halts it, whatever an earlier test left in IMEM
    # (instruction memory is not reset, and a stray HALT reached from 0x22
    # would set HALTED again).
    await host.load_program({0x20: ISA.encode("NOP"),
                             0x21: ISA.encode("HALT"),
                             0x22: ISA.encode("JMP", abs=0x22)})
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



# ------------------------------------------------- the debug register file
DBG_LAST = 0x26                 # the highest debug register this build has

#: Every debug register of a thread: the storage cell behind it, the register
#: ids that reach that cell (TD answers both to 0x0A and to the 0x1A alias),
#: the bits that read back, and whether a host debug write must leave it
#: alone. Ordered by id, and covering 0x00..0x26 with no gaps.
DBG_CELLS = [("r%d" % n, (n,), 0xFFFF, False) for n in range(8)] + [
    # cell           ids            mask    read only
    ("PC",          (0x08,),        0x03FF, False),
    ("FLAGS",       (0x09, 0x1B),   0x0007, False),
    ("TD",          (0x0A, 0x1A),   0xFFFF, False),
    ("NOW",         (0x0B, 0x19),   0xFFFF, True),
    ("SR",          (0x0C, 0x1D),   0xFFFF, False),
    ("CNT",         (0x0D, 0x1E),   0x001F, False),
    ("CRC",         (0x0E, 0x1F),   0xFFFF, False),
    ("RS0",         (0x0F,),        0x03FF, False),
    ("TICK_INT",    (0x10,),        0xFFFF, False),
    ("TICK_FRAC",   (0x11,),        0x00FF, False),
    ("OUTGRP",      (0x12,),        0x03FF, False),
    ("INGRP",       (0x13,),        0x03FF, False),
    ("BE_CFG",      (0x14,),        0x0282, False),
    ("BE_PINS",     (0x15,),        0x03FF, False),
    ("BE_RELOAD",   (0x16,),        0x001F, False),
    ("CRC_POLY",    (0x17,),        0xFFFF, False),
    ("CRC_INIT",    (0x18,),        0xFFFF, False),
    ("TID",         (0x1C,),        0xFFFF, True),
    ("STEPS",       (0x20,),        0xFFFF, False),
    ("RS1_DEPTH",   (0x21,),        0x0FFF, False),
    ("WAIT_ACTIVE", (0x22,),        0x0001, False),
    ("DT",          (0x23,),        0xFFFF, False),
    ("TICK_SEEN",   (0x24,),        0x0001, False),
    ("LATCH",       (0x25,),        0x007F, False),
    ("FIFO_COUNTS", (0x26,),        0xFFFF, True),
]
CELL_OF = {reg: name for name, ids, _m, _ro in DBG_CELLS for reg in ids}
MASK_OF = {name: mask for name, _i, mask, _ro in DBG_CELLS}
RO_CELLS = {name for name, _i, _m, ro in DBG_CELLS if ro}

#: FLAGS takes Z, C and T from three separate bits of one written word, so a
#: value with all three equal cannot tell the three flops apart. These two
#: are complements of each other, so whichever the register already holds,
#: the other one moves it.
DBG_FORCE = {0x09: 0b010, 0x1B: 0b101}

#: The tick period is max(TICK_INT,1) * 256 + TICK_FRAC clocks, and a
#: TICK_INT write clears the accumulator, so parking TICK_INT at the top of
#: the range buys 65 535 clocks in which NOW and TICK_SEEN hold still. Every
#: pass below is parked at its start and is far shorter than that.
TICK_PARK = 0xFFFF

#: Register ids per pass. Within a pass every write goes to one thread, so
#: nothing touches the other three and a value that leaked into one of them
#: is still there when the pass ends and they are read. Reading all four
#: threads after every single write instead costs three times the simulation
#: time and finds the same faults, one write earlier.
DBG_PASS = 5


def dbg_value(reg, thread):
    """A distinctive value for debug register `reg` of `thread`.

    Different for every (register, thread) pair, so a write that lands in the
    wrong register or in the wrong thread, and a read mux that slices the
    wrong thread out of a packed vector, all leave a value somewhere it does
    not belong instead of one that happens to match.
    """
    return ((reg + 1) * 0x0111 + (thread + 1) * 0x1249 + 0x0A35) & 0xFFFF


async def dbg_read_thread(host, thread):
    """Every debug register of one thread, in one auto-incrementing read."""
    return await host.read(SP_DEBUG, thread << 8, DBG_LAST + 1)


def dbg_want(model, thread):
    """What `dbg_read_thread` must return for the modelled state."""
    return [model[CELL_OF[r]][thread] for r in range(DBG_LAST + 1)]


def dbg_diff(thread, got, want):
    return ["t%d reg %#04x: %#06x not %#06x" % (thread, r, got[r], want[r])
            for r in range(DBG_LAST + 1) if got[r] != want[r]]


async def dbg_check(host, model, threads):
    """Read `threads` back and return every register that is not as modelled."""
    bad = []
    for t in threads:
        bad += dbg_diff(t, await dbg_read_thread(host, t), dbg_want(model, t))
    return bad


async def dbg_park_ticks(host, model):
    """Re-arm every thread's tick accumulator, so NOW holds still."""
    for t in range(4):
        await host.write_debug(t, DBG_CSR0 + CSR_TICK_INT, TICK_PARK)
        model["TICK_INT"][t] = TICK_PARK


@cocotb.test()
async def test_debug_register_cross_talk(dut):
    """A debug write moves the register it names, of the thread it names, and
    nothing else.

    Every implemented debug register id is written with a distinctive value
    and the whole 4 x 39 register picture is then required to be exactly what
    was written: the register that was named, of the thread that was named,
    and no other. Both aliases are written through the alias and read through
    the primary id (0x1A for TD, 0x1B for FLAGS, and 0x1D..0x1F for the bit
    engine), and the read-only ids are written too, where the intended change
    is none at all. Threads hold different values in every register, so a
    read mux that takes the wrong thread's slice is visible as well.

    Nothing else in the suite writes one debug register and then looks at the
    others, or at the other three threads, so this is the only check on the
    decode of the debug register file.
    """
    host = LoomHost(dut)
    await host.start()
    model = {name: [0, 0, 0, 0] for name, _i, _m, _ro in DBG_CELLS}

    # Give every writable cell a fingerprint that differs per thread. LATCH
    # is written with LAT_VALID clear: a staged write would be applied (and
    # cleared) by a later TD write, and this test owns the whole picture.
    # TICK_INT is left parked.
    for index, (name, ids, mask, ro) in enumerate(DBG_CELLS):
        if ro or name == "TICK_INT":
            continue
        if index % 8 == 0:
            await dbg_park_ticks(host, model)
        for t in range(4):
            value = dbg_value(ids[0], t) & (0x3F if name == "LATCH" else mask)
            await host.write_debug(t, ids[0], value)
            model[name][t] = value
    for t in range(4):
        model["TID"][t] = t
    await dbg_park_ticks(host, model)
    first = [await dbg_read_thread(host, t) for t in range(4)]
    for t in range(4):
        model["NOW"][t] = first[t][0x0B]     # read only, parked, not zero
    bad = [d for t in range(4) for d in dbg_diff(t, first[t], dbg_want(model, t))]
    assert not bad, "the fingerprints did not land: " + "; ".join(bad[:8])

    ids = list(range(DBG_LAST + 1))
    for start in range(0, len(ids), DBG_PASS):
        chunk = ids[start:start + DBG_PASS]
        th = (start // DBG_PASS) % 4
        await dbg_park_ticks(host, model)
        for reg in chunk:
            cell = CELL_OF[reg]
            mask = MASK_OF[cell]
            value = DBG_FORCE.get(reg, dbg_value(reg, th))
            if cell == "TICK_INT":
                value |= 0xF000              # a long period, whatever it is
            if (value & mask) == model[cell][th]:
                value ^= mask                # never write back what is there
            await host.write_debug(th, reg, value)
            if cell not in RO_CELLS:
                model[cell][th] = value & mask
                if cell == "PC":
                    model["WAIT_ACTIVE"][th] = 0   # a debug PC write clears it
            bad = await dbg_check(host, model, [th])
            assert not bad, \
                "debug write %#04x = %#06x on thread %d also moved: %s" \
                % (reg, value, th, "; ".join(bad[:8]))
        others = [t for t in range(4) if t != th]
        bad = await dbg_check(host, model, others)
        assert not bad, \
            "writing %s on thread %d reached another thread: %s" \
            % (" ".join("%#04x" % r for r in chunk), th, "; ".join(bad[:8]))

    # CTRL.RESET clears WAIT_ACTIVE, for the thread it names and no other.
    for t in range(4):
        await host.write_debug(t, DBG_WAIT_ACTIVE, 1)
    assert [await host.read_debug(t, DBG_WAIT_ACTIVE) for t in range(4)] \
        == [1, 1, 1, 1]
    await host.reset_thread(2)
    await ClockCycles(dut.clk, 8)
    assert [await host.read_debug(t, DBG_WAIT_ACTIVE) for t in range(4)] \
        == [1, 1, 0, 1], "CTRL.RESET clears WAIT_ACTIVE, for one thread only"
    assert await host.badop() == 0


# --------------------------------------------------- the SPI byte layer
# Two HOST_PROTOCOL promises the mutation run (tools/mutate, round 2) found
# nothing checking: loom_spi_host survived with MISO idling high, and with the
# byte state never cleared while CS_n is high.

@cocotb.test()
async def test_miso_idles_low(dut):
    """HOST_MISO is driven 0 while CS_n is high (HOST_PROTOCOL)."""
    host = LoomHost(dut)
    await host.start()
    # A read that ends on a data bit of 1 leaves the shifter full of ones:
    # the case where a MISO that followed the shifter while idle would be high.
    await host.write(SP_IMEM, 0, [0xFFFF])
    assert await host.read(SP_IMEM, 0, 1) == [0xFFFF]
    for _ in range(24):                  # CS_n has been high for 4 clocks
        await ClockCycles(dut.clk, 1)
        miso = str(dut.uo_out.value)[0]  # uo_out[7:0], MSB first; X must fail too
        assert miso == "0", "MISO is %r while CS_n is high" % miso


@cocotb.test()
async def test_cs_rising_mid_byte_voids_the_byte(dut):
    """CS_n rising resets the byte counter at any point, and a word cut short
    is discarded (HOST_PROTOCOL): framing survives, nothing half-written lands."""
    host = LoomHost(dut)
    await host.start()
    ident = await host.read1(SP_CTRL, CTRL_ID)
    await host.set_reset_pc(2, 0x055)

    # Bare partial bytes: the next transaction must frame from bit 0 again.
    for nbits in (1, 3, 7):
        await host.transfer(b"", tail_bits=nbits, tail=0xA5)
        assert await host.read1(SP_CTRL, CTRL_ID) == ident, \
            "a byte cut after %d bits shifted the framing" % nbits

    # A write of RESET_PC2 cut inside its data word: nothing may land.
    addr = CTRL_RESET_PC0 + 2
    head = bytes([0x80 | (SP_CTRL << 4), (addr >> 8) & 0xFF, addr & 0xFF, 0x00])
    for nbits in (1, 4, 7):
        await host.transfer(head, tail_bits=nbits, tail=0xAA)
        assert await host.read1(SP_CTRL, addr) == 0x055, \
            "a data word cut after %d bits of its low byte was written" % nbits


@cocotb.test()
async def test_reset_sets_td_to_now(dut):
    """CTRL.RESET sets TD := NOW for the thread it resets (HOST_PROTOCOL
    0x0004). Nothing read TD after a reset until a mutant that skipped the
    assignment survived."""
    host = LoomHost(dut)
    await host.start()
    # A slow tick first, so NOW holds still across the transactions below
    # (writing TICK_INT clears the accumulator, SEMANTICS 4).
    await host.write_debug(1, DBG_CSR0 + CSR_TICK_INT, 60000)
    await host.write_debug(1, DBG_TD, 0x1234)
    now0 = await host.read_debug(1, DBG_NOW)
    assert now0 != 0x1234
    await host.reset_thread(1)
    td = await host.read_debug(1, DBG_TD)
    now1 = await host.read_debug(1, DBG_NOW)
    assert td in (now0, now1), "TD %#x after CTRL.RESET, NOW %#x..%#x" % (td, now0, now1)
    assert await host.read_debug(0, DBG_TD) != td or now0 == 0, \
        "CTRL.RESET of thread 1 must leave thread 0's TD alone"
