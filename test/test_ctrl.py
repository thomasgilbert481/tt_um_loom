# SPDX-License-Identifier: Apache-2.0
"""Control flow, CSRs, shared flags and BADOP (docs/SEMANTICS.md 6.2, 6.5,
6.6 and 9). Expected values are computed here from the specification."""

import cocotb
from cocotb.triggers import ClockCycles

from spi_host import (
    LoomHost, ISA, asm, run_snippet, SP_CTRL, CTRL_SFLAGS, CTRL_SFLAGS_CLR,
    DBG_PC, DBG_FLAGS, DBG_RS0, DBG_RS1_DEPTH, DBG_WAIT_ACTIVE, DBG_TD, DBG_DT,
    CSR_FLAGS, CSR_NOW, CSR_TD, CSR_TID, CSR_OUTGRP, CSR_INGRP, CSR_SFLAGS,
    CSR_OD_MASK, CSR_TICK_INT,
)

BCC = {"BZ": ("Z", True), "BNZ": ("Z", False),
       "BC": ("C", True), "BNC": ("C", False),
       "BT": ("T", True), "BNT": ("T", False)}


def reserved_words(count=2):
    """Words that match no instruction in isa.yaml (SEMANTICS section 9)."""
    out = []
    for w in range(1 << 16):
        if ISA.decode(w) is None:
            out.append(w)
            if len(out) == count:
                return out
    raise AssertionError("the encoding space has no reserved word")


@cocotb.test()
async def test_conditional_branches(dut):
    """Every Bcc, taken and not taken. rel8 is relative to the next PC."""
    host = LoomHost(dut)
    await host.start()
    for name, (flag, want_set) in BCC.items():
        for value in (0, 1):
            taken = (value == 1) == want_set
            flags = {"Z": 1, "C": 2, "T": 4}[flag] * value
            await run_snippet(host, [
                asm("CSRW", csr=CSR_FLAGS, ra=1),   # r1 holds the flag word
                asm(name, rel=1),                   # skip the next instruction
                asm("LDI", rd=3, imm=0xAA),
                asm("LDI", rd=4, imm=0x55),
            ], regs={1: flags, 3: 0, 4: 0})
            r3 = await host.read_reg(0, 3)
            r4 = await host.read_reg(0, 4)
            assert r4 == 0x55
            assert r3 == (0 if taken else 0xAA), \
                f"{name} with {flag}={value}: taken={taken}, r3={r3:#x}"


@cocotb.test()
async def test_branch_backwards_and_djnz(dut):
    """A negative rel8 branches backwards; DJNZ tests the decremented value."""
    host = LoomHost(dut)
    await host.start()
    for n in (1, 2, 5):
        await run_snippet(host, [
            asm("LDI", rd=1, imm=n),
            asm("ADDI", rd=2, imm=1),
            asm("DJNZ", rd=1, rel=-2),
        ], regs={1: 0, 2: 0})
        assert await host.read_reg(0, 1) == 0
        assert await host.read_reg(0, 2) == n, f"DJNZ {n} iterations"
    # DJNZ sets no flags, and wraps: 0 - 1 = 0xFFFF, which is not zero.
    await run_snippet(host, [
        asm("CSRW", csr=CSR_FLAGS, ra=3),
        asm("DJNZ", rd=1, rel=1),            # r1 = 0xFFFF, taken
        asm("LDI", rd=4, imm=0xAA),
        asm("CSRR", rd=5, csr=CSR_FLAGS),
    ], regs={1: 0, 3: 0x7, 4: 0, 5: 0})
    assert await host.read_reg(0, 1) == 0xFFFF
    assert await host.read_reg(0, 4) == 0, "DJNZ should have branched"
    assert await host.read_reg(0, 5) == 0x7, "DJNZ must not touch the flags"


@cocotb.test()
async def test_call_ret_depths(dut):
    """RET at depth 0 falls through; depth 1 and 2 return; depth 3 overflows."""
    host = LoomHost(dut)
    await host.start()

    # Depth 0: RET with an empty stack behaves as PC <= next.
    await run_snippet(host, [asm("RET"), asm("LDI", rd=1, imm=0x5A)],
                      regs={1: 0})
    assert await host.read_reg(0, 1) == 0x5A
    assert await host.read_debug(0, DBG_RS1_DEPTH) >> 10 == 0

    # Depth 1 and 2, then an overflow that drops the oldest return address.
    prog = {
        0x00: asm("CALL", abs=0x10),
        0x01: asm("LDI", rd=1, imm=0xFF),     # lost by the overflow
        0x02: asm("HALT"),
        0x10: asm("CALL", abs=0x20),
        0x11: asm("ADDI", rd=2, imm=1),
        0x12: asm("RET"),                     # depth 0 here: falls through
        0x13: asm("HALT"),
        0x20: asm("CALL", abs=0x30),
        0x21: asm("ADDI", rd=3, imm=1),
        0x22: asm("RET"),
        0x30: asm("RET"),
    }
    await host.load_program(prog, verify=False)
    await host.set_reset_pc(0, 0)
    await host.reset_thread(0)
    for n in range(8):
        await host.write_reg(0, n, 0)
    await host.run(0b0001)
    await host.wait_halted(0b0001)
    assert await host.read_reg(0, 1) == 0, "the dropped return must not run"
    assert await host.read_reg(0, 2) == 1
    assert await host.read_reg(0, 3) == 1
    assert await host.read_debug(0, DBG_PC) == 0x14
    assert await host.read_debug(0, DBG_RS1_DEPTH) >> 10 == 0

    # The stack registers are visible in the debug space.
    prog2 = {0x00: asm("CALL", abs=0x05), 0x05: asm("CALL", abs=0x09),
             0x09: asm("HALT")}
    await host.load_program(prog2, verify=False)
    await host.set_reset_pc(0, 0)
    await host.reset_thread(0)
    await host.run(0b0001)
    await host.wait_halted(0b0001)
    assert await host.read_debug(0, DBG_RS0) == 0x06
    assert await host.read_debug(0, DBG_RS1_DEPTH) & 0x3FF == 0x01
    assert await host.read_debug(0, DBG_RS1_DEPTH) >> 10 == 2


@cocotb.test()
async def test_jp_on_a_pin(dut):
    """JP pin, v, rel6 branches on one synchronised input."""
    host = LoomHost(dut)
    await host.start()
    for level in (0, 1):
        host.set_in(8, level)                 # IN0 = ui_in[0] = pin index 8
        await ClockCycles(dut.clk, 8)
        for v in (0, 1):
            await run_snippet(host, [
                asm("JP", pin=8, val=v, rel=1),
                asm("LDI", rd=3, imm=0xAA),
                asm("LDI", rd=4, imm=0x55),
            ], regs={3: 0, 4: 0})
            taken = (level == v)
            assert await host.read_reg(0, 3) == (0 if taken else 0xAA), \
                f"JP pin8=={v} with the pin at {level}"
            assert await host.read_reg(0, 4) == 0x55
    host.set_in(8, 0)
    await ClockCycles(dut.clk, 4)


@cocotb.test()
async def test_csr_read_write(dut):
    """CSRR and CSRW for the CSRs built at M1."""
    host = LoomHost(dut)
    await host.start()
    # TID reads the thread number and ignores writes.
    for t in range(4):
        await run_snippet(host, [asm("CSRR", rd=1, csr=CSR_TID),
                                 asm("CSRW", csr=CSR_TID, ra=2)],
                          thread=t, start=0x60, regs={1: 0xFFFF, 2: 3})
        assert await host.read_reg(t, 1) == t
        assert await host.read_csr(t, CSR_TID) == t

    # OUTGRP / INGRP round trip through their CSRs (10 bits).
    await run_snippet(host, [asm("CSRW", csr=CSR_OUTGRP, ra=1),
                             asm("CSRW", csr=CSR_INGRP, ra=2),
                             asm("CSRR", rd=3, csr=CSR_OUTGRP),
                             asm("CSRR", rd=4, csr=CSR_INGRP)],
                      regs={1: 0xFFFF, 2: 0x0155})
    assert await host.read_reg(0, 3) == 0x3FF
    assert await host.read_reg(0, 4) == 0x155

    # FLAGS is {T, C, Z} in bits 2:0.
    for value in range(8):
        await run_snippet(host, [asm("CSRW", csr=CSR_FLAGS, ra=1),
                                 asm("CSRR", rd=2, csr=CSR_FLAGS)],
                          regs={1: value | 0xFFF8, 2: 0})
        assert await host.read_reg(0, 2) == value
        assert await host.read_debug(0, DBG_FLAGS) == value

    # NOW is read only and advances; TD is writable.
    await run_snippet(host, [asm("CSRR", rd=1, csr=CSR_NOW),
                             asm("CSRR", rd=2, csr=CSR_NOW),
                             asm("CSRW", csr=CSR_NOW, ra=3),
                             asm("CSRW", csr=CSR_TD, ra=4),
                             asm("CSRR", rd=5, csr=CSR_TD)],
                      regs={3: 0, 4: 0x1234})
    n1 = await host.read_reg(0, 1)
    n2 = await host.read_reg(0, 2)
    assert ((n2 - n1) & 0xFFFF) == 4, f"NOW advanced by {(n2 - n1) & 0xFFFF}"
    assert await host.read_reg(0, 5) == 0x1234
    assert await host.read_debug(0, DBG_TD) == 0x1234

    # OD_MASK and SFLAGS are global; CSRW SFLAGS ORs into the register.
    await host.write(SP_CTRL, CTRL_SFLAGS_CLR, 0xFF)
    await run_snippet(host, [asm("CSRW", csr=CSR_OD_MASK, ra=1),
                             asm("CSRR", rd=2, csr=CSR_OD_MASK),
                             asm("CSRW", csr=CSR_SFLAGS, ra=3),
                             asm("CSRW", csr=CSR_SFLAGS, ra=4),
                             asm("CSRR", rd=5, csr=CSR_SFLAGS)],
                      regs={1: 0xFFA5, 3: 0x03, 4: 0x18})
    assert await host.read_reg(0, 2) == 0xA5
    assert await host.read_reg(0, 5) == 0x1B, "CSRW SFLAGS must OR"
    assert await host.read1(SP_CTRL, CTRL_SFLAGS) == 0x1B
    await host.write(SP_CTRL, CTRL_SFLAGS_CLR, 0xFF)
    # An unimplemented CSR reads 0 and its write is ignored (SEMANTICS 6.6).
    await run_snippet(host, [asm("CSRW", csr=0x0D, ra=1),
                             asm("CSRR", rd=2, csr=0x0D)],
                      regs={1: 0xFFFF, 2: 0xFFFF})
    assert await host.read_reg(0, 2) == 0


@cocotb.test()
async def test_sig_clr_waits(dut):
    """SIG sets, CLR clears, WAITS waits for a shared flag and clears it."""
    host = LoomHost(dut)
    await host.start()
    await host.write(SP_CTRL, CTRL_SFLAGS_CLR, 0xFF)
    await run_snippet(host, [asm("SIG", flag=3), asm("SIG", flag=5),
                             asm("CLR", flag=3)])
    assert await host.read1(SP_CTRL, CTRL_SFLAGS) == 1 << 5

    # WAITS stalls until the flag is set, then clears it atomically.
    await host.write(SP_CTRL, CTRL_SFLAGS_CLR, 0xFF)
    await host.load_program({0x00: asm("WAITS", flag=2),
                             0x01: asm("ADDI", rd=1, imm=1),
                             0x02: asm("HALT")}, verify=False)
    await host.set_reset_pc(0, 0)
    await host.reset_thread(0)
    await host.write_reg(0, 1, 0)
    await host.run(0b0001)
    await ClockCycles(dut.clk, 100)
    assert await host.halted() == 0, "WAITS must stall on a clear flag"
    assert await host.read_debug(0, DBG_WAIT_ACTIVE) == 1
    assert await host.read_debug(0, DBG_PC) == 0, "a stalled wait keeps its PC"
    await host.write(SP_CTRL, CTRL_SFLAGS, 1 << 2)
    await host.wait_halted(0b0001)
    assert await host.read_reg(0, 1) == 1
    assert await host.read1(SP_CTRL, CTRL_SFLAGS) == 0, "WAITS must clear it"
    assert await host.read_debug(0, DBG_WAIT_ACTIVE) == 0


@cocotb.test()
async def test_waits_atomic_between_threads(dut):
    """Two threads in adjacent slots: one SIG wakes exactly one WAITS."""
    host = LoomHost(dut)
    await host.start()
    await host.write(SP_CTRL, CTRL_SFLAGS_CLR, 0xFF)
    prog = {}
    for t in (0, 1):
        base = 0x00 + 0x10 * t
        prog[base + 0] = asm("WAITS", flag=1)
        prog[base + 1] = asm("ADDI", rd=t, imm=1)
        prog[base + 2] = asm("HALT")
    await host.load_program(prog, verify=False)
    for t in (0, 1):
        await host.set_reset_pc(t, 0x10 * t)
        await host.reset_thread(t)
        await host.write_reg(t, t, 0)
    await host.run(0b0011)
    await ClockCycles(dut.clk, 100)
    assert await host.halted() == 0

    await host.write(SP_CTRL, CTRL_SFLAGS, 1 << 1)
    await ClockCycles(dut.clk, 400)
    halted = await host.halted()
    assert halted in (0b0001, 0b0010), \
        f"exactly one thread must win the flag, HALTED={halted:#b}"
    assert await host.read1(SP_CTRL, CTRL_SFLAGS) == 0

    await host.write(SP_CTRL, CTRL_SFLAGS, 1 << 1)
    await host.wait_halted(0b0011)
    assert await host.read_reg(0, 0) == 1
    assert await host.read_reg(1, 1) == 1


@cocotb.test()
async def test_badop(dut):
    """Reserved words and unbuilt instructions are NOPs that set BADOP[t]."""
    host = LoomHost(dut)
    await host.start()
    unbuilt = [asm("PUSH", ra=1), asm("POP", rd=1), asm("WAITB", cond=0),
               asm("SHO"), asm("SHI"), asm("LDSR", ra=1), asm("STSR", rd=1),
               asm("CRCI"), asm("STCRC", rd=1),
               asm("LD", rd=1, ra=2, imm=0), asm("ST", rd=1, ra=2, imm=0)]
    for word in reserved_words(2) + unbuilt:
        await host.clear_badop(0xFFFF)
        assert await host.badop() == 0
        await host.load_program({0x00: word, 0x01: asm("ADDI", rd=1, imm=1),
                                 0x02: asm("HALT")}, verify=False)
        await host.set_reset_pc(0, 0)
        await host.reset_thread(0)
        await host.write_reg(0, 1, 0)
        await host.run(0b0001)
        await host.wait_halted(0b0001)
        assert await host.badop() & 1, f"BADOP[0] not set by {word:#06x}"
        assert await host.read_reg(0, 1) == 1, "it must execute as one NOP slot"
        assert await host.read_debug(0, DBG_PC) == 3

    # BADOP is per thread and sticky until the host clears it.
    await host.clear_badop(0xFFFF)
    word = reserved_words(1)[0]
    await host.load_program({0x30: word, 0x31: asm("HALT")}, verify=False)
    await host.set_reset_pc(2, 0x30)
    await host.reset_thread(2)
    await host.run(0b0100)
    await host.wait_halted(0b0100)
    assert await host.badop() == 0b0100
    await host.clear_badop(0b0100)
    assert await host.badop() == 0


@cocotb.test()
async def test_jmp(dut):
    """JMP goes to an absolute address, forwards and backwards."""
    host = LoomHost(dut)
    await host.start()
    prog = {0x00: asm("JMP", abs=0x04),
            0x01: asm("LDI", rd=1, imm=0xAA),
            0x02: asm("HALT"),
            0x03: asm("LDI", rd=3, imm=0xFF),     # never reached
            0x04: asm("LDI", rd=2, imm=0x55),
            0x05: asm("JMP", abs=0x01)}
    await host.load_program(prog, verify=False)
    await host.set_reset_pc(0, 0)
    await host.reset_thread(0)
    for n in range(4):
        await host.write_reg(0, n, 0)
    await host.run(0b0001)
    await host.wait_halted(0b0001)
    assert await host.read_reg(0, 1) == 0xAA
    assert await host.read_reg(0, 2) == 0x55
    assert await host.read_reg(0, 3) == 0
    assert await host.read_debug(0, DBG_PC) == 0x03


@cocotb.test()
async def test_dly(dut):
    """DLY waits imm8 ticks from now and leaves TD alone; DLY 0 is one slot."""
    host = LoomHost(dut)
    await host.start()
    # DLY 0 completes immediately and does not touch the deadline register.
    await run_snippet(host, [asm("CSRW", csr=CSR_TD, ra=1),
                             asm("DLY", imm=0),
                             asm("NOP")], regs={1: 0x4321})
    assert await host.read_debug(0, DBG_TD) == 0x4321
    assert await host.read_debug(0, DBG_WAIT_ACTIVE) == 0

    # DLY k waits k ticks measured from the slot it first issues in.
    for k in (1, 5, 20):
        await run_snippet(host, [asm("CSRW", csr=CSR_TICK_INT, ra=1),
                                 asm("CSRR", rd=2, csr=CSR_NOW),
                                 asm("DLY", imm=k),
                                 asm("CSRR", rd=3, csr=CSR_NOW)],
                          regs={1: 50, 2: 0, 3: 0})
        before = await host.read_reg(0, 2)
        after = await host.read_reg(0, 3)
        delta = (after - before) & 0xFFFF
        # NOW advances by one more tick between the two CSRR slots at most.
        assert k <= delta <= k + 1, f"DLY {k}: NOW advanced by {delta}"
        assert await host.read_debug(0, DBG_DT) == (before + k) & 0xFFFF
    await host.write_csr(0, CSR_TICK_INT, 1)


@cocotb.test()
async def test_tick_csr_and_nop(dut):
    """CSRW TICK_INT restarts the accumulator; NOP costs exactly one slot."""
    host = LoomHost(dut)
    await host.start()
    await run_snippet(host, [asm("CSRW", csr=CSR_TICK_INT, ra=1),
                             asm("CSRR", rd=2, csr=CSR_TICK_INT),
                             asm("NOP"), asm("NOP"), asm("NOP")],
                      regs={1: 100})
    assert await host.read_reg(0, 2) == 100
    assert await host.read_csr(0, CSR_TICK_INT) == 100
    # With a 100-clock tick, three NOPs (12 clocks) advance NOW by 0 ticks.
    await host.write_csr(0, CSR_TICK_INT, 1)
