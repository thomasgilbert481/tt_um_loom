# SPDX-License-Identifier: Apache-2.0
"""Directed tests for every [M1] ALU, immediate and unary instruction.

Expected values are computed here from docs/SEMANTICS.md 6.1, never from the
golden model. Programs are built with tools.loomisa, never from literal hex.
"""

import cocotb

from spi_host import LoomHost, asm, run_snippet, DBG_FLAGS, CSR_FLAGS

M = 0xFFFF


# ------------------------------------------------ SEMANTICS 6.1, in Python
def f_add(a, b):
    r = a + b
    return r & M, 1 if r > M else 0


def f_sub(a, b):
    return (a - b) & M, 1 if a >= b else 0


def f_shl(a, n):
    return (a << n) & M, ((a >> (16 - n)) & 1) if n else 0


def f_shr(a, n):
    return a >> n, ((a >> (n - 1)) & 1) if n else 0


def f_ror(a, n):
    r = a if n == 0 else ((a >> n) | (a << (16 - n))) & M
    return r, ((r >> 15) & 1) if n else 0


def f_rev(a):
    return int(format(a, "016b")[::-1], 2), 0


def f_par(a):
    return a, bin(a).count("1") & 1


def f_swap(a):
    return ((a & 0xFF) << 8) | (a >> 8), 0


ALU_OPS = {
    "ADD": f_add,
    "SUB": f_sub,
    "AND": lambda a, b: (a & b, 0),
    "OR":  lambda a, b: (a | b, 0),
    "XOR": lambda a, b: (a ^ b, 0),
    "SHL": lambda a, b: f_shl(a, b & 0xF),
    "SHR": lambda a, b: f_shr(a, b & 0xF),
    "ROR": lambda a, b: f_ror(a, b & 0xF),
}

ALUI_OPS = {
    "ADDI": f_add,
    "SUBI": f_sub,
    "ANDI": lambda a, b: (a & b, 0),
    "ORI":  lambda a, b: (a | b, 0),
    "XORI": lambda a, b: (a ^ b, 0),
    "SHLI": lambda a, b: f_shl(a, b & 0xF),
    "SHRI": lambda a, b: f_shr(a, b & 0xF),
}

UNARY_OPS = {
    "MOV":  lambda a: (a, None),                 # no flags at all
    "NOT":  lambda a: (~a & M, 0),
    "NEG":  lambda a: ((-a) & M, 1 if a == 0 else 0),
    "REV":  f_rev,
    "PAR":  f_par,
    "SWAP": f_swap,
}


def flags_word(z, c, t=0):
    return (t << 2) | (c << 1) | z


async def check(host, dut, instrs, regs, expect, thread=0):
    """Run a snippet and compare {reg: value} plus an optional 'flags' key."""
    await run_snippet(host, instrs, thread=thread, regs=regs)
    for key, want in expect.items():
        if key == "flags":
            got = await host.read_debug(thread, DBG_FLAGS)
        else:
            got = await host.read_reg(thread, key)
        assert got == want, (
            f"{instrs}: {key} = {got:#06x}, expected {want:#06x}")


@cocotb.test()
async def test_alu_three_operand(dut):
    """ADD SUB AND OR XOR SHL SHR ROR: result, Z and C."""
    host = LoomHost(dut)
    await host.start()
    vectors = [
        (0x0000, 0x0000), (0xFFFF, 0x0001), (0x8000, 0x8000),
        (0x1234, 0x5678), (0xABCD, 0x000F), (0x0001, 0x0000),
        (0x8001, 0x0001),
    ]
    for name, model in ALU_OPS.items():
        for a, b in vectors:
            res, c = model(a, b)
            await check(host, dut,
                        [asm(name, rd=3, ra=1, rb=2),
                         asm("CSRR", rd=4, csr=CSR_FLAGS)],
                        {1: a, 2: b},
                        {3: res, 4: flags_word(1 if res == 0 else 0, c)})


@cocotb.test()
async def test_shift_edges(dut):
    """Shift amounts 0 and 15, and the ROR carry rule."""
    host = LoomHost(dut)
    await host.start()
    for a in (0x8001, 0xFFFF, 0x0001, 0x1248):
        for n in (0, 1, 15):
            for name, model in (("SHL", f_shl), ("SHR", f_shr), ("ROR", f_ror)):
                res, c = model(a, n)
                await check(host, dut,
                            [asm(name, rd=3, ra=1, rb=2),
                             asm("CSRR", rd=4, csr=CSR_FLAGS)],
                            {1: a, 2: n},
                            {3: res, 4: flags_word(1 if res == 0 else 0, c)})
    # The shift amount is only rb[3:0]: 0x10 shifts by 0, not by 16.
    await check(host, dut,
                [asm("SHL", rd=3, ra=1, rb=2), asm("CSRR", rd=4, csr=CSR_FLAGS)],
                {1: 0xBEEF, 2: 0x0010},
                {3: 0xBEEF, 4: flags_word(0, 0)})


@cocotb.test()
async def test_alui(dut):
    """ADDI..SHRI write rd = rd op imm6; CMPI writes nothing."""
    host = LoomHost(dut)
    await host.start()
    for name, model in ALUI_OPS.items():
        for a, imm in ((0x0000, 0), (0xFFFF, 1), (0x0010, 0x3F), (0x1234, 4)):
            res, c = model(a, imm)
            await check(host, dut,
                        [asm(name, rd=1, imm=imm),
                         asm("CSRR", rd=4, csr=CSR_FLAGS)],
                        {1: a},
                        {1: res, 4: flags_word(1 if res == 0 else 0, c)})
    # CMPI: flags of rd - imm6, rd unchanged. Carry means "no borrow".
    for a, imm in ((5, 5), (5, 6), (6, 5), (0, 0x3F)):
        res, c = f_sub(a, imm)
        await check(host, dut,
                    [asm("CMPI", rd=1, imm=imm),
                     asm("CSRR", rd=4, csr=CSR_FLAGS)],
                    {1: a},
                    {1: a, 4: flags_word(1 if res == 0 else 0, c)})


@cocotb.test()
async def test_sub_and_cmp_carry(dut):
    """SUB/CMP set C = 1 exactly when there is no borrow (a >= b)."""
    host = LoomHost(dut)
    await host.start()
    for a, b in ((0, 0), (0, 1), (1, 0), (0x8000, 0x7FFF), (0x7FFF, 0x8000),
                 (0xFFFF, 0xFFFF)):
        res, c = f_sub(a, b)
        z = 1 if res == 0 else 0
        await check(host, dut,
                    [asm("SUB", rd=3, ra=1, rb=2),
                     asm("CSRR", rd=4, csr=CSR_FLAGS)],
                    {1: a, 2: b}, {3: res, 4: flags_word(z, c)})
        # CMP uses rd as the left operand and ra as the right one, writes nothing.
        await check(host, dut,
                    [asm("CMP", rd=1, ra=2), asm("CSRR", rd=4, csr=CSR_FLAGS)],
                    {1: a, 2: b, 3: 0xA5A5},
                    {1: a, 3: 0xA5A5, 4: flags_word(z, c)})


@cocotb.test()
async def test_unary(dut):
    """MOV NOT NEG REV PAR SWAP, and TEST's Z rule."""
    host = LoomHost(dut)
    await host.start()
    values = (0x0000, 0x0001, 0x8000, 0xFFFF, 0x1234, 0xABCD, 0x00FF)
    for name, model in UNARY_OPS.items():
        for a in values:
            res, c = model(a)
            expect = {3: res}
            if c is not None:
                expect[4] = flags_word(1 if res == 0 else 0, c)
            else:
                expect[4] = flags_word(0, 0)     # MOV leaves the flags alone
            await check(host, dut,
                        [asm("CSRW", csr=CSR_FLAGS, ra=0),   # clear the flags
                         asm(name, rd=3, ra=1),
                         asm("CSRR", rd=4, csr=CSR_FLAGS)],
                        {0: 0, 1: a, 3: 0}, expect)
    # TEST: Z = ((rd & ra) == 0), C = 0, nothing written.
    for a, b in ((0xF0F0, 0x0F0F), (0xFF00, 0x0100), (0, 0xFFFF)):
        z = 1 if (a & b) == 0 else 0
        await check(host, dut,
                    [asm("TEST", rd=1, ra=2), asm("CSRR", rd=4, csr=CSR_FLAGS)],
                    {1: a, 2: b}, {1: a, 2: b, 4: flags_word(z, 0)})


@cocotb.test()
async def test_ldi_ldih(dut):
    """LDI zero-extends imm8; LDIH replaces the high byte only; no flags."""
    host = LoomHost(dut)
    await host.start()
    for imm in (0x00, 0x01, 0x7F, 0x80, 0xFF):
        await check(host, dut, [asm("LDI", rd=2, imm=imm)],
                    {2: 0xFFFF}, {2: imm})
    for low, imm in ((0x00, 0xAB), (0x5A, 0xFF), (0xFF, 0x00)):
        await check(host, dut,
                    [asm("LDI", rd=2, imm=low), asm("LDIH", rd=2, imm=imm)],
                    {2: 0x1234}, {2: (imm << 8) | low})
    # Neither instruction touches the flags: set C with a SUB first.
    await check(host, dut,
                [asm("SUB", rd=3, ra=1, rb=2),      # 1 - 0: C = 1, Z = 0
                 asm("LDI", rd=5, imm=0),
                 asm("LDIH", rd=5, imm=0),
                 asm("CSRR", rd=4, csr=CSR_FLAGS)],
                {1: 1, 2: 0}, {5: 0, 4: flags_word(0, 1)})
