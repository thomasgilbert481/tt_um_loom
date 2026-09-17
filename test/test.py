# SPDX-License-Identifier: Apache-2.0
# Loom M0.5 test: exercise the IHP RM_IHPSG13_1P_512x16_c2_bm_bist SRAM macro
# through the tt_um_loom pin protocol on branch `sram-smoke`.
#
# The DUT is driven exactly the way a host would drive it on the demo board:
# byte registers over uio, edge strobes on ui_in, read-back on uo_out and uio.
# The RTL case compiles the vendored behavioural model from macro/; the GL case
# compiles the same model next to the synthesised netlist, so both run the same
# checks against the same memory model.
#
# Checks:
#   walking ones        every one of the 16 data bits, and its complement,
#                       survives a write/read round trip at one address
#   address uniqueness  a distinct pattern is written to all 512 words and all
#                       512 are read back: catches stuck or shorted address
#                       bits, which a single-address test cannot
#   random access       interleaved writes and reads against a shadow model
#   write wins          a write and a read issued on the same cycle: the write
#                       lands, the read does not (the timing contract in
#                       src/loom_imem_macro.v)

import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

CLK_NS = 20  # 50 MHz, matches CLOCK_PERIOD in src/config.json

WORDS = 512
MASK16 = 0xFFFF

# ui_in bit positions, see src/tt_um_loom.v
REGSEL_SHIFT = 0
REG_WR = 1 << 2
RD_OE = 1 << 3
MEM_WR = 1 << 4
MEM_RD = 1 << 5
UIO_SEL = 1 << 6

# REGSEL values
R_ADDR_LO = 0
R_ADDR_HI = 1
R_WDATA_LO = 2
R_WDATA_HI = 3

HOLD = 3  # clocks a strobe is held high, and held low again


def pattern(addr: int) -> int:
    """A different 16-bit word for every address, invertible so a wrong read
    points at the address it actually came from. 2053 is odd, so the map is a
    bijection modulo 2**16."""
    return (addr * 2053 + 0x1234) & MASK16


class Host:
    """Drives the tt_um_loom pin protocol. Caches the byte registers so that a
    sweep over consecutive addresses does not rewrite ADDR_HI 512 times."""

    def __init__(self, dut):
        self.dut = dut
        # Mirrors the DUT's byte registers. Only REG_WR changes them, so the
        # commands and the read-backs below cannot make this stale.
        self.regs = {}

    async def _set_reg(self, sel: int, val: int):
        if self.regs.get(sel) == val:
            return
        dut = self.dut
        dut.uio_in.value = val
        dut.ui_in.value = sel << REGSEL_SHIFT
        await ClockCycles(dut.clk, 1)
        dut.ui_in.value = (sel << REGSEL_SHIFT) | REG_WR
        await ClockCycles(dut.clk, HOLD)
        dut.ui_in.value = sel << REGSEL_SHIFT
        await ClockCycles(dut.clk, HOLD)
        self.regs[sel] = val

    async def _command(self, bits: int):
        dut = self.dut
        dut.ui_in.value = bits
        await ClockCycles(dut.clk, HOLD)
        dut.ui_in.value = 0
        await ClockCycles(dut.clk, HOLD)

    async def set_addr(self, addr: int):
        assert 0 <= addr < WORDS
        await self._set_reg(R_ADDR_LO, addr & 0xFF)
        await self._set_reg(R_ADDR_HI, (addr >> 8) & 1)

    async def set_wdata(self, data: int):
        await self._set_reg(R_WDATA_LO, data & 0xFF)
        await self._set_reg(R_WDATA_HI, (data >> 8) & 0xFF)

    async def status(self) -> int:
        dut = self.dut
        dut.ui_in.value = RD_OE | UIO_SEL
        await ClockCycles(dut.clk, 1)
        assert int(dut.uio_oe.value) == 0xFF, "uio_oe must be all ones while RD_OE"
        val = int(dut.uio_out.value)
        dut.ui_in.value = 0
        await ClockCycles(dut.clk, 1)
        return val

    async def read_rdata(self) -> int:
        """The last word read, low byte from uo_out and high byte from uio."""
        dut = self.dut
        lo = int(dut.uo_out.value)
        dut.ui_in.value = RD_OE  # UIO_SEL low -> uio carries RDATA[15:8]
        await ClockCycles(dut.clk, 1)
        assert int(dut.uio_oe.value) == 0xFF, "uio_oe must be all ones while RD_OE"
        hi = int(dut.uio_out.value)
        dut.ui_in.value = 0
        await ClockCycles(dut.clk, 1)
        return (hi << 8) | lo

    async def write(self, addr: int, data: int):
        await self.set_addr(addr)
        await self.set_wdata(data)
        await self._command(MEM_WR)

    async def read(self, addr: int) -> int:
        await self.set_addr(addr)
        await self._command(MEM_RD)
        return await self.read_rdata()


async def start(dut):
    cocotb.start_soon(Clock(dut.clk, CLK_NS, unit="ns").start())
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)
    return Host(dut)


@cocotb.test()
async def test_reset_and_pin_hygiene(dut):
    """After reset: nothing read yet, uio released, RDATA zero."""
    host = await start(dut)

    assert int(dut.uio_oe.value) == 0, "uio must be an input while RD_OE is low"
    assert int(dut.uo_out.value) == 0, "RDATA should reset to zero"

    st = await host.status()
    assert st & 1 == 0, f"BUSY set after reset (status {st:#04x})"
    assert st & 2 == 0, f"RD_VALID set before any read (status {st:#04x})"

    # A read makes RD_VALID stick.
    await host.write(0, 0xBEEF)
    got = await host.read(0)
    assert got == 0xBEEF, f"read back {got:#06x}, expected 0xbeef"
    st = await host.status()
    assert st & 2 == 2, f"RD_VALID not set after a read (status {st:#04x})"
    assert st & 1 == 0, f"BUSY still set long after the access (status {st:#04x})"


@cocotb.test()
async def test_walking_ones(dut):
    """Each of the 16 data bits, on its own and inverted, survives a round trip."""
    host = await start(dut)
    addr = 0x155  # 1_0101_0101, an address with both halves busy

    for bit in range(16):
        one_hot = 1 << bit
        await host.write(addr, one_hot)
        got = await host.read(addr)
        assert got == one_hot, f"bit {bit}: wrote {one_hot:#06x}, read {got:#06x}"

        inverted = one_hot ^ MASK16
        await host.write(addr, inverted)
        got = await host.read(addr)
        assert got == inverted, f"bit {bit} inverted: wrote {inverted:#06x}, read {got:#06x}"


@cocotb.test()
async def test_address_uniqueness(dut):
    """Write a distinct word to all 512 addresses, then read all 512 back."""
    host = await start(dut)

    for addr in range(WORDS):
        await host.write(addr, pattern(addr))

    bad = []
    for addr in range(WORDS):
        got = await host.read(addr)
        if got != pattern(addr):
            bad.append((addr, got))
            if len(bad) > 8:
                break
    assert not bad, (
        "address sweep mismatches (addr, read, expected): "
        + ", ".join(f"({a:#05x}, {g:#06x}, {pattern(a):#06x})" for a, g in bad)
    )
    dut._log.info(f"{WORDS} words written and read back with no mismatch")


@cocotb.test()
async def test_random_access(dut):
    """Interleaved random writes and reads against a shadow model."""
    host = await start(dut)
    rng = random.Random(20260917)
    shadow = {}

    # seed a handful of addresses so reads always hit written memory
    for _ in range(12):
        addr = rng.randrange(WORDS)
        data = rng.randrange(1 << 16)
        shadow[addr] = data
        await host.write(addr, data)

    for _ in range(60):
        if rng.random() < 0.5:
            addr = rng.randrange(WORDS)
            data = rng.randrange(1 << 16)
            shadow[addr] = data
            await host.write(addr, data)
        else:
            addr = rng.choice(sorted(shadow))
            got = await host.read(addr)
            assert got == shadow[addr], (
                f"addr {addr:#05x}: read {got:#06x}, expected {shadow[addr]:#06x}"
            )


@cocotb.test()
async def test_write_wins_over_read(dut):
    """MEM_WR and MEM_RD asserted together: the write lands, the read does not.

    This is the single-port conflict rule in the header of
    src/loom_imem_macro.v (A_REN is driven with ~we, so no read is issued on a
    write cycle and RDATA keeps whatever the port produced before)."""
    host = await start(dut)
    addr = 0x0AA

    await host.write(addr, 0x3333)
    got = await host.read(addr)
    assert got == 0x3333, f"setup read gave {got:#06x}"

    # Same address, new data, both commands in one pulse.
    await host.set_addr(addr)
    await host.set_wdata(0x2222)
    await host._command(MEM_WR | MEM_RD)

    got = await host.read_rdata()
    assert got != 0x2222, (
        f"RDATA became {got:#06x}: the read was serviced on a write cycle, "
        "so the write did not win"
    )

    # The write itself must have landed.
    got = await host.read(addr)
    assert got == 0x2222, f"write during the conflict cycle was lost, read {got:#06x}"
