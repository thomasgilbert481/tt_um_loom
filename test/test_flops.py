# SPDX-License-Identifier: Apache-2.0
"""The FLOPS fallback of the instruction memory keeps working (D-005, D-020).

The chip is built with ``IMEM_IMPL "MACRO"``, the 512 x 16 SRAM macro. The
flip-flop array stays behind the same ``loom_imem`` interface as the fallback
(docs/ARCHITECTURE.md 12), so something has to build and run it: ``tb.v``
instantiates a second ``tt_um_loom`` with ``IMEM_IMPL "FLOPS"`` and
``IMEM_WORDS`` 256 on its own pins (the ``*_flops`` signals), and this module
drives it with the helpers every other module uses, through ``FlopsDut``,
which maps the usual names (``clk``, ``ui_in``, ``uo_out``,
``user_project`` ...) onto that instance.

* ``test_flops_host_port``: CAPS reports 256 words, the default reset vectors
  are 0, 64, 128, 192, every address bit selects its own word, and two
  threads run programs fetched from the array (one from its default reset
  vector, one from the top words).
* ``test_flops_cosim``: random programs on the FLOPS build against the golden
  model with ``imem_words=256``, compared on every cycle by the
  ``test_cosim`` harness: loaded through the back door into the flop array,
  then one seed loaded over the SPI host port. ``LOOM_FLOPS_SEEDS`` (default
  4), ``LOOM_FLOPS_CYCLES`` (default 4000, per seed) and
  ``LOOM_FLOPS_SPI_SEEDS`` (default 1) size it.

RTL only: the gate-level netlist is the MACRO build and has no FLOPS
instance, so the module skips itself when ``GATES=yes``; in an RTL run a
missing FLOPS instance fails the tests.
"""

import os

import cocotb

from spi_host import (CTRL_RESET_PC0, DBG_PC, SP_CTRL, SP_IMEM,  # noqa: E402
                      LoomHost, asm)
from test_cosim import (ISA, SPI_CYCLES, _run_programs,          # noqa: E402
                        _seed_plan, _start_clock)
from tools.loomgen import generate                               # noqa: E402

IMEM_WORDS = 256
#: CAPS of the FLOPS build: log2(256) = 8 in [15:12], and the same M2
#: features as the MACRO build (SETP D, bit engine, FIFOs of depth 4).
EXPECT_CAPS = 0x8000 | 0x200 | 0x80 | 0x10 | 0x08 | 0x02   # [9]: slice A


def _env_int(name: str, default: int) -> int:
    return int(os.environ.get(name, str(default)) or default)


SEEDS = _env_int("LOOM_FLOPS_SEEDS", 4)
CYCLES = _env_int("LOOM_FLOPS_CYCLES", 4000)
SPI_SEEDS = _env_int("LOOM_FLOPS_SPI_SEEDS", 1)
SEED_BASE = 9001


class FlopsDut:
    """``tb``'s FLOPS instance under the names ``spi_host`` and ``test_cosim``
    use for the default one."""

    def __init__(self, tb):
        self.clk = tb.clk_flops
        self.rst_n = tb.rst_n_flops
        self.ena = tb.ena_flops
        self.ui_in = tb.ui_in_flops
        self.uio_drv = tb.uio_drv_flops
        self.uo_out = tb.uo_out_flops
        self.uio_out = tb.uio_out_flops
        self.uio_oe = tb.uio_oe_flops
        self.user_project = tb.user_project_flops
        self._log = tb._log


def _require_flops(dut):
    """Fail, rather than skip, an RTL run without the FLOPS instance."""
    try:
        dut.user_project_flops.u_loom.u_imem.g_flops.mem      # noqa: B018
    except AttributeError:
        raise AssertionError("tb.user_project_flops with the flop array "
                             "u_imem.g_flops.mem not found (test/tb.v builds "
                             "it unless GL_TEST)") from None


#: The gate-level flows set GATES=yes; the netlist has no FLOPS instance.
ABSENT = os.environ.get("GATES", "").lower() == "yes"


@cocotb.test(skip=ABSENT)
async def test_flops_host_port(dut):
    """FLOPS build: CAPS, default reset vectors, addressing, programs run."""
    _require_flops(dut)
    fd = FlopsDut(dut)
    host = LoomHost(fd)
    await host.start()

    caps = await host.caps()
    assert caps == EXPECT_CAPS, \
        f"FLOPS build CAPS {caps:#06x}, expected {EXPECT_CAPS:#06x}"
    # D-017: t * (IMEM_WORDS / 4).
    for t in range(4):
        got = await host.read1(SP_CTRL, CTRL_RESET_PC0 + t)
        assert got == t * (IMEM_WORDS // 4), f"RESET_PC[{t}] = {got:#x}"

    # Walking-one addresses and the last word, as in test_host.
    addrs = [0] + [1 << b for b in range(IMEM_WORDS.bit_length() - 1)]
    for i, a in enumerate(addrs):
        await host.write(SP_IMEM, a, 0x3C00 + i)
    for i, a in enumerate(addrs):
        got = await host.read1(SP_IMEM, a)
        assert got == 0x3C00 + i, f"IMEM[{a:#04x}] = {got:#06x}, aliased?"
    await host.write(SP_IMEM, IMEM_WORDS - 1, 0x0FF0)
    assert await host.read1(SP_IMEM, IMEM_WORDS - 1) == 0x0FF0

    # Thread 0 from reset vector 0 jumps to the top words; thread 3 starts at
    # its default vector 192. Both run at once, so the array serves fetches
    # in consecutive slots. load_program reads every word back.
    top = IMEM_WORDS - 3
    await host.load_program({
        0x00: asm("LDI", rd=1, imm=0x5A),
        0x01: asm("JMP", abs=top),
        top: asm("LDIH", rd=1, imm=0xA5),
        top + 1: asm("HALT"),
        192: asm("LDI", rd=2, imm=0x33),
        193: asm("LDIH", rd=2, imm=0xC3),
        194: asm("HALT"),
    })
    await host.run(0b1001)
    await host.wait_halted(0b1001)
    assert await host.read_reg(0, 1) == 0xA55A
    assert await host.read_reg(3, 2) == 0xC333
    assert await host.read_debug(0, DBG_PC) == top + 2
    assert await host.read_debug(3, DBG_PC) == 195


@cocotb.test(skip=ABSENT or SEEDS + SPI_SEEDS <= 0)
async def test_flops_cosim(dut):
    """FLOPS build: random programs in lockstep with the golden model."""
    _require_flops(dut)
    fd = FlopsDut(dut)
    clock = _start_clock(fd)

    def maker(seed, profile, threads, cycles):
        # The build's features come from CAPS (test_cosim's _Build), as in
        # the macro test, so a program means the same thing on both sides.
        def make(avoid, build):
            return generate(seed=seed, threads=threads, imem_words=IMEM_WORDS,
                            profile=profile, cycles=cycles, avoid=avoid, isa=ISA,
                            features=build.features, fifo_depth=build.fifo_depth)
        return make

    backdoor = [(maker(seed, profile, threads, CYCLES),
                 "flops backdoor profile=%s threads=%d" % (profile, threads), CYCLES)
                for seed, profile, threads in _seed_plan(SEEDS, SEED_BASE)]
    spi = [(maker(SEED_BASE + 500 + i, "mixed", 2, SPI_CYCLES),
            "flops spi profile=mixed threads=2", SPI_CYCLES)
           for i in range(SPI_SEEDS)]
    try:
        if backdoor:
            await _run_programs(fd, backdoor)
        if spi:
            await _run_programs(fd, spi, spi=True)
    finally:
        clock.stop()
