# SPDX-License-Identifier: Apache-2.0
"""Reusable cocotb SPI master for the Loom host port (docs/HOST_PROTOCOL.md).

Everything the tests do goes through the pins, so the same test set runs on
the RTL and on the gate-level netlist.

    host = LoomHost(dut)
    await host.start()               # reset, clock the pad model
    host.load_program({0: word, 1: word, ...})
    await host.run(0b0001)

Pin map (docs/ARCHITECTURE.md 3.1):
    ui_in[4] CS_n   ui_in[5] SCK   ui_in[6] MOSI   uo_out[7] MISO
    ui_in[3:0] IN0..IN3            ui_in[7] IN4
    uo_out[5:0] OUT0..OUT5         uio[7:0] BIDIR0..7
"""

import os
import sys

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, RisingEdge

# Make `tools.loomisa` importable when cocotb is started from test/ without
# PYTHONPATH (CI runs from the repository root, the WSL dev loop does not).
_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO not in sys.path:
    sys.path.insert(0, _REPO)

CLK_NS = 20                 # 50 MHz
SCK_CLOCKS = 8              # SCK period in core clocks; the protocol minimum

# Host address spaces
SP_CTRL, SP_IMEM, SP_DMEM, SP_FIFO, SP_DEBUG, SP_STEP = 0, 1, 2, 3, 4, 5

# CTRL registers
CTRL_ID, CTRL_VERSION, CTRL_RUN, CTRL_HALTED, CTRL_RESET = 0, 1, 2, 3, 4
CTRL_RESET_PC0 = 0x08
CTRL_IRQ_EN, CTRL_IRQ_STAT, CTRL_IRQ_STAT2 = 0x10, 0x11, 0x12
CTRL_SFLAGS, CTRL_SFLAGS_CLR, CTRL_OD_MASK = 0x13, 0x14, 0x15
CTRL_PIN_OUT, CTRL_PIN_OE, CTRL_PIN_IN = 0x16, 0x17, 0x18
CTRL_CAPS, CTRL_BADOP, CTRL_SWIRQ = 0x19, 0x1A, 0x1B
CTRL_IRQ_EN2 = 0x1C

# CAPS bits (docs/SEMANTICS.md 5)
CAPS_FIFO, CAPS_BE, CAPS_DMEM = 0x08, 0x10, 0x20
CAPS_ROM, CAPS_LAT, CAPS_AUTO = 0x40, 0x80, 0x100

# FIFO space (docs/HOST_PROTOCOL.md SPACE 3)
FIFO_QUEUE = 0x0000                   # + t: write pushes INQ[t], read pops OUTQ[t]
FIFO_STATUS = 0x0100                  # + t: status word

# DEBUG registers (docs/INTERFACES.md documents the ones above 0x20)
DBG_R0 = 0x00
DBG_PC, DBG_FLAGS, DBG_TD, DBG_NOW = 0x08, 0x09, 0x0A, 0x0B
DBG_RS0 = 0x0F
DBG_CSR0 = 0x10                       # 0x10..0x1F are CSR 0x00..0x0F
DBG_STEPS, DBG_RS1_DEPTH, DBG_WAIT_ACTIVE, DBG_DT = 0x20, 0x21, 0x22, 0x23
DBG_SR, DBG_CNT, DBG_CRC = 0x0C, 0x0D, 0x0E
DBG_TICK_SEEN, DBG_LATCH, DBG_FIFO_COUNTS = 0x24, 0x25, 0x26

CSR_TICK_INT, CSR_TICK_FRAC, CSR_OUTGRP, CSR_INGRP = 0x00, 0x01, 0x02, 0x03
CSR_NOW, CSR_TD, CSR_FLAGS, CSR_TID = 0x09, 0x0A, 0x0B, 0x0C
CSR_OD_MASK, CSR_PIN_OUT, CSR_PIN_OE = 0x10, 0x11, 0x12
CSR_PIN_IN, CSR_SFLAGS, CSR_HOST_IRQ = 0x13, 0x14, 0x15
CSR_BE_CFG, CSR_BE_PINS, CSR_BE_RELOAD = 0x04, 0x05, 0x06
CSR_CRC_POLY, CSR_CRC_INIT = 0x07, 0x08
CSR_SR, CSR_CNT, CSR_CRC = 0x0D, 0x0E, 0x0F

FLAG_Z, FLAG_C, FLAG_T = 1, 2, 4


def resolve(sig, default=0):
    """int(sig) but tolerant of X/Z bits (gate level before initialisation)."""
    try:
        return int(sig.value)
    except (ValueError, TypeError):
        text = str(sig.value)
        return int("".join("0" if ch not in "01" else ch for ch in text), 2) \
            if text else default


class PadMonitor:
    """Samples every pad in the middle of every clock cycle (falling edge).

    Entry ``c`` describes cycle ``c``, counted from ``start()``: ``ui[c]`` is
    what the design samples at the rising edge that ends the cycle (the
    testbench only changes inputs right after rising edges), and ``uo[c]``,
    ``uio[c]``, ``oe[c]`` are the registered outputs during the cycle. So a
    pad register loaded at edge ``e`` first shows its new value in entry
    ``e``, and an input level first seen in entry ``c`` is taken by the first
    synchroniser flop at edge ``c + 1``. The numbering is this monitor's own;
    tests use differences, or align on an event they can compute.
    """

    def __init__(self, dut):
        self.dut = dut
        self.ui, self.uo, self.uio, self.oe = [], [], [], []
        self._task = None

    def start(self):
        self._task = cocotb.start_soon(self._run())
        return self

    def stop(self):
        if self._task is not None:
            self._task.cancel()
            self._task = None

    async def _run(self):
        dut = self.dut
        while True:
            await FallingEdge(dut.clk)
            self.ui.append(resolve(dut.ui_in))
            self.uo.append(resolve(dut.uo_out))
            self.uio.append(resolve(dut.uio_out))
            self.oe.append(resolve(dut.uio_oe))

    @property
    def now(self):
        """Index of the most recent sampled cycle."""
        return len(self.ui) - 1

    @staticmethod
    def changes(seq, bit, start=0):
        """[(cycle, level)] for every change of ``bit`` in ``seq`` from ``start``."""
        out = []
        for c in range(max(start, 1), len(seq)):
            a, b = (seq[c - 1] >> bit) & 1, (seq[c] >> bit) & 1
            if a != b:
                out.append((c, b))
        return out

    def rises(self, seq, bit, start=0):
        return [c for c, v in self.changes(seq, bit, start) if v]

    def sck_rises(self, start=0):
        """First cycle of every SCK high phase (ui_in[5]) since ``start``."""
        return self.rises(self.ui, 5, start)

    @staticmethod
    def host_commit(sck_rise_cycle):
        """First cycle in which the effect of a host word is visible, given
        the cycle in which the SCK level of its last bit first appears.

        The host write commit rule (docs/spec-questions/rtl-m2.md 7): with E
        the first clock edge at which the first synchroniser flop samples
        SCK high for the word's last bit, every effect of the word (a write
        in any space, the pop of a FIFO read word, BADOP[14]) is registered
        at edge E + 4. This testbench changes inputs right after a rising
        edge, so a level first seen in entry c is taken by the first flop at
        edge E = c + 1 (measured on the RTL). The second flop has it at
        c + 2, the edge detector fires during cycle c + 2, loom_spi_host
        raises byte_done at edge c + 3, loom_host_ctl registers the pulse or
        write strobe at edge c + 4 and the target register loads at edge
        E + 4 = c + 5, first visible in entry c + 5. (A DEBUG write of
        r0..r7 can wait up to three more edges for the register-file port.)
        """
        return sck_rise_cycle + 5

    async def align(self, residue, modulo=4):
        """Wait (at least one cycle) until ``now % modulo == residue``."""
        await RisingEdge(self.dut.clk)
        while (self.now + 1) % modulo != residue % modulo:
            await RisingEdge(self.dut.clk)


class LoomHost:
    """SPI master plus a small pad model for the bidirectional pins."""

    def __init__(self, dut, sck_clocks=SCK_CLOCKS):
        self.dut = dut
        self.sck_clocks = sck_clocks
        self.half = max(1, sck_clocks // 2)
        self.ui_ext = 0                 # IN0..IN3 in [3:0], IN4 in bit 7
        self.uio_ext = 0                # what the outside world drives on uio
        self.uio_pull = 0               # bits with an external pull-up
        self._cs = 1
        self._sck = 0
        self._mosi = 0

    # ------------------------------------------------------------- pad model
    def _ui_value(self):
        v = self.ui_ext & 0x8F
        v |= (self._cs & 1) << 4
        v |= (self._sck & 1) << 5
        v |= (self._mosi & 1) << 6
        return v

    def _drive_ui(self):
        self.dut.ui_in.value = self._ui_value()

    def set_in(self, index, value):
        """Drive pin index 8..12 (IN0..IN4) from the testbench."""
        bit = {8: 0, 9: 1, 10: 2, 11: 3, 12: 7}[index]
        if value:
            self.ui_ext |= 1 << bit
        else:
            self.ui_ext &= ~(1 << bit) & 0xFF
        self._drive_ui()

    def set_uio_ext(self, value, pull=None):
        """Value the outside world drives onto uio (seen only where oe == 0).

        tb.v does the loopback for the bits the design drives; `pull` marks
        bits with an external pull-up, which simply read 1 when released.
        """
        self.uio_ext = value & 0xFF
        if pull is not None:
            self.uio_pull = pull & 0xFF
        self.dut.uio_drv.value = self.uio_ext | self.uio_pull

    # ------------------------------------------------------------- lifecycle
    async def start(self, reset_cycles=10):
        # impl="py" keeps the clock a normal cocotb task. The simulator-side
        # (GPI) clock is not torn down when a test ends, so with one host per
        # test the drivers pile up and Icarus eventually crashes on exit.
        try:
            clock = Clock(self.dut.clk, CLK_NS, unit="ns", impl="py")
        except TypeError:                      # cocotb without the impl option
            clock = Clock(self.dut.clk, CLK_NS, unit="ns")
        cocotb.start_soon(clock.start())
        self.dut.ena.value = 1
        self._drive_ui()
        self.dut.uio_drv.value = 0
        self.dut.rst_n.value = 0
        await ClockCycles(self.dut.clk, reset_cycles)
        self.dut.rst_n.value = 1
        await ClockCycles(self.dut.clk, 4)

    # -------------------------------------------------------------- bit layer
    def _miso(self):
        return (resolve(self.dut.uo_out) >> 7) & 1

    async def transfer(self, data, tail_bits=0, tail=0):
        """One CS-framed transaction. Returns the bytes shifted out on MISO.

        With ``tail_bits`` (1..7) the transaction ends mid-byte: after the
        whole bytes of ``data``, only the first ``tail_bits`` bits of ``tail``
        are clocked before CS_n rises, which HOST_PROTOCOL says voids the
        partial byte and any word it belonged to.
        """
        clk = self.dut.clk
        self._cs = 0
        self._drive_ui()
        await ClockCycles(clk, 4)
        out = bytearray()
        frames = [(byte, 8) for byte in data]
        if tail_bits:
            frames.append((tail, tail_bits))
        for byte, nbits in frames:
            rx = 0
            for i in range(nbits):
                self._mosi = (byte >> (7 - i)) & 1
                self._drive_ui()
                await ClockCycles(clk, self.half)
                rx = (rx << 1) | self._miso()
                self._sck = 1
                self._drive_ui()
                await ClockCycles(clk, self.sck_clocks - self.half)
                self._sck = 0
                self._drive_ui()
            if nbits == 8:
                out.append(rx)
        await ClockCycles(clk, 8)
        self._cs = 1
        self._drive_ui()
        await ClockCycles(clk, 4)
        return bytes(out)

    # ------------------------------------------------------------ word layer
    async def write(self, space, addr, words):
        if isinstance(words, int):
            words = [words]
        payload = bytearray([0x80 | (space << 4), (addr >> 8) & 0xFF, addr & 0xFF])
        for w in words:
            payload += bytes([(w >> 8) & 0xFF, w & 0xFF])
        await self.transfer(bytes(payload))

    async def read(self, space, addr, count=1):
        payload = bytes([space << 4, (addr >> 8) & 0xFF, addr & 0xFF]) \
            + bytes(1 + 2 * count)
        rx = await self.transfer(payload)
        body = rx[4:]
        return [(body[2 * i] << 8) | body[2 * i + 1] for i in range(count)]

    async def read1(self, space, addr):
        return (await self.read(space, addr, 1))[0]

    # ----------------------------------------------------------- convenience
    async def load_program(self, program, verify=True):
        """program: {address: word}. Written in ascending contiguous runs."""
        items = sorted(program.items())
        runs = []
        for addr, word in items:
            if runs and addr == runs[-1][0] + len(runs[-1][1]):
                runs[-1][1].append(word)
            else:
                runs.append((addr, [word]))
        for addr, words in runs:
            await self.write(SP_IMEM, addr, words)
        if verify:
            for addr, words in runs:
                got = await self.read(SP_IMEM, addr, len(words))
                assert got == words, \
                    f"IMEM readback at {addr:#x}: {got} != {words}"

    async def run(self, mask):
        await self.write(SP_CTRL, CTRL_RUN, mask)

    async def halt(self):
        await self.write(SP_CTRL, CTRL_RUN, 0)

    async def step(self, thread):
        await self.write(SP_STEP, thread, 1)

    async def reset_thread(self, thread):
        await self.write(SP_CTRL, CTRL_RESET, 1 << thread)

    async def set_reset_pc(self, thread, pc):
        await self.write(SP_CTRL, CTRL_RESET_PC0 + thread, pc)

    async def read_debug(self, thread, reg):
        return await self.read1(SP_DEBUG, (thread << 8) | reg)

    async def write_debug(self, thread, reg, value):
        await self.write(SP_DEBUG, (thread << 8) | reg, value)

    async def read_reg(self, thread, n):
        return await self.read_debug(thread, DBG_R0 + n)

    async def write_reg(self, thread, n, value):
        await self.write_debug(thread, DBG_R0 + n, value)

    async def read_csr(self, thread, csr):
        return await self.read_debug(thread, DBG_CSR0 + csr)

    async def write_csr(self, thread, csr, value):
        await self.write_debug(thread, DBG_CSR0 + csr, value)

    async def halted(self):
        return await self.read1(SP_CTRL, CTRL_HALTED)

    async def badop(self):
        return await self.read1(SP_CTRL, CTRL_BADOP)

    async def caps(self):
        return await self.read1(SP_CTRL, CTRL_CAPS)

    async def clear_badop(self, mask=0xFFFF):
        await self.write(SP_CTRL, CTRL_BADOP, mask)

    async def wait_halted(self, mask, timeout_slots=4000):
        """Poll HALTED until every thread in `mask` has halted."""
        for _ in range(timeout_slots):
            if (await self.read1(SP_CTRL, CTRL_HALTED)) & mask == mask:
                return True
            await ClockCycles(self.dut.clk, 40)
        raise AssertionError(f"threads {mask:#x} did not halt")


async def run_program(host, program, thread=0, start=0, mask=None,
                      timeout_slots=4000, verify=True):
    """Load a program, point one thread at it, run it, and wait for HALT."""
    await host.load_program(program, verify=verify)
    await host.set_reset_pc(thread, start)
    await host.reset_thread(thread)
    await host.run(mask if mask is not None else (1 << thread))
    await host.wait_halted(1 << thread, timeout_slots)


# --------------------------------------------------------------- ISA helpers
from tools.loomisa import load as _load_isa        # noqa: E402

ISA = _load_isa()


def asm(name, **operands):
    """Encode one instruction through tools.loomisa: never a literal hex word."""
    return ISA.encode(name, **operands)


async def run_snippet(host, instrs, thread=0, start=0, regs=None, csrs=None,
                      timeout_slots=4000):
    """Load `instrs` (a HALT is appended) at `start`, preset registers and
    CSRs through the DEBUG space, run the thread and wait until it halts."""
    words = list(instrs) + [asm("HALT")]
    await host.load_program({start + i: w for i, w in enumerate(words)},
                            verify=False)
    if getattr(host, "_rpc_cache", {}).get(thread) != start:
        await host.set_reset_pc(thread, start)
        host._rpc_cache = getattr(host, "_rpc_cache", {})
        host._rpc_cache[thread] = start
    await host.reset_thread(thread)
    for n, v in (regs or {}).items():
        await host.write_reg(thread, n, v)
    for n, v in (csrs or {}).items():
        await host.write_csr(thread, n, v)
    await host.run(1 << thread)
    await host.wait_halted(1 << thread, timeout_slots)
