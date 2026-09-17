# SPDX-License-Identifier: Apache-2.0
# Loom M0 test: decode the hard-wired UART transmitter on OUT0.
#
# Check IDs (docs/VERIFICATION.md): this is the M0 smoke test that later
# becomes L3-UART-TX once the same message is produced by firmware.

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, Timer
from cocotb.utils import get_sim_time

CLK_NS = 20                       # 50 MHz
BAUD = 115_200
DIV = int(50_000_000 // BAUD)     # 434, must match the RTL
BIT_NS = DIV * CLK_NS             # 8680 ns, exact for the RTL divider
EXPECTED = b"LOOM\r\n"


def tx_level(dut) -> int:
    return int(dut.uo_out.value) & 1


async def reset(dut):
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)


async def uart_rx_byte(dut, timeout_bits=40):
    """Wait for a start bit, sample 8 data bits LSB first, check the stop bit.

    Returns (byte, start_time_ns).
    """
    waited = 0
    while tx_level(dut) == 1:
        await RisingEdge(dut.clk)
        waited += 1
        assert waited < timeout_bits * DIV, "no start bit seen"
    t_start = get_sim_time("ns")
    # move to the middle of bit 0 (1.5 bit periods after the start edge)
    await Timer(BIT_NS + BIT_NS // 2, unit="ns")
    val = 0
    for i in range(8):
        val |= tx_level(dut) << i
        await Timer(BIT_NS, unit="ns")
    assert tx_level(dut) == 1, "stop bit was not high"
    return val, t_start


@cocotb.test()
async def test_idle_high_when_disabled(dut):
    """With IN0 low the line must stay idle (high)."""
    cocotb.start_soon(Clock(dut.clk, CLK_NS, unit="ns").start())
    await reset(dut)
    for _ in range(50):
        await ClockCycles(dut.clk, 100)
        assert tx_level(dut) == 1, "TX went low while disabled"
    assert int(dut.uio_oe.value) == 0
    assert int(dut.uio_out.value) == 0


@cocotb.test()
async def test_message_and_timing(dut):
    """IN0 high: expect 'LOOM\\r\\n' at 115200 8N1 with exact 10-bit frame spacing."""
    cocotb.start_soon(Clock(dut.clk, CLK_NS, unit="ns").start())
    await reset(dut)
    dut.ui_in.value = 1

    received = bytearray()
    starts = []
    for _ in range(len(EXPECTED)):
        b, t0 = await uart_rx_byte(dut)
        received.append(b)
        starts.append(t0)
    dut._log.info(f"received {bytes(received)!r}")
    assert bytes(received) == EXPECTED, f"got {bytes(received)!r}"

    # Frames are back to back: consecutive start edges are exactly 10 bit
    # periods apart, within one clock (the RX loop resamples on clk edges).
    for a, b in zip(starts, starts[1:]):
        gap = b - a
        assert abs(gap - 10 * BIT_NS) <= CLK_NS, f"frame gap {gap} ns"

    # After the message the line idles high for at least one bit period.
    await Timer(BIT_NS // 2, unit="ns")
    for _ in range(DIV // 2):
        await RisingEdge(dut.clk)
        assert tx_level(dut) == 1, "line did not idle after the message"
