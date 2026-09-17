# SPDX-License-Identifier: Apache-2.0
"""End to end: a UART transmitter written in Loom assembly.

This replaces the M0 hard-wired transmitter test. The program is built with
tools.loomisa, loaded over the SPI host port and run on thread 0; the same
decoder as the M0 test (start bit, sample 8 data bits LSB first at the middle
of each bit, check the stop bit) reads "LOOM\\r\\n" back off OUT0.

Timing: TICK_INT = 434 gives 115200 baud from 50 MHz. One bit time is 434
clocks, which is not a multiple of the 4-clock slot, so individual bit edges
alternate between 432 and 436 clocks. Ten bit times are 4340 clocks, which is
a multiple of 4, so the frame-to-frame spacing is exact: the deadline register
keeps the whole message on one grid and nothing accumulates.
"""

import cocotb
from cocotb.triggers import ClockCycles

from spi_host import (
    LoomHost, asm, SP_CTRL, CTRL_PIN_OUT, CSR_TICK_INT, CSR_OUTGRP,
)
from test_timing import watch_pad, OUT0

MESSAGE = b"LOOM\r\n"
TICK_INT = 434                      # 50 MHz / 115200 baud
BIT = TICK_INT                      # one bit time in core clocks
SEND = 0x20                         # address of the send_byte subroutine


def uart_program(message):
    """main: set the tick period and the OUT group, idle high, anchor the
    deadline, then LDI + CALL per byte. send_byte emits one 8N1 frame with
    one WAITD per bit, so every edge is placed by the deadline register."""
    prog = {
        0x00: asm("CSRW", csr=CSR_TICK_INT, ra=1),   # r1 = TICK_INT
        0x01: asm("CSRW", csr=CSR_OUTGRP, ra=4),     # r4 = base 16, count 1
        0x02: asm("SETP", pin=OUT0, val=1),          # idle high
        0x03: asm("SETD", imm=0),                    # TD = NOW: one grid
    }
    addr = 0x04
    for byte in message:
        prog[addr] = asm("LDI", rd=2, imm=byte)
        prog[addr + 1] = asm("CALL", abs=SEND)
        addr += 2
    prog[addr] = asm("WAITD", imm=2)                 # let the last stop finish
    prog[addr + 1] = asm("HALT")

    prog.update({
        SEND + 0: asm("WAITD", imm=1),               # next bit boundary
        SEND + 1: asm("SETP", pin=OUT0, val=0),      # start bit
        SEND + 2: asm("LDI", rd=3, imm=8),
        SEND + 3: asm("WAITD", imm=1),
        SEND + 4: asm("OUT", ra=2),                  # data bit, LSB first
        SEND + 5: asm("SHRI", rd=2, imm=1),
        SEND + 6: asm("DJNZ", rd=3, rel=-4),         # back to SEND + 3
        SEND + 7: asm("WAITD", imm=1),
        SEND + 8: asm("SETP", pin=OUT0, val=1),      # stop bit
        SEND + 9: asm("RET"),
    })
    return prog


def level_at(edges, clock, initial=1):
    level = initial
    for when, value in edges:
        if when > clock:
            break
        level = value
    return level


def decode(edges, bit=BIT, initial=1):
    """Decode 8N1 frames from a list of (clock, level) pad transitions.

    Same algorithm as the M0 test: find the start bit, move 1.5 bit times in,
    sample 8 bits one bit time apart, then check the stop bit.
    """
    falling = [c for c, v in edges if v == 0]
    out = []
    starts = []
    pos = -1
    for c in falling:
        if c <= pos:
            continue                       # inside a frame already decoded
        starts.append(c)
        value = 0
        for i in range(8):
            if level_at(edges, c + bit + bit // 2 + i * bit, initial):
                value |= 1 << i
        assert level_at(edges, c + 9 * bit + bit // 2, initial) == 1, \
            f"stop bit low in the frame starting at clock {c}"
        out.append(value)
        pos = c + 9 * bit + bit // 2
    return bytes(out), starts


@cocotb.test()
async def test_uart_tx_message(dut):
    """The firmware UART sends LOOM\\r\\n on OUT0 with exact frame spacing."""
    host = LoomHost(dut)
    await host.start()
    await host.write(SP_CTRL, CTRL_PIN_OUT, 0)
    await host.load_program(uart_program(MESSAGE), verify=True)
    await host.set_reset_pc(0, 0)
    await host.reset_thread(0)
    await host.write_reg(0, 1, TICK_INT)
    await host.write_reg(0, 4, (1 << 5) | OUT0)      # OUTGRP: base 16, count 1
    edges = []
    monitor = cocotb.start_soon(watch_pad(dut, edges))
    await host.run(0b0001)
    await host.wait_halted(0b0001, timeout_slots=4000)
    await ClockCycles(dut.clk, 2 * BIT)
    monitor.cancel()

    received, starts = decode(edges)
    dut._log.info(f"received {received!r} at frame starts {starts}")
    assert received == MESSAGE, f"got {received!r}, expected {MESSAGE!r}"

    # Ten bit times is 4340 clocks, a multiple of the 4-clock slot, so every
    # frame starts exactly one frame time after the previous one.
    assert len(starts) == len(MESSAGE)
    gaps = [b - a for a, b in zip(starts, starts[1:])]
    assert all(g == 10 * BIT for g in gaps), \
        f"frame spacing {gaps}, expected {10 * BIT} clocks each"

    # Individual bit edges sit on the slot grid: each one is 0..3 clocks late
    # against its ideal deadline, and the error never accumulates.
    all_edges = [c for c, _ in edges]
    assert all(c % 4 == (all_edges[0] % 4) for c in all_edges), \
        "every pin write lands at the same phase of the thread's slot"
    # First edge: the idle-high SETP, one bit time before the first start bit
    # (SETD 0 then WAITD 1). Last edge: the final stop bit, 9 bit times into
    # the last frame. So the whole span is 1 + 10*(n-1) + 9 bit times.
    span = all_edges[-1] - all_edges[0]
    ideal = (1 + 10 * (len(MESSAGE) - 1) + 9) * BIT
    assert abs(span - ideal) < 4, f"drift over the message: {span} vs {ideal}"


@cocotb.test()
async def test_uart_idles_high_and_stops(dut):
    """The line idles high before the message and after the last stop bit."""
    host = LoomHost(dut)
    await host.start()
    await host.write(SP_CTRL, CTRL_PIN_OUT, 0)
    await host.load_program(uart_program(b"A"), verify=False)
    await host.set_reset_pc(0, 0)
    await host.reset_thread(0)
    await host.write_reg(0, 1, TICK_INT)
    await host.write_reg(0, 4, (1 << 5) | OUT0)
    edges = []
    monitor = cocotb.start_soon(watch_pad(dut, edges))
    await host.run(0b0001)
    await host.wait_halted(0b0001, timeout_slots=4000)
    await ClockCycles(dut.clk, 3 * BIT)
    monitor.cancel()
    received, starts = decode(edges)
    assert received == b"A", f"got {received!r}"
    # After the final stop bit the line stays high.
    last = starts[-1] + 10 * BIT
    assert all(level_at(edges, last + i) == 1 for i in range(0, 2 * BIT, 37))
    await ClockCycles(dut.clk, 4)
