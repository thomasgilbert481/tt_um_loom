"""End to end: a UART transmitter in Loom assembly, decoded from the pads.

Check IDs: L3-UART-TX and L2-DEADLINE.  The firmware is written with
``tools.loomisa.encode``, runs on the golden model, and the only thing the test
looks at afterwards is the OUT0 pad trace, decoded by the small 8N1 receiver
below.  The assertion that matters is the frame spacing: consecutive start bits
are exactly ten bit times apart even though the per-bit code takes a different
number of slots for a one than for a zero.
"""

import pytest

from tools.loomisa import load
from tools.loomsim import Machine
from tools.loomsim.harness import PadTrace

ISA = load()

TICK_INT = 434              # 115200 baud at 50 MHz
BIT_CLOCKS = TICK_INT       # one tick per bit, so one bit is 434 clocks
MESSAGE = b"LOOM\r\n"
OUT0 = 16
TX = 0x020                  # address of the transmit subroutine


def build_uart_image(message=MESSAGE):
    """A UART transmitter: set up the tick, then CALL the bit loop per byte."""
    words = {}
    main = [
        ("LDI", dict(rd=0, imm=TICK_INT & 0xFF)),
        ("LDIH", dict(rd=0, imm=TICK_INT >> 8)),
        ("CSRW", dict(csr=0x00, ra=0)),           # TICK_INT = 434
        ("SETP", dict(pin=OUT0, val=1)),          # idle high
        ("SETD", dict(imm=2)),                    # anchor the deadline
    ]
    for byte in message:
        main.append(("LDI", dict(rd=1, imm=byte)))
        main.append(("CALL", dict(abs=TX)))
    main.append(("WAITD", dict(imm=2)))           # let the last stop bit finish
    main.append(("HALT", {}))

    # Every SETP sits exactly three slots after the WAITD that timed it, so the
    # start bit, the data bits and the stop bit all land on the same grid.
    transmit = [
        ("WAITD", dict(imm=1)),                   # +0: next bit boundary
        ("NOP", {}),                              # +1
        ("NOP", {}),                              # +2
        ("SETP", dict(pin=OUT0, val=0)),          # +3: start bit
        ("LDI", dict(rd=2, imm=8)),               # +4: eight data bits
        ("WAITD", dict(imm=1)),                   # +5: bit loop
        ("SHRI", dict(rd=1, imm=1)),              # +6: C = the outgoing bit
        ("BC", dict(rel=2)),                      # +7: -> +10
        ("SETP", dict(pin=OUT0, val=0)),          # +8: send a zero
        ("JMP", dict(abs=TX + 11)),               # +9: -> +11
        ("SETP", dict(pin=OUT0, val=1)),          # +10: send a one
        ("DJNZ", dict(rd=2, rel=-7)),             # +11: -> +5
        ("WAITD", dict(imm=1)),                   # +12
        ("NOP", {}),                              # +13
        ("NOP", {}),                              # +14
        ("SETP", dict(pin=OUT0, val=1)),          # +15: stop bit
        ("RET", {}),                              # +16
    ]
    for offset, (name, operands) in enumerate(main):
        words[offset] = ISA.encode(name, **operands)
    for offset, (name, operands) in enumerate(transmit):
        words[TX + offset] = ISA.encode(name, **operands)
    return words


def decode_uart(levels, bit_clocks):
    """A minimal 8N1 receiver over a per-cycle list of line levels.

    Returns ``(start_cycle, byte, stop_bit)`` for every frame found after the
    line has first been idle high.  Each bit is sampled at its centre, which is
    what a real receiver does and what makes the test insensitive to the slot
    quantisation of the transmitter's edges.
    """
    frames = []
    index = 0
    count = len(levels)
    while index < count and levels[index] == 0:       # wait for idle
        index += 1
    while index < count:
        while index < count and levels[index] == 1:   # look for a start bit
            index += 1
        if index >= count:
            break
        start = index
        centres = [start + (2 * bit_clocks * k + bit_clocks) // 2 for k in range(10)]
        if centres[-1] >= count:
            break
        value = 0
        for bit in range(8):
            if levels[centres[bit + 1]]:
                value |= 1 << bit
        frames.append((start, value, levels[centres[9]]))
        index = centres[9]
    return frames


def run_uart(message=MESSAGE):
    trace = PadTrace()
    machine = Machine(build_uart_image(message), isa=ISA, on_cycle=trace)
    machine.host_set_run(0b0001)
    for _ in range(60000):
        machine.step_cycle()
        if machine.halted & 1:
            break
    assert machine.halted & 1, "the transmitter did not finish"
    machine.run_cycles(2 * BIT_CLOCKS)            # trailing idle for the decoder
    return machine, trace


def test_uart_sends_the_message():
    machine, trace = run_uart()
    frames = decode_uart(trace.uo_bit(0), BIT_CLOCKS)
    assert bytes(value for _, value, _ in frames) == MESSAGE
    assert all(stop == 1 for _, _, stop in frames)
    assert machine.badop == 0


def test_uart_frames_are_exactly_ten_bit_times_apart():
    """The deadline register makes the frame period exact, not approximate."""
    _, trace = run_uart()
    frames = decode_uart(trace.uo_bit(0), BIT_CLOCKS)
    starts = [start for start, _, _ in frames]
    spacing = [b - a for a, b in zip(starts, starts[1:])]
    assert spacing == [10 * BIT_CLOCKS] * (len(MESSAGE) - 1)


def test_uart_bit_edges_land_within_one_slot_of_the_ideal_grid():
    """Every edge sits on the bit grid to within the four-clock slot quantum."""
    _, trace = run_uart()
    levels = trace.uo_bit(0)
    frames = decode_uart(levels, BIT_CLOCKS)
    edges = [cycle for cycle, _ in trace.edges(levels)]
    start = frames[0][0]
    for cycle in edges:
        if cycle < start:
            continue                              # the initial idle-high edge
        offset = (cycle - start) % BIT_CLOCKS
        error = min(offset, BIT_CLOCKS - offset)
        assert error <= 4, (cycle, error)


def test_uart_line_is_idle_high_before_the_first_start_bit():
    _, trace = run_uart()
    levels = trace.uo_bit(0)
    frames = decode_uart(levels, BIT_CLOCKS)
    start = frames[0][0]
    assert levels[start - 1] == 1
    assert levels[start] == 0
    # The line was driven high well before the first frame.
    assert sum(levels[:start]) > BIT_CLOCKS


def test_uart_uses_the_return_stack_and_never_overflows_it():
    machine = Machine(build_uart_image(b"Hi"), isa=ISA)
    machine.host_set_run(0b0001)
    calls = rets = 0
    depths = set()
    for _ in range(60000):
        record = machine.step_cycle()
        if record is not None:
            calls += record.mnemonic == "CALL"
            rets += record.mnemonic == "RET"
        depths.add(machine.threads[0].depth)
        if machine.halted & 1:
            break
    assert (calls, rets) == (2, 2)
    assert depths == {0, 1}                        # one level, never overflowing


def test_uart_timing_is_unaffected_by_another_thread():
    """ISO-1 end to end: thread 1 hammering the pipeline moves no UART edge."""
    _, quiet = run_uart(b"Lo")
    quiet_edges = quiet.edges(quiet.uo_bit(0))

    image = build_uart_image(b"Lo")
    image[0x100] = ISA.encode("JMP", abs=0x100)
    busy = PadTrace()
    machine = Machine(image, isa=ISA, on_cycle=busy)
    machine.host_set_run(0b0011)
    for _ in range(60000):
        machine.step_cycle()
        if machine.halted & 1:
            break
    machine.run_cycles(2 * BIT_CLOCKS)
    assert busy.edges(busy.uo_bit(0)) == quiet_edges


@pytest.mark.parametrize("message", [b"\x00", b"\xFF", b"\x55\xAA"])
def test_uart_handles_the_edge_case_bytes(message):
    _, trace = run_uart(message)
    frames = decode_uart(trace.uo_bit(0), BIT_CLOCKS)
    assert bytes(value for _, value, _ in frames) == message
