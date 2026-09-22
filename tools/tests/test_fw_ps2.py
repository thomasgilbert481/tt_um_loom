"""L3-PS2-HOST through the host port, on the model and the RTL.

``firmware/ps2_host.loom`` is assembled, loaded and read entirely through
``tools.loomhost.Loom``, with ``tools.protomodels.ps2.Ps2Keyboard`` clocking
frames onto CLK (IN0) and DATA (IN1) and ``Ps2Monitor`` recording what was
really on the wire. Every body takes a ``backend``
(``tools/tests/fw_backend.py``), so the same bodies run here under pytest on
the golden model and under cocotb on the RTL (``test/test_fw.py``).

The rates come from the PS/2 standard, not from the implementation: a
device clocks at 10 to 16.7 kHz, which at 50 MHz is 5000 to 3000 core clocks
per bit, and both ends of that band are tested. The timeouts come from the
program's own constants, which are derived from that band: one tick is 64
clocks, a bit times out after 128 ticks (8192 clocks, longer than the 5000
of a 10 kHz period) and a resync ends after 200 ticks (12800 clocks) of
quiet.

A frame is eleven bits, so one frame is 33000 clocks at the fast end and
55000 at the slow one, which is why the scenarios are short: three frames
is already 99000 simulated clocks.
"""

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels.ps2 import (CLOCKS_10K, CLOCKS_16K7, Ps2Keyboard,
                                   Ps2Monitor, odd_parity)
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"

FAST, SLOW = CLOCKS_16K7, CLOCKS_10K        # 16.67 kHz and 10 kHz at 50 MHz
TICKC = 64                                  # the program's tick, in clocks
TMO = 128 * TICKC                           # a bit that never comes: 8192
IDLET = 200 * TICKC                         # the resync drain: 12800
PARITY_ERR, FRAME_ERR = 0x100, 0x200        # the result bits

MAKE, BREAK = 0x1C, 0xF0                    # scancode set 2: "A", and break


def setup(backend, period_clocks=FAST):
    """Load the program, run it, and put a device on the two pins."""
    bench = backend.bench()
    kbd = bench.add(Ps2Keyboard("IN0", "IN1", period_clocks=period_clocks))
    mon = bench.add(Ps2Monitor("IN0", "IN1"))
    program = assemble_file(FIRMWARE / "ps2_host.loom", isa=ISA, strict=True)
    loom = backend.loom(bench, isa=ISA)
    loom.load(program)
    loom.run(0)
    return bench, kbd, mon, loom


def clock_out(bench, kbd, settle=400):
    """Run until the device has finished clocking everything queued."""
    limit = kbd.idle_at() - bench.cycle + settle + 1000
    assert bench.run_until(lambda: not kbd.busy, limit, every=64)
    bench.step(settle)


# ------------------------------------------------------------- good frames
def test_ps2_receives_a_make_and_a_break(backend):
    """Three frames at the fastest legal clock, 16.67 kHz."""
    bench, kbd, mon, loom = setup(backend)
    kbd.send([MAKE, BREAK, MAKE])
    clock_out(bench, kbd)
    assert loom.pop(0, 3) == [MAKE, BREAK, MAKE]
    assert len(mon.bits) == 3 * 11                  # eleven edges per frame
    assert [f[0] for f in mon.frames()] == [0, 0, 0]        # three start bits
    assert loom.pop_available(0) == [] and loom.badop() == 0


def test_ps2_receives_at_the_slowest_clock(backend):
    """10 kHz: a 5000-clock bit, still inside the program's 8192-clock timeout."""
    bench, kbd, mon, loom = setup(backend, period_clocks=SLOW)
    kbd.send([0x66])                                # backspace, make
    clock_out(bench, kbd)
    assert loom.pop(0, 1) == [0x66]
    assert loom.badop() == 0


@model_only("eight frames at 3000 to 5000 clocks a bit is a quarter million clocks")
@pytest.mark.parametrize("period", [FAST, SLOW])
def test_ps2_receives_every_bit_pattern_it_is_given(backend, period):
    bench, kbd, mon, loom = setup(backend, period_clocks=period)
    codes = [0x00, 0xFF, 0x55, 0xAA]
    kbd.send(codes)
    clock_out(bench, kbd)
    assert loom.pop(0, len(codes)) == codes
    # the parity bit really varied with the byte: 0x00 and 0xFF need a 1,
    # 0x55 and 0xAA (four ones) need a 1 as well; check the model's rule
    assert [odd_parity(c) for c in codes] == [1, 1, 1, 1]
    assert [f[9] for f in mon.frames()] == [1, 1, 1, 1]


# ----------------------------------------------------------------- errors
def test_ps2_a_parity_error_is_reported_in_bit_8(backend):
    """One flipped parity bit costs one word, and the next frame is normal."""
    bench, kbd, mon, loom = setup(backend)
    kbd.send_frame(0x5A, bad_parity=True)
    kbd.send_frame(MAKE)
    clock_out(bench, kbd)
    assert loom.pop(0, 2) == [PARITY_ERR | 0x5A, MAKE]
    assert loom.badop() == 0


def test_ps2_a_low_stop_bit_is_reported_in_bit_9(backend):
    """A framing error still delivers the byte, and costs no extra frame."""
    bench, kbd, mon, loom = setup(backend)
    kbd.send_frame(0x3B, stop=0)
    kbd.send_frame(MAKE)
    clock_out(bench, kbd)
    assert loom.pop(0, 2) == [FRAME_ERR | 0x3B, MAKE]
    assert loom.badop() == 0


def test_ps2_a_frame_cut_short_costs_one_word(backend):
    """The device stops after five bits: the timed waits give up, once."""
    bench, kbd, mon, loom = setup(backend)
    kbd.send_frame(0x77, truncate=5)
    clock_out(bench, kbd)
    bench.step(TMO + 4 * 64)                        # the bit timeout expires
    assert loom.pop(0, 1) == [FRAME_ERR]            # bit 9, no byte
    kbd.send_frame(MAKE)
    clock_out(bench, kbd)
    assert loom.pop(0, 1) == [MAKE]                 # back in step
    assert loom.badop() == 0


@model_only("a bad start bit plus a 12800-clock drain plus a good frame")
def test_ps2_an_edge_without_a_start_bit_is_one_word_and_a_resync(backend):
    bench, kbd, mon, loom = setup(backend)
    kbd.send_frame(0x1C, no_start=True)
    clock_out(bench, kbd)
    assert loom.pop(0, 1) == [FRAME_ERR]            # reported once, not eleven times
    bench.step(IDLET + 4 * 64)                      # the drain ends
    kbd.send_frame(MAKE)
    clock_out(bench, kbd)
    assert loom.pop(0, 1) == [MAKE]
    assert loom.badop() == 0


# ------------------------------------------------------------- the program
def test_ps2_host_assembles_strict_with_no_diagnostic():
    """L2-DEADLINE for ps2_host.loom.

    The device owns the clock, so the program has no ``WAITD`` and no
    deadline pair to prove: every wait is either timed with ``T`` (and
    anchored by the ``SETD`` before it) or the deliberate untimed wait for
    the next frame. What the checker must show is that nothing is left
    unbounded between two deadlines, and there is nothing to report.
    """
    program = assemble_file(FIRMWARE / "ps2_host.loom", isa=ISA, strict=True)
    assert program.diagnostics == []
    report = program.deadlines[0]
    assert report.period == TICKC                   # .csr TICK_INT, 64
    assert report.pairs == []
    assert list(program.threads) == [0]
    assert max(program.words) < 512 // 4            # D-020: thread 0 owns 0..127


def test_ps2_model_parity_rule_matches_the_standard():
    """Odd parity over the nine bits, which is what the program checks."""
    for value in (0x00, 0x01, 0x03, 0xFF, 0x1C, 0xF0):
        ones = bin(value).count("1") + odd_parity(value)
        assert ones % 2 == 1
