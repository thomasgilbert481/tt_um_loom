"""L3-WS2812 through the host port, on the model and the RTL.

``firmware/ws2812.loom`` is assembled, loaded and fed entirely through
``tools.loomhost.Loom``, with ``tools.protomodels.ws2812.Ws2812Rx`` on OUT0
measuring every pulse against the WS2812B datasheet window before it decodes
a single bit. Every body takes a ``backend`` (``tools/tests/fw_backend.py``),
so the same bodies run here under pytest on the golden model and under cocotb
on the RTL (``test/test_fw.py``).

The expected numbers come from the datasheet and from ``docs/SEMANTICS.md``,
never from what the model or the RTL happened to do: at 50 MHz the program
drives T0H = 20 clocks, T1H = 40, period = 64 (so T0L = 44 and T1L = 24), and
SEMANTICS 6.10 says a deadline-latched write lands on the exact clock edge at
which ``NOW`` reaches ``TD``, so every one of those widths is exact, not
within a slot.

The program sets its own ``TICK_INT`` (the datasheet fixes the rate), so the
host only loads it and runs it. It needs the golden model built with the
deadline latch, which the RTL has unconditionally; see :func:`ws_bench`.
"""

import random

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels.bench import Lines
from tools.protomodels.ws2812 import Ws2812Rx

ISA = load()
FIRMWARE = REPO / "firmware"

#: The waveform the program drives, in core clocks at 50 MHz (see its header).
T0H, T1H, PERIOD = 20, 40, 64
T0L, T1L = PERIOD - T0H, PERIOD - T1H
RESET = 2500                                    # 50 us, the datasheet minimum
FRAME_RESET = 2750                              # what the program waits, 55 us


def ws_bench(backend, **kwargs):
    """A bench whose chip has the deadline latch of SEMANTICS 6.10.

    The golden model builds it as the ``SETPD`` feature; the RTL has it
    always (``src/loom_core.v``), and ``RtlBench`` takes no ``features``.
    """
    if backend.name == "model":
        kwargs.setdefault("features", ("FIFO", "SETPD"))
    return backend.bench(**kwargs)


def boot(backend, bench):
    """Load the program through the host port and run it."""
    program = assemble_file(FIRMWARE / "ws2812.loom", isa=ISA, strict=True)
    loom = backend.loom(bench, isa=ISA)
    loom.load(program)
    loom.run(0)
    return loom, program


def setup(backend, **kwargs):
    bench = ws_bench(backend, **kwargs)
    rx = bench.add(Ws2812Rx("OUT0"))
    loom, program = boot(backend, bench)
    return bench, rx, loom


def send(bench, loom, rx, data, frames=1, extra=4000):
    """Push ``data`` and run until ``frames`` frames have latched."""
    loom.push(0, data)
    budget = len(data) * 8 * PERIOD + FRAME_RESET * frames + extra
    assert bench.run_until(lambda: len(rx.frames) >= frames, budget, every=64), \
        "only %d frame(s) after %d clocks: %s" % (len(rx.frames), budget,
                                                  rx.report())
    return rx.frames


def check_timing(rx):
    """Every pulse inside the datasheet window, and exact by SEMANTICS 6.10."""
    assert rx.violations == [], rx.report()
    assert set(rx.high_clocks) <= {T0H, T1H}, sorted(set(rx.high_clocks))
    assert set(rx.low_clocks) <= {T0L, T1L}, sorted(set(rx.low_clocks))
    assert set(rx.periods) <= {PERIOD}, sorted(set(rx.periods))


def colours(count, seed):
    """``count`` LEDs of pseudo-random colour, as (green, red, blue)."""
    rng = random.Random(seed)
    return [(rng.randrange(256), rng.randrange(256), rng.randrange(256))
            for _ in range(count)]


def grb_bytes(leds):
    return [value for led in leds for value in led]


# ---------------------------------------------------------------- the frame
def test_ws2812_drives_eight_leds_of_random_colours(backend):
    """Twenty-four bytes in one frame, every pulse inside the datasheet window."""
    bench, rx, loom = setup(backend)
    leds = colours(8, 20260922)
    send(bench, loom, rx, grb_bytes(leds))
    assert len(rx.frames) == 1
    assert rx.frames[0].leds == leds
    assert rx.frames[0].bits == 24 * 8
    check_timing(rx)
    assert rx.frames[0].reset_clocks >= RESET
    assert loom.badop() == 0


def test_ws2812_bit_widths_are_clock_exact(backend):
    """0x00 and 0xFF: the two extreme bytes, and both widths, measured."""
    bench, rx, loom = setup(backend)
    send(bench, loom, rx, [0xFF, 0x00, 0xA5])
    frame = rx.frames[0]
    assert frame.data == b"\xff\x00\xa5"
    check_timing(rx)
    # 8 ones, 8 zeros, then 1010 0101: the widths in the order they went out
    expected = [T1H] * 8 + [T0H] * 8 + [T1H if b == "1" else T0H
                                        for b in "10100101"]
    assert rx.high_clocks == expected
    # every low but the last is the complement of its bit inside a 64-clock
    # period; the last one is the reset
    lows = [T1L if h == T1H else T0L for h in expected[:-1]]
    assert rx.low_clocks == lows
    assert rx.periods == [PERIOD] * 23


def test_ws2812_ignores_the_high_byte_of_a_word(backend):
    """Bits 15:8 are not sent: SWAP moves them below the eight shifts."""
    bench, rx, loom = setup(backend)
    send(bench, loom, rx, [0xFF41, 0x0142, 0x8043])
    assert rx.frames[0].data == b"ABC"
    check_timing(rx)


# --------------------------------------------------------------- the reset
def test_ws2812_a_reset_separates_two_frames(backend):
    """INQ empty at a byte boundary ends the frame; the next one waits 50 us."""
    bench, rx, loom = setup(backend)
    first = colours(1, 1)
    send(bench, loom, rx, grb_bytes(first))
    second = colours(1, 2)
    send(bench, loom, rx, grb_bytes(second), frames=2)
    assert [f.leds for f in rx.frames] == [first, second]
    check_timing(rx)
    assert rx.resets and all(r >= RESET for r in rx.resets), rx.resets
    # the gap the program leaves is measured from the last falling edge and
    # is the one the strip latches on
    assert rx.frames[1].start - rx.frames[0].end >= FRAME_RESET


def test_ws2812_bytes_queued_during_a_frame_extend_it(backend):
    """The wire format is a byte stream: back-to-back pushes are one frame."""
    bench, rx, loom = setup(backend)
    leds = colours(4, 3)
    loom.push(0, grb_bytes(leds[:2]))
    loom.push(0, grb_bytes(leds[2:]))            # queued while the first runs
    assert bench.run_until(lambda: rx.frames, 12 * 8 * PERIOD + 8000, every=64)
    assert rx.frames[0].leds == leds             # one frame of four LEDs
    assert len(rx.resets) == 1                   # no reset inside it
    check_timing(rx)


def test_ws2812_an_empty_line_is_never_driven_high(backend):
    """Nothing pushed: the pad stays low, whatever the program is doing."""
    bench, rx, loom = setup(backend)
    bench.step(4000)
    assert rx.high_clocks == [] and rx.frames == []
    assert bench.lines.uo & 1 == 0
    assert loom.badop() == 0


# ------------------------------------------------------------- the program
def test_ws2812_assembles_strict_with_no_diagnostic():
    """L2-DEADLINE for ws2812.loom: every pair proved, nothing unbounded.

    The two byte-fetch ``POP``s carry a ``.bounded`` declaration
    (``tools/loomasm/README.md`` sections 4 and 7), so the checker counts
    them as one slot each and analyses their intervals like any other. Those
    two intervals are the tightest in the program: 10 slots against a
    44-clock budget and 9 against 40, one slot of slack each. A declaration
    is an assumption and not a proof, so this also pins down that there are
    exactly two of them and what they are attached to.
    """
    program = assemble_file(FIRMWARE / "ws2812.loom", isa=ISA, strict=True)
    assert program.diagnostics == []
    report = program.deadlines[0]
    assert report.period == 1                     # TICK_INT 1: one tick, one clock
    assert not report.infeasible and not report.unbounded
    assert len(report.pairs) == 19
    assert report.worst_slack == 4                # one slot, on the two fetches
    assert [(d.name, d.addr) for d in report.declarations] == \
        [("POP", 0x019), ("POP", 0x027)]
    assert all("only this thread pops INQ" in d.reason
               for d in report.declarations)
    assert sorted(program.bounded) == [0x019, 0x027]
    assert list(program.threads) == [0]
    assert max(program.words) < 512 // 4          # D-020: thread 0 owns 0..127


# --------------------------------------------------- the decoder itself
def feed(rx, pattern):
    """Drive a ``(level, clocks)`` waveform through the decoder, cycle by cycle."""
    lines = Lines()
    cycle = 0
    for level, clocks in pattern:
        for _ in range(clocks):
            lines.cycle = cycle
            lines.uo = level                          # OUT0 is uo[0]
            rx.observe(lines)
            cycle += 1
    return cycle


def frame_pattern(data, *, high=None, low=None, tail=RESET + 100):
    """One LED's worth of bytes on the wire, MSB first, then a reset."""
    out = [(0, 100)]
    for byte in data:
        for i in range(7, -1, -1):
            one = (byte >> i) & 1
            out.append((1, high if high is not None else (T1H if one else T0H)))
            out.append((0, low if low is not None else (T1L if one else T0L)))
    out[-1] = (0, tail)
    return out


def test_ws2812_decoder_accepts_a_clean_frame():
    rx = Ws2812Rx("OUT0")
    feed(rx, frame_pattern(b"\xa5\x10\x00"))
    assert rx.violations == []
    assert [f.data for f in rx.frames] == [b"\xa5\x10\x00"]
    assert rx.frames[0].leds == [(0xA5, 0x10, 0x00)]
    # the frame closes the moment the strip would latch, not when the line
    # eventually rises again, so a longer idle is still one 50 us reset
    assert rx.resets == [RESET]


@pytest.mark.parametrize("high", [T0H - 8, (T0H + T1H) // 2, T1H + 8])
def test_ws2812_decoder_rejects_a_high_pulse_outside_both_windows(high):
    """0.24, 0.60 and 0.96 us: too short, between the two windows, too long."""
    rx = Ws2812Rx("OUT0")
    feed(rx, frame_pattern(b"\xff\xff\xff", high=high, low=T1L))
    assert rx.violations and rx.violations[0].kind == "high", rx.report()
    assert sum(v.kind == "high" for v in rx.violations) == 24


def test_ws2812_decoder_rejects_a_gap_too_short_to_be_a_reset():
    """4 us of low: no bit has one, and the strip would not latch on it."""
    rx = Ws2812Rx("OUT0")
    feed(rx, frame_pattern(b"\xff\xff\xff", tail=200)
         + frame_pattern(b"\x00\x00\x00")[1:])
    assert [v.kind for v in rx.violations] == ["low"], rx.report()
    assert "reset" in rx.violations[0].detail
    assert [f.data for f in rx.frames] == [b"\xff\xff\xff", b"\x00\x00\x00"]
