"""L3-TLC5940 on the model and the RTL: firmware/tlc5940_host.loom, a host
for the TI TLC5940 16-channel grayscale PWM LED driver (D-035).

The program (thread 0: GSCLK = OUT5, BLANK = BIDIR0 with a board pull-up;
thread 1: SIN = OUT0, SCLK = OUT1, XLAT = OUT2, VPRG = OUT3, DCPRG = OUT4)
drives ``tools.protomodels.tlc5940.Tlc5940``, which shifts, latches and
counts as SLVS515D sections 8.3 and 8.4 describe, checks every interval
against the 6.3 timing table, and records each PWM cycle: the GSCLK
pulses, the grayscale and dot correction values it ran on, and each
output's on-time in GSCLK periods and in clocks.

What is expected comes from the data sheet as ``protomodels.tlc5940``
cites it (an output with value GS is on for GS GSCLK periods; 4096 GSCLK
pulses a cycle; XLAT for grayscale only while BLANK is high; the extra
SCLK of 8.4.3) and from the program header and docs/SEMANTICS.md section
2 (one slot is 4 clocks, so the 3-slot GSCLK loop is 12 clocks and a cycle
with no new frame 12,294 slots), never from what the model or the RTL did.

The RTL runs the full 4096-pulse cycle, not a shortened one: three cycles,
about 160,000 clocks with the host's words. Random frames and the model's
own checks run on the model only.
"""

import functools
import random

from tools.loomasm import assemble_text
from tools.loomisa import REPO, load
from tools.protomodels.tlc5940 import (CHANNELS, GS_STEPS, TIMING, Tlc5940,
                                       pwm_hz)
from tools.tests.fw_backend import model_only

ISA = load()
SOURCE = REPO / "firmware" / "tlc5940_host.loom"
CLOCK = 50e6
SLOT = 4                                    # clocks a slot (SEMANTICS 2)
GSCLK_CLOCKS = 3 * SLOT                     # SETP 1, SETP 0, DJNZ (header)
#: A cycle with no new frame, BLANK falling to BLANK falling: SETP BLANK 0
#: and MOV, 4096 three-slot pulses, then SETP BLANK 1, SETD, WAITS and BT.
IDLE_CYCLE = SLOT * (2 + 3 * GS_STEPS + 4)  # 49,176 clocks
POLLS = 200_000
TIMED = ("twh0", "twl0", "twh1", "twl1", "twh2", "twh3", "tsu0", "tsu1",
         "tsu2", "tsu3", "tsu4", "tsu5", "th0", "th1", "th2", "th3", "th4")


@functools.lru_cache(maxsize=None)
def program():
    return assemble_text(SOURCE.read_text(encoding="utf-8"), SOURCE.name,
                         isa=ISA, strict=True)


def host_words(values):
    """The 16 host words of one packet, OUT15 first (program header)."""
    assert len(values) == CHANNELS
    return [values[n] for n in reversed(range(CHANNELS))]


def setup(backend):
    """The program loaded, a TLC5940 on the pins (BLANK's pull-up on the
    board), threads 0 and 1 started together."""
    kwargs = dict(pullups=0x01)
    if backend.name == "model":
        kwargs.update(features=("FIFO",), imem_words=512)
    bench = backend.bench(**kwargs)
    chip = bench.add(Tlc5940(clock=CLOCK))
    loom = backend.loom(bench, isa=ISA)
    loom.load({"words": dict(program().words), "imem_words": 512})
    loom.run([0, 1])                 # one RUN write: the same edge
    return bench, chip, loom


def run_cycles(bench, chip, count, max_cycles):
    assert bench.run_until(lambda: len(chip.cycles) >= count, max_cycles,
                           every=256), (len(chip.cycles), chip.violations[:5])


def clean(bench, chip):
    """No timing miss or protocol error, no line driven both ways."""
    assert chip.violations == []
    assert bench.contention_free()


def check_cycle(cycle, gs, dc):
    """A whole PWM cycle on ``gs`` with ``dc``: 4096 pulses, each output on
    for exactly its value in GSCLK periods and 12 clocks a period."""
    assert cycle.edges == GS_STEPS, cycle
    assert cycle.gs == tuple(gs) and cycle.dc == tuple(dc) and cycle.dcprg == 1
    assert cycle.on_edges == list(gs), cycle
    assert cycle.on_clocks == [GSCLK_CLOCKS * g for g in gs], cycle


def check_timing(chip):
    for name in TIMED:
        assert chip.measured[name], name
        assert min(chip.measured[name]) >= TIMING[name], name
    assert min(chip.measured["t_sclk"]) >= 1e9 / TIMING["f_sclk"]
    assert min(chip.measured["t_gsclk"]) >= 1e9 / TIMING["f_gsclk"]


DC = [63 - 4 * n for n in range(CHANNELS)]                 # 63 .. 3
FRAME_A = [0, 1, 2, 3, 100, 255, 256, 1000, 2047, 2048, 3000, 4000,
           4093, 4094, 4095, 77]
FRAME_B = [4095 - g for g in FRAME_A]


# ------------------------------------------------------------- scenarios
def test_tlc5940_dc_and_two_frames(backend):
    """Dot correction, then frames A and B: BLANK stays high until A is in;
    the DC register gets the 16 host values through the 96-bit mode; A's
    first XLAT completes only at the extra SCLK after it (8.4.3), before
    BLANK falls; cycle 1 shows A and cycles 2 and 3 B, each with 4096
    GSCLK pulses and every output on for exactly its value in GSCLK
    periods (12 clocks each); every XLAT lands while BLANK is high; every
    interval meets the 6.3 table; the PWM cycle with no new frame is
    12,294 slots, 1,016.8 Hz at 50 MHz."""
    bench, chip, loom = setup(backend)
    loom.push(1, host_words(DC), max_polls=POLLS)
    assert bench.run_until(lambda: chip.dc_latches, 6_000, every=64)
    bench.step(2_000)
    assert chip.cycles == [] and chip.cycle is None      # still blank
    assert chip.prev["BLANK"] == 1                       # the pull-up
    loom.push(1, host_words(FRAME_A) + host_words(FRAME_B), max_polls=POLLS)
    run_cycles(bench, chip, 3, 3 * IDLE_CYCLE + 20_000)
    clean(bench, chip)
    assert [dc for _, dc in chip.dc_latches] == [tuple(DC)]
    assert [gs for _, gs in chip.gs_latches] == [tuple(FRAME_A), tuple(FRAME_B)]
    gs_xlats = [x for x in chip.xlats if x[1] == 0]
    assert len(gs_xlats) == 2 and all(blank == 1 for _, _, blank in chip.xlats)
    assert gs_xlats[0][0] < chip.gs_latches[0][0] < chip.cycles[0].start
    for cycle, gs in zip(chip.cycles[:3], (FRAME_A, FRAME_B, FRAME_B)):
        check_cycle(cycle, gs, DC)
    check_timing(chip)
    assert chip.cycles[1].start - chip.cycles[0].start > IDLE_CYCLE  # latched B
    assert chip.cycles[2].start - chip.cycles[1].start == IDLE_CYCLE
    assert round(CLOCK / IDLE_CYCLE, 1) == 1016.8


@model_only("five 49,000-clock PWM cycles; the RTL run covers three")
def test_tlc5940_idle_cycle_rate(backend):
    """One frame, then nothing: every cycle shows it, and BLANK falls every
    12,294 slots exactly, 1,016.8 Hz at 50 MHz (Equation 9 with
    f(update) = 1,017 Hz)."""
    bench, chip, loom = setup(backend)
    loom.push(1, host_words(DC) + host_words(FRAME_A), max_polls=POLLS)
    run_cycles(bench, chip, 5, 5 * IDLE_CYCLE + 20_000)
    clean(bench, chip)
    for cycle in chip.cycles[:5]:
        check_cycle(cycle, FRAME_A, DC)
    starts = [cy.start for cy in chip.cycles[:5]]
    assert [b - a for a, b in zip(starts, starts[1:])] == [IDLE_CYCLE] * 4
    assert abs(pwm_hz(chip.cycles[:5], CLOCK) - 1016.8) < 0.05


@model_only("a 49,000-clock PWM cycle per frame; the RTL run covers two frames")
def test_tlc5940_random_frames(backend):
    """Random dot correction and four random frames pushed back to back:
    cycle k shows frame k whole (the host keeps INQ fed, so thread 1 shifts
    each frame inside the previous cycle), on-time equal to value."""
    rng = random.Random(5940)
    dc = [rng.randrange(64) for _ in range(CHANNELS)]
    frames = [[rng.randrange(GS_STEPS) for _ in range(CHANNELS)] for _ in range(4)]
    frames[1][3], frames[2][9] = 0, 4095
    bench, chip, loom = setup(backend)
    words = host_words(dc)
    for f in frames:
        words += host_words(f)
    loom.push(1, words, max_polls=POLLS)
    run_cycles(bench, chip, 4, 4 * IDLE_CYCLE + 30_000)
    clean(bench, chip)
    for cycle, gs in zip(chip.cycles[:4], frames):
        check_cycle(cycle, gs, dc)
    assert all(blank == 1 for _, _, blank in chip.xlats)


@model_only("host words above the value bits; covered by the model")
def test_tlc5940_high_bits_ignored(backend):
    """Bits above 5:0 of a DC word and above 11:0 of a GS word are ignored,
    as the program header says."""
    bench, chip, loom = setup(backend)
    loom.push(1, [w | 0xFFC0 for w in host_words(DC)]
              + [w | 0xF000 for w in host_words(FRAME_A)], max_polls=POLLS)
    run_cycles(bench, chip, 1, IDLE_CYCLE + 20_000)
    clean(bench, chip)
    check_cycle(chip.cycles[0], FRAME_A, DC)


# ------------------------------------------------------- the model itself
class _Lines:
    def __init__(self, cycle, values):
        self.cycle, self.values = cycle, values

    def get(self, pad):
        return self.values.get(pad, 0)


class _Driver:
    """Drives a bare Tlc5940 one clock at a time (no Loom)."""

    def __init__(self):
        self.chip = Tlc5940(clock=CLOCK)
        self.state = {self.chip.pads["BLANK"]: 1, self.chip.pads["DCPRG"]: 1}
        self.c = 0
        self.tick()

    def tick(self, n=1, **levels):
        for name, value in levels.items():
            self.state[self.chip.pads[name]] = value
        for _ in range(n):
            self.chip.observe(_Lines(self.c, dict(self.state)))
            self.c += 1

    def shift(self, values, bits):
        for n in reversed(range(CHANNELS)):
            for b in reversed(range(bits)):
                self.tick(2, SIN=(values[n] >> b) & 1)
                self.tick(2, SCLK=1)
                self.tick(2, SCLK=0)

    def xlat(self):
        self.tick(2, XLAT=1)
        self.tick(2, XLAT=0)

    def cycle(self, pulses=GS_STEPS):
        self.tick(2, BLANK=0)
        for _ in range(pulses):
            self.tick(2, GSCLK=1)
            self.tick(2, GSCLK=0)
        self.tick(2, BLANK=1)


def test_tlc5940_model_counts_and_flags():
    """The model: DC through the 96-bit mode, a GS latch that waits for the
    extra SCLK, on-time = value in GSCLK periods, a short cycle cut off by
    BLANK; and the errors it must report: a cycle started before the extra
    SCLK, a GS latch while BLANK is low, data changing on the SCLK edge."""
    d = _Driver()
    d.tick(2, VPRG=1)
    d.shift(DC, 6)
    d.xlat()
    d.tick(2, VPRG=0)
    d.shift(FRAME_A, 12)
    d.xlat()
    assert d.chip.gs is None and d.chip.pending == tuple(FRAME_A)
    d.tick(2, SCLK=1)
    d.tick(2, SCLK=0)
    assert d.chip.gs == tuple(FRAME_A) and d.chip.dc == tuple(DC)
    d.cycle()
    assert d.chip.violations == []
    assert d.chip.cycles[0].on_edges == FRAME_A
    assert d.chip.cycles[0].on_clocks == [4 * g for g in FRAME_A]
    d.cycle(pulses=256)                         # BLANK early: cut short
    assert d.chip.cycles[1].on_edges == [min(g, 256) for g in FRAME_A]
    # Errors: dot correction again, then a cycle with no extra SCLK.
    d.tick(2, VPRG=1)
    d.shift(DC, 6)
    d.xlat()
    d.tick(2, VPRG=0)
    d.shift(FRAME_B, 12)
    d.xlat()
    d.cycle(pulses=4)
    kinds = [v[1] for v in d.chip.violations]
    assert kinds == ["PWM cycle started before the first GS update completed"]
    d.tick(2, BLANK=0)
    d.xlat()                                    # GS latch while lit
    kinds = [v[1] for v in d.chip.violations]
    assert "GS latched while BLANK low" in kinds
    d2 = _Driver()
    d2.tick(1, SIN=1, SCLK=1)                   # same clock: tsu0 = 0 ns
    assert [v[1] for v in d2.chip.violations] == ["tsu0"]
