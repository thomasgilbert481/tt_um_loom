"""L3-BRIDGE on the model and the RTL: firmware/bridge_ir_ws2812.loom.

Two threads of one program, no host in the loop: thread 0 decodes NEC on
IN0 (``tools.protomodels.ir_nec.IrSource`` drives a receiver module's
demodulated output) and applies the command map to one STATE word; thread
1 drives a WS2812B strip on OUT0 from it, and
``tools.protomodels.ws2812.Ws2812Rx`` checks every pulse against the
datasheet windows before it decodes a byte. The host loads the image, sets
thread 0's tick and starts both threads; nothing passes through the FIFOs.

Expected values come from the NEC description (the reference model), the
WS2812B datasheet (its model) and the program header (the command map,
the colour table, the level rule, STATE's layout), never from what the
model or the RTL did.

Clocks. Thread 1's timing is in clocks (TICK_INT 1) and is checked as at
50 MHz (``Ws2812Rx(clock_mhz=50)``). Thread 0's tick comes from the host:
at 50 MHz it is 1757 + 208/256 clocks (1/16 of the NEC unit) and a frame
with its repeat is 9 M clocks, so the RTL scenarios use the slow board of
``test_fw_ir_nec``: thread 0's tick 64 + 32/256 and the NEC waveform
generated for 1.824 MHz, 27.4 times fewer clocks. That is the same program
and the same clock-for-clock behaviour of both threads as a 50 MHz chip
with a remote 27 times fast; the 50 MHz bodies run on the model.
"""

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels import ir_nec as N
from tools.protomodels.bench import Model, pad_of
from tools.protomodels.ws2812 import Ws2812Rx
from tools.tests.fw_backend import model_only

ISA = load()
SOURCE = REPO / "firmware" / "bridge_ir_ws2812.loom"
IMEM = 512
PROGRAM = assemble_file(SOURCE, isa=ISA, imem_words=IMEM, strict=True)
SYM = PROGRAM.symbols
DATA = SYM["DATA"]
STATE, NLEDS = DATA, DATA + 1

# The program header: datasheet timing in clocks at 50 MHz, the reset gap.
T0H, T1H, PERIOD, RESET = 20, 40, 64, 2750
LED_CLOCKS = 24 * PERIOD

# The command map (program header): standard NEC, address 0x00.
ADDR = 0x00
DIGITS = [0x16, 0x0C, 0x18, 0x5E, 0x08, 0x1C, 0x5A, 0x42, 0x52, 0x4A]
VOL_UP, VOL_DOWN, PLAY = 0x15, 0x07, 0x43
# The colours, (R, G, B) at level 7, from the header's table.
COLOURS = [(255, 255, 255), (255, 0, 0), (255, 96, 0), (255, 224, 0),
           (0, 255, 0), (0, 255, 255), (0, 0, 255), (128, 0, 255),
           (255, 0, 160), (255, 160, 64)]
ON = 0x80


def state(colour, level, on=True):
    """STATE as the header lays it out."""
    return colour | level << 4 | (ON if on else 0)


def shown(st):
    """The (R, G, B) every LED shows for STATE ``st``: c >> (7 - L), or
    dark when off."""
    if not st & ON:
        return (0, 0, 0)
    level = (st >> 4) & 7
    return tuple(c >> (7 - level) for c in COLOURS[st & 15])


class Board:
    """Thread 0's clock: the tick the header gives for it."""

    def __init__(self, name, clock_hz, tick):
        self.name, self.clock_hz, self.tick = name, float(clock_hz), tick

    def cycles(self, us):
        return int(round(us * self.clock_hz / 1e6))

    def __repr__(self):
        return self.name


B1824 = Board("1824kHz", 1_824_000, (64, 32))
B50 = Board("50MHz", 50_000_000, (1757, 208))


class Edges(Model):
    """Every change of OUT0, as (cycle, level)."""

    def __init__(self, pin="OUT0"):
        self.pad, self.prev, self.log = pad_of(pin), None, []

    def observe(self, lines):
        level = lines.get(self.pad)
        if self.prev is not None and level != self.prev:
            self.log.append((lines.cycle, level))
        self.prev = level


def bridge_bench(backend):
    """The FIFOs, the deadline latch and LD/ST with the 512-word memory on
    the model; the RTL has all of them and ``RtlBench`` takes neither."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "SETPD", "DMEM"), imem_words=IMEM)
    return backend.bench()


def boot(backend, board=B1824, *, leds=8, st=None, gap_us=40_000.0):
    """The bridge with an IR source on IN0 and the strip decoder and an
    edge log on OUT0; NLEDS (and STATE, if given) patched in the image,
    thread 0's tick set, both threads running."""
    bench = bridge_bench(backend)
    src = bench.add(N.IrSource("IN0", board.clock_hz, gap_us=gap_us))
    strip = bench.add(Ws2812Rx("OUT0", clock_mhz=50.0))
    edges = bench.add(Edges())
    words = dict(PROGRAM.words)
    words[NLEDS] = leds
    if st is not None:
        words[STATE] = st
    loom = backend.loom(bench, isa=ISA)
    loom.load(words)
    loom.write_csr(0, "TICK_INT", board.tick[0])
    loom.write_csr(0, "TICK_FRAC", board.tick[1])
    loom.run([0, 1], exclusive=True)
    src.free_at = bench.cycle + 1000            # the first key starts from here
    return bench, loom, src, strip, edges


# ------------------------------------------------------------------ helpers
def press(src, board, command, *, repeats=0, address=ADDR, pulses=None):
    """One key: its frame (or ``pulses``), then ``repeats`` repeat codes
    108 ms apart from the frame's start, as a held key sends them. Returns
    the cycle the last burst ends."""
    start = src.send(pulses if pulses is not None else N.frame_pulses(address, command))
    for k in range(1, repeats + 1):
        src.send(N.repeat_pulses(), at=start + board.cycles(k * N.PERIOD_US))
    return src.free_at


def settle(bench, src, leds, slack=20000):
    """Run until the IR source is idle and then long enough for the strip
    frame in flight and a whole frame after it, with their resets."""
    end = max(src.free_at, bench.cycle) + 2 * (leds * LED_CLOCKS + RESET) + slack
    bench.step(end - bench.cycle)


def assert_wire(strip, leds):
    """Every pulse inside the datasheet windows (the decoder judges each as
    it ends), and the header's exact figures: highs of 20 or 40 clocks,
    every period 64, at least 2750 clocks low between frames; every frame whole
    (3 * leds bytes) and uniform (one colour on every LED: not torn)."""
    assert strip.violations == [], strip.report()
    assert set(strip.high_clocks) <= {T0H, T1H}, sorted(set(strip.high_clocks))
    assert set(strip.periods) <= {PERIOD}, sorted(set(strip.periods))
    gaps = [b.start - a.end for a, b in zip(strip.frames, strip.frames[1:])]
    assert all(g >= RESET for g in gaps), gaps
    for frame in strip.frames:
        assert len(frame.data) == 3 * leds, len(frame.data)
        assert len(set(frame.rgb)) == 1, "a torn frame: %s" % sorted(set(frame.rgb))


def shown_frames(strip):
    """The colour of each frame, in order."""
    return [frame.rgb[0] for frame in strip.frames]


# --------------------------------------------------------- RTL scenarios
def test_bridge_a_frame_and_its_repeat_change_the_strip_as_mapped(backend):
    """From red at level 4 (patched into STATE), VOL+ held for one repeat
    code: the frame raises the level to 5 and the repeat to 6, each change
    is one strip frame that starts after the burst that caused it ended,
    and there are no other frames (the strip is refreshed only on a
    change)."""
    bench, loom, src, strip, _ = boot(backend, st=state(1, 4))
    frame_end = press(src, B1824, VOL_UP)
    rep_start = src.send(N.repeat_pulses(), at=src.starts[0] + B1824.cycles(N.PERIOD_US))
    settle(bench, src, 8)
    assert_wire(strip, 8)
    assert shown_frames(strip) == [shown(state(1, 4)), shown(state(1, 5)), shown(state(1, 6))]
    assert shown(state(1, 6)) == (127, 0, 0)
    assert strip.frames[0].start < src.starts[0]
    assert frame_end < strip.frames[1].start < rep_start
    assert strip.frames[2].start > rep_start
    assert loom.badop() == 0


def test_bridge_strip_edges_are_the_same_with_and_without_ir_traffic(backend):
    """Both runs load the same image (red at level 4, 96 LEDs: a strip
    frame of 150 k clocks) and make the same host transactions; in the
    second a whole NEC frame for key 1 (red, the colour already shown)
    arrives while the strip frame is going out, so thread 0 measures every
    pulse, looks the key up and stores STATE (unchanged) during it. Every
    OUT0 edge falls on the same clock in both runs (thread 1 never waits on
    thread 0: SEMANTICS 2 and 6.11), and the unchanged state sends no
    second frame."""
    leds, run = 96, 96 * LED_CLOCKS + RESET + 8000
    quiet = boot(backend, leds=leds, st=state(1, 4))
    quiet[0].step(run)
    bench, loom, src, strip, edges = boot(backend, leds=leds, st=state(1, 4))
    end = press(src, B1824, DIGITS[1])
    bench.step(run)
    assert not src.busy
    assert strip.frames[0].start < end < strip.frames[0].end, "the key must end mid-frame"
    assert_wire(strip, leds)
    assert shown_frames(strip) == [(31, 0, 0)]
    assert edges.log == quiet[4].log, "%d edges quiet, %d with traffic" % (
        len(quiet[4].log), len(edges.log))
    assert loom.badop() == 0 and quiet[1].badop() == 0


# ------------------------------------------------------- model scenarios
@model_only("fifteen keys, 2 M clocks; the RTL runs a frame and its repeat")
def test_bridge_every_key_of_the_map(backend):
    """From the image's state (off, colour 0, level 4): PLAY turns the
    strip on (white); digit 0 changes nothing (white already: no frame);
    digits 1 .. 9 pick their colours; VOL- dims one level; PLAY turns it
    off; VOL+ while off does nothing; PLAY turns it on again with the colour
    and level kept. Keys 5 ms apart (the receiver needs only an idle line
    before a leader)."""
    bench, loom, src, strip, _ = boot(backend, leds=4, gap_us=5_000.0)
    for key in [PLAY] + DIGITS + [VOL_DOWN, PLAY, VOL_UP, PLAY]:
        press(src, B1824, key)
    settle(bench, src, 4)
    assert_wire(strip, 4)
    expect = [state(0, 4, on=False), state(0, 4)]
    expect += [state(c, 4) for c in range(1, 10)]
    expect += [state(9, 3), state(9, 3, on=False), state(9, 3)]
    assert shown_frames(strip) == [shown(s) for s in expect]
    assert shown(state(9, 3)) == (15, 10, 4)
    assert loom.badop() == 0


def _bad_inverse(command):
    """A frame to ADDR whose command inverse is wrong in one bit."""
    bits = N.frame_bits(ADDR, command)
    bits[31] ^= 1
    return N.bits_pulses(bits)


@model_only("a stream of faulty frames; the RTL runs the same receiver")
def test_bridge_ignores_bad_frames_and_they_end_a_hold(backend):
    """Each of these changes nothing on the strip: a frame to another
    address, an extended address, a wrong command inverse, a frame that
    stops after 20 bits, a frame whose marks are 60 % long, a key not in
    the map (CH-, 0x45). VOL+ then raises the level, a frame to another
    address ends the hold, and the repeat code after it does nothing. Key
    1 still works afterwards. Keys 5 ms apart."""
    bench, loom, src, strip, _ = boot(backend, leds=4, st=state(3, 4), gap_us=5_000.0)
    good = N.frame_pulses(ADDR, DIGITS[1])
    faults = [N.frame_pulses(0x01, DIGITS[1]),
              N.frame_pulses(0xA500, DIGITS[1], extended=True),
              _bad_inverse(DIGITS[1]),
              N.truncate(good, 2 + 2 * 20),
              N.vary(good, mark=1.6),
              N.frame_pulses(ADDR, 0x45)]
    for pulses in faults:
        press(src, B1824, None, pulses=pulses)
    start = src.send(N.frame_pulses(ADDR, VOL_UP))
    src.send(N.frame_pulses(0x01, VOL_UP))
    src.send(N.repeat_pulses())
    press(src, B1824, DIGITS[1])
    settle(bench, src, 4)
    assert_wire(strip, 4)
    assert shown_frames(strip) == [shown(state(3, 4)), shown(state(3, 5)), shown(state(1, 5))]
    assert strip.frames[1].start > start
    assert loom.badop() == 0


@model_only("a 150-LED strip and a held key, 1.2 M clocks")
def test_bridge_never_tears_a_frame_while_a_held_key_ramps(backend):
    """150 LEDs take 230 k clocks a frame and the repeat codes come every
    197 k clocks on this board, so STATE changes while frames are going
    out. From level 2, VOL+ and four repeats: every frame is still one
    colour on every LED, the levels never go back, at least one change
    coalesces (fewer frames than states), the fifth step finds level 7 and
    sends nothing, and the strip ends on level 7."""
    bench, loom, src, strip, _ = boot(backend, leds=150, st=state(6, 2))
    press(src, B1824, VOL_UP, repeats=4)
    settle(bench, src, 150)
    assert_wire(strip, 150)
    levels = [f[2] for f in shown_frames(strip)]
    assert levels[0] == 255 >> 5 and levels[-1] == 255
    assert levels == sorted(levels) and len(set(levels)) == len(levels)
    assert all(lvl in [255 >> (7 - k) for k in range(8)] for lvl in levels)
    assert len(levels) < 6, levels
    assert loom.badop() == 0


@model_only("50 MHz: an NEC frame alone is 3.4 M clocks, so this runs the start")
def test_bridge_at_50_mhz_the_image_blanks_60_leds_and_ignores_noise(backend):
    """The real clock: thread 0's tick 1757 + 208/256, the image as built
    (off, 60 LEDs): the first frame blanks all 60 LEDs, every pulse inside
    the datasheet windows, and a 2 ms low pulse on IN0 (shorter than any
    leader: noise) changes nothing."""
    bench, loom, src, strip, _ = boot(backend, B50, leds=60)   # the image's own N
    src.send([(N.MARK, 2000.0)])
    bench.step(60 * LED_CLOCKS + RESET + B50.cycles(3000))
    assert_wire(strip, 60)
    assert shown_frames(strip) == [(0, 0, 0)]
    assert len(strip.frames[0].data) == 180
    assert loom.badop() == 0


def test_bridge_assembles_strict_with_every_pair_proved():
    """--strict and no diagnostic; thread 1's pairs all proved at one clock
    a tick; thread 0 has no WAITD (as ir_nec_rx.loom), so no pair; no FIFO
    instruction anywhere (no host in the loop); the layout the header
    gives: data at 256, both spills below 384, thread 3's quarter free."""
    assert PROGRAM.diagnostics == [] and sorted(PROGRAM.threads) == [0, 1]
    t1 = PROGRAM.deadlines[1]
    assert t1.period == 1 and t1.pairs and not t1.infeasible and not t1.unbounded
    assert t1.worst_slack >= 0
    t0 = PROGRAM.deadlines[0]
    assert t0.period == 64 and not t0.pairs and not t0.infeasible
    words = PROGRAM.words
    for addr, word in words.items():
        if addr < DATA or addr >= SYM["apply"]:
            decoded = ISA.decode(word)
            assert decoded is None or decoded[0].name not in ("PUSH", "POP"), hex(addr)
    assert max(words) < 384
    assert (words[STATE], words[NLEDS], words[DATA + 2]) == (0x40, 60, 63)
    assert SYM["KEYS"] + SYM["NKEYS"] == SYM["apply"] and SYM["t0end"] == SYM["change"]
