"""L3-HUB75 on the model and the RTL: firmware/hub75.loom, a 32 x 16 HUB75
RGB LED matrix panel at 1/8 scan with three bit planes of binary-coded
modulation (D-035).

The program (thread 0 shifts: R1 G1 B1 R2 G2 B2 = BIDIR0..5, CLK = BIDIR6;
thread 1 scans: A B C = OUT0..2, LAT = OUT3, OE = BIDIR7) drives
``tools.protomodels.hub75.Hub75Panel``, which shifts, latches and decodes
as the panel's column drivers and row decoder do, checks every CLK, LAT
and OE interval against TI TLC5916 (SLVS695D) section 7.9, flags any clock
at which the panel is lit while the address or the latch changes, and
integrates the lit clocks per pixel.

What is expected comes from the panel descriptions and the data sheet as
``protomodels.hub75`` cites them, and from the program header (the frame
buffer layout, the plane weights W0 x 2^p ticks of 16 clocks, the host
words), never from what the model or the RTL did.

The RTL runs one row pair (three planes, about 5,000 clocks after the
image is loaded); the whole frame, the brightness levels of every pixel,
the host's updates and a late shifter run on the model only.
"""

import functools
import random

import pytest

from tools.loomasm import assemble_text
from tools.loomisa import REPO, load
from tools.protomodels.hub75 import TLC5916, Hub75Panel, levels
from tools.tests.fw_backend import model_only

ISA = load()
SOURCE = REPO / "firmware" / "hub75.loom"
CLOCK = 50e6
TICK = 16                       # thread 1's tick (program header)
W0_LINE = ".equ    W0 = 40 "
W0 = 40                         # plane 0's on-time in ticks (program header)
PLANES = 3
UNITS, UNIT_WORDS = 24, 16      # (row pair, plane) units of the frame buffer
FB_A, HOLE, FB_B = 0x030, 0x080, 0x0A0
FRAME = 40_000                  # more than one frame: 8 x (7 x 640 + gaps)
POLLS = 100_000


def phys(a):
    """The IMEM address of logical frame-buffer word ``a`` (header)."""
    addr = FB_A + a
    return addr + (FB_B - HOLE) if addr >= HOLE else addr


def pattern(x, y):
    """The image's built-in test pattern (program header)."""
    return (x >> 2, y >> 1, 7 - (x >> 2))


def pack(level):
    """The 384 frame-buffer words of ``level(x, y) -> (r, g, b)``, in the
    header's layout: unit 3r + p, word k = columns 31-2k (bits 5:0) and
    30-2k (bits 13:8), bits R1 G1 B1 R2 G2 B2, bit p of each level."""
    def byte(x, r, p):
        top, bottom = level(x, r), level(x, r + 8)
        bits = [(top[c] >> p) & 1 for c in range(3)]
        bits += [(bottom[c] >> p) & 1 for c in range(3)]
        return sum(b << i for i, b in enumerate(bits))
    words = []
    for u in range(UNITS):
        r, p = divmod(u, PLANES)
        words += [byte(31 - 2 * k, r, p) | byte(30 - 2 * k, r, p) << 8
                  for k in range(UNIT_WORDS)]
    return words


def latch_of(words, u):
    """The six column latches after unit ``u`` of the 384 ``words`` is
    shifted in and latched, from the header's layout (column 31 first)."""
    out = [0] * 6
    for k in range(UNIT_WORDS):
        w = words[u * UNIT_WORDS + k]
        for line in range(6):
            out[line] |= ((w >> line) & 1) << (31 - 2 * k)
            out[line] |= ((w >> (8 + line)) & 1) << (30 - 2 * k)
    return tuple(out)


def prefix(old, new, changed, j):
    """``old`` with the first ``j`` of the ``changed`` words from ``new``."""
    words = list(old)
    for a in changed[:j]:
        words[a] = new[a]
    return words


def unit_latch(level, u):
    """What latching unit ``u`` must leave in the six column latches: line
    i (R1 G1 B1 R2 G2 B2), bit x = column x."""
    r, p = divmod(u, PLANES)
    out = []
    for line in range(6):
        y, colour = r + 8 * (line // 3), line % 3
        out.append(sum(((level(x, y)[colour] >> p) & 1) << x for x in range(32)))
    return tuple(out)


@functools.lru_cache(maxsize=None)
def program(w0=W0):
    text = SOURCE.read_text(encoding="utf-8")
    assert text.count(W0_LINE) == 1
    text = text.replace(W0_LINE, ".equ    W0 = %d " % w0)
    return assemble_text(text, SOURCE.name, isa=ISA, strict=True)


def image(level=None, w0=W0):
    """The program's words, with the frame buffer of ``level`` if given."""
    words = dict(program(w0).words)
    if level is not None:
        for a, w in enumerate(pack(level)):
            words[phys(a)] = w
    return {"words": words, "imem_words": 512}


def setup(backend, level=None, w0=W0):
    """The image loaded, a panel on the pins (OE's pull-up on the board),
    threads 0 and 1 started together."""
    kwargs = dict(pullups=0x80)
    if backend.name == "model":
        kwargs.update(features=("FIFO", "SETPD", "DMEM"), imem_words=512)
    bench = backend.bench(**kwargs)
    panel = bench.add(Hub75Panel(clock=CLOCK))
    loom = backend.loom(bench, isa=ISA)
    loom.load(image(level, w0))
    loom.run([0, 1])                 # one RUN write: the same edge
    return bench, panel, loom


def run_pulses(bench, panel, count, max_cycles):
    assert bench.run_until(lambda: len(panel.pulses) >= count, max_cycles,
                           every=64), (len(panel.pulses), panel.violations[:5])


def clean(bench, panel):
    """No timing miss, no ghosting, no line driven both ways."""
    assert panel.violations == []
    assert bench.contention_free()


def check_frame_order(panel, pulses, w0=W0):
    """Each pulse is unit k of the scan: row pair k // 3, plane k % 3, lit
    for exactly W0 x 2^p ticks of 16 clocks (program header)."""
    for k, pulse in enumerate(pulses):
        r, p = divmod(k % UNITS, PLANES)
        assert pulse.addr == r, (k, pulse)
        assert pulse.clocks == w0 * TICK << p, (k, pulse.clocks)


# ------------------------------------------------------------- scenarios
def test_hub75_one_row_pair(backend):
    """Row pair 0, its three planes: each latch holds exactly the unit's
    192 bits (the built-in pattern), each plane is lit for 640, 1280 and
    2560 clocks, so rows 0 and 8 show the pattern's levels at 640 clocks a
    level; every CLK, LAT and OE interval meets TLC5916 7.9 and nothing is
    lit while the address or the latch changes."""
    bench, panel, _ = setup(backend)
    run_pulses(bench, panel, 3, 12_000)
    clean(bench, panel)
    first = panel.pulses[:3]
    check_frame_order(panel, first)
    for k in range(3):
        assert panel.latches[k][1] == 0
        assert panel.latches[k][2] == unit_latch(pattern, k)
    assert panel.clocks_shifted >= 3 * 32
    seen = levels(panel.on_time(first), W0 * TICK)
    for y in (0, 8):
        assert seen[y] == [pattern(x, y) for x in range(32)], y
    for name in ("tw_clk", "tsu_d", "th_d", "tw_le", "tsu_le", "tw_oe"):
        assert panel.measured[name], name
        assert min(panel.measured[name]) >= TLC5916[name], name
    assert min(panel.measured["t_clk"]) >= 1e9 / TLC5916["f_clk_max"]


@model_only("a frame is 38,000 clocks; the RTL run covers one row pair")
def test_hub75_whole_frame(backend):
    """Two whole frames of the built-in pattern: 48 pulses in scan order,
    every latch the unit the header puts at that place, every pixel's
    on-time exactly its level times 640 clocks per frame, no ghosting."""
    bench, panel, _ = setup(backend)
    run_pulses(bench, panel, 2 * UNITS, 2 * FRAME + 10_000)
    clean(bench, panel)
    frames = panel.pulses[:2 * UNITS]
    check_frame_order(panel, frames)
    for k, latch in enumerate(panel.latches[:2 * UNITS]):
        assert latch[1] == (k % UNITS) // PLANES
        assert latch[2] == unit_latch(pattern, k % UNITS), k
    for f in range(2):
        seen = levels(panel.on_time(frames[f * UNITS:(f + 1) * UNITS]), W0 * TICK)
        assert seen == [[pattern(x, y) for x in range(32)] for y in range(16)]
    gaps = [b.start - a.end for a, b in zip(frames, frames[1:])]
    period = frames[UNITS].start - frames[0].start
    lit = sum(p.clocks for p in frames[:UNITS])
    assert lit == 8 * 7 * W0 * TICK
    assert max(gaps) < W0 * TICK // 4          # the shifter is never late
    assert period < 40_000                     # > 1.25 kHz at 50 MHz


@model_only("a frame is 38,000 clocks; the RTL run covers one row pair")
@pytest.mark.parametrize("seed", [1, 2])
def test_hub75_brightness_levels(backend, seed):
    """A random image, every pixel and colour at a random level 0..7, loaded
    by the host: one frame recovers every level from the on-time alone."""
    rng = random.Random(seed)
    table = [[tuple(rng.randrange(8) for _ in range(3)) for _ in range(32)]
             for _ in range(16)]
    level = lambda x, y: table[y][x]                          # noqa: E731
    bench, panel, _ = setup(backend, level)
    run_pulses(bench, panel, UNITS, FRAME + 10_000)
    clean(bench, panel)
    check_frame_order(panel, panel.pulses[:UNITS])
    assert levels(panel.on_time(panel.pulses[:UNITS]), W0 * TICK) == table


@model_only("the host's SPI transfers take most of a frame; covered by the model")
def test_hub75_host_update(backend):
    """The host rewrites row pair 2 (units 6..8) to white and one pixel to
    (5, 0, 2) through INQ while the panel refreshes, plus one address past
    the frame buffer: every latch is the old or the new unit, whole (no
    bit plane torn), the next frame shows the new image, and the dropped
    word changed nothing."""
    bench, panel, loom = setup(backend)
    run_pulses(bench, panel, 4, 12_000)
    table = [[pattern(x, y) for x in range(32)] for y in range(16)]
    for x in range(32):
        table[2][x] = table[10][x] = (7, 7, 7)
    table[13][5] = (5, 0, 2)
    new = lambda x, y: table[y][x]                            # noqa: E731
    old_words, new_words = pack(pattern), pack(new)
    changed = [a for a in range(384) if old_words[a] != new_words[a]]
    for a in changed:
        loom.push(0, [a, new_words[a]], max_polls=POLLS)
    loom.push(0, [384, 0x3F3F], max_polls=POLLS)
    done = len(panel.pulses)
    start = done + UNITS - done % UNITS + UNITS    # a whole frame after
    run_pulses(bench, panel, start + UNITS, 3 * FRAME)
    clean(bench, panel)
    check_frame_order(panel, panel.pulses[:start + UNITS])
    # The host's words land one at a time, in order, and only between
    # units: every latch is the frame buffer as it stood after some prefix
    # of the updates, whole, and never an older prefix than before.
    last = [0] * UNITS
    for k, (_, _, latch) in enumerate(panel.latches[:start + UNITS]):
        u = k % UNITS
        states = [j for j in range(len(changed) + 1)
                  if latch == latch_of(prefix(old_words, new_words, changed, j), u)]
        assert states and max(states) >= last[u], (k, states)
        last[u] = max(states)
    frame = panel.pulses[start:start + UNITS]
    assert levels(panel.on_time(frame), W0 * TICK) == table
    loom.halt([0, 1])
    expect = image(new)["words"]
    got = loom.read_imem(0, 0x1D0)
    assert [a for a in sorted(expect) if a < 0x1D0 and got[a] != expect[a]] == []


@model_only("a frame is 38,000 clocks; a timing property of the model run")
def test_hub75_late_shifter_keeps_weights(backend):
    """W0 = 8 ticks (128 clocks), far less than the 604 clocks a unit takes
    to shift: the scanner waits for every unit, so the panel is dark for
    longer between planes, but each plane is still lit for exactly
    8 x 16 x 2^p clocks and every level is exact."""
    bench, panel, _ = setup(backend, w0=8)
    run_pulses(bench, panel, UNITS, FRAME)
    clean(bench, panel)
    frame = panel.pulses[:UNITS]
    check_frame_order(panel, frame, w0=8)
    seen = levels(panel.on_time(frame), 8 * TICK)
    assert seen == [[pattern(x, y) for x in range(32)] for y in range(16)]
    gaps = [b.start - a.end for a, b in zip(frame, frame[1:])]
    after_plane_0 = gaps[0::PLANES]
    assert min(after_plane_0) > 600 - 8 * TICK - 100   # the scanner waited


class _Lines:
    def __init__(self, cycle, values):
        self.cycle, self.values = cycle, values

    def get(self, pad):
        return self.values.get(pad, 0)


def test_hub75_model_flags_ghosting_and_timing():
    """The model itself: an address change while lit, a latch opened while
    lit and data changing on the CLK edge are all reported."""
    panel = Hub75Panel(clock=CLOCK)
    pads = panel.pads
    state = {pads["OE"]: 1}

    def clock(cycle, **levels_):
        for name, value in levels_.items():
            state[pads[name]] = value
        panel.observe(_Lines(cycle, dict(state)))

    clock(0)
    clock(1, R1=1)
    clock(2, CLK=1, G1=1)                # data on the rising edge: tsu_d
    clock(3, CLK=0)
    clock(4, OE=0)                       # lit
    clock(5, A=1)                        # address changes while lit
    clock(6, LAT=1)                      # latch open while lit
    clock(7, LAT=0, OE=1)
    kinds = [v[1] for v in panel.violations]
    assert "tsu_d" in kinds
    assert "address changed while lit" in kinds
    assert "latch open while lit" in kinds
    assert "tw_oe" in kinds              # 3 clocks = 60 ns < 500 ns
