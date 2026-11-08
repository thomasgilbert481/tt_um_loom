"""L3 for the Feld-Hell transmitter, on the model and the RTL.

``firmware/feldhell_tx.loom`` (thread 0, KEY = OUT0, TONE = OUT1) draws
ASCII from the host in a 7 x 7 font and sends it column by column, bottom
pixel first, as an on/off keyed square-wave carrier. The reference is
``tools.protomodels.feldhell``, written from the published descriptions
of the mode it cites (Greenman ZL1BPU, "Feld-Hell"; secondary sources):
its encoder gives the expected pixel stream and bitmap, its receiver
rebuilds the bitmap from the TONE pin alone (tone energy in each
half-pixel slot), and its key checker holds every KEY edge to the pixel
grid.

Expected values come from the mode's description (7 columns of 7 pixels
a character, bottom to top and left to right, each pixel two equal
half-pixels, 122.5 pixels a second, 400 ms a character) and from the
program header's tick rule (a tick is half a carrier period; 8 carrier
cycles a pixel), never from what the model or the RTL did. KEY edges are
latched (SEMANTICS 6.10), so they must sit on the pixel grid to the
clock; TONE edges are ordinary writes one slot after their ticks.

Scale. At 50 MHz a pixel is 408 k clocks and a character 20 M, beyond
either backend. The program takes its time base from the host, so the
scenarios run a fast "board": TICK_INT 64 (the shortest tick the
deadline checker proves), a carrier period of 128 clocks, a pixel of
1024 clocks, a character of 50 176. The program logic is the same at any
speed; only the counts change. The 50 MHz figures (980 Hz, 122.5 pixels
a second, 400 ms a character) are checked by arithmetic, without a
simulation (``test_feldhell_tick_arithmetic_at_50mhz``). RTL budget,
stated: the three scenarios that run on the RTL total 0.61 M
clocks (counted on the model); the long text is ``model_only``.

Every body here is named ``test_feldhell_*``: test/test_fw.py collects
the bodies of all modules into one namespace.
"""

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels import feldhell as F
from tools.protomodels.ir_nec import EdgeRecorder
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
IMEM = 512

TICK = 64                           # clocks a tick: half a carrier period
CPP = 8                             # carrier cycles a pixel (program header)
PIXEL = 2 * TICK * CPP              # 1024 clocks
HALF = PIXEL // 2                   # a half-pixel slot
COLUMN = F.ROWS * PIXEL             # 7168 clocks
CHAR = F.COLUMNS * COLUMN           # 50 176 clocks
FONT = 320                          # the font's address (program header)


def program():
    return assemble_file(FIRMWARE / "feldhell_tx.loom", isa=ISA, strict=True)


def setup(backend):
    """Bench, host and the two pin recorders, the program running."""
    if backend.name == "model":
        bench = backend.bench(features=("FIFO", "SETPD", "DMEM"), imem_words=IMEM)
    else:
        bench = backend.bench()
    key = bench.add(EdgeRecorder("OUT0"))
    tone = bench.add(EdgeRecorder("OUT1"))
    loom = backend.loom(bench, isa=ISA)
    loom.load(program().words)
    loom.write_csr(0, "TICK_INT", TICK)
    loom.write_csr(0, "TICK_FRAC", 0)
    loom.run([0])
    return bench, loom, key, tone


def send(bench, loom, key, data, n_chars, budget):
    """Feed ``data`` into INQ[0] as it has room and run until the key has
    been up two columns after the last of its expected edges, or
    ``budget`` clocks pass."""
    want = len(key.edges) + 2 * sum(
        1 for lev, _ in F.runs_of(F.pixels_of(F.columns_of(data))) if lev)
    del n_chars
    words, end = list(data), bench.cycle + budget
    done = lambda: (len(key.edges) >= want
                    and bench.cycle - key.edges[-1][0] > 2 * COLUMN)
    while words or not done():
        if words:
            st = loom.fifo_status(0)
            free = st["depth"] - st["inq"]
            if free > 0:
                loom.push(0, words[:free])
                words = words[free:]
        assert bench.cycle < end, "%d of %d key edges" % (len(key.edges), want)
        bench.run_until(done if not words else (lambda: False),
                        max(1, min(COLUMN, end - bench.cycle)), every=256)


def origin_of(key, pixels):
    """The first pixel slot's start: the first key rise less the pixels
    before the first on pixel of the expected stream."""
    return key.edges[0][0] - pixels.index(1) * PIXEL


def show(title, columns):
    """The bitmap as ASCII art in the test output (pytest -s shows it)."""
    print("\n%s" % title)
    for row in F.render(columns):
        print("    " + row)


def check_text(key, tone, data, origin=None):
    """The key on the pixel grid to the clock, the bitmap rebuilt from the
    tone pin, the carrier: two ticks a period, high one, CPP periods a
    pixel, one slot after the pixel's tick, and silent while the key is up.
    Returns the origin (the first pixel slot's start)."""
    cols = F.columns_of(data)
    pixels = F.pixels_of(cols)
    if origin is None:
        origin = origin_of(key, pixels)
    bad = F.check_key(key.edges, origin, PIXEL, pixels)
    assert not bad, "key edges off the grid (index, got, want): %r" % bad[:6]
    got, split = F.receive(tone.edges, origin, HALF, len(cols))
    show("sent %r, rebuilt from TONE:" % bytes(data), got)
    assert got == cols, "rebuilt bitmap differs:\n%s\nwant\n%s" % (
        "\n".join(F.render(got)), "\n".join(F.render(cols)))
    assert split == []                          # both half-pixels alike
    rises = [c for c, v in tone.edges if v]
    falls = [c for c, v in tone.edges if not v]
    assert len(rises) == len(falls) == CPP * sum(pixels)
    assert {f - r for r, f in zip(rises, falls)} == {TICK}
    marks = list(zip(key.edges[0::2], key.edges[1::2]))
    for (a, _), (b, _) in marks:
        inside = [r for r in rises if a <= r < b]
        assert len(inside) == CPP * (b - a) // PIXEL
        assert 0 < inside[0] - a <= 8           # one slot after the key edge
        assert {y - x for x, y in zip(inside, inside[1:])} == {2 * TICK}
    assert all(any(a < f <= b + 8 for (a, _), (b, _) in marks) for f in falls)
    return origin


def test_feldhell_word(backend):
    """"HI": the bitmap rebuilt from the tone pin is the font's, every key
    edge on the 1024-clock pixel grid to the clock (122.5 pixels a second
    at 50 MHz), the carrier 8 periods of 128 clocks a pixel (980 Hz), the
    second character 7 columns (400 ms) after the first."""
    bench, loom, key, tone = setup(backend)
    send(bench, loom, key, b"HI", 2, 4 * CHAR)
    origin = check_text(key, tone, b"HI")
    i_first = F.pixels_of(F.columns_of(b"I")).index(1)
    i_rise = [c for c, v in key.edges if v and c >= origin + CHAR][0]
    assert i_rise == origin + CHAR + i_first * PIXEL
    assert loom.pop_available(0) == []
    assert loom.badop() == 0


def test_feldhell_unknown_and_lower_case(backend):
    """A BEL (0x07, no glyph) is sent as a space and reported 0xC007;
    lower-case o is drawn as O."""
    bench, loom, key, tone = setup(backend)
    send(bench, loom, key, b"\x07o", 2, 4 * CHAR)
    check_text(key, tone, b" O")                # the space has no edges
    assert loom.pop_available(0) == [0xC007]
    assert loom.badop() == 0


def test_feldhell_column_grid_after_idle(backend):
    """"E", nothing for a while, then "L": the line stays blank, and the L
    starts a whole number of columns after the E (the receiver's text does
    not move up or down), at least 7 columns."""
    bench, loom, key, tone = setup(backend)
    send(bench, loom, key, b"E", 1, 3 * CHAR)
    origin_e = check_text(key, tone, b"E")
    bench.run_until(lambda: False, 3 * COLUMN + 1234, every=1024)
    n = len(key.edges)
    send(bench, loom, key, b"L", 1, 3 * CHAR)
    rise = key.edges[n][0]
    origin_l = rise - F.pixels_of(F.columns_of(b"L")).index(1) * PIXEL
    assert (origin_l - origin_e) % COLUMN == 0
    assert origin_l - origin_e >= CHAR + 3 * COLUMN
    key.edges, tone.edges = key.edges[n:], [e for e in tone.edges if e[0] >= rise - 8]
    check_text(key, tone, b"L", origin=origin_l)
    assert loom.badop() == 0


@model_only("19 characters, 1.0 M clocks, more than the batch's RTL budget; "
            "HI, the unknown byte and the idle grid run there")
def test_feldhell_long_text(backend):
    """A pangram, rebuilt from the tone pin and on the pixel grid; the
    characters back to back, 400 ms (7 columns) each."""
    text = b"THE QUICK BROWN FOX"
    bench, loom, key, tone = setup(backend)
    send(bench, loom, key, text, len(text), (len(text) + 3) * CHAR)
    check_text(key, tone, text)
    assert loom.pop_available(0) == []
    assert loom.badop() == 0


def test_feldhell_font_in_the_image():
    """The program's table is the model's font (3 words a glyph at 320,
    0x20 .. 0x5F), and the code fits thread 0's quarter."""
    prog = program()
    table = F.font_table()
    assert len(table) == 192
    assert [prog.words[FONT + k] for k in range(192)] == table
    code = [a for a in prog.words if a < FONT]
    assert max(code) < 128
    assert sorted(prog.words) == sorted(code) + list(range(FONT, FONT + 192))


def test_feldhell_tick_arithmetic_at_50mhz():
    """The header's 50 MHz settings against the mode: 980 Hz, a pixel of
    8.163 ms (122.5 a second), 400 ms (2.5 a second) a character."""
    tick = 25510 + 52 / 256                     # TICK_INT 25510, TICK_FRAC 52
    f_tone = 50e6 / (2 * tick)
    assert abs(f_tone - 980.0) / 980.0 < 1e-6
    pixel_s = 2 * CPP * tick / 50e6
    assert abs(pixel_s - F.PIXEL_S) < 1e-8 and abs(F.PIXEL_S - 8.163e-3) < 1e-6
    assert abs(F.COLUMNS * F.ROWS * pixel_s - 0.400) < 1e-6
    assert abs(F.CHAR_RATE - 2.5) < 1e-12
    assert 2 * CPP * F.ROWS * F.COLUMNS == 784  # ticks a character


def test_feldhell_model_font_and_receiver():
    """The model's font: 64 glyphs, the last two columns blank, H's left
    stroke full and E's bottom row on; a synthetic carrier rebuilds."""
    assert len(F.FONT) == 64
    for code in range(F.FIRST, F.LAST + 1):
        assert F.glyph_columns(chr(code))[5:] == [0, 0]
    assert F.glyph_columns("H")[0] == 0x7F
    assert all(c & 1 for c in F.glyph_columns("E")[:5])
    assert F.glyph_char(ord("a")) == "A" and F.glyph_char(0x0D) is None
    cols = F.columns_of(b"OK")
    edges, t = [], 1000
    for p in F.half_pixels_of(cols):            # a square carrier, 4 cycles a half
        for _ in range(4):
            if p:
                edges += [(t, 1), (t + 64, 0)]
            t += 128
    assert F.receive(edges, 1000, 512, len(cols)) == (cols, [])
    assert F.runs_of([0, 1, 1, 0]) == [(0, 1), (1, 2), (0, 1)]
