"""L3-HOTSWAP: one thread's protocol replaced while the other three run.

Loading a program through the host port stops all four threads (SEMANTICS 7:
host IMEM writes are valid only while ``RUN == 0``). Here nothing is ever
halted after ``RUN``:

    thread 0  uart_tx_fifo    TX = OUT0                 TICK_INT 868
    thread 1  dmx_controller  TX = OUT1                 TICK_INT 200
    thread 2  i2c_master      SCL = BIDIR0, SDA = BIDIR1  TICK_INT 64
    thread 3  hot_swap_dshot  DOUT = OUT3               TICK 4 + 43/256

Threads 0..2 are the shipped files with only ``.thread`` and pins changed.
Thread 3 starts as protocol A, one-shot DShot600 (``DshotRx`` checks every
edge against Betaflight's 20/7/14 ticks), takes a swap word through INQ[3]
between frames, and its resident loader writes protocol B with ``ST``: the
shipped ``ws2812.loom`` assembled for thread 3 on OUT3, which then drives a
WS2812B frame that ``Ws2812Rx`` checks against the datasheet.

The other threads' claim is SEMANTICS 2 (each thread owns its slot whatever
the others do) and 6.11 (an ``LD``/``ST`` pair takes no cycle of any other
thread's): their pin edges must fall on exactly the same clocks in a run
with the swap as in a run without it, as ``test_fw_four_at_once.py``
measures. Both runs load the same image and queue the same data before
``RUN``; the others' data is all queued then (four words or fewer each), so
the host traffic that differs after ``RUN`` cannot move their edges either.
Their rates are chosen so that their transfers span the swap.
"""

from tools.loomasm import assemble_text
from tools.loomisa import REPO, load
from tools.protomodels import dshot as D
from tools.protomodels.bench import Model, pad_of
from tools.protomodels.dmx import TX as DMX_TX, DmxRx
from tools.protomodels.i2c import I2cEeprom
from tools.protomodels.uart import UartRx
from tools.protomodels.ws2812 import Ws2812Rx
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
IMEM = 512
BASE, LOADER = 384, 492                       # hot_swap_dshot.loom
ROOM = LOADER - BASE
SWAP = 0x8000
DSHOT_TICK = (4, 43)                          # DShot600 at 50 MHz (dshot_tx.loom)
P = DSHOT_TICK[0] + DSHOT_TICK[1] / 256.0     # clocks per tick (SEMANTICS 4)
BIT, T0H, T1H = 20, 7, 14                     # ticks (Betaflight dshot_dpwm.h)

#: (protocol, program, thread, pin overrides, host tick)
OTHERS = (
    ("uart", "uart_tx_fifo", 0, {}, (868, 0)),
    ("dmx", "dmx_controller", 1, {"TX": "OUT1"}, (200, 0)),
    ("i2c", "i2c_master", 2, {}, (64, 0)),
)
PINS = {"uart": ("OUT0",), "dmx": ("OUT1",), "i2c": ("BIDIR0", "BIDIR1"),
        "t3": ("OUT3",)}
UART_BYTES = [0x55, 0xA3, 0x0F, 0xC6]
DMX_WORDS = [0x00, 2, 0x5A, 0xA5]             # start code 0, two slots
START, STOP = 0x100, 0x200                    # i2c_master
I2C_WORDS = [START | 0xA0, 0x10, 0x11, STOP | 0x22]
DSHOT = [(1046, False), (48, True)]           # protocol A's frames
LED = [0x12, 0x34, 0x56, 0xAB, 0xCD, 0xEF]    # protocol B: two LEDs, G R B


def program(source, thread, pins=None):
    text = (FIRMWARE / (source + ".loom")).read_text(encoding="utf-8")
    if thread != 3 or source != "hot_swap_dshot":
        assert text.count(".thread 0") == 1, source
        text = text.replace(".thread 0", ".thread %d" % thread)
    prog = assemble_text(text, source + ".loom", isa=ISA, imem_words=IMEM,
                         strict=True, pins=pins or {})
    assert prog.errors == [], prog.errors
    return prog


def image():
    words = {}
    for _, source, thread, pins, _ in OTHERS:
        prog = program(source, thread, pins)
        base = thread * IMEM // 4
        assert all(base <= a < base + IMEM // 4 for a in prog.words), source
        words.update(prog.words)
    a = program("hot_swap_dshot", 3)
    assert all(BASE <= addr < IMEM for addr in a.words)
    words.update(a.words)
    return words


def b_image():
    """Protocol B as the loader takes it: contiguous from BASE, under LOADER."""
    prog = program("ws2812", 3, {"DOUT": "OUT3"})
    addrs = sorted(prog.words)
    assert addrs == list(range(BASE, BASE + len(addrs))), "not one block at BASE"
    assert len(addrs) <= ROOM, "%d words do not fit below the loader" % len(addrs)
    return [prog.words[a] for a in addrs]


IMAGE = image()
B_WORDS = b_image()


class Edges(Model):
    """Every change of the watched pins, as (cycle, pin, level)."""

    def __init__(self, names):
        self.pads = {name: pad_of(name) for name in names}
        self.prev = {}
        self.log = []

    def observe(self, lines):
        for name, pad in self.pads.items():
            level = lines.get(pad)
            if name in self.prev and self.prev[name] != level:
                self.log.append((lines.cycle, name, level))
            self.prev[name] = level

    def of(self, protocol):
        return [e for e in self.log if e[1] in PINS[protocol]]


class Gate(Model):
    """Shows ``model`` the lines only while ``on``: OUT3 carries DShot before
    the swap and WS2812 after it, and each receiver must see only its own."""

    def __init__(self, model, on):
        self.model, self.on = model, on

    def observe(self, lines):
        if self.on:
            self.model.observe(lines)


def make_bench(backend):
    """FIFOs, the latch, the bit engine (DMX's SHO) and data memory (ST)."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "SETPD", "BE", "DMEM"),
                             imem_words=IMEM, pullups=0b11)
    return backend.bench(pullups=0b11)


class Run:
    """One run: the same host sequence up to RUN, all four threads started."""

    def __init__(self, backend, dshot=DSHOT):
        self.bench = bench = make_bench(backend)
        self.uart = bench.add(UartRx("OUT0", 868))
        self.dmx = bench.add(DmxRx("OUT1", limits=DMX_TX))
        self.i2c = bench.add(I2cEeprom("BIDIR0", "BIDIR1", memory=bytes(256)))
        self.dshot = D.DshotRx("OUT3", clock_mhz=50.0, rate=600000)
        self.ws = Ws2812Rx("OUT3")
        self.a_gate = bench.add(Gate(self.dshot, True))
        self.b_gate = bench.add(Gate(self.ws, False))
        self.edges = bench.add(Edges([p for pins in PINS.values() for p in pins]))
        self.loom = loom = backend.loom(bench, isa=ISA)
        loom.load(IMAGE)
        loom.push(0, UART_BYTES)
        loom.push(1, DMX_WORDS)
        loom.push(2, I2C_WORDS)
        loom.push(3, [D.packet_of(v, t) for v, t in dshot])
        for _, _, thread, _, tick in OTHERS:
            loom.write_csr(thread, "TICK_INT", tick[0])
            loom.write_csr(thread, "TICK_FRAC", tick[1])
        loom.write_csr(3, "TICK_INT", DSHOT_TICK[0])
        loom.write_csr(3, "TICK_FRAC", DSHOT_TICK[1])
        loom.run([0, 1, 2, 3], exclusive=True)
        self.t_run = bench.cycle

    def until(self, predicate, budget, what):
        assert self.bench.run_until(predicate, budget, every=16), \
            "%s: not after %d clocks" % (what, budget)

    def swap(self, words=None, tick=(1, 0)):
        """The swap word, then the image; the receivers change over once the
        last image word is in INQ (the WS2812 program starts with a 55 us
        reset, so its first rise is later than that)."""
        words = B_WORDS if words is None else words
        self.loom.push(3, [SWAP])
        self.t_swap = self.bench.cycle
        self.loom.push(3, [tick[0], tick[1], len(words)] + list(words))
        self.t_image = self.bench.cycle
        self.a_gate.on, self.b_gate.on = False, True

    def finish(self, end):
        assert self.bench.cycle <= end, "the run is already at %d" % self.bench.cycle
        self.bench.step(end - self.bench.cycle)


def check_others(run):
    got = [f.value for f in run.uart.frames]
    assert got == UART_BYTES, "UART frames %s" % [hex(v) for v in got]
    assert not any(f.framing_error for f in run.uart.frames)
    (packet,) = run.dmx.packets()
    assert (packet.start_code, packet.data) == (0, DMX_WORDS[2:])
    assert run.dmx.violations(packet, DMX_TX) == []
    assert run.loom.pop(2, 4) == [0xA0, 0x10, 0x11, 0x22], "all acknowledged"
    assert bytes(run.i2c.memory[0x10:0x12]) == b"\x11\x22"


def check_dshot(rx, frames):
    """Every window of the model, then every edge on its tick: an edge k ticks
    after the frame's first lands within one clock of k * P (SEMANTICS 4 and
    6.10 rule 1; dshot_tx's own check)."""
    assert rx.violations == [], rx.report()
    assert [(f.value, f.telemetry) for f in rx.frames] == frames
    for f in rx.frames:
        assert f.crc_ok
        want = []
        for j in range(16):
            bit = (f.frame >> (15 - j)) & 1
            want += [BIT * j, BIT * j + (T1H if bit else T0H)]
        assert len(f.edges) == len(want)
        for edge, ticks in zip(f.edges, want):
            assert abs(edge - f.edges[0] - ticks * P) < 1, (f.frame, edge, ticks)


def check_ws(rx, data):
    assert rx.violations == [], rx.violations
    assert [f.data for f in rx.frames] == [bytes(data)], rx.frames


#: Both runs stop at the same clock after RUN: the UART's four bytes at
#: 868 clocks a bit end about 33,400 clocks in and the I2C transfer about
#: 40,000; the swap runs from about 900 to 26,000 (model figures, used only
#: to size the run; every check below is against the specifications).
SPAN = 42000


def swap_run(backend):
    run = Run(backend)
    run.until(lambda: run.dshot.in_frame, 4000, "the first DShot frame")
    run.swap()
    run.loom.push(3, LED)
    run.until(lambda: len(run.ws.frames) >= 1, 30000, "the WS2812 frame")
    run.finish(run.t_run + SPAN)
    return run


def test_swap_dshot_for_ws2812_while_three_run(backend):
    """The PHY-GL-SDF candidate. DShot frames as Betaflight times them, a
    swap pushed mid-frame that waits for the end of that frame and of the
    frame queued before it, the image written by the loader, the WS2812
    frame as the datasheet times it, and threads 0..2's every pin edge on
    the same clock as in a run without the swap."""
    run = swap_run(backend)
    check_dshot(run.dshot, DSHOT)
    first, second = run.dshot.frames
    assert first.edges[0] < run.t_swap < first.edges[-1], \
        "the swap word was not pushed while the first frame was on the wire"
    b_edges = [c for c, _, _ in run.edges.of("t3") if c > second.edges[-1]]
    assert b_edges and b_edges[0] > run.t_image, "protocol B started early"
    # WS2812's own reset: the line stays low 55 us (> 50 us) before B's first rise
    assert b_edges[0] - second.edges[-1] >= 50 * 50
    check_ws(run.ws, LED)
    check_others(run)
    assert run.loom.badop() == 0

    # Each transfer drives its pins while the loader is writing the image.
    for name in ("uart", "dmx", "i2c"):
        during = [c for c, _, _ in run.edges.of(name) if run.t_swap < c < run.t_image]
        assert len(during) >= 4, "%s is idle during the swap" % name

    still = Run(backend)
    still.until(lambda: len(still.dshot.frames) >= len(DSHOT), 6000, "DShot alone")
    still.finish(still.t_run + SPAN)
    assert still.t_run == run.t_run, "the host sequence up to RUN differs"
    check_dshot(still.dshot, DSHOT)
    check_others(still)
    for name in ("uart", "dmx", "i2c"):
        assert run.edges.of(name) == still.edges.of(name), \
            "%s: %d edges without the swap, %d with it; first difference at %s" % (
                name, len(still.edges.of(name)), len(run.edges.of(name)),
                next((a, b) for a, b in zip(still.edges.of(name) + [None],
                                            run.edges.of(name) + [None]) if a != b))
    # And protocol A's own edges are the same until the swap took over.
    a_edges = [e for e in run.edges.of("t3") if e[0] <= second.edges[-1]]
    assert a_edges == still.edges.of("t3")


@model_only("the refusal paths are two branches of the loader; the RTL runs "
            "the accepted path, whose instructions include all of theirs")
def test_refused_images_leave_dshot_running(backend):
    """An image longer than ROOM would overwrite the loader: its words are
    taken and dropped, nothing is written, and DShot carries on with its
    tick. An empty image (n = 0) is the same with nothing to drop."""
    run = Run(backend, dshot=DSHOT[:1])
    run.until(lambda: len(run.dshot.frames) >= 1, 6000, "the first frame")
    run.swap(words=[0xFFFF] * (ROOM + 1), tick=(1, 0))
    run.a_gate.on, run.b_gate.on = True, False
    run.loom.push(3, [D.packet_of(1500)])
    run.until(lambda: len(run.dshot.frames) >= 2, 6000, "a frame after the refusal")
    run.loom.push(3, [SWAP, 1, 0, 0, D.packet_of(0)])
    run.until(lambda: len(run.dshot.frames) >= 3, 6000, "a frame after n = 0")
    check_dshot(run.dshot, [DSHOT[0], (1500, False), (0, False)])
    assert run.loom.fifo_status(3)["inq"] == 0
    assert run.loom.badop() == 0


@model_only("the RTL runs the accepted swap above; this only adds the image "
            "the loader can just take")
def test_swap_in_a_full_size_image(backend):
    """ROOM words is the largest image: B's 52 words padded to ROOM with NOPs
    after its last word (never reached), and B runs as before."""
    nop = ISA.encode("NOP")
    run = Run(backend, dshot=DSHOT[:1])
    run.until(lambda: len(run.dshot.frames) >= 1, 6000, "the first frame")
    run.swap(words=B_WORDS + [nop] * (ROOM - len(B_WORDS)))
    run.loom.push(3, LED[:3])
    run.until(lambda: len(run.ws.frames) >= 1, 30000, "the WS2812 frame")
    check_ws(run.ws, LED[:3])
    assert run.loom.badop() == 0
