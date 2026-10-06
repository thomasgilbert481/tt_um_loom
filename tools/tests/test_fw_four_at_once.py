"""Four protocols at once: the claim the barrel pipeline is built for.

Four shipped programs run side by side, one per thread, each in its own
quarter of the 512-word memory and on its own pins:

    thread 0  uart_tx_fifo  TX = OUT0                         TICK_INT 128
    thread 1  spi_master    MOSI = OUT1, SCK = OUT2,          TICK_INT 64
                            CS_n = OUT3, MISO = IN0
    thread 2  i2c_master    SCL = BIDIR0, SDA = BIDIR1        TICK_INT 8
    thread 3  ws2812        DOUT = OUT4                       its own tick

Each program is the file in ``firmware/`` with only its ``.thread`` line and
its pins changed (the SPI master's output group, ``OUTGRP``, moves with its
pins). Every run loads the same image, queues the same data and writes the
same ticks through the host port, then starts threads with one ``RUN``
write; only that write's mask differs. So everything up to ``RUN`` is the
same transaction sequence, and a protocol's pin edges in the run where all
four go at once must fall on exactly the clocks they fall on when it runs
alone: SEMANTICS 2 gives each thread its own slot whatever the others do
(ISO-1 proves it for the state, formal/README.md; this shows it on the pins).
The rates are chosen so that the four transfers overlap in time (the
WS2812 program starts with a 55 us reset), and every tick is a multiple of
four clocks, so no edge depends on where a tick fell in the slot grid.
"""

from tools.loomasm import assemble_text
from tools.loomisa import REPO, load
from tools.protomodels.bench import Model, pad_of
from tools.protomodels.i2c import I2cEeprom
from tools.protomodels.spi import SpiFlash
from tools.protomodels.uart import UartRx
from tools.protomodels.ws2812 import Ws2812Rx

ISA = load()
FIRMWARE = REPO / "firmware"
IMEM = 512                                   # the macro build: quarters of 128 words

#: (protocol, program, thread, source substitutions, host tick or None)
LAYOUT = (
    ("uart", "uart_tx_fifo", 0, (), (128, 0)),
    ("spi", "spi_master", 1, (
        (".pins   MOSI = OUT0, SCK = OUT1, CS_N = OUT2, MISO = IN0",
         ".pins   MOSI = OUT1, SCK = OUT2, CS_N = OUT3, MISO = IN0"),
        (".csr    OUTGRP, 16 | (3 << 5)", ".csr    OUTGRP, 17 | (3 << 5)"),
    ), (64, 0)),
    ("i2c", "i2c_master", 2, (), (8, 0)),
    ("ws2812", "ws2812", 3, (
        (".pins   DOUT = OUT0", ".pins   DOUT = OUT4"),
    ), None),
)
PINS = {"uart": ("OUT0",), "spi": ("OUT1", "OUT2", "OUT3"),
        "i2c": ("BIDIR0", "BIDIR1"), "ws2812": ("OUT4",)}
THREAD = {name: thread for name, _, thread, _, _ in LAYOUT}

# The host commands, from each program's header in firmware/README.md.
SKIP, LAST = 0x4000, 0x8000                              # spi_master
START, STOP = 0x100, 0x200                               # i2c_master
UART_BYTES = [0x55, 0xA3, 0x0F, 0xC6]
SPI_WORDS = [SKIP | 0x9F, 0x00, 0x00, LAST | 0x00]       # read the JEDEC ID
I2C_WORDS = [START | 0xA0, 0x10, 0x11, STOP | 0x22]      # write 0x11, 0x22 at 0x10
LED = [0x12, 0x34, 0x56]                                 # one LED, G R B
RUN_CLOCKS = 12000           # the WS2812 frame latches last, about 7,000 after RUN


def program(source, thread, substitutions):
    text = (FIRMWARE / (source + ".loom")).read_text(encoding="utf-8")
    for old, new in ((".thread 0", ".thread %d" % thread),) + tuple(substitutions):
        assert text.count(old) == 1, "%s: %r is not in the source once" % (source, old)
        text = text.replace(old, new)
    return assemble_text(text, source + ".loom", isa=ISA, imem_words=IMEM, strict=True)


def image():
    """The four programs, each at its own thread's quarter, as one image."""
    words = {}
    for _, source, thread, subs, _ in LAYOUT:
        prog = program(source, thread, subs)
        assert prog.errors == [], prog.errors
        base, size = thread * IMEM // 4, IMEM // 4
        assert all(base <= a < base + size for a in prog.words), \
            "%s does not fit thread %d's quarter" % (source, thread)
        words.update(prog.words)
    return words


IMAGE = image()


class Edges(Model):
    """Every change of the protocols' output pins, as (cycle, pin, level)."""

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


def run(backend, threads):
    """One run: the same host sequence every time, then RUN for ``threads``."""
    if backend.name == "model":
        bench = backend.bench(features=("FIFO", "SETPD"), imem_words=IMEM, pullups=0b11)
    else:
        bench = backend.bench(pullups=0b11)
    devices = {
        "uart": bench.add(UartRx("OUT0", 128)),
        "spi": bench.add(SpiFlash("OUT2", "OUT1", "IN0", "OUT3", memory=bytes(64))),
        "i2c": bench.add(I2cEeprom("BIDIR0", "BIDIR1", memory=bytes(256))),
        "ws2812": bench.add(Ws2812Rx("OUT4")),
    }
    edges = bench.add(Edges([p for pins in PINS.values() for p in pins]))
    loom = backend.loom(bench, isa=ISA)
    loom.load(IMAGE)
    loom.push(THREAD["uart"], UART_BYTES)
    loom.push(THREAD["spi"], SPI_WORDS)
    loom.push(THREAD["i2c"], I2C_WORDS)
    loom.push(THREAD["ws2812"], LED)
    for _, _, thread, _, tick in LAYOUT:
        if tick is not None:
            loom.write_csr(thread, "TICK_INT", tick[0])
            loom.write_csr(thread, "TICK_FRAC", tick[1])
    loom.run([THREAD[name] for name in threads], exclusive=True)
    bench.step(RUN_CLOCKS)
    return bench, loom, devices, edges


def check_devices(loom, devices, names):
    if "uart" in names:
        got = [f.value for f in devices["uart"].frames]
        assert got == UART_BYTES, "UART frames %s" % [hex(v) for v in got]
        assert not any(f.framing_error for f in devices["uart"].frames)
    if "spi" in names:
        assert loom.pop(THREAD["spi"], 3) == [0xEF, 0x40, 0x16], "the JEDEC ID"
    if "i2c" in names:
        assert loom.pop(THREAD["i2c"], 4) == [0xA0, 0x10, 0x11, 0x22], "all acknowledged"
        assert bytes(devices["i2c"].memory[0x10:0x12]) == b"\x11\x22"
    if "ws2812" in names:
        frames = devices["ws2812"].frames
        assert len(frames) == 1 and frames[0].leds == [tuple(LED)], \
            "WS2812 frames %s" % [f.leds for f in frames]


def test_four_protocols_at_once_land_on_the_same_clocks_as_alone(backend):
    """Every device gets its data with all four threads running, the four
    transfers overlap, and each protocol's every pin edge is on the same
    clock as when its thread runs alone."""
    names = [name for name, _, _, _, _ in LAYOUT]
    bench, loom, devices, together = run(backend, names)
    check_devices(loom, devices, names)
    assert loom.badop() == 0

    spans = {name: (together.of(name)[0][0], together.of(name)[-1][0]) for name in names}
    assert max(s[0] for s in spans.values()) < min(s[1] for s in spans.values()), \
        "the four transfers do not overlap: %s" % spans

    for name in names:
        _, solo_loom, solo_devices, alone = run(backend, [name])
        check_devices(solo_loom, solo_devices, [name])
        assert together.of(name) == alone.of(name), \
            "%s: %d edges alone, %d with the others; first difference at %s" % (
                name, len(alone.of(name)), len(together.of(name)),
                next((a, b) for a, b in zip(alone.of(name) + [None], together.of(name) + [None])
                     if a != b))
