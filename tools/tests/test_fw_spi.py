"""L3-SPI-MASTER on the golden model: firmware/spi_master.loom reads a flash.

The program is loaded and fed through ``tools.loomhost.Loom`` over
``ModelTransport``; ``tools.protomodels.spi.SpiFlash`` answers on the pads
(MOSI OUT0, SCK OUT1, CS_n OUT2, MISO IN0). Modelled rate: TICK_INT 32, so
half an SCK period is 32 clocks: 781 kHz at 50 MHz.
"""

import pytest

from tools.loomasm import assemble_file
from tools.loomhost import Loom, ModelTransport
from tools.loomisa import REPO, load
from tools.protomodels.bench import Bench, Model, pad_of
from tools.protomodels.spi import SpiFlash

ISA = load()
PINS = ("OUT1", "OUT0", "IN0", "OUT2")                   # SCK, MOSI, MISO, CS_n
MEMORY = bytes((i * 13 + 1) & 0xFF for i in range(1024))
HALF = 32
SLOT = 4
LAST, SKIP, CONFIG = 0x8000, 0x4000, 0x2000


class Edges(Model):
    """Records every change of SCK and CS_n on the pads."""

    def __init__(self):
        self.sck, self.cs = pad_of("OUT1"), pad_of("OUT2")
        self.prev = None
        self.sck_edges, self.cs_edges = [], []

    def observe(self, lines):
        now = (lines.get(self.sck), lines.get(self.cs))
        if self.prev is not None:
            if now[0] != self.prev[0]:
                self.sck_edges.append(lines.cycle)
            if now[1] != self.prev[1]:
                self.cs_edges.append((lines.cycle, now[1]))
        self.prev = now


def setup(mode=0, lsb_first=False):
    bench = Bench()
    flash = bench.add(SpiFlash(*PINS, memory=MEMORY, mode=mode, lsb_first=lsb_first))
    edges = bench.add(Edges())
    program = assemble_file(REPO / "firmware" / "spi_master.loom", isa=ISA, strict=True)
    loom = Loom(ModelTransport(bench), isa=ISA)
    loom.load(program)
    loom.write_csr(0, "TICK_INT", HALF)
    loom.run(0)
    if mode or lsb_first:
        loom.push(0, [CONFIG | mode | (lsb_first << 2)])
    return bench, flash, edges, loom


def jedec(loom):
    loom.push(0, [SKIP | 0x9F, 0x00, 0x00, LAST | 0x00])
    return loom.pop(0, 3)


def read(loom, addr, count):
    loom.push(0, [SKIP | 0x03, SKIP | (addr >> 16), SKIP | ((addr >> 8) & 0xFF),
                  SKIP | (addr & 0xFF)] + [0] * (count - 1) + [LAST])
    return loom.pop(0, count)


def test_reads_the_jedec_id_in_mode_0():
    bench, flash, edges, loom = setup()
    assert jedec(loom) == [0xEF, 0x40, 0x16]
    # CS_n is low from reset until the program drives it high (uo_out resets
    # to 0, docs/spec-questions/firmware.md FW-6): an empty selection
    assert flash.transactions[0] == []
    assert flash.commands == [0x9F] and len(flash.transactions) == 2
    assert [b for b, _ in flash.transactions[1]] == [0x9F, 0, 0, 0]
    assert loom.fifo_status(0)["outq"] == 0                # SKIP pushed nothing
    assert loom.badop() == 0


def test_reads_data_bytes_with_a_24_bit_address():
    bench, flash, edges, loom = setup()
    assert read(loom, 0x0001F0, 6) == list(MEMORY[0x1F0:0x1F6])
    assert jedec(loom) == [0xEF, 0x40, 0x16]
    assert flash.commands == [0x03, 0x9F] and len([t for t in flash.transactions if t]) == 2


@pytest.mark.parametrize("mode", [1, 2, 3])
@pytest.mark.parametrize("lsb_first", [False, True])
def test_every_mode_and_bit_order(mode, lsb_first):
    bench, flash, edges, loom = setup(mode, lsb_first)
    assert jedec(loom) == [0xEF, 0x40, 0x16]
    assert read(loom, 0x000123, 2) == list(MEMORY[0x123:0x125])


def test_lsb_first_in_mode_0():
    bench, flash, edges, loom = setup(0, True)
    assert jedec(loom) == [0xEF, 0x40, 0x16]


def test_sck_timing_and_chip_select_framing():
    bench, flash, edges, loom = setup()
    jedec(loom)
    # CS_n rises once when the program starts, then frames the transaction
    assert [v for _, v in edges.cs_edges] == [1, 0, 1]
    (_, _), (fall, _), (rise, _) = edges.cs_edges
    inside = [c for c in edges.sck_edges if fall < c < rise]
    assert len(inside) == 4 * 16 and inside == edges.sck_edges
    assert inside[0] - fall >= HALF                         # CS_n set-up
    assert rise - inside[-1] >= HALF                        # CS_n hold
    # Mode 0: each byte is 16 edges, leading then trailing. Every half period
    # is exactly HALF clocks (a multiple of the slot, so no dither) except the
    # one after a byte's last sample, where SCK waits while the master stores
    # the byte and fetches the next command from INQ.
    for byte in range(4):
        group = inside[byte * 16:(byte + 1) * 16]
        gaps = [b - a for a, b in zip(group, group[1:])]
        assert gaps[:14] == [HALF] * 14 and gaps[14] >= HALF
    assert [inside[i * 16] - inside[i * 16 - 1] for i in (1, 2, 3)] == [HALF] * 3


def test_full_duplex_returns_one_word_per_byte_without_skip():
    bench, flash, edges, loom = setup()
    loom.push(0, [0x9F, 0x00, LAST | 0x00])
    assert loom.pop(0, 3) == [0xFF, 0xEF, 0x40]
