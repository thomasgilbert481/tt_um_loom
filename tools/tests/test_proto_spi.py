"""tools.protomodels.spi: the master model against the flash and a custom device.

The master drives IN1 (SCK), IN2 (MOSI) and IN3 (CS_n); the device drives
MISO on IN0. No firmware runs; the chip only provides the pads.
"""

import pytest

from tools.protomodels.bench import Bench
from tools.protomodels.spi import SpiDevice, SpiFlash, SpiMaster

PINS = ("IN1", "IN2", "IN0", "IN3")          # SCK, MOSI, MISO, CS_n
MEMORY = bytes((i * 7 + 3) & 0xFF for i in range(512))


def run(bench, master, *transfers):
    for data in transfers:
        master.transfer(bytes(data))
    assert bench.run_until(lambda: not master.busy, 50000, every=16)
    bench.step(8)
    return master.results


@pytest.mark.parametrize("mode", range(4))
@pytest.mark.parametrize("lsb_first", [False, True])
def test_master_reads_the_flash_in_every_mode(mode, lsb_first):
    bench = Bench()
    flash = bench.add(SpiFlash(*PINS, memory=MEMORY, mode=mode, lsb_first=lsb_first))
    master = bench.add(SpiMaster(*PINS, mode=mode, lsb_first=lsb_first, half=5))
    jedec, read = run(bench, master, [0x9F, 0, 0, 0], [0x03, 0x00, 0x01, 0x0E, 0, 0, 0, 0])
    assert jedec == bytes([0xFF, 0xEF, 0x40, 0x16])
    assert read[4:] == MEMORY[0x10E:0x112]
    assert flash.commands == [0x9F, 0x03]


def test_read_wraps_at_the_end_of_memory():
    bench = Bench()
    bench.add(SpiFlash(*PINS, memory=MEMORY, size=512))
    master = bench.add(SpiMaster(*PINS))
    (read,) = run(bench, master, [0x03, 0x00, 0x01, 0xFF, 0, 0])
    assert read[4:] == bytes([MEMORY[511], MEMORY[0]])


def test_unknown_commands_answer_ff_and_are_logged():
    bench = Bench()
    flash = bench.add(SpiFlash(*PINS))
    master = bench.add(SpiMaster(*PINS))
    (out,) = run(bench, master, [0x05, 0x00])
    assert out == b"\xff\xff" and flash.commands == [0x05]


class Echo(SpiDevice):
    """Sends back each byte one position later, complemented."""

    def on_byte(self, index, value):
        return value ^ 0xFF


@pytest.mark.parametrize("mode", range(4))
def test_full_duplex_with_a_custom_device(mode):
    bench = Bench()
    dev = bench.add(Echo(*PINS, mode=mode))
    master = bench.add(SpiMaster(*PINS, mode=mode, half=4))
    (out,) = run(bench, master, [0x12, 0x34, 0x56])
    assert out == bytes([0xFF, 0xED, 0xCB])
    assert dev.transactions == [[(0x12, 0xFF), (0x34, 0xED), (0x56, 0xCB)]]


def test_miso_is_released_while_deselected():
    bench = Bench()
    dev = bench.add(Echo(*PINS))
    bench.add(SpiMaster(*PINS))                            # idle: CS_n high
    bench.step(10)
    assert not dev.selected
    assert bench.lines.get(bench.pad("IN0")) == 0          # nobody drives it
