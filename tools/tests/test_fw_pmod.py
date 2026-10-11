"""Pmod builds on the model and the RTL: shipped programs moved to Tiny
Tapeout's recommended pinouts (https://tinytapeout.com/specs/pinouts/) with
the assembler's pin overrides (``--pin NAME=PIN``, D-035), no source edited.

* UART on the demo board's own USB-UART link: the design's RX on ``ui_in[3]``
  (IN3) and TX on ``uo_out[4]`` (OUT4);
* UART on the UART Pmod's top row: TX on ``uio[1]``, RX on ``uio[2]``;
* I2C on the I2C Pmod's top row: SCL ``uio[2]``, SDA ``uio[3]``;
* SPI master on the SPI Pmod's top row, which is also the QSPI Pmod's flash
  wiring: CS ``uio[0]``, MOSI ``uio[1]``, MISO ``uio[2]``, SCK ``uio[3]``,
  reading a flash's JEDEC ID.

Every body takes a ``backend`` (``tools/tests/fw_backend.py``), so the same
bodies run on the golden model under pytest and on the RTL in
``test/test_fw.py``. ``PMODS`` is the table ``firmware/README.md`` prints.
"""

import pytest

from tools.loomasm import assemble_file
from tools.loomhost.rates import tick
from tools.loomisa import REPO, load
from tools.protomodels.i2c import I2cEeprom, I2cMonitor
from tools.protomodels.spi import SpiFlash
from tools.protomodels.uart import UartRx, UartTx

ISA = load()
FIRMWARE = REPO / "firmware"
CLOCK_HZ = 50e6

#: build -> (program, pin overrides)
PMODS = {
    "uart_tx_board": ("uart_tx_fifo", {"TX": "OUT4"}),
    "uart_rx_board": ("uart_rx", {"RX": "IN3"}),
    "uart_tx_pmod": ("uart_tx_fifo", {"TX": "BIDIR1"}),
    "uart_rx_pmod": ("uart_rx", {"RX": "BIDIR2"}),
    "i2c_pmod": ("i2c_master", {"SCL": "BIDIR2", "SDA": "BIDIR3"}),
    "spi_pmod": ("spi_master", {"CS_N": "BIDIR0", "MOSI": "BIDIR1",
                                "MISO": "BIDIR2", "SCK": "BIDIR3"}),
}

START, STOP, READ, NACK = 0x100, 0x200, 0x400, 0x800     # i2c_master
R_NACK = 0x8000
SKIP, LAST = 0x4000, 0x8000                              # spi_master


def boot(backend, bench, build, tick_int, tick_frac=0):
    source, pins = PMODS[build]
    program = assemble_file(FIRMWARE / (source + ".loom"), isa=ISA, strict=True, pins=pins)
    loom = backend.loom(bench, isa=ISA)
    loom.load(program)
    loom.write_csr(0, "TICK_INT", tick_int)
    loom.write_csr(0, "TICK_FRAC", tick_frac)
    loom.run(0)
    return loom


@pytest.mark.parametrize("build, pin", [("uart_tx_board", "OUT4"), ("uart_tx_pmod", "BIDIR1")])
def test_uart_tx_on_the_board_link_and_the_uart_pmod(backend, build, pin):
    """115200 baud, the rate a host terminal expects."""
    tick_int, tick_frac = tick(CLOCK_HZ, 115200)
    bits = tick_int + tick_frac / 256
    bench = backend.bench()
    rx = bench.add(UartRx(pin, bits))
    loom = boot(backend, bench, build, tick_int, tick_frac)
    loom.push(0, b"Hi")
    assert bench.run_until(lambda: len(rx.frames) == 2, int(25 * bits) + 4000, every=256)
    assert rx.data == b"Hi" and not rx.errors
    assert bench.contention_free() and loom.badop() == 0


@pytest.mark.parametrize("build, pin", [("uart_rx_board", "IN3"), ("uart_rx_pmod", "BIDIR2")])
def test_uart_rx_on_the_board_link_and_the_uart_pmod(backend, build, pin):
    """The receiver's fastest tested rate, 64 clocks per bit (TICK 8)."""
    bench = backend.bench()
    tx = bench.add(UartTx(pin, 64))
    loom = boot(backend, bench, build, 8)
    bench.step(200)
    tx.send(b"\x5a\xa5")
    assert bench.run_until(lambda: not tx.busy, 4000, every=64)
    bench.step(200)
    assert loom.pop(0, 2) == [0x5A, 0xA5]
    assert loom.badop() == 0


def test_i2c_on_the_i2c_pmod(backend):
    """A write and a random read back through a 24C02 on uio[2] and uio[3]."""
    bench = backend.bench(pullups=0b1100)
    ee = bench.add(I2cEeprom("BIDIR2", "BIDIR3", memory=bytes(256)))
    bench.add(I2cMonitor("BIDIR2", "BIDIR3"))
    loom = boot(backend, bench, "i2c_pmod", 8)
    loom.push(0, [START | 0xA0, 0x10, 0x11, STOP | 0x22])
    assert loom.pop(0, 4) == [0xA0, 0x10, 0x11, 0x22]
    loom.push(0, [START | 0xA0, 0x10, START | 0xA1, READ, READ | NACK | STOP])
    assert loom.pop(0, 5) == [0xA0, 0x10, 0xA1, 0x11, R_NACK | 0x22]
    assert bytes(ee.memory[0x10:0x12]) == b"\x11\x22"
    assert bench.lines.uio_oe & 0b0011 == 0                  # uio[0..1] untouched
    assert bench.contention_free() and loom.badop() == 0


def test_spi_master_reads_a_flash_id_on_the_spi_pmod(backend):
    """JEDEC ID (0x9F) from a flash wired as the QSPI Pmod wires its flash."""
    bench = backend.bench()
    bench.add(SpiFlash("BIDIR3", "BIDIR1", "BIDIR2", "BIDIR0", jedec_id=(0xEF, 0x40, 0x18)))
    loom = boot(backend, bench, "spi_pmod", 32)
    loom.push(0, [SKIP | 0x9F, 0x00, 0x00, LAST | 0x00])
    assert loom.pop(0, 3) == [0xEF, 0x40, 0x18]
    assert bench.lines.uio_oe & 0b1111 == 0b1011             # CS, MOSI, SCK out; MISO in
    assert bench.contention_free() and loom.badop() == 0
