"""tt_helper.py itself, run in CPython by tools/loomhost/ttboard_sim.py.

TTBoardTransport pastes the real helper into FakeTTBoard, which runs it with
stand-ins for the firmware's machine, time, DemoBoard and pin map. The far
end here is a pin-level SPI slave that checks mode 0 and the HOST_PROTOCOL
timing and loops each byte back one byte later. test/test_host.py runs the
same helper against the RTL's host port.
"""

import pytest

from tools.loomhost import TransportError, TTBoardTransport
from tools.loomhost.ttboard_sim import (DBV3_ALPHA_MAP, DBV3_MAP, RP2040_MAP, FakeTTBoard,
                                        spi_function)

FIRST = 0xA5          # what the slave sends while the first byte comes in


class LoopbackSlave:
    """An SPI mode 0 slave on GPIO numbers, with time from ``sleep_us`` only.

    Asserts: SCK low whenever CS_n moves; MOSI and MISO only sampled on SCK
    rises with CS_n low; every SCK phase, the CS_n setup before the first
    rise, the hold after the last fall and the CS_n high gap at least
    ``min_us``. MISO shifts out the previous byte received (``FIRST`` for
    the first), changing on SCK falls as Loom's does.
    """

    def __init__(self, pins, min_us):
        self.cs, self.sck, self.mosi, self.miso, self.irq = pins
        self.min_us = min_us
        self.level = {self.cs: 1, self.sck: 0, self.mosi: 0}
        self.now = 0.0
        self.edge = -1e9            # time of the last CS_n or SCK edge
        self.irq_level = 0
        self.transactions = []
        self.resets = []
        self._rx = self._tx = None

    def _gap(self, what):
        assert self.now - self.edge >= self.min_us - 1e-9, \
            "%s after %.3f us, needs %.3f" % (what, self.now - self.edge, self.min_us)
        self.edge = self.now

    def set(self, n, v):
        assert n in self.level, "GP%d is not CS_n, SCK or MOSI" % n
        old, self.level[n] = self.level[n], v
        if old == v:
            return
        if n == self.cs:
            assert self.level[self.sck] == 0, "CS_n moved with SCK high"
            self._gap("CS_n edge")
            if v == 0:
                self._rx, self._tx, self._bits, self._cur = [], FIRST, 0, 0
            else:
                assert self._bits % 8 == 0, "CS_n rose mid-byte"
                tx = bytes(self._rx)
                rx = bytes([FIRST]) + tx[:-1] if tx else b""
                self.transactions.append((tx, rx))
        elif n == self.sck:
            assert self.level[self.cs] == 0, "SCK moved with CS_n high"
            self._gap("SCK edge")
            if v:                                     # rise: sample MOSI
                self._cur = (self._cur << 1) | self.level[self.mosi]
                self._bits += 1
                if self._bits % 8 == 0:
                    self._rx.append(self._cur & 0xFF)
            elif self._bits % 8 == 0:                 # fall after a byte: next byte out
                self._tx = self._rx[-1]

    def get(self, n):
        if n == self.irq:
            return self.irq_level
        assert n == self.miso, "GP%d read" % n
        if self.level[self.cs]:
            return 0                                  # Loom drives MISO 0 when idle
        return (self._tx >> (7 - self._bits % 8)) & 1

    def sleep_us(self, us):
        self.now += us

    def on_reset(self, active):
        self.resets.append((active, self.level[self.cs], self.level[self.sck]))


def transport(board_map, chip, *, clock_hz=50_000_000, sck_hz=1_000_000, mode=None,
              pins=(21, 22, 23, 40, 39)):
    slave = LoopbackSlave(pins, min_us=0.5e6 / sck_hz)
    board = FakeTTBoard(slave, board_map, chip=chip)
    t = TTBoardTransport(serial=board, clock_hz=clock_hz, sck_hz=sck_hz, mode=mode)
    return t, board, slave


def test_spi_functions_match_the_datasheets():
    # RP2040 board: Loom's pins are SPI0 CSn, SCK, TX, RX (D-012).
    assert [spi_function(n) for n in (17, 18, 19, 16)] == \
        [(0, "CSn"), (0, "SCK"), (0, "TX"), (0, "RX")]
    # v3 board: GP21..23 are SPI0, GP40 is SPI1 RX (pico-sdk rp2350 io_bank0.h).
    assert [spi_function(n) for n in (21, 22, 23, 40)] == \
        [(0, "CSn"), (0, "SCK"), (0, "TX"), (1, "RX")]


def test_v3_board_bitbangs_on_the_firmware_map():
    t, board, slave = transport(DBV3_MAP, "RP2350B")
    assert (t.helper_mode, t.helper_pins) == ("bitbang", (21, 22, 23, 40, 39))
    assert board.calls == [("enable", "tt_um_loom"), ("clock", 50_000_000),
                           ("reset", True), ("reset", False)]
    # CS_n high and SCK low through the project reset (HOST_PROTOCOL, F-3).
    assert slave.resets == [(True, 1, 0), (False, 1, 0)]
    assert t.transfer(b"\x12\x34\x56\xff") == bytes([FIRST, 0x12, 0x34, 0x56])
    assert t.transfer(b"\x80\x01") == bytes([FIRST, 0x80])
    assert slave.transactions[0][0] == b"\x12\x34\x56\xff"
    assert len(slave.transactions) == 2


@pytest.mark.parametrize("clock_hz, sck_hz", [(50_000_000, 6_250_000),
                                              (50_000_000, 1_000_000),
                                              (2_000_000, 250_000),
                                              (1_000_000, 37_000)])
def test_bitbang_timing_holds_at_any_allowed_sck(clock_hz, sck_hz):
    t, _, slave = transport(DBV3_MAP, "RP2350B", clock_hz=clock_hz, sck_hz=sck_hz)
    tx = bytes(range(0, 256, 17))
    assert t.transfer(tx) == bytes([FIRST]) + tx[:-1]
    assert t.transfer(b"\x00") == bytes([FIRST])


def test_v3_alpha_map_is_followed():
    t, _, _ = transport(DBV3_ALPHA_MAP, "RP2350B", pins=(16, 17, 18, 37, 36))
    assert (t.helper_mode, t.helper_pins) == ("bitbang", (16, 17, 18, 37, 36))
    assert t.transfer(b"\x3c\xc3") == bytes([FIRST, 0x3C])


def test_rp2040_board_keeps_hardware_spi0():
    t, _, slave = transport(RP2040_MAP, "RP2040", pins=(17, 18, 19, 16, 15))
    assert (t.helper_mode, t.helper_pins) == ("spi", (17, 18, 19, 16, 15))
    assert t.transfer(b"\x01\x02\x03") == bytes([FIRST, 0x01, 0x02])
    slave.irq_level = 1
    assert t.irq() is True


@pytest.mark.parametrize("chip, pins, mode", [
    ("RP2350B", (21, 22, 23, 40, 39), "bitbang"),
    ("RP2040", (17, 18, 19, 16, 15), "spi")])
def test_without_the_firmware_map_gp40_tells_the_boards_apart(chip, pins, mode):
    t, _, _ = transport(None, chip, pins=pins)
    assert (t.helper_mode, t.helper_pins) == (mode, pins)
    assert t.transfer(b"\x5a\x00") == bytes([FIRST, 0x5A])


def test_hardware_spi_cannot_reach_gp40():
    with pytest.raises(TransportError, match="GP40 is not SPI0 RX"):
        transport(DBV3_MAP, "RP2350B", mode="spi")


def test_v3_irq_is_gp39():
    t, _, slave = transport(DBV3_MAP, "RP2350B")
    assert t.irq() is False
    slave.irq_level = 1
    assert t.irq() is True


@pytest.mark.parametrize("clock_hz, sck_hz", [(4_000_000, 1_000_000), (50_000_000, 0)])
def test_sck_above_clock_over_8_is_refused(clock_hz, sck_hz):
    with pytest.raises(ValueError, match="clock / 8"):
        TTBoardTransport(serial=object(), clock_hz=clock_hz, sck_hz=sck_hz)


def test_unknown_mode_is_refused():
    with pytest.raises(ValueError, match="bitbang"):
        TTBoardTransport(serial=object(), mode="pio")
