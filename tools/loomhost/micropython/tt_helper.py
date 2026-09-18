# tt_helper.py: Loom host-port helper for the Tiny Tapeout demo board (RP2040).
# SPDX-License-Identifier: Apache-2.0
#
# TTBoardTransport pastes this into tt-micropython-firmware's raw REPL, then
# calls _lsetup(project, clock_hz, sck_hz) once and _lx('<hex>') per
# transaction. The RP2040's SPI0 function set lands on Loom's host pins:
#   GP17 CS_n = ui_in[4]  GP18 SCK = ui_in[5]  GP19 MOSI = ui_in[6]
#   GP16 MISO = uo_out[7] GP15 HOST_IRQ = uo_out[6]
# The firmware owns those GPIOs after project selection, so they are taken
# back here for the SPI peripheral. Not tested on hardware yet.
from machine import Pin, SPI


def _lsetup(project, clock_hz, sck_hz):
    global _spi, _cs, _irq
    try:
        tt = DemoBoard.get()                       # noqa: F821 (firmware global)
        getattr(tt.shuttle, project).enable()
        tt.clock_project_PWM(clock_hz)
        tt.reset_project(True)
        tt.reset_project(False)
    except Exception as exc:
        print("tt setup skipped:", exc)
    _cs = Pin(17, Pin.OUT, value=1)
    _irq = Pin(15, Pin.IN)
    _spi = SPI(0, baudrate=sck_hz, polarity=0, phase=0, bits=8,
               firstbit=SPI.MSB, sck=Pin(18), mosi=Pin(19), miso=Pin(16))


def _lx(hextx):
    tx = bytes.fromhex(hextx)
    rx = bytearray(len(tx))
    _cs.value(0)
    try:
        _spi.write_readinto(tx, rx)
    finally:
        _cs.value(1)
    print(rx.hex())


def _lirq():
    print(_irq.value())
