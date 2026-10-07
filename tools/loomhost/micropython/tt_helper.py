# tt_helper.py: Loom host-port helper for the Tiny Tapeout demo boards.
# SPDX-License-Identifier: Apache-2.0
#
# TTBoardTransport pastes this into tt-micropython-firmware's raw REPL, then
# calls _lsetup(project, clock_hz, sck_hz) once and _lx('<hex>') per
# transaction. Loom's host pins are project pins (HOST_PROTOCOL, Electrical):
#   CS_n = ui_in[4]  SCK = ui_in[5]  MOSI = ui_in[6]
#   MISO = uo_out[7] HOST_IRQ = uo_out[6]
# Their GPIO numbers come from the firmware's own map (GPIOMap in
# ttboard.pins.gpio_map), so the helper follows the board it runs on:
# - RP2040 demo board: GP17 CS_n, GP18 SCK, GP19 MOSI, GP16 MISO, GP15 IRQ.
#   That is the RP2040's SPI0 function set, so SPI0 moves the bytes.
# - v3 demo board (RP2350B): GP21 CS_n, GP22 SCK, GP23 MOSI, GP40 MISO,
#   GP39 IRQ. GP21..23 are SPI0 functions but GP40 is SPI1 RX, so no one
#   SPI block fits, and the helper bit-bangs SPI mode 0 on the pins.
# The firmware owns those GPIOs after project selection, so they are taken
# back here, with CS_n high and SCK low before the project reset. Not tested
# on hardware yet.
from machine import Pin, SPI
import time

_RP2040_SPI0 = (17, 18, 19, 16)            # CS_n, SCK, MOSI, MISO
_spi = None


def _lpins():
    # (CS_n, SCK, MOSI, MISO, IRQ) as GPIO numbers.
    try:
        import ttboard.pins.gpio_map as gm
        m = gm.GPIOMap
        return (m.UI_IN4, m.UI_IN5, m.UI_IN6, m.UO_OUT7, m.UO_OUT6)
    except Exception:
        pass
    try:
        Pin(40, Pin.IN)                    # the firmware's own RP2350B probe
        return (21, 22, 23, 40, 39)
    except Exception:
        return (17, 18, 19, 16, 15)


def _lsetup(project, clock_hz, sck_hz, mode=None):
    global _spi, _cs, _sck, _mosi, _miso, _irq, _half
    tt = None
    try:
        tt = DemoBoard.get()                       # noqa: F821 (firmware global)
        getattr(tt.shuttle, project).enable()
        tt.clock_project_PWM(clock_hz)
    except Exception as exc:
        tt = None
        print("tt setup skipped:", exc)
    cs, sck, mosi, miso, irq = _lpins()
    if mode is None:
        mode = "spi" if (cs, sck, mosi, miso) == _RP2040_SPI0 else "bitbang"
    _cs = Pin(cs, Pin.OUT, value=1)
    _irq = Pin(irq, Pin.IN)
    # Half an SCK period in whole microseconds, rounded up: the shortest SCK
    # phase, CS_n setup, CS_n hold and CS_n high gap, however fast the pin
    # calls are.
    _half = (500000 + sck_hz - 1) // sck_hz
    if mode == "spi":
        _spi = SPI(0, baudrate=sck_hz, polarity=0, phase=0, bits=8,
                   firstbit=SPI.MSB, sck=Pin(sck), mosi=Pin(mosi), miso=Pin(miso))
    else:
        _spi = None
        _sck = Pin(sck, Pin.OUT, value=0)
        _mosi = Pin(mosi, Pin.OUT, value=0)
        _miso = Pin(miso, Pin.IN)
    if tt is not None:
        tt.reset_project(True)
        tt.reset_project(False)
    print(mode, cs, sck, mosi, miso, irq)


def _lbb(tx):
    # SPI mode 0, MSB first: MOSI changes while SCK is low, both sides
    # sample on the rise, and Loom moves MISO after the fall.
    cs = _cs.value
    sck = _sck.value
    mosi = _mosi.value
    miso = _miso.value
    wait = time.sleep_us
    half = _half
    rx = bytearray(len(tx))
    cs(0)
    try:
        for i in range(len(tx)):
            b = tx[i]
            r = 0
            for _ in range(8):
                mosi((b >> 7) & 1)
                b <<= 1
                wait(half)
                r = (r << 1) | miso()
                sck(1)
                wait(half)
                sck(0)
            rx[i] = r
        wait(half)
    finally:
        sck(0)
        cs(1)
        wait(half)
    return rx


def _lx(hextx):
    tx = bytes.fromhex(hextx)
    if _spi is None:
        rx = _lbb(tx)
    else:
        # SPI0 starts each bit half a period before its rise; the CS_n hold
        # after the last fall and the gap after the rise are waited here.
        rx = bytearray(len(tx))
        _cs.value(0)
        try:
            _spi.write_readinto(tx, rx)
            time.sleep_us(_half)
        finally:
            _cs.value(1)
            time.sleep_us(_half)
    print(rx.hex())


def _lirq():
    print(_irq.value())
