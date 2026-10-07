"""A Tiny Tapeout demo board's raw REPL in CPython, to run ``tt_helper.py``.

:class:`FakeTTBoard` is a serial-like object that answers
:class:`~tools.loomhost.serial_transport.TTBoardTransport` the way
tt-micropython-firmware's raw REPL does, and runs the code it is sent, the
helper ``tools/loomhost/micropython/tt_helper.py`` included, with stand-ins
for what that code imports on the board: ``machine.Pin``, ``machine.SPI``,
``time.sleep_us``, ``DemoBoard`` and the firmware's pin map,
``ttboard.pins.gpio_map.GPIOMap``. So the helper's own lines choose the
transfer (hardware SPI or bit-bang) and wiggle the pins, not a copy of them.

The pins go to a *gpio* object with three methods: ``set(n, level)`` drives
GPIO ``n``, ``get(n)`` reads it, ``sleep_us(us)`` lets time pass; an optional
``on_reset(active)`` hears ``DemoBoard.reset_project``.
:class:`BenchGpio` is the one that matters: it puts the GPIOs on the pads of
a clocked bench through the board's pin map, so on ``test/rtl_bench.RtlBench``
the helper clocks the RTL's real host port. A board-side pin call takes
``clocks_per_call`` chip clocks (0 by default: the fastest a CPU could be)
and ``sleep_us`` takes its microseconds at ``clk_hz``.

``machine.SPI`` only accepts pins that carry that SPI block's function on the
chip (``spi_function``: RP2040 and RP2350 share the GPIO bank 0 F1 pattern),
and its ``write_readinto`` clocks SPI mode 0 through the same gpio object, so
the RP2040 board's SPI0 path runs on the bench as well.

Pin maps (from tt-micropython-firmware, ``src/ttboard/pins/``):
:data:`RP2040_MAP` is ``GPIOMapTT04`` (v2.0.4 ``gpio_map.py``),
:data:`DBV3_MAP` is ``GPIOMapTTDBv3`` and :data:`DBV3_ALPHA_MAP` is
``GPIOMapTTDBv3Alpha`` (``gpio_map_dbv3.py`` on ``main``, d485c7a).
"""

from __future__ import annotations

import contextlib
import io
import sys
import time as _time
import traceback
import types
from typing import Dict, List, Optional

from tools.protomodels.bench import UI, UO, Drive, Model


def _pin_map(ui, uo) -> Dict[str, int]:
    names = {"UI_IN%d" % k: gpio for k, gpio in enumerate(ui)}
    names.update({"UO_OUT%d" % k: gpio for k, gpio in enumerate(uo)})
    return names


#: GPIOMapTT04: the RP2040 demo board (TT04 onwards).
RP2040_MAP = _pin_map(ui=(9, 10, 11, 12, 17, 18, 19, 20), uo=(5, 6, 7, 8, 13, 14, 15, 16))
#: GPIOMapTTDBv3: the v3 demo board (RP2350B).
DBV3_MAP = _pin_map(ui=range(17, 25), uo=range(33, 41))
#: GPIOMapTTDBv3Alpha: the v3 board's alpha revision.
DBV3_ALPHA_MAP = _pin_map(ui=range(12, 20), uo=range(30, 38))

#: Highest GPIO number per chip (RP2040: GP0..29, RP2350B: GP0..47).
CHIP_GPIOS = {"RP2040": 29, "RP2350B": 47}

_SPI_ROLES = ("RX", "CSn", "SCK", "TX")


def spi_function(gpio: int):
    """``(block, role)`` of a GPIO's F1 (SPI) function, e.g. ``(1, "RX")`` for
    GP40. Bank 0 repeats SPI0, SPI1 every eight pins, RX CSn SCK TX every four
    (RP2040 datasheet table 2; pico-sdk ``io_bank0.h`` for the RP2350)."""
    return (gpio // 8) % 2, _SPI_ROLES[gpio % 4]


class FakeTTBoard:
    """The raw REPL of a TT demo board running tt-micropython-firmware.

    Args:
        gpio: the pin world (``set``, ``get``, ``sleep_us``).
        gpio_map: the firmware's ``GPIOMap`` as ``{"UI_IN4": 21, ...}``, or
            None for firmware without ``ttboard.pins.gpio_map``.
        chip: ``"RP2040"`` or ``"RP2350B"``; sets which GPIOs exist.
        project: the shuttle project ``DemoBoard`` offers.
    """

    RAW_BANNER = b"raw REPL; CTRL-B to exit\r\n>"

    def __init__(self, gpio, gpio_map: Optional[Dict[str, int]] = DBV3_MAP, *,
                 chip: str = "RP2350B", project: str = "tt_um_loom") -> None:
        self.gpio = gpio
        self.gpio_map = gpio_map
        self.chip = chip
        self.max_gpio = CHIP_GPIOS[chip]
        self.project = project
        self.modes: Dict[int, str] = {}
        self.calls: List[tuple] = []          # DemoBoard calls, in order
        self.executed: List[str] = []
        self.out = bytearray()
        self.code = bytearray()
        self.raw = False
        self.closed = False
        self.namespace: dict = {}
        self.modules = self._modules()
        self.namespace["DemoBoard"] = self._demoboard()

    # ------------------------------------------------------------ serial
    def reset_input_buffer(self) -> None:
        self.out.clear()

    def write(self, data: bytes) -> None:
        for byte in data:
            if not self.raw:
                if byte == 0x01:
                    self.raw = True
                    self.out += self.RAW_BANNER
                continue
            if byte == 0x04:
                self._run(self.code.decode("utf-8"))
                self.code.clear()
            elif byte == 0x02:
                self.raw = False
            else:
                self.code.append(byte)

    def read(self, n: int) -> bytes:
        chunk = bytes(self.out[:n])
        del self.out[:n]
        return chunk

    def close(self) -> None:
        self.closed = True

    # -------------------------------------------------------------- exec
    def _run(self, code: str) -> None:
        self.executed.append(code)
        out, err = io.StringIO(), ""
        saved = {name: sys.modules.get(name) for name in self.modules}
        sys.modules.update(self.modules)
        try:
            with contextlib.redirect_stdout(out):
                exec(compile(code, "<stdin>", "exec"), self.namespace)
        except Exception as exc:                       # reported, as the board does
            err = "Traceback (most recent call last):\r\n" + "".join(
                traceback.format_exception_only(type(exc), exc)).replace("\n", "\r\n")
        finally:
            for name, module in saved.items():
                if module is None:
                    sys.modules.pop(name, None)
                else:
                    sys.modules[name] = module
        text = out.getvalue().replace("\n", "\r\n")
        self.out += b"OK" + text.encode() + b"\x04" + err.encode() + b"\x04>"

    # ------------------------------------------------------- the stand-ins
    def check_gpio(self, n: int) -> None:
        if not 0 <= n <= self.max_gpio:
            raise ValueError("invalid pin")         # MicroPython's message

    def _modules(self) -> Dict[str, types.ModuleType]:
        board = self
        gpio = self.gpio

        class Pin:
            IN, OUT, PULL_UP, PULL_DOWN = 0, 1, 1, 2

            def __init__(self, n, mode=None, pull=None, value=None):
                board.check_gpio(n)
                self.n = n
                if mode is not None:
                    board.modes[n] = "out" if mode == Pin.OUT else "in"
                if value is not None:
                    self.value(value)

            def value(self, v=None):
                if v is None:
                    return gpio.get(self.n)
                if board.modes.get(self.n) != "out":
                    raise AssertionError("GP%d written while not an output" % self.n)
                gpio.set(self.n, 1 if v else 0)
                return None

        class SPI:
            MSB, LSB = 0, 1

            def __init__(self, block, baudrate=1_000_000, polarity=0, phase=0, bits=8,
                         firstbit=0, sck=None, mosi=None, miso=None):
                for pin, role in ((sck, "SCK"), (mosi, "TX"), (miso, "RX")):
                    if spi_function(pin.n) != (block, role):
                        raise ValueError("GP%d is not SPI%d %s" % (pin.n, block, role))
                if (polarity, phase, bits, firstbit) != (0, 0, 8, SPI.MSB):
                    raise AssertionError("Loom needs SPI mode 0, 8 bits, MSB first")
                self.sck, self.mosi, self.miso = sck.n, mosi.n, miso.n
                board.modes[self.sck] = board.modes[self.mosi] = "out"
                board.modes[self.miso] = "in"
                self.baudrate = baudrate
                gpio.set(self.sck, 0)

            def write_readinto(self, tx, rx):
                half = 0.5e6 / self.baudrate
                for i, byte in enumerate(bytes(tx)):
                    r = 0
                    for k in range(8):
                        gpio.set(self.mosi, (byte >> (7 - k)) & 1)
                        gpio.sleep_us(half)
                        r = (r << 1) | gpio.get(self.miso)
                        gpio.set(self.sck, 1)
                        gpio.sleep_us(half)
                        gpio.set(self.sck, 0)
                    rx[i] = r

        machine = types.ModuleType("machine")
        machine.Pin, machine.SPI = Pin, SPI

        class _Time(types.ModuleType):
            """CPython's ``time`` plus MicroPython's ``sleep_us``."""

            def __getattr__(self, name):
                return getattr(_time, name)

        utime = _Time("time")
        utime.sleep_us = gpio.sleep_us
        modules = {"machine": machine, "time": utime}
        if self.gpio_map is not None:
            pkg = types.ModuleType("ttboard")
            pins = types.ModuleType("ttboard.pins")
            gmap = types.ModuleType("ttboard.pins.gpio_map")
            gmap.GPIOMap = type("GPIOMap", (), dict(self.gpio_map))
            pkg.pins, pins.gpio_map = pins, gmap
            modules.update({"ttboard": pkg, "ttboard.pins": pins,
                            "ttboard.pins.gpio_map": gmap})
        return modules

    def _demoboard(self):
        board = self

        class Design:
            def enable(self):
                board.calls.append(("enable", board.project))

        class Board:
            shuttle = types.SimpleNamespace(**{self.project: Design()})

            def clock_project_PWM(self, hz):
                board.calls.append(("clock", hz))

            def reset_project(self, active):
                board.calls.append(("reset", bool(active)))
                hook = getattr(board.gpio, "on_reset", None)
                if hook is not None:
                    hook(bool(active))

        class DemoBoard:
            @staticmethod
            def get():
                return Board()

        return DemoBoard


class BenchGpio(Model):
    """Demo-board GPIOs on the pads of a clocked bench.

    ``ui_in`` GPIOs (per ``gpio_map``) are driven onto the bench's ``ui``
    pads from the cycle after the write; ``uo_out`` GPIOs read the pad as the
    last bench cycle showed it. Each ``set`` or ``get`` advances the bench
    ``clocks_per_call`` cycles, each ``sleep_us`` its microseconds at
    ``clk_hz`` (rounded up). ``bench`` needs ``add``, ``step`` and ``lines``.
    """

    def __init__(self, bench, gpio_map: Dict[str, int], *, clk_hz: int = 50_000_000,
                 clocks_per_call: int = 0) -> None:
        self.bench = bench
        self.clk_hz = clk_hz
        self.clocks_per_call = clocks_per_call
        self.ui = {gpio_map["UI_IN%d" % k]: k for k in range(8)}
        self.uo = {gpio_map["UO_OUT%d" % k]: k for k in range(8)}
        self.levels: Dict[int, int] = {}
        bench.add(self)

    def drive(self, drive: Drive, cycle: int) -> None:
        for bit, level in self.levels.items():
            drive.set((UI, bit), level)

    def set(self, gpio: int, level: int) -> None:
        if gpio not in self.ui:
            raise AssertionError("GP%d is not a ui_in pin of this board" % gpio)
        self.levels[self.ui[gpio]] = level
        self.bench.step(self.clocks_per_call)

    def get(self, gpio: int) -> int:
        if gpio in self.uo:
            level = self.bench.lines.get((UO, self.uo[gpio]))
        elif gpio in self.ui:
            level = self.levels.get(self.ui[gpio], 0)
        else:
            raise AssertionError("GP%d is not a project pin of this board" % gpio)
        self.bench.step(self.clocks_per_call)
        return level

    def sleep_us(self, us) -> None:
        clocks = us * self.clk_hz / 1e6
        self.bench.step(int(clocks) + (clocks > int(clocks)))
