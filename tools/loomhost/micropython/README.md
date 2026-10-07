# MicroPython side of the Loom host transports

Two short programs that run on a Raspberry Pi microcontroller and move SPI
transactions for `tools/loomhost`. Neither has run on hardware yet. The
Python side of both transports is unit-tested against fake serial ports
(`tools/tests/test_loomhost_serial.py`), and `tt_helper.py` itself runs in
CPython through `tools/loomhost/ttboard_sim.py`: against a checking SPI
slave (`tools/tests/test_ttboard_helper.py`) and against the RTL's host
port (`test/test_host.py`, `test_ttboard_helper_*`).

## `pico_bridge.py`: a Raspberry Pi Pico as a USB-to-SPI bridge

For any Loom whose host pins are wired out (it was written for the FPGA
prototype, which D-024 dropped). Copy it to
the Pico as `main.py` (`mpremote cp pico_bridge.py :main.py`), then:

```python
from tools.loomhost import Loom, PicoTransport
loom = Loom(PicoTransport("COM7"))      # or /dev/ttyACM0
print(hex(loom.id()))                   # 0x4c4d
```

Wiring uses SPI0 on the same GPIO numbers as the Tiny Tapeout demo board:

| Pico | Loom | TT pad |
|---|---|---|
| GP17 (out) | HOST_CS_n | `ui_in[4]` |
| GP18 (out) | HOST_SCK | `ui_in[5]` |
| GP19 (out) | HOST_MOSI | `ui_in[6]` |
| GP16 (in) | HOST_MISO | `uo_out[7]` |
| GP20 (in) | HOST_IRQ | `uo_out[6]` |

SCK must be at most the Loom clock divided by 8 (HOST_PROTOCOL, Electrical);
the bridge starts at 1 MHz and `PicoTransport(..., sck_hz=...)` changes it.
The line protocol is documented at the top of `serial_transport.py`.

## `tt_helper.py`: the Tiny Tapeout demo boards

tt-micropython-firmware has no SPI driver for project pins, so
`TTBoardTransport` enters the board's raw REPL, pastes this helper, calls
`_lsetup(project, clock_hz, sck_hz)` (select the project and start its
clock, take Loom's host pins back from the firmware with CS_n high and SCK
low, then pulse the project reset) and runs `_lx('<hex>')` once per
transaction:

```python
from tools.loomhost import Loom, TTBoardTransport
loom = Loom(TTBoardTransport("COM5", clock_hz=50_000_000, sck_hz=1_000_000))
```

The helper reads the GPIO numbers of `ui_in[4..6]` and `uo_out[6..7]` from
the firmware's own map (`GPIOMap` in `ttboard.pins.gpio_map`), so it
follows the board it is on, and picks the transfer from them:

| Board | CS_n | SCK | MOSI | MISO | HOST_IRQ | Transfer |
|---|---|---|---|---|---|---|
| RP2040 demo board | GP17 | GP18 | GP19 | GP16 | GP15 | hardware SPI0 |
| v3 demo board (RP2350B) | GP21 | GP22 | GP23 | GP40 | GP39 | bit-bang |

On the v3 board GP21, GP22 and GP23 carry SPI0 functions but GP40 is SPI1
RX (pico-sdk `rp2350/hardware_regs/include/hardware/regs/io_bank0.h`), so
no one SPI block reaches all four pins, and the helper bit-bangs SPI mode 0
with `machine.Pin`. A PIO program would have to move a PIO block's GPIO
window up to reach GP40 and would share the PIO blocks with the firmware's
own clock generator; bit-banging needs neither. `TTBoardTransport(...,
mode="spi" | "bitbang")` overrides the choice, and `.helper_mode` and
`.helper_pins` say what the board chose.

Timing holds by construction at any allowed SCK. The helper waits half an
SCK period, rounded up to a whole microsecond, in every SCK phase, before
the first rise (CS_n setup), after the last fall (CS_n hold) and after CS_n
rises (the gap before the next transaction), whatever the pin calls cost.
The transport refuses an SCK above the Loom clock divided by 8. A
bit-banged byte therefore takes at least 16 microseconds, about 60 kB/s at
best; the time each pin call adds on the RP2350 has not been measured.
