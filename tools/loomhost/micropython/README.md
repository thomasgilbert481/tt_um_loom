# MicroPython side of the Loom host transports

Two short programs that run on an RP2040 and move SPI transactions for
`tools/loomhost`. Neither has run on hardware yet; the Python side of both
transports is unit-tested against fake serial ports
(`tools/tests/test_loomhost_serial.py`).

## `pico_bridge.py`: a Raspberry Pi Pico as a USB-to-SPI bridge

For the FPGA prototype, or any Loom whose host pins are wired out. Copy it to
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

## `tt_helper.py`: the Tiny Tapeout demo board

On the TT demo board the RP2040 already sits on the project pins, and its
SPI0 function set lands exactly on Loom's host pins (GP17 CS_n, GP18 SCK,
GP19 MOSI, GP16 MISO; HOST_IRQ on GP15). tt-micropython-firmware has no SPI
driver for project pins, so `TTBoardTransport` enters the board's raw REPL,
pastes this helper, calls `_lsetup(project, clock_hz, sck_hz)` (select the
project, start its clock, pulse its reset, then take GP16..GP19 back for SPI0)
and runs `_lx('<hex>')` once per transaction:

```python
from tools.loomhost import Loom, TTBoardTransport
loom = Loom(TTBoardTransport("COM5", clock_hz=50_000_000, sck_hz=1_000_000))
```

The v3 demo board (RP2350B) maps project pins differently; it needs a PIO or
bit-banged variant of `_lx`, which is future work.
