"""Hardware transports over USB serial. ``pyserial`` is imported lazily.

:class:`PicoTransport` talks to a Raspberry Pi Pico running
``tools/loomhost/micropython/pico_bridge.py``, a line protocol:

====================  ====================  =================================
host sends            bridge answers        meaning
====================  ====================  =================================
``H``                 ``LOOMBRIDGE <n>``    hello, protocol version ``n``
``X <hex>``           ``R <hex>``           one transaction, CS low around it
``I``                 ``Q 0`` / ``Q 1``     level of HOST_IRQ
``S <hz>``            ``OK``                set the SCK frequency
anything wrong        ``E <message>``       error
====================  ====================  =================================

:class:`TTBoardTransport` talks to the Tiny Tapeout demo board's RP2040 over
its MicroPython REPL (tt-micropython-firmware), in raw-REPL mode: it pastes
``tools/loomhost/micropython/tt_helper.py`` once and then calls its
``_lx('<hex>')`` per transaction. The helper drives the host port through the
RP2040's SPI0, which lands exactly on Loom's host pins (HOST_PROTOCOL,
Electrical): CS_n ``ui_in[4]`` = GP17, SCK ``ui_in[5]`` = GP18, MOSI
``ui_in[6]`` = GP19, MISO ``uo_out[7]`` = GP16; HOST_IRQ ``uo_out[6]`` = GP15.

Both accept an already-open serial-like object (``serial=``), which is how
the unit tests drive them with a fake.
"""

from __future__ import annotations

import pathlib
from typing import Optional

from .transport import Transport, TransportError

MICROPYTHON_DIR = pathlib.Path(__file__).resolve().parent / "micropython"
BRIDGE_PROTOCOL = 1


def _open_serial(port: str, baudrate: int, timeout: float):
    try:
        import serial  # pyserial, only needed for real hardware
    except ImportError as exc:                           # pragma: no cover
        raise TransportError("pyserial is not installed: pip install pyserial") from exc
    return serial.Serial(port, baudrate=baudrate, timeout=timeout)


class PicoTransport(Transport):
    """USB serial to a Pico running the ``pico_bridge.py`` line protocol."""

    def __init__(self, port: Optional[str] = None, *, baudrate: int = 115200,
                 timeout: float = 2.0, sck_hz: Optional[int] = None,
                 serial=None, hello: bool = True) -> None:
        if serial is None:
            if port is None:
                raise TransportError("PicoTransport needs a port or a serial object")
            serial = _open_serial(port, baudrate, timeout)
        self.serial = serial
        if hello:
            reply = self._command("H")
            if not reply.startswith("LOOMBRIDGE"):
                raise TransportError("no Loom bridge on the port (got %r)" % reply)
            self.bridge_version = int(reply.split()[1])
            if self.bridge_version != BRIDGE_PROTOCOL:
                raise TransportError("bridge speaks protocol %d, host %d"
                                     % (self.bridge_version, BRIDGE_PROTOCOL))
        if sck_hz is not None:
            self.set_sck(sck_hz)

    def _command(self, line: str) -> str:
        self.serial.write((line + "\n").encode("ascii"))
        raw = self.serial.readline()
        if not raw:
            raise TransportError("bridge timed out after %r" % line[:20])
        reply = raw.decode("ascii", "replace").strip()
        if reply.startswith("E"):
            raise TransportError("bridge error: " + reply[1:].strip())
        return reply

    def set_sck(self, hz: int) -> None:
        if self._command("S %d" % int(hz)) != "OK":
            raise TransportError("bridge refused SCK %d Hz" % hz)

    def transfer(self, tx: bytes) -> bytes:
        tx = bytes(tx)
        reply = self._command("X " + tx.hex())
        if not reply.startswith("R"):
            raise TransportError("unexpected bridge reply %r" % reply[:40])
        rx = bytes.fromhex(reply[1:].strip())
        if len(rx) != len(tx):
            raise TransportError("bridge returned %d bytes for %d" % (len(rx), len(tx)))
        return rx

    def irq(self) -> Optional[bool]:
        reply = self._command("I")
        if not reply.startswith("Q"):
            raise TransportError("unexpected bridge reply %r" % reply[:40])
        return reply.split()[1] == "1"

    def close(self) -> None:
        self.serial.close()


class TTBoardTransport(Transport):
    """The Tiny Tapeout demo board's RP2040, through its MicroPython raw REPL."""

    RAW_PROMPT = b"raw REPL; CTRL-B to exit\r\n>"

    def __init__(self, port: Optional[str] = None, *, baudrate: int = 115200,
                 timeout: float = 5.0, project: str = "tt_um_loom",
                 clock_hz: int = 50_000_000, sck_hz: int = 1_000_000,
                 serial=None, setup: bool = True) -> None:
        if serial is None:
            if port is None:
                raise TransportError("TTBoardTransport needs a port or a serial object")
            serial = _open_serial(port, baudrate, timeout)
        self.serial = serial
        self.clk_hz = clock_hz
        self._enter_raw_repl()
        if setup:
            helper = (MICROPYTHON_DIR / "tt_helper.py").read_text(encoding="utf-8")
            self.exec(helper)
            self.exec("_lsetup(%r, %d, %d)" % (project, int(clock_hz), int(sck_hz)))

    # ---------------------------------------------------------- raw REPL
    def _read_until(self, token: bytes) -> bytes:
        data = bytearray()
        while not data.endswith(token):
            chunk = self.serial.read(1)
            if not chunk:
                raise TransportError("REPL timed out waiting for %r" % token)
            data += chunk
        return bytes(data)

    def _enter_raw_repl(self) -> None:
        self.serial.write(b"\r\x03\x03")                  # interrupt anything running
        self.serial.reset_input_buffer()
        self.serial.write(b"\r\x01")                      # Ctrl-A: raw REPL
        self._read_until(self.RAW_PROMPT)

    def exec(self, code: str) -> str:
        """Run ``code`` on the board; return its stdout, raise on an exception."""
        self.serial.write(code.encode("utf-8") + b"\x04")
        head = self._read_until(b"OK")
        if not head.endswith(b"OK"):                       # pragma: no cover
            raise TransportError("raw REPL did not accept the code")
        out = self._read_until(b"\x04")[:-1]
        err = self._read_until(b"\x04")[:-1]
        self._read_until(b">")
        if err.strip():
            raise TransportError("board raised: " + err.decode("utf-8", "replace").strip())
        return out.decode("utf-8", "replace")

    # ---------------------------------------------------------- transport
    def transfer(self, tx: bytes) -> bytes:
        tx = bytes(tx)
        out = self.exec("_lx(%r)" % tx.hex()).strip()
        rx = bytes.fromhex(out)
        if len(rx) != len(tx):
            raise TransportError("board returned %d bytes for %d" % (len(rx), len(tx)))
        return rx

    def irq(self) -> Optional[bool]:
        return self.exec("_lirq()").strip() == "1"

    def close(self) -> None:
        try:
            self.serial.write(b"\x02")                    # Ctrl-B: friendly REPL
        finally:
            self.serial.close()
