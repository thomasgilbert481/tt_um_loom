"""Loom host library: talk to a Loom chip over its SPI host port.

``docs/HOST_PROTOCOL.md`` defines the wire format; :mod:`.protocol` encodes
it, :class:`Loom` is the high-level API and the transports carry the bytes:

* :class:`ModelTransport` - the golden model (``tools.loomsim``), clock by
  clock, with pin models attached through ``tools.protomodels.bench.Bench``;
* :class:`SimTransport` - the RTL: the bytes are clocked through the SPI pads
  of a bench that steps the simulated chip (``test/rtl_bench.py``);
* :class:`PicoTransport` - USB serial to a Raspberry Pi Pico bridge;
* :class:`TTBoardTransport` - the Tiny Tapeout demo board's RP2040 REPL.

The serial transports import ``pyserial`` only when they open a port.
Command line smoke tool: ``python -m tools.loomhost --help``.
"""

from . import protocol
from .loom import (Loom, LoomError, LoomStateError, LoomTimeout, LoomVerifyError,
                   load_image, thread_mask)
from .serial_transport import PicoTransport, TTBoardTransport
from .sim_transport import SimTransport
from .transport import ModelTransport, Transport, TransportError

__all__ = [
    "Loom", "LoomError", "LoomStateError", "LoomTimeout", "LoomVerifyError",
    "ModelTransport", "PicoTransport", "SimTransport", "TTBoardTransport", "Transport",
    "TransportError", "load_image", "protocol", "thread_mask",
]
