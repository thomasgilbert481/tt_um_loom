"""Pin-level protocol reference models that run beside the golden model.

Each model is a small registered circuit clocked with the chip: every cycle
it drives some pads and observes the resolved lines (see :mod:`.bench`).

* :mod:`.uart` - ``UartTx`` (8N1 encoder with a settable bit period and
  deliberate framing errors) and ``UartRx`` (mid-bit sampling decoder with
  framing-error detection and edge timing records);
* :mod:`.spi` - ``SpiMaster`` and ``SpiDevice`` (modes 0-3, MSB or LSB
  first) and ``SpiFlash`` (JEDEC ID 0x9F, READ 0x03);
* :mod:`.i2c` - ``I2cEeprom`` (24C02-style: byte and page write, current
  and random read, clock stretching, forced NACK), ``I2cMaster`` and
  ``I2cMonitor``; the open-drain wired-AND is resolved by the bench.
"""

from .bench import Bench, Drive, Lines, Model, pad_of

__all__ = ["Bench", "Drive", "Lines", "Model", "pad_of"]
