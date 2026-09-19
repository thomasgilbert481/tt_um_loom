"""SimTransport: host transactions clocked out on the chip's SPI pads.

:class:`SimTransport` is the transport for a bench whose chip implements the
host port at the pin level, which today means the RTL:
``test/rtl_bench.py`` (``RtlBench``) clocks ``tb.v`` one edge per bench
cycle. The transport adds one pin model to the bench, :class:`HostPins`, that
drives ``ui_in[4]`` CS_n, ``ui_in[5]`` SCK and ``ui_in[6]`` MOSI and samples
``uo_out[7]`` MISO, and :meth:`SimTransport.transfer` simply steps the bench
for as many cycles as the transaction takes. So the host and every other pin
model share one clock, and the firmware keeps running while the host talks,
exactly as with :class:`~tools.loomhost.transport.ModelTransport` on the
golden model. This module needs no simulator: any object with the bench API
(``add``, ``step``, ``cycle``, ``lines``) will do.

Timing, in core clocks, with the defaults of ``ModelTransport`` (SPI mode 0,
docs/HOST_PROTOCOL.md "Electrical"):

* CS_n falls; ``cs_setup`` (4) clocks later the first bit starts;
* each bit is ``half`` = ``clocks_per_byte / 16`` (4) clocks with SCK low
  and MOSI holding the bit, then ``half`` clocks with SCK high: SCK is
  clk / 8, the protocol's fastest, and a byte is ``clocks_per_byte`` (64);
* MISO is sampled in the last SCK-low cycle of each bit, just before the
  rise. The chip moves MISO on SCK's fall, which it sees through a two-flop
  synchroniser, and the new bit is on the pad in the fourth low cycle (the
  sampling point of ``test/spi_host.py``, used by the cocotb suite since M1);
* after the last bit, SCK low and CS_n still low for ``cs_hold`` (4)
  clocks, then CS_n high for ``cs_gap`` (8) clocks.

A transaction of ``n`` bytes therefore takes ``cs_setup + n *
clocks_per_byte + cs_hold + cs_gap`` clocks, the same as on
``ModelTransport``, and a word's last SCK rise sits where ``ModelTransport``
assumes it: the chip commits the word at edge ``E + 4`` (HOST_PROTOCOL,
"Transaction format"), which is the edge ``ModelTransport`` commits it at.

``irq()`` is the level of ``uo_out[6]`` (HOST_IRQ) in the last bench cycle.
"""

from __future__ import annotations

from typing import List, Optional

from tools.protomodels.bench import UI, UO, Drive, Lines, Model

from .transport import Transport, TransportError

CS_N_PAD = (UI, 4)
SCK_PAD = (UI, 5)
MOSI_PAD = (UI, 6)
IRQ_PAD = (UO, 6)
MISO_PAD = (UO, 7)


class HostPins(Model):
    """The host side of the SPI port as a pin model.

    Idle it drives CS_n high, SCK and MOSI low. :meth:`start` arms one
    transaction beginning at a given bench cycle; the waveform is computed
    from the cycle offset, and the MISO samples are collected in
    :attr:`miso` (one bit per bit clocked).
    """

    def __init__(self, *, half: int = 4, cs_setup: int = 4, cs_hold: int = 4) -> None:
        self.half = half
        self.cs_setup = cs_setup
        self.cs_hold = cs_hold
        self.tx = b""
        self.t0: Optional[int] = None
        self.miso: List[int] = []
        self._bit_clocks = 2 * half
        self._bits_end = cs_setup

    def start(self, tx: bytes, t0: int) -> int:
        """Arm a transaction whose CS_n falls in cycle ``t0``; returns the
        cycle count until CS_n rises again."""
        self.tx = bytes(tx)
        self.t0 = t0
        self.miso = []
        self._bits_end = self.cs_setup + 8 * len(self.tx) * self._bit_clocks
        return self._bits_end + self.cs_hold

    def finish(self) -> None:
        self.t0 = None

    def _levels(self, cycle: int):
        """``(cs_n, sck, mosi, sample)`` for a bench cycle."""
        if self.t0 is None:
            return 1, 0, 0, False
        k = cycle - self.t0
        if k < 0 or k >= self._bits_end + self.cs_hold:
            return 1, 0, 0, False
        if k < self.cs_setup or k >= self._bits_end:
            return 0, 0, 0, False
        j = k - self.cs_setup
        bit_index, within = divmod(j, self._bit_clocks)
        byte, bit = divmod(bit_index, 8)
        mosi = (self.tx[byte] >> (7 - bit)) & 1
        return 0, int(within >= self.half), mosi, within == self.half - 1

    def drive(self, drive: Drive, cycle: int) -> None:
        cs, sck, mosi, _ = self._levels(cycle)
        drive.set(CS_N_PAD, cs)
        drive.set(SCK_PAD, sck)
        drive.set(MOSI_PAD, mosi)

    def observe(self, lines: Lines) -> None:
        if self.t0 is not None and self._levels(lines.cycle)[3]:
            self.miso.append(lines.get(MISO_PAD))


class SimTransport(Transport):
    """Moves host transactions through the SPI pads of a clocked bench.

    ``bench`` must present the bench API of ``tools.protomodels.bench.Bench``
    (``add``, ``step``, ``cycle``, ``lines``) and resolve ``ui_in`` from the
    models' drive, as ``test/rtl_bench.RtlBench`` does. The golden model has
    no pin-level host port, so on it use ``ModelTransport`` instead.
    """

    def __init__(self, bench, *, clocks_per_byte: int = 64, cs_setup: int = 4,
                 cs_hold: int = 4, cs_gap: int = 8, clk_hz: int = 50_000_000) -> None:
        if clocks_per_byte < 64 or clocks_per_byte % 16:
            raise ValueError("clocks_per_byte must be a multiple of 16 and at least 64 "
                             "(SCK <= clk/8, even phases)")
        if cs_setup < 4 or cs_hold < 4:
            raise ValueError("HOST_PROTOCOL needs CS_n low 4 clocks around the SCK edges")
        self.bench = bench
        self.clocks_per_byte = clocks_per_byte
        self.cs_setup = cs_setup
        self.cs_hold = cs_hold
        self.cs_gap = cs_gap
        self.clk_hz = clk_hz
        self.transactions = 0
        self.pins = HostPins(half=clocks_per_byte // 16, cs_setup=cs_setup, cs_hold=cs_hold)
        bench.add(self.pins)

    @property
    def cycle(self) -> int:
        return self.bench.cycle

    def idle(self, cycles: int) -> None:
        """Advance the bench with chip select high."""
        self.bench.step(cycles)

    def delay(self, seconds: float) -> None:
        self.idle(max(0, int(round(seconds * self.clk_hz))))

    def transfer(self, tx: bytes) -> bytes:
        tx = bytes(tx)
        if not tx:
            return b""
        span = self.pins.start(tx, self.bench.cycle)
        try:
            self.bench.step(span)
        finally:
            self.pins.finish()
        bits = self.pins.miso
        if len(bits) != 8 * len(tx):
            raise TransportError("sampled %d MISO bits for %d bytes" % (len(bits), len(tx)))
        rx = bytearray()
        for i in range(len(tx)):
            byte = 0
            for bit in bits[8 * i:8 * i + 8]:
                byte = (byte << 1) | bit
            rx.append(byte)
        self.bench.step(self.cs_gap)
        self.transactions += 1
        return bytes(rx)

    def irq(self) -> bool:
        """HOST_IRQ (``uo_out[6]``) as the pad showed it in the last bench cycle."""
        return bool(self.bench.lines.get(IRQ_PAD))
