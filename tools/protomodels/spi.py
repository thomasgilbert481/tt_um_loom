"""SPI reference models: a master, a generic device (slave) and a small flash.

Mode ``m`` has ``CPOL = m >> 1`` (SCK idle level) and ``CPHA = m & 1``. The
leading edge is the one away from the idle level. With ``CPHA = 0`` data is
sampled on the leading edge and shifted on the trailing edge, and the first
bit of a transaction is on the line as soon as CS_n falls; with ``CPHA = 1``
data is shifted on the leading edge and sampled on the trailing edge.
Chip select is active low. Bytes go MSB first unless ``lsb_first``.
"""

from __future__ import annotations

from collections import deque
from typing import Dict, List, Optional, Sequence, Tuple

from .bench import Drive, Lines, Model, PinSpec, pad_of


def _bit(byte: int, index: int, lsb_first: bool) -> int:
    return (byte >> (index if lsb_first else 7 - index)) & 1


class SpiDevice(Model):
    """An SPI slave. Override :meth:`on_select` and :meth:`on_byte`.

    ``on_byte(index, value)`` receives each complete input byte and returns
    the byte to send in the *next* position (``None`` sends 0xFF). The first
    output byte comes from ``on_select()``. MISO is driven only while
    selected. Every transaction is kept in :attr:`transactions` as a list of
    ``(byte_in, byte_out)`` pairs.
    """

    def __init__(self, sck: PinSpec, mosi: PinSpec, miso: PinSpec, cs: PinSpec, *,
                 mode: int = 0, lsb_first: bool = False) -> None:
        self.sck, self.mosi = pad_of(sck), pad_of(mosi)
        self.miso, self.cs = pad_of(miso), pad_of(cs)
        self.cpol, self.cpha = (mode >> 1) & 1, mode & 1
        self.lsb_first = lsb_first
        self.selected = False
        self.prev_sck = self.cpol
        self.level = 1
        self.bits = 0
        self.shift = 0
        self.out = 0xFF
        self.index = 0
        self.current: List[Tuple[int, int]] = []
        self.transactions: List[List[Tuple[int, int]]] = []

    # ---- behaviour hooks
    def on_select(self) -> int:
        return 0xFF

    def on_byte(self, index: int, value: int) -> Optional[int]:
        return None

    def on_deselect(self) -> None:
        pass

    # ---- pins
    def drive(self, drive: Drive, cycle: int) -> None:
        if self.selected:
            drive.set(self.miso, self.level)

    def observe(self, lines: Lines) -> None:
        cs = lines.get(self.cs)
        sck = lines.get(self.sck)
        if self.selected and cs:
            self.selected = False
            self.transactions.append(self.current)
            self.on_deselect()
        elif not self.selected and not cs:
            self.selected = True
            self.bits = self.shift = self.index = 0
            self.current = []
            self.out = self.on_select() & 0xFF
            if self.cpha == 0:
                self.level = _bit(self.out, 0, self.lsb_first)
        if self.selected and sck != self.prev_sck:
            leading = self.prev_sck == self.cpol
            if leading != bool(self.cpha):          # a sample edge
                bit = lines.get(self.mosi)
                if self.lsb_first:
                    self.shift = (self.shift >> 1) | (bit << 7)
                else:
                    self.shift = ((self.shift << 1) | bit) & 0xFF
                self.bits += 1
                if self.bits == 8:
                    self.current.append((self.shift, self.out))
                    nxt = self.on_byte(self.index, self.shift)
                    self.index += 1
                    self.out = 0xFF if nxt is None else nxt & 0xFF
                    self.bits = 0
                    self.shift = 0
            elif self.bits < 8:                     # a shift edge
                self.level = _bit(self.out, self.bits, self.lsb_first)
        self.prev_sck = sck


class SpiFlash(SpiDevice):
    """A tiny SPI NOR flash: ``0x9F`` JEDEC ID and ``0x03`` READ (24-bit address).

    Other commands are recorded and answered with 0xFF.
    """

    def __init__(self, sck: PinSpec, mosi: PinSpec, miso: PinSpec, cs: PinSpec, *,
                 memory: bytes = b"", size: int = 4096,
                 jedec_id: Sequence[int] = (0xEF, 0x40, 0x16), mode: int = 0,
                 lsb_first: bool = False) -> None:
        super().__init__(sck, mosi, miso, cs, mode=mode, lsb_first=lsb_first)
        self.memory = bytearray(max(size, len(memory)))
        self.memory[:len(memory)] = memory
        self.jedec_id = bytes(jedec_id)
        self.commands: List[int] = []
        self.cmd = None
        self.addr = 0

    def on_select(self) -> int:
        self.cmd = None
        self.addr = 0
        return 0xFF

    def on_byte(self, index: int, value: int) -> Optional[int]:
        if index == 0:
            self.cmd = value
            self.commands.append(value)
        if self.cmd == 0x9F:
            return self.jedec_id[index] if index < len(self.jedec_id) else 0xFF
        if self.cmd == 0x03:
            if 1 <= index <= 3:
                self.addr = (self.addr << 8) | value
            if index >= 3:
                byte = self.memory[self.addr % len(self.memory)]
                self.addr += 1
                return byte
        return 0xFF


class SpiMaster(Model):
    """Drives SCK, MOSI and CS_n; samples MISO. Queue with :meth:`transfer`.

    ``half`` is half an SCK period in clocks. Each finished transaction's
    received bytes are appended to :attr:`results`.
    """

    def __init__(self, sck: PinSpec, mosi: PinSpec, miso: PinSpec, cs: PinSpec, *,
                 mode: int = 0, lsb_first: bool = False, half: int = 8,
                 cs_setup: int = 8, cs_hold: int = 8) -> None:
        self.sck, self.mosi = pad_of(sck), pad_of(mosi)
        self.miso, self.cs = pad_of(miso), pad_of(cs)
        self.cpol, self.cpha = (mode >> 1) & 1, mode & 1
        self.lsb_first = lsb_first
        self.half, self.cs_setup, self.cs_hold = half, cs_setup, cs_hold
        self.levels: Dict[str, int] = {"sck": self.cpol, "mosi": 0, "cs": 1}
        self._events: deque = deque()
        self._due: List[Tuple[int, int]] = []
        self._rx: List[int] = []
        self._free_at = 0
        self.now = 0
        self.results: List[bytes] = []

    def transfer(self, data: bytes) -> None:
        t = max(self._free_at, self.now + 1)
        ev = self._events
        first = _bit(data[0], 0, self.lsb_first) if data else 0
        ev.append((t, "cs", 0))
        if self.cpha == 0:
            ev.append((t, "mosi", first))
        t += self.cs_setup
        for i, byte in enumerate(data):
            for b in range(8):
                ev.append((t, "sck", 1 - self.cpol))              # leading edge
                if self.cpha == 0:
                    ev.append((t, "sample", i * 8 + b))
                else:
                    ev.append((t, "mosi", _bit(byte, b, self.lsb_first)))
                t += self.half
                ev.append((t, "sck", self.cpol))                  # trailing edge
                if self.cpha == 1:
                    ev.append((t, "sample", i * 8 + b))
                else:
                    nb = b + 1
                    if nb < 8:
                        ev.append((t, "mosi", _bit(byte, nb, self.lsb_first)))
                    elif i + 1 < len(data):
                        ev.append((t, "mosi", _bit(data[i + 1], 0, self.lsb_first)))
                t += self.half
        t += self.cs_hold
        ev.append((t, "cs", 1))
        ev.append((t, "done", len(data)))
        self._free_at = t + self.cs_hold

    @property
    def busy(self) -> bool:
        return bool(self._events)

    def drive(self, drive: Drive, cycle: int) -> None:
        self.now = cycle
        ev = self._events
        while ev and ev[0][0] <= cycle:
            _, what, value = ev.popleft()
            if what in ("sck", "mosi", "cs"):
                self.levels[what] = value
            else:
                self._due.append((0 if what == "sample" else 1, value))
        drive.set(self.sck, self.levels["sck"])
        drive.set(self.mosi, self.levels["mosi"])
        drive.set(self.cs, self.levels["cs"])

    def observe(self, lines: Lines) -> None:
        for kind, value in self._due:
            if kind == 0:
                self._rx.append(lines.get(self.miso))
            else:
                bits, self._rx = self._rx, []
                out = bytearray()
                for i in range(value):
                    byte = 0
                    for b in range(8):
                        byte |= bits[i * 8 + b] << (b if self.lsb_first else 7 - b)
                    out.append(byte)
                self.results.append(bytes(out))
        self._due = []
