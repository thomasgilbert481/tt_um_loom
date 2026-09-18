"""I2C reference models: a 24C02-style EEPROM, a bus monitor and a master.

Both lines are open drain: a model only ever pulls a ``uio`` pad low or lets
go of it. The bench resolves each line as a wired AND with the pull-ups
given to :class:`~tools.protomodels.bench.Bench` (``pullups=``), so SDA is
low whenever the chip or any device pulls it low. The chip drives I2C with
``OD_MASK`` set, so its pin writes are "release" (1) or "pull low" (0).
"""

from __future__ import annotations

from collections import deque
from typing import Iterable, List, Optional, Tuple

from .bench import UIO, Drive, Lines, Model, PinSpec, pad_of


def _od_pad(pin: PinSpec):
    pad = pad_of(pin)
    if pad[0] != UIO:
        raise ValueError("I2C lines must be bidirectional (uio) pads, got %r" % (pin,))
    return pad


class I2cEeprom(Model):
    """A 24C02-like EEPROM: 256 bytes, 8-byte pages, 7-bit address 0x50.

    * byte and page write (the address wraps inside the page; the data is
      committed at STOP, which also starts the optional write cycle during
      which the device NACKs its address, as for ACK polling);
    * current-address read, random read (dummy write, repeated START) and
      sequential read (the address wraps at the end of memory);
    * ``present=False`` never acknowledges; ``nack_data_index=n`` refuses the
      n-th data byte of a write (0 = the first after the word address);
    * ``stretch_clocks`` holds SCL low for that many clocks after the eighth
      bit of every received byte (or only of the byte positions in
      ``stretch_bytes``, 0 = the address byte), so the master must wait for
      SCL to rise before it counts the high time.
    """

    def __init__(self, scl: PinSpec, sda: PinSpec, *, address: int = 0x50,
                 size: int = 256, page: int = 8, memory: bytes = b"",
                 present: bool = True, nack_data_index: Optional[int] = None,
                 stretch_clocks: int = 0, stretch_bytes: Optional[Iterable[int]] = None,
                 write_cycle_clocks: int = 0) -> None:
        self.scl, self.sda = _od_pad(scl), _od_pad(sda)
        self.address = address
        self.memory = bytearray(size)
        self.memory[:len(memory)] = memory
        self.page = page
        self.present = present
        self.nack_data_index = nack_data_index
        self.stretch_clocks = stretch_clocks
        self.stretch_bytes = None if stretch_bytes is None else set(stretch_bytes)
        self.write_cycle_clocks = write_cycle_clocks
        self.prev_scl = self.prev_sda = 1
        self.phase = "idle"             # idle | rx | ack | tx | wait_ack
        self.role = "addr"              # what the byte being received is
        self.bits = self.shift = 0
        self.rw = 0
        self.ptr = 0
        self.tx_byte = self.tx_bits = 0
        self.master_ack = False
        self.sda_low = False
        self.hold_until = -1
        self.busy_until = -1
        self.tbyte = 0                  # byte position inside the transaction
        self.data_index = 0
        self.pending: List[Tuple[int, int]] = []
        self.log: List[tuple] = []
        self.stretches = 0

    def drive(self, drive: Drive, cycle: int) -> None:
        if self.sda_low:
            drive.pull_low(self.sda)
        if cycle < self.hold_until:
            drive.pull_low(self.scl)

    def _present(self, bit_index: int) -> None:
        self.sda_low = not ((self.tx_byte >> (7 - bit_index)) & 1)

    def _load_tx(self) -> None:
        self.tx_byte = self.memory[self.ptr]
        self.ptr = (self.ptr + 1) % len(self.memory)
        self.tx_bits = 0
        self._present(0)

    def observe(self, lines: Lines) -> None:
        scl, sda = lines.get(self.scl), lines.get(self.sda)
        cycle = lines.cycle
        if scl and self.prev_scl and sda != self.prev_sda:
            if not sda:                                   # START / repeated START
                self.log.append(("start", cycle))
                self.pending = []
                self.phase, self.role = "rx", "addr"
                self.bits = self.shift = self.tbyte = 0
                self.sda_low = False
            else:                                         # STOP
                self.log.append(("stop", cycle))
                if self.pending:
                    for addr, value in self.pending:
                        self.memory[addr] = value
                    self.log.append(("write", [a for a, _ in self.pending]))
                    self.pending = []
                    self.busy_until = cycle + self.write_cycle_clocks
                self.phase = "idle"
                self.sda_low = False
        elif scl and not self.prev_scl:                   # rising SCL: sample
            if self.phase == "rx":
                self.shift = ((self.shift << 1) | sda) & 0x1FF
                self.bits += 1
            elif self.phase == "wait_ack":
                self.master_ack = sda == 0
        elif not scl and self.prev_scl:                   # falling SCL: shift
            self._falling(cycle)
        self.prev_scl, self.prev_sda = scl, sda

    def _falling(self, cycle: int) -> None:
        if self.phase == "rx" and self.bits == 8:
            value = self.shift & 0xFF
            ack = False
            if self.role == "addr":
                self.rw = value & 1
                ack = (self.present and (value >> 1) == self.address
                       and cycle >= self.busy_until)
                self.log.append(("addr", value >> 1, self.rw, ack))
            elif self.role == "word":
                self.ptr = value % len(self.memory)
                ack = True
                self.data_index = 0
            else:
                ack = self.nack_data_index != self.data_index
                if ack:
                    self.pending.append((self.ptr, value))
                    base = self.ptr - self.ptr % self.page
                    self.ptr = base + (self.ptr + 1) % self.page
                self.log.append(("data", value, ack))
                self.data_index += 1
            if self.stretch_clocks and (self.stretch_bytes is None
                                        or self.tbyte in self.stretch_bytes):
                self.hold_until = cycle + 1 + self.stretch_clocks
                self.stretches += 1
            self.tbyte += 1
            if ack:
                self.sda_low = True
                self.phase = "ack"
            else:
                self.phase = "idle"
        elif self.phase == "ack":
            self.sda_low = False
            if self.role == "addr" and self.rw:
                self.phase = "tx"
                self._load_tx()
            else:
                self.role = "word" if self.role == "addr" else "data"
                self.phase = "rx"
                self.bits = self.shift = 0
        elif self.phase == "tx":
            self.tx_bits += 1
            if self.tx_bits < 8:
                self._present(self.tx_bits)
            else:
                self.sda_low = False
                self.phase = "wait_ack"
        elif self.phase == "wait_ack":
            if self.master_ack:
                self.phase = "tx"
                self._load_tx()
            else:
                self.phase = "idle"


class I2cMonitor(Model):
    """Decodes bus traffic into ``("start",)``, ``("rstart",)``, ``("stop",)``
    and ``("byte", value, acked)`` events, with the cycle as the last item."""

    def __init__(self, scl: PinSpec, sda: PinSpec) -> None:
        self.scl, self.sda = pad_of(scl), pad_of(sda)
        self.prev_scl = self.prev_sda = 1
        self.in_frame = False
        self.bits: List[int] = []
        self.events: List[tuple] = []
        self.high_times: List[int] = []    # SCL high durations, in clocks
        self._rise = None

    def observe(self, lines: Lines) -> None:
        scl, sda = lines.get(self.scl), lines.get(self.sda)
        cycle = lines.cycle
        if scl and self.prev_scl and sda != self.prev_sda:
            if not sda:
                self.events.append(("rstart" if self.in_frame else "start", cycle))
                self.in_frame = True
            else:
                self.events.append(("stop", cycle))
                self.in_frame = False
            self.bits = []
        elif scl and not self.prev_scl:
            self._rise = cycle
            if self.in_frame:
                self.bits.append(sda)
                if len(self.bits) == 9:
                    value = sum(b << (7 - i) for i, b in enumerate(self.bits[:8]))
                    self.events.append(("byte", value, self.bits[8] == 0, cycle))
                    self.bits = []
        elif not scl and self.prev_scl and self._rise is not None:
            self.high_times.append(cycle - self._rise)
        self.prev_scl, self.prev_sda = scl, sda

    def kinds(self) -> List[str]:
        return [e[0] for e in self.events]


class I2cMaster(Model):
    """An open-drain I2C master that honours clock stretching.

    Queue operations with :meth:`start`, :meth:`write`, :meth:`read` and
    :meth:`stop`; results (``True`` for an ACK after a write, the byte for a
    read) are appended to :attr:`results` in order. A bit takes four
    ``quarter`` periods of SCL; after releasing SCL the master waits until the
    line is actually high before timing the high phase.
    """

    def __init__(self, scl: PinSpec, sda: PinSpec, *, quarter: int = 10) -> None:
        self.scl, self.sda = _od_pad(scl), _od_pad(sda)
        self.q = quarter
        self.scl_rel = self.sda_rel = 1
        self._ops: deque = deque()
        self.results: List[object] = []
        self._gen = self._main()
        self._wait = 1
        self._wait_scl = False
        self._active = False
        self.lines: Optional[Lines] = None

    def start(self) -> None:
        self._ops.append(("start", None))

    def stop(self) -> None:
        self._ops.append(("stop", None))

    def write(self, value: int) -> None:
        self._ops.append(("write", value & 0xFF))

    def read(self, ack: bool = True) -> None:
        self._ops.append(("read", ack))

    @property
    def busy(self) -> bool:
        return bool(self._ops) or self._active

    def drive(self, drive: Drive, cycle: int) -> None:
        if not self.scl_rel:
            drive.pull_low(self.scl)
        if not self.sda_rel:
            drive.pull_low(self.sda)

    def observe(self, lines: Lines) -> None:
        self.lines = lines
        if self._wait_scl:
            if not lines.get(self.scl):
                return
            self._wait_scl = False
        elif self._wait > 1:
            self._wait -= 1
            return
        step = next(self._gen)
        if step == "scl":
            self._wait_scl = True
        else:
            self._wait = step

    def _sda(self) -> int:
        return self.lines.get(self.sda)

    def _main(self):
        q = self.q
        while True:
            if not self._ops:
                self._active = False
                yield 1
                continue
            self._active = True
            op, arg = self._ops.popleft()
            if op == "start":
                if self.scl_rel:                       # bus idle: SDA falls, SCL high
                    self.sda_rel = 1
                    yield q
                else:                                  # repeated START
                    self.sda_rel = 1
                    yield 2 * q
                    self.scl_rel = 1
                    yield "scl"
                    yield q
                self.sda_rel = 0
                yield 2 * q
                self.scl_rel = 0
                yield q
            elif op == "stop":
                self.sda_rel = 0
                yield q
                self.scl_rel = 1
                yield "scl"
                yield q
                self.sda_rel = 1
                yield 2 * q
            elif op == "write":
                for i in range(8):
                    self.sda_rel = (arg >> (7 - i)) & 1
                    yield q
                    self.scl_rel = 1
                    yield "scl"
                    yield 2 * q
                    self.scl_rel = 0
                    yield q
                self.sda_rel = 1
                yield q
                self.scl_rel = 1
                yield "scl"
                yield q
                self.results.append(self._sda() == 0)
                yield q
                self.scl_rel = 0
                yield q
            else:                                      # read
                self.sda_rel = 1
                value = 0
                for _ in range(8):
                    yield q
                    self.scl_rel = 1
                    yield "scl"
                    yield q
                    value = (value << 1) | self._sda()
                    yield q
                    self.scl_rel = 0
                self.results.append(value)
                self.sda_rel = 0 if arg else 1
                yield q
                self.scl_rel = 1
                yield "scl"
                yield 2 * q
                self.scl_rel = 0
                yield q
                self.sda_rel = 1
