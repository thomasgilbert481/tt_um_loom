"""An I2C bus master timed by the I2C specification, for slave firmware.

:class:`~tools.protomodels.i2c.I2cMaster` is a convenience master with one
``quarter`` period; this one is the reference for testing a *slave*
(``firmware/i2c_slave_eeprom.loom``): every interval it produces comes from
UM10204 (I2C-bus specification, rev. 7, table 10) for the mode that the bit
rate selects, and every bit the slave drives is checked against the same
table.

* **Rate.** ``rate`` in bit/s at ``clock`` Hz picks Standard mode (up to
  100 kHz) or Fast mode (up to 400 kHz). The SCL low phase is the mode's
  minimum ``tLOW`` and the high phase the rest of the period, which must
  still meet ``tHIGH``: at 50 MHz, 235 + 265 clocks at 100 kHz and 65 + 60
  at 400 kHz. The shortest legal low phase is the hardest case for a slave,
  which must answer inside it. START hold, repeated-START and STOP set-up
  and the bus-free time are the table's minima as well.
* **Data hold.** The master changes SDA ``hold`` clocks after it pulls SCL
  low (default 0, the specification's minimum ``tHD;DAT``), so a slave must
  not mistake a data change right after the fall for a START or STOP.
* **Clock stretching.** After releasing SCL the master waits until the line
  is high before it times the high phase.
* **Checks on the slave.** For every bit the slave drives (the ACK of a
  byte written, the bits of a byte read) SDA is looked at ``tVD`` after the
  fall (``tVD;ACK`` / ``tVD;DAT``: 3.45 us in Standard mode, 0.9 us in Fast
  mode), at the rise and in the last clock of the high phase; the three must
  agree, or the bit is recorded in :attr:`violations`. For every bit the
  master drives, SDA must read back what it sent while SCL is high (a slave
  pulling SDA low then would corrupt it); a mismatch is recorded too.

Transactions are queued with :meth:`transaction` (or the 24C02 helpers
:meth:`eeprom_write`, :meth:`eeprom_read`, :meth:`eeprom_current_read`) and
executed in order; each returns a :class:`Transaction` whose ``acks`` (one
per byte written, ``True`` = ACK) and ``data`` (bytes read) fill in, and
whose ``done`` is set once its STOP and the bus-free time are over. A NACK
ends a write transaction early with a STOP, as a master does.
"""

from __future__ import annotations

import math
from collections import deque
from typing import Deque, List, Optional, Sequence, Tuple

from .bench import UIO, Drive, Lines, Model, PinSpec, pad_of

#: UM10204 table 10 minima (maxima for tVD), in seconds, per mode.
SPEC = {
    "Sm": dict(rate=100_000, low=4.7e-6, high=4.0e-6, su_sta=4.7e-6, hd_sta=4.0e-6,
               su_sto=4.0e-6, buf=4.7e-6, su_dat=250e-9, vd=3.45e-6),
    "Fm": dict(rate=400_000, low=1.3e-6, high=0.6e-6, su_sta=0.6e-6, hd_sta=0.6e-6,
               su_sto=0.6e-6, buf=1.3e-6, su_dat=100e-9, vd=0.9e-6),
}


class I2cTiming:
    """The master's intervals in clocks for one bit rate (see the module doc)."""

    def __init__(self, rate: int, clock: int = 50_000_000, hold: int = 0) -> None:
        if rate <= SPEC["Sm"]["rate"]:
            self.mode = "Sm"
        elif rate <= SPEC["Fm"]["rate"]:
            self.mode = "Fm"
        else:
            raise ValueError("%d bit/s is above Fast mode (400 kHz)" % rate)
        spec = SPEC[self.mode]

        def clocks(seconds: float) -> int:
            return int(math.ceil(seconds * clock - 1e-9))

        self.rate, self.clock = rate, clock
        self.period = int(round(clock / rate))
        self.low = clocks(spec["low"])
        self.high = self.period - self.low
        if self.high < clocks(spec["high"]):
            raise ValueError("%d bit/s leaves SCL high %d clocks, under tHIGH"
                             % (rate, self.high))
        self.su_sta, self.hd_sta = clocks(spec["su_sta"]), clocks(spec["hd_sta"])
        self.su_sto, self.buf = clocks(spec["su_sto"]), clocks(spec["buf"])
        self.su_dat = clocks(spec["su_dat"])
        self.vd = int(spec["vd"] * clock)            # a maximum: round down
        self.hold = hold
        if not 0 <= hold <= min(self.low - self.su_dat, self.vd):
            raise ValueError("hold %d: must leave tSU;DAT before the rise and "
                             "not exceed tVD" % hold)
        if self.vd >= self.low:
            raise ValueError("tVD is not inside the low phase")

    def __repr__(self) -> str:
        return ("I2cTiming(%s, %d bit/s: low %d, high %d, tVD %d clocks)"
                % (self.mode, self.rate, self.low, self.high, self.vd))


class Transaction:
    """One queued transaction and what came back."""

    def __init__(self, ops: Sequence[Tuple[str, object]]) -> None:
        self.ops = list(ops)
        self.acks: List[bool] = []         # one per byte written
        self.data: List[int] = []          # the bytes read
        self.done = False
        self.aborted = False               # a NACK cut it short

    def __repr__(self) -> str:
        return "Transaction(acks=%r, data=%r, done=%r)" % (self.acks, self.data, self.done)


class I2cBusMaster(Model):
    """An open-drain I2C master with specification timing (module doc)."""

    def __init__(self, scl: PinSpec, sda: PinSpec, *, rate: int = 100_000,
                 clock: int = 50_000_000, hold: int = 0) -> None:
        self.scl, self.sda = pad_of(scl), pad_of(sda)
        if self.scl[0] != UIO or self.sda[0] != UIO:
            raise ValueError("I2C lines must be bidirectional (uio) pads")
        self.t = I2cTiming(rate, clock, hold)
        self.scl_rel = self.sda_rel = 1
        self.queue: Deque[Transaction] = deque()
        self.history: List[Transaction] = []
        self.violations: List[tuple] = []
        self.lines: Optional[Lines] = None
        self._gen = self._main()
        self._wait = 1
        self._wait_scl = False
        self._own_bus = False              # SCL is ours (low) after a START

    # ------------------------------------------------------------ requests
    def transaction(self, ops: Sequence[Tuple[str, object]]) -> Transaction:
        """Queue ``ops``: ``("start", None)``, ``("write", byte)``,
        ``("read", ack)`` and ``("stop", None)``."""
        tr = Transaction(ops)
        self.queue.append(tr)
        self.history.append(tr)
        return tr

    @staticmethod
    def _addr(device: int, read: bool) -> int:
        return ((device & 0x7F) << 1) | int(read)

    def eeprom_write(self, word_addr: int, data: bytes, device: int = 0x50) -> Transaction:
        """Byte write (one byte) or page write: S, address W, word address, data, P."""
        ops = [("start", None), ("write", self._addr(device, False)),
               ("write", word_addr & 0xFF)]
        ops += [("write", b) for b in data] + [("stop", None)]
        return self.transaction(ops)

    def eeprom_read(self, word_addr: int, count: int, device: int = 0x50) -> Transaction:
        """Random read: S, address W, word address, Sr, address R, ``count``
        bytes (ACK all but the last, NACK the last), P."""
        ops = [("start", None), ("write", self._addr(device, False)),
               ("write", word_addr & 0xFF), ("start", None),
               ("write", self._addr(device, True))]
        ops += [("read", i + 1 < count) for i in range(count)] + [("stop", None)]
        return self.transaction(ops)

    def eeprom_current_read(self, count: int, device: int = 0x50) -> Transaction:
        """Current-address read (sequential if ``count`` > 1)."""
        ops = [("start", None), ("write", self._addr(device, True))]
        ops += [("read", i + 1 < count) for i in range(count)] + [("stop", None)]
        return self.transaction(ops)

    def address_only(self, device: int, read: bool = False) -> Transaction:
        """S, one address byte, P (a presence probe)."""
        return self.transaction([("start", None), ("write", self._addr(device, read)),
                                 ("stop", None)])

    @property
    def busy(self) -> bool:
        return any(not tr.done for tr in self.history)

    # --------------------------------------------------------------- clock
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

    def _cycle(self) -> int:
        return self.lines.cycle

    # ----------------------------------------------------------- sequencer
    # ``yield n`` resumes n observes later (n >= 1); ``yield "scl"`` resumes
    # in the first observe that sees SCL high. A pin change made in the
    # observe of cycle c shows on the pad from cycle c + 1.
    def _main(self):
        t = self.t
        while True:
            if not self.queue:
                yield 1
                continue
            tr = self.queue.popleft()
            ops = deque(tr.ops)
            while ops:
                op, arg = ops.popleft()
                if op == "start":
                    yield from self._start()
                elif op == "stop":
                    yield from self._stop()
                elif op == "write":
                    ack = yield from self._write(arg)
                    tr.acks.append(ack)
                    if not ack:                          # give up: STOP now
                        tr.aborted = True
                        yield from self._stop()
                        break
                else:
                    value = yield from self._read(bool(arg))
                    tr.data.append(value)
            tr.done = True

    def _start(self):
        t = self.t
        if self._own_bus:                  # repeated START: SCL is low
            if t.hold:
                yield t.hold
            self.sda_rel = 1
            yield t.low - t.hold
            self.scl_rel = 1
            yield "scl"
            yield t.su_sta
        else:                              # from a free bus
            yield t.buf
        self.sda_rel = 0                   # START: SDA falls while SCL is high
        yield t.hd_sta
        self.scl_rel = 0
        self._own_bus = True

    def _stop(self):
        t = self.t
        if t.hold:
            yield t.hold
        self.sda_rel = 0
        yield t.low - t.hold
        self.scl_rel = 1
        yield "scl"
        yield t.su_sto
        self.sda_rel = 1                   # STOP: SDA rises while SCL is high
        self._own_bus = False
        yield t.buf

    def _master_bit(self, value: int):
        """One bit the master drives; entered and left with SCL just pulled low."""
        t = self.t
        if t.hold:
            yield t.hold
        self.sda_rel = value
        yield t.low - t.hold
        self.scl_rel = 1
        yield "scl"
        seen = self._sda()
        if seen != value:
            self.violations.append(("sda_overridden", self._cycle(), value))
        yield t.high - 1
        if self._sda() != value:
            self.violations.append(("sda_overridden_late", self._cycle(), value))
        self.scl_rel = 0
        return seen

    def _slave_bit(self, what: str):
        """One bit the slave drives (SDA released by the master)."""
        t = self.t
        if t.hold:
            yield t.hold
        self.sda_rel = 1
        yield t.vd + 1 - t.hold
        early = self._sda()                # tVD after the fall: must be valid
        yield t.low - t.vd - 1
        self.scl_rel = 1
        yield "scl"
        sample = self._sda()
        yield t.high - 1
        late = self._sda()
        if not early == sample == late:
            self.violations.append((what, self._cycle(), early, sample, late))
        self.scl_rel = 0
        return sample

    def _write(self, byte: int):
        for i in range(8):
            yield from self._master_bit((byte >> (7 - i)) & 1)
        ack = yield from self._slave_bit("ack_not_valid")
        return ack == 0

    def _read(self, ack: bool):
        value = 0
        for _ in range(8):
            bit = yield from self._slave_bit("data_not_valid")
            value = (value << 1) | bit
        yield from self._master_bit(0 if ack else 1)
        return value
