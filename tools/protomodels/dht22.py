"""AM2302 (DHT22) reference models: a sensor, a host, the frame and a checker.

Written from the Aosong AM2302 datasheet ("Digital-output relative humidity
& temperature sensor/module AM2302", the version distributed by Adafruit),
never from ``firmware/dht22_*.loom``. That version has prose and figures
but no timing table, so each figure is cited by section and page:

* 6(2), p.4 text: the host pulls the line low "at least 1~10ms" (the
  figure: "1ms minimum"), then lets go and "wait[s] 20-40us" for the
  sensor, which pulls low 80 us and then high 80 us;
* 6(2), p.4 figures: every bit is a 50 us low and then a high of "26~28us"
  for a 0 or "70us" for a 1;
* 6(2), p.3 figure: after the last bit "Sensor pulls low", then
  "Transmission ended, RL pulls up". No length is given for that low: 50 us
  here is UNCHECKED (taken equal to a bit's low, as other versions of the
  datasheet are said to give it);
* 6(2), p.3 text: 16 bits of humidity, 16 of temperature, 8 of checksum,
  MSB first; humidity and temperature in tenths (652 = 65.2 %RH, 351 = 35.1
  C); "when highest bit of temperature is 1, it means the temperature is
  below 0" with the example 1000 0000 0110 0101 = -10.1 C, so the
  temperature is sign and magnitude; the checksum is "the last 8 bits" of
  the sum of the four bytes (example 0xEE);
* 6(2), p.3: "the interval of whole process must beyond 2 seconds"; section
  7: collecting period 2 s typical;
* section 5, p.2 figure: a 1 kOhm pull-up from DATA to VDD. On the bench the
  pull-up is ideal (the line is high one clock after the last driver lets
  go); 1 kOhm with a few tens of pF is well under a microsecond.

The datasheet gives no tolerance on 80, 50 or 70 us and only a range for a
0's high. :data:`WINDOWS` therefore holds the checker's windows, each one
marked: ``"datasheet"`` where the datasheet gives the range, ``"unchecked"``
where the window is this model's choice around the datasheet's single value
(+-5 us; +5/-2 for a 1's high; the end low 45..55).

* :class:`Dht22Sensor` answers a start pulse with the response and 40 bits
  on a :class:`tools.protomodels.onewire.OneWireBus` line (the same open-drain
  wire with a pull-up, one pad per Loom thread). Every width can be set, so
  a test can put each one at the edge of its window or outside it, and the
  checksum and the response can be corrupted or left out.
* :class:`Dht22Host` makes the start pulse and reads the reply by the width
  of each high, the threshold at 49 us (between 28 and 70).
* :func:`check_reply` holds the line's edges after a release to the
  windows and decodes them; :func:`check_start` holds a host's start pulse.

Times are in clocks of the chip (50 MHz unless ``clock`` says otherwise);
the constants are microseconds.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Sequence, Tuple

from .onewire import CLOCK, Agent, OneWireBus  # noqa: F401  (the line model)

#: The datasheet's figures, microseconds (module doc).
AM2302 = dict(
    start_min=1000,         # host low, 6(2) p.4: "at least 1~10ms", "1ms minimum"
    go=(20, 40),            # host releases, sensor answers after 20..40
    rel=80,                 # response low
    reh=80,                 # response high
    low=50,                 # every bit's low
    h0=(26, 28),            # a 0's high
    h1=70,                  # a 1's high
    end=50,                 # the end low: UNCHECKED (module doc)
    interval_s=2,           # between reads
)

#: The checker's windows, microseconds, inclusive, with where each comes from.
WINDOWS: Dict[str, Tuple[float, float, str]] = dict(
    go=(20, 40, "datasheet"),
    rel=(75, 85, "unchecked"),
    reh=(75, 85, "unchecked"),
    low=(45, 55, "unchecked"),
    h0=(26, 28, "datasheet"),
    h1=(68, 75, "unchecked"),
    end=(45, 55, "unchecked"),
)

#: The host's decision between a 0 and a 1, microseconds of high.
THRESHOLD_US = 49


def to_wire_temp(tenths: int) -> int:
    """A temperature in tenths of a degree as the sensor sends it: sign and
    magnitude (datasheet p.3: -10.1 C is 0x8065)."""
    if not -0x7FFF <= tenths <= 0x7FFF:
        raise ValueError("no 15-bit magnitude for %d" % tenths)
    return (0x8000 | -tenths) if tenths < 0 else tenths


def from_wire_temp(word: int) -> int:
    """The inverse of :func:`to_wire_temp` (0x8000, minus zero, is 0)."""
    magnitude = word & 0x7FFF
    return -magnitude if word & 0x8000 else magnitude


def checksum(data: Sequence[int]) -> int:
    """The low byte of the sum of the bytes (datasheet p.3)."""
    return sum(data) & 0xFF


def frame(humidity: int, temp_wire: int, csum: Optional[int] = None) -> bytes:
    """The five bytes on the wire: humidity, temperature (as sent), checksum
    (or ``csum`` to make a corrupt one)."""
    body = bytes([humidity >> 8 & 0xFF, humidity & 0xFF,
                  temp_wire >> 8 & 0xFF, temp_wire & 0xFF])
    return body + bytes([checksum(body) if csum is None else csum & 0xFF])


def frame_bits(data: bytes) -> List[int]:
    """The 40 bits in wire order, MSB first."""
    return [(byte >> (7 - i)) & 1 for byte in data for i in range(8)]


def decode(data: bytes) -> Tuple[int, int, bool]:
    """``(humidity, temperature in tenths, checksum good)`` of five bytes."""
    data = bytes(data)
    humidity = data[0] << 8 | data[1]
    temp = from_wire_temp(data[2] << 8 | data[3])
    return humidity, temp, checksum(data[:4]) == data[4]


def bits_to_bytes(bits: Sequence[int]) -> bytes:
    out = bytearray()
    for i in range(0, len(bits) - len(bits) % 8, 8):
        value = 0
        for b in bits[i:i + 8]:
            value = value << 1 | b
        out.append(value)
    return bytes(out)


def _us(value_us: float, clock: int) -> int:
    return int(round(value_us * clock / 1_000_000))


def check_reply(edges: Sequence[Tuple[int, int]], release: int, *,
                clock: int = CLOCK, nbits: int = 40) -> dict:
    """Hold the sensor's side of one read to :data:`WINDOWS` and decode it.

    ``edges`` are the line's changes ``(cycle, level)`` from the host's
    release (the rise at ``release``) on. Returns ``bits`` (decoded by the
    high widths at :data:`THRESHOLD_US`), ``data`` (the whole bytes),
    ``widths`` (``(name, index, us)`` in wire order) and ``violations``
    (``(name, index, us, window)`` for each width outside its window, and
    ``("missing", n, None, None)`` if the line stopped after ``n`` edges)."""
    us = clock / 1_000_000
    want = [(0, "go", 0)]                       # (level after the edge, name, index)
    want += [(1, "rel", 0), (0, "reh", 0)]
    for i in range(nbits):
        want += [(1, "low", i), (0, "high", i)]
    want += [(1, "end", 0)]
    widths, violations, bits = [], [], []
    last = release
    for k, (level, name, index) in enumerate(want):
        if k >= len(edges) or edges[k][1] != level:
            violations.append(("missing", k, None, None))
            break
        width = (edges[k][0] - last) / us
        last = edges[k][0]
        widths.append((name, index, width))
        if name == "high":
            bit = int(width > THRESHOLD_US)
            bits.append(bit)
            name = "h1" if bit else "h0"
        lo, hi, _ = WINDOWS[name]
        if not lo <= width <= hi:
            violations.append((name, index, width, (lo, hi)))
    return dict(bits=bits, data=bits_to_bytes(bits), widths=widths,
                violations=violations)


def check_start(low_us: float) -> List[tuple]:
    """A host's start low against the datasheet's 1 ms minimum."""
    if low_us < AM2302["start_min"]:
        return [("start", 0, low_us, (AM2302["start_min"], None))]
    return []


class Dht22Sensor(Agent):
    """An AM2302 on the line (module doc). It answers every low of at least
    ``detect_us`` (default 500: the datasheet does not say what the sensor
    needs, only that the host gives at least 1 ms) when the host lets go,
    and records a violation for a start low under 1 ms.

    Error injection, all keyword arguments: ``csum`` a checksum byte to send
    instead of the right one; ``respond=False`` stays silent; ``stop_after``
    lets go of the line for good after that many bits (the host times out);
    every width in microseconds: ``go_us``, ``rel_us``, ``reh_us``,
    ``low_us``, ``h0_us``, ``h1_us``, ``end_us``; ``high_us`` a dict of
    ``{bit index: width}`` for single bits. The values (``humidity``, ``temp``
    in tenths) may be changed between reads."""

    name = "sensor"

    def __init__(self, humidity: int = 652, temp: int = 351, *, clock: int = CLOCK,
                 detect_us: float = 500, go_us: float = 30, rel_us: float = 80,
                 reh_us: float = 80, low_us: float = 50, h0_us: float = 27,
                 h1_us: float = 70, end_us: float = 50, csum: Optional[int] = None,
                 respond: bool = True, stop_after: Optional[int] = None,
                 high_us: Optional[Dict[int, float]] = None) -> None:
        self.humidity, self.temp = humidity, temp
        self.clock_hz = clock
        self.detect_us = detect_us
        self.go_us, self.rel_us, self.reh_us = go_us, rel_us, reh_us
        self.low_us, self.h0_us, self.h1_us, self.end_us = low_us, h0_us, h1_us, end_us
        self.csum, self.respond, self.stop_after = csum, respond, stop_after
        self.high_us = dict(high_us or {})
        self.starts: List[Tuple[int, float]] = []      # (release cycle, low us)
        self.sent: List[bytes] = []
        self.violations: List[tuple] = []
        self.wake = 1 << 62
        self._fall: Optional[int] = None
        self._plan: List[Tuple[int, bool]] = []
        self._pull = False

    def frame(self) -> bytes:
        return frame(self.humidity, to_wire_temp(self.temp), self.csum)

    def _schedule(self, release: int) -> None:
        data = self.frame()
        bits = frame_bits(data)
        if self.stop_after is not None:
            bits = bits[:self.stop_after]
        t = self.go_us                                   # us after the release
        plan = [(t, True), (t + self.rel_us, False), (t + self.rel_us + self.reh_us, True)]
        t += self.rel_us + self.reh_us
        for i, bit in enumerate(bits):
            t += self.low_us
            plan.append((t, False))
            if self.stop_after is not None and i == len(bits) - 1:
                break
            t += self.high_us.get(i, self.h1_us if bit else self.h0_us)
            plan.append((t, True))
        if self.stop_after is None:
            plan.append((t + self.end_us, False))
        elif self.stop_after == 0:
            plan.append((t + self.low_us, False))
        self._plan = [(release + _us(t, self.clock_hz), p) for t, p in plan]
        self.sent.append(data)

    def clock(self, level: int, cycle: int) -> bool:
        if self._plan:
            while self._plan and self._plan[0][0] <= cycle + 1:
                self._pull = self._plan.pop(0)[1]
            self.wake = self._plan[0][0] - 1 if self._plan else 1 << 62
            return self._pull
        if level == 0 and self._fall is None:
            self._fall = cycle
        elif level == 1 and self._fall is not None:
            low_us = (cycle - self._fall) * 1_000_000 / self.clock_hz
            self._fall = None
            if low_us >= self.detect_us:
                self.starts.append((cycle, low_us))
                self.violations += check_start(low_us)
                if self.respond:
                    self._schedule(cycle)
                    self.wake = self._plan[0][0] - 1
        return self._pull


class Read:
    """One read by :class:`Dht22Host`: filled in when ``done``."""

    def __init__(self, start_us: float) -> None:
        self.start_us = start_us
        self.done = False
        self.release: Optional[int] = None
        self.edges: List[Tuple[int, int]] = []
        self.result: Optional[dict] = None

    @property
    def data(self) -> bytes:
        return self.result["data"] if self.result else b""

    @property
    def violations(self) -> List[tuple]:
        return self.result["violations"] if self.result else []

    def decoded(self) -> Tuple[int, int, bool]:
        return decode(self.data)

    def __repr__(self) -> str:
        return "Read(start %s us, done %s, %d edges)" % (self.start_us, self.done,
                                                          len(self.edges))


class Dht22Host(Agent):
    """A host for testing a sensor: the start low of ``start_us``, then the
    line is left to the sensor and every change recorded until it has been
    high for ``quiet_us`` (200 us, longer than any pulse of a read); the
    read is then checked and decoded by :func:`check_reply`."""

    name = "host"

    def __init__(self, *, clock: int = CLOCK, quiet_us: float = 200) -> None:
        self.clock_hz = clock
        self.quiet = _us(quiet_us, clock)
        self.reads: List[Read] = []
        self.wake = 1 << 62
        self._read: Optional[Read] = None
        self._pull = False
        self._release_at: Optional[int] = None
        self._level = 1

    def read(self, start_us: float = 1000) -> Read:
        """Start a read at the next clock the bus gives this host."""
        rd = Read(start_us)
        self.reads.append(rd)
        self._read = rd
        self.wake = 0
        return rd

    def clock(self, level: int, cycle: int) -> bool:
        rd = self._read
        if rd is None:
            self.wake = 1 << 62
            return False
        if not self._pull and rd.release is None and self._release_at is None:
            self._pull = True                           # the fall, from the next clock
            self._release_at = cycle + 1 + _us(rd.start_us, self.clock_hz)
            self.wake = self._release_at - 1
            return True
        if self._pull:
            if cycle + 1 >= self._release_at:
                self._pull = False
                self.wake = 1 << 62
            else:
                self.wake = self._release_at - 1
            return self._pull
        if rd.release is None:
            if level == 1:
                rd.release = cycle
                self._level = 1
            self.wake = cycle + 1
            return False
        if level != self._level:
            rd.edges.append((cycle, level))
            self._level = level
        last = rd.edges[-1][0] if rd.edges else rd.release
        if level == 1 and cycle - last >= self.quiet:
            rd.result = check_reply(rd.edges, rd.release, clock=self.clock_hz)
            rd.done = True
            self._read, self._release_at = None, None
            self.wake = 1 << 62
        else:
            self.wake = last + self.quiet if level == 1 else 1 << 62
        return False
