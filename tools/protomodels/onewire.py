"""1-Wire reference models: the bus, DS18B20 devices and a master.

Written from Maxim's application note 126 ("1-Wire Communication Through
Software", table 2, standard speed), application note 187 ("1-Wire Search
Algorithm") and the DS18B20 datasheet (AC electrical characteristics, the
1-Wire signalling and ROM / function command sections), never from the
firmware in ``firmware/onewire_*.loom``.

* :class:`OneWireBus` is the wire: one or more ``uio`` pads joined by the
  board (a 1-Wire bus between two Loom threads uses one pin per thread, as
  two threads never share a pin), with the pull-up of the bench. Every
  driver is open drain, so the line is the wired AND of everything that
  pulls it low: the chip on any of the pads, and the agents attached to the
  bus (:class:`Ds18b20` devices, a :class:`OneWireMaster`). The bus mirrors
  the chip's pull on one pad onto the others one clock later (the board
  wire), and logs every low period of the line with who pulled it.
* :class:`Ds18b20` is a device: its own 64-bit ROM and scratchpad, the
  presence pulse, READ ROM, MATCH ROM, SKIP ROM, SEARCH ROM, READ and WRITE
  SCRATCHPAD. It samples each write slot at both ends of the datasheet's
  window (15 and 60 us after the fall) and records a violation when the two
  differ, and it holds a 0 in a read slot for exactly ``rdv_us`` (15 us, the
  datasheet's guarantee, by default): a master that samples late reads a 1.
* :class:`OneWireMaster` is a master with AN126's delays and AN187's search,
  for testing a device; it checks the device's presence pulse and every bit
  the device drives against the datasheet.
* :func:`master_slots` classifies the low pulses one driver made and
  :func:`check_master_slots` holds them to the datasheet's windows.

The agents are registered circuits like every bench model, but the bus
clocks each one only in the cycles where the line changes and from the
cycle it asks for (:attr:`Agent.wake`), which is all a 1-Wire part needs and
keeps a search of many devices cheap; :meth:`OneWireBus.run_alone` clocks
the models against each other with no chip and no bench.

Times are in clocks of the chip (50 MHz unless ``clock`` says otherwise);
the constants below are microseconds.
"""

from __future__ import annotations

import bisect
from collections import deque
from typing import Deque, Dict, Iterable, List, Optional, Sequence, Tuple

from .bench import UIO, Drive, Lines, Model, PinSpec, pad_of

CLOCK = 50_000_000

#: AN126 table 2, standard speed, microseconds.
AN126 = dict(A=6, B=64, C=60, D=10, E=9, F=55, G=0, H=480, I=70, J=410)

#: DS18B20 datasheet, AC electrical characteristics (microseconds).
DS18B20 = dict(
    t_slot=(60, 120),       # time slot
    t_rec=1,                # recovery time, minimum
    t_low0=(60, 120),       # write 0 low time
    t_low1=(1, 15),         # write 1 low time (15 is the maximum, exclusive)
    t_rdv=15,               # read data valid after the falling edge
    t_rsth=480,             # reset time high, minimum
    t_rstl=480,             # reset time low, minimum
    t_pdhigh=(15, 60),      # presence-detect high
    t_pdlow=(60, 240),      # presence-detect low
    w_sample=(15, 60),      # the device samples a write slot in this window
)

READ_ROM, MATCH_ROM, SKIP_ROM, SEARCH_ROM = 0x33, 0x55, 0xCC, 0xF0
READ_SCRATCHPAD, WRITE_SCRATCHPAD = 0xBE, 0x4E
FAMILY_DS18B20 = 0x28


def crc8(data: Iterable[int]) -> int:
    """Maxim's 1-Wire CRC-8 (x^8 + x^5 + x^4 + 1, AN27): the bits enter LSB
    first, in the order they travel on the wire, into a register cleared to
    0. Running it over data followed by its CRC byte gives 0."""
    crc = 0
    for byte in data:
        for i in range(8):
            mix = (crc ^ (byte >> i)) & 1
            crc >>= 1
            if mix:
                crc ^= 0x8C                 # 0x31 reflected
    return crc


def make_rom(serial: int, family: int = FAMILY_DS18B20, crc: Optional[int] = None) -> bytes:
    """The 8 ROM bytes in wire order: family code, 48-bit serial number (LSB
    first), CRC of the seven before it (or ``crc`` to make a corrupt one)."""
    body = bytes([family & 0xFF]) + (serial & ((1 << 48) - 1)).to_bytes(6, "little")
    return body + bytes([crc8(body) if crc is None else crc & 0xFF])


def rom_int(rom: bytes) -> int:
    """The ROM as a 64-bit integer, bit 0 = the first bit on the wire."""
    return int.from_bytes(bytes(rom), "little")


def rom_words(rom: bytes) -> List[int]:
    """The ROM as four 16-bit words, bits 15:0 first."""
    value = rom_int(rom)
    return [(value >> (16 * i)) & 0xFFFF for i in range(4)]


def search_order(roms: Iterable[bytes]) -> List[bytes]:
    """The order in which AN187's search finds ``roms``: at every
    discrepancy the first pass takes 0 and later passes take the branches
    left in order, so the IDs come out sorted by their bits in wire order,
    first bit most significant."""
    def key(rom: bytes) -> str:
        v = rom_int(rom)
        return "".join(str((v >> i) & 1) for i in range(64))
    return sorted((bytes(r) for r in roms), key=key)


class Agent:
    """Something on the wire besides the chip. Each clock it sees the level
    of the line and says whether it pulls the line low in the next clock: a
    registered open-drain driver, as every bench model is.

    The bus calls :meth:`clock` in every cycle in which the line changes and
    in every cycle from :attr:`wake` on; an agent whose answer cannot change
    before some cycle sets ``wake`` to it, which only saves time: the answer
    it last gave stands in the cycles between."""

    name = "agent"
    wake = 0

    def clock(self, level: int, cycle: int) -> bool:
        return False


class OneWireBus(Model):
    """The 1-Wire line on one or more ``uio`` pads (module doc).

    Build the bench with ``pullups=bus.pullups``: the bench's pull-up is the
    board's (4.7 kOhm typical), and nothing on a 1-Wire bus drives high.
    Attach devices and masters with :meth:`add`.

    Recorded, in clocks: :attr:`edges` (every change of the line, ``(cycle,
    level)``) and :attr:`pulls` (for each driver, the ``[start, end)``
    intervals in which it pulled the line: ``"chip:<pad>"`` for the chip on
    one of the pads, the agent's ``name`` otherwise).
    """

    def __init__(self, pins: Sequence[PinSpec] = ("BIDIR0",), *, clock: int = CLOCK) -> None:
        self.pads = [pad_of(p) for p in pins]
        if not self.pads or any(port != UIO for port, _ in self.pads):
            raise ValueError("a 1-Wire line needs bidirectional (uio) pads: %r" % (pins,))
        self.mask = 0
        for _, bit in self.pads:
            self.mask |= 1 << bit
        self.clock_hz = clock
        self.us = clock / 1_000_000
        self.agents: List[Agent] = []
        self.level = 1
        self.edges: List[Tuple[int, int]] = []
        self.pulls: Dict[str, List[Tuple[int, int]]] = {}
        self._open: Dict[str, int] = {}
        self._agent_pull: Dict[str, bool] = {}
        self._chip_pull = self._chip_seen = 0
        self._dirty = False
        self._alone = 0

    @property
    def pullups(self) -> int:
        """The ``uio`` bits to give the bench as pull-ups."""
        return self.mask

    def add(self, agent: Agent) -> Agent:
        self.agents.append(agent)
        self._agent_pull[agent.name] = False
        return agent

    def drive(self, drive: Drive, cycle: int) -> None:
        agents = any(self._agent_pull.values())
        for pad in self.pads:
            if agents or self._chip_pull & ~(1 << pad[1]):
                drive.pull_low(pad)

    def observe(self, lines: Lines) -> None:
        cycle = lines.cycle
        chip = lines.uio_oe & ~lines.uio_out & self.mask
        level = 1 if (lines.uio & self.mask) == self.mask else 0
        changed = level != self.level
        if changed:
            self.edges.append((cycle, level))
            self.level = level
        if chip != self._chip_seen or self._dirty:
            self._record(cycle, chip)
        pulls = self._agent_pull
        for agent in self.agents:
            if changed or cycle >= agent.wake:
                pull = bool(agent.clock(level, cycle))
                if pull != pulls[agent.name]:
                    pulls[agent.name] = pull
                    self._dirty = True              # effective from the next cycle
        self._chip_pull = chip

    def _record(self, cycle: int, chip: int) -> None:
        """Open and close the pull intervals for the drivers of this cycle."""
        active = {name for name, on in self._agent_pull.items() if on}
        for port, bit in self.pads:
            if chip >> bit & 1:
                active.add("chip:%s%d" % (port, bit))
        for name in active:
            self._open.setdefault(name, cycle)
        for name in [n for n in self._open if n not in active]:
            self.pulls.setdefault(name, []).append((self._open.pop(name), cycle))
        self._chip_seen, self._dirty = chip, False

    def run_alone(self, until, max_cycles: int) -> bool:
        """Clock the line and its agents with no chip and no bench, to check
        the reference models against each other: the line is low exactly
        when an agent pulls it, and the cycles in which nothing can change
        are skipped. Runs until ``until()`` holds or ``max_cycles`` pass;
        returns ``until()``."""
        lines = Lines()
        end = self._alone + max_cycles
        while self._alone < end and not until():
            cycle = self._alone
            lines.cycle = cycle
            lines.uio = 0 if any(self._agent_pull.values()) else self.mask
            self.observe(lines)
            if self._dirty:
                self._alone = cycle + 1
            else:
                self._alone = max(cycle + 1, min((a.wake for a in self.agents), default=end))
        return until()

    def intervals(self, name: str) -> List[Tuple[int, Optional[int]]]:
        """``name``'s pull intervals, and the one still open, if any, with end None."""
        out = list(self.pulls.get(name, []))
        if name in self._open:
            out.append((self._open[name], None))
        return out

    def chip_name(self, pin: PinSpec) -> str:
        port, bit = pad_of(pin)
        return "chip:%s%d" % (port, bit)


#: DS18B20 scratchpad bytes 0..7 at power-up: +85 C, TH 75, TL 70, 12-bit
#: configuration, then the reserved bytes 0xFF, 0x0C, 0x10 (datasheet).
POWER_UP = bytes([0x50, 0x05, 0x4B, 0x46, 0x7F, 0xFF, 0x0C, 0x10])


def _bits(data: Iterable[int], n: int = 8) -> List[int]:
    out: List[int] = []
    for byte in data:
        out += [(byte >> i) & 1 for i in range(n)]
    return out


class Ds18b20(Agent):
    """A DS18B20 on the wire (module doc).

    Args:
        rom: its 8 ROM bytes in wire order (:func:`make_rom`; a corrupt CRC
            is allowed, and the device sends what it is given).
        scratchpad: bytes 0..7 (byte 8, the CRC, is computed unless
            ``scratchpad_crc`` overrides it).
        pd_wait_us, pd_low_us: the presence pulse, inside the datasheet's
            15..60 and 60..240 us.
        rdv_us: how long after the master's fall a 0 is held in a read slot;
            the datasheet guarantees 15 us, which is the default.

    Recorded: :attr:`log` (``(cycle, event, ...)``: reset, command bytes,
    MATCH ROM results, the search's drop-out bit, scratchpad bytes written),
    :attr:`violations` (``(cycle, kind, detail)`` for every master behaviour
    outside the datasheet: a write bit not stable over 15..60 us, a low
    pulse that is neither a slot nor a reset, a slot shorter than 60 us or
    a recovery shorter than 1 us, a slot that starts before the reset's
    480 us high time is over).
    """

    def __init__(self, rom: bytes, scratchpad: Optional[bytes] = None, *,
                 name: Optional[str] = None, clock: int = CLOCK,
                 pd_wait_us: float = 30, pd_low_us: float = 120, rdv_us: float = 15,
                 scratchpad_crc: Optional[int] = None) -> None:
        self.rom = bytes(rom)
        if len(self.rom) != 8:
            raise ValueError("a ROM is 8 bytes")
        self.name = name or "ds18b20:%s" % self.rom[::-1].hex()
        us = clock / 1_000_000
        self.scratchpad = bytearray(POWER_UP if scratchpad is None else scratchpad)
        if len(self.scratchpad) != 8:
            raise ValueError("give scratchpad bytes 0..7")
        self.scratchpad_crc = scratchpad_crc
        self.c_pd_wait, self.c_pd_low = round(pd_wait_us * us), round(pd_low_us * us)
        self.c_rdv = round(rdv_us * us)
        self.c_s15 = round(DS18B20["w_sample"][0] * us)
        self.c_s60 = round(DS18B20["w_sample"][1] * us) - 1
        self.c_rstl = round(DS18B20["t_rstl"] * us)
        self.c_rsth = round(DS18B20["t_rsth"] * us)
        self.c_low_max = round(DS18B20["t_low0"][1] * us)
        self.c_slot = round(DS18B20["t_slot"][0] * us)
        self.c_rec = round(DS18B20["t_rec"] * us)
        self.log: List[tuple] = []
        self.violations: List[tuple] = []
        self.written: List[bytes] = []
        self.prev = 1
        self.pull = (0, 0)                # pulls in cycles [from, until)
        self.low_start: Optional[int] = None
        self.last_fall: Optional[int] = None
        self.last_rise: Optional[int] = None
        self.reset_rise: Optional[int] = None
        self.samples: List[int] = []
        self._early = 1
        self.mode = "idle"                # idle | presence | rx | tx
        self.tx: Deque[int] = deque()
        self.rx: List[int] = []
        self.need = 0
        self.then = None

    # ------------------------------------------------------------ bit level
    def clock(self, level: int, cycle: int) -> bool:
        mine = self.pull[0] <= cycle < self.pull[1]
        if self.mode == "presence":
            if cycle >= self.pull[1] and level:
                self._receive(8, self._rom_command)       # ready for a ROM command
        elif self.prev and not level and not mine:
            self._fall(cycle)
        if not self.prev and level and self.low_start is not None:
            self._rise(cycle)
        if self.samples and self.samples[0] == cycle:
            self.samples.pop(0)
            self._sample(cycle, level)
        self.prev = level
        # Nothing but an edge of the line can change the answer before the
        # next pull boundary, sample or the end of the presence pulse.
        due = [t for t in (self.pull[0] - 1, self.pull[1] - 1) if t > cycle]
        if self.samples:
            due.append(self.samples[0])
        if self.mode == "presence" and cycle < self.pull[1]:
            due.append(self.pull[1])
        self.wake = min(due) if due else 1 << 62
        return self.pull[0] <= cycle + 1 < self.pull[1]

    def _fall(self, cycle: int) -> None:
        if self.reset_rise is not None and cycle - self.reset_rise < self.c_rsth:
            self.violations.append((cycle, "slot_before_trsth", cycle - self.reset_rise))
        if self.last_fall is not None and cycle - self.last_fall < self.c_slot + self.c_rec:
            self.violations.append((cycle, "slot_too_short", cycle - self.last_fall))
        if self.last_rise is not None and cycle - self.last_rise < self.c_rec:
            self.violations.append((cycle, "recovery_too_short", cycle - self.last_rise))
        self.reset_rise = None
        self.last_fall = self.low_start = cycle
        if self.mode == "tx":
            if not self.tx.popleft():
                self.pull = (cycle + 1, cycle + self.c_rdv)
            if not self.tx:
                self._next()
        elif self.mode == "rx":
            self.samples = [cycle + self.c_s15, cycle + self.c_s60]

    def _rise(self, cycle: int) -> None:
        low = cycle - self.low_start
        self.low_start = None
        self.last_rise = cycle
        if low >= self.c_rstl:
            self.log.append((cycle, "reset"))
            self.mode, self.samples, self.tx, self.then = "presence", [], deque(), None
            self.pull = (cycle + self.c_pd_wait, cycle + self.c_pd_wait + self.c_pd_low)
            self.reset_rise = cycle
        elif low > self.c_low_max:
            self.violations.append((cycle, "low_neither_slot_nor_reset", low))

    def _sample(self, cycle: int, level: int) -> None:
        if self.mode != "rx":
            return
        if self.samples:                                  # the 15 us point
            self._early = level
            return
        if level != self._early:                          # the 60 us point
            self.violations.append((cycle, "write_bit_unstable", self._early, level))
        self.rx.append(self._early)
        if len(self.rx) == self.need:
            self._next()

    # ------------------------------------------------------- protocol level
    def _receive(self, n: int, then) -> None:
        self.mode, self.rx, self.need, self.then = "rx", [], n, then

    def _send(self, bits: Sequence[int], then) -> None:
        self.mode, self.tx, self.then = "tx", deque(bits), then

    def _idle(self, *_) -> None:
        self.mode, self.then = "idle", None

    def _next(self) -> None:
        then, received = self.then, self.mode == "rx"
        value = sum(b << i for i, b in enumerate(self.rx))
        self.mode, self.then = "idle", None
        if then is not None:
            if received:
                then(value)
            else:
                then()

    def scratchpad_bytes(self) -> bytes:
        """The nine bytes READ SCRATCHPAD sends."""
        crc = crc8(self.scratchpad) if self.scratchpad_crc is None else self.scratchpad_crc
        return bytes(self.scratchpad) + bytes([crc & 0xFF])

    def _rom_command(self, byte: int) -> None:
        self.log.append((self.last_fall, "rom_command", byte))
        if byte == READ_ROM:
            self._send(_bits(self.rom), self._function)
        elif byte == MATCH_ROM:
            self._receive(64, self._match)
        elif byte == SKIP_ROM:
            self._function()
        elif byte == SEARCH_ROM:
            self._search(0)
        else:
            self._idle()

    def _match(self, value: int) -> None:
        ok = value == rom_int(self.rom)
        self.log.append((self.last_fall, "match", ok))
        if ok:
            self._function()
        else:
            self._idle()

    def _search(self, i: int) -> None:
        if i == 64:
            self._idle()                  # the master goes back to a reset
            return
        bit = (rom_int(self.rom) >> i) & 1

        def chosen(value: int) -> None:
            if value == bit:
                self._search(i + 1)
            else:
                self.log.append((self.last_fall, "search_dropped", i))
                self._idle()
        self._send([bit, 1 - bit], lambda: self._receive(1, chosen))

    def _function(self, *_) -> None:
        self._receive(8, self._function_command)

    def _function_command(self, byte: int) -> None:
        self.log.append((self.last_fall, "function_command", byte))
        if byte == READ_SCRATCHPAD:
            self._send(_bits(self.scratchpad_bytes()), self._idle)
        elif byte == WRITE_SCRATCHPAD:
            self._receive(24, self._write_scratchpad)
        else:
            self._idle()

    def _write_scratchpad(self, value: int) -> None:
        data = value.to_bytes(3, "little")
        self.written.append(data)
        self.log.append((self.last_fall, "scratchpad_written", data))
        self.scratchpad[2], self.scratchpad[3] = data[0], data[1]
        self.scratchpad[4] = (data[2] & 0x60) | 0x1F      # only R1, R0 are writable
        self._idle()

    def events(self, kind: str) -> List[tuple]:
        return [e for e in self.log if e[1] == kind]


class Transaction:
    """One queued master transaction and what came back."""

    def __init__(self, ops: Sequence[tuple]) -> None:
        self.ops = list(ops)
        self.presence: List[bool] = []     # one per reset
        self.data = bytearray()            # the bytes read
        self.roms: List[bytes] = []        # what a search found
        self.search_error: Optional[str] = None
        self.done = False

    def __repr__(self) -> str:
        return ("Transaction(presence=%r, data=%s, roms=%r, done=%r)"
                % (self.presence, bytes(self.data).hex(), [r.hex() for r in self.roms], self.done))


class OneWireMaster(Agent):
    """A standard-speed master: AN126's delays, AN187's search (module doc).

    ``write1_low_us``, ``read_low_us`` and ``read_sample_us`` override the
    low time of a write-1 slot (AN126 A), of a read slot (A) and the read
    sample point after the fall (A + E), to hold a device to the edges of
    the datasheet's windows; every other interval is AN126's. Each read
    slot is watched from the fall to the sample and up to 60 us, each write
    slot after the release, and each presence pulse is measured; anything
    outside the DS18B20 datasheet goes to :attr:`violations` and the
    presence pulses to :attr:`presence_pulses` (``(high, low)`` in us: the
    line's, which with several devices is the union of their pulses).
    """

    name = "master"

    def __init__(self, *, clock: int = CLOCK, write1_low_us: Optional[float] = None,
                 read_low_us: Optional[float] = None,
                 read_sample_us: Optional[float] = None) -> None:
        us = self.us = clock / 1_000_000
        c = {k: round(v * us) for k, v in AN126.items()}
        self.c = c
        self.w1_low = round(write1_low_us * us) if write1_low_us else c["A"]
        self.r_low = round(read_low_us * us) if read_low_us else c["A"]
        self.r_sample = round(read_sample_us * us) if read_sample_us else c["A"] + c["E"]
        if not 0 < self.r_low < self.r_sample < round(60 * us):
            raise ValueError("a read slot needs 0 < low < sample < 60 us")
        self.slot = c["A"] + c["B"]                    # 70 us, every slot
        self.c_rec = round(DS18B20["t_rec"] * us)
        self.c_60 = round(60 * us)
        self.queue: Deque[Transaction] = deque()
        self.history: List[Transaction] = []
        self.violations: List[tuple] = []
        self.presence_pulses: List[Tuple[float, float]] = []
        self.pulling = False
        self.level = 1
        self.cycle = 0
        self.wake = 0                      # the cycle the sequencer runs next
        self.idle_poll = 50                # clocks between looks at an empty queue
        self._seen: List[int] = []         # every change of the line: cycle ...
        self._seen_level: List[int] = []   # ... and the new level
        self._gen = self._main()

    # ------------------------------------------------------------ requests
    def transaction(self, ops: Sequence[tuple]) -> Transaction:
        """Queue ``ops``: ``("reset",)``, ``("write", data)``, ``("read", n)``,
        ``("search",)`` (every ROM ID on the bus, AN187)."""
        tr = Transaction(ops)
        self.queue.append(tr)
        self.history.append(tr)
        return tr

    def read_rom(self) -> Transaction:
        return self.transaction([("reset",), ("write", [READ_ROM]), ("read", 8)])

    def match_rom(self, rom: bytes, then: Sequence[tuple] = ()) -> Transaction:
        return self.transaction([("reset",), ("write", [MATCH_ROM] + list(rom))] + list(then))

    def skip_rom(self, then: Sequence[tuple] = ()) -> Transaction:
        return self.transaction([("reset",), ("write", [SKIP_ROM])] + list(then))

    def search(self) -> Transaction:
        return self.transaction([("search",)])

    @property
    def busy(self) -> bool:
        return any(not tr.done for tr in self.history)

    # --------------------------------------------------------------- clock
    def clock(self, level: int, cycle: int) -> bool:
        if level != self.level:                     # the bus calls on every change
            self._seen.append(cycle)
            self._seen_level.append(level)
        self.level, self.cycle = level, cycle
        if cycle >= self.wake:
            self.wake = cycle + next(self._gen)
        return self.pulling

    def _level_at(self, cycle: int) -> int:
        """The line in ``cycle`` (a past one), from the changes seen."""
        i = bisect.bisect_right(self._seen, cycle)
        return self._seen_level[i - 1] if i else 1

    def _changes(self, start: int, end: int) -> List[Tuple[int, int]]:
        """The changes of the line in cycles ``[start, end)``."""
        i, j = bisect.bisect_left(self._seen, start), bisect.bisect_left(self._seen, end)
        return list(zip(self._seen[i:j], self._seen_level[i:j]))

    # ----------------------------------------------------------- sequencer
    # ``yield n`` resumes n clocks later; a change of ``pulling`` made while
    # the line of cycle c is observed shows on the line from cycle c + 1.
    # The sequencer sleeps through each interval and reads what the line did
    # in it from the changes :meth:`clock` recorded.
    def _main(self):
        while True:
            if not self.queue:
                yield self.idle_poll
                continue
            tr = self.queue.popleft()
            for op in tr.ops:
                if op[0] == "reset":
                    tr.presence.append((yield from self._reset()))
                elif op[0] == "write":
                    for byte in op[1]:
                        for i in range(8):
                            yield from self._write_bit((byte >> i) & 1)
                elif op[0] == "read":
                    for _ in range(op[1]):
                        value = 0
                        for i in range(8):
                            value |= (yield from self._read_bit()) << i
                        tr.data.append(value)
                elif op[0] == "search":
                    yield from self._search(tr)
                else:
                    raise ValueError("unknown 1-Wire operation %r" % (op,))
            tr.done = True

    def _reset(self):
        """Reset (AN126 G, H), presence sample (I), J; returns the presence."""
        c = self.c
        self.pulling = True
        yield c["H"]
        self.pulling = False
        release = self.cycle + 1
        yield c["I"]
        presence = not self.level                   # in cycle release + I - 1
        yield c["J"]
        changes = self._changes(release + 1, self.cycle + 1)
        fall = release if not self._level_at(release) else None
        fall = fall if fall is not None else next((t for t, v in changes if not v), None)
        rise = next((t for t, v in changes if v and fall is not None and t > fall), None)
        if fall is not None:
            if rise is None:
                self.violations.append((self.cycle, "line_low_after_reset", fall))
            else:
                high, low = (fall - release) / self.us, (rise - fall) / self.us
                self.presence_pulses.append((high, low))
                lo, hi = DS18B20["t_pdhigh"]
                if not lo <= high <= hi:
                    self.violations.append((fall, "t_pdhigh", high))
                # Several devices' pulses overlap into one longer low, so
                # only the minimum is a bus-level check; a test with one
                # device holds presence_pulses to the whole 60..240 us.
                if low < DS18B20["t_pdlow"][0]:
                    self.violations.append((fall, "t_pdlow", low))
        return presence

    def _write_bit(self, bit: int):
        low = self.w1_low if bit else self.c["C"]
        self.pulling = True
        yield low
        self.pulling = False
        release = self.cycle + 1
        yield self.slot - low
        if not self._level_at(release) or any(not v for _, v in
                                              self._changes(release, self.cycle + 1)):
            self.violations.append((release, "line_held_in_write_slot", bit))

    def _read_bit(self):
        self.pulling = True
        yield self.r_low
        self.pulling = False
        fall = self.cycle - self.r_low + 1
        yield self.r_sample - self.r_low
        bit = self.level                            # in cycle fall + r_sample - 1
        rose = self._level_at(fall + self.r_low) or any(
            v for _, v in self._changes(fall + self.r_low, self.cycle + 1))
        if not bit and rose:
            self.violations.append((self.cycle, "read_0_not_held_from_the_fall", fall))
        yield self.slot - self.r_sample
        if not self._level_at(fall + self.c_60):
            self.violations.append((fall + self.c_60, "read_slot_not_released_by_60us", fall))
        return bit

    def _search(self, tr: Transaction):
        """AN187: every device on the bus, in the order the search finds them."""
        last_disc, prev = 0, 0
        while True:
            if not (yield from self._reset()):
                tr.search_error = tr.search_error or "no_presence"
                return
            for i in range(8):
                yield from self._write_bit((SEARCH_ROM >> i) & 1)
            rom, last_zero = 0, 0
            for n in range(1, 65):
                id_bit = yield from self._read_bit()
                cmp_bit = yield from self._read_bit()
                if id_bit and cmp_bit:
                    tr.search_error = "no_device"
                    return
                if id_bit != cmp_bit:
                    d = id_bit
                else:
                    d = (prev >> (n - 1)) & 1 if n < last_disc else int(n == last_disc)
                    if not d:
                        last_zero = n
                rom |= d << (n - 1)
                yield from self._write_bit(d)
            found = rom.to_bytes(8, "little")
            tr.roms.append(found)
            if crc8(found):
                tr.search_error = "crc"
                return
            last_disc, prev = last_zero, rom
            if not last_disc:
                return


# ------------------------------------------------------------------ timing
def master_slots(intervals: Sequence[Tuple[int, Optional[int]]],
                 clock: int = CLOCK) -> List[dict]:
    """The low pulses one master made (``OneWireBus.intervals(name)``), each
    as a dict: ``start`` (cycle), ``low`` (us), ``gap`` (us since the end of
    the previous pulse, ``None`` for the first), ``period`` (us since the
    previous start) and ``kind``: ``reset`` (480 us or more), ``write0``
    (60 to 120 us), ``short`` (under 15 us: a write 1 or a read slot) or
    ``other``. An interval still open is left out."""
    us = clock / 1_000_000
    out: List[dict] = []
    prev = None
    for start, end in intervals:
        if end is None:
            continue
        low = (end - start) / us
        if low >= DS18B20["t_rstl"]:
            kind = "reset"
        elif DS18B20["t_low0"][0] <= low < DS18B20["t_low0"][1]:
            kind = "write0"
        elif DS18B20["t_low1"][0] <= low < DS18B20["t_low1"][1]:
            kind = "short"
        else:
            kind = "other"
        out.append(dict(start=start, end=end, low=low, kind=kind,
                        gap=None if prev is None else (start - prev[1]) / us,
                        period=None if prev is None else (start - prev[0]) / us))
        prev = (start, end)
    return out


def check_master_slots(slots: Sequence[dict]) -> List[tuple]:
    """Hold a master's pulses (:func:`master_slots`) to the DS18B20
    datasheet: every pulse a reset, a write 0 or a short slot; a recovery of
    at least 1 us between pulses; a slot (fall to fall) of at least 60 us
    plus that recovery; and at least 480 us high after a reset. Returns
    ``(start, problem, value)`` for each failure."""
    bad: List[tuple] = []
    rec = DS18B20["t_rec"]
    for i, s in enumerate(slots):
        if s["kind"] == "other":
            bad.append((s["start"], "low_time", s["low"]))
        if s["gap"] is None:
            continue
        before = slots[i - 1]
        if before["kind"] == "reset":
            if s["gap"] < DS18B20["t_rsth"]:
                bad.append((s["start"], "t_rsth", s["gap"]))
            continue
        if s["gap"] < rec:
            bad.append((s["start"], "t_rec", s["gap"]))
        if s["period"] < DS18B20["t_slot"][0] + rec:
            bad.append((s["start"], "t_slot", s["period"]))
    return bad
