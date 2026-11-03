"""SDI-12 reference models: the line, a data recorder, a sensor, a checker.

Written from "SDI-12, A Serial-Digital Interface Standard for
Microprocessor-Based Sensors", version 1.4, 2021-01-30 (sdi-12.org), not
from the firmware. Every figure below names its section, table or figure;
all were checked against the document's text.

* the line (3.1, Table 1) is one wire in negative logic: marking, logical
  1 and the idle state, is -0.5 to 1.0 V; spacing, logical 0, is 3.5 to
  5.5 V. Here the line's level is the wire's: 1 = spacing (high voltage),
  0 = marking. A released line is marking (it is pulled low);
* a character (4.1, Table 2) is 1200 baud: a start bit (spacing), seven
  data bits LSB first, an even parity bit, a stop bit (marking): 7E1, ten
  bits, 8.33 ms. Characters are printable ASCII (4.2, Table 3) except
  <CR><LF>; a command ends with '!', a response with <CR><LF> (4.4);
* timing (7.0 and Figure 3): every figure is +-0.40 ms except the gap
  between a stop bit and the next start bit, at most 1.66 ms with no
  tolerance. A break is spacing for at least 12 ms; a sensor does not
  take spacing shorter than 6.5 ms for one and always takes spacing
  longer than 12 ms for one, and then needs 8.33 ms of marking before it
  looks for an address; the recorder releases the line within 7.5 ms of
  the end of its command's last stop bit (+0.40); the sensor marks the line
  for 8.33 ms (-0.40) and its first start bit begins within 15 ms of that
  stop bit (+0.40); it releases the line within 7.5 ms of its last
  character (+0.40); a sensor wakes within 100 ms of a break and sleeps
  after 100 ms of marking (+0.40) or a wrong address; a break must precede
  a command to another sensor or after 87 ms of marking;
* retries (7.2): with no response, the recorder waits at least 16.67 ms
  after the command's last stop bit and at most 87 ms, then retries
  without a break; after at least two more tries, one of them more than
  100 ms after the break's fall, it repeats the whole sequence, break
  included, at least two more times. A retry is due for no response, 8.33
  ms of marking after a response's start bit, or an invalid response.

:class:`Sdi12Line` is the bench model (``tools.protomodels.bench.Model``):
the chip's ports, each a TX pin in wire polarity and a DIR pin (1 = its
driver is on), and the model devices drive the line; the line is what the
chip's RX pins see, one clock after it was driven. It records every edge
and every driver's on and off times, and decodes every character with
:class:`LineDecoder`. :class:`Sdi12Sensor` and :class:`Sdi12Recorder` are
devices with error injection (a short break, a parity error, a late or
missing response). :func:`check_transaction` measures one command and
its response against section 7.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, Iterable, List, Optional, Sequence, Tuple

from tools.protomodels.bench import Drive, Lines, Model, PinSpec, pad_of

BAUD = 1200                         # 4.1
MS = 1e-3
TOL = 0.40 * MS                     # 7.0: every figure but the gap
BREAK_MIN = 12.0 * MS               # 7.0: the recorder's break
BREAK_NEVER = 6.5 * MS              # 7.0: shorter is never a break
BREAK_ALWAYS = 12.0 * MS            # 7.0: longer is always a break
MARK_AFTER_BREAK = 8.33 * MS        # 7.0: before the sensor looks for an address
RELEASE_MAX = 7.5 * MS              # 7.0: recorder and sensor, +TOL
RESPONSE_MARK = 8.33 * MS           # 7.0: the sensor's marking, -TOL
RESPONSE_START_MAX = 15.0 * MS      # 7.0: first start bit after the command, +TOL
GAP_MAX = 1.66 * MS                 # 7.0: stop bit to start bit, no tolerance
WAKE_MAX = 100.0 * MS               # 7.0: a sensor is awake by then after a break
SLEEP_MARKING = 100.0 * MS          # 7.0: marking after which a sensor sleeps
BREAK_DUE = 87.0 * MS               # 7.0, 7.1: marking after which a break is due
RETRY_MIN = 16.67 * MS              # 7.2
RETRY_MAX = 87.0 * MS               # 7.2
RETRY_MARKING = 8.33 * MS           # 7.2 case 2


def even_parity(value: int) -> int:
    """The parity bit that makes the seven data bits and it even (4.1)."""
    return bin(value & 0x7F).count("1") & 1


def frame_bits(char: int, parity_error: bool = False) -> List[int]:
    """The ten logical bits of ``char`` (4.1, Table 2): start 0, seven data
    bits LSB first, even parity (inverted on request), stop 1."""
    p = even_parity(char) ^ (1 if parity_error else 0)
    return [0] + [(char >> i) & 1 for i in range(7)] + [p, 1]


def wire(bit: int) -> int:
    """Negative logic (3.1, Table 1): logical 1 (marking) is the low
    voltage, 0 the high; the line's level is 1 for the high voltage."""
    return 0 if bit else 1


def text(chars: Iterable[int]) -> str:
    return "".join(chr(c) for c in chars)


# -------------------------------------------------------------- the line
@dataclass
class Char:
    """A character on the line: the cycle its start bit began, its value,
    and who was driving the line then."""

    start: int
    value: int = 0
    parity_ok: bool = True
    framing_ok: bool = True
    drivers: frozenset = frozenset()
    end: float = 0.0                # the end of the stop bit, nominal


@dataclass
class Spacing:
    """A spacing run, from its rising edge to its falling edge."""

    start: int
    end: int

    @property
    def clocks(self) -> int:
        return self.end - self.start


class LineDecoder:
    """Decodes the line at ``T`` clocks a bit: every character from its
    start bit's edge, each bit sampled in its middle (4.1), and every
    spacing run, of which those longer than ``break_bits`` bit times
    (longer than any character can make: a start bit, seven 0s and a 0
    parity bit are 9 bits) are breaks, not characters."""

    def __init__(self, T: float, break_bits: float = 9.5) -> None:
        self.T = float(T)
        self.break_clocks = break_bits * self.T
        self.chars: List[Char] = []
        self.runs: List[Spacing] = []
        self.breaks: List[Spacing] = []
        self.level = 0
        self._since = 0
        self._char: Optional[Char] = None
        self._bits: List[int] = []
        self._next = 0.0

    def feed(self, level: int, cycle: int, drivers: frozenset = frozenset()) -> None:
        """The line's level (1 = spacing) during ``cycle``."""
        if level != self.level:
            if level == 0:
                run = Spacing(self._since, cycle)
                self.runs.append(run)
                if run.clocks >= self.break_clocks:
                    self.breaks.append(run)
                    self._char = None           # it was no character
                    if self.chars and self.chars[-1].start == run.start:
                        self.chars.pop()        # (decoded from its first bits)
            elif self._char is None:
                self._char = Char(cycle, drivers=drivers)
                self._bits = []
                self._next = cycle + 0.5 * self.T
            self._since = cycle
            self.level = level
        if self._char is not None and cycle >= self._next:
            self._sample(1 - level)

    def _sample(self, bit: int) -> None:
        ch = self._char
        self._bits.append(bit)
        k = len(self._bits)
        if k == 1 and bit:
            self._char = None                   # a glitch, not a start bit
            return
        if k < 10:
            self._next = ch.start + (k + 0.5) * self.T
            return
        bits = self._bits
        ch.value = sum(b << i for i, b in enumerate(bits[1:8]))
        ch.parity_ok = bits[8] == even_parity(ch.value)
        ch.framing_ok = bits[9] == 1
        ch.end = ch.start + 10 * self.T
        self.chars.append(ch)
        self._char = None


class Sdi12Line(Model):
    """The SDI-12 data line on the bench.

    ``ports`` are the chip's drivers, each ``(TX, DIR)``: while DIR is 1
    the line is driven to TX's level (wire polarity, 1 = spacing), as the
    three-state buffer of the programs' headers does. ``devices`` are
    model recorders and sensors. The level of cycle ``c`` comes from the
    chip's pins as they were in cycle ``c - 1`` and from every device's
    drive; with nobody driving the line is marking (0). Every RX pin sees
    it. Two drivers at different levels are a contention, recorded, and
    the line is then spacing.
    """

    def __init__(self, devices: Iterable = (), *, clock: float,
                 ports: Sequence[Tuple[PinSpec, PinSpec]] = (("OUT0", "OUT1"),),
                 rx: Sequence[PinSpec] = ("IN0",)) -> None:
        self.devices = list(devices)
        self.ports = [(pad_of(tx), pad_of(d)) for tx, d in ports]
        self.port_names = ["chip%d" % i for i in range(len(self.ports))]
        self.rx_pads = [pad_of(p) for p in rx]
        self.clock = float(clock)
        self.T = self.clock / BAUD
        self.decoder = LineDecoder(self.T)
        self.level = 0
        self.edges: List[Tuple[int, int]] = []
        self.on: Dict[str, List[List[Optional[int]]]] = {}   # name: [[on, off]]
        self.contentions: List[int] = []
        self._chip: List[Optional[int]] = [None] * len(self.ports)
        self._drivers: frozenset = frozenset()

    def add(self, device) -> None:
        self.devices.append(device)

    def ms(self, clocks: float) -> float:
        return clocks / self.clock / MS

    def drive(self, drive: Drive, cycle: int) -> None:
        levels = {}
        for name, lvl in zip(self.port_names, self._chip):
            if lvl is not None:
                levels[name] = lvl
        for dev in self.devices:
            if dev.level is not None:
                levels[dev.name] = dev.level
        names = frozenset(levels)
        for name in names - self._drivers:
            self.on.setdefault(name, []).append([cycle, None])
        for name in self._drivers - names:
            self.on[name][-1][1] = cycle
        self._drivers = names
        vals = set(levels.values())
        if len(vals) > 1:
            self.contentions.append(cycle)
        level = 1 if 1 in vals else 0
        if level != self.level:
            self.edges.append((cycle, level))
        self.level = level
        for pad in self.rx_pads:
            drive.set(pad, level)

    def observe(self, lines: Lines) -> None:
        for i, (tx, d) in enumerate(self.ports):
            self._chip[i] = lines.get(tx) if lines.get(d) else None
        for dev in self.devices:
            dev.clock(self.level, lines.cycle)
        self.decoder.feed(self.level, lines.cycle, self._drivers)

    def driven_at(self, name: str, cycle: float) -> bool:
        return any(a <= cycle and (b is None or cycle < b)
                   for a, b in self.on.get(name, []))

    def chars_by(self, name: str) -> List[Char]:
        return [c for c in self.decoder.chars if name in c.drivers]


# ------------------------------------------------------------- devices
class _Device:
    """A device's driver: a queue of ``(cycle, level)`` changes, a level
    of ``None`` releasing the line, and its own decoder of the line at its
    own bit time (``clock_error`` makes it fast or slow)."""

    def __init__(self, name: str, clock: float, clock_error: float = 0.0) -> None:
        self.name = name
        self.clock_hz = float(clock)
        self.T = clock / (BAUD * (1 + clock_error))
        self.level: Optional[int] = None
        self.now = 0
        self.decoder = LineDecoder(self.T)
        self._changes: Deque[Tuple[int, Optional[int]]] = deque()

    def ms(self, ms: float) -> float:
        """Milliseconds in clocks of the bench."""
        return ms * MS * self.clock_hz

    def _at(self, t: float, level: Optional[int]) -> None:
        self._changes.append((int(round(t)), level))

    def _send(self, t0: float, chars: Sequence[int], parity_error_at: Optional[int] = None,
              gap_bits: float = 0.0) -> float:
        """Drive ``chars`` from ``t0``, back to back or ``gap_bits`` apart
        (marking between them); returns the end of the last stop bit."""
        t = t0
        for i, c in enumerate(chars):
            for k, bit in enumerate(frame_bits(c, parity_error_at == i)):
                self._at(t + k * self.T, wire(bit))
            t += (10 + gap_bits) * self.T
        return t - gap_bits * self.T

    @property
    def busy(self) -> bool:
        return bool(self._changes)

    def _step(self, level: int, cycle: int) -> None:
        self.now = cycle
        self.decoder.feed(level, cycle)
        nxt = cycle + 1
        while self._changes and self._changes[0][0] <= nxt:
            self.level = self._changes.popleft()[1]


@dataclass
class Command:
    """A command a model device saw or sent, and what became of it."""

    text: str
    end: float                          # the end of its last stop bit
    start: int = 0                      # its first start bit (the break's fall)
    response: List[int] = field(default_factory=list)
    answered: bool = False


class Sdi12Sensor(_Device):
    """A sensor at ``address``: ``responses`` maps a command without its
    address ("!", "I!", "M!", "D0!") to what follows the address in the
    response. It needs a break of ``break_ms`` (default 12, the length
    every sensor must take as one) and 8.33 ms of marking after it before
    it looks for an address, answers ``?!`` like ``a!``, and sleeps after
    100 ms of marking or a wrong address (7.0).

    Error injection: ``start_ms`` (the response's first start bit after the
    command's stop bit, default 10), ``mark_ms`` (how long it marks the
    line before that, default 8.33), ``gap_bits`` between characters,
    ``silent`` (the number of commands to leave unanswered), ``wake_ms``
    (it answers nothing that ends sooner after the break's fall) and
    ``parity_error_at`` (the index of a response character sent with bad
    parity, in the next response only) and ``answer_as`` (an address it
    puts at the head of its responses instead of its own: a bad format).
    """

    def __init__(self, address: str, responses: Dict[str, str], *, clock: float,
                 name: str = "sensor", clock_error: float = 0.0,
                 break_ms: float = 12.0, start_ms: float = 10.0, mark_ms: float = 8.33,
                 gap_bits: float = 0.0, silent: int = 0, wake_ms: float = 0.0,
                 parity_error_at: Optional[int] = None,
                 answer_as: Optional[str] = None) -> None:
        super().__init__(name, clock, clock_error)
        self.address = ord(address)
        self.answer_as = ord(answer_as) if answer_as else self.address
        self.responses = dict(responses)
        self.break_clocks = self.ms(break_ms)
        self.start_ms, self.mark_ms, self.gap_bits = start_ms, mark_ms, gap_bits
        self.silent, self.wake_ms = silent, wake_ms
        self.parity_error_at = parity_error_at
        self.state = "asleep"
        self.commands: List[Command] = []
        self.wakes: List[Spacing] = []      # the breaks it took
        self._runs = self._chars = 0
        self._cmd: List[int] = []
        self._awake_at = 0.0
        self._brk = 0
        self._last = 0.0
        self._bad = False

    def clock(self, level: int, cycle: int) -> None:
        """Take the line's level of ``cycle``; set the drive for ``cycle + 1``."""
        self._step(level, cycle)
        dec = self.decoder
        while self._runs < len(dec.runs):
            run = dec.runs[self._runs]
            self._runs += 1
            if run.clocks >= self.break_clocks:
                self.state, self._brk = "marking", run.start
                self._awake_at = run.end + self.ms(MARK_AFTER_BREAK / MS)
                self._last, self._cmd, self._bad = run.end, [], False
                self.wakes.append(run)
                self._chars = len(dec.chars)
        while self._chars < len(dec.chars):
            ch = dec.chars[self._chars]
            self._chars += 1
            self._char(ch)
        if self.state != "asleep" and not self.busy and self.level is None \
                and cycle - self._last > self.ms(SLEEP_MARKING / MS):
            self.state = "asleep"

    def _char(self, ch: Char) -> None:
        if self.state == "asleep" or self.level is not None or self.busy:
            return                              # asleep, or its own
        if self.state == "marking":
            if ch.start < self._awake_at:
                self.state = "asleep"           # too little marking
                return
            self.state = "awake"
        self._last = ch.end
        if not self._cmd and ch.value not in (self.address, ord("?")):
            self.state = "asleep"               # another sensor's address
            return
        self._cmd.append(ch.value)
        self._bad |= not (ch.parity_ok and ch.framing_ok)
        if ch.value != ord("!"):
            return
        cmd, bad, self._cmd, self._bad = text(self._cmd), self._bad, [], False
        rec = Command(cmd, ch.end, self._brk)
        self.commands.append(rec)
        body = self.responses.get(cmd[1:])
        if cmd[0] == "?" and cmd != "?!":
            body = None                         # 4.4.3: '?' with a! only
        if bad or body is None or ch.end - self._brk < self.ms(self.wake_ms):
            return
        if self.silent:
            self.silent -= 1
            return
        chars = [self.answer_as] + [ord(c) for c in body] + [13, 10]
        start = ch.end + self.ms(self.start_ms)
        self._at(start - self.ms(self.mark_ms), 0)          # marking
        end = self._send(start, chars, self.parity_error_at, self.gap_bits)
        self.parity_error_at = None
        self._at(end, None)                                 # released
        rec.response, rec.answered = chars, True
        self._last = end


class Sdi12Recorder(_Device):
    """A data recorder that sends what it is told and records what comes
    back; the tests drive the timing (it does not retry by itself).

    :meth:`command` sends a break of ``break_ms`` (``None``: no break) and
    ``mark_ms`` of marking, then the command's characters ``gap_bits``
    apart, the one at ``parity_error_at`` with bad parity, and releases the
    line ``release_ms`` after the last stop bit. Every character on the
    line that it did not send after that goes into the command's
    ``response`` up to a <LF>.
    """

    def __init__(self, *, clock: float, name: str = "recorder",
                 clock_error: float = 0.0) -> None:
        super().__init__(name, clock, clock_error)
        self.sent: List[Command] = []
        self._chars = 0

    def command(self, cmd: str, *, break_ms: Optional[float] = 12.5,
                mark_ms: float = 9.17, gap_bits: float = 0.0,
                parity_error_at: Optional[int] = None, release_ms: float = 0.0,
                at: Optional[float] = None) -> Command:
        t = max(float(self.now + 2), at or 0.0)
        if self._changes:
            t = max(t, float(self._changes[-1][0]))
        start = int(round(t))
        self._at(t, 0)                                      # driven, marking
        if break_ms is not None:
            self._at(t, 1)
            t += self.ms(break_ms)
            self._at(t, 0)
        t += self.ms(mark_ms) if break_ms is not None else self.T
        end = self._send(t, [ord(c) for c in cmd], parity_error_at, gap_bits)
        self._at(end + self.ms(release_ms), None)
        rec = Command(cmd, end, start)
        self.sent.append(rec)
        return rec

    def clock(self, level: int, cycle: int) -> None:
        """Take the line's level of ``cycle``; set the drive for ``cycle + 1``."""
        self._step(level, cycle)
        dec = self.decoder
        self._chars = min(self._chars, len(dec.chars))     # a break pops one
        while self._chars < len(dec.chars):
            ch = dec.chars[self._chars]
            self._chars += 1
            if not self.sent or ch.start < self.sent[-1].end:
                continue                                    # its own
            rec = self.sent[-1]
            if rec.response and rec.response[-1] == 10:
                continue
            rec.response.append(ch.value)
            rec.answered = rec.response[-1] == 10


# ------------------------------------------------------------- checker
@dataclass
class Timing:
    """One command and its response on the line, in milliseconds, and the
    figures of section 7 they break (``violations``, empty if none)."""

    command: str = ""
    response: str = ""
    break_ms: Optional[float] = None    # None: no break before the command
    mark_ms: Optional[float] = None     # the break's end to the first start bit
    release_ms: Optional[float] = None  # the command's end to the recorder's release
    start_ms: Optional[float] = None    # the command's end to the response's start bit
    sensor_mark_ms: Optional[float] = None   # the sensor's own marking before it
    sensor_release_ms: Optional[float] = None
    gaps_ms: List[float] = field(default_factory=list)
    cmd_end: float = 0.0
    violations: List[str] = field(default_factory=list)


def _off_after(line: Sdi12Line, name: str, cycle: float) -> Optional[int]:
    for a, b in line.on.get(name, []):
        if a <= cycle and (b is None or cycle < b):
            return b
    return None


def check_transaction(line: Sdi12Line, recorder: str, sensor: str,
                      since: float = 0) -> Timing:
    """The first command ``recorder`` sent from cycle ``since`` on, and what
    ``sensor`` answered, measured on the line against section 7."""
    t = Timing()
    ms, tol = line.ms, TOL / MS
    cmd = []
    for ch in line.chars_by(recorder):
        if ch.start >= since:
            cmd.append(ch)
            if ch.value == ord("!"):
                break
    if not cmd or cmd[-1].value != ord("!"):
        t.violations.append("no command from %s" % recorder)
        return t
    t.command, t.cmd_end = text(c.value for c in cmd), cmd[-1].end
    brk = [b for b in line.decoder.breaks if since <= b.start and b.end <= cmd[0].start]
    if brk:
        t.break_ms, t.mark_ms = ms(brk[-1].clocks), ms(cmd[0].start - brk[-1].end)
        if t.break_ms < BREAK_MIN / MS - tol:
            t.violations.append("break %.2f ms < 12 ms" % t.break_ms)
        if t.mark_ms < MARK_AFTER_BREAK / MS - tol:
            t.violations.append("marking after the break %.2f ms < 8.33 ms" % t.mark_ms)
    off = _off_after(line, recorder, cmd[-1].start)
    if off is not None:
        t.release_ms = ms(off - t.cmd_end)
    if t.release_ms is None or t.release_ms > RELEASE_MAX / MS + tol:
        t.violations.append("recorder released at %s ms > 7.5 ms" % t.release_ms)
    resp = []
    for ch in line.chars_by(sensor):
        if ch.start >= t.cmd_end:
            resp.append(ch)
            if ch.value == 10:
                break
    for group in (cmd, resp):
        for a, b in zip(group, group[1:]):
            t.gaps_ms.append(ms(b.start - a.end))
    if any(g > GAP_MAX / MS for g in t.gaps_ms):
        t.violations.append("a gap of %.2f ms > 1.66 ms" % max(t.gaps_ms))
    if not resp:
        return t
    t.response = text(c.value for c in resp)
    t.start_ms = ms(resp[0].start - t.cmd_end)
    if t.start_ms > RESPONSE_START_MAX / MS + tol:
        t.violations.append("response started at %.2f ms > 15 ms" % t.start_ms)
    on = [a for a, b in line.on.get(sensor, [])
          if a <= resp[0].start and (b is None or resp[0].start < b)]
    if on:
        t.sensor_mark_ms = ms(resp[0].start - on[-1])
    if t.sensor_mark_ms is None or t.sensor_mark_ms < RESPONSE_MARK / MS - tol:
        t.violations.append("sensor marked %s ms < 8.33 ms" % t.sensor_mark_ms)
    off = _off_after(line, sensor, resp[-1].start)
    if off is not None:
        t.sensor_release_ms = ms(off - resp[-1].end)
    if t.sensor_release_ms is None or t.sensor_release_ms > RELEASE_MAX / MS + tol:
        t.violations.append("sensor released at %s ms > 7.5 ms" % t.sensor_release_ms)
    return t
