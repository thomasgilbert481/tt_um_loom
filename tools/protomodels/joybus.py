"""N64 / GameCube controller bus ("joybus") reference models.

A console (host) and a controller (device) for the single open-drain data
wire of the Nintendo 64 and GameCube controller ports, a frame checker for
the bit shapes, and error injection. Written from public secondary sources
(there is no public specification from the console makers), never from
``firmware/joybus_*.loom``. The sources, read 2026-11-05:

* **A** N64brew wiki, "Joybus Protocol" (https://n64brew.dev/wiki/Joybus_Protocol):
  the 0, 1 and controller stop bits are 4 us long and the console stop bit
  3 us; the line idles high; an accessory "must begin responding with data
  within about 62.5 microseconds"; a device that does not know a command
  does not answer and the console reports a timeout. Commands (N64): 0x00
  info and 0xFF reset, 1 byte out, 3 back (0x05 0x00 and a status byte);
  0x01 poll, 1 out, 4 back. GameCube: 0x00 / 0xFF info, 3 back; 0x40 short
  poll, 3 out, 8 back; 0x43 long poll, 3 out, 10 back; 0x41 origin, length
  "?" on that page.
* **C** raphnet, gc_n64_usb-v3, ``gcn64txrx.S``
  (https://github.com/raphnet/gc_n64_usb-v3): a 0 is 3 us low then 1 us
  high, a 1 is 1 us low then 3 us high; MSB first; the receiver gives up
  after about 50 us with no edge. **D** the same repository's
  ``gcn64_protocol.c``: GameCube controller identifiers 0x090020 and
  0x090023; N64 0x050000, 0x050001, 0x050002.
* **E** JonnyHaystack, joybus-pio, ``src/joybus.pio``
  (https://github.com/JonnyHaystack/joybus-pio): 250 kbit/s, a bit is 1 us
  low, 2 us of data, 1 us high; the stop bit is 1 us low, then high.
* **F** int03.co.uk, "GameCube controller" notes
  (http://www.int03.co.uk/crema/hardware/gamecube/gc-control.html): about
  4 us a bit on both sides (an earlier 5 us estimate withdrawn); a 0 is 3 us
  low and 1 us high; MSB first; the poll sent as 0x40 0x03 0x02 (the last
  bit the rumble); the 8-byte reply layout; a pull-up to 3.43 V.

What is checked against two sources, and what is not:

* bit cell 4 us, 0 = 3 + 1, 1 = 1 + 3, MSB first: A, C, E, F agree;
* console stop bit: 1 us low (C sends a 1's low; E's stop is 1 us low) and
  3 us in all (A): the split is single-sourced;
* device stop bit: 1 us low (E only) and 4 us in all (A): UNCHECKED split;
* the device's reply delay: no source gives a figure. A gives an upper
  bound of about 62.5 us; C times out at about 50 us; F saw about 15 us in
  early tests and none later. The model device answers DELAY_US = 2 us
  after the stop bit's rise: UNCHECKED, the "few microseconds" of the brief;
* tolerances: NO source gives one. :data:`WINDOWS` are this model's own
  (+-0.25 us on a firmware edge, which Loom's latched writes make exact to
  the clock), marked ``"unchecked"``;
* N64 info status byte: A says bits 1:0 are 1 for a pak inserted and 2 for
  none, D lists 0x00 for none. Not reconciled; the model does not care.
* 0x41 origin reply length: not given by A (it says "?"), 10 bytes in other
  adapters by recollection only: UNCHECKED, so it is not in the model
  device's default table.

The models sit on a :class:`tools.protomodels.onewire.OneWireBus` line (the
same open-drain wire with a pull-up, one ``uio`` pad per Loom thread, joined
by the bench). Times are in chip clocks (50 MHz unless ``clock`` says
otherwise); the constants are microseconds.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Sequence, Tuple

from .onewire import CLOCK, Agent, OneWireBus  # noqa: F401  (the line model)

#: The sources' figures, microseconds (module doc).
JOYBUS = dict(
    cell=4.0,               # A, F, E (250 kbit/s)
    low0=3.0,               # C, F, E: a 0's low
    low1=1.0,               # C, E: a 1's low
    console_stop_low=1.0,   # C, E (3 us in all, A)
    device_stop_low=1.0,    # E only: UNCHECKED split (4 us in all, A)
    reply_max=62.5,         # A: "within about 62.5 microseconds"
    delay=2.0,              # the model device's turnaround: UNCHECKED
)

#: The checker's windows, microseconds, inclusive, with where each comes from.
WINDOWS: Dict[str, Tuple[float, float, str]] = dict(
    low1=(0.75, 1.25, "unchecked"),
    low0=(2.75, 3.25, "unchecked"),
    cell=(3.75, 4.25, "unchecked"),
    stop_low=(0.75, 2.25, "unchecked"),   # 1 us (E); 2 us in some adapters
    reply=(0.5, 62.5, "A, upper bound only"),
)

#: The decision between a 0 and a 1, microseconds of low (between 1 and 3).
THRESHOLD_US = 2.0

#: Bytes out (command and arguments) and back, per command (A).
N64_COMMANDS = {0x00: (1, 3), 0xFF: (1, 3), 0x01: (1, 4)}
GC_COMMANDS = {0x00: (1, 3), 0xFF: (1, 3), 0x40: (3, 8)}


def bits_of(data: Sequence[int]) -> List[int]:
    """The bits of ``data``, MSB first."""
    return [(byte >> (7 - i)) & 1 for byte in data for i in range(8)]


def bytes_of(bits: Sequence[int]) -> bytes:
    """Whole bytes from MSB-first bits (a partial last byte is dropped)."""
    out = []
    for i in range(0, len(bits) - 7, 8):
        v = 0
        for b in bits[i:i + 8]:
            v = v << 1 | b
        out.append(v)
    return bytes(out)


def _us(value_us: float, clock: int) -> int:
    return int(round(value_us * clock / 1_000_000))


def _in(name: str, value: float) -> bool:
    lo, hi, _ = WINDOWS[name]
    return lo <= value <= hi


def lows_of(edges: Sequence[Tuple[int, int]]) -> List[Tuple[int, int]]:
    """``[(fall, rise)]`` cycles from a line's edges ``[(cycle, level)]``
    (a low still open at the end is left out)."""
    out, fall = [], None
    for cycle, level in edges:
        if level == 0 and fall is None:
            fall = cycle
        elif level == 1 and fall is not None:
            out.append((fall, cycle))
            fall = None
    return out


def frames_of(lows: Sequence[Tuple[int, int]], *, clock: int = CLOCK,
              gap_us: float = 5.0) -> List[List[Tuple[int, int]]]:
    """Split lows into frames: a new frame starts after ``gap_us`` of high
    line (more than any high inside a frame, 3 us)."""
    gap = _us(gap_us, clock)
    frames: List[List[Tuple[int, int]]] = []
    for low in lows:
        if frames and low[0] - frames[-1][-1][1] < gap:
            frames[-1].append(low)
        else:
            frames.append([low])
    return frames


def check_frame(lows: Sequence[Tuple[int, int]], *, clock: int = CLOCK,
                nbytes: Optional[int] = None) -> dict:
    """Hold one frame's lows to :data:`WINDOWS` and decode it.

    ``lows`` are the frame's ``(fall, rise)`` cycles; with ``nbytes`` the
    frame must be that many bytes and a stop bit, otherwise the last low is
    taken as the stop bit when there are ``8 n + 1``. Returns ``data``
    (whole bytes), ``bits``, ``stop`` (its low in us, or None) and
    ``violations``: ``("low", i, us)`` for a data low in neither window,
    ``("cell", i, us)`` for a fall-to-fall time out of its window,
    ``("stop", us)``, ``("nostop",)`` and ``("count", n)``.
    """
    us = 1_000_000 / clock
    n = len(lows)
    if nbytes is not None:
        ndata = min(n, 8 * nbytes)
    else:
        ndata = n - 1 if n % 8 == 1 else n
    violations: List[tuple] = []
    bits = []
    for i in range(ndata):
        width = (lows[i][1] - lows[i][0]) * us
        bits.append(1 if width < THRESHOLD_US else 0)
        if not (_in("low1", width) or _in("low0", width)):
            violations.append(("low", i, width))
        if i + 1 < n:
            cell = (lows[i + 1][0] - lows[i][0]) * us
            if not _in("cell", cell):
                violations.append(("cell", i, cell))
    stop = None
    if n > ndata:
        stop = (lows[ndata][1] - lows[ndata][0]) * us
        if not _in("stop_low", stop):
            violations.append(("stop", stop))
        if n > ndata + 1:
            violations.append(("count", n))
    else:
        violations.append(("nostop",))
    if nbytes is not None and ndata < 8 * nbytes:
        violations.append(("count", n))
    elif nbytes is None and ndata % 8:
        violations.append(("count", n))
    return dict(data=bytes_of(bits), bits=bits, stop=stop, violations=violations)


def plan_frame(data: Sequence[int], start: int, *, clock: int = CLOCK,
               stop_low_us: float = 1.0, stop: bool = True,
               lows_us: Optional[Dict[int, float]] = None) -> List[Tuple[int, bool]]:
    """The pull schedule ``[(cycle, pull)]`` of one frame from ``start``:
    each bit a 4 us cell (3 + 1 for a 0, 1 + 3 for a 1), then the stop bit's
    low. ``lows_us`` overrides single bits' lows (error injection: a short
    low); ``stop=False`` leaves the stop bit out."""
    plan = []
    t = 0.0
    for i, bit in enumerate(bits_of(data)):
        low = (lows_us or {}).get(i, JOYBUS["low1"] if bit else JOYBUS["low0"])
        plan.append((start + _us(t, clock), True))
        plan.append((start + _us(t + low, clock), False))
        t += JOYBUS["cell"]
    if stop:
        plan.append((start + _us(t, clock), True))
        plan.append((start + _us(t + stop_low_us, clock), False))
    return plan


class _Player(Agent):
    """An agent that plays a pull schedule ``[(cycle, pull)]``."""

    def __init__(self) -> None:
        self.wake = 1 << 62
        self._plan: List[Tuple[int, bool]] = []
        self._pull = False

    def _play(self, cycle: int) -> bool:
        while self._plan and self._plan[0][0] <= cycle + 1:
            self._pull = self._plan.pop(0)[1]
        self.wake = self._plan[0][0] - 1 if self._plan else 1 << 62
        return self._pull


class JoybusDevice(_Player):
    """A controller on the line (module doc). It decodes every console
    frame by the length of each low (:data:`THRESHOLD_US`), and when the
    command is one of ``lengths`` (bytes out, default 0x00, 0xFF, 0x01 and
    0x40 with its two arguments) and its stop bit has risen, it answers
    ``replies[command]`` ``delay_us`` later with its own stop bit. A command
    with no reply is not answered (A).

    Error injection: ``respond=False`` stays silent; ``stop=False`` leaves
    the device stop bit out; ``lows_us`` ``{bit index: low}`` changes single
    bits' lows (a short low); ``truncate`` sends only that many bytes;
    ``delay_us`` and ``stop_low_us`` move the turnaround and the stop bit.
    ``frames`` records each console frame: ``(fall cycle, check_frame
    result)``; ``sent`` each reply sent."""

    name = "device"

    def __init__(self, replies: Optional[Dict[int, bytes]] = None, *,
                 clock: int = CLOCK, lengths: Optional[Dict[int, int]] = None,
                 delay_us: float = JOYBUS["delay"], respond: bool = True,
                 stop: bool = True, stop_low_us: float = 1.0,
                 lows_us: Optional[Dict[int, float]] = None,
                 truncate: Optional[int] = None) -> None:
        super().__init__()
        self.replies = dict(replies if replies is not None else
                            {0x00: b"\x05\x00\x00", 0xFF: b"\x05\x00\x00",
                             0x01: bytes(4)})
        self.lengths = dict(lengths if lengths is not None else
                            {0x00: 1, 0xFF: 1, 0x01: 1, 0x40: 3})
        self.clock_hz = clock
        self.delay_us, self.respond, self.stop = delay_us, respond, stop
        self.stop_low_us, self.lows_us, self.truncate = stop_low_us, lows_us, truncate
        self.frames: List[Tuple[int, dict]] = []
        self.sent: List[Tuple[int, bytes]] = []
        self._gap = _us(5.0, clock)
        self._lows: List[Tuple[int, int]] = []
        self._fall: Optional[int] = None
        self._skip = False

    def clock(self, level: int, cycle: int) -> bool:
        if self._plan or self._pull:
            return self._play(cycle)
        if level == 0 and self._fall is None:
            if self._lows and cycle - self._lows[-1][1] >= self._gap:
                self._lows, self._skip = [], False      # a new frame
            self._fall = cycle
        elif level == 1 and self._fall is not None:
            self._lows.append((self._fall, cycle))
            self._fall = None
            if not self._skip:
                self._took(cycle)
        return False

    def _took(self, cycle: int) -> None:
        n = len(self._lows)
        if n < 8:
            return
        cmd = check_frame(self._lows[:8], clock=self.clock_hz, nbytes=1)["data"][0]
        out = self.lengths.get(cmd)
        if out is None or cmd not in self.replies:
            self._skip = True                           # not answered (A)
            return
        if n < 8 * out + 1:
            return
        result = check_frame(self._lows, clock=self.clock_hz, nbytes=out)
        self.frames.append((self._lows[0][0], result))
        self._skip = True
        if self.respond:
            data = self.replies[cmd][:self.truncate]
            start = cycle + _us(self.delay_us, self.clock_hz)
            self._plan = plan_frame(data, start, clock=self.clock_hz,
                                    stop_low_us=self.stop_low_us, stop=self.stop,
                                    lows_us=self.lows_us)
            self.wake = self._plan[0][0] - 1
            self.sent.append((start, bytes(data)))


class Transaction:
    """One command by :class:`JoybusHost`: filled in when ``done``."""

    def __init__(self, data: bytes, nbytes: int) -> None:
        self.data, self.nbytes = bytes(data), nbytes
        self.done = False
        self.release: Optional[int] = None       # the console stop bit's rise
        self.edges: List[Tuple[int, int]] = []
        self.result: Optional[dict] = None

    @property
    def reply(self) -> bytes:
        return self.result["data"] if self.result else b""

    @property
    def violations(self) -> List[tuple]:
        return self.result["violations"] if self.result else []

    @property
    def turnaround_us(self) -> Optional[float]:
        """The console stop bit's rise to the reply's first fall."""
        return self.result.get("turnaround") if self.result else None

    def __repr__(self) -> str:
        return "Transaction(%s, %d back, done %s)" % (self.data.hex(), self.nbytes, self.done)


class JoybusHost(_Player):
    """A console for testing a device: :meth:`send` plays the command with
    exact 4 us cells and the console stop bit, then records the line until
    it has been high ``quiet_us`` after the last edge (or ``timeout_us``
    with no reply at all) and holds the reply to :func:`check_frame` with
    the expected length; the turnaround is checked against
    ``WINDOWS["reply"]``. Error injection: ``lows_us`` (a short low in the
    command), ``stop=False`` (no console stop bit)."""

    name = "host"

    def __init__(self, *, clock: int = CLOCK, quiet_us: float = 10.0,
                 timeout_us: float = 100.0) -> None:
        super().__init__()
        self.clock_hz = clock
        self.quiet = _us(quiet_us, clock)
        self.timeout = _us(timeout_us, clock)
        self.transactions: List[Transaction] = []
        self._tx: Optional[Transaction] = None
        self._opts: dict = {}
        self._level = 1

    def send(self, data: Sequence[int], nbytes: int, *, stop: bool = True,
             lows_us: Optional[Dict[int, float]] = None) -> Transaction:
        """Start a command at the next clock the bus gives this host."""
        tx = Transaction(bytes(data), nbytes)
        self.transactions.append(tx)
        self._tx, self._opts = tx, dict(stop=stop, lows_us=lows_us)
        self.wake = 0
        return tx

    def clock(self, level: int, cycle: int) -> bool:
        tx = self._tx
        if tx is None:
            self.wake = 1 << 62
            return False
        if tx.release is None and not self._plan and not self._pull:
            if not tx.edges and self._opts is not None:
                self._plan = plan_frame(tx.data, cycle + 1, clock=self.clock_hz,
                                        **self._opts)
                self._opts = None
                return self._play(cycle)
        if self._plan or self._pull:
            return self._play(cycle)
        if tx.release is None:
            if level == 1:
                tx.release, self._level = cycle, 1
            self.wake = cycle + 1
            return False
        if level != self._level:
            tx.edges.append((cycle, level))
            self._level = level
        last = tx.edges[-1][0] if tx.edges else tx.release
        limit = self.quiet if tx.edges else self.timeout
        if level == 1 and cycle - last >= limit:
            self._finish(tx)
        else:
            self.wake = last + limit if level == 1 else 1 << 62
        return False

    def _finish(self, tx: Transaction) -> None:
        lows = lows_of(tx.edges)
        result = check_frame(lows, clock=self.clock_hz, nbytes=tx.nbytes)
        if lows:
            turn = (lows[0][0] - tx.release) * 1_000_000 / self.clock_hz
            result["turnaround"] = turn
            if not _in("reply", turn):
                result["violations"].append(("reply", turn))
        else:
            result["turnaround"] = None
            result["violations"].append(("noreply",))
        tx.result, tx.done = result, True
        self._tx = None
        self.wake = 1 << 62
