"""ST SWIM reference models: a debug host, an STM8-style target and a
checker of every low on the line.

Written from ST UM0470 "STM8 SWIM communication protocol and debug module",
DocID14024 Rev 4 (August 2016), never from ``firmware/swim_*.loom``:

* 3.2 and Figures 4, 5: the entry sequence. The SWIM pin is forced low
  16 us; then "four pulses at 1 kHz followed by four pulses at 2 kHz",
  starting and ending with the pin at 1; the target detects the frequency
  ratio "regardless of the internal LSI frequency value"; after a delay it
  sends a synchronisation frame of 128 SWIM clock periods low; the line
  must then be released at 1 for at least 300 ns before communication;
  the host writes 0xA0 to SWIM_CSR (step 6); the SWIM clock is HSI/2 =
  8 MHz and the SWIM starts at the low speed (step 8). The delay before
  the sync and the order of the high and low halves of each pulse are not
  legible in the copy read (Figure 5); this model takes each pulse as a
  high half then a low half (QUESTIONS.md item 1);
* 3.3.1, 3.3.2: return-to-zero bits. Low speed: 22 SWIM clocks a bit, a 1
  is 2 low and 20 high, a 0 is 20 low and 2 high, decoded 1 for at most 8
  samples low and 0 for 9 or more. High speed: 10 clocks, 2 or 8 low,
  decoded at 4 / 5 (in :data:`HS`, for reference: nothing here runs it);
* 3.4, Figures 8, 9: a frame is a header bit (0 from the host, 1 from the
  target), b2..b0 of a command or b7..b0 of a byte, a parity bit "XOR
  between all b(i)", then an ACK (1) or NACK (0) from the receiver, which
  NACKs a parity error or when it is not ready;
* 3.5 and Table 1: SRST 000, ROTF 001, WOTF 010 (011 and 1xx reserved);
  ROTF and WOTF are the command, N (1 to 255), @E @H @L, then N bytes. For
  ROTF "if the host sends a NACK to a data byte, the device will send the
  same byte again"; for WOTF the host sends a NACKed byte again "until it
  is acknowledged". SRST resets only with SWIM_CSR.SWIM_DM set; with it
  clear, ROTF and WOTF reach only the SWIM registers;
* 3.6: the communication reset: the host holds the pin low 128 SWIM
  clocks; the target resets its state machine after more than 64, goes to
  the low speed and answers with the sync frame;
* 3.9 and Table 3: open drain; inter-bit time > 0, inter-frame time >= 0;
  the low time of a 0 (low speed 1.6 to 2.4 us, high speed 768 to 832 ns)
  and of a 1 (150 to 250 ns, 192 to 208 ns), in a column headed "HSI =
  10 MHz". With 3.9's sampling period of 100 ns those are 16 to 24 and
  1.5 to 2.5 SWIM clocks at the low speed; this model holds every low to
  those windows in SWIM clocks (:data:`TABLE3`, QUESTIONS.md item 2);
* 3.10.1: SWIM_CSR at 0x7F80, reset value 0x00; bit 5 SWIM_DM, bit 4 HS.

Neither the delay from the entry to the sync, the time a receiver takes
to send its ACK, nor any timeout is given; the models' choices are
parameters, and the checks hold only what UM0470 gives.

* :class:`SwimTarget` detects the entry by the ratio of its pulses, sends
  the sync, decodes every host bit by its low time (the 8 / 9 sample rule)
  and holds it to Table 3, ACKs or NACKs every frame, serves SRST, ROTF and
  WOTF from a memory dict, and resets on a long low. Its SWIM clock can be
  off nominal (``hsi_error``). Error injection: ``nack`` (host frames it
  NACKs although good), ``bad_parity`` (ROTF byte indices it sends with the
  wrong parity, once per time listed), ``silent``.
* :class:`SwimHost` runs a queue of :class:`Op` (entry, comm reset, SRST,
  ROTF, WOTF), retries NACKed frames, NACKs bad target frames, measures
  the sync and holds the target's lows to Table 3. Error injection:
  ``bad_parity`` (frames it first sends with the wrong parity).
* :func:`check_lows` holds a list of low intervals (what a bus recorded
  for one driver) to Table 3 and decodes them; :func:`frames` groups the
  bits; :func:`check_entry` holds a host's entry sequence to 3.2.

The line is :class:`tools.protomodels.onewire.OneWireBus`, the open-drain
wire with its pull-up, one pad per Loom thread. Times are in clocks of the
chip (50 MHz unless ``clock`` says otherwise).
"""

from __future__ import annotations

import dataclasses
from typing import Dict, Iterator, List, Optional, Sequence, Set, Tuple

from .onewire import CLOCK, Agent, OneWireBus  # noqa: F401  (the line model)

SWIM_HZ = 8_000_000         # 3.2 step 8: HSI 16 MHz / 2
#: 3.3.2 low speed, in SWIM clocks: bit, low of a 0, low of a 1, most
#: samples low still decoded as a 1.
LS = dict(bit=22, low0=20, low1=2, max1=8)
#: 3.3.1 high speed, for reference.
HS = dict(bit=10, low0=8, low1=2, max1=4)
#: 3.9 Table 3, low speed, read in SWIM clocks (module doc): (min, max).
TABLE3 = dict(tb0=(16.0, 24.0), tb1=(1.5, 2.5))
SYNC_SC = 128               # 3.2 step 4, 3.6
RESET_SC = 128              # 3.6: the host's communication reset
RESET_DETECT_SC = 64        # 3.6: the target resets after more than this
ENTRY_LOW_US = 16           # 3.2 step 1
ENTRY_PULSES = (4, 1000), (4, 2000)     # 3.2 step 2: (count, Hz)
RELEASE_NS = 300            # 3.2 step 5
SRST, ROTF, WOTF = 0, 1, 2  # Table 1
SWIM_CSR = 0x7F80           # 3.10.1
SWIM_DM, HS_BIT = 0x20, 0x10
CSR_INIT = 0xA0             # 3.2 step 6


def parity(value: int) -> int:
    """3.4: the XOR of all the b(i)."""
    return bin(value).count("1") & 1


def decode(low_sc: float) -> int:
    """3.3.2: 1 for at most 8 samples low, 0 for 9 or more. A low of L SWIM
    clocks gives floor or ceil of L samples; the model splits at 8.5."""
    return 1 if low_sc < LS["max1"] + 0.5 else 0


def check_lows(pulls: Sequence[Tuple[int, Optional[int]]], clock: int = CLOCK,
               swim_hz: float = SWIM_HZ) -> Tuple[List[object], List[tuple]]:
    """Each low ``(fall, rise)`` in clocks: a bit (1 or 0, decoded by 3.3.2
    and held to Table 3), or "long" (over 64 SWIM clocks: an entry low, a
    sync or a reset, 3.2 and 3.6). Returns the items and the violations,
    ``(index, name, low in SWIM clocks)``; an open low is skipped."""
    sc = clock / swim_hz
    items: List[object] = []
    violations: List[tuple] = []
    for i, (fall, rise) in enumerate(pulls):
        if rise is None:
            continue
        low = (rise - fall) / sc
        if low > RESET_DETECT_SC:
            items.append("long")
            continue
        bit = decode(low)
        lo, hi = TABLE3["tb1" if bit else "tb0"]
        if not lo <= low <= hi:
            violations.append((i, "tb1" if bit else "tb0", round(low, 3)))
        items.append(bit)
    return items, violations


def frames(items: Sequence[object]) -> List[List[int]]:
    """Split decoded items at every "long" low into runs of bits."""
    out: List[List[int]] = [[]]
    for item in items:
        if item == "long":
            out.append([])
        else:
            out[-1].append(int(item))
    return [run for run in out if run]


def bits_value(bits: Sequence[int]) -> int:
    value = 0
    for b in bits:
        value = value << 1 | b
    return value


def frame_bits(header: int, value: int, n: int) -> List[int]:
    """3.4: header, the n bits MSB first, the parity."""
    return [header] + [(value >> (n - 1 - i)) & 1 for i in range(n)] + [parity(value)]


def check_entry(pulls: Sequence[Tuple[int, Optional[int]]], clock: int = CLOCK,
                tol: float = 0.05, scale: float = 1.0) -> List[tuple]:
    """A host's entry sequence (its first nine lows) against 3.2: the first
    low 16 us, then four lows and four highs of 500 us (1 kHz), four of
    250 us (2 kHz), each within ``tol``; the first pulse's high half from
    the end of the 16 us low. ``scale`` divides the pulse times (the
    time-scaled build of the tests). Returns ``(index, name, us)``."""
    us = clock / 1_000_000
    out: List[tuple] = []
    lows = [p for p in pulls if p[1] is not None][:9]
    if len(lows) < 9:
        return [(len(lows), "missing", None)]
    first = (lows[0][1] - lows[0][0]) / us
    if abs(first - ENTRY_LOW_US) > tol * ENTRY_LOW_US:
        out.append((0, "entry_low", round(first, 3)))
    prev_rise = lows[0][1]
    halves = [500 / scale] * 4 + [250 / scale] * 4
    for i, ((fall, rise), half) in enumerate(zip(lows[1:], halves), 1):
        for name, t in (("high", (fall - prev_rise) / us), ("low", (rise - fall) / us)):
            if abs(t - half) > tol * half:
                out.append((i, name, round(t, 3)))
        prev_rise = rise
    return out


class _Reset(Exception):
    """A low past 64 SWIM clocks seen while receiving (3.6)."""


class _Proc(Agent):
    """An agent run by a generator. The generator sets ``self.pull`` (in
    effect from the next cycle, as for every bench model) and yields
    ``("t", cycle)`` to sleep until ``cycle`` or ``("l", level, cycle)`` to
    sleep until the line is at ``level`` or ``cycle`` comes."""

    def __init__(self, clock: int, swim_hz: float) -> None:
        self.clock_hz = clock
        self.sc = clock / swim_hz           # chip clocks per SWIM clock
        self.us = clock / 1_000_000
        self.pull = False
        self.level, self.now = 1, 0
        self.violations: List[tuple] = []
        self.last_fall = 0                  # the last bit's fall, either end
        self._gen: Optional[Iterator] = None
        self._req: Optional[tuple] = None
        self.wake = 0

    def clock(self, level: int, cycle: int) -> bool:
        self.level, self.now = level, cycle
        if self._gen is None:
            self._gen = self.run()
        while True:
            r = self._req
            if r is not None:
                if r[0] == "t" and cycle < r[1]:
                    break
                if r[0] == "l" and level != r[1] and cycle < r[2]:
                    break
            try:
                self._req = next(self._gen)
            except StopIteration:
                self._req = ("t", 1 << 62)
        self.wake = self._req[1] if self._req[0] == "t" else self._req[2]
        return self.pull

    def run(self) -> Iterator:
        yield ("t", 1 << 62)

    # ------------------------------------------------------------ helpers
    def sleep(self, clocks: float) -> Iterator:
        yield ("t", self.now + max(1, round(clocks)))

    def wait_level(self, level: int, timeout: Optional[float] = None) -> Iterator:
        """Until the line is at ``level``; True, or False on the timeout."""
        end = 1 << 62 if timeout is None else self.now + max(1, round(timeout))
        if self.level != level:
            yield ("l", level, end)
        return self.level == level

    def rx_bit(self, timeout: Optional[float] = None) -> Iterator:
        """One bit sent by the other end: its fall (None on the timeout), its
        low decoded by 3.3.2 and held to Table 3. A low past 64 SWIM clocks
        raises :class:`_Reset` once the line is high again."""
        yield from self.wait_level(1)       # our own last low, or the other's
        ok = yield from self.wait_level(0, timeout)
        if not ok:
            return None
        fall = self.now
        ok = yield from self.wait_level(1, (RESET_DETECT_SC + 1) * self.sc)
        if not ok:
            yield from self.wait_level(1)
            raise _Reset()
        low = (self.now - fall) / self.sc
        bit = decode(low)
        lo, hi = TABLE3["tb1" if bit else "tb0"]
        if not lo <= low <= hi:
            self.violations.append(("tb1" if bit else "tb0", fall, round(low, 3)))
        self.last_fall = fall
        return bit

    def tx_bits(self, bits: Sequence[int], cell: float, low0: float, low1: float) -> Iterator:
        """Bits from now, one cell each, MSB first, each a fall then its low;
        returns at the last bit's rise (``last_fall`` is its fall)."""
        start = self.now
        for i, b in enumerate(bits):
            yield ("t", start + round(i * cell * self.sc))
            self.pull = True
            self.last_fall = self.now + 1
            yield ("t", self.now + round((low1 if b else low0) * self.sc))
            self.pull = False


class SwimTarget(_Proc):
    """An STM8-style SWIM target on the line (module doc). ``memory`` maps
    24-bit addresses to bytes (others read 0x00, writes to them are
    ignored); ``csr`` is SWIM_CSR. ``active=True`` starts it past the entry.

    Its own times, in its own SWIM clocks (``hsi_error`` puts its clock
    off 8 MHz): bits of 22 clocks, lows of 20 and 2 (3.3.2); the ACK's
    fall ``ack_delay`` clocks after the end of the host's last cell; a
    ROTF byte ``resp_delay`` after that; the sync ``sync_delay_us`` after
    the entry or the reset's rise. Records: ``entries`` and ``resets``
    (cycles of the sync falls' causes), ``frames`` ``(index, n, value,
    good, acked)`` for every host frame, ``log`` ``(cmd, n, addr, bytes)``
    for every command, ``host_acks`` ``(byte index, ack)``, ``violations``
    (host lows outside Table 3, a missing host ACK)."""

    name = "target"

    def __init__(self, memory: Optional[Dict[int, int]] = None, *, csr: int = 0,
                 clock: int = CLOCK, hsi_error: float = 0.0, active: bool = False,
                 ack_delay: float = 2, resp_delay: float = 4, sync_delay_us: float = 10,
                 nack: Sequence[int] = (), bad_parity: Sequence[int] = (),
                 silent: bool = False, frame_timeout: float = 400,
                 ack_timeout: float = 2000) -> None:
        super().__init__(clock, SWIM_HZ * (1 + hsi_error))
        self.memory = dict(memory or {})
        self.csr = csr
        self.active = active
        self.ack_delay, self.resp_delay = ack_delay, resp_delay
        self.sync_delay_us = sync_delay_us
        self.nack: Set[int] = set(nack)
        self.bad_parity: List[int] = list(bad_parity)
        self.silent = silent
        self.frame_timeout, self.ack_timeout = frame_timeout, ack_timeout
        self.entries: List[int] = []
        self.resets: List[int] = []
        self.syncs: List[Tuple[int, int]] = []
        self.frames: List[tuple] = []
        self.log: List[tuple] = []
        self.host_acks: List[Tuple[int, int]] = []
        self._n = 0                         # host frames seen
        self._bytes = 0                     # ROTF bytes sent

    # ------------------------------------------------------------ memory
    def read(self, addr: int) -> int:
        if addr == SWIM_CSR:
            return self.csr
        if not self.csr & SWIM_DM:          # 3.5.2: SWIM registers only
            return 0
        return self.memory.get(addr, 0) & 0xFF

    def write(self, addr: int, value: int) -> None:
        if addr == SWIM_CSR:
            self.csr = value
        elif self.csr & SWIM_DM and addr in self.memory:
            self.memory[addr] = value

    # -------------------------------------------------------------- run
    def run(self) -> Iterator:
        need_sync = False
        while True:
            if not self.active:
                yield from self._entry()
                need_sync = True
            try:
                if need_sync:
                    yield from self._sync()
                self.active, need_sync = True, False
                while True:
                    yield from self._command()
            except _Reset:                  # 3.6: reset, low speed, sync
                self.resets.append(self.now)
                self.active, need_sync = True, True

    def _entry(self) -> Iterator:
        """3.2: a low, then sixteen halves, the first eight alike, the last
        eight alike and about half as long (the ratio, not the times)."""
        while True:
            yield from self.wait_level(0)
            yield from self.wait_level(1)
            t, halves = self.now, []
            while len(halves) < 16:
                limit = 4 * halves[0] if halves else 20_000 * self.us
                ok = yield from self.wait_level(1 - self.level, limit)
                if not ok:
                    break
                halves.append(self.now - t)
                t = self.now
            if len(halves) == 16:
                long_, short = halves[:8], halves[8:]
                ml, ms = sum(long_) / 8, sum(short) / 8
                alike = all(abs(h - ml) <= ml / 4 for h in long_) and \
                    all(abs(h - ms) <= ms / 4 for h in short)
                if alike and 1.5 <= ml / ms <= 2.5:
                    self.entries.append(self.now)
                    return

    def _sync(self) -> Iterator:
        yield from self.sleep(self.sync_delay_us * self.us)
        start = self.now
        self.pull = True
        yield from self.sleep(SYNC_SC * self.sc)
        self.pull = False
        self.syncs.append((start + 1, self.now + 1))

    def _ack(self, bit: int) -> Iterator:
        """The receiver's ACK (1) or NACK (0), after the sender's last cell."""
        yield ("t", self.last_fall + round((LS["bit"] + self.ack_delay) * self.sc))
        yield from self.tx_bits([bit], LS["bit"], LS["low0"], LS["low1"])

    def _rx_frame(self, n: int) -> Iterator:
        """A host frame of n bits (3.4): ACKed, or NACKed (bad header or
        parity, or an index in ``nack``) and received again."""
        while True:
            bits = [(yield from self.rx_bit())]
            while len(bits) < n + 2:
                bit = yield from self.rx_bit(self.frame_timeout * self.sc)
                if bit is None:             # a frame cut short: start over
                    self.violations.append(("short_frame", self.now, len(bits)))
                    bits = None
                    break
                bits.append(bit)
            if bits is None:
                continue
            value = bits_value(bits[1:n + 1])
            good = bits[0] == 0 and parity(value) == bits[n + 1]
            i, self._n = self._n, self._n + 1
            ack = int(good and i not in self.nack)
            self.nack.discard(i)
            self.frames.append((i, n, value, good, ack))
            if self.silent:
                continue
            yield from self._ack(ack)
            if ack:
                return value

    def _command(self) -> Iterator:
        cmd = yield from self._rx_frame(3)
        if cmd == SRST:                     # 3.5.1: a reset only with SWIM_DM
            self.log.append((SRST, int(bool(self.csr & SWIM_DM)), None, []))
            return
        if cmd not in (ROTF, WOTF):
            self.log.append((cmd, 0, None, []))
            return
        n = yield from self._rx_frame(8)
        addr = 0
        for _ in range(3):
            addr = addr << 8 | (yield from self._rx_frame(8))
        data: List[int] = []
        self.log.append((cmd, n, addr, data))
        for k in range(n):
            if cmd == WOTF:
                value = yield from self._rx_frame(8)
                self.write((addr + k) & 0xFFFFFF, value)
                data.append(value)
                continue
            value = self.read((addr + k) & 0xFFFFFF)
            data.append(value)
            while True:
                bits = frame_bits(1, value, 8)
                i = self._bytes
                if i in self.bad_parity:     # once per time it is listed
                    self.bad_parity.remove(i)
                    bits[-1] ^= 1
                yield ("t", self.last_fall + round((LS["bit"] + self.resp_delay) * self.sc))
                yield from self.tx_bits(bits, LS["bit"], LS["low0"], LS["low1"])
                ack = yield from self.rx_bit(self.ack_timeout * self.sc)
                self.host_acks.append((i, ack))
                if ack is None:
                    self.violations.append(("no_host_ack", self.now, i))
                    return
                if ack:
                    break
            self._bytes += 1

    @property
    def srsts(self) -> int:
        """SRSTs taken: with SWIM_DM set at the time (3.5.1)."""
        return sum(1 for c in self.log if c[0] == SRST and c[1])


@dataclasses.dataclass
class Op:
    """One host operation: ``kind`` "entry", "reset", "srst", "rotf" or
    "wotf". When ``done``: ``status`` "ok", "no_sync", "no_answer" (no ACK
    or byte in time), "nacked" (a frame NACKed ``tries`` times),
    "bad_frame" (a target frame bad ``tries`` times) or "stuck" (a low past
    64 SWIM clocks); ``data`` the bytes read; ``sync_sc`` the sync's low in
    nominal SWIM clocks; ``nacks`` NACKs taken, ``bad`` target frames
    NACKed."""
    kind: str
    n: int = 0
    addr: int = 0
    data: List[int] = dataclasses.field(default_factory=list)
    done: bool = False
    status: Optional[str] = None
    sync_sc: Optional[float] = None
    nacks: int = 0
    bad: int = 0


class _Fail(Exception):
    def __init__(self, status: str) -> None:
        self.status = status


class SwimHost(_Proc):
    """A SWIM debug host on the line (module doc), at the nominal 8 MHz
    SWIM clock. Its own times, in SWIM clocks: bits of ``cell`` (22) with
    lows ``low0`` (20) and ``low1`` (2); every frame's first fall, and its
    ACK of a target byte, ``gap`` after the end of the last cell; entry
    pulse times divided by ``entry_scale`` (the tests' scaled build).
    Timeouts: ``resp_timeout`` for an ACK or a byte, ``sync_timeout_us``.
    ``bad_parity``: indices of its frames (counted over all frames sent)
    whose first sending has the parity flipped; ``nack``: indices of the
    target frames it receives (counted over all of them) that it NACKs
    although they are good. ``received`` lists ``(index, value, good,
    ack)`` for every target frame."""

    name = "host"

    def __init__(self, *, clock: int = CLOCK, cell: float = 22, low0: float = 20,
                 low1: float = 2, gap: float = 2, entry_scale: float = 1.0,
                 resp_timeout: float = 2000, sync_timeout_us: float = 1000,
                 tries: int = 4, bad_parity: Sequence[int] = (),
                 nack: Sequence[int] = ()) -> None:
        super().__init__(clock, SWIM_HZ)
        self.cell, self.low0, self.low1, self.gap = cell, low0, low1, gap
        self.entry_scale = entry_scale
        self.resp_timeout, self.sync_timeout_us = resp_timeout, sync_timeout_us
        self.tries = tries
        self.bad_parity: Set[int] = set(bad_parity)
        self.ops: List[Op] = []
        self.sent = 0                       # frames sent, retries included
        self.nack: Set[int] = set(nack)
        self.received: List[tuple] = []
        self._queue: List[Op] = []

    # --------------------------------------------------------- the API
    def _add(self, op: Op) -> Op:
        self.ops.append(op)
        self._queue.append(op)
        return op

    def entry(self) -> Op:
        return self._add(Op("entry"))

    def comm_reset(self) -> Op:
        return self._add(Op("reset"))

    def srst(self) -> Op:
        return self._add(Op("srst"))

    def rotf(self, n: int, addr: int) -> Op:
        return self._add(Op("rotf", n, addr))

    def wotf(self, addr: int, data: Sequence[int]) -> Op:
        return self._add(Op("wotf", len(data), addr, list(data)))

    @property
    def idle(self) -> bool:
        return all(op.done for op in self.ops)

    # -------------------------------------------------------------- run
    def run(self) -> Iterator:
        while True:
            if not self._queue:
                yield ("t", self.now + 64)
                continue
            op = self._queue.pop(0)
            try:
                yield from getattr(self, "_" + op.kind)(op)
                op.status = "ok"
            except _Fail as fail:
                op.status = fail.status
            except _Reset:
                op.status = "stuck"
            op.done = True

    # ------------------------------------------------------- operations
    def _next_fall(self) -> int:
        return max(self.now + 1, self.last_fall + round((self.cell + self.gap) * self.sc))

    def _hold_low(self, clocks: float) -> Iterator:
        self.pull = True
        yield from self.sleep(clocks)
        self.pull = False

    def _entry(self, op: Op) -> Iterator:
        """3.2: 16 us low, then each pulse a high half and a low half."""
        yield from self._hold_low(ENTRY_LOW_US * self.us)
        for count, hz in ENTRY_PULSES:
            half = 1_000_000 / hz / 2 / self.entry_scale * self.us
            for _ in range(count):
                yield from self.sleep(half)
                yield from self._hold_low(half)
        yield from self._sync(op)

    def _reset(self, op: Op) -> Iterator:
        """3.6: 128 SWIM clocks low, then the target's sync."""
        yield ("t", self._next_fall())
        yield from self._hold_low(RESET_SC * self.sc)
        yield from self._sync(op)

    def _sync(self, op: Op) -> Iterator:
        yield from self.wait_level(1)
        ok = yield from self.wait_level(0, self.sync_timeout_us * self.us)
        if not ok:
            raise _Fail("no_sync")
        fall = self.now
        ok = yield from self.wait_level(1, 4 * SYNC_SC * self.sc)
        if not ok:
            raise _Fail("no_sync")
        op.sync_sc = (self.now - fall) / self.sc
        # 3.2 step 5: released at least 300 ns before the first frame.
        ready = self.now + round(RELEASE_NS / 1000 * self.us) + 1
        self.last_fall = ready - round((self.cell + self.gap) * self.sc)

    def _send(self, op: Op, value: int, n: int) -> Iterator:
        """A host frame (header 0), resent on a NACK (3.4)."""
        for _ in range(self.tries):
            bits = frame_bits(0, value, n)
            if self.sent in self.bad_parity:
                bits[-1] ^= 1
            self.sent += 1
            yield ("t", self._next_fall())
            yield from self.tx_bits(bits, self.cell, self.low0, self.low1)
            ack = yield from self.rx_bit(self.resp_timeout * self.sc)
            if ack is None:
                raise _Fail("no_answer")
            if ack:
                return
            op.nacks += 1
        raise _Fail("nacked")

    def _recv(self, op: Op) -> Iterator:
        """A target frame (header 1), NACKed and received again if bad."""
        for _ in range(self.tries):
            bits = []
            for _ in range(10):
                bit = yield from self.rx_bit(self.resp_timeout * self.sc)
                if bit is None:
                    raise _Fail("no_answer")
                bits.append(bit)
            value = bits_value(bits[1:9])
            good = bits[0] == 1 and parity(value) == bits[9]
            i = len(self.received)
            ack = int(good and i not in self.nack)
            self.received.append((i, value, good, ack))
            yield ("t", self._next_fall())
            yield from self.tx_bits([ack], self.cell, self.low0, self.low1)
            if ack:
                return value
            op.bad += 1
        raise _Fail("bad_frame")

    def _srst(self, op: Op) -> Iterator:
        yield from self._send(op, SRST, 3)

    def _head(self, op: Op, cmd: int) -> Iterator:
        yield from self._send(op, cmd, 3)
        yield from self._send(op, op.n, 8)
        for shift in (16, 8, 0):
            yield from self._send(op, (op.addr >> shift) & 0xFF, 8)

    def _rotf(self, op: Op) -> Iterator:
        yield from self._head(op, ROTF)
        for _ in range(op.n):
            op.data.append((yield from self._recv(op)))

    def _wotf(self, op: Op) -> Iterator:
        yield from self._head(op, WOTF)
        for value in op.data:
            yield from self._send(op, value, 8)
