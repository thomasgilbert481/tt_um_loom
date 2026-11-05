"""MIL-STD-1553B words, bus, timing checker and BC / RT models.

Written from MIL-STD-1553B Notice 4 (the public text on the DoD ASSIST
site), never from Loom's firmware. Paragraphs, as numbered there:

* 4.3.3.3.2 data code: Manchester II bi-phase, a 1 is a positive half then
  a negative half, a 0 the inverse, a zero crossing mid-bit;
* 4.3.3.3.3 bit rate 1.0 Mbit/s, long-term stability +-0.1 %;
* 4.3.3.4 word size 20 bit times (3 sync, 16 data, 1 parity);
* 4.3.3.5.1.1 command sync (and 4.3.3.5.3.1 status sync): positive for 1.5
  bit times, then negative for 1.5; 4.3.3.5.2.1 data sync: the inverse;
* 4.3.3.5.1.2-5 command fields (RT address 5 bits, T/R, subaddress/mode 5
  bits, word count/mode code 5 bits, 0 = 32 words); 4.3.3.5.1.6 odd parity;
* 4.3.3.5.3 status word: RT address, message error (bit time 9),
  instrumentation, service request, three reserved, broadcast command
  received (15), busy, subsystem flag, dynamic bus control acceptance,
  terminal flag (19), parity;
* 4.3.3.7 intermessage gap >= 4.0 us, 4.3.3.8 response time 4.0 to 12.0 us,
  4.3.3.9 no-response time-out >= 14.0 us, each measured from the mid-bit
  zero crossing of the last bit before to the mid-sync zero crossing of the
  next word;
* 4.4.1.1 word validation (sync, Manchester, 16 bits plus parity, odd
  parity), 4.4.1.2 contiguous words;
* 4.5.2.1.1.2 transmitter output: zero crossings within +-25 ns of the ideal,
  measured from the previous zero crossing; 4.5.2.1.2.1 receiver input:
  zero-crossing deviations of +-150 ns tolerated.

*Unchecked:* the paragraph numbers and figures were written from the
standard's text as the author knew it on 2026-11-04; the PDF was not
re-read in that session. The usual secondary sources for the same figures,
the DDC "MIL-STD-1553 Designer's Guide" and Holt's transceiver data sheets
(HI-1573: the TX/TX-bar, RX/RX-bar logic interface assumed here), were not
consulted either.

The logic interface is a transceiver's: two transmit inputs (TXA high drives
the bus positive, TXB high negative, both low = not driving, both high
forbidden) and two receive outputs (RXA high while the bus is positive, RXB
while it is negative, both low while it is quiet).

Bus levels below are +1, -1 and 0 (quiet). Times are in clocks of the bench.
"""

from __future__ import annotations

import dataclasses
from typing import Dict, List, Optional, Sequence, Tuple

from tools.protomodels.bench import Drive, Lines, Model, PinSpec, pad_of

CLOCK = 50_000_000              # the bench clock, Hz (48 MHz also used)
BIT_US = 1.0                    # 4.3.3.3.3
RATE_TOL = 0.001                # 4.3.3.3.3, long term
ZC_TX_NS = 25.0                 # 4.5.2.1.1.2
ZC_RX_NS = 150.0                # 4.5.2.1.2.1
GAP_MIN_US = 4.0                # 4.3.3.7
RESP_MIN_US, RESP_MAX_US = 4.0, 12.0    # 4.3.3.8
NO_RESP_MIN_US = 14.0           # 4.3.3.9

CS, DATA = "cs", "data"         # command/status sync, data sync

# status word bits, as 16-bit values (bit time 4 is bit 15)
ME, INSTR, SREQ, BCR, BUSY, SSF, DBCA, TF = (
    1 << 10, 1 << 9, 1 << 8, 1 << 4, 1 << 3, 1 << 2, 1 << 1, 1 << 0)
BROADCAST = 31


def half_clocks(clock_hz: float = CLOCK) -> float:
    """Clocks in half a bit time."""
    return clock_hz * BIT_US / 2e6


def us(clocks: float, clock_hz: float = CLOCK) -> float:
    return clocks * 1e6 / clock_hz


def odd_parity(value: int) -> int:
    """The parity bit that makes the 17 bits' count of 1s odd (4.3.3.5.1.6)."""
    return 1 ^ (bin(value & 0xFFFF).count("1") & 1)


def command(rt: int, tr: int, sa: int, wc: int) -> int:
    """A command word (4.3.3.5.1): wc 32 is written as 0."""
    return ((rt & 31) << 11) | ((tr & 1) << 10) | ((sa & 31) << 5) | (wc & 31)


@dataclasses.dataclass(frozen=True)
class Command:
    rt: int
    tr: int
    sa: int
    wc: int                     # the field, 0..31

    @classmethod
    def of(cls, word: int) -> "Command":
        return cls(word >> 11 & 31, word >> 10 & 1, word >> 5 & 31, word & 31)

    @property
    def mode(self) -> bool:
        return self.sa in (0, 31)

    @property
    def words(self) -> int:
        """Data words in the message: the word count (0 = 32), or for a mode
        command one word for mode codes 16..31 (4.3.3.5.1.7)."""
        if self.mode:
            return 1 if self.wc >= 16 else 0
        return self.wc or 32


def status(rt: int, bits: int = 0) -> int:
    """A status word (4.3.3.5.3) for terminal ``rt`` with the given bits."""
    return ((rt & 31) << 11) | (bits & 0x7FF)


def word_halves(kind: str, value: int, parity_error: bool = False,
                violate: Optional[int] = None,
                bad_sync: bool = False) -> List[int]:
    """The 40 half-bit levels of one word (4.3.3.3.2, 4.3.3.5).

    ``violate`` = b (1..17) sends bit b with both halves equal (a Manchester
    error, 4.4.1.1); ``parity_error`` sends the even parity; ``bad_sync``
    sends a sync of one bit time per half instead of 1.5 (not a valid sync).
    """
    first = 1 if kind == CS else -1
    if bad_sync:
        sync = [first] * 2 + [-first] * 2 + [first, -first]
    else:
        sync = [first] * 3 + [-first] * 3
    bits = [(value >> (15 - i)) & 1 for i in range(16)]
    bits.append(odd_parity(value) ^ (1 if parity_error else 0))
    out = list(sync)
    for i, b in enumerate(bits, start=1):
        a = 1 if b else -1
        out += [a, a] if violate == i else [a, -a]
    return out


# ------------------------------------------------------------ the decoder
Edges = List[Tuple[int, int]]           # (cycle the level starts, level)


def level_at(edges: Edges, t: float) -> int:
    """The bus level at time ``t`` (0 before the first edge)."""
    lo, hi, lvl = 0, len(edges), 0
    while lo < hi:                      # last edge at or before t
        mid = (lo + hi) // 2
        if edges[mid][0] <= t:
            lo = mid + 1
        else:
            hi = mid
    return edges[lo - 1][1] if lo else lvl


@dataclasses.dataclass
class Word:
    """One word as read off the bus by :func:`decode`."""

    kind: str                   # CS or DATA
    value: int
    errors: List[str]           # empty: valid (4.4.1.1)
    mid_sync: int               # the mid-sync zero crossing, clock
    parity_mid: Optional[int]   # the parity bit's mid-bit crossing, clock
    crossings: List[int]        # every zero crossing of the word, in order

    @property
    def valid(self) -> bool:
        return not self.errors


def crossings_of(edges: Edges) -> List[Tuple[int, int, int]]:
    """``(time, level before, level after)`` of every change of level."""
    out, prev = [], 0
    for t, lvl in edges:
        if lvl != prev:
            out.append((t, prev, lvl))
            prev = lvl
    return out


def decode(edges: Edges, clock_hz: float = CLOCK, since: int = 0) -> List[Word]:
    """Every word on a bus level record, independent of any firmware.

    A mid-sync crossing is a +-1 to -+1 change with the level before it held
    for at least 1.25 bit times and the level after it for at least 1.25
    (Manchester data never holds a level longer than one bit). The word's
    half-bits are then read at their centres, j = 0 .. 36 half-bits after
    the mid-sync crossing (j = 0..2: the sync's second half)."""
    h = half_clocks(clock_hz)
    xs = [x for x in crossings_of(edges) if x[0] >= since]
    words = []
    for i, (t, before, after) in enumerate(xs):
        if not (before and after and before == -after):
            continue
        t_prev = xs[i - 1][0] if i else -10 ** 9
        t_next = xs[i + 1][0] if i + 1 < len(xs) else 10 ** 12
        if t - t_prev < 2.5 * h or t_next - t < 2.5 * h:
            continue
        words.append(_read_word(edges, xs, t, before, h))
    return words


def _read_word(edges: Edges, xs, m: int, before: int, h: float) -> Word:
    kind = CS if before > 0 else DATA
    errors = []
    lv = [level_at(edges, m + h * (j + 0.5)) for j in range(37)]
    if lv[0:3] != [-before] * 3:
        errors.append("sync")
    value, bits = 0, []
    for b in range(17):
        a, c = lv[3 + 2 * b], lv[4 + 2 * b]
        if a == 0 or a != -c:
            errors.append("manchester bit %d" % (b + 1))
        bits.append(1 if a > 0 else 0)
    for b in bits[:16]:
        value = (value << 1) | b
    if not any(e.startswith("manchester") for e in errors):
        if bits[16] != odd_parity(value):
            errors.append("parity")
    end = m + 37 * h
    cross = [t for t, p, n in xs if m <= t < end and p and n and p == -n]
    pmid = min(cross, key=lambda t: abs(t - (m + 36 * h))) if cross else None
    if pmid is not None and abs(pmid - (m + 36 * h)) > h / 2:
        pmid = None
    return Word(kind, value, errors, m, pmid, cross)


def messages(words: Sequence[Word], clock_hz: float = CLOCK,
             tol_clocks: float = 8) -> List[List[Word]]:
    """Group words into runs of contiguous words: each word's mid-sync 20 bit
    times after the previous one's (4.4.1.2), within ``tol_clocks``."""
    step = 40 * half_clocks(clock_hz)
    runs: List[List[Word]] = []
    for w in words:
        if runs and abs(w.mid_sync - runs[-1][-1].mid_sync - step) <= tol_clocks:
            runs[-1].append(w)
        else:
            runs.append([w])
    return runs


# ----------------------------------------------------- the timing checker
def zc_deviation_ns(word: Word, clock_hz: float = CLOCK) -> float:
    """Largest deviation of a zero-crossing interval in ``word`` from a whole
    number of half-bits, in ns: 4.5.2.1.1.2 measures each crossing from
    the previous one and allows +-25 ns at a transmitter."""
    h = half_clocks(clock_hz)
    worst = 0.0
    for a, b in zip(word.crossings, word.crossings[1:]):
        d = b - a
        worst = max(worst, abs(d - round(d / h) * h))
    return worst * 1e9 / clock_hz


def grid_deviation(word: Word, clock_hz: float = CLOCK) -> float:
    """Largest distance, in clocks, of a crossing from the ideal grid of
    half-bits anchored at the word's mid-sync crossing."""
    h = half_clocks(clock_hz)
    return max((abs(t - word.mid_sync - round((t - word.mid_sync) / h) * h)
                for t in word.crossings), default=0.0)


def response_us(last: Word, reply: Word, clock_hz: float = CLOCK) -> float:
    """4.3.3.8 / 4.3.3.9: from the last word's parity mid-bit crossing to the
    reply's mid-sync crossing."""
    assert last.parity_mid is not None, "the last word has no parity crossing"
    return us(reply.mid_sync - last.parity_mid, clock_hz)


def gap_us(last: Word, nxt: Word, clock_hz: float = CLOCK) -> float:
    """4.3.3.7: the same measure between two messages."""
    return response_us(last, nxt, clock_hz)


def check_word_timing(word: Word, clock_hz: float = CLOCK,
                      zc_ns: float = ZC_TX_NS) -> List[str]:
    """Faults of one transmitted word against the transmitter's figures:
    every crossing interval within ``zc_ns`` of a whole number of half-bits
    and 39 to 40 crossings' worth of word (a crossing mid-bit in all 17)."""
    faults = []
    if not word.valid:
        faults.append("invalid word: %s" % ", ".join(word.errors))
    dev = zc_deviation_ns(word, clock_hz)
    if dev > zc_ns:
        faults.append("zero crossing %.1f ns off (limit %.0f)" % (dev, zc_ns))
    return faults


def check_contiguous(run: Sequence[Word], clock_hz: float = CLOCK,
                     tol: float = RATE_TOL) -> List[str]:
    """Consecutive words of a message 20 bit times apart within the rate's
    tolerance (4.3.3.3.3, 4.4.1.2)."""
    step = 40 * half_clocks(clock_hz)
    faults = []
    for a, b in zip(run, run[1:]):
        d = b.mid_sync - a.mid_sync
        if abs(d - step) > max(step * tol, 1.0):
            faults.append("word spacing %d clocks, not %.1f" % (d, step))
    return faults


# ------------------------------------------------------------------ bus
def _log(edges: Edges, cycle: int, level: int) -> None:
    if edges[-1][1] != level:
        edges.append((cycle, level))


class Bus(Model):
    """One 1553 bus through ideal transceivers.

    ``transmitters`` are the chip's (TXA, TXB) pad pairs, ``nodes`` the
    model terminals. Each cycle the bus level is that of the one terminal
    driving (+1 TXA, -1 TXB); two at once is a collision (recorded, level
    0) and TXA with TXB both high is recorded in ``both_high``. RXA and RXB
    (``rx``) follow the level one clock later, as a registered model must.
    ``tx_logs[i]`` holds the level edges of transmitter i alone, ``log``
    those of the bus.
    """

    def __init__(self, transmitters: Sequence[Tuple[PinSpec, PinSpec]] = (("OUT0", "OUT1"),),
                 rx: Tuple[PinSpec, PinSpec] = ("IN0", "IN1"),
                 nodes: Sequence["Node"] = ()) -> None:
        self.tx_pads = [(pad_of(a), pad_of(b)) for a, b in transmitters]
        self.rxa, self.rxb = pad_of(rx[0]), pad_of(rx[1])
        self.nodes = list(nodes)
        self.tx_logs: List[Edges] = [[(0, 0)] for _ in self.tx_pads]
        self.log: Edges = [(0, 0)]
        self.collisions: List[int] = []
        self.both_high: List[Tuple[int, int]] = []
        self._level = 0

    def add(self, node: "Node") -> "Node":
        self.nodes.append(node)
        node.bus = self
        return node

    def drive(self, drive: Drive, cycle: int) -> None:
        drive.set(self.rxa, 1 if self._level > 0 else 0)
        drive.set(self.rxb, 1 if self._level < 0 else 0)

    def observe(self, lines: Lines) -> None:
        c = lines.cycle
        level, active = 0, 0
        for i, (pa, pb) in enumerate(self.tx_pads):
            a, b = lines.get(pa), lines.get(pb)
            lvl = 0
            if a and b:
                self.both_high.append((c, i))
            else:
                lvl = a - b
            _log(self.tx_logs[i], c, lvl)
            if lvl:
                active, level = active + 1, lvl
        for node in self.nodes:
            lvl = node.level(c)
            if lvl:
                active, level = active + 1, lvl
        if active > 1:
            self.collisions.append(c)
            level = 0
        self._level = level
        _log(self.log, c, level)
        for node in self.nodes:
            node.hear(c, level)

    def words(self, since: int = 0, clock_hz: float = CLOCK) -> List[Word]:
        return decode(self.log, clock_hz, since)

    def tx_words(self, i: int = 0, since: int = 0,
                 clock_hz: float = CLOCK) -> List[Word]:
        return decode(self.tx_logs[i], clock_hz, since)


class Node:
    """A model terminal: plays back scheduled half-bit waveforms."""

    def __init__(self, clock_hz: float = CLOCK) -> None:
        self.clock_hz = clock_hz
        self.h = half_clocks(clock_hz)
        self.bus: Optional[Bus] = None
        self.waves: List[Tuple[int, List[int]]] = []    # (start, halves)

    def play(self, start: int, halves: List[int]) -> int:
        """Schedule ``halves`` from cycle ``start``; returns the end cycle."""
        self.waves.append((start, halves))
        return start + int(round(len(halves) * self.h))

    def level(self, cycle: int) -> int:
        while self.waves:
            start, halves = self.waves[0]
            j = int((cycle - start) // self.h)
            if j < 0:
                return 0
            if j < len(halves):
                return halves[j]
            self.waves.pop(0)
        return 0

    def hear(self, cycle: int, level: int) -> None:
        """Called with the bus level of every cycle."""

    @property
    def busy(self) -> bool:
        return bool(self.waves)


def message_halves(words: Sequence[Tuple[str, int]],
                   faults: Optional[Dict[int, dict]] = None,
                   gaps: Optional[Dict[int, int]] = None) -> List[int]:
    """The half-bits of words sent back to back: ``faults[i]`` are
    :func:`word_halves` options for word i, ``gaps[i]`` quiet half-bits
    put before word i (which breaks 4.4.1.2's contiguity)."""
    out: List[int] = []
    for i, (kind, value) in enumerate(words):
        out += [0] * (gaps or {}).get(i, 0)
        out += word_halves(kind, value, **(faults or {}).get(i, {}))
    return out


class BCNode(Node):
    """A model bus controller: sends commands and data words exactly on the
    half-bit grid of its clock."""

    def send(self, start: int, cmd: int, data: Sequence[int] = (),
             faults: Optional[Dict[int, dict]] = None,
             gaps: Optional[Dict[int, int]] = None) -> int:
        """Send ``cmd`` then ``data`` from ``start``; returns the cycle of the
        last parity bit's mid-bit crossing."""
        words = [(CS, cmd)] + [(DATA, d) for d in data]
        halves = message_halves(words, faults, gaps)
        self.play(start, halves)
        return start + int(round((len(halves) - 1) * self.h))


class RTNode(Node):
    """A model remote terminal at address ``rt``.

    It decodes the bus once it has been quiet for 1.5 us, takes a valid
    command for ``rt`` (or broadcast) with exactly its data words and
    answers after ``response_us`` (4.3.3.8: mid-bit of the last parity bit
    to the status mid-sync): the status word ``status(rt, bits)``, then for
    a transmit command its words from ``tx_words``. An invalid or missing
    data word sets ME with no answer (4.4.3.6). ``fault`` (one message) is
    ``"silent"``, ``{"word": i, ...}`` (``word_halves`` options for reply
    word i) or ``{"rt": a}`` (answer with another address).
    ``received`` lists ``(command, data words, errors)``.
    """

    def __init__(self, rt: int, clock_hz: float = CLOCK,
                 response_us: float = 8.0, tx_words: Sequence[int] = (),
                 bits: int = 0) -> None:
        super().__init__(clock_hz)
        self.rt, self.response_us = rt, response_us
        self.tx_words, self.bits = list(tx_words), bits
        self.fault = None
        self.received: List[Tuple[int, List[int], List[str]]] = []
        self.replies: List[int] = []            # mid-sync cycles of replies
        self._seg: Optional[int] = None
        self._last = 0
        self._own: List[Tuple[int, int]] = []

    def hear(self, cycle: int, level: int) -> None:
        if level:
            if self._seg is None:
                self._seg = cycle
            self._last = cycle
        elif self._seg is not None and cycle - self._last > 3 * self.h:
            seg, self._seg = self._seg, None
            if not any(a <= seg <= b for a, b in self._own):
                self._take(seg)

    def _take(self, seg: int) -> None:
        runs = messages(decode(self.bus.log, self.clock_hz, seg - 1), self.clock_hz)
        if not runs or runs[0][0].kind != CS or not runs[0][0].valid:
            return                              # 4.4.3.3: no response
        run = runs[0]
        cmd = Command.of(run[0].value)
        if cmd.rt not in (self.rt, BROADCAST):
            return
        data = [w.value for w in run[1:]]
        errors = [e for w in run[1:] for e in w.errors]
        want = cmd.words if not cmd.tr else 0
        if errors or len(run) - 1 != want or len(runs) > 1:
            errors = errors or ["word count %d, not %d" % (len(run) - 1, want)]
            self.bits |= ME
        self.received.append((run[0].value, data, errors))
        if errors or cmd.rt == BROADCAST:
            if cmd.rt == BROADCAST and not errors:
                self.bits |= BCR
            return
        fault, self.fault = self.fault, None
        if fault == "silent":
            return
        rt = fault.get("rt", self.rt) if isinstance(fault, dict) else self.rt
        words = [(CS, status(rt, self.bits))]
        if cmd.tr:
            words += [(DATA, w) for w in (self.tx_words + [0] * 32)[:cmd.words]]
        faults = {fault["word"]: {k: v for k, v in fault.items() if k != "word"}} \
            if isinstance(fault, dict) and "word" in fault else None
        mid = run[-1].parity_mid + self.response_us * self.clock_hz / 1e6
        start = int(round(mid - 3 * self.h))
        end = self.play(start, message_halves(words, faults))
        self._own.append((start, end))
        self.replies.append(int(round(mid)))
        self.bits &= ~(ME | BCR)                # 4.3.3.5.4
