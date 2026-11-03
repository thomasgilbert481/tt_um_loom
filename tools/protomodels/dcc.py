"""NMRA DCC reference model: packets, a half-bit encoder, a checking decoder.

Written from the NMRA standards, never from ``firmware/dcc_*.loom``:

* **S-9.1**: "Electrical Standards for Digital Command Control", the
  version posted on nmra.org (marked DRAFT, dated 2026-06-16; fetched
  2026-11-02). Section 2 (Technique for Encoding Bits): a bit is two parts
  between alternate transitions of a bipolar square wave; timing is measured
  between zero crossings (note 4); a decoder cannot know which part is
  positive (note 3). Section 2.1: stretched 0s, one half only, optional for
  a command station and off by default; every decoder must tolerate them.
  Table 2 (section 2.2, One Bit Timing): a 1 whose part A is outside
  t1min .. t1max, or whose parts differ by more than t1d, is bad; a decoder
  must accept a 1 whose part is inside tr1 and whose parts differ by up to
  tr1d. **Table 2.1** (DCC Bit Timing), in us, min / nominal / max:

  ======== ====================================== ===== ===== =====
  t1       a 1 half, transmitted                  55    58    61
  tr1      a 1 half, received                     52    58    64
  t0       a 0 half, transmitted, not stretched   95    100   (*)
  t0s      a stretched 0 half, transmitted        95          9900
  t0total  a whole stretched 0                                12000
  tr0      a 0 half, received                     90    100   10000
  t1d      halves of a transmitted 1 differ                   3
  tr1d     halves of a received 1 differ                      6
  t0d      halves of a transmitted unstretched 0              3
  ======== ====================================== ===== ===== =====

  (*) the text layer of the posted PDF reads "1 23 5" in that cell, which
  is not a number we can trust: **unchecked**, and not used. The command
  station here sends 100 us halves, inside every reading.
* **S-9.2**: "Communications Standards for Digital Command Control",
  adopted July 2004. Section A (General Packet Format): a preamble of 1s,
  of which a command station sends at least 14 and a decoder must not
  accept fewer than 10 complete or require more than 12; a 0 packet start
  bit; bytes MSB first, the first the address; a 0 data byte start bit
  before every further byte; a 1 packet end bit, which may count as a
  preamble bit of the next packet (note 3), after which the bitstream goes
  on for at least 26 us. Section B (Baseline Packets): speed and direction
  ``0AAAAAAA 01DCSSSS EEEEEEEE`` with the speed table of figure 2, the
  error-detection byte the XOR of the others; the reset packet ``00 00 00``;
  the idle packet ``FF 00 FF``; the broadcast stop ``00 01DC000S``.
  Section C: packets repeated as often as possible; a decoder that meets a
  bad start bit, end bit or error byte takes the next valid preamble as a
  new packet.

The models are :class:`~tools.protomodels.bench.Model` s clocked by the
core clock; ``clock_mhz`` turns microseconds into clocks.
"""

from __future__ import annotations

import dataclasses
from typing import Dict, List, Optional, Sequence

from tools.protomodels.bench import Drive, Lines, Model, PinSpec, pad_of

# ------------------------------------------------- S-9.1 Table 2.1, in us
T1_MIN, T1_NOM, T1_MAX = 55.0, 58.0, 61.0
TR1_MIN, TR1_MAX = 52.0, 64.0
T0_MIN, T0_NOM = 95.0, 100.0
T0S_MAX = 9900.0
T0_TOTAL_MAX = 12000.0
TR0_MIN, TR0_MAX = 90.0, 10000.0
T1D_MAX, TR1D_MAX, T0D_MAX = 3.0, 6.0, 3.0

# ---------------------------------------------------- S-9.2 section A
PREAMBLE_TX_MIN = 14          # a command station sends at least these 1s
PREAMBLE_RX_MIN = 10          # a decoder must not accept fewer
PREAMBLE_RX_REQ = 12          # nor require more

IDLE = (0xFF, 0x00)           # S-9.2 section B, without the error byte
RESET = (0x00, 0x00)


def error_byte(data: Sequence[int]) -> int:
    """S-9.2 section B: the XOR of the bytes before it."""
    e = 0
    for b in data:
        e ^= b & 0xFF
    return e


def speed_byte(forward: bool, step: Optional[int]) -> int:
    """The instruction byte ``01DCSSSS`` for a 28-step speed (S-9.2 figure
    2): ``step`` 0 is stop, ``None`` the emergency stop, 1 .. 28 the steps.
    The table's columns are C S3 S2 S1 S0: stop 0 0000, E-stop 0 0001, and
    step s has S3..S0 = (s + 3) >> 1 and C = (s + 3) & 1 (step 1 0 0010,
    step 2 1 0010, step 28 1 1111): C is the speed's least significant bit
    (section B, byte two)."""
    if step is None:
        c, s = 0, 1                           # 00001 E-Stop
    elif step == 0:
        c, s = 0, 0                           # 00000 Stop
    elif 1 <= step <= 28:
        c, s = (step + 3) & 1, (step + 3) >> 1
    else:
        raise ValueError("speed step %r outside 0 .. 28" % (step,))
    return 0x40 | (0x20 if forward else 0) | (c << 4) | s


def speed_packet(address: int, forward: bool, step: Optional[int]) -> List[int]:
    """The baseline speed and direction packet, without its error byte."""
    if not 1 <= address <= 127:
        raise ValueError("baseline address %r outside 1 .. 127" % (address,))
    return [address, speed_byte(forward, step)]


def broadcast_stop(forward: bool = True, cut_power: bool = False) -> List[int]:
    """S-9.2 section B: ``00000000 01DC000S``; S = 1 stops delivering
    energy to the motor. C (bit 4) left 0: direction not ignored."""
    return [0x00, 0x40 | (0x20 if forward else 0) | (1 if cut_power else 0)]


def packet_bits(data: Sequence[int], *, preamble: int = PREAMBLE_TX_MIN,
                bad_error_byte: bool = False) -> List[int]:
    """The bits of one packet (S-9.2 section A): ``preamble`` 1s, the start
    bit, every byte (the error byte appended, inverted if
    ``bad_error_byte``) MSB first with a 0 before each further byte, and
    the 1 end bit."""
    err = error_byte(data) ^ (0xFF if bad_error_byte else 0)
    out = [1] * preamble
    for k, b in enumerate(list(data) + [err]):
        out.append(0)                         # packet start / data byte start
        out += [(b >> (7 - j)) & 1 for j in range(8)]
    out.append(1)                             # packet end bit
    return out


def halves_of(bits: Sequence[int], *, t1: float = T1_NOM, t0: float = T0_NOM,
              t1b: Optional[float] = None,
              stretch: Optional[Dict[int, float]] = None,
              override: Optional[Dict[int, float]] = None) -> List[float]:
    """Half-bit durations in us for ``bits``.

    ``t1`` and ``t0`` are the halves of a 1 and a 0; ``t1b``, if given, is
    the second half of every 1 (an asymmetric 1, Table 2's Period B).
    ``stretch`` maps a bit index (a 0) to the length of its first half,
    the one S-9.1 section 2.1 lets a command station stretch. ``override``
    maps a half index to a length: the error injection of a half outside
    every window."""
    out: List[float] = []
    for i, b in enumerate(bits):
        if b:
            out += [t1, t1 if t1b is None else t1b]
        else:
            first = (stretch or {}).get(i, t0)
            out += [first, t0]
    for i, us in (override or {}).items():
        out[i] = us
    return out


class DccPlayer(Model):
    """Drives a line with half-bits, alternately high and low.

    ``queue`` takes half lengths in us; the edges are placed at the rounded
    running sum, so long streams do not drift. The first half is high.
    With the queue empty the line holds its level (a stopped stream)."""

    def __init__(self, pin: PinSpec, *, clock_mhz: float = 50.0) -> None:
        self.pad = pad_of(pin)
        self.clock_mhz = clock_mhz
        self.level = 0
        self._edges: List[int] = []          # clock of every edge to come
        self._t_us = None                    # running sum, in us
        self._now = 0

    def queue(self, halves: Sequence[float], *, start_cycle: Optional[int] = None) -> None:
        if self._t_us is None:
            begin = self._now + 4 if start_cycle is None else start_cycle
            self._t_us = begin / self.clock_mhz
            self._edges.append(begin)
        for us in halves:
            self._t_us += us
            self._edges.append(int(round(self._t_us * self.clock_mhz)))

    def queue_packets(self, packets: Sequence[Sequence[int]], **kw) -> None:
        """Every packet with the encoder's defaults or ``kw``."""
        pre = kw.pop("preamble", PREAMBLE_TX_MIN)
        for data in packets:
            self.queue(halves_of(packet_bits(data, preamble=pre), **kw))

    @property
    def idle(self) -> bool:
        return not self._edges

    def drive(self, drive: Drive, cycle: int) -> None:
        drive.set(self.pad, self.level)

    def observe(self, lines: Lines) -> None:
        self._now = lines.cycle + 1
        while self._edges and self._edges[0] <= self._now:
            self._edges.pop(0)
            self.level ^= 1


class Wire(Model):
    """The board wire from a chip output to a chip input (one clock late,
    as a registered model is): the station's OUT0 into the decoder's IN0."""

    def __init__(self, src: PinSpec, dst: PinSpec) -> None:
        self.src, self.dst = pad_of(src), pad_of(dst)
        self.level = 0

    def drive(self, drive: Drive, cycle: int) -> None:
        drive.set(self.dst, self.level)

    def observe(self, lines: Lines) -> None:
        self.level = lines.get(self.src)


# ------------------------------------------------------------ the decoder
@dataclasses.dataclass
class DccPacket:
    """A packet as received: ``data`` holds every byte, the error byte
    last; ``halves`` the clocks of each half from the start bit to the end
    bit; ``preamble`` the 1 halves in a row before the start bit."""

    data: List[int]
    start: int
    preamble: int
    halves: List[int]
    after_packet: bool

    @property
    def payload(self) -> List[int]:
        return self.data[:-1]

    @property
    def xor_ok(self) -> bool:
        return error_byte(self.data) == 0


@dataclasses.dataclass
class Violation:
    kind: str
    cycle: int
    detail: str = ""

    def __str__(self) -> str:                               # pragma: no cover
        return "%s at %d: %s" % (self.kind, self.cycle, self.detail)


class DccRx(Model):
    """Times every half-bit on ``pin`` and decodes packets as S-9.2 has it.

    Each half is classified against the receive windows of S-9.1 Table
    2.1 (tr1, tr0); a preamble is ``2 * PREAMBLE_RX_MIN`` 1 halves or more
    in a row; the start bit's two 0 halves fix the pairing; a 1 is two 1
    halves differing by tr1d or less (Table 2). Packets that fail are
    listed in :attr:`rejected` with the reason (``bit``, ``xor``,
    ``format``, ``preamble``); good ones in :attr:`packets` (idle packets
    included).

    With ``transmit=True`` the line is also held to the command-station
    figures: every half of a 1 inside t1, the two halves of a 1 within
    t1d, every 0 half 95 us or more and at most t0s, a whole 0 at most
    t0total, an unstretched 0's halves within t0d, and at least 14 full
    preamble 1s before every start bit, the end bit before it not counted.
    A bit is then a high half and the low half after it (the station
    starts every bit with a rise); breaches go to :attr:`violations`."""

    def __init__(self, pin: PinSpec, *, clock_mhz: float = 50.0,
                 transmit: bool = False) -> None:
        self.pad = pad_of(pin)
        self.clock_mhz = clock_mhz
        self.transmit = transmit
        self.packets: List[DccPacket] = []
        self.rejected: List[Violation] = []
        self.violations: List[Violation] = []
        self.halves: List[int] = []           # every half seen, clocks
        self._level: Optional[int] = None
        self._edge: Optional[int] = None
        self._run = 0                         # 1 halves in a row
        self._after = False                   # the run began at an end bit
        self._state = "hunt"
        self._pkt: List[int] = []
        self._first: Optional[int] = None     # first half of a bit, clocks
        self._bits: List[int] = []
        self._ph: List[int] = []
        self._start = 0
        self._pre = 0
        self._txhigh: Optional[int] = None    # transmit check: the high half

    def us(self, clocks: int) -> float:
        return clocks / self.clock_mhz

    def _cls(self, clocks: int) -> Optional[int]:
        u = self.us(clocks)
        if TR1_MIN <= u <= TR1_MAX:
            return 1
        if TR0_MIN <= u <= TR0_MAX:
            return 0
        return None

    def observe(self, lines: Lines) -> None:
        v = lines.get(self.pad)
        if self._level is None:
            self._level = v
            return
        if v == self._level:
            return
        c = lines.cycle
        if self._edge is not None:
            self._half(c - self._edge, self._level, c)
        self._edge, self._level = c, v

    # ---------------------------------------------------- receive side
    def _reject(self, kind: str, cycle: int, detail: str = "") -> None:
        self.rejected.append(Violation(kind, cycle, detail))
        self._state, self._run, self._after = "hunt", 0, False

    def _half(self, clocks: int, level: int, cycle: int) -> None:
        self.halves.append(clocks)
        if self.transmit:
            self._tx_half(clocks, level, cycle)
        k = self._cls(clocks)
        if self._state == "hunt":
            if k == 1:
                self._run += 1
            elif k == 0 and self._run >= 2 * PREAMBLE_RX_MIN:
                self._state, self._first = "start", clocks
                self._pre, self._start = self._run, cycle - clocks
            elif k == 0 and self._run >= 4:
                self._reject("preamble", cycle, "%d 1 halves" % self._run)
            else:
                self._run, self._after = 0, False
            return
        if self._state == "start":
            if k != 0:
                self._reject("bit", cycle, "start bit's second half")
                return
            self._state, self._pkt, self._bits = "bits", [], []
            self._ph = [self._first, clocks]
            self._first = None
            return
        self._ph.append(clocks)
        if self._first is None:
            self._first = clocks
            if k is None:
                self._reject("bit", cycle, "half of %.2f us" % self.us(clocks))
            return
        a, self._first = self._first, None
        ka = self._cls(a)
        if k is None or ka != k:
            self._reject("bit", cycle, "halves %.2f, %.2f us" % (self.us(a), self.us(clocks)))
            return
        if k == 1 and abs(self.us(a) - self.us(clocks)) > TR1D_MAX:
            self._reject("bit", cycle, "1 halves differ by %.2f us"
                         % abs(self.us(a) - self.us(clocks)))
            return
        self._bit(k, cycle)

    def _bit(self, b: int, cycle: int) -> None:
        if len(self._bits) < 8:
            self._bits.append(b)
            if len(self._bits) == 8:
                self._pkt.append(sum(x << (7 - j) for j, x in enumerate(self._bits)))
            return
        self._bits = []                       # the separator
        if b == 0:
            return
        p = DccPacket(self._pkt, self._start, self._pre, self._ph, self._after)
        self._state = "hunt"
        if len(p.data) < 3:
            self._reject("format", cycle, "%d bytes" % len(p.data))
        elif not p.xor_ok:
            self._reject("xor", cycle, " ".join("%02X" % x for x in p.data))
        else:
            self.packets.append(p)
        self._run, self._after = 2, True      # S-9.2 note 3: the end bit

    # --------------------------------------------------- transmit side
    def _viol(self, kind: str, cycle: int, detail: str) -> None:
        self.violations.append(Violation(kind, cycle, detail))

    def _tx_half(self, clocks: int, level: int, cycle: int) -> None:
        u = self.us(clocks)
        if level == 1:
            self._txhigh = clocks
            return
        if self._txhigh is None:
            return                            # the line began mid-bit
        a, b = self.us(self._txhigh), u
        self._txhigh = None
        if a < T0_MIN and b < T0_MIN:         # a 1
            for x in (a, b):
                if not T1_MIN <= x <= T1_MAX:
                    self._viol("t1", cycle, "1 half of %.3f us" % x)
            if abs(a - b) > T1D_MAX:
                self._viol("t1d", cycle, "%.3f / %.3f us" % (a, b))
        elif a >= T0_MIN and b >= T0_MIN:     # a 0
            if max(a, b) > T0S_MAX or a + b > T0_TOTAL_MAX:
                self._viol("t0", cycle, "0 of %.3f + %.3f us" % (a, b))
            if abs(a - b) > T0D_MAX and min(a, b) > T0_NOM + T0D_MAX:
                # stretched, but section 2.1 stretches one half only; the
                # other is taken as plain if within t0d of t0 nominal (t0's
                # max is unreadable in the posted table)
                self._viol("t0", cycle, "both halves long: %.3f / %.3f" % (a, b))
        else:
            self._viol("pair", cycle, "a 1 half and a 0 half: %.3f / %.3f" % (a, b))

    def check_preambles(self) -> None:
        """At least 14 full 1s before each start bit, the end bit before it
        not counted (S-9.2 section A); a transmit check."""
        for p in self.packets:
            full = (p.preamble - (2 if p.after_packet else 0)) // 2
            if full < PREAMBLE_TX_MIN:
                self._viol("preamble", p.start, "%d full 1s" % full)

    def report(self) -> str:                                # pragma: no cover
        return "%d packets, rejected %s, violations %s" % (
            len(self.packets), [str(v) for v in self.rejected[:4]],
            [str(v) for v in self.violations[:4]])


# ------------------------------------------- the firmware's report words
def parse_reports(words: Sequence[int]) -> List[List[int]]:
    """``dcc_decoder``'s OUTQ words back into packets (bytes without the
    error byte): a status word ``0x8000 | n << 8 | first byte`` and then
    ceil((n - 1) / 2) words of two bytes, the earlier in bits 15:8. This is
    the program's host format (its header), not part of the standard."""
    out, i = [], 0
    while i < len(words):
        w = words[i]
        assert w & 0x8000, "word %d (0x%04X) is not a status word" % (i, w)
        n = (w >> 8) & 0xF
        data, i = [w & 0xFF], i + 1
        while len(data) < n:
            d = words[i]
            i += 1
            data.append(d >> 8)
            if len(data) < n:
                data.append(d & 0xFF)
        out.append(data)
    return out
