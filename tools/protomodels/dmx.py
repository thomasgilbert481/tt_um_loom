"""DMX512 reference models (ANSI E1.11, USITT DMX512-A): a transmitter that
drives a chip input and a receiver that decodes and times a chip output.

Written from the standard's timing, not from the firmware, for
``firmware/dmx_controller.loom`` and ``firmware/dmx_fixture.loom`` (D-035).

The line (E1.11 section 4 and the timing diagram of section 8): mark (1) is
idle. A packet is a BREAK (the line low), a MARK AFTER BREAK (MAB, high),
then up to 513 slots: the start code (slot 0; 0 is dimmer data, the others
are alternate start codes) and 1 .. 512 data slots. A slot is an 8N2
frame at 250 kbit/s: a start bit (low), eight data bits LSB first, two stop
bits (high). Between slots the line may stay at mark (MARK TIME BETWEEN
SLOTS, MTBF) and between the last slot and the next break (MARK BEFORE
BREAK, MTBP).

The timing figures, :data:`TX` for what a transmitter must send and
:data:`RX` for what a receiver must accept, are E1.11-2008's "timing
diagram values" table (section 8.0 Table 6 in the 2008 edition as the
author recalls it; the table number is unchecked, the standard was not to
hand when this was written). Each figure was checked against at least one
secondary source quoting the standard where marked "checked":

=====================  ===========================  ==========================
figure                 transmitted                  received
=====================  ===========================  ==========================
bit time               3.92 .. 4.08 us (250 kbit/s, 3.92 .. 4.08 us
                       +-2 %) unchecked
BREAK                  >= 92 us, 176 typical, no    >= 88 us (checked: NXP
                       maximum (checked: TeensyDMX  AN3315, Zilog RD0016)
                       notes)
MAB                    >= 12 us, < 1 s (checked:    >= 8 us, < 1 s (checked:
                       TeensyDMX notes)             Zilog RD0016)
MTBF (between slots)   0 .. < 1 s (unchecked)       0 .. < 1 s (unchecked)
MTBP (before break)    0 .. < 1 s (unchecked)       0 .. < 1 s (unchecked)
BREAK to BREAK         >= 1204 us, <= 1 s           >= 1196 us, <= 1.25 s
                       (unchecked: AN1076 gives     (checked: NXP AN3315)
                       1204 us as the minimum
                       packet time)
=====================  ===========================  ==========================

Times are in microseconds in the tables and in core clocks on the bench
(:data:`CLOCK`, 50 MHz, so one bit is 200 clocks). Edges a model drives are
placed at ``round(t)`` of exact fractional times, so nothing drifts.
"""

from __future__ import annotations

import bisect
import dataclasses
import itertools
from collections import deque
from typing import Dict, List, Optional, Sequence, Tuple

from .bench import Drive, Lines, Model, PinSpec, pad_of

CLOCK = 50e6                        # the datasheet's clock, Hz
US = CLOCK / 1e6                    # clocks per microsecond
BIT_US = 4.0                        # 250 kbit/s
SLOT_BITS = 11                      # 8N2
MAX_SLOTS = 512                     # data slots after the start code
FOREVER = 1e6                       # "< 1 s", in us


@dataclasses.dataclass(frozen=True)
class Limits:
    """One column of E1.11's timing table, in microseconds (``None``: no limit)."""

    bit: Tuple[float, float]
    brk: Tuple[float, Optional[float]]
    mab: Tuple[float, Optional[float]]
    mtbf: Tuple[float, Optional[float]]
    mtbp: Tuple[float, Optional[float]]
    btb: Tuple[float, Optional[float]]


#: What a transmitter must send.
TX = Limits(bit=(3.92, 4.08), brk=(92.0, None), mab=(12.0, FOREVER),
            mtbf=(0.0, FOREVER), mtbp=(0.0, FOREVER), btb=(1204.0, FOREVER))
#: What a receiver must accept.
RX = Limits(bit=(3.92, 4.08), brk=(88.0, None), mab=(8.0, FOREVER),
            mtbf=(0.0, FOREVER), mtbp=(0.0, FOREVER), btb=(1196.0, 1.25e6))


def frame_bits(value: int, stop1: int = 1, stop2: int = 1) -> List[int]:
    """The eleven line levels of one 8N2 slot: start, data LSB first, stops."""
    return [0] + [(value >> i) & 1 for i in range(8)] + [stop1, stop2]


class DmxTx(Model):
    """Drives DMX512 packets onto a pad (a model controller on a chip input).

    Every timing can be set per transmitter and overridden per packet, so a
    test can sit at E1.11's limits or past them. Defaults are E1.11's
    transmit minimums for the marks and the typical break.

    ``send_packet`` queues one packet and returns the cycle its break
    starts. Error injection: ``break_us`` below 88 (a short break),
    ``bad_stop`` (slot indices, 0 = the start code, whose first stop bit is
    sent low: a framing error), and a short packet is simply fewer slots.
    ``mtbf_us`` may be one value or one per slot gap.
    """

    def __init__(self, pin: PinSpec, *, bit_us: float = BIT_US,
                 break_us: float = 176.0, mab_us: float = 12.0,
                 mtbf_us: float = 0.0, mtbp_us: float = 0.0,
                 idle: int = 1) -> None:
        self.pad = pad_of(pin)
        self.bit_us = bit_us
        self.break_us = break_us
        self.mab_us = mab_us
        self.mtbf_us = mtbf_us
        self.mtbp_us = mtbp_us
        self.level = idle
        self.now = 0
        self._changes: deque = deque()          # (cycle, level), in order
        self._free_at = 0.0                     # first cycle a break may start
        self.packets_sent: List[Tuple[int, int, List[int]]] = []

    def _at(self, t: float, level: int) -> None:
        self._changes.append((int(round(t)), level))

    def send_packet(self, start_code: int, slots: Sequence[int] = (), *,
                    break_us: Optional[float] = None, mab_us: Optional[float] = None,
                    mtbf_us=None, mtbp_us: Optional[float] = None,
                    bit_us: Optional[float] = None, bad_stop: Sequence[int] = (),
                    start: Optional[int] = None) -> int:
        bit = (self.bit_us if bit_us is None else bit_us) * US
        brk = (self.break_us if break_us is None else break_us) * US
        mab = (self.mab_us if mab_us is None else mab_us) * US
        gaps = self.mtbf_us if mtbf_us is None else mtbf_us
        mtbp = (self.mtbp_us if mtbp_us is None else mtbp_us) * US
        t = max(self._free_at, float(self.now + 1))
        if start is not None:
            t = max(t, float(start))
        t0 = t
        self._at(t, 0)                          # break
        t += brk
        self._at(t, 1)                          # mark after break
        t += mab
        frames = [start_code] + list(slots)
        for i, value in enumerate(frames):
            if i:
                gap = gaps[i - 1] if isinstance(gaps, (list, tuple)) else gaps
                t += gap * US
            stop1 = 0 if i in bad_stop else 1
            for k, level in enumerate(frame_bits(value, stop1)):
                self._at(t + k * bit, level)
            t += SLOT_BITS * bit
            self._at(t, 1)
        self._free_at = t + mtbp
        self.packets_sent.append((int(round(t0)), start_code, list(slots)))
        return int(round(t0))

    def send_low(self, low_us: float, mark_us: float = 12.0) -> int:
        """A lone low run (a short break, or a glitch) then mark."""
        t = max(self._free_at, float(self.now + 1))
        self._at(t, 0)
        self._at(t + low_us * US, 1)
        self._free_at = t + (low_us + mark_us) * US
        return int(round(t))

    @property
    def busy(self) -> bool:
        return bool(self._changes)

    def drive(self, drive: Drive, cycle: int) -> None:
        self.now = cycle
        changes = self._changes
        while changes and changes[0][0] <= cycle:
            self.level = changes.popleft()[1]
        drive.set(self.pad, self.level)


@dataclasses.dataclass
class DmxSlot:
    value: int
    start: int                      # cycle of the start bit's falling edge
    framing_error: bool             # a stop bit sampled low
    edges: List[Tuple[int, int]]    # (cycle, level) of every edge in the slot
    mark_before: int                # clocks of mark since the last slot's end


@dataclasses.dataclass
class DmxPacket:
    break_start: int                # cycle the break fell
    break_clocks: int
    mab_clocks: int
    slots: List[DmxSlot]            # slot 0 is the start code
    mark_after: Optional[int] = None    # MTBP: last slot's end to the next break
    btb_clocks: Optional[int] = None    # this break to the next one

    @property
    def start_code(self) -> int:
        return self.slots[0].value

    @property
    def data(self) -> List[int]:
        return [s.value for s in self.slots[1:]]


class DmxRx(Model):
    """Records a pad's edges and decodes DMX512 packets from them.

    :meth:`packets` decodes offline: a low run of at least ``RX.brk[0]``
    is a break, the high run after it the MAB, then each falling edge
    starts a slot sampled at the middle of its bits on the nominal 4 us
    grid (a +-2 % sender is still sampled well inside each bit). A low run
    longer than a slot's but shorter than a break is kept in
    :attr:`short_lows` and ends the packet. :meth:`violations` checks a
    decoded packet against a column of the timing table, every edge of
    every slot included.
    """

    def __init__(self, pin: PinSpec, *, bit_us: float = BIT_US,
                 limits: Limits = RX, idle: int = 1) -> None:
        self.pad = pad_of(pin)
        self.bit = bit_us * US
        self.limits = limits
        self.idle = idle
        # Edges count from the first cycle seen: a pad that comes out of
        # reset low is not taken for a break.
        self.prev: Optional[int] = None
        self.edges: List[Tuple[int, int]] = []
        self.short_lows: List[Tuple[int, int]] = []

    def observe(self, lines: Lines) -> None:
        v = lines.get(self.pad)
        if v != self.prev:
            if self.prev is not None:
                self.edges.append((lines.cycle, v))
            self.prev = v

    def _level(self, t: float) -> int:
        """The line level at cycle ``t`` from the edge list."""
        k = bisect.bisect_right(self._cycles, t)
        return self.edges[k - 1][1] if k else 1 - self.edges[0][1]

    def _rise_after(self, i: int) -> Optional[int]:
        """Index of the first rising edge after edge ``i``."""
        for j in range(i + 1, len(self.edges)):
            if self.edges[j][1] == 1:
                return j
        return None

    def packets(self) -> List[DmxPacket]:
        edges, bit = self.edges, self.bit
        brk_min = self.limits.brk[0] * US
        out: List[DmxPacket] = []
        self._cycles = [c for c, _ in edges]
        self.short_lows = []
        packet: Optional[DmxPacket] = None
        last_end = 0
        i = 0
        while i < len(edges):
            cycle, v = edges[i]
            if v != 0:
                i += 1
                continue
            r = self._rise_after(i)
            low = (edges[r][0] if r is not None else None)
            if low is None:
                break                               # still low at the end
            run = low - cycle
            if run >= brk_min:                      # a break
                if packet is not None:
                    packet.btb_clocks = cycle - packet.break_start
                    if packet.slots:
                        packet.mark_after = cycle - last_end
                    out.append(packet)
                nxt = r + 1 if r + 1 < len(edges) else None
                mab = (edges[nxt][0] - low) if nxt is not None else None
                packet = DmxPacket(cycle, run, mab if mab is not None else -1, [])
                i = r + 1
                continue
            if packet is None:                      # not synchronised yet
                i = r + 1
                continue
            if run > 10.5 * bit:                    # longer than any slot can be low
                self.short_lows.append((cycle, run))
                packet.btb_clocks = None
                out.append(packet)
                packet = None
                i = r + 1
                continue
            samples = [self._level(cycle + (k + 0.5) * bit) for k in range(SLOT_BITS)]
            end = cycle + SLOT_BITS * bit
            value = sum(b << k for k, b in enumerate(samples[1:9]))
            slot_edges = list(itertools.takewhile(lambda e: e[0] < end - bit / 2,
                                                  itertools.islice(edges, i, None)))
            mark = cycle - last_end if packet.slots else cycle - (packet.break_start
                                                                   + packet.break_clocks
                                                                   + packet.mab_clocks)
            packet.slots.append(DmxSlot(value, cycle, not (samples[9] and samples[10]),
                                        slot_edges, int(round(mark))))
            last_end = int(round(end))
            i += len(slot_edges)
        if packet is not None:
            out.append(packet)
        return out

    def violations(self, packet: DmxPacket, limits: Optional[Limits] = None,
                   slack: float = 0.0) -> List[str]:
        """Every way ``packet`` breaks a column of the timing table.

        Each bit edge of each slot must lie within the bit-time window of
        that slot's start edge, ``k * 3.92 .. k * 4.08`` us for the edge
        that starts bit ``k``, widened by ``slack`` clocks (the slot grid of
        a chip transmitter, say). A framing error, a slot past the 513th,
        and every mark or break outside its window are reported too.
        """
        lim = limits or self.limits
        out: List[str] = []

        def window(name: str, clocks: Optional[int], span) -> None:
            if clocks is None:
                return
            lo, hi = span
            if clocks < lo * US - slack:
                out.append("%s %.2f us < %.2f us" % (name, clocks / US, lo))
            if hi is not None and clocks > hi * US + slack:
                out.append("%s %.2f us > %.2f us" % (name, clocks / US, hi))

        window("break", packet.break_clocks, lim.brk)
        window("MAB", packet.mab_clocks, lim.mab)
        window("MTBP", packet.mark_after, lim.mtbp)
        window("break to break", packet.btb_clocks, lim.btb)
        if not packet.slots:
            out.append("no start code")
        if len(packet.slots) > MAX_SLOTS + 1:
            out.append("%d slots after the start code" % (len(packet.slots) - 1))
        lo, hi = lim.bit[0] * US, lim.bit[1] * US
        for n, slot in enumerate(packet.slots):
            if n:
                window("MTBF before slot %d" % n, slot.mark_before, lim.mtbf)
            if slot.framing_error:
                out.append("slot %d: framing error" % n)
            for cycle, _ in slot.edges[1:]:
                d = cycle - slot.start
                k = int(round(d / self.bit))
                if not k * lo - slack <= d <= k * hi + slack:
                    out.append("slot %d: edge %d clocks after its start, bit %d"
                               % (n, d, k))
        return out


def packet_clocks(slots: int, *, break_us: float = 176.0, mab_us: float = 12.0,
                  bit_us: float = BIT_US) -> int:
    """Clocks from a break's fall to the end of a packet of ``slots`` data
    slots sent back to back."""
    return int(round((break_us + mab_us + (slots + 1) * SLOT_BITS * bit_us) * US))
