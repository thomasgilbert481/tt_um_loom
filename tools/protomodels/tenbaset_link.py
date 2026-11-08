"""10BASE-T link pulses: normal link pulses (NLP), fast link pulse (FLP)
bursts and the base link code word, plus the link integrity test.

Written from the protocol's description, not from the firmware
(``firmware/tenbaset_link.loom``, ``firmware/tenbaset_link_rx.loom``).

The line. A link pulse is one positive pulse on the transmit pair; between
pulses the pair is idle (0 V). At logic level the chip drives TD+ and TD-:
a pulse is TD+ high with TD- low, idle is both low. The receive side is the
squelch comparator's output RD: high while a positive pulse is on the pair.

Sources and what was checked. IEEE 802.3 is free through the IEEE GET
program, but neither ieeexplore (HTTP 418 to the fetch tool) nor
standards.ieee.org (purchase only for the base standard) could be read in
this session, nor an archived copy. So **no figure here was checked against
the standard itself.** Where a figure was read, it was read in the secondary
source Wikipedia, "Autonegotiation" (revision of 2026-08-11), which cites
IEEE 802.3-2012 clauses 14 and 28; the rest is **from memory, unchecked**:

* Clause 14 (10BASE-T), link test pulse:
  - width 100 ns nominal, 200 ns maximum, unipolar positive (Wikipedia):
    :data:`PULSE_NS`, :data:`PULSE_MAX_NS`;
  - one every 16 ms +- 8 ms while no data is sent (Wikipedia):
    :data:`NLP_PERIOD_US`;
  - link integrity test (the state diagram of 14.2.1.7 as Wikipedia cites
    it; the timer values are **from memory, unchecked**):
    link_test_min_timer 2-7 ms, link_test_max_timer 25-150 ms,
    link_loss_timer 50-150 ms, lc_max (consecutive good pulses for link
    pass) 2-10. Wikipedia gives only "50-150 ms" for link failure.
    :class:`LinkIntegrity` implements the diagram as remembered: see there.
* Clause 28 (auto-negotiation), FLP burst:
  - 17 clock pulses 125 us +- 14 us apart (Wikipedia: "17 NLP at a 125 us
    time interval with a tolerance of 14 us"): :data:`FLP_CLOCK_US`;
  - a data pulse 62.5 us after a clock pulse for a 1 (Wikipedia); the
    +- 7 us window (55.5-69.5 us) is **from memory, unchecked**:
    :data:`FLP_DATA_US`;
  - bursts 16 ms +- 8 ms apart, as NLPs (Wikipedia: "the same 16 ms"):
    :data:`FLP_PERIOD_US`;
  - bit 0 of the code word is the first data position (Wikipedia);
  - base page: bits 4:0 the selector, 12:5 the technology ability field
    A0..A7 (A0 10BASE-T, A1 10BASE-T full duplex, A2 100BASE-TX, A3
    100BASE-TX full duplex, A4 100BASE-T4, A5 pause, A6 asymmetric pause),
    13 remote fault, 14 acknowledge, 15 next page (Wikipedia). The 802.3
    selector value 00001 is **from memory, unchecked**: :func:`base_page`;
  - a receiver groups pulses into a burst while they come closer than
    flp_test_max_timer, 165-185 us (**from memory, unchecked**); this model
    ends a group after :data:`GROUP_US` = 190 us without a pulse;
  - ability match needs three identical code words in a row (Wikipedia):
    the host's job, not the firmware's (``FW_README.md``).

Time is in core clocks, as everywhere in ``tools/protomodels``: the models
take ``clock_hz`` and convert the microsecond windows.
"""

from __future__ import annotations

import dataclasses
from typing import List, Optional, Sequence, Tuple

from tools.protomodels.bench import Drive, Lines, Model, PinSpec, pad_of

# ------------------------------------------------------------- the figures
PULSE_NS = 100.0                        # nominal width (Wikipedia)
PULSE_MAX_NS = 200.0                    # maximum width (Wikipedia)
NLP_PERIOD_US = (8000.0, 16000.0, 24000.0)     # min, nominal, max (Wikipedia)
FLP_PERIOD_US = (8000.0, 16000.0, 24000.0)     # as NLPs (Wikipedia)
FLP_CLOCK_US = (111.0, 125.0, 139.0)           # clock to clock (Wikipedia)
FLP_DATA_US = (55.5, 62.5, 69.5)               # clock to data (62.5 Wikipedia; window unchecked)
FLP_CLOCKS = 17
FLP_BITS = 16
GROUP_US = 190.0                        # > flp_test_max_timer 185 us (unchecked)
LINK_TEST_MIN_MS = (2.0, 7.0)           # unchecked
LINK_TEST_MAX_MS = (25.0, 150.0)        # unchecked
LINK_LOSS_MS = (50.0, 150.0)            # Wikipedia gives this range for link failure
LC_MAX = (2, 10)                        # unchecked

SELECTOR_8023 = 0b00001                 # unchecked
A0_10BASET, A1_10BASET_FD = 1 << 5, 1 << 6
A2_100TX, A3_100TX_FD, A4_100T4 = 1 << 7, 1 << 8, 1 << 9
A5_PAUSE, A6_ASM_PAUSE = 1 << 10, 1 << 11
RF, ACK, NP = 1 << 13, 1 << 14, 1 << 15


def base_page(half: bool = True, full: bool = False, rf: bool = False,
              ack: bool = False, np: bool = False, extra: int = 0) -> int:
    """The base link code word for an 802.3 port with these abilities."""
    word = SELECTOR_8023 | extra
    word |= A0_10BASET if half else 0
    word |= A1_10BASET_FD if full else 0
    word |= RF if rf else 0
    word |= ACK if ack else 0
    word |= NP if np else 0
    return word & 0xFFFF


def burst_positions(word: int) -> List[Tuple[int, bool]]:
    """The 33 positions of a burst in order: ``(index, pulse?)``; even
    indices are clock pulses (always there), odd ones data bit
    ``index // 2``, D0 first."""
    out = []
    for k in range(2 * FLP_CLOCKS - 1):
        out.append((k, True if k % 2 == 0 else bool((word >> (k // 2)) & 1)))
    return out


class Wire(Model):
    """A board wire from a chip output to a chip input, one clock late as a
    registered model is: here TD+ through magnetics and a cable into the
    partner's squelch comparator, folded to logic level."""

    def __init__(self, src: PinSpec, dst: PinSpec) -> None:
        self.src, self.dst = pad_of(src), pad_of(dst)
        self.level = 0

    def drive(self, drive: Drive, cycle: int) -> None:
        drive.set(self.dst, self.level)

    def observe(self, lines: Lines) -> None:
        self.level = lines.get(self.src)


# ------------------------------------------------------------ the checker
@dataclasses.dataclass
class Pulse:
    rise: int                           # clocks
    fall: int

    @property
    def width(self) -> int:
        return self.fall - self.rise


@dataclasses.dataclass
class Event:
    """One pulse group: an NLP (one pulse) or an FLP burst."""
    kind: str                           # "nlp" or "flp"
    pulses: List[Pulse]
    word: Optional[int] = None          # the code word of a burst
    clocks: List[Pulse] = dataclasses.field(default_factory=list)

    @property
    def start(self) -> int:
        return self.pulses[0].rise


class PulseMonitor(Model):
    """Watches TD+ and TD- (or RD alone, ``tdn=None``), cuts the pulses
    into NLPs and FLP bursts and checks every figure of the module docstring
    it can: width, TD- low during a pulse, every burst position inside its
    window, 17 clock pulses, and (``check_period``) the NLP and burst
    periods. Violations are kept, never raised, so a test sees them all."""

    def __init__(self, tdp: PinSpec, tdn: Optional[PinSpec] = None, *,
                 clock_hz: float = 50e6, check_period: bool = True) -> None:
        self.tdp = pad_of(tdp)
        self.tdn = pad_of(tdn) if tdn is not None else None
        self.cpu = clock_hz / 1e6        # clocks per microsecond
        self.check_period = check_period
        self.pulses: List[Pulse] = []
        self.events: List[Event] = []
        self.violations: List[str] = []
        self.tdn_high = 0               # cycles TD- was high
        self._group: List[Pulse] = []
        self._level = 0
        self._rise = 0
        self._cycle = -1

    def us(self, clocks: float) -> float:
        return clocks / self.cpu

    def observe(self, lines: Lines) -> None:
        self._cycle = c = lines.cycle
        level = lines.get(self.tdp)
        if self.tdn is not None and lines.get(self.tdn):
            self.tdn_high += 1
            if level:
                self.violations.append("TD- high during a pulse at %d" % c)
        if level and not self._level:
            if self._group and self.us(c - self._group[-1].rise) > GROUP_US:
                self._close()
            self._rise = c
        elif self._level and not level:
            p = Pulse(self._rise, c)
            if p.width * 1e3 / self.cpu > PULSE_MAX_NS:
                self.violations.append("pulse at %d is %d clocks wide" % (p.rise, p.width))
            self.pulses.append(p)
            self._group.append(p)
        self._level = level

    def flush(self) -> List[Event]:
        """Close the group in progress if it has ended; the events so far."""
        if self._group and self.us(self._cycle - self._group[-1].rise) > GROUP_US:
            self._close()
        return self.events

    def _close(self) -> None:
        group, self._group = self._group, []
        ev = Event("nlp", group) if len(group) == 1 else self._decode(group)
        if self.check_period and self.events and self.events[-1].kind == ev.kind:
            lo, _, hi = NLP_PERIOD_US if ev.kind == "nlp" else FLP_PERIOD_US
            dt = self.us(ev.start - self.events[-1].start)
            if not lo <= dt <= hi:
                self.violations.append("%s period %.1f us" % (ev.kind, dt))
        self.events.append(ev)

    def _decode(self, group: Sequence[Pulse]) -> Event:
        """Clause 28 as quoted: the first pulse is a clock; each later pulse
        is a data 1 if it falls in the data window after the last clock, the
        next clock if it falls in the clock window, else a violation."""
        clocks, word, bit, seen = [group[0]], 0, 0, False
        for p in group[1:]:
            dt = self.us(p.rise - clocks[-1].rise)
            if FLP_DATA_US[0] <= dt <= FLP_DATA_US[2] and not seen:
                word |= 1 << bit
                seen = True
            elif FLP_CLOCK_US[0] <= dt <= FLP_CLOCK_US[2]:
                clocks.append(p)
                bit, seen = bit + 1, False
            else:
                self.violations.append("burst pulse at %d is %.2f us after a clock" % (p.rise, dt))
        if len(clocks) != FLP_CLOCKS:
            self.violations.append("burst at %d has %d clock pulses" % (group[0].rise, len(clocks)))
        return Event("flp", list(group), word & 0xFFFF, clocks)

    def report(self) -> str:
        return "%d pulses, %d events, violations %s" % (
            len(self.pulses), len(self.events), self.violations[:6])


# ------------------------------------------------------------ the source
class LinkPulseSource(Model):
    """A link partner's transmitter folded to RD: drives ``pin`` high for
    each scheduled pulse. Nominal timing unless told otherwise: pulses
    :data:`PULSE_NS` wide, burst positions half of :data:`FLP_CLOCK_US`
    apart (62.5 us, 3125 clocks at 50 MHz)."""

    def __init__(self, pin: PinSpec, *, clock_hz: float = 50e6) -> None:
        self.pin = pad_of(pin)
        self.cpu = clock_hz / 1e6
        self.width = int(round(PULSE_NS * self.cpu / 1e3))
        self.spans: List[Tuple[int, int]] = []        # [rise, fall) in clocks
        self._k = 0

    def clocks(self, us: float) -> int:
        return int(round(us * self.cpu))

    def nlp(self, at: int, width: Optional[int] = None) -> int:
        """One NLP rising at clock ``at``; returns ``at``."""
        self.spans.append((at, at + (width or self.width)))
        self.spans.sort()
        return at

    def burst(self, at: int, word: int, position_us: float = FLP_CLOCK_US[1] / 2,
              data_us: Optional[float] = None) -> int:
        """An FLP burst for ``word`` whose first clock rises at ``at``;
        ``data_us`` moves the data pulses off the half-way point. Returns
        the clock after the last pulse."""
        end = at
        for k, present in burst_positions(word):
            t = at + self.clocks((k // 2) * 2 * position_us)
            if k % 2:
                t += self.clocks(data_us if data_us is not None else position_us)
            if present:
                self.nlp(t)
                end = t + self.width
        return end

    def drive(self, drive: Drive, cycle: int) -> None:
        spans = self.spans
        while self._k < len(spans) and spans[self._k][1] <= cycle:
            self._k += 1
        high = self._k < len(spans) and spans[self._k][0] <= cycle
        drive.set(self.pin, 1 if high else 0)


# -------------------------------------------------- the link integrity test
class LinkIntegrity:
    """Clause 14's link integrity test for NLPs, as remembered (unchecked),
    on event times in microseconds.

    Link test fail: every received link test pulse restarts
    link_test_min_timer and link_test_max_timer. A pulse that arrives
    before link_test_min_timer is done sets link_count to 0; otherwise it
    adds one. link_test_max_timer expiring sets link_count to 0. When
    link_count reaches lc_max the link passes. Link test pass: every pulse
    restarts link_loss_timer; its expiry fails the link (link_count 0).

    A pulse group of an FLP burst is a run of link test pulses closer than
    any link_test_min_timer: in fail it sets link_count to 0, in pass it
    restarts link_loss_timer like any pulse. So a 10BASE-T port that does
    not auto-negotiate never passes on bursts. ``t_max`` serves as both
    link_test_max_timer and link_loss_timer (their ranges overlap).
    Returns ``[(time_us, "pass" | "fail")]``, the loss times included up to
    ``until``."""

    def __init__(self, t_min_us: float, t_max_us: float, lc_max: int) -> None:
        self.t_min, self.t_max, self.lc_max = t_min_us, t_max_us, lc_max

    def run(self, events: Sequence[Tuple[float, str, float]],
            until: float) -> List[Tuple[float, str]]:
        """``events``: ``(first pulse us, kind, last pulse us)``."""
        out: List[Tuple[float, str]] = []
        state, count, last = "fail", 0, None
        for first, kind, end in sorted(events):
            if last is not None and first - last > self.t_max:
                if state == "pass":
                    out.append((last + self.t_max, "fail"))
                    state = "fail"
                count = 0
            if state == "fail":
                if kind == "flp" or (last is not None and first - last < self.t_min):
                    count = 0
                else:
                    count += 1
                if count >= self.lc_max:
                    state = "pass"
                    out.append((first, "pass"))
            last = end
        if state == "pass" and last is not None and until - last > self.t_max:
            out.append((last + self.t_max, "fail"))
        return out
