"""CPPM (combined PPM, "PPM sum"): encoder, decoder, timing checker and
error injection, clock-exact on the bench.

Written from public secondary sources, not from the firmware; CPPM has no
formal standard. Every figure below names its source; all were read
2026-11-07 and none was checked against a transmitter on the bench:

* [EdgeTX] (open-source transmitter firmware) ``radio/src/pulses/ppm.cpp``:
  each channel a pulse train of fixed "stop" (separator) pulses, the rest
  after the last pulse at least 3 ms (``PPM_SAFE_MARGIN * 2`` ticks of a
  2 MHz timer); ``pulses.h``: ``PPM_DEF_PERIOD`` 22.5 ms; ``myeeprom.h``:
  separator ``delay * 50 + 300`` us (300 us by default, 50 us steps);
  ``edgetx.h``: ``PPM_CENTER`` 1500 us, range +-512 us; polarity selectable.
* [BF] Betaflight ``src/main/drivers/rx/rx_pwm.c`` (a decoder): rising-edge
  input capture, channel = time between rising edges, sync 2700 us or more,
  channel valid 750 .. 2250 us, at least 4 channels.
* [AP] ArduPilot ``libraries/AP_RCProtocol/AP_RCProtocol_PPMSum.cpp``: sync
  2700 us or more, channel valid over 700 and under 2300 us.

A frame of ``N`` channels is ``N + 1`` separator pulses of width ``sep`` at
the active level; channel ``i`` is the time from the start of separator
``i`` to the start of separator ``i + 1``, and the time from separator
``N + 1`` to the next frame's first is the sync gap. Polarity ``idle`` is
the line's level between pulses (0: separators high).

Times are in ticks (the protocol's microsecond) unless they say clocks; a
bench converts with ``tick_clocks``. Edges are ``(cycle, level)``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

from tools.protomodels.bench import Drive, Lines, Model, PinSpec, pad_of

SYNC_MIN_DECODE = 2700          # [BF], [AP]: an interval this long is a sync
CH_MIN, CH_MAX = 750, 2250      # [BF] valid channel window
AP_CH_MIN, AP_CH_MAX = 700, 2300  # [AP], exclusive at both ends
REST_MIN = 3000                 # [EdgeTX] rest after the last pulse
SEP_DEFAULT = 300               # [EdgeTX] separator
FRAME_DEFAULT = 22500           # [EdgeTX] PPM_DEF_PERIOD
CENTER = 1500                   # [EdgeTX] PPM_CENTER

Edge = Tuple[int, int]


# ------------------------------------------------------------------ encoder
def frame_pulses(channels: Sequence[int], *, sep: int = SEP_DEFAULT,
                 frame: int = FRAME_DEFAULT, rest_min: int = REST_MIN,
                 sync: Optional[int] = None) -> List[Tuple[int, int]]:
    """One frame as ``[(start, width)]`` of its separator pulses, in ticks
    from the frame's first separator, and the frame's length as the last
    pair's ``start`` (a zero-width marker). The sync gap is ``frame`` less
    the channels, at least ``rest_min``; ``sync`` forces it (error
    injection: a gap under 2700 is a missing sync)."""
    starts = [0]
    for ch in channels:
        starts.append(starts[-1] + ch)
    gap = sync if sync is not None else max(frame - sum(channels), rest_min)
    pulses = [(s, sep) for s in starts]
    pulses.append((starts[-1] + gap, 0))
    return pulses


def edges(frames: Sequence[Sequence[int]], *, start: int, tick_clocks: int,
          idle: int = 0, sep: int = SEP_DEFAULT, frame: int = FRAME_DEFAULT,
          rest_min: int = REST_MIN, syncs: Optional[Sequence[Optional[int]]] = None
          ) -> Tuple[List[Edge], int]:
    """The edges of ``frames`` back to back from cycle ``start``, and the
    cycle the last frame ends (its sync gap included)."""
    out: List[Edge] = []
    t = start
    for k, chans in enumerate(frames):
        sync = syncs[k] if syncs is not None else None
        pulses = frame_pulses(chans, sep=sep, frame=frame, rest_min=rest_min, sync=sync)
        for s, w in pulses[:-1]:
            out.append((t + s * tick_clocks, idle ^ 1))
            out.append((t + (s + w) * tick_clocks, idle))
        t += pulses[-1][0] * tick_clocks
    return out, t


def glitch(edge_list: Sequence[Edge], at: int, width: int, idle: int = 0) -> List[Edge]:
    """Error injection: a pulse of ``width`` clocks at the active level at
    cycle ``at`` (between two pulses), added to ``edge_list``."""
    return sorted(list(edge_list) + [(at, idle ^ 1), (at + width, idle)])


class CppmSource(Model):
    """Drives ``pin`` (a ``ui`` pad) from a schedule of edges; ``idle``
    before the first. More edges may be appended with :meth:`schedule`."""

    def __init__(self, pin: PinSpec = "IN0", idle: int = 0) -> None:
        self.pad = pad_of(pin)
        self.level = idle
        self.events: List[Edge] = []
        self._next = 0

    def schedule(self, more: Sequence[Edge]) -> None:
        self.events = self.events[self._next:] + sorted(more)
        self._next = 0

    def drive(self, drive: Drive, cycle: int) -> None:
        ev = self.events
        while self._next < len(ev) and ev[self._next][0] <= cycle:
            self.level = ev[self._next][1]
            self._next += 1
        drive.set(self.pad, self.level)


# ------------------------------------------------------------------ decoder
class EdgeWatch(Model):
    """Records every change of ``pin`` (any pad, a chip output included) as
    ``(cycle, level)``; the cycle is the one the new level was first seen."""

    def __init__(self, pin: PinSpec = "OUT0") -> None:
        self.pad = pad_of(pin)
        self.level: Optional[int] = None
        self.edges: List[Edge] = []

    def observe(self, lines: Lines) -> None:
        v = lines.get(self.pad)
        if v != self.level:
            if self.level is not None:
                self.edges.append((lines.cycle, v))
            self.level = v


@dataclass
class Frame:
    """A decoded frame: its channels in ticks, the sync gap after it, and the
    cycle of the rising edge that ended that gap."""
    channels: List[int]
    sync: int
    end: int
    out_of_range: List[int] = field(default_factory=list)   # indexes, [BF] window
    nosync: bool = False        # more than max_channels with no sync gap: void


def ticks(clocks: int, tick_clocks: int) -> int:
    """Clocks to the nearest tick."""
    return (clocks + tick_clocks // 2) // tick_clocks


def decode(edge_list: Sequence[Edge], tick_clocks: int, *,
           sync_min: int = SYNC_MIN_DECODE, max_channels: int = 8) -> List[Frame]:
    """[BF]'s decoder: intervals between rising edges; one of ``sync_min``
    ticks or more ends a frame. Channels before the first sync gap (a
    partial frame) are not reported; a frame is reported when the sync gap
    after it ends, at the next rising edge. A channel past ``max_channels``
    (Betaflight's limit is its capture count) means the sync gap was
    missed: the frame so far is reported void (``nosync``, at that edge)
    and nothing more until the next sync gap."""
    rises = [c for c, v in edge_list if v == 1]
    frames: List[Frame] = []
    current: Optional[List[int]] = None        # None: no sync gap seen yet
    for a, b in zip(rises, rises[1:]):
        w = ticks(b - a, tick_clocks)
        if w >= sync_min:
            if current is not None:
                frames.append(Frame(current, w, b, _bad(current)))
            current = []
        elif current is not None:
            if len(current) == max_channels:
                frames.append(Frame(current, 0, b, _bad(current), nosync=True))
                current = None
            else:
                current.append(w)
    return frames


def _bad(channels: Sequence[int]) -> List[int]:
    return [i for i, c in enumerate(channels) if not CH_MIN <= c <= CH_MAX]


def check_timing(edge_list: Sequence[Edge], tick_clocks: int, *, sep: int,
                 idle: int, tol: int = 0) -> List[str]:
    """Every pulse at the active level is ``sep`` ticks to within ``tol``
    clocks; every interval between pulse starts is a valid channel ([BF]
    window) or a sync gap of at least [EdgeTX]'s 3 ms rest; edges alternate.
    Returns the violations (empty: the train meets the sources)."""
    bad: List[str] = []
    active = idle ^ 1
    for (c0, v0), (c1, v1) in zip(edge_list, edge_list[1:]):
        if v0 == v1:
            bad.append("cycle %d: two edges to %d" % (c1, v1))
        elif v0 == active and abs((c1 - c0) - sep * tick_clocks) > tol:
            bad.append("cycle %d: separator of %d clocks, not %d" % (c0, c1 - c0, sep * tick_clocks))
    starts = [c for c, v in edge_list if v == active]
    for a, b in zip(starts, starts[1:]):
        w = (b - a) / tick_clocks
        if not (CH_MIN * tick_clocks - tol <= b - a <= CH_MAX * tick_clocks + tol
                or b - a >= REST_MIN * tick_clocks - tol):
            bad.append("cycle %d: interval of %.2f ticks is neither a channel nor a sync" % (a, w))
    return bad


def frame_starts(edge_list: Sequence[Edge], tick_clocks: int, idle: int,
                 sync_min: int = SYNC_MIN_DECODE) -> List[int]:
    """The cycle of every frame's first separator start (after a sync gap)."""
    starts = [c for c, v in edge_list if v == idle ^ 1]
    return [b for a, b in zip(starts, starts[1:]) if b - a >= sync_min * tick_clocks]


# ------------------------------------------------- the firmware's host words
def channel_word(index: int, width: int) -> int:
    """cppm_rx.loom's channel word, from its header's format."""
    return (index & 7) << 12 | min(width, 0xFFF)


def status_word(kind: int, count: int, mask: int = 0, dropped: bool = False) -> int:
    """cppm_rx.loom's status word, from its header's format."""
    return 0x8000 | (0x4000 if dropped else 0) | (kind & 3) << 12 | (count & 15) << 8 | (mask & 0xFF)


def split_reports(words: Sequence[int]) -> List[Tuple[List[Tuple[int, int]], int]]:
    """Group the decoder's OUTQ words into ``[(channels, status)]``, the
    channels as ``(index, width)``."""
    out: List[Tuple[List[Tuple[int, int]], int]] = []
    chans: List[Tuple[int, int]] = []
    for w in words:
        if w & 0x8000:
            out.append((chans, w))
            chans = []
        else:
            chans.append(((w >> 12) & 7, w & 0xFFF))
    return out
