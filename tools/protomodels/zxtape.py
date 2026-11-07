"""ZX Spectrum cassette signal: encoder, decoder and pin models.

Written from the format's published descriptions, never from the firmware
(``firmware/zxtape_save.loom``, ``firmware/zxtape_load.loom``). Sources,
both secondary to Sinclair's ROM itself, which was not read:

* [TZX] the TZX format specification, worldofspectrum.net/TZXformat.html,
  read 2026-11-07. Section 2: "1 T state = (1/3500000)s"; after a data
  block "the 'current pulse level' ... is the opposite of the last pulse
  level played, so that a subsequent pulse will produce an edge", and there
  should be "at least 1 ms. pause of the opposite level" before the line
  goes low. Section 5 table (standard ROM timings, T-states): pilot pulse
  2168, sync 667 and 735, bit 0 855, bit 1 1710. Block ID 10: "The pilot
  tone consists in 8063 pulses if the first data byte (flag byte) is < 128,
  3223 otherwise." Block ID 11 lists the same six values as fields.
* [ROM] "The Complete Spectrum ROM Disassembly" (Logan and O'Hara) as
  published by SkoolKit, skoolkid.github.io/rom/asm/04C2.html (SA-BYTES),
  0556.html (LD-BYTES), 05E3.html (LD-EDGE-2, LD-EDGE-1), read 2026-11-07:
  each bit is two equal pulses, a 1 "exactly twice as long" as a 0; most
  significant bit first; "no gaps either between the sync pulse and the
  first bit of the flag, or between bytes"; the parity byte is the XOR of
  the flag and the data. The loader wants 256 pairs of leader edges
  "within about 3,000 T states" of each other, takes the first short pulse
  (about 1100 T) as sync, and compares a bit's two pulses together against
  "approx. 2,400 T states". The ROM's leader loop counts 0x1F80 and 0x0C98
  (8064, 3224 passes); TZX says 8063 and 3223 pulses: unresolved, TZX's
  figure is used (QUESTIONS.md in the W-078 scratch directory).

A pulse is one half-period of the square wave: a level held for a time and
ended by an edge. A block's pulse list is the pilot, sync 1, sync 2 and two
pulses per bit; its edge list starts with the edge that begins the first
pilot pulse, so it has one edge more than pulses.
"""

from __future__ import annotations

import dataclasses
import random
from typing import List, Optional, Sequence, Tuple

from .bench import Drive, Lines, Model, PinSpec, pad_of

T_HZ = 3_500_000.0                  # [TZX] section 2


@dataclasses.dataclass(frozen=True)
class Timings:
    """Pulse lengths in T-states and pilot counts [TZX section 5, ID 10]."""

    pilot: int = 2168
    sync1: int = 667
    sync2: int = 735
    zero: int = 855
    one: int = 1710
    pilot_header: int = 8063        # flag < 128
    pilot_data: int = 3223          # flag >= 128


ROM = Timings()
#: The ROM loader's bit threshold, both pulses together [ROM 05CA].
ROM_BIT_THRESHOLD_T = 2400


def checksum(flag: int, data: Sequence[int]) -> int:
    """The parity byte: the XOR of the flag and every data byte [ROM]."""
    x = flag & 0xFF
    for b in data:
        x ^= b & 0xFF
    return x


def block_bytes(flag: int, data: Sequence[int]) -> List[int]:
    """The bytes on the tape: flag, data, checksum."""
    return [flag & 0xFF] + [b & 0xFF for b in data] + [checksum(flag, data)]


def pilot_count(flag: int, t: Timings = ROM) -> int:
    return t.pilot_header if (flag & 0xFF) < 128 else t.pilot_data


def bits_msb_first(byte: int) -> List[int]:
    return [(byte >> (7 - i)) & 1 for i in range(8)]


def block_pulses(flag: int, data: Sequence[int], t: Timings = ROM,
                 pilot: Optional[int] = None) -> List[int]:
    """The pulse lengths of one block, in T-states (``pilot`` overrides the
    ROM's pilot count, as a shortened test block does)."""
    n = pilot_count(flag, t) if pilot is None else pilot
    out = [t.pilot] * n + [t.sync1, t.sync2]
    for byte in block_bytes(flag, data):
        for bit in bits_msb_first(byte):
            out += [t.one if bit else t.zero] * 2
    return out


def clocks_of(t_states: float, clock_hz: float) -> int:
    """A length in T-states as the nearest whole number of clocks."""
    return int(round(t_states * clock_hz / T_HZ))


def vary(pulses: Sequence[float], k: float = 1.0, jitter: float = 0.0,
         seed: int = 1) -> List[float]:
    """A tape player's pulses: every one scaled by ``k`` (above 1: the tape
    runs slow) and moved by a uniform error of up to ``jitter`` (same unit
    as the pulses), each independently."""
    rng = random.Random(seed)
    return [p * k + (rng.uniform(-jitter, jitter) if jitter else 0.0) for p in pulses]


def edges_of(pulses: Sequence[float], start: int, clock_hz: float,
             unit_hz: float = T_HZ) -> List[int]:
    """Edge cycles of a pulse list (pulses in 1/unit_hz seconds): the first
    edge at ``start``, each later one a pulse after the one before, rounded
    to the clock without accumulating the rounding."""
    out, at = [start], 0.0
    for p in pulses:
        at += p
        out.append(start + int(round(at * clock_hz / unit_hz)))
    return out


def pulses_of(edges: Sequence[int]) -> List[int]:
    """Edge cycles to pulse lengths in clocks."""
    return [b - a for a, b in zip(edges, edges[1:])]


# ------------------------------------------------------------------ decoder
@dataclasses.dataclass
class Decoded:
    """One block read from a pulse list by :func:`decode_block`."""

    pilot: int                      # pilot pulses before sync 1
    bytes: List[int]                # flag, data, checksum as read
    complete: bool                  # the pulses ended on a byte boundary
    error: Optional[str] = None

    @property
    def ok(self) -> bool:
        x = 0
        for b in self.bytes:
            x ^= b
        return self.complete and self.error is None and len(self.bytes) >= 2 and x == 0


def _near(p: float, nominal: float, tol: float) -> bool:
    return abs(p / nominal - 1.0) <= tol


def decode_block(pulses: Sequence[float], t: Timings = ROM, unit: float = 1.0,
                 tol: float = 0.15) -> Decoded:
    """Read one block from its pulses (``pulses[i] / unit`` T-states).

    Every pulse is classed by its nominal length within ``tol`` (relative),
    the format's own rule rather than a loader's: a run of pilot pulses,
    sync 1, sync 2, then pairs of equal pulses, both of a 0 or both of a 1.
    Leading pulses that are not pilot are skipped; the block ends at the
    end of the list or at the first pulse that fits nothing, which is an
    error unless it falls on a byte boundary."""
    p = [x / unit for x in pulses]
    i = 0
    while i < len(p) and not _near(p[i], t.pilot, tol):
        i += 1
    start = i
    while i < len(p) and _near(p[i], t.pilot, tol):
        i += 1
    n = i - start
    if i + 2 > len(p) or not (_near(p[i], t.sync1, tol) and _near(p[i + 1], t.sync2, tol)):
        return Decoded(n, [], False, "no sync after %d pilot pulses" % n)
    i += 2
    out: List[int] = []
    byte, bits = 0, 0
    while i + 1 < len(p):
        a, b = p[i], p[i + 1]
        if _near(a, t.zero, tol) and _near(b, t.zero, tol):
            bit = 0
        elif _near(a, t.one, tol) and _near(b, t.one, tol):
            bit = 1
        else:
            break
        byte, bits, i = (byte << 1) | bit, bits + 1, i + 2
        if bits == 8:
            out.append(byte)
            byte, bits = 0, 0
    if bits:
        return Decoded(n, out, False, "%d bits of a byte" % bits)
    return Decoded(n, out, True)


# --------------------------------------------------------------- pin models
class EdgeRecorder(Model):
    """Every change of one pad: ``edges`` holds ``(cycle, level)``, the cycle
    being the first with the new level. The level before the first is 0."""

    def __init__(self, pin: PinSpec) -> None:
        self.pad = pad_of(pin)
        self.level = 0
        self.edges: List[Tuple[int, int]] = []

    def observe(self, lines: Lines) -> None:
        level = lines.get(self.pad)
        if level != self.level:
            self.edges.append((lines.cycle, level))
            self.level = level

    def cycles(self) -> List[int]:
        return [c for c, _ in self.edges]


class TapeSource(Model):
    """A tape player's output on an input pin (a ``ui`` pad): low until the
    first edge, then toggling at every cycle of :meth:`play`'s list."""

    def __init__(self, pin: PinSpec) -> None:
        self.pad = pad_of(pin)
        self.level = 0
        self.edges: List[int] = []
        self._next = 0

    def play(self, edges: Sequence[int]) -> None:
        """Queue edge cycles (rising order, after any already queued)."""
        assert not self.edges[self._next:] or edges[0] > self.edges[-1]
        self.edges += list(edges)

    @property
    def free_at(self) -> int:
        return self.edges[-1] if self.edges else 0

    def drive(self, drive: Drive, cycle: int) -> None:
        while self._next < len(self.edges) and self.edges[self._next] <= cycle:
            self.level ^= 1
            self._next += 1
        drive.set(self.pad, self.level)


class Wire(Model):
    """The saver's MIC pin wired to the loader's EAR pin (a cable with a
    comparator at its end): the input follows the output one clock later."""

    def __init__(self, src: PinSpec, dst: PinSpec) -> None:
        self.src, self.dst = pad_of(src), pad_of(dst)
        self.level = 0

    def drive(self, drive: Drive, cycle: int) -> None:
        drive.set(self.dst, self.level)

    def observe(self, lines: Lines) -> None:
        self.level = lines.get(self.src)
