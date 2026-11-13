"""A reference 10BASE-T receiver for the transmit pair TD+/TD-.

Written from IEEE 802.3 clause 3 (the MAC frame) and clause 14 (10BASE-T),
as far as the clauses could be reached, and from nothing in Loom: it decodes
from the two pad levels sampled once a clock, never from model or RTL state
(D-036 step 6, brief W-117 X4).  Where a figure comes from a secondary source
it is marked [S], and [S, unchecked] where it could not be checked at all.

What it does, in the order a receiver meets it:

1. **Line levels.** The differential level is TD+ minus TD-: ``+1`` (TD+
   high, TD- low) is "high", ``-1`` is "low", and equal levels are no drive
   (idle).  "High is TD+ above TD-" [S, unchecked: the convention of every
   10BASE-T transmitter in the secondary sources].
2. **Manchester decode** (clause 7.3.1.1 for the coding, used by clause 14
   [S: clause numbers secondary]).  Every bit cell has a transition at its
   middle and the bit is the level after it: low to high is a 1, high to low
   a 0 [S: Wikipedia "Manchester code", the IEEE 802.3 convention].  The
   receiver takes the first driven cycle as the start of the first bit cell,
   expects each middle transition one bit time after the last one it found,
   and re-locks to every transition within half a half bit of the expected
   place (clock recovery, so a transmitter off by a fraction of a clock still
   decodes).  The first missing middle transition ends the bits.
3. **Preamble and SFD** (clause 3.2.1, 3.2.2: seven octets 10101010 and the
   SFD 10101011, in transmission order [S: Wikipedia "Ethernet frame"]).  The
   receiver needs at least ``min_preamble`` alternating bits and then the two
   1s that end the SFD; a receiver may lose the first bits of the preamble,
   so their number is reported, not required to be 56.
4. **FCS** (clause 3.2.9 [S]): a 32-bit CRC with the generator
   0x04C11DB7, register preset to all ones, every bit in transmission order
   (each octet LSB first, clause 3.3 [S]), and the complement sent ``x^31``
   term first.  Running the same register over the data and the FCS bits
   leaves the residue 0xC704DD7B [S: the well-known CRC-32 magic number].
5. **TP_IDL** (clause 14.3.1.2.1, Figure 14-10 [S: UNH-IOL 10BASE-T test
   14.1.7]): after the last bit the line stays high for a start-of-idle pulse
   and then goes to no drive.  Its length in clocks is reported; with
   ``clock_hz`` it is checked against 250 to 450 ns: IEEE 802.3-2022
   14.3.1.2.1, the start-of-idle template (positive from the last zero
   crossing until at least 2.25 to 2.5 BT, full amplitude no later than
   4.5 BT; checked 2026-11-09).
"""

from __future__ import annotations

import dataclasses
from typing import List, Optional, Sequence

CRC32_POLY = 0x04C11DB7
CRC32_RESIDUE = 0xC704DD7B
#: TP_IDL start-of-idle pulse window in ns (802.3-2022 14.3.1.2.1 template).
TP_IDL_MIN_NS = 250.0
TP_IDL_MAX_NS = 450.0
SFD_BITS = (1, 0, 1, 0, 1, 0, 1, 1)


def line_levels(tdp: Sequence[int], tdn: Sequence[int]) -> List[int]:
    """Per-cycle differential level: +1 high, -1 low, 0 not driven."""
    return [(p & 1) - (n & 1) for p, n in zip(tdp, tdn)]


def crc32_register(bits: Sequence[int], preset: int = 0xFFFFFFFF) -> int:
    """The clause 3.2.9 shift register after ``bits`` (transmission order)."""
    crc = preset
    for b in bits:
        fb = ((crc >> 31) & 1) ^ (b & 1)
        crc = (crc << 1) & 0xFFFFFFFF
        if fb:
            crc ^= CRC32_POLY
    return crc


def bits_to_bytes(bits: Sequence[int]) -> bytes:
    """Octets from bits in transmission order, each octet LSB first."""
    return bytes(sum((bits[i + k] & 1) << k for k in range(8))
                 for i in range(0, len(bits) - len(bits) % 8, 8))


def bytes_to_bits(data: bytes) -> List[int]:
    return [(byte >> k) & 1 for byte in data for k in range(8)]


def lsb_first_value(bits: Sequence[int]) -> int:
    """A field of bits read LSB first as a number (``bits[0]`` is bit 0)."""
    return sum((b & 1) << k for k, b in enumerate(bits))


@dataclasses.dataclass
class Decoded:
    """The Manchester layer: bits, where they were, and how the line ended."""

    bits: List[int]
    start: Optional[int]              # first driven cycle
    middles: List[int]                # cycle of each bit's middle transition
    high_start: Optional[int] = None  # start of the final high before no drive
    idle_at: Optional[int] = None     # first undriven cycle after the bits
    errors: List[str] = dataclasses.field(default_factory=list)

    @property
    def tp_idl_clocks(self) -> Optional[int]:
        if self.high_start is None or self.idle_at is None:
            return None
        return self.idle_at - self.high_start


def decode_manchester(levels: Sequence[int], half: int, start: int = 0) -> Decoded:
    """Decode the first burst of Manchester at or after cycle ``start``.

    ``half`` is the nominal half-bit time in clocks (clock / 20 MHz at
    10 Mbit/s).  The first driven cycle starts the first bit cell.
    """
    if half < 1:
        raise ValueError("half must be at least one clock")
    n = len(levels)
    a = next((i for i in range(start, n) if levels[i] != 0), None)
    out = Decoded(bits=[], start=a, middles=[])
    if a is None:
        return out
    tol = half // 2
    expect = a + half
    while True:
        found = None
        for delta in sorted(range(-tol, tol + 1), key=abs):
            i = expect + delta
            if 0 < i < n and levels[i] != 0 and levels[i - 1] != 0 \
                    and levels[i] != levels[i - 1] and i > (out.middles[-1] if out.middles else a):
                found = i
                break
        if found is None:
            break
        out.bits.append(1 if levels[found] > 0 else 0)
        out.middles.append(found)
        expect = found + 2 * half
    last = out.middles[-1] if out.middles else a
    out.idle_at = next((i for i in range(last, n) if levels[i] == 0), None)
    if out.idle_at is None:
        out.errors.append("line still driven at the end of the trace")
        return out
    j = out.idle_at - 1
    if levels[j] <= 0:
        out.errors.append("no start-of-idle high before the line went idle")
        return out
    while j > last and levels[j - 1] > 0:
        j -= 1
    out.high_start = j
    return out


@dataclasses.dataclass
class Frame:
    """One received frame (clause 3 view) with its line-level checks."""

    decoded: Decoded
    preamble_bits: int = 0           # preamble and SFD bits, the closing 11 included
    sfd_found: bool = False
    data: bytes = b""                # every octet after the SFD, FCS included
    residue: Optional[int] = None
    tp_idl_ns: Optional[float] = None
    errors: List[str] = dataclasses.field(default_factory=list)

    @property
    def payload(self) -> bytes:
        """The octets before the FCS (destination address .. pad)."""
        return self.data[:-4] if len(self.data) >= 4 else b""

    @property
    def fcs(self) -> Optional[int]:
        """The FCS octets as sent, read as a little-endian 32-bit number."""
        return int.from_bytes(self.data[-4:], "little") if len(self.data) >= 4 else None

    @property
    def fcs_ok(self) -> bool:
        return self.residue == CRC32_RESIDUE

    @property
    def ok(self) -> bool:
        return self.sfd_found and self.fcs_ok and not self.errors


def receive_frame(tdp: Sequence[int], tdn: Sequence[int], half: int,
                  clock_hz: Optional[float] = None, start: int = 0,
                  min_preamble: int = 8) -> Frame:
    """Receive the first frame at or after cycle ``start`` from the pad levels."""
    dec = decode_manchester(line_levels(tdp, tdn), half, start)
    frame = Frame(decoded=dec, errors=list(dec.errors))
    bits = dec.bits
    k = 0
    while k + 1 < len(bits) and bits[k] != bits[k + 1]:
        k += 1                      # alternating 1010...: the preamble
    # bits[k] == bits[k + 1] == 1 closes the SFD (10101011).
    if k + 1 >= len(bits) or bits[k] != 1 or k + 1 < min_preamble:
        frame.errors.append("no SFD after %d alternating bits" % k)
        return frame
    frame.sfd_found = True
    frame.preamble_bits = k + 2     # preamble and SFD bits received (64 if none lost)
    body = bits[k + 2:]
    if len(body) % 8:
        frame.errors.append("%d bits after the SFD is not whole octets" % len(body))
    frame.data = bits_to_bytes(body)
    if len(body) >= 32:
        frame.residue = crc32_register(body[:len(body) - len(body) % 8])
        if not frame.fcs_ok:
            frame.errors.append("FCS residue 0x%08X" % frame.residue)
    else:
        frame.errors.append("frame shorter than its FCS")
    if clock_hz is not None and dec.tp_idl_clocks is not None:
        frame.tp_idl_ns = dec.tp_idl_clocks * 1e9 / clock_hz
        if not TP_IDL_MIN_NS <= frame.tp_idl_ns <= TP_IDL_MAX_NS:
            frame.errors.append("TP_IDL high %.0f ns outside %g..%g ns"
                                % (frame.tp_idl_ns, TP_IDL_MIN_NS, TP_IDL_MAX_NS))
    return frame


def receive_all(tdp: Sequence[int], tdn: Sequence[int], half: int,
                clock_hz: Optional[float] = None) -> List[Frame]:
    """Every frame in the trace, each starting after the previous one's idle."""
    frames: List[Frame] = []
    levels = line_levels(tdp, tdn)
    start = 0
    while True:
        if next((i for i in range(start, len(levels)) if levels[i] != 0), None) is None:
            return frames
        frame = receive_frame(tdp, tdn, half, clock_hz, start)
        frames.append(frame)
        if frame.decoded.idle_at is None:
            return frames
        start = frame.decoded.idle_at
