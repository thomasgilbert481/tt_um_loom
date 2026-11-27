"""IEEE 802.15.4 868/915 MHz BPSK PHY reference model: frames, the FCS,
differential encoding, spreading, a chip-line recorder with a timing
checker and decoder, and a chip source for a receiver.

Written from the standard, never from ``firmware/zb868_*.loom``:

* **IEEE Std 802.15.4-2006**, read 2026-11-26 through its text layer (the
  IEEE GET landing page gave no text; the copy read was the 2006 PDF at
  people.ece.ubc.ca/~edc/7860/data/802.15.4-2006.pdf):

  - 6.3: the PPDU is SHR (Preamble, SFD), PHR, PSDU; the leftmost field is
    sent first, multi-octet fields least significant octet first, and each
    octet least significant bit first.
  - 6.3.1, Table 19: the 868 and 915 MHz BPSK preamble is 4 octets (32
    symbols) of binary zeros.
  - 6.3.2, Figure 17: the SFD is 8 bits, bits 0..7 = 1 1 1 0 0 1 0 1
    (0xA7, bit 0 sent first).
  - 6.3.3, Table 21: the PHR is a 7-bit Frame Length (the PSDU octets, 0 to
    aMaxPHYPacketSize) and a reserved bit; lengths 0-4 and 6-8 are
    reserved, 5 is an acknowledgment, 9-127 an MPDU. 6.4.1 Table 22:
    aMaxPHYPacketSize = 127.
  - 6.6.1: 20 kb/s in the 868 MHz band, 40 kb/s in the 915 MHz band.
  - 6.6.2.1: every PPDU bit, from the preamble to the last PSDU octet, goes
    through differential encoding, then bit-to-chip mapping, LSB first.
  - 6.6.2.2, equations (2) and (3): E_n = R_n xor E_(n-1), E_0 = 0 for
    each packet; the receiver's R_n = E_n xor E_(n-1), with E_0 = 0.
  - 6.6.2.3, Table 27: a 0 is chips c0..c14 = 1 1 1 1 0 1 0 1 1 0 0 1 0 0 0,
    a 1 the inverse. 6.6.2.4.2: c0 is sent first (the text says "c15 ...
    last"; Table 27 has 15 chips, c0..c14).
  - 6.6.2.4: 300 kchip/s (868 MHz) and 600 kchip/s (915 MHz); a chip 1 is
    a positive pulse, a 0 a negative one, raised-cosine shaped (roll-off 1).
  - 6.6.3.3: symbol rate 20 / 40 ksymbol/s, accuracy +-40 ppm.
  - 7.2.1.9 and Figure 43: the FCS is the 16-bit ITU-T CRC, G16(x) = x^16 +
    x^12 + x^5 + 1, over the MHR and payload: M(x) = b0 x^(k-1) + ... +
    b(k-1), with b0 the first bit sent; R(x) = x^16 M(x) mod G16 = r0 x^15
    + ... + r15; the FCS is r0..r15, r0 sent first. Figure 43: the register
    starts at zero. Worked example: the MHR 0x02 0x00 0x6A (bits 0100 0000
    0000 0000 0101 0110) has the FCS bits 0010 0111 1001 1110.

Out of scope (and stated in the programs' headers): the carrier and the
raised-cosine pulse. The pin carries the chip stream at baseband, a chip
1 as a high level; the line idles low between frames.
"""

from __future__ import annotations

import random
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

from tools.protomodels.bench import Drive, Lines, Model, PinSpec, pad_of

CHIPS0 = (1, 1, 1, 1, 0, 1, 0, 1, 1, 0, 0, 1, 0, 0, 0)   # Table 27, c0 first
CHIPS1 = tuple(1 - c for c in CHIPS0)
NCHIP = 15
PREAMBLE_OCTETS = 4                                       # Table 19
SFD = 0xA7                                                # Figure 17
MAX_PSDU = 127                                            # Table 22
CHIP_RATE = {868: 300e3, 915: 600e3}                      # 6.6.2.4
BIT_RATE = {868: 20e3, 915: 40e3}                         # 6.6.1
RATE_TOLERANCE = 40e-6                                    # 6.6.3.3
G16 = (1 << 16) | (1 << 12) | (1 << 5) | 1                # 7.2.1.9, eq. (12)


def chip_clocks(band: int, clock_hz: float) -> float:
    """Clocks a chip at ``clock_hz``: 150 at 45 MHz in the 868 MHz band."""
    return clock_hz / CHIP_RATE[band]


def octet_bits(octets: Sequence[int]) -> List[int]:
    """Octets to bits in transmission order, LSB first (6.3)."""
    return [(o >> i) & 1 for o in octets for i in range(8)]


def bits_octets(bits: Sequence[int]) -> List[int]:
    return [sum(bits[8 * k + i] << i for i in range(8)) for k in range(len(bits) // 8)]


def fcs_bits(octets: Sequence[int]) -> List[int]:
    """r0..r15 of 7.2.1.9, by the long division the clause states: M(x)
    with b0 the highest power, times x^16, modulo G16."""
    bits = octet_bits(octets)
    m = 0
    for b in bits:
        m = (m << 1) | b
    m <<= 16
    for deg in range(m.bit_length() - 1, 15, -1):
        if (m >> deg) & 1:
            m ^= G16 << (deg - 16)
    return [(m >> (15 - i)) & 1 for i in range(16)]        # r0 = x^15 first


def fcs_octets(octets: Sequence[int]) -> List[int]:
    """The FCS as the two octets that go on the air (r0 = bit 0 of the first)."""
    return bits_octets(fcs_bits(octets))


def with_fcs(mpdu: Sequence[int]) -> List[int]:
    """The PSDU: the MHR and payload with their FCS appended."""
    return list(mpdu) + fcs_octets(mpdu)


def fcs_ok(psdu: Sequence[int]) -> bool:
    return len(psdu) >= 2 and fcs_octets(psdu[:-2]) == list(psdu[-2:])


def ppdu_bits(psdu: Sequence[int], preamble_octets: int = PREAMBLE_OCTETS,
              reserved: int = 0) -> List[int]:
    """The raw PPDU bits R_1, R_2, ...: preamble, SFD, PHR, PSDU (6.3)."""
    if len(psdu) > MAX_PSDU:
        raise ValueError("PSDU of %d octets: aMaxPHYPacketSize is 127" % len(psdu))
    phr = len(psdu) | (reserved & 1) << 7
    return octet_bits([0] * preamble_octets + [SFD, phr] + list(psdu))


def diff_encode(raw: Sequence[int]) -> List[int]:
    """6.6.2.2 equation (2), E_0 = 0."""
    out, e = [], 0
    for r in raw:
        e ^= r
        out.append(e)
    return out


def diff_decode(enc: Sequence[int]) -> List[int]:
    """6.6.2.2 equation (3), E_0 = 0."""
    out, prev = [], 0
    for e in enc:
        out.append(e ^ prev)
        prev = e
    return out


def spread(enc: Sequence[int]) -> List[int]:
    """Table 27: each encoded bit to its 15 chips, c0 first."""
    out: List[int] = []
    for e in enc:
        out.extend(CHIPS1 if e else CHIPS0)
    return out


def ppdu_chips(psdu: Sequence[int], preamble_octets: int = PREAMBLE_OCTETS) -> List[int]:
    return spread(diff_encode(ppdu_bits(psdu, preamble_octets)))


# ------------------------------------------------------- despreading
def despread(chips: Sequence[int]) -> List[Tuple[int, int]]:
    """Each whole 15-chip group to ``(E, d)``: ``d`` chips disagree with the
    0 sequence of Table 27, and E is the sign of the correlation 15 - 2d
    (1 when d >= 8; 15 is odd, so there is no tie)."""
    out = []
    for k in range(len(chips) // NCHIP):
        d = sum(c != s for c, s in zip(chips[NCHIP * k:NCHIP * (k + 1)], CHIPS0))
        out.append((1 if d >= 8 else 0, d))
    return out


@dataclass
class Frame:
    """A decoded PPDU. ``chip_errors`` is the most chips any bit had wrong."""

    length: int
    reserved: int
    psdu: List[int]
    fcs_ok: bool
    chip_errors: int
    problems: List[str] = field(default_factory=list)


def decode_chips(chips: Sequence[int], preamble_octets: int = PREAMBLE_OCTETS) -> Optional[Frame]:
    """Decode a chip stream that starts at c0 of the first preamble bit."""
    groups = despread(chips)
    shr = 8 * (preamble_octets + 1)
    if len(groups) < shr + 8:
        return None
    raw = diff_decode([e for e, _ in groups])
    problems = []
    if any(raw[:8 * preamble_octets]):
        problems.append("preamble not all zeros")
    if bits_octets(raw[8 * preamble_octets:shr]) != [SFD]:
        problems.append("SFD is not 0xA7")
    phr = bits_octets(raw[shr:shr + 8])[0]
    length = phr & 0x7F
    end = shr + 8 + 8 * length
    if len(raw) < end:
        problems.append("stream ends inside the PSDU")
        end = len(raw) - (len(raw) - shr - 8) % 8
    psdu = bits_octets(raw[shr + 8:end])
    worst = max(min(d, NCHIP - d) for _, d in groups[:end])
    return Frame(length, phr >> 7, psdu, fcs_ok(psdu) and len(psdu) == length,
                 worst, problems)


# --------------------------------------------------------- pin models
class ChipLine(Model):
    """Records the chip line of a transmitter and checks it.

    A frame starts at a rising edge after the line has been low for at
    least ``gap`` chips (the first chip of the preamble is c0 of a 0, a 1;
    the line idles low). Chips are read at mid-chip, ``t0 + (k + 1/2) Tc``."""

    def __init__(self, pin: PinSpec) -> None:
        self.pad = pad_of(pin)
        self.edges: List[Tuple[int, int]] = []
        self.level = 0

    def observe(self, lines: Lines) -> None:
        v = lines.get(self.pad)
        if v != self.level:
            self.level = v
            self.edges.append((lines.cycle, v))

    def bursts(self, tc: float,
               preamble_octets: int = PREAMBLE_OCTETS) -> List[List[Tuple[int, int]]]:
        """The edges of each frame. A frame starts at the first rising edge
        after the previous frame's last chip (the first chip of a
        preamble, c0 of a 0, is a 1; the line idles low) and is as long as
        its PHR says; a frame that is cut short ends at its last edge."""
        out: List[List[Tuple[int, int]]] = []
        edges, i = self.edges, 0
        while i < len(edges):
            if not edges[i][1]:
                i += 1
                continue
            t0 = edges[i][0]
            head = decode_chips(self.sample(edges[i:], tc, NCHIP * 8 * (preamble_octets + 2)),
                                preamble_octets)
            end = t0 + (NCHIP * 8 * (preamble_octets + 2 + head.length) * tc
                        if head is not None else len(edges) * tc * 1e6)
            j = i
            while j < len(edges) and edges[j][0] <= end + tc / 4:
                j += 1
            out.append(edges[i:j])
            i = j
        return out

    @staticmethod
    def sample(edges, tc: float, n: int) -> List[int]:
        t0, out, j, level = edges[0][0], [], 0, 0
        for k in range(n):
            t = t0 + (k + 0.5) * tc
            while j < len(edges) and edges[j][0] <= t:
                level = edges[j][1]
                j += 1
            out.append(level)
        return out

    def frames(self, tc: float, preamble_octets: int = PREAMBLE_OCTETS) -> List[Frame]:
        out = []
        for edges in self.bursts(tc, preamble_octets):
            head = decode_chips(self.sample(edges, tc, NCHIP * 8 * (preamble_octets + 2)),
                                preamble_octets)
            if head is None:
                continue
            n = NCHIP * 8 * (preamble_octets + 2 + head.length)
            out.append(decode_chips(self.sample(edges, tc, n), preamble_octets))
        return out

    def check(self, tc: float, tol_clocks: float = 1.0,
              nominal: Optional[float] = None) -> List[str]:
        """Every edge of every frame on its chip grid within ``tol_clocks``
        (so every chip period is a whole number of chips within twice
        that), no pulse shorter than a chip, and the chip rate measured
        over each frame within 6.6.3.3's +-40 ppm of ``nominal``."""
        nominal = tc if nominal is None else nominal
        bad: List[str] = []
        for edges in self.bursts(tc):
            t0 = edges[0][0]
            for (a, _), (b, _) in zip(edges, edges[1:]):
                if b - a < tc - 2 * tol_clocks:
                    bad.append("pulse of %d clocks at %d: shorter than a chip" % (b - a, a))
            for c, _ in edges:
                k = round((c - t0) / tc)
                if abs(c - t0 - k * tc) > tol_clocks:
                    bad.append("edge at %d is %.2f clocks off the chip grid (chip %d)"
                               % (c, c - t0 - k * tc, k))
            k = round((edges[-1][0] - t0) / tc)
            if k:
                measured = (edges[-1][0] - t0) / k
                if abs(measured / nominal - 1) > RATE_TOLERANCE:
                    bad.append("chip %.4f clocks, %.2e from %.4f"
                               % (measured, measured / nominal - 1, nominal))
        return bad


def flip_chips(chips: Sequence[int], per_bit: int, seed: int = 1,
               skip_bits: int = 0) -> List[int]:
    """Invert ``per_bit`` distinct chips, chosen at random, in every bit
    from bit ``skip_bits`` on."""
    rng = random.Random(seed)
    out = list(chips)
    for k in range(skip_bits, len(chips) // NCHIP):
        for i in rng.sample(range(NCHIP), per_bit):
            out[NCHIP * k + i] ^= 1
    return out


class ChipSource(Model):
    """Drives a receiver's input with a chip stream at baseband: ``tc``
    clocks a chip, made ``ppm`` parts per million fast (a positive error
    shortens the chip). Edges sit at the rounded running sum, so long
    streams do not drift. The line idles low before and after."""

    def __init__(self, pin: PinSpec) -> None:
        self.pad = pad_of(pin)
        self.level = 0
        self._events: List[Tuple[int, int]] = []
        self._now = 0

    def play(self, chips: Sequence[int], tc: float, *, ppm: float = 0.0,
             lead: int = 16) -> int:
        """Queue ``chips``; returns the cycle the line falls idle."""
        period = tc / (1.0 + ppm * 1e-6)
        t0 = max(self._now, self._events[-1][0] if self._events else 0) + lead
        level = 0
        for k, c in enumerate(list(chips) + [0]):
            if c != level:
                self._events.append((t0 + int(round(k * period)), c))
                level = c
        return t0 + int(round(len(chips) * period))

    @property
    def idle(self) -> bool:
        return not self._events

    def drive(self, drive: Drive, cycle: int) -> None:
        drive.set(self.pad, self.level)

    def observe(self, lines: Lines) -> None:
        self._now = lines.cycle + 1
        while self._events and self._events[0][0] <= self._now:
            _, self.level = self._events.pop(0)


class Wire(Model):
    """A board wire from a chip output to a chip input (one clock late)."""

    def __init__(self, src: PinSpec, dst: PinSpec) -> None:
        self.src, self.dst = pad_of(src), pad_of(dst)
        self.level = 0

    def drive(self, drive: Drive, cycle: int) -> None:
        drive.set(self.dst, self.level)

    def observe(self, lines: Lines) -> None:
        self.level = lines.get(self.src)
