"""IEC 61158-2 MBP (31.25 kbit/s Manchester bus powered) at logic level.

The physical layer of Foundation Fieldbus H1 and PROFIBUS PA, as seen on
the logic side of a media attachment unit (MAU): TX data and a transmit
enable towards the MAU, RX data from it. Written from the sources below,
not from ``firmware/mbp_tx.loom`` or ``firmware/mbp_rx.loom``.

IEC 61158-2 itself is not free; every figure here is **secondary**:

* [P] Fisher-Rosemount, US 7,694,061 B2 (filed 2004-09-08), the background
  on the H1 PhPDU (read 2026-11-07): the PhPDU is a preamble, a start
  delimiter (SD), the DLPDU and an end delimiter (ED); the preamble is one
  to eight octets of ``1 0 1 0 1 0 1 0``; SD = ``1 N+ N- 1 0 N- N+ 0``;
  ED = ``1 N+ N- N+ N- 1 0 1``; N+ and N- are bits with no transition
  in the bit period, the line held high (N+) or low (N-); the nominal bit
  time is 32 us; the DLPDU ends in a two-octet frame check sequence. The
  patent states the Manchester polarity as "0" low-to-high, "1"
  high-to-low.
* [K] T. R. Kuphaldt, *Lessons in Industrial Instrumentation*, "H1 FF
  physical layer" (read 2026-11-07): 31.25 kbit/s; a high-to-low
  transition is a 0, a low-to-high transition a 1 (the opposite of [P]).
  ``POL = 0`` below follows [K] (IEEE 802.3 Manchester, the bit engine's
  native code); ``POL = 1`` follows [P]. QUESTIONS.md item 1.
* From memory, **unchecked** (not found in a readable source): the FCS is
  the ones' complement of CRC-16 x^16 + x^12 + x^5 + 1 with initial value
  0xFFFF over the DLPDU octets, sent MSB first, with the residue 0x1D0F
  over octets and FCS (the IEC 61158-4-1 type 1 FCS); octets are sent MSB
  first; bit-rate tolerance +-0.2 %; transmit jitter of a mid-bit
  transition within +-1.0 us of its ideal place; a receiver accepts
  +-3.2 us (10 % of a bit). QUESTIONS.md items 2 to 4.

Contents: the symbol and half-bit encoder (:func:`frame_symbols`,
:func:`halfbits`), the CRC and FCS (:func:`crc16`, :func:`with_fcs`), a
bench source that drives a frame onto a pin at any rate (:class:`Source`),
the MAU between a transmitter and a receiver pin (:class:`Mau`), an edge
recorder (:class:`Recorder`), an independent decoder working from recorded
edges (:func:`decode`) and the timing checker (:func:`check_timing`).
"""

from __future__ import annotations

import dataclasses
from typing import List, Optional, Sequence, Tuple

from tools.protomodels.bench import Model

BIT_RATE = 31_250                   # [P], [K]
BIT_S = 1.0 / BIT_RATE              # 32 us [P]
RATE_TOL = 0.002                    # +-0.2 %, from memory, unchecked
TX_JITTER_S = 1.0e-6                # from memory, unchecked
RX_JITTER_S = 3.2e-6                # from memory, unchecked

PREAMBLE = ("1", "0", "1", "0", "1", "0", "1", "0")      # [P]
SD = ("1", "N+", "N-", "1", "0", "N-", "N+", "0")        # [P]
ED = ("1", "N+", "N-", "N+", "N-", "1", "0", "1")        # [P]
POLY, INIT, RESIDUE = 0x1021, 0xFFFF, 0x1D0F            # unchecked


# ------------------------------------------------------------ CRC and FCS
def crc16(octets: Sequence[int], crc: int = INIT) -> int:
    """CRC-16 x^16+x^12+x^5+1, MSB first, no reflection, no final XOR."""
    for octet in octets:
        crc ^= (octet & 0xFF) << 8
        for _ in range(8):
            crc = ((crc << 1) ^ POLY) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def fcs(octets: Sequence[int]) -> Tuple[int, int]:
    """The two FCS octets, high first: ``~crc16(octets)``."""
    value = crc16(octets) ^ 0xFFFF
    return value >> 8, value & 0xFF


def with_fcs(octets: Sequence[int]) -> List[int]:
    return list(octets) + list(fcs(octets))


def fcs_ok(octets_and_fcs: Sequence[int]) -> bool:
    """The receiver's check: the CRC over octets and FCS is the residue."""
    return crc16(octets_and_fcs) == RESIDUE


# --------------------------------------------------- symbols and half-bits
def octet_symbols(octet: int) -> List[str]:
    return ["1" if (octet >> (7 - i)) & 1 else "0" for i in range(8)]


def frame_symbols(octets: Sequence[int], preamble: int = 1, *,
                  fcs_octets: Optional[Sequence[int]] = None,
                  sd: Sequence[str] = SD, ed: Sequence[str] = ED) -> List[str]:
    """Preamble, SD, the octets, the FCS (computed, or ``fcs_octets``), ED."""
    tail = list(fcs(octets) if fcs_octets is None else fcs_octets)
    out = list(PREAMBLE) * preamble + list(sd)
    for octet in list(octets) + tail:
        out += octet_symbols(octet)
    return out + list(ed)


def halfbits(symbols: Sequence[str], pol: int = 0) -> List[int]:
    """Line levels, two per symbol. ``pol = 0``: a 1 is low then high [K];
    ``pol = 1``: a 1 is high then low [P]. N+ is high, N- low, both ways."""
    out: List[int] = []
    for s in symbols:
        if s == "N+":
            out += [1, 1]
        elif s == "N-":
            out += [0, 0]
        else:
            bit = 1 if s == "1" else 0
            first = (1 - bit) if pol == 0 else bit
            out += [first, 1 - first]
    return out


def octets_from_halfbits(levels: Sequence[int], pol: int = 0) -> "Frame":
    """Decode one PhPDU from a half-bit level stream (see :func:`decode`)."""
    sd = halfbits(SD, pol)
    ed = halfbits(ED, pol)
    pre = halfbits(PREAMBLE, pol)
    frame = Frame()
    n = len(levels)
    start = None
    for i in range(0, n - 15):
        if list(levels[i:i + 16]) == sd:
            start = i
            break
    if start is None:
        frame.errors.append("no SD")
        return frame
    # preamble bits before the SD: whole Manchester bits matching 1010...
    m = 0                               # bits counted back from the SD
    while start - 2 * (m + 1) >= 0:
        k = 14 - 2 * (m % 8)            # its two half-bits in ``pre``
        at = start - 2 * (m + 1)
        if list(levels[at:at + 2]) != pre[k:k + 2]:
            break
        m += 1
    frame.preamble_bits = m
    i = start + 16
    while True:
        if i + 16 > n:
            frame.errors.append("no ED")
            return frame
        chunk = list(levels[i:i + 16])
        if chunk == ed:
            frame.ended = True
            frame.end_halfbit = i + 16
            return frame
        octet = 0
        for k in range(8):
            a, b = chunk[2 * k], chunk[2 * k + 1]
            if a == b:
                frame.errors.append("coding error in octet %d" % len(frame.octets))
                return frame
            bit = b if pol == 0 else a
            octet = (octet << 1) | bit
        frame.octets.append(octet)
        i += 16


@dataclasses.dataclass
class Frame:
    """A decoded PhPDU: the octets between SD and ED (FCS included)."""

    octets: List[int] = dataclasses.field(default_factory=list)
    preamble_bits: int = 0
    ended: bool = False
    end_halfbit: int = 0
    errors: List[str] = dataclasses.field(default_factory=list)

    @property
    def good(self) -> bool:
        return self.ended and not self.errors and fcs_ok(self.octets)

    @property
    def data(self) -> List[int]:
        return self.octets[:-2]


# ------------------------------------------------------------ bench models
class Source(Model):
    """Drives ``levels`` (half-bits) onto pin ``pin`` from ``start`` (a
    cycle), at ``rate`` bit/s on a ``clock_hz`` clock, then ``idle``.
    ``jitter`` (seconds, one value per half-bit boundary, optional) moves
    each level change."""

    def __init__(self, pin, levels: Sequence[int], start: int, clock_hz: float,
                 rate: float = BIT_RATE, idle: int = 0,
                 jitter: Optional[Sequence[float]] = None) -> None:
        self.pin, self.idle = pin, idle
        self.levels = list(levels)
        half = clock_hz / rate / 2.0
        self.times = []
        for i in range(len(self.levels) + 1):
            j = jitter[i] if jitter and i < len(jitter) else 0.0
            self.times.append(start + i * half + j * clock_hz)
        self.pad = None
        self.i = -1                     # index of the level now driven

    def drive(self, drive, cycle):
        if self.pad is None:
            from tools.protomodels.bench import pad_of
            self.pad = pad_of(self.pin)
        while self.i + 1 < len(self.times) and cycle >= self.times[self.i + 1]:
            self.i += 1
        if 0 <= self.i < len(self.levels):
            drive.set(self.pad, self.levels[self.i])
        else:
            drive.set(self.pad, self.idle)

    @property
    def done_cycle(self) -> int:
        return int(self.times[-1]) + 1


class Mau(Model):
    """The MAU at logic level: ``rx`` follows ``tx`` while ``txe`` is high
    (the bus echo every transmitter hears), else ``idle``. One clock late."""

    def __init__(self, tx="OUT0", txe="OUT1", rx="IN0", idle: int = 0) -> None:
        from tools.protomodels.bench import pad_of
        self.tx, self.txe, self.rx = pad_of(tx), pad_of(txe), pad_of(rx)
        self.idle = idle
        self.level = idle

    def drive(self, drive, cycle):
        drive.set(self.rx, self.level)

    def observe(self, lines):
        self.level = lines.get(self.tx) if lines.get(self.txe) else self.idle


class Recorder(Model):
    """Every change of ``pins`` (``uo`` pins), as ``(cycle, pin, level)``."""

    def __init__(self, pins=("OUT0", "OUT1")) -> None:
        from tools.protomodels.bench import pad_of
        self.names = list(pins)
        self.pads = [pad_of(p) for p in pins]
        self.last = [0] * len(pins)
        self.edges: List[Tuple[int, str, int]] = []

    def observe(self, lines):
        for k, pad in enumerate(self.pads):
            v = lines.get(pad)
            if v != self.last[k]:
                self.last[k] = v
                self.edges.append((lines.cycle, self.names[k], v))

    def of(self, name) -> List[Tuple[int, int]]:
        return [(c, v) for c, n, v in self.edges if n == name]


# ---------------------------------------------------- decoder and timing
def edge_grid(edges: Sequence[Tuple[int, int]], clock_hz: float,
              rate: float = BIT_RATE) -> List[int]:
    """The half-bit index of every edge, counted from the first."""
    half = clock_hz / rate / 2.0
    t0 = edges[0][0]
    return [int(round((c - t0) / half)) for c, _ in edges]


def decode(edges: Sequence[Tuple[int, int]], clock_hz: float,
           rate: float = BIT_RATE, pol: int = 0,
           end: Optional[int] = None) -> Frame:
    """Decode the PhPDU on a recorded line, independently of any firmware.

    ``edges`` are ``(cycle, level)`` changes of the line during one frame.
    Every edge sits on a half-bit boundary, so the run between two edges is
    a whole number of half-bits; the half-bit before the first edge had the
    other level (the first preamble bit's first half, when the line idled at
    that level). ``end`` (a cycle, the end of the frame: the transmit enable
    falling) extends the last level to it; without it the last level is
    taken to last one half-bit."""
    if not edges:
        return Frame(errors=["no edges"])
    index = edge_grid(edges, clock_hz, rate)
    levels = [1 - edges[0][1]]
    for k in range(len(edges) - 1):
        levels += [edges[k][1]] * (index[k + 1] - index[k])
    half = clock_hz / rate / 2.0
    tail = 1 if end is None else int(round((end - edges[-1][0]) / half))
    levels += [edges[-1][1]] * tail
    return octets_from_halfbits(levels, pol)


def check_timing(edges: Sequence[Tuple[int, int]], clock_hz: float,
                 rate: float = BIT_RATE) -> dict:
    """Edges against the half-bit grid of the nominal rate.

    Returns ``max_dev_s``, the largest distance of an edge from its place
    on the grid fitted to the first and last edges; ``rate_err``, the
    fitted rate's relative error; and ``runs``, the set of run lengths in
    half-bits (Manchester data gives 1 and 2; N+ and N- next to a data bit
    give 3; nothing in a valid PhPDU gives more than 3, [P])."""
    index = edge_grid(edges, clock_hz, rate)
    span_c = edges[-1][0] - edges[0][0]
    half = span_c / index[-1] if index[-1] else clock_hz / rate / 2.0
    dev = max(abs((c - edges[0][0]) - n * half) for (c, _), n in zip(edges, index))
    nominal = clock_hz / rate / 2.0
    runs = {b - a for a, b in zip(index, index[1:])}
    return {"max_dev_s": dev / clock_hz, "rate_err": nominal / half - 1.0,
            "runs": runs}


def jitter_pattern(n: int, amplitude_s: float, seed: int = 1) -> List[float]:
    """A repeatable pseudo-random jitter, uniform in +-``amplitude_s``."""
    out, x = [], seed & 0xFFFF or 1
    for _ in range(n):
        x = (x * 75 + 74) % 65537
        out.append(amplitude_s * (2.0 * x / 65536.0 - 1.0))
    return out
