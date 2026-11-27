"""RDS / RBDS (radio data system) reference model at baseband: the code,
groups, differential and biphase coding, a line recorder with a timing
checker, a biphase player, and a receiver that syncs by syndrome.

Written from the standards, never from ``firmware/rds_*.loom``. Read
2026-11-26 with the web fetch tool (the PDFs' text layers):

* **NRSC-4-A** (2005), "United States RBDS Standard", Part I
  (nrscstandards.org archive; NRSC-4-B of 2011 prints only the differences
  from IEC 62106, which since 2021 replaces it as IEC 62106-9):

  - 1.5: the clock is the 57 kHz subcarrier divided by 48.
  - 1.6, Tables 1 and 2: differential coding. An input 0 leaves the output
    as the previous output bit, an input 1 complements it; the receiver
    takes the XOR of consecutive bits, so "the data is correctly decoded
    whether or not the demodulated data signal is inverted"; the message
    clock is 1187.5 Hz.
  - 1.7: each (differentially coded) bit is sent as a biphase symbol, an
    odd impulse pair: two half-symbols of opposite sign. The two equations
    that say which sign comes first for a 1 are images the text layer does
    not carry: :data:`ONE_FIRST` (a 1 starts high) is from memory,
    unchecked. Nothing a receiver does depends on it (Table 2).
  - 2.1: a group is 104 bits, 4 blocks of 26: a 16-bit information word
    and a 10-bit checkword. 2.2: MSB first; "fully synchronous", no gaps
    between groups or blocks.
  - 2.3: the checkword is (m(x) x^10 mod g(x)) + d(x), g(x) = x^10 + x^8 +
    x^7 + x^5 + x^4 + x^3 + 1, d(x) the block's offset word, sent MSB
    first after the information word. The code detects every burst of 10
    bits or less and "is capable of correcting any single burst of span 5
    bits or less".
  - 2.4: blocks are identified by the offset words A, B, C or C', D added
    to blocks 1 to 4; block synchronisation comes from the error-checking
    decoder detecting slips (Annex C).
  - 3.1.2: block 2 starts with the group type code A3..A0 and the version
    bit B0 (0: version A, PI in block 1 only; 1: version B, PI in blocks 1
    and 3); C' marks block 3 of a version B group.
  - 3.1.5.1, notes to type 0 groups: the PS name is 8 characters, sent 2
    a group, placed by the segment address C1 C0 in block 2, addresses
    increasing left to right, b7 of each character first; one DI bit a
    group, d3 at C1 C0 = 00. Figure 12, the 0A layout, is an image: the
    bit positions used by :func:`block2_0a` (TP 10, PTY 9..5, TA 4, M/S
    3, DI 2, C1 C0 1..0) are from memory, unchecked.

* **NRSC-4-A annexes** (2005): Annex A Table A.1, the offset words
  (:data:`OFFSETS`; E is not used in RDS); Annex B B.1 Figure B.1, whose
  bottom row gives the check bits 0110111001 for the message 1; B.2.1
  Table B.1, the syndromes of the offsets (:data:`SYNDROMES`) through the
  parity-check matrix of Figure B.3; B.2.2, error trapping (a burst is
  found when the five left-most syndrome stages are zero; the decoder
  premultiplies by x^325 mod g(x)); Annex C C.1.1, acquisition: the
  syndrome of the last 26 bits on every data clock until two valid offset
  words in sequence n x 26 bits apart (n = 1 here); C.1.2, loss: decide
  over several blocks ("e.g. up to 50").

* **ITU-R BS.643-3** (05/2011), Annex 1: 1.1, subcarrier 57 kHz +- 6 Hz;
  1.4, "the basic data rate is 1 187.5 bit/s +- 0.125 bit/s" (+-105 ppm,
  :data:`RATE_TOLERANCE`); 1.5 the differential coding again.

Checked here numerically: Table B.1's syndrome is y(x) x^325 mod g(x)
(:func:`table_syndrome`), so the plain remainder y(x) mod g(x) of a clean
block is its offset word itself (:func:`remainder`).

The receiver (:func:`receive`) is a decoder in the sense of Annex C with
the choices any implementation has to make, stated as parameters: n = 1;
one candidate at a time; loss after ``loss`` consecutive blocks that were
not clean; burst correction (span 5) only while in sync.
"""

from __future__ import annotations

import dataclasses
from typing import Iterable, List, Optional, Sequence, Tuple

from tools.protomodels.bench import Drive, Lines, Model, PinSpec, pad_of

# ------------------------------------------------------------ the code
G = 0x5B9                                   # x^10+x^8+x^7+x^5+x^4+x^3+1 (2.3)
OFFSETS = {"A": 0x0FC, "B": 0x198, "C": 0x168, "C'": 0x350, "D": 0x1B4}  # Table A.1
SYNDROMES = {"A": 0x3D8, "B": 0x3D4, "C": 0x25C, "C'": 0x3CC, "D": 0x258}  # Table B.1
NAMES = ("A", "B", "C", "C'", "D")          # report index order
BIT_RATE = 1187.5                           # BS.643-3 1.4, NRSC-4-A 1.6
RATE_TOLERANCE = 0.125 / 1187.5             # BS.643-3 1.4
ONE_FIRST = 1                               # a 1 starts high (1.7: from memory, unchecked)
CORRECTABLE_SPAN = 5                        # 2.3


def remainder(poly: int) -> int:
    """``poly(x) mod g(x)``."""
    for b in range(poly.bit_length() - 1, 9, -1):
        if poly >> b & 1:
            poly ^= G << (b - 10)
    return poly


def times_x(v: int, k: int) -> int:
    for _ in range(k):
        v <<= 1
        if v & 0x400:
            v ^= G
    return v


def table_syndrome(block26: int) -> int:
    """The syndrome of Table B.1 (Figure B.3's H): y(x) x^325 mod g(x)."""
    return times_x(remainder(block26), 325)


def checkword(info: int, offset: str) -> int:
    """2.3: (m(x) x^10 mod g(x)) + d(x)."""
    return remainder(info << 10) ^ OFFSETS[offset]


def block(info: int, offset: str) -> int:
    """The 26-bit block, information word first."""
    return (info & 0xFFFF) << 10 | checkword(info, offset)


def group_offsets(info_b: int) -> Tuple[str, str, str, str]:
    """A, B, C or C' (B0 = bit 11 of block 2, 3.1.2), D."""
    return ("A", "B", "C'" if info_b >> 11 & 1 else "C", "D")


def group_bits(infos: Sequence[int]) -> List[int]:
    """The 104 bits of a group from its four information words."""
    bits: List[int] = []
    for info, off in zip(infos, group_offsets(infos[1])):
        b = block(info, off)
        bits += [b >> (25 - k) & 1 for k in range(26)]
    return bits


def block2_0a(*, tp: int, pty: int, ta: int, ms: int, di_bit: int, seg: int,
              version_b: bool = False) -> int:
    """Block 2 of a type 0 group (3.1.2, 3.1.5.1; positions of the fields
    below B0 from memory, unchecked: see the module docstring)."""
    return (int(version_b) << 11 | (tp & 1) << 10 | (pty & 31) << 5
            | (ta & 1) << 4 | (ms & 1) << 3 | (di_bit & 1) << 2 | seg & 3)


def ps_groups(pi: int, ps: str, *, tp=1, pty=5, ta=0, ms=1, di=0b0001,
              block3=0xE0CD, version_b=False) -> List[List[int]]:
    """Four type 0A (or 0B) groups that carry the 8-character PS name; DI
    d3 first (3.1.5.1 note 5). In 0B block 3 repeats the PI."""
    ps = ps.ljust(8)[:8]
    groups = []
    for seg in range(4):
        b2 = block2_0a(tp=tp, pty=pty, ta=ta, ms=ms, seg=seg,
                       di_bit=di >> (3 - seg) & 1, version_b=version_b)
        c = pi if version_b else block3
        d = ord(ps[2 * seg]) << 8 | ord(ps[2 * seg + 1])
        groups.append([pi, b2, c, d])
    return groups


def stream(groups: Sequence[Sequence[int]], repeat: int = 1) -> List[int]:
    bits: List[int] = []
    for _ in range(repeat):
        for g in groups:
            bits += group_bits(g)
    return bits


def diff_encode(bits: Iterable[int], prev: int = 0) -> List[int]:
    """1.6 Table 1."""
    out = []
    for b in bits:
        prev ^= b & 1
        out.append(prev)
    return out


def diff_decode(levels: Iterable[int], prev: int = 0) -> List[int]:
    """1.6 Table 2."""
    out = []
    for v in levels:
        out.append(v ^ prev)
        prev = v
    return out


def biphase(levels: Iterable[int]) -> List[int]:
    """1.7: two half-symbols a bit, of opposite sign."""
    out = []
    for v in levels:
        first = ONE_FIRST if v else 1 - ONE_FIRST
        out += [first, 1 - first]
    return out


# ------------------------------------------------------------ the receiver
NEXT = {"A": ("B",), "B": ("C", "C'"), "C": ("D",), "C'": ("D",), "D": ("A",)}
BLOCK_OF = {"A": 0, "B": 1, "C": 2, "C'": 2, "D": 3}

# status word of a report (the receiver firmware's host format, documented
# in firmware/rds_dec.loom; the model produces the same reports)
ST_CORRECTED, ST_BAD, ST_OVF, ST_SYNC, ST_LOST = 8, 16, 32, 64, 128


@dataclasses.dataclass(frozen=True)
class Report:
    info: int                   # information word (corrected when corrected)
    offset: str                 # offset name: matched, or expected if not clean
    corrected: bool = False     # a burst of span <= 5 was corrected
    bad: bool = False           # not correctable: info as received
    sync: bool = False          # the block that confirmed a candidate: sync from here
    lost: bool = False          # sync was lost after this block
    overflow: bool = False      # reports were dropped before this one

    def words(self) -> List[int]:
        st = (NAMES.index(self.offset) | ST_CORRECTED * self.corrected
              | ST_BAD * self.bad | ST_OVF * self.overflow
              | ST_SYNC * self.sync | ST_LOST * self.lost)
        return [self.info, st]


def parse_reports(words: Sequence[int]) -> List[Report]:
    out = []
    for k in range(0, len(words) - 1, 2):
        info, st = words[k], words[k + 1]
        out.append(Report(info, NAMES[st & 7] if (st & 7) < 5 else "?%d" % (st & 7),
                          bool(st & ST_CORRECTED), bool(st & ST_BAD),
                          bool(st & ST_SYNC), bool(st & ST_LOST), bool(st & ST_OVF)))
    return out


def find_burst(syn: int) -> Optional[int]:
    """The error pattern of span <= 5 inside 26 bits whose remainder is
    ``syn`` (2.3: such a burst is always correctable, so it is unique), or
    None."""
    for p in range(26):
        for b in range(1, 32, 2):                   # lowest bit set: one form each
            if p + b.bit_length() - 1 > 25:
                break
            if remainder(b << p) == syn:
                return b << p
    return None


def receive(bits: Sequence[int], *, loss: int = 8, correct: bool = True) -> List[Report]:
    """Annex C acquisition (n = 1, one candidate at a time: while a
    candidate waits its 26 bits, other matches are ignored; at the 26th
    bit a match of a following offset gives sync and that block is the
    first reported, otherwise the search restarts at that bit), then block
    by block: clean, corrected (a burst
    of span <= 5, Annex B.2.2) or bad; sync is lost after ``loss``
    consecutive blocks that were not clean. Block 3's offset for the burst
    search is C' when the last block 2 that was not bad had B0 = 1."""
    out: List[Report] = []
    window = 0
    cand: Optional[Tuple[str, int]] = None
    count = 0
    insync = False
    k = 0                                           # block in group, in sync
    b0 = 0
    bad_run = 0
    for bit in bits:
        window = (window << 1 | bit & 1) & ((1 << 26) - 1)
        s = remainder(window)
        info = window >> 10
        if not insync:
            match = next((n for n in NAMES if OFFSETS[n] == s), None)
            if cand is not None:
                count -= 1
                if count:
                    continue
                if match in NEXT[cand[0]]:
                    out.append(Report(info, match, sync=True))
                    insync, k, count, bad_run = True, (BLOCK_OF[match] + 1) % 4, 26, 0
                    if match == "B":
                        b0 = info >> 11 & 1
                    cand = None
                    continue
                cand = None
            if match is not None:
                cand, count = (match, info), 26
            continue
        count -= 1
        if count:
            continue
        count = 26
        names = [("A",), ("B",), ("C", "C'"), ("D",)][k]
        clean = next((n for n in names if OFFSETS[n] == s), None)
        if clean is not None:
            rep = Report(info, clean)
            bad_run = 0
        else:
            name = ("C'" if b0 else "C") if k == 2 else names[0]
            e = find_burst(s ^ OFFSETS[name]) if correct else None
            if e is not None:
                rep = Report(info ^ (e >> 10), name, corrected=True)
            else:
                rep = Report(info, name, bad=True)
            bad_run += 1
        if k == 1 and not rep.bad:              # a bad block 2 leaves B0 as it was
            b0 = rep.info >> 11 & 1
        if bad_run >= loss:
            rep = dataclasses.replace(rep, lost=True)
            insync, cand = False, None
        out.append(rep)
        k = (k + 1) % 4
    return out


# --------------------------------------------------------- pin models
def half_symbol_clocks(clock_hz: float) -> float:
    return clock_hz / (2 * BIT_RATE)


class Player(Model):
    """Drives the biphase baseband of the data bits ``bits`` on one pin:
    differentially coded (from ``prev``), two half-symbols a bit of
    ``hs_clocks`` clocks each (any real number; edges at the rounded
    running sum). ``flips`` inverts data bits before the coding (a bit
    error in the decoded stream); ``drop`` removes data bits (a slip).
    The line rests at ``idle`` before and after."""

    def __init__(self, pin: PinSpec, *, idle: int = 0) -> None:
        self.pad = pad_of(pin)
        self.level = self.idle_level = idle
        self._events: List[Tuple[int, int]] = []
        self._now = 0

    def play(self, bits: Sequence[int], hs_clocks: float, *, lead: int = 8,
             flips: Iterable[int] = (), drop: Iterable[int] = (), prev: int = 0) -> int:
        flips, drop = set(flips), set(drop)
        data = [b ^ (k in flips) for k, b in enumerate(bits) if k not in drop]
        halves = biphase(diff_encode(data, prev))
        t0 = self._now + lead
        for j, v in enumerate(halves):
            self._events.append((t0 + int(round(j * hs_clocks)), v))
        end = t0 + int(round(len(halves) * hs_clocks))
        self._events.append((end, self.idle_level))
        return end

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


class Line(Model):
    """Records an encoder's baseband pin and decodes it as a receiver
    would: half-symbols on the grid of the first edge, paired so that
    every bit has its mid-bit transition (1.7), the pairs' levels
    differentially decoded (1.6 Table 2)."""

    def __init__(self, pin: PinSpec, rest: int = 0) -> None:
        self.pad = pad_of(pin)
        self.rest = self.level = rest
        self.edges: List[Tuple[int, int]] = []

    def observe(self, lines: Lines) -> None:
        v = lines.get(self.pad)
        if v != self.level:
            self.level = v
            self.edges.append((lines.cycle, v))

    def halves(self, hs: float) -> List[int]:
        """Levels at the middle of each half-symbol, from the one before
        the first edge to the one after the last."""
        if not self.edges:
            return []
        t0 = self.edges[0][0]
        n = int(round((self.edges[-1][0] - t0) / hs)) + 1
        out, j, level = [], 0, self.rest
        for i in range(n + 1):
            t = t0 + (i - 0.5) * hs
            while j < len(self.edges) and self.edges[j][0] <= t:
                level = self.edges[j][1]
                j += 1
            out.append(level)
        return out

    def symbols(self, hs: float) -> List[int]:
        """The coded bits: the first half of each pair, in the pairing
        with a transition in the middle of every pair."""
        h = self.halves(hs)
        for phase in (0, 1):
            pairs = [(h[i], h[i + 1]) for i in range(phase, len(h) - 1, 2)]
            if pairs and all(a != b for a, b in pairs):
                return [int(a == ONE_FIRST) for a, _ in pairs]
        raise AssertionError("no pairing of the half-symbols is valid biphase")

    def bits(self, hs: float, prev: int = 0) -> List[int]:
        return diff_decode(self.symbols(hs), prev)

    def check(self, hs: float, tol_clocks: float, clock_hz: float) -> List[str]:
        """Every edge within ``tol_clocks`` of the grid of ``hs`` clocks
        from the first edge; the half-symbol rate over the whole record
        within BS.643-3 1.4's +-0.125 bit/s of 2 x 1187.5 at ``clock_hz``."""
        bad: List[str] = []
        if len(self.edges) < 2:
            return ["no signal"]
        t0 = self.edges[0][0]
        for c, _ in self.edges:
            k = round((c - t0) / hs)
            if abs(c - t0 - k * hs) > tol_clocks:
                bad.append("edge at %d is %.2f clocks off the grid (half-symbol %d)"
                           % (c, c - t0 - k * hs, k))
        last = self.edges[-1][0]
        k = round((last - t0) / hs)
        rate = clock_hz / ((last - t0) / k) / 2
        if abs(rate / BIT_RATE - 1) > RATE_TOLERANCE:
            bad.append("bit rate %.4f bit/s, %.1f ppm from 1187.5"
                       % (rate, 1e6 * (rate / BIT_RATE - 1)))
        return bad
