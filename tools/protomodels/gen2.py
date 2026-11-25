"""EPC Gen2 UHF RFID at baseband: a reader model and the standard's checks.

Written from the standard, never from ``firmware/gen2_tag.loom``:

[G2] GS1, "EPC Radio-Frequency Identity Generation-2 UHF RFID Standard",
     Release 3.0.1, Ratified, Feb 2026 (read as text and, for Figures 6-11
     and 6-15, as rendered pages):

* 6.3.1.2.3, Figure 6-1: PIE; Tari is the length of a data-0; high = CW,
  low = attenuated CW; a symbol ends with its low pulse PW.
* 6.3.1.2.4: Tari 6.25 us to 25 us; parameters in Tari +-1 %.
* Table 6-5: PW from MAX(0.265 Tari, 2 us) to 0.525 Tari.
* 6.3.1.2.8, Figure 6-4: preamble = delimiter (12.5 us +-5 %), data-0,
  RTcal (2.5 to 3.0 Tari, = data-0 + data-1, so a data-1 is 1.5 to 2.0
  Tari), TRcal (1.1 RTcal to 3 RTcal, eq. 2); a frame-sync has no TRcal;
  Query is preceded by a preamble, every other command by a frame-sync. The
  tag sets pivot = RTcal / 2. BLF = DR / TRcal (eq. 1).
* 6.3.1.3: the low level is the tag's state during the CW before a reply,
  the high level that of the first high of a preamble.
* 6.3.1.3.2.1, Figures 6-8 to 6-10: FM0 inverts at every symbol boundary,
  a data-0 also in the middle; it ends with a dummy data-1.
* 6.3.1.3.2.2, Figure 6-11: FM0 preamble 1 0 1 0 v 1, half levels
  1 1 0 1 0 0 1 0 0 0 1 1 (read off the figure); TRext = 1 puts 12 data-0s
  (pilot tone) in front.
* 6.3.1.3.2.3, Figures 6-12 to 6-14: baseband Miller inverts between two
  data-0s and in the middle of a data-1; the waveform is the baseband times
  a square wave of M cycles a symbol; it ends with a dummy data-1.
* 6.3.1.3.2.4, Figure 6-15: Miller preamble = 4 M/BLF of subcarrier (16 M/BLF
  if TRext), then 0 1 0 1 1 1; the figure shows no inversion between the
  pilot and the first 0 (the pilot reads as if a 1 came before it).
* Table 6-9: BLF tolerance FrT (nominal temperature, Query column) by DR and
  TRcal; frequency variation during backscatter +-2.5 %.
* Table 6-10: M = 1 (FM0), 2, 4, 8.
* 6.3.1.5, Table 6-11: CRC-16, x^16 + x^12 + x^5 + 1, preset FFFFh,
  residue 1D0Fh, sent ones-complemented; Table 6-12 and Annex F.1: CRC-5,
  x^5 + x^3 + 1, preset 01001, MSB first, check residue 00000.
* 6.3.1.6, Table 6-16: T1 from MAX(RTcal, 10 Tpri) (1 - |FrT|) - 2 us to
  MAX(RTcal, 10 Tpri) (1 + |FrT|) + 2 us, from the last rising edge of the
  reader's last bit to the first rising edge of the reply; T2 3 to 20 Tpri;
  T4 2 RTcal; Tpri = the period of an FM0 symbol or of a subcarrier cycle,
  i.e. 1 / BLF.
* 6.3.2.12.2.1, Tables 6-33/6-34: Query = 1000 DR M(2) TRext Sel(2)
  Session(2) Target Q(4) CRC-5, 22 bits; a Query with a CRC-5 error is
  invalid; slot 0 replies with RN16.
* 6.3.2.12.2.6, Table 6-46: ACK = 01 RN16, after a frame-sync; a wrong RN16
  sends the tag to arbitrate without a reply. Table 6-18 / 6.3.2.1.2.1: the
  reply is PC, EPC, PacketCRC (CRC-16 over PC and EPC); PC bits 10h-14h
  (the top five) are L, the EPC length in words.
"""

from __future__ import annotations

from typing import List, Optional, Sequence, Tuple

from tools.protomodels.bench import Drive, Lines, Model, PinSpec, pad_of

DELIM_S = 12.5e-6                               # 6.3.1.2.8
PILOT_FM0 = 12                                  # Figure 6-11
FM0_PREAMBLE_HALVES = [1, 1, 0, 1, 0, 0, 1, 0, 0, 0, 1, 1]   # Figure 6-11
MILLER_PREAMBLE_BITS = [0, 1, 0, 1, 1, 1]       # Figure 6-15
FREQ_VARIATION = 0.025                          # Table 6-9


# ------------------------------------------------------------------ CRCs
def crc5(bits: Sequence[int]) -> int:
    """Annex F.1: preload 01001, clock the bits MSB first."""
    q = 0b01001
    for b in bits:
        fb = ((q >> 4) & 1) ^ b
        q = (q << 1) & 0x1F
        if fb:
            q ^= 0b01001
    return q


def crc16_bits(bits: Sequence[int]) -> int:
    """Table 6-11: the residue register (not yet complemented)."""
    c = 0xFFFF
    for b in bits:
        fb = ((c >> 15) & 1) ^ b
        c = (c << 1) & 0xFFFF
        if fb:
            c ^= 0x1021
    return c


def word_bits(w: int, n: int = 16) -> List[int]:
    return [(w >> (n - 1 - i)) & 1 for i in range(n)]


def bits_word(bits: Sequence[int]) -> int:
    v = 0
    for b in bits:
        v = (v << 1) | b
    return v


def packet_crc(words: Sequence[int]) -> int:
    """6.3.1.5: ones-complement of the CRC-16 over PC and EPC."""
    bits = [b for w in words for b in word_bits(w)]
    return crc16_bits(bits) ^ 0xFFFF


def epc_reply_words(epc: Sequence[int]) -> List[int]:
    """PC (L = EPC words, UMI = XI = T = 0, AFI 0), EPC, PacketCRC."""
    pc = (len(epc) & 0x1F) << 11
    words = [pc] + list(epc)
    return words + [packet_crc(words)]


# -------------------------------------------------------------- commands
def query_bits(dr: int = 0, m: int = 0, trext: int = 0, sel: int = 0,
               session: int = 0, target: int = 0, q: int = 0,
               bad_crc: bool = False) -> List[int]:
    """Table 6-33. ``dr`` 0 = 8, 1 = 64/3; ``m`` 0..3 = M 1, 2, 4, 8."""
    body = ([1, 0, 0, 0, dr & 1] + word_bits(m, 2) + [trext & 1]
            + word_bits(sel, 2) + word_bits(session, 2) + [target & 1]
            + word_bits(q, 4))
    c = crc5(body) ^ (1 if bad_crc else 0)
    return body + word_bits(c, 5)


def ack_bits(rn16: int) -> List[int]:
    """Table 6-46."""
    return [0, 1] + word_bits(rn16)


# ------------------------------------------------------------- link rules
def blf_hz(dr: int, trcal_s: float) -> float:
    """Eq. (1): BLF = DR / TRcal."""
    return (64.0 / 3.0 if dr else 8.0) / trcal_s


_FRT = {                       # Table 6-9, "For Query", nominal temperature
    0: [(17.2, 25, 0.19), (25, 25, 0.10), (25, 31.25, 0.12), (31.25, 31.25, 0.10),
        (31.25, 50, 0.10), (50, 50, 0.07), (50, 75, 0.07), (75, 200, 0.04)],
    1: [(33.3, 33.3, 0.15), (33.3, 66.7, 0.22), (66.7, 66.7, 0.10),
        (66.7, 83.3, 0.12), (83.3, 83.3, 0.10), (83.3, 133.3, 0.10),
        (133.3, 133.3, 0.07), (133.3, 200, 0.07), (200, 225, 0.05)],
}


def frt(dr: int, trcal_s: float) -> Optional[float]:
    """Table 6-9's tolerance for this DR and TRcal (a named TRcal matches
    within its +-1 %), or None outside the table."""
    us = trcal_s * 1e6
    for lo, hi, tol in _FRT[dr]:
        if lo == hi and abs(us - lo) <= 0.01 * lo:
            return tol
    for lo, hi, tol in _FRT[dr]:
        if lo < hi and lo < us <= hi:
            return tol
    return None


def t1_window_s(dr: int, rtcal_s: float, trcal_s: float) -> Tuple[float, float]:
    """Table 6-16: T1 min and max (Tpri = 1 / BLF, note 1)."""
    tol = frt(dr, trcal_s) or 0.0
    nom = max(rtcal_s, 10.0 / blf_hz(dr, trcal_s))
    return nom * (1 - tol) - 2e-6, nom * (1 + tol) + 2e-6


# ------------------------------------------------------------- the codes
def fm0_halves(bits: Sequence[int], level: int) -> Tuple[List[int], int]:
    """6.3.1.3.2.1: invert at every boundary, and mid-symbol for a 0."""
    out = []
    for b in bits:
        level ^= 1
        out.append(level)
        if b == 0:
            level ^= 1
        out.append(level)
    return out, level


def miller_halves(bits: Sequence[int], k: int, level: int,
                  prev: int) -> Tuple[List[int], int]:
    """6.3.1.3.2.3: the subcarrier (M = k cycles a bit, so 2k halves)
    inverts every half; the baseband inverts between two 0s and in the
    middle of a 1, which cancels that half's subcarrier inversion."""
    out = []
    for b in bits:
        for j in range(2 * k):
            sub = 1
            if j == 0 and prev == 0 and b == 0:
                sub = 0
            if j == k and b == 1:
                sub = 0
            level ^= sub
            out.append(level)
        prev = b
    return out, level


def reply_halves(bits: Sequence[int], m: int, trext: int) -> List[int]:
    """The whole reply as half-cycle levels, from the low CW level: the
    preamble (Figures 6-11, 6-15), then ``bits`` (the dummy 1 included)."""
    if m == 0:
        pre, lvl = fm0_halves([0] * PILOT_FM0, 0) if trext else ([], 0)
        pre = pre + FM0_PREAMBLE_HALVES
        lvl = pre[-1]
        data, _ = fm0_halves(bits, lvl)
        return pre + data
    k = 1 << m
    pilot = (16 if trext else 4) * 2 * k
    pre = [(i + 1) & 1 for i in range(pilot)]          # plain subcarrier
    rest, _ = miller_halves(list(MILLER_PREAMBLE_BITS) + list(bits), k,
                            pre[-1], prev=1)
    return pre + rest


def preamble_halves(m: int, trext: int) -> int:
    if m == 0:
        return 2 * PILOT_FM0 * trext + len(FM0_PREAMBLE_HALVES)
    k = 1 << m
    return ((16 if trext else 4) + len(MILLER_PREAMBLE_BITS)) * 2 * k


def decode_halves(halves: Sequence[int], m: int, trext: int, nbits: int) -> List[int]:
    """The reader's decision: after the preamble, a bit is a 1 when the
    line does not change at its middle (FM0: no mid inversion; Miller: the
    baseband inversion cancels the subcarrier's)."""
    k = 1 if m == 0 else 1 << m
    at = preamble_halves(m, trext)
    bits = []
    for i in range(nbits):
        a = at + 2 * k * i
        if a + k >= len(halves):
            break
        bits.append(1 if halves[a + k - 1] == halves[a + k] else 0)
    return bits


# ---------------------------------------------------------- the reader
class Gen2Reader(Model):
    """The reader at baseband: drives the envelope (``env``, high = CW, low
    = a modulated pulse) and records the tag's backscatter line (``bs``).
    Times are in clocks; :meth:`send` lays out one PIE frame."""

    def __init__(self, env: PinSpec, bs: PinSpec, clock_hz: float) -> None:
        self.env, self.bs = pad_of(env), pad_of(bs)
        self.clock = clock_hz
        self.level = 1
        self._changes: List[Tuple[int, int]] = []
        self.bs_edges: List[Tuple[int, int]] = []
        self._bs: Optional[int] = None

    def clocks(self, seconds: float) -> int:
        return int(round(seconds * self.clock))

    def send(self, bits: Sequence[int], at: int, tari: float, rtcal: float,
             trcal: Optional[float] = None, pw: Optional[float] = None) -> int:
        """One frame from ``at``: delimiter, data-0, RTcal, TRcal if given
        (a preamble), then PIE ``bits`` (data-1 = RTcal - Tari). Returns
        the clock of the last rising edge (T1 starts there)."""
        pw = pw if pw is not None else max(0.265 * tari, 2e-6) * 1.2
        t = at
        self._changes.append((t, 0))
        t += self.clocks(DELIM_S)
        self._changes.append((t, 1))
        syms = [tari, rtcal] + ([trcal] if trcal is not None else [])
        syms += [tari if b == 0 else rtcal - tari for b in bits]
        for s in syms:
            t += self.clocks(s - pw)
            self._changes.append((t, 0))
            t += self.clocks(pw)
            self._changes.append((t, 1))
        self._changes.sort()
        return t

    def drive(self, drive: Drive, cycle: int) -> None:
        ch = self._changes
        while ch and ch[0][0] <= cycle:
            self.level = ch.pop(0)[1]
        drive.set(self.env, self.level)

    def observe(self, lines: Lines) -> None:
        v = lines.get(self.bs)
        if self._bs is None:
            self._bs = v
            return
        if v != self._bs:
            self.bs_edges.append((lines.cycle, v))
            self._bs = v

    def edges_after(self, cycle: int) -> List[Tuple[int, int]]:
        return [e for e in self.bs_edges if e[0] > cycle]


class Reply:
    """A backscattered reply as the reader measures it."""

    def __init__(self, edges: Sequence[Tuple[int, int]], half_nominal: float):
        assert edges and edges[0][1] == 1, "a reply starts with a rising edge"
        self.t0 = edges[0][0]
        ks = [0]
        h = half_nominal
        for (a, _), (b, _) in zip(edges, edges[1:]):
            ks.append(ks[-1] + max(1, int(round((b - a) / h))))
        n = len(edges)
        if n > 1:                                   # least squares slope
            mk = sum(ks) / n
            mt = sum(t for t, _ in edges) / n
            den = sum((k - mk) ** 2 for k in ks)
            h = sum((k - mk) * (t - mt) for k, (t, _) in zip(ks, edges)) / den
        self.half = h                               # clocks a half cycle
        self.ks = ks
        self.edges = list(edges)
        self.residual = max(abs(t - (self.t0 + k * h)) for k, (t, _) in zip(ks, edges))

    def halves(self, count: int) -> List[int]:
        out, j, lvl = [], 0, 0
        for k in range(count):
            while j < len(self.ks) and self.ks[j] <= k:
                lvl = self.edges[j][1]
                j += 1
            out.append(lvl)
        return out

    def blf(self, clock_hz: float) -> float:
        return clock_hz / (2.0 * self.half)


def check_reply(reply: Reply, bits_expected: Optional[Sequence[int]], m: int,
                trext: int, nbits: int, dr: int, rtcal: float, trcal: float,
                last_rise: int, clock_hz: float) -> Tuple[List[int], List[str]]:
    """Decode ``nbits`` (dummy included) and hold the reply to the
    standard: BLF within Table 6-9, every edge on the grid within the
    frequency variation, T1 within Table 6-16, the waveform exactly the
    preamble and the code of the decoded bits, ending with the dummy 1."""
    errs = []
    nom = blf_hz(dr, trcal)
    tol = frt(dr, trcal)
    got = reply.blf(clock_hz)
    if tol is None or abs(got / nom - 1) > tol:
        errs.append("BLF %.0f Hz, nominal %.0f, tolerance %s" % (got, nom, tol))
    if reply.residual > FREQ_VARIATION * reply.half:
        errs.append("edge off the grid by %.1f clocks" % reply.residual)
    lo, hi = t1_window_s(dr, rtcal, trcal)
    t1 = (reply.t0 - last_rise) / clock_hz
    if not lo <= t1 <= hi:
        errs.append("T1 %.2f us outside %.2f..%.2f" % (t1 * 1e6, lo * 1e6, hi * 1e6))
    k = 1 if m == 0 else 1 << m
    count = preamble_halves(m, trext) + 2 * k * nbits
    halves = reply.halves(count)
    bits = decode_halves(halves, m, trext, nbits)
    if halves != reply_halves(bits, m, trext):
        errs.append("waveform is not the preamble and code of its own bits")
    if not bits or bits[-1] != 1:
        errs.append("no dummy 1 at the end")
    if bits_expected is not None and bits[:len(bits_expected)] != list(bits_expected):
        errs.append("bits differ from those expected")
    if reply.ks[-1] > count:
        errs.append("edges after the dummy bit's end")
    return bits, errs


# ------------------------------------------------------------- the tag
class Gen2Tag(Model):
    """A tag at baseband, for the reader program: reads PIE on ``env``
    (high = CW) the way 6.3.1.2.8 tells a tag to (pivot = RTcal / 2), and
    backscatters on ``bs`` after T1 = MAX(RTcal, 10 Tpri) (Table 6-16,
    nominal), at BLF = DR / TRcal exactly: Query (CRC-5 good, Q = 0) gets
    an RN16, an ACK with that RN16 gets PC + EPC + PacketCRC. It records
    every frame's measured symbols for :func:`check_frame` and the time
    from each reply's end to the next frame (T2)."""

    def __init__(self, env: PinSpec, bs: PinSpec, clock_hz: float,
                 epc: Sequence[int], rn16: int = 0x5A3C) -> None:
        self.env, self.bs = pad_of(env), pad_of(bs)
        self.clock = clock_hz
        self.epc, self.rn16 = list(epc), rn16
        self.level: Optional[int] = None         # until the line is first seen
        self.out = 0
        self.cycle = 0
        self.rises: List[int] = []
        self.falls: List[int] = []
        self.frames: List[dict] = []
        self.replies: List[Tuple[int, int]] = []        # (first edge, end)
        self.t2: List[float] = []
        self._changes: List[Tuple[int, int]] = []
        self._frame: Optional[dict] = None
        self.trcal = self.rtcal = None
        self.m = self.trext = self.dr = 0
        self.state = "arbitrate"

    def drive(self, drive: Drive, cycle: int) -> None:
        ch = self._changes
        while ch and ch[0][0] <= cycle:
            self.out = ch.pop(0)[1]
        drive.set(self.bs, self.out)

    def observe(self, lines: Lines) -> None:
        self.cycle = c = lines.cycle
        v = lines.get(self.env)
        if self.level is None:
            self.level = v
            return
        if v == self.level:
            f = self._frame
            if f and f["rises"] and c - f["rises"][-1] > 4 * f.get("rtcal", 1e9):
                self._frame = None                      # an unfinished frame
            return
        self.level = v
        if v == 0:
            if self._frame is None:
                self._frame = {"start": c, "falls": [c], "rises": [], "bits": []}
                if self.replies:
                    self.t2.append((c - self.replies[-1][1]) / self.clock)
            else:
                self._frame["falls"].append(c)
            return
        f = self._frame
        if f is None:
            return
        f["rises"].append(c)
        r = f["rises"]
        if len(r) == 3:
            f["tari"], f["rtcal"] = r[1] - r[0], r[2] - r[1]
        elif len(r) == 4 and r[3] - r[2] > f["rtcal"]:
            f["trcal"] = r[3] - r[2]
        elif len(r) >= 4:
            f["bits"].append(1 if 2 * (r[-1] - r[-2]) > f["rtcal"] else 0)
            self._command(f)

    def _command(self, f: dict) -> None:
        b = f["bits"]
        if b[:2] == [0, 1] and len(b) == 18:
            self._end(f)
            if self.state == "reply" and bits_word(b[2:]) == self.rn16:
                words = epc_reply_words(self.epc)
                if getattr(self, "epc_crc_flip", False):
                    words[-1] ^= 1                      # a damaged PacketCRC
                self._reply([x for w in words for x in word_bits(w)], f)
            else:
                self.state = "arbitrate"
        elif b[:4] == [1, 0, 0, 0] and len(b) == 22:
            self._end(f)
            if "trcal" not in f or crc5(b) != 0:
                return
            self.dr, self.m, self.trext = b[4], bits_word(b[5:7]), b[7]
            self.rtcal, self.trcal = f["rtcal"], f["trcal"]
            if bits_word(b[13:17]) == 0:
                self.state = "reply"
                self._reply(word_bits(self.rn16), f)
        elif len(b) > 22 or (len(b) >= 2 and b[:2] == [0, 0]) or b[:2] == [1, 1]:
            self._end(f)

    def _end(self, f: dict) -> None:
        self.frames.append(f)
        self._frame = None

    def _reply(self, bits: List[int], f: dict) -> None:
        half = self.trcal / (2 * (64 / 3 if self.dr else 8))
        t1 = max(self.rtcal, 20 * half)
        t0 = f["rises"][-1] + t1
        halves = reply_halves(bits + [1], self.m, self.trext) + [0]
        lvl = 0
        for k, v in enumerate(halves):
            if v != lvl:
                self._changes.append((int(round(t0 + k * half)), v))
                lvl = v
        self._changes.sort()
        self.replies.append((int(round(t0)), int(round(t0 + (len(halves) - 1) * half))))


def check_frame(f: dict, clock_hz: float) -> List[str]:
    """A reader's frame against 6.3.1.2.4, Table 6-5 and 6.3.1.2.8."""
    errs = []
    us = 1e6 / clock_hz
    tari = f["tari"] * us
    delim = (f["rises"][0] - f["falls"][0]) * us
    if not 12.5 * 0.95 <= delim <= 12.5 * 1.05:
        errs.append("delimiter %.2f us" % delim)
    if not 6.25 * 0.99 <= tari <= 25 * 1.01:
        errs.append("Tari %.2f us" % tari)
    if not 2.5 * 0.99 <= f["rtcal"] * us / tari <= 3.0 * 1.01:
        errs.append("RTcal %.2f Tari" % (f["rtcal"] * us / tari))
    if "trcal" in f and not 1.1 <= f["trcal"] / f["rtcal"] <= 3.0:
        errs.append("TRcal %.2f RTcal" % (f["trcal"] / f["rtcal"]))
    lo, hi = max(0.265 * tari, 2.0), 0.525 * tari
    for a, b in zip(f["falls"][1:], f["rises"][1:]):
        if not lo <= (b - a) * us <= hi:
            errs.append("PW %.2f us" % ((b - a) * us))
            break
    r = f["rises"]
    syms = [(b - a) * us / tari for a, b in zip(r[3 if "trcal" in f else 2:], r[4 if "trcal" in f else 3:])]
    for s in syms:
        if not (0.99 <= s <= 1.01 or 1.5 * 0.99 <= s <= 2.0 * 1.01):
            errs.append("a symbol of %.2f Tari" % s)
            break
    return errs
