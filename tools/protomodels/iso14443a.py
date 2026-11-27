"""ISO/IEC 14443 Type A (NFC-A) at 106 kbit/s, at baseband: frames, CRC_A,
the modified Miller pauses of the reader (PCD), the Manchester-on-subcarrier
load modulation of the card (PICC), and models of both ends.

Written from the sources below, never from ``firmware/nfca_card.loom`` or
``firmware/nfca_reader.loom`` (METH-1). The ISO text is paywalled; each
figure names where it was read, and the ones no readable source states are
marked "from memory, unchecked".

Sources
-------
[RC522]  NXP MFRC522 data sheet, Rev. 3.9 (27 April 2016).
         Table 4 (communication overview): at 106 kBd the reader sends 100 %
         ASK, modified Miller, bit length 128/fc; the card answers with
         subcarrier load modulation, subcarrier fc/16, Manchester.
         Figure 6: 106 kBd framing, a start bit, 8 data bits, odd parity.
         Table 48 (BitFramingReg, RxAlign / TxLastBits): bit-oriented frames
         for the bitwise anticollision. Tables 49/50 (CollReg): CollPos is
         the first collision in the received frame, data bits only.
         Table 56 (ModeReg CRCPreset): the 6363h preset.
[PM3]    Proxmark3, armsrc/iso14443a.c (github.com/Proxmark/proxmark3):
         SEC_X "drop after half a period", SEC_Y none, SEC_Z "drop at
         start"; start of communication Z; a 1 is X, a 0 is Z after a 0 and
         Y after a 1; end of communication "Logic 0 followed by Sequence
         Y". Card: D = 1 (also the start), E = 0, F = the end. The card
         answers "in 1172 or 1236" carrier periods. SEL 93h with NVB 20h,
         SELECT 93h 70h, NVB = (2 + bits / 8) << 4 | bits % 8, BCC = UID0 ^
         UID1 ^ UID2 ^ UID3, SAK bit 04h = UID not complete, parity only
         after a whole byte, REQA 26h / WUPA 52h sent as 7 bits, the reader
         takes a 1 at the collision position.
[NFC]    libnfc, libnfc/iso14443-subr.c, iso14443a_crc: preset 6363h, the
         reflected x^16 + x^12 + x^5 + 1 update, no final XOR, low byte
         first.

From memory, unchecked (ISO/IEC 14443-2 / -3 not read):
- the pause length t1 between 28/fc and 40.5/fc (the reader here uses
  32/fc, which [PM3]'s two-of-eight sub-tick mask also suggests);
- FDT = (n * 128 + 84)/fc after a last data bit 1 and (n * 128 + 20)/fc
  after a 0, n = 9, measured from the end (rising edge) of the reader's
  last pause to the card's first modulation edge, with a tolerance of
  +0.4 us; [PM3] states only the 1172 and 1236;
- the card's states (IDLE, READY, ACTIVE) and that a READY card stays
  READY and silent on a prefix or UID that is not its own;
- the card sends the split byte's parity bit after the byte's last bit,
  computed over the whole byte;
- ATQA is sent low byte first.

Baseband: no 13.56 MHz carrier. The reader's line is the field, high =
carrier, low = a pause; the card's line is its load switch, high = loaded,
with the subcarrier on-off keyed onto it.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Sequence, Tuple

from tools.protomodels.bench import Drive, Lines, Model, PinSpec, pad_of

FC = 13.56e6
BIT_FC = 128                  # [RC522] Table 4
SUB_FC = 16                   # [RC522] Table 4: fc / 16
PAUSE_FC = (28, 40.5)         # t1, from memory, unchecked
FDT_FC = {1: 9 * 128 + 84, 0: 9 * 128 + 20}     # 1236, 1172 [PM3]
FDT_TOL_S = 0.4e-6            # from memory, unchecked
REQA, WUPA = 0x26, 0x52       # [PM3]
SEL1 = 0x93                   # [PM3]


def crc_a(data: Sequence[int]) -> int:
    """CRC_A [NFC]: preset 6363h, reflected x^16 + x^12 + x^5 + 1."""
    crc = 0x6363
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return crc


def with_crc(data: Sequence[int]) -> List[int]:
    c = crc_a(data)
    return list(data) + [c & 0xFF, c >> 8]


def odd_parity(byte: int) -> int:
    """The parity bit that makes the 9 bits odd [RC522] Figure 6."""
    return 1 ^ (bin(byte & 0xFF).count("1") & 1)


def byte_bits(byte: int) -> List[int]:
    return [(byte >> i) & 1 for i in range(8)]


def std_bits(data: Sequence[int]) -> List[int]:
    """A standard frame: every byte LSB first and its odd parity."""
    out: List[int] = []
    for b in data:
        out += byte_bits(b) + [odd_parity(b)]
    return out


def short_bits(cmd: int) -> List[int]:
    """A short frame: 7 bits, no parity (REQA, WUPA) [PM3]."""
    return [(cmd >> i) & 1 for i in range(7)]


def bcc(uid: Sequence[int]) -> int:
    return uid[0] ^ uid[1] ^ uid[2] ^ uid[3]


def uid_bytes(uid: Sequence[int]) -> List[int]:
    """UID0..3 and BCC: the 40 bits of cascade level 1."""
    return list(uid) + [bcc(uid)]


def nvb(known: int) -> int:
    """NVB for SEL, NVB and ``known`` UID bits [PM3]."""
    return ((2 + known // 8) << 4) | (known % 8)


def anticoll_bits(uid_known: Sequence[int], known: int) -> List[int]:
    """ANTICOLLISION: SEL, NVB, then the first ``known`` bits of UID0..BCC
    (``uid_known``, bytes), parity only after a whole byte [PM3]."""
    out = std_bits([SEL1, nvb(known)])
    for j in range(known):
        out.append((uid_known[j // 8] >> (j % 8)) & 1)
        if j % 8 == 7:
            out.append(odd_parity(uid_known[j // 8]))
    return out


def select_bytes(uid: Sequence[int]) -> List[int]:
    return with_crc([SEL1, 0x70] + uid_bytes(uid))


def uid_reply_bits(uid: Sequence[int], known: int) -> List[int]:
    """The card's answer to an ANTICOLLISION with ``known`` bits: UID and BCC
    from bit ``known`` on, each byte's parity after its bit 7 (from memory,
    unchecked: the split byte's parity over the whole byte)."""
    ub = uid_bytes(uid)
    out = []
    for j in range(known, 40):
        out.append((ub[j // 8] >> (j % 8)) & 1)
        if j % 8 == 7:
            out.append(odd_parity(ub[j // 8]))
    return out


# ------------------------------------------------------------ modified Miller
def miller_sequence(bits: Sequence[int]) -> str:
    """The reader's sequences for a frame [PM3]: start Z, a 1 is X, a 0 is Z
    after a 0 (or first) and Y after a 1; the end is a logic 0 then Y."""
    seq, prev = "Z", 0
    for b in list(bits) + [0]:
        if b:
            seq += "X"
        else:
            seq += "Z" if prev == 0 else "Y"
        prev = b
    return seq + "Y"


def miller_pauses(bits: Sequence[int], t0: float, clock: float,
                  pause_fc: float = 32) -> Tuple[List[Tuple[int, int]], int]:
    """(fall, rise) of every pause from ``t0`` (clocks), and the last data
    bit (0 or 1), which picks the FDT [PM3]."""
    t = clock * BIT_FC / FC
    p = clock * pause_fc / FC
    out = []
    for i, s in enumerate(miller_sequence(bits)):
        if s == "Y":
            continue
        fall = t0 + i * t + (t / 2 if s == "X" else 0)
        out.append((int(round(fall)), int(round(fall + p))))
    return out, (bits[-1] if bits else 0)


def pauses_of(edges: Sequence[Tuple[int, int]]) -> List[Tuple[int, int]]:
    """(fall, rise) pairs from a line's (cycle, level) edges."""
    out, fall = [], None
    for c, v in edges:
        if v == 0:
            fall = c
        elif fall is not None:
            out.append((fall, c))
            fall = None
    return out


def split_frames(pauses: Sequence[Tuple[int, int]], clock: float,
                 gap_bits: float = 3.0) -> List[List[Tuple[int, int]]]:
    """Frames: runs of pauses with less than ``gap_bits`` bits between."""
    t = clock * BIT_FC / FC
    frames: List[List[Tuple[int, int]]] = []
    for p in pauses:
        if frames and p[0] - frames[-1][-1][0] < gap_bits * t:
            frames[-1].append(p)
        else:
            frames.append([p])
    return frames


def decode_miller(pauses: Sequence[Tuple[int, int]], clock: float,
                  tol_clocks: float = 2.0) -> Tuple[List[int], List[str]]:
    """Decode one reader frame and hold every pause to the standard: its
    length within t1 (``PAUSE_FC``, from memory, unchecked, widened by one
    clock of quantisation), its fall on the half-bit grid of the start's
    fall within ``tol_clocks`` (the chip's clock is not the carrier: the
    grid is quantised), and the sequence rules of [PM3]. Returns the data
    bits and the violations."""
    errs: List[str] = []
    t = clock * BIT_FC / FC
    lo, hi = (clock * f / FC for f in PAUSE_FC)
    f0 = pauses[0][0]
    seq: Dict[int, str] = {}
    for fall, rise in pauses:
        width = rise - fall
        if not lo - 1 <= width <= hi + 1:
            errs.append("pause at %d is %d clocks, t1 is %.1f..%.1f"
                        % (fall, width, lo, hi))
        k = int(round((fall - f0) / (t / 2)))
        off = fall - f0 - k * t / 2
        if abs(off) > tol_clocks:
            errs.append("pause at %d is %.1f clocks off the grid" % (fall, off))
        i, s = k // 2, ("X" if k % 2 else "Z")
        if i in seq:
            errs.append("two pauses in bit %d" % i)
        seq[i] = s
    last = max(seq)
    full = "".join(seq.get(i, "Y") for i in range(last + 3))
    if full[0] != "Z":
        errs.append("no start Z")
    n = last if full[last] == "X" else last - 1    # data bits 1..n
    bits = [1 if s == "X" else 0 for s in full[1:n + 1]]
    want = miller_sequence(bits)
    full = "".join(seq.get(i, "Y") for i in range(max(len(want), last + 1)))
    if full != want:
        errs.append("sequence %s is not the coding of its bits %s"
                    % (full, want))
    return bits, errs


# ------------------------------------------------ Manchester on a subcarrier
def manchester_changes(bits: Sequence[int], t0: float, clock: float
                       ) -> List[Tuple[int, int]]:
    """The card's load line for start D, ``bits``, end F [RC522] Table 4,
    [PM3]: a bit is 16 half periods h = 8/fc; D has the subcarrier (on at
    each even h, off at each odd) in its first half, E in its second."""
    t = clock * BIT_FC / FC
    h = t / 16
    out = []
    for i, b in enumerate([1] + list(bits)):
        start = t0 + i * t + (0 if b else t / 2)
        for j in range(8):
            out.append((int(round(start + j * h)), 1 - (j & 1)))
    return out


def fdt_window(last_rise: int, last_bit: int, clock: float) -> Tuple[float, float]:
    """Clocks within which the card's first edge must come: FDT after the
    end of the last pause, +0.4 us (from memory, unchecked; [PM3] 1172 and
    1236)."""
    lo = last_rise + FDT_FC[last_bit] * clock / FC
    return lo, lo + FDT_TOL_S * clock


def decode_manchester(edges: Sequence[Tuple[int, int]], clock: float,
                      strict: bool = True, tol_clocks: float = 2.0
                      ) -> Tuple[List[int], List[int], List[str], Optional[int]]:
    """Decode a card's answer from its load line's (cycle, level) edges, as a
    reader does: from the start's first rise, count the subcarrier's edges in
    each half bit (windows moved h/2 early so no edge sits on a boundary): 4
    or more of 8 is subcarrier. D = first half (1), E = second (0), both =
    a collision ([RC522] Tables 49/50: the first one is reported, data bits
    only), neither = F, the end. With ``strict`` (one card), every edge must
    be within ``tol_clocks`` of its place on the fc/16 grid and every half
    must have 8 edges or none. The card's carrier-locked grid is exact; on a
    chip clock that is not a multiple of fc an edge is quantised to the
    clock, and so is the first rise the grid is counted from: 2 clocks. Returns (bits after the start, collision
    indices, violations, the first rise)."""
    errs: List[str] = []
    rises = [c for c, v in edges if v == 1]
    if not rises:
        return [], [], ["no answer"], None
    t0 = rises[0]
    t = clock * BIT_FC / FC
    h = t / 16
    times = [c for c, _ in edges if c >= t0]
    bits: List[int] = []
    coll: List[int] = []
    i, k = 0, 0
    while True:
        counts = []
        for s in (0, 1):
            a = t0 + i * t + s * t / 2 - h / 2
            n = 0
            while k < len(times) and times[k] < a + t / 2:
                if strict:
                    j = int(round((times[k] - (a + h / 2)) / h))
                    if abs(times[k] - (a + h / 2 + j * h)) > tol_clocks:
                        errs.append("edge at %d is off the fc/16 grid by %.1f"
                                    % (times[k], times[k] - (a + h / 2 + j * h)))
                n += 1
                k += 1
            counts.append(n)
        on = [n >= 4 for n in counts]
        if strict and any(n not in (0, 8) for n in counts):
            errs.append("bit %d: %s edges in its halves, 8 or 0 each" % (i, counts))
        if not any(on):
            break
        if i == 0:
            if on != [True, False]:
                errs.append("the start is not D")
        elif all(on):
            coll.append(len(bits))
            bits.append(1)
        else:
            bits.append(1 if on[0] else 0)
        i += 1
    if k < len(times):
        errs.append("%d edges after the end" % (len(times) - k))
    return bits, coll, errs, t0


def bits_to_bytes(bits: Sequence[int], parity: bool = True
                  ) -> Tuple[List[int], List[str]]:
    """Bytes of a standard frame's bits, with a check of every parity."""
    out, errs = [], []
    step = 9 if parity else 8
    for n in range(0, len(bits) - step + 1, step):
        b = sum(bit << i for i, bit in enumerate(bits[n:n + 8]))
        out.append(b)
        if parity and bits[n + 8] != odd_parity(b):
            errs.append("parity error in byte %d" % (n // 9))
    if len(bits) % step:
        errs.append("%d bits left over" % (len(bits) % step))
    return out, errs


# ------------------------------------------------------------------ the card
IDLE, READY, ACTIVE = "IDLE", "READY", "ACTIVE"


class Picc:
    """One card's logic (states from memory, unchecked; frames [PM3]).
    ``bad_bcc`` / ``bad_crc`` make it send a wrong BCC or SAK CRC_A."""

    def __init__(self, uid: Sequence[int], atqa: int = 0x0004, sak: int = 0x08,
                 offset: int = 0, bad_bcc: bool = False, bad_crc: bool = False):
        self.uid = list(uid)
        self.atqa, self.sak, self.offset = atqa, sak, offset
        self.bad_bcc, self.bad_crc = bad_bcc, bad_crc
        self.state = IDLE

    def _ub(self) -> List[int]:
        ub = uid_bytes(self.uid)
        if self.bad_bcc:
            ub[4] ^= 0x01
        return ub

    def handle(self, bits: Sequence[int]) -> Optional[List[int]]:
        """The bits this card answers ``bits`` with, or None."""
        n = len(bits)
        if n == 7:
            cmd = sum(b << i for i, b in enumerate(bits))
            if cmd in (REQA, WUPA) and self.state in (IDLE, READY):
                self.state = READY
                return std_bits([self.atqa & 0xFF, self.atqa >> 8])
            return None
        if self.state != READY or n < 18:
            return None
        head, errs = bits_to_bytes(bits[:18])
        if errs or head[0] != SEL1:
            return None
        ub = self._ub()
        if head[1] == 0x70:
            data, errs = bits_to_bytes(bits)
            if errs or len(data) != 9 or crc_a(data) != 0:
                return None
            if data[2:7] != uid_bytes(self.uid):
                return None
            self.state = ACTIVE
            sak = with_crc([self.sak])
            if self.bad_crc:
                sak[1] ^= 0x01
            return std_bits(sak)
        nbytes, nbits = head[1] >> 4, head[1] & 15
        known = (nbytes - 2) * 8 + nbits
        if not (2 <= nbytes <= 6 and nbits <= 7 and 0 <= known < 40):
            return None
        if bits != anticoll_bits(ub, known):
            return None                         # not this card's prefix
        out = []
        for j in range(known, 40):
            out.append((ub[j // 8] >> (j % 8)) & 1)
            if j % 8 == 7:
                out.append(odd_parity(ub[j // 8]))
        return out


class CardField(Model):
    """Model cards in the reader's field: reads the reader's pause line
    (``tx``, a chip output) and drives the reader's receive line (``rx``)
    with the OR of every card's load line, as two cards' load modulation
    adds on the reader's antenna. Each card answers FDT after the end of
    the last pause (``fdt_window``'s low end) plus its ``offset`` clocks."""

    def __init__(self, tx: PinSpec, rx: PinSpec, clock: float,
                 cards: Sequence[Picc]) -> None:
        self.tx, self.rx = pad_of(tx), pad_of(rx)
        self.clock = clock
        self.cards = list(cards)
        self.t = clock * BIT_FC / FC
        self.tx_edges: List[Tuple[int, int]] = []
        self.frames: List[Tuple[List[Tuple[int, int]], List[int], List[str]]] = []
        self.answers: List[Tuple[int, List[int]]] = []    # (card, bits)
        self._prev = 0                  # the field is off until the reader drives it
        self._fall: Optional[int] = 0
        self._open: List[Tuple[int, int]] = []
        self._sched: List[List[Tuple[int, int]]] = [[] for _ in self.cards]
        self._lvl = [0] * len(self.cards)

    def drive(self, drive: Drive, cycle: int) -> None:
        v = 0
        for n, ch in enumerate(self._sched):
            while ch and ch[0][0] <= cycle:
                self._lvl[n] = ch.pop(0)[1]
            v |= self._lvl[n]
        drive.set(self.rx, v)

    def observe(self, lines: Lines) -> None:
        v = lines.get(self.tx)
        c = lines.cycle
        if v != self._prev:
            self.tx_edges.append((c, v))
            if v == 0:
                self._fall = c
            elif self._fall is not None:
                if c - self._fall > 2 * self.t:     # the field was off
                    self._open = []
                    for card in self.cards:
                        card.state = IDLE
                else:
                    self._open.append((self._fall, c))
                self._fall = None
            self._prev = v
        elif (self._open and v == 1
              and c - self._open[-1][0] > 3 * self.t):
            self._end_of_frame()

    def _end_of_frame(self) -> None:
        pauses, self._open = self._open, []
        bits, errs = decode_miller(pauses, self.clock)
        self.frames.append((pauses, bits, errs))
        last_rise = pauses[-1][1]
        last_bit = bits[-1] if bits else 0
        for n, card in enumerate(self.cards):
            ans = card.handle(bits)
            if ans is None:
                continue
            self.answers.append((n, ans))
            t0 = fdt_window(last_rise, last_bit, self.clock)[0] + card.offset
            self._sched[n] = manchester_changes(ans, t0, self.clock)


# ---------------------------------------------------------------- the reader
class Pcd(Model):
    """A model reader for the card: drives the field line ``field`` (a chip
    input: high = carrier, low = a pause) and records the card's load line
    ``lm``. :meth:`send` lays out one frame; :meth:`answer` decodes what
    came after it and holds it to FDT and the fc/16 grid."""

    def __init__(self, field: PinSpec, lm: PinSpec, clock: float,
                 pause_fc: float = 32) -> None:
        self.field, self.lm = pad_of(field), pad_of(lm)
        self.clock = clock
        self.pause_fc = pause_fc
        self.level = 1
        self._changes: List[Tuple[int, int]] = []
        self.lm_edges: List[Tuple[int, int]] = []
        self._lm = 0
        self.last: Tuple[int, int] = (0, 0)

    def bit_clocks(self) -> float:
        return self.clock * BIT_FC / FC

    def send(self, bits: Sequence[int], at: int) -> int:
        """One frame from ``at``; returns the end of its last pause."""
        pauses, last_bit = miller_pauses(bits, at, self.clock, self.pause_fc)
        for f, r in pauses:
            self._changes += [(f, 0), (r, 1)]
        self._changes.sort()
        self.last = (pauses[-1][1], last_bit)
        return pauses[-1][1]

    def drive(self, drive: Drive, cycle: int) -> None:
        ch = self._changes
        while ch and ch[0][0] <= cycle:
            self.level = ch.pop(0)[1]
        drive.set(self.field, self.level)

    def observe(self, lines: Lines) -> None:
        v = lines.get(self.lm)
        if v != self._lm:
            self.lm_edges.append((lines.cycle, v))
            self._lm = v

    def answer(self) -> Tuple[List[int], List[str], Optional[float]]:
        """The answer to the last frame: (bits, violations, FDT in clocks)."""
        last_rise, last_bit = self.last
        edges = [e for e in self.lm_edges if e[0] > last_rise]
        bits, coll, errs, t0 = decode_manchester(edges, self.clock)
        if t0 is None:
            return [], errs, None
        lo, hi = fdt_window(last_rise, last_bit, self.clock)
        if not lo <= t0 <= hi:
            errs.append("first edge %d clocks after the last pause, FDT window "
                        "%.1f..%.1f" % (t0 - last_rise, lo - last_rise, hi - last_rise))
        if coll:
            errs.append("collisions at %s" % coll)
        return bits, errs, t0 - last_rise

