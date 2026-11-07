"""POCSAG (Radio-paging code No. 1) reference model: codewords, pages, a bit
player, a line recorder with a timing checker, and a decoder.

Written from the standards, never from ``firmware/pocsag_*.loom``:

* **ITU-R M.584-2** (11/1997), "Codes and formats for radio paging", Annex 1
  "Radio-paging code No. 1" (POCSAG), fetched 2026-11-07 from itu.int and
  read through its text layer:

  - 1 and 1.1: a transmission is a preamble and then batches; the preamble
    is reversals ``101010...`` for at least 576 bits (a batch plus a
    codeword); transmission may cease at the end of a batch.
  - 1.2: a batch is a synchronisation codeword and 8 frames of 2 codewords;
    a pager is in frame ``RIC & 7`` (the 3 least significant bits of its
    21-bit identity) and its address codewords go only in that frame;
    message codewords follow the address directly, may run into the next
    batch (the sync codeword is never displaced) and end at the next
    address or idle codeword; every codeword with nothing to carry is idle,
    and the last codeword of a transmission should be idle.
  - 1.3: 32-bit codewords, most significant bit (bit 1) first.
    1.3.1 Table 1, the sync codeword ``0x7CD215D8``. 1.3.2 an address
    codeword: bit 1 = 0, bits 2-19 the 18 most significant bits of the
    identity, bits 20-21 the function bits, bits 22-31 the check bits,
    bit 32 even parity over the codeword. 1.3.3 a message codeword: bit 1 =
    1, bits 2-21 message bits. 1.3.4 Table 2, the idle codeword, "a valid
    address codeword" (see :data:`IDLE`).
  - 1.4: the 21 information bits are the coefficients of x^30 .. x^10,
    divided modulo 2 by x^10 + x^9 + x^8 + x^6 + x^5 + x^3 + 1; the
    remainder (x^9 .. x^0) gives bits 22-31; bit 32 makes the parity even.
  - 2.1: numeric messages: function bits 00; Table 3, 4-bit characters
    0-9, 1010 spare, 1011 "U" (urgency), 1100 space, 1101 hyphen, 1110 and
    1111 the closing and opening brackets (their glyphs are images in the
    PDF and were not read: :data:`NUMERIC` uses ``]`` and ``[``, from
    memory, unchecked); each character's bits are sent "in numerical
    order starting with bit No. 1" (least significant first), 5 characters
    to a codeword, the unused end of the last filled with spaces.
  - 2.2: alphanumeric: function bits 11, 7-bit IA5 characters, bit 1 first,
    packed across codewords in 20-bit blocks (encoded here; the firmware
    sends numeric only).
  - 2.5.1: a pager stops decoding a message at an idle or address codeword
    or after two successive indecipherable codewords.

* **ITU-R M.539-3** (09/1994, withdrawn 2007), "Technical and operational
  characteristics of international radio-paging systems", fetched the same
  day, section 4.3: data rate 512 bit/s or 1 200 bit/s "with an accuracy of
  +- 1 x 10^-5"; direct FSK, non-return-to-zero, positive shift for a 0.

M.584-2 itself gives no bit rate. The logic level here is the data bit (1
high), what a transmitter's data input takes before the FSK modulator puts
the 1 on the negative shift; a receiver's data slicer gives the same.
"""

from __future__ import annotations

import dataclasses
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from tools.protomodels.bench import Drive, Lines, Model, PinSpec, pad_of

# ------------------------------------------------------------ the code
#: M.584-2 Annex 1 1.4: x^10 + x^9 + x^8 + x^6 + x^5 + x^3 + 1.
GENERATOR = 0b111_0110_1001
#: 1.3.1 Table 1.
SYNC = 0x7CD215D8
#: 1.3.4 Table 2. The PDF's table reads 0111 1010 1100 1001 1100 0001 1001
#: 0111 (0x7AC9C197), which is not a codeword: its syndrome is 494 and its
#: parity odd, against 1.3.4's "a valid address codeword". 0x7A89C197, one
#: bit away (bit 10), is valid and is the value the brief and the paging
#: industry use; QUESTIONS.md asks the director to confirm.
IDLE = 0x7A89C197
IDLE_AS_PRINTED = 0x7AC9C197
PREAMBLE_BITS = 576                     # 1.1, at least
RATES = (512.0, 1200.0)                 # M.539-3 4.3
RATE_TOLERANCE = 1e-5                   # M.539-3 4.3
FRAMES = 8
BATCH_CODEWORDS = 16
FUNC_NUMERIC, FUNC_ALPHA = 0, 3         # 2.1, 2.2

#: 2.1 Table 3, value -> character. 0xA is "spare" (shown here as '*').
NUMERIC = "0123456789*U -]["
SPACE = 0xC


def bch_remainder(info21: int) -> int:
    """1.4: the 10 check bits of a 21-bit information field (bit 1 of the
    codeword is the field's bit 20)."""
    v = (info21 & 0x1FFFFF) << 10
    for i in range(30, 9, -1):
        if v >> i & 1:
            v ^= GENERATOR << (i - 10)
    return v


def ones(v: int) -> int:
    return bin(v).count("1")


def codeword(info21: int) -> int:
    """The 32-bit codeword: information, check bits, even parity."""
    cw31 = ((info21 & 0x1FFFFF) << 10) | bch_remainder(info21)
    return (cw31 << 1) | (ones(cw31) & 1)


def syndrome(cw: int) -> int:
    """Remainder of bits 1-31 divided by the generator (0 for a codeword)."""
    v = (cw >> 1) & 0x7FFFFFFF
    for i in range(30, 9, -1):
        if v >> i & 1:
            v ^= GENERATOR << (i - 10)
    return v


def is_valid(cw: int) -> bool:
    return syndrome(cw) == 0 and ones(cw) % 2 == 0


def correct(cw: int) -> Tuple[Optional[int], int]:
    """Single-bit error correction with the parity bit as a check.

    Returns ``(codeword, errors)``: ``errors`` 0 or 1 with the corrected
    word, or ``(None, 2)`` when the syndrome and parity show more than one
    error (or one that no single flip explains)."""
    if is_valid(cw):
        return cw, 0
    for j in range(32):
        c = cw ^ (1 << j)
        if is_valid(c):
            return c, 1
    return None, 2


def address_codeword(ric: int, func: int = FUNC_NUMERIC) -> int:
    """1.3.2: flag 0, the 18 MSBs of the 21-bit RIC, the function bits."""
    return codeword(((ric >> 3) & 0x3FFFF) << 2 | (func & 3))


def message_codeword(data20: int) -> int:
    """1.3.3: flag 1 and 20 message bits (bit 2 is ``data20``'s bit 19)."""
    return codeword(1 << 20 | (data20 & 0xFFFFF))


def is_address(cw: int) -> bool:
    return not cw >> 31


def address_fields(cw: int) -> Tuple[int, int]:
    """``(the 18 address bits, the function bits)`` of an address codeword."""
    return (cw >> 13) & 0x3FFFF, (cw >> 11) & 3


def data20(cw: int) -> int:
    """Bits 2-21 of a codeword, bit 2 as the most significant."""
    return (cw >> 11) & 0xFFFFF


# ------------------------------------------------------------ messages
def numeric_codes(text: str) -> List[int]:
    """2.1 Table 3: characters to 4-bit codes."""
    return [NUMERIC.index(ch) for ch in text]


def pack_bits(bits: Sequence[int]) -> List[int]:
    """Bits in transmission order into 20-bit message fields (the first bit
    is bit 2 of the codeword, the field's bit 19)."""
    assert len(bits) % 20 == 0
    out = []
    for i in range(0, len(bits), 20):
        v = 0
        for b in bits[i:i + 20]:
            v = v << 1 | b
        out.append(v)
    return out


def numeric_fields(codes: Sequence[int]) -> List[int]:
    """2.1: each code least significant bit first, 5 to a codeword, the
    last codeword filled with spaces."""
    codes = list(codes) + [SPACE] * (-len(codes) % 5)
    return pack_bits([(c >> k) & 1 for c in codes for k in range(4)])


def numeric_text(fields: Sequence[int]) -> str:
    """The inverse of :func:`numeric_fields`, pad spaces included."""
    out = []
    for f in fields:
        for d in range(5):
            nib = (f >> (16 - 4 * d)) & 0xF            # bits as sent
            code = sum(((nib >> (3 - k)) & 1) << k for k in range(4))
            out.append(NUMERIC[code])
    return "".join(out)


def alpha_fields(text: str, fill: int = 0x04) -> List[int]:
    """2.2: 7-bit characters, bit 1 first, contiguous 20-bit blocks; the
    end is filled with ``fill`` (EOT) characters, the last one cut."""
    bits = [(ord(ch) >> k) & 1 for ch in text for k in range(7)]
    end = len(bits) + (-len(bits) % 20)
    bits += [(fill >> k) & 1 for k in range(7)] * 3
    return pack_bits(bits[:end])


# ----------------------------------------------------------- transmission
def batches(ric: int, func: int, fields: Sequence[int]) -> List[List[int]]:
    """1.2: the batches (16 codewords each, sync not included) for one page.

    Idle until the pager's frame, the address in the frame's first
    codeword (either codeword of the frame is allowed; this encoder takes
    the first), the message codewords straight after, across batches if
    needed, then idle to the end of the batch, with at least one idle
    after the message (a full batch of idle if the message ends a batch)."""
    stream = [address_codeword(ric, func)] + [message_codeword(f) for f in fields]
    cws = [IDLE] * (2 * (ric & 7)) + stream + [IDLE]
    cws += [IDLE] * (-len(cws) % BATCH_CODEWORDS)
    return [cws[i:i + 16] for i in range(0, len(cws), 16)]


def word_bits(cw: int) -> List[int]:
    return [(cw >> (31 - k)) & 1 for k in range(32)]


def transmission(ric: int, func: int, fields: Sequence[int],
                 preamble: int = PREAMBLE_BITS) -> List[int]:
    """1.1 and 1.2: the bits of a whole transmission, in order."""
    bits = [1 - (k & 1) for k in range(preamble)]            # 1010...
    for batch in batches(ric, func, fields):
        bits += word_bits(SYNC)
        for cw in batch:
            bits += word_bits(cw)
    return bits


# ---------------------------------------------------- host words (Loom)
def tx_host_words(ric: int, func: int, codes: Sequence[int]) -> List[int]:
    """INQ words for ``firmware/pocsag_tx.loom``: RIC[15:0]; then n (the
    number of 4-bit characters) in bits 15:8, the function bits in 6:5 and
    RIC[20:16] in 4:0; then the characters four to a word, the first in
    bits 3:0 (so the words read least significant bit first are the
    message bits in transmission order)."""
    n = len(codes)
    words = [ric & 0xFFFF, (n & 0xFF) << 8 | (func & 3) << 5 | (ric >> 16) & 0x1F]
    for i in range(0, n, 4):
        w = 0
        for j, c in enumerate(codes[i:i + 4]):
            w |= (c & 0xF) << (4 * j)
        words.append(w)
    return words


KIND_ADDRESS, KIND_MESSAGE, KIND_BAD = 1, 2, 3


@dataclasses.dataclass
class Report:
    """One report of ``firmware/pocsag_rx.loom`` (two OUTQ words)."""

    kind: int                   # 1 our address, 2 message codeword, 3 indecipherable
    corrected: bool             # a single-bit error was corrected
    overflow: bool              # reports were lost before this one (OUTQ full)
    index: int                  # codeword position in its batch, 0..15
    data: int                   # bits 2-21 of the codeword

    @property
    def func(self) -> int:
        return self.data & 3

    def words(self, flag: int) -> List[int]:
        """The two words, for a codeword with flag bit ``flag``."""
        cw = (flag & 1) << 31 | (self.data & 0xFFFFF) << 11
        return [cw >> 16, (cw & 0xF800) | (self.index & 15) << 5 | self.overflow << 4
                | (self.kind & 3) << 2 | self.corrected << 1]


def parse_reports(words: Sequence[int]) -> List[Report]:
    """Word A: codeword bits 1-16; word B: bits 17-21 in 15:11, then the
    position in 8:5, overflow 4, kind 3:2, corrected 1."""
    out = []
    for a, b in zip(words[0::2], words[1::2]):
        out.append(Report(b >> 2 & 3, bool(b >> 1 & 1), bool(b >> 4 & 1),
                          b >> 5 & 0xF, (a & 0x7FFF) << 5 | b >> 11))
    return out


# ------------------------------------------------------------ the decoder
@dataclasses.dataclass
class Received:
    batch: int                  # 0, 1, ... in the order the syncs were found
    index: int                  # 0..15 within the batch
    raw: int                    # as received
    word: Optional[int]         # corrected (None: indecipherable)
    errors: int                 # 0, 1, or 2 (more than one)


def decode(bits: Sequence[int]) -> List[Received]:
    """A pager's view of a bit stream: a bit-by-bit search for the sync
    codeword (exact match), 16 codewords, then the next sync must follow
    or the search starts again (1.2, 2.3). Every codeword is corrected
    when one error explains it (:func:`correct`)."""
    out: List[Received] = []
    window, i, batch, n = 0, 0, -1, len(bits)
    while i < n:
        window = ((window << 1) | bits[i]) & 0xFFFFFFFF
        i += 1
        if window != SYNC or i < 32:
            continue
        while True:
            batch += 1
            if i + 16 * 32 > n:
                rest = (n - i) // 32
            else:
                rest = 16
            for k in range(rest):
                raw = 0
                for b in bits[i:i + 32]:
                    raw = raw << 1 | b
                i += 32
                word, errors = correct(raw)
                out.append(Received(batch, k, raw, word, errors))
            if rest < 16 or i + 32 > n:
                return out
            nxt = 0
            for b in bits[i:i + 32]:
                nxt = nxt << 1 | b
            if nxt != SYNC:
                window = 0
                break
            i += 32
    return out


def pages(received: Sequence[Received], ric: int) -> List[Tuple[int, List[int]]]:
    """``[(function bits, [message fields])]`` for each page addressed to
    ``ric`` in its own frame (1.2), ended by an idle or address codeword or
    two indecipherable codewords in a row (2.5.1)."""
    out: List[Tuple[int, List[int]]] = []
    current: Optional[List[int]] = None
    bad = 0
    for r in received:
        w = r.word
        if w is None:
            bad += 1
            if bad >= 2:
                current = None
            continue
        bad = 0
        if is_address(w):
            current = None
            if w == IDLE:
                continue
            addr, func = address_fields(w)
            if addr == (ric >> 3) & 0x3FFFF and r.index // 2 == ric & 7:
                current = []
                out.append((func, current))
        elif current is not None:
            current.append(data20(w))
    return out


def pager_reports(received: Sequence[Received], ric: int) -> List[Report]:
    """What a pager with ``ric`` shows, codeword by codeword, under the
    rules of 1.2 that :func:`pages` applies: its address in its frame
    (kind 1), the message codewords after it (kind 2) and, inside a
    message, the indecipherable ones (kind 3, the raw bits), each marked
    when one error was corrected. Messages also end where a sync was lost
    (``Received.batch`` restarting is not visible here, so a test that
    loses sync checks that separately)."""
    out: List[Report] = []
    inmsg = False
    for r in received:
        w = r.word
        if w is None:
            if inmsg:
                out.append(Report(KIND_BAD, False, False, r.index, data20(r.raw)))
            continue
        fixed = r.errors == 1
        if is_address(w):
            inmsg = False
            if w == IDLE:
                continue
            addr, _ = address_fields(w)
            if addr == (ric >> 3) & 0x3FFFF and r.index // 2 == ric & 7:
                inmsg = True
                out.append(Report(KIND_ADDRESS, fixed, False, r.index, data20(w)))
        elif inmsg:
            out.append(Report(KIND_MESSAGE, fixed, False, r.index, data20(w)))
    return out


# --------------------------------------------------------- pin models
class Player(Model):
    """Drives one NRZ line with bits at ``bit_clocks`` clocks a bit (any
    real number; edges sit at the rounded running sum, so long streams do
    not drift). ``flips`` inverts the bits at those indices (bit errors).
    The line rests at ``idle`` before and after."""

    def __init__(self, pin: PinSpec, *, idle: int = 0) -> None:
        self.pad = pad_of(pin)
        self.level = idle
        self.idle_level = idle
        self._events: List[Tuple[int, int]] = []
        self._now = 0

    def play(self, bits: Sequence[int], bit_clocks: float, *, lead: int = 8,
             flips: Iterable[int] = ()) -> None:
        flips = set(flips)
        t0 = self._now + lead
        for k, b in enumerate(bits):
            self._events.append((t0 + int(round(k * bit_clocks)), b ^ (k in flips)))
        self._events.append((t0 + int(round(len(bits) * bit_clocks)), self.idle_level))

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
    """A board wire from a chip output to a chip input (one clock late, as a
    registered model is)."""

    def __init__(self, src: PinSpec, dst: PinSpec) -> None:
        self.src, self.dst = pad_of(src), pad_of(dst)
        self.level = 0

    def drive(self, drive: Drive, cycle: int) -> None:
        drive.set(self.dst, self.level)

    def observe(self, lines: Lines) -> None:
        self.level = lines.get(self.src)


class Line(Model):
    """Records a transmitter's data line (and its key line, if it has one)
    and checks the timing against M.539-3 4.3.

    The first rising edge of the data line is the start of the preamble's
    first bit (a 1, 1.1). Bits are read at the middle of each bit time,
    ``t0 + (k + 1/2) T`` with ``T = clock / rate``, while the key line is
    high (or to one bit after the last edge without a key line)."""

    def __init__(self, data: PinSpec, key: Optional[PinSpec] = None) -> None:
        self.pad = pad_of(data)
        self.key_pad = pad_of(key) if key is not None else None
        self.edges: List[Tuple[int, int]] = []          # (cycle, new level)
        self.key_edges: List[Tuple[int, int]] = []
        self.level = self.key = 0

    def observe(self, lines: Lines) -> None:
        v = lines.get(self.pad)
        if v != self.level:
            self.level = v
            self.edges.append((lines.cycle, v))
        if self.key_pad is not None:
            k = lines.get(self.key_pad)
            if k != self.key:
                self.key = k
                self.key_edges.append((lines.cycle, k))

    @property
    def t0(self) -> Optional[int]:
        return next((c for c, v in self.edges if v), None)

    def bits(self, bit_clocks: float, nbits: Optional[int] = None) -> List[int]:
        t0 = self.t0
        if t0 is None:
            return []
        if nbits is None:
            end = self.key_edges[-1][0] if self.key_edges and not self.key_edges[-1][1] \
                else self.edges[-1][0] + bit_clocks
            nbits = int((end - t0) / bit_clocks + 0.25)
        out, j, level = [], 0, 0
        for k in range(nbits):
            t = t0 + (k + 0.5) * bit_clocks
            while j < len(self.edges) and self.edges[j][0] <= t:
                level = self.edges[j][1]
                j += 1
            out.append(level)
        return out

    def check(self, bit_clocks: float, tol_clocks: float = 1.0,
              nominal: Optional[float] = None) -> List[str]:
        """Every data edge on the grid of ``bit_clocks`` (the bit the
        transmitter was set to) within ``tol_clocks``; the rate measured
        over the whole transmission within M.539-3's +- 1e-5 of
        ``nominal`` (default ``bit_clocks``); the key line up before the
        first bit. Edges after the key goes down are not on the air."""
        nominal = bit_clocks if nominal is None else nominal
        bad: List[str] = []
        t0 = self.t0
        if t0 is None:
            return ["no transmission"]
        off = self.key_span(bit_clocks)
        edges = self.edges if off is None else \
            [e for e in self.edges if e[0] < t0 + off * bit_clocks]   # key down: not sent
        for c, _ in edges:
            k = round((c - t0) / bit_clocks)
            if abs(c - t0 - k * bit_clocks) > tol_clocks:
                bad.append("edge at %d is %.1f clocks off the grid (bit %d)"
                           % (c, c - t0 - k * bit_clocks, k))
        last = edges[-1][0]
        k = round((last - t0) / bit_clocks)
        if k:
            measured = (last - t0) / k
            if abs(measured / nominal - 1) > RATE_TOLERANCE:
                bad.append("bit time %.4f clocks, %.2e from %.4f"
                           % (measured, measured / nominal - 1, nominal))
        if self.key_pad is not None and (not self.key_edges or self.key_edges[0][0] > t0):
            bad.append("key line not up before the first bit")
        return bad

    def key_span(self, bit_clocks: float) -> Optional[float]:
        """Bit times from the first bit's start to the key line's fall."""
        if not self.key_edges or self.key_edges[-1][1] or self.t0 is None:
            return None
        return (self.key_edges[-1][0] - self.t0) / bit_clocks
