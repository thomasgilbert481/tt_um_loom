"""AX.25 over Bell 202 AFSK reference models for ``firmware/ax25_tx.loom``
and ``firmware/ax25_rx.loom``: a UI frame builder, the FCS, the bit chain
(zero-bit insertion, flags, NRZI, tones) and a reference decoder chain
(tone timing to bits, NRZI, flags, unstuffing, FCS), with a timing checker.

Written from the specifications, not from the firmware.

Sources (read 2026-11-07 from the TAPR PDF unless marked *unchecked*):

* AX.25 Link Access Protocol for Amateur Packet Radio, version 2.2 (TAPR,
  July 1998):
  3.1 the flag 01111110, which may be shared by two frames;
  3.4 PID 0xF0 = no layer 3 protocol;
  3.6 zero-bit insertion: a 0 after every five contiguous 1s, deleted on
  receive;
  3.7 the FCS is computed as ISO 3309 recommends;
  3.8 every octet least significant bit first, "the FCS field ... most-
  significant bit first";
  3.9 invalid: fewer than 136 bits flags included, not octet-aligned;
  3.10 abort: at least fifteen contiguous 1s;
  3.11 time fill: contiguous flags;
  3.12, 3.12.2, 3.12.3 the address field: destination then source, each a
  callsign of six characters (upper-case letters and digits, padded with
  spaces) and an SSID octet CRRSSSS0, every octet shifted left one bit,
  bit 0 the extension bit, 1 only on the last address octet; R = 1 when
  not used;
  4.3.3.6 (figure 4.4) the UI frame, control 000P0011;
  6.1.2 (figure 6.1) a command frame has C = 1 in the destination SSID
  octet and C = 0 in the source's.
* ISO 3309 / ISO/IEC 13239 FCS-16 as RFC 1662 C.2 gives it: generator
  x^16 + x^12 + x^5 + 1, preset to all ones, ones complement sent, and in
  the reflected form 0x8408 the low octet first. The catalogue check value
  of this CRC (CRC-16/X-25, also IBM-SDLC) for the ASCII "123456789" is
  0x906E and its residue 0xF0B8 (from memory, *unchecked*; the residue is
  RFC 1662's PPPGOODFCS16, which tools/protomodels/hdlc.py cites).
* AX.25 does not define the physical layer. 1200 bit/s packet uses Bell
  202 tones (mark 1200 Hz, space 2200 Hz) with NRZI, a 0 a change of tone
  and a 1 none (the TNC-2 and APRS Protocol Reference 1.0 convention;
  *unchecked*, from memory). Bell 202 itself: tools/protomodels/bell202.py.

Conventions here: a "tone" is 1 for mark, 0 for space; the firmware's
NRZI line high is mark (either sense decodes the same, NRZI being
transition-coded).
"""

from __future__ import annotations

import bisect
from typing import List, Optional, Sequence, Tuple

from . import bell202 as B

FLAG = 0x7E
UI = 0x03                       # 4.3.3.6: control 000P0011, P = 0
PID_NONE = 0xF0                 # 3.4: no layer 3
MIN_BITS = 136                  # 3.9: shorter, flags included, is invalid
CHECK = 0x906E                  # CRC-16/X-25 of b"123456789" (unchecked)
RESIDUE = 0xF0B8                # the reflected register after a good frame
BIT_S = B.BIT_S


# ------------------------------------------------------------ the frame
def fcs16(data: Sequence[int]) -> int:
    """ISO 3309 FCS-16 of ``data`` (octets in transmission order), in the
    reflected form: bit 0 is the first bit sent (the x^15 coefficient)."""
    crc = 0xFFFF
    for octet in data:
        crc ^= octet
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return crc ^ 0xFFFF


def with_fcs(data: Sequence[int]) -> bytes:
    """The octets and their FCS, low octet first (each octet LSB first on
    the wire, so the FCS goes x^15 coefficient first, 3.8)."""
    f = fcs16(data)
    return bytes(data) + bytes([f & 0xFF, f >> 8])


def fcs_good(octets: Sequence[int]) -> bool:
    """A frame with its FCS: the register over all of it is the residue."""
    crc = 0xFFFF
    for octet in octets:
        crc ^= octet
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return crc == RESIDUE


def ssid_octet_raw(ssid: int, c: int) -> int:
    """The SSID octet before the shift: C, R = 1, R = 1, SSID (3.12.2)."""
    assert 0 <= ssid <= 15 and c in (0, 1)
    return (c << 6) | 0x30 | ssid


def address_raw(call: str, ssid: int, c: int) -> bytes:
    """One address subfield, 7 octets, before the shift (3.12.2, 3.12.3)."""
    call = call.upper()
    assert 1 <= len(call) <= 6 and all(ch.isalnum() for ch in call), call
    return call.ljust(6).encode("ascii") + bytes([ssid_octet_raw(ssid, c)])


def address_field(dest: str, dest_ssid: int, src: str, src_ssid: int, *,
                  command: bool = True) -> bytes:
    """The 14 address octets on the wire: shifted left one bit, the
    extension bit 1 on the last only (3.12); C bits per 6.1.2."""
    cd, cs = (1, 0) if command else (0, 1)
    raw = address_raw(dest, dest_ssid, cd) + address_raw(src, src_ssid, cs)
    out = bytearray((o << 1) & 0xFF for o in raw)
    out[-1] |= 1
    return bytes(out)


def ui_frame(dest: str, dest_ssid: int, src: str, src_ssid: int,
             info: Sequence[int] = b"", *, command: bool = True) -> bytes:
    """A UI frame between the flags, without the FCS."""
    return (address_field(dest, dest_ssid, src, src_ssid, command=command)
            + bytes([UI, PID_NONE]) + bytes(info))


def host_words(dest: str, dest_ssid: int, src: str, src_ssid: int,
               info: Sequence[int] = b"", *, command: bool = True) -> List[int]:
    """ax25_tx.loom's INQ words (its header): n, the 14 address octets
    before the shift two a word, the information two a word."""
    cd, cs = (1, 0) if command else (0, 1)
    raw = address_raw(dest, dest_ssid, cd) + address_raw(src, src_ssid, cs)
    info = bytes(info)
    assert len(info) <= 240
    words = [len(info)]
    words += [raw[i] | raw[i + 1] << 8 for i in range(0, 14, 2)]
    padded = info + b"\0" * (len(info) & 1)
    words += [padded[i] | padded[i + 1] << 8 for i in range(0, len(padded), 2)]
    return words


# ---------------------------------------------------------- the bit chain
def octet_bits(data: Sequence[int]) -> List[int]:
    """Every octet LSB first (3.8; the FCS too, given low octet first)."""
    return [(o >> i) & 1 for o in data for i in range(8)]


FLAG_BITS = octet_bits([FLAG])


def stuff(bits: Sequence[int]) -> List[int]:
    """Zero-bit insertion (3.6): a 0 after every five contiguous 1s."""
    out, ones = [], 0
    for b in bits:
        out.append(b)
        ones = ones + 1 if b else 0
        if ones == 5:
            out.append(0)
            ones = 0
    return out


def frame_line_bits(octets_with_fcs: Sequence[int], *, lead_flags: int = 1,
                    tail_flags: int = 1) -> List[int]:
    """Flags, the stuffed frame, flags: the bits before NRZI."""
    return (FLAG_BITS * lead_flags + stuff(octet_bits(octets_with_fcs))
            + FLAG_BITS * tail_flags)


def nrzi(bits: Sequence[int], tone: int = 1) -> List[int]:
    """The tone of each bit: a 0 changes it, a 1 keeps it."""
    out = []
    for b in bits:
        if not b:
            tone ^= 1
        out.append(tone)
    return out


def un_nrzi(tones: Sequence[int], tone: int = 1) -> List[int]:
    """The bits from the tones: no change is a 1."""
    out = []
    for t in tones:
        out.append(1 if t == tone else 0)
        tone = t
    return out


class Frame:
    """What lay between two flags: the destuffed bits, or an abort."""

    def __init__(self, bits: List[int], aborted: bool = False) -> None:
        self.bits, self.aborted = bits, aborted

    @property
    def whole(self) -> bool:
        return not self.aborted and len(self.bits) % 8 == 0

    @property
    def octets(self) -> bytes:
        n = len(self.bits) // 8
        return bytes(sum(self.bits[8 * k + i] << i for i in range(8))
                     for k in range(n))

    @property
    def good(self) -> bool:
        """Whole, long enough (3.9: 136 bits with both flags) and its FCS."""
        return (self.whole and len(self.bits) + 16 >= MIN_BITS
                and fcs_good(self.octets))

    @property
    def data(self) -> bytes:
        """The octets without the FCS."""
        return self.octets[:-2]


def deframe(bits: Sequence[int]) -> List[Frame]:
    """HDLC receive on a destuffed-to-be bit stream: flags (shared ones
    included, 3.1), zero-bit deletion (3.6), aborts (seven 1s; 3.10 sends
    fifteen or more). Flags with nothing between them give no frame."""
    frames: List[Frame] = []
    cur: Optional[List[int]] = None     # None: hunting for a flag
    ones = 0
    for b in bits:
        if b:
            ones += 1
            if ones == 7 and cur is not None:
                if len(cur) > 5:
                    frames.append(Frame(cur[:-5], aborted=True))
                cur = None
            elif cur is not None and ones <= 5:
                cur.append(1)
            continue
        if ones == 6:                   # a flag
            if cur is not None and len(cur) > 6:
                frames.append(Frame(cur[:-6]))   # its 0 and five 1s were taken
            cur = []
        elif ones == 5:
            pass                        # a stuff bit
        elif cur is not None:
            cur.append(0)
        ones = 0
    return frames


# ------------------------------------------------- tones, the reference way
def tones_at(times: Sequence[float], t0: float, n: int) -> List[int]:
    """The tone at the centre of each of ``n`` bits from ``t0``: the tone
    of the half-period that holds the centre (B.tone_of)."""
    hp = B.half_periods(times)
    starts = [a for a, _ in hp]
    out = []
    for k in range(n):
        t = t0 + (k + 0.5) * BIT_S
        i = bisect.bisect_right(starts, t) - 1
        if i < 0 or i >= len(hp):
            break
        out.append(B.tone_of(hp[i][1]))
    return out


def fit_grid(times: Sequence[float], *, fit_bits: int = 48) -> Tuple[float, float]:
    """The bit grid of a phase-continuous signal with one bit clock: try a
    start every 1/66 bit across one bit after the first edge, read the
    tones at the centres of the next ``fit_bits`` bits, and keep the start
    whose ideal continuous-phase wave (B.phase_error) the edges match best;
    then refine. Returns ``(t0, worst edge error in seconds)`` over the
    fitted stretch."""
    first = times[0]
    window = [t for t in times if t < first + (fit_bits + 1) * BIT_S]

    def score(t0):
        tones = tones_at(times, t0, fit_bits)
        edges = [t for t in window if t0 <= t < t0 + len(tones) * BIT_S]
        return B.phase_error(edges, tones, t0), t0

    best = min(score(first + k * BIT_S / 66) for k in range(66))
    t0 = best[1]
    step = BIT_S / 66
    best = min(score(t0 + j * step / 20) for j in range(-20, 21))
    return best[1], best[0]


def decode_tones(times: Sequence[float], t0: Optional[float] = None
                 ) -> Tuple[float, List[int]]:
    """Every bit's tone from the edge times: ``(t0, tones)``."""
    if t0 is None:
        t0, _ = fit_grid(times)
    n = int((times[-1] - t0) / BIT_S)
    return t0, tones_at(times, t0, n)


def decode(times: Sequence[float]) -> Tuple[float, List[int], List[Frame]]:
    """The reference receiver: tones, NRZI, frames. ``(t0, bits, frames)``."""
    t0, tones = decode_tones(times)
    bits = un_nrzi(tones, tones[0]) if tones else []
    return t0, bits, deframe(bits)


# ------------------------------------------------------------- the timing
def check_timing(times: Sequence[float], tones: Sequence[int], t0: float,
                 clock_hz: float) -> dict:
    """The transmitter's edges against Bell 202 and the bit grid:
    ``mark``/``space`` the worst steady half-period error relative to
    nominal (HCF_SPEC-54's +-1 %, bell202.py), ``mark_s``/``space_s`` the
    same in seconds, ``phase_s`` the worst distance of any edge from the
    ideal continuous-phase wave of ``tones`` on the grid ``t0`` (refined
    here over the whole span), ``t0`` that refined start."""
    mark, space = B.steady_half_periods(times)
    out = {}
    for name, hps, hz in (("mark", mark, B.MARK_HZ), ("space", space, B.SPACE_HZ)):
        nominal = 0.5 / hz
        out[name] = max((abs(h / nominal - 1) for h in hps), default=None)
        out[name + "_s"] = max((abs(h - nominal) for h in hps), default=None)
    edges = [t for t in times if t0 <= t < t0 + len(tones) * BIT_S]
    t1, err = B.check_phase_continuity(edges, list(tones), t0, span_s=8e-6)
    out["phase_s"], out["t0"] = err, t1
    return out
