"""WSPR (Weak Signal Propagation Reporter) reference models: a message
encoder (source coding, convolutional code, interleaver, sync merge) that
gives the 162 four-level symbols of a type 1 message, the 4-FSK tone plan
of a phase-accumulator square-wave source, and a checker that recovers
the symbols from a recorded pin and measures each tone's frequency.

Written from the protocol description, not from ``firmware/wspr_tx.loom``.

Sources:

* A. Talbot G4JNT, "The WSPR Coding Process", June 2009 (a non-normative
  description of K1JT's encoder, which its author checked against the
  symbols WSPR.EXE lists), read on 2026-11-07 from the copy at
  github.com/vazeri/WSPREncode (files/1783129/wspr_coding_process.pdf):
  - "Source Coding": callsign of up to six characters A-Z, 0-9, space; the
    third character forced to be a digit by prefixing a space ("G4JNT"
    becomes " G4JNT"), padded with spaces to six; values '0'-'9' 0-9,
    'A'-'Z' 10-35, space 36; N1 = Ch1, N2 = N1*36 + Ch2, N3 = N2*10 + Ch3,
    N4..N6 = 27*N(k-1) + Ch(k) - 10 (28 bits). Locator: Loc1, Loc2 'A'-'R'
    as 0-17, Loc3, Loc4 0-9; M1 = (179 - 10 Loc1 - Loc3) * 180 + 10 Loc2 +
    Loc4 (15 bits); power 0..60 dBm, M = M1 * 128 + Pwr + 64 (22 bits);
  - "Bit Packing": the 50 bits N then M, MSB first, then zeros: 81 bits
    are used;
  - "Convolutional Encoding": rate 1/2, constraint length 32; each bit
    shifted into the LSB of a cleared 32-bit register; after each shift the
    parities of the register ANDed with 0xF2D05351 and then with
    0xE4613C47 are appended: 162 bits;
  - "Interleaving": P = 0; for I = 0..255, J = the 8-bit reversal of I; if
    J < 162, D[J] = S[P] and P += 1;
  - "Merge With Sync Vector": Symbol[n] = Sync[n] + 2 * Data[n], with the
    162-bit vector printed there (its last two entries are printed
    "0.0,0"; read as 0, 0);
  - "Modulation": a tone spacing of 12000 / 8192 Hz per symbol value, the
    symbol length its reciprocal, 162 symbols in about 110.6 s.
* J. Taylor K1JT, the WSPR protocol as implemented in WSPR / WSJT-X
  (wsprcode): the same polynomials and sync vector. Not read for this
  module (the G4JNT text says it agrees with WSPR.EXE); unchecked here.
* Test vectors: the Rust crate ``wspr`` (docs.rs/crate/wspr, src/lib.rs,
  ``test_encode_wspr``), read on 2026-11-07: G1ABC IO83 37 and KA1BCD AA00
  33. The crate cites the G4JNT text; it does not say its vectors came from
  wsprcode, so they confirm this module against an independent
  implementation of the same description, not against K1JT's program.

WSPR transmissions start one second into an even UTC minute; that
scheduling belongs to the host, not to this model.
"""

from __future__ import annotations

from typing import List, Sequence, Tuple

POLY = (0xF2D05351, 0xE4613C47)                 # G4JNT "Convolutional Encoding"
SYNC = tuple(int(c) for c in (                  # G4JNT "Merge With Sync Vector"
    "110000001000111000100101111000000010010100"
    "000010110011010001101000011010101010010010"
    "110001101010001000001001001110110011010001"
    "110000010100110000000110101100011000"))
assert len(SYNC) == 162
TONE_SPACING_HZ = 12000.0 / 8192.0              # G4JNT "Modulation"
SYMBOL_S = 8192.0 / 12000.0
SYMBOLS = 162


def char_value(ch: str) -> int:
    """'0'-'9' 0-9, 'A'-'Z' 10-35, space 36 (G4JNT "Source Coding")."""
    if ch == " ":
        return 36
    if "0" <= ch <= "9":
        return ord(ch) - ord("0")
    if "A" <= ch <= "Z":
        return ord(ch) - ord("A") + 10
    raise ValueError("character %r is not A-Z, 0-9 or space" % ch)


def normalise_call(call: str) -> str:
    """Upper case; a space in front when the second character is a digit,
    so that the third one is; spaces after, to six characters."""
    call = call.strip().upper()
    if len(call) >= 2 and call[1].isdigit():
        call = " " + call
    if len(call) > 6 or not call[2:3].isdigit():
        raise ValueError("callsign %r cannot be a type 1 WSPR callsign" % call)
    return call.ljust(6)


def pack_call(call: str) -> int:
    c = [char_value(ch) for ch in normalise_call(call)]
    n = c[0]
    n = n * 36 + c[1]
    n = n * 10 + c[2]
    for v in c[3:]:
        if v < 10:
            raise ValueError("the last three characters must be letters or space")
        n = 27 * n + v - 10
    return n


def pack_locator_power(loc: str, dbm: int) -> int:
    loc = loc.upper()
    l1, l2 = ord(loc[0]) - ord("A"), ord(loc[1]) - ord("A")
    l3, l4 = int(loc[2]), int(loc[3])
    if not (0 <= l1 <= 17 and 0 <= l2 <= 17 and 0 <= dbm <= 60):
        raise ValueError("locator %r or power %r out of range" % (loc, dbm))
    m1 = (179 - 10 * l1 - l3) * 180 + 10 * l2 + l4
    return m1 * 128 + dbm + 64


def message_bits(call: str, loc: str, dbm: int) -> List[int]:
    """The 81 bits the encoder reads: N (28) then M (22), MSB first, then
    31 zeros (G4JNT "Bit Packing")."""
    n, m = pack_call(call), pack_locator_power(loc, dbm)
    value = (n << 22) | m
    return [(value >> (49 - k)) & 1 for k in range(50)] + [0] * 31


def parity(x: int) -> int:
    return bin(x).count("1") & 1


def convolve(bits: Sequence[int]) -> List[int]:
    """Rate 1/2, K = 32: two parity bits per input bit, in POLY order."""
    reg, out = 0, []
    for b in bits:
        reg = ((reg << 1) | b) & 0xFFFFFFFF
        out.extend(parity(reg & p) for p in POLY)
    return out


def rev8(i: int) -> int:
    return int("{:08b}".format(i)[::-1], 2)


def interleave(src: Sequence[int]) -> List[int]:
    dst, p = [0] * SYMBOLS, 0
    for i in range(256):
        j = rev8(i)
        if j < SYMBOLS:
            dst[j] = src[p]
            p += 1
    assert p == SYMBOLS
    return dst


def symbols(call: str, loc: str, dbm: int) -> List[int]:
    """The 162 channel symbols 0..3 of a type 1 message."""
    data = interleave(convolve(message_bits(call, loc, dbm)))
    return [s + 2 * d for s, d in zip(SYNC, data)]


# ------------------------------------------------------------ tone plan
def tone_hz(base_step: int, spacing_step: int, sym: int, fs: float,
            bits: int = 16) -> float:
    """The mean frequency of a phase accumulator of ``bits`` bits that adds
    ``base + sym * spacing`` every sample at ``fs`` samples a second: the
    accumulator's top bit is a square wave at that frequency, exactly, on
    average (the phase is continuous across symbols, as 4-FSK keeps it)."""
    return (base_step + sym * spacing_step) * fs / (1 << bits)


def tone_pin(syms: Sequence[int], base_step: int, spacing_step: int,
             samples_per_symbol: int, bits: int = 16) -> List[int]:
    """The level of the accumulator's top bit in every sample of a
    transmission: sample n shows the phase after the steps of samples
    0 .. n-1, the phase starting at 0 (the program's documented contract).
    """
    mask, top = (1 << bits) - 1, bits - 1
    phase, out = 0, []
    for s in syms:
        step = base_step + s * spacing_step
        for _ in range(samples_per_symbol):
            out.append((phase >> top) & 1)
            phase = (phase + step) & mask
    return out


def pin_samples(edges: Sequence[Tuple[int, int]], initial: int, start: int,
                period: float, count: int, offset: float) -> List[int]:
    """The level of a recorded pin at ``start + n * period + offset`` for
    n = 0 .. count-1 (the middle of each sample period)."""
    out, level, i = [], initial, 0
    for n in range(count):
        t = start + n * period + offset
        while i < len(edges) and edges[i][0] <= t:
            level = edges[i][1]
            i += 1
        out.append(level)
    return out


def recover_symbols(levels: Sequence[int], base_step: int, spacing_step: int,
                    samples_per_symbol: int, bits: int = 16) -> List[int]:
    """Read the symbols back from the sampled pin: for each symbol, the one
    value 0..3 whose tone reproduces every sample of it from the phase the
    symbols before left. A symbol no value explains is -1, and the phase is
    then lost: every later symbol is -1 too."""
    mask, top, L = (1 << bits) - 1, bits - 1, samples_per_symbol
    phase, out = 0, []
    for j in range(len(levels) // L):
        chunk, found = levels[j * L:(j + 1) * L], None
        for s in range(4):
            step, ph, ok = base_step + s * spacing_step, phase, True
            for lv in chunk:
                if (ph >> top) & 1 != lv:
                    ok = False
                    break
                ph = (ph + step) & mask
            if ok and found is None:
                found, end = s, ph
        if found is None:
            return out + [-1] * (len(levels) // L - j)
        out.append(found)
        phase = end
    return out


def measured_hz(edges: Sequence[Tuple[int, int]], t0: int, t1: int,
                clock_hz: float) -> float:
    """The square wave's frequency in [t0, t1): rising edges counted
    between the first and the last one in the window."""
    rises = [c for c, v in edges if v == 1 and t0 <= c < t1]
    if len(rises) < 2:
        raise ValueError("fewer than two rising edges in the window")
    return (len(rises) - 1) * clock_hz / (rises[-1] - rises[0])
