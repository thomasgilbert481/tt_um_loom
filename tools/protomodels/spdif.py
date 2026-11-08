"""S/PDIF (IEC 60958 consumer) reference model, for firmware/spdif_tx.loom.

Written from public descriptions of IEC 60958 / AES3, not from the
firmware. IEC 60958-1 and -3 themselves were not read. Read 2026-11-07:

* EBU Tech 3250-E (2004, the AES3/EBU interface), tech.ebu.ch:
  2.2.1 a subframe is 32 time slots: 0..3 preamble, 4..7 auxiliary, the
  audio LSB first with its MSB in slot 27 (a 24-bit word from slot 4,
  unused LSBs 0), 28 validity V, 29 user U, 30 channel status C, 31 parity
  P, and "time slots 4 to 31 inclusive will carry an even number of ones";
  2.2.2 two subframes a frame, the first starts with preamble X (IEC's M),
  or Z (IEC's B) once every 192 frames, the second always with Y (IEC's
  W); 2.1.11 a block is 192 frames; 2.3 biphase mark: "the first state of
  a symbol is always different from the second state of the previous
  symbol", the two states of a 1 differ, of a 0 are equal; 2.4 the
  preambles as eight half-cell states after a preceding 0, B 11101000,
  M 11100010, W 11100100 (complemented after a 1); 2.1.13 the unit
  interval UI is the shortest nominal interval, a half time slot, so a
  frame is 128 UI; 4 C bit 0 of the block is in the frame with preamble
  B; 6.2.5.1 transmitter intrinsic jitter "less than 0,025 UI" peak
  (measured through a 700 Hz high-pass, which this model does not apply:
  it bounds the unfiltered jitter, a stricter test); 6.3.6 receiver
  jitter tolerance (Fig. 11: 0.25 UI peak to peak at high frequency, only
  from a summary of the figure, unchecked).
* Wikipedia, "S/PDIF", Protocol specifications: the consumer channel
  status is identical on both channels; byte 0 bit 0 consumer (0), bit 1
  linear PCM (0), bit 2 copy permitted (1), bits 6..7 mode 0; byte 1
  category; byte 2 source and channel numbers; byte 3 bits 0..3 the
  sampling frequency, written 0000 44.1 kHz, 0100 48 kHz, 1100 32 kHz;
  bits 4..5 clock accuracy (00 printed "1100ppm", taken as 1000 ppm,
  level II: the level name is from memory);
  byte 4 bit 0 the maximum word length (0: 20 bits), bits 1..3 the sample
  length. Bit order inside the written codes is not stated: this model
  reads them left to right as bits 24, 25, 26, 27, so 32 kHz sets bits 24
  and 25 (byte 3 = 0x03). That reading, and byte 4 = 0x02 for 16 bits of a
  20-bit maximum, are from memory, unchecked (QUESTIONS.md of W-104).
* Wikipedia, "AES3", Synchronisation preamble: the same patterns.

Channel status bytes go LSB first: C bit k of the block is bit k % 8 of
byte k // 8, carried in frame k.
"""

from __future__ import annotations

import dataclasses
from typing import Dict, List, Optional, Sequence, Tuple

from tools.protomodels.bench import Lines, Model, PinSpec, pad_of

FRAMES = 192                    # a block (EBU 2.1.11)
UI_FRAME = 128                  # half-cells a frame (EBU 2.1.13, 2.2.1)
UI_SUB = 64                     # half-cells a subframe
JITTER_TX_PEAK = 0.025          # UI, transmitter intrinsic, EBU 6.2.5.1
JITTER_RX_PP = 0.25             # UI p-p, receiver tolerance, unchecked

# Eight half-cell levels after a preceding 0 (EBU 2.4).
PREAMBLES = {"B": (1, 1, 1, 0, 1, 0, 0, 0),
             "M": (1, 1, 1, 0, 0, 0, 1, 0),
             "W": (1, 1, 1, 0, 0, 1, 0, 0)}

# Byte 3 bits 0..3 (bits 24..27), the written code read left to right.
FS_CODES = {0b0000: 44100, 0b0010: 48000, 0b0011: 32000}


def parity(bits: Sequence[int]) -> int:
    """The P bit that makes ``bits`` (slots 4..30) and P even."""
    return sum(bits) & 1


def subframe_bits(audio: int, v: int = 0, u: int = 0, c: int = 0) -> List[int]:
    """Slots 4..31 for a 24-bit audio word (LSB in slot 4), V, U, C and P."""
    bits = [(audio >> i) & 1 for i in range(24)] + [v & 1, u & 1, c & 1]
    return bits + [parity(bits)]


def sample16(sample: int) -> int:
    """A 16-bit sample in the 24-bit audio field: slots 12..27."""
    return (sample & 0xFFFF) << 8


def encode_subframe(pre: str, slots: Sequence[int], last: int) -> List[int]:
    """64 half-cell levels: the preamble (complemented after a 1) and the
    biphase mark code of slots 4..31; ``last`` is the level before."""
    cells = [p ^ last for p in PREAMBLES[pre]]
    level = cells[-1]
    for b in slots:
        level ^= 1                  # every bit starts with a transition
        cells.append(level)
        level ^= b                  # a 1 has one in its middle
        cells.append(level)
    return cells


def status_bits(status: Sequence[int]) -> List[int]:
    """The 192 C bits of a block from up to 24 status bytes, LSB first."""
    data = list(status) + [0] * (24 - len(status))
    return [(data[k // 8] >> (k % 8)) & 1 for k in range(FRAMES)]


def consumer_status(status: Sequence[int]) -> Dict[str, object]:
    """The consumer fields of a channel status block (bytes 0..4)."""
    b = list(status) + [0] * (5 - len(status))
    return {"professional": b[0] & 1, "non_pcm": (b[0] >> 1) & 1,
            "copy_permitted": (b[0] >> 2) & 1, "emphasis": (b[0] >> 3) & 7,
            "mode": (b[0] >> 6) & 3, "category": b[1],
            "source": b[2] & 15, "channel": b[2] >> 4,
            "fs": FS_CODES.get(b[3] & 15), "clock_accuracy": (b[3] >> 4) & 3,
            "word_length_code": b[4] & 15}


class EdgeRecorder(Model):
    """Every change of one pad: ``edges`` holds ``(cycle, level)``, the cycle
    being the first with the new level; ``idle`` is the level before."""

    def __init__(self, pin: PinSpec, idle: int = 0) -> None:
        self.pad = pad_of(pin)
        self.level = idle
        self.edges: List[Tuple[int, int]] = []

    def observe(self, lines: Lines) -> None:
        level = lines.get(self.pad)
        if level != self.level:
            self.edges.append((lines.cycle, level))
            self.level = level


@dataclasses.dataclass
class Cells:
    """Half-cell levels recovered from edges, and how far the edges lay
    from the ideal grid."""

    levels: List[int]               # one per UI, from the first edge on
    first: int                      # cycle of the first edge
    widths: List[int]               # each interval between edges, in UI
    jitter_pp: float                # edge deviation, max - min, in UI


def cells_from_edges(edges: Sequence[Tuple[int, int]], ui: float,
                     tol: float = JITTER_TX_PEAK) -> Cells:
    """Hold the recorded edges to a grid of ``ui`` clocks anchored on the
    first edge. Every interval must be 1, 2 or 3 UI (3 only in a preamble);
    every edge's deviation from the grid, taken about the mean deviation,
    must be within ``tol`` UI peak (the transmitter limit by default)."""
    assert edges, "no edges"
    first = edges[0][0]
    levels: List[int] = []
    widths: List[int] = []
    devs = [0.0]
    pos = 0
    for (c0, lv), (c1, _) in zip(edges, edges[1:]):
        n = round((c1 - c0) / ui)
        assert n in (1, 2, 3), "interval of %d clocks at cycle %d is %.2f UI" % (
            c1 - c0, c0, (c1 - c0) / ui)
        levels += [lv] * n
        widths.append(n)
        pos += n
        devs.append((c1 - first) / ui - pos)
    levels.append(edges[-1][1])     # the cell the last edge starts
    mean = (max(devs) + min(devs)) / 2
    peak = max(abs(d - mean) for d in devs)
    assert peak <= tol + 1e-9, "edge jitter %.3f UI peak, limit %.3f" % (peak, tol)
    return Cells(levels, first, widths, max(devs) - min(devs))


@dataclasses.dataclass
class Subframe:
    pre: str                        # "B", "M" or "W"
    start: int                      # index of its first cell
    aux: int                        # slots 4..7
    audio: int                      # slots 4..27, the 24-bit field
    v: int
    u: int
    c: int

    @property
    def sample(self) -> int:
        """The 16-bit sample of slots 12..27, as a signed value."""
        s = (self.audio >> 8) & 0xFFFF
        return s - 0x10000 if s & 0x8000 else s


def _preamble(cells: Sequence[int], i: int) -> Optional[str]:
    """The preamble at cell ``i``, in either polarity, or None."""
    seg = tuple(cells[i:i + 8])
    for name, pat in PREAMBLES.items():
        if seg == pat or seg == tuple(1 - p for p in pat):
            return name
    return None


def decode(levels: Sequence[int]) -> List[Subframe]:
    """Every whole subframe from the first preamble on. Checks the preamble
    of each, the transition at every bit boundary, and even parity."""
    start = next((i for i in range(len(levels) - 8) if _preamble(levels, i)), None)
    assert start is not None, "no preamble"
    out: List[Subframe] = []
    i = start
    while i + UI_SUB <= len(levels):
        pre = _preamble(levels, i)
        assert pre, "no preamble at cell %d (subframe %d)" % (i, len(out))
        bits = []
        for j in range(4, 32):
            a, b = levels[i + 2 * j], levels[i + 2 * j + 1]
            assert a != levels[i + 2 * j - 1], "no bit-boundary transition at cell %d" % (
                i + 2 * j)
            bits.append(int(a != b))
        assert sum(bits) % 2 == 0, "odd parity in subframe %d" % len(out)
        audio = sum(b << k for k, b in enumerate(bits[:24]))
        out.append(Subframe(pre, i, audio & 15, audio, bits[24], bits[25], bits[26]))
        i += UI_SUB
    return out


@dataclasses.dataclass
class Stream:
    """Subframes grouped into frames: the first frame starts at the first
    B or M (a left subframe)."""

    frames: List[Tuple[Subframe, Subframe]]
    block_starts: List[int]         # frame indices carrying B

    @property
    def samples(self) -> List[Tuple[int, int]]:
        return [(l.sample, r.sample) for l, r in self.frames]

    def status(self, block: int = 0) -> List[int]:
        """The 24 channel status bytes of a whole block (the ``block``-th B
        in the stream), checking that both channels carry the same C bits."""
        f0 = self.block_starts[block]
        assert f0 + FRAMES <= len(self.frames), "block %d is not whole" % block
        out = [0] * 24
        for k in range(FRAMES):
            left, right = self.frames[f0 + k]
            assert left.c == right.c, "C differs between channels in frame %d" % k
            out[k // 8] |= left.c << (k % 8)
        return out


def frames(subs: Sequence[Subframe]) -> Stream:
    """Pair subframes: B or M then W. Checks that B comes every 192 frames."""
    i = next(k for k, s in enumerate(subs) if s.pre in "BM")
    pairs: List[Tuple[Subframe, Subframe]] = []
    while i + 1 < len(subs):
        left, right = subs[i], subs[i + 1]
        assert left.pre in "BM" and right.pre == "W", (
            "subframes %d, %d carry %s, %s" % (i, i + 1, left.pre, right.pre))
        pairs.append((left, right))
        i += 2
    starts = [k for k, (left, _) in enumerate(pairs) if left.pre == "B"]
    for a, b in zip(starts, starts[1:]):
        assert b - a == FRAMES, "B preambles %d frames apart" % (b - a)
    return Stream(pairs, starts)


def receive(edges: Sequence[Tuple[int, int]], ui: float,
            tol: float = JITTER_TX_PEAK) -> Stream:
    """Edges of a pin to frames: :func:`cells_from_edges`, :func:`decode`,
    :func:`frames`."""
    return frames(decode(cells_from_edges(edges, ui, tol).levels))


def encode_stream(samples: Sequence[Tuple[int, int]], status: Sequence[int],
                  first_frame: int = 0, last: int = 0) -> List[int]:
    """Half-cell levels of frames ``first_frame`` .. of a block, 16-bit
    samples (left, right), V = U = 0, C from ``status``, starting after the
    level ``last``. The reference the receiver is checked against."""
    cbits = status_bits(status)
    cells: List[int] = []
    for k, (left, right) in enumerate(samples):
        f = (first_frame + k) % FRAMES
        for pre, s in (("B" if f == 0 else "M", left), ("W", right)):
            sub = encode_subframe(pre, subframe_bits(sample16(s), c=cbits[f]), last)
            cells += sub
            last = sub[-1]
    return cells
