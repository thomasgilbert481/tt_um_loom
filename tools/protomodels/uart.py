"""UART reference models: an 8N1 encoder that drives a chip input and a
decoder that watches a chip output.

Bit periods are given in core clocks and may be fractional (a 50 MHz chip at
115200 baud is 434.03 clocks per bit). Frame edges are placed at
``start + round(k * bit_clocks)`` so a stream never drifts.
"""

from __future__ import annotations

import dataclasses
from collections import deque
from typing import Iterable, List, Optional, Tuple

from .bench import Drive, Lines, Model, PinSpec, pad_of


class UartTx(Model):
    """Drives 8N1 frames (LSB first) onto a pad; idles at ``idle``.

    ``send`` queues bytes; ``send_frame(value, stop=0)`` sends a frame whose
    stop bit is low, i.e. a framing error. ``bit_clocks`` may be changed
    between frames to model a transmitter with a baud-rate error.
    """

    def __init__(self, pin: PinSpec, bit_clocks: float, *, idle: int = 1,
                 data_bits: int = 8, gap_bits: float = 0.0) -> None:
        self.pad = pad_of(pin)
        self.bit_clocks = float(bit_clocks)
        self.idle = idle
        self.data_bits = data_bits
        self.gap_bits = gap_bits
        self.level = idle
        self.now = 0
        self._changes: deque = deque()      # (cycle, level), increasing cycle
        self._free_at = 0.0                 # first cycle a new frame may start
        self.frames_sent: List[Tuple[int, int, int]] = []   # (start, value, stop)

    def send(self, data: Iterable[int]) -> None:
        for value in bytes(data) if isinstance(data, (bytes, bytearray)) else data:
            self.send_frame(value)

    def send_frame(self, value: int, *, stop: int = 1, start: Optional[int] = None) -> int:
        """Queue one frame; returns its start cycle."""
        t0 = max(self._free_at, float(self.now + 1))
        if start is not None:
            t0 = max(t0, float(start))
        bits = [0] + [(value >> i) & 1 for i in range(self.data_bits)] + [stop]
        for k, bit in enumerate(bits):
            self._changes.append((int(round(t0 + k * self.bit_clocks)), bit))
        end = t0 + len(bits) * self.bit_clocks
        self._changes.append((int(round(end)), self.idle))
        # After a low stop bit the line must idle for a bit, or the next
        # start bit would have no falling edge to be found by.
        gap = max(self.gap_bits, 0.0 if stop else 1.0)
        self._free_at = end + gap * self.bit_clocks
        self.frames_sent.append((int(round(t0)), value, stop))
        return int(round(t0))

    @property
    def busy(self) -> bool:
        return bool(self._changes)

    def drive(self, drive: Drive, cycle: int) -> None:
        self.now = cycle
        changes = self._changes
        while changes and changes[0][0] <= cycle:
            self.level = changes.popleft()[1]
        drive.set(self.pad, self.level)


@dataclasses.dataclass
class UartFrame:
    value: int
    start: int                      # cycle of the falling start edge
    framing_error: bool
    edges: List[Tuple[int, int]]    # (cycle, level) of every edge in the frame


class UartRx(Model):
    """Decodes 8N1 frames from a pad by sampling each bit at its middle.

    A frame starts on a falling edge; the start bit must still be low at
    its middle (otherwise the edge was a glitch). A low stop bit is a
    framing error; the frame is still recorded with ``framing_error``.
    Mid-bit sampling tolerates a transmitter whose bit period differs from
    ``bit_clocks`` by several per cent; :meth:`edge_offsets` measures the
    actual edge placement against the nominal grid.
    """

    def __init__(self, pin: PinSpec, bit_clocks: float, *, data_bits: int = 8,
                 idle: int = 1) -> None:
        self.pad = pad_of(pin)
        self.bit_clocks = float(bit_clocks)
        self.data_bits = data_bits
        # Not armed until the line has been seen idle, so a pad that comes
        # out of reset low is not taken for a start bit.
        self.prev = 1 - idle
        self.frames: List[UartFrame] = []
        self.edges: List[Tuple[int, int]] = []
        self._start: Optional[int] = None
        self._samples: List[int] = []
        self._next = 0
        self._frame_edges: List[Tuple[int, int]] = []

    def _sample_at(self, k: int) -> int:
        return self._start + int(round((k + 0.5) * self.bit_clocks))

    def observe(self, lines: Lines) -> None:
        v = lines.get(self.pad)
        cycle = lines.cycle
        if v != self.prev:
            self.edges.append((cycle, v))
            if self._start is not None:
                self._frame_edges.append((cycle, v))
        if self._start is None:
            if self.prev == 1 and v == 0:
                self._start = cycle
                self._samples = []
                self._frame_edges = [(cycle, 0)]
                self._next = self._sample_at(0)
        elif cycle >= self._next:
            self._samples.append(v)
            k = len(self._samples)
            if k == 1 and v != 0:
                self._start = None          # glitch, not a start bit
            elif k == self.data_bits + 2:
                value = sum(bit << i for i, bit in enumerate(self._samples[1:-1]))
                self.frames.append(UartFrame(value, self._start, v == 0,
                                             list(self._frame_edges)))
                self._start = None
            else:
                self._next = self._sample_at(k)
        self.prev = v

    @property
    def data(self) -> bytes:
        """Every correctly framed byte, in order."""
        return bytes(f.value for f in self.frames if not f.framing_error)

    @property
    def errors(self) -> List[UartFrame]:
        return [f for f in self.frames if f.framing_error]

    def edge_offsets(self, frame: UartFrame) -> List[Tuple[int, float]]:
        """For each edge of ``frame``: ``(bit index, clocks off the nominal grid)``,
        measured from the start edge."""
        out = []
        for cycle, _ in frame.edges:
            pos = (cycle - frame.start) / self.bit_clocks
            k = int(round(pos))
            out.append((k, (cycle - frame.start) - k * self.bit_clocks))
        return out
