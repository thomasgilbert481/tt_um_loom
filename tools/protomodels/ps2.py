"""PS/2 reference model: a device (keyboard or mouse) that clocks scancodes.

:class:`Ps2Keyboard` drives the two lines of a PS/2 port, CLK and DATA, as
the device does in the device-to-host direction: it owns the clock, puts a
bit on DATA while CLK is high and pulls CLK low half a period later, which
is the edge the host samples on (``docs/SEMANTICS.md`` 6.4 ``WAITE``). A
frame is eleven bits - start 0, eight data bits least significant first, an
odd parity bit, stop 1 - and the clock runs at 10 to 16.7 kHz, which at
50 MHz is 5000 to 3000 core clocks per bit.

Real PS/2 lines are open collector with pull-ups; here the device is the
only thing that ever drives them, so it drives them push-pull and idles
both high, which the host program sees identically.

Faults are per frame, so a test can put one bad frame in a stream of good
ones: ``bad_parity`` flips the parity bit, ``stop=0`` sends a low stop bit,
``no_start=True`` sends a first bit that is not a start bit, and
``truncate=n`` stops the device after ``n`` clock pulses, which is how a
frame is cut short.
"""

from __future__ import annotations

from collections import deque
from typing import Iterable, List, Optional, Tuple

from .bench import Drive, Lines, Model, PinSpec, pad_of

#: Core clocks per CLK period at 50 MHz for the two ends of the PS/2 band.
CLOCKS_16K7 = 3000              # 16.67 kHz, the fastest a device may clock
CLOCKS_10K = 5000               # 10 kHz, the slowest


def odd_parity(value: int) -> int:
    """The PS/2 parity bit: it makes the number of ones in the nine bits odd."""
    return 1 - (bin(value & 0xFF).count("1") & 1)


class Ps2Keyboard(Model):
    """A PS/2 device that clocks queued bytes to the host.

    Args:
        clk: the CLK pad (an input of the chip).
        data: the DATA pad.
        period_clocks: core clocks per CLK period (3000 .. 5000 is the band).
        gap_clocks: idle time after a frame before the next one may start;
            one clock period by default.
    """

    def __init__(self, clk: PinSpec, data: PinSpec, *,
                 period_clocks: int = CLOCKS_16K7,
                 gap_clocks: Optional[int] = None) -> None:
        self.clk_pad = pad_of(clk)
        self.data_pad = pad_of(data)
        self.period = int(period_clocks)
        self.gap = self.period if gap_clocks is None else int(gap_clocks)
        self.clk = self.data = 1                    # both lines idle high
        self.now = 0
        self._changes: deque = deque()              # (cycle, clk, data)
        self._free_at = 0
        #: ``(start cycle, byte, list of bits)`` of every frame clocked out.
        self.frames_sent: List[Tuple[int, int, List[int]]] = []

    # ------------------------------------------------------------- driving
    def send(self, data: Iterable[int], **kwargs) -> None:
        for value in (bytes(data) if isinstance(data, (bytes, bytearray))
                      else data):
            self.send_frame(value, **kwargs)

    def send_frame(self, value: int, *, stop: int = 1, bad_parity: bool = False,
                   no_start: bool = False, truncate: Optional[int] = None,
                   start: Optional[int] = None) -> int:
        """Queue one frame; returns the cycle its first CLK period starts.

        ``truncate=n`` clocks only the first ``n`` bits and then lets both
        lines go idle, which is a frame the host must give up on.
        """
        value &= 0xFF
        parity = odd_parity(value) ^ (1 if bad_parity else 0)
        bits = ([1 if no_start else 0]
                + [(value >> i) & 1 for i in range(8)]
                + [parity, stop & 1])
        if truncate is not None:
            bits = bits[:truncate]
        t0 = max(self._free_at, self.now + 1)
        if start is not None:
            t0 = max(t0, int(start))
        half = self.period // 2
        for k, bit in enumerate(bits):
            at = t0 + k * self.period
            self._changes.append((at, 1, bit))          # DATA set, CLK high
            self._changes.append((at + half, 0, bit))   # the sampling edge
        end = t0 + len(bits) * self.period
        self._changes.append((end, 1, 1))               # both lines idle
        self._free_at = end + self.gap
        self.frames_sent.append((t0, value, bits))
        return t0

    @property
    def busy(self) -> bool:
        return bool(self._changes)

    def idle_at(self) -> int:
        """The cycle by which everything queued has been clocked out."""
        return self._free_at

    def drive(self, drive: Drive, cycle: int) -> None:
        self.now = cycle
        changes = self._changes
        while changes and changes[0][0] <= cycle:
            _, self.clk, self.data = changes.popleft()
        drive.set(self.clk_pad, self.clk)
        drive.set(self.data_pad, self.data)


class Ps2Monitor(Model):
    """Records the DATA level at every CLK falling edge, as the host sees it.

    Independent of the firmware: it says what was really on the wire, so a
    test can tell a bad frame from a bad receiver.
    """

    def __init__(self, clk: PinSpec, data: PinSpec) -> None:
        self.clk_pad = pad_of(clk)
        self.data_pad = pad_of(data)
        self.prev = 1
        #: ``(cycle, DATA)`` at every falling edge of CLK.
        self.samples: List[Tuple[int, int]] = []

    def observe(self, lines: Lines) -> None:
        clk = lines.get(self.clk_pad)
        if self.prev and not clk:
            self.samples.append((lines.cycle, lines.get(self.data_pad)))
        self.prev = clk

    @property
    def bits(self) -> List[int]:
        return [bit for _, bit in self.samples]

    def frames(self, length: int = 11) -> List[List[int]]:
        bits = self.bits
        return [bits[i:i + length] for i in range(0, len(bits), length)]
