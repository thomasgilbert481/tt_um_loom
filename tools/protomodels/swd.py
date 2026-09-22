"""SWD reference model: an ARM debug port on SWCLK and SWDIO.

:class:`SwdDp` follows the ADIv5 serial wire protocol from the host's side
of the wire: it counts the line reset, recognises the JTAG-to-SWD select
sequence, decodes each packet request (including its parity, stop and park
bits) and answers with the turnaround cycle, a three-bit ACK and, for a
read, 32 data bits and their parity. It samples SWDIO at the rising edge of
SWCLK and changes it at the falling edge, as both ends of a real link do,
so a master that drives or samples on the wrong phase reads rubbish rather
than getting away with it.

The DP drives SWDIO only during its own part of a packet and lets go for
every turnaround, which is why the bench needs a pull-up on that pad
(``Bench(pullups=...)``): between the two ends the line is undriven for one
cycle each way.

Faults a test can ask for: ``ack`` other than OK (there is then no data
phase, as the protocol says), and ``parity_error``, which flips the parity
bit of the data.
"""

from __future__ import annotations

from collections import deque
from typing import Deque, Dict, List, Optional, Tuple

from .bench import UIO, Drive, Lines, Model, PinSpec, pad_of

ACK_OK, ACK_WAIT, ACK_FAULT = 0b001, 0b010, 0b100

#: The 16-bit JTAG-to-SWD select sequence, least significant bit first.
SWITCH_SEQUENCE = 0xE79E
#: Cycles of SWDIO high that make a line reset (ADIv5 says at least 50).
RESET_CYCLES = 50


class SwdDp(Model):
    """A debug port that answers a DPIDR read.

    Args:
        swclk: the chip output that clocks the link.
        swdio: the bidirectional pad; needs a bench pull-up.
        dpidr: the value of DP register 0x00.
        ack: the ACK to answer with (``ACK_OK`` by default).
        parity_error: send the wrong parity bit after the data.
    """

    def __init__(self, swclk: PinSpec, swdio: PinSpec, *,
                 dpidr: int = 0x2BA01477, ack: int = ACK_OK,
                 parity_error: bool = False,
                 reset_cycles: int = RESET_CYCLES) -> None:
        self.clk_pad = pad_of(swclk)
        self.io_pad = pad_of(swdio)
        if self.io_pad[0] != UIO:
            raise ValueError("SWDIO must be a bidirectional (uio) pad")
        self.registers: Dict[Tuple[int, int], int] = {(0, 0): dpidr & 0xFFFFFFFF}
        self.ack = ack
        self.parity_error = parity_error
        self.reset_cycles = reset_cycles

        self.mode = "jtag"              # "jtag" until the select sequence
        self.phase = "reset"
        self.highs = 0                  # SWDIO-high run inside a line reset
        self.ones = 0                   # SWDIO-high run anywhere
        self.shift = self.count = 0
        self.out: Deque[Optional[int]] = deque()
        self.driving = False
        self.value = 1
        self.prev_clk = 0

        #: one entry per packet request seen: ``(APnDP, RnW, address, valid)``
        self.packets: List[Tuple[int, int, int, bool]] = []
        self.line_resets = 0
        self.switches = 0               # accepted JTAG-to-SWD sequences
        self.bad_packets = 0

    # ------------------------------------------------------------- driving
    def drive(self, drive: Drive, cycle: int) -> None:
        if self.driving:
            drive.set(self.io_pad, self.value)

    # -------------------------------------------------------------- packet
    def _respond(self, apndp: int, rnw: int, addr: int) -> None:
        """Queue the turnaround, the ACK and, for an accepted read, the data."""
        self.out.append(None)                               # turnaround cycle
        for i in range(3):
            self.out.append((self.ack >> i) & 1)
        if self.ack == ACK_OK and rnw:
            value = self.registers.get((apndp, addr), 0)
            for i in range(32):
                self.out.append((value >> i) & 1)
            parity = bin(value).count("1") & 1
            self.out.append(parity ^ (1 if self.parity_error else 0))
        self.phase = "response"

    def _request(self, word: int) -> None:
        start = word & 1
        apndp = (word >> 1) & 1
        rnw = (word >> 2) & 1
        addr = (word >> 3) & 3                              # A[2:3]
        parity = (word >> 5) & 1
        stop = (word >> 6) & 1
        park = (word >> 7) & 1
        want = apndp ^ rnw ^ (addr & 1) ^ ((addr >> 1) & 1)
        valid = start == 1 and stop == 0 and park == 1 and parity == want
        self.packets.append((apndp, rnw, addr * 4, valid))
        if valid:
            self._respond(apndp, rnw, addr * 4)
        else:
            self.bad_packets += 1
            self.phase = "reset"
            self.highs = 0

    def _host_bit(self, bit: int) -> None:
        self.ones = self.ones + 1 if bit else 0
        if self.ones >= self.reset_cycles and self.phase != "reset":
            self.phase, self.highs = "reset", self.ones    # a reset anywhere
            return
        if self.phase == "reset":
            if bit:
                self.highs += 1
            else:
                if self.highs >= self.reset_cycles:
                    self.line_resets += 1
                    if self.mode == "jtag":
                        self.phase, self.shift, self.count = "switch", 0, 1
                    else:
                        self.phase = "idle"
                self.highs = 0
        elif self.phase == "switch":
            self.shift |= bit << self.count
            self.count += 1
            if self.count == 16:
                if self.shift == SWITCH_SEQUENCE:
                    self.mode = "swd"
                    self.switches += 1
                self.phase, self.highs = "reset", 0
        elif self.phase == "idle":
            if bit:
                self.phase, self.shift, self.count = "request", 1, 1
        elif self.phase == "request":
            self.shift |= bit << self.count
            self.count += 1
            if self.count == 8:
                self._request(self.shift)

    # --------------------------------------------------------------- Model
    def observe(self, lines: Lines) -> None:
        clk = lines.get(self.clk_pad)
        if clk and not self.prev_clk:                       # rising edge
            if not self.driving and self.phase != "response":
                self._host_bit(lines.get(self.io_pad))
        elif self.prev_clk and not clk:                     # falling edge
            if self.out:
                nxt = self.out.popleft()
                self.driving = nxt is not None
                if self.driving:
                    self.value = nxt
            else:
                self.driving = False
                if self.phase == "response":
                    self.phase, self.ones = "idle", 0
        self.prev_clk = clk
