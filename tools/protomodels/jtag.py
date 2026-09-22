"""JTAG reference model: an IEEE 1149.1 TAP with an IDCODE register.

:class:`JtagTap` is the whole sixteen-state TAP controller. It samples TMS
and TDI on the rising edge of TCK and moves TDO on the falling edge, as the
standard requires, so a master that drives TMS and TDI while TCK is low and
reads TDO before the rising edge talks to it correctly and one that does not
gets wrong data rather than lucky data.

What is modelled: the state machine, the IDCODE instruction selected by
Test-Logic-Reset, BYPASS (one bit, zero after capture), the 4-bit IR with
its 0b0001 capture value, and the shift path. That is what a TAP reset and
an IDCODE read need; loading another instruction shifts the IR and then
selects BYPASS unless it is the IDCODE opcode.

:attr:`JtagTap.visited` records every state the TAP entered, so a test can
assert that the master really passed through Test-Logic-Reset and Shift-DR
rather than only that the bits came out right.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple

from .bench import Drive, Lines, Model, PinSpec, pad_of

TLR = "Test-Logic-Reset"
RTI = "Run-Test/Idle"

#: ``state -> (state if TMS == 0, state if TMS == 1)``, IEEE 1149.1 figure 6-1.
NEXT: Dict[str, Tuple[str, str]] = {
    TLR:            (RTI, TLR),
    RTI:            (RTI, "Select-DR-Scan"),
    "Select-DR-Scan": ("Capture-DR", "Select-IR-Scan"),
    "Capture-DR":   ("Shift-DR", "Exit1-DR"),
    "Shift-DR":     ("Shift-DR", "Exit1-DR"),
    "Exit1-DR":     ("Pause-DR", "Update-DR"),
    "Pause-DR":     ("Pause-DR", "Exit2-DR"),
    "Exit2-DR":     ("Shift-DR", "Update-DR"),
    "Update-DR":    (RTI, "Select-DR-Scan"),
    "Select-IR-Scan": ("Capture-IR", TLR),
    "Capture-IR":   ("Shift-IR", "Exit1-IR"),
    "Shift-IR":     ("Shift-IR", "Exit1-IR"),
    "Exit1-IR":     ("Pause-IR", "Update-IR"),
    "Pause-IR":     ("Pause-IR", "Exit2-IR"),
    "Exit2-IR":     ("Shift-IR", "Update-IR"),
    "Update-IR":    (RTI, "Select-DR-Scan"),
}

IR_BITS = 4
IR_IDCODE = 0b0001              # the opcode Test-Logic-Reset loads
IR_BYPASS = 0b1111


class JtagTap(Model):
    """A TAP on TCK/TMS/TDI/TDO.

    Args:
        tck, tms, tdi: chip outputs the TAP listens to.
        tdo: the chip input the TAP drives.
        idcode: the 32-bit IDCODE. A real one always has bit 0 set.
    """

    def __init__(self, tck: PinSpec, tms: PinSpec, tdi: PinSpec, tdo: PinSpec,
                 *, idcode: int = 0x1BA00477) -> None:
        self.tck_pad = pad_of(tck)
        self.tms_pad = pad_of(tms)
        self.tdi_pad = pad_of(tdi)
        self.tdo_pad = pad_of(tdo)
        self.idcode = idcode & 0xFFFFFFFF
        self.state = TLR
        self.ir = IR_IDCODE
        self.ir_shift = 0
        self.dr = self.idcode
        self.dr_bits = 32
        self.tdo = 0
        self.prev_tck = 0
        #: every state entered, in order (``visited[0]`` is the reset state)
        self.visited: List[str] = [TLR]
        #: every value shifted out of the IR at Update-IR
        self.ir_updates: List[int] = []
        self.shifts = 0

    # ------------------------------------------------------------- driving
    def drive(self, drive: Drive, cycle: int) -> None:
        drive.set(self.tdo_pad, self.tdo)

    # -------------------------------------------------------------- states
    def _enter(self, state: str) -> None:
        self.state = state
        self.visited.append(state)
        if state == TLR:
            self.ir = IR_IDCODE
        elif state == "Capture-DR":
            if self.ir == IR_IDCODE:
                self.dr, self.dr_bits = self.idcode, 32
            else:
                self.dr, self.dr_bits = 0, 1        # BYPASS captures a 0
        elif state == "Capture-IR":
            self.ir_shift = 0b0001                  # IEEE 1149.1 7.2.1
        elif state == "Update-IR":
            self.ir_updates.append(self.ir_shift)
            self.ir = self.ir_shift

    def observe(self, lines: Lines) -> None:
        tck = lines.get(self.tck_pad)
        if tck and not self.prev_tck:                        # rising edge
            tms = lines.get(self.tms_pad)
            tdi = lines.get(self.tdi_pad)
            if self.state == "Shift-DR":
                self.dr = (self.dr >> 1) | (tdi << (self.dr_bits - 1))
                self.shifts += 1
            elif self.state == "Shift-IR":
                self.ir_shift = ((self.ir_shift >> 1) | (tdi << (IR_BITS - 1))
                                 ) & ((1 << IR_BITS) - 1)
                self.shifts += 1
            self._enter(NEXT[self.state][tms])
        elif self.prev_tck and not tck:                      # falling edge
            if self.state == "Shift-DR":
                self.tdo = self.dr & 1
            elif self.state == "Shift-IR":
                self.tdo = self.ir_shift & 1
            else:
                self.tdo = 0
        self.prev_tck = tck

    # ---------------------------------------------------------------- view
    @property
    def reset_count(self) -> int:
        """How many times the TAP entered Test-Logic-Reset."""
        return sum(1 for s in self.visited[1:] if s == TLR)

    def reached(self, state: str) -> bool:
        return state in self.visited
