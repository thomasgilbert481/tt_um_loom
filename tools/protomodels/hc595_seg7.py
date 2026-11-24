"""A 74HC595 with a 7-segment digit on its outputs, the reference for ``firmware/hc595_seg7.loom``.

Written from the TI SN74HC595 datasheet (SCLS041), not from the firmware:

* Function table: on a rising SRCLK the shift register shifts, QA taking
  SER and each stage taking the one before (QH' is the old QH); on a rising
  RCLK the shift register is copied into the storage register. SRCLR is
  taken as tied high and OE as tied low (outputs always enabled).
* Source: SCLS041J (December 1982, revised October 2021), section 6.6
  "Timing Requirements", the SN74HC595 column (TA = -40 to 85 C, section
  6.3 "Recommended Operating Conditions"), checked against the PDF on
  2026-11-23/24 (W-126). MIN, at VCC = 4.5 V / 2 V / 6 V:

  ======================================  =====  ======  =====
  parameter                               4.5 V  2 V     6 V
  ======================================  =====  ======  =====
  tw, SRCLK or RCLK high or low           20 ns  100 ns  17 ns
  tsu, SER before SRCLK rise              25 ns  125 ns  21 ns
  tsu, SRCLK rise before RCLK rise (1)    19 ns  94 ns   16 ns
  th, SER after SRCLK rise                0 ns   0 ns    0 ns
  fclock, MAX                             25 MHz 5 MHz   29 MHz
  ======================================  =====  ======  =====

  Note (1) of the table: this set-up lets the storage register take stable
  data from the shift register. The trial version of this model (2026-11-19)
  had assumed figures, called them TA = 25 C, and had 25 ns for SRCLK before
  RCLK; the 25 C column at 4.5 V is 16, 20, 15, 0 ns and 31 MHz.
  ``vcc`` picks the column. Loom's pads are 3.3 V, between the 2 V and
  4.5 V columns, and the datasheet gives no 3.3 V figure, so the default is
  4.5 V and a test also runs the 2 V column, the slower one.
* th is 0 ns in the datasheet, so the hold check can never fire on a
  board-clock sample. A SER change on the same clock as an SRCLK rise is a
  race a sample cannot order; the reference gives the flip-flop the old SER
  and flags the change as a tsu violation (0 clocks of set-up). The trial
  version took th as 5 ns for the same purpose; that check never fired,
  because the same-clock case is seen as set-up.
* fclock is checked as the time from one SRCLK rise to the next, at least
  1 / fclock.

The display wiring is a board convention: QA..QG drive segments a..g of a
common-cathode digit, QH the decimal point. :func:`segments` turns a
storage-register byte into the set of lit segments.

:class:`HC595` sits on the three pins, keeps the shift and storage
registers, every latch as ``(cycle, byte)``, every SRCLK rise and the last
SER change before it, and lists every violation at the board clock it is
given.
"""

from __future__ import annotations

from typing import List, Optional, Set, Tuple

from tools.protomodels.bench import Lines, Model, PinSpec, pad_of

# SCLS041J 6.6, SN74HC595 column (TA = -40 to 85 C): MIN times, MAX fclock.
# VCC: (tw, tsu SER, tsu SRCLK->RCLK, th, fclock)
TIMING = {
    2.0: (100e-9, 125e-9, 94e-9, 0.0, 5e6),
    4.5: (20e-9, 25e-9, 19e-9, 0.0, 25e6),
    6.0: (17e-9, 21e-9, 16e-9, 0.0, 29e6),
}
T_W, T_SU, T_SU_RCLK, T_H, F_CLOCK = TIMING[4.5]

SEGMENT_NAMES = "abcdefg"


def segments(byte: int) -> Set[str]:
    """The lit segments of a storage-register byte: QA..QG = a..g, QH = dp."""
    lit = {SEGMENT_NAMES[i] for i in range(7) if (byte >> i) & 1}
    if byte & 0x80:
        lit.add("dp")
    return lit


class _Pulse:
    """Edges and widths of one clock line."""

    def __init__(self) -> None:
        self.level: Optional[int] = None
        self.last_edge: Optional[int] = None
        self.rises: List[int] = []
        self.highs: List[int] = []
        self.lows: List[int] = []


class HC595(Model):
    """SER, SRCLK and RCLK of a 74HC595: registers, latches and violations."""

    def __init__(self, ser: PinSpec, srclk: PinSpec, rclk: PinSpec, *,
                 clock_hz: float = 50e6, vcc: float = 4.5) -> None:
        self.ser_pad, self.srclk_pad, self.rclk_pad = pad_of(ser), pad_of(srclk), pad_of(rclk)
        self.clock_hz = float(clock_hz)
        self.vcc = vcc
        self.t_w, self.t_su, self.t_su_rclk, self.t_h, self.f_clock = TIMING[vcc]
        self.shift = 0
        self.storage = 0
        self.latches: List[Tuple[int, int]] = []         # (cycle, byte)
        self.ser_changes: List[int] = []
        self.setups: List[int] = []                       # SER change to SRCLK rise, clocks
        self.violations: List[str] = []
        self._ser: Optional[int] = None
        self._sr = _Pulse()
        self._rc = _Pulse()

    def _secs(self, clocks: int) -> float:
        return clocks / self.clock_hz

    def _width(self, p: _Pulse, name: str, c: int, was: int) -> None:
        if p.last_edge is None:
            return
        w = c - p.last_edge
        (p.highs if was else p.lows).append(w)
        if self._secs(w) < self.t_w:
            self.violations.append("cycle %d: %s %s %d clocks (tw)"
                                   % (c, name, "high" if was else "low", w))

    def observe(self, lines: Lines) -> None:
        c = lines.cycle
        ser = lines.get(self.ser_pad)
        sr = lines.get(self.srclk_pad)
        rc = lines.get(self.rclk_pad)
        if self._ser is not None and ser != self._ser:
            self.ser_changes.append(c)
            if self._sr.rises and self._secs(c - self._sr.rises[-1]) < self.t_h:
                self.violations.append("cycle %d: SER changed %d clocks after an "
                                       "SRCLK rise (th)" % (c, c - self._sr.rises[-1]))
        if self._sr.level is not None and sr != self._sr.level:
            self._width(self._sr, "SRCLK", c, self._sr.level)
            self._sr.last_edge = c
            if sr == 1:
                # The value SER had before this edge is what the flip-flop takes.
                d = self._ser if self._ser is not None else ser
                if self.ser_changes:
                    su = c - self.ser_changes[-1]
                    self.setups.append(su)
                    if self._secs(su) < self.t_su:
                        self.violations.append("cycle %d: SER changed %d clocks before "
                                               "the SRCLK rise (tsu)" % (c, su))
                if self._sr.rises and self._secs(c - self._sr.rises[-1]) * self.f_clock < 1:
                    self.violations.append("cycle %d: SRCLK rises %d clocks apart (fclock)"
                                           % (c, c - self._sr.rises[-1]))
                self.shift = ((self.shift << 1) | d) & 0xFF
                self._sr.rises.append(c)
        if self._rc.level is not None and rc != self._rc.level:
            self._width(self._rc, "RCLK", c, self._rc.level)
            self._rc.last_edge = c
            if rc == 1:
                if self._sr.rises and self._secs(c - self._sr.rises[-1]) < self.t_su_rclk:
                    self.violations.append("cycle %d: RCLK rose %d clocks after an "
                                           "SRCLK rise (tsu)" % (c, c - self._sr.rises[-1]))
                self.storage = self.shift
                self.latches.append((c, self.storage))
                self._rc.rises.append(c)
        self._ser, self._sr.level, self._rc.level = ser, sr, rc

    @property
    def srclk_rises(self) -> List[int]:
        return self._sr.rises

    @property
    def rclk_rises(self) -> List[int]:
        return self._rc.rises
