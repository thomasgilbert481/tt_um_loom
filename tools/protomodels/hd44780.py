"""HD44780U reference model: a character LCD controller on its 4-bit bus.

Written from the Hitachi HD44780U (LCD-II) datasheet, ADE-207-272(Z),
'99.9, Rev. 0.0 (60 pages; page numbers below are the printed ones), never
from the firmware. What it gives:

* p.22, "Interfacing to the MPU" and Figure 9: in 4-bit mode only DB4..DB7
  carry data, "the four high order bits ... are transferred before the four
  low order bits", and a busy-flag read is two more 4-bit operations (BF and
  AC6..AC4, then AC3..AC0).
* p.24-25, Table 6: the instruction codes and the execution times "(max)
  (when fcp or fOSC is 270 kHz)": 37 us for everything but Return home
  (1.52 ms) and the busy-flag read (0 us); Clear display has **no time
  printed** (the model gives it Return home's 1.52 ms, QUESTIONS.md). The
  times scale with the oscillator ("37 us x 270 / 250 = 40 us" at 250 kHz).
  The note: "When an instruction is being executed for internal operation,
  no instruction other than the busy flag/address read instruction can be
  executed" (p.24): an instruction or data byte that arrives while busy is
  **rejected** here and recorded as a violation.
* p.25, Figure 10: the address counter is updated t_ADD = 1.5 / fOSC after
  the busy flag clears, so a read in between shows the old address.
* p.11, Figures 4 and 5: a 2-line display (N = 1) has DDRAM 0x00..0x27 on
  line 1 and 0x40..0x67 on line 2, "not consecutive"; the address counter
  runs from 0x27 to 0x40 and from 0x67 back to 0x00.
* p.46, Figure 24, "Initializing by Instruction", 4-bit interface: power on,
  wait more than 15 ms after VCC rises to 4.5 V or more than 40 ms after it
  rises to 2.7 V; 0x3 ("BF cannot be checked before this instruction");
  more than 4.1 ms; 0x3; more than 100 us; 0x3; then 0x2 (function set,
  4 bits, sent while the interface is still 8 bits: one nibble), then the
  function set in two nibbles with N and F ("cannot be changed after this
  point"), display off, display clear, entry mode set.
* p.49 (VCC = 2.7 to 4.5 V) and p.52 (4.5 to 5.5 V), bus timing, Figures
  25 and 26 (p.58) for where each is measured: :data:`TIMING`. Both
  columns give fOSC 190 / 270 / 350 kHz (min / typ / max) for the
  oscillator resistor of that column.

The bus is digital here: one value per pad per clock, so t_Er and t_Ef
(<= 25 ns) cannot be checked and every interval is a whole number of
clocks. Each edge is the first cycle a pad shows its new value.

Everything is a registered circuit clocked by ``clk`` (the bench contract,
``tools/protomodels/bench.py``): the model sees the lines of cycle ``c``
and changes what it drives from ``c + 1``. When it reads (R/W = 1, RS = 0)
it drives DB4..DB7 push-pull from t_DDR after E rises (the maximum: the
slowest LCD) until t_DHR after E falls (the minimum: the earliest release).
"""

from __future__ import annotations

from typing import Dict, List, Optional, Sequence, Tuple

from .bench import Drive, Lines, Model, PinSpec, pad_of

CLOCK = 50_000_000
FOSC = {"min": 190_000, "typ": 270_000, "max": 350_000}   # p.49, p.52
FOSC_TABLE = 270_000                                       # Table 6's column

#: Bus timing in ns, ``(min, max)`` (None: not given), per supply column.
#: p.49 (VCC 2.7 to 4.5 V) and p.52 (4.5 to 5.5 V); Figure 25 (write) and
#: Figure 26 (read), p.58.
TIMING: Dict[str, Dict[str, Tuple[Optional[float], Optional[float]]]] = {
    "2.7-4.5V": {
        "t_cycE": (1000, None),   # E rise to the next E rise
        "PW_EH": (450, None),     # E high
        "t_Er_Ef": (None, 25),    # E rise and fall time (not checkable here)
        "t_AS": (60, None),       # RS, R/W stable before E rises
        "t_AH": (20, None),       # RS, R/W held after E falls
        "t_DSW": (195, None),     # write data stable before E falls
        "t_H": (10, None),        # write data held after E falls
        "t_DDR": (None, 360),     # read data valid after E rises
        "t_DHR": (5, None),       # read data held after E falls
    },
    "4.5-5.5V": {
        "t_cycE": (500, None), "PW_EH": (230, None), "t_Er_Ef": (None, 20),
        "t_AS": (40, None), "t_AH": (10, None), "t_DSW": (80, None),
        "t_H": (10, None), "t_DDR": (None, 160), "t_DHR": (5, None),
    },
}

#: Initialization waits of Figure 24 (p.46), in ns: "more than" each.
POWER_ON_NS = {"2.7-4.5V": 40_000_000, "4.5-5.5V": 15_000_000}
INIT_WAIT_2_NS = 4_100_000
INIT_WAIT_3_NS = 100_000

EXEC_NS = 37_000            # Table 6, at 270 kHz
EXEC_LONG_NS = 1_520_000    # Return home (and, by the model's reading, Clear)
T_ADD_CYCLES_OF_FOSC = 1.5  # Figure 10: t_ADD = 1.5 / fOSC

LINE_START = (0x00, 0x40)   # Figure 4, p.11
LINE_LEN = 40               # 0x00..0x27, 0x40..0x67


class Hd44780(Model):
    """The controller and a 2x16 (or 1-line) display on Loom's pins.

    Args:
        rs, rw, e: the three control inputs (Loom outputs).
        data: DB4..DB7, in that order (Loom BIDIR pins).
        clock: the bench clock in Hz, for every check in ns.
        fosc: the LCD oscillator in Hz: the execution times are Table 6's
            scaled by 270 kHz / fosc. Default the slowest of p.49, 190 kHz.
        supply: ``"2.7-4.5V"`` or ``"4.5-5.5V"``: the timing column, and
            the power-on wait of Figure 24.
        powered_at: the cycle VCC reached its level (negative: before the
            bench started).
        rw_wired: False when the board ties R/W low: the model then sees R/W
            as 0 whatever Loom drives.
        cols: the visible columns per line, for :meth:`render`.
    """

    def __init__(self, *, rs: PinSpec = "OUT0", rw: PinSpec = "OUT1",
                 e: PinSpec = "OUT2",
                 data: Sequence[PinSpec] = ("BIDIR0", "BIDIR1", "BIDIR2", "BIDIR3"),
                 clock: int = CLOCK, fosc: int = FOSC["min"],
                 supply: str = "2.7-4.5V", powered_at: int = 0,
                 rw_wired: bool = True, cols: int = 16) -> None:
        self.rs_pad, self.rw_pad, self.e_pad = pad_of(rs), pad_of(rw), pad_of(e)
        self.data_pads = [pad_of(p) for p in data]
        self.clock, self.fosc, self.supply = clock, fosc, supply
        self.timing = TIMING[supply]
        self.powered_at, self.rw_wired, self.cols = powered_at, rw_wired, cols
        self.violations: List[Tuple[int, str, str]] = []
        self.events: List[tuple] = []        # (cycle, kind, rs, value)
        self.measured: Dict[str, List[float]] = {k: [] for k in (
            "t_cycE", "PW_EH", "t_AS", "t_AH", "t_DSW", "t_H")}
        # Lines and their last changes.
        self.e = self.rs = self.rw = 0
        self.db = 0
        self.e_rise: Optional[int] = None
        self.e_fall: Optional[int] = None
        self.prev_rise: Optional[int] = None
        self.rsrw_change = -10 ** 9
        self.db_change = -10 ** 9
        self.cycle = -1
        # Reading: the nibble driven, from when, until when.
        self.read_value = 0
        self.read_from: Optional[int] = None
        self.read_until: Optional[int] = None
        # The controller.
        self.eight_bit = True          # after power on (p.23, internal reset: DL = 1)
        self.nibble_hi: Optional[int] = None   # the first nibble of a byte
        self.pair_rsrw: Optional[Tuple[int, int]] = None
        self.init_step = 0             # 0..4 through Figure 24, 5 done
        self.init_last: Optional[int] = None
        self.function_set_done = False
        self.lines = 1
        self.font = 0
        self.ddram = [0x20] * 0x80
        self.ac = 0
        self.ac_pending: Optional[Tuple[int, int]] = None   # (cycle, value)
        self.increment, self.shift = True, False
        self.display = self.cursor = self.blink = False
        self.display_shift = 0
        self.busy_until = -1
        self.executed: List[Tuple[int, int, int]] = []   # (cycle, rs, byte)
        self.rejected: List[Tuple[int, int, int]] = []

    # ------------------------------------------------------------ helpers
    def ns(self, cycles: float) -> float:
        return cycles * 1e9 / self.clock

    def cycles(self, ns: float) -> int:
        """The first whole number of clocks that is at least ``ns``."""
        c = ns * self.clock / 1e9
        return int(c) if c == int(c) else int(c) + 1

    def _violate(self, name: str, detail: str) -> None:
        self.violations.append((self.cycle, name, detail))

    def _check_min(self, name: str, cycles: int) -> None:
        value = self.ns(cycles)
        if name in self.measured:
            self.measured[name].append(value)
        low = self.timing[name][0]
        if low is not None and value < low:
            self._violate(name, "%.1f ns < %s ns" % (value, low))

    @property
    def busy(self) -> bool:
        return self.cycle < self.busy_until

    def _exec_cycles(self, long: bool) -> int:
        ns = (EXEC_LONG_NS if long else EXEC_NS) * FOSC_TABLE / self.fosc
        return self.cycles(ns)

    def _ac_now(self) -> int:
        if self.ac_pending is not None and self.cycle >= self.ac_pending[0]:
            self.ac = self.ac_pending[1]
            self.ac_pending = None
        return self.ac

    # -------------------------------------------------------------- drive
    def drive(self, drive: Drive, cycle: int) -> None:
        if self.read_from is None or cycle < self.read_from:
            return
        if self.read_until is not None and cycle >= self.read_until:
            self.read_from = self.read_until = None
            return
        for i, pad in enumerate(self.data_pads):
            drive.set(pad, (self.read_value >> i) & 1)

    # ------------------------------------------------------------ observe
    def observe(self, lines: Lines) -> None:
        self.cycle = c = lines.cycle
        e = lines.get(self.e_pad)
        rs = lines.get(self.rs_pad)
        rw = lines.get(self.rw_pad) if self.rw_wired else 0
        db = 0
        for i, pad in enumerate(self.data_pads):
            db |= lines.get(pad) << i
        if (rs, rw) != (self.rs, self.rw):
            if self.e:
                self._violate("t_AS", "RS or R/W changed while E is high")
            elif self.e_fall is not None and self.e_fall > self.rsrw_change:
                self._check_min("t_AH", c - self.e_fall)
            self.rsrw_change = c
        old_db = self.db
        if db != old_db and not rw and not self.rw:
            if (self.e_fall is not None and self.e_fall > self.db_change
                    and self.last_write and not self.e):
                self._check_min("t_H", c - self.e_fall)
            self.db_change = c
        if e and not self.e:
            self._rise(c, rs, rw)
        elif self.e and not e:
            self._fall(c, rs, rw, old_db)
        self.e, self.rs, self.rw, self.db = e, rs, rw, db

    last_write = False

    def _rise(self, c: int, rs: int, rw: int) -> None:
        if self.prev_rise is not None:
            self._check_min("t_cycE", c - self.prev_rise)
        self._check_min("t_AS", c - self.rsrw_change)
        self.e_rise = self.prev_rise = c
        if not rw:
            return
        if rs:
            self._violate("read", "data read (RS = 1, R/W = 1) is not modelled")
            return
        if self.init_step < 3:
            self._violate("init", "busy flag read before it can be checked (Figure 24)")
        first = self.eight_bit or self.nibble_hi is None
        if first:
            self.read_byte = (0x80 if self.busy else 0) | self._ac_now()
            self.events.append((c, "read", rs, self.read_byte))
        nibble = self.read_byte >> 4 if first else self.read_byte & 0xF
        self.read_value = nibble
        self.read_from = c + self.cycles(self.timing["t_DDR"][1])
        self.read_until = None

    read_byte = 0

    def _fall(self, c: int, rs: int, rw: int, nibble: int) -> None:
        self._check_min("PW_EH", c - self.e_rise)
        self.e_fall = c
        self.last_write = not rw
        if rw:
            self.read_until = c + max(1, self.cycles(self.timing["t_DHR"][0]))
            self._pair(rs, rw, None)
            return
        self._check_min("t_DSW", c - self.db_change)
        if self.eight_bit:
            self._execute(rs, nibble << 4, "nibble8")   # DB0..DB3 read as 0
            return
        high = self._pair(rs, rw, nibble)
        if high is not None:
            self._execute(rs, high << 4 | nibble, "byte")

    def _pair(self, rs: int, rw: int, nibble: Optional[int]) -> Optional[int]:
        """4-bit mode: the first nibble is kept, the second completes the
        transfer and returns the first (writes) or None. Both must have the
        same RS and R/W (p.22: one transfer is two 4-bit operations)."""
        if self.eight_bit:
            return None
        if self.nibble_hi is None:
            self.nibble_hi = -1 if nibble is None else nibble
            self.pair_rsrw = (rs, rw)
            return None
        high, self.nibble_hi = self.nibble_hi, None
        if self.pair_rsrw != (rs, rw):
            self._violate("nibbles", "RS or R/W differ between the two nibbles")
            return None
        return None if nibble is None else high

    # ------------------------------------------------------------ execute
    def _init_check(self, c: int, rs: int, byte: int) -> None:
        """Figure 24, step by step: what the step needs, and its wait."""
        step = self.init_step
        need = (3, 3, 3, 2)[step] if step < 4 else None
        since = c - (self.powered_at if step == 0 else (self.init_last or 0))
        wait = {0: POWER_ON_NS[self.supply], 1: INIT_WAIT_2_NS, 2: INIT_WAIT_3_NS}.get(step)
        if step < 4:
            if rs or byte >> 4 != need:
                self._violate("init", "step %d: 0x%X sent, Figure 24 has 0x%X"
                              % (step + 1, byte >> 4, need))
            if wait is not None and self.ns(since) <= wait:
                self._violate("init", "step %d: %.0f ns after the last, needs more than %d"
                              % (step + 1, self.ns(since), wait))
        elif step == 4 and (rs or byte & 0xF0 != 0x20):
            self._violate("init", "step 5: 0x%02X sent, Figure 24 has the 4-bit function set" % byte)
        self.init_step = min(step + 1, 5)
        self.init_last = c

    def _execute(self, rs: int, byte: int, kind: str) -> None:
        c = self.cycle
        self.events.append((c, kind, rs, byte))
        if self.init_step < 5:
            self._init_check(c, rs, byte)
        if self.init_step > 3 and self.busy:
            self.rejected.append((c, rs, byte))
            self._violate("busy", "%s 0x%02X sent %.0f ns before the busy flag clears"
                          % ("data" if rs else "instruction", byte,
                             self.ns(self.busy_until - c)))
            return
        long = False
        ac = self._ac_now()
        if rs:
            self._store(byte)
        elif byte == 0:
            self._violate("instruction", "0x00 is no instruction")
        elif byte == 0x01:                       # Clear display
            self.ddram = [0x20] * 0x80
            self.ac, self.increment, self.display_shift = 0, True, 0
            self.target, long = "dd", True
        elif byte >> 1 == 0x01:                  # Return home
            self.ac, self.display_shift, self.target, long = 0, 0, "dd", True
        elif byte >> 2 == 0x01:                  # Entry mode set
            self.increment, self.shift = bool(byte & 2), bool(byte & 1)
        elif byte >> 3 == 0x01:                  # Display on/off control
            self.display, self.cursor, self.blink = (bool(byte & 4), bool(byte & 2),
                                                     bool(byte & 1))
        elif byte >> 4 == 0x01:                  # Cursor or display shift
            step = 1 if byte & 4 else -1
            if byte & 8:
                self.display_shift = (self.display_shift - step) % LINE_LEN
            else:
                self.ac = self._advance(ac, step > 0)
        elif byte >> 5 == 0x01:                  # Function set
            self.eight_bit = bool(byte & 0x10)
            if self.eight_bit:
                self.function_set_done = False    # a re-initialization
            if not self.function_set_done and not self.eight_bit and kind == "byte":
                self.lines, self.font = (2 if byte & 8 else 1), (byte >> 2) & 1
                self.function_set_done = True
            self.nibble_hi = None
        elif byte >> 6 == 0x01:                  # Set CGRAM address
            self.ac, self.target = byte & 0x3F, "cg"
        else:                                    # Set DDRAM address
            addr = byte & 0x7F
            if self.lines == 2 and (addr % 0x40) >= LINE_LEN or addr >= 0x50 and self.lines == 1:
                self._violate("address", "DDRAM address 0x%02X is not on the display" % addr)
            self.ac, self.target = addr, "dd"
        self.busy_until = c + self._exec_cycles(long)
        self.executed.append((c, rs, byte))

    target = "dd"

    def _advance(self, ac: int, up: bool) -> int:
        if self.target == "cg":
            return (ac + (1 if up else -1)) & 0x3F
        if self.lines == 1:
            return (ac + (1 if up else -1)) % 0x50
        if up:
            return 0x40 if ac == 0x27 else 0x00 if ac == 0x67 else ac + 1
        return 0x27 if ac == 0x40 else 0x67 if ac == 0x00 else ac - 1

    def _store(self, byte: int) -> None:
        """A data write: into DDRAM or CGRAM at AC, then AC moves t_ADD after
        the busy flag clears (Figure 10)."""
        ac = self._ac_now()
        if self.target == "dd":
            self.ddram[ac & 0x7F] = byte
        else:
            self.cgram = getattr(self, "cgram", [0] * 64)
            self.cgram[ac & 0x3F] = byte & 0x1F
        if self.shift:
            self.display_shift = (self.display_shift + (1 if self.increment else -1)) % LINE_LEN
        busy_end = self.cycle + self._exec_cycles(False)
        t_add = self.cycles(T_ADD_CYCLES_OF_FOSC * 1e9 / self.fosc)
        self.ac_pending = (busy_end + t_add, self._advance(ac, self.increment))

    # ------------------------------------------------------------- views
    @property
    def initialized(self) -> bool:
        """Figure 24 run to its end: the 4-bit function set taken."""
        return self.init_step == 5 and not self.eight_bit

    def line(self, row: int) -> str:
        """What line ``row`` shows (DDRAM through the display shift), as text.
        Codes 0x20..0x7D are taken as ASCII, as ROM code A00 draws them
        (Table 4, p.17), except 0x5C, which A00 draws as a yen sign: it and
        every other code show as ``?``."""
        start = LINE_START[row]
        out = []
        for j in range(self.cols):
            code = self.ddram[start + (j + self.display_shift) % LINE_LEN]
            out.append(chr(code) if 0x20 <= code <= 0x7D and code != 0x5C else "?")
        return "".join(out)

    def render(self) -> List[str]:
        """The display as text, one string per line (2x16 by default)."""
        return [self.line(r) for r in range(self.lines)]

    def bytes_written(self) -> List[Tuple[int, int]]:
        """``(rs, byte)`` of every transfer executed, in order."""
        return [(rs, b) for _, rs, b in self.executed]
