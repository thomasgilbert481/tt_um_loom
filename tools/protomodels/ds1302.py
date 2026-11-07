"""DS1302 reference models: the register file, the chip, a host, a wire, a checker.

Written from the Dallas Semiconductor DS1302 data sheet dated 070900 (14
pages; the copy at rcscomponents.kiev.ua/datasheets/DS1302.pdf), never from
the firmware. The Maxim 19-5420 revision could not be fetched (analog.com
timed out), so its figure and table numbers are not given. What the 070900
copy gives, by page:

* p.3, "Command byte" (Figure 2): bit 7 "must be a logic 1. If it is 0,
  writes to the DS1302 will be disabled"; bit 6 clock (0) or RAM (1); bits
  5:1 the register; bit 0 write (0) or read (1); "always input starting with
  the LSB". "Reset and clock control": transfers start with RST (CE) high;
  "If the RST input is low all data transfer terminates and the I/O pin goes
  to a high impedance state"; "SCLK must be at a logic 0 when RST is driven
  to a logic 1 state"; inputs valid at the rising edge, outputs on the
  falling edge. "Data input": the byte at the next eight rises,
  "Additional SCLK cycles are ignored". "Data output": "the first data bit to
  be transmitted occurs on the first falling edge after the last bit of the
  command byte is written. Additional SCLK cycles retransmit the data bytes
  ... so long as RST remains high"; "the I/O pin is tristated upon each
  rising edge of SCLK". "Burst mode": address 31.
* p.4: no storage at clock addresses 9..31 or RAM 31; bursts start at
  address 0; a clock burst write must write the first eight registers "in
  order for the data to be transferred"; a RAM burst stores each byte
  written. Clock halt (seconds bit 7) and write protect (control bit 7):
  initial states "not defined"; the control register's bits 0..6 "are
  forced to 0 and will always read a 0"; WP "prevents a write operation to
  any other register". The trickle charger: TCS bits 7:4 = 1010 enables, DS
  bits 3:2, RS bits 1:0; "powers up with the trickle charger disabled".
* p.5: with WP set "no data transfer will occur to any of the eight
  clock/calendar registers (this includes the control register)" in a clock
  burst write; "The trickle charger is not accessible in burst mode"; a
  clock burst read copies the time to secondary registers first (the time
  here does not run, so reading the primaries is the same thing).
* p.6, Figure 3: single-byte read and write: R/W, A0..A4, R/C, 1, then
  D0..D7, all LSB first.
* p.7, Figure 4, the register map, is an image: only what the text says of
  it is used (the command bytes follow from Figure 2: 80H/81H seconds ..
  8EH/8FH control, 90H/91H trickle, BEH/BFH clock burst, C0H..FDH RAM,
  FEH/FFH RAM burst).
* p.10, AC electrical characteristics, VCC = 2.0 V and 5.0 V: :data:`TIMING`.
  The table's min and max columns came out of the text extraction as one
  number per row; which column each sits in is read from the parameter
  (a set-up, hold, width or inactive time is a minimum, a delay to valid or
  to high impedance, a frequency or a rise time a maximum), which is the
  data sheet's usual layout but **unchecked** against the drawing.
* p.11, Figures 5 and 6 (read and write timing) are drawings: which SCLK
  edge tCCH is measured from is **unchecked**; the checker takes the last
  SCLK edge of either kind before CE falls (the stricter reading).
* p.11 note 6: RST, SCLK and I/O have 40 kOhm pull-downs: a released I/O
  reads 0 (the bench's default: no pull-up).

Everything here is a registered circuit clocked by ``clk`` (the bench
contract, ``tools/protomodels/bench.py``): a model sees the lines of cycle
``c`` and changes what it drives from ``c + 1``.
"""

from __future__ import annotations

from collections import deque
from typing import Dict, List, Optional, Sequence, Tuple

from .bench import UI, UIO, UO, Drive, Lines, Model, PinSpec, pad_of

CLOCK = 50_000_000

READ = 0x01                       # command bit 0
RAM_BIT = 0x40                    # command bit 6
BURST = 31                        # the burst address
SECONDS, MINUTES, HOURS, DATE, MONTH, DAY, YEAR, CONTROL, TRICKLE = range(9)
WP = 0x80                         # control bit 7
CH = 0x80                         # seconds bit 7, clock halt
N_BURST = 8                       # clock burst registers (0..7, CONTROL last)
N_RAM = 31


def command(address: int, *, ram: bool = False, read: bool = False) -> int:
    """The command byte of Figure 2: ``1 RAM A4..A0 RD``."""
    if not 0 <= address <= BURST:
        raise ValueError("address %d" % address)
    return 0x80 | (RAM_BIT if ram else 0) | address << 1 | (READ if read else 0)


CLOCK_BURST_W, CLOCK_BURST_R = command(BURST), command(BURST, read=True)
RAM_BURST_W, RAM_BURST_R = command(BURST, ram=True), command(BURST, ram=True, read=True)
CONTROL_W = command(CONTROL)

#: p.10, ``name -> (value at 2.0 V, value at 5.0 V, "min" or "max", unit)``.
TIMING: Dict[str, Tuple[float, float, str, str]] = {
    "tDC": (200, 50, "min", "ns"),       # data to CLK set-up
    "tCDH": (280, 70, "min", "ns"),      # CLK to data hold
    "tCDD": (800, 200, "max", "ns"),     # CLK to data delay
    "tCL": (1000, 250, "min", "ns"),     # CLK low time
    "tCH": (1000, 250, "min", "ns"),     # CLK high time
    "fCLK": (0.5, 2.0, "max", "MHz"),    # CLK frequency (from DC)
    "tR": (2000, 500, "max", "ns"),      # CLK rise and fall
    "tCC": (4000, 1000, "min", "ns"),    # RST (CE) to CLK set-up
    "tCCH": (240, 60, "min", "ns"),      # CLK to RST hold
    "tCWH": (4000, 1000, "min", "ns"),   # RST inactive time
    "tCDZ": (280, 70, "max", "ns"),      # RST to I/O high Z
    "tCCZ": (280, 70, "max", "ns"),      # SCLK to I/O high Z
}


def limit(name: str, vcc: float) -> float:
    """The p.10 figure for ``vcc`` 2.0 or 5.0 (ns, or MHz for fCLK)."""
    v2, v5, _kind, _unit = TIMING[name]
    if vcc == 2.0:
        return v2
    if vcc == 5.0:
        return v5
    raise ValueError("the table gives 2.0 V and 5.0 V only, not %r" % vcc)


# ------------------------------------------------------------ the registers
class Registers:
    """The DS1302's registers as the data sheet's text describes them.

    ``clock`` holds registers 0..7 (seconds .. year, then the control
    register), ``trickle`` register 8, ``ram`` the 31 bytes. The power-on
    state is "not defined" for CH and WP (p.4); the default here is the one
    ds1302_device.loom documents (clock halted, WP set, date 01/01/00, day
    1, trickle charger disabled, RAM 0), and any other can be given.
    """

    def __init__(self, clock: Optional[Sequence[int]] = None, trickle: int = 0,
                 ram: Optional[Sequence[int]] = None) -> None:
        self.clock = list(clock if clock is not None else
                          [CH, 0, 0, 0x01, 0x01, 0x01, 0x00, WP])
        self.trickle = trickle & 0xFF
        self.ram = list(ram if ram is not None else [0] * N_RAM)
        assert len(self.clock) == N_BURST and len(self.ram) == N_RAM
        self.clock[CONTROL] &= WP                   # bits 6:0 read 0 (p.4)

    @property
    def wp(self) -> bool:
        return bool(self.clock[CONTROL] & WP)

    def read(self, cmd: int, index: int = 0) -> int:
        """Byte ``index`` of the read that ``cmd`` starts: a single read
        retransmits its byte; a burst runs on from address 0 (wrapping after
        the last register: the data sheet does not say, see QUESTIONS)."""
        address, ram = cmd >> 1 & 31, bool(cmd & RAM_BIT)
        if ram:
            return self.ram[index % N_RAM] if address == BURST else self.ram[address]
        if address == BURST:
            return self.clock[index % N_BURST]
        if address < N_BURST:
            return self.clock[address]
        if address == TRICKLE:
            return self.trickle
        return 0                                    # 9..30: no storage (p.4)

    def write(self, cmd: int, data: Sequence[int]) -> int:
        """A write transfer whose complete data bytes were ``data``; returns
        how many bytes were stored."""
        if not cmd & 0x80 or cmd & READ or not data:
            return 0
        address, ram = cmd >> 1 & 31, bool(cmd & RAM_BIT)
        data = [b & 0xFF for b in data]
        if ram and address == BURST:
            if self.wp:
                return 0
            n = min(len(data), N_RAM)
            self.ram[:n] = data[:n]
            return n
        if address == BURST:                        # the clock burst (p.5)
            if self.wp or len(data) < N_BURST:
                return 0
            self.clock = data[:N_BURST]
            self.clock[CONTROL] &= WP
            return N_BURST
        byte = data[0]                              # more bytes are ignored
        if not ram and address == CONTROL:          # writable under WP
            self.clock[CONTROL] = byte & WP
            return 1
        if self.wp:
            return 0
        if ram:
            self.ram[address] = byte
        elif address < N_BURST:
            self.clock[address] = byte
        elif address == TRICKLE:
            self.trickle = byte
        else:
            return 0
        return 1


# ------------------------------------------------------------ the chip
class Ds1302Chip(Model):
    """The chip on three pads. CE and SCLK are read (the host drives them),
    I/O is driven push-pull while the chip sends a read's bit and released
    otherwise (the pull-down then holds it low).

    Delays in clocks, inside the p.10 limits at 50 MHz: ``tcdd`` from an
    SCLK fall to the bit on I/O (<= 200 ns at 5 V: 10 clocks), ``tccz`` from
    a rise to the release (<= 70 ns: 3), ``tcdz`` from CE falling to the
    release (<= 70 ns: 3).

    A command whose bit 7 is 0 is ignored, reads included (the data sheet
    says only that writes are disabled; not driving is the safe reading).
    A write is applied when CE falls, from the bytes completed by then: for
    every rule of :meth:`Registers.write` that is the same thing as storing
    each byte when it completes, since nothing reads the registers inside
    the transfer.
    """

    def __init__(self, ce: PinSpec = "OUT0", sclk: PinSpec = "OUT1",
                 io: PinSpec = "BIDIR0", registers: Optional[Registers] = None,
                 *, tcdd: int = 4, tccz: int = 2, tcdz: int = 2) -> None:
        self.ce, self.sclk, self.io = pad_of(ce), pad_of(sclk), pad_of(io)
        if self.io[0] != UIO:
            raise ValueError("I/O must be a uio pad")
        self.regs = registers or Registers()
        self.tcdd, self.tccz, self.tcdz = max(tcdd, 1), max(tccz, 1), max(tcdz, 1)
        self.drive_on = self.driving = False
        self.drive_val = 0
        self.events: List[Tuple[int, Optional[int]]] = []   # (cycle, value or None)
        self.prev_ce = self.prev_sclk = 0
        self.active = False
        self.bits: List[int] = []
        self.cmd: Optional[int] = None
        self.data: List[int] = []
        self.reading = False
        self.index = self.bitpos = 0
        self.transfers: List[Tuple[int, List[int], int]] = []   # (cmd, bytes, stored)
        self.sclk_high_at_ce_rise = 0

    def drive(self, drive: Drive, cycle: int) -> None:
        keep = []
        for due, value in self.events:
            if due <= cycle:
                self.drive_on, self.drive_val = value is not None, value or 0
            else:
                keep.append((due, value))
        self.events = keep
        self.driving = self.drive_on
        if self.drive_on:
            drive.set(self.io, self.drive_val)

    def _at(self, cycle: int, delay: int, value: Optional[int]) -> None:
        self.events.append((cycle + delay, value))

    def observe(self, lines: Lines) -> None:
        ce, sclk, io = lines.get(self.ce), lines.get(self.sclk), lines.get(self.io)
        c = lines.cycle
        if ce and not self.prev_ce:
            self.active, self.bits, self.cmd, self.data = True, [], None, []
            self.reading, self.index, self.bitpos = False, 0, 0
            self.sclk_high_at_ce_rise += sclk
        elif not ce and self.prev_ce and self.active:
            self.active = False
            # CE low ends the transfer: a bit still on its way is dropped
            self.events = [(due, v) for due, v in self.events if v is None]
            if self.reading or self.drive_on:
                self._at(c, self.tcdz, None)
            if self.reading:                        # the whole bytes sent
                sent = [self.regs.read(self.cmd, i) for i in range(self.index)]
                self.transfers.append((self.cmd, sent, 0))
            elif self.cmd is not None:
                stored = self.regs.write(self.cmd, self.data)
                self.transfers.append((self.cmd, list(self.data), stored))
        elif ce and self.active:
            if sclk and not self.prev_sclk:
                self._rise(c, io)
            elif not sclk and self.prev_sclk and self.reading:
                bit = self.regs.read(self.cmd, self.index) >> self.bitpos & 1
                self._at(c, self.tcdd, bit)
                self.bitpos += 1
                if self.bitpos == 8:
                    self.bitpos, self.index = 0, self.index + 1
        self.prev_ce, self.prev_sclk = ce, sclk

    def _rise(self, c: int, io: int) -> None:
        if self.reading:
            self._at(c, self.tccz, None)             # tristated at each rise
            return
        self.bits.append(io)
        if len(self.bits) % 8:
            return
        byte = sum(b << i for i, b in enumerate(self.bits[-8:]))
        if self.cmd is None:
            self.cmd = byte
            if not byte & 0x80:
                self.active = False                 # not a command: ignored
                self.cmd = None
            elif byte & READ:
                self.reading = True
        else:
            self.data.append(byte)


# ------------------------------------------------------------ a host
class Ds1302Host(Model):
    """A microcontroller driving the interface, at timings given in clocks.

    The defaults are the data sheet's 2.0 V minima at 50 MHz (p.10): SCLK
    low and high 1 us each (500 kHz, the maximum), CE to the first rise 4
    us, CE low 4 us between transfers, the last fall to CE falling 240 ns;
    the host changes I/O ``hold`` clocks after each fall (280 ns, tCDH from
    the previous rise is then the whole high phase plus that), samples the
    chip's bit in the last clock before each rise, and on a read lets go of
    I/O ``release`` clocks after the eighth command rise (tCDH, 280 ns).

    Queue operations with :meth:`read` and :meth:`write`; each finished one
    is appended to :attr:`results` as ``(cmd, bytes)``.
    """

    def __init__(self, ce: PinSpec = "IN0", sclk: PinSpec = "IN1",
                 io: PinSpec = "BIDIR1", *, low: int = 50, high: int = 50,
                 hold: int = 14, release: int = 14, cc: int = 200,
                 cch: int = 12, cwh: int = 200) -> None:
        self.ce_pad, self.sclk_pad, self.io_pad = pad_of(ce), pad_of(sclk), pad_of(io)
        if self.ce_pad[0] != UI or self.sclk_pad[0] != UI or self.io_pad[0] != UIO:
            raise ValueError("CE and SCLK are ui pads, I/O a uio pad")
        assert 0 <= hold < low and 0 < release < high and cc > low
        self.low, self.high, self.hold, self.release = low, high, hold, release
        self.cc, self.cch, self.cwh = cc, cch, cwh
        self.ce = self.sclk = self.io = self.io_oe = self.driving = 0
        self.line_io = 0
        self.queue: deque = deque()
        self.results: List[Tuple[int, List[int]]] = []
        self.wait = 1
        self._gen = self._run()

    def read(self, cmd: int, n: int = 1) -> None:
        self.queue.append((cmd | READ, n, None))

    def write(self, cmd: int, data: Sequence[int]) -> None:
        self.queue.append((cmd & ~READ, len(data), list(data)))

    @property
    def busy(self) -> bool:
        return bool(self.queue) or self.ce == 1

    def drive(self, drive: Drive, cycle: int) -> None:
        drive.set(self.ce_pad, self.ce)
        drive.set(self.sclk_pad, self.sclk)
        self.driving = self.io_oe
        if self.io_oe:
            drive.set(self.io_pad, self.io)

    def observe(self, lines: Lines) -> None:
        self.line_io = lines.get(self.io_pad)
        self.wait -= 1
        while self.wait <= 0:
            self.wait = next(self._gen)

    def _clock(self, bit: Optional[int], first: bool, turn: bool = False):
        """One SCLK cycle from the fall before it (or from CE's rise). Sends
        ``bit`` (None: receives); returns the line sampled before the rise."""
        if not first and self.hold:
            yield self.hold
        if bit is not None:
            self.io_oe, self.io = 1, bit
        yield self.low - (0 if first else self.hold)
        sampled = self.line_io
        self.sclk = 1
        if turn:
            yield self.release
            self.io_oe = 0                          # the chip drives from the fall
            yield self.high - self.release
        else:
            yield self.high
        self.sclk = 0
        return sampled

    def _run(self):
        while True:
            if not self.queue:
                yield 1
                continue
            cmd, n, data = self.queue.popleft()
            self.ce = 1                             # SCLK is low (p.3)
            yield self.cc - self.low
            for k in range(8):
                yield from self._clock(cmd >> k & 1, k == 0, data is None and k == 7)
            got = []
            for i in range(n):
                byte = 0
                for k in range(8):
                    bit = None if data is None else data[i] >> k & 1
                    byte |= (yield from self._clock(bit, False)) << k
                got.append(byte if data is None else data[i])
            yield self.cch
            self.ce, self.io_oe = 0, 0
            self.results.append((cmd, got))
            yield self.cwh


# ------------------------------------------------------------ the board
class Ds1302Wire(Model):
    """The board between two Loom threads: CE and SCLK from the host
    thread's OUT pads to the device thread's IN pads, and I/O one push-pull
    line joining two ``uio`` pads. Each side follows the other's own drive
    (``uio_oe``, ``uio_out``) one clock later, as a registered model must.
    Cycles in which both pads drive are kept in :attr:`contentions` (with
    the two values): the turnaround rule says there must be none."""

    def __init__(self, links: Sequence[Tuple[PinSpec, PinSpec]] = (("OUT0", "IN0"), ("OUT1", "IN1")),
                 io: Tuple[PinSpec, PinSpec] = ("BIDIR0", "BIDIR1")) -> None:
        self.links = [(pad_of(a), pad_of(b)) for a, b in links]
        self.a, self.b = pad_of(io[0]), pad_of(io[1])
        if self.a[0] != UIO or self.b[0] != UIO:
            raise ValueError("I/O joins two uio pads")
        self.levels = [0] * len(self.links)
        self.drv_a = self.drv_b = None              # what each pad's chip drives
        self.contentions: List[Tuple[int, int, int]] = []

    def drive(self, drive: Drive, cycle: int) -> None:
        for (src, dst), level in zip(self.links, self.levels):
            drive.set(dst, level)
        if self.drv_a is not None:
            drive.set(self.b, self.drv_a)
        if self.drv_b is not None:
            drive.set(self.a, self.drv_b)

    def observe(self, lines: Lines) -> None:
        self.levels = [lines.get(src) for src, _dst in self.links]
        oe, out = lines.uio_oe, lines.uio_out
        a, b = self.a[1], self.b[1]
        self.drv_a = (out >> a & 1) if oe >> a & 1 else None
        self.drv_b = (out >> b & 1) if oe >> b & 1 else None
        if self.drv_a is not None and self.drv_b is not None:
            self.contentions.append((lines.cycle, self.drv_a, self.drv_b))


class Probe(Model):
    """Records the bus where one side sits: CE, SCLK and the I/O line as
    resolved, and who drives I/O (``host(lines)`` and ``device(lines)``
    return 1 while that side drives). :attr:`edges` lists ``(cycle, ce,
    sclk, io, host_drives, device_drives)`` at every change."""

    def __init__(self, ce: PinSpec, sclk: PinSpec, io: PinSpec, host, device,
                 clock: int = CLOCK) -> None:
        self.ce, self.sclk, self.io = pad_of(ce), pad_of(sclk), pad_of(io)
        self.host, self.device = host, device
        self.clock_hz = clock
        self.edges: List[Tuple[int, int, int, int, int, int]] = []
        self._last = None

    def observe(self, lines: Lines) -> None:
        state = (lines.get(self.ce), lines.get(self.sclk), lines.get(self.io),
                 int(bool(self.host(lines))), int(bool(self.device(lines))))
        if state != self._last:
            self.edges.append((lines.cycle,) + state)
            self._last = state


def loom_drives(pin: PinSpec):
    """``Probe`` side for a Loom thread's BIDIR pad: its ``uio_oe`` bit."""
    bit = pad_of(pin)[1]
    return lambda lines: lines.uio_oe >> bit & 1


def model_drives(model):
    """``Probe`` side for a model: its ``driving`` flag, which its ``drive``
    sets for the cycle (so the order of the models' ``observe`` calls does
    not matter)."""
    return lambda lines: model.driving


# ------------------------------------------------------------ the checker
def check(edges: Sequence[Tuple[int, int, int, int, int, int]], clock_hz: int = CLOCK,
          vcc: float = 2.0, *, device_figures: bool = True) -> Dict[str, object]:
    """Every edge of a :class:`Probe` against p.10 at ``vcc`` (2.0 or 5.0).

    Host-side figures: tCL, tCH, fCLK, tCC, tCCH, tCWH, tDC, tCDH, SCLK low
    when CE rises. Device-side (``device_figures``): tCDD, tCCZ, tCDZ. And
    contention: no cycle with both sides driving I/O. Returns
    ``{"violations": [(name, cycle, measured_ns, limit_ns)], "transfers":
    [(cmd, [bytes])], name: [measured_ns, ...] for every figure}``.
    """
    ns = 1e9 / clock_hz
    out: Dict[str, list] = {k: [] for k in
                            ("tCL", "tCH", "period", "tCC", "tCCH", "tCWH", "tDC",
                             "tCDH", "tCDD", "tCCZ", "tCDZ")}
    bad: List[Tuple[str, int, float, float]] = []

    def need(name, cycle, clocks, maximum=False, value=None):
        t = clocks * ns
        lim = value if value is not None else limit(name, vcc)
        out["period" if name == "fCLK" else name].append(t)
        if (t > lim) if maximum else (t < lim):
            bad.append((name, cycle, t, lim))

    transfers: List[Tuple[int, List[int]]] = []
    prev = None
    ce_rise = ce_fall = rise = fall = None
    io_change = hold_from = cdd_from = None
    bits: List[int] = []
    reading = contended = False
    for cyc, ce, sclk, io, h, d in edges:
        if prev is None:
            prev = (cyc, ce, sclk, io, h, d)
            continue
        _, pce, psclk, pio, ph, pd = prev
        if h and d and not contended:
            bad.append(("contention", cyc, 0.0, 0.0))
        contended = bool(h and d)
        if (h and (io != pio or not ph)) or (ph and not h):
            if hold_from is not None:               # the host's data moved
                need("tCDH", cyc, cyc - hold_from)
                hold_from = None
            io_change = cyc
        if ce and not pce:
            if sclk:
                bad.append(("SCLK high at CE rise", cyc, 0.0, 0.0))
            if ce_fall is not None:
                need("tCWH", cyc, cyc - ce_fall)
            ce_rise, rise, fall, bits, reading = cyc, None, None, [], False
        if ce and pce and sclk and not psclk:       # a rise
            if rise is None:
                need("tCC", cyc, cyc - ce_rise)
            else:
                need("fCLK", cyc, cyc - rise, value=1e3 / limit("fCLK", vcc))
            if fall is not None:
                need("tCL", cyc, cyc - fall)
            if cdd_from is not None and device_figures:
                bad.append(("tCDD", cyc, (cyc - cdd_from) * ns, limit("tCDD", vcc)))
                cdd_from = None                     # no data before the rise
            if ph and io_change is not None:
                need("tDC", cyc, cyc - io_change)
                hold_from = cyc
            bits.append(pio)
            rise = cyc
            if len(bits) == 8 and bits[7] and bits[0]:
                reading = True
        if ce and pce and psclk and not sclk:       # a fall
            if rise is not None:
                need("tCH", cyc, cyc - rise)
            fall = cyc
            if reading:
                cdd_from = cyc
        if d and cdd_from is not None and (not pd or io != pio):
            if device_figures:
                need("tCDD", cyc, cyc - cdd_from, maximum=True)
            cdd_from = None
        if pd and not d and device_figures:
            if not ce and ce_fall is not None:
                need("tCDZ", cyc, cyc - ce_fall, maximum=True)
            elif rise is not None and (fall is None or rise > fall):
                need("tCCZ", cyc, cyc - rise, maximum=True)
        if pce and not ce:                          # CE falls
            last = max(x for x in (rise, fall, ce_rise) if x is not None)
            if rise is not None:
                need("tCCH", cyc, cyc - last)
            ce_fall = cyc
            cdd_from = None
            whole = len(bits) // 8
            byts = [sum(b << i for i, b in enumerate(bits[8 * k:8 * k + 8]))
                    for k in range(whole)]
            if byts:
                transfers.append((byts[0], byts[1:]))
        prev = (cyc, ce, sclk, io, h, d)
    result: Dict[str, object] = dict(out)
    result["violations"] = bad
    result["transfers"] = transfers
    return result
