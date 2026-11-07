"""IEEE 488 (GPIB, HP-IB) reference models: a wired bus, a controller,
an extra listener and a handshake checker.

Written from public descriptions of the standard, never from the firmware.
IEEE 488.1 (the handshake state diagrams, timing T1, the interface
functions) is not free and no copy was open, so every figure is
secondary or unchecked:

* Lines and logic. Eight data lines DIO1..8, three handshake lines DAV, NRFD,
  NDAC, five management lines ATN, EOI, IFC, REN, SRQ; the slowest listener
  sets the pace (electronics-notes.com, "GPIB / IEEE 488 Protocol, Commands
  & Operation"; Wikipedia "IEEE-488", "Characteristics"). Active lines are
  "pulled low" (same page): negative logic, a line is asserted when low. The
  handshake lines are open collector, so a line is asserted if any device
  asserts it (Wikipedia, "Capabilities": E1 open collector; secondary).
* Handshake (Keithley 3321 manual, section 4, figure 4-2 text): the talker
  puts the byte on DIO, checks NRFD and if high sets DAV low; the listener on
  DAV low sets NRFD low and, when it has the byte, NDAC high; when every
  listener has it NDAC goes high (the wired OR); the talker sets DAV high;
  the listener sees DAV high and sets NDAC low. Initially the talker holds DAV
  high and the listeners hold NRFD high and NDAC low (search summary of the
  same family of Keithley manuals; secondary).
* Command bytes, sent with ATN asserted (NI, "GPIB messages", ni.com
  supplemental 06): MLA = 0x20 + PAD, MTA = 0x40 + PAD, MSA = 0x60 + SAD,
  UNL = 0x3F, UNT = 0x5F, SPE = 0x18, SPD = 0x19; bit 7 (DIO8) is a don't
  care; PAD 31 is reserved, the controller is usually 0. SDC = 0x04,
  GET = 0x08, DCL = 0x14: **from memory, unchecked**.
* Serial poll (Marvin Test, ATEasy user guide, "Polling"): address the
  device to talk, send SPE, read its poll byte, send SPD and UNT; "usually
  bit #6 in the serial poll byte is set to 1 when SRQ is asserted". That
  the device releases SRQ once its byte with bit 6 has been read is **from
  memory, unchecked** (488.1 SR function, APRS).
* Timing, **from memory, unchecked**: T1, the settling time from data valid
  to DAV asserted, is at least 2 us for open-collector drivers (the 350 ns
  tri-state figure turned up in a search summary only; this model keeps the
  slower, safe value); a device must answer ATN (engage its acceptor
  handshake, release DAV and EOI as talker) within 200 ns. The controller
  here waits T1 after asserting ATN before it asserts DAV, so a slower
  answer does no harm on this bus; the checker measures it anyway.

What is here:

* :class:`GpibBus`, the board: the chip's DIO on its eight open-drain uio
  pads, its handshake and SRQ drivers on uo pins through an external
  open-collector inverting driver (uo high = line asserted), its receivers
  on ui pins; any number of :class:`Party` objects (controller, listeners)
  on the same wires, and observers that see the resolved :class:`State`;
* :class:`Controller`, a scripted controller-in-charge: commands under ATN,
  writes as talker, reads as listener until EOI, waits for SRQ, serial polls;
* :class:`Listener`, an L4 listener at an address, with a settable delay
  (the slow second listener);
* :class:`HandshakeChecker`, the rules of the handshake on the wires and on
  the chip's own drivers, the bytes that moved, and the ATN response times.
"""

from __future__ import annotations

from typing import Callable, Dict, Iterator, List, Optional, Sequence, Tuple

from .bench import Drive, Lines, Model, PinSpec, pad_of

#: Handshake and management lines a party can assert, in a fixed order.
LINES = ("ATN", "DAV", "NRFD", "NDAC", "EOI", "IFC", "REN", "SRQ")

# Command bytes (module docstring: NI "GPIB messages"; SDC, GET, DCL unchecked)
UNL, UNT, SPE, SPD = 0x3F, 0x5F, 0x18, 0x19
SDC, GET, DCL = 0x04, 0x08, 0x14


def mla(pad: int) -> int:
    return 0x20 | (pad & 0x1F)


def mta(pad: int) -> int:
    return 0x40 | (pad & 0x1F)


#: T1, data valid to DAV, open collector: 2 us (from memory, unchecked).
T1_NS = 2000.0
#: A device's answer to ATN, 200 ns (from memory, unchecked).
T_ATN_NS = 200.0
#: The serial poll byte's request-service bit (bit 6, Marvin Test "Polling").
RQS = 0x40


def ns_to_clocks(ns: float, clock_hz: float) -> int:
    """Whole clocks covering ``ns`` at ``clock_hz`` (rounded up)."""
    c = ns * 1e-9 * clock_hz
    return int(c) if c == int(c) else int(c) + 1


class State:
    """The resolved bus during one clock: which lines are asserted, the
    data byte in positive logic (a 1 is a DIO line held low), and what the
    chip's own drivers did (``chip``: line name -> asserted)."""

    __slots__ = ("cycle", "lines", "dio", "chip", "chip_dio")

    def __init__(self) -> None:
        self.cycle = -1
        self.lines: Dict[str, bool] = {n: False for n in LINES}
        self.dio = 0
        self.chip: Dict[str, bool] = {n: False for n in LINES}
        self.chip_dio = 0

    def __getitem__(self, name: str) -> bool:
        return self.lines[name]


class Party:
    """Something on the bus besides the chip. ``asserts`` holds the lines it
    asserts and ``dio`` the data bits it asserts, both registered: set them
    in :meth:`clock`, they act from the next clock."""

    def __init__(self) -> None:
        self.asserts: set = set()
        self.dio = 0

    def clock(self, state: State) -> None:
        """See the resolved bus of one clock; update what is asserted."""


#: The chip's pins (``firmware/gpib_device.loom``): DIO1..8 on BIDIR0..7,
#: receivers and drivers.
DIO_PINS = tuple("BIDIR%d" % i for i in range(8))
IN_PINS = {"ATN": "IN0", "DAV": "IN1", "NRFD": "IN2", "NDAC": "IN3", "EOI": "IN4"}
OUT_PINS = {"DAV": "OUT0", "NRFD": "OUT1", "NDAC": "OUT2", "EOI": "OUT3",
            "SRQ": "OUT4"}


class GpibBus(Model):
    """The board: one wired bus joining the chip and every :class:`Party`.

    A handshake or management line is asserted (low) if any party asserts it
    or the chip's driver pin for it is high (an open-collector inverter on the
    board). The chip's receivers see the line one clock later (the board
    wire is a register here, as every bench model must be). DIO is the
    chip's eight uio pads themselves, open drain: a party pulls a pad low,
    the bench resolves it with the chip's own drive and the pull-ups (the
    bench needs ``pullups=0xFF``: the bus terminations)."""

    def __init__(self, dio: Sequence[PinSpec] = DIO_PINS,
                 ins: Optional[Dict[str, PinSpec]] = None,
                 outs: Optional[Dict[str, PinSpec]] = None) -> None:
        self.dio_pads = [pad_of(p) for p in dio]
        self.in_pads = {n: pad_of(p) for n, p in (ins or IN_PINS).items()}
        self.out_pads = {n: pad_of(p) for n, p in (outs or OUT_PINS).items()}
        self.parties: List[Party] = []
        self.observers: List[Callable[[State], None]] = []
        self.chip_asserts: Dict[str, bool] = {n: False for n in LINES}
        self.state = State()

    def add(self, party: Party) -> Party:
        self.parties.append(party)
        return party

    def asserted_by_parties(self, name: str) -> bool:
        return any(name in p.asserts for p in self.parties)

    def drive(self, drive: Drive, cycle: int) -> None:
        for name, pad in self.in_pads.items():
            on = self.chip_asserts[name] or self.asserted_by_parties(name)
            drive.set(pad, 0 if on else 1)
        dio = 0
        for p in self.parties:
            dio |= p.dio
        for i, pad in enumerate(self.dio_pads):
            if (dio >> i) & 1:
                drive.pull_low(pad)

    def observe(self, lines: Lines) -> None:
        s = self.state
        s.cycle = lines.cycle
        for name in LINES:
            pad = self.out_pads.get(name)
            s.chip[name] = bool(pad is not None and lines.get(pad))
        self.chip_asserts = dict(s.chip)
        for name in LINES:
            s.lines[name] = s.chip[name] or self.asserted_by_parties(name)
        dio = chip = 0
        for i, (port, bit) in enumerate(self.dio_pads):
            if not lines.get((port, bit)):
                dio |= 1 << i
            if (lines.uio_oe >> bit) & 1 and not (lines.uio_out >> bit) & 1:
                chip |= 1 << i
        s.dio, s.chip_dio = dio, chip
        for fn in self.observers:
            fn(s)
        for p in self.parties:
            p.clock(s)


class Acceptor(Party):
    """The acceptor and source handshakes as generators, one step a clock;
    :class:`Controller` and :class:`Listener` are built on them."""

    def __init__(self, delay: int = 1, recover: int = 1) -> None:
        super().__init__()
        self.delay, self.recover = delay, recover
        self.s = State()
        self.errors: List[Tuple[int, str]] = []

    def _set(self, name: str, on: bool) -> None:
        (self.asserts.add if on else self.asserts.discard)(name)

    def _until(self, pred: Callable[[], bool]) -> Iterator[None]:
        while not pred():
            yield

    def _wait(self, n: int) -> Iterator[None]:
        for _ in range(n):
            yield

    def _ready(self) -> None:
        """Acceptor ready (ACRS): NDAC asserted, NRFD released."""
        self._set("NDAC", True)
        self._set("NRFD", False)

    def _idle(self) -> None:
        self._set("NDAC", False)
        self._set("NRFD", False)

    def _accept(self, got: List[Tuple[int, bool, bool]],
                more: Callable[[], bool] = lambda: True) -> Iterator[None]:
        """From ready: take one byte as ``(byte, eoi, atn)``; end ready if
        ``more()`` (asked once the byte is in ``got``), else not ready
        (NRFD held asserted, so no talker may start another byte)."""
        yield from self._until(lambda: self.s["DAV"])
        yield from self._wait(self.delay)
        got.append((self.s.dio, self.s["EOI"], self.s["ATN"]))
        self._set("NRFD", True)                 # not ready
        yield
        self._set("NDAC", False)                # accepted
        yield from self._until(lambda: not self.s["DAV"])
        self._set("NDAC", True)                 # not accepted
        yield from self._wait(self.recover)
        if more():
            self._set("NRFD", False)            # ready again
        yield

    def _source(self, byte: int, eoi: bool, t1: int) -> Iterator[None]:
        """One byte as source: DIO and EOI, T1, NRFD released, DAV, NDAC
        released, DAV released."""
        self.dio = byte & 0xFF
        self._set("EOI", eoi)
        yield from self._wait(t1)
        yield from self._until(lambda: not self.s["NRFD"])
        if not self.s["NDAC"]:
            self.errors.append((self.s.cycle, "no acceptor: NRFD and NDAC both high"))
        self._set("DAV", True)
        yield
        yield from self._until(lambda: not self.s["NDAC"])
        self._set("DAV", False)
        self._set("EOI", False)
        self.dio = 0
        yield


class Controller(Acceptor):
    """The controller-in-charge at primary address ``pad`` (0 by default).

    Actions queue up and run one after the other: :meth:`command`,
    :meth:`write`, :meth:`read`, :meth:`wait_srq`, :meth:`serial_poll`,
    :meth:`release`. T1 is ``t1`` clocks (default 2 us at ``clock_hz``,
    module docstring). Results: ``reads`` (one list of ``(byte, eoi)`` per
    :meth:`read`), ``polls`` (one byte per :meth:`serial_poll`), ``errors``."""

    def __init__(self, pad: int = 0, *, clock_hz: float = 50e6,
                 t1: Optional[int] = None, delay: int = 1, recover: int = 1) -> None:
        super().__init__(delay, recover)
        self.pad = pad
        self.t1 = t1 if t1 is not None else ns_to_clocks(T1_NS, clock_hz)
        self.queue: List[Callable[[], Iterator[None]]] = []
        self.gen: Optional[Iterator[None]] = None
        self.reads: List[List[Tuple[int, bool]]] = []
        self.polls: List[int] = []
        self.srq_seen: List[int] = []

    @property
    def busy(self) -> bool:
        return self.gen is not None or bool(self.queue)

    def clock(self, state: State) -> None:
        self.s = state
        if self.gen is None and self.queue:
            self.gen = self.queue.pop(0)()
        if self.gen is not None:
            try:
                next(self.gen)
            except StopIteration:
                self.gen = None

    # ------------------------------------------------------------ actions
    def command(self, cmds: Sequence[int]) -> None:
        """ATN asserted, then each command byte with the source handshake.

        If the controller was a listener (NRFD held asserted after a read),
        it keeps NRFD and NDAC as they are for T1 after asserting ATN and
        only then lets go of them. A talker waiting for NRFD therefore sees
        ATN before it can see NRFD released; releasing both in the same
        clock let the device assert DAV over the first command byte, which
        the first version of this model did (QUESTIONS.md, Q4)."""
        def run():
            was_listening = "NRFD" in self.asserts
            self._set("ATN", True)
            yield
            if was_listening:
                yield from self._wait(self.t1)
            self._idle()                        # not an acceptor of its own bytes
            yield
            for c in cmds:
                yield from self._source(c, False, self.t1)
        self.queue.append(run)

    def write(self, data: Sequence[int], eoi: bool = True) -> None:
        """ATN released, the bytes as talker, EOI with the last if ``eoi``."""
        def run():
            self._idle()
            self._set("ATN", False)
            yield
            for i, b in enumerate(data):
                yield from self._source(b, eoi and i == len(data) - 1, self.t1)
        self.queue.append(run)

    def read(self, limit: int = 256, until_eoi: bool = True) -> None:
        """ATN released, accept bytes until EOI (or ``limit`` bytes), then
        hold NRFD asserted until the next action."""
        def run():
            got: List[Tuple[int, bool, bool]] = []
            self._set("NDAC", True)
            self._set("NRFD", True)
            yield
            self._set("ATN", False)
            yield
            self._set("NRFD", False)
            def more():
                return len(got) < limit and not (until_eoi and got[-1][1])
            while True:
                yield from self._accept(got, more)
                if not more():
                    break                       # NRFD stays asserted
            self.reads.append([(b, e) for b, e, _ in got])
        self.queue.append(run)

    def wait_srq(self) -> None:
        def run():
            yield from self._until(lambda: self.s["SRQ"])
            self.srq_seen.append(self.s.cycle)
        self.queue.append(run)

    def serial_poll(self, pad: int) -> None:
        """UNL, the controller's MLA, SPE, the device's MTA; one byte; SPD,
        UNT (the order of an NI-488.2 ibrsp, module docstring)."""
        self.command([UNL, mla(self.pad), SPE, mta(pad)])
        self.read(1, until_eoi=False)

        def keep():
            self.polls.append(self.reads.pop()[0][0])
            return iter(())
        self.queue.append(keep)
        self.command([SPD, UNT])

    def pause(self, clocks: int) -> None:
        """Hold every line as it is for ``clocks`` clocks."""
        def run():
            yield from self._wait(clocks)
        self.queue.append(run)

    def release(self) -> None:
        """Let go of every line (ATN released)."""
        def run():
            self._idle()
            self._set("ATN", False)
            yield
        self.queue.append(run)


class Listener(Acceptor):
    """A basic listener (L4: unaddressed by its MTA) at primary address
    ``pad``, ``delay`` clocks from DAV to taking the byte and ``recover``
    clocks from DAV released to ready again: the slow second listener.
    Its acceptor handshake runs whenever ATN is asserted or it is addressed
    to listen; data bytes go to ``received`` as ``(byte, eoi)``."""

    def __init__(self, pad: int, delay: int = 1, recover: int = 1) -> None:
        super().__init__(delay, recover)
        self.pad = pad
        self.lads = False
        self.received: List[Tuple[int, bool]] = []
        self.commands: List[int] = []
        self.gen = self._run()

    def clock(self, state: State) -> None:
        self.s = state
        next(self.gen)

    def _run(self) -> Iterator[None]:
        while True:
            if not (self.s["ATN"] or self.lads):
                self._idle()
                yield
                continue
            self._ready()
            if not self.s["DAV"]:
                yield
                continue
            yield from self._wait(self.delay)
            byte, eoi, atn = self.s.dio, self.s["EOI"], self.s["ATN"]
            self._set("NRFD", True)
            yield
            self._set("NDAC", False)
            yield from self._until(lambda: not self.s["DAV"])
            self._set("NDAC", True)
            yield from self._wait(self.recover)
            if atn:
                self._command(byte & 0x7F)
            elif self.lads:
                self.received.append((byte, eoi))

    def _command(self, c: int) -> None:
        self.commands.append(c)
        if c == mla(self.pad):
            self.lads = True
        elif c == UNL or c == mta(self.pad):
            self.lads = False


class Transfer:
    """One byte that moved: who sourced it, the byte, EOI and ATN with it,
    the cycle DAV was asserted, T1 before it and the cycle NDAC rose."""

    __slots__ = ("by_chip", "byte", "eoi", "atn", "dav", "t1", "done")

    def __init__(self, by_chip, byte, eoi, atn, dav, t1) -> None:
        self.by_chip, self.byte, self.eoi, self.atn = by_chip, byte, eoi, atn
        self.dav, self.t1, self.done = dav, t1, None

    def __repr__(self) -> str:
        return "Transfer(%s 0x%02X eoi=%d atn=%d dav=%d t1=%d)" % (
            "chip" if self.by_chip else "bus", self.byte, self.eoi, self.atn,
            self.dav, self.t1)


class HandshakeChecker:
    """Checks the three-wire handshake clock by clock (attach with
    ``bus.observers.append(checker)``). On the wires: no DAV while NRFD is
    asserted; DIO and EOI hold while DAV is asserted; a byte that the chip
    sources is not withdrawn before NDAC rises unless ATN took the bus, and
    the chip never asserts DAV while ATN is asserted. On the
    chip's own drivers, as acceptor: NDAC is released only while DAV is
    asserted (or with NRFD released too: going idle), and NRFD is released
    only while DAV is released. Records ``transfers``, the ATN answer times
    (``atn_ndac``: ATN asserted to the chip's NDAC asserted; ``atn_release``:
    to the chip's DAV, EOI and DIO released) and ``violations``."""

    def __init__(self) -> None:
        self.prev: Optional[Tuple[Dict[str, bool], Dict[str, bool], int]] = None
        self.last_change = 0
        self.transfers: List[Transfer] = []
        self.violations: List[Tuple[int, str]] = []
        self.atn_ndac: List[int] = []
        self.atn_release: List[int] = []
        self._atn_at: Optional[int] = None
        self._want_ndac = self._want_release = False

    def _bad(self, c: int, text: str) -> None:
        self.violations.append((c, text))

    def __call__(self, s: State) -> None:
        c, L, chip = s.cycle, s.lines, s.chip
        if self.prev is None:
            self.prev = (dict(L), dict(chip), s.dio)
            return
        pl, pc, pdio = self.prev
        if s.dio != pdio or L["EOI"] != pl["EOI"]:
            if pl["DAV"] and L["DAV"]:
                self._bad(c, "DIO or EOI changed while DAV asserted")
            self.last_change = c
        if L["DAV"] and not pl["DAV"]:
            if L["NRFD"]:
                self._bad(c, "DAV asserted while NRFD asserted")
            self.transfers.append(Transfer(chip["DAV"], s.dio, L["EOI"], L["ATN"],
                                           c, c - self.last_change))
        if pl["NDAC"] and not L["NDAC"] and L["DAV"] and self.transfers:
            self.transfers[-1].done = c
        if chip["DAV"] and not pc["DAV"] and L["ATN"]:
            self._bad(c, "chip asserted DAV under ATN (its byte would be a command)")
        if pc["DAV"] and not chip["DAV"] and L["NDAC"] and not L["ATN"]:
            self._bad(c, "chip withdrew DAV before NDAC rose, with no ATN")
        if pc["NDAC"] and not chip["NDAC"] and chip["NRFD"] and not pl["DAV"]:
            self._bad(c, "chip released NDAC (accepted) with no DAV")
        if pc["NRFD"] and not chip["NRFD"] and pl["DAV"]:
            self._bad(c, "chip ready (NRFD released) while DAV still asserted")
        self._atn(c, L, pl, chip, s.chip_dio)
        self.prev = (dict(L), dict(chip), s.dio)

    def _atn(self, c, L, pl, chip, chip_dio) -> None:
        if L["ATN"] and not pl["ATN"]:
            self._atn_at = c
            self._want_ndac = not (chip["NDAC"] or chip["NRFD"])   # from idle
            self._want_release = chip["DAV"] or chip["EOI"] or bool(chip_dio)
        if self._atn_at is None:
            return
        if not L["ATN"]:
            self._atn_at = None
            return
        if self._want_ndac and chip["NDAC"]:
            self.atn_ndac.append(c - self._atn_at)
            self._want_ndac = False
        if self._want_release and not (chip["DAV"] or chip["EOI"] or chip_dio):
            self.atn_release.append(c - self._atn_at)
            self._want_release = False
