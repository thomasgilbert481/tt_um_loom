"""The TI TLC5940 16-channel LED driver at its logic pins.

Written from the data sheet, not from any firmware
(``firmware/tlc5940_host.loom`` is checked against it).

Source: TI, "TLC5940 16-Channel LED Driver With DOT Correction and
Grayscale PWM Control", SLVS515D (December 2004, revised November 2015).

* 8.3.1 Serial Interface: "The rising edge of SCLK signal shifts the data
  from the SIN pin to the internal register"; "All data are clocked in
  with the MSB first"; 96 or 192 bits by mode; "the XLAT signal should
  only latch the grayscale data at the end of the grayscale cycle".
* 8.4.1 Table 3: VPRG = GND, 192-bit grayscale mode; VPRG = VCC, 96-bit
  dot correction input mode; DCPRG L takes the DC values from the EEPROM,
  H from the DC register. The shift register, DC and GS registers are
  unknown after power-up and "should be properly stored through the serial
  interface before starting the operation".
* 8.4.2: 16 x 6 bits, "Big-Endian", DC 15.5 (OUT15's MSB) first; "the
  TLC5940 writes the data in the input shift register to DC register when
  XLAT is high" (a level latch); "SCLK and SIN must not be changed while
  XLAT is high"; "BLANK signal does not need to be high to latch".
* 8.4.3: 16 x 12 bits, OUT15's MSB first (Figure 18: bit 191 is GS15.11);
  "a rising edge of the XLAT signal latches the data into the grayscale
  register"; "new grayscale data should be latched at the end of a
  grayscale cycle when BLANK is high"; "The first GS data input cycle
  after dot correction requires an additional SCLK pulse after the XLAT
  signal to complete the grayscale update cycle." After a GS latch the
  shift register holds the status information data (8.4.4).
* 8.3.6 and 8.4.5: BLANK high turns every output off and resets the
  grayscale counter. "The grayscale PWM cycle starts with the falling edge
  of BLANK. The first GSCLK pulse after BLANK goes low increases the
  grayscale counter by one and switches on all OUTn with grayscale value
  not zero"; outputs whose value equals the counter are switched off; "A
  BLANK=H signal after 4096 GSCLK pulses ... completes the grayscale PWM
  cycle"; at FFFh the counter stops and all outputs turn off.
* 6.3 Recommended Operating Conditions (VCC 3 to 5.5 V, -40 to 85 C), the
  AC timing in :data:`TIMING`.

Readings (QUESTIONS.md): the off-by-one between "switched on by the first
pulse" and "switched off when the counter equals the value" is taken the
way Equation 8 (brightness = GSn / 4095) and Equation 9 (f(GSCLK) = 4096 x
f(update)) imply: an output with value GS is on from GSCLK rising edge 1
to edge GS + 1, GS whole GSCLK periods, and all are off from edge 4096
on. "The additional SCLK pulse" is taken as: the GS register takes the
latched data only at the first SCLK rising edge after that XLAT, so a PWM
cycle that starts before it runs on the old (after power-up, undefined)
values. Not modelled: SOUT and the status data, XERR, LOD, TEF, the EEPROM
(its DC values are unknown here), the 20 ns output delays of 8.3.5 and
BLANK pulses under 300 ns (8.3.6), the cascade.

The model samples its lines once a clock as the bench resolves them, so
every interval is a whole number of clocks; ``clock`` turns them into ns.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple

from .bench import Lines, Model, PinSpec, pad_of

#: SLVS515D 6.3, AC characteristics, minimums in ns; frequencies in Hz.
TIMING = {
    "twh0": 16.0,           # SCLK high
    "twl0": 16.0,           # SCLK low
    "twh1": 16.0,           # GSCLK high
    "twl1": 16.0,           # GSCLK low
    "twh2": 20.0,           # XLAT high
    "twh3": 20.0,           # BLANK high
    "tsu0": 5.0,            # SIN to SCLK rising
    "tsu1": 10.0,           # SCLK falling to XLAT rising
    "tsu2": 10.0,           # VPRG change to SCLK rising
    "tsu3": 10.0,           # VPRG change to XLAT rising
    "tsu4": 10.0,           # BLANK falling to GSCLK rising
    "tsu5": 30.0,           # XLAT rising to GSCLK rising
    "th0": 3.0,             # SCLK rising to SIN change
    "th1": 10.0,            # XLAT falling to SCLK rising
    "th2": 10.0,            # SCLK rising to VPRG change
    "th3": 10.0,            # XLAT falling to VPRG change
    "th4": 10.0,            # GSCLK rising to BLANK rising
    "f_sclk": 30e6,         # f(SCLK), maximum
    "f_gsclk": 30e6,        # f(GSCLK), maximum
}

CHANNELS = 16
GS_BITS, DC_BITS = 12, 6
GS_STEPS = 4096
SIGNALS = ("SIN", "SCLK", "XLAT", "BLANK", "GSCLK", "VPRG", "DCPRG")
#: Default wiring (firmware/tlc5940_host.loom's pins).
PINS = {"SIN": "OUT0", "SCLK": "OUT1", "XLAT": "OUT2", "VPRG": "OUT3",
        "DCPRG": "OUT4", "GSCLK": "OUT5", "BLANK": "BIDIR0"}


def unpack(sr: int, bits: int) -> Tuple[int, ...]:
    """Channel values ``(OUT0 .. OUT15)`` from a shift register of
    ``16 * bits`` bits, OUT15's MSB at the top (8.4.2, 8.4.3)."""
    mask = (1 << bits) - 1
    return tuple((sr >> (bits * n)) & mask for n in range(CHANNELS))


class Cycle:
    """One grayscale PWM cycle, from BLANK falling to BLANK rising.

    ``edges``: GSCLK rising edges counted; ``gs`` and ``dc``: the register
    values it ran on (``None`` if undefined); ``on_edges[n]``: GSCLK periods
    output n was on; ``on_clocks[n]``: bench clocks it was on."""

    def __init__(self, start: int, gs, dc, dcprg: int) -> None:
        self.start, self.end = start, None
        self.gs, self.dc, self.dcprg = gs, dc, dcprg
        self.edges = 0
        self.on_edges = [0] * CHANNELS
        self.on_clocks = [0] * CHANNELS
        self.first_edge: Optional[int] = None

    def __repr__(self) -> str:
        return "Cycle(%d..%s, %d edges)" % (self.start, self.end, self.edges)


class Tlc5940(Model):
    """The chip: the input shift register, the DC and GS registers, the
    grayscale counter and the 16 outputs, with the 6.3 timing checks.

    Args:
        clock: the bench clock in Hz, to turn clocks into ns.
        pins: overrides of :data:`PINS`, signal name to pin.
        timing: overrides of :data:`TIMING`.

    After a run: ``cycles`` (closed :class:`Cycle` s), ``dc_latches`` and
    ``gs_latches`` (``(cycle, values)``, OUT0 first), ``xlats`` (``(cycle,
    VPRG, BLANK)`` at every XLAT rising edge), ``violations`` (``(cycle,
    what, ns)``), ``measured`` (every interval checked, in ns).
    """

    def __init__(self, *, clock: float = 50e6,
                 pins: Optional[Dict[str, PinSpec]] = None,
                 timing: Optional[Dict[str, float]] = None) -> None:
        self.clock = float(clock)
        self.timing = dict(TIMING, **(timing or {}))
        names = dict(PINS, **(pins or {}))
        self.pads = {n: pad_of(names[n]) for n in SIGNALS}
        self.sr = 0                             # 192 bits (unknown: 0 here)
        self.dc: Optional[Tuple[int, ...]] = None
        self.gs: Optional[Tuple[int, ...]] = None
        self.pending: Optional[Tuple[int, ...]] = None
        self.after_dc = False       # a DC latch since the last GS latch
        self.count = 0
        self.out = [0] * CHANNELS
        self.cycle: Optional[Cycle] = None
        self.cycles: List[Cycle] = []
        self.dc_latches: List[Tuple[int, Tuple[int, ...]]] = []
        self.gs_latches: List[Tuple[int, Tuple[int, ...]]] = []
        self.xlats: List[Tuple[int, int, int]] = []
        self.violations: List[Tuple[int, str, float]] = []
        self.measured: Dict[str, List[float]] = {k: [] for k in self.timing}
        self.sclk_edges = 0
        self.prev: Optional[Dict[str, int]] = None
        self.t: Dict[str, Optional[int]] = dict.fromkeys(
            ("sin", "vprg", "sclk_r", "sclk_f", "xlat_r", "xlat_f",
             "blank_r", "blank_f", "gsclk_r", "gsclk_f"))

    def ns(self, clocks: float) -> float:
        return clocks * 1e9 / self.clock

    def _check(self, name: str, since: Optional[int], c: int) -> None:
        if since is None:
            return
        value = self.ns(c - since)
        self.measured[name].append(value)
        if value < self.timing[name]:
            self.violations.append((c, name, value))

    def _after(self, a: str, b: str) -> bool:
        """Event ``a`` happened, and after the last event ``b`` (or no b)."""
        ta, tb = self.t[a], self.t[b]
        return ta is not None and (tb is None or ta > tb)

    def _period(self, name: str, last: Optional[int], c: int) -> None:
        if last is not None:
            period = self.ns(c - last)
            self.measured.setdefault("t_" + name[2:], []).append(period)
            if period < 1e9 / self.timing[name]:
                self.violations.append((c, name, period))

    # ------------------------------------------------------------ the pins
    def observe(self, lines: Lines) -> None:
        c = lines.cycle
        v = {n: lines.get(p) for n, p in self.pads.items()}
        p, self.prev = self.prev, v
        if p is None:
            return
        t = self.t
        dc_mode = v["VPRG"] == 1
        if v["SIN"] != p["SIN"]:
            self._check("th0", t["sclk_r"], c)
            if v["XLAT"] and dc_mode:
                self.violations.append((c, "SIN changed while XLAT high", 0.0))
            t["sin"] = c
        if v["VPRG"] != p["VPRG"]:
            self._check("th2", t["sclk_r"], c)
            self._check("th3", t["xlat_f"], c)
            t["vprg"] = c
        if v["SCLK"] and not p["SCLK"]:
            self._sclk_rise(c, v)
        elif p["SCLK"] and not v["SCLK"]:
            self._check("twh0", t["sclk_r"], c)
            t["sclk_f"] = c
        if v["XLAT"] and not p["XLAT"]:
            self._xlat_rise(c, v)
        elif p["XLAT"] and not v["XLAT"]:
            self._check("twh2", t["xlat_r"], c)
            t["xlat_f"] = c
            if p["VPRG"]:
                self.dc_latches.append((c, self.dc))
                self.after_dc = True
        if v["XLAT"] and dc_mode:               # the DC latch is transparent
            self.dc = unpack(self.sr & ((1 << 96) - 1), DC_BITS)
        if v["BLANK"] and not p["BLANK"]:
            self._check("th4", t["gsclk_r"], c)
            t["blank_r"] = c
            self._close(c)
        elif p["BLANK"] and not v["BLANK"]:
            self._check("twh3", t["blank_r"], c)
            t["blank_f"] = c
            self._open(c, v)
        if v["GSCLK"] and not p["GSCLK"]:
            self._gsclk_rise(c, v)
        elif p["GSCLK"] and not v["GSCLK"]:
            self._check("twh1", t["gsclk_r"], c)
            t["gsclk_f"] = c
        if self.cycle is not None:
            for n in range(CHANNELS):
                self.cycle.on_clocks[n] += self.out[n]

    def _sclk_rise(self, c: int, v: Dict[str, int]) -> None:
        t = self.t
        self._period("f_sclk", t["sclk_r"], c)
        self._check("twl0", t["sclk_f"], c)
        if self._after("sin", "sclk_r"):
            self._check("tsu0", t["sin"], c)
        if self._after("vprg", "sclk_r"):
            self._check("tsu2", t["vprg"], c)
        if self._after("xlat_f", "sclk_r"):
            self._check("th1", t["xlat_f"], c)
        if v["XLAT"] and v["VPRG"]:
            self.violations.append((c, "SCLK rose while XLAT high", 0.0))
        self.sr = ((self.sr << 1) | v["SIN"]) & ((1 << (16 * GS_BITS)) - 1)
        t["sclk_r"] = c
        self.sclk_edges += 1
        if self.pending is not None:            # 8.4.3: the update completes
            self.gs, self.pending = self.pending, None
            self.gs_latches.append((c, self.gs))

    def _xlat_rise(self, c: int, v: Dict[str, int]) -> None:
        t = self.t
        if self._after("sclk_f", "xlat_r"):
            self._check("tsu1", t["sclk_f"], c)
        if self._after("vprg", "xlat_r"):
            self._check("tsu3", t["vprg"], c)
        t["xlat_r"] = c
        self.xlats.append((c, v["VPRG"], v["BLANK"]))
        if v["VPRG"]:
            return                              # DC: the level latch above
        if not v["BLANK"]:
            self.violations.append((c, "GS latched while BLANK low", 0.0))
        values = unpack(self.sr, GS_BITS)
        if self.after_dc:
            self.pending = values               # waits for one more SCLK
        else:
            self.gs = values
            self.gs_latches.append((c, values))
        self.after_dc = False
        dc = 0 if self.dc is None else sum(d << (DC_BITS * n)
                                           for n, d in enumerate(self.dc))
        self.sr = dc << 24                      # status data (8.4.4), LOD/TEF 0

    def _open(self, c: int, v: Dict[str, int]) -> None:
        if self.pending is not None:
            self.violations.append(
                (c, "PWM cycle started before the first GS update completed", 0.0))
        if self.gs is None:
            self.violations.append((c, "PWM cycle started with GS undefined", 0.0))
        dc = self.dc if v["DCPRG"] else None    # DCPRG low: the EEPROM's
        self.count = 0
        self.cycle = Cycle(c, self.gs, dc, v["DCPRG"])

    def _close(self, c: int) -> None:
        self.out = [0] * CHANNELS
        self.count = 0
        cycle, self.cycle = self.cycle, None
        if cycle is not None:
            cycle.end = c
            self.cycles.append(cycle)

    def _gsclk_rise(self, c: int, v: Dict[str, int]) -> None:
        t = self.t
        self._period("f_gsclk", t["gsclk_r"], c)
        self._check("twl1", t["gsclk_f"], c)
        if self._after("blank_f", "gsclk_r"):
            self._check("tsu4", t["blank_f"], c)
        if self._after("xlat_r", "gsclk_r"):
            self._check("tsu5", t["xlat_r"], c)
        t["gsclk_r"] = c
        cycle = self.cycle
        if v["BLANK"] or cycle is None:
            return                              # held in reset by BLANK
        cycle.edges += 1
        if cycle.first_edge is None:
            cycle.first_edge = c
        if self.count < GS_STEPS:
            self.count += 1
        gs = self.gs or (0,) * CHANNELS         # a GS latch acts at once
        for n in range(CHANNELS):
            # On from edge 1 to edge GS + 1; all off from edge 4096.
            self.out[n] = int(self.count <= gs[n] and self.count < GS_STEPS)
            cycle.on_edges[n] += self.out[n]

    # ------------------------------------------------------- what is seen
    def complete(self) -> List[Cycle]:
        """The closed cycles that had all 4096 GSCLK pulses."""
        return [cy for cy in self.cycles if cy.edges >= GS_STEPS]


def pwm_hz(cycles: List[Cycle], clock: float = 50e6) -> float:
    """PWM frequency from the BLANK-falling to BLANK-falling spacing of
    consecutive ``cycles`` (their mean)."""
    starts = [cy.start for cy in cycles]
    if len(starts) < 2:
        raise ValueError("need two cycles")
    return clock * (len(starts) - 1) / (starts[-1] - starts[0])
