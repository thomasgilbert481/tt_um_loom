"""Clock-by-clock bench: one golden-model Machine plus any number of pin models.

The bench is the only thing that steps the machine. Every cycle it

1. asks each model what it drives (``Model.drive``), from the state the model
   registered at the previous edge;
2. resolves every pad: ``ui_in`` bits driven by models (the rest idle, with the
   host chip select ``ui_in[4]`` high), and each ``uio`` bit as a wired line:
   low if anyone drives it low (the chip with ``uio_oe`` set and ``uio_out``
   0, a push-pull model driving 0, or an open-drain model pulling low), else
   high if anyone drives it high, else the pull-up value. The chip's own
   drive is therefore looped back as the Tiny Tapeout pad does
   (SEMANTICS section 3), and I2C gets its open-drain wired-AND for free;
3. hands the resolved pads to the machine (sampled at the edge that ends the
   cycle, SEMANTICS 3) and steps it one cycle;
4. lets each model observe the resolved lines of that cycle
   (``Model.observe``) and calls the per-cycle observers with the retire
   record.

So every model is a registered circuit clocked by ``clk``: what it sees in
cycle ``c`` can change what it drives from cycle ``c + 1``. A line driven both
high and low in the same cycle is recorded in :attr:`Bench.contentions`.

Pins are named as in ``isa/isa.yaml`` (``IN0``, ``OUT3``, ``BIDIR5``) or given
as pin indices, and map to pads through the pin index space of ARCHITECTURE
3.1; :func:`pad_of` also accepts raw pads ``("ui", 4)``, which is how the host
pins (not reachable by firmware) are named.
"""

from __future__ import annotations

from typing import Callable, Iterable, List, Optional, Tuple, Union

from tools.loomisa import Isa
from tools.loomisa import load as load_isa
from tools.loomsim import Machine

UI, UIO, UO = "ui", "uio", "uo"
Pad = Tuple[str, int]
PinSpec = Union[str, int, Pad]

CS_N_BIT = 1 << 4           # ui_in[4], the host chip select, idle high

_ISA: Optional[Isa] = None


def _isa() -> Isa:
    global _ISA
    if _ISA is None:
        _ISA = load_isa()
    return _ISA


def pad_of(pin: PinSpec, isa: Optional[Isa] = None) -> Pad:
    """Map a pin name, a pin index or a raw pad to ``(port, bit)``."""
    if isinstance(pin, tuple):
        port, bit = pin
        if port not in (UI, UIO, UO) or not 0 <= int(bit) <= 7:
            raise ValueError("bad pad %r" % (pin,))
        return port, int(bit)
    if isinstance(pin, str):
        table = (isa or _isa()).pin_by_name
        upper = {name.upper(): index for name, index in table.items()}
        if pin.upper() not in upper:
            raise ValueError("unknown pin name %r" % (pin,))
        index = upper[pin.upper()]
    else:
        index = int(pin)
    if 0 <= index <= 7:
        return UIO, index
    if 8 <= index <= 11:
        return UI, index - 8
    if index == 12:
        return UI, 7
    if 16 <= index <= 21:
        return UO, index - 16
    raise ValueError("pin index %d is not a pad" % index)


class Drive:
    """What the models drive during one cycle. Reset by the bench each cycle."""

    __slots__ = ("ui_mask", "ui_val", "uio_mask", "uio_val", "uio_low")

    def __init__(self) -> None:
        self.clear()

    def clear(self) -> None:
        self.ui_mask = self.ui_val = 0
        self.uio_mask = self.uio_val = self.uio_low = 0

    def set(self, pad: Pad, value: int) -> None:
        """Push-pull drive of a ``ui`` or ``uio`` pad."""
        port, bit = pad
        m = 1 << bit
        if port == UI:
            self.ui_mask |= m
            self.ui_val = (self.ui_val | m) if value else (self.ui_val & ~m)
        elif port == UIO:
            self.uio_mask |= m
            self.uio_val = (self.uio_val | m) if value else (self.uio_val & ~m)
        else:
            raise ValueError("uo pads are chip outputs; a model cannot drive them")

    def pull_low(self, pad: Pad) -> None:
        """Open-drain pull-down of a ``uio`` pad (release = do nothing)."""
        port, bit = pad
        if port != UIO:
            raise ValueError("open-drain drive needs a uio pad")
        self.uio_low |= 1 << bit


class Lines:
    """Resolved pad values during one cycle. One object, updated every cycle."""

    __slots__ = ("cycle", "ui", "uio", "uo", "uio_out", "uio_oe")

    def __init__(self) -> None:
        self.cycle = -1
        self.ui = self.uio = self.uo = self.uio_out = self.uio_oe = 0

    def get(self, pad: Pad) -> int:
        port, bit = pad
        if port == UI:
            return (self.ui >> bit) & 1
        if port == UIO:
            return (self.uio >> bit) & 1
        return (self.uo >> bit) & 1


class Model:
    """Base class of a pin-level model. Both hooks are optional."""

    def drive(self, drive: Drive, cycle: int) -> None:
        """Contribute this cycle's drive. Must depend only on registered state."""

    def observe(self, lines: Lines) -> None:
        """See the resolved lines of the cycle that just ran; update state."""


Observer = Callable[[object], None]


class Bench:
    """Steps a :class:`tools.loomsim.Machine` and its pin models together.

    Args:
        machine: an existing machine, or ``None`` to build one from ``image``
            and ``features`` (default ``{"FIFO"}``) and the other keyword
            arguments of :class:`~tools.loomsim.Machine`.
        pullups: ``uio`` bits with an external pull-up (I2C lines, say).
        ui_idle: ``ui_in`` value where no model drives (host CS_n high).
    """

    def __init__(self, machine: Optional[Machine] = None, *, image=None,
                 features: Iterable[str] = ("FIFO",), pullups: int = 0,
                 ui_idle: int = CS_N_BIT, isa: Optional[Isa] = None,
                 **machine_kwargs) -> None:
        self.isa = isa or (machine.isa if machine is not None else _isa())
        if machine is None:
            machine = Machine(image, features=features, isa=self.isa, **machine_kwargs)
        self.machine = machine
        self.pullups = pullups & 0xFF
        self.ui_idle = ui_idle & 0xFF
        self.models: List[Model] = []
        self.observers: List[Observer] = []
        self.contentions: List[Tuple[int, int]] = []
        self.lines = Lines()
        self._drive = Drive()

    # --------------------------------------------------------------- setup
    def add(self, model: Model) -> Model:
        self.models.append(model)
        return model

    def remove(self, model: Model) -> None:
        self.models.remove(model)

    def add_observer(self, fn: Observer) -> None:
        """``fn(record)`` after every cycle; ``record`` may be ``None``."""
        self.observers.append(fn)

    def pad(self, pin: PinSpec) -> Pad:
        return pad_of(pin, self.isa)

    @property
    def cycle(self) -> int:
        return self.machine.cycle

    # ---------------------------------------------------------------- clock
    def step(self, cycles: int = 1) -> None:
        """Advance ``cycles`` clocks."""
        m = self.machine
        d = self._drive
        lines = self.lines
        models = self.models
        observers = self.observers
        pullups = self.pullups
        for _ in range(cycles):
            cycle = m.cycle
            d.clear()
            for model in models:
                model.drive(d, cycle)
            pin_out = m.pin_out
            uo = (pin_out >> 8) & 0x3F
            uio_out = pin_out & 0xFF
            oe = m.pin_oe & 0xFF
            ui = (self.ui_idle & ~d.ui_mask) | (d.ui_val & d.ui_mask)
            low = d.uio_low | (d.uio_mask & ~d.uio_val) | (oe & ~uio_out)
            high = (d.uio_mask & d.uio_val) | (oe & uio_out)
            uio = (high | (pullups & ~(high | low))) & ~low & 0xFF
            if low & high:
                self.contentions.append((cycle, low & high & 0xFF))
            m.set_pad_inputs(ui_in=ui, uio_in=uio)
            record = m.step_cycle()
            lines.cycle = cycle
            lines.ui = ui
            lines.uio = uio
            lines.uo = uo
            lines.uio_out = uio_out
            lines.uio_oe = oe
            for model in models:
                model.observe(lines)
            for fn in observers:
                fn(record)

    def run_until(self, predicate: Callable[[], bool], max_cycles: int,
                  every: int = 1) -> bool:
        """Step until ``predicate()`` holds, testing it every ``every`` cycles.

        Returns False if ``max_cycles`` pass first.
        """
        spent = 0
        while spent < max_cycles:
            if predicate():
                return True
            chunk = min(every, max_cycles - spent)
            self.step(chunk)
            spent += chunk
        return predicate()

    def contention_free(self) -> bool:
        return not self.contentions
