# SPDX-License-Identifier: Apache-2.0
"""RtlBench: the RTL twin of ``tools.protomodels.bench.Bench``.

``Bench`` steps the golden model and a set of pin models together, one clock
at a time; the L3 firmware tests (``tools/tests/test_fw_*.py``) are written
against it and against ``tools.loomhost.Loom``. ``RtlBench`` offers the same
API (``add``, ``remove``, ``add_observer``, ``pad``, ``cycle``, ``step``,
``run_until``, ``lines``, ``contentions``, ``contention_free``) over
``tb.v``, so the *unchanged* test bodies and the *unchanged*
``tools.protomodels`` models run on the RTL, with the program loaded and the
data moved through the real SPI host pads by ``tools.loomhost.SimTransport``.

Why the design is what it is
----------------------------

* **Synchronous test bodies through the thread bridge.** The test bodies are
  plain functions that call ``bench.step(n)`` and ``loom.push(...)``; cocotb
  code must ``await``. Rather than keep an async copy of every test, a cocotb
  test runs the body with ``cocotb.task.bridge`` (a thread the scheduler
  hands control to while the simulator is paused) and ``RtlBench.step``
  enters the simulator through ``cocotb.task.resume``, which runs a
  coroutine on the scheduler and blocks the thread until it returns. Only one
  of the two ever runs, so there is no concurrency to reason about, and a
  failing ``assert`` anywhere (in the body, in a model, in an observer)
  propagates to the cocotb test as it would under pytest.
* **One resume per step, not per cycle.** ``step(n)`` runs all ``n`` cycles
  inside one coroutine. Measured on this design (Icarus 14, cocotb 2.1, this
  laptop): the simulator itself costs 130 to 270 us per simulated clock and
  the per-cycle pad work maybe 35 us more, so a scenario is seconds, not
  minutes; one ``resume`` per 64 cycles costs nothing measurable, while one
  ``resume`` per cycle roughly doubles the cost of a cycle.
* **The bench drives ``clk`` itself.** Two 10 ns timers per cycle, no
  ``Clock`` task: slightly cheaper than a Python clock plus an edge trigger,
  and nothing is left running when a test ends (the GPI clock pile-up that
  ``spi_host.py`` warns about segfaults Icarus at exit).
* **Cycle numbering and pads as in the model.** SEMANTICS 1: edge 0 is the
  last rising edge that samples ``rst_n`` low and cycle ``k`` lies between
  edges ``k`` and ``k + 1``. The bench releases ``rst_n`` in the middle of
  cycle 0 and then, for every cycle ``k``, at the falling edge (every
  register settled): reads ``uo_out``, ``uio_out`` and ``uio_oe`` (cycle-
  ``k`` values), asks the models what they drive, resolves the pads with
  ``tools.protomodels.bench.resolve_pads`` (the arithmetic of
  ``Bench.step``: ``ui_in`` bits from the models over the idle value with
  the host CS_n high, each ``uio`` bit a wired line with the chip's own drive
  looped back, open-drain pulls, pull-ups, contention recorded), writes
  ``ui_in`` and ``uio_drv``, lets the models observe the lines and calls the
  observers; then it raises ``clk`` for edge ``k + 1``, which samples those
  pads. That is exactly the order of ``Bench.step`` (drive, resolve, edge,
  observe), so a model sees the same lines in the same cycle on both.
* **Observers get a** ``tools.loomsim.state.RetireRecord`` built from the
  RTL's retire record (SEMANTICS 8, ``tr_*`` of ``loom_top``) in its W cycle,
  with ``x_cycle`` = W cycle - 1 and the mnemonic decoded through
  ``tools.loomisa``, or ``None`` in a bubble cycle, as the model's
  ``step_cycle`` returns them. The record is read only while an observer is
  registered.

Differences from ``Bench`` that a test can see:

* ``lines.uo`` is all eight ``uo_out`` bits, host MISO (7) and HOST_IRQ (6)
  included (the model's pads stop at ``OUT5``);
* where the chip drives a ``uio`` bit and a model drives it the other way,
  ``tb.v`` shows the chip its own value (the TT pad loopback), where the
  golden model is handed the wired-AND result. Both record the contention;
  only a test that already fails ``contention_free()`` can tell;
* there is no ``machine`` attribute: the chip state is only reachable
  through the host port, as on silicon.

The bench resets the DUT when it is built, so every test body starts from
cycle 0 of a fresh chip (instruction memory is not reset, as on silicon).
"""

from __future__ import annotations

import os
import sys
from typing import Callable, List, Optional, Tuple

import cocotb
from cocotb.triggers import Timer

try:                                    # cocotb 2.x
    from cocotb.task import bridge, resume
except ImportError:                     # pragma: no cover - older 2.0 layouts
    try:
        from cocotb import bridge, resume
    except ImportError:
        from cocotb import external as bridge, function as resume

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO not in sys.path:
    sys.path.insert(0, _REPO)

from tools.loomhost import Loom, SimTransport                      # noqa: E402
from tools.loomisa import Isa                                     # noqa: E402
from tools.loomisa import load as load_isa                        # noqa: E402
from tools.loomsim.state import RetireRecord                      # noqa: E402
from tools.protomodels.bench import (CS_N_BIT, Drive, Lines, Model,  # noqa: E402
                                     PinSpec, pad_of, resolve_pads)

HALF_NS = 10                            # 50 MHz, as test/spi_host.py
RESET_CYCLES = 10

_ISA: Optional[Isa] = None


def _isa() -> Isa:
    global _ISA
    if _ISA is None:
        _ISA = load_isa()
    return _ISA


def _int(handle) -> int:
    """``int(handle.value)``, naming the signal if it holds X or Z."""
    try:
        return int(handle.value)
    except ValueError:
        raise AssertionError("RTL signal %s is not resolvable: %s"
                             % (handle._path, handle.value)) from None


def _pads(handle) -> int:
    """``int(handle.value)`` with X and Z bits read as 0.

    ``uo_out[7]`` (HOST_MISO) is the host shift register's top bit, which has
    no reset, so it is X until the port has loaded its first word; a real host
    sees a level. The pins the firmware drives come from reset registers, so
    only the host pins are ever X here (``test/spi_host.py`` reads them the
    same way).
    """
    try:
        return int(handle.value)
    except ValueError:
        text = str(handle.value)
        return int("".join(ch if ch in "01" else "0" for ch in text), 2) if text else 0


class RtlBench:
    """Steps ``tb.v`` and its pin models together; the API of ``Bench``.

    Must be built and used from a bridge thread (see :func:`run_body`).

    Args:
        dut: the cocotb handle of ``tb``.
        pullups: ``uio`` bits with an external pull-up (I2C lines, say).
        ui_idle: ``ui_in`` value where no model drives (host CS_n high).
        reset_cycles: rising edges with ``rst_n`` low before cycle 0.
    """

    def __init__(self, dut, *, pullups: int = 0, ui_idle: int = CS_N_BIT,
                 isa: Optional[Isa] = None, reset_cycles: int = RESET_CYCLES) -> None:
        self.dut = dut
        self.isa = isa or _isa()
        self.pullups = pullups & 0xFF
        self.ui_idle = ui_idle & 0xFF
        self.models: List[Model] = []
        self.observers: List[Callable[[object], None]] = []
        self.contentions: List[Tuple[int, int]] = []
        self.lines = Lines()
        self._drive = Drive()
        self._cycle = 0
        self._half = Timer(HALF_NS, unit="ns")   # one trigger, reused
        self._ui = self._uio = -1
        self._mnemonic = {}
        loom = dut.user_project.u_loom
        self._tr = (loom.tr_valid, loom.tr_thread, loom.tr_pc, loom.tr_ir, loom.tr_done,
                    loom.tr_we, loom.tr_rd, loom.tr_val, loom.tr_flags, loom.tr_next_pc)
        resume(self._reset)(reset_cycles)

    # --------------------------------------------------------------- setup
    def add(self, model: Model) -> Model:
        self.models.append(model)
        return model

    def remove(self, model: Model) -> None:
        self.models.remove(model)

    def add_observer(self, fn: Callable[[object], None]) -> None:
        """``fn(record)`` after every cycle; ``record`` may be ``None``."""
        self.observers.append(fn)

    def pad(self, pin: PinSpec):
        return pad_of(pin, self.isa)

    @property
    def cycle(self) -> int:
        """The next cycle to run (cycles run since the reset)."""
        return self._cycle

    # ---------------------------------------------------------------- clock
    def step(self, cycles: int = 1) -> None:
        """Advance ``cycles`` clocks (one trip into the simulator)."""
        if cycles > 0:
            resume(self._cycles)(cycles)

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

    # ------------------------------------------------------------ internals
    async def _reset(self, cycles: int) -> None:
        dut = self.dut
        dut.ena.value = 1
        dut.rst_n.value = 0
        dut.ui_in.value = self.ui_idle
        dut.uio_drv.value = self.pullups
        dut.clk.value = 0
        await self._half
        for i in range(cycles):
            dut.clk.value = 1               # a rising edge that samples rst_n low
            await self._half
            dut.clk.value = 0
            if i + 1 < cycles:
                await self._half
        dut.rst_n.value = 1                 # mid cycle 0: the last edge was edge 0
        self._cycle = 0

    def _record(self, cycle: int) -> Optional[RetireRecord]:
        tr = self._tr
        if not _int(tr[0]):
            return None
        ir = _int(tr[3])
        mnemonic = self._mnemonic.get(ir, False)
        if mnemonic is False:
            decoded = self.isa.decode(ir)
            mnemonic = decoded[0].name if decoded is not None else None
            self._mnemonic[ir] = mnemonic
        return RetireRecord(x_cycle=cycle - 1, thread=_int(tr[1]), pc=_int(tr[2]), ir=ir,
                            done=bool(_int(tr[4])), we=bool(_int(tr[5])), rd=_int(tr[6]),
                            val=_int(tr[7]), flags=_int(tr[8]), next_pc=_int(tr[9]),
                            mnemonic=mnemonic)

    async def _cycles(self, count: int) -> None:
        dut = self.dut
        clk, ui_in, uio_drv = dut.clk, dut.ui_in, dut.uio_drv
        uo_out, uio_out, uio_oe = dut.uo_out, dut.uio_out, dut.uio_oe
        half = self._half
        d = self._drive
        lines = self.lines
        models = self.models
        observers = self.observers
        pullups, ui_idle = self.pullups, self.ui_idle
        for _ in range(count):
            # Mid cycle k: every register has settled since edge k.
            cycle = self._cycle
            uo = _pads(uo_out)
            out = _int(uio_out)
            oe = _int(uio_oe)
            d.clear()
            for model in models:
                model.drive(d, cycle)
            ui, uio, conflict = resolve_pads(d, out, oe, pullups, ui_idle)
            if conflict:
                self.contentions.append((cycle, conflict))
            if ui != self._ui:
                ui_in.value = ui
                self._ui = ui
            if uio != self._uio:
                uio_drv.value = uio
                self._uio = uio
            record = self._record(cycle) if observers else None
            lines.cycle = cycle
            lines.ui = ui
            lines.uio = uio
            lines.uo = uo
            lines.uio_out = out
            lines.uio_oe = oe
            for model in models:
                model.observe(lines)
            for fn in observers:
                fn(record)
            self._cycle = cycle + 1
            await half
            clk.value = 1                   # edge k + 1 samples the pads above
            await half
            clk.value = 0


#: Host polls (FIFO status reads, about 400 clocks each) before ``Loom`` gives
#: up. The model's 10000 would be a quarter of an hour of simulation.
RTL_MAX_POLLS = 200


class RtlBackend:
    """The ``backend`` of the L3 test bodies on the RTL (``tools/tests/fw_backend.py``)."""

    name = "rtl"

    def __init__(self, dut) -> None:
        self.dut = dut

    def bench(self, **kwargs) -> RtlBench:
        return RtlBench(self.dut, **kwargs)

    def transport(self, bench: RtlBench) -> SimTransport:
        return SimTransport(bench)

    def loom(self, bench: RtlBench, isa: Optional[Isa] = None, **kwargs) -> Loom:
        """The host API over the SPI pads of ``bench``."""
        kwargs.setdefault("max_polls", RTL_MAX_POLLS)
        return Loom(self.transport(bench), isa=isa, **kwargs)


async def run_body(dut, body: Callable[..., object], *args, **kwargs):
    """Run a synchronous test body in a bridge thread; returns its result.

    The trailing timer is not cosmetic: a cocotb test that ends in the same
    time step as its last signal write (here ``clk`` going low at the end of
    the last cycle) makes ``vvp`` segfault at ``$finish``, which deletes
    ``results.xml`` and fails the run. Letting simulation time advance once
    more before the test returns is enough; measured on Icarus 14 with
    cocotb 2.1, with a cocotb ``Clock`` and without, with the thread bridge
    and without.
    """
    try:
        return await bridge(body)(*args, **kwargs)
    finally:
        await Timer(HALF_NS, unit="ns")
