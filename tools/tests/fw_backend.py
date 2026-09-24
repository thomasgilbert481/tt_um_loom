"""Backends for the L3 firmware tests: one test body, golden model or RTL.

``test_fw_uart.py``, ``test_fw_spi.py``, ``test_fw_i2c.py`` and
``test_fw_spi_slave.py`` build everything they touch through a ``backend``:

    bench = backend.bench(pullups=0b11)          # the API of protomodels.Bench
    loom = backend.loom(bench, isa=ISA)          # tools.loomhost.Loom on it

so the same body runs

* on the **golden model** (:data:`MODEL`): ``tools.protomodels.bench.Bench``
  steps a ``tools.loomsim.Machine`` and ``tools.loomhost.ModelTransport``
  moves the host bytes. This is what pytest runs, through the ``backend``
  fixture of ``tools/tests/conftest.py``;
* on the **RTL** (``test/rtl_bench.py``, ``RtlBackend``): ``RtlBench`` clocks
  ``tb.v`` one edge per bench cycle with the same pad resolution and
  ``tools.loomhost.SimTransport`` clocks the host bytes through the real SPI
  pads. ``test/test_fw.py`` collects the bodies with :func:`scenarios` and
  runs each as a cocotb test.

On the model, a valid slot that fetches or ``LD``s a word the image never
loaded and nothing stored fails the scenario at that read
(:class:`UnloadedReadError`), the check ``test/tb.v`` makes on the RTL, where
such a word is X (``docs/spec-questions/firmware-m3.md`` item 9).

A body the RTL run must skip carries :func:`model_only` with the reason: the
slow 115200-baud UART cases (434 clocks a bit) and the middle of the SPI
master's mode sweep. A simulated clock costs a fraction of a millisecond in
Icarus, so those would add minutes and prove what their faster siblings
already prove.
"""

from __future__ import annotations

import dataclasses
import inspect
from typing import Any, Callable, Dict, Iterator, List, Optional, Sequence, Tuple

from tools.loomhost import Loom, ModelTransport
from tools.protomodels.bench import Bench


class UnloadedReadError(AssertionError):
    """A valid slot read a memory word no image loaded and nothing stored."""


def _fail_unloaded_read(cycle: int, thread: int, addr: int) -> None:
    raise UnloadedReadError(
        "cycle %d: thread %d's slot reads IMEM word 0x%03X, which no image "
        "loaded and nothing stored (the RTL testbench stops on the X there)"
        % (cycle, thread, addr))


class ModelBackend:
    """Benches and transports that run against ``tools.loomsim``."""

    name = "model"

    def bench(self, **kwargs) -> Bench:
        bench = Bench(**kwargs)
        bench.machine.on_unloaded_read = _fail_unloaded_read
        return bench

    def transport(self, bench: Bench) -> ModelTransport:
        return ModelTransport(bench)

    def loom(self, bench: Bench, isa=None, **kwargs) -> Loom:
        """The host API over this backend's transport on ``bench``."""
        return Loom(self.transport(bench), isa=isa, **kwargs)

    def __repr__(self) -> str:                      # pytest ids
        return self.name


#: The golden-model backend (one instance is enough: it holds no state).
MODEL = ModelBackend()


def model_only(reason: str) -> Callable:
    """Mark a test body that only runs on the golden model, with the reason."""
    def decorate(fn):
        fn.model_only = reason
        return fn
    return decorate


# --------------------------------------------------------------- collection
@dataclasses.dataclass
class Scenario:
    """One test body with one set of parameters."""

    name: str                       # test name with the parameters in it
    func: Callable
    kwargs: Dict[str, Any]
    module: str
    model_only: Optional[str] = None
    doc: Optional[str] = None


def _names(argnames) -> List[str]:
    if isinstance(argnames, str):
        return [n.strip() for n in argnames.split(",") if n.strip()]
    return list(argnames)


def _slug(value: Any) -> str:
    text = "%s" % (value,)
    return "".join(ch if ch.isalnum() else "_" for ch in text)


def _parametrize(func) -> List[Tuple[str, Dict[str, Any]]]:
    """``[(name suffix, kwargs)]`` for every combination of the parametrize
    marks on ``func`` (the marks pytest itself reads, expanded here so the
    cocotb run covers the same cases)."""
    combos: List[Tuple[str, Dict[str, Any]]] = [("", {})]
    for mark in getattr(func, "pytestmark", []):
        if getattr(mark, "name", None) != "parametrize":
            continue
        names = _names(mark.args[0])
        grown = []
        for suffix, kwargs in combos:
            for values in mark.args[1]:
                row: Sequence[Any] = values if len(names) > 1 else [values]
                extra = dict(zip(names, row))
                bit = "".join("_%s_%s" % (n, _slug(v)) for n, v in extra.items())
                grown.append((suffix + bit, {**kwargs, **extra}))
        combos = grown
    return combos


def scenarios(module) -> Iterator[Scenario]:
    """Every ``test_*`` body of ``module`` that takes a ``backend``, with its
    parametrize marks expanded into one :class:`Scenario` each."""
    for name, func in sorted(vars(module).items()):
        if not name.startswith("test_") or not inspect.isfunction(func):
            continue
        if "backend" not in inspect.signature(func).parameters:
            continue
        for suffix, kwargs in _parametrize(func):
            yield Scenario(name=name + suffix, func=func, kwargs=kwargs,
                           module=module.__name__.rsplit(".", 1)[-1],
                           model_only=getattr(func, "model_only", None),
                           doc=inspect.getdoc(func))
