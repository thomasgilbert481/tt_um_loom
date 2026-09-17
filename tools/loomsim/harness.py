"""Small driving helpers built on :class:`~tools.loomsim.Machine`.

Nothing here adds behaviour to the model: these are the three things every
caller of the model ends up writing anyway, which is loading a list of
instructions at a thread's reset vector, running until that thread halts, and
recording what the pads did while it ran.  Keeping them here means the test
suite, the co-simulation harness and the CLI all drive the model the same way.
"""

from __future__ import annotations

import dataclasses
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Tuple

from .machine import Machine
from .state import RetireRecord, CycleTrace

#: A program is a sequence of ``(mnemonic, {operand: value})`` pairs.
Program = Sequence[Tuple[str, dict]]


def assemble(isa, program: Program, base: int = 0) -> Dict[int, int]:
    """Encode a program through :mod:`tools.loomisa` into an image.

    ``base`` is the address of the first instruction; the model's default reset
    vector for thread ``t`` is ``t * 0x100``.
    """
    image = {}
    for offset, (name, operands) in enumerate(program):
        image[base + offset] = isa.encode(name, **operands)
    return image


@dataclasses.dataclass
class PadTrace:
    """Per-cycle record of the pad outputs, usable as ``Machine(on_cycle=...)``.

    ``samples[k]`` is what the pads held during cycle ``k``, so a write with X
    cycle ``x`` first appears at index ``x + 2``.
    """

    samples: List[Tuple[int, int, int]] = dataclasses.field(default_factory=list)

    def __call__(self, trace: CycleTrace) -> None:
        while len(self.samples) < trace.cycle:
            self.samples.append(self.samples[-1] if self.samples else (0, 0, 0))
        self.samples.append((trace.uo_out, trace.uio_out, trace.uio_oe))

    def uo_bit(self, index: int) -> List[int]:
        """The value of ``uo_out[index]`` in every cycle, one entry per cycle."""
        return [(s[0] >> index) & 1 for s in self.samples]

    def uio_bit(self, index: int) -> List[int]:
        """The value of ``uio_out[index]`` in every cycle."""
        return [(s[1] >> index) & 1 for s in self.samples]

    def changes(self) -> List[Tuple[int, Tuple[int, int, int]]]:
        """``(cycle, pads)`` for every cycle whose pads differ from the previous."""
        out = []
        previous = None
        for cycle, pads in enumerate(self.samples):
            if previous is not None and pads != previous:
                out.append((cycle, pads))
            previous = pads
        return out

    def edges(self, values: List[int]) -> List[Tuple[int, int]]:
        """``(cycle, new_value)`` for every change in a per-cycle bit series."""
        out = []
        for cycle in range(1, len(values)):
            if values[cycle] != values[cycle - 1]:
                out.append((cycle, values[cycle]))
        return out


def run_thread(isa, program: Program, thread: int = 0,
               setup: Optional[Callable[[Machine], None]] = None,
               max_cycles: int = 20000, extra: Optional[Dict[int, int]] = None,
               **machine_kwargs) -> Tuple[Machine, List[RetireRecord]]:
    """Load ``program`` at thread ``t``'s reset vector and run until it halts.

    ``setup`` is called once, before ``RUN`` is written, with the machine as its
    only argument; use it for host debug writes and CSR initialisation.
    ``extra`` adds further image words (subroutines, other threads' code).
    Returns the machine and the retire records of *that thread* only.
    """
    base = thread * 0x100
    image = assemble(isa, program, base)
    if extra:
        image.update(extra)
    machine_kwargs.setdefault("isa", isa)      # reuse the caller's parsed ISA
    machine = Machine(image, **machine_kwargs)
    if setup is not None:
        setup(machine)
    machine.host_set_run(1 << thread)
    records: List[RetireRecord] = []
    for _ in range(max_cycles):
        record = machine.step_cycle()
        if record is not None and record.thread == thread:
            records.append(record)
        if (machine.halted >> thread) & 1:
            return machine, records
    raise AssertionError("thread %d did not halt within %d cycles" % (thread, max_cycles))


def slot_count(records: Iterable[RetireRecord]) -> int:
    """How many slots a run took (every valid slot, done or stalled)."""
    return len(list(records))
