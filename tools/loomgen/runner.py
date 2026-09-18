"""Run a generated program on the golden model alone.

This is the model half of what ``test/test_cosim.py`` does, with the same
host-action schedule, so a failing co-simulation seed can be replayed and
inspected without a simulator:

* cycle ``LOAD_CYCLE``: the host writes every word of the image (committed at
  the edge that ends that cycle, SEMANTICS 7);
* cycle ``RUN_CYCLE - 1``: the host writes ``CTRL.RUN``, so ``RUN`` is visible
  from cycle ``RUN_CYCLE``;
* every cycle ``k``: the pads hold ``plan.at(k)`` at the edge that ends it.

Non-default entry points are set with a debug ``PC`` write in the load cycle,
which SEMANTICS 7 allows while a thread is halted.
"""

from __future__ import annotations

from typing import Callable, List, Optional

from tools.loomsim import Machine, RetireRecord

from .generator import GeneratedProgram

LOAD_CYCLE = 0
#: A multiple of four, so the first slot of the run belongs to thread 0.
RUN_CYCLE = 4


def host_actions(machine: Machine, prog: GeneratedProgram, cycle: int) -> bool:
    """Issue the host actions scheduled for ``cycle``; True if there were any."""
    if cycle == LOAD_CYCLE:
        for addr in range(prog.imem_words):
            machine.host_write_imem(addr, prog.image.get(addr, 0))
        for t in range(len(prog.entries)):
            if prog.entries[t] != prog.default_entries[t]:
                machine.host_write_debug(t, "PC", prog.entries[t])
        return True
    if cycle == RUN_CYCLE - 1:
        machine.host_set_run(prog.run_mask)
        return True
    return False


def run_model(prog: GeneratedProgram, cycles: int,
              on_record: Optional[Callable[[RetireRecord], None]] = None,
              machine: Optional[Machine] = None) -> Machine:
    """Run ``prog`` from reset for ``cycles`` cycles after ``RUN_CYCLE``."""
    machine = machine or Machine(imem_words=prog.imem_words, loopback=True)
    plan = prog.stimulus
    for cycle in range(RUN_CYCLE + cycles):
        host_actions(machine, prog, cycle)
        ui, uio = plan.at(cycle)
        machine.set_pad_inputs(ui_in=ui, uio_in=uio)
        record = machine.step_cycle()
        if record is not None and on_record is not None:
            on_record(record)
    return machine


def trace(prog: GeneratedProgram, cycles: int,
          thread: Optional[int] = None) -> List[RetireRecord]:
    """Every retire record of a run (optionally of one thread only)."""
    out: List[RetireRecord] = []

    def keep(record: RetireRecord) -> None:
        if thread is None or record.thread == thread:
            out.append(record)

    run_model(prog, cycles, keep)
    return out
