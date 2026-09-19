"""Run a generated program on the golden model alone.

This is the model half of what ``test/test_cosim.py`` does, with the same
host-action schedule, so a failing co-simulation seed can be replayed and
inspected without a simulator:

* cycle ``LOAD_CYCLE``: the host writes every word of the image (committed at
  the edge that ends that cycle, SEMANTICS 7);
* cycle ``RUN_CYCLE - 1``: the host writes ``CTRL.RUN``, so ``RUN`` is visible
  from cycle ``RUN_CYCLE``;
* every cycle ``k``: the pads hold ``plan.at(k)`` at the edge that ends it.

The model is built for the program's own build (``prog.features``,
``prog.fifo_depth``), which is the build ``CTRL.CAPS`` reported when the
program was made.

Non-default entry points are set with a debug ``PC`` write in the load cycle,
which SEMANTICS 7 allows while a thread is halted.

Host traffic. A program generated with ``host_traffic=True`` carries a
:class:`~tools.loomgen.hostplan.HostPlan`; :class:`ModelHost` replays it here
with the SPI timing of ``test/spi_host.py`` approximated (a byte is 64 clocks
and every effect of a word lands at the end of the word). That is enough to
keep raw ``PUSH``/``POP`` programs live in the model, but it is **not** the
co-simulation's schedule: there every host action commits at the cycle
observed from the RTL (SEMANTICS 10). To replay a co-simulation failure
exactly, use the host log the harness writes into the failure JSON
(:func:`replay_host_log`), which has the observed cycle of every action.
"""

from __future__ import annotations

from typing import Callable, Dict, List, Optional, Sequence, Tuple

from tools.loomsim import Machine, RetireRecord

from .generator import GeneratedProgram
from .hostplan import CLOCKS_PER_BYTE, HostPlan, HostTxn

LOAD_CYCLE = 0
#: A multiple of four, so the first slot of the run belongs to thread 0.
RUN_CYCLE = 4

#: Clocks of CS_n setup before the first byte of a transaction, and of hold
#: plus release after the last one (``test/spi_host.py``).
CS_SETUP = 4
CS_HOLD = 12


def build_machine(prog: GeneratedProgram, **kwargs) -> Machine:
    """A model built like the chip the program was generated for."""
    return Machine(imem_words=prog.imem_words, features=prog.features,
                   fifo_depth=prog.fifo_depth, loopback=True, **kwargs)


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


# ------------------------------------------------------------- host actions
def fifo_error(machine: Machine) -> None:
    """Set ``BADOP[14]`` at the edge that ends this cycle, and nothing else.

    SEMANTICS 6.7: a FIFO read word whose peek found ``OUTQ`` empty pops
    nothing and sets the host FIFO error at the end of the word. The model
    does that inside :meth:`Machine.host_fifo_pop` when the queue is still
    empty in the pop cycle, but a thread ``PUSH`` may have filled it between
    the peek and the pop, and then ``host_fifo_pop`` would pop an entry the
    port never read. :meth:`Machine.host_fifo_error` is the error alone.
    """
    machine.host_fifo_error()


def pop_after_peek(machine: Machine, thread: int, peeked: Optional[int]) -> None:
    """The pop half of the port's split pop (SEMANTICS 6.7), at this cycle's
    edge: pop when the peek found an entry, otherwise only the error bit."""
    if peeked is not None:
        machine.host_fifo_pop(thread)
    elif not machine.threads[thread].outq:
        machine.host_fifo_pop(thread)                # empty then and now
    else:
        fifo_error(machine)


def peek_word(machine: Machine, thread: int, popping: bool) -> Optional[int]:
    """What a FIFO read word takes from ``OUTQ[thread]`` as it is loaded: the
    head, or the entry after it when the previous word of the same
    transaction is being popped (SEMANTICS 6.7). ``None`` if there is none."""
    queue = machine.threads[thread].outq
    index = 1 if popping else 0
    return queue[index] if len(queue) > index else None


# ------------------------------------------------------- the host plan, model
class ModelHost:
    """Replays a :class:`HostPlan` on the model with approximate SPI timing.

    See the module docstring: plausible, not cycle-exact. Every transaction
    takes its gap, the CS_n setup, 64 clocks a byte and the CS_n hold; each
    word's effects land at the end of the word, and a FIFO read is an atomic
    peek-and-pop there.
    """

    def __init__(self, plan: HostPlan, start: int, run_mask: int = 0):
        self.plan = plan
        self.run_mask = run_mask
        self.index = 0
        self.events: List[Tuple[int, HostTxn, int]] = []   # (cycle, txn, word)
        self.next_cycle = start
        self.log: List[Tuple] = []

    def _schedule(self) -> None:
        txn = self.plan[self.index]
        self.index += 1
        start = self.next_cycle + txn.gap + CS_SETUP
        if txn.kind == "push":
            for j in range(len(txn.words)):
                self.events.append((start + CLOCKS_PER_BYTE * (5 + 2 * j), txn, j))
        elif txn.kind == "pop":
            for j in range(txn.count):
                self.events.append((start + CLOCKS_PER_BYTE * (6 + 2 * j), txn, j))
        else:
            self.events.append((start + CLOCKS_PER_BYTE * 5, txn, 0))
        self.next_cycle = start + CLOCKS_PER_BYTE * txn.nbytes + CS_HOLD

    def at(self, machine: Machine, cycle: int) -> None:
        """Issue whatever the plan does in ``cycle`` (before the model steps)."""
        while self.next_cycle <= cycle:
            self._schedule()
        due = [e for e in self.events if e[0] == cycle]
        self.events = [e for e in self.events if e[0] > cycle]
        for _, txn, word in due:
            self.apply(machine, txn, word)

    def apply(self, machine: Machine, txn: HostTxn, word: int) -> None:
        if txn.kind == "push":
            machine.host_fifo_push(txn.thread, txn.words[word])
        elif txn.kind == "pop":
            pop_after_peek(machine, txn.thread,
                           peek_word(machine, txn.thread, False))
        elif txn.reg == "IRQ_EN":
            machine.host_write_irq_en(txn.value)
        elif txn.reg == "IRQ_EN2":
            machine.host_write_irq_en2(txn.value)
        elif txn.reg == "SWIRQ":
            machine.host_clear_swirq(txn.value)
        elif txn.reg == "BADOP":
            machine.host_clear_badop(txn.value)
        elif txn.reg == "SFLAGS":
            machine.host_write_sflags_set(txn.value)
        elif txn.reg == "SFLAGS_CLR":
            machine.host_write_sflags_clr(txn.value)
        elif txn.reg == "RUN":
            machine.host_set_run(txn.value)
        self.log.append((machine.cycle, str(txn)))


# --------------------------------------------------------- host log replay
#: One recorded host action: ``(cycle, kind, *args)``. The harness writes
#: these into a failure JSON (``test/test_cosim.py``), the cycle being the one
#: in which the action was issued to the model, so replaying them here puts
#: every commit on the edge the RTL committed it.
HostLog = Sequence[Sequence]


def replay_host_log(machine: Machine, log_by_cycle: Dict[int, List], cycle: int) -> None:
    """Issue every logged action of ``cycle``."""
    for entry in log_by_cycle.get(cycle, ()):
        kind, args = entry[0], entry[1:]
        if kind == "imem_write":
            machine.host_write_imem(args[0], args[1])
        elif kind == "imem_read":
            machine.host_read_imem(args[0])
        elif kind == "run":
            machine.host_set_run(args[0])
        elif kind == "fifo_push":
            machine.host_fifo_push(args[0], args[1])
        elif kind == "fifo_pop":
            machine.host_fifo_pop(args[0])
        elif kind == "fifo_error":
            fifo_error(machine)
        elif kind == "irq_en":
            machine.host_write_irq_en(args[0])
        elif kind == "irq_en2":
            machine.host_write_irq_en2(args[0])
        elif kind == "swirq_clr":
            machine.host_clear_swirq(args[0])
        elif kind == "badop_clr":
            machine.host_clear_badop(args[0])
        elif kind == "sflags_set":
            machine.host_write_sflags_set(args[0])
        elif kind == "sflags_clr":
            machine.host_write_sflags_clr(args[0])
        else:
            raise ValueError("unknown host log action %r" % (kind,))


def group_host_log(log: HostLog) -> Dict[int, List]:
    """``{cycle: [action, ...]}`` from the flat list a failure JSON carries."""
    out: Dict[int, List] = {}
    for entry in log or ():
        out.setdefault(int(entry[0]), []).append(list(entry[1:]))
    return out


# ------------------------------------------------------------------ running
def run_model(prog: GeneratedProgram, cycles: int,
              on_record: Optional[Callable[[RetireRecord], None]] = None,
              machine: Optional[Machine] = None,
              host_log: Optional[HostLog] = None,
              host_traffic: bool = True) -> Machine:
    """Run ``prog`` from reset for ``cycles`` cycles after ``RUN_CYCLE``.

    ``host_log`` replays the host actions a co-simulation run observed (and
    then the program's own host plan is not used). Otherwise, if the program
    has a host plan and ``host_traffic`` is set, :class:`ModelHost` drives it.
    """
    machine = machine or build_machine(prog)
    plan = prog.stimulus
    by_cycle = group_host_log(host_log) if host_log else None
    driver = ModelHost(prog.host, RUN_CYCLE, prog.run_mask) \
        if (prog.host is not None and host_traffic and by_cycle is None) else None
    for cycle in range(RUN_CYCLE + cycles):
        if by_cycle is not None:
            replay_host_log(machine, by_cycle, cycle)
        else:
            host_actions(machine, prog, cycle)
            if driver is not None:
                driver.at(machine, cycle)
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
