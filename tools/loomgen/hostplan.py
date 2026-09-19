"""Host traffic plans: what the host does over SPI while the threads run.

``test_cosim_over_the_host_port`` (``test/test_cosim.py``) sends a plan
through the real SPI pads after ``CTRL.RUN``: FIFO pushes into ``INQ[t]``,
FIFO reads that pop ``OUTQ[t]`` (``docs/SEMANTICS.md`` 6.7), and CTRL writes
that work the host interrupt (6.8): ``IRQ_EN``, ``IRQ_EN2``, the ``SWIRQ`` and
``BADOP`` write-1-to-clear registers, ``SFLAGS``/``SFLAGS_CLR`` and a ``RUN``
write that restarts threads a ``HALT`` stopped. The transactions run one
after another, each after its ``gap`` of idle clocks with CS_n high, and the
list starts over at the top when it runs out, so the traffic lasts as long as
the run does.

Liveness contract with :mod:`tools.loomgen.generator`: every running thread
gets one push into its ``INQ`` and one read of its ``OUTQ`` in every round of
the plan (in a shuffled order), and a round is a handful of transactions, so
a thread that blocks in a raw ``POP`` (empty ``INQ``) or a raw ``PUSH`` (full
``OUTQ``) is served within two rounds, a few thousand clocks. The generator
only emits the raw forms when the program comes with a plan
(``generate(host_traffic=True)``); without one it guards every
``PUSH``/``POP`` with a timed ``WAITB``.

The plan also aims at the host error paths on purpose: pushes into the INQ
of a thread that is not running fill it and then set ``BADOP[14]``, and
reads of such a thread's (always empty) OUTQ peek nothing and set it too.
Reads of two or three words exercise the "entry after the head" rule for a
word loaded while the previous one is being popped.

Everything is plain data, so a plan round-trips through the co-simulation
failure JSON with the program.
"""

from __future__ import annotations

import dataclasses
from typing import Dict, List, Sequence, Tuple

#: CTRL registers a plan may write (HOST_PROTOCOL space 0), by name. The
#: harness maps them to addresses; the model call for each is in the harness.
CTRL_WRITES: Tuple[str, ...] = ("IRQ_EN", "IRQ_EN2", "SWIRQ", "BADOP", "SFLAGS",
                                "SFLAGS_CLR", "RUN")

TXN_KINDS = ("push", "pop", "write")

#: SPI cost in core clocks (``test/spi_host.py``: SCK = clk / 8, so 64 clocks
#: a byte, 4 clocks of CS setup, 8 + 4 clocks of hold and release).
CLOCKS_PER_BYTE = 64
TXN_OVERHEAD = 16


@dataclasses.dataclass(frozen=True)
class HostTxn:
    """One CS-framed host transaction.

    ``push``: ``words`` go into ``INQ[thread]`` (FIFO space 0x0000 + t).
    ``pop``: ``count`` words are read from ``OUTQ[thread]``.
    ``write``: ``value`` is written to the CTRL register ``reg``.
    ``gap``: idle clocks, CS_n high, before the transaction starts.
    """

    kind: str
    thread: int = 0
    words: Tuple[int, ...] = ()
    count: int = 0
    reg: str = ""
    value: int = 0
    gap: int = 0

    @property
    def nbytes(self) -> int:
        """Bytes on the wire: command, two address bytes, (dummy,) data."""
        if self.kind == "push":
            return 3 + 2 * len(self.words)
        if self.kind == "pop":
            return 4 + 2 * self.count
        return 5

    @property
    def clocks(self) -> int:
        """Rough length in clocks, gap included (for the model-only driver)."""
        return self.gap + TXN_OVERHEAD + CLOCKS_PER_BYTE * self.nbytes

    def to_obj(self) -> Dict:
        return {"kind": self.kind, "thread": self.thread, "words": list(self.words),
                "count": self.count, "reg": self.reg, "value": self.value,
                "gap": self.gap}

    @staticmethod
    def from_obj(obj: Dict) -> "HostTxn":
        return HostTxn(kind=str(obj["kind"]), thread=int(obj.get("thread", 0)),
                       words=tuple(int(w) for w in obj.get("words", ())),
                       count=int(obj.get("count", 0)), reg=str(obj.get("reg", "")),
                       value=int(obj.get("value", 0)), gap=int(obj.get("gap", 0)))

    def __str__(self) -> str:
        if self.kind == "push":
            return "push INQ[%d] %s" % (self.thread, " ".join("%04X" % w for w in self.words))
        if self.kind == "pop":
            return "read OUTQ[%d] x%d" % (self.thread, self.count)
        return "write %s=%04X" % (self.reg, self.value)


class HostPlan:
    """The transactions, in order; the harness cycles through them."""

    def __init__(self, txns: Sequence[HostTxn]):
        self.txns: List[HostTxn] = list(txns)
        for txn in self.txns:
            if txn.kind not in TXN_KINDS:
                raise ValueError("unknown host transaction kind %r" % txn.kind)
            if txn.kind in ("push", "pop") and not 0 <= txn.thread < 4:
                raise ValueError("host transaction for thread %r" % txn.thread)
            if txn.kind == "write" and txn.reg not in CTRL_WRITES:
                raise ValueError("host write to %r, which a plan may not write" % txn.reg)
        if not self.txns:
            raise ValueError("an empty host plan")

    def __len__(self) -> int:
        return len(self.txns)

    def __getitem__(self, index: int) -> HostTxn:
        return self.txns[index % len(self.txns)]

    def to_obj(self) -> Dict:
        return {"txns": [t.to_obj() for t in self.txns]}

    @staticmethod
    def from_obj(obj: Dict) -> "HostPlan":
        return HostPlan([HostTxn.from_obj(t) for t in obj["txns"]])

    def __eq__(self, other) -> bool:
        return isinstance(other, HostPlan) and self.to_obj() == other.to_obj()

    def __repr__(self) -> str:
        return "<HostPlan %d transactions>" % len(self.txns)


def _word(rng) -> int:
    return rng.choice((rng.randrange(1 << 16), rng.randrange(256),
                       rng.choice((0x0000, 0xFFFF, 0x8000, 0x0001, 0x5A5A))))


def build_host_plan(rng, run_mask: int, fifo_depth: int, rounds: int = 6,
                    extras: Sequence[str] = CTRL_WRITES) -> HostPlan:
    """A plan of ``rounds`` rounds; see the module docstring for the rules.

    It opens by enabling every interrupt cause (``IRQ_EN``, ``IRQ_EN2``), so
    ``HOST_IRQ`` follows the FIFOs, the shared flags and ``HALTED`` for the
    whole run and the co-simulation compares a pin that moves. Every round
    then has two pushes and one read per running thread (in a random order),
    one CTRL write taken from a rotation so that all of them appear within a
    few rounds, and now and then a push or read of a thread that is not
    running, whose INQ nothing drains and whose OUTQ nothing fills: both end
    in ``BADOP[14]`` (SEMANTICS 6.7). Gaps are short, and a multiple of
    nothing in particular, so words land in every phase of the slot grid.
    """
    running = [t for t in range(4) if (run_mask >> t) & 1]
    idle = [t for t in range(4) if not (run_mask >> t) & 1]
    if not running:
        raise ValueError("run_mask selects no thread")
    txns: List[HostTxn] = []

    def gap() -> int:
        # Short: a transaction already costs 350 clocks and a thread blocked
        # in a raw POP waits for its turn in the round.
        return rng.choice((0, 0, 0, 0, 3, 7, 13, 29, 61))

    def push(thread: int) -> HostTxn:
        count = rng.choice((1, 1, 1, 2, 2, 3))
        if rng.random() < 0.15:
            count = fifo_depth + 1                  # overflows unless popped meanwhile
        return HostTxn("push", thread=thread, words=tuple(_word(rng) for _ in range(count)),
                       gap=gap())

    def pop(thread: int) -> HostTxn:
        return HostTxn("pop", thread=thread, count=rng.choice((1, 1, 1, 2, 2, 3)),
                       gap=gap())

    def ctrl(reg: str = "") -> HostTxn:
        reg = reg or rng.choice(list(extras))
        if reg == "IRQ_EN":
            value = rng.choice((rng.randrange(1 << 16), 0x00F0, 0x000F, 0xFF00,
                                0xFFFF, 0x0000))
        elif reg == "IRQ_EN2":
            value = rng.choice((rng.randrange(16), 0xF, 0x0))
        elif reg in ("SWIRQ", "BADOP"):
            value = rng.choice((0xFFFF, rng.randrange(1 << 16), 1 << 14))
        elif reg in ("SFLAGS", "SFLAGS_CLR"):
            value = rng.randrange(256)
        else:                                       # RUN: restart what HALT stopped
            value = run_mask
        return HostTxn("write", reg=reg, value=value, gap=gap())

    # Switch every interrupt cause on first (SEMANTICS 6.8), so HOST_IRQ is
    # a live signal for the rest of the run.
    txns.append(HostTxn("write", reg="IRQ_EN", value=0xFFFF, gap=gap()))
    txns.append(HostTxn("write", reg="IRQ_EN2", value=0xF, gap=gap()))
    rotation = [r for r in extras]
    rng.shuffle(rotation)
    for index in range(max(1, rounds)):
        batch: List[HostTxn] = []
        for thread in running:
            batch.append(push(thread))
            batch.append(push(thread))
            batch.append(pop(thread))
        for step in (0, 1):
            batch.append(ctrl(rotation[(2 * index + step) % len(rotation)]))
        if rng.random() < 0.5:
            thread = rng.choice(idle or running)
            batch.append(push(thread) if rng.random() < 0.5 else pop(thread))
        rng.shuffle(batch)
        txns += batch
        if index == max(1, rounds) // 2:
            # One quiet stretch, in this order: with no cause enabled and
            # SWIRQ cleared, HOST_IRQ has to go low (SEMANTICS 6.8), and the
            # next write brings it back for the shared flags.
            txns += [HostTxn("write", reg="IRQ_EN", value=0x0000, gap=gap()),
                     HostTxn("write", reg="IRQ_EN2", value=0x0, gap=gap()),
                     HostTxn("write", reg="SWIRQ", value=0xF, gap=gap()),
                     HostTxn("write", reg="IRQ_EN", value=0xFF00, gap=gap())]
    return HostPlan(txns)
