"""Static deadline analysis.

Loom keeps time with a per-thread deadline register ``TD``. ``SETD m`` anchors
it at ``NOW + m``; ``WAITD k`` advances it by ``k`` ticks and stalls until
``NOW`` reaches it. Firmware is correct only if the code between two deadline
instructions always finishes before the later deadline: every slot is exactly
four clocks (``docs/SEMANTICS.md`` section 2), so the budget is

* ``k`` tick periods from a ``WAITD`` to the next ``WAITD k``, and
* ``(m + k)`` tick periods from a ``SETD m`` to the next ``WAITD k``, less the
  clocks the ``SETD`` may run after the tick that set the ``NOW`` it reads (its
  *phase*), plus the three clocks of the target's own slot-grid grace.

A ``WAITD`` completes at most three clocks after its tick, and the next
``WAITD``'s slot is four clocks wide, so ``k * P`` is exact for it. A ``SETD``
can run anywhere in a tick unless the code before it fixes where: the phase
analysis (:func:`_phases`) bounds it from the last ``WAITD k`` completion or
tick restart (``CSRW TICK_INT``/``TICK_FRAC``) on every path, and falls back to
``P - 1`` clocks (tools finding T-1, ``docs/VERIFICATION.md``).

The checker builds each thread's control-flow graph, cuts it at every anchor,
and takes the longest path, in slots, from each anchor to every ``WAITD``
reachable without passing another anchor.

``WAITD 0`` leaves ``TD`` where it is, so it sets no new deadline and is not an
anchor: it is one ordinary slot on the path.

What it is sound about, and what it is not, is listed under "Deadline checker"
in ``tools/loomasm/README.md``.
"""

from __future__ import annotations

import dataclasses
import math
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple

from .diag import DEADLINE, ERROR, WARNING, Diagnostic

SLOT_CLOCKS = 4

#: Mnemonics that can anchor an interval. ``WAITD 0`` is excluded at run time
#: by :func:`is_anchor`, because it does not move ``TD``.
ANCHOR_MNEMONICS = frozenset({"WAITD", "SETD"})
#: Always unbounded: no static bound on how long they stall, unless the
#: source discharges one with ``.bounded`` (see :data:`BOUNDABLE`).
UNBOUNDED_ALWAYS = frozenset({"DLY", "PUSH", "POP"})
#: The blocking instructions a ``.bounded "<reason>"`` declaration may
#: discharge. A FIFO access guarded by a ``WAITB INQ_NE, T`` (or
#: ``OUTQ_NF``) that completed by condition cannot stall, and only the author
#: can see that; ``DLY`` is not here, because stalling is the whole point of
#: it.
BOUNDABLE = frozenset({"PUSH", "POP"})
#: Unbounded unless the instruction carries the T (deadline timeout) bit.
TIMED_WAITS = frozenset({"WAITP", "WAITE", "WAITS", "WAITB"})
#: Conditional transfers: fall through or take the relative branch.
REL_BRANCHES = frozenset({"BZ", "BNZ", "BC", "BNC", "BT", "BNT", "DJNZ", "JP"})

UNBOUNDED = math.inf


@dataclasses.dataclass(frozen=True)
class Node:
    """One instruction of a thread's section, as the analysis sees it.

    ``bounded`` is the reason text of a ``.bounded`` declaration on this
    instruction, or None. It is an assumption the checker takes from the
    author and does not verify; see the README.
    """

    addr: int
    name: str
    fields: Dict[str, int]
    line: int
    #: The ``isa.yaml`` timing class; ``two_slot`` (``LD``/``ST``, SEMANTICS
    #: 6.11) costs two slots on every path, everything else one.
    timing: str = "one_slot"
    #: The reason of a ``.bounded`` declaration on this PUSH/POP, if any.
    bounded: Optional[str] = None


def is_deadline_target(node: Node) -> bool:
    """A ``WAITD k`` with ``k != 0``: it moves ``TD`` and so ends an interval.

    ``WAITD 0`` commits ``TD <= TD + 0`` (``SEMANTICS.md`` 6.4), so on any path
    that reaches it from a ``WAITD`` anchor the deadline has already been
    reached and it behaves as a one-slot NOP. After a ``SETD m`` anchor it does
    wait, but it still sets no new deadline, so it neither ends nor starts an
    interval and is never itself reported as infeasible.
    """
    return node.name == "WAITD" and int(node.fields.get("imm", 0)) != 0


def is_anchor(node: Node) -> bool:
    """A deadline instruction that starts an interval: ``SETD`` or ``WAITD k``."""
    if node.name not in ANCHOR_MNEMONICS:
        return False
    return node.name == "SETD" or is_deadline_target(node)


@dataclasses.dataclass(frozen=True)
class DeadlinePair:
    """Longest path from one deadline instruction to the next ``WAITD``."""

    thread: int
    src_addr: int
    src_name: str
    src_line: int
    dst_addr: int
    dst_line: int
    ticks: int                      # the k of the target's WAITD k
    slots: float                    # math.inf when unbounded
    period: Optional[int]           # tick period in clocks, None if unknown
    slot_clocks: int = SLOT_CLOCKS  # from isa.yaml meta.slot_clocks
    #: Ticks the anchor itself contributes: the m of ``SETD m``, 0 for a
    #: ``WAITD`` anchor, and None when a ``CSRW TD`` on the path has made the
    #: anchor's own value meaningless (the budget then falls back to k * P).
    src_ticks: Optional[int] = 0
    #: For a ``SETD`` anchor, the most clocks it can run after the tick that
    #: set the ``NOW`` it reads (T-1); None for a ``WAITD`` anchor, whose
    #: three clocks are the grace of the slot grid and cancel out.
    src_phase: Optional[int] = None

    @property
    def unbounded(self) -> bool:
        return self.slots == UNBOUNDED

    @property
    def clocks(self) -> Optional[int]:
        return None if self.unbounded else int(self.slots) * self.slot_clocks

    @property
    def anchor_credit_known(self) -> bool:
        """False when ``CSRW TD`` overwrote the deadline the anchor set."""
        return self.src_ticks is not None

    @property
    def budget_ticks(self) -> int:
        """``m + k`` from a ``SETD m``, ``k`` from a ``WAITD`` anchor."""
        return self.ticks + (self.src_ticks or 0)

    @property
    def limit(self) -> Optional[int]:
        """The most clocks the path may take: ``budget_ticks * P``, less the
        ``SETD``'s phase and plus the target's grace of ``slot_clocks - 1``
        (which is exactly what a ``WAITD`` anchor's own lag cancels)."""
        if self.period is None:
            return None
        base = self.budget_ticks * self.period
        if self.src_phase is None:
            return base
        return base - self.src_phase + (self.slot_clocks - 1)

    @property
    def slack(self) -> Optional[int]:
        if self.limit is None or self.clocks is None:
            return None
        return self.limit - self.clocks

    @property
    def feasible(self) -> Optional[bool]:
        """True/False when it can be decided, None when it cannot."""
        slack = self.slack
        return None if slack is None else slack >= 0


@dataclasses.dataclass(frozen=True)
class Declaration:
    """One ``.bounded`` declaration the analysis took on trust."""

    addr: int
    name: str                       # the mnemonic it was attached to
    line: int
    reason: str

    def __str__(self) -> str:
        return "0x%03X %s (line %d): %s" % (
            self.addr, self.name, self.line, self.reason)


@dataclasses.dataclass
class ThreadDeadlines:
    thread: int
    period: Optional[int]
    enabled: bool
    slot_clocks: int = SLOT_CLOCKS
    pairs: List[DeadlinePair] = dataclasses.field(default_factory=list)
    notes: List[str] = dataclasses.field(default_factory=list)
    declarations: List[Declaration] = dataclasses.field(default_factory=list)

    @property
    def infeasible(self) -> List[DeadlinePair]:
        return [p for p in self.pairs if p.feasible is False]

    @property
    def unbounded(self) -> List[DeadlinePair]:
        return [p for p in self.pairs if p.unbounded]

    @property
    def worst_slack(self) -> Optional[int]:
        slacks = [p.slack for p in self.pairs if p.slack is not None]
        return min(slacks) if slacks else None

    @property
    def worst_slots(self) -> Optional[int]:
        counted = [int(p.slots) for p in self.pairs if not p.unbounded]
        return max(counted) if counted else None


def _slot_cost(node: Node) -> float:
    if node.name in UNBOUNDED_ALWAYS:
        if node.bounded and node.name in BOUNDABLE:
            return 1.0                  # discharged by a .bounded declaration
        return UNBOUNDED
    if node.name in TIMED_WAITS and not node.fields.get("tmo", 0):
        return UNBOUNDED
    if node.timing == "two_slot":
        return 2.0
    return 1.0


class _Graph:
    """The control-flow graph of one thread's section."""

    def __init__(self, nodes: Dict[int, Node], pc_bits: int):
        self.nodes = nodes
        self.mask = (1 << pc_bits) - 1
        self.left_section = False
        self.returns = sorted(
            {(n.addr + 1) & self.mask for n in nodes.values() if n.name == "CALL"}
            & set(nodes))
        self._raw: Dict[int, List[int]] = {}
        for addr in nodes:
            self._raw[addr] = self._compute_raw(addr)

    def _compute_raw(self, addr: int) -> List[int]:
        node = self.nodes[addr]
        name, fields = node.name, node.fields
        if name == "HALT":
            return []
        if name in ("JMP", "CALL"):
            targets = [fields["abs"] & self.mask]
        elif name == "RET":
            targets = list(self.returns)
        elif name in REL_BRANCHES:
            nxt = (addr + 1) & self.mask
            targets = [nxt, (nxt + fields["rel"]) & self.mask]
        else:
            targets = [(addr + 1) & self.mask]
        kept = [t for t in dict.fromkeys(targets) if t in self.nodes]
        if len(kept) != len(set(targets)):
            self.left_section = True
        return kept

    def raw_succs(self, addr: int) -> List[int]:
        return self._raw[addr]

    def cut_succs(self, addr: int) -> List[int]:
        """Successors with the graph cut at every anchor."""
        return [] if is_anchor(self.nodes[addr]) else self._raw[addr]

    def reach_from(self, addr: int) -> Set[int]:
        """Nodes reachable from ``addr`` in the cut graph, ``addr`` included."""
        seen = {addr}
        stack = [addr]
        while stack:
            for succ in self.cut_succs(stack.pop()):
                if succ not in seen:
                    seen.add(succ)
                    stack.append(succ)
        return seen


def _rewrites_td(node: Node, td_csr: Optional[int]) -> bool:
    """``CSRW TD, rN`` replaces the deadline with a value only known at run time."""
    return (td_csr is not None and node.name == "CSRW"
            and node.fields.get("csr") == td_csr)


#: What a path to one target costs: (slots, was TD rewritten on some path).
_Path = Tuple[float, bool]


def _longest_paths(graph: _Graph,
                   td_csr: Optional[int] = None) -> Dict[int, Dict[int, _Path]]:
    """For every node, the longest slot count to each reachable ``WAITD k``.

    The cost of a node includes the node itself, so the entry for a ``WAITD``
    that is reached directly is 1 slot. The second element of each value is
    True when some path to that target rewrites ``TD`` with ``CSRW``, which
    invalidates the anchor's own tick credit.
    """
    nodes = graph.nodes
    reach = {addr: graph.reach_from(addr) for addr in nodes}
    cyclic = {
        addr for addr in nodes
        if any(addr in reach[succ] for succ in graph.cut_succs(addr))
    }
    targets = {
        addr: {t for t in reach[addr] if is_deadline_target(nodes[t])}
        for addr in nodes
    }

    def children(addr: int) -> List[int]:
        if is_anchor(nodes[addr]) or addr in cyclic:
            return []
        return graph.cut_succs(addr)

    result: Dict[int, Dict[int, _Path]] = {}
    for start in nodes:
        if start in result:
            continue
        stack = [(start, False)]
        while stack:
            addr, expanded = stack.pop()
            if addr in result:
                continue
            if not expanded:
                stack.append((addr, True))
                for child in children(addr):
                    if child not in result:
                        stack.append((child, False))
                continue
            node = nodes[addr]
            if is_deadline_target(node):
                result[addr] = {addr: (1.0, False)}
            elif node.name == "SETD":
                result[addr] = {}
            elif addr in cyclic:
                rewritten = any(_rewrites_td(nodes[t], td_csr)
                                for t in reach[addr])
                result[addr] = {t: (UNBOUNDED, rewritten) for t in targets[addr]}
            else:
                weight = _slot_cost(node)
                here = _rewrites_td(node, td_csr)
                merged: Dict[int, _Path] = {}
                for child in children(addr):
                    for target, (cost, rewritten) in result[child].items():
                        total = UNBOUNDED if weight == UNBOUNDED else cost + weight
                        flag = rewritten or here
                        if target in merged:
                            old_cost, old_flag = merged[target]
                            merged[target] = (max(old_cost, total),
                                              old_flag or flag)
                        else:
                            merged[target] = (total, flag)
                result[addr] = merged
    return result


#: The phase is not known: the SETD may read NOW at any point of a tick.
UNKNOWN_PHASE = math.inf

#: Whether a SETD pair's budget takes the SETD's phase into account (T-1).
#: Off by default until the programs that fail the sound budget are fixed
#: (docs/PLAN.md M4); ``loomasm --sound-setd`` and ``sound_setd=True`` turn
#: it on. Off, a SETD is taken to run on its tick, which is optimistic.
SOUND_SETD_DEFAULT = False


def _phase_after(node: Node, phase_in: float, tick_csrs: Set[int],
                 slot_clocks: int) -> float:
    """Clocks since the last known tick position, at the X cycle of the
    instruction that follows ``node`` on a path."""
    cost = _slot_cost(node)
    if cost == UNBOUNDED or node.name in TIMED_WAITS:
        # completes when an event or a timeout says: no fixed position
        return UNKNOWN_PHASE
    if is_deadline_target(node):
        # a WAITD k (on time, which the pair into it proves) completes in the
        # first X cycle at or after its tick: at most slot_clocks - 1 later
        return (slot_clocks - 1) + slot_clocks * cost
    if node.name == "CSRW" and node.fields.get("csr") in tick_csrs:
        # SEMANTICS 4: the write clears ACC at its commit edge, two clocks
        # after its X cycle, and the next tick is a whole period after that
        return slot_clocks * cost - 2
    return phase_in + slot_clocks * cost


def _phases(graph: _Graph, entry: Optional[int], period: int,
            tick_csrs: Set[int], slot_clocks: int) -> Dict[int, float]:
    """For every node, the most clocks its X cycle can lie after the last
    known tick position, over every path into it (a forward dataflow over the
    whole graph, uncut). The thread's entry, a node nothing jumps to, and any
    path through an unbounded instruction or a timed wait start from
    :data:`UNKNOWN_PHASE`; so does a path that has gone a whole period
    without a known position."""
    nodes = graph.nodes
    preds: Dict[int, List[int]] = {addr: [] for addr in nodes}
    for addr in nodes:
        for succ in graph.raw_succs(addr):
            preds[succ].append(addr)
    if entry is None or entry not in nodes:
        entry = min(nodes)
    phase: Dict[int, float] = {}
    work: List[int] = []
    for addr in nodes:
        if addr == entry or not preds[addr]:
            phase[addr] = UNKNOWN_PHASE
            work.append(addr)
    while work:
        addr = work.pop()
        out = _phase_after(nodes[addr], phase[addr], tick_csrs, slot_clocks)
        if out >= period:
            out = UNKNOWN_PHASE
        for succ in graph.raw_succs(addr):
            old = phase.get(succ)
            new = out if old is None else max(old, out)
            if new != old:
                phase[succ] = new
                work.append(succ)
    return phase


def analyse_thread(thread: int, nodes: Sequence[Node], period: Optional[int],
                   pc_bits: int = 10, enabled: bool = True,
                   slot_clocks: int = SLOT_CLOCKS,
                   td_csr: Optional[int] = None,
                   entry: Optional[int] = None,
                   tick_csrs: Iterable[int] = (),
                   sound_setd: bool = SOUND_SETD_DEFAULT) -> ThreadDeadlines:
    """Analyse one thread's section.

    ``td_csr`` is the number of the ``TD`` CSR from ``isa.yaml``; it lets the
    checker notice a ``CSRW TD`` that makes a ``SETD``'s own ticks meaningless.
    ``entry`` is the thread's first address and ``tick_csrs`` the numbers of
    ``TICK_INT`` and ``TICK_FRAC``, for the phase of each ``SETD``; with
    ``sound_setd`` the phase is taken off each ``SETD`` pair's budget (T-1).
    """
    report = ThreadDeadlines(thread=thread, period=period, enabled=enabled,
                             slot_clocks=slot_clocks)
    report.declarations = [
        Declaration(addr=n.addr, name=n.name, line=n.line, reason=n.bounded)
        for n in sorted(nodes, key=lambda n: n.addr) if n.bounded]
    if not enabled:
        report.notes.append("deadline check disabled by .deadline_check off")
        return report
    table = {node.addr: node for node in nodes}
    if not table:
        report.notes.append("no code in this thread")
        return report
    graph = _Graph(table, pc_bits)
    anchors = [n for n in table.values() if is_anchor(n)]
    if not anchors:
        report.notes.append("no WAITD or SETD in this thread: nothing to check")
        return report
    if period is None:
        report.notes.append(
            "tick period unknown: use .tick or a constant .csr TICK_INT "
            "to get a feasibility verdict")
    if graph.left_section:
        report.notes.append(
            "some control flow leaves this thread's section (a jump out, or "
            "falling off the end); those paths are not followed")

    solved = _longest_paths(graph, td_csr)
    phases = (_phases(graph, entry, period, set(tick_csrs), slot_clocks)
              if period is not None and sound_setd else {})
    for anchor in sorted(anchors, key=lambda n: n.addr):
        merged: Dict[int, _Path] = {}
        for succ in graph.raw_succs(anchor.addr):
            for target, (cost, rewritten) in solved[succ].items():
                if target in merged:
                    old_cost, old_flag = merged[target]
                    merged[target] = (max(old_cost, cost), old_flag or rewritten)
                else:
                    merged[target] = (cost, rewritten)
        for target in sorted(merged):
            dst = table[target]
            cost, rewritten = merged[target]
            src_phase = None
            if anchor.name == "SETD":
                # SETD m sets TD = NOW + m, so its own m is part of the budget,
                # unless a CSRW TD on the path replaced the deadline it set.
                src_ticks = None if rewritten else int(anchor.fields.get("imm", 0))
                if period is not None and sound_setd:
                    known = phases.get(anchor.addr, UNKNOWN_PHASE)
                    src_phase = int(min(known, period - 1))
            else:
                src_ticks = 0               # WAITD anchors add nothing of their own
            report.pairs.append(DeadlinePair(
                thread=thread, src_addr=anchor.addr, src_name=anchor.name,
                src_line=anchor.line, dst_addr=target, dst_line=dst.line,
                ticks=int(dst.fields.get("imm", 0)), slots=cost,
                period=period, slot_clocks=slot_clocks, src_ticks=src_ticks,
                src_phase=src_phase))
    if any(not p.anchor_credit_known for p in report.pairs):
        report.notes.append(
            "a CSRW TD lies between a SETD and a WAITD on some path, so that "
            "SETD's own ticks are not credited to the budget (pessimistic)")
    return report


def _budget_text(pair: DeadlinePair) -> str:
    """``k x P``, or with a SETD's phase ``k x P - phase + grace``."""
    text = "%d x %d" % (pair.budget_ticks, pair.period)
    if pair.src_phase is not None:
        text += " - %d (SETD phase) + %d" % (pair.src_phase,
                                            pair.slot_clocks - 1)
    return text


def diagnostics_for(report: ThreadDeadlines, filename: str) -> List[Diagnostic]:
    """Turn a thread report into warnings (unbounded) and errors (infeasible)."""
    out: List[Diagnostic] = []
    for pair in report.pairs:
        where = "thread %d: 0x%03X %s -> 0x%03X WAITD %d" % (
            pair.thread, pair.src_addr, pair.src_name, pair.dst_addr, pair.ticks)
        if pair.unbounded:
            out.append(Diagnostic(
                WARNING, DEADLINE, filename, pair.dst_line,
                "unbounded between deadlines (%s): a loop or an untimed "
                "wait/blocking instruction lies on the path; anchor it with "
                "SETD after that instruction" % where))
        elif pair.feasible is False:
            out.append(Diagnostic(
                ERROR, DEADLINE, filename, pair.dst_line,
                "deadline cannot be met (%s): %d slots = %d clocks, budget "
                "%s = %d clocks, short by %d" % (
                    where, int(pair.slots), pair.clocks, _budget_text(pair),
                    pair.limit, -pair.slack)))
    return out


def summary_lines(report: ThreadDeadlines) -> List[str]:
    """Human-readable block for the listing."""
    period = ("%d clocks" % report.period) if report.period is not None else "unknown"
    lines = ["thread %d deadline analysis (tick period %s)" % (report.thread, period)]
    for note in report.notes:
        lines.append("  note: %s" % note)
    for declared in report.declarations:
        lines.append("  bounded by declaration: %s" % declared)
    if not report.pairs:
        if report.enabled and not report.notes:
            lines.append("  no deadline pairs found")
        return lines
    lines.append("  %-22s %-22s %6s %8s %6s %9s %9s" % (
        "from", "to", "slots", "clocks", "ticks", "budget", "slack"))
    for pair in sorted(report.pairs, key=lambda p: (p.src_addr, p.dst_addr)):
        src = "0x%03X %s" % (pair.src_addr, pair.src_name)
        if pair.src_name == "SETD":
            src += " %d" % (pair.src_ticks if pair.anchor_credit_known else 0)
        dst = "0x%03X WAITD %d" % (pair.dst_addr, pair.ticks)
        if pair.unbounded:
            lines.append("  %-22s %-22s %6s %8s %6d %9s %9s" % (
                src, dst, "inf", "-", pair.budget_ticks, "-", "UNBOUNDED"))
            continue
        budget = "-" if pair.limit is None else str(pair.limit)
        slack = "-" if pair.slack is None else str(pair.slack)
        mark = "" if pair.feasible is not False else "  MISSED"
        if pair.src_phase is not None:
            mark += "  (SETD phase <= %d)" % pair.src_phase
        if not pair.anchor_credit_known:
            mark += "  (TD rewritten: SETD ticks not credited)"
        lines.append("  %-22s %-22s %6d %8d %6d %9s %9s%s" % (
            src, dst, int(pair.slots), pair.clocks, pair.budget_ticks,
            budget, slack, mark))
    worst = report.worst_slack
    tail = "  %d deadline pair%s, %d unbounded, %d infeasible" % (
        len(report.pairs), "" if len(report.pairs) == 1 else "s",
        len(report.unbounded), len(report.infeasible))
    if worst is not None:
        tail += ", worst slack %d clocks" % worst
    elif report.worst_slots is not None:
        tail += ", longest path %d slots" % report.worst_slots
    if report.declarations:
        tail += ", %d bounded by declaration" % len(report.declarations)
    lines.append(tail)
    return lines


def all_summary_lines(reports: Iterable[ThreadDeadlines]) -> List[str]:
    lines: List[str] = []
    for report in reports:
        lines.extend(summary_lines(report))
        lines.append("")
    return lines
