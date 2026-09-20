"""
tools.mutate.report: turn a list of `MutantResult` into the tables L7 wants.

Three of them:

  * kill rate per module, with the MUT-TARGET 90 per cent verdict;
  * which check did the killing, so a reader can see how much of the rate is
    the linter catching a mutant that never reached a simulation;
  * the survivors, one line each, because they are the output that matters.
"""

from __future__ import annotations

import json
import pathlib
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from tools.mutate.operators import OPERATORS
from tools.mutate.runner import MutantResult

TARGET_RATE = 90.0

#: Survivors that have been read and found to change nothing observable.
#: MUT-TARGET allows a survivor only if it is an equivalent mutant *and* the
#: reason is written down, which is what this file is. The key is the mutant
#: id, whose digest covers the mutated line, so an entry stops applying the
#: moment that line of RTL changes and the claim has to be made again.
EQUIVALENTS_PATH = pathlib.Path(__file__).resolve().parent / "equivalents.json"


def load_equivalents(path: Optional[pathlib.Path] = None) -> Dict[str, str]:
    path = path or EQUIVALENTS_PATH
    if not path.exists():
        return {}
    with open(path, "r", encoding="utf-8") as fh:
        return dict(json.load(fh))


def _table(rows: Sequence[Sequence[str]], header: Sequence[str]) -> str:
    out = ["| " + " | ".join(header) + " |", "|" + "|".join(["---"] * len(header)) + "|"]
    for r in rows:
        out.append("| " + " | ".join(r) + " |")
    return "\n".join(out)


def kill_rates(results: Iterable[MutantResult]) -> Dict[str, Tuple[int, int, int]]:
    """module -> (killed, survived, error)."""
    acc: Dict[str, List[int]] = {}
    for r in results:
        slot = acc.setdefault(r.mutation.module, [0, 0, 0])
        slot[{"killed": 0, "survived": 1}.get(r.status, 2)] += 1
    return {k: (v[0], v[1], v[2]) for k, v in sorted(acc.items())}


def dynamic_rates(results: Iterable[MutantResult]) -> Dict[str, Tuple[int, int]]:
    """module -> (mutants that got past the linter, of those, killed in a sim).

    The linter is a real gate (CI's `lint` job runs `-Wall` and fails on a
    warning), so a lint kill counts. It is still worth separating: a mutant
    that never compiled says nothing about the tests.
    """
    acc: Dict[str, List[int]] = {}
    for r in results:
        if r.killed_by == "lint":
            continue
        slot = acc.setdefault(r.mutation.module, [0, 0])
        slot[0] += 1
        slot[1] += r.status == "killed"
    return {k: (v[0], v[1]) for k, v in sorted(acc.items())}


def equivalent_counts(
    results: Iterable[MutantResult], equivalents: Dict[str, str]
) -> Dict[str, int]:
    acc: Dict[str, int] = {}
    for r in results:
        if r.status != "killed" and r.mutation.ident in equivalents:
            acc[r.mutation.module] = acc.get(r.mutation.module, 0) + 1
    return acc


def by_module_table(
    results: Sequence[MutantResult], equivalents: Optional[Dict[str, str]] = None
) -> str:
    equivalents = load_equivalents() if equivalents is None else equivalents
    eq = equivalent_counts(results, equivalents)
    rows = []
    tk = ts = te = 0
    for module, (killed, survived, errors) in kill_rates(results).items():
        total = killed + survived + errors
        rate = 100.0 * killed / total if total else 0.0
        n_eq = eq.get(module, 0)
        adj_total = total - n_eq
        adj = 100.0 * killed / adj_total if adj_total else 0.0
        rows.append(
            [
                "`%s.v`" % module,
                str(total),
                str(killed),
                str(survived + errors),
                str(n_eq),
                "%.1f %%" % rate,
                "%.1f %%" % adj,
                "yes" if adj >= TARGET_RATE else "**no**",
            ]
        )
        tk += killed
        ts += survived
        te += errors
    total = tk + ts + te
    teq = sum(eq.values())
    adj_total = total - teq
    rows.append(
        [
            "**all**",
            "**%d**" % total,
            "**%d**" % tk,
            "**%d**" % (ts + te),
            "**%d**" % teq,
            "**%.1f %%**" % (100.0 * tk / total if total else 0.0),
            "**%.1f %%**" % (100.0 * tk / adj_total if adj_total else 0.0),
            "",
        ]
    )
    return _table(
        rows,
        [
            "Module",
            "Mutants",
            "Killed",
            "Survived",
            "Equivalent",
            "Kill rate",
            "Less equivalents",
            "MUT-TARGET",
        ],
    )


def by_operator_table(results: Sequence[MutantResult]) -> str:
    acc: Dict[str, List[int]] = {}
    for r in results:
        slot = acc.setdefault(r.mutation.operator, [0, 0])
        slot[0] += 1
        slot[1] += r.status == "killed"
    rows = []
    for op in OPERATORS:
        if op not in acc:
            continue
        total, killed = acc[op]
        rows.append(
            ["`%s`" % op, str(total), str(killed), "%.1f %%" % (100.0 * killed / total)]
        )
    return _table(rows, ["Operator", "Mutants", "Killed", "Kill rate"])


def by_check_table(results: Sequence[MutantResult]) -> str:
    """Which rung killed how many, and how much wall time each rung cost."""
    counts: Dict[str, int] = {}
    secs: Dict[str, float] = {}
    for r in results:
        if r.status == "killed" and r.killed_by:
            counts[r.killed_by] = counts.get(r.killed_by, 0) + 1
        for c in r.checks:
            secs[c.name] = secs.get(c.name, 0.0) + c.seconds
    order = sorted(secs, key=lambda n: -counts.get(n, 0))
    rows = [
        [
            "`%s`" % name,
            str(counts.get(name, 0)),
            "%.0f s" % secs.get(name, 0.0),
        ]
        for name in order
    ]
    return _table(rows, ["First check that failed", "Mutants killed", "Total wall time"])


def survivor_rows(results: Sequence[MutantResult]) -> List[MutantResult]:
    out = [r for r in results if r.status != "killed"]
    out.sort(key=lambda r: (r.mutation.path, r.mutation.line, r.mutation.col))
    return out


def survivor_table(
    results: Sequence[MutantResult], equivalents: Optional[Dict[str, str]] = None
) -> str:
    equivalents = load_equivalents() if equivalents is None else equivalents
    rows = []
    for r in survivor_rows(results):
        m = r.mutation
        reason = equivalents.get(m.ident, "")
        rows.append(
            [
                "`%s`" % m.ident,
                "`%s.v`:%d" % (m.module, m.line),
                "`%s`" % m.operator,
                m.description.replace("|", "\\|"),
                "`%s`" % m.mutated.strip().replace("|", "\\|")[:70],
                ("equivalent - " + reason) if reason else "**open**",
            ]
        )
    if not rows:
        return "No survivors."
    return _table(
        rows, ["Mutant", "Line", "Operator", "Change", "Mutated line", "Verdict"]
    )


def timing_summary(results: Sequence[MutantResult]) -> str:
    total_cpu = sum(r.seconds for r in results)
    killed = [r for r in results if r.status == "killed"]
    survived = [r for r in results if r.status != "killed"]
    parts = [
        "%d mutants, %.0f s of check time in total" % (len(results), total_cpu),
        "killed: %d, mean %.1f s each"
        % (len(killed), (sum(r.seconds for r in killed) / len(killed)) if killed else 0.0),
        "survived: %d, mean %.0f s each"
        % (
            len(survived),
            (sum(r.seconds for r in survived) / len(survived)) if survived else 0.0,
        ),
    ]
    return "; ".join(parts)


def full_report(
    results: Sequence[MutantResult],
    wall: float = 0.0,
    jobs: int = 0,
    equivalents: Optional[Dict[str, str]] = None,
) -> str:
    equivalents = load_equivalents() if equivalents is None else equivalents
    out = ["## Mutation run", ""]
    if wall:
        out.append(
            "%.0f s wall (%.1f min) at %d jobs. %s."
            % (wall, wall / 60.0, jobs, timing_summary(results))
        )
    else:
        out.append(timing_summary(results) + ".")
    dyn = dynamic_rates(results)
    d_total = sum(v[0] for v in dyn.values())
    d_killed = sum(v[1] for v in dyn.values())
    out.append("")
    out.append(
        "Of the %d mutants the linter did not catch, %d (%.1f per cent) were "
        "killed in simulation."
        % (d_total, d_killed, 100.0 * d_killed / d_total if d_total else 0.0)
    )
    out += ["", "### Kill rate per module", "", by_module_table(results, equivalents)]
    out += ["", "### Kill rate per operator", "", by_operator_table(results)]
    out += ["", "### Where mutants died", "", by_check_table(results)]
    out += ["", "### Survivors", "", survivor_table(results, equivalents)]
    return "\n".join(out) + "\n"
