"""Carry documented equivalents across edits that only moved their lines.

A mutant id holds the line number and a digest of (path, line, mutated
line), so any edit above a line gives every mutant below it a new id, and
`equivalents.json` stops matching even where the mutated line itself is
untouched. `rekey` finds, for each documented id, the mutant the current
source generates from the *same* original and mutated text (same file, same
operator) and moves the reason to that mutant's id.

Only exact text matches are carried. A line whose text changed, or that is
gone, drops out and is reported, so its claim has to be made again: that is
the rule `report.EQUIVALENTS_PATH` states. A carried reason still has to be
re-read at the next pass, because code around an unchanged line can change
what the line does.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from tools.mutate.operators import Mutation

#: How a documented id fared: carried with the same id, carried to a new id,
#: carried but chosen among several identical lines, not generated any more,
#: or never seen in the results given (so nothing is known about it).
STATUSES = ("same", "moved", "ambiguous", "gone", "unknown")


@dataclass(frozen=True)
class Rekeyed:
    old_id: str
    new_id: Optional[str]
    status: str
    note: str = ""


def _key(m: Mutation) -> Tuple[str, str, str, str]:
    return (m.path.replace("\\", "/"), m.operator, m.original, m.mutated)


def rekey(
    equivalents: Dict[str, str],
    records: Dict[str, Mutation],
    current: Sequence[Mutation],
) -> List[Rekeyed]:
    """One entry per documented id, in the order of `equivalents`.

    `records` maps old ids to the mutation they named (from a results file
    of the run that documented them); `current` is what the source generates
    now. Among several current mutants with the same text (a line repeated
    per thread, say), the one nearest the old line number wins and the entry
    is marked ambiguous. Two old ids that land on one new id are both
    reported as ambiguous with the clash in the note.
    """
    index: Dict[Tuple[str, str, str, str], List[Mutation]] = {}
    for m in current:
        index.setdefault(_key(m), []).append(m)

    out: List[Rekeyed] = []
    for old_id in equivalents:
        old = records.get(old_id)
        if old is None:
            out.append(Rekeyed(old_id, None, "unknown", "not in the results given"))
            continue
        hits = index.get(_key(old), [])
        if not hits:
            note = "%s:%d: line changed or removed" % (old.path, old.line)
            out.append(Rekeyed(old_id, None, "gone", note))
            continue
        best = min(hits, key=lambda m: (abs(m.line - old.line), m.line))
        if len(hits) > 1:
            note = "%d identical lines; nearest to old line %d" % (len(hits), old.line)
            out.append(Rekeyed(old_id, best.ident, "ambiguous", note))
        elif best.ident == old_id:
            out.append(Rekeyed(old_id, best.ident, "same"))
        else:
            note = "line %d -> %d" % (old.line, best.line)
            out.append(Rekeyed(old_id, best.ident, "moved", note))

    seen: Dict[str, str] = {}
    clashes = set()
    for r in out:
        if r.new_id is None:
            continue
        if r.new_id in seen:
            clashes.update((r.old_id, seen[r.new_id]))
        seen.setdefault(r.new_id, r.old_id)
    if clashes:
        clash = "shares its new id with another entry"
        out = [
            Rekeyed(r.old_id, r.new_id, "ambiguous", (r.note + "; " if r.note else "") + clash)
            if r.old_id in clashes
            else r
            for r in out
        ]
    return out


def carried(equivalents: Dict[str, str], result: Iterable[Rekeyed]) -> Dict[str, str]:
    """The new `equivalents.json` content: carried entries only, same reasons."""
    return {r.new_id: equivalents[r.old_id] for r in result if r.new_id is not None}


def summary(result: Sequence[Rekeyed]) -> str:
    counts = {s: 0 for s in STATUSES}
    for r in result:
        counts[r.status] += 1
    return ", ".join("%d %s" % (counts[s], s) for s in STATUSES if counts[s])
