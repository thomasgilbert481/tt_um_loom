"""
Command line for the L7 mutation tool (`docs/VERIFICATION.md`, MUT-RUN).

    python -m tools.mutate list                       # what would be run
    python -m tools.mutate run  --jobs 8 --out r.jsonl
    python -m tools.mutate report r.jsonl             # the L7 tables
    python -m tools.mutate show <mutant id>           # one mutant's diff

`run` must be started from a shell that has sourced `scripts/dev_env.sh`, so
Verilator, Icarus and cocotb are on PATH. It never writes to `src/`.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import sys
import time
from typing import List, Optional, Sequence

from tools.mutate.operators import (
    OPERATORS,
    Mutation,
    iter_operator_counts,
    mutations_for_file,
)
from tools.mutate.report import full_report, load_equivalents, survivor_rows
from tools.mutate.runner import (
    REPO,
    TARGET_MODULES,
    Config,
    load_results,
    run_all,
)


def collect(
    modules: Sequence[str],
    operators: Optional[Sequence[str]],
    defines: Sequence[str],
    sample: int = 0,
    seed: int = 0,
    core_sample: int = 0,
) -> List[Mutation]:
    """Every mutant of the named modules, optionally thinned to a sample.

    Sampling is stratified by module and operator and driven by `seed`, so the
    same `--sample` always picks the same mutants.
    """
    out: List[Mutation] = []
    for name in modules:
        path = REPO / "src" / (name + ".v")
        if not path.exists():
            raise SystemExit("no such module: %s" % path)
        muts = mutations_for_file(
            str(path), rel="src/%s.v" % name, defines=defines, operators=operators
        )
        limit = core_sample if (core_sample and name == "loom_core") else sample
        if limit and len(muts) > limit:
            muts = _stratified(muts, limit, seed)
        out.extend(muts)
    return out


def _stratified(muts: Sequence[Mutation], limit: int, seed: int) -> List[Mutation]:
    """Thin to about `limit` mutants, keeping every operator in proportion.

    The order inside an operator is a hash of the mutant and the seed, not a
    shuffle, so raising `--sample` only ever adds mutants. A run can therefore
    be deepened with `--resume` instead of started again.
    """
    groups: dict = {}
    for m in muts:
        groups.setdefault(m.operator, []).append(m)
    kept: List[Mutation] = []
    for op in sorted(groups):
        share = max(1, round(limit * len(groups[op]) / len(muts)))
        pool = sorted(
            groups[op],
            key=lambda m: hashlib.sha1(("%d:%s" % (seed, m.digest)).encode()).digest(),
        )
        kept.extend(pool[:share])
    kept.sort(key=lambda m: (m.path, m.line, m.col, m.operator, m.digest))
    return kept


def _add_selection_args(p: argparse.ArgumentParser) -> None:
    p.add_argument(
        "--modules",
        default=",".join(TARGET_MODULES),
        help="comma-separated source module names (default: the six of MUT-TARGET)",
    )
    p.add_argument(
        "--operators",
        default="",
        help="comma-separated subset of: " + ", ".join(OPERATORS),
    )
    p.add_argument(
        "--define", action="append", default=[], help="a macro to treat as defined"
    )
    p.add_argument(
        "--sample", type=int, default=0, help="at most N mutants per module (0: all)"
    )
    p.add_argument(
        "--core-sample",
        type=int,
        default=0,
        help="a different cap for loom_core, which has by far the most mutants",
    )
    p.add_argument("--seed", type=int, default=1, help="sampling seed")
    p.add_argument(
        "--only",
        default="",
        help="comma-separated mutant ids (or id prefixes) to keep, for "
        "re-running one survivor",
    )


def _selection(args) -> List[Mutation]:
    muts = collect(
        [m.strip() for m in args.modules.split(",") if m.strip()],
        [o.strip() for o in args.operators.split(",") if o.strip()] or None,
        args.define,
        sample=args.sample,
        seed=args.seed,
        core_sample=args.core_sample,
    )
    only = [o.strip() for o in getattr(args, "only", "").split(",") if o.strip()]
    if only:
        muts = [m for m in muts if any(m.ident.startswith(o) for o in only)]
        if not muts:
            raise SystemExit("--only matched no mutant in the selection")
    return muts


def cmd_list(args) -> int:
    muts = _selection(args)
    if args.json:
        json.dump([m.as_dict() for m in muts], sys.stdout, indent=1)
        print()
        return 0
    for m in muts:
        print("%-44s %-9s %s" % (m.ident, m.operator, m.description))
    print()
    counts = iter_operator_counts(muts)
    print("%d mutants: %s" % (len(muts), " ".join("%s=%d" % kv for kv in counts.items() if kv[1])))
    return 0


def cmd_run(args) -> int:
    muts = _selection(args)
    cfg = Config(jobs=args.jobs)
    if args.work:
        cfg.work = pathlib.Path(args.work)
    if args.build:
        cfg.build = pathlib.Path(args.build)
    if args.rungs:
        cfg.rungs = tuple(r.strip() for r in args.rungs.split(",") if r.strip())
    out_path = pathlib.Path(args.out) if args.out else None
    done = set()
    if out_path and args.resume:
        done = {r.mutation.ident for r in load_results(out_path)}
        print("resuming: %d of %d already done" % (len(done), len(muts)))
    elif out_path and out_path.exists():
        out_path.unlink()

    if not args.no_baseline:
        from tools.mutate.runner import Workspace, baseline_ok

        print("baseline: running the ladder on the unmutated source ...", flush=True)
        ws = Workspace(cfg)
        ws.build()
        ok, checks = baseline_ok(cfg, ws)
        for c in checks:
            print("  %-12s %-4s %6.1f s %s" % (c.name, "ok" if c.passed else "FAIL", c.seconds, c.detail))
        if not ok:
            print("baseline failed: the tree is not green, so mutants mean nothing")
            return 2

    started = time.time()
    width = len(str(len(muts)))

    def progress(n, total, res):
        m = res.mutation
        tag = res.killed_by or ("SURVIVED" if res.status == "survived" else res.status)
        print(
            "[%*d/%d] %6.1fs %-44s %-12s %s"
            % (width, n, total, res.seconds, m.ident, tag, res.detail[:60]),
            flush=True,
        )

    results = run_all(muts, cfg, out_path, done=done, progress=progress)
    if out_path and args.resume:
        results = load_results(out_path)
    wall = time.time() - started
    print()
    print(full_report(results, wall=wall, jobs=cfg.jobs))
    equivalents = load_equivalents()
    open_survivors = [
        r for r in survivor_rows(results) if r.mutation.ident not in equivalents
    ]
    if open_survivors:
        print(
            "%d survivor(s) are not in tools/mutate/equivalents.json; each one "
            "wants a documented reason or a new check." % len(open_survivors)
        )
    return 1 if open_survivors and args.strict else 0


def cmd_report(args) -> int:
    results = load_results(pathlib.Path(args.results))
    if not results:
        raise SystemExit("no results in %s" % args.results)
    text = full_report(results)
    if args.out:
        pathlib.Path(args.out).write_text(text, encoding="utf-8", newline="\n")
        print("wrote %s" % args.out)
    else:
        print(text)
    return 0


def cmd_show(args) -> int:
    muts = _selection(args)
    hits = [m for m in muts if m.ident == args.ident or args.ident in m.ident]
    if not hits:
        print("no mutant matching %r in the current selection" % args.ident)
        return 1
    for m in hits:
        print("%s  (%s)" % (m.ident, m.description))
        print("%s:%d:%d" % (m.path, m.line, m.col))
        lines = (REPO / m.path).read_text(encoding="utf-8").replace("\r\n", "\n").split("\n")
        lo = max(0, m.line - 1 - args.context)
        hi = min(len(lines), m.line + args.context)
        for i in range(lo, hi):
            if i == m.line - 1:
                print("-%5d %s" % (i + 1, m.original))
                print("+%5d %s" % (i + 1, m.mutated))
            else:
                print(" %5d %s" % (i + 1, lines[i]))
        print()
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    p = argparse.ArgumentParser(prog="python -m tools.mutate", description=__doc__)
    sub = p.add_subparsers(dest="cmd", required=True)

    lp = sub.add_parser("list", help="list the mutants that would be run")
    _add_selection_args(lp)
    lp.add_argument("--json", action="store_true")
    lp.set_defaults(func=cmd_list)

    rp = sub.add_parser("run", help="build and check every mutant")
    _add_selection_args(rp)
    rp.add_argument("--jobs", type=int, default=4)
    rp.add_argument("--out", default="", help="JSON-lines results file")
    rp.add_argument("--resume", action="store_true", help="skip mutants already in --out")
    rp.add_argument("--work", default="", help="work directory (default ~/loom_mutate_work)")
    rp.add_argument("--build", default="", help="sim_build root (default ~/loom_mutate_build)")
    rp.add_argument("--rungs", default="", help="comma-separated subset of the ladder")
    rp.add_argument("--no-baseline", action="store_true")
    rp.add_argument(
        "--strict",
        action="store_true",
        help="exit 1 if a survivor is not documented in equivalents.json",
    )
    rp.set_defaults(func=cmd_run)

    pp = sub.add_parser("report", help="print the L7 tables from a results file")
    pp.add_argument("results")
    pp.add_argument("--out", default="")
    pp.set_defaults(func=cmd_report)

    sp = sub.add_parser("show", help="print one mutant's one-line diff in context")
    _add_selection_args(sp)
    sp.add_argument("ident", help="a mutant id, or enough of one to match")
    sp.add_argument("--context", type=int, default=4, help="lines of context")
    sp.set_defaults(func=cmd_show)

    args = p.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
