"""
tools.mutate.runner: build each mutant and run the check ladder on it.

A mutant is never applied to `src/`. Each one gets a private work directory

    <work>/<mutant id>/
        src/      a copy of the real src/ with one line changed
        test/     a copy of test/ (own results.xml, coverage, sim_build)
        macro tools isa firmware docs -> symlinks back to the repo

built by hard-linking a template that is made once per run, so preparing a
mutant costs a few milliseconds. `make` runs in that directory with
`SRC_DIR=<work>/<id>/src`, which is the trick the hand-written mutants of
`test/README.md` use.

The ladder is cheapest-first and stops at the first check that fails: a mutant
is killed by that check, and the checks after it are never run. The default
rungs are

    1. lint         verilator --lint-only -Wall            ~0.3 s
    2. cosim-short  test_cosim, 3 seeds x 2500 cycles      ~5 s
    3. unit (own)   the L1 modules closest to this file    seconds to a minute
    4. cosim-deep   6 seeds x 4000 cycles + one SPI seed   ~25 s
    5. unit (rest)  the remaining L1 modules               ~4 minutes

so a mutant that dies early costs seconds and only a survivor pays for the
whole suite. That is L1 + L2 of `docs/VERIFICATION.md`, which is what MUT-RUN
asks for; the L3 firmware suite and the FLOPS variant are not in the ladder.

Results are appended to a JSON-lines file as they finish, so a run that is
interrupted can be resumed with `--resume`.
"""

from __future__ import annotations

import json
import os
import pathlib
import re
import shlex
import shutil
import signal
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from tools.mutate.operators import Mutation

REPO = pathlib.Path(__file__).resolve().parents[2]

# The L1 unit modules of VERIFICATION.md, cheapest first (measured on the
# unmutated design, Icarus, 2026-09-19). test_fw (L3) and test_flops (the
# FLOPS memory variant) are deliberately not here.
L1_MODULES: Tuple[str, ...] = (
    "test_irq",
    "test_timing",
    "test_uart",
    "test_host",
    "test_pins",
    "test_setpd",
    "test_ctrl",
    "test_be",
    "test_fifo",
    "test_alu",
)

# Which L1 modules test a given source file most directly. They are promoted
# ahead of the rest of the suite so a local fault dies on a local test.
AFFINITY: Dict[str, Tuple[str, ...]] = {
    "loom_fifo": ("test_fifo",),
    "loom_be": ("test_be",),
    "loom_pins": ("test_pins", "test_timing"),
    "loom_timer": ("test_timing", "test_setpd"),
    "loom_spi_host": ("test_host",),
    "loom_core": ("test_irq", "test_ctrl"),
    "loom_alu": ("test_alu",),
    "loom_host_ctl": ("test_host",),
    "loom_regfile": ("test_alu",),
    "loom_imem": ("test_host",),
    "loom_top": ("test_host",),
    "loom_sync": ("test_host",),
    "tt_um_loom": ("test_host",),
}

# The six modules MUT-TARGET names.
TARGET_MODULES: Tuple[str, ...] = (
    "loom_core",
    "loom_timer",
    "loom_pins",
    "loom_be",
    "loom_fifo",
    "loom_spi_host",
)

# test_cosim.py's `fail()`:
#   "co-simulation divergence (<kind>): seed <n>, <label>, cycle <c>"
# The label carries the build description and has commas of its own
# ("backdoor profile=m2 threads=4 caps=909A 512 words, BE, FIFO, SETPD depth
# 4"), so it has to be matched lazily up to the *first* ", cycle".
_DIVERGENCE_RE = re.compile(
    r"co-simulation divergence \(([^)]*)\): seed (\d+), (.*?), cycle (\d+)"
)
_VERILATOR_ERR_RE = re.compile(r"^%(Error|Warning)[^:]*:.*$", re.M)


# --------------------------------------------------------------------------
# Configuration
# --------------------------------------------------------------------------


@dataclass
class Config:
    repo: pathlib.Path = REPO
    work: pathlib.Path = pathlib.Path(
        os.environ.get("LOOM_MUTATE_WORK", os.path.expanduser("~/loom_mutate_work"))
    )
    build: pathlib.Path = pathlib.Path(
        os.environ.get("LOOM_MUTATE_BUILD", os.path.expanduser("~/loom_mutate_build"))
    )
    jobs: int = 4
    #: seconds; a rung that outruns this counts as killed by timeout
    lint_timeout: float = 120.0
    cosim_timeout: float = 600.0
    unit_timeout: float = 900.0
    #: rungs to run at all (a subset of "lint", "cosim-short", "unit-own",
    #: "cosim-deep", "unit-rest")
    rungs: Tuple[str, ...] = (
        "lint",
        "cosim-short",
        "unit-own",
        "cosim-deep",
        "unit-rest",
    )
    cosim_short: Dict[str, str] = field(
        default_factory=lambda: {
            "LOOM_COSIM_SEEDS": "3",
            "LOOM_COSIM_CYCLES": "2500",
            "LOOM_COSIM_SPI_SEEDS": "0",
            "LOOM_COSIM_STATE_EVERY": "400",
        }
    )
    cosim_deep: Dict[str, str] = field(
        default_factory=lambda: {
            "LOOM_COSIM_SEEDS": "6",
            "LOOM_COSIM_CYCLES": "4000",
            "LOOM_COSIM_SPI_SEEDS": "1",
            "LOOM_COSIM_SPI_CYCLES": "8000",
            "LOOM_COSIM_STATE_EVERY": "400",
        }
    )
    #: keep the work directory of survivors for inspection
    keep_survivors: bool = True
    verbose: bool = False


# --------------------------------------------------------------------------
# Results
# --------------------------------------------------------------------------


@dataclass
class CheckRun:
    name: str
    passed: bool
    seconds: float
    detail: str = ""
    #: set when a co-simulation rung failed, so the report can say where
    seed: Optional[int] = None
    cycle: Optional[int] = None


@dataclass
class MutantResult:
    mutation: Mutation
    status: str  # "killed" | "survived" | "error"
    killed_by: Optional[str]
    detail: str
    seconds: float
    checks: List[CheckRun]
    seed: Optional[int] = None
    cycle: Optional[int] = None

    def as_dict(self) -> Dict[str, object]:
        return {
            "mutation": self.mutation.as_dict(),
            "status": self.status,
            "killed_by": self.killed_by,
            "detail": self.detail,
            "seconds": round(self.seconds, 2),
            "seed": self.seed,
            "cycle": self.cycle,
            "checks": [
                {
                    "name": c.name,
                    "passed": c.passed,
                    "seconds": round(c.seconds, 2),
                    "detail": c.detail,
                    "seed": c.seed,
                    "cycle": c.cycle,
                }
                for c in self.checks
            ],
        }

    @staticmethod
    def from_dict(d: Dict[str, object]) -> "MutantResult":
        return MutantResult(
            mutation=Mutation.from_dict(d["mutation"]),  # type: ignore[arg-type]
            status=str(d["status"]),
            killed_by=d.get("killed_by"),  # type: ignore[arg-type]
            detail=str(d.get("detail", "")),
            seconds=float(d.get("seconds", 0.0)),
            checks=[
                CheckRun(
                    name=str(c["name"]),
                    passed=bool(c["passed"]),
                    seconds=float(c.get("seconds", 0.0)),
                    detail=str(c.get("detail", "")),
                    seed=c.get("seed"),
                    cycle=c.get("cycle"),
                )
                for c in d.get("checks", [])  # type: ignore[union-attr]
            ],
            seed=d.get("seed"),  # type: ignore[arg-type]
            cycle=d.get("cycle"),  # type: ignore[arg-type]
        )


# --------------------------------------------------------------------------
# Shell
# --------------------------------------------------------------------------


def _run(cmd: str, cwd: pathlib.Path, env: Dict[str, str], timeout: float):
    """Run `cmd` in bash. Returns (returncode, output, timed_out, seconds)."""
    start = time.time()
    full = "cd %s && %s" % (shlex.quote(str(cwd)), cmd)
    try:
        proc = subprocess.Popen(
            ["bash", "-c", full],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=env,
            start_new_session=True,
            text=True,
            errors="replace",
        )
    except OSError as exc:  # pragma: no cover - environment failure
        return 127, str(exc), False, time.time() - start
    try:
        out, _ = proc.communicate(timeout=timeout)
        return proc.returncode, out, False, time.time() - start
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except OSError:
            pass
        out, _ = proc.communicate()
        return -9, out or "", True, time.time() - start


# --------------------------------------------------------------------------
# Work directories
# --------------------------------------------------------------------------


class Workspace:
    """The template every mutant directory is hard-linked from."""

    LINKS = ("macro", "tools", "isa", "firmware", "docs", "scripts")

    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.root = cfg.work / "_template"

    def build(self) -> None:
        cfg = self.cfg
        cfg.work.mkdir(parents=True, exist_ok=True)
        cfg.build.mkdir(parents=True, exist_ok=True)
        if self.root.exists():
            shutil.rmtree(self.root)
        self.root.mkdir(parents=True)
        shutil.copytree(cfg.repo / "src", self.root / "src")
        shutil.copytree(
            cfg.repo / "test",
            self.root / "test",
            ignore=shutil.ignore_patterns("sim_build", "__pycache__", "*.fst", "*.vcd"),
        )
        for name in self.LINKS:
            target = cfg.repo / name
            if not target.exists():
                continue
            try:
                os.symlink(target, self.root / name)
            except OSError:  # Windows without developer mode
                shutil.copytree(target, self.root / name)
        # Byte-compile once so 2 500 mutants do not each recompile test_cosim.
        subprocess.run(
            [sys.executable, "-m", "compileall", "-q", str(self.root / "test")],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )

    def checkout(self, mut: Optional[Mutation], ident: str) -> pathlib.Path:
        """A fresh directory holding the mutant (or the original for `None`)."""
        dest = self.cfg.work / ident
        if dest.exists():
            shutil.rmtree(dest)
        # -a keeps symlinks as symlinks, -l hard-links the regular files, so a
        # checkout costs milliseconds instead of copying the tree 2 500 times.
        # Every file the run then writes is a new file, never a write through
        # the link, so the template (and `src/`) cannot be reached this way.
        try:
            rc = subprocess.run(
                ["cp", "-al", str(self.root), str(dest)],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.STDOUT,
            ).returncode
        except OSError:
            rc = 1
        if rc != 0:  # no POSIX cp, or a filesystem without hard links
            shutil.copytree(self.root, dest, symlinks=True)
        if mut is not None:
            # `dest / <absolute path>` is that absolute path, which would put
            # the mutant straight into the real src/. A mutant's path is always
            # repo-relative; refuse anything else rather than trust the caller.
            rel = pathlib.PurePosixPath(mut.path.replace("\\", "/"))
            if (
                rel.is_absolute()
                or pathlib.PureWindowsPath(mut.path).is_absolute()
                or ".." in rel.parts
            ):
                raise ValueError("mutation path must be repo-relative: %r" % mut.path)
            path = dest / rel
            text = path.read_text(encoding="utf-8")
            new = mut.apply(text.replace("\r\n", "\n"))
            # Unlink first: the file is a hard link to the template.
            path.unlink()
            path.write_text(new, encoding="utf-8", newline="\n")
        return dest


# --------------------------------------------------------------------------
# The ladder
# --------------------------------------------------------------------------


def ladder_for(module: str, cfg: Config) -> List[Tuple[str, str, Dict[str, str], float]]:
    """(check name, kind, extra env, timeout) in the order they are tried."""
    own = [m for m in AFFINITY.get(module, ()) if m in L1_MODULES]
    rest = [m for m in L1_MODULES if m not in own]
    steps: List[Tuple[str, str, Dict[str, str], float]] = []
    if "lint" in cfg.rungs:
        steps.append(("lint", "lint", {}, cfg.lint_timeout))
    if "cosim-short" in cfg.rungs:
        steps.append(("cosim-short", "test_cosim", dict(cfg.cosim_short), cfg.cosim_timeout))
    if "unit-own" in cfg.rungs:
        for m in own:
            steps.append((m, m, {}, cfg.unit_timeout))
    if "cosim-deep" in cfg.rungs:
        steps.append(("cosim-deep", "test_cosim", dict(cfg.cosim_deep), cfg.cosim_timeout))
    if "unit-rest" in cfg.rungs:
        for m in rest:
            steps.append((m, m, {}, cfg.unit_timeout))
    return steps


def _base_env(cfg: Config, work: pathlib.Path) -> Dict[str, str]:
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join(
        [str(work), str(cfg.repo)] + ([env["PYTHONPATH"]] if env.get("PYTHONPATH") else [])
    )
    env.pop("LOOM_COSIM_REPLAY", None)
    env["LOOM_COSIM_KEEP_GOING"] = "0"
    return env


def _results_verdict(test_dir: pathlib.Path) -> Tuple[bool, str]:
    """(passed, detail) from a cocotb results.xml."""
    path = test_dir / "results.xml"
    if not path.exists():
        return False, "no results.xml (the build or the simulator failed)"
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except OSError as exc:  # pragma: no cover
        return False, str(exc)
    if "<failure" not in text and "<error" not in text:
        return True, ""
    # A passing testcase is self-closing, so a regex over the whole file walks
    # straight past it into the next one; split on the tag instead.
    for chunk in text.split("<testcase")[1:]:
        head, _, tail = chunk.partition(">")
        if head.rstrip().endswith("/"):
            continue
        body = tail.split("</testcase>", 1)[0]
        if "<failure" not in body and "<error" not in body:
            continue
        name = re.search(r'\bname="([^"]+)"', head)
        why = re.search(r'message="([^"]*)"', body)
        detail = "%s failed" % (name.group(1) if name else "a test")
        if why and why.group(1).strip():
            detail += ": " + why.group(1).strip()[:120]
        return False, detail
    return False, "a test failed"


def run_check(
    name: str,
    kind: str,
    extra: Dict[str, str],
    timeout: float,
    work: pathlib.Path,
    cfg: Config,
    build_dir: pathlib.Path,
) -> CheckRun:
    env = _base_env(cfg, work)
    env.update(extra)
    if kind == "lint":
        rc, out, timed_out, secs = _run(
            "verilator --lint-only -Wall -Isrc --top-module tt_um_loom src/*.v",
            work,
            env,
            timeout,
        )
        if timed_out:
            return CheckRun(name, False, secs, "timeout")
        if rc == 0:
            return CheckRun(name, True, secs)
        m = _VERILATOR_ERR_RE.search(out)
        return CheckRun(name, False, secs, (m.group(0) if m else out.strip().split("\n")[-1])[:200])

    test_dir = work / "test"
    results = test_dir / "results.xml"
    if results.exists():
        results.unlink()
    cmd = "make -s SRC_DIR=%s SIM_BUILD=%s COCOTB_TEST_MODULES=%s" % (
        shlex.quote(str(work / "src")),
        shlex.quote(str(build_dir)),
        shlex.quote(kind),
    )
    rc, out, timed_out, secs = _run(cmd, test_dir, env, timeout)
    if timed_out:
        return CheckRun(name, False, secs, "timeout after %.0f s" % secs)
    passed, detail = _results_verdict(test_dir)
    if passed and rc != 0:
        passed, detail = False, "make exited %d" % rc
    seed = cycle = None
    if not passed:
        div = _DIVERGENCE_RE.search(out)
        if div:
            seed, cycle = int(div.group(2)), int(div.group(4))
            detail = "seed %d, cycle %d, %s: %s" % (
                seed,
                cycle,
                div.group(3).strip().split()[0],  # "backdoor" / the SPI label
                div.group(1),
            )
        elif "error" in out.lower() and not detail:
            detail = out.strip().split("\n")[-1][:200]
    return CheckRun(name, passed, secs, detail, seed, cycle)


def run_mutant(mut: Mutation, cfg: Config, ws: Workspace) -> MutantResult:
    ident = mut.ident
    start = time.time()
    checks: List[CheckRun] = []
    work = ws.checkout(mut, ident)
    build_dir = cfg.build / ident
    try:
        for name, kind, extra, timeout in ladder_for(mut.module, cfg):
            run = run_check(name, kind, extra, timeout, work, cfg, build_dir)
            checks.append(run)
            if not run.passed:
                return MutantResult(
                    mutation=mut,
                    status="killed",
                    killed_by=run.name,
                    detail=run.detail,
                    seconds=time.time() - start,
                    checks=checks,
                    seed=run.seed,
                    cycle=run.cycle,
                )
        return MutantResult(
            mutation=mut,
            status="survived",
            killed_by=None,
            detail="",
            seconds=time.time() - start,
            checks=checks,
        )
    finally:
        shutil.rmtree(build_dir, ignore_errors=True)
        keep = cfg.keep_survivors and checks and all(c.passed for c in checks)
        if not keep:
            shutil.rmtree(work, ignore_errors=True)


# --------------------------------------------------------------------------
# Driving a whole run
# --------------------------------------------------------------------------


def baseline_ok(cfg: Config, ws: Workspace) -> Tuple[bool, List[CheckRun]]:
    """Run the full ladder on the unmutated copy. Everything must pass."""
    work = ws.checkout(None, "_baseline")
    build_dir = cfg.build / "_baseline"
    checks: List[CheckRun] = []
    try:
        for name, kind, extra, timeout in ladder_for("loom_core", cfg):
            run = run_check(name, kind, extra, timeout, work, cfg, build_dir)
            checks.append(run)
            if not run.passed:
                return False, checks
        return True, checks
    finally:
        shutil.rmtree(build_dir, ignore_errors=True)
        shutil.rmtree(work, ignore_errors=True)


def run_all(
    muts: Sequence[Mutation],
    cfg: Config,
    out_path: Optional[pathlib.Path] = None,
    done: Optional[Iterable[str]] = None,
    progress=None,
) -> List[MutantResult]:
    """Run every mutant, `cfg.jobs` at a time, appending results as they land."""
    ws = Workspace(cfg)
    ws.build()
    skip = set(done or ())
    todo = [m for m in muts if m.ident not in skip]
    results: List[MutantResult] = []
    lock = threading.Lock()
    handle = open(out_path, "a", encoding="utf-8") if out_path else None
    counter = {"n": 0}

    def one(mut: Mutation) -> MutantResult:
        try:
            res = run_mutant(mut, cfg, ws)
        except Exception as exc:  # pragma: no cover - keep the run going
            res = MutantResult(mut, "error", None, repr(exc), 0.0, [])
        with lock:
            results.append(res)
            counter["n"] += 1
            if handle:
                handle.write(json.dumps(res.as_dict()) + "\n")
                handle.flush()
            if progress:
                progress(counter["n"], len(todo), res)
        return res

    try:
        with ThreadPoolExecutor(max_workers=cfg.jobs) as pool:
            list(pool.map(one, todo))
    finally:
        if handle:
            handle.close()
        shutil.rmtree(ws.root, ignore_errors=True)
    return results


def load_results(path: pathlib.Path) -> List[MutantResult]:
    out: List[MutantResult] = []
    if not path.exists():
        return out
    with open(path, "r", encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if line:
                out.append(MutantResult.from_dict(json.loads(line)))
    return out
