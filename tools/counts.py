"""One table of the counts the documents quote, measured from the tree.

Hand-kept counts drift with every batch of firmware.
This script measures what it can and prints a Markdown table stamped with
the commit, so README.md, docs/info.md and docs/VERIFICATION_REPORT.md can
quote one source.

Measured here:

- ``.loom`` programs in ``firmware/``, split into the fifteen written with
  the hardware (:data:`DESIGN_TARGET`, the list README.md and docs/info.md
  give) and the rest, written after the hardware was complete (D-032, D-035);
- firmware test modules: ``tools/tests/test_fw_*.py`` files, and the ones
  ``test/test_fw.py`` lists in ``MODULES`` (those run on the RTL);
- L3 scenarios on the model and on the RTL, enumerated with
  :func:`tools.tests.fw_backend.scenarios` over ``MODULES``, the way
  ``test/test_fw.py`` does;
- ``.bounded`` declarations and the programs that use them;
- rows of the bug ledger, ``docs/BUGS.md``.

Not measurable from the tree, so given on the command line with their
source and printed as given: cocotb tests in CI's ``test`` job, the
``tools`` suite total, and the PHY-GL-SDF running total.

Usage, from the repository root::

    python -m tools.counts [--cocotb N --cocotb-source TEXT]
                           [--suite N --suite-source TEXT]
                           [--glsdf TEXT --glsdf-source TEXT]
"""

from __future__ import annotations

import argparse
import importlib
import pathlib
import re
import subprocess
import sys

REPO = pathlib.Path(__file__).resolve().parents[1]
FIRMWARE = REPO / "firmware"
TESTS = REPO / "tools" / "tests"

#: The programs written with the hardware: the two M1 UART demos and the
#: thirteen design targets of M2 to M4 (README.md, docs/info.md,
#: firmware/README.md's table up to ``manchester_loopback``).
DESIGN_TARGET = (
    "uart_hello", "uart_tx",
    "uart_tx_fifo", "uart_rx", "spi_master", "spi_slave", "i2c_master",
    "ws2812", "ps2_host", "jtag_master", "swd_master", "i2c_slave_eeprom",
    "can_loopback", "usb_ls_device", "manchester_loopback",
)

#: A ``.bounded`` declaration: the directive first on its line (comments
#: that mention ``.bounded`` do not count).
BOUNDED = re.compile(r"^\s*\.bounded\b")

#: A row of the bug ledger: ``| <number> |`` at the start of the line.
BUG_ROW = re.compile(r"^\|\s*\d+\s*\|")


def commit() -> str:
    """``git rev-parse --short HEAD``; if git cannot run here (a worktree
    whose ``.git`` file holds a Windows path, read from WSL), the same
    answer read from the git directory by hand."""
    try:
        out = subprocess.run(["git", "rev-parse", "--short", "HEAD"],
                             cwd=REPO, capture_output=True, text=True,
                             timeout=30)
        if out.returncode == 0 and out.stdout.strip():
            return out.stdout.strip()
    except (OSError, subprocess.SubprocessError):
        pass
    return _commit_by_hand()


def _gitdir() -> pathlib.Path:
    dot = REPO / ".git"
    if dot.is_dir():
        return dot
    text = dot.read_text().strip()
    path = text.split(":", 1)[1].strip() if text.startswith("gitdir:") else text
    m = re.match(r"^([A-Za-z]):[/\\](.*)$", path)
    if m and not pathlib.Path(path).exists():
        path = f"/mnt/{m.group(1).lower()}/{m.group(2)}".replace("\\", "/")
    return pathlib.Path(path)


def _commit_by_hand() -> str:
    gd = _gitdir()
    head = (gd / "HEAD").read_text().strip()
    if not head.startswith("ref:"):
        return head[:7]
    ref = head.split(":", 1)[1].strip()
    common = gd
    if (gd / "commondir").exists():
        common = (gd / (gd / "commondir").read_text().strip()).resolve()
    for base in (gd, common):
        if (base / ref).exists():
            return (base / ref).read_text().strip()[:7]
    packed = common / "packed-refs"
    if packed.exists():
        for line in packed.read_text().splitlines():
            if line.endswith(" " + ref):
                return line[:7]
    return "unknown"


def programs() -> dict:
    """``.loom`` files in ``firmware/``, split design-target / post-hardware."""
    names = sorted(p.stem for p in FIRMWARE.glob("*.loom"))
    design = [n for n in names if n in DESIGN_TARGET]
    return {"all": len(names), "design": len(design),
            "post": len(names) - len(design),
            "missing_design": sorted(set(DESIGN_TARGET) - set(names))}


def rtl_modules() -> list[str]:
    """The firmware modules ``test/test_fw.py`` lists in ``MODULES``."""
    src = (REPO / "test" / "test_fw.py").read_text()
    block = re.search(r"MODULES = \((.*?)\)", src, re.S).group(1)
    return re.findall(r"test_fw_\w+", block)


def fw_test_files() -> int:
    """``tools/tests/test_fw_*.py`` files."""
    return len(list(TESTS.glob("test_fw_*.py")))


def scenario_counts(modules: list[str]) -> dict:
    """Scenarios over ``modules``: all of them run on the model; those not
    marked ``model_only`` run on the RTL in every CI run."""
    if str(REPO) not in sys.path:
        sys.path.insert(0, str(REPO))
    from tools.tests.fw_backend import scenarios
    model = rtl = 0
    for m in modules:
        found = list(scenarios(importlib.import_module("tools.tests." + m)))
        model += len(found)
        rtl += sum(1 for s in found if not s.model_only)
    return {"model": model, "rtl": rtl, "model_only": model - rtl}


def bounded() -> dict:
    """``.bounded`` declarations in ``firmware/*.loom`` and the programs
    using them, in all and among the design-target programs."""
    decl = progs = decl_d = progs_d = 0
    for p in sorted(FIRMWARE.glob("*.loom")):
        n = sum(1 for line in p.read_text(encoding="utf-8").splitlines()
                if BOUNDED.match(line))
        if n:
            decl += n
            progs += 1
            if p.stem in DESIGN_TARGET:
                decl_d += n
                progs_d += 1
    return {"decl": decl, "programs": progs,
            "design_decl": decl_d, "design_programs": progs_d}


def bug_rows() -> int:
    """Rows of the table in ``docs/BUGS.md``."""
    text = (REPO / "docs" / "BUGS.md").read_text(encoding="utf-8")
    return sum(1 for line in text.splitlines() if BUG_ROW.match(line))


def measure() -> dict:
    mods = rtl_modules()
    return {"commit": commit(), "programs": programs(),
            "test_modules": fw_test_files(), "rtl_modules": len(mods),
            "scenarios": scenario_counts(mods), "bounded": bounded(),
            "bugs": bug_rows()}


def table(c: dict, given: list[tuple[str, str, str]] = ()) -> str:
    """The Markdown table: measured rows, then the given ones."""
    p, s, b = c["programs"], c["scenarios"], c["bounded"]
    rows = [
        ("`.loom` programs in `firmware/`", p["all"], "measured"),
        ("written with the hardware (design targets)", p["design"], "measured"),
        ("written after the hardware was complete", p["post"], "measured"),
        ("`tools/tests/test_fw_*.py` files (two, `_backend` and `_build`, are infrastructure)",
         c["test_modules"], "measured"),
        ("firmware test modules in `test/test_fw.py` MODULES (each on model and RTL)",
         c["rtl_modules"], "measured"),
        ("L3 scenarios on the model", s["model"], "measured"),
        ("of them on the RTL in every CI run", s["rtl"], "measured"),
        ("on the model only (`model_only`)", s["model_only"], "measured"),
        ("`.bounded` declarations", b["decl"], "measured"),
        ("programs using `.bounded`", b["programs"], "measured"),
        ("of them design targets (declarations)",
         f'{b["design_programs"]} ({b["design_decl"]})', "measured"),
        ("rows in `docs/BUGS.md`", c["bugs"], "measured"),
    ]
    rows += list(given)
    out = [f'Counts at {c["commit"]} (`python -m tools.counts`)', "",
           "| What | Count | Source |", "|---|---|---|"]
    out += [f"| {w} | {n} | {src} |" for w, n, src in rows]
    return "\n".join(out)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--cocotb", help="cocotb tests in CI's test job")
    ap.add_argument("--cocotb-source", default="given")
    ap.add_argument("--suite", help="Python tests in the tools suite")
    ap.add_argument("--suite-source", default="given")
    ap.add_argument("--glsdf", help="PHY-GL-SDF running total, e.g. '94 of 94'")
    ap.add_argument("--glsdf-source", default="given")
    a = ap.parse_args(argv)
    given = []
    if a.cocotb:
        given.append(("cocotb tests in CI's `test` job", a.cocotb, a.cocotb_source))
    if a.suite:
        given.append(("Python tests, `tools` suite", a.suite, a.suite_source))
    if a.glsdf:
        given.append(("PHY-GL-SDF scenarios passed on the netlist", a.glsdf,
                      a.glsdf_source))
    print(table(measure(), given))
    return 0


if __name__ == "__main__":
    sys.exit(main())
