"""Tests for the mutation runner and its report (MUT-RUN, MUT-TARGET).

They run without a simulator, so `scripts/check_all.sh` covers them.

The parts that need Icarus are exercised by running the tool; what is checked
here is everything around them: the order of the ladder, how a cocotb
`results.xml` is read, that a checkout really does put the mutant in a scratch
copy and leave `src/` alone, the sampling, and the report tables.
"""

import json
import os
import pathlib
import shutil

import pytest

from tools.mutate.operators import Mutation, mutations_for_file
from tools.mutate.__main__ import collect, main
from tools.mutate.report import (
    by_check_table,
    by_module_table,
    by_operator_table,
    dynamic_rates,
    full_report,
    kill_rates,
    load_equivalents,
    survivor_rows,
)
from tools.mutate.runner import (
    L1_MODULES,
    REPO,
    TARGET_MODULES,
    CheckRun,
    Config,
    MutantResult,
    Workspace,
    _DIVERGENCE_RE,
    _results_verdict,
    _run,
    ladder_for,
    load_results,
)

needs_bash = pytest.mark.skipif(shutil.which("bash") is None, reason="needs bash")


def a_mutation(module="loom_fifo", line=64, operator="arith"):
    return Mutation(
        path="src/%s.v" % module,
        line=line,
        col=18,
        operator=operator,
        description="`+` -> `-`",
        original="    cnt <= cnt + push;",
        mutated="    cnt <= cnt - push;",
    )


def a_result(status="killed", killed_by="lint", module="loom_fifo", operator="arith"):
    checks = [CheckRun("lint", killed_by != "lint", 0.4, "")]
    if killed_by != "lint":
        checks.append(CheckRun(killed_by or "cosim-deep", status == "survived", 5.0, ""))
    return MutantResult(
        mutation=a_mutation(module=module, operator=operator),
        status=status,
        killed_by=killed_by if status == "killed" else None,
        detail="",
        seconds=6.0,
        checks=checks,
    )


# ------------------------------------------------------------------ ladder


def test_ladder_is_cheapest_first_and_stops_with_the_units():
    steps = [s[0] for s in ladder_for("loom_fifo", Config())]
    assert steps[0] == "lint"
    assert steps[1] == "cosim-short"
    # The module's own unit test is promoted ahead of the rest of the suite.
    assert steps[2] == "test_fifo"
    assert "cosim-deep" in steps
    assert steps.index("cosim-deep") < steps.index("test_alu")


def test_ladder_covers_every_l1_module_exactly_once():
    for module in TARGET_MODULES:
        steps = [s[0] for s in ladder_for(module, Config())]
        for unit in L1_MODULES:
            assert steps.count(unit) == 1, (module, unit)


def test_ladder_honours_the_rung_selection():
    cfg = Config(rungs=("lint", "cosim-short"))
    assert [s[0] for s in ladder_for("loom_core", cfg)] == ["lint", "cosim-short"]


def test_ladder_for_an_unknown_module_still_runs_the_whole_suite():
    steps = [s[0] for s in ladder_for("loom_nonesuch", Config())]
    assert set(L1_MODULES) <= set(steps)


def test_the_l3_suite_is_not_in_the_ladder():
    steps = [s[0] for s in ladder_for("loom_core", Config())]
    assert "test_fw" not in steps and "test_flops" not in steps


# ------------------------------------------------------------ divergences


def test_divergence_line_gives_the_seed_and_the_cycle():
    # The label carries commas of its own, which an earlier greedy pattern
    # tripped over, so the seed and the cycle never reached the report.
    line = (
        "  1234.00ns co-simulation divergence (unresolvable RTL signal): "
        "seed 1, backdoor profile=m2 threads=4 caps=909A 512 words, BE, FIFO, "
        "SETPD depth 4, cycle 85"
    )
    m = _DIVERGENCE_RE.search(line)
    assert m is not None
    assert m.group(1) == "unresolvable RTL signal"
    assert (m.group(2), m.group(4)) == ("1", "85")
    assert m.group(3).strip().split()[0] == "backdoor"


def test_divergence_line_of_a_retire_record():
    line = "co-simulation divergence (retire record): seed 5001, spi seed, cycle 38710"
    m = _DIVERGENCE_RE.search(line)
    assert (m.group(2), m.group(4)) == ("5001", "38710")


def test_a_cosim_kill_carries_the_seed_and_cycle_through_json(tmp_path):
    check = CheckRun("cosim-short", False, 6.0, "seed 1, cycle 133, backdoor", 1, 133)
    res = MutantResult(a_mutation(), "killed", "cosim-short", check.detail, 6.0, [check], 1, 133)
    path = tmp_path / "r.jsonl"
    path.write_text(json.dumps(res.as_dict()) + "\n", encoding="utf-8")
    back = load_results(path)[0]
    assert (back.seed, back.cycle) == (1, 133)
    assert (back.checks[0].seed, back.checks[0].cycle) == (1, 133)


# ------------------------------------------------------------------ shell


@needs_bash
def test_run_returns_the_status_and_the_output(tmp_path):
    rc, out, timed_out, secs = _run(
        "echo hello; exit 3", tmp_path, dict(os.environ), timeout=60.0
    )
    assert rc == 3 and "hello" in out and timed_out is False and secs >= 0


@needs_bash
def test_run_kills_a_check_that_outruns_its_timeout(tmp_path):
    # A mutant that makes the design hang must not hang the run with it.
    _rc, _out, timed_out, secs = _run("sleep 60", tmp_path, dict(os.environ), timeout=1.0)
    assert timed_out is True and secs < 30


# ------------------------------------------------------------- results.xml


def _write_results(tmp_path, body):
    (tmp_path / "results.xml").write_text(body, encoding="utf-8")
    return tmp_path


def test_results_verdict_passes_a_clean_file(tmp_path):
    d = _write_results(
        tmp_path,
        '<?xml version="1.0"?><testsuites><testsuite name="all">'
        '<testcase classname="test_fifo" name="test_push" time="1"/>'
        "</testsuite></testsuites>",
    )
    assert _results_verdict(d) == (True, "")


def test_results_verdict_names_the_failing_test(tmp_path):
    d = _write_results(
        tmp_path,
        "<testsuites><testsuite>"
        '<testcase classname="test_fifo" name="test_ok" time="1"/>'
        '<testcase classname="test_fifo" name="test_pop"><failure message="x"/></testcase>'
        "</testsuite></testsuites>",
    )
    passed, detail = _results_verdict(d)
    assert passed is False and "test_pop" in detail


def test_results_verdict_treats_a_missing_file_as_a_kill(tmp_path):
    passed, detail = _results_verdict(tmp_path)
    assert passed is False and "results.xml" in detail


def test_results_verdict_ignores_a_skip(tmp_path):
    d = _write_results(
        tmp_path,
        "<testsuites><testsuite>"
        '<testcase classname="test_cosim" name="test_spi"><skipped/></testcase>'
        "</testsuite></testsuites>",
    )
    assert _results_verdict(d)[0] is True


# --------------------------------------------------------------- checkout


@pytest.fixture(scope="module")
def workspace(tmp_path_factory):
    """One template for the checkout tests: building it copies src/ and test/."""
    if not (REPO / "src").is_dir():
        pytest.skip("needs the repo")
    root = tmp_path_factory.mktemp("ws")
    ws = Workspace(Config(work=root / "w", build=root / "b"))
    ws.build()
    return ws


def test_checkout_mutates_the_copy_and_never_the_real_source(workspace):
    ws = workspace
    real = (REPO / "src" / "loom_fifo.v").read_bytes()
    mut = mutations_for_file(
        str(REPO / "src" / "loom_fifo.v"), rel="src/loom_fifo.v", operators=["arith"]
    )[0]
    dest = ws.checkout(mut, mut.ident)

    copy = (dest / "src" / "loom_fifo.v").read_text(encoding="utf-8")
    assert copy.split("\n")[mut.line - 1] == mut.mutated
    # The template and the repo are untouched, byte for byte.
    assert (ws.root / "src" / "loom_fifo.v").read_bytes() == real
    assert (REPO / "src" / "loom_fifo.v").read_bytes() == real
    # The other files of the copy are still the originals.
    assert (dest / "src" / "loom_core.v").read_bytes() == (
        REPO / "src" / "loom_core.v"
    ).read_bytes()
    # test/ came along, so `make` has a Makefile and a testbench to use.
    assert (dest / "test" / "Makefile").is_file()
    assert (dest / "test" / "tb.v").is_file()


@pytest.mark.parametrize(
    "path", [str(REPO / "src" / "loom_fifo.v"), "../src/loom_fifo.v", "/etc/passwd"]
)
def test_checkout_refuses_a_path_that_escapes_the_work_directory(workspace, path):
    mut = Mutation(
        path=path,
        line=1,
        col=1,
        operator="arith",
        description="x",
        original="a",
        mutated="b",
    )
    with pytest.raises(ValueError):
        workspace.checkout(mut, "escape_attempt")


def test_checkout_without_a_mutation_is_the_original(workspace):
    dest = workspace.checkout(None, "_baseline")
    assert (dest / "src" / "loom_fifo.v").read_bytes() == (
        REPO / "src" / "loom_fifo.v"
    ).read_bytes()


# -------------------------------------------------------------- selection


def test_collect_defaults_to_the_six_modules_of_mut_target():
    muts = collect(TARGET_MODULES, None, ())
    assert {m.module for m in muts} == set(TARGET_MODULES)
    assert len(muts) > 500


def test_sampling_is_nested_so_a_run_can_be_deepened():
    small = {m.ident for m in collect(["loom_timer"], None, (), sample=40, seed=1)}
    large = {m.ident for m in collect(["loom_timer"], None, (), sample=90, seed=1)}
    assert small <= large
    assert 25 <= len(small) <= 55


def test_sampling_keeps_every_operator_represented():
    full = collect(["loom_timer"], None, ())
    sampled = collect(["loom_timer"], None, (), sample=40, seed=3)
    assert {m.operator for m in sampled} == {m.operator for m in full}


def test_core_sample_applies_only_to_the_core():
    muts = collect(["loom_core", "loom_fifo"], None, (), core_sample=50)
    assert len([m for m in muts if m.module == "loom_core"]) <= 60
    assert len([m for m in muts if m.module == "loom_fifo"]) == len(
        collect(["loom_fifo"], None, ())
    )


# ----------------------------------------------------------------- report


def test_kill_rates_and_tables():
    results = [
        a_result("killed", "lint"),
        a_result("killed", "cosim-short"),
        a_result("survived", None),
        a_result("killed", "test_fifo", module="loom_timer", operator="const"),
    ]
    rates = kill_rates(results)
    assert rates["loom_fifo"] == (2, 1, 0)
    assert rates["loom_timer"] == (1, 0, 0)
    table = by_module_table(results)
    assert "`loom_fifo.v`" in table and "66.7 %" in table
    assert "**no**" in table  # below MUT-TARGET
    assert "`arith`" in by_operator_table(results)
    assert "`lint`" in by_check_table(results)


def test_dynamic_rate_leaves_out_the_lint_kills():
    results = [a_result("killed", "lint"), a_result("killed", "cosim-short"), a_result("survived", None)]
    assert dynamic_rates(results)["loom_fifo"] == (2, 1)


def test_survivors_are_listed_with_their_line():
    results = [a_result("killed", "lint"), a_result("survived", None)]
    survivors = survivor_rows(results)
    assert len(survivors) == 1
    text = full_report(results, wall=120.0, jobs=4)
    assert "loom_fifo.v`:64" in text
    assert "2.0 min" in text


def test_report_says_so_when_nothing_survived():
    assert "No survivors." in full_report([a_result("killed", "lint")])


def test_a_documented_equivalent_leaves_the_denominator():
    results = [a_result("killed", "lint"), a_result("survived", None)]
    eq = {results[1].mutation.ident: "the default is always overridden"}
    assert "50.0 %" in by_module_table(results, {})
    table = by_module_table(results, eq)
    assert "100.0 %" in table and "yes" in table
    text = full_report(results, equivalents=eq)
    assert "equivalent - the default is always overridden" in text


def test_an_undocumented_survivor_is_marked_open():
    assert "**open**" in full_report([a_result("survived", None)], equivalents={})


def test_the_equivalents_file_parses():
    eq = load_equivalents()
    assert isinstance(eq, dict)
    assert all(isinstance(k, str) and isinstance(v, str) for k, v in eq.items())


# ------------------------------------------------------------ persistence


def test_results_round_trip_through_json(tmp_path):
    res = a_result("killed", "cosim-short")
    res.seed, res.cycle = 1, 133
    path = tmp_path / "r.jsonl"
    path.write_text(json.dumps(res.as_dict()) + "\n", encoding="utf-8")
    back = load_results(path)
    assert len(back) == 1
    assert back[0].mutation == res.mutation
    assert (back[0].seed, back[0].cycle) == (1, 133)
    assert [c.name for c in back[0].checks] == [c.name for c in res.checks]


def test_load_results_of_a_missing_file_is_empty(tmp_path):
    assert load_results(tmp_path / "nope.jsonl") == []


# ---------------------------------------------------------------- the CLI


def test_cli_list_and_show(capsys):
    assert main(["list", "--modules", "loom_fifo"]) == 0
    out = capsys.readouterr().out
    assert "mutants:" in out and "loom_fifo_L" in out

    assert main(["show", "--modules", "loom_fifo", "loom_fifo_L0064", "--context", "2"]) == 0
    out = capsys.readouterr().out
    assert "src/loom_fifo.v:64:" in out
    # the changed line twice (before and after) plus its context
    assert out.count("\n-") >= 1 and out.count("\n+") >= 1
    assert "    62 " in out and "    66 " in out


def test_cli_only_narrows_to_one_mutant(capsys):
    assert main(["list", "--modules", "loom_fifo"]) == 0
    first = capsys.readouterr().out.split("\n")[0].split()[0]
    assert main(["list", "--modules", "loom_fifo", "--only", first]) == 0
    out = capsys.readouterr().out
    assert "1 mutants:" in out and first in out


def test_cli_only_that_matches_nothing_is_an_error():
    with pytest.raises(SystemExit):
        main(["list", "--modules", "loom_fifo", "--only", "loom_nonesuch"])


def test_cli_list_json_is_reloadable(capsys):
    assert main(["list", "--modules", "loom_fifo", "--json"]) == 0
    data = json.loads(capsys.readouterr().out)
    assert data and Mutation.from_dict(data[0]).module == "loom_fifo"


def test_cli_report_of_a_results_file(tmp_path, capsys):
    path = tmp_path / "r.jsonl"
    path.write_text(json.dumps(a_result().as_dict()) + "\n", encoding="utf-8")
    assert main(["report", str(path)]) == 0
    assert "Kill rate per module" in capsys.readouterr().out
