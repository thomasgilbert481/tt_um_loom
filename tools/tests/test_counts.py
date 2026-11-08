"""Smoke test for tools/counts.py, the one source of the documents' counts."""

from tools import counts


def test_counts_table_prints_positive_counts(capsys):
    assert counts.main(["--cocotb", "1", "--cocotb-source", "test",
                        "--glsdf", "1 of 1"]) == 0
    out = capsys.readouterr().out
    assert out.startswith("Counts at ")
    assert "| What | Count | Source |" in out
    assert "| cocotb tests in CI's `test` job | 1 | test |" in out
    c = counts.measure()
    p, s, b = c["programs"], c["scenarios"], c["bounded"]
    for n in (p["all"], p["design"], p["post"], c["test_modules"],
              c["rtl_modules"], s["model"], s["rtl"], b["decl"],
              b["programs"], c["bugs"]):
        assert n > 0
    assert s["rtl"] + s["model_only"] == s["model"]
    assert b["programs"] <= b["decl"]


def test_program_count_equals_the_loom_glob():
    names = list(counts.FIRMWARE.glob("*.loom"))
    p = counts.programs()
    assert p["all"] == len(names)
    assert p["design"] + p["post"] == len(names)
    assert p["missing_design"] == []
    assert p["design"] == len(counts.DESIGN_TARGET)


def test_commit_is_a_short_hash():
    sha = counts.commit()
    assert sha != "unknown"
    assert len(sha) >= 7
    int(sha, 16)
