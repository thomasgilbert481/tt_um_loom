"""`tools.mutate.rekey`: equivalents follow lines that only moved.

The mutant id carries the line number, so an edit above a documented
equivalent orphans its entry in `equivalents.json`. These tests pin the
rule for carrying it: the same file, operator, original and mutated text,
nothing looser.
"""

from tools.mutate.operators import Mutation, mutations_for_text
from tools.mutate.rekey import carried, rekey, summary

SNIPPET = """module t (input wire clk, input wire a, output reg q);
  always @(posedge clk) begin
    if (a) q <= 1'b1;
    else q <= 1'b0;
  end
endmodule
"""


def mut(line, original, mutated, operator="const", path="src/t.v"):
    return Mutation(path=path, line=line, col=5, operator=operator,
                    description="d", original=original, mutated=mutated)


def test_a_line_that_moved_carries_its_reason_to_the_new_id():
    old = mut(10, "  x = 4'd3;", "  x = 4'd4;")
    new = mut(12, "  x = 4'd3;", "  x = 4'd4;")
    eq = {old.ident: "because"}
    [r] = rekey(eq, {old.ident: old}, [new])
    assert (r.status, r.new_id) == ("moved", new.ident)
    assert r.new_id != old.ident
    assert carried(eq, [r]) == {new.ident: "because"}


def test_a_line_that_did_not_move_keeps_its_id():
    old = mut(10, "  x = 4'd3;", "  x = 4'd4;")
    [r] = rekey({old.ident: "because"}, {old.ident: old}, [old])
    assert (r.status, r.new_id) == ("same", old.ident)


def test_a_changed_line_drops_out_and_is_reported():
    old = mut(10, "  x = 4'd3;", "  x = 4'd4;")
    now = mut(10, "  x = 4'd3 + y;", "  x = 4'd4 + y;")
    eq = {old.ident: "because"}
    [r] = rekey(eq, {old.ident: old}, [now])
    assert (r.status, r.new_id) == ("gone", None)
    assert "src/t.v:10" in r.note
    assert carried(eq, [r]) == {}


def test_the_operator_and_the_file_must_match_too():
    old = mut(10, "  x = 4'd3;", "  x = 4'd4;")
    other_op = mut(11, "  x = 4'd3;", "  x = 4'd4;", operator="index")
    other_file = mut(11, "  x = 4'd3;", "  x = 4'd4;", path="src/u.v")
    [r] = rekey({old.ident: "because"}, {old.ident: old}, [other_op, other_file])
    assert r.status == "gone"


def test_identical_lines_pick_the_nearest_and_say_so():
    old = mut(40, "  x = 4'd3;", "  x = 4'd4;")
    far, near = mut(10, "  x = 4'd3;", "  x = 4'd4;"), mut(43, "  x = 4'd3;", "  x = 4'd4;")
    [r] = rekey({old.ident: "because"}, {old.ident: old}, [far, near])
    assert (r.status, r.new_id) == ("ambiguous", near.ident)
    assert "2 identical lines" in r.note


def test_an_id_without_a_record_is_unknown():
    [r] = rekey({"loom_core_L0001C001_const_abcd": "because"}, {}, [])
    assert (r.status, r.new_id) == ("unknown", None)


def test_two_entries_landing_on_one_id_are_both_flagged():
    a = mut(10, "  x = 4'd3;", "  x = 4'd4;")
    b = mut(20, "  x = 4'd3;", "  x = 4'd4;")
    only = mut(15, "  x = 4'd3;", "  x = 4'd4;")
    result = rekey({a.ident: "one", b.ident: "two"}, {a.ident: a, b.ident: b}, [only])
    assert [r.status for r in result] == ["ambiguous", "ambiguous"]
    assert all("shares its new id" in r.note for r in result)


def test_the_real_mutator_after_a_line_is_inserted_above():
    before = mutations_for_text(SNIPPET, "src/t.v")
    after = mutations_for_text("// a new first line\n" + SNIPPET, "src/t.v")
    assert before and len(before) == len(after)
    eq = {m.ident: "reason %d" % i for i, m in enumerate(before)}
    result = rekey(eq, {m.ident: m for m in before}, after)
    assert {r.status for r in result} == {"moved"}
    assert sorted(carried(eq, result)) == sorted(m.ident for m in after)
    assert summary(result) == "%d moved" % len(before)


def test_equivalents_are_keyed_to_the_current_source():
    """Every id in equivalents.json is a mutant the source in `src/` makes
    now. An edit to `src/` that moves a documented line fails here until
    `python -m tools.mutate rekey --from-rev <the revision it was keyed
    against> --write` carries the entries along."""
    from tools.mutate.__main__ import collect
    from tools.mutate.report import load_equivalents
    from tools.mutate.runner import TARGET_MODULES
    ids = {m.ident for m in collect(list(TARGET_MODULES), None, [])}
    stale = sorted(set(load_equivalents()) - ids)
    assert stale == [], "equivalents.json names mutants the source no longer has: %s" % stale


def test_rekey_from_a_git_revision_runs(capsys):
    """`rekey --from-rev HEAD` builds its records from `git show HEAD:src/...`
    (no results file needed) and finds every module's mutants there."""
    from tools.mutate.__main__ import main
    assert main(["rekey", "--from-rev", "HEAD"]) == 0
    assert "documented equivalents" in capsys.readouterr().out
