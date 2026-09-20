"""Unit tests for the mutator itself (MUT-RUN, `tools.mutate.operators`).

The contract is "given this line, exactly these mutants", so most tests hand a
snippet to `mutations_for_text` and compare the set of mutated lines. The last
group runs the mutator over the real `src/*.v` and checks the invariants that
the runner relies on: one changed line per mutant, no change inside a comment,
and every mutant applies to the file it came from.
"""

import pathlib

import pytest

from tools.mutate.operators import (
    OPERATORS,
    Mutation,
    bump_literal,
    code_mask,
    mutations_for_file,
    mutations_for_text,
    one_bit_signals,
)

REPO = pathlib.Path(__file__).resolve().parents[2]
SRC = REPO / "src"


def muts(text, operators=None, defines=()):
    return mutations_for_text(text, "t.v", defines=defines, operators=operators)


def lines_of(text, operators=None, defines=()):
    """The set of mutated lines, stripped, for easy comparison."""
    return sorted(m.mutated.strip() for m in muts(text, operators, defines))


# ---------------------------------------------------------------- code_mask


def test_code_mask_blanks_line_comments_but_keeps_columns():
    src = "  assign a = b + c;  // a + b + c\n"
    masked = code_mask(src)
    assert masked[0].rstrip() == "  assign a = b + c;"
    assert len(masked[0]) == len(src.split("\n")[0])


def test_code_mask_blanks_block_comments_across_lines():
    src = "a;\n/* here + there\n   and - back */\nb;\n"
    masked = code_mask(src)
    assert masked[0].strip() == "a;"
    assert masked[1].strip() == ""
    assert masked[2].strip() == ""
    assert masked[3].strip() == "b;"


def test_code_mask_blanks_strings_and_directives():
    src = '`default_nettype none\n$display("x + y");\n'
    masked = code_mask(src)
    assert masked[0].strip() == ""
    assert "+" not in masked[1]


def test_no_mutants_inside_comments():
    src = "// count = count + 1;\n/* a - b */\n"
    assert muts(src) == []


def test_ifdef_disabled_text_is_skipped():
    src = "`ifdef GL_TEST\nassign a = b + c;\n`else\nassign a = b - c;\n`endif\n"
    # Nothing defined: the `else` branch is live.
    assert lines_of(src, operators=["arith"]) == ["assign a = b + c;"]
    # With GL_TEST defined the other branch is.
    assert lines_of(src, operators=["arith"], defines=["GL_TEST"]) == [
        "assign a = b - c;"
    ]


def test_nested_ifdef():
    src = (
        "`ifdef A\n"
        "`ifdef B\n"
        "assign x = a + b;\n"
        "`endif\n"
        "`endif\n"
        "assign y = c + d;\n"
    )
    assert lines_of(src, operators=["arith"]) == ["assign y = c - d;"]
    assert lines_of(src, operators=["arith"], defines=["A"]) == ["assign y = c - d;"]
    assert lines_of(src, operators=["arith"], defines=["A", "B"]) == [
        "assign x = a - b;",
        "assign y = c - d;",
    ]


# ----------------------------------------------------------------- operators


def test_arith_swaps_both_ways():
    assert lines_of("assign a = b + c - d;", operators=["arith"]) == [
        "assign a = b + c + d;",
        "assign a = b - c - d;",
    ]


def test_arith_leaves_part_select_operator_alone():
    # `+:` and `-:` are part-select operators, not addition.
    assert lines_of("assign a = q[8*t +: 8];", operators=["arith"]) == []


def test_relop_swaps_strictness():
    assert lines_of("assign a = (x < y);", operators=["relop"]) == [
        "assign a = (x <= y);"
    ]
    assert lines_of("assign a = (x >= y);", operators=["relop"]) == [
        "assign a = (x > y);"
    ]


def test_relop_never_touches_a_nonblocking_assignment():
    src = "always @(posedge clk) begin\n  q <= d;\n  mem[wp] <= wdata;\nend\n"
    assert muts(src, operators=["relop"]) == []


def test_relop_finds_a_comparison_written_as_le():
    assert lines_of("assign a = (x <= y);", operators=["relop"]) == [
        "assign a = (x < y);"
    ]


def test_relop_on_a_comparison_on_the_right_of_an_assignment():
    # No parentheses: the `=` already seen makes this a comparison, not a
    # nonblocking assignment.
    assert lines_of("wire over = acc >= period;", operators=["relop"]) == [
        "wire over = acc > period;"
    ]


def test_eqop_swaps():
    assert lines_of("if (a == b) x = 1;", operators=["eqop"]) == ["if (a != b) x = 1;"]
    assert lines_of("if (a != b) x = 1;", operators=["eqop"]) == ["if (a == b) x = 1;"]


def test_bitop_swaps_single_and_double():
    assert lines_of("assign a = b & c;", operators=["bitop"]) == ["assign a = b | c;"]
    assert lines_of("assign a = b || c;", operators=["bitop"]) == ["assign a = b && c;"]


def test_bitop_swaps_a_reduction_operator():
    assert lines_of("assign a = (&d[14:0]);", operators=["bitop"]) == [
        "assign a = (|d[14:0]);"
    ]


def test_cond_inv_wraps_the_condition():
    assert lines_of("if (push && !full) x = 1;", operators=["cond_inv"]) == [
        "if (!(push && !full)) x = 1;"
    ]


def test_cond_inv_skips_a_condition_that_spans_lines():
    src = "if (a &&\n    b) x = 1;\n"
    assert muts(src, operators=["cond_inv"]) == []


def test_const_bumps_a_sized_literal_in_both_directions():
    assert lines_of("q <= 4'd7;", operators=["const"]) == ["q <= 4'd6;", "q <= 4'd8;"]


def test_const_wraps_inside_the_declared_width():
    assert lines_of("q <= 16'd0;", operators=["const"]) == [
        "q <= 16'd1;",
        "q <= 16'd65535;",
    ]


def test_const_keeps_base_and_case():
    assert "q <= 16'hFFFE;" in lines_of("q <= 16'hFFFF;", operators=["const"])
    assert "q <= 8'h01;" in lines_of("q <= 8'h00;", operators=["const"])
    assert "q <= 3'b110;" in lines_of("q <= 3'b101;", operators=["const"])


def test_const_flips_a_one_bit_literal():
    assert lines_of("q <= 1'b0;", operators=["const"]) == ["q <= 1'b1;"]


def test_const_leaves_a_declared_width_alone():
    assert muts("reg [15:0] q;", operators=["const", "index"]) == []
    assert muts("reg [15:0] mem [0:DEPTH-1];", operators=["const", "index"]) == []
    assert muts("input wire [3:0] sel,", operators=["const", "index"]) == []


def test_index_bumps_a_bit_select():
    assert lines_of("assign a = d[15];", operators=["index"]) == [
        "assign a = d[14];",
        "assign a = d[16];",
    ]


def test_index_bumps_a_part_select():
    got = lines_of("assign now_all[16*t +: 16] = now;", operators=["index"])
    assert "assign now_all[15*t +: 16] = now;" in got
    assert "assign now_all[16*t +: 17] = now;" in got


def test_index_does_not_go_negative():
    assert lines_of("assign a = d[0];", operators=["index"]) == ["assign a = d[1];"]


def test_stuck_forces_a_control_bit_in_a_condition():
    src = "reg push, full;\nalways @(posedge clk) if (push && !full) q <= 1'b1;\n"
    got = [m for m in muts(src, operators=["stuck"])]
    mutated = sorted(m.mutated.strip() for m in got)
    assert "always @(posedge clk) if (1'b0 && !full) q <= 1'b1;" in mutated
    assert "always @(posedge clk) if (push && !1'b1) q <= 1'b1;" in mutated
    assert len(got) == 4  # two signals, two constants


def test_stuck_skips_a_condition_that_is_only_the_signal():
    # `cond_inv` already covers both constants there.
    src = "reg push;\nalways @(posedge clk) if (push) q <= 1'b1;\n"
    assert muts(src, operators=["stuck"]) == []


def test_stuck_uses_a_single_bit_select_of_a_vector():
    src = "wire [3:0] sel;\nwire we;\nalways @(posedge clk) if (sel[t] && we) q <= d;\n"
    mutated = sorted(m.mutated.strip() for m in muts(src, operators=["stuck"]))
    assert "always @(posedge clk) if (1'b1 && we) q <= d;" in mutated


def test_stuck_ignores_a_vector_used_whole():
    src = "wire [3:0] sel;\nwire we;\nalways @(posedge clk) if (|sel && we) q <= d;\n"
    names = {m.description for m in muts(src, operators=["stuck"])}
    assert not any("`sel`" in n for n in names)


# ---------------------------------------------------------- helper functions


@pytest.mark.parametrize(
    "text,delta,want",
    [
        ("4'd7", 1, "4'd8"),
        ("4'd7", -1, "4'd6"),
        ("3'd7", 1, "3'd0"),
        ("16'd0", -1, "16'd65535"),
        ("16'hFFFF", 1, "16'h0000"),
        ("8'h0a", 1, "8'h0b"),
        ("5'b00111", -1, "5'b00110"),
        ("256", 1, "257"),
        ("0", -1, None),
        ("16'dx", 1, None),
        ("4'bz1", 1, None),
    ],
)
def test_bump_literal(text, delta, want):
    assert bump_literal(text, delta) == want


def test_one_bit_signals():
    src = (
        "module m (\n"
        "  input  wire clk,\n"
        "  input  wire [3:0] sel,\n"
        "  output reg  done\n"
        ");\n"
        "  reg [15:0] acc;\n"
        "  reg        busy;\n"
        "  wire       go = clk & busy;\n"
        "  wire [7:0] sum, diff;\n"
        "endmodule\n"
    )
    ones, vectors = one_bit_signals(code_mask(src))
    assert {"clk", "done", "busy", "go"} <= ones
    assert {"sel", "acc", "sum", "diff"} <= vectors
    assert not ({"sel", "acc", "sum", "diff"} & ones)


def test_unknown_operator_is_an_error():
    with pytest.raises(ValueError):
        mutations_for_text("assign a = b;", "t.v", operators=["nope"])


# --------------------------------------------------------------- identity


def test_duplicate_results_are_one_mutant():
    # `1'b1` -1 wraps to `1'b0`, which `+1` also reaches from `1'b0`.
    got = muts("q <= 1'b1;", operators=["const"])
    assert len(got) == 1


def test_apply_and_ident():
    src = "assign a = b + c;\n"
    m = muts(src, operators=["arith"])[0]
    assert m.line == 1 and m.operator == "arith"
    assert m.apply(src).split("\n")[0] == "assign a = b - c;"
    assert m.ident.startswith("t_L0001C")
    assert Mutation.from_dict(m.as_dict()) == m


def test_apply_refuses_a_stale_mutant():
    m = muts("assign a = b + c;\n", operators=["arith"])[0]
    with pytest.raises(ValueError):
        m.apply("assign a = b * c;\n")


# ------------------------------------------------------- the real sources


@pytest.mark.parametrize(
    "name",
    ["loom_core", "loom_timer", "loom_pins", "loom_be", "loom_fifo", "loom_spi_host"],
)
def test_real_source_mutants_are_one_line_each(name):
    path = SRC / (name + ".v")
    text = path.read_text(encoding="utf-8").replace("\r\n", "\n")
    got = mutations_for_file(str(path), rel="src/%s.v" % name)
    assert got, "no mutants for %s" % name
    original = text.split("\n")
    seen = set()
    for m in got:
        assert m.operator in OPERATORS
        after = m.apply(text).split("\n")
        diff = [i for i, (a, b) in enumerate(zip(original, after)) if a != b]
        assert diff == [m.line - 1], m.ident
        assert m.digest not in seen
        seen.add(m.digest)


def test_real_source_mutants_never_touch_a_comment():
    name = "loom_timer"
    path = SRC / (name + ".v")
    text = path.read_text(encoding="utf-8").replace("\r\n", "\n")
    masked = code_mask(text)
    for m in mutations_for_file(str(path), rel="src/%s.v" % name):
        col = m.col - 1
        assert masked[m.line - 1][col] != " ", m.ident
