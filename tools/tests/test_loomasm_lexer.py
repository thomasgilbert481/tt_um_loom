"""Lexer and constant-expression tests for tools/loomasm."""

import pytest

from tools.loomasm.expr import (ExprError, UnresolvedSymbol, evaluate,
                                symbols_in)
from tools.loomasm.lexer import (NAME, NUMBER, PUNCT, LexError, split_commas,
                                 tokenize_line)


def kinds(text):
    return [t.kind for t in tokenize_line(text, 1)]


def texts(text):
    return [t.text for t in tokenize_line(text, 1)]


def values(text):
    return [t.value for t in tokenize_line(text, 1) if t.kind == NUMBER]


# ------------------------------------------------------------------- tokens
def test_empty_and_comment_lines_have_no_tokens():
    assert tokenize_line("", 1) == []
    assert tokenize_line("   \t  ", 1) == []
    assert tokenize_line("; just a comment", 1) == []
    assert tokenize_line("   ; leading space then comment", 1) == []


def test_comment_ends_the_line():
    assert texts("NOP ; and the rest is ignored , , ,") == ["NOP"]


def test_label_and_statement_on_one_line():
    assert texts("start: SETP TX, 1") == ["start", ":", "SETP", "TX", ",", "1"]
    assert kinds("start: SETP TX, 1") == [
        NAME, PUNCT, NAME, NAME, PUNCT, NUMBER]


def test_identifiers_may_hold_dots_and_underscores():
    assert texts(".deadline_check on") == [".deadline_check", "on"]
    assert texts("my.label") == ["my.label"]


@pytest.mark.parametrize("text,expected", [
    ("0", 0),
    ("434", 434),
    ("0x1F", 0x1F),
    ("0XfF", 0xFF),
    ("0b1010", 0b1010),
    ("0b1010_0101", 0xA5),
    ("1_000", 1000),
])
def test_number_bases(text, expected):
    assert values(text) == [expected]


@pytest.mark.parametrize("text,expected", [
    ("'A'", 0x41),
    ("'0'", 0x30),
    ("' '", 0x20),
    (r"'\r'", 0x0D),
    (r"'\n'", 0x0A),
    (r"'\0'", 0x00),
    (r"'\t'", 0x09),
    (r"'\\'", 0x5C),
    (r"'\''", 0x27),
])
def test_character_literals(text, expected):
    assert values(text) == [expected]


def test_two_character_punctuation():
    assert texts("1 << 2 >> 3") == ["1", "<<", "2", ">>", "3"]


@pytest.mark.parametrize("text,fragment", [
    ("NOP @", "unexpected character"),
    ("0x", "no digits"),
    ("123abc", "malformed number"),
    (r"'\q'", "unknown escape"),
    ("'ab'", "exactly one character"),
    ("'", "unterminated"),
    ("''", "empty character literal"),
])
def test_lex_errors(text, fragment):
    with pytest.raises(LexError) as info:
        tokenize_line(text, 7)
    assert fragment in info.value.message
    assert info.value.line == 7
    assert info.value.col >= 1


# ------------------------------------------------------------- comma splits
def test_split_commas():
    groups = split_commas(tokenize_line("r1, r2, r3", 1))
    assert [[t.text for t in g] for g in groups] == [["r1"], ["r2"], ["r3"]]


def test_split_commas_ignores_commas_inside_parentheses():
    groups = split_commas(tokenize_line("r1, lo8((1 + 2))", 1))
    assert len(groups) == 2
    assert [t.text for t in groups[1]] == ["lo8", "(", "(", "1", "+", "2", ")", ")"]


def test_split_commas_of_nothing_is_no_groups():
    assert split_commas([]) == []


def test_trailing_comma_makes_an_empty_group():
    groups = split_commas(tokenize_line("r1,", 1))
    assert len(groups) == 2 and groups[1] == []


# ------------------------------------------------------------- expressions
def ev(text, symbols=None):
    symbols = symbols or {}

    def resolve(name):
        if name in symbols:
            return symbols[name]
        raise UnresolvedSymbol(name)

    return evaluate(tokenize_line(text, 1), resolve)


@pytest.mark.parametrize("text,expected", [
    ("1 + 2 * 3", 7),
    ("(1 + 2) * 3", 9),
    ("8 / 3", 2),
    ("-8 / 3", -2),                       # truncates towards zero, as C does
    ("-5", -5),
    ("- -5", 5),
    ("+7", 7),
    ("1 << 8", 256),
    ("0x1234 >> 8", 0x12),
    ("0xFF & 0x0F", 0x0F),
    ("0xF0 | 0x0F", 0xFF),
    ("1 | 2 & 3", 3),                     # & binds tighter than |
    ("1 + 2 << 4", 48),                   # + binds tighter than <<
    ("lo8(0x1234)", 0x34),
    ("hi8(0x1234)", 0x12),
    ("HI8(0xABCD)", 0xAB),                # function names are case-insensitive
    ("lo8(0x1234 + 1)", 0x35),
    ("'A' + 1", 0x42),
])
def test_expression_values(text, expected):
    assert ev(text) == expected


def test_symbols_are_resolved_and_are_case_sensitive():
    assert ev("BASE + 2", {"BASE": 0x100}) == 0x102
    with pytest.raises(UnresolvedSymbol):
        ev("base", {"BASE": 1})


@pytest.mark.parametrize("text,fragment", [
    ("1 / 0", "division by zero"),
    ("1 <<", "expected a value"),
    ("1 2", "unexpected"),
    ("(1", "missing ')'"),
    ("lo8 1", "unknown symbol"),
    ("nope(1)", "unknown function"),
    ("1 << -1", "negative"),
    ("1 << 500", "too large"),
    (",", "cannot start an expression"),
])
def test_expression_errors(text, fragment):
    with pytest.raises(ExprError) as info:
        ev(text)
    assert fragment in info.value.message


def test_empty_expression_is_an_error():
    with pytest.raises(ExprError):
        evaluate([], lambda name: 0)


def test_symbols_in_ignores_function_names():
    assert symbols_in(tokenize_line("lo8(BASE + 1)", 1)) == {"BASE"}
    assert symbols_in(tokenize_line("1 + 2", 1)) == set()
    assert symbols_in(tokenize_line("loop", 1)) == {"loop"}
