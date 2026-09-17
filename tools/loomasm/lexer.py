"""Tokeniser for Loom assembly source.

One source line is tokenised independently of every other line: the language
has no multi-line constructs, which is what lets the assembler report every
bad line in a file instead of stopping at the first one.
"""

from __future__ import annotations

import dataclasses
from typing import List, Optional

NAME = "name"
NUMBER = "number"
PUNCT = "punct"

# Escapes accepted inside a character literal.
ESCAPES = {
    "r": 0x0D, "n": 0x0A, "t": 0x09, "0": 0x00, "\\": 0x5C,
    "'": 0x27, '"': 0x22, "a": 0x07, "b": 0x08, "f": 0x0C, "v": 0x0B,
    "e": 0x1B,
}

_PUNCT2 = ("<<", ">>")
_PUNCT1 = ",:=+-*/()&|"

_NAME_START = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ_."
_NAME_BODY = _NAME_START + "0123456789"


@dataclasses.dataclass(frozen=True)
class Token:
    """A single token. ``value`` is set for NUMBER tokens only."""

    kind: str
    text: str
    line: int
    col: int
    value: Optional[int] = None

    def is_punct(self, text: str) -> bool:
        return self.kind == PUNCT and self.text == text

    def __str__(self) -> str:
        return self.text


class LexError(Exception):
    """A malformed token. Carries the line and column for the diagnostic."""

    def __init__(self, message: str, line: int, col: int):
        super().__init__(message)
        self.message = message
        self.line = line
        self.col = col


def _scan_number(text: str, i: int, line: int) -> "tuple[Token, int]":
    start = i
    base = 10
    if text[i] == "0" and i + 1 < len(text) and text[i + 1] in "xX":
        base, i = 16, i + 2
        digits = "0123456789abcdefABCDEF_"
    elif text[i] == "0" and i + 1 < len(text) and text[i + 1] in "bB":
        base, i = 2, i + 2
        digits = "01_"
    else:
        digits = "0123456789_"
    body = i
    while i < len(text) and text[i] in digits:
        i += 1
    raw = text[body:i].replace("_", "")
    if not raw:
        raise LexError("number has no digits after its base prefix", line, start + 1)
    if i < len(text) and (text[i] in _NAME_BODY):
        raise LexError(
            "malformed number '%s'" % text[start:i + 1], line, start + 1)
    return Token(NUMBER, text[start:i], line, start + 1, int(raw, base)), i


def _scan_char(text: str, i: int, line: int) -> "tuple[Token, int]":
    start = i
    i += 1
    if i >= len(text):
        raise LexError("unterminated character literal", line, start + 1)
    if text[i] == "\\":
        i += 1
        if i >= len(text):
            raise LexError("unterminated character literal", line, start + 1)
        esc = text[i]
        if esc not in ESCAPES:
            raise LexError("unknown escape '\\%s'" % esc, line, i + 1)
        value = ESCAPES[esc]
        i += 1
    else:
        if text[i] == "'":
            raise LexError("empty character literal", line, start + 1)
        value = ord(text[i])
        i += 1
    if i >= len(text) or text[i] != "'":
        raise LexError(
            "character literal must hold exactly one character", line, start + 1)
    i += 1
    if value > 0xFFFF:
        raise LexError("character does not fit in 16 bits", line, start + 1)
    return Token(NUMBER, text[start:i], line, start + 1, value), i


def tokenize_line(text: str, line: int) -> List[Token]:
    """Tokenise one source line. Raises :class:`LexError` on a bad character."""
    tokens: List[Token] = []
    i = 0
    while i < len(text):
        ch = text[i]
        if ch in " \t\r\n\f\v":
            i += 1
            continue
        if ch == ";":
            break                                   # comment to end of line
        if ch in _NAME_START:
            start = i
            while i < len(text) and text[i] in _NAME_BODY:
                i += 1
            tokens.append(Token(NAME, text[start:i], line, start + 1))
            continue
        if ch.isdigit():
            token, i = _scan_number(text, i, line)
            tokens.append(token)
            continue
        if ch == "'":
            token, i = _scan_char(text, i, line)
            tokens.append(token)
            continue
        if text[i:i + 2] in _PUNCT2:
            tokens.append(Token(PUNCT, text[i:i + 2], line, i + 1))
            i += 2
            continue
        if ch in _PUNCT1:
            tokens.append(Token(PUNCT, ch, line, i + 1))
            i += 1
            continue
        raise LexError("unexpected character %r" % ch, line, i + 1)
    return tokens


def split_commas(tokens: List[Token]) -> List[List[Token]]:
    """Split a token list on commas that are not inside parentheses.

    An empty token list yields no groups; a trailing or doubled comma yields an
    empty group, which the caller reports as a missing operand.
    """
    if not tokens:
        return []
    groups: List[List[Token]] = [[]]
    depth = 0
    for token in tokens:
        if token.is_punct("("):
            depth += 1
        elif token.is_punct(")"):
            depth -= 1
        if depth == 0 and token.is_punct(","):
            groups.append([])
            continue
        groups[-1].append(token)
    return groups
