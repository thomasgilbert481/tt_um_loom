"""Line structure: labels, one mnemonic or directive, comma-separated operands."""

from __future__ import annotations

import dataclasses
from typing import List, Optional, Tuple

from .diag import ERROR, SYNTAX, Diagnostic
from .lexer import NAME, LexError, Token, split_commas, tokenize_line


@dataclasses.dataclass
class Stmt:
    """One source line after tokenising.

    Fields below ``args`` are filled in by the assembler's first pass.
    """

    line: int
    text: str
    labels: List[str] = dataclasses.field(default_factory=list)
    head: Optional[Token] = None
    args: List[List[Token]] = dataclasses.field(default_factory=list)
    broken: bool = False            # a lex error: skipped by both passes

    thread: int = 0
    addr: int = 0
    size: int = 0
    short: bool = False             # MOV16/.csr chose the one-word form
    is_data: bool = False           # produced by .word, not an instruction

    @property
    def name(self) -> str:
        return self.head.text if self.head is not None else ""

    @property
    def upper(self) -> str:
        """Mnemonics and pseudo-ops are matched in upper case."""
        return self.name.upper()

    @property
    def lower(self) -> str:
        """Directives are matched in lower case."""
        return self.name.lower()

    @property
    def is_directive(self) -> bool:
        return self.name.startswith(".")


def parse_source(text: str, filename: str) -> Tuple[List[Stmt], List[Diagnostic]]:
    """Split ``text`` into statements; one diagnostic per malformed line."""
    stmts: List[Stmt] = []
    diagnostics: List[Diagnostic] = []
    for index, raw in enumerate(text.splitlines()):
        lineno = index + 1
        stmt = Stmt(line=lineno, text=raw.rstrip())
        try:
            tokens = tokenize_line(raw, lineno)
        except LexError as exc:
            diagnostics.append(Diagnostic(
                ERROR, SYNTAX, filename, lineno, exc.message, exc.col))
            stmt.broken = True
            stmts.append(stmt)
            continue

        pos = 0
        while (pos + 1 < len(tokens) and tokens[pos].kind == NAME
               and tokens[pos + 1].is_punct(":")):
            stmt.labels.append(tokens[pos].text)
            pos += 2
        tokens = tokens[pos:]

        if tokens:
            head = tokens[0]
            if head.kind != NAME:
                diagnostics.append(Diagnostic(
                    ERROR, SYNTAX, filename, lineno,
                    "expected a mnemonic, a directive or a label, found '%s'"
                    % head.text, head.col))
                stmt.broken = True
            else:
                stmt.head = head
                stmt.args = split_commas(tokens[1:])
        stmts.append(stmt)
    return stmts, diagnostics
