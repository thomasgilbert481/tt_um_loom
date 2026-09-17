"""Diagnostics shared by every stage of the assembler."""

from __future__ import annotations

import dataclasses
from typing import List

ERROR = "error"
WARNING = "warning"

# Diagnostic kinds. An error of a kind in FATAL_KINDS means no image is
# produced; "deadline" and "pin" are advisory and never suppress the image.
SYNTAX = "syntax"
SYMBOL = "symbol"
RANGE = "range"
LAYOUT = "layout"
DEADLINE = "deadline"
PIN = "pin"

FATAL_KINDS = frozenset({SYNTAX, SYMBOL, RANGE, LAYOUT})


@dataclasses.dataclass(frozen=True)
class Diagnostic:
    severity: str
    kind: str
    file: str
    line: int
    message: str
    column: int = 0

    @property
    def fatal(self) -> bool:
        return self.severity == ERROR and self.kind in FATAL_KINDS

    def __str__(self) -> str:
        where = "%s:%d" % (self.file, self.line)
        if self.column:
            where += ":%d" % self.column
        return "%s: %s: %s" % (where, self.severity, self.message)


class AsmError(Exception):
    """Raised by :func:`tools.loomasm.assemble` when assembly failed.

    ``diagnostics`` holds every diagnostic produced, not only the first error,
    so a caller can print the whole list.
    """

    def __init__(self, diagnostics: List[Diagnostic]):
        self.diagnostics = list(diagnostics)
        errors = [d for d in self.diagnostics if d.severity == ERROR]
        head = str(errors[0]) if errors else "assembly failed"
        if len(errors) > 1:
            head += "  (+%d more error%s)" % (
                len(errors) - 1, "" if len(errors) == 2 else "s")
        super().__init__(head)

    @property
    def errors(self) -> List[Diagnostic]:
        return [d for d in self.diagnostics if d.severity == ERROR]
