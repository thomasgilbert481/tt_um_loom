"""The ``--listing`` report: address, word, thread, timing class, source."""

from __future__ import annotations

from typing import Dict, List, TYPE_CHECKING

from tools.loomisa import Isa

from .deadline import all_summary_lines

if TYPE_CHECKING:                       # pragma: no cover
    from .assembler import Program, WordInfo
    from .parser import Stmt

HEADER = "ADDR  WORD  TH  TIMING     LINE  SOURCE"
RULE = "----  ----  --  ---------  ----  " + "-" * 40


def _row(info: "WordInfo", lineno: str, text: str) -> str:
    return "%04X  %04X  %-2d  %-9s  %4s  %s" % (
        info.addr, info.word, info.thread, info.timing, lineno, text)


def _blank_row(lineno: str, text: str) -> str:
    return "%4s  %4s  %-2s  %-9s  %4s  %s" % ("", "", "", "", lineno, text)


def build_listing(program: "Program", stmts: "List[Stmt]", isa: Isa) -> List[str]:
    """One block per source line, then one deadline summary per thread."""
    by_line: Dict[int, List["WordInfo"]] = {}
    for info in program.word_info:
        by_line.setdefault(info.line, []).append(info)

    lines = [
        "; loomasm listing for %s" % program.source,
        "; ISA %s, one slot = %d clocks, %d-word instruction memory "
        "(thread t starts at t * %d)" % (
            program.isa_version, int(isa.meta["slot_clocks"]),
            program.imem_words,
            program.imem_words // int(isa.meta["threads"])),
        ";",
        HEADER,
        RULE,
    ]
    for stmt in stmts:
        words = sorted(by_line.get(stmt.line, []), key=lambda w: w.addr)
        if not words:
            lines.append(_blank_row(str(stmt.line), stmt.text))
            continue
        for index, info in enumerate(words):
            if index == 0:
                lines.append(_row(info, str(stmt.line), stmt.text))
            else:
                lines.append(_row(info, "", "| %s" % info.text))
            if info.bounded:
                lines.append(_blank_row(
                    "", "| bounded by declaration: %s" % info.bounded))

    lines.append("")
    if program.deadlines:
        lines.extend(all_summary_lines(
            program.deadlines[t] for t in sorted(program.deadlines)))
    errors = program.errors
    warnings = program.warnings
    lines.append("%d word%s, %d thread%s, %d error%s, %d warning%s" % (
        len(program.words), "" if len(program.words) == 1 else "s",
        len(program.threads), "" if len(program.threads) == 1 else "s",
        len(errors), "" if len(errors) == 1 else "s",
        len(warnings), "" if len(warnings) == 1 else "s"))
    return lines
