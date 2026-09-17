"""Loom assembler.

The language is described in ``tools/loomasm/README.md``. Every encoding comes
from ``tools.loomisa``, which is the only code that reads ``isa/isa.yaml``;
this package never contains an opcode.

In-process use::

    from tools.loomasm import assemble, disassemble

    program = assemble("firmware/uart_tx.loom")
    program.words                      # {address: 16-bit word}
    program.symbols                    # {label or .equ name: value}
    program.threads                    # {thread: ThreadInfo(entry, size)}
    program.listing                    # listing lines, deadline summary last
    program.diagnostics                # errors and warnings, with file:line
    program.deadlines[0].worst_slack   # deadline analysis, per thread
    program.to_image()                 # the JSON image the tools exchange

    disassemble(0x9001)                # 'WAITD 1'

Command line::

    python -m tools.loomasm firmware/uart_tx.loom -o firmware/build/uart_tx.json
    python -m tools.loomasm firmware/uart_tx.loom --listing
"""

from __future__ import annotations

from .assembler import (Program, ThreadInfo, WordInfo, assemble, assemble_file,
                        assemble_text)
from .deadline import DeadlinePair, ThreadDeadlines
from .diag import AsmError, Diagnostic
from .disasm import disassemble

__all__ = [
    "AsmError", "DeadlinePair", "Diagnostic", "Program", "ThreadDeadlines",
    "ThreadInfo", "WordInfo", "assemble", "assemble_file", "assemble_text",
    "disassemble",
]
