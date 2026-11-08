"""A pin trace as two lines of plain text, for a terminal.

    ____    ________          __
        |__|        |________|      (high on the top line, low on the bottom)

:func:`text_trace` takes the edges of one pin, ``[(cycle, value)]`` as
:class:`tools.protomodels.<name>.EdgeLog` records them, and draws the window
``[c0, c1)`` with ``step`` clocks a character.
"""

from __future__ import annotations

from typing import List, Sequence, Tuple


def level_at(edges: Sequence[Tuple[int, int]], cycle: int, initial: int) -> int:
    """The pin's value during ``cycle``."""
    v = initial
    for c, nv in edges:
        if c > cycle:
            break
        v = nv
    return v


def text_trace(label: str, edges: Sequence[Tuple[int, int]], c0: int, c1: int,
               step: int, initial: int = 0) -> List[str]:
    """Two lines: ``_`` on the top line where the pin is high, on the bottom
    line where it is low, and ``|`` on the bottom line at each change."""
    pad = " " * len(label)
    top, bottom = [], []
    prev = level_at(edges, c0, initial)
    for c in range(c0, c1, step):
        v = level_at(edges, c + step - 1, initial)
        changed = v != prev or any(c <= e < c + step for e, _ in edges)
        top.append("_" if v and not changed else " ")
        bottom.append("|" if changed else ("_" if not v else " "))
        prev = v
    return [pad + " " + "".join(top), label + " " + "".join(bottom)]


def ruler(label: str, c0: int, c1: int, step: int, every: int,
          clk_ns: float = 20.0) -> str:
    """A time axis under a trace: a mark every ``every`` characters, in us."""
    cells = []
    n = (c1 - c0 + step - 1) // step
    k = 0
    while k < n:
        if k % every == 0:
            tag = "%.2gus" % ((c0 + k * step) * clk_ns / 1000.0)
            tag = ("^" + tag)[: max(1, min(every, n - k))]
            cells.append(tag)
            k += len(tag)
        else:
            cells.append(" ")
            k += 1
    return " " * (len(label) + 1) + "".join(cells)
