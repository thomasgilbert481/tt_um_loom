"""tools.loomnew: start a new protocol for Loom in one command.

    python -m tools.loomnew <name> [--pin OUT0] [--tick 25] [--force]

writes three files that work as generated:

* ``firmware/<name>.loom``: the header every program carries, ``.imem 512``,
  a pin, a host command and a report, and a minimal timed loop (a square
  wave, every edge on the exact clock of its tick);
* ``tools/protomodels/<name>.py``: a model stub, an edge log and a checker;
* ``tools/tests/test_fw_<name>.py``: scenarios that run on the golden model
  under pytest, and on the RTL with ``bash scripts/try_rtl.sh``.

Then "edit, assemble, test" is three commands (printed at the end).
``docs/TRY_IT.md`` walks from these files to a real protocol.

``python -m tools.loomnew.quickstart`` is the demo ``scripts/quickstart.sh``
runs; ``tools.loomnew.rtl`` is the cocotb module that runs one test module
on the RTL; ``tools.loomnew.trace`` draws a pin trace as text.
"""

from __future__ import annotations

import datetime
import pathlib
import re
from typing import Dict, Optional

from . import templates

NAME_RE = re.compile(r"^[a-z][a-z0-9_]{1,30}$")


def paths(name: str, root: pathlib.Path) -> Dict[str, pathlib.Path]:
    """The three files of protocol ``name`` under the repository ``root``."""
    return {
        "firmware": root / "firmware" / ("%s.loom" % name),
        "model": root / "tools" / "protomodels" / ("%s.py" % name),
        "test": root / "tools" / "tests" / ("test_fw_%s.py" % name),
    }


def render(name: str, pin: str = "OUT0", tick: int = 25,
           date: Optional[str] = None) -> Dict[str, str]:
    """The text of the three files, keyed as :func:`paths`."""
    if not NAME_RE.match(name):
        raise ValueError("name %r: use 2 to 31 characters, lower case letters, "
                         "digits and _, starting with a letter" % name)
    if tick < 16:
        raise ValueError("tick %d: the scaffold's loop is proved for 16 clocks "
                         "and up" % tick)
    hz = 50e6 / (2 * tick)
    rate = "%g kHz" % (hz / 1e3) if hz < 1e6 else "%g MHz" % (hz / 1e6)
    fields = dict(name=name, pin=pin, tick=tick, rate=rate,
                  date=date or datetime.date.today().isoformat())
    return {
        "firmware": templates.FIRMWARE.format(**fields),
        "model": templates.MODEL.format(**fields),
        "test": templates.TEST.format(**fields),
    }


def create(name: str, root: pathlib.Path, *, pin: str = "OUT0", tick: int = 25,
           force: bool = False) -> Dict[str, pathlib.Path]:
    """Write the three files; refuse to overwrite any of them unless ``force``."""
    texts = render(name, pin=pin, tick=tick)
    where = paths(name, root)
    clash = [p for p in where.values() if p.exists()]
    if clash and not force:
        raise FileExistsError("already there (use --force to overwrite): "
                              + ", ".join(str(p) for p in clash))
    for key, path in where.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(texts[key], encoding="utf-8", newline="\n")
    return where
