"""tools.loomnew: start a new protocol for Loom in one command.

    python -m tools.loomnew <name> [--pin OUT0] [--tick 25] [--force]

writes three files that work as generated:

* ``firmware/<name>.loom``: the header every program carries, ``.imem 512``,
  a pin, a host command and a report, and a minimal timed loop (a square
  wave, every edge on the exact clock of its tick);
* ``tools/protomodels/<name>.py``: a model stub, an edge log and a checker;
* ``tools/tests/test_fw_<name>.py``: scenarios that run on the golden model
  under pytest, and on the RTL with ``bash scripts/try_rtl.sh``.

The generated test assembles with the staged-edge check on
(``staged_check=True``, ``loomasm --staged-edges``) and asserts it is
clean: ``--strict`` alone lets a staged ``SETP ... D`` edge land up to 5
clocks late. That is why the wave's ``.tick`` is 17, not 16: its longest
stretch between deadlines is 12 clocks, and an exact staged edge needs a
slack of 5.

    python -m tools.loomnew <name> --rx [--pin IN0] [--tick 8]

writes a receiver instead, the pattern of ``firmware/baudot_rx.loom``
(wait for the start edge, re-anchor with ``SETD 0``, sample mid-bit): an
8N1 byte receiver at 16 ticks a bit that pushes each byte (bit 8: framing
error) to the host, a pin model ``ByteTx`` that drives the input pin
(``Model.drive``, ``Drive.set``), and a test that sends bytes and a bad
stop bit and checks the words, with the same staged check.

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

TX_TICK, TX_MIN_TICK = 25, 17   # the wave's half period; its .tick
RX_TICK, RX_MIN_TICK = 8, 4     # the receiver's tick (16 a bit); its .tick


def paths(name: str, root: pathlib.Path) -> Dict[str, pathlib.Path]:
    """The three files of protocol ``name`` under the repository ``root``."""
    return {
        "firmware": root / "firmware" / ("%s.loom" % name),
        "model": root / "tools" / "protomodels" / ("%s.py" % name),
        "test": root / "tools" / "tests" / ("test_fw_%s.py" % name),
    }


def render(name: str, pin: Optional[str] = None, tick: Optional[int] = None,
           date: Optional[str] = None, rx: bool = False) -> Dict[str, str]:
    """The text of the three files, keyed as :func:`paths`.

    ``rx`` picks the receiver scaffold; ``pin`` and ``tick`` default to
    ``OUT0`` and 25 for the transmitter, ``IN0`` and 8 for the receiver."""
    if not NAME_RE.match(name):
        raise ValueError("name %r: use 2 to 31 characters, lower case letters, "
                         "digits and _, starting with a letter" % name)
    if pin is None:
        pin = "IN0" if rx else "OUT0"
    if tick is None:
        tick = RX_TICK if rx else TX_TICK
    least = RX_MIN_TICK if rx else TX_MIN_TICK
    if tick < least:
        raise ValueError("tick %d: the scaffold's %s is proved for %d clocks "
                         "and up" % (tick, "receiver" if rx else "loop", least))
    fields = dict(name=name, pin=pin, tick=tick,
                  date=date or datetime.date.today().isoformat())
    if rx:
        bit = 16 * tick
        fields.update(bit=bit, rate=_rate(50e6 / bit, "baud"))
        return {
            "firmware": templates.RX_FIRMWARE.format(**fields),
            "model": templates.RX_MODEL.format(**fields),
            "test": templates.RX_TEST.format(**fields),
        }
    fields.update(rate=_rate(50e6 / (2 * tick), "Hz"))
    return {
        "firmware": templates.FIRMWARE.format(**fields),
        "model": templates.MODEL.format(**fields),
        "test": templates.TEST.format(**fields),
    }


def _rate(value: float, unit: str) -> str:
    if value >= 1e6:
        return "%g M%s" % (value / 1e6, unit)
    return "%g k%s" % (value / 1e3, unit)


def create(name: str, root: pathlib.Path, *, pin: Optional[str] = None,
           tick: Optional[int] = None, force: bool = False,
           rx: bool = False) -> Dict[str, pathlib.Path]:
    """Write the three files; refuse to overwrite any of them unless ``force``."""
    texts = render(name, pin=pin, tick=tick, rx=rx)
    where = paths(name, root)
    clash = [p for p in where.values() if p.exists()]
    if clash and not force:
        raise FileExistsError("already there (use --force to overwrite): "
                              + ", ".join(str(p) for p in clash))
    for key, path in where.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(texts[key], encoding="utf-8", newline="\n")
    return where
