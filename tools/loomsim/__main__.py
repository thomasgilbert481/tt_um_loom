"""Command line for the Loom golden model.

::

    python -m tools.loomsim run image.json --cycles 2000 --run-mask 0b0001
    python -m tools.loomsim run image.json --cycles 2000 --trace
    python -m tools.loomsim run image.json --feature FIFO --feature BE --feature SETPD

The image format is the one the assembler writes: a JSON object
``{"words": {"<address>": <word>, ...}}``.  Addresses and words may be decimal
or ``0x``-prefixed.  ``run`` prints one line per retired slot and one line per
pad change (``irq`` is the registered HOST_IRQ output, pad ``uo_out[6]``),
which is enough to eyeball a protocol program without a waveform viewer.

``--feature`` builds an optional feature and may be repeated: ``FIFO``
(PUSH/POP/WAITB, SEMANTICS 6.7), ``BE`` (the bit engine in manual mode, 6.9)
and ``SETPD`` (the deadline-latched ``SETP ... D``, 6.10).  Without any, the
model is the M1 build.
"""

from __future__ import annotations

import argparse
import sys
from typing import List, Optional

from .hostmap import CLI_FEATURES, FIFO_DEPTHS
from .machine import DEFAULT_FIFO_DEPTH, Machine, load_image_file
from .state import CycleTrace


def _int(text: str) -> int:
    """Accept 10, 0x0A, 0b1010 and 0o12."""
    return int(text, 0)


def _feature(text: str) -> str:
    """One of the optional features, case-insensitive."""
    name = text.strip().upper()
    if name not in CLI_FEATURES:
        raise argparse.ArgumentTypeError(
            "unknown feature %r (choose from %s)" % (text, ", ".join(CLI_FEATURES)))
    return name


def _depth(text: str) -> int:
    value = _int(text)
    if value not in FIFO_DEPTHS:
        raise argparse.ArgumentTypeError("FIFO depth must be 2, 4 or 8")
    return value


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python -m tools.loomsim",
        description="Run a Loom image on the golden model.")
    sub = parser.add_subparsers(dest="command", required=True)

    run = sub.add_parser("run", help="run an image for a number of cycles")
    run.add_argument("image", help="JSON image file")
    run.add_argument("--cycles", type=_int, default=1000,
                     help="number of clock cycles to run (default 1000)")
    run.add_argument("--run-mask", type=_int, default=0b0001,
                     help="CTRL.RUN value written before the first cycle")
    run.add_argument("--trace", action="store_true",
                     help="print every cycle, not only retires and pad changes")
    run.add_argument("--feature", action="append", default=[], type=_feature,
                     metavar="NAME",
                     help="build an optional feature: %s (repeatable)"
                     % ", ".join(CLI_FEATURES))
    run.add_argument("--fifo-depth", type=_depth, default=DEFAULT_FIFO_DEPTH,
                     help="INQ/OUTQ depth with --feature FIFO: 2, 4 or 8 (default 4)")
    run.add_argument("--imem-words", type=_int, default=1024,
                     help="instruction memory size in words (default 1024)")
    run.add_argument("--loopback", action="store_true",
                     help="feed uio_out back into uio_in for driven bits")
    run.add_argument("--ui-in", type=_int, default=0,
                     help="constant value held on the ui_in pads")
    run.add_argument("--uio-in", type=_int, default=0,
                     help="constant value held on the uio_in pads")
    run.add_argument("--until-halt", action="store_true",
                     help="stop early once every started thread has halted")
    return parser


def _pads(trace: CycleTrace) -> str:
    return ("pads uo_out=%02X uio_out=%02X uio_oe=%02X irq=%d"
            % (trace.uo_out, trace.uio_out, trace.uio_oe, trace.host_irq))


def _pad_line(trace: CycleTrace) -> str:
    return "cycle %-6d %s" % (trace.cycle, _pads(trace))


def cmd_run(args: argparse.Namespace, out=None) -> int:
    # Resolved here, not in the signature, so a caller that redirects
    # sys.stdout (a test, a pipe) still gets the output.
    out = sys.stdout if out is None else out
    image = load_image_file(args.image)
    lines: List[str] = []
    previous: Optional[tuple] = None

    def on_cycle(trace: CycleTrace) -> None:
        nonlocal previous
        pads = (trace.uo_out, trace.uio_out, trace.uio_oe, trace.host_irq)
        if args.trace:
            lines.append("cycle %-6d ph=%d %s"
                         % (trace.cycle, trace.ph, _pads(trace)))
        elif previous is not None and pads != previous:
            lines.append(_pad_line(trace))
        previous = pads

    machine = Machine(image, features=args.feature, imem_words=args.imem_words,
                      fifo_depth=args.fifo_depth, loopback=args.loopback,
                      on_cycle=on_cycle)
    machine.set_pad_inputs(ui_in=args.ui_in, uio_in=args.uio_in)
    machine.host_set_run(args.run_mask)

    started = args.run_mask & 0xF
    for _ in range(args.cycles):
        record = machine.step_cycle()
        if record is not None:
            lines.append(str(record))
        if args.until_halt and started and (machine.halted & started) == started:
            break

    for line in lines:
        print(line, file=out)
    print("stopped at cycle %d, run=%X halted=%X badop=%04X"
          % (machine.cycle, machine.run, machine.halted, machine.badop), file=out)
    return 0


def main(argv: Optional[List[str]] = None) -> int:
    args = _build_parser().parse_args(argv)
    if args.command == "run":
        return cmd_run(args)
    return 1


if __name__ == "__main__":
    sys.exit(main())
