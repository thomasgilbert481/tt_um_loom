"""Command line for the Loom assembler.

    python -m tools.loomasm SOURCE.loom [-o IMAGE.json] [--listing [FILE]]
                            [--pin NAME=PIN ...]

Exit status: 0 when the image was produced, 1 when assembly failed (or, with
``--strict``, when the deadline check found an infeasible schedule).

``--pin SCL=BIDIR2`` moves a pin the source's ``.pins`` names, for example
to put a program on a Tiny Tapeout Pmod's pins without editing it
(``firmware/README.md``, "Pmod builds").
"""

from __future__ import annotations

import argparse
import pathlib
import sys
from typing import List

from .assembler import assemble_file, imem_choices
from .diag import DEADLINE, ERROR, Diagnostic, AsmError


def _report(diagnostics: List[Diagnostic], stream) -> None:
    for diagnostic in diagnostics:
        print(str(diagnostic), file=stream)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        prog="python -m tools.loomasm",
        description="Assemble a Loom program into a JSON image.")
    parser.add_argument("source", help="the .loom source file")
    parser.add_argument("-o", "--output", metavar="FILE",
                        help="write the JSON image here (default: stdout)")
    parser.add_argument("--listing", nargs="?", const="-", metavar="FILE",
                        help="write the listing here ('-' or no value: stdout)")
    parser.add_argument("--imem-words", type=int, metavar="W",
                        choices=imem_choices(),
                        help="instruction-memory size to lay out for; thread t "
                             "starts at t * (W / 4). Overrides a .imem in the "
                             "source. Default 1024")
    parser.add_argument("--strict", action="store_true",
                        help="treat deadline errors as a failure")
    parser.add_argument("--no-deadline-check", action="store_true",
                        help="skip the deadline analysis entirely")
    parser.add_argument("--optimistic-setd", action="store_true",
                        help="the deadline rule from before tools finding "
                             "T-1: take each SETD to run on its tick. "
                             "Optimistic; for comparison only")
    parser.add_argument("-q", "--quiet", action="store_true",
                        help="print errors only, not warnings")
    parser.add_argument("--pin", action="append", default=[], metavar="NAME=PIN",
                        help="move the pin alias NAME of the source's .pins to "
                             "PIN (an isa.yaml pin name or an index); repeatable")
    args = parser.parse_args(argv)

    pins = {}
    for item in args.pin:
        name, sep, value = item.partition("=")
        if not sep or not name.strip() or not value.strip():
            print("loomasm: --pin expects NAME=PIN, got %r" % item, file=sys.stderr)
            return 1
        pins[name.strip()] = value.strip()

    try:
        program = assemble_file(args.source, strict=False,
                                deadline_check=not args.no_deadline_check,
                                imem_words=args.imem_words,
                                sound_setd=not args.optimistic_setd,
                                pins=pins)
    except AsmError as exc:
        _report(exc.diagnostics, sys.stderr)
        return 1
    except (OSError, ValueError) as exc:
        print("loomasm: %s" % exc, file=sys.stderr)
        return 1

    shown = program.errors if args.quiet else program.diagnostics
    _report(shown, sys.stderr)

    failed = args.strict and any(
        d.severity == ERROR and d.kind == DEADLINE for d in program.diagnostics)

    if args.listing is not None:
        text = program.listing_text()
        if args.listing == "-":
            sys.stdout.write(text)
        else:
            path = pathlib.Path(args.listing)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(text, encoding="utf-8", newline="\n")
            print("wrote %s" % path, file=sys.stderr)

    if failed:
        print("loomasm: --strict: deadline check failed, no image written",
              file=sys.stderr)
        return 1

    if args.output:
        path = program.write_image(args.output)
        print("wrote %s (%d words)" % (path, len(program.words)), file=sys.stderr)
    elif args.listing is None:
        sys.stdout.write(program.image_json())
    return 0


if __name__ == "__main__":
    sys.exit(main())
