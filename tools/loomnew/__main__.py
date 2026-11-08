"""``python -m tools.loomnew <name>``: scaffold a new protocol (see __init__)."""

from __future__ import annotations

import argparse
import pathlib
import sys

from . import create


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        prog="python -m tools.loomnew",
        description="Create firmware/<name>.loom, tools/protomodels/<name>.py "
                    "and tools/tests/test_fw_<name>.py: a square wave that "
                    "assembles --strict and passes its test as generated.")
    ap.add_argument("name", help="protocol name: lower case, digits and _")
    ap.add_argument("--pin", default="OUT0",
                    help="output pin for the generated wave, OUT0..OUT5 (default OUT0)")
    ap.add_argument("--tick", type=int, default=25,
                    help="half period in clocks the generated test uses "
                         "(default 25: 1 MHz at 50 MHz; 16 or more)")
    ap.add_argument("--root", default=None,
                    help="repository root to write into (default: this one)")
    ap.add_argument("--force", action="store_true",
                    help="overwrite files that already exist")
    args = ap.parse_args(argv)

    from tools.loomisa import REPO, load
    pins = {n.upper() for n in load().pin_by_name}
    if args.pin.upper() not in pins or not args.pin.upper().startswith("OUT"):
        ap.error("--pin %s: use an output pin, OUT0..OUT5 (a BIDIR pin also "
                 "needs its output enable set; see docs/ISA.md)" % args.pin)
    root = pathlib.Path(args.root).resolve() if args.root else REPO
    try:
        where = create(args.name, root, pin=args.pin.upper(), tick=args.tick,
                       force=args.force)
    except (ValueError, FileExistsError) as exc:
        print("loomnew: %s" % exc, file=sys.stderr)
        return 1
    for path in where.values():
        try:
            shown = path.relative_to(root)
        except ValueError:
            shown = path
        print("wrote %s" % shown)
    n = args.name
    print()
    print("Next: edit, assemble, test.")
    print("  1. edit   firmware/%s.loom (and the model, and the test)" % n)
    print("  2. python -m tools.loomasm --strict firmware/%s.loom --listing" % n)
    print("  3. python -m pytest -q tools/tests/test_fw_%s.py" % n)
    print("RTL, with Icarus Verilog and cocotb: bash scripts/try_rtl.sh test_fw_%s" % n)
    return 0


if __name__ == "__main__":
    sys.exit(main())
