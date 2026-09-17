"""Command line for the ISA loader: check encodings, generate artefacts."""

from __future__ import annotations

import argparse
import sys

from . import REPO, generated_files, load


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="python -m tools.loomisa")
    sub = parser.add_subparsers(dest="cmd", required=True)
    sub.add_parser("check", help="report overlaps and encoding-space coverage")
    gen = sub.add_parser("gen", help="write src/loom_isa.vh and docs/ISA.md")
    gen.add_argument("--check", action="store_true",
                     help="do not write; exit 1 if a generated file is stale")
    args = parser.parse_args(argv)

    isa = load()
    problems = isa.check()
    for line in problems:
        print(f"ERROR: {line}")
    if problems:
        return 1

    if args.cmd == "check":
        used, total = isa.coverage()
        print(f"ISA {isa.version}: {len(isa.instructions)} instructions, no overlaps, "
              f"{used}/{total} words used ({100.0 * used / total:.1f}%)")
        return 0

    stale = []
    for path, text in generated_files(isa).items():
        current = path.read_text(encoding="utf-8") if path.exists() else None
        if current != text:
            stale.append(path)
            if not args.check:
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(text, encoding="utf-8", newline="\n")
                print(f"wrote {path.relative_to(REPO)}")
    if args.check and stale:
        for path in stale:
            print(f"STALE: {path.relative_to(REPO)} (run: python -m tools.loomisa gen)")
        return 1
    if not stale:
        print("generated files are up to date")
    return 0


if __name__ == "__main__":
    sys.exit(main())
