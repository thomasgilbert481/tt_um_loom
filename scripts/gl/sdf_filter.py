#!/usr/bin/env python3
"""Make a post-route SDF that Icarus can annotate onto the gate-level netlist.

    sdf_filter.py IN.sdf OUT.sdf [--cells-only]

Icarus's ``$sdf_annotate`` (with ``-gspecify -ginterconnect``) aborts on the
full SDF with a ``vpi_scan`` assertion (a NULL handle): the SRAM macro's model
has no specify block. Dropping the macro's CELL block alone did not help;
dropping it and every INTERCONNECT line that ends on the macro did, so this
does both. Everything else is kept.

``--cells-only`` also drops the top-level CELL, which holds every
INTERCONNECT (wire) delay: on run 35940928210's typical corner, Icarus
simulates about 1.9 s per simulated microsecond with cell delays only, after
about 6 minutes of annotation, and tens of times slower with the wire delays,
which are 0 to 0.2 ns there. scripts/gl/README.md has the numbers.
"""
import re
import sys

MACRO = "RM_IHPSG13_1P_512x16_c2_bm_bist"


def filter_sdf(text: str, cells_only: bool) -> str:
    parts = re.split(r"(?=\n \(CELL\n)", text)
    keep = []
    for part in parts:
        head = part[:300]
        if part.startswith("\n (CELL"):
            if '(CELLTYPE "%s")' % MACRO in head:
                continue
            if cells_only and re.search(r'\(CELLTYPE "tt_um_loom"\)', head):
                continue
        keep.append(part)
    out = "".join(keep)
    lines = [l for l in out.split("\n")
             if not ("INTERCONNECT" in l and "u_macro" in l)]
    out = "\n".join(lines)
    if not out.rstrip().endswith(")"):
        out = out.rstrip() + "\n)\n"
    return out


def main(argv):
    if len(argv) < 3:
        sys.exit(__doc__)
    cells_only = "--cells-only" in argv[3:]
    with open(argv[1]) as fh:
        text = fh.read()
    with open(argv[2], "w") as fh:
        fh.write(filter_sdf(text, cells_only))


if __name__ == "__main__":
    main(sys.argv)
