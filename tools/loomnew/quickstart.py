"""The demo of ``scripts/quickstart.sh``: one program, start to PASS.

    python -m tools.loomnew.quickstart [TEXT]

Assembles ``firmware/uart_tx_fifo.loom`` with ``--strict`` (every deadline
proved), loads it into the golden model through the host API (the same
calls a test or the demo board makes), sends TEXT (default ``Hello, judge``)
as 8N1 UART at 1.5625 Mbaud on OUT0 from a 50 MHz clock, decodes the pin
with the UART model, prints the first byte's trace and the measured bit
times, writes the whole trace as SVG, and prints PASS if the decoded text
is what was sent and every edge is on the bit grid.
"""

from __future__ import annotations

import json
import sys

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.loomnew.trace import text_trace
from tools.protomodels.bench import Model
from tools.protomodels.uart import UartRx
from tools.tests.fw_backend import MODEL

SOURCE = REPO / "firmware" / "uart_tx_fifo.loom"
OUT_DIR = REPO / "firmware" / "build"
BIT = 32                    # clocks a bit: 1.5625 Mbaud at 50 MHz
CLK_NS = 20


class PadRecorder(Model):
    """Every change of the three pad ports, in the format of tools.waves."""

    def __init__(self) -> None:
        self.changes = []
        self.last = None

    def observe(self, lines) -> None:
        v = (lines.ui, lines.uio, lines.uo)
        if v != self.last:
            self.changes.append((lines.cycle,) + v)
            self.last = v


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    text = argv[0] if argv else "Hello, judge"
    isa = load()
    print("1. Assemble %s with --strict" % SOURCE.relative_to(REPO).as_posix())
    prog = assemble_file(SOURCE, isa=isa, strict=True)
    rep = prog.deadlines[0]
    print("   %d words, %d deadline pairs proved at a %d-clock tick, worst "
          "slack %d clocks, %d diagnostics"
          % (len(prog.words), len(rep.pairs), rep.period, rep.worst_slack,
             len(prog.diagnostics)))

    print("2. Load it into the golden model and send %r" % text)
    bench = MODEL.bench()
    rx = bench.add(UartRx("OUT0", BIT))
    rec = bench.add(PadRecorder())
    loom = MODEL.loom(bench, isa=isa)
    loom.load(prog)                         # host API: write the image
    loom.write_csr(0, "TICK_INT", BIT)      # one tick = one bit
    loom.run(0)                             # start thread 0
    loom.push(0, text.encode("ascii"))      # one INQ word a byte
    bench.run_until(lambda: len(rx.frames) >= len(text),
                    max_cycles=BIT * 12 * len(text) + 50000, every=BIT)
    got = "".join(chr(f.value) for f in rx.frames)
    print("   decoded on OUT0 by tools/protomodels/uart.py: %r" % got)

    problems = []
    if got != text:
        problems.append("decoded %r, sent %r" % (got, text))
    if any(f.framing_error for f in rx.frames):
        problems.append("a framing error")
    for f in rx.frames:
        off = [c - f.start for c, _ in f.edges]
        if any(o % BIT for o in off):
            problems.append("byte %r: an edge off the %d-clock bit grid: %s"
                            % (chr(f.value), BIT, off))

    if rx.frames:
        f = rx.frames[0]
        c0, c1 = f.start - 2 * BIT, f.start + 11 * BIT
        print("3. The first byte, %r = 0x%02X, on OUT0 (one character = 4 "
              "clocks, one bit = 8 characters):" % (chr(f.value), f.value))
        names = ["S"] + [str(i) for i in range(8)] + ["P"]
        print("        " + " " * 16 + "".join(n.ljust(8) for n in names))
        for line in text_trace("   OUT0", rx.edges, c0, c1, 4, initial=1):
            print(line)
        gaps = [b[0] - a[0] for a, b in zip(f.edges, f.edges[1:])]
        print("   edge to edge, clocks: %s (every one a multiple of %d: exact)"
              % (gaps, BIT))

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    data = OUT_DIR / "quickstart_uart.json"
    svg = OUT_DIR / "quickstart_uart.svg"
    json.dump({"module": "quickstart", "scenario": "uart", "clk_ns": CLK_NS,
               "benches": [rec.changes]}, open(data, "w"))
    if rx.frames:
        from tools.waves import draw
        t0 = (rx.frames[0].start - 2 * BIT) * CLK_NS / 1000.0
        t1 = (rx.frames[-1].start + 11 * BIT) * CLK_NS / 1000.0
        draw(str(data), str(svg), t0, t1, ["TX=OUT0"])
        print("4. The whole trace as SVG: %s" % svg.relative_to(REPO).as_posix())

    if problems:
        for p in problems:
            print("   problem: " + p)
        print("FAIL")
        return 1
    print("PASS: %d bytes sent and decoded, every edge on the bit grid"
          % len(text))
    return 0


if __name__ == "__main__":
    sys.exit(main())
