"""Command line for the random program generator, for debugging a seed.

    python -m tools.loomgen --seed 12 --threads 4 --profile pins --listing
    python -m tools.loomgen --seed 12 --profile m2 --host-traffic --listing
    python -m tools.loomgen --seed 12 --features "" --listing       # M1 build
    python -m tools.loomgen --seed 12 -o seed12.json
    python -m tools.loomgen --seed 12 --run 4000          # the model only
    python -m tools.loomgen --replay test/cosim_failures/seed_12.json \\
        --run 4000 --trace 2 --from 900 --to 1300          # one thread's slots

``--replay`` reads a program written by ``test/test_cosim.py`` (or by ``-o``)
instead of generating one. ``--run`` executes it on the golden model, built
for the program's own features, with the same host schedule and pad stimulus
the co-simulation harness uses (``tools.loomgen.runner``), so the cycle
numbers match the harness report. A failure file that carries a host log (the
SPI variant records one) is replayed action by action at the cycles the
harness observed.
"""

from __future__ import annotations

import argparse
import json
import sys

from tools.loomasm.disasm import disassemble

from .generator import (AVOID_FLAGS, DEFAULT_FEATURES, DEFAULT_FIFO_DEPTH,
                        PROFILES, GeneratedProgram, generate)
from .runner import RUN_CYCLE, run_model


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="python -m tools.loomgen")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--imem-words", type=int, default=256)
    parser.add_argument("--profile", default="mixed", choices=sorted(PROFILES))
    parser.add_argument("--cycles", type=int, default=6000,
                        help="expected run length recorded in the plan")
    parser.add_argument("--avoid", default="",
                        help="comma separated: " + ",".join(sorted(AVOID_FLAGS)))
    parser.add_argument("--features", default=",".join(DEFAULT_FEATURES),
                        help="comma separated build features (empty for M1): "
                             "FIFO,BE,SETPD")
    parser.add_argument("--fifo-depth", type=int, default=DEFAULT_FIFO_DEPTH,
                        choices=(2, 4, 8))
    parser.add_argument("--host-traffic", action="store_true",
                        help="generate an SPI host plan and code that uses it")
    parser.add_argument("--replay", metavar="JSON",
                        help="load a program instead of generating one")
    parser.add_argument("--listing", action="store_true",
                        help="print the disassembly")
    parser.add_argument("--run", type=int, metavar="CYCLES", default=0,
                        help="run on the golden model and summarise")
    parser.add_argument("--trace", type=int, metavar="THREAD", default=None,
                        help="with --run: print that thread's retire records")
    parser.add_argument("--from", dest="first", type=int, default=0,
                        help="with --trace: first W cycle to print")
    parser.add_argument("--to", dest="last", type=int, default=None,
                        help="with --trace: last W cycle to print")
    parser.add_argument("-o", "--output", metavar="FILE",
                        help="write the program as JSON")
    args = parser.parse_args(argv)

    host_log = None
    if args.replay:
        with open(args.replay, "r", encoding="utf-8") as handle:
            obj = json.load(handle)
        prog = GeneratedProgram.from_obj(obj)
        host_log = obj.get("failure", {}).get("host_log")
    else:
        avoid = tuple(a for a in args.avoid.split(",") if a)
        features = tuple(f for f in args.features.replace(" ", "").split(",") if f)
        prog = generate(seed=args.seed, threads=args.threads,
                        imem_words=args.imem_words, profile=args.profile,
                        cycles=args.cycles, avoid=avoid, features=features,
                        fifo_depth=args.fifo_depth,
                        host_traffic=args.host_traffic)

    if args.output:
        with open(args.output, "w", encoding="utf-8", newline="\n") as handle:
            json.dump(prog.to_obj(), handle, indent=2)
            handle.write("\n")
        print("wrote %s" % args.output, file=sys.stderr)
    if args.listing or not (args.run or args.output):
        sys.stdout.write(prog.disassembly())

    if args.run:
        last = args.last if args.last is not None else RUN_CYCLE + args.run

        def show(record):
            w_cycle = record.x_cycle + 1
            if args.trace is not None and record.thread == args.trace \
                    and args.first <= w_cycle <= last:
                print("W %6d  %s  ; %s" % (w_cycle, record,
                                            disassemble(record.ir)))

        machine = run_model(prog, args.run, show, host_log=host_log)
        print("model after %d cycles: run=%X halted=%X badop=%04X sflags=%02X "
              "irq=%d" % (machine.cycle, machine.run, machine.halted,
                          machine.badop, machine.sflags, machine.host_irq))
        for thread in range(4):
            state = machine.threads[thread]
            print("  t%d pc=%03X steps=%-5d now=%-5d td=%-5d wait_active=%d "
                  "depth=%d flags=%d inq=%d outq=%d sr=%04X cnt=%d crc=%04X "
                  "lat=%d" % (thread, state.pc, state.steps, state.now,
                              state.td, state.wait_active, state.depth,
                              state.flags, len(state.inq), len(state.outq),
                              state.sr, state.cnt, state.crc, state.lat_valid))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
