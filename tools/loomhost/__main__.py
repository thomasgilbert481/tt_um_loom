"""Smoke tool: drive a Loom (model or hardware) from the command line.

    python -m tools.loomhost --model firmware/uart_tx_fifo.loom \\
        --uart-rx OUT0:40 "load" "run 0" "push 0 'Hi'" "idle 2000" "dump 0"

Each positional argument is one command:

    id | caps | load | run [T..] | halt [T..] | step T [N] | reset [T..]
    dump T | reg T NAME [VALUE] | csr T NAME [VALUE] | push T WORD.. | pop T [N]
    status T | badop | sflags [SET] | irq | idle CLOCKS

A WORD may be a number (``0x41``) or a quoted string whose characters are
pushed one per word (``'Hi'``). ``load`` loads the ``--model`` image (or
``--image`` with a hardware transport). ``idle`` only exists for the model.
"""

from __future__ import annotations

import argparse
import shlex
import sys
from typing import List

from .loom import Loom, LoomError, load_image
from .transport import ModelTransport, TransportError


def _words(tokens: List[str]) -> List[int]:
    out: List[int] = []
    for tok in tokens:
        if len(tok) >= 2 and tok[0] == tok[-1] and tok[0] in "'\"":
            out += [ord(c) for c in tok[1:-1]]
        else:
            out.append(int(tok, 0))
    return out


def _threads(tokens: List[str]):
    return [int(t, 0) for t in tokens] if tokens else None


def run_command(loom: Loom, line: str, image, out=None) -> None:
    tok = shlex.split(line, posix=False)
    if not tok:
        return
    cmd, args = tok[0].lower(), tok[1:]
    say = lambda text: print(text, file=out or sys.stdout)   # noqa: E731
    if cmd == "id":
        say("ID 0x%04X VERSION %d.%d" % ((loom.id(),) + loom.version()))
    elif cmd == "caps":
        say(" ".join("%s=%s" % kv for kv in loom.caps().items()))
    elif cmd == "load":
        if image is None:
            raise LoomError("no image given (--model or --image)")
        words = loom.load(image)
        say("loaded %d words, verified" % len(words))
    elif cmd == "run":
        loom.run(_threads(args))
    elif cmd == "halt":
        loom.halt(_threads(args))
    elif cmd == "reset":
        loom.reset(_threads(args))
    elif cmd == "step":
        loom.step(int(args[0], 0), int(args[1], 0) if len(args) > 1 else 1)
    elif cmd == "dump":
        state = loom.dump(int(args[0], 0))
        say(" ".join("%s=%X" % kv for kv in state.items()))
    elif cmd in ("reg", "csr"):
        t, name = int(args[0], 0), args[1]
        if len(args) > 2:
            (loom.write_reg if cmd == "reg" else loom.write_csr)(t, name, int(args[2], 0))
        else:
            value = (loom.read_reg if cmd == "reg" else loom.read_csr)(t, name)
            say("%s = 0x%04X (%d)" % (name, value, value))
    elif cmd == "push":
        loom.push(int(args[0], 0), _words(args[1:]))
    elif cmd == "pop":
        words = loom.pop(int(args[0], 0), int(args[1], 0) if len(args) > 1 else 1)
        say(" ".join("0x%04X" % w for w in words))
    elif cmd == "status":
        say(" ".join("%s=%d" % kv for kv in loom.fifo_status(int(args[0], 0)).items()))
    elif cmd == "badop":
        say("BADOP 0x%04X (cleared)" % loom.badop())
    elif cmd == "sflags":
        if args:
            loom.set_sflags(int(args[0], 0))
        say("SFLAGS 0x%02X" % loom.sflags())
    elif cmd == "irq":
        say("IRQ %d" % loom.irq_pending())
    elif cmd == "idle":
        if not isinstance(loom.transport, ModelTransport):
            raise LoomError("idle needs the model transport")
        loom.transport.idle(int(args[0], 0))
    else:
        raise LoomError("unknown command %r" % cmd)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="python -m tools.loomhost",
                                     description=__doc__.split("\n\n")[0])
    where = parser.add_mutually_exclusive_group(required=True)
    where.add_argument("--model", metavar="IMAGE",
                       help="run on the golden model; IMAGE is a .json image or .loom source")
    where.add_argument("--pico", metavar="PORT", help="Pico bridge on this serial port")
    where.add_argument("--ttboard", metavar="PORT", help="TT demo board on this serial port")
    parser.add_argument("--image", help="image for 'load' with a hardware transport")
    parser.add_argument("--no-fifo", action="store_true", help="model: M1 build, no FIFOs")
    parser.add_argument("--uart-rx", metavar="PIN:CLOCKS",
                        help="model: decode UART frames on PIN, CLOCKS per bit")
    parser.add_argument("--max-polls", type=int, default=10000, metavar="N",
                        help="give up a push or pop after N status polls")
    parser.add_argument("commands", nargs="*", help="commands, one per argument")
    args = parser.parse_args(argv)

    image = None
    uart = None
    try:
        if args.model:
            from tools.protomodels.bench import Bench
            image = load_image(args.model)[0]
            bench = Bench(features=() if args.no_fifo else ("FIFO",))
            if args.uart_rx:
                from tools.protomodels.uart import UartRx
                pin, clocks = args.uart_rx.split(":")
                uart = bench.add(UartRx(pin, float(clocks)))
            transport = ModelTransport(bench)
        elif args.pico:
            from .serial_transport import PicoTransport
            transport = PicoTransport(args.pico)
        else:
            from .serial_transport import TTBoardTransport
            transport = TTBoardTransport(args.ttboard)
        if args.image:
            image = load_image(args.image)[0]
        loom = Loom(transport, max_polls=args.max_polls)
        for line in args.commands:
            run_command(loom, line, image)
        if uart is not None:
            print("UART: %r (%d framing errors)" % (uart.data, len(uart.errors)))
        if isinstance(transport, ModelTransport):
            print("model: %d cycles, %d transactions" % (transport.cycle, transport.transactions))
    except (LoomError, TransportError, ValueError, OSError) as exc:
        print("loomhost: %s" % exc, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
