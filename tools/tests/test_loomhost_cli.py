"""python -m tools.loomhost: the command-line smoke tool, on the model."""

from tools.loomhost.__main__ import main
from tools.loomisa import REPO

UART_TX = str(REPO / "firmware" / "uart_tx_fifo.loom")


def test_cli_loads_runs_and_decodes_a_uart(capsys):
    code = main(["--model", UART_TX, "--uart-rx", "OUT0:32",
                 "id", "caps", "load", "csr 0 TICK_INT 32", "csr 0 TICK_INT",
                 "run 0", "push 0 'Hi' 0x21", "idle 1500", "status 0", "halt 0",
                 "reg 0 r1", "dump 0", "badop", "sflags 3", "irq", "step 0"])
    out = capsys.readouterr().out
    assert code == 0
    assert "ID 0x4C4D VERSION 0.1" in out
    assert "imem_words=1024" in out
    assert "loaded 18 words, verified" in out
    assert "TICK_INT = 0x0020 (32)" in out
    assert "UART: b'Hi!' (0 framing errors)" in out
    assert "BADOP 0x0000" in out and "SFLAGS 0x03" in out and "IRQ 0" in out
    assert "model:" in out and "transactions" in out


def test_cli_pop_and_reset(capsys):
    source = REPO / "firmware" / "uart_rx.loom"
    code = main(["--model", str(source), "--max-polls", "5", "load", "reset 0", "pop 0 1"])
    captured = capsys.readouterr()
    assert code == 1                                    # nothing arrives: timeout
    assert "OUTQ[0] stayed empty" in captured.err


def test_cli_reports_errors_with_status_1(capsys):
    assert main(["--model", UART_TX, "frobnicate"]) == 1
    assert "unknown command" in capsys.readouterr().err
    assert main(["--model", UART_TX, "--no-fifo", "load", "push 0 1"]) == 1
    assert "no FIFOs" in capsys.readouterr().err
