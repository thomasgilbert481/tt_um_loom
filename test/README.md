# Loom testbench

cocotb tests for `tt_um_loom`, based on the Tiny Tapeout sample testbench.
See [the website](https://tinytapeout.com/hdl/testing/) for the flow itself.

## Layout

- `spi_host.py` — a reusable SPI master for the host port
  (`docs/HOST_PROTOCOL.md`), plus program-loading and run-control helpers.
  Everything the tests do goes through the pads, so the same tests run on the
  gate-level netlist.
- `test_host.py` — CTRL identification, IMEM, run control, STEP, DEBUG.
- `test_alu.py` — every ALU, immediate and unary instruction.
- `test_ctrl.py` — branches, DJNZ, CALL/RET, JP, JMP, CSRs, shared flags,
  DLY, BADOP.
- `test_pins.py` — SETP, OEP, OUT, IN, open drain, WAITP, WAITE, timed waits.
- `test_timing.py` — when pin edges happen: slot grid, jitter, drift.
- `test_uart.py` — end to end, a UART transmitter written in Loom assembly.
- `tb.v` — the wrapper, including the Tiny Tapeout pad model that loops
  `uio_out` back into `uio_in` for the bits the design drives. The testbench
  drives the outside world's value on `uio_drv`.

Programs are always built through `tools.loomisa` (`isa.encode(...)`), never
from literal instruction hex, and expected values are computed in the test
from `docs/SEMANTICS.md`.

## How to run

To run the RTL simulation:

```sh
make -B
```

The waveform dump is opt-in because dumping the instruction memory dominates
the run time:

```sh
make -B PLUSARGS=+dump
```

To run gatelevel simulation, first harden your project and copy `../runs/wokwi/results/final/verilog/gl/{your_module_name}.v` to `gate_level_netlist.v`.

Then run:

```sh
make -B GATES=yes
```

If you wish to save the waveform in VCD format instead of FST format, edit tb.v to use `$dumpfile("tb.vcd");` and then run:

```sh
make -B FST=
```

This will generate `tb.vcd` instead of `tb.fst`.

## How to view the waveform file

Using GTKWave

```sh
gtkwave tb.fst tb.gtkw
```

Using Surfer

```sh
surfer tb.fst
```
