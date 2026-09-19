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
- `rtl_bench.py` — `RtlBench`, the RTL twin of `tools.protomodels.bench.Bench`:
  it clocks `tb.v` one edge per bench cycle with the same pad resolution, so
  the `tools.protomodels` pin models run against the design, and
  `tools.loomhost.SimTransport` moves the host bytes through the real SPI
  pads. Test bodies stay synchronous and run in a `cocotb.task.bridge`
  thread; the module header says why.
- `test_fw.py` — the L3 firmware tests (`tools/tests/test_fw_uart.py`,
  `test_fw_spi.py`, `test_fw_spi_slave.py`, `test_fw_i2c.py`) on the RTL: the
  same bodies pytest runs on the golden model, with the RTL backend of
  `rtl_bench.py`. Each scenario loads its program, sets its tick and moves
  its data through the SPI host port. Bodies marked `model_only` there (the
  115200-baud UART cases and most of the SPI mode sweep) are skipped here;
  see `tools/tests/fw_backend.py`.
- `test_flops.py` — the FLOPS fallback of the instruction memory (D-020):
  host port, addressing, programs and a short co-simulation on the second
  design instance in `tb.v`.
- `cosim_coverage.py`, `cosim_coverage_m2.py` — the functional coverage the
  co-simulation collects (L2-COV): the M1 bins and, in the subclass, the M2
  ones (FIFOs, bit engine, deadline-latched `SETP`, `HOST_IRQ`, the host's
  side of the queues).
- `tb.v` — the wrapper, including the Tiny Tapeout pad model that loops
  `uio_out` back into `uio_in` for the bits the design drives. The testbench
  drives the outside world's value on `uio_drv`. `user_project` is the chip
  build (instruction memory = the 512 x 16 SRAM macro, simulated by its
  vendored model under `macro/`); in RTL simulation only, `user_project_flops`
  is a second instance with `IMEM_IMPL "FLOPS"` and 256 words on its own
  `*_flops` pins, idle unless `test_flops.py` clocks it.

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

For this project the netlist comes from a CI run's `tt_submission` artifact
(`tt_submission/tt_um_loom.v`) and `PDK_ROOT` needs only the three cell-model
files CI uses; `docs/tt_cmos5l_facts.md` section 12 has the recipe. The
gate-level list is every module except `test_cosim` (it skips itself on a
netlist) and `test_flops` (RTL only): 70 tests, about 35 minutes.

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

## Co-simulation against the golden model (`test_cosim.py`)

`test_cosim` runs constrained-random programs from `tools/loomgen` on the RTL
and on the Python golden model `tools/loomsim` in lockstep, one clock at a
time, and compares the retire record, the pad outputs (including `HOST_IRQ`,
`uo_out[6]`) and a set of guard registers on every cycle, plus the full
architectural state periodically and at the end of each seed. The two sides
were written independently from `docs/SEMANTICS.md`. The alignment scheme is
documented at the top of the file. The backdoor load writes the instruction
array of whichever backend was built: `memory` inside the SRAM macro's
behavioural model (`u_imem.g_macro.u_macro.sram.i_SRAM_1P_behavioral_bm_bist`,
512 words) or `mem` of the flop array (`u_imem.g_flops`).

**The build comes from `CTRL.CAPS`.** Every seed starts by reading `CAPS` over
the SPI port and building the model for exactly what it reports (SEMANTICS 5):
the instruction memory size, the FIFOs and their depth, the bit engine and the
deadline-latched `SETP`. The program is generated for the same build, so the
M2 instructions either execute on both sides or are `NOP` + `BADOP` on both.
A `CAPS` bit the model cannot be built for (data memory, boot ROM, bit-engine
auto mode) fails the test instead of quietly skipping the features, and the
full-state compare covers the M2 state as well: `INQ_CNT`/`OUTQ_CNT`,
`SR`/`CNT`/`CRC` and the bit-engine CSRs, the deadline latch, `IRQ_EN`,
`IRQ_EN2`.

**Host traffic.** The two seeds that load the image over the real SPI pads
(`test_cosim_over_the_host_port`) keep the host busy for the whole run with
the program's own plan (`tools/loomgen/hostplan.py`): pushes into `INQ[t]`,
reads that pop `OUTQ[t]`, and CTRL writes of `IRQ_EN`, `IRQ_EN2`, `SWIRQ`,
`BADOP`, `SFLAGS`, `SFLAGS_CLR` and `RUN`. Each one is mirrored into the model
at the cycle the RTL commits it (the byte layer's `byte_done` says when a word
ends and when a FIFO read peeks the queue), the matching `loom_host_ctl` pulse
and data bus are compared against what was sent, and the words that come back
on MISO are compared with the model's peek. Programs generated with a plan
also use the blocking forms — a raw `PUSH` on a full `OUTQ`, a raw `POP` on an
empty `INQ` — which the host's traffic releases.

Environment knobs:

| Variable | Default | Meaning |
|---|---|---|
| `LOOM_COSIM_SEEDS` | 12 | random seeds loaded through the backdoor |
| `LOOM_COSIM_CYCLES` | 4000 | cycles per seed |
| `LOOM_COSIM_SPI_SEEDS` | 2 | seeds loaded, started and fed over the SPI host port |
| `LOOM_COSIM_SPI_CYCLES` | 8000 | cycles per SPI seed, counted from `RUN` |
| `LOOM_COSIM_STATE_EVERY` | 500 | cycles between full architectural-state compares |
| `LOOM_COSIM_KEEP_GOING` | 0 | 1: record a divergence and go on to the next seed |
| `LOOM_COSIM_REPLAY` | | run one saved program instead of the random set |
| `LOOM_FLOPS_SEEDS` | 4 | backdoor seeds on the FLOPS instance (`test_flops.py`) |
| `LOOM_FLOPS_CYCLES` | 4000 | cycles per FLOPS seed |
| `LOOM_FLOPS_SPI_SEEDS` | 1 | FLOPS seeds loaded over the SPI host port |

The default run is about 35 seconds: 12 backdoor seeds (48 000 cycles) and two
SPI seeds (83 000 cycles, most of it the 512-word image going through the port
at SCK = clk/8). The seed plan puts the `m2` profile first, so a broken M2
feature shows up in seed 1 rather than five seeds later.

A divergence fails the test with the seed, cycle, thread, PC, disassembly and
both records, and writes `cosim_failures/seed_<n>.json` (image, stimulus, host
plan and the host actions as they were observed), which
`python -m tools.loomgen --replay test/cosim_failures/seed_<n>.json --run 4000
--trace 1` replays in the model at the same cycles. Functional coverage is
printed at the end, written to `cosim_coverage.json`, and every bin that
stayed empty is listed. The module skips itself at gate level (no hierarchy).

Fault-injection check. Deliberate one-line RTL bugs, compiled from a scratch
copy of `src/` through `make SRC_DIR=...` (the real `src/` is never touched):

| Mutant | Caught |
|---|---|
| SUB carry inverted (2026-09-18) | seed 1, cycle 133 |
| tick one 1/256 step late (2026-09-18) | seed 1, cycle 125 |
| thread-side "OUTQ not full" off by one | seed 1, cycle 417 (`WAITB OUTQ_NF, T` stalls in the RTL, completes in the model) |
| `SHO` shifts out of the wrong end of `SR` | seed 1, cycle 266 (`uo_out[5:0]`) |
| the `SETP ... D` rule-2 compare inverted | seed 1, cycle 250 (`uio_out`) |
| host-side "INQ full" off by one | SPI seed 5001, cycle 38710 (`BADOP[14]`) |

A clean run is evidence, not an absence of checking.
