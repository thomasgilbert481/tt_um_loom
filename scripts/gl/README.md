# Gate-level tools

Reproduce and debug CI's `gl_test` job locally. Background and the findings
that led to these scripts: `docs/tt_cmos5l_facts.md` section 12, BUGS 4.

| Script | Where | Does |
|---|---|---|
| `fetch.sh RUN_ID OUT_DIR` | Git Bash (needs `gh`) | downloads the run's `tt_submission` artifact (the netlist) and the three IHP cell-model files at the PDK commit the run pinned |
| `run.sh FETCH_DIR [MODULES] [FILTER] [PLUSARGS]` | WSL | runs cocotb at gate level on that netlist in a Linux-side copy of the repo; log in `$BUILD/test/gl.log` |
| `diff.sh FETCH_DIR MODULE TEST [--max N] [--xscan C,...]` | WSL | runs one test on the RTL and on the netlist with the same named signals dumped and compares them clock by clock |
| `mkdump.py`, `diffvcd.py` | (used by `diff.sh`) | the dump lists and the comparison |
| `sdf_filter.py IN OUT [--cells-only]` | WSL | makes a post-route SDF (the `GDS_logs` artifact's `runs/wokwi/final/sdf/<corner>/`) that Icarus can annotate: drops the SRAM macro, whose model has no specify block; `--cells-only` drops the wire delays too |

With `SDF=<file>` in the environment, `run.sh` runs the netlist with that
SDF's delays (`test/Makefile` adds `-gspecify -ginterconnect`, `test/tb.v`
calls `$sdf_annotate`). Icarus applies the delays and ignores the timing
checks. Measured on run 35940928210's typical corner: about 6 minutes to
annotate, then about 1.9 s per simulated microsecond with cell delays only;
the wire delays make it tens of times slower again. So a test of a few hundred
microseconds takes about 10 minutes, and the whole cocotb suite (181 ms) would
take days: run a chosen subset.

```bash
scripts/gl/fetch.sh 35419160398 /c/Users/Thoma/asic/gl_35419160398
MSYS_NO_PATHCONV=1 wsl bash /mnt/c/Users/Thoma/asic/tt_um_loom/scripts/gl/run.sh /mnt/c/Users/Thoma/asic/gl_35419160398 test_timing
MSYS_NO_PATHCONV=1 wsl bash /mnt/c/Users/Thoma/asic/tt_um_loom/scripts/gl/diff.sh /mnt/c/Users/Thoma/asic/gl_35419160398 test_timing test_other_threads_do_not_move_the_edges
```

What `diff.sh` reports: the first clocks where a signal is 0 in one run and 1
in the other (a real functional difference, or a race), and the first clock
after reset at which a GL output is X/Z again together with every dumped GL
signal that is X/Z then (X that RTL simulation hid, as in BUGS 4). The signal
list is in `mkdump.py`; add more with `--extra` there if a divergence needs
them. Names must exist in the RTL under the same path.
