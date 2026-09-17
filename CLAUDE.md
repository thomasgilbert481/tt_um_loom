# Loom (tt_um_loom): instructions for Claude sessions in this repo

You are implementing a protocol emulator ASIC for the Jane Street competition
(Tiny Tapeout, IHP CMOS5L, 8x4 tiles, deadline 2027-01-18). The architecture
was set by Fable 5.1; you (usually Opus 5) implement it. Thomas Gilbert owns
the project.

## Read order, every session

1. `docs/PLAN.md`: find the current milestone and the first unchecked box.
2. `docs/SEMANTICS.md`: the cycle-exact contract that RTL and golden model both
   implement. It wins over ARCHITECTURE.md where they differ.
3. `docs/ARCHITECTURE.md`: the design and its rationale.
4. `docs/DECISIONS.md`: the last few entries, so you do not relitigate.
5. `docs/BUGS.md` and `docs/AREA.md`.
6. `isa/isa.yaml` and `docs/ISA.md` when touching anything that decodes or
   assembles.

Do not read `docs/tt_cmos5l_facts.md` unless you are making the M2 memory
decision or configuring the flow; it is reference material.

## Rules

- **The spec wins.** If the RTL and `docs/ARCHITECTURE.md` disagree, fix the
  RTL. If the spec is wrong or impossible, stop, write a `docs/DECISIONS.md`
  entry proposing the change with the reason, and ask Thomas. Do not quietly
  implement something else.
- **`isa/isa.yaml` is the only place an encoding lives.** `tools/loomisa` is
  the only code that parses it. After editing the YAML run
  `python -m tools.loomisa gen`, which writes `src/loom_decode.v` (the decoder
  module), `src/loom_isa.vh` (constants for testbenches and formal, not for
  synthesis) and `docs/ISA.md`; commit them; never hand-edit them. Hand-written
  RTL takes every decode signal and operand field from `loom_decode` and never
  compares instruction bits itself. Python code encodes and decodes through
  `tools.loomisa` (`isa.encode("ADD", rd=1, ra=2, rb=3)`), never with literals.
- **Golden model and RTL are written from the spec, not from each other.**
  When you write `tools/loomsim`, do not open `src/`. When you write RTL, do not
  open `tools/loomsim`. This independence is a stated part of the methodology
  (VERIFICATION.md METH-1). Tests may look at both.
- **Every bug found by a test, a proof or a mutant goes in `docs/BUGS.md`**
  (date, symptom, root cause, which check ID caught it, which check now covers
  it). Fixing without logging is not done.
- **Verilog subset:** Verilog-2005 that Icarus 14, Verilator 5 with
  `--lint-only -Wall`, Yosys 0.63 and the TT LibreLane flow accept.
  `default_nettype none` in every file. Synchronous active-low reset. No
  `initial` in synthesisable code. No latches outside `loom_imem.v`. Parameters
  not macros. One file per module, file name = module name.
- **Do not edit `src/config.json`** except `CLOCK_PERIOD` and
  `PL_TARGET_DENSITY_PCT`, and only with a DECISIONS entry. Never edit
  `.github/workflows/gds.yaml`, `docs.yaml`, `test.yaml` beyond adding our own
  jobs in new files.
- **Tiny Tapeout hygiene:** all outputs assigned; unused inputs listed in the
  `_unused` wire; `ena` ignored; pins documented in `info.yaml`; `source_files`
  and `test/Makefile` `PROJECT_SOURCES` kept in sync (the flow fails otherwise).
- **Commits:** small, one topic, message body says what was verified. No
  `Co-Authored-By` lines and no AI attribution trailers (Thomas's standing
  rule). Commit as Thomas (`gh auth` is configured). Do not push unless the
  session asked you to.
- **End every session in a green state:** tests pass locally, generated files
  fresh, `docs/PLAN.md` boxes updated, a short "next step" note at the bottom of
  `docs/PLAN.md` under "Session log".
- **Ask Fable, not yourself, about architecture.** A session titled
  "Loom Mx review" with Fable does milestone reviews. Ordinary implementation
  choices are yours; document them in the module header.

## Environment (Thomas's laptop)

- Windows 11. Repo at `C:\Users\Thoma\asic\tt_um_loom`. Git Bash `HOME` is
  wrong on this machine (points at a Cadence directory), so always use absolute
  paths, never `~`, in Bash tool commands.
- Simulation and formal run in WSL Ubuntu (`wsl -d Ubuntu`). OSS CAD Suite is
  at `/home/homa/oss-cad-suite` (Icarus 14, Verilator 5.047, Yosys 0.63,
  SymbiYosys, cocotb 2.1 in the bundled Python). The repo is reachable at
  `/mnt/c/Users/Thoma/asic/tt_um_loom`. Standard invocation from the Windows
  side:

  ```bash
  wsl -d Ubuntu -- bash -c 'source /home/homa/oss-cad-suite/environment && cd /mnt/c/Users/Thoma/asic/tt_um_loom/test && make'
  ```

  `/mnt/c` is slow for large `sim_build` directories; if a run is slow, set
  `SIM_BUILD=/home/homa/loom_build` in the make invocation.
- Lint: `verilator --lint-only -Wall -Isrc src/tt_um_loom.v` from the same
  WSL shell (plus every file in `info.yaml`).
- Local hardening: Docker Desktop with the `hpretl/iic-osic-tools` image (has
  OpenROAD, Yosys, KLayout, Magic). Use it for quick synthesis area checks
  (`yosys -p "synth; stat"` with the sg13cmos5l liberty once the PDK is
  installed) between CI runs. The official GDS comes from the GitHub `gds`
  workflow, never from a local run. Beware: the template's `.devcontainer`
  still targets `ihp-sg13g2` with tt-support-tools `main` and an older
  LibreLane; CI uses `pdk: ihp-sg13cmos5l`, tools branch `ihp-sg13cmos5l`
  (the action branch is `ihp-cmos5l`) and `--pdk ihp-sg13cmos5l --manual-pdk`.
  The PDK is a pinned commit of the IHP-Open-PDK `dev` branch fetched by the
  action's `install_sg13cmos5l.sh`; see `docs/tt_cmos5l_facts.md` for the hash.
- FPGA: iCEBreaker (iCE40UP5K) with nextpnr-ice40 in the same OSS CAD Suite.
  The previous FPGA project's Makefile is at `/home/homa/Makefile` for the
  pin-constraint and build pattern.
- Python on Windows is 3.14 with no cocotb; use the WSL Python for anything
  that imports cocotb. Pure-Python tools (`tools/`) must run on both.

## How to run things

```bash
# Everything (ISA check, generated-file freshness, pytest, Verilator lint, cocotb):
MSYS_NO_PATHCONV=1 wsl -d Ubuntu -- bash /mnt/c/Users/Thoma/asic/tt_um_loom/scripts/check_all.sh
# Same without the simulation:
MSYS_NO_PATHCONV=1 wsl -d Ubuntu -- bash /mnt/c/Users/Thoma/asic/tt_um_loom/scripts/check_all.sh quick
```

Inside a WSL shell: `source scripts/dev_env.sh` first (finds the OSS CAD Suite,
sets `LOOM_PY` to the Python cocotb runs under, puts PyYAML and pytest in a
private directory on `PYTHONPATH`), then `"$LOOM_PY" -m tools.loomisa gen`,
`"$LOOM_PY" -m pytest -q tools`, `make -C test`.

Two traps when driving WSL from the Windows-side Bash tool: `MSYS_NO_PATHCONV=1`
is required or `/mnt/c/...` arguments are rewritten into Git paths, and shell
variables inside a `wsl ... bash -c '...'` one-liner are expanded to nothing
before bash sees them, so put anything with variables in a script file.
Gate-level simulation needs the netlist from the CI artefact and the three PDK
cell-model files; the recipe is in `docs/BUGS.md` entry 2 and the project memory.

## Milestone M0 for a fresh session (if PLAN M0 boxes are still open)

The M0 RTL and test already exist (`src/tt_um_loom.v` = hard-wired UART TX,
`test/test.py`). Your job in M0 is only to confirm they pass in WSL, fix any
cocotb 2.x API mismatch, and hand Thomas the exact `gh` commands to create the
public repo (`gh repo create thomasgilbert481/tt_um_loom --public --source=.
--push`) plus the GitHub Pages step for the viewer job. Then start M1 at the
top of its checklist: `isa/isa.yaml` review, `tools/gen`, assembler.
