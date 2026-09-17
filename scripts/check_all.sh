#!/bin/bash
# One command that runs every local check. Exit status is non-zero if any fails.
#
#     bash scripts/check_all.sh            # everything
#     bash scripts/check_all.sh quick      # skip the cocotb simulation
#
# From Windows (Git Bash or the Claude Code Bash tool) run it through WSL:
#
#     MSYS_NO_PATHCONV=1 wsl -d Ubuntu -- bash /mnt/c/Users/Thoma/asic/tt_um_loom/scripts/check_all.sh
#
# Use a script file like this one rather than a one-liner: shell variables in
# commands passed through wsl.exe are expanded (to nothing) before bash sees them.

set -u
cd "$(dirname "${BASH_SOURCE[0]}")/.." || exit 1
# shellcheck disable=SC1091
source scripts/dev_env.sh
mode="${1:-full}"
fail=0

step() { echo; echo "=== $1 ==="; }

step "ISA: encodings, generated files"
"$LOOM_PY" -m tools.loomisa check || fail=1
"$LOOM_PY" -m tools.loomisa gen --check || fail=1

step "Python tests (tools/)"
"$LOOM_PY" -m pytest -q tools || fail=1

step "Verilator lint (-Wall), TT top"
verilator --lint-only -Wall -Isrc --top-module tt_um_loom src/*.v || fail=1

step "Verilator lint (-Wall), generated decoder"
verilator --lint-only -Wall --top-module loom_decode src/loom_decode.v || fail=1

if [ "$mode" != "quick" ]; then
  step "cocotb RTL simulation (Icarus)"
  ( cd test && make clean >/dev/null 2>&1; make SIM_BUILD="${LOOM_SIM_BUILD:-$HOME/loom_sim_build}" 2>&1 | grep -E "PASS=|FAIL=|passed|failed|Error|error" )
  if [ ! -f test/results.xml ] || grep -q "<failure" test/results.xml; then fail=1; fi
fi

echo
if [ "$fail" -eq 0 ]; then echo "ALL CHECKS PASSED"; else echo "SOME CHECKS FAILED"; fi
exit "$fail"
