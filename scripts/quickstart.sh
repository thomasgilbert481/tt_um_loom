#!/usr/bin/env bash
# One command from a fresh clone to a passing Loom program:
#
#     ./scripts/quickstart.sh            (or: bash scripts/quickstart.sh)
#
# Linux, macOS or WSL with Python 3.10 or later. No sudo: everything goes
# into .venv in the clone (PyYAML and pytest; cocotb too if Icarus Verilog
# is found). Then it
#   1. assembles firmware/uart_tx_fifo.loom with --strict, runs it on the
#      golden model through the host API, decodes the pin and prints the
#      trace and PASS (tools/loomnew/quickstart.py);
#   2. checks that the new-protocol scaffold passes its own test as
#      generated, and runs the worked example's tests (docs/TRY_IT.md);
#   3. if iverilog is on PATH, runs the worked example on the RTL through
#      the real SPI pads (scripts/try_rtl.sh); otherwise says how to get it.
#
# Options: --no-rtl skips step 3. PYTHON=python3.12 picks the interpreter,
# LOOM_VENV=/path picks the venv directory.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
RTL=auto
for arg in "$@"; do
  case "$arg" in
    --no-rtl) RTL=no ;;
    -h|--help) sed -n 2,21p "$0"; exit 0 ;;
    *) echo "quickstart: unknown option $arg (try --help)" >&2; exit 2 ;;
  esac
done
t_all=$(date +%s)
say() { printf '\n== [%3d s] %s\n' "$(( $(date +%s) - t_all ))" "$*"; }

say "Python and a local venv"
PY="${PYTHON:-python3}"
if ! command -v "$PY" >/dev/null 2>&1; then
  echo "quickstart: $PY not found; install Python 3.10 or later" >&2; exit 1
fi
if ! "$PY" -c 'import sys; sys.exit(sys.version_info < (3, 10))'; then
  echo "quickstart: $("$PY" -V 2>&1) is too old; Loom's tools need 3.10 or later" >&2
  echo "  (PYTHON=python3.12 ./scripts/quickstart.sh picks another one)" >&2
  exit 1
fi
VENV="${LOOM_VENV:-$ROOT/.venv}"
if [ ! -x "$VENV/bin/python" ]; then
  if ! "$PY" -m venv "$VENV" 2>/dev/null; then
    rm -rf "$VENV"
    echo "quickstart: '$PY -m venv' failed. On Debian and Ubuntu the venv module" >&2
    echo "  is a separate package: sudo apt install python3-venv (or python3.X-venv)," >&2
    echo "  then run this again. Or use the devcontainer (docs/TRY_IT.md)." >&2
    exit 1
  fi
fi
VPY="$VENV/bin/python"
export PATH="$VENV/bin:$PATH"
export PYTHONPATH="$ROOT${PYTHONPATH:+:$PYTHONPATH}"
echo "$("$VPY" -V) in ${VENV#$ROOT/}"
"$VPY" -m pip install --quiet --disable-pip-version-check pytest==8.4.2 PyYAML==6.0.3

say "1. A program on the golden model"
"$VPY" -m tools.loomnew.quickstart

say "2. The scaffold, as generated, and the worked example (pytest, model)"
"$VPY" -m pytest -q -p no:cacheprovider tools/tests/test_loomnew.py tools/tests/test_fw_stepper.py

say "3. The worked example on the RTL (Icarus Verilog + cocotb)"
if [ "$RTL" = no ]; then
  echo "skipped (--no-rtl)"
elif ! command -v iverilog >/dev/null 2>&1; then
  echo "skipped: iverilog not found. Everything above ran on the golden model."
  echo "  For the RTL: apt install iverilog, brew install icarus-verilog, or the"
  echo "  devcontainer / GitHub Codespaces (docs/TRY_IT.md); then run this again."
else
  "$VPY" -m pip install --quiet --disable-pip-version-check cocotb==2.0.1
  bash scripts/try_rtl.sh test_fw_stepper test_stepper_moves_forward_then_back
fi

printf '\nquickstart: PASS in %d s. Next: docs/TRY_IT.md, "Write your own protocol".\n' \
  "$(( $(date +%s) - t_all ))"
