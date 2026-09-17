#!/bin/bash
# Source this file to get a working Loom development shell on Linux or WSL:
#
#     source scripts/dev_env.sh
#
# It finds the OSS CAD Suite (Icarus, Verilator, Yosys, SymbiYosys, cocotb),
# selects the Python that cocotb runs under, and makes sure the two pure-Python
# dependencies of the tools (PyYAML, pytest) are importable without modifying
# the suite: they are installed into a private directory on PYTHONPATH.
#
# Overridable: OSS_CAD_SUITE, LOOM_PYDEPS.

LOOM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export LOOM_ROOT

OSS_CAD_SUITE="${OSS_CAD_SUITE:-$HOME/oss-cad-suite}"
if [ -f "$OSS_CAD_SUITE/environment" ]; then
  # shellcheck disable=SC1091
  source "$OSS_CAD_SUITE/environment"
  LOOM_PY="$OSS_CAD_SUITE/bin/tabbypy3"
else
  LOOM_PY="$(command -v python3)"
fi
export LOOM_PY

LOOM_PYDEPS="${LOOM_PYDEPS:-$HOME/loom_pydeps}"
mkdir -p "$LOOM_PYDEPS"
export PYTHONPATH="$LOOM_ROOT:$LOOM_PYDEPS${PYTHONPATH:+:$PYTHONPATH}"

_need=""
"$LOOM_PY" -c "import yaml" 2>/dev/null || _need="$_need pyyaml"
"$LOOM_PY" -c "import pytest" 2>/dev/null || _need="$_need pytest"
if [ -n "$_need" ]; then
  echo "loom dev_env: installing$_need into $LOOM_PYDEPS"
  "$LOOM_PY" -m pip install --quiet --target "$LOOM_PYDEPS" $_need
fi
unset _need
