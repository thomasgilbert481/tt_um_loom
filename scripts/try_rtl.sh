#!/usr/bin/env bash
# Run one firmware test module's bodies on the RTL (Icarus Verilog + cocotb):
#
#     bash scripts/try_rtl.sh test_fw_<name> [SUBSTRING]
#
# The same bodies pytest runs on the golden model, here through the real SPI
# host pads of src/ (tools/loomnew/rtl.py, test/rtl_bench.py). SUBSTRING keeps
# only the scenarios whose name contains it. model_only bodies are skipped
# with their reason; LOOM_FW_SET=all runs them too.
#
# Needs iverilog and cocotb-config on PATH: scripts/quickstart.sh puts cocotb
# in .venv when Icarus is found, so `source .venv/bin/activate` first, or use
# the OSS CAD Suite (source scripts/dev_env.sh). SIM_BUILD overrides the build
# directory (default test/sim_build/try). Icarus runs about 6,400 clocks a
# second: keep a module's RTL scenarios to about two minutes.
set -euo pipefail

if [ $# -lt 1 ]; then
  echo "usage: bash scripts/try_rtl.sh test_fw_<name> [SUBSTRING]" >&2
  exit 2
fi
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODULE="${1%.py}"
MODULE="${MODULE##*/}"
if [ ! -f "$ROOT/tools/tests/$MODULE.py" ]; then
  echo "try_rtl: no tools/tests/$MODULE.py" >&2
  exit 2
fi
for tool in iverilog cocotb-config make; do
  if ! command -v "$tool" >/dev/null 2>&1; then
    echo "try_rtl: $tool not found. Install Icarus Verilog (apt install iverilog," >&2
    echo "  brew install icarus-verilog) and run scripts/quickstart.sh again, or" >&2
    echo "  activate the venv: source .venv/bin/activate" >&2
    exit 3
  fi
done

BUILD="${SIM_BUILD:-sim_build/try}"
cd "$ROOT/test"
mkdir -p "$BUILD"
LOG="$(cd "$BUILD" && pwd)/try_rtl.log"
rm -f results.xml
echo "try_rtl: $MODULE on the RTL ($(iverilog -V 2>&1 | head -1)); build $BUILD"
start=$(date +%s)
set +e
LOOM_RTL_MODULE="$MODULE" LOOM_RTL_MATCH="${2:-}" \
  make --no-print-directory COCOTB_TEST_MODULES=tools.loomnew.rtl SIM_BUILD="$BUILD" \
  > "$LOG" 2>&1
status=$?
set -e
grep -E "PASS|FAIL|SKIP|model only|Error|error:" "$LOG" \
  | grep -v "^\s*$" | tail -25 || true
end=$(date +%s)
if [ ! -f results.xml ]; then
  echo "try_rtl: no results.xml (make exited $status); see $LOG"
  exit 1
fi
fails=$(grep -c "<failure" results.xml || true)
tests=$(grep -c "<testcase" results.xml || true)
echo "try_rtl: $tests scenario(s), $fails failed, $((end - start)) s (log: $LOG)"
if [ "$fails" != "0" ] || [ "$tests" = "0" ]; then
  echo "FAIL"
  exit 1
fi
echo "PASS"
