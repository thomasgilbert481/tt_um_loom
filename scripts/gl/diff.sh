#!/bin/bash
# RTL-vs-gate-level differential run of one cocotb test (facts section 12).
# Runs the test twice in a Linux-side copy of the repo, once on the RTL and
# once on the netlist fetched by fetch.sh, each dumping the same named signals
# (mkdump.py), then compares the dumps clock by clock (diffvcd.py). Run inside
# WSL. The repo's own test/tb.v is not changed: the dump hook is added to the
# copy.
#
#   scripts/gl/diff.sh FETCH_DIR MODULE TEST [diffvcd.py options]
#
# e.g. scripts/gl/diff.sh /home/homa/gl_35419160398 test_timing \
#          test_other_threads_do_not_move_the_edges --xscan 9540,9541
set -u
fetch="${1:?usage: diff.sh FETCH_DIR MODULE TEST [diffvcd options]}"
mod="${2:?module}"
tst="${3:?test name}"
shift 3
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$repo/scripts/dev_env.sh" >/dev/null 2>&1
run_id="$(cat "$fetch/run_id" 2>/dev/null || echo local)"
BUILD="${BUILD:-$HOME/loom_gldiff_$run_id}"
mkdir -p "$BUILD"
for d in src test macro tools isa firmware; do
  rm -rf "${BUILD:?}/$d"
  cp -r "$repo/$d" "$BUILD/$d"
done
cp "$fetch/tt_submission/tt_um_loom.v" "$BUILD/test/gate_level_netlist.v"
"$LOOM_PY" "$repo/scripts/gl/mkdump.py" "$BUILD/test/gate_level_netlist.v" "$BUILD/src" || exit 1

# The dump hook: test/Makefile puts src/ on the include path.
"$LOOM_PY" - "$BUILD/test/tb.v" <<'PY'
import sys
p = sys.argv[1]
s = open(p).read()
anchor = "  // Wire up the inputs and outputs:"
hook = """  initial begin
    if ($test$plusargs("diffdump")) begin
      $dumpfile("diff.vcd");
`ifdef GL_TEST
      `include "gl_dump.vh"
`else
      `include "rtl_dump.vh"
`endif
    end
  end

"""
assert anchor in s, "tb.v changed: no anchor for the dump hook"
open(p, "w").write(s.replace(anchor, hook + anchor, 1))
PY

export PDK_ROOT="$fetch/pdk"
export PYTHONPATH="$BUILD:${PYTHONPATH:-}"
cd "$BUILD/test" || exit 1
rm -rf sim_build results.xml diff.vcd rtl.vcd gl.vcd
make FST= COCOTB_TEST_MODULES="$mod" COCOTB_TEST_FILTER="$tst" PLUSARGS=+diffdump > rtl.log 2>&1
echo "RTL: $(grep -o 'TESTS=.*SKIP=[0-9]*' rtl.log | head -1)"
mv diff.vcd rtl.vcd
make GATES=yes FST= COCOTB_TEST_MODULES="$mod" COCOTB_TEST_FILTER="$tst" PLUSARGS=+diffdump 2>&1 \
  | grep -v "sorry: ifnone" > gl.log
echo "GL:  $(grep -o 'TESTS=.*SKIP=[0-9]*' gl.log | head -1)"
mv diff.vcd gl.vcd
"$LOOM_PY" "$repo/scripts/gl/diffvcd.py" rtl.vcd gl.vcd "$@"
