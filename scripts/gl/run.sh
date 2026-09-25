#!/bin/bash
# Run the cocotb tests at gate level on a netlist fetched by fetch.sh, in a
# Linux-side copy of the repo (a /mnt/c build is slow). Run inside WSL.
#
#   scripts/gl/run.sh FETCH_DIR [MODULES] [FILTER] [PLUSARGS]
#
# MODULES defaults to test/Makefile's gate-level list; FILTER is a
# COCOTB_TEST_FILTER regex. The copy is refreshed from the working tree on
# every call, so edited tests are picked up; the netlist stays the fetched one.
# Output goes to $BUILD/test/gl.log (BUILD defaults to ~/loom_gl_<run id>).
set -u
fetch="${1:?usage: run.sh FETCH_DIR [MODULES] [FILTER] [PLUSARGS]}"
mods="${2:-}"
filt="${3:-}"
plus="${4:-}"
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$repo/scripts/dev_env.sh" >/dev/null 2>&1
run_id="$(cat "$fetch/run_id" 2>/dev/null || echo local)"
BUILD="${BUILD:-$HOME/loom_gl_$run_id}"
mkdir -p "$BUILD"
for d in src test macro tools isa firmware; do
  rm -rf "${BUILD:?}/$d"
  cp -r "$repo/$d" "$BUILD/$d"
done
cp "$fetch/tt_submission/tt_um_loom.v" "$BUILD/test/gate_level_netlist.v"
export PDK_ROOT="$fetch/pdk"
export PYTHONPATH="$BUILD:${PYTHONPATH:-}"
cd "$BUILD/test" || exit 1
rm -rf sim_build results.xml
# grep -v drops Icarus's "sorry: ifnone" notes about the cell specify blocks.
make GATES=yes ${mods:+COCOTB_TEST_MODULES=$mods} ${filt:+COCOTB_TEST_FILTER=$filt} \
     ${plus:+PLUSARGS=$plus} ${SDF:+SDF=$SDF} 2>&1 | grep --line-buffered -v "sorry: ifnone" > gl.log
grep -E "TESTS=|FAIL " gl.log
! grep -q "<failure" results.xml 2>/dev/null
