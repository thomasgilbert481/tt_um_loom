#!/bin/bash
# Fetch what CI's gl_test job uses for one hardening run, so the gate-level
# tests can be run and debugged locally (docs/tt_cmos5l_facts.md section 12):
#   - the netlist, from the run's `tt_submission` artifact (tt_um_loom.v);
#   - the three IHP cell-model files, from IHP-Open-PDK at the commit the run
#     pinned (tt_submission/pdk.json, PDK_VERSION), laid out under pdk/ the
#     way test/Makefile expects PDK_ROOT.
# Needs `gh` (authenticated) and `curl`; run it where `gh` lives (Git Bash on
# this machine), then run.sh / diff.sh inside WSL.
#
#   scripts/gl/fetch.sh RUN_ID OUT_DIR
set -eu
run="${1:?usage: fetch.sh RUN_ID OUT_DIR}"
out="${2:?usage: fetch.sh RUN_ID OUT_DIR}"
repo="$(gh repo view --json nameWithOwner -q .nameWithOwner)"
mkdir -p "$out"
rm -rf "$out/tt_submission"
gh run download "$run" --repo "$repo" -n tt_submission -D "$out/sub"
mv "$out/sub/tt_submission" "$out/tt_submission"
rm -rf "$out/sub"
pdk_ver="$(sed -n 's/.*"PDK_VERSION": *"\([0-9a-f]*\)".*/\1/p' "$out/tt_submission/pdk.json")"
[ -n "$pdk_ver" ] || { echo "no PDK_VERSION in pdk.json" >&2; exit 1; }
base="https://raw.githubusercontent.com/IHP-GmbH/IHP-Open-PDK/$pdk_ver/ihp-sg13cmos5l/libs.ref"
for f in sg13cmos5l_stdcell/verilog/sg13cmos5l_stdcell.v \
         sg13cmos5l_stdcell/verilog/sg13cmos5l_udp.v \
         sg13cmos5l_io/verilog/sg13cmos5l_io.v; do
  mkdir -p "$out/pdk/ihp-sg13cmos5l/libs.ref/$(dirname "$f")"
  curl -sSfL -o "$out/pdk/ihp-sg13cmos5l/libs.ref/$f" "$base/$f"
done
echo "$run" > "$out/run_id"
echo "netlist: $out/tt_submission/tt_um_loom.v (commit $(sed -n 's/.*"commit": *"\([0-9a-f]\{7\}\).*/\1/p' "$out/tt_submission/commit_id.json"))"
echo "PDK_ROOT: $out/pdk (IHP-Open-PDK $pdk_ver)"
