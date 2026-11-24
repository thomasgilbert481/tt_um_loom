#!/bin/bash
# Run the PHY-GL-SDF scenario groups of sdf_groups.txt on a fetched netlist with
# an Icarus-ready SDF, N groups at a time. Run in WSL.
#
#   scripts/gl/sdf_groups.sh FETCH_DIR SDF_FILE [N] [TAG ...]
#
# FETCH_DIR is a directory made by fetch.sh; SDF_FILE comes from sdf_filter.py
# (the typical corner, --cells-only, as docs/VERIFICATION.md records). With no
# TAG every group runs. Each group gets its own build directory under
# ${SDF_OUT:-$HOME/loom_sdf_groups}/<TAG>; a summary line per group goes to
# stdout and the full log to <TAG>.log there.
set -u
here=$(cd "$(dirname "$0")" && pwd)
fetch=${1:?FETCH_DIR}; sdf=${2:?SDF_FILE}; n=${3:-3}; shift 3 2>/dev/null || shift $#
out=${SDF_OUT:-$HOME/loom_sdf_groups}
mkdir -p "$out"
export here fetch sdf out
one() {
    line=$(awk -F'\t' -v t="$1" '$1 == t' "$here/sdf_groups.txt")
    [ -n "$line" ] || { echo "$1 unknown group"; return; }
    IFS=$'\t' read -r tag mod fw filt <<< "$line"
    (
        [ "$fw" != "-" ] && export LOOM_FW_ONLY=$fw
        export SDF=$sdf BUILD=$out/$tag
        bash "$here/run.sh" "$fetch" "$mod" "$filt"
    ) > "$out/$tag.log" 2>&1
    rc=$?
    res=$(grep -E "TESTS=" "$out/$tag/test/gl.log" 2>/dev/null | tail -1)
    echo "$tag exit $rc ${res:-no result}"
}
export -f one
if [ $# -gt 0 ]; then printf '%s\n' "$@"; else grep -v '^#' "$here/sdf_groups.txt" | cut -f1; fi |
    xargs -P "$n" -I{} bash -c 'one {}'
