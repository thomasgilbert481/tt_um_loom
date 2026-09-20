#!/bin/bash
# Summarise a hardening run: the numbers that decide whether it is good, and
# the ones that decide whether the next one will fit in six hours
# (docs/AREA.md, "Routing time is the binding constraint").
#
#   scripts/harden_report.sh RUN_ID [WORK_DIR]
#
# Needs `gh` (authenticated), so run it where gh lives (Git Bash here).
# Downloads the run's tt_submission artifact, and GDS_logs only if it has to,
# into WORK_DIR (default: a temporary directory that is kept, because the logs
# are large and worth reusing).
set -u
run="${1:?usage: harden_report.sh RUN_ID [WORK_DIR]}"
work="${2:-/c/Users/Thoma/asic/gl_runs/$run}"
repo="$(gh repo view --json nameWithOwner -q .nameWithOwner)"

echo "=== jobs ==="
gh run view "$run" --repo "$repo" --json conclusion,jobs \
  --jq '(.jobs[] | "\(.name) \(.conclusion // .status) \(.startedAt) -> \(.completedAt)")'

mkdir -p "$work"
if [ ! -f "$work/tt_submission/stats/metrics.csv" ]; then
  gh run download "$run" --repo "$repo" -n tt_submission -D "$work/sub" 2>/dev/null \
    && mv "$work/sub/tt_submission" "$work/tt_submission" && rm -rf "$work/sub"
fi
m="$work/tt_submission/stats/metrics.csv"
if [ ! -f "$m" ]; then
  echo "(no tt_submission artifact: the run did not get that far)"
else
  echo
  echo "=== sign-off ==="
  grep -E "^(design__instance__utilization|design__instance__count__stdcell|design__instance__area__stdcell|route__drc_errors|design__lvs_error__count|antenna__violating__nets|magic__illegal_overlap__count|magic__drc_error__count|power__total)," "$m"
  echo
  echo "=== timing ==="
  grep -E "^timing__(setup|hold)__ws__corner|^timing__setup_vio__count__corner|^timing__setup__tns__corner" "$m"
  echo
  echo "=== slew and cap ==="
  grep -E "^design__max_(slew|cap)_violation__count__corner" "$m"
fi

# Routing is the long pole, and its inputs are in the logs, not the metrics.
if [ -d "$work/runs" ] || [ -f "$work/GDS_logs.ok" ]; then :; else
  echo
  echo "(fetching GDS_logs for the routing numbers; this is a few hundred MB)"
  gh run download "$run" --repo "$repo" -n GDS_logs -D "$work" 2>/dev/null && touch "$work/GDS_logs.ok"
fi
if [ -d "$work/runs" ]; then
  echo
  echo "=== step runtimes (the five longest) ==="
  for d in "$work"/runs/*/[0-9]*/; do
    [ -f "$d/runtime.txt" ] && echo "$(cat "$d/runtime.txt") $(basename "$d")"
  done | sort -r | head -5
  echo
  echo "=== global routing congestion (what predicts the next run's routing time) ==="
  grep -A 9 "Final congestion report" "$work"/runs/*/*globalrouting*/*.log 2>/dev/null | tail -8
fi
