#!/bin/bash
# Run the L4 formal proofs (docs/VERIFICATION.md L4). Exit status is non-zero
# if any property group does not end in the state formal/README.md records.
#
#     bash scripts/formal.sh              # every group, every task
#     bash scripts/formal.sh quick        # skip the bounded runs that an
#                                         # unbounded proof in the same group
#                                         # already subsumes
#     bash scripts/formal.sh fifo sched   # named groups only
#
# From Windows (Git Bash or the Claude Code Bash tool) run it through WSL:
#
#     MSYS_NO_PATHCONV=1 wsl -d Ubuntu -- bash /mnt/c/Users/Thoma/asic/tt_um_loom/scripts/formal.sh
#
# Use a script file like this one rather than a one-liner: shell variables in
# commands passed through wsl.exe are expanded (to nothing) before bash sees
# them.
#
# Two properties of VERIFICATION.md L4 do not hold as worded and are recorded
# as findings in formal/README.md (F-1, TIMER-1; F-4, ISO-1 with the host's
# debug port in play). Their tasks (timer:xfail, iso:dbg, iso:xfail) carry
# `expect fail` in the .sby file, so this script is green while the findings
# stand and goes red the day the behaviour changes without the record being
# updated. There is no separate list of expected failures to keep in sync.
# F-2 (PIN-1) was a finding too until D-023 fixed the design; its assertions
# are ordinary proofs now.
#
# Engine, mode, depth and result per property are recorded in
# formal/README.md; this script only decides pass or fail.

set -u
cd "$(dirname "${BASH_SOURCE[0]}")/.." || exit 1
# shellcheck disable=SC1091
source scripts/dev_env.sh

# Every group, cheapest first so a broken harness shows up in seconds. `iso`
# is last and is by far the most expensive: it is the only two-copy harness.
GROUPS_ALL="isa fifo timer pins spi isacore sched wait iso"

# What `quick` leaves out: bounded runs that an unbounded proof in the same
# group already subsumes, plus the long ISO-1 and WAIT-1 runs, which belong
# to the `formal-full` nightly of docs/VERIFICATION.md and not to
# `formal-quick` (iso:prove is about 33 minutes since F-7, iso:bmc 14 to
# 37, wait:bmc 3).
QUICK_SKIP="sched:bmc isacore:bmc spi:bmc iso:bmc iso:prove wait:bmc wait:cover"

mode="full"
case "${1:-}" in
  quick) mode="quick"; shift ;;
  full)  shift ;;
esac

groups="$*"
[ -n "$groups" ] || groups="$GROUPS_ALL"

if ! command -v sby >/dev/null 2>&1; then
  echo "formal: sby not found. scripts/dev_env.sh needs the OSS CAD Suite." >&2
  exit 1
fi

fail=0
results=""

# The ISA properties are generated from isa/isa.yaml, like every other
# generated file in this repo: check they are fresh before proving anything.
echo "=== generated ISA properties ==="
if "$LOOM_PY" formal/gen_isa_props.py --check; then
  results="$results
  isa:generated PASS"
else
  results="$results
  isa:generated FAIL (run \"\$LOOM_PY\" formal/gen_isa_props.py)"
  fail=1
fi

cd formal || exit 1

for g in $groups; do
  if [ ! -f "$g.sby" ]; then
    echo "formal: no such property group '$g' (formal/$g.sby)" >&2
    fail=1
    continue
  fi
  echo
  echo "=== $g ==="
  # The task names are the lines of the file's [tasks] section.
  tasks=$(sed -n '/^\[tasks\]/,/^\[options\]/p' "$g.sby" \
          | sed -n 's/^\([A-Za-z0-9_]\{1,\}\)[[:space:]]*$/\1/p')
  for t in $tasks; do
    case " $QUICK_SKIP " in
      *" $g:$t "*) if [ "$mode" = "quick" ]; then
                     echo "  $g:$t SKIP (quick)"
                     continue
                   fi ;;
    esac
    start=$(date +%s)
    if sby -f "$g.sby" "$t" >/dev/null 2>&1; then
      st="PASS"
    else
      st="FAIL"
      fail=1
    fi
    end=$(date +%s)
    echo "  $g:$t $st ($((end - start)) s)"
    results="$results
  $g:$t $st"
  done
done

echo
echo "=== formal summary ==="
echo "$results"
echo
if [ "$fail" -eq 0 ]; then
  echo "ALL FORMAL PROPERTY GROUPS AS RECORDED IN formal/README.md"
else
  echo "SOME FORMAL PROPERTY GROUPS DIFFER FROM formal/README.md"
fi
exit "$fail"
