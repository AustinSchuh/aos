#!/bin/bash
#
# Reproduces the EngFlow cluster instability described in README.md by
# driving sustained high-concurrency remote execution, while sampling
# cluster state alongside it, then collecting evidence when it trips.
#
# Usage:
#   ./reproduce.sh [output-dir]
#
# Environment overrides:
#   TARGET             bazel target to hammer  (default: //aos/events:aio_test)
#   RUNS               --runs_per_test value   (default: 10000)
#   ENGFLOW_NAMESPACE  namespace to watch      (default: engflow)
#
# Expect a failure somewhere between run 2000 and 5000, typically within
# 2-5 minutes.  Five separate attempts all failed in that band; none
# completed.

set -uo pipefail

TARGET="${TARGET:-//aos/events:aio_test}"
RUNS="${RUNS:-10000}"
NS="${ENGFLOW_NAMESPACE:-engflow}"
OUT="${1:-engflow-repro-$(date -u +%Y%m%dT%H%M%SZ)}"

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CORE_PODS_RE="worker-cas|worker-default|scheduler"

mkdir -p "$OUT" || exit 1
OUT="$(cd "$OUT" && pwd)"

SAMPLER_PID=""
EVENTS_PID=""
cleanup() {
  [ -n "$SAMPLER_PID" ] && kill "$SAMPLER_PID" 2>/dev/null
  [ -n "$EVENTS_PID" ] && kill "$EVENTS_PID" 2>/dev/null
  return 0
}
trap cleanup EXIT INT TERM

echo "Target:     $TARGET"
echo "Runs:       $RUNS"
echo "Namespace:  $NS"
echo "Output:     $OUT"
echo

# Baseline before any load, so "was it already unhealthy?" is answerable.
echo "== baseline snapshot (before load) =="
"$HERE/collect_evidence.sh" "$OUT/baseline" >"$OUT/baseline-collect.log" 2>&1
echo "  saved to $OUT/baseline/"

echo "== starting cluster monitors =="
(
  while true; do
    echo "=== $(date -u +%Y-%m-%dT%H:%M:%SZ) ==="
    kubectl get pods -n "$NS" --no-headers 2>&1 | grep -E "$CORE_PODS_RE"
    sleep 5
  done
) >"$OUT/pod-state-samples.log" 2>&1 &
SAMPLER_PID=$!
echo "  pod sampler (5s interval), pid $SAMPLER_PID"

kubectl get events -n "$NS" --watch-only >"$OUT/events-stream.log" 2>&1 &
EVENTS_PID=$!
echo "  event stream, pid $EVENTS_PID"

echo
echo "== running load: bazel test $TARGET --runs_per_test=$RUNS =="
echo "   (this is the part that is expected to fail; tail $OUT/bazel.log)"
start_epoch="$(date +%s)"
bazel test "$TARGET" \
  --runs_per_test="$RUNS" \
  --test_output=errors \
  >"$OUT/bazel.log" 2>&1
BAZEL_EXIT=$?
end_epoch="$(date +%s)"
elapsed=$(( end_epoch - start_epoch ))

cleanup
SAMPLER_PID=""
EVENTS_PID=""

echo
echo "== bazel exited $BAZEL_EXIT after ${elapsed}s =="

# Highest run number bazel reported reaching, to show how far it got.
reached="$(grep -oE "run [0-9]+ of $RUNS" "$OUT/bazel.log" 2>/dev/null \
  | grep -oE '[0-9]+' | sort -n | tail -1)"
[ -n "$reached" ] && echo "   reached approximately run $reached of $RUNS"

echo
echo "== after-the-fact snapshot =="
"$HERE/collect_evidence.sh" "$OUT/after" >"$OUT/after-collect.log" 2>&1
echo "  saved to $OUT/after/"

echo
echo "== matching against known failure signatures =="
match() {
  local label="$1" pattern="$2"
  if grep -qE "$pattern" "$OUT/bazel.log" 2>/dev/null; then
    echo "  MATCH  $label"
    return 0
  fi
  return 1
}

any=1
match "(1) UNAVAILABLE: io exception" \
      "UNAVAILABLE: io exception" && any=0
match "(2/3) HazelcastInstanceNotActiveException" \
      "HazelcastInstanceNotActiveException" && any=0
match "(4) NOT_FOUND: Operation not found" \
      "NOT_FOUND: Operation not found" && any=0
match "(5) TCP connection refused to the VIP" \
      "finishConnect\(\.\.\) failed: Connection refused" && any=0
match "auth expired (DIFFERENT problem -- see README)" \
      "not authorized to use remote execution" && any=0

if [ "$any" != "0" ]; then
  if [ "$BAZEL_EXIT" = "0" ]; then
    echo "  none -- the run COMPLETED CLEANLY."
    echo "  Worth recording: this did not reproduce on this attempt."
  else
    echo "  none of the known signatures matched, but bazel still failed."
    echo "  This may be a new failure mode -- inspect $OUT/bazel.log."
  fi
fi

echo
echo "== pod restarts observed during the run =="
if [ -s "$OUT/pod-state-samples.log" ]; then
  # Column 4 of `kubectl get pods --no-headers` is the restart count.
  awk '/^===/ {ts=$2; next}
       NF >= 4 {
         name=$1; r=$4; sub(/[^0-9].*/, "", r);
         if (name in seen && seen[name] != r)
           printf "  %s  %s: %s -> %s\n", ts, name, seen[name], r;
         seen[name]=r;
       }' "$OUT/pod-state-samples.log" | head -40
  echo "  (full timeline: $OUT/pod-state-samples.log)"
else
  echo "  no samples captured"
fi

echo
echo "Done.  Everything in: $OUT/"
echo "  bazel.log               client-side failure"
echo "  pod-state-samples.log   pod/restart timeline during the run"
echo "  events-stream.log       kubelet probe failures and kills"
echo "  baseline/ and after/    full cluster snapshots either side"

exit "$BAZEL_EXIT"
