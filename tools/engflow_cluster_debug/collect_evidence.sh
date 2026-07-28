#!/bin/bash
#
# Read-only evidence collector for the EngFlow cluster instability
# described in README.md.  Never modifies cluster state -- safe to run at
# any time, including while a build is in flight.
#
# Usage:
#   ./collect_evidence.sh [output-dir]
#
# Environment overrides:
#   ENGFLOW_NAMESPACE  namespace to inspect      (default: engflow)
#   ENGFLOW_VIP        LoadBalancer VIP to probe (default: 192.168.11.10)
#   ENGFLOW_VIP_PORT   port on that VIP          (default: 443)

# Deliberately no `set -e`: a single failing kubectl (a pod that just went
# away, a metrics-server hiccup) must not abort the rest of the
# collection.  Individual failures are captured in their output files.
set -uo pipefail

NS="${ENGFLOW_NAMESPACE:-engflow}"
VIP="${ENGFLOW_VIP:-192.168.11.10}"
VIP_PORT="${ENGFLOW_VIP_PORT:-443}"
OUT="${1:-engflow-evidence-$(date -u +%Y%m%dT%H%M%SZ)}"

# Pods whose restarts/logs are most relevant to this issue.
CORE_PODS_RE="worker-cas|worker-default|scheduler"

mkdir -p "$OUT" || exit 1
echo "Collecting evidence into $OUT/"

# Runs a command, saving both the command line and its output/errors.
run() {
  local name="$1"
  shift
  echo "  ${name}"
  {
    echo "\$ $*"
    echo
    "$@" 2>&1
  } >"$OUT/$name"
}

echo "== cluster and namespace state =="
run pods.txt                   kubectl get pods -n "$NS" -o wide
run nodes.txt                  kubectl get nodes -o wide
run services.txt               kubectl get svc -n "$NS" -o wide
run events.txt                 kubectl get events -n "$NS" --sort-by=.lastTimestamp
run top-nodes.txt              kubectl top nodes
run top-pods.txt               kubectl top pods -n "$NS"

echo "== the external LoadBalancer (BGP/MetalLB suspect) =="
run describe-svc-external.txt  kubectl describe svc external -n "$NS"
run endpoints-external.txt     kubectl get endpoints external -n "$NS"
run svc-external.yaml          kubectl get svc external -n "$NS" -o yaml

# MetalLB lives in its own namespace and the name varies by install.
echo "== metallb (namespace auto-detected) =="
METALLB_NS="$(kubectl get pods -A 2>/dev/null \
  | grep -i metallb | head -1 | awk '{print $1}')"
if [ -n "$METALLB_NS" ]; then
  echo "  (found in namespace: $METALLB_NS)"
  run metallb-pods.txt         kubectl get pods -n "$METALLB_NS" -o wide
  for pod in $(kubectl get pods -n "$METALLB_NS" -o name 2>/dev/null \
                 | grep -i speaker); do
    safe="$(basename "$pod")"
    run "metallb-log-${safe}.txt" \
        kubectl logs -n "$METALLB_NS" "$pod" --tail=500
  done
else
  echo "  metallb pods not found -- record that fact" \
    >"$OUT/metallb-NOT-FOUND.txt"
  echo "  not found (noted)"
fi

echo "== per-pod detail for core components =="
for pod in $(kubectl get pods -n "$NS" --no-headers 2>/dev/null \
               | grep -E "$CORE_PODS_RE" | awk '{print $1}'); do
  run "describe-${pod}.txt" kubectl describe pod -n "$NS" "$pod"

  # A non-zero restart count means there is a --previous log worth having;
  # that log is the only place a kill reason (crash vs. graceful shutdown)
  # is visible, and it is lost on the next restart.
  restarts="$(kubectl get pod -n "$NS" "$pod" --no-headers 2>/dev/null \
                | awk '{print $4}' | sed 's/[^0-9].*//')"
  if [ -n "$restarts" ] && [ "$restarts" != "0" ]; then
    echo "  ${pod} has ${restarts} restart(s) -- grabbing previous logs"
    run "previous-log-${pod}.txt" \
        kubectl logs -n "$NS" "$pod" --previous --all-containers=true --tail=2000
  fi
done

echo "== direct VIP reachability (no gRPC, no application layer) =="
{
  # Connect only -- deliberately does NOT read.  Reading from a TLS port
  # blocks until the server responds, and the server is waiting for a
  # ClientHello that a raw socket never sends, so a read-based probe
  # times out even against a perfectly healthy endpoint.  (That false
  # positive is easy to mistake for a black-hole; don't reintroduce it.)
  echo "\$ timeout 5 bash -c 'exec 3<>/dev/tcp/${VIP}/${VIP_PORT}'"
  echo
  timeout 5 bash -c "exec 3<>/dev/tcp/${VIP}/${VIP_PORT}" 2>&1
  code=$?
  echo "exit=${code}"
  case "$code" in
    0)   echo "interpretation: TCP connect OK" ;;
    1)   echo "interpretation: connection REFUSED (nothing listening on that path)" ;;
    124) echo "interpretation: TIMED OUT -- black-holed" ;;
    *)   echo "interpretation: unexpected exit code" ;;
  esac

  # A successful TCP connect only proves something accepted the socket;
  # completing a TLS handshake proves the backend behind the VIP is
  # actually serving.
  echo
  if command -v openssl >/dev/null 2>&1; then
    echo "\$ openssl s_client -connect ${VIP}:${VIP_PORT}"
    timeout 8 openssl s_client -connect "${VIP}:${VIP_PORT}" \
      -servername "${ENGFLOW_SNI:-engflow.spacecookies.dev}" </dev/null 2>&1 \
      | grep -E "CONNECTED|subject=|Verify return code|errno|no peer certificate"
    echo "tls_probe_exit=$?"
  else
    echo "(openssl unavailable -- TLS handshake not verified)"
  fi
} >"$OUT/vip-probe.txt"
echo "  $(grep '^exit=' "$OUT/vip-probe.txt")"

echo
echo "Done.  Evidence in: $OUT/"
echo "Start with: describe-svc-external.txt (BGP flap), then any previous-log-*.txt"
