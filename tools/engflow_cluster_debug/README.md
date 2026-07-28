# EngFlow cluster instability — investigation notes and reproducer

Sustained high-concurrency remote-execution load against
`engflow.spacecookies.dev` reliably fails partway through with a variety
of gRPC/Hazelcast/connection errors. Five separate attempts to run a
10000-iteration test stress run all failed between roughly run 2000 and
run 5000; none completed.

This is **not** an AOS code issue — every failure is on the
remote-execution path, and the same tests pass consistently at lower
iteration counts and under `--config=asan`. This directory holds what was
found, plus scripts to reproduce it and collect evidence.

- `collect_evidence.sh` — read-only snapshot of cluster state, including
  the MetalLB speaker logs that carry the key evidence. Safe to run any
  time; the fault is visible with no load running at all.
- `reproduce.sh` — runs the load, monitors the cluster concurrently, and
  captures evidence when it trips.

## TL;DR

**Node-to-node networking between the four cluster nodes is unreliable,
and `server1868-2` (192.168.10.102) is by far the worst offender.**
Everything else appears to be downstream of that.

MetalLB's speaker logs show its memberlist gossip (port 7946, on the
`192.168.10.0/24` node network) repeatedly failing with 1-second TCP ping
timeouts, causing nodes to declare each other dead:

```
memberlist: Failed fallback TCP ping: timeout 1s:
  read tcp 192.168.10.104:38526->192.168.10.102:7946: i/o timeout
memberlist: Suspect server1868-2 has failed, no acks received
```

Across the four speaker pods, suspicion counts are lopsided:

| Node suspected | Times |
| -------------- | ----- |
| `server1868-2` | 33    |
| `server1868-3` | 9     |
| `server1868-4` | 4     |
| `server1868-1` | 1     |

That single underlying fault explains all three observed layers:

1. **MetalLB** loses memberlist quorum → withdraws and re-announces the
   `external` VIP's BGP route → route flaps between nodes. With
   `externalTrafficPolicy: Local` and only 2 of 4 scheduler pods
   registered as endpoints, any window where the announcing node has no
   local endpoint black-holes client connections.
2. **Hazelcast** (used by both the scheduler's service discovery and
   CAS's `ReplicaTracker`) does its own node-to-node communication over
   the same network, and stalls the same way — one scheduler was caught
   blocked 9.385s on a socket read.
3. **kubelet** kills pods whose liveness probes miss a **1-second**
   timeout with a **single-failure** threshold. A 9-second stall trips
   that trivially, so healthy-but-stalled pods get killed, which surfaces
   to bazel as `HazelcastInstanceNotActiveException` or `NOT_FOUND: Operation not found` depending on timing.

The timeline confirms the ordering — memberlist failures at 16:40:56
through 16:41:30, scheduler killed at 16:41:21, 9.385s stall logged at
16:41:33, BGP re-announcement cascade at 16:41:34.

Throttling bazel's own remote-exec concurrency (`--jobs=20`) did _not_
prevent recurrence, consistent with a network fault rather than client
load being the driver.

**Possibly relevant:** `server1868-2` — the most-suspected node — is also
the only node still running kernel **7.0.10**; the other three run
**7.0.12**. This is correlation only, and it may just mean that node has
not been rebooted recently. But given it is the standout on both axes, it
is worth ruling in or out early.

## Symptoms observed (client side)

All hit during otherwise-identical `--runs_per_test=10000` invocations of
`bazel test //aos/events:aio_test`, at various points between run ~2000
and run ~5000:

1. `io.grpc.StatusRuntimeException: UNAVAILABLE: io exception` inside
   `GrpcRemoteExecutor.getOperationResponse`.
2. `com.hazelcast.core.HazelcastInstanceNotActiveException: Failed to serialize 'com.engflow.type.CompactDigest'` inside
   `ReplicaTracker.get()` (`com.engflow.re.cas.distributed`).
3. `com.hazelcast.core.HazelcastInstanceNotActiveException: State: SHUT_DOWN Operation: class com.hazelcast.map.impl.operation.GetOperation`
   from `DistributedActionStatusTracker.findAssignedOwner`.
4. `io.grpc.StatusRuntimeException: NOT_FOUND: Operation not found: <op-id>`
   — scheduler lost track of an in-flight action, most plausibly because
   the replica owning it was killed mid-flight.
5. `java.net.ConnectException: finishConnect(..) failed: Connection refused: engflow.spacecookies.dev/192.168.11.10:443`. Hit twice,
   including once after the fleet had shown zero restarts for 9+ minutes
   beforehand — pod health does not predict this.

(5) is the most informative one: it is a pure TCP-level refusal to the
VIP, with no gRPC or application layer involved at all, which is what
points at the routing layer rather than at any backend service.

Note that the VIP is _usually_ reachable — `openssl s_client` completes a
valid handshake against it at rest. The failures are intermittent, which
is consistent with the route flapping described below rather than with a
persistently broken endpoint.

## Root cause chain

### 0. Node-to-node network faults (the actual root)

From the MetalLB speaker logs (`collect_evidence.sh` pulls these
automatically into `metallb-log-*.txt`):

```
{"component":"Memberlist","level":"error","ts":"2026-07-28T16:40:56Z",
 "msg":"memberlist: Failed fallback TCP ping: timeout 1s:
        read tcp 192.168.10.104:38526->192.168.10.102:7946: i/o timeout"}
{"component":"Memberlist","level":"info","ts":"2026-07-28T16:40:56Z",
 "msg":"memberlist: Suspect server1868-2 has failed, no acks received"}
```

These are plain TCP pings between nodes on the node network
(`192.168.10.0/24`, port 7946) timing out after a full second. Node IPs:

| Node           | Internal IP    | Kernel                 |
| -------------- | -------------- | ---------------------- |
| `server1868-1` | 192.168.10.101 | 7.0.12+deb13-amd64     |
| `server1868-2` | 192.168.10.102 | **7.0.10**+deb13-amd64 |
| `server1868-3` | 192.168.10.103 | 7.0.12+deb13-amd64     |
| `server1868-4` | 192.168.10.104 | 7.0.12+deb13-amd64     |

Between 10 and 28 memberlist failure lines appear in each of the four
speaker logs, concentrated on `server1868-2`. Anything doing node-to-node
communication over this network — MetalLB memberlist, Hazelcast, kubelet
probes traversing it — is exposed to the same faults.

**This is the thing to fix.** Everything below is downstream.

### 1. MetalLB BGP route flapping on the `external` Service

```
$ kubectl describe svc external -n engflow
LoadBalancer Ingress:     192.168.11.10 (VIP)
Port:                     https  443/TCP
TargetPort:               8080/TCP
External Traffic Policy:  Local
Endpoints:                172.31.34.78:8080,172.31.219.206:8080   # only 2 of 4 schedulers
Events:
  Normal  nodeAssigned  16s (x454 over 31d)  metallb-speaker  announcing from node "server1868-2"
  Normal  nodeAssigned  10s (x212 over 18d)  metallb-speaker  announcing from node "server1868-3"
  Normal  nodeAssigned   9s (x229 over 18d)  metallb-speaker  announcing from node "server1868-4"
  Normal  nodeAssigned   9s (x209 over 14d)  metallb-speaker  announcing from node "server1868-1"
```

The VIP's BGP announcement has moved between all four nodes **1104 times
over the past several weeks**, and was still re-announcing every 9-16
seconds at the moment of capture — with no load running. This looks like
a standing condition, not something the stress test created.

`externalTrafficPolicy: Local` means a node forwards only to _locally
running_ endpoint pods and will not hairpin to another node. Only 2 of 4
scheduler pods were registered as endpoints. If the route is announced
from a node whose local scheduler pod isn't a registered endpoint (or
has none), connections to the VIP via that path fail. That matches
symptoms 5 and 6.

### 2. Hazelcast-dependent scheduler internals stalling

Two scheduler pods restarted during testing. Their `--previous` logs:

`scheduler-...-s9xhd` — served normal traffic right up to its final log
line, then:

```
Sending shutdown signal to process
waiting up to PT15S to shut down...
shutdown successful: 0
```

A clean, graceful shutdown (exit 0). It was _asked_ to stop — almost
certainly a failed liveness probe triggering a kubelet kill — it did not
crash.

`scheduler-...-wf4jt` — the actual smoking gun:

```
W [ProductionExecutorProvider.logSlowTasks] Found ongoing slow task on
  engflow-scheduler-service-discovery: runtime: 9.385s, state: RUNNABLE
	at java.base/sun.nio.ch.SocketDispatcher.read0(Native Method)
```

An internal service-discovery task (thread names elsewhere in the log are
`hz.*`, i.e. Hazelcast) blocked on a raw socket read for 9.385s and
counting. A liveness probe anywhere near as tight as the worker config
below would trip during such a stall.

Also present during otherwise-normal operation in both logs, unrelated to
shutdown: `[PoolGroups.isOom] OOM detection failed, poolId unknown: Pool{name=default}`. Logged at low severity; probably not causal, but
unexplained.

### 3. Worker liveness probes are extremely tight

```
$ kubectl describe pod -n engflow <worker-default-pod>
Liveness:  http-get http://:liveness-port/healthz delay=0s timeout=1s period=10s #success=1 #failure=1
Requests:
  cpu:     1
  memory:  4Gi
```

**1-second timeout, single-failure threshold, on a 1-CPU request.** Any
transient scheduling delay, GC pause, or blocked socket read that pushes
the health endpoint past 1 second kills the pod immediately, with zero
tolerance for jitter. Under thousands of concurrent test-runner
containers, transient CPU contention alone could plausibly do this.

The scheduler pods' own probe config was not checked — worth doing.

### 4. Possibly unrelated, noted for completeness

- `engflow-cluster-vector-*` (Vector.dev log shippers) have **no resource
  limits** and sit at 22-39GB RSS each. No OOMKill evidence found, but
  unbounded memory on a log shipper is worth tightening regardless.
- `engflow-cluster-victorialogs-server-0` had a high historical restart
  count (68 over 9h), but its most recent restart was not time-correlated
  with the incident window.

## Reproducing

Check whether the fault is present right now, with no load at all:

```
./collect_evidence.sh
```

Three things to look at in the output directory, in order:

```bash
# 1. The root cause -- node-to-node ping failures and who they blame.
grep -h "Suspect.*has failed" metallb-log-*.txt | sort | uniq -c | sort -rn

# 2. The BGP flapping it causes.  If these nodeAssigned events are only
#    seconds old with nothing running, it is independent of load.
grep -A15 '^Events:' describe-svc-external.txt

# 3. Whether the VIP is reachable this instant.
cat vip-probe.txt     # exit=0 connected, 1 refused, 124 black-holed
```

On the run captured while writing this, with no build in flight, (1) and
(2) both showed the fault: 47 memberlist suspicions and `nodeAssigned`
events only seconds old. The VIP itself connected fine at that moment —
it is intermittent, so a single passing probe does not clear it.

A caution on (3), learned the hard way: probe the VIP by _connecting
only_, never by reading. A read against a TLS port blocks waiting for a
handshake a raw socket never initiates, so `cat < /dev/tcp/host/443`
times out against a perfectly healthy endpoint and looks exactly like a
black-hole. `collect_evidence.sh` uses `exec 3<>/dev/tcp/...` plus an
`openssl s_client` handshake for this reason.

To reproduce the full cascade under load:

```
./reproduce.sh
```

This runs `bazel test //aos/events:aio_test --runs_per_test=10000` while
sampling pod state every 5s and streaming cluster events, then collects a
full evidence snapshot when it finishes or fails. Override with e.g.
`TARGET=//some:other_test RUNS=5000 ./reproduce.sh`. Any test that takes
a few seconds per invocation works; the point is sustaining ~100
concurrent remote actions for several minutes.

Expect a failure somewhere between run 2000 and 5000, taking roughly 2-5
minutes. The script prints which of the six known signatures it matched.

To probe the VIP directly during a failure window — this is the cleanest
separation of "network/LB problem" from "backend problem", since it
involves no gRPC or application code at all:

```bash
# Connect only.  Do not read: see the caution above.
timeout 5 bash -c 'exec 3<>/dev/tcp/192.168.11.10/443'; echo "exit=$?"

# And confirm the backend actually serves, not just that a socket opened:
openssl s_client -connect 192.168.11.10:443 \
  -servername engflow.spacecookies.dev </dev/null 2>&1 | head -20
```

## Suggested next steps

In rough priority order:

1. **Investigate `server1868-2`'s network path.** It is suspected 33
   times versus 9/4/1 for the others. Check NIC/link counters
   (`ip -s link`, `ethtool -S <iface>`), the switch port it is on, MTU
   consistency across nodes, and any bonding/VLAN config that differs
   from its peers. A one-second TCP ping timeout on a local LAN is a real
   fault, not tuning noise.
2. **Rule the kernel discrepancy in or out.** `server1868-2` is on 7.0.10
   while the others are on 7.0.12. Either reboot/upgrade it to match, or
   confirm the difference is irrelevant — right now it is an unexplained
   variable on the exact node that misbehaves most.
3. **Reproduce independently of Kubernetes.** A sustained
   `ping -f` / `iperf3` / repeated `nc -z <node> 7946` between node pairs
   (especially `.104 → .102`) during a quiet period would confirm whether
   the loss exists below the cluster layer entirely.
4. **Loosen the liveness probes.** Regardless of root cause, `timeout=1s`
   with `failureThreshold=1` on a 1-CPU request gives zero tolerance for
   jitter and converts brief stalls into pod kills, which is what turns a
   network blip into a failed build. Consider `timeoutSeconds: 5` and
   `failureThreshold: 3`.
5. **Check whether `externalTrafficPolicy: Local` with only 2/4 scheduler
   endpoints is intended.** If not required (e.g. for source-IP
   preservation), switching to `Cluster` would make the VIP tolerate route
   movement instead of black-holing on it.

Still unchecked:

- Whether the scheduler's service-discovery Hazelcast cluster is the
  _same_ cluster CAS's `ReplicaTracker` uses. If so, symptoms 1-4 tie
  together even more tightly.
- The scheduler pods' own liveness probe config (only `worker-default`'s
  was inspected).
- EngFlow release notes for `2.164.1` (`engflow-worker` and
  `engflow-scheduler`) for known issues matching this profile.
- Whether `[PoolGroups.isOom] OOM detection failed, poolId unknown` —
  logged steadily during normal operation — indicates a real
  misconfiguration.

## Environment

- Cluster: 4 nodes, `server1868-1..4`, Kubernetes v1.33.13, all
  control-plane.
- Namespace: `engflow`. Images:
  `953333998565.dkr.ecr.us-west-2.amazonaws.com/engflow-{worker,scheduler}:2.164.1`.
- Components: `scheduler` (4 replicas), `worker-cas-0..3` (StatefulSet),
  `worker-default` (4 replicas, `worker` + docker-in-docker sidecar),
  plus `analyzer`, `grafana`, `invocation-index-pg-cluster`, `seaweedfs`,
  and the Vector/VictoriaLogs/VictoriaMetrics observability stack.
- Client: bazel 8.5.0 with
  `--credential_helper=engflow.spacecookies.dev=<engflow_auth binary>`.

Note: the EngFlow auth token expired mid-investigation, producing a
_different_ and unrelated failure
(`Remote execution is not supported by the remote server, or the current account is not authorized`). Re-authenticating with `engflow_auth login --store=file <cluster-url>` fixed that. If you see that message rather
than the ones above, it is an auth problem, not this problem — and note
that after re-auth you must `bazel shutdown`, because the running bazel
server caches the expired credential state.
