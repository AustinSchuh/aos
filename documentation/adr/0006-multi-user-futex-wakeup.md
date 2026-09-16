# ADR 0006: Futex wakeups for shared-memory queues — running AOS as more than one user

## Status

Accepted. Implemented in AOS, in the same change as this document: `ipc_lib::WakeupKind` and `--queue_wakeup` (`aos/ipc_lib/lockless_queue.h`), the futex branch of `LocklessQueueWakeUpper::Wakeup()`, the futex kind of `ThreadSignalReceiver` (`aos/ipc_lib/thread_signal.h`) with the io_uring backend's native waits in `aio_linux.cc` and the ring-fd implementation for the epoll and libuv backends in `aos/events/futex_wait_ring.cc`, `--io_uring_setup`, the `QueueEventLoop` integration, the `.v9` queue files, `--permissions` applied exactly, `starterd --shm_group`, the `hidepid` refusal (`aos/util/proc_stat.h`), and `--skip_queue_uid_check`. Written against `wpilib-release-2027` (51ff3f813). The one thing not verified here is the cross-user test itself, which needs root; it skips otherwise.

## Context

### The rule that binds a queue to one user

A shared-memory queue wakes its watchers with a thread-directed realtime signal, and signals have a permission model that is about users, not about access to the memory. `kill(2)`:

> For a process to have permission to send a signal, it must either be privileged (under Linux: have the `CAP_KILL` capability in the user namespace of the target process), or the real or effective user ID of the sending process must equal the real or saved set-user-ID of the target process.

`rt_tgsigqueueinfo(2)` is under the same rule. So a wakeup from user A to user B is refused with `EPERM` unless A holds `CAP_KILL`, and the wakeup back from B to A has to pass on its own. Everything below follows from that one paragraph.

### What a wakeup is today

When a sender publishes on a channel, `LocklessQueueWakeUpper::Wakeup()` (`aos/ipc_lib/lockless_queue.cc:817`) walks the channel's `Watcher` slots, snapshots each live one's `(pid, tid, priority)`, sorts them by priority, boosts its own `SCHED_FIFO` priority to the highest watcher's, and sends each watcher one signal (`rt_tgsigqueueinfo`, `thread_signal_linux.cc:26`). On the receiving side, `QueueEventLoop` blocks `kWakeupSignal` and owns a `signalfd` (`ThreadSignalReceiver`), which is the fd the event loop actually waits on: `IoUringImpl` holds a multishot poll on it, `EpollImpl` an `OnReadable()`, and the libuv guest loop of ADR 0004 a `uv_poll`. A wakeup carries no information; the loop drains the signalfd and rescans every watcher (`QueueEventLoop::HandleEvent()`, `queue_event_loop.cc:928`).

The kernel enforces the permission rule at every send, and `ThreadSignalSender::Signal()` dies on `EPERM` rather than lose a wakeup. What AOS adds is an earlier, readable failure: `InitializeLocklessQueueMemory()` records `GetUserId()` in the queue the first time it is initialized, and a later process with a different one dies at attach naming both users (`lockless_queue.cc:701`). That check implements nothing; it exists so that a mismatch is a message at startup rather than a watcher that never wakes. `GetUserId()` (`aos/realtime_linux.cc:287`) picks whichever of a process's uids peers can signal, and documents the trick that lets a privileged process fit: keep the real uid, change only the effective one. `starterd --user` does exactly that (`starterd.cc:74-90`) so that its children, which it starts as other users (`subprocess.cc:1090`), can still reach it.

So "AOS as multiple users" — a system daemon and a user's robot program as genuinely different users, with file permissions between them — is not a permissions problem on the files. It is that the wakeup goes through the one kernel object whose permission is about users rather than about who can map the memory.

The rule is also checked against the target's credentials _at the moment of each send_, which makes a process that changes uid, even briefly, a hazard to everyone that wakes it. starterd does exactly that: `Sudo` (`subprocess.cc:114`) does `setresuid(0, 0, 0)` around the cgroup `mkdir`, `rmdir` and `tasks` writes when an application starts or stops, and glibc's `setresuid` is process-wide, so for those syscalls the whole of starterd, event loop thread included, has real and saved uid 0. Signals already queued before the switch are unaffected: pending signals survive a credential change, and the signalfd read has no permission check. A wakeup _sent_ inside that window is another matter. A sender running as the application user matches neither of starterd's uids, `rt_tgsigqueueinfo` returns `EPERM`, and since `ThreadSignalSender::Signal()` treats everything but `ESRCH` as fatal, the sending application dies. Nothing is dropped silently, and starterd never notices; the crash lands on whichever application sent a starter RPC at the wrong moment. Moving the cgroup work to a helper thread and using the raw per-thread `setresuid` syscall would narrow it, but the escalation runs on the thread the watchers are aimed at, so it cannot close it. A `FUTEX_WAKE` has no credential check at all.

There is a third defect in the signal path worth naming because the replacement removes it: `RLIMIT_SIGPENDING` is a per-user cap on queued realtime signals, so an unrelated thread saturating it makes `rt_tgsigqueueinfo` fail with `EAGAIN` and `ThreadSignalSender::Signal()` has to die rather than lose the wakeup (`thread_signal_linux.cc:34-49`, [aos#14](https://github.com/RealtimeRoboticsGroup/aos/issues/14)).

### What the rest of the queue already does across users

Every other piece of the queue is keyed on the shared memory, not on who maps it:

- The queue setup lock and each `RobustOwnershipTracker` are PI futexes with robust-list death notification. The kernel finds the PI owner by tid (`attach_to_pi_owner()` in `kernel/futex/pi.c`) and has had no credential check there since 2.6.36; the robust list is walked on thread exit regardless of who else waits. Verified against current mainline while writing this.
- Death detection reads `/proc/<pid>/task/<tid>/stat` (`robust_ownership_tracker_linux.cc:18`, `util/proc_stat.cc:32`), which is world-readable unless `/proc` is mounted with `hidepid`.
- The files under `--shm_base` are opened with `--permissions` (default `0770`) through the process umask (`shm_mapping_linux.cc:39`, `shm_event_loop.cc:9-20`).

### What the kernel offers now

Linux 6.7 added futex operations to io_uring: `IORING_OP_FUTEX_WAIT`, `IORING_OP_FUTEX_WAKE` and `IORING_OP_FUTEX_WAITV` (`io_uring/futex.c`). A wait is one-shot, accepts shared (non-`FUTEX2_PRIVATE`) futexes, completes with `res == 0` when woken, `-EAGAIN` when the word did not hold the expected value at arm time, and `-ECANCELED` on cancel. An io_uring waiter sits in the same futex hash bucket as a `futex(2)` waiter, so a plain `FUTEX_WAKE` from any process that maps the page wakes it. And an io_uring instance's fd is pollable: `io_uring_poll()` reports `EPOLLIN` when completions are available or, on a `DEFER_TASKRUN` ring, when task work is pending (`ctx->poll_wq`, since 6.3).

ADR 0001 already committed the io_uring backend to a 6.1 enforced floor with 6.12 recommended, and `//tools/platforms/io_uring` names 6.12 as the line for defaulting to it (`tools/platforms/BUILD:71-96`).

The machines this runs on are mostly on 6.12 or 4.16, with a few in between. The futex wakeup needs io_uring's futex operations on either `Aio` backend — the epoll backend included, because the fd it polls for this is an io_uring's — so 6.7 is its floor everywhere. Everything below 6.7 runs the epoll backend with signals, single-user. That is a decision rather than a gap: 4.16 in particular predates io_uring (5.1), `futex_waitv(2)` (5.16) and the futex2 syscalls (6.7) entirely, so no futex-backed fd exists there short of a helper thread per waited word, and multiple users is not a requirement on old kernels.

## Decision

Replace the signal with a futex wait on the queue's existing `next_queue_index`, through io_uring, on every Linux kernel that has io_uring futex operations.

1. **The futex word is the channel's `next_queue_index`.** It already moves once per send, after the message is visible, which is exactly what a futex protocol needs. No field is added to the queue memory. A sender issues one `FUTEX_WAKE` per send, unconditionally; the per-watcher signal loop, the priority sort and the PI boost go away with it.
2. **Watchers wait on that word through `Aio`.** `IoUringImpl` submits one `IORING_OP_FUTEX_WAIT` per watched channel on its own ring. The epoll backend and the libuv guest loop get the same thing through a small io_uring per event loop that holds only futex waits and exposes its ring fd — the file descriptor an fd-driven loop can wait on.
3. **The wakeup kind is a per-platform build default with a flag override**, the way `--aio_backend` is, and is recorded in the queue header at initialization so two kinds never share a queue. The queue file suffix bumps from `.v8` to `.v9`. On kernels below 6.7 and on macOS and Windows, the signal path stays exactly as it is.
4. **The uid check stays for the signal kind, as the early diagnostic it is, and gains an override** for a deployment that makes signals work across users itself, with `CAP_KILL` and the real-uid trick. The futex kind has no uid check: file permissions are the access control.
5. **`--permissions` is applied explicitly**, not through the umask, and `--shm_base` is expected to be a group-shared, setgid directory owned by the deployment.
6. **A `/proc` mounted with `hidepid` refuses futex-kind queues.**

Not decided here: cross-user wakeups on macOS or Windows, and containers (pid namespaces), which the PI futexes already do not cross.

## Design

### The word: `next_queue_index`

`LocklessQueueMemory::next_queue_index` (`lockless_queue_memory.h`) is a 32-bit atomic that a sender advances once per message. The property that makes it usable as a futex word is _when_ it advances: after the message is visible, never before. On the normal path the sender publishes by swapping its message into the queue slot (`lockless_queue.cc:1209`) and only then moves `next_queue_index` forward (`:1286`, "to save the next user some computation time"). On the got-beat path (`:1084`) a sender moves it forward only after observing that another sender's message already occupies the slot. So every change of the word happens-after some message became visible, and a watcher that sees the new value and scans finds that message.

The protocol is the standard sequence-futex, and the whole point of it is that a wakeup can be spurious but can never be lost:

- **Sender**: publish, advance `next_queue_index` (both already the case), then `futex(&next_queue_index, FUTEX_WAKE, INT_MAX)`. Shared, not private: the page is `MAP_SHARED` and the waiter is in another process. The wake is unconditional. A wake with nobody waiting is a cheap syscall, and a count to skip it would be one more piece of state to keep right across process death for nothing worth caring about.
- **Watcher**, on every pass: `v = next_queue_index` (raw, acquire), check the channel for new data, then arm a wait with expected value `v`. The wait completes with `0` if a sender woke it, or immediately with `-EAGAIN` if the word moved between the load and the arm. Both mean the same thing — look again — and the loop treats them identically.

The load-then-check-then-wait order is what makes it correct: a sender that publishes after the watcher's load advances the word after it too, so either the arm fails with `-EAGAIN` or the wake finds the waiter. A sender that publishes before the load is found by the check. With several senders the word can be advanced by a sender other than the one that published — B, having lost the slot to A's message, advances past it and A's own advance then fails harmlessly — but in every case the advance is after the publish, which is all the protocol needs.

**Wrapping.** `QueueIndex` wraps the raw value modulo `MaxIndex(0xffffffff, queue_size)` (`index.h`), not modulo `2^32`, and `0xffffffff` is the invalid sentinel the queue starts at. None of that reaches the futex: the kernel compares the word for equality with the value the watcher loaded and nothing else, and the watcher never does arithmetic on the raw word — its reading position is a `QueueIndex` whose `Increment()` and `DecrementBy()` already handle the wrap. The one thing wrapping can do to a futex is ABA: the word returning to exactly `v` between the watcher's load and the kernel's compare at arm time. That needs the channel to wrap all the way around, about `2^32` sends, inside a window that is a few instructions wide. It is not a case worth designing for.

Fetchers map the queue read-only and watchers use the writable mapping, but the futex key for a shared file mapping is `(inode, offset)`, so either mapping of the page names the same futex. The wait is armed on the writable mapping because that is the one a watcher holds.

A sender therefore never reads the `Watcher` slots at all; the snapshot loop and `watcher_copy_` in `LocklessQueueWakeUpper` are deleted.

### What the sender loses: the PI boost

The boost existed because the sender signaled watchers one at a time: a low-priority sender that woke a high-priority watcher first could be preempted by it before reaching the second watcher, which is a priority inversion for the second. One `FUTEX_WAKE` has no such gap. The kernel queues waiters on a futex in priority order (a `plist` keyed on the task's normal priority, `kernel/futex/core.c`) and wakes them all from one call, so a realtime watcher is woken before a non-realtime one without the sender doing anything.

### `Aio`: the same receiver, waiting on words

The first draft of this ADR gave `Aio` a second registration API next to `RegisterThreadSignalReceiver()`: a group of waiters, added and armed one by one, with the owner re-arming each after its scan. Review asked how much of that the existing receiver API could carry instead, and the answer turned out to be all of it, because of one observation: arming a wait _before_ the scan is as correct as re-arming it after.

The protocol needs the word to be loaded before the queue is scanned, and the wait to be armed with that loaded value. Whether the arm happens before or after the scan changes nothing: a send after the arm completes the wait, a send before the load is what the scan finds, and a send between the two either fails the arm with `-EAGAIN` or wakes it, both of which are a spurious pass and never a lost one. So the arm can be done where the signalfd is drained today — in `ConsumeThreadSignalReceiver()`, before the scan — and the post-scan step disappears, and with it the reason for a separate API.

`ThreadSignalReceiver` therefore has a kind (`ipc_lib::WakeupKind`, the queue's), and a futex-kind one carries the words: `AddWakeupWord()`, one per watched channel, called before registration. The `Aio` API is unchanged. `RegisterThreadSignalReceiver()` arms a `FUTEX_WAIT` on every word with the value it holds then; a completed wait disarms and the callback runs once per `Poll()` in which any completed; `ConsumeThreadSignalReceiver()` re-arms whichever completed with the word's current value; `UnregisterThreadSignalReceiver()` cancels what is in flight. The two contracts of the signal kind hold in the same shape: completions coalesce into one callback, and no completion is ever consumed after the last callback. `QueueEventLoop::HandleEvent()` is untouched: consume, scan, sleep. `Startup()` builds a receiver of the queues' kind, adds each watcher's word, and registers it where it registered the signalfd before.

A completion names its channel, which a signal cannot. `HandleEvent()` does not use that, and should not: it scans every watcher on every wakeup because that scan is what keeps delivery ordered across channels. A late wakeup for one channel says nothing about what arrived on the others in the meantime, and reading only the woken channel would deliver messages out of order. The scan stays exactly as it is.

Backends:

- **`IoUringImpl`**: one `IORING_OP_FUTEX_WAIT` SQE per word (`futex_flags = FUTEX2_SIZE_U32`, `mask = FUTEX_BITSET_MATCH_ANY`, no `FUTEX2_PRIVATE`), staged at registration and on each re-arm, submitted by the next `io_uring_enter()` the loop was about to make anyway to wait. The waits are persistent registrations in ADR 0001's sense: unregistering cancels through `IORING_OP_ASYNC_CANCEL` and orphans each until its terminal completion drains, `~IoUringImpl()` lets `io_uring_queue_exit()` reap them, and `HandleFork()` re-arms them in the child with their recorded expected values. Each armed wait holds one ring slot, so `--aio_queue_depth` has to cover a loop's watched channels plus its timers; registration happens in `Startup()` before the loop goes realtime, so too small a depth is a startup failure naming the flag, never a mid-run one. One detail worth knowing: the cancel ack for a futex wait carries the number of waits removed rather than zero, so the backend's ack check accepts any non-negative result.
- **`EpollImpl` and `UvAio` on Linux**: `MakeRingFutexWaitGroups()` (`aos/events/futex_wait_ring.h`), one instance per backend, gives the receiver a second, minimal io_uring created on the loop thread at registration, whose only contents are its futex waits, and registers the ring's fd with the backend's own `OnReadable()` — `EpollImpl`'s epoll instance, `UvAio`'s `uv_poll` — the same way both watch the signalfd. On readable, the callback drains every completion into disarmed waits and invokes the receiver's callback once. Unregistering cancels each armed wait and blocks, briefly, until that wait's completion has been seen, so the state can be freed; completions for other waits met on the way are held for the next drain, and a NOP keeps the fd readable for them. `IoUringImpl` could use the same object through its embedded epoll instead of native SQEs; it does not, because it is the backend the robot runs on and the hop buys it nothing.

  That ring is a _plain_ ring, not the `SINGLE_ISSUER | DEFER_TASKRUN` configuration ADR 0001 chose for the io_uring backend, and the difference is the whole point of it: nobody ever waits on this ring. Its owner is asleep in `epoll_wait()` on its fd, and what has to reach that owner is a kick. A plain ring posts a completion through task work added with `TWA_SIGNAL`, which interrupts `epoll_wait()`, runs on the way back to userspace, and leaves the fd readable for the retry the backends already make on `EINTR`. On a `DEFER_TASKRUN` ring the completion is only posted when the owner enters the ring, and queueing the deferred work does not wake a poller already asleep on the fd, so an owner blocked in `epoll_wait()` never learns there is anything to enter for. The first draft of this ADR proposed `DEFER_TASKRUN` here; the contract test hung on every fd-polling backend until the ring was made plain.

- **Kernels below 6.7, macOS, Windows**: a futex-kind receiver cannot be constructed there (`ThreadSignalReceiver`'s constructor refuses it off Linux), and `QueueEventLoop` never asks for one, because the queue it attached to is of the signal kind.

### io_uring ring tiers

`--io_uring_setup` selects the io_uring backend's ring configuration. Each value is named after exactly the `IORING_SETUP_*` flags it sets:

| `--io_uring_setup` | flags |
| --- | --- |
| `single_issuer_defer_taskrun` (default) | `SINGLE_ISSUER \| DEFER_TASKRUN \| R_DISABLED` (ADR 0001) |
| `single_issuer_coop_taskrun` | `SINGLE_ISSUER \| COOP_TASKRUN \| TASKRUN_FLAG \| R_DISABLED` |
| `single_issuer` | `SINGLE_ISSUER \| R_DISABLED` |
| `coop_taskrun` | `COOP_TASKRUN \| TASKRUN_FLAG` |
| `kernel_default` | none |

The three `single_issuer*` values bind the ring to the thread that first drives it and create it disabled so that binding happens at the first `Poll()`; the other two bind nothing. The flag exists so the tiers can be measured against each other and so every test runs on every one of them; nothing on a robot sets it.

All five pass the whole suite. On `coop_taskrun` and `kernel_default` the thread binding, the automatic downgrade and the created-disabled ring do not exist, so the five tests of those skip; the three `single_issuer*` tiers keep all three and run everything.

`SQPOLL` is deliberately not offered. It was measured (see below) and it is not the answer for this loop: it cannot deliver signal wakeups at all, because a signalfd reports readable when the _polling task_ has the signal pending and on an SQPOLL ring that task is the kernel submission thread, which never has the loop thread's thread-directed wakeups pending. It also submits asynchronously, which breaks the assumption the rest of the backend makes that a submitted entry has been consumed by the time the submit returns. Supporting it cost a wait-for-the-kernel-thread spin after every submit, a refusal path for signal-kind receivers, and twenty tests that skipped.

### Choosing the kind

A new flag, `--queue_wakeup`, takes `futex` or `signal`. It defaults to `signal` on every platform. That is deliberate and temporary: the futex kind is what this ADR is for, but a deployment's wakeup mechanism should not change merely because it was built for a newer kernel, so the futex kind stays opt-in until it has real-world mileage. The default is compiled in (`//aos/ipc_lib:lockless_queue`'s `local_defines`) rather than probed, so making it the platform's choice later is a `select()` on `//tools/platforms/io_uring:has_support`, exactly as `AOS_AIO_DEFAULT_BACKEND` is (`aos/events/BUILD:135-142`). The flag is authoritative, with no probing and no fallback: `futex` on a kernel without io_uring futex ops is a loud constructor crash naming the floor, the same as `--aio_backend=io_uring` on an old kernel. `kMinimumKernelMinor` (`aio_linux.cc:140`) rises to 7 for the io_uring backend, since an `-EINVAL` on a persistent registration is fatal by ADR 0001's rule and a floor that admits kernels the registration cannot work on is a floor in name only.

`InitializeLocklessQueueMemory()` records the kind in the header the first time a queue is initialized and refuses a later process of the other kind, in the same `LOG(FATAL)` shape as today's uid mismatch. That field is the only layout change, and it is enough of one: `ShmPath()` appends `.v8` to every queue file (`memory_mapped_queue.cc:23`), and this change bumps it to `.v9`, so an old binary and a new one on the same `--shm_base` never map each other's files. `Watcher` gains no kind field: the kind is a property of the queue, so a sender knows from the header which path to take, and mixed populations cannot exist.

The flag is for tests and for a host that wants to exercise the signal path against a fresh `--shm_base`. Nothing on a robot sets it.

### The signal kind across users, for a deployment that wants it

The signal kind keeps working, and it can work across users with today's code for a deployment that arranges it: `CAP_KILL` on the binaries that send, and the real-uid trick for the direction `CAP_KILL` does not cover. The kernel is the arbiter either way, and a send it refuses still dies loudly in `ThreadSignalSender::Signal()`. What must not stand in the way is the uid check, which is only the early diagnostic: `--skip_queue_uid_check` turns it off for such a deployment. That keeps two working options open on a 6.12 machine — signals with capabilities, or the futex kind — and leaves the choice with whoever runs it.

### Files: permissions the deployment can reason about

Today `--permissions` is a request the umask edits (`shm_event_loop.cc:9-20`), which means two processes with different umasks can create the same tree with different modes, and the first one to start wins. That was tolerable when every process was the same user. It is not a policy once there are two.

- After the `O_CREAT | O_EXCL` create in `MapShm()` succeeds, the creator `fchmod()`s the file to `--permissions`. `MkdirP()` already applied the mode to each directory it created, exactly — which turned out to be one step too exact: a directory made under a setgid parent inherits the setgid bit from the kernel, and applying `0770` on top of it cleared that bit, so nothing below the top level would have kept the shared group. It now keeps an inherited setgid bit. The umask no longer participates anywhere, and `--permissions` means what its name says.
- `--shm_base` itself is the deployment's: a setgid directory owned by a group every AOS user is in, so every file created under it inherits the group. On the robot, starterd creates it before it drops privileges (`starterd.cc:36`, it is already root there and already touches the tree for `--purge_shm_base`); a `--shm_group` flag names the group. Elsewhere a `tmpfiles.d` line does it: `d /dev/shm/aos 2770 root aos -`.
- Every participant needs read and write on every queue file: readers own watcher and pinner slots in the same file the senders write. There is no read-only role in this design, and `--permissions` is applied to files and directories alike.

`ShmEventLoop`'s own comment on `--permissions` notes that the default umask of `0022` strips group write, which is what made the default `0770` effectively `0750` on most machines. Applying the mode explicitly changes that for everyone, single-user deployments included. That is deliberate and is called out under Consequences.

### Death detection across users

The ownership tracker's `/proc` probe is the one piece of the queue that depends on being able to _see_ the other user's process, and its failure mode is the wrong way round: a `/proc` mounted with `hidepid=1` or `2` makes another user's threads invisible, `ReadStartTimeTicks()` returns `kNoStartTimeTicks`, and `OwnerIsDefinitelyAbsolutelyDead()` declares a live owner dead (`robust_ownership_tracker_linux.cc:40`). The next `Make()` would reclaim the slot under a running watcher.

`hidepid` cannot be told from a real exit by the probe, so it is checked once at queue initialization instead: a futex-kind queue reads `/proc`'s mount options from `/proc/self/mountinfo` and refuses to initialize, with a message naming `hidepid`, if it is set. Refusing rather than warning is the decision: a warning would be read once and the corruption it warns about would happen later, quietly. Signal-kind queues keep working under `hidepid`, which is what they do today.

### `ThreadEventLoop`

`ProcessLocalQueue` uses the same header, so a `ThreadEventLoop` on Linux gets futex wakeups with no work of its own, which answers ADR 0005's open question about signal-free wakeups for the in-process loop. A `FUTEX2_PRIVATE` wait would skip the inode lookup for that case; it is a later optimization behind the same API, not part of this change.

### What does not change

- `Watcher`, `Sender` and `Pinner` slot ownership, the setup lock, and death recovery: all futex-based already, all uid-agnostic.
- The `Watcher` slot still records `pid`, the owner's tid and `priority`, and `Make()` still claims a slot. The signal kind reads them to aim its signal; the futex kind writes them and nothing on its send path reads them. They stay because the print tool shows them and because removing them would make the two kinds' code differ for no reason.
- `HandleEvent()`'s scan of every watcher and its ordering logic, by design (see above).
- The one-completion-per-`Poll()` rule and the coalescing contract on `Aio`.
- The signal path's code, for the platforms and deployments that keep it. It is not deprecated.

## Consequences

- **Processes of different users share queues on Linux 6.7 and later**, with file permissions as the only access control. That is the requirement.
- **The sender's wakeup is one syscall.** Today it is one per live watcher plus two `sched_setscheduler` calls when boosting. `Wakeup()` no longer walks the slots, sorts, or changes its own priority.
- **The `RLIMIT_SIGPENDING` failure of aos#14 cannot happen on the futex path**, and neither can the `EPERM` an application gets for waking starterd while it is briefly root. A `FUTEX_WAKE` has no per-user queue to exhaust and no credential check.
- **Queue files become `.v9`.** A robot crossing the change needs `--purge_shm_base` or a reboot once; an old and a new binary on one machine do not see each other's channels, loudly (the old file is simply not there) rather than subtly.
- **`--aio_queue_depth` has to cover watched channels.** A loop watching many channels can fail at `Startup()` where it did not before; the failure names the flag and happens before the loop goes realtime.
- **The epoll backend and the libuv guest loop pay one extra syscall per wakeup** for the ring fd hop. Both are the non-realtime shapes.
- **The io_uring backend's enforced kernel floor rises from 6.1 to 6.7.** The recommended floor was already 6.12, and the io_uring machines are on 6.12, so nothing in the fleet is excluded by the change.
- **`--permissions` becomes exact.** Files that were `0750` on a machine with the default umask become `0770`. Anyone relying on the umask to _tighten_ the mode has to pass a tighter `--permissions`.
- **Kernels below 6.7, macOS and Windows are unchanged and stay single-user** unless a deployment does the `CAP_KILL` arrangement above. Multiple users on an old kernel is out of scope.
- **A `hidepid` `/proc` refuses futex-kind queues.** That is a deployment constraint that did not exist before, traded for not corrupting a live watcher's slot.
- **The watcher scan runs for every wakeup, as it does today.** Per-channel completions do not reduce it and are not meant to.

## Alternatives considered

- **A dedicated sequence counter as the futex word**, incremented in `Wakeup()`. The first draft of this ADR. It works, and it is one more word in the header and one more store per send to get a property `next_queue_index` already has. Rejected in favor of the index.
- **One futex word per `Watcher` slot** instead of per channel. Rejected because the sender keeps a syscall per watcher and keeps walking the slots, so the PI boost has to stay too; the per-channel word is what lets both go.
- **One futex word per thread, in a global wakeup table** (`/dev/shm/aos/wakeups`, one slot per registered thread, indexed from the `Watcher` slot). One wait per thread instead of one per watched channel. Rejected: a second shared file with its own robust slot allocator, mapped by every process, to save re-arms whose cost on the io_uring backend is an SQE that rides a syscall the loop makes anyway.
- **`IORING_OP_FUTEX_WAITV`**: one SQE waiting on up to 128 words, completing with the index of the one that fired. One SQE per burst instead of one per channel, but every re-arm rebuilds the vector and the kernel queues on every bucket again. Kept as an optimization if watcher counts ever make per-word waits expensive; not chosen because independent per-word re-arming is simpler and the counts are small.
- **Helper threads and an eventfd, for kernels below 6.7.** A thread blocked in `futex(2)` writes an eventfd the loop polls. Below 5.16 there is no `futex_waitv(2)`, so with per-channel words that is one thread per watched channel per loop; making it one thread per loop means the per-thread wakeup table above, built for old kernels alone. Either way it is an extra realtime thread whose priority has to be reasoned about and a context switch on every wakeup. Rejected: multiple users on old kernels is not supported, so they keep signals and there is no third `--queue_wakeup` kind.
- **The status quo: one `GetUserId()` per queue via the real-uid trick.** Works today and is why it took this long to matter. It puts a privileged real uid in every process that needs to talk to an unprivileged one, and it has no answer for two unprivileged users.
- **An eventfd per watcher, handed to senders over a Unix socket.** Needs a broker process and gives every sender an fd table to keep in sync on the realtime send path. Rejected.
- **`IORING_OP_FUTEX_WAKE` on the send side.** The sender is on the realtime send path and does not necessarily own a ring; a `futex(2)` call is one syscall either way.

`CAP_KILL` with the signal kind is not in this list because it is not rejected; see "The signal kind across users" above.

## Verification

- **Lost-wakeup stress**: a `queue_racer`-style test where senders burst and every watcher has to observe every message within a bound, under `--queue_wakeup=futex` on both `--aio_backend` values and under `--queue_wakeup=signal`. This is the test that holds the protocol honest, and it runs with several senders on one channel so the got-beat advance of `next_queue_index` is exercised.
- **`Aio` contract**: `aio_test_lib.cc` gains futex-kind receiver suites, parameterized over `IoUringImpl`, `EpollImpl` and `UvAio`: wake, completed-stays-completed until consumed, re-arm on consume, coalescing into one callback per `Poll()`, no callback after unregister, a successor on the same word, misuse, and re-arm across `fork()`. `aio_test` also runs once per `--io_uring_setup` tier.
- **Every backend, every option, every loop type**: the `shm_event_loop_test` suite runs over `ShmEventLoop` and `ThreadEventLoop`, against epoll, io_uring and libuv, under both wakeup kinds, and for io_uring under every ring tier — generated from lists in `aos/events/BUILD` so a new option cannot be added without every combination running. Every target names the options it runs with, so nothing is covered only by whatever a host defaults to, and changing a default cannot silently move what is tested.
- **Cross-user**: `multi_user_pingpong_test` (`aos/testing/ping_pong`) runs `Ping` as `daemon` and `Pong` as `nobody`, forked from a root test process, on a world-writable `--shm_base` under the futex kind; it skips itself when not root, and it has not been run here. It is the one test in this list that exercises the requirement rather than the mechanism, so it has to be run by hand on a robot image until CI has a root leg.
- **Permissions**: a process with umask `077` creates a queue; a second user in the group attaches and sends.
- **`hidepid`**: a futex-kind queue on a `/proc` mounted with `hidepid=2` refuses to initialize; a signal-kind one does not.
- **Latency**: `ping` (`aos/testing/ping_pong`) records every round trip and logs the distribution when it exits. Run against `pong` as a second process on the io_uring backend, both loops at their own `SCHED_FIFO` 5, each pinned with `taskset` to a fixed physical core of an otherwise idle Ryzen 9 9950X (kernel 7.0, `powersave` governor, no `isolcpus`), 1 kHz pings, 20 seconds per run, five runs of each kind on separate cores and three with both on one core. A round trip is two wakeups, one each way. The ranges are across runs:

  | `--queue_wakeup` | cores | round trips / run | median | p90 | p99 | max |
  | --- | --- | --- | --- | --- | --- | --- |
  | `futex` | 14 / 15 | 20987 | 5.84–5.97 µs | 6.35–6.64 µs | 8.1–9.0 µs | 36–121 µs |
  | `signal` | 14 / 15 | 20988 | 6.67–6.95 µs | 7.27–7.54 µs | 9.7–11.3 µs | 35–85 µs |
  | `futex` | 15 / 15 | 20987 | 4.05–4.11 µs | 4.50–4.56 µs | 7.2–7.4 µs | 34–39 µs |
  | `signal` | 15 / 15 | 20989 | 4.93–5.19 µs | 5.47–5.64 µs | 8.4–8.9 µs | 37–41 µs |

  The futex path is about a microsecond faster per round trip at the median and at p90, consistently across runs and on both core layouts, and a little tighter at p99. The maxima are the host's, not the mechanism's: a development machine with frequency scaling and no isolated cores.

  The same benchmark on the robot controller — a systemcore, four Cortex-A76 cores on 6.12.77, the robot's own daemons left running, both loops `SCHED_FIFO` 5, pinned to cores 2 and 3 (or both to 3), `performance` governor, io_uring backend, five 20-second runs of each kind:

  | `--queue_wakeup` | cores | median | p90 | p99 | max |
  | --- | --- | --- | --- | --- | --- |
  | `futex` | 2 / 3 | 13.8–14.2 µs | 17.2–17.6 µs | 20.9–41 µs | 133–154 µs |
  | `signal` | 2 / 3 | 15.3–17.0 µs | 19.5–20.0 µs | 24–46 µs | 128–147 µs |
  | `futex` | 3 / 3 | 27.7–28.6 µs | 31.8–32.8 µs | 35.3–35.8 µs | 139–157 µs |
  | `signal` | 3 / 3 | 31.9–32.9 µs | 36.3–36.5 µs | 39.2 µs | 129–156 µs |

  Same direction, about two microseconds per round trip, on the hardware that matters. Two things differ from the x86 host: everything is roughly twice as slow, and the same-core layout is _slower_ than two cores there, where on the host it was faster — the A76's handoff between two realtime tasks on one core costs more than its cross-core wakeup.

- **Realtime tuning on the systemcore**: the same harness, one 20-second run per cell, varying what a deployment controls. Governor: `ondemand` (the image's default), `performance`, `schedutil`. Scheduling: CFS, `SCHED_FIFO` 5 (below the kernel's irq threads at 50), `SCHED_FIFO` 55 (above them). Affinity: pinned to cores 2 and 3, or unpinned. Median / p99 of the round trip, futex kind:

  | governor | CFS, pinned | CFS, free | FIFO 5, pinned | FIFO 5, free | FIFO 55, pinned | FIFO 55, free |
  | --- | --- | --- | --- | --- | --- | --- |
  | `ondemand` | 23.8 / 119 | 29.9 / 113 | 21.9 / 31 | 22.6 / 36 | 21.9 / 31 | 22.6 / 35 |
  | `performance` | 17.6 / 117 | 19.3 / 60 | 14.2 / 29 | 14.3 / 25 | 13.7 / 20 | 14.4 / 21 |
  | `schedutil` | 22.6 / 104 | 28.8 / 84 | 21.5 / 55 | 22.3 / 42 | 21.7 / 31 | 21.9 / 34 |

  And the signal kind, for comparison:

  | governor | CFS, pinned | CFS, free | FIFO 5, pinned | FIFO 5, free | FIFO 55, pinned | FIFO 55, free |
  | --- | --- | --- | --- | --- | --- | --- |
  | `ondemand` | 32.7 / 160 | 46.3 / 149 | 24.0 / 72 | 24.9 / 55 | 24.9 / 36 | 24.4 / 39 |
  | `performance` | 20.3 / 156 | 30.8 / 63 | 15.3 / 47 | 15.9 / 25 | 15.3 / 25 | 16.2 / 25 |
  | `schedutil` | 27.6 / 124 | 45.4 / 129 | 24.1 / 56 | 24.5 / 47 | 24.1 / 37 | 24.3 / 41 |

  What the table says, in order of effect:

  - **Realtime scheduling is what makes the tail.** Under CFS the p99 is 60–160 µs and the maxima reach one to three milliseconds on every governor; any `SCHED_FIFO` priority brings p99 under 60 µs and the maxima under 250 µs. Which priority matters far less than having one: 55 tightens p99 and the maximum somewhat over 5 (20 versus 29 µs at the median's best), which is the cost of sitting below the irq threads.
  - **The governor is the biggest lever on the median.** `performance` is 8 µs faster per round trip than `ondemand` or `schedutil` at every priority, which is the A76 idling at a low clock between 1 ms pings and paying to ramp. A deployment that wants the numbers in the first table has to set the governor; the image's default does not.
  - **Affinity barely matters once the loops are realtime**, a microsecond either way. Under CFS, pinning helps the median and hurts the tail.
  - **The futex kind is faster in every one of the 18 cells**, by 2–3 µs at the median under realtime and by more under CFS, and its p99 is lower in all but two.

  One run per cell, so the p99 column is noisy at the ±10 µs level; the medians are stable to a microsecond across the repeated baseline runs.

- **Backend and ring tier on the systemcore**: the baseline configuration (`performance`, `SCHED_FIFO` 5, pinned to cores 2 and 3), the epoll backend and the libuv guest loop (`ping_uv`/`pong_uv`, the same programs on a `uv_loop_t` they own) against each `--io_uring_setup` tier of the io_uring backend, three 20-second runs each, median / p99 ranges:

  | backend | `--io_uring_setup` | `IORING_SETUP_*` flags on the ring | `futex` | `signal` |
  | --- | --- | --- | --- | --- |
  | libuv guest (`UvAio`) | — | (no ring; libuv's epoll) | 16.4–18.1 / 37–48 µs | 15.7–15.7 / 40–49 µs |
  | `epoll` | — | (no ring) | 16.5–16.6 / 31–33 µs | 16.3–17.8 / 27–44 µs |
  | `io_uring` | `single_issuer_defer_taskrun` (default) | `SINGLE_ISSUER \| DEFER_TASKRUN \| R_DISABLED` | 13.7–13.9 / 20–36 µs | 15.4–16.4 / 44–47 µs |
  | `io_uring` | `single_issuer_coop_taskrun` | `SINGLE_ISSUER \| COOP_TASKRUN \| TASKRUN_FLAG \| R_DISABLED` | 14.8–15.0 / 32–37 µs | 16.6–17.1 / 29–45 µs |
  | `io_uring` | `single_issuer` | `SINGLE_ISSUER \| R_DISABLED` | 14.8–15.1 / 24–44 µs | 17.0–17.3 / 37–53 µs |
  | `io_uring` | `coop_taskrun` | `COOP_TASKRUN \| TASKRUN_FLAG` | 14.9–15.0 / 33–43 µs | 16.7–17.0 / 47–50 µs |
  | `io_uring` | `kernel_default` | (none) | 14.8–15.0 / 28–35 µs | 17.1–17.5 / 28–33 µs |
  | `io_uring` | `sqpoll` (measured, then dropped) | `SQPOLL` | see below | not possible |

  `R_DISABLED` is not a tier of its own: every `SINGLE_ISSUER` ring is created disabled so the thread binding happens at the first `Poll()` rather than at construction (ADR 0001), and `TASKRUN_FLAG` is what lets the backend see that `COOP_TASKRUN` left work pending.

  The io_uring backend on its default tier is the fastest configuration by two to three microseconds over epoll, and `single_issuer_defer_taskrun` is a microsecond ahead of every other tier, including the two that keep `SINGLE_ISSUER` but drop `DEFER_TASKRUN`: `single_issuer` measures the same as `kernel_default`, and `single_issuer_coop_taskrun` the same as `coop_taskrun`. The microsecond is `DEFER_TASKRUN`'s, not the binding's — completion work that runs only inside the loop's own wait, never at some other syscall — which is the determinism ADR 0001 chose it for showing up as latency. The `io_uring_setup(2)` page argues for `COOP_TASKRUN` over the interrupting default on cost grounds (the IPI, the kernel/user transition, "the needless interruption of the tasks userspace activities") and for `DEFER_TASKRUN` as the stricter form of the same idea; the numbers agree with it. For the futex kind the ring hop the fd-polling backends pay (`futex_wait_ring.h`) costs about what the backends differ by: on epoll the two kinds measure the same, and on libuv the futex kind is a microsecond behind the signal kind. The libuv row is from a later session on the same device, whose epoll and io_uring controls repeated the medians above (16.2–16.6 µs and 13.6–13.8 µs for the futex kind, 16.4–16.9 µs and 15.4–16.6 µs for the signal kind) with p99s spread across 23–46 µs, so the p99 columns are comparable only to that width. libuv lands where epoll does, which is what it is underneath; the guest loop's own turn adds nothing the median can see.

  SQPOLL pinned was not a measurement but a failure, and an instructive one: round trips of two to three seconds, and an RCU stall in the kernel log naming both rings' `iou-sqp` threads, spinning on CPUs 2 and 3. A ring's submission thread inherits its creator's CPU affinity, so pinning the loop to a core pins its kernel thread to the same core, where the `SCHED_FIFO` loop starves it until the RT throttle lets it run, about once a second per direction. Unpinned, so the scheduler can put the two kernel threads on the other two cores, three runs each against the default tier unpinned the same way:

  | `--io_uring_setup`, unpinned | `IORING_SETUP_*` flags | round trips / run | median | p90 | p99 | max |
  | --- | --- | --- | --- | --- | --- | --- |
  | `single_issuer_defer_taskrun` | `SINGLE_ISSUER \| DEFER_TASKRUN \| R_DISABLED` | 19992–19993 | 14.3–14.4 µs | 17.9–18.0 µs | 21–25 µs | 67–83 µs |
  | `sqpoll` | `SQPOLL` | 13141–15508 | 16.1–18.2 µs | 22–31 µs | 43–52 µs | 94 µs–375 ms |

  SQPOLL is slower at the median, twice as wide at p99, lost a quarter of its round trips (ping counts the pongs that came back before the next ping went out), and costs two of the robot's four cores in kernel threads to get that. That, with the signal wakeups it cannot carry and the synchronous-submit assumption it breaks, is why `--io_uring_setup` no longer offers it. The numbers are kept here so the question does not get asked again.

- **The robust and PI paths**: `robust_ownership_tracker_test` and `lockless_queue_death_test` run with the dying process as a different user, to pin the kernel behavior this design relies on.

## Open questions

- **Running the cross-user test.** It needs root and two unprivileged users, which CI does not have today. Until it runs somewhere regularly, the multi-user requirement is verified by hand.

## References

- `aos/ipc_lib/lockless_queue.cc` — `InitializeLocklessQueueMemory()` uid check (`:701`), `LocklessQueueWatcher::Make()` (`:755`), `LocklessQueueWakeUpper::Wakeup()` (`:817`), the send path's `next_queue_index` advances (`:1084`, `:1286`)
- `aos/ipc_lib/lockless_queue_memory.h`, `aos/ipc_lib/index.h` — `next_queue_index`, `QueueIndex` wrapping
- `aos/ipc_lib/memory_mapped_queue.cc` — `ShmPath()` and the `.v8` suffix
- `aos/ipc_lib/thread_signal.h`, `aos/ipc_lib/thread_signal_linux.cc` — the signal path and the `RLIMIT_SIGPENDING` note
- `aos/ipc_lib/robust_ownership_tracker_linux.cc`, `aos/util/proc_stat.cc` — the `/proc` probe
- `aos/ipc_lib/shm_mapping_linux.cc`, `aos/events/shm_event_loop.cc` — file creation and `--permissions`
- `aos/realtime_linux.cc` — `GetUserId()`
- `aos/starter/starterd.cc`, `aos/starter/subprocess.cc` — `--user`, per-application users, the real-uid trick
- `aos/events/aio.h`, `aos/events/aio_linux.cc` — `RegisterThreadSignalReceiver()`, `RequireMinimumKernelVersion()`, `--aio_backend`, `--io_uring_setup`
- `aos/events/futex_wait_ring.h`, `aos/events/futex_wait_ring.cc` — the ring-fd implementation for the epoll and libuv backends
- `aos/events/aio_test_lib.cc` — the futex wait group contract suites
- `aos/testing/ping_pong/multi_user_pingpong_test.cc` — the cross-user test
- `aos/events/queue_event_loop.cc` — `HandleEvent()`
- `aos/testing/ping_pong` — the round-trip benchmark
- `tools/platforms/BUILD`, `tools/platforms/io_uring/BUILD`, `aos/events/BUILD` — the io_uring constraint and the compiled-in default
- `documentation/adr/0001-aio-io-uring-single-issuer.md` — ring configuration, persistent registrations, fork, kernel floor
- `documentation/adr/0004-aio-libuv-guest-loop.md` — everything a guest loop waits on has to be an fd
- `documentation/adr/0005-thread-event-loop.md` — the in-process loop and its open question on signal-free wakeups
- Linux `io_uring/futex.c`, `kernel/futex/pi.c`, `kernel/futex/core.c`, `io_uring/io_uring.c` (`io_uring_poll()`, `io_activate_pollwq()`)
- `kill(2)`, `rt_tgsigqueueinfo(2)`, `futex(2)`, `io_uring_setup(2)`, `proc(5)` (`hidepid`)
- https://github.com/RealtimeRoboticsGroup/aos/issues/14
