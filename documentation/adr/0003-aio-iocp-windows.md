# ADR 0003: IOCP backend for Aio on Windows — emulating readiness on a platform that only reports completions

## Status

Accepted. Implemented in `aos/events/aio_windows.cc`, with `aos/events/pipe_windows.cc`, `aos/ipc_lib/thread_signal_windows.cc` and `aos/events/winsock_init.h` supplying what the platform lacks. `//aos/events:aio_test` runs against it with every case that does not need `fork(2)`, less one real gap recorded under Consequences; see Verification.

## Context

`Aio` (`aos/events/aio.h`) is AOS's cross-platform completion-based async I/O primitive. ADR 0001 covers io_uring and epoll on Linux; ADR 0002 covers kqueue on macOS. This is the fourth backend, and the first on a kernel that is not POSIX.

`Aio`'s public surface is completion-based, and so is IOCP — io_uring is the only other native fit; epoll and kqueue had to be adapted upward. That does not help where it would matter most:

**`Aio` also carries a readiness API, and Windows has no readiness notification.** `OnReadable()`, `OnWritable()`, `OnEvents()`, `SetEvents()` and `EnableWritable()`/`DisableWritable()` are inherited from `EPoll` and callers depend on them. IOCP reports that an operation _finished_, never that a descriptor is _ready_. So this is the one backend that emulates readiness on top of completions, the reverse of epoll and kqueue, and why the shared `internal::ReadinessBackend` does not fit. Where kqueue had to be taught Linux's _ordering_, IOCP has to be taught Linux's _question_.

The other divergences are ordinary platform gaps, several of them load-bearing. Each was established by probe, not inferred from documentation:

- **A socket's completion-port association is permanent for the life of the socket, and cannot be queried.** Closing the first port does not release it: re-associating gets `ERROR_INVALID_PARAMETER` (87). Nothing asks a socket which port it is on, and nothing detaches it. (`NtSetInformationFile(FileReplaceCompletionInformation)` can replace it; native-API and untried, see Alternatives.)
- **There is no `fork()`.** ADR 0001's fork machinery has nothing to do here.
- **There is no `signalfd`, no `timerfd`, and no thread-directed signal.** macOS lacks the first two (ADR 0002); Windows lacks the whole signal model. A thread wakeup is a named `Event`, which is not a descriptor.
- **Turning "this object became signalled" into a completion is a kernel service, but not a documented one.** `NtAssociateWaitCompletionPacket` attaches a waitable object to a port directly. The Win32 route, `RegisterWaitForSingleObject`, does the same job with a pool thread and a callback.
- **A waited timeout, and a plain waitable timer, are quantised to the system timer tick**, ~15.6ms by default; only a high-resolution waitable timer is exempt. Linux gives hrtimers for free.
- **`FileDescriptor` is an opaque `HANDLE`**, `void *` here and `int` elsewhere (`aos/events/file_descriptor.h`). Anything that spells a descriptor as an `int`, `close(2)`s one, or invents one as `-1` stops compiling.
- **Only sockets can be the target of `WSARecv`/`WSASend`**, which is what the completion-based half of this backend issues. That is a limit of this backend, not the platform: a regular file opened `FILE_FLAG_OVERLAPPED` associates with a port and reads through it with `ReadFile` normally. It is why `RegularFileAsyncReadWriteTest` is excluded, and why the exclusion is a gap rather than a law.
- **There is no submission queue and no completion-queue overflow**, so `--aio_queue_depth` has no referent.

## Decision

Write a fourth backend against IOCP directly rather than a readiness shim over an existing one, and treat the Linux semantics as the contract, as ADR 0002 did. Concretely:

1. **Readiness is synthesised from a zero-byte `WSARecv()`**, with a `MSG_PEEK` to tell apart what its completion can mean. The mask handed to the caller is what epoll would have produced, masked to the subscription — ADR 0002's decision 4, reached independently for the same reason: it lets the dispatch below stay a transcription of the Linux backend.
2. **`IocpImpl` derives from `Aio::Impl`, not `internal::ReadinessBackend`.** That base takes readiness as the primitive; here the direction is reversed, so nothing above the seam is reusable. The cost is discussed below.
3. **Legacy handlers and raw requests are mutually exclusive on a descriptor, in both directions, and live in separate namespaces**, even though this backend stores both in one `FdState`.
4. **Timers live in the shared `TimerQueue`, and the head deadline is delivered by a high-resolution waitable timer** through a wait completion packet, never by a wait timeout. Ties are FIFO, as on every backend.
5. **A wakeup is a named manual-reset `Event`**, delivered by a wait completion packet. Manual-reset is load-bearing: it puts the consumption point in our code, where `read()` sits on Linux.
6. **A `ThreadSignalReceiver`'s identity binds at registration, not construction**, and at most one may be bound to a thread at a time.
7. **No thread but the caller's.** Every waitable object this backend watches reaches the port through a wait completion packet.
8. **`AsyncRequest::internal_state` is the same 64 bytes on every platform.**
9. **A handle that is not a socket is still watchable through the legacy API**, by waiting on it as an object rather than reading it as a stream. This is what lets `GlibMainLoop` run here.
10. **Flags whose mechanism does not exist here are still defined, and ignored**, so a caller can name them without an `#ifdef`. Not free — see `--aio_backend` under Consequences.
11. **One socket belongs to one `Aio`, and that is a caller obligation**, since the backend cannot enforce it.

## Design

### Readiness from completions: a zero-byte read, and a peek

A legacy registration arms a zero-byte `WSARecv()` for `kIn`. `kOut` is not its mirror image — see the next section for why. A zero-byte receive completes when the socket becomes readable and takes nothing off the stream — the closest thing IOCP has to "tell me when this would not block". But it completes identically for "data arrived" and "the peer shut down", which epoll distinguishes as `EPOLLIN` and `EPOLLHUP`. So the completion handler peeks one byte with `MSG_PEEK`: positive is `kIn`, an error other than `WSAEWOULDBLOCK` is `kErr`, and `WSAEWOULDBLOCK` means the wakeup was spurious and the watch is re-armed.

A watch is one overlapped operation, so delivering it disarms it — where an `epoll_ctl()` registration is level-triggered and persistent. Arming is therefore driven by every change that can alter the answer, and by nothing else: each legacy mutator (`OnReadable()`, `SetEvents()`, `EnableWritable()`, …) runs `UpdateSocketState()` through `UpdateSocket()`, and both dispatch paths run it again on the way out, after the callback has had its chance to consume what the watch reported. `UpdateSocketState()` is idempotent, guarded on the `read_pending`/`write_event_armed`/`handle_watch_armed` flags, so running it more often than strictly needed costs a branch.

This used to be a sweep over every registration at the top of each `Poll()`, which made arming O(registrations) per poll to re-arm the one watch that had just fired — the same shape as the retirement sweep, and removed for the same reason. The sweep was also, incidentally, retrying an arm whose `WSARecv()` failed synchronously — every poll, forever, reporting nothing. A failed arm now CHECKs. It queues nothing, so no completion is owed and no change will arrive to re-arm it: the fd stops waking and the loop goes quiet, which is the failure mode hardest to diagnose from the outside. `EpollImpl::UpdateRegistration()` dies on the same failure and so does `KqueueImpl`, so this is the shared answer rather than a Windows one.

### Writability is the one thing completions cannot express

The write watch was originally the read watch's mirror image, a zero-byte `WSASend()`, and the symmetry is false. A zero-byte send has nothing to buffer, so it completes at once however full the socket is; there is no state in which it waits. `OnWritable()` therefore reported writability that was not there on every `Poll()` — measured at 20 dispatches in 20 polls against a socket `select()` called unwritable — where epoll and kqueue stay quiet. `EPollLikeBasicWritable` could not see it, because `FillPipe()` loops on `write_ready()` and so stops polling exactly when the interesting case begins.

Writability comes from `WSAEventSelect(FD_WRITE)` instead. That delivers on a `WSAEVENT`, which is just a waitable handle, so it reaches `Poll()` through the same wait completion packets as everything else here — no thread, no new wakeup mechanism. Three properties make it usable, each measured rather than taken from documentation: it coexists with the overlapped zero-byte `WSARecv()` on the same socket, so the read watch and its `MSG_PEEK` are untouched; the stack re-signals `FD_WRITE` after _any_ send fails with `WSAEWOULDBLOCK`, including the caller's own, which `Aio` never sees; and `WSAEventSelect()` signals immediately when the socket is already writable, which covers arming on an idle socket.

Two things it cannot do, and both cost something visible:

`FD_WRITE` is edge-triggered where `EPOLLOUT` is level-triggered. Once consumed it does not fire again merely because the socket is still writable, and re-calling `WSAEventSelect()` does not re-record it (measured). So `select()` with a zero timeout is the level oracle — it decides whether to dispatch `kOut` — and `FD_WRITE` only decides when to stop blocking. That is one `select()` per armed write watch per `Poll()`, skipped entirely when no caller has registered one, which on this platform is every caller today.

The re-signal needs a send to have failed, and an _exact_ fill never fails. A non-blocking `send()` on a nearly-full socket returns a short count rather than an error, so a caller that stops when its data runs out can land exactly on full having never seen `WSAEWOULDBLOCK`. Measured: 2617344 bytes written that way, then the peer drained everything, and `FD_WRITE` stayed silent while `select()` said writable. Nothing can arm the latch on our behalf — a one-byte probe send would race against the peer draining and could inject a stray byte into the caller's stream — so `Poll()` caps its wait at 100 ms whenever a write watch is armed. It is a backstop for that one case, not the mechanism: `FD_WRITE` covers every case where a send actually failed.

The remaining cost is `WSAEventSelect()`'s own documented side effects: it puts the socket into non-blocking mode, and it replaces any `WSAEventSelect()` someone else had on that socket. Both are further reasons a socket may belong to only one `Aio`.

`kErr` (`0x08`) is folded into the _read_ watch rather than getting its own. A socket is nearly always immediately writable, so a write watch armed to detect errors would complete at once and report writability nobody asked for — the same trap ADR 0002 describes for `EVFILT_WRITE`. A read watch only completes on data, an orderly shutdown, or an error, which is sufficient.

Zero from the peek — the peer is gone — is the same `EV_EOF` conflation ADR 0002 found, with one more turn of the screw: kqueue at least gets to ask which filter reported it, while here one read watch stands in for both. The subscription decides. A registration watching `kIn` is the reader, and its peer closing is `EVFILT_READ|EV_EOF`, which macOS reports as `kIn`, and `EPOLLHUP`, which Linux keeps out of the in/out/err trio: not an error, consumable only by a read that observes the zero-length result. A registration not watching `kIn` is watching for errors, and what matters to it is the write side going away: `EVFILT_WRITE|EV_EOF`, and `EPOLLERR`.

Getting that wrong was fatal. Reporting every hangup as `kErr` sent an ordinary `OnReadable()`-only registration into the `err_fn` CHECK the moment its peer closed — a process abort where Linux and macOS dispatch a read. ADR 0002's decision 3 is about exactly this trap, and the existing coverage stopped one poll short of it; `LegacyReadableSurvivesPollingPastHangup` now pins it on every backend. `terminal` is tracked apart from the mask so a hangup reaches `OnEvents` as `kErr` — epoll's `terminal ? (ready | kErr) : ready` — without ever reaching `err_fn` alone.

Dispatch below that point transcribes `EPoll::InOutEventData::DoCallbacks()`, message text included: a readiness bit with no handler is a busy loop, and an error with no `err_fn` is fatal. Both dispatch sites first apply `events &= state.legacy_events | kErr` — the subscribed mask, with the error bit always through because a watch may exist purely to detect it.

### Handles that are not sockets

Only a socket can join a completion port, and `ToSocket()` is a bare `reinterpret_cast`, so nothing downstream could tell the difference. A non-socket handle reached `CreateIoCompletionPort()`, failed with `ERROR_INVALID_HANDLE`, and was retried — and logged — on every `Poll()`, while never being watched.

`IsSocket()` (a `getsockopt(SO_TYPE)`, cached per registration) splits the two. A non-socket takes the wait-packet path instead of the zero-byte read, and reports the caller's subscribed mask when the object signals. That is not an approximation of epoll; it is what the platform means. glib's own `g_poll()` on Windows waits on the handle and, when it signals, reports back every bit the caller asked for — in glib's terms, the requested `events` mask is echoed as the `revents` result, unchanged — so a handle-backed registration here has exactly the resolution glib itself provides.

One hazard shapes it. A handle we do not own must not be reset, and re-arming a still-signalled handle right after dispatch queues its packet again at once, faster than `Poll()` can drain. So every `HandleWatch` is one-shot: delivery disarms it, and `DispatchHandleSignal()` re-arms it only once the callback has returned and the owner has had its chance to consume the handle. The `ThreadSignalReceiver` takes the same path — a legacy `OnReadable()` on its `Event` handle, the shape `EpollImpl` gives the `signalfd` — with a callback that consumes before it notifies. `WaitableHandleIsReportedUntilConsumed` pins both halves: a handle the callback leaves set is reported again, one it resets is not.

### Timers: the shared `TimerQueue`, delivered by a high-resolution waitable timer

Armed timers live in `TimerQueue<AsyncRequest, TimerQueueTraits>`, the same container and `(deadline, sequence)` ordering as the kqueue backend, so ADR 0002's requirement (2) is met by sharing the implementation. Arming is O(log n); a `queued` flag on the request answers membership in O(1). `Poll()` fires at most one expired timer per call, and only when nothing else was dispatched: `Aio`'s one-completion-per-`Poll()` rule.

The per-request record holds only the timer node: a deadline, three tree links, a sequence, a colour and a `queued` flag. A raw `AsyncRead`/`AsyncWrite` keeps nothing there -- the kernel takes its buffer through the `WSABUF` at submit time, and the completion is identified by which `OVERLAPPED` came back -- which is what leaves room for the tree node inside the 64 bytes every backend gets. The per-fd `OVERLAPPED`s live in `FdState`; that is what makes a retired registration un-recyclable until its completion lands (below).

The head deadline is delivered by one `CREATE_WAITABLE_TIMER_HIGH_RESOLUTION` timer, armed for it and attached to the port by a wait completion packet; the wait itself has no timeout. The timer takes its due time in 100ns units and is exempt from the system tick, so a deadline is never rounded up to a whole millisecond — which is what `GetQueuedCompletionStatus()`'s own timeout does, and what ADR 0002 rejected `uv_timer_t` for. Measured through this path on an idle machine: lateness floor 35–100µs, median 0.2–1.0ms, worst 1.2ms. Expiry is still quantised to the platform clock interval on this Windows 10 build, so delivery is not nanosecond-exact; but the request is, sub-millisecond deadlines are honoured rather than rounded, and the worst case tightened from 4.5ms. `Poll()` re-arms only when the head deadline changes, withdraws the packet when the queue empties, and treats a fire that lands before `monotonic_clock` agrees — they are read from different clocks — as not yet due.

One kernel timer for the head rather than one per `Timer`, for two reasons. `Aio` orders equal deadlines by arming sequence (ADR 0002's requirement 2), and the port's FIFO of N independent timer packets would not preserve that when several fire in one tick; the tree does, exactly. And a timer object plus a packet per `Timer` would be two kernel handles created, set and cancelled on paths — `Schedule()`, `Cancel()`, a timer re-armed from its own callback — that today are pointer operations on the tree, with `Poll()` touching the kernel only when the head actually changes.

`timeBeginPeriod(1)` is still called once per process, for two measured reasons: the high-resolution timer is tighter with the 1ms tick (a 500µs deadline was 0.18ms late at the median with it, 0.47ms without), and every other wait in the process, `aos::SleepFor()` included, is `Sleep()`-based and tick-bound. There is no matching `timeEndPeriod()`; since Windows 10 2004 the setting is per-process.

### The wakeup Event is manual-reset, and that is the `signalfd` analogue

`ThreadSignalSender::Signal(pid, tid)` is `rt_tgsigqueueinfo(2)` on Linux. Windows has no thread-directed signal, so `thread_signal_windows.cc` rendezvouses through a named object: both halves derive `Local\aos-wakeup-<pid>-<tid>` from the target pair, the receiver creates it and the sender opens it. This addresses one thread, so Windows does _not_ inherit the macOS broadcast problem ADR 0002 documents.

An auto-reset `Event` is consumed by whoever waits on it, and the waiter here is the kernel: the wait clears the bit before any of our code runs, so a wakeup would exist only as an in-flight completion, with nothing kernel-side to recover it from. `signalfd` has the opposite property — the signal sits in the fd until _we_ `read()` it — and that is what makes `aio.h`'s consume-then-notify guarantee implementable. Manual-reset restores it: `ConsumeWakeup()`'s `ResetEvent()` occupies the position `read()` occupies on Linux, and the registered callback consumes before it notifies, so a burst collapses into the one callback about to run. A signal landing after the reset stays set and produces a later callback — the same intended edge `aio_linux.cc` documents. Consume, then re-arm — the watch is re-armed only after the callback returns — never the reverse: re-arming a still-set bit delivers at once and spins.

This also gives, for free, the guarantee ADR 0002 built machinery for on macOS: a wakeup nobody consumed belongs to whoever registers next. Only `ConsumeWakeup()` clears the bit — unregistering drops the packet and unbinds, and touches nothing else — so a loop that observes a wakeup with nothing registered does not destroy it, and re-registering re-arms against a bit that is still set. That is why the two successor-wakeup tests written against kqueue, where `EVFILT_SIGNAL`'s count is consumed by being reported, passed here unchanged. The guarantee spans a receiver's unregister/re-register cycle, not its destruction: a named Event dies with its last handle, and `BindToCurrentThread()` respects that by creating the replacement handle before closing the old one.

**Identity binds at registration.** The Event's name carries a thread id, so the constructor can only name the thread that built the receiver; Linux and macOS resolve the target when the signal is _sent_, so that never mattered there. `BindToCurrentThread()`, called from `RegisterThreadSignalReceiver()` on the polling thread, re-creates the Event for the thread that will serve it. `ShmEventLoop` constructs and registers three lines apart in `Startup()`, on the loop's thread.

The same change made "one bound receiver per thread" a `CHECK` on every platform. Two receivers on one thread were already broken on Linux — two `signalfd`s with the same mask, either of which can dequeue the signal, and a second destructor unblocking what the first still needs blocked. Windows made it fail loudly instead of quietly.

### Waitable objects reach the port as wait completion packets

A wait completion packet is a kernel object associated with a port, a target object, and a key; when the target signals, the kernel queues the packet to the port. No callback, no thread — it is what the Windows thread pool is itself built on. `NtCreateWaitCompletionPacket`, `NtAssociateWaitCompletionPacket` and `NtCancelWaitCompletionPacket` are resolved from `ntdll` at first use and `CHECK`ed; present since Windows 8.

Semantics, by probe: a packet is one-shot and must be re-associated after each delivery; associating with an already-signalled object queues it immediately; associating a packet that is still associated fails; cancelling with `RemoveSignaledPacket` also withdraws a packet that is queued but not yet dequeued. Closing the target or the port while associated delivers nothing and leaves the packet cancellable.

A `WaitPacket` owns one packet and the one fact the kernel does not report back, whether it is currently associated; the three things that wait through one — `DeadlineTimer`, `HandleWatch`, `WriteWatch` — own theirs. `Poll()` marks a packet disarmed on delivery, and its owner decides when it is armed again: the timer when the head deadline changes, a write watch at once, a handle watch only after the callback has returned. Teardown is cancel-and-close with nothing in flight anywhere, since a packet is only ever dequeued by `Poll()`, and cancelling with `RemoveSignaledPacket` withdraws a packet the kernel has queued but `Poll()` has not yet dequeued — which is what lets a dequeued packet's context be trusted as a live `FdState`.

So every user callback runs inside `Poll()`, **single-threaded from the caller's point of view**, as on the other backends. Every `WSARecv`/`WSASend` passes a null completion routine, so socket completions go to the port rather than an APC, and the backend creates no threads. The `RegisterWaitForSingleObject` design this replaced is under Alternatives; its use-after-free is under Lessons.

### Association is permanent, and the failure is silent

Re-associating an already-associated socket fails with `ERROR_INVALID_PARAMETER`, and that is treated as success: normally it means re-registering an fd this loop retired earlier, where re-association is harmless and unavoidable since the state carrying the flag went back to the pool.

It can also mean the socket belongs to a _different_ `Aio`'s port, which is unrecoverable: this loop never sees a completion for it and a `Poll()` waiting on one blocks forever. Windows cannot tell the two apart, so the check stays permissive and the obligation moves to the caller. Not hypothetical: `RawRequestReusedOnASecondAioTest` shared one `Pipe` between two `Aio`s and hung `aio_test` as an 1800s timeout. Its subject was an `AsyncRequest` outliving its `Aio`; the shared descriptor was incidental, and one pipe per instance preserves the subject exactly.

`Aio`s are brought up and torn down rarely — one per process, for the life of the process, is the shape every caller has — so nothing here is designed around moving a socket between two of them. The rule is documented and left at that.

### Teardown: cancel everything, then drain

Every overlapped operation still out at teardown owes exactly one completion, and every one of them is cancelled before the drain starts: `~IocpImpl()` cancels the completion-based requests _and_ the legacy read watches — overlapped operations too, counted in `pending_io_count_` the same way — on the live registrations and the retired ones alike, then drains until the count reaches zero. That is why the drain terminates rather than wedges: nothing that increments the count is still armed when it begins, and a cancelled operation always completes. The other watches are not overlapped operations at all — a `WriteWatch` or `HandleWatch` is a wait completion packet, cancelled with `RemoveSignaledPacket` when its registration is deleted — and the destructor `CHECK`s that none survive, so only socket I/O can arrive during the drain; it skips the internal keys regardless.

### Staying malloc-free on the arming path

`aio.h` promises that `AsyncRead`/`AsyncWrite` do not allocate, and `realtime_windows.cc` enforces it as Linux does: it detours `ucrtbase`'s `malloc` and aborts on one while realtime.

The promise is about the user's view of the API: nothing a caller can reach from a realtime section — `AsyncRead`, `AsyncWrite`, `Cancel`, `Poll` and the callbacks it runs — mallocs. What the backend does with kernel objects behind that is its own business, and it is allowed to lean on the platform a little where Linux would not have to: a `WriteWatch` or `HandleWatch` is constructed in place inside its `FdState`, and the event, timer and packet handles behind it are kernel allocations rather than heap ones, so creating one on the re-arm path after a dispatch is fine where a `new` would trip the hook.

So per-fd states come from a pre-allocated pool; `retired_` and `free_list_` are intrusive stacks sharing one link (the memberships are exclusive); and the selective scrub is `MoveMatchingTo()` — a state still owed a completion stays parked, because the kernel holds a pointer into its `OVERLAPPED`. Nothing on that path pushes to a vector that can reallocate.

Pool exhaustion falls back to `new` rather than aborting, matching `FdRegistrationTable::Allocate()`: the pool is sized so it does not happen, and if the sizing is wrong the malloc hook is what should say so. `AsyncRegistrationPoolFallsBackBeyondCapacityTest` pins the fallback; `AsyncRegistrationPoolRecycleTest`, under `ScopedRealtime`, pins that recycling never needs it.

`registrations_` is an `IntrusiveRbTree` keyed on the descriptor, as epoll's `live_` is. It holds legacy registrations as well as async ones, and legacy registrations are unbounded, so no `std::vector` reservation could be relied on and the `insert()` that reallocated could be the one an `AsyncRead()` performed. The tree makes lookup O(log n) and insertion allocation-free.

The ownership split is the readiness backends': `all_` owns every state for the loop's lifetime, and the tree, the free list and the retired list are non-owning views, so moving a state between them is pointer assignment. A registration the `Async*` API created is handed back at the completion that leaves it with nothing outstanding — `MaybeRetireAsyncRegistration()`, as `ReadinessBackend` does it — so the pool is a high-water mark of simultaneously-outstanding requests rather than of distinct descriptors ever used. Releasing from inside a dispatch is safe because `ReleaseRegistration()` only parks the state; nothing is recycled until `ScrubRetiredRegistrations()` runs at the top of the next `Poll()`, with no callback on the stack and none of the `std::function`s it destroys executing.

### No shared base, and what that costs

epoll and kqueue share `internal::ReadinessBackend`: registration table, raw-request machinery, drain loop, dispatch rules. IOCP cannot, for two reasons. The obvious one is that the base is organised around readiness as the primitive. The load-bearing one is lifetime: a registration retired while the kernel still owns an `OVERLAPPED` inside it cannot be recycled until that completion is collected, or the next `AsyncRead()` to draw the slot shares an `OVERLAPPED` with an operation the kernel is still writing to. `ClearRetiredPending()` and `retired_` exist for that window; the shared table has no notion of a registration that is dead but not yet reclaimable.

The consequence is that the rules `aio.h` states have two implementations, and they drift. Instances caught and fixed:

- **The legacy hooks resolved a raw-only registration.** Only io_uring keeps legacy state in a separate table; the shared readiness backend keeps one `FdRegistration` holding both, as this backend keeps one `FdState`, and reproduces "not found" with an `async_only` flag, so `DeleteFd()` on a raw-only descriptor dies rather than proceeding. `GetActiveLegacyRegistration()` is that check under another name: `DeleteFd()`, `SetEvents()`, `EnableWritable()` and `DisableWritable()` go through it.
- **Exclusivity ran in neither direction completely.** `AsyncRead()` rejected only an `OnReadable` handler and `AsyncWrite()` only an `OnWritable` one; `OnReadable()` checked only a pending read, `OnWritable()` only a pending write, and `OnError()` — which collides with every raw request — checked nothing. All six directions now match the shared backend's messages, narrower checks first so the message names the collision.
- **A wakeup was posted per synchronously-failed request**, then one per batch, where none is needed: the list is drained at the top of `Poll()` before it waits on anything, and a `Poll()` that retires a request reports it. The surplus made the next `Poll()` report work with nothing to do. `NoCallbackRequestsRetireInOnePoll` pins it.
- **A read-side hangup was an error.** Above; the only fatal one.

None were visible from the shared tests until the suite ran on Windows — which is the point of that suite. `aio_test` is parameterised over every backend, and it is the only thing that holds the two implementations of the rules to the same answers; a difference it does not cover is one nobody sees until a caller does. Every drift above was fixed by first making the shared test fail on Windows.

## Consequences

- **Readiness costs a syscall the other backends do not pay**: a zero-byte `WSARecv()` per notification, plus a `MSG_PEEK` on completion.
- **`AsyncRead`/`AsyncWrite` accept sockets only**, so `RegularFileAsyncReadWriteTest` does not run here. Windows itself is fine with files (see Context); closing the gap means a `ReadFile`/`WriteFile` path and telling a file handle from a socket, which `IsSocket()` already does for the legacy API. A real feature, not yet built. `Pipe` is therefore a connected `AF_UNIX` socket pair (`pipe_windows.cc`), which incidentally makes each end readable and writable at once — what `RawReadAndWriteOnOneFdTakeTwoPolls` needs and gets from `socketpair()` elsewhere.
- **A descriptor can never be handed to a second `Aio`, and this cannot be diagnosed.** The second instance's completions go to the dead port and `Poll(true)` blocks forever. `AssociateSocket()` stays permissive; the constraint is documented instead.
- **Timer delivery is bounded by the platform clock interval, not nanoseconds.** Deadlines are held in nanoseconds, ordered exactly and requested to 100ns; delivery measured 35µs–1.2ms late on this hardware.
- **`--aio_backend` accepts any value here and always runs IOCP**, where Linux dies on an unknown one — the portability trap Decision 10 avoids, pointing the other way. CHECKing the value against `"iocp"` would close it and has not been done.
- **`--aio_queue_depth` is accepted and ignored.** The io_uring regression tests that set it to force an overflow still run, but with no overflow to provoke they degrade to liveness checks: a green `TimerPollsSurviveCqOverflow` on Windows is not evidence about overflow handling.
- **`GlibMainLoop` runs on Windows.** It needed the non-socket handle path plus a type bridge in `glib_main_loop.cc`, since `GPollFD::fd` is a `gint64` holding a `HANDLE`.
- **`aio_uv` and its tests remain incompatible on Windows**, as ADR 0004 notes — they run on Linux and macOS, and `BUILD` marks both `@platforms//:incompatible` here: libuv's loop on Windows _is_ a completion port nothing else can wait on, and `uv_poll` accepts only sockets.
- **The dispatch rules have a second implementation to keep honest.** The standing cost of the design, and the first place to look when a backend disagrees.

## Alternatives considered

- **A readiness shim over `WSAPoll()`/`select()`, feeding the existing `ReadinessBackend`.** Rejected twice over. It gives up IOCP, the platform's scalable primitive and the one that matches `Aio`'s contract, and makes every genuine `AsyncRead()` a synchronous read behind a readiness edge. And it cannot coexist with the completion half: `select()`/`WSAAsyncSelect()` leave notification state armed on the socket that conflicts with outstanding overlapped I/O and can wedge both (`pipe_windows.cc` says so at its one stateless `WSAPoll` call).
- **Auto-reset `Event`s for wakeups.** Rejected by measurement: after a registered wait fired, auto-reset came back _cleared_ (the kernel consumed it), manual-reset _still set_ (we consume it). Only the latter implements consume-then-notify or survives a dropped completion.
- **A semaphore instead of an Event**, restoring `signalfd`'s queue depth. Rejected: the contract deliberately coalesces, so depth would produce N callbacks for one burst.
- **`QueueUserAPC` for wakeups**, the closest analogue to a thread-directed signal. Not taken: APCs only run at alertable waits, so `Poll()` would become `GetQueuedCompletionStatusEx(..., fAlertable=TRUE)`, and cross-process delivery needs `OpenThread` with `THREAD_SET_CONTEXT` — a permissions question the named Event sidesteps.
- **`RegisterWaitForSingleObject`, the documented Win32 wait.** What this backend used first. It runs a callback on a pool thread — the only code here not on the caller's thread — and its teardown is a use-after-free unless every site uses `UnregisterWaitEx(…, INVALID_HANDLE_VALUE)`, the form that blocks until a running callback returns. Replaced by wait completion packets, which need neither. The cost is one `Nt*` dependency, the only one in AOS: undocumented in the Win32 sense and resolved by name at runtime. Accepted because the API has been stable since Windows 8, the thread pool itself depends on it, and it removes a thread and a class of bug.
- **A dedicated thread calling `WaitForMultipleObjects`.** Caps at `MAXIMUM_WAIT_OBJECTS` (64) per wait, so the general case needs a tree of threads, and it is a thread of our own to wake, synchronise and tear down.
- **`NtSetInformationFile(FileReplaceCompletionInformation)` to break the permanent association.** Would only turn "cannot" into "can, natively"; the caller constraint would still be the sane rule.
- **`WSADuplicateSocket` to work around it.** A duplicated handle _can_ join a second port, but the backend would then issue I/O on a different `SOCKET` than the caller holds, changing what a `FileDescriptor` means internally. The test that motivated it did not need it.
- **A process-wide socket→port registry, to make the two-`Aio` hang loud.** Deferred: `AssociateSocket()` is reachable from the no-malloc arming path, so the registry would have to be fixed-capacity. Not needed yet.
- **A waitable timer per armed timer.** The ordering requirement is a userspace property either way, and a kernel timer per armed timer buys nothing once the queue is ordered — ADR 0002's reason for one knote.
- **`GetQueuedCompletionStatus()`'s own timeout for the head deadline.** What this backend did first. Whole milliseconds, rounded up, then quantised to the tick: a sub-millisecond deadline always waited at least a millisecond, and a repeating timer ran late every cycle. The coarsening ADR 0002 rejected `uv_timer_t` for.
- **Splitting `FdState` into separate legacy and raw maps**, as io_uring does. One state per fd is the right shape when both share an `OVERLAPPED` and a socket association, and it is what the shared readiness backend does too; the lookups were narrowed instead.
- **Folding this backend into `internal::FdRegistrationTable`.** Deferred until the table grows a "can this be recycled yet" hook; before that it is either wrong for IOCP or leaks slots.

## Verification

- `//aos/events:aio_test` on `iocp`, at the time of writing: 96 of 105 `TEST_P` cases instantiated, 91 pass, 5 skipped as io_uring-specific (as on epoll and kqueue), stable over 60 consecutive runs. Not instantiated: five that `fork(2)`, `RegularFileAsyncReadWriteTest`, one macOS-only case, and the Linux-only pair. Everything else is shared with the other three backends, which is the point: the contract is the test.
- The permanent association, IOCP's support for regular files, auto- versus manual-reset under a kernel wait, wait-completion-packet semantics, high-resolution timer latency, and FIFO ordering among equal deadlines were each established by standalone probe before anything was written against them.
- The thread-pool wait's use-after-free, before it was designed out, was confirmed by instrumentation: it presented as `EPollLikeBasicWritable/iocp` dying in about 7% of full-suite runs with `0xC0000005` and no gtest output, clean in isolation. Poisoning the wait context at teardown instead of freeing it, and aborting in the callback on the poison, fired on **40 of 40 runs** — the race was constant; only allocator reuse decided whether it crashed, which is why it surfaced in an unrelated later test.
- `LegacyReadableSurvivesPollingPastHangup` was checked to fail without its fix: `Check failed: state.has_err_fn && state.err_fn … Received events = 0x8`.
- `//aos/ipc_lib:thread_signal_test` covers the wakeup primitive with no `Aio` in the way, which is what located the receiver fault in the `Aio` integration.

## Lessons learned

- **The platform whose model matches the API can still need the most emulation.** IOCP matches `Aio`'s completion surface exactly and still needed a readiness emulator, because the API carries more than its headline abstraction.
- **Emulating one I/O model on another leaks in the translation layer, not the logic underneath.** Every Windows-specific defect here — the zero-byte read's two meanings, the legacy/raw namespaces collapsing into one structure, the surplus wakeup per failed submit — was in the translation. None was in the dispatch below it.
- **A second implementation of a documented rule drifts, and shared tests do not catch it until they run.** Every divergence was invisible on Linux and obvious the moment the suite ran on Windows. A behavioural difference is worse than a missing feature: what Windows cannot do announces itself at compile time; what this backend accepted and every other rejected passed its own tests and would have failed somebody's port.
- **A crash in one test can belong to the previous test's teardown.** Everything pointed at `EPollLikeBasicWritable`: the only test that died, at its first line, clean 15/15 alone. It was collateral from a receiver test three tests earlier. When a failure only reproduces in a suite, the suspects are every test that ran before it.
- **An API named `Unregister…` need not mean the thing is finished.** `UnregisterWait()` returns while the callback still runs; only `UnregisterWaitEx(…, INVALID_HANDLE_VALUE)` waits. The cost was a use-after-free that fired on every run and crashed on almost none. A design with no callback thread was the better fix than the right flag.
- **Distinguish a test's subject from its vehicle before calling it impossible on a platform.** `RawRequestReusedOnASecondAioTest` looked like it needed something Windows cannot do, and two workarounds were costed against that reading. Its subject was the reused request; the shared descriptor was incidental.
- **A platform default tuned for desktop responsiveness is not neutral.** The 15.6ms tick does not degrade a repeating timer gracefully; it makes it permanently late.
- **Prose about coverage rots as silently as prose about behaviour.** A note here said `TimerCompletionUserDataIsNull` was Linux-gated; it always ran everywhere. ADR 0002 said the shared readiness backend covers "epoll, kqueue and IOCP"; `aio_windows.cc` references it zero times. An earlier draft of this document said the readiness backends keep legacy and raw state in separate maps; only io_uring does. And it said the high-resolution timer would be "100ns-granular"; measured, it is not. Nothing was wrong when written except the reach of the claim, and no test catches that.

## References

- ADR 0001: io_uring backend for Aio — `documentation/adr/0001-aio-io-uring-single-issuer.md`; the contract this backend is meeting.
- ADR 0002: kqueue backend for Aio on macOS — `documentation/adr/0002-aio-kqueue-macos.md`; the same exercise on a POSIX kernel, and the source of the readiness-masking and one-completion-per-`Poll()` decisions.
- ADR 0004: libuv guest loop — `documentation/adr/0004-aio-libuv-guest-loop.md`; why it has no Windows spelling.
- `aos/events/aio.h` — the event encoding, the no-malloc guarantee, and constraint 2 (a request may outlive its `Aio`).
- `aos/events/file_descriptor.h` — why a descriptor is `void *` here.
- `aos/ipc_lib/thread_signal.h` — the wakeup contract, and `thread_signal_windows.cc` for the named-Event rendezvous.
- `aos/events/aio_readiness_backend.h` — the shared seam this backend does not use, and the hook it would need.
- `aos/events/aio_windows.cc`, `aos/events/pipe_windows.cc`, `aos/ipc_lib/thread_signal_windows.cc`
