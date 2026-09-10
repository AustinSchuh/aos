# ADR 0004: libuv backend for Aio — running AOS as a guest on somebody else's event loop

## Status

Accepted. Implemented in `aos/events/aio_uv.h` and `aos/events/aio_uv.cc`, held to the `Aio` contract by `//aos/events:aio_test_uv` and covered past what that suite can reach by `aos/events/aio_uv_backend_test.cc`. Runs on Linux and macOS. `//aos/events:aio_uv` is incompatible with Windows only, where libuv's loop is an IOCP that nothing else can wait on.

## Context

`Aio` (`aos/events/aio.h`) is AOS's cross-platform async I/O primitive. ADR 0001 covers io_uring and epoll, ADR 0002 kqueue, ADR 0003 IOCP. Those four have something in common that this one does not: each of them _owns_ the waiting. `Poll()` is where the process blocks, and `Run()` is the program's main loop.

That is the wrong shape for embedding AOS in an application that already has an event loop. The available answer was to run both — a thread for the host's loop, a thread for AOS's, and a handoff on every message. That costs a wakeup per message and puts a queue between two schedulers, which is exactly the latency AOS exists to avoid.

The question this backend answers is therefore not "how does AOS wait on this platform" but "what does AOS look like when it is not the one waiting". Everything below follows from inverting that ownership:

- **The loop is borrowed, never owned.** It is not run, stopped, or closed here. Its owner drives it, and everything AOS registered runs when they do.
- **Libuv's callbacks arrive on turns of a loop this code does not control**, including the close callbacks that end a handle's life. Anything handed to libuv therefore has to outlive the object that registered it.
- **The only way to wait on a kernel object through libuv is `uv_poll` on a descriptor**, so everything AOS needs waking for has to be an fd.
- **`uv_timer_t` has millisecond resolution**, which is coarser than AOS schedules with.

## Decision

Add a fifth backend that is a guest rather than a host, and make the places where that differs loud rather than subtle.

1. **`UvAio` never calls `uv_run()`.** `Run()` and `Poll()` are fatal, with a message saying to run the owner's loop instead.
2. **`Aio` gains a protected constructor taking an `Impl`, and a virtual destructor** so a subclass can be owned through an `Aio *`. Nothing else becomes virtual: every method still forwards to `impl_`, so a subclass supplies a backend rather than overriding behaviour.
3. **libuv stays out of `//aos/events:aio`.** Only `//aos/events:aio_uv` depends on `@libuv`, and `aio_uv.h` forward-declares `uv_loop_s` so no consumer of it needs `<uv.h>`.
4. **Every handle put on the loop is heap allocated and freed from its own close callback.** A handle owned as a member could not survive long enough, because that callback runs on a later turn of a loop this object does not drive.
5. **Timers are a descriptor carrying the deadline, watched with `uv_poll`, not `uv_timer_t`.** A `timerfd` on Linux; the descriptor keeps the nanosecond deadline AOS schedules with and is just another thing to poll — the same trade `EpollImpl` makes.
6. **The wakeup is `uv_poll` on a descriptor the `ThreadSignalReceiver` can be observed through** — its `signalfd` on Linux, which is what `EpollImpl` does with it too.
7. **`Quit()` stops AOS's own handles and, by default, leaves the loop running.** `UvQuitBehavior::kStopLoop` is available for when AOS is the only thing on the loop.
8. **`AsyncRead`, `AsyncWrite` and `Cancel` are emulated on readiness**, the way `EpollImpl` does it. A completion with no descriptor left to wait on goes out through a `uv_idle_t`, so it still reaches the caller from a turn of the loop rather than from inside the call that produced it.
9. **The realtime malloc check is the loop owner's to suspend, not this backend's to assume.** `UvAio::ScopedLoopTurn` exempts libuv's own allocations across a turn and puts the check back around AOS's callbacks. With no such scope in effect the backend leaves the caller's realtime state alone — an `ShmEventLoop` that went realtime is still realtime when its handlers run.

## Design

### Refusing to drive the loop, and what that costs

The refusal is the cheap half; the expensive half is what it implies about lifetimes. `uv_close()` does not free a handle, it schedules a callback, and that callback runs on the owner's next turn of the loop. Nothing here can wait for it, because waiting would mean running the loop.

So every handle this backend creates — the `uv_poll_t`s, the `uv_prepare_t` behind `BeforeWait()`, the `uv_async_t` behind `kStopLoop` — is heap allocated and deletes itself from its own close callback. Nothing libuv can still reach may be a member of `UvAio` or of a registration, and the destructor closes handles without waiting for the closes to finish. `DeleteFdDefersFreeingItsHandle` is the test that holds this down: it cycles a registration through exactly that state, and the fixture's `uv_loop_close()` is what notices a leak or a double free.

### Timers: a descriptor carrying the deadline, not `uv_timer_t`

`uv_timer_t` is the obvious choice and is wrong here for one reason: it takes milliseconds. Every AOS deadline is absolute and nanosecond-exact, so routing timers through it would quietly coarsen all of them by up to a millisecond. A descriptor that carries the deadline keeps it intact and reduces a timer to what libuv is good at — something to poll. On Linux that descriptor is a `timerfd`; where the platform has none, the substitution is what the Consequences describe, and the paths above it never learn the difference.

### `Quit()` is a guest's Quit

Stopping the loop would end its owner's `uv_run()`, which AOS has no standing to decide: the loop has their work on it too, and an unexpected return from `uv_run()` is a behaviour change in their program, not in AOS's. So `Quit()` stops what AOS registered and leaves the loop alone. It is the one lifecycle method a guest implements rather than refuses — a guest has something of its own to stop, where it has nothing of its own to drive.

`UvQuitBehavior::kStopLoop` exists for the case where AOS _is_ the program and its exit is meant to end it. The `uv_stop()` goes through a `uv_async_t` rather than being called directly, because libuv handles are not thread safe and `uv_async_send()` is the one libuv call that is.

That argument decides the whole of `Quit()`, not just the `uv_stop()`. `aio.h` documents `Quit()` as callable from any thread and usable from a signal handler, and stopping poll handles is neither — it is loop-thread work, like every libuv handle operation except `uv_async_send()`. So `Quit()` does only what that promise allows: a relaxed store to `should_run_` and a `uv_async_send()`. The handle work — stopping the polls, stopping the prepare, and `uv_stop()` under `kStopLoop` — happens in the async's callback, on the thread that owns the handles. The async is therefore created in both quit behaviours, not just `kStopLoop`.

Coalescing comes free: any number of sends before the loop turns produce one callback, which is what `Quit()`'s documented stickiness wants anyway.

### Errors are reported before readiness

`uv_poll`'s callback reports an error as a negative status rather than as an event bit. It is dispatched first, ahead of the readable and writable callbacks, so those never act on a descriptor that is already broken.

### The completion API on a readiness backend

libuv reports readiness, so `AsyncRead`/`AsyncWrite` park a handler that does the I/O when the descriptor is ready and complete the request from there — the same emulation `EpollImpl::AsyncRead()` performs. Three things differ from `ReadinessBackend`, and all three follow from not driving the loop:

- **Completions are not rationed.** A completion runs from the loop callback that produced it, so one turn can deliver several. One per call is something only a `Poll()` can promise, and there is none here.
- **A completion with nothing left to wait on** — a submit that resolved immediately, and `Cancel()` — goes out through a `uv_idle_t`. An idle handle rather than the prepare one: while it is active libuv polls with a zero timeout, so the turn that delivers the completion is a turn that ends. They are threaded through the request's own link, so queueing one allocates nothing.
- **Submitting allocates**, in libuv's bookkeeping and in the registration beside it, where the other backends submit to a ring they preallocated. Those backends promise a realtime caller that submitting allocates nothing; this one cannot, so it turns the check off for the submit rather than quietly tripping it. Completion callbacks stay outside that: what a caller does with its own data is still held to the promise.

What a guest cannot promise at all is in the contract suite as skips rather than as failures: `Poll()`'s rationing and reentry, surviving `fork()`, and catching teardown with work still registered. Each is a property of the loop belonging to somebody else, not something left unfinished here.

## Consequences

- **AOS work becomes work the host loop already waits on**, with no second thread and no handoff. That is the point of the backend.
- **`Aio` becomes polymorphic.** The virtual destructor adds a vtable, and a vptr to every instance — the hosting backends' included. Nothing depended on its absence. (It was never trivially destructible; it owns its `Impl` through a `unique_ptr`.)
- **The guest's refusal of `Run()` and `Poll()` is a runtime abort, not a compile error.** Driving stays on `Aio`'s base interface, so the call that aborts is writable. Why that trade is kept — and what would reopen it — is under "Taking driving out of `Aio`'s interface" below.
- **A `UvAio` cannot be driven for testing without a real libuv loop**, since the usual `Poll()` loop is fatal. `aio_test_lib.cc`'s `TestAio` owns one and turns it over in place of `Poll()`; `aio_uv_backend_test.cc` runs `uv_run()` directly.
- **Handles outlive their registrations by design**, so a leak here shows up as a `uv_loop_close()` failure rather than as anything closer to the mistake. The test fixture exists to make that noticeable.
- **The backend runs anywhere libuv can wait on a descriptor, which is everywhere but Windows.** On Linux the two things AOS needs waking for already are descriptors, a `timerfd` per timer and the receiver's own `signalfd`. macOS has neither, and is supported anyway because a kqueue is itself a descriptor: one holding a single `EVFILT_TIMER` stands in for the timerfd and one holding a single `EVFILT_SIGNAL` for the signalfd, so below six helpers behind an `#ifdef` there is only ever an fd. ADR 0002 covers that substitution in full. Windows is the exception that stands: libuv's loop is an IOCP there, and its own documentation rules the mechanism out -- "on windows only sockets can be polled with poll handles".

## Alternatives considered

- **Two event loops and a bridge.** The status quo this replaces: a thread each and a handoff per message. Correct, and it costs a wakeup and a queue on every message, between two schedulers that then cannot be reasoned about together.
- **`uv_timer_t` for timers.** Rejected for its millisecond resolution; see above. ADR 0001 reaches the same structure — a `timerfd` rather than the framework's native timer — but for a different defect: io_uring's timeout ops accumulated phase error, where `uv_timer_t` merely rounds. Same conclusion, different reason.
- **Making `Aio` virtual throughout**, so a backend could override behaviour rather than supply an implementation. Not taken: the methods are the contract, and letting a subclass change them would make `Aio` mean something different depending on which one you have. A protected constructor taking an `Impl` gives the extension point without that.
- **Taking driving out of `Aio`'s interface altogether**, so a guest has nothing to refuse. `EventLoop` sets the precedent: it has no `Run()` — only `OnRun()` to register a callback — and stopping is an `ExitHandle` (`aos/events/event_loop.h`) a caller is handed, not a method on the interface. Driving lives on the concrete implementations that can drive because, as the comment above `ExitHandle` puts it, different implementations provide it at different scopes, or possibly not at all. Under that split a guest backend would simply lack `Run()` and `Poll()`, and the two fatal stubs and their death tests would be a compile error instead.

  **Accepted as the right long-term shape, and deferred.** `EventLoop` reached the same conclusion for the same reason, and this backend is the second piece of evidence for it. What defers it is churn, not doubt: too much has moved recently to re-architect every layer that composes an `Aio` on top of it.

  What deferring costs, and what it does not:

  - `Quit()` was never part of the problem. It is not a driving method, and this backend implements it — stop what AOS registered — rather than refusing it; only `Run()` and `Poll()` are stubs. `ExitHandle` is not an alternative to `Quit()` so much as a consumer of it: the handle `ShmEventLoop::MakeExitHandle()` returns bottoms out in `Aio::Quit()`. At the bottom of that stack something has to expose the raw stop, and `Aio` is the bottom.
  - `Poll()` is the contract's spine, but barely any of its surface. The one-completion-per-call rule, the EINTR behaviour, and the cancel-then-drain rule — `Cancel()` a request and keep `Poll()`ing until its callback runs, constraint 2 in `aio.h` — are all written against it, so it cannot leave the contract. It can leave the _interface_ cheaply though: outside tests it has one caller (`aos/network/web_proxy.cc`), plus `EPoll::Poll()` forwarding, and `ShmEventLoop` reaches `Quit()` exactly once. The stubs are loud in the test suite and nearly invisible in production code, which is the other half of why waiting costs little.
  - The split does not transplant. `EventLoop`'s works because a caller who drives holds the concrete type; `Aio`'s four hosting backends are selected at runtime by `--aio_backend` behind one concrete type, so "move driving to the implementations" here means splitting the class into a registration base and a drivable type — and every layer that forwards through an `Aio *` splits with it. `EPoll` forwards `Run()` and `Poll()` through the borrowed `Aio *` its second constructor takes (`aos/events/epoll.cc`), so under the split it either demands the drivable type, and a guest can no longer sit behind its registration API at all, or it faces this same choice one level up. The stubs would not disappear; they would relocate.

  So the split is worth doing and is not worth doing now: a call that cannot be written beats one that aborts loudly, but it is bought with an interface split across every composing layer, against a call that today dies with a message saying what to run instead. What would move it up the list is the refusal firing somewhere other than a death test, or a third backend arriving that cannot drive.

- **Owning the loop, and offering the host a descriptor to wait on instead.** This is the inverse embedding: AOS runs its own loop and exposes an fd the host polls. It works only for hosts whose loops can watch a foreign descriptor, and the host still has to be taught to call back into `Poll()` when that descriptor wakes — the bridge's dispatch protocol, minus the thread. Being the guest asks nothing of the host's loop beyond what it already does for its own work.

## Verification

`//aos/events:aio_test_uv` is the suite that documents what the `Aio` interface promises — `aio_test_lib.cc`, compiled once into `//aos/events:aio_test_lib` — instantiated against this backend by `aio_test_uv.cc`, which owns a loop per `Aio` and turns it over where the other backends `Poll()`. The bodies are shared rather than conditional: what a guest cannot promise is a field on the backend parameter, not an `#ifdef` beside each test. That is what says whether `UvAio` delivers the contract rather than merely compiles against it, and it is what found the divergences this backend would otherwise have shipped with: error strings that did not match the other backends', a missing reentry check on `BeforeWait()`, registration-kind checks that were not made, and output events that waited for a handler before arming. 93 of its tests pass; the 11 a guest cannot promise skip with the reason.

`aio_uv_backend_test.cc` covers what that suite structurally cannot reach. `TestAio` supplies `Run()` and `Poll()`, so the refusals (`RunIsRefusedDeathTest`, `PollIsRefusedDeathTest`) have to be tested somewhere nothing hides them. `QuitLeavesTheBorrowedLoopUsable` covers the default `Quit()`, which leaves the host's loop running where `TestAio` stops it. `DeleteFdDefersFreeingItsHandle` cycles a registration through the state where libuv still owns the poll handle, which is what leaks or double-frees if the deferral is wrong, and what the fixture's `uv_loop_close()` notices. `QuitRacesALiveLoopSafely` covers `Quit()` arriving from another thread while `uv_run()` is live on a second one, which is the shape the contract promises and the one an inline `uv_poll_stop()` gets wrong: under `--config=tsan` the inline version reports a data race in `uv__io_stop`.

## Lessons learned

- **"Who owns the waiting" is an architectural question, not an implementation detail.** Four backends hid it because the answer was always "we do". Writing one where it is not forced the `Impl` seam to become a real extension point.
- **A callback you cannot wait for changes who may own what.** Refusing to call `uv_run()` is a one-line decision that dictates heap allocation for every handle in the backend.
- **A backend is not finished when it compiles against an interface.** Running the contract suite against this one is what found the divergences it would otherwise have shipped with, and most of them were wording and missing checks — the kind of thing only a test written against the interface, rather than against the implementation, can see.

## References

- ADR 0001: io_uring backend for Aio — `documentation/adr/0001-aio-io-uring-single-issuer.md`
- ADR 0002: kqueue backend for Aio on macOS — `documentation/adr/0002-aio-kqueue-macos.md`
- ADR 0003: IOCP backend for Aio on Windows — `documentation/adr/0003-aio-iocp-windows.md`
- `aos/events/aio.h` — the contract this backend implements
- `aos/events/aio_uv.h`, `aos/events/aio_uv.cc`, `aos/events/aio_uv_backend_test.cc`
- `aos/events/aio_test_lib.cc` — the contract suite, run against this backend as `//aos/events:aio_test_uv`
- `aos/events/event_loop.h` — `OnRun()` and `ExitHandle`, the split the driving-interface alternative weighs
- `aos/events/epoll.h`, `aos/events/epoll.cc` — the wrapper that forwards driving through a borrowed `Aio *`
