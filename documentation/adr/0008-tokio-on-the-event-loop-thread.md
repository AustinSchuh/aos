# ADR 0008: Tokio as a guest on the event loop's thread, driven by its Aio

## Status

Accepted. Implemented on Linux:

- **Implemented:** `Aio::WaitFd()` and `PrepareToWait()` on io_uring and epoll, and their C API; the `aos_events_tokio` crate (`aos/events/tokio_event_loop.rs`), with `TokioShmEventLoop` for shm and `run_simulation()` for simulation; the scheduler's 5 ms slices at an infinite replay rate, and `StartExternalRun()`, `FinishExternalRun()` and `EndSlice()`. ADR 0007's `robot_server` runs on it.
- **Not yet implemented:** kqueue's `WaitFd()` and `PrepareToWait()` are written but untested, since nothing here builds for macOS. IOCP refuses both (decision 8).

## Context

AOS processes want the Rust async ecosystem: axum and hyper for the robot's web server (ADR 0007), rustls for TLS, tungstenite for websockets, rmcp for MCP. Almost all of it is built on tokio, and much of it (`tower-http`'s `ServeDir`, `tokio::fs`, rmcp's HTTP transport) needs a running tokio runtime.

AOS objects are single-threaded. Every watcher, fetcher, sender and timer belongs to its event loop's thread, and callbacks run there.

The AOS Rust runtime (`aos/events/event_loop_runtime.rs`) can't host tokio code. It runs one task, polls it when any AOS event fires, and gives it a waker that panics, so a future that isn't an AOS object is never polled again.

Running tokio on a thread of its own, joined to the AOS thread by queues, works, but every request that touches AOS costs two context switches, and it can't run in simulation: nothing on the tokio thread knows about simulated time.

## Decision

1. **Tokio runs as a guest on the event loop's thread.** A current-thread tokio runtime with a `LocalSet` is the outer loop, and the event loop's `Aio` is one of its event sources, so AOS callbacks run inside tokio, between tokio tasks.
2. **`Aio` gains an interface for being driven from outside**, alongside `Poll(false)`:
   - `WaitFd()`: an fd that becomes readable when `Poll(false)` has work: the ring's fd for io_uring, the epoll fd, or the kqueue fd.
   - `PrepareToWait()`: called right before the outer loop waits. It runs the before-wait functions, submits anything staged since the last `Poll()`, and returns either "poll now" or a deadline to wake by.

   The driver drains with `Poll(false)`, calls `PrepareToWait()`, and waits until `WaitFd()` is readable or the deadline passes.

   **On io_uring, `WaitFd()` moves the ring to the `COOP_TASKRUN | TASKRUN_FLAG` tier**, the one `Aio` already uses for a loop built on one thread and run on another, so it has to come before any raw request. Under `SINGLE_ISSUER | DEFER_TASKRUN`, finished work waits for a `GETEVENTS` enter, so the ring's fd never becomes readable for it. An eventfd registered with the ring is signaled 20–40 ms late when the work came from another eventfd's wakeup (`Quit()`, a thread signal). On the COOP tier, a finished request wakes the ring's owner wherever it sleeps, without an IPI, and lands in the CQ, where the ring's fd sees it. Whoever finishes the request still pays one `try_to_wake_up()` per batch, as under `DEFER_TASKRUN`. The loop gives up `DEFER_TASKRUN`'s batching, which a non-realtime loop doesn't need; realtime loops keep it.

3. **Work that leaves the thread comes back through tokio.** The blocking pool (`ServeDir`'s file reads), `Send` tasks and other runtimes wake a local task's waker, and tokio's own driver wakes the thread. No AOS-side mechanism is needed.
4. **AOS futures get real wakers, in a new crate.** `aos_events_tokio` gives tokio tasks fetchers, senders, watches and timers on the C API. A watch is a no-argument watcher whose callback only wakes the task, plus a fetcher the task reads with: a watcher's message is only valid inside its callback, and the task runs after it returns. AOS objects stay `!Send`, so only local tasks can use them. AOS won't destroy fetchers or senders while an event loop runs, so ones dropped then wait until it stops. `event_loop_runtime.rs` stays as it is.
5. **One driver for shm and simulation.** The driver registers `WaitFd()` with `AsyncFd`, calls `PrepareToWait()` from tokio's `on_thread_park` hook, sleeps until the deadline, and calls `Poll(false)`. It drives `ShmEventLoop`'s `Aio` in a process, and `scheduler_aio()` in a simulation, through `SimulatedEventLoopFactory::StartExternalRun()` and `FinishExternalRun()`, which split `Run()` around the waiting.

   **In simulation, simulated time only moves on when tokio has nothing to run.** The simulation runs from tokio's park hook, which tokio calls only when no task is ready. An AOS callback that wakes a task also calls the scheduler's `EndSlice()`, so tokio runs the task at that event's simulated time before the simulation carries on.

6. **The simulation scheduler yields to its `Aio` between slices of events.** At an infinite replay rate, the scheduler used to run every event inside one timer callback. It runs 5 ms slices instead, so the scheduler's fds and the guest runtime get a turn. Finite replay rates don't change.
7. **Realtime processes don't host tokio.** The guest runtime is for non-realtime processes: the web server, MCP, tools.
8. **Windows comes later, without a redesign.** IOCP has no readiness fd, and mio can't take a foreign completion port, so the driver will need a helper thread that waits on `Aio`'s completion port and wakes tokio. The driver takes its wakeup source as an interface for that.

## Design

```
               one thread
  +--------------------------------------------------------------+
  |  tokio current-thread runtime + LocalSet                     |
  |    tasks: HTTP, TLS, websockets, MCP ...                     |
  |      | use AOS objects directly (!Send, local tasks only)    |
  |      v                                                       |
  |  aos_events_tokio driver: AsyncFd(aio.WaitFd()),             |
  |    PrepareToWait() on park, sleep until its deadline,        |
  |    then aio.Poll(false)                                      |
  |      |                                                       |
  |      v                                                       |
  |  Aio --> AOS callbacks (watchers, timers, shm wakeups,       |
  |          simulated events) --> wake AOS futures              |
  +--------------------------------------------------------------+
        ^ tokio's own wakers, only from work that left the thread:
        | spawn_blocking, Send tasks, other runtimes
```

**A request.** A browser asks for a channel's latest message. A local task accepts the connection, does TLS, parses the request, calls a fetcher, and writes the response, all on one thread.

**A watched channel streamed to a browser.** A local task awaits a watch. Each message wakes it; it fetches the message and writes it to the websocket. It chooses whether to drop, coalesce or wait when the browser is slow.

**Simulation.** tokio's park hook drives `scheduler_aio()` whenever no task is ready. An event that wakes a task ends the slice, and the task runs at that event's simulated time. `monotonic_now()` is simulated time; network I/O runs in real time, as `web_proxy` does during replay.

## Consequences

Positive:

- The tokio ecosystem works unmodified in AOS processes.
- Using AOS from tokio code costs a function call, not two context switches.
- Shm, simulation and log replay share one driver.

Trade-offs:

- Tokio's own I/O goes through mio's epoll or kqueue, and AOS's I/O stays on `Aio`, on io_uring's COOP tier.
- `tokio::fs` and `spawn_blocking` use tokio's thread pool, so in simulation their timing against simulated events isn't deterministic.
- At an infinite replay rate, the scheduler's yield costs a little replay speed.
- Windows waits for the IOCP helper thread.

## Alternatives considered

- **Two threads joined by queues.** Two context switches per request, and no simulation.
- **An AOS-native executor with a runtime-agnostic stack** (hyper, axum's `Router`, rustls). No extra threads, but no `ServeDir` or rmcp HTTP transport, and the same wall for every later integration.
- **`Aio` hosting tokio, stepping tokio from AOS.** Tokio has no public way to run its ready tasks without blocking, or to report its next deadline.
- **Tokio on top of `Aio` as its reactor.** Tokio's driver isn't pluggable; this would mean forking tokio.

## Tried, and why not

- **A registered eventfd as `WaitFd()` on io_uring**, keeping `SINGLE_ISSUER | DEFER_TASKRUN`. The ring's own fd never became readable for finished work under `DEFER_TASKRUN`, and an eventfd registered with the ring woke the loop 23–40 ms late after a `Quit()` or a thread signal. Hence the COOP tier.
- **Running the simulation eagerly between tokio's turns.** Simulated time ran hundreds of seconds ahead of the task an event had woken. The simulation now runs only from tokio's park hook, and an event that wakes a task ends the slice.
- **The scheduler draining every event in one callback** at an infinite replay rate, as it did. Anything else on its `Aio` (log replay's sockets, and the guest runtime) starved for the whole run. Hence the 5 ms slices.
- **Spawning tasks from `run()`'s setup function.** Setup runs before the `LocalSet` exists, so `spawn_local()` there panics. Setup makes AOS objects; the future it returns spawns tasks.
- **Rust AOS binaries' defaults.** `aos::Init` set glog's `logtostderr`, which absl doesn't have, so every Rust binary died at startup; and Rust logged only errors by default, so `log::info!` was silent. Both are fixed alongside this work, so a server's logs show up next to C++'s.

## Verification

Done, on an Orin (6.18-rt) and a 7.1 desktop kernel:

- `aio_test`'s `ExternalDriver*` tests: io_uring on the COOP tier and epoll both wake the outer loop on time for a finished read, a legacy fd, a timer, and `Quit()` from another thread, submit staged work, and run before-wait functions. With `DEFER_TASKRUN`, the ring's fd never became readable for a finished read; with a registered eventfd, wakeups were 23–40 ms late.
- `tokio_event_loop.rs`'s tests: an AOS timer wakes a task on time; bytes from a TCP socket sent on an AOS channel reach a task watching it on another event loop; `exit()` ends `run()`; in simulation, a ping sent one simulated second in reaches its watcher at exactly that simulated time.
- `InfiniteRateYieldsToSchedulerAio` and `ExternalRunMatchesRun` for decisions 5 and 6.

Still to do:

- The same tests on macOS (kqueue).
- How much the scheduler's yield costs on a long log replay.

## References

- ADR 0001 (io_uring backend), ADR 0002 (kqueue), ADR 0003 (IOCP), ADR 0007 (the robot server).
- `aos/events/aio.h`, `aos/events/event_scheduler.cc`, `aos/events/event_loop_runtime.rs`.
- tokio-uring, which drives an io_uring ring from a tokio current-thread runtime the same way.
