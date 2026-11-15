#![warn(unsafe_op_in_unsafe_fn)]

//! Tokio as a guest on an AOS event loop's thread.
//!
//! A current-thread tokio runtime runs on the same thread as the event loop, as the outer loop.
//! The event loop's `Aio` is one of tokio's event sources: the driver waits for
//! `Aio::WaitFd()` with [`AsyncFd`], and calls `Aio::Poll(false)` when it is readable, so AOS
//! callbacks run inside tokio, between tokio tasks. Tasks use AOS objects directly, with no
//! thread hand-offs. See `documentation/adr/0008-tokio-on-the-event-loop-thread.md`.
//!
//! AOS objects here are `!Send`: only tasks on this thread (`tokio::task::spawn_local`) can hold
//! them. Work sent elsewhere (`spawn_blocking`, another runtime) comes back through tokio's own
//! channels and wakers.
//!
//! On io_uring, the event loop's ring runs on the `COOP_TASKRUN` tier rather than
//! `DEFER_TASKRUN`: its fd only polls readable for finished work there (see `Aio::WaitFd()`).
//!
//! ```no_run
//! # use aos_events_tokio::*;
//! # use std::path::Path;
//! # use aos_configuration::read_config_from;
//! let config = read_config_from(Path::new("path/to/aos_config.json")).unwrap();
//! let event_loop = TokioShmEventLoop::new(&config);
//! event_loop.run(|aos| {
//!     // Make AOS objects here, before the event loop starts.
//!     let mut pings = aos.make_watch("/test", "aos.examples.Ping");
//!     async move {
//!         while let Some(message) = pings.next().await.data() {
//!             println!("Got a {}-byte ping", message.len());
//!         }
//!     }
//! });
//! ```

use std::{
    cell::{Cell, RefCell},
    ffi::CString,
    future::Future,
    marker::PhantomData,
    os::{
        fd::{AsRawFd, RawFd},
        raw::c_void,
    },
    pin::Pin,
    rc::Rc,
    sync::{
        atomic::{AtomicI64, Ordering},
        Arc,
    },
    task::{Context as TaskContext, Poll, Waker},
    time::Duration,
};

use aos_configuration::Configuration;
use aos_configuration_fbs::aos::Configuration as RustConfiguration;
use aos_flatbuffers::{transmute_table_to, Flatbuffer};
use tokio::{io::unix::AsyncFd, sync::Notify};

use aos_events_event_loop_c::{
    aos_aio_create, aos_aio_destroy, aos_aio_poll, aos_aio_prepare_to_wait, aos_aio_should_run,
    aos_aio_t, aos_aio_wait_fd, aos_context_t, aos_error_code, aos_error_destroy, aos_error_t,
    aos_event_loop_add_timer, aos_event_loop_destroy, aos_event_loop_make_fetcher,
    aos_event_loop_make_no_arg_watcher, aos_event_loop_make_sender, aos_event_loop_monotonic_now,
    aos_event_loop_node, aos_event_loop_t, aos_exit_handle_destroy, aos_exit_handle_exit,
    aos_exit_handle_t, aos_fetcher_context, aos_fetcher_destroy, aos_fetcher_fetch,
    aos_fetcher_fetch_next, aos_fetcher_t, aos_monotonic_clock_now, aos_node_t, aos_sender_destroy,
    aos_sender_send_copy, aos_sender_t, aos_shm_event_loop_create_on_aio,
    aos_shm_event_loop_make_exit_handle, aos_shm_event_loop_shutdown, aos_shm_event_loop_startup,
    aos_simulated_event_loop_factory_end_slice,
    aos_simulated_event_loop_factory_finish_external_run,
    aos_simulated_event_loop_factory_is_running, aos_simulated_event_loop_factory_make_exit_handle,
    aos_simulated_event_loop_factory_scheduler_aio,
    aos_simulated_event_loop_factory_start_external_run, aos_simulated_event_loop_factory_t,
    aos_timer_handler_schedule, aos_timer_handler_t,
};
use aos_events_simulated_event_loop::SimulatedEventLoopFactory;

/// The `Aio` an event loop runs on, driven from tokio.
struct Aio(*mut aos_aio_t);

impl Aio {
    fn poll(&self) -> bool {
        // SAFETY: self.0 is a live Aio, and this is the thread driving it.
        unsafe { aos_aio_poll(self.0, false) }
    }

    fn should_run(&self) -> bool {
        // SAFETY: as above.
        unsafe { aos_aio_should_run(self.0) }
    }

    /// `Aio::PrepareToWait()`: `None` to poll again now, or the deadline to wake by, in
    /// nanoseconds on the real monotonic clock (`i64::MAX` for none).
    fn prepare_to_wait(&self) -> Option<i64> {
        let mut poll_now = false;
        let mut deadline_ns = i64::MAX;
        // SAFETY: as above, and the out-pointers are to locals.
        unsafe { aos_aio_prepare_to_wait(self.0, &mut poll_now, &mut deadline_ns) };
        if poll_now {
            None
        } else {
            Some(deadline_ns)
        }
    }

    fn wait_fd(&self) -> RawFd {
        // SAFETY: as above.
        let fd = unsafe { aos_aio_wait_fd(self.0) };
        RawFd::try_from(fd).expect("Aio's wait fd isn't a POSIX fd")
    }
}

/// `Aio::WaitFd()`, for [`AsyncFd`].
struct WaitFd(RawFd);

impl AsRawFd for WaitFd {
    fn as_raw_fd(&self) -> RawFd {
        self.0
    }
}

/// What the driver and tokio's park hook share.
struct ParkState {
    /// The deadline the driver is currently sleeping until, in nanoseconds on the real monotonic
    /// clock.
    armed_deadline_ns: AtomicI64,
    /// Wakes the driver to look again, when the park hook finds work or an earlier deadline.
    rearm: Notify,
}

/// A raw pointer which tokio's park hook may hold.
///
/// The hook has to be `Send + Sync`, but on a current-thread runtime it only ever runs on the
/// thread that calls `block_on()`, which is the event loop's thread.
struct AioForParkHook(*mut aos_aio_t);

impl AioForParkHook {
    /// A method rather than a field access, so closures capture the whole (Send) wrapper rather
    /// than the raw pointer inside it.
    fn aio(&self) -> Aio {
        Aio(self.0)
    }
}

// SAFETY: See the type's documentation: it's only used on the event loop's thread.
unsafe impl Send for AioForParkHook {}
// SAFETY: As above.
unsafe impl Sync for AioForParkHook {}

/// How many AOS callbacks to run before letting tokio's tasks have a turn.  Poll() runs one each,
/// and something can keep it busy forever: a simulation at an infinite replay rate rearms its
/// timer for "now" after every slice of events.
const POLLS_PER_TURN: usize = 64;

/// Drives `aio` until `keep_going` says to stop: drains it with `Poll(false)`, asks
/// `PrepareToWait()`, and waits for its fd, its deadline, or the park hook.
async fn drive(aio: &Aio, wait_fd: RawFd, park: &ParkState, keep_going: &dyn Fn(&Aio) -> bool) {
    // SAFETY: The Aio owns the fd and outlives this future: run() and run_simulation() drop the
    // future before the Aio can go.
    let wait_fd =
        unsafe { AsyncFd::register_with_interest(WaitFd(wait_fd), tokio::io::Interest::READABLE) }
            .expect("Failed to register the Aio's wait fd with tokio");
    loop {
        let mut polls = 0;
        while polls < POLLS_PER_TURN && aio.poll() {
            if !keep_going(aio) {
                return;
            }
            polls += 1;
        }
        if !keep_going(aio) {
            return;
        }
        if polls == POLLS_PER_TURN {
            tokio::task::yield_now().await;
            continue;
        }
        let Some(deadline_ns) = aio.prepare_to_wait() else {
            // Poll() has more to deliver right away, but let tokio's tasks run first.
            tokio::task::yield_now().await;
            continue;
        };
        park.armed_deadline_ns.store(deadline_ns, Ordering::Relaxed);
        let sleep = sleep_until_ns(deadline_ns);
        tokio::select! {
            guard = wait_fd.readable() => {
                // Readiness is level-triggered on the Aio's side: Poll() does the consuming, so
                // just ask tokio to check again next time.
                guard.expect("Waiting on the Aio's wait fd failed").clear_ready();
            }
            _ = sleep => {}
            _ = park.rearm.notified() => {}
        }
    }
}

/// An AOS `ShmEventLoop` with a tokio runtime as its outer loop. See the module documentation.
pub struct TokioShmEventLoop<'config> {
    aio: Aio,
    event_loop: *mut aos_event_loop_t,
    _config: PhantomData<&'config Configuration>,
}

impl Drop for TokioShmEventLoop<'_> {
    fn drop(&mut self) {
        // SAFETY: Both are live and owned here, and the event loop goes first because it is
        // registered with the Aio.
        unsafe {
            aos_event_loop_destroy(self.event_loop);
            aos_aio_destroy(self.aio.0);
        }
    }
}

impl<'config> TokioShmEventLoop<'config> {
    /// Makes the event loop, on an `Aio` of its own.
    pub fn new(config: &'config impl Flatbuffer<RustConfiguration<'static>>) -> Self {
        // SAFETY: The configuration outlives the event loop ('config), and the Aio outlives the
        // event loop (see Drop).
        unsafe {
            let aio = aos_aio_create();
            let event_loop = aos_shm_event_loop_create_on_aio(
                transmute_table_to::<Configuration>(&config.message()._tab),
                aio,
            );
            Self {
                aio: Aio(aio),
                event_loop,
                _config: PhantomData,
            }
        }
    }

    /// Runs the event loop and tokio on this thread.
    ///
    /// `setup` runs first, before the event loop starts: make watches, fetchers, senders and
    /// timers there. The future it returns runs on a tokio `LocalSet` once the event loop has
    /// started, and may `spawn_local()` more tasks.
    ///
    /// Returns the future's output, or `None` if the event loop was told to stop first (an
    /// [`AosHandle::exit()`], or ^C).
    ///
    /// # Panics
    ///
    /// Panics if the event loop reports an error when it shuts down.
    pub fn run<F, Fut>(self, setup: F) -> Option<Fut::Output>
    where
        F: FnOnce(&AosHandle) -> Fut,
        Fut: Future + 'static,
    {
        // First, before setup or startup can submit anything: on io_uring this moves the ring to
        // the tier an outer loop can wait on, which can't carry raw requests across.
        let wait_fd = self.aio.wait_fd();

        let handle = AosHandle {
            inner: HandleInner::new(self.event_loop, ExitSource::Shm),
        };
        let user_future = setup(&handle);

        let (runtime, park) = build_runtime(&self.aio);

        // SAFETY: The event loop is live, and the Aio it borrows hasn't been polled yet.
        unsafe { aos_shm_event_loop_startup(self.event_loop) };

        let local = tokio::task::LocalSet::new();
        let output = local.block_on(&runtime, async {
            tokio::select! {
                output = user_future => Some(output),
                _ = drive(&self.aio, wait_fd, &park, &|aio| aio.should_run()) => None,
            }
        });
        // Tasks the user spawned may hold AOS objects; they go before the event loop shuts down.
        drop(local);
        drop(runtime);

        // SAFETY: The event loop is live, started, and its Aio won't be polled again.
        let error = unsafe { aos_shm_event_loop_shutdown(self.event_loop) };
        check_shutdown_error(error, "The event loop shut down");
        handle.inner.finish();
        output
    }
}

/// Panics if `error` (owned) isn't null.
fn check_shutdown_error(error: *mut aos_error_t, what: &str) {
    if !error.is_null() {
        // SAFETY: We own the error that came back.
        let code = unsafe {
            let code = aos_error_code(error);
            aos_error_destroy(error);
            code
        };
        panic!("{what} with error code {code}");
    }
}

/// Builds the current-thread runtime that drives `aio`, with the park hook that calls
/// `PrepareToWait()`.
fn build_runtime(aio: &Aio) -> (tokio::runtime::Runtime, Arc<ParkState>) {
    let park = Arc::new(ParkState {
        armed_deadline_ns: AtomicI64::new(i64::MAX),
        rearm: Notify::new(),
    });
    let park_hook_state = park.clone();
    let park_hook_aio = AioForParkHook(aio.0);
    let runtime = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        // Runs right before tokio parks, which is where the event loop is about to wait: run
        // the before-wait functions, hand over what's staged, and make sure the driver isn't
        // sleeping past something due.  tokio checks for ready tasks again after this, so
        // waking the driver here keeps it from parking.
        .on_thread_park(move || {
            let aio = park_hook_aio.aio();
            let wake_driver = match aio.prepare_to_wait() {
                None => true,
                Some(deadline_ns) => {
                    deadline_ns < park_hook_state.armed_deadline_ns.load(Ordering::Relaxed)
                }
            };
            if wake_driver {
                park_hook_state.rearm.notify_one();
            }
        })
        .build()
        .expect("Failed to build the tokio runtime");
    (runtime, park)
}

/// A simulation factory pointer which tokio's park hook may hold; see [`AioForParkHook`].
struct FactoryForParkHook(*mut aos_simulated_event_loop_factory_t);

impl FactoryForParkHook {
    fn is_running(&self) -> bool {
        // SAFETY: The factory outlives the runtime whose hook this is.
        unsafe { aos_simulated_event_loop_factory_is_running(self.0) }
    }
}

// SAFETY: As for AioForParkHook: only used on the simulation's thread.
unsafe impl Send for FactoryForParkHook {}
// SAFETY: As above.
unsafe impl Sync for FactoryForParkHook {}

/// Builds the current-thread runtime for a simulation, whose park hook runs the simulation.
///
/// Simulated time only moves on when tokio has nothing to run.  tokio calls the hook right before
/// it parks, when no task is ready; the hook runs simulated events until one wakes a task, which
/// also ends the scheduler's slice right after that event (see `wake_callback`).  tokio checks
/// for ready tasks again after the hook, so the woken task runs at that event's simulated time,
/// and the hook carries on once tokio is idle again.
fn build_simulation_runtime(
    aio: &Aio,
    factory: *mut aos_simulated_event_loop_factory_t,
) -> (tokio::runtime::Runtime, Arc<ParkState>) {
    let park = Arc::new(ParkState {
        armed_deadline_ns: AtomicI64::new(i64::MAX),
        rearm: Notify::new(),
    });
    let park_hook_state = park.clone();
    let park_hook_aio = AioForParkHook(aio.0);
    let park_hook_factory = FactoryForParkHook(factory);
    let runtime = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .on_thread_park(move || {
            let aio = park_hook_aio.aio();
            for _ in 0..POLLS_PER_TURN {
                if !park_hook_factory.is_running() {
                    // Let the driver see it and return.
                    park_hook_state.rearm.notify_one();
                    return;
                }
                TASK_WOKEN.with(|woken| woken.set(false));
                if aio.poll() {
                    if TASK_WOKEN.with(|woken| woken.get()) {
                        // tokio sees the woken task and runs it rather than parking.
                        return;
                    }
                    continue;
                }
                match aio.prepare_to_wait() {
                    None => continue,
                    Some(deadline_ns) => {
                        // Nothing to run until the deadline, or until the Aio's fd says so: let
                        // tokio park, with the driver waiting for both.
                        let armed = park_hook_state
                            .armed_deadline_ns
                            .swap(deadline_ns, Ordering::Relaxed);
                        if deadline_ns < armed {
                            park_hook_state.rearm.notify_one();
                        }
                        return;
                    }
                }
            }
            // A lot of simulation without waking anything: keep tokio from sleeping, give its own
            // work (sockets, timers) a look, and come back.
            park_hook_state.rearm.notify_one();
        })
        .build()
        .expect("Failed to build the tokio runtime");
    (runtime, park)
}

/// The simulation's driver task.  The park hook runs the simulation; this only waits for what
/// should wake tokio up to run the hook again: the scheduler Aio's fd, its deadline, or the hook
/// asking.  Returns once the simulation has stopped.
async fn wait_for_simulation(
    wait_fd: RawFd,
    park: &ParkState,
    factory: *mut aos_simulated_event_loop_factory_t,
) {
    // SAFETY: As in drive().
    let wait_fd =
        unsafe { AsyncFd::register_with_interest(WaitFd(wait_fd), tokio::io::Interest::READABLE) }
            .expect("Failed to register the Aio's wait fd with tokio");
    loop {
        // SAFETY: The factory is live for the simulation.
        if !unsafe { aos_simulated_event_loop_factory_is_running(factory) } {
            return;
        }
        let deadline_ns = park.armed_deadline_ns.load(Ordering::Relaxed);
        tokio::select! {
            guard = wait_fd.readable() => {
                guard.expect("Waiting on the Aio's wait fd failed").clear_ready();
            }
            _ = sleep_until_ns(deadline_ns) => {
                park.armed_deadline_ns.store(i64::MAX, Ordering::Relaxed);
            }
            _ = park.rearm.notified() => {}
        }
    }
}

/// Sleeps until `deadline_ns` on the real monotonic clock; forever for `i64::MAX`.
async fn sleep_until_ns(deadline_ns: i64) {
    if deadline_ns == i64::MAX {
        std::future::pending::<()>().await;
    }
    // SAFETY: Takes no arguments and only reads a clock.
    let now_ns = unsafe { aos_monotonic_clock_now() };
    let remaining = u64::try_from(deadline_ns.saturating_sub(now_ns)).unwrap_or(0);
    tokio::time::sleep(Duration::from_nanos(remaining)).await;
}

/// What [`run_simulation()`]'s setup makes handles with.
pub struct SimulationSetup {
    factory: *mut aos_simulated_event_loop_factory_t,
    handles: RefCell<Vec<Rc<HandleInner>>>,
}

impl SimulationSetup {
    /// A handle on one of the simulation's event loops, for tasks to use it with.
    ///
    /// # Safety
    ///
    /// `event_loop` must be one of the simulation's event loops, and outlive
    /// [`run_simulation()`].
    pub unsafe fn handle(&self, event_loop: *mut aos_event_loop_t) -> AosHandle {
        let inner = HandleInner::new(event_loop, ExitSource::Factory(self.factory));
        self.handles.borrow_mut().push(inner.clone());
        AosHandle { inner }
    }
}

/// Runs a simulation with tokio as the outer loop, in place of the factory's `Run()`.
///
/// tokio drives the simulation scheduler's `Aio`, which runs the simulated events; between
/// batches of them, tokio runs its tasks. Tasks see simulated time through
/// [`AosHandle::monotonic_now_ns()`] and AOS timers, while tokio's own timers and sockets run
/// in real time. See `documentation/adr/0008-tokio-on-the-event-loop-thread.md`.
///
/// `setup` runs first, before the simulation starts: make handles on the simulation's event
/// loops ([`SimulationSetup::handle()`]), and AOS objects with them. The future it returns
/// runs on a tokio `LocalSet`, and may `spawn_local()` more tasks.
///
/// Runs until the simulation stops, when its events run out or something exits it. Returns
/// the future's output if it finished first.
///
/// # Panics
///
/// Panics if the simulation reports an error.
pub fn run_simulation<F, Fut>(
    factory: &mut SimulatedEventLoopFactory<'_>,
    setup: F,
) -> Option<Fut::Output>
where
    F: FnOnce(&SimulationSetup) -> Fut,
    Fut: Future + 'static,
{
    let factory = factory.as_raw();
    // SAFETY: The factory is live for this call, and owns its scheduler's Aio.
    let aio = Aio(unsafe { aos_simulated_event_loop_factory_scheduler_aio(factory) });
    // First, before anything can submit raw requests; see TokioShmEventLoop::run().
    let wait_fd = aio.wait_fd();

    let simulation = SimulationSetup {
        factory,
        handles: RefCell::new(Vec::new()),
    };
    let user_future = setup(&simulation);

    let (runtime, park) = build_simulation_runtime(&aio, factory);
    // SAFETY: The factory is live.
    check_shutdown_error(
        unsafe { aos_simulated_event_loop_factory_start_external_run(factory) },
        "The simulation failed to start",
    );

    let output = Rc::new(RefCell::new(None));
    let local = tokio::task::LocalSet::new();
    {
        let output = output.clone();
        local.spawn_local(async move {
            *output.borrow_mut() = Some(user_future.await);
        });
    }
    local.block_on(&runtime, wait_for_simulation(wait_fd, &park, factory));
    // Tasks may hold AOS objects; they go before the simulation's run ends.
    drop(local);
    drop(runtime);

    // SAFETY: The factory is live, and its external run has stopped.
    check_shutdown_error(
        unsafe { aos_simulated_event_loop_factory_finish_external_run(factory) },
        "The simulation stopped",
    );
    for inner in simulation.handles.borrow().iter() {
        inner.finish();
    }
    let output = output.borrow_mut().take();
    output
}

/// State a C callback wakes a task through.  Lives as long as the event loop: AOS watchers can't
/// be removed, and a timer's callback may still be registered.
struct WakeState {
    /// Set by the callback; cleared by whoever consumes the event.
    fired: Cell<bool>,
    waker: RefCell<Option<Waker>>,
    /// In a simulation, its factory: waking a task ends the slice of simulated events, so the
    /// task runs before simulated time moves on.  Null otherwise.
    simulation: *mut aos_simulated_event_loop_factory_t,
}

thread_local! {
    /// Whether an AOS callback woke a waiting task since this was last cleared.  The simulation
    /// driver's park hook uses it to stop running simulated events and let the task run.
    static TASK_WOKEN: Cell<bool> = const { Cell::new(false) };
}

extern "C" fn wake_callback(user_data: *mut c_void) {
    // SAFETY: user_data is an Rc<WakeState> kept alive in HandleInner::wakers for the event
    // loop's lifetime, and this runs on the event loop's thread.
    let state = unsafe { &*(user_data as *const WakeState) };
    state.fired.set(true);
    if let Some(waker) = state.waker.borrow_mut().take() {
        waker.wake();
        TASK_WOKEN.with(|woken| woken.set(true));
        if !state.simulation.is_null() {
            // SAFETY: The factory outlives the simulation's handles.
            unsafe { aos_simulated_event_loop_factory_end_slice(state.simulation) };
        }
    }
}

/// An AOS object a task dropped, waiting for the event loop to stop: AOS doesn't allow
/// destroying fetchers or senders while it runs.
enum Deferred {
    Fetcher(*mut aos_fetcher_t),
    Sender(*mut aos_sender_t),
}

/// Where an [`AosHandle::exit()`] goes.
#[derive(Clone, Copy)]
enum ExitSource {
    /// The `ShmEventLoop` itself.
    Shm,
    /// The simulation the event loop is part of.
    Factory(*mut aos_simulated_event_loop_factory_t),
}

struct HandleInner {
    /// Null once run() has returned and the event loop is gone.
    event_loop: Cell<*mut aos_event_loop_t>,
    exit_source: ExitSource,
    wakers: RefCell<Vec<Rc<WakeState>>>,
    exit_handle: Cell<*mut aos_exit_handle_t>,
    /// Fetchers and senders dropped while the event loop ran; run() destroys them after it stops.
    graveyard: RefCell<Vec<Deferred>>,
    /// Fetchers and senders not yet dropped.  Zero by the time run() destroys the event loop, or
    /// one escaped it.
    live_objects: Cell<usize>,
}

impl HandleInner {
    fn new(event_loop: *mut aos_event_loop_t, exit_source: ExitSource) -> Rc<Self> {
        Rc::new(HandleInner {
            event_loop: Cell::new(event_loop),
            exit_source,
            wakers: RefCell::new(Vec::new()),
            exit_handle: Cell::new(std::ptr::null_mut()),
            graveyard: RefCell::new(Vec::new()),
            live_objects: Cell::new(0),
        })
    }

    /// Cleans up once the event loop has stopped: destroys what tasks dropped, checks nothing
    /// escaped, and retires the handle.
    fn finish(&self) {
        // Everything setup made and tasks dropped is in the graveyard by now: the future's and
        // the spawned tasks' state went with the LocalSet.
        self.empty_graveyard();
        assert_eq!(
            self.live_objects.get(),
            0,
            "An AOS fetcher or sender outlived the run: don't return one, or keep one anywhere \
             longer-lived than the future"
        );
        self.event_loop.set(std::ptr::null_mut());
        let exit_handle = self.exit_handle.replace(std::ptr::null_mut());
        if !exit_handle.is_null() {
            // SAFETY: Made by exit(), and owned here.
            unsafe { aos_exit_handle_destroy(exit_handle) };
        }
    }

    fn event_loop(&self) -> *mut aos_event_loop_t {
        let event_loop = self.event_loop.get();
        assert!(
            !event_loop.is_null(),
            "Used an AOS handle after its run returned"
        );
        event_loop
    }

    fn bury(&self, deferred: Deferred) {
        self.graveyard.borrow_mut().push(deferred);
        self.live_objects.set(self.live_objects.get() - 1);
    }

    /// Destroys what tasks dropped.  Only once the event loop has stopped.
    fn empty_graveyard(&self) {
        for deferred in self.graveyard.borrow_mut().drain(..) {
            // SAFETY: Each was made by this event loop, buried exactly once, and the event loop
            // is stopped but not yet destroyed.
            unsafe {
                match deferred {
                    Deferred::Fetcher(fetcher) => aos_fetcher_destroy(fetcher),
                    Deferred::Sender(sender) => aos_sender_destroy(sender),
                }
            }
        }
    }
}

/// Access to the event loop from tokio tasks on its thread. Cheap to clone; `!Send`.
#[derive(Clone)]
pub struct AosHandle {
    inner: Rc<HandleInner>,
}

fn c_string(s: &str) -> CString {
    CString::new(s).expect("AOS names can't contain NUL")
}

impl AosHandle {
    fn new_wake_state(&self) -> Rc<WakeState> {
        let state = Rc::new(WakeState {
            fired: Cell::new(false),
            waker: RefCell::new(None),
            simulation: match self.inner.exit_source {
                ExitSource::Shm => std::ptr::null_mut(),
                ExitSource::Factory(factory) => factory,
            },
        });
        self.inner.wakers.borrow_mut().push(state.clone());
        state
    }

    /// The event loop's monotonic time, in nanoseconds. Simulated time in simulation.
    pub fn monotonic_now_ns(&self) -> i64 {
        // SAFETY: The event loop is live while any handle exists inside run().
        unsafe { aos_event_loop_monotonic_now(self.inner.event_loop()) }
    }

    /// The node this event loop runs on, or `None` in a configuration without nodes.  What the
    /// configuration's lookups (`ConfigurationExt::get_channel`) take to find channels as the
    /// event loop does.
    pub fn node(&self) -> Option<&aos_node_t> {
        // SAFETY: The event loop is live.  The node is part of the configuration, which outlives
        // run(), and so every handle.
        unsafe { aos_event_loop_node(self.inner.event_loop()).as_ref() }
    }

    /// Makes a fetcher. Dies if the channel doesn't exist.
    pub fn make_fetcher(&self, channel_name: &str, channel_type: &str) -> RawFetcher {
        let name = c_string(channel_name);
        let ty = c_string(channel_type);
        // SAFETY: The event loop is live, and the strings outlive the call.
        let fetcher = unsafe {
            aos_event_loop_make_fetcher(self.inner.event_loop(), name.as_ptr(), ty.as_ptr())
        };
        self.inner
            .live_objects
            .set(self.inner.live_objects.get() + 1);
        RawFetcher {
            fetcher,
            handle: self.inner.clone(),
        }
    }

    /// Makes a sender. Dies if the channel doesn't exist or can't be sent on here.
    pub fn make_sender(&self, channel_name: &str, channel_type: &str) -> RawSender {
        let name = c_string(channel_name);
        let ty = c_string(channel_type);
        // SAFETY: As above.
        let sender = unsafe {
            aos_event_loop_make_sender(self.inner.event_loop(), name.as_ptr(), ty.as_ptr())
        };
        self.inner
            .live_objects
            .set(self.inner.live_objects.get() + 1);
        RawSender {
            sender,
            handle: self.inner.clone(),
        }
    }

    /// Makes a watch on a channel: an async stream of its messages. Only before the event loop
    /// starts (in `run()`'s setup).
    ///
    /// The watcher only wakes the task; [`Watch::next()`] reads with a fetcher. A message is
    /// therefore read when the task runs rather than inside the event loop's callback, and a
    /// task that falls behind by more than the channel's queue misses the oldest ones.
    pub fn make_watch(&self, channel_name: &str, channel_type: &str) -> Watch {
        let fetcher = self.make_fetcher(channel_name, channel_type);
        let state = self.new_wake_state();
        let name = c_string(channel_name);
        let ty = c_string(channel_type);
        // SAFETY: The event loop is live; the WakeState lives as long as it does (see
        // HandleInner::wakers).
        unsafe {
            aos_event_loop_make_no_arg_watcher(
                self.inner.event_loop(),
                name.as_ptr(),
                ty.as_ptr(),
                Some(wake_callback),
                Rc::as_ptr(&state) as *mut c_void,
            )
        };
        Watch { fetcher, state }
    }

    /// Makes a timer on the event loop's clock: simulated time in simulation.
    pub fn make_timer(&self) -> Timer {
        let state = self.new_wake_state();
        // SAFETY: As for make_watch().
        let timer = unsafe {
            aos_event_loop_add_timer(
                self.inner.event_loop(),
                Some(wake_callback),
                Rc::as_ptr(&state) as *mut c_void,
            )
        };
        Timer { timer, state }
    }

    /// Tells the event loop to stop, or in a simulation, the whole simulation. `run()` returns
    /// `None` once it has.
    pub fn exit(&self) {
        if self.inner.exit_handle.get().is_null() {
            let event_loop = self.inner.event_loop();
            // SAFETY: The event loop (and in a simulation, its factory) is live.
            let exit_handle = unsafe {
                match self.inner.exit_source {
                    ExitSource::Shm => aos_shm_event_loop_make_exit_handle(event_loop),
                    ExitSource::Factory(factory) => {
                        aos_simulated_event_loop_factory_make_exit_handle(factory)
                    }
                }
            };
            self.inner.exit_handle.set(exit_handle);
        }
        // SAFETY: Made just above, and destroyed only after run() is done with the event loop.
        unsafe { aos_exit_handle_exit(self.inner.exit_handle.get()) };
    }
}

/// A fetched message: its context, and its bytes if it had any.
pub struct Message<'a> {
    context: aos_context_t,
    _data: PhantomData<&'a [u8]>,
}

impl<'a> Message<'a> {
    /// The message's bytes, or `None` if there wasn't one.
    pub fn data(&self) -> Option<&'a [u8]> {
        if self.context.data.is_null() {
            None
        } else {
            // SAFETY: The fetcher holds the message, and the borrow ('a) keeps it from fetching
            // another until this is gone.
            Some(unsafe {
                std::slice::from_raw_parts(self.context.data as *const u8, self.context.size)
            })
        }
    }

    /// When the message was sent, on the event loop's monotonic clock.
    pub fn monotonic_event_time_ns(&self) -> i64 {
        self.context.monotonic_event_time
    }

    /// When the message was sent, on the realtime clock.
    pub fn realtime_event_time_ns(&self) -> i64 {
        self.context.realtime_event_time
    }
}

/// An AOS fetcher. `!Send`.
pub struct RawFetcher {
    fetcher: *mut aos_fetcher_t,
    handle: Rc<HandleInner>,
}

impl Drop for RawFetcher {
    fn drop(&mut self) {
        // The event loop may still be running; run() destroys it once it has stopped.
        self.handle.bury(Deferred::Fetcher(self.fetcher));
    }
}

impl RawFetcher {
    /// Fetches the latest message. Returns whether there was a new one.
    pub fn fetch(&mut self) -> bool {
        // SAFETY: Live, on its thread.
        unsafe { aos_fetcher_fetch(self.fetcher) }
    }

    /// Fetches the next message after the last one fetched. Returns whether there was one.
    pub fn fetch_next(&mut self) -> bool {
        // SAFETY: As above.
        unsafe { aos_fetcher_fetch_next(self.fetcher) }
    }

    /// The last message fetched.
    pub fn message(&self) -> Message<'_> {
        Message {
            // SAFETY: As above.
            context: unsafe { aos_fetcher_context(self.fetcher) },
            _data: PhantomData,
        }
    }
}

/// An AOS sender. `!Send`.
pub struct RawSender {
    sender: *mut aos_sender_t,
    handle: Rc<HandleInner>,
}

impl Drop for RawSender {
    fn drop(&mut self) {
        // As for RawFetcher.
        self.handle.bury(Deferred::Sender(self.sender));
    }
}

impl RawSender {
    /// Sends a copy of `data`. Returns one of the `aos_const_raw_sender_error_*` codes.
    pub fn send_copy(&mut self, data: &[u8]) -> i32 {
        // SAFETY: Live, on its thread, and data outlives the call.
        unsafe { aos_sender_send_copy(self.sender, data.as_ptr() as *const c_void, data.len()) }
    }
}

/// A channel's messages as an async stream. See [`AosHandle::make_watch()`].
pub struct Watch {
    fetcher: RawFetcher,
    state: Rc<WakeState>,
}

impl Watch {
    /// The next message on the channel, waiting until there is one.
    pub async fn next(&mut self) -> Message<'_> {
        std::future::poll_fn(|cx| {
            // Clear before fetching: a message sent after the fetch finds nothing then fires
            // again, rather than being lost between the two.
            self.state.fired.set(false);
            if self.fetcher.fetch_next() {
                return Poll::Ready(());
            }
            *self.state.waker.borrow_mut() = Some(cx.waker().clone());
            Poll::Pending
        })
        .await;
        self.fetcher.message()
    }

    /// The newest message on the channel once there is one newer than the last returned,
    /// skipping any in between.  For a task that only wants the latest, such as video, and
    /// that may fall more than the channel's storage behind, which kills [`Watch::next()`].
    pub async fn latest(&mut self) -> Message<'_> {
        std::future::poll_fn(|cx| {
            // As in next().
            self.state.fired.set(false);
            if self.fetcher.fetch() {
                return Poll::Ready(());
            }
            *self.state.waker.borrow_mut() = Some(cx.waker().clone());
            Poll::Pending
        })
        .await;
        self.fetcher.message()
    }
}

/// An AOS timer, on the event loop's clock. See [`AosHandle::make_timer()`].
pub struct Timer {
    timer: *mut aos_timer_handler_t,
    state: Rc<WakeState>,
}

impl Timer {
    /// Waits until `deadline_ns` on the event loop's monotonic clock.
    pub async fn sleep_until(&mut self, deadline_ns: i64) {
        self.state.fired.set(false);
        // SAFETY: The timer is live while the event loop is; one-shot (period 0).
        unsafe { aos_timer_handler_schedule(self.timer, deadline_ns, 0) };
        TimerFired(&self.state).await
    }
}

struct TimerFired<'a>(&'a WakeState);

impl Future for TimerFired<'_> {
    type Output = ();
    fn poll(self: Pin<&mut Self>, cx: &mut TaskContext<'_>) -> Poll<()> {
        if self.0.fired.get() {
            return Poll::Ready(());
        }
        *self.0.waker.borrow_mut() = Some(cx.waker().clone());
        Poll::Pending
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::Path;

    use aos_configuration::read_config_from;
    use aos_testing_path::artifact_path;
    use tokio::io::{AsyncReadExt, AsyncWriteExt};

    fn config() -> impl Flatbuffer<RustConfiguration<'static>> {
        read_config_from(&artifact_path(Path::new(
            "aos/testing/ping_pong/pingpong_config.json",
        )))
        .unwrap()
    }

    fn init() {
        aos_test_init::test_init();
    }

    /// A timer on the event loop wakes a tokio task.
    #[test]
    fn timer_wakes_task() {
        init();
        let config = config();
        let event_loop = TokioShmEventLoop::new(&config);
        let result = event_loop.run(|aos| {
            let aos = aos.clone();
            let mut timer = aos.make_timer();
            async move {
                let start = aos.monotonic_now_ns();
                timer.sleep_until(start + 20_000_000).await;
                aos.monotonic_now_ns() - start
            }
        });
        let elapsed_ns = result.expect("The event loop stopped early");
        assert!(elapsed_ns >= 20_000_000, "woke after {elapsed_ns} ns");
        assert!(elapsed_ns < 500_000_000, "woke after {elapsed_ns} ns");
    }

    /// Tokio sockets and AOS work together on one thread, and a message wakes a task watching for
    /// it. One event loop reads a TCP connection and sends what it got on an AOS channel; the
    /// other connects, writes, and watches the channel. Two event loops on two threads, since one
    /// event loop can't both send on a channel and watch it.
    #[test]
    fn socket_to_channel_to_watch() {
        init();
        let config = config();
        let (address_tx, address_rx) = std::sync::mpsc::channel();
        let (watching_tx, watching_rx) = std::sync::mpsc::channel::<()>();
        std::thread::scope(|scope| {
            let config = &config;
            scope.spawn(move || {
                let event_loop = TokioShmEventLoop::new(config);
                event_loop.run(|aos| {
                    let mut sender = aos.make_sender("/test", "aos.examples.Ping");
                    async move {
                        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
                        address_tx.send(listener.local_addr().unwrap()).unwrap();
                        let (mut socket, _) = listener.accept().await.unwrap();
                        let mut buffer = Vec::new();
                        socket.read_to_end(&mut buffer).await.unwrap();
                        // Any bytes do for a raw sender; nobody parses them here.
                        sender.send_copy(&buffer);
                    }
                });
            });
            let event_loop = TokioShmEventLoop::new(config);
            let result = event_loop.run(|aos| {
                let mut watch = aos.make_watch("/test", "aos.examples.Ping");
                async move {
                    let address = address_rx.recv().unwrap();
                    let mut client = tokio::net::TcpStream::connect(address).await.unwrap();
                    client.write_all(b"hello").await.unwrap();
                    drop(client);
                    drop(watching_tx);
                    watch.next().await.data().map(|data| data.to_vec())
                }
            });
            let _ = watching_rx.recv();
            assert_eq!(result.unwrap().unwrap(), b"hello");
        });
    }

    /// In a simulation, tasks run between simulated events and see simulated time: a timer task
    /// sends a ping one simulated second in, another task watching for it gets it at exactly that
    /// simulated time, and a tokio TCP round trip works in the middle of the simulation.
    #[test]
    fn simulation_runs_tasks_on_simulated_time() {
        use aos_events_event_loop_c::aos_event_loop_destroy as destroy_event_loop;
        use aos_events_simulated_event_loop::SimulatedEventLoopFactory;
        use ping_rust_fbs::aos::examples as ping;

        const VALUE: i32 = 3132;
        init();
        let config = read_config_from(&artifact_path(Path::new(
            "aos/testing/ping_pong/multinode_pingpong_test_combined_config.json",
        )))
        .unwrap();
        let mut factory = SimulatedEventLoopFactory::new(&config);
        let ping_loop = factory.make_event_loop("ping", "pi1");
        let listen_loop = factory.make_event_loop("listen", "pi1");

        let result = run_simulation(&mut factory, |simulation| {
            // SAFETY: Both are this simulation's, and destroyed only after the run.
            let (ping_aos, listen_aos) =
                unsafe { (simulation.handle(ping_loop), simulation.handle(listen_loop)) };
            let mut timer = ping_aos.make_timer();
            let mut sender = ping_aos.make_sender("/test", "aos.examples.Ping");
            let mut watch = listen_aos.make_watch("/test", "aos.examples.Ping");
            async move {
                let start_ns = ping_aos.monotonic_now_ns();
                let send = async {
                    timer.sleep_until(start_ns + 1_000_000_000).await;
                    let mut fbb = flatbuffers::FlatBufferBuilder::new();
                    let mut builder = ping::PingBuilder::new(&mut fbb);
                    builder.add_value(VALUE);
                    let message = builder.finish();
                    fbb.finish_minimal(message);
                    sender.send_copy(fbb.finished_data());
                };
                let receive = async {
                    let message = watch.next().await;
                    let value = flatbuffers::root::<ping::Ping>(message.data().unwrap())
                        .unwrap()
                        .value();
                    (value, listen_aos.monotonic_now_ns() - start_ns)
                };
                let ((), received) = tokio::join!(send, receive);

                let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
                let address = listener.local_addr().unwrap();
                let echo = async {
                    let (mut socket, _) = listener.accept().await.unwrap();
                    let mut buffer = [0u8; 4];
                    socket.read_exact(&mut buffer).await.unwrap();
                    socket.write_all(&buffer).await.unwrap();
                };
                let call = async {
                    let mut client = tokio::net::TcpStream::connect(address).await.unwrap();
                    client.write_all(b"echo").await.unwrap();
                    let mut buffer = [0u8; 4];
                    client.read_exact(&mut buffer).await.unwrap();
                    buffer
                };
                let ((), echoed) = tokio::join!(echo, call);

                listen_aos.exit();
                (received, echoed)
            }
        });
        // SAFETY: Made by the factory above, and the run is over.
        unsafe {
            destroy_event_loop(ping_loop);
            destroy_event_loop(listen_loop);
        }
        let ((value, elapsed_ns), echoed) = result.expect("The simulation ended first");
        assert_eq!(value, VALUE);
        assert_eq!(
            elapsed_ns, 1_000_000_000,
            "the ping arrived at simulated +{elapsed_ns} ns"
        );
        assert_eq!(&echoed, b"echo");
    }

    /// Watch::latest() skips to the newest message: three are sent before the watching task
    /// looks, and it gets only the last.  Two event loops on two threads, as above.
    #[test]
    fn latest_skips_to_the_newest() {
        init();
        let config = config();
        let (watching_tx, watching_rx) = std::sync::mpsc::channel::<()>();
        let (sent_tx, sent_rx) = std::sync::mpsc::channel::<()>();
        std::thread::scope(|scope| {
            let config = &config;
            scope.spawn(move || {
                let event_loop = TokioShmEventLoop::new(config);
                event_loop.run(|aos| {
                    let aos = aos.clone();
                    let mut sender = aos.make_sender("/test", "aos.examples.Ping");
                    async move {
                        watching_rx.recv().unwrap();
                        for message in [b"one".as_slice(), b"two", b"three"] {
                            sender.send_copy(message);
                        }
                        sent_tx.send(()).unwrap();
                        aos.exit();
                    }
                });
            });
            let event_loop = TokioShmEventLoop::new(config);
            let result = event_loop.run(|aos| {
                let mut watch = aos.make_watch("/test", "aos.examples.Ping");
                async move {
                    watching_tx.send(()).unwrap();
                    sent_rx.recv().unwrap();
                    watch.latest().await.data().map(|data| data.to_vec())
                }
            });
            assert_eq!(result.unwrap().unwrap(), b"three");
        });
    }

    /// exit() stops the event loop, and run() returns None.
    #[test]
    fn exit_stops_run() {
        init();
        let config = config();
        let event_loop = TokioShmEventLoop::new(&config);
        let result = event_loop.run(|aos| {
            let aos = aos.clone();
            async move {
                aos.exit();
                std::future::pending::<()>().await
            }
        });
        assert!(result.is_none());
    }
}
