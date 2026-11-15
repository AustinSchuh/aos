//! A Rust application for tokio_simulation_test: a task that records when a ping arrives.

use aos_events_event_loop_c::aos_event_loop_t;
use aos_events_tokio::TokioSimulation;
use ping_rust_fbs::aos::examples as ping;

/// What the task saw: the last ping's value, and the simulated time it arrived, or -1.
#[repr(C)]
pub struct PingSeen {
    pub value: i32,
    pub monotonic_now_ns: i64,
}

/// Adds a task that watches `/test` for pings on `event_loop`, writing each to `seen`.
///
/// # Safety
///
/// `simulation` is a live, unrun `aos_tokio_simulation_t`, `event_loop` one of its event loops,
/// and `seen` outlives the run.
#[no_mangle]
pub unsafe extern "C" fn tokio_simulation_test_watch_pings(
    simulation: *mut TokioSimulation,
    event_loop: *mut aos_event_loop_t,
    seen: *mut PingSeen,
) {
    // SAFETY: As the caller promises.
    let simulation = unsafe { &mut *simulation };
    // SAFETY: As the caller promises.
    let aos = unsafe { simulation.setup().handle(event_loop) };
    let mut pings = aos.make_watch("/test", "aos.examples.Ping");
    simulation.spawn(async move {
        loop {
            let message = pings.next().await;
            let Some(data) = message.data() else {
                continue;
            };
            let value = ping::root_as_ping(data).expect("Not a Ping").value();
            // SAFETY: As the caller promises.
            unsafe {
                *seen = PingSeen {
                    value,
                    monotonic_now_ns: aos.monotonic_now_ns(),
                };
            }
        }
    });
}
