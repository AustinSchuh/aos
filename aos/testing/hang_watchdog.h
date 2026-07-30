#ifndef AOS_TESTING_HANG_WATCHDOG_H_
#define AOS_TESTING_HANG_WATCHDOG_H_

namespace aos::testing {

// Starts a background watchdog that dumps every thread's stack and aborts
// shortly before bazel's test timeout would kill the process.
//
// A test that hangs under remote execution is otherwise killed from outside
// with no grace: the worker tears the container down, the log truncates
// mid-test, and the only artifact is "which test was running."  Arming a
// watchdog *inside* the process converts that into a loud failure whose
// test.log carries a backtrace of every thread, delivered through the
// normal remote-execution log pipeline with no worker or infrastructure
// support needed.
//
// The deadline comes from the TEST_TIMEOUT environment variable, which
// bazel exports to every test (in seconds); the watchdog fires 15 seconds
// before it (or at 3/4 of it for short timeouts) so the dump completes and
// flushes while the process is still allowed to run.  No-op when
// TEST_TIMEOUT is unset (not running under bazel) or on platforms without
// tgkill()/procfs (the dump mechanism is Linux-specific).
//
// The stack dumping is done from a signal handler using
// absl::GetStackTrace()/absl::Symbolize(), the same
// as-safe-as-practical-in-a-crash-path approach absl's own failure signal
// handler takes.
void MaybeStartHangWatchdog();

}  // namespace aos::testing

#endif  // AOS_TESTING_HANG_WATCHDOG_H_
