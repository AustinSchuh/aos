#include "aos/events/shm_event_loop.h"

#include "aos/ipc_lib/thread_signal.h"

namespace aos {

// Wakeups arrive as a named Event on Windows rather than as a signal (see
// ThreadSignalSender/ThreadSignalReceiver), so there is no signal disposition
// to suppress here.
void ShmEventLoop::IgnoreWakeupSignal() {}

// TODO(austin): Windows has no sigaction.  The equivalent would be a
// SetConsoleCtrlHandler which Exit()s each registered event loop.  Until that
// exists, Ctrl-C terminates the process instead of unwinding the loop cleanly.
void ShmEventLoop::RegisterSignalHandler() {}

void ShmEventLoop::UnregisterSignalHandler() {}

// There's no realtime scheduler to initialize: InitRT() is Linux-only, and
// realtime_windows.cc emulates the thread priority/affinity model instead.
void ShmEventLoop::InitializeRealtime() {}

}  // namespace aos
