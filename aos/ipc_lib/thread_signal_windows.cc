#ifndef NOMINMAX
#define NOMINMAX
#endif
#include "aos/ipc_lib/thread_signal.h"

#include <windows.h>

#include <cstdio>

#include "absl/log/absl_check.h"

namespace aos::ipc_lib {

ThreadSignalSender::ThreadSignalSender() {
  const pid_t pid = GetCurrentProcessId();
  const pid_t tid = GetCurrentThreadId();
  char name[64];
  int len = snprintf(name, sizeof(name), "Local\\aos-wakeup-%d-%d",
                     static_cast<int>(pid), static_cast<int>(tid));
  ABSL_CHECK(len > 0 && len < static_cast<int>(sizeof(name)));
  event_handle_ = CreateEventA(NULL, FALSE, FALSE, name);
  ABSL_PCHECK(event_handle_ != NULL) << "CreateEventA failed";
}

ThreadSignalSender::~ThreadSignalSender() {
  if (event_handle_ != NULL) {
    CloseHandle(event_handle_);
  }
}

ThreadSignalSender::ThreadSignalSender(ThreadSignalSender &&other) {
  event_handle_ = other.event_handle_;
  other.event_handle_ = NULL;
}

ThreadSignalSender &ThreadSignalSender::operator=(ThreadSignalSender &&other) {
  std::swap(event_handle_, other.event_handle_);
  return *this;
}

void ThreadSignalSender::Signal(pid_t pid, pid_t tid) {
  char name[64];
  int len = snprintf(name, sizeof(name), "Local\\aos-wakeup-%d-%d",
                     static_cast<int>(pid), static_cast<int>(tid));
  ABSL_CHECK(len > 0 && len < static_cast<int>(sizeof(name)));
  HANDLE hEvent = OpenEventA(EVENT_MODIFY_STATE, FALSE, name);
  if (hEvent == NULL) {
    const DWORD error = GetLastError();
    // The target thread may have already exited and closed its event before we
    // got here.  That's a benign race we can't avoid (the analog of ESRCH on
    // Linux, which we likewise ignore).  Anything else -- notably a permissions
    // error -- is a real problem and should be loud rather than silently
    // dropping the wakeup.
    ABSL_PCHECK(error == ERROR_FILE_NOT_FOUND)
        << "OpenEventA(" << name << ") failed with " << error;
    return;
  }
  ABSL_PCHECK(SetEvent(hEvent)) << "SetEvent(" << name << ") failed";
  ABSL_PCHECK(CloseHandle(hEvent)) << "CloseHandle failed";
}

namespace {

// Both halves derive the Event name from the target (pid, tid) pair, so a
// sender in another process can find the receiver's Event without sharing a
// handle.
void WakeupEventName(pid_t pid, pid_t tid, char *name, size_t size) {
  int len = snprintf(name, size, "Local\\aos-wakeup-%d-%d",
                     static_cast<int>(pid), static_cast<int>(tid));
  ABSL_CHECK(len > 0 && len < static_cast<int>(size));
}

}  // namespace

ThreadSignalReceiver::ThreadSignalReceiver() {
  char name[64];
  WakeupEventName(GetCurrentProcessId(), GetCurrentThreadId(), name,
                  sizeof(name));
  // Auto-reset and initially unsignaled: each wait consumes exactly one wakeup,
  // which matches the "drain one signal per notification" semantics the
  // signalfd backends provide.  If a ThreadSignalSender for this same thread
  // already created the Event, this returns another handle to that same object.
  event_handle_ = CreateEventA(NULL, FALSE, FALSE, name);
  ABSL_PCHECK(event_handle_ != NULL) << "CreateEventA(" << name << ") failed";
}

ThreadSignalReceiver::~ThreadSignalReceiver() {
  if (event_handle_ != NULL) {
    CloseHandle(event_handle_);
  }
}

void ThreadSignalReceiver::ConsumeWakeup() {
  // The Event is auto-reset, so the wait which delivered the wakeup already
  // cleared it.  Reset again anyway to drop a wakeup that arrived after that
  // wait but before we got here, so we don't immediately wake a second time.
  ABSL_PCHECK(ResetEvent(event_handle_)) << "ResetEvent failed";
}

void ThreadSignalReceiver::LeaveSignalBlocked() {
  // Nothing to do: wakeups arrive as an Event rather than a signal, so there is
  // no process-wide disposition that could kill us after destruction.
}

}  // namespace aos::ipc_lib
