#ifndef AOS_EVENTS_AIO_UNIX_H_
#define AOS_EVENTS_AIO_UNIX_H_

#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstddef>
#include <cstring>
#include <string>

namespace aos::internal {

// Pieces every POSIX Aio backend needs, kept in one place so they cannot
// drift apart between aio_linux.cc and aio_darwin.cc.

// Whether fd is a socket, and if so what error it has pending.
inline bool IsSocket(int fd) {
  struct stat st;
  if (fstat(fd, &st) == -1) {
    return false;
  }
  return static_cast<bool>(S_ISSOCK(st.st_mode));
}

// The socket detail EPoll has always added to an unhandled error event, so
// that CHECK reads the same on every backend.  Empty for anything that is not
// a socket, or a socket with no pending error.
inline std::string GetSocketErrorStr(int fd) {
  std::string error_str;
  if (IsSocket(fd)) {
    int error = 0;
    socklen_t errlen = sizeof(error);
    if (getsockopt(fd, SOL_SOCKET, SO_ERROR, (void *)&error, &errlen) == 0) {
      if (error) {
        error_str = "Socket error: " + std::string(strerror(error));
      }
    }
  }
  return error_str;
}

// How a backend notices it is running in a forked child.
//
// A kernel event object does not survive fork(): an io_uring's queues are
// mapped from the parent, a kqueue is not inherited at all, and an epoll
// instance is inherited but shared, so parent and child would steal each
// other's events.  Every backend therefore has to rebuild before a forked
// child uses it.
//
// The detection is a counter bumped by a pthread_atfork handler, which each
// loop compares against its own copy at the top of every public entry point
// (CheckForFork()).  Deliberately lazy, rather than rebuilding every live
// loop from inside the handler:
//
//   * A child that never touches the loop -- fork()+exec(), which is most of
//     what forking is for around here -- pays nothing, and is unaffected by a
//     rebuild it does not need.
//   * It is the only way the "a child may not inherit raw I/O" rule can hold
//     without making fork()+exec() fatal; see
//     Aio::Impl::CheckNoRawRequestsInFlightOnFork().
//   * The rebuild runs on the thread that goes on to drive the loop, rather
//     than inside an atfork handler, where async-signal-safety rules make
//     almost everything it needs to do questionable.
//
// A relaxed atomic load never syscalls, which is what makes the
// per-entry-point check cheap enough to be unconditional.  size_t because
// this only ever answers "did a fork happen since I last looked", so any
// width works -- and size_t stays single-instruction lock-free everywhere (a
// fixed 64-bit type would drag 32-bit ARM through libatomic locks; unsigned
// int wouldn't grow on 64-bit targets).

// How many times this process has been forked *into* -- bumped in the child.
size_t ForkCount();
// How many times this process has *called* fork(), which io_uring needs on
// top of the child counter: on DEFER_TASKRUN rings a multishot op
// outstanding in the parent becomes uncancelable after any fork() until a
// resync.
size_t ParentForkCount();

// Registers the handlers that bump both counters.  Idempotent, and every
// backend calls it from its constructor: an epoll-only build that left this
// to the io_uring backend never noticed a fork at all, and the child hung.
// A backend needing more than the counters registers its own handler on top;
// pthread_atfork stacks them.
void RegisterForkCounters();

}  // namespace aos::internal

#endif  // AOS_EVENTS_AIO_UNIX_H_
