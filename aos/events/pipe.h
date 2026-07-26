#ifndef AOS_EVENTS_PIPE_H_
#define AOS_EVENTS_PIPE_H_

#include <fcntl.h>
#include <unistd.h>

#include <string>
#include <string_view>

#include "absl/log/absl_check.h"

namespace aos {

// A simple wrapper around both ends of a pipe along with some helpers to easily
// read/write data through it.
class Pipe {
 public:
  Pipe() {
    ABSL_PCHECK(pipe(fds_) == 0);
    ABSL_PCHECK(fcntl(fds_[0], F_SETFL, O_NONBLOCK) == 0);
    ABSL_PCHECK(fcntl(fds_[1], F_SETFL, O_NONBLOCK) == 0);
    // Close-on-exec: an event loop's wakeup pipe must not leak into the
    // children a process spawns (starterd fork()+exec()s constantly).  Set
    // after the fact rather than with pipe2(), which macOS does not have.
    ABSL_PCHECK(fcntl(fds_[0], F_SETFD, FD_CLOEXEC) == 0);
    ABSL_PCHECK(fcntl(fds_[1], F_SETFD, FD_CLOEXEC) == 0);
  }
  ~Pipe() {
    if (fds_[0] >= 0) {
      ABSL_PCHECK(close(fds_[0]) == 0);
    }
    if (fds_[1] >= 0) {
      ABSL_PCHECK(close(fds_[1]) == 0);
    }
  }

  int read_fd() const { return fds_[0]; }
  int write_fd() const { return fds_[1]; }
  void close_read_fd() {
    ABSL_PCHECK(close(fds_[0]) == 0);
    fds_[0] = -1;
  }
  void close_write_fd() {
    ABSL_PCHECK(close(fds_[1]) == 0);
    fds_[1] = -1;
  }

  void Write(std::string_view data) {
    ABSL_CHECK_EQ(write(write_fd(), data.data(), data.size()),
                  static_cast<ssize_t>(data.size()));
  }

  std::string Read(size_t size) {
    std::string result;
    result.resize(size);
    ABSL_CHECK_EQ(read(read_fd(), result.data(), size),
                  static_cast<ssize_t>(size));
    return result;
  }

 private:
  int fds_[2];
};

}  // namespace aos

#endif  // AOS_EVENTS_PIPE_H_
