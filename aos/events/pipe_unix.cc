#include "aos/events/pipe.h"

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#include "absl/log/absl_check.h"

namespace aos {

Pipe::Pipe() {
  ABSL_PCHECK(pipe(fds_) == 0);
  ABSL_PCHECK(fcntl(fds_[0], F_SETFL, O_NONBLOCK) == 0);
  ABSL_PCHECK(fcntl(fds_[1], F_SETFL, O_NONBLOCK) == 0);
}

Pipe::~Pipe() {
  if (fds_[0] >= 0) {
    ABSL_PCHECK(close(fds_[0]) == 0);
  }
  if (fds_[1] >= 0) {
    ABSL_PCHECK(close(fds_[1]) == 0);
  }
}

void Pipe::close_read_fd() {
  ABSL_PCHECK(close(fds_[0]) == 0);
  fds_[0] = -1;
}

void Pipe::close_write_fd() {
  ABSL_PCHECK(close(fds_[1]) == 0);
  fds_[1] = -1;
}

void Pipe::Write(const std::string &data) {
  ABSL_CHECK_EQ(write(write_fd(), data.data(), data.size()),
                static_cast<ssize_t>(data.size()));
}

std::string Pipe::Read(size_t size) {
  std::string result;
  result.resize(size);
  ABSL_CHECK_EQ(read(read_fd(), result.data(), size),
                static_cast<ssize_t>(size));
  return result;
}

bool Pipe::write_ready() {
  pollfd poll_fd;
  poll_fd.fd = write_fd();
  poll_fd.events = POLLWRNORM;
  poll_fd.revents = 0;
  const int ret = poll(&poll_fd, 1, 0);
  return ret > 0 && (poll_fd.revents & POLLWRNORM) != 0;
}

}  // namespace aos
