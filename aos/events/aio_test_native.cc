// The Aio contract suite, run against the backends that own their own loop.
//
// The bodies are in aio_test_lib.cc, compiled once into
// //aos/events:aio_test_lib.  This file only says which backends to run them
// against -- see aio_test_lib.h for why the two are separate, and
// aio_test_uv.cc for the other instantiation.

#include <vector>

#include "gtest/gtest.h"

#include "aos/events/aio_test_lib.h"

namespace aos::testing {
namespace {

std::unique_ptr<TestAioDriver> MakeHosting() {
  return std::make_unique<HostingAioDriver>();
}

// The backends this platform actually has.  --aio_backend is accepted and
// ignored where there is only one, so naming the Linux backends everywhere
// would not select anything -- it would just run the whole suite twice
// against the same backend, and leave the parameter disagreeing with what is
// under test (which FailedIoErrorTest reads).
std::vector<AioBackend> Backends() {
#if defined(__linux__)
  return {
      AioBackend{.name = "io_uring",
                 .make = &MakeHosting,
                 .has_queue_depth = true,
                 .is_io_uring = true},
      AioBackend{
          .name = "epoll", .make = &MakeHosting, .has_queue_depth = true},
  };
#elif defined(_WIN32)
  // IOCP has no submission queue, so --aio_queue_depth is not defined there.
  return {AioBackend{.name = "iocp", .make = &MakeHosting}};
#else
  return {AioBackend{
      .name = "kqueue", .make = &MakeHosting, .has_queue_depth = true}};
#endif
}

// io_uring's own behavior.  Empty everywhere else, which instantiates the
// suite with no parameters -- gtest is told that is deliberate below.
std::vector<AioBackend> IoUringBackend() {
  std::vector<AioBackend> result;
  for (const AioBackend &backend : Backends()) {
    if (backend.is_io_uring) result.push_back(backend);
  }
  return result;
}

}  // namespace

INSTANTIATE_TEST_SUITE_P(AioBackends, AioTest, ::testing::ValuesIn(Backends()),
                         &BackendTestName);

INSTANTIATE_TEST_SUITE_P(IoUring, AioIoUringTest,
                         ::testing::ValuesIn(IoUringBackend()),
                         &BackendTestName);

// Every platform but Linux has no io_uring, so AioIoUringTest is instantiated
// with nothing there.  That is the point of the suite -- these are io_uring's
// contract, not one the others fail to meet -- so tell gtest rather than have
// it warn about a suite nobody instantiated.
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(AioIoUringTest);
}  // namespace aos::testing
