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

// The backends this platform has.  --aio_backend is fatal on any name the
// platform has no backend for (see UnknownBackendDies).
//
// On Linux, AOS_EPOLL_ONLY / AOS_IO_URING_ONLY narrow this to one backend, so
// aos/events/BUILD can give each its own target the way it does for
// shm_event_loop_test.cc.  A build that defines neither runs both.
std::vector<AioBackend> Backends() {
#if defined(__linux__)
  std::vector<AioBackend> backends;
#ifndef AOS_EPOLL_ONLY
  backends.push_back(AioBackend{.name = "io_uring",
                                .make = &MakeHosting,
                                .has_queue_depth = true,
                                .is_io_uring = true});
#endif  // AOS_EPOLL_ONLY
#ifndef AOS_IO_URING_ONLY
  backends.push_back(AioBackend{
      .name = "epoll", .make = &MakeHosting, .has_queue_depth = true});
#endif  // AOS_IO_URING_ONLY
  return backends;
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

// Each backend under each kind of ThreadSignalReceiver.  The futex kind is
// Linux only, so elsewhere this is just the backends.
std::vector<AioReceiverParam> ReceiverParams(
    const std::vector<AioBackend> &backends) {
  std::vector<AioReceiverParam> result;
  for (const AioBackend &backend : backends) {
    result.push_back(AioReceiverParam{.backend = backend,
                                      .kind = ipc_lib::WakeupKind::kSignal});
#if defined(__linux__)
    result.push_back(AioReceiverParam{.backend = backend,
                                      .kind = ipc_lib::WakeupKind::kFutex});
#endif
  }
  return result;
}

}  // namespace

INSTANTIATE_TEST_SUITE_P(AioBackends, AioTest, ::testing::ValuesIn(Backends()),
                         &BackendTestName);

INSTANTIATE_TEST_SUITE_P(IoUring, AioIoUringTest,
                         ::testing::ValuesIn(IoUringBackend()),
                         &BackendTestName);

INSTANTIATE_TEST_SUITE_P(AioBackends, AioReceiverTest,
                         ::testing::ValuesIn(ReceiverParams(Backends())),
                         &ReceiverTestName);

INSTANTIATE_TEST_SUITE_P(IoUring, AioIoUringReceiverTest,
                         ::testing::ValuesIn(ReceiverParams(IoUringBackend())),
                         &ReceiverTestName);

// Every platform but Linux has no io_uring, so AioIoUringTest is instantiated
// with nothing there.  That is the point of the suite -- these are io_uring's
// contract, not one the others fail to meet -- so tell gtest rather than have
// it warn about a suite nobody instantiated.
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(AioIoUringTest);
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(AioIoUringReceiverTest);
}  // namespace aos::testing
