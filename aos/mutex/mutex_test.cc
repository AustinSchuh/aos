#include "aos/mutex/mutex.h"

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#endif

#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>
#include <vector>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "gtest/gtest.h"

#include "aos/ipc_lib/aos_sync.h"
#include "aos/logging/implementations.h"
#include "aos/testing/test_logging.h"
#include "aos/testing/test_shm.h"
#include "aos/util/death_test_log_implementation.h"

namespace aos::testing {

namespace chrono = ::std::chrono;
namespace this_thread = ::std::this_thread;

class MutexTest : public ::testing::Test {
 public:
  Mutex test_mutex_;

 protected:
  void SetUp() override { aos::testing::EnableTestLogging(); }
};

typedef MutexTest MutexDeathTest;
typedef MutexTest MutexLockerTest;
typedef MutexTest MutexLockerDeathTest;
typedef MutexTest IPCMutexLockerTest;
typedef MutexTest IPCMutexLockerDeathTest;
typedef MutexTest IPCRecursiveMutexLockerTest;

TEST_F(MutexTest, TryLock) {
  EXPECT_EQ(Mutex::State::kLocked, test_mutex_.TryLock());
  EXPECT_EQ(Mutex::State::kLockFailed, test_mutex_.TryLock());

  test_mutex_.Unlock();
}

TEST_F(MutexTest, Lock) {
  ASSERT_FALSE(test_mutex_.Lock());
  EXPECT_EQ(Mutex::State::kLockFailed, test_mutex_.TryLock());

  test_mutex_.Unlock();
}

TEST_F(MutexTest, Unlock) {
  ASSERT_FALSE(test_mutex_.Lock());
  EXPECT_EQ(Mutex::State::kLockFailed, test_mutex_.TryLock());
  test_mutex_.Unlock();
  EXPECT_EQ(Mutex::State::kLocked, test_mutex_.TryLock());

  test_mutex_.Unlock();
}

// Sees what happens with multiple unlocks.
TEST_F(MutexDeathTest, RepeatUnlock) {
  ASSERT_FALSE(test_mutex_.Lock());
  test_mutex_.Unlock();
  EXPECT_DEATH(
      {
        logging::SetImplementation(
            std::make_shared<util::DeathTestLogImplementation>());
        test_mutex_.Unlock();
      },
      ".*multiple unlock.*");
}

// Sees what happens if you unlock without ever locking (or unlocking) it.
TEST_F(MutexDeathTest, NeverLock) {
  EXPECT_DEATH(
      {
        logging::SetImplementation(
            std::make_shared<util::DeathTestLogImplementation>());
        test_mutex_.Unlock();
      },
      ".*multiple unlock.*");
}

// Tests that locking a mutex multiple times from the same thread fails nicely.
TEST_F(MutexDeathTest, RepeatLock) {
  EXPECT_DEATH(
      {
        logging::SetImplementation(
            std::make_shared<util::DeathTestLogImplementation>());
        ASSERT_FALSE(test_mutex_.Lock());
        ASSERT_FALSE(test_mutex_.Lock());
      },
      ".*multiple lock.*");
}

bool IsAligned(const void *ptr, size_t alignment) {
  // Standard check: is the address a multiple of alignment?
  // Using (alignment - 1) as a mask only works for powers of 2.
  return (reinterpret_cast<uintptr_t>(ptr) & (alignment - 1)) == 0;
}

// Tests that Lock behaves correctly when the previous owner exits with the lock
// held (which is the same as dying any other way).
TEST_F(MutexTest, OwnerDiedDeathLock) {
  SharedMemoryBlock memory(sizeof(Mutex));
  ASSERT_TRUE(IsAligned(memory.get(), alignof(Mutex)));
  Mutex *mutex = static_cast<Mutex *>(memory.get());
  new (mutex) Mutex();

  std::thread thread([&]() { ASSERT_FALSE(mutex->Lock()); });
  thread.join();
  EXPECT_TRUE(mutex->Lock());

  mutex->Unlock();
  mutex->~Mutex();
}

// Tests that TryLock behaves correctly when the previous owner dies.
TEST_F(MutexTest, OwnerDiedDeathTryLock) {
  SharedMemoryBlock memory(sizeof(Mutex));
  ASSERT_TRUE(IsAligned(memory.get(), alignof(Mutex)));
  Mutex *mutex = static_cast<Mutex *>(memory.get());
  new (mutex) Mutex();

  std::thread thread([&]() { ASSERT_FALSE(mutex->Lock()); });
  thread.join();
  EXPECT_EQ(Mutex::State::kOwnerDied, mutex->TryLock());

  mutex->Unlock();
  mutex->~Mutex();
}

// TODO(brians): Test owner dying by being SIGKILLed and SIGTERMed.

// This sequence of mutex operations used to mess up the robust list and cause
// one of the mutexes to not get owner-died like it should.
TEST_F(MutexTest, DontCorruptRobustList) {
  // I think this was the allocator lock in the original failure.
  Mutex mutex1;
  // This one should get owner-died afterwards (iff the kernel accepts the
  // robust list and uses it). I think it was the task_death_notification lock
  // in the original failure.
  Mutex mutex2;

  std::thread thread([&]() {
    ASSERT_FALSE(mutex1.Lock());
    ASSERT_FALSE(mutex2.Lock());
    mutex1.Unlock();
  });
  thread.join();

  EXPECT_EQ(Mutex::State::kLocked, mutex1.TryLock());
  EXPECT_EQ(Mutex::State::kOwnerDied, mutex2.TryLock());

  mutex1.Unlock();
  mutex2.Unlock();
}

// Verifies that ThreadSanitizer understands that a contended mutex establishes
// a happens-before relationship.
TEST_F(MutexTest, ThreadSanitizerContended) {
  int counter = 0;
  std::thread thread1([this, &counter]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    MutexLocker locker(&test_mutex_);
    ++counter;
  });
  std::thread thread2([this, &counter]() {
    MutexLocker locker(&test_mutex_);
    ++counter;
  });
  thread1.join();
  thread2.join();
  EXPECT_EQ(2, counter);
}

// Verifiers that ThreadSanitizer understands how a mutex works.
// For some reason this used to fail when the other tests didn't...
// The loops make it fail more reliably when it's going to.
TEST_F(MutexTest, ThreadSanitizerMutexLocker) {
  for (int i = 0; i < 100; ++i) {
    int counter = 0;
    ::std::thread thread([&counter, this]() {
      for (int i = 0; i < 300; ++i) {
        MutexLocker locker(&test_mutex_);
        ++counter;
      }
    });
    for (int i = 0; i < 300; ++i) {
      MutexLocker locker(&test_mutex_);
      --counter;
    }
    thread.join();
    EXPECT_EQ(0, counter);
  }
}

// Verifies that ThreadSanitizer understands that an uncontended mutex
// establishes a happens-before relationship.
TEST_F(MutexTest, ThreadSanitizerUncontended) {
  int counter = 0;
  std::thread thread1([this, &counter]() {
    MutexLocker locker(&test_mutex_);
    ++counter;
  });
  std::thread thread2([this, &counter]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    MutexLocker locker(&test_mutex_);
    ++counter;
  });
  thread1.join();
  thread2.join();
  EXPECT_EQ(2, counter);
}

// Makes sure that we don't SIGSEGV or something with multiple threads.
TEST_F(MutexTest, MultiThreadedLock) {
  std::thread thread([this] {
    ASSERT_FALSE(test_mutex_.Lock());
    test_mutex_.Unlock();
  });
  ASSERT_FALSE(test_mutex_.Lock());
  test_mutex_.Unlock();
  thread.join();
}

TEST_F(MutexLockerTest, Basic) {
  {
    aos::MutexLocker locker(&test_mutex_);
    EXPECT_EQ(Mutex::State::kLockFailed, test_mutex_.TryLock());
  }
  EXPECT_EQ(Mutex::State::kLocked, test_mutex_.TryLock());

  test_mutex_.Unlock();
}

// Tests that MutexLocker behaves correctly when the previous owner dies.
TEST_F(MutexLockerDeathTest, OwnerDied) {
  SharedMemoryBlock memory(sizeof(Mutex));
  ASSERT_TRUE(IsAligned(memory.get(), alignof(Mutex)));
  Mutex *mutex = static_cast<Mutex *>(memory.get());
  new (mutex) Mutex();

  EXPECT_DEATH(
      {
        std::thread thread([&]() { ASSERT_FALSE(mutex->Lock()); });
        thread.join();

        logging::SetImplementation(
            std::make_shared<util::DeathTestLogImplementation>());
        MutexLocker locker(mutex);
      },
      ".*previous owner of mutex.*died.*");

  mutex->~Mutex();
}

TEST_F(IPCMutexLockerTest, Basic) {
  {
    aos::IPCMutexLocker locker(&test_mutex_);
    EXPECT_EQ(Mutex::State::kLockFailed, test_mutex_.TryLock());
    EXPECT_FALSE(locker.owner_died());
  }
  EXPECT_EQ(Mutex::State::kLocked, test_mutex_.TryLock());

  test_mutex_.Unlock();
}

// Tests what happens when the caller doesn't check if the previous owner died
// with an IPCMutexLocker.
TEST_F(IPCMutexLockerDeathTest, NoCheckOwnerDied) {
  EXPECT_DEATH(
      { aos::IPCMutexLocker locker(&test_mutex_); },
      ".*nobody checked if the previous owner of mutex.*died.*");
}

TEST_F(IPCRecursiveMutexLockerTest, Basic) {
  {
    aos::IPCRecursiveMutexLocker locker(&test_mutex_);
    EXPECT_EQ(Mutex::State::kLockFailed, test_mutex_.TryLock());
    EXPECT_FALSE(locker.owner_died());
  }
  EXPECT_EQ(Mutex::State::kLocked, test_mutex_.TryLock());

  test_mutex_.Unlock();
}

// Tests actually locking a mutex recursively with IPCRecursiveMutexLocker.
TEST_F(IPCRecursiveMutexLockerTest, RecursiveLock) {
  {
    aos::IPCRecursiveMutexLocker locker(&test_mutex_);
    EXPECT_EQ(Mutex::State::kLockFailed, test_mutex_.TryLock());
    {
      aos::IPCRecursiveMutexLocker locker(&test_mutex_);
      EXPECT_EQ(Mutex::State::kLockFailed, test_mutex_.TryLock());
      EXPECT_FALSE(locker.owner_died());
    }
    EXPECT_EQ(Mutex::State::kLockFailed, test_mutex_.TryLock());
    EXPECT_FALSE(locker.owner_died());
  }
  EXPECT_EQ(Mutex::State::kLocked, test_mutex_.TryLock());

  test_mutex_.Unlock();
}

// Tests that IPCMutexLocker behaves correctly when the previous owner dies.
TEST_F(IPCMutexLockerTest, OwnerDied) {
  SharedMemoryBlock memory(sizeof(Mutex));
  ASSERT_TRUE(IsAligned(memory.get(), alignof(Mutex)));
  Mutex *mutex = static_cast<Mutex *>(memory.get());
  new (mutex) Mutex();

  std::thread thread([&]() { ASSERT_FALSE(mutex->Lock()); });
  thread.join();
  {
    aos::IPCMutexLocker locker(mutex);
    EXPECT_EQ(Mutex::State::kLockFailed, mutex->TryLock());
    EXPECT_TRUE(locker.owner_died());
  }
  EXPECT_EQ(Mutex::State::kLocked, mutex->TryLock());

  mutex->Unlock();
  mutex->~Mutex();
}

}  // namespace aos::testing

namespace aos::testing {

// Microbenchmarks for the uncontended fixed cost of each aos_mutex path.
// Reported via printf rather than asserted on: the point is visibility into
// how expensive the paths are on each platform, not enforcing numbers.
//
// The private case exercises the futex-word path (which on Windows includes
// the per-operation FutexCache classification lookup); the shared file-backed
// case exercises whatever the OS backs shared-memory mutexes with (a named
// kernel mutex on Windows, the same futex path as private elsewhere); first
// touch measures the one-time per-thread-per-address cost (on Windows,
// classification plus named kernel mutex creation).
namespace {

// A block of file-backed shared memory, so mutexes in it take the
// shared-memory path on platforms which distinguish one.
class FileBackedMemory {
 public:
  static constexpr size_t kSize = 4 * 1024 * 1024;

  FileBackedMemory() {
#ifdef _WIN32
    char temp_path[MAX_PATH];
    ABSL_CHECK_NE(GetTempPathA(sizeof(temp_path), temp_path), 0u);
    static int counter = 0;
    const std::string path = std::string(temp_path) + "aos_mutex_benchmark_" +
                             std::to_string(GetCurrentProcessId()) + "_" +
                             std::to_string(++counter);
    file_ =
        CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE,
                    FILE_SHARE_READ | FILE_SHARE_WRITE, NULL, CREATE_ALWAYS,
                    FILE_ATTRIBUTE_NORMAL | FILE_FLAG_DELETE_ON_CLOSE, NULL);
    ABSL_CHECK_NE(file_, INVALID_HANDLE_VALUE) << GetLastError();
    mapping_ = CreateFileMappingA(file_, NULL, PAGE_READWRITE, 0, kSize, NULL);
    ABSL_CHECK(mapping_ != nullptr) << GetLastError();
    data_ = MapViewOfFile(mapping_, FILE_MAP_WRITE | FILE_MAP_READ, 0, 0, 0);
    ABSL_CHECK(data_ != nullptr) << GetLastError();
#else
    char path[] = "/tmp/aos_mutex_benchmark_XXXXXX";
    const int fd = mkstemp(path);
    ABSL_PCHECK(fd != -1);
    ABSL_PCHECK(unlink(path) == 0);
    ABSL_PCHECK(ftruncate(fd, kSize) == 0);
    data_ = mmap(nullptr, kSize, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    ABSL_PCHECK(data_ != MAP_FAILED);
    ABSL_PCHECK(close(fd) == 0);
#endif
    memset(data_, 0, kSize);
  }

  ~FileBackedMemory() {
#ifdef _WIN32
    UnmapViewOfFile(data_);
    CloseHandle(mapping_);
    CloseHandle(file_);
#else
    munmap(data_, kSize);
#endif
  }

  void *data() { return data_; }

 private:
#ifdef _WIN32
  HANDLE file_ = INVALID_HANDLE_VALUE;
  HANDLE mapping_ = nullptr;
#endif
  void *data_ = nullptr;
};

using BenchmarkClock = ::std::chrono::steady_clock;

double NsPerIteration(BenchmarkClock::time_point start,
                      BenchmarkClock::time_point end, int64_t iterations) {
  return static_cast<double>(
             ::std::chrono::duration_cast<::std::chrono::nanoseconds>(end -
                                                                      start)
                 .count()) /
         static_cast<double>(iterations);
}

void BenchmarkLockUnlock(const char *name, aos_mutex *m, int64_t iterations) {
  // Warm any per-thread classification/cache state so first-touch costs don't
  // pollute the steady-state number.
  ABSL_CHECK_EQ(0, mutex_grab(m));
  mutex_unlock(m);

  const BenchmarkClock::time_point start = BenchmarkClock::now();
  for (int64_t i = 0; i < iterations; ++i) {
    ABSL_CHECK_EQ(0, mutex_grab(m));
    mutex_unlock(m);
  }
  const BenchmarkClock::time_point end = BenchmarkClock::now();
  printf("%-38s %10.1f ns/op\n", name, NsPerIteration(start, end, iterations));
}

}  // namespace

TEST(MutexBenchmarkTest, PrivateLockUnlock) {
  aos_mutex m;
  memset(&m, 0, sizeof(m));
  BenchmarkLockUnlock("private mutex lock+unlock", &m, 200000);
}

TEST(MutexBenchmarkTest, SharedLockUnlock) {
  FileBackedMemory memory;
  BenchmarkLockUnlock("shared mutex lock+unlock",
                      static_cast<aos_mutex *>(memory.data()), 50000);
}

TEST(MutexBenchmarkTest, SharedFirstTouch) {
  FileBackedMemory memory;
  constexpr int64_t kCount = 1024;
  constexpr size_t kStride = 64;
  static_assert(kCount * kStride <= FileBackedMemory::kSize);
  char *const base = static_cast<char *>(memory.data());

  const BenchmarkClock::time_point start = BenchmarkClock::now();
  for (int64_t i = 0; i < kCount; ++i) {
    aos_mutex *m = reinterpret_cast<aos_mutex *>(base + i * kStride);
    ABSL_CHECK_EQ(0, mutex_grab(m));
    mutex_unlock(m);
  }
  const BenchmarkClock::time_point end = BenchmarkClock::now();
  printf("%-38s %10.1f ns/op\n", "shared mutex first touch (per addr)",
         NsPerIteration(start, end, kCount));
}

}  // namespace aos::testing

namespace aos::testing {

// std::mutex as a reference point for the aos_mutex numbers above.
TEST(MutexBenchmarkTest, StdMutexLockUnlock) {
  ::std::mutex m;
  constexpr int64_t kIterations = 200000;
  m.lock();
  m.unlock();
  const BenchmarkClock::time_point start = BenchmarkClock::now();
  for (int64_t i = 0; i < kIterations; ++i) {
    m.lock();
    m.unlock();
  }
  const BenchmarkClock::time_point end = BenchmarkClock::now();
  printf("%-38s %10.1f ns/op\n", "std::mutex lock+unlock",
         NsPerIteration(start, end, kIterations));
}

// Regression test for a suspected macOS lost wakeup: sys_futex_unlock_pi
// there stores 0 (destroying the FUTEX_WAITERS bit) and wakes only ONE
// waiter.  The winner's acquire then CASes a bare TID in (nothing re-arms
// WAITERS on its behalf), so ITS unlock takes the no-waiters fast path and
// never wakes the remaining waiters, which sleep forever.  Windows escapes
// this by waking all waiters (each loser re-arms WAITERS before re-sleeping);
// Linux's FUTEX_UNLOCK_PI handles the bit in the kernel.
//
// With several threads parked on one mutex, every one of them must
// eventually acquire it once the holder unlocks.  LOG(FATAL)s on failure
// rather than hanging in join so the result is visible.
TEST(MutexWakeupTest, AllWaitersEventuallyAcquire) {
  aos_mutex m;
  memset(&m, 0, sizeof(m));
  ABSL_CHECK_EQ(0, mutex_grab(&m));

  constexpr int kThreads = 4;
  ::std::atomic<int> started{0};
  ::std::atomic<int> acquired{0};
  ::std::vector<::std::thread> threads;
  for (int i = 0; i < kThreads; ++i) {
    threads.emplace_back([&]() {
      started.fetch_add(1);
      ABSL_CHECK_EQ(0, mutex_grab(&m));
      acquired.fetch_add(1);
      // Hold briefly so successors are really parked when we unlock.
      ::std::this_thread::sleep_for(::std::chrono::milliseconds(1));
      mutex_unlock(&m);
    });
  }

  // Let every thread block in the kernel wait before the first unlock.
  while (started.load() < kThreads) {
    ::std::this_thread::yield();
  }
  ::std::this_thread::sleep_for(::std::chrono::milliseconds(100));
  mutex_unlock(&m);

  const BenchmarkClock::time_point deadline =
      BenchmarkClock::now() + ::std::chrono::seconds(10);
  while (acquired.load() < kThreads) {
    if (BenchmarkClock::now() > deadline) {
      ABSL_LOG(FATAL) << "lost wakeup: only " << acquired.load() << " of "
                      << kThreads
                      << " waiters ever acquired the mutex; the rest are "
                         "still parked with nobody left to wake them";
    }
    ::std::this_thread::sleep_for(::std::chrono::milliseconds(1));
  }
  for (auto &thread : threads) {
    thread.join();
  }
}

}  // namespace aos::testing

#ifdef _WIN32
namespace aos::testing {

// The private path with the FutexCache classification lookup bypassed
// (everything assumed process-private), to isolate what the cache costs on
// every operation.  Compare against PrivateLockUnlock.
TEST(MutexBenchmarkTest, PrivateNoCacheLockUnlock) {
  mutex_set_assume_private_for_testing(true);
  aos_mutex m;
  memset(&m, 0, sizeof(m));
  BenchmarkLockUnlock("private mutex lock+unlock (no cache)", &m, 200000);
  mutex_set_assume_private_for_testing(false);
}

}  // namespace aos::testing
#endif
