#include "frc/orin/decode_pool.h"

#include <linux/futex.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <climits>

#include "absl/log/absl_check.h"

namespace frc::apriltag {
namespace {

// How long the caller spins for the helpers' last items before sleeping.  An
// item takes tens of microseconds, so the last one usually finishes within
// this, and a futex wake costs about as much.
constexpr std::chrono::microseconds kCallerSpin{20};

void FutexWait(std::atomic<uint32_t> *word, uint32_t expected) {
  syscall(SYS_futex, reinterpret_cast<uint32_t *>(word),
          FUTEX_WAIT | FUTEX_PRIVATE_FLAG, expected, nullptr, nullptr, 0);
}

void FutexWake(std::atomic<uint32_t> *word, int count) {
  syscall(SYS_futex, reinterpret_cast<uint32_t *>(word),
          FUTEX_WAKE | FUTEX_PRIVATE_FLAG, count, nullptr, nullptr, 0);
}

void CpuRelax() {
#if defined(__aarch64__)
  asm volatile("yield" ::: "memory");
#elif defined(__x86_64__)
  asm volatile("pause" ::: "memory");
#endif
}

}  // namespace

DecodePool::DecodePool(int threads) {
  ABSL_CHECK_GE(threads, 1);
  static_assert(sizeof(std::atomic<uint32_t>) == sizeof(uint32_t));
  helpers_.reserve(threads - 1);
  for (int thread = 1; thread < threads; ++thread) {
    helpers_.emplace_back([this, thread]() { HelperLoop(thread); });
  }
}

DecodePool::~DecodePool() {
  stop_.store(true, std::memory_order_relaxed);
  generation_.fetch_add(1, std::memory_order_release);
  FutexWake(&generation_, INT_MAX);
  for (std::thread &helper : helpers_) {
    helper.join();
  }
}

void DecodePool::Run(size_t n,
                     absl::FunctionRef<void(int thread, size_t i)> body) {
  if (n == 0) {
    return;
  }
  if (helpers_.empty() || n == 1) {
    for (size_t i = 0; i < n; ++i) {
      body(0, i);
    }
    return;
  }
  ABSL_CHECK_LE(n, UINT32_MAX);

  const uint32_t generation = generation_.load(std::memory_order_relaxed) + 1;
  n_ = n;
  body_ = &body;
  done_.store(0, std::memory_order_relaxed);
  unclaimed_.store(static_cast<uint64_t>(generation) << 32 | n,
                   std::memory_order_release);
  generation_.store(generation, std::memory_order_release);
  // Items beyond what the caller and the woken helpers can run at once would
  // only wake helpers to find nothing left.
  FutexWake(&generation_,
            static_cast<int>(std::min<size_t>(helpers_.size(), n - 1)));

  Work(0, generation);

  // Everything is claimed, so what's left is at most one item per helper.
  const auto spin_end = std::chrono::steady_clock::now() + kCallerSpin;
  while (done_.load(std::memory_order_acquire) != n) {
    if (std::chrono::steady_clock::now() >= spin_end) {
      // Seen by whichever helper finishes the last item, or it isn't and the
      // caller sees that item done instead.
      caller_waiting_.store(1, std::memory_order_seq_cst);
      uint32_t done;
      while ((done = done_.load(std::memory_order_seq_cst)) != n) {
        FutexWait(&done_, done);
      }
      caller_waiting_.store(0, std::memory_order_relaxed);
      break;
    }
    CpuRelax();
  }
  body_ = nullptr;
}

void DecodePool::Work(int thread, uint32_t generation) {
  uint64_t unclaimed = unclaimed_.load(std::memory_order_relaxed);
  while (true) {
    if (static_cast<uint32_t>(unclaimed >> 32) != generation ||
        static_cast<uint32_t>(unclaimed) == 0) {
      return;
    }
    if (!unclaimed_.compare_exchange_weak(unclaimed, unclaimed - 1,
                                          std::memory_order_acquire,
                                          std::memory_order_relaxed)) {
      continue;
    }
    // The item is claimed, so Run() is still waiting on this generation.
    const uint32_t n = n_;
    (*body_)(thread, static_cast<uint32_t>(unclaimed) - 1);
    const uint32_t done = done_.fetch_add(1, std::memory_order_seq_cst) + 1;
    if (done == n && caller_waiting_.load(std::memory_order_seq_cst) != 0) {
      FutexWake(&done_, 1);
    }
    unclaimed = unclaimed_.load(std::memory_order_relaxed);
  }
}

void DecodePool::HelperLoop(int thread) {
  uint32_t seen = 0;
  while (true) {
    const uint32_t generation = generation_.load(std::memory_order_acquire);
    if (generation == seen) {
      FutexWait(&generation_, seen);
      continue;
    }
    if (stop_.load(std::memory_order_relaxed)) {
      return;
    }
    seen = generation;
    Work(thread, generation);
  }
}

}  // namespace frc::apriltag
