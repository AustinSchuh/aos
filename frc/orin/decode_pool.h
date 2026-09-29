#ifndef FRC_ORIN_DECODE_POOL_H_
#define FRC_ORIN_DECODE_POOL_H_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <thread>
#include <vector>

#include "absl/functional/function_ref.h"

namespace frc::apriltag {

// Runs a loop body over [0, n) on the calling thread and a fixed set of helper
// threads, for work items which cost about the same.  Each thread claims one
// item at a time with an atomic compare-and-swap, so there are no locks, no
// task queue, and no yields.  The caller claims items too instead of sleeping,
// and waits only for the items already claimed when it runs out.
class DecodePool {
 public:
  // threads counts the caller, so threads - 1 helpers start here.  They
  // inherit the creating thread's scheduling policy and CPU affinity.
  explicit DecodePool(int threads);
  DecodePool(const DecodePool &) = delete;
  DecodePool &operator=(const DecodePool &) = delete;
  ~DecodePool();

  int threads() const { return helpers_.size() + 1; }

  // Calls body(thread, i) once for each i in [0, n), in no particular order,
  // and returns once every call has returned.  thread is in [0, threads()),
  // with 0 for the caller, and no two calls with the same thread overlap.
  // Only one thread may call Run() at a time.
  void Run(size_t n, absl::FunctionRef<void(int thread, size_t i)> body);

 private:
  void HelperLoop(int thread);

  // Runs claimed items of generation until none are left.
  void Work(int thread, uint32_t generation);

  // The generation of the latest Run() in the high 32 bits, and how many of
  // its items are unclaimed in the low 32.  Items are claimed from the top
  // down, and the generation keeps a helper which wakes late from claiming an
  // item of a later Run() it has not seen.
  std::atomic<uint64_t> unclaimed_{0};
  // The generation of the latest Run(), which helpers sleep on.
  std::atomic<uint32_t> generation_{0};
  // Items of the latest Run() which have finished, and whether the caller is
  // asleep waiting for the last of them.
  std::atomic<uint32_t> done_{0};
  std::atomic<uint32_t> caller_waiting_{0};
  std::atomic<bool> stop_{false};

  // Only read by a thread holding a claimed item, so Run() can't change them
  // underneath it.
  uint32_t n_ = 0;
  const absl::FunctionRef<void(int, size_t)> *body_ = nullptr;

  std::vector<std::thread> helpers_;
};

}  // namespace frc::apriltag

#endif  // FRC_ORIN_DECODE_POOL_H_
