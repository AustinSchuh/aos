#include "frc/orin/decode_pool.h"

#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

namespace frc::apriltag::testing {

class DecodePoolTest : public ::testing::TestWithParam<int> {};

// Tests that every item runs exactly once, on a valid thread which isn't
// running another item, and that Run() waits for all of them, over many runs
// of different sizes.
TEST_P(DecodePoolTest, RunsEachItemOnce) {
  const int threads = GetParam();
  DecodePool pool(threads);
  ASSERT_EQ(pool.threads(), threads);

  std::vector<std::atomic<int>> busy(threads);
  for (size_t run = 0; run < 2000; ++run) {
    const size_t n = run % 67;
    std::vector<std::atomic<int>> calls(n);
    pool.Run(n, [&](int thread, size_t i) {
      ASSERT_GE(thread, 0);
      ASSERT_LT(thread, threads);
      ASSERT_LT(i, n);
      EXPECT_EQ(busy[thread].fetch_add(1), 0);
      // Uneven items, so the caller sometimes waits for a helper.
      if (i % 13 == 0) {
        std::this_thread::sleep_for(std::chrono::microseconds(30));
      }
      calls[i].fetch_add(1, std::memory_order_relaxed);
      busy[thread].fetch_sub(1);
    });
    for (size_t i = 0; i < n; ++i) {
      ASSERT_EQ(calls[i].load(std::memory_order_relaxed), 1)
          << "run " << run << " item " << i;
    }
  }
}

// Tests that the caller waits for a slow helper's item, past the spin.
TEST_P(DecodePoolTest, WaitsForSlowItems) {
  const int threads = GetParam();
  DecodePool pool(threads);
  for (size_t run = 0; run < 20; ++run) {
    std::atomic<int> finished = 0;
    pool.Run(threads * 2, [&](int, size_t) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      finished.fetch_add(1);
    });
    EXPECT_EQ(finished.load(), threads * 2);
  }
}

INSTANTIATE_TEST_SUITE_P(Threads, DecodePoolTest,
                         ::testing::Values(1, 2, 3, 6));

}  // namespace frc::apriltag::testing
