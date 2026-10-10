#include "aos/util/crc32.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <random>
#include <string_view>
#include <vector>

#include "absl/types/span.h"
#include "gtest/gtest.h"

namespace aos::testing {
namespace {

// The definition of the CRC, one bit at a time, to check everything else
// against.
uint32_t ReferenceCrc32(absl::Span<const uint8_t> data, uint32_t crc) {
  for (const uint8_t byte : data) {
    crc ^= byte;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320 : 0);
    }
  }
  return crc;
}

std::vector<uint8_t> RandomBytes(size_t size, uint32_t seed) {
  std::mt19937 generator(seed);
  std::vector<uint8_t> result(size);
  for (uint8_t &byte : result) {
    byte = generator();
  }
  return result;
}

}  // namespace

// Tests that we get the standard check value.  Log files on disk depend on
// this never changing.
TEST(Crc32Test, KnownAnswer) {
  const std::string_view check = "123456789";
  const absl::Span<const uint8_t> data(
      reinterpret_cast<const uint8_t *>(check.data()), check.size());
  // 0xCBF43926 is the published CRC-32 of this string.  We don't do the final
  // inversion.
  EXPECT_EQ(ComputeCrc32(data), ~0xCBF43926u);
  EXPECT_EQ(crc32_internal::AccumulateCrc32Portable(data, 0xFFFFFFFFu),
            ~0xCBF43926u);

  EXPECT_EQ(ComputeCrc32({}), 0xFFFFFFFFu);
}

// Tests every size around the boundaries the fast paths care about (8, 16, and
// 64 byte blocks), at every alignment, with a variety of starting values.
TEST(Crc32Test, MatchesReference) {
  const std::vector<uint8_t> bytes = RandomBytes(1024, 971);
  std::mt19937 generator(1868);
  for (size_t offset = 0; offset < 17; ++offset) {
    for (size_t size = 0; size < 700; ++size) {
      const absl::Span<const uint8_t> data(bytes.data() + offset, size);
      const uint32_t start = generator();
      const uint32_t expected = ReferenceCrc32(data, start);
      ASSERT_EQ(AccumulateCrc32(data, start), expected)
          << ": offset " << offset << " size " << size;
      ASSERT_EQ(crc32_internal::AccumulateCrc32Portable(data, start), expected)
          << ": offset " << offset << " size " << size;
    }
  }
}

// Tests buffers long enough to spend most of their time in the widest loop.
TEST(Crc32Test, LargeBuffers) {
  const std::vector<uint8_t> bytes = RandomBytes(300000, 254);
  for (const size_t size : {size_t{4096}, size_t{65536}, size_t{131072},
                            size_t{131073}, size_t{299999}, size_t{300000}}) {
    const absl::Span<const uint8_t> data(bytes.data(), size);
    const uint32_t expected = ReferenceCrc32(data, 0xFFFFFFFFu);
    EXPECT_EQ(ComputeCrc32(data), expected) << ": size " << size;
    EXPECT_EQ(crc32_internal::AccumulateCrc32Portable(data, 0xFFFFFFFFu),
              expected)
        << ": size " << size;
  }
}

// Tests that checksumming a buffer in arbitrary pieces matches doing it all at
// once.  The log writer does this a message at a time.
TEST(Crc32Test, AccumulateInPieces) {
  const std::vector<uint8_t> bytes = RandomBytes(20000, 4646);
  const uint32_t expected = ComputeCrc32(bytes);
  ASSERT_EQ(expected, ReferenceCrc32(bytes, 0xFFFFFFFFu));

  std::mt19937 generator(118);
  for (int iteration = 0; iteration < 200; ++iteration) {
    std::optional<uint32_t> crc;
    absl::Span<const uint8_t> remaining(bytes);
    while (!remaining.empty()) {
      const size_t piece = std::min<size_t>(
          remaining.size(),
          std::uniform_int_distribution<size_t>(0, 600)(generator));
      crc = AccumulateCrc32(remaining.subspan(0, piece), crc);
      remaining = remaining.subspan(piece);
    }
    ASSERT_EQ(crc.value(), expected);
  }
}

}  // namespace aos::testing
