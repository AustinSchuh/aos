#include "aos/events/logging/snappy_encoder.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

#include "absl/types/span.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "aos/events/logging/buffer_encoder_param_test.h"
#include "aos/testing/tmpdir.h"
#include "aos/util/file.h"

namespace aos::logger::testing {

INSTANTIATE_TEST_SUITE_P(
    Snappy, BufferEncoderTest,
    ::testing::Combine(::testing::Values([](size_t max_message_size) {
                         return std::make_unique<SnappyEncoder>(
                             max_message_size);
                       }),
                       ::testing::Values([](std::string_view filename) {
                         return std::make_unique<SnappyDecoder>(filename);
                       }),
                       ::testing::Range(0, 100)));

// Tests that ReadSome() stops at the end of a chunk when the next one does not
// fit in what is left of the buffer, and picks up from there on the next call.
// That is what lets every chunk get uncompressed directly into the buffer it
// was asked to fill.
TEST(SnappyDecoderTest, ReadSomeStopsBetweenChunks) {
  const std::string file_path = aos::testing::TestTmpDir() + "/chunks.sz";

  // 10 messages of 400 bytes, with the encoder set to finish a chunk once it
  // has 1000 bytes.  That makes 3 chunks of 1200 bytes and one of 400.
  std::vector<uint8_t> expected;
  {
    SnappyEncoder encoder(400, 1000);
    for (int i = 0; i < 10; ++i) {
      std::vector<uint8_t> message(400);
      for (size_t j = 0; j < message.size(); ++j) {
        message[j] = i * 7 + j / 16;
      }
      DataEncoder::SpanCopier copier(message);
      ASSERT_EQ(encoder.Encode(&copier, 0), message.size());
      expected.insert(expected.end(), message.begin(), message.end());
    }
    encoder.Finish();

    std::ofstream output_file(file_path, std::ios::binary);
    for (const absl::Span<const uint8_t> span : encoder.queue()) {
      output_file.write(reinterpret_cast<const char *>(span.data()),
                        span.size());
    }
  }

  {
    // Read() fills whatever it is given, even though that means splitting the
    // second chunk.
    SnappyDecoder decoder(file_path);
    std::vector<uint8_t> buffer(2000);
    ASSERT_EQ(decoder.Read(buffer.data(), buffer.data() + buffer.size()),
              2000u);
    EXPECT_TRUE(std::equal(buffer.begin(), buffer.end(), expected.begin()));
  }

  SnappyDecoder decoder(file_path);
  std::vector<uint8_t> decoded;
  const auto read_some = [&decoder, &decoded](size_t size) {
    std::vector<uint8_t> buffer(size);
    const size_t result =
        decoder.ReadSome(buffer.data(), buffer.data() + buffer.size());
    decoded.insert(decoded.end(), buffer.begin(), buffer.begin() + result);
    return result;
  };

  // The second chunk does not fit after the first, so only the first comes
  // back.
  EXPECT_EQ(read_some(2000), 1200u);
  // The second fits exactly, and the third is not started.
  EXPECT_EQ(read_some(1200), 1200u);
  // A chunk bigger than the whole buffer still has to make progress.
  EXPECT_EQ(read_some(500), 500u);
  // The rest of the third chunk, followed by the last chunk because there is
  // room for it.
  EXPECT_EQ(read_some(2000), 1100u);
  EXPECT_EQ(read_some(2000), 0u);

  EXPECT_EQ(decoded, expected);
}

}  // namespace aos::logger::testing
