#include "frc/vision/util/zip_writer.h"

#include <cstring>

#include "gtest/gtest.h"

namespace frc::vision::testing {
namespace {

uint32_t Get32(const std::string &data, size_t at) {
  uint32_t value = 0;
  for (int i = 3; i >= 0; --i) {
    value = (value << 8) | static_cast<unsigned char>(data[at + i]);
  }
  return value;
}

uint16_t Get16(const std::string &data, size_t at) {
  return static_cast<unsigned char>(data[at]) |
         (static_cast<unsigned char>(data[at + 1]) << 8);
}

TEST(ZipWriterTest, Crc32IsZips) {
  // The standard check value.
  EXPECT_EQ(Crc32("123456789"), 0xcbf43926u);
  EXPECT_EQ(Crc32(""), 0u);
}

// Reads the archive back through its central directory, as unzip does.
TEST(ZipWriterTest, RoundTrips) {
  ZipWriter writer;
  writer.Add("frame_0000.png", std::string("\x89PNG\r\n\x1a\n data", 13));
  writer.Add("board.txt", "12x9");
  const std::string zip = writer.Finish();

  const size_t end = zip.size() - 22;
  ASSERT_EQ(Get32(zip, end), 0x06054b50u);
  EXPECT_EQ(Get16(zip, end + 10), 2);
  size_t entry = Get32(zip, end + 16);
  std::vector<std::pair<std::string, std::string>> files;
  for (int i = 0; i < 2; ++i) {
    ASSERT_EQ(Get32(zip, entry), 0x02014b50u);
    const uint32_t crc = Get32(zip, entry + 16);
    const uint32_t size = Get32(zip, entry + 24);
    const uint16_t name_length = Get16(zip, entry + 28);
    const uint32_t local = Get32(zip, entry + 42);
    const std::string name = zip.substr(entry + 46, name_length);
    ASSERT_EQ(Get32(zip, local), 0x04034b50u);
    const size_t data = local + 30 + Get16(zip, local + 26);
    const std::string contents = zip.substr(data, size);
    EXPECT_EQ(Crc32(contents), crc) << name;
    files.emplace_back(name, contents);
    entry += 46 + name_length;
  }
  EXPECT_EQ(files[0].first, "frame_0000.png");
  EXPECT_EQ(files[0].second.size(), 13u);
  EXPECT_EQ(files[1].first, "board.txt");
  EXPECT_EQ(files[1].second, "12x9");
}

}  // namespace
}  // namespace frc::vision::testing
