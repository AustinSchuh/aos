#ifndef FRC_VISION_UTIL_ZIP_WRITER_H_
#define FRC_VISION_UTIL_ZIP_WRITER_H_

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace frc::vision {

// The CRC-32 a zip file records, of `data`.
uint32_t Crc32(std::string_view data);

// Writes a zip file of files stored as they are, uncompressed: for files that
// are compressed already, such as PNGs, and archives of under 4 GiB.
class ZipWriter {
 public:
  // Adds `name`, holding `contents`.
  void Add(std::string_view name, std::string_view contents);

  // The zip file.
  std::string Finish() const;

 private:
  struct Entry {
    std::string name;
    uint32_t crc;
    uint32_t size;
    uint32_t offset;
  };

  std::string data_;
  std::vector<Entry> entries_;
};

}  // namespace frc::vision

#endif  // FRC_VISION_UTIL_ZIP_WRITER_H_
