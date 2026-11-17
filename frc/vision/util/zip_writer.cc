#include "frc/vision/util/zip_writer.h"

#include <array>

namespace frc::vision {
namespace {

void Put16(std::string &out, uint16_t value) {
  out += static_cast<char>(value & 0xff);
  out += static_cast<char>(value >> 8);
}

void Put32(std::string &out, uint32_t value) {
  for (int i = 0; i < 4; ++i) {
    out += static_cast<char>((value >> (8 * i)) & 0xff);
  }
}

// 1980-01-01 00:00, the earliest time a zip can record: no clock, so every
// archive of the same files is the same.
constexpr uint16_t kDosTime = 0;
constexpr uint16_t kDosDate = (1 << 5) | 1;

}  // namespace

uint32_t Crc32(std::string_view data) {
  static const std::array<uint32_t, 256> kTable = [] {
    std::array<uint32_t, 256> table;
    for (uint32_t i = 0; i < 256; ++i) {
      uint32_t c = i;
      for (int k = 0; k < 8; ++k) {
        c = (c & 1) ? 0xedb88320u ^ (c >> 1) : c >> 1;
      }
      table[i] = c;
    }
    return table;
  }();
  uint32_t crc = 0xffffffffu;
  for (unsigned char byte : data) {
    crc = kTable[(crc ^ byte) & 0xff] ^ (crc >> 8);
  }
  return crc ^ 0xffffffffu;
}

void ZipWriter::Add(std::string_view name, std::string_view contents) {
  const Entry entry{.name = std::string(name),
                    .crc = Crc32(contents),
                    .size = static_cast<uint32_t>(contents.size()),
                    .offset = static_cast<uint32_t>(data_.size())};
  // The local file header.
  Put32(data_, 0x04034b50);
  Put16(data_, 20);  // Version needed: 2.0.
  Put16(data_, 0);   // Flags.
  Put16(data_, 0);   // Stored.
  Put16(data_, kDosTime);
  Put16(data_, kDosDate);
  Put32(data_, entry.crc);
  Put32(data_, entry.size);
  Put32(data_, entry.size);
  Put16(data_, entry.name.size());
  Put16(data_, 0);  // Extra field.
  data_ += entry.name;
  data_ += contents;
  entries_.push_back(entry);
}

std::string ZipWriter::Finish() const {
  std::string out = data_;
  const uint32_t directory_offset = out.size();
  for (const Entry &entry : entries_) {
    Put32(out, 0x02014b50);
    Put16(out, 20);  // Made by 2.0.
    Put16(out, 20);  // Version needed.
    Put16(out, 0);   // Flags.
    Put16(out, 0);   // Stored.
    Put16(out, kDosTime);
    Put16(out, kDosDate);
    Put32(out, entry.crc);
    Put32(out, entry.size);
    Put32(out, entry.size);
    Put16(out, entry.name.size());
    Put16(out, 0);  // Extra field.
    Put16(out, 0);  // Comment.
    Put16(out, 0);  // Disk.
    Put16(out, 0);  // Internal attributes.
    Put32(out, 0);  // External attributes.
    Put32(out, entry.offset);
    out += entry.name;
  }
  const uint32_t directory_size = out.size() - directory_offset;
  Put32(out, 0x06054b50);
  Put16(out, 0);
  Put16(out, 0);
  Put16(out, entries_.size());
  Put16(out, entries_.size());
  Put32(out, directory_size);
  Put32(out, directory_offset);
  Put16(out, 0);  // Comment.
  return out;
}

}  // namespace frc::vision
