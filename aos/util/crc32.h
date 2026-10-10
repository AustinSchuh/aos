#ifndef AOS_EVENTS_LOGGING_CRC32_H_
#define AOS_EVENTS_LOGGING_CRC32_H_

#include <stdint.h>

#include <optional>

#include "absl/types/span.h"

namespace aos {

// Returns the CRC-32 (IEEE 802.3 polynomial) register after running data
// through it, starting from all ones.  Note that this is the bitwise inverse of
// what zlib's crc32() returns, because the final inversion is left off.
uint32_t ComputeCrc32(const absl::Span<const uint8_t> data);

// Continues a CRC from current_checksum, which is the result of a previous
// call, or starts a new one if it is nullopt.  Checksumming a buffer in pieces
// this way gives the same answer as checksumming it all at once.
uint32_t AccumulateCrc32(
    absl::Span<const uint8_t> data,
    std::optional<uint32_t> current_checksum = std::nullopt);

namespace crc32_internal {

// The implementation used when there is no hardware support to lean on.
// Exposed so it can be tested on machines which never fall back to it.
uint32_t AccumulateCrc32Portable(absl::Span<const uint8_t> data, uint32_t crc);

}  // namespace crc32_internal

}  // namespace aos

#endif  // AOS_EVENTS_LOGGING_CRC32_H_
