#include "aos/util/crc32.h"

#include <array>
#include <bit>
#include <cstddef>
#include <cstring>

#if defined(__x86_64__) && (defined(__GNUC__) || defined(__clang__))
#include <immintrin.h>
#endif
#if defined(__aarch64__) && defined(__ARM_FEATURE_CRC32)
#include <arm_acle.h>
#endif

namespace aos {

namespace {

// The reflected form of the CRC-32 (IEEE 802.3) polynomial.  Every
// implementation below works in the reflected domain, where bit 0 of a byte is
// the highest order coefficient.
constexpr uint32_t kPolynomial = 0xEDB88320;

// Builds the lookup tables for slicing-by-8.  kTables[0] is the classic
// byte-at-a-time table; kTables[n][b] is the CRC of byte b followed by n zero
// bytes, which is what lets 8 input bytes be folded in with 8 independent
// lookups instead of 8 lookups that each wait on the last.
constexpr std::array<std::array<uint32_t, 256>, 8> MakeTables() {
  std::array<std::array<uint32_t, 256>, 8> tables{};
  for (uint32_t byte = 0; byte < 256; ++byte) {
    uint32_t crc = byte;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1) ^ ((crc & 1) ? kPolynomial : 0);
    }
    tables[0][byte] = crc;
  }
  for (size_t slice = 1; slice < tables.size(); ++slice) {
    for (uint32_t byte = 0; byte < 256; ++byte) {
      const uint32_t previous = tables[slice - 1][byte];
      tables[slice][byte] = (previous >> 8) ^ tables[0][previous & 0xFF];
    }
  }
  return tables;
}

constexpr std::array<std::array<uint32_t, 256>, 8> kTables = MakeTables();

// The word-at-a-time loads below read the input as little endian.
static_assert(std::endian::native == std::endian::little);

#if defined(__x86_64__) && (defined(__GNUC__) || defined(__clang__))
#define AOS_CRC32_HAS_PCLMUL 1

// Returns x^power mod the CRC polynomial, in the reflected 32 bit form.
constexpr uint32_t XPowModPolynomial(int power) {
  // x^0, since the highest order coefficient is bit 0.
  uint32_t result = 0x80000000;
  for (int i = 0; i < power; ++i) {
    result = (result >> 1) ^ ((result & 1) ? kPolynomial : 0);
  }
  return result;
}

// Returns the constant which a 64 bit reflected value can be carry-less
// multiplied by to move it bits further up the message, modulo the CRC
// polynomial.  A reflected carry-less multiply produces one more factor of x
// than the operands hold, hence the "- 1".
constexpr uint64_t FoldConstant(int bits) {
  return static_cast<uint64_t>(XPowModPolynomial(bits - 1)) << 32;
}

// These need to be variables to force them to be computed at compile time.
// Computing them takes far longer than checksumming a small message does.
constexpr uint64_t kFoldBy512Low = FoldConstant(512 + 64);
constexpr uint64_t kFoldBy512High = FoldConstant(512);
constexpr uint64_t kFoldBy128Low = FoldConstant(128 + 64);
constexpr uint64_t kFoldBy128High = FoldConstant(128);

// Folds state forwards by the distance baked into constants, and adds data in
// at the place it landed.  The low half of state is 64 bits earlier in the
// message than the high half, so it has 64 bits further to go and needs its
// own constant.
__attribute__((target("pclmul"))) inline __m128i Fold(__m128i state,
                                                      __m128i constants,
                                                      __m128i data) {
  return _mm_xor_si128(
      _mm_xor_si128(_mm_clmulepi64_si128(state, constants, 0x00),
                    _mm_clmulepi64_si128(state, constants, 0x11)),
      data);
}

// Reduces data to 16 bytes which have the same CRC that all of data did when
// started from crc, and returns those bytes in out.  size must be a multiple
// of 16 and at least 64.
//
// This is the folding scheme from Intel's "Fast CRC Computation for Generic
// Polynomials Using PCLMULQDQ Instruction": 128 bits of message are congruent
// to themselves multiplied by x^n and placed n bits later, so 4 lanes of state
// leapfrog down the buffer 64 bytes at a time without any dependency between
// them.
__attribute__((target("pclmul"))) void FoldTo16Bytes(const uint8_t *data,
                                                     size_t size, uint32_t crc,
                                                     uint8_t *out) {
  const auto load = [](const uint8_t *pointer) {
    return _mm_loadu_si128(reinterpret_cast<const __m128i *>(pointer));
  };

  // Starting from crc is the same as starting from 0 with crc xored into the
  // first 4 bytes.
  __m128i lane0 = _mm_xor_si128(load(data), _mm_cvtsi32_si128(crc));
  __m128i lane1 = load(data + 16);
  __m128i lane2 = load(data + 32);
  __m128i lane3 = load(data + 48);
  data += 64;
  size -= 64;

  const __m128i fold_by_512 = _mm_set_epi64x(kFoldBy512High, kFoldBy512Low);
  while (size >= 64) {
    lane0 = Fold(lane0, fold_by_512, load(data));
    lane1 = Fold(lane1, fold_by_512, load(data + 16));
    lane2 = Fold(lane2, fold_by_512, load(data + 32));
    lane3 = Fold(lane3, fold_by_512, load(data + 48));
    data += 64;
    size -= 64;
  }

  // Each lane is 16 bytes in front of the next, so fold them together one at a
  // time, and then keep going for whatever whole blocks are left.
  const __m128i fold_by_128 = _mm_set_epi64x(kFoldBy128High, kFoldBy128Low);
  __m128i state = Fold(lane0, fold_by_128, lane1);
  state = Fold(state, fold_by_128, lane2);
  state = Fold(state, fold_by_128, lane3);
  while (size >= 16) {
    state = Fold(state, fold_by_128, load(data));
    data += 16;
    size -= 16;
  }

  _mm_storeu_si128(reinterpret_cast<__m128i *>(out), state);
}

bool HasPclmul() {
  static const bool result = __builtin_cpu_supports("pclmul");
  return result;
}
#endif  // defined(__x86_64__) && (defined(__GNUC__) || defined(__clang__))

}  // namespace

namespace crc32_internal {

uint32_t AccumulateCrc32Portable(const absl::Span<const uint8_t> data,
                                 uint32_t crc) {
  const uint8_t *pointer = data.data();
  size_t size = data.size();
  while (size >= 8) {
    uint32_t low;
    uint32_t high;
    memcpy(&low, pointer, sizeof(low));
    memcpy(&high, pointer + sizeof(low), sizeof(high));
    low ^= crc;
    crc = kTables[7][low & 0xFF] ^ kTables[6][(low >> 8) & 0xFF] ^
          kTables[5][(low >> 16) & 0xFF] ^ kTables[4][low >> 24] ^
          kTables[3][high & 0xFF] ^ kTables[2][(high >> 8) & 0xFF] ^
          kTables[1][(high >> 16) & 0xFF] ^ kTables[0][high >> 24];
    pointer += 8;
    size -= 8;
  }
  while (size > 0) {
    crc = kTables[0][(crc ^ *pointer) & 0xFF] ^ (crc >> 8);
    ++pointer;
    --size;
  }
  return crc;
}

}  // namespace crc32_internal

uint32_t ComputeCrc32(const absl::Span<const uint8_t> data) {
  return AccumulateCrc32(data, std::nullopt);
}

uint32_t AccumulateCrc32(absl::Span<const uint8_t> data,
                         std::optional<uint32_t> current_checksum) {
  uint32_t crc =
      current_checksum.has_value() ? current_checksum.value() : 0xFF'FF'FF'FF;
#if defined(__aarch64__) && defined(__ARM_FEATURE_CRC32)
  // ARMv8's CRC32 instructions use this polynomial, so there is nothing to
  // fall back from.
  const uint8_t *pointer = data.data();
  size_t size = data.size();
  while (size >= 8) {
    uint64_t word;
    memcpy(&word, pointer, sizeof(word));
    crc = __crc32d(crc, word);
    pointer += 8;
    size -= 8;
  }
  while (size > 0) {
    crc = __crc32b(crc, *pointer);
    ++pointer;
    --size;
  }
  return crc;
#else
#if defined(AOS_CRC32_HAS_PCLMUL)
  if (data.size() >= 64 && HasPclmul()) {
    const size_t folded_size = data.size() & ~static_cast<size_t>(15);
    std::array<uint8_t, 16> folded;
    FoldTo16Bytes(data.data(), folded_size, crc, folded.data());
    // The folded bytes stand in for everything up to folded_size, so finish
    // them and the tail off the ordinary way.
    crc = crc32_internal::AccumulateCrc32Portable(folded, 0);
    data = data.subspan(folded_size);
  }
#endif
  return crc32_internal::AccumulateCrc32Portable(data, crc);
#endif
}

}  // namespace aos
