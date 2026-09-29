#include <stdint.h>

#include <algorithm>
#include <memory>
#include <random>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "third_party/apriltag/apriltag.h"
#include "third_party/apriltag/common/image_u8.h"

#include "aos/testing/random_seed.h"
#include "frc/orin/cuda.h"
#include "frc/orin/threshold.h"

extern "C" {
image_u8_t *threshold(apriltag_detector_t *td, image_u8_t *im);
}

namespace frc::apriltag::testing {
namespace {

// Fills the GPU outputs before thresholding, and the bytes past their ends,
// which the threshold must leave alone.
constexpr uint8_t kSentinel = 0xa5;
constexpr size_t kGuardBytes = 256;

enum class Pattern {
  // Uniform noise, so every block has plenty of contrast.
  kNoise,
  // Noise of a different range in each 8x8 raw block, so blocks on both sides
  // of min_white_black_diff sit next to each other.
  kPatchyContrast,
  // One value everywhere, so every block is flat unless min_white_black_diff
  // is 0.
  kConstant,
  // Dark and bright rectangles on a gradient, like tags in a scene.
  kRectangles,
};

std::string PatternName(Pattern pattern) {
  switch (pattern) {
    case Pattern::kNoise:
      return "noise";
    case Pattern::kPatchyContrast:
      return "patchy contrast";
    case Pattern::kConstant:
      return "constant";
    case Pattern::kRectangles:
      return "rectangles";
  }
  return "unknown";
}

std::vector<uint8_t> MakeImage(Pattern pattern, size_t width, size_t height,
                               std::mt19937 *random) {
  std::vector<uint8_t> image(width * height);
  std::uniform_int_distribution<int> byte(0, 255);
  switch (pattern) {
    case Pattern::kNoise:
      for (uint8_t &pixel : image) {
        pixel = byte(*random);
      }
      break;
    case Pattern::kPatchyContrast: {
      std::uniform_int_distribution<int> spread_distribution(0, 48);
      for (size_t block_y = 0; block_y < height; block_y += 8) {
        for (size_t block_x = 0; block_x < width; block_x += 8) {
          const int spread = spread_distribution(*random);
          const int base =
              std::uniform_int_distribution<int>(0, 255 - spread)(*random);
          std::uniform_int_distribution<int> pixel(base, base + spread);
          for (size_t y = block_y; y < block_y + 8; ++y) {
            for (size_t x = block_x; x < block_x + 8; ++x) {
              image[y * width + x] = pixel(*random);
            }
          }
        }
      }
      break;
    }
    case Pattern::kConstant:
      std::fill(image.begin(), image.end(), 128);
      break;
    case Pattern::kRectangles: {
      for (size_t y = 0; y < height; ++y) {
        for (size_t x = 0; x < width; ++x) {
          image[y * width + x] = 64 + 128 * (x + y) / (width + height);
        }
      }
      const size_t count = 1 + width * height / 4096;
      for (size_t i = 0; i < count; ++i) {
        const size_t x0 =
            std::uniform_int_distribution<size_t>(0, width - 1)(*random);
        const size_t y0 =
            std::uniform_int_distribution<size_t>(0, height - 1)(*random);
        const size_t x1 = std::min(
            width,
            x0 + 1 + std::uniform_int_distribution<size_t>(0, 63)(*random));
        const size_t y1 = std::min(
            height,
            y0 + 1 + std::uniform_int_distribution<size_t>(0, 63)(*random));
        const uint8_t value = i % 2 == 0 ? 20 : 235;
        for (size_t y = y0; y < y1; ++y) {
          for (size_t x = x0; x < x1; ++x) {
            image[y * width + x] = value;
          }
        }
      }
      break;
    }
  }
  return image;
}

// Checks that a GPU image of width x height pixels matches the aprilrobotics
// one, and that the guard bytes past it are untouched.
void ExpectImagesMatch(const std::vector<uint8_t> &gpu,
                       const image_u8_t &aprilrobotics, size_t width,
                       size_t height, std::string_view label) {
  ASSERT_EQ(static_cast<size_t>(aprilrobotics.width), width);
  ASSERT_EQ(static_cast<size_t>(aprilrobotics.height), height);
  for (size_t y = 0; y < height; ++y) {
    for (size_t x = 0; x < width; ++x) {
      ASSERT_EQ(gpu[y * width + x],
                aprilrobotics.buf[y * aprilrobotics.stride + x])
          << label << " differs at " << x << ", " << y;
    }
  }
  for (size_t i = width * height; i < gpu.size(); ++i) {
    ASSERT_EQ(gpu[i], kSentinel)
        << label << " written " << i - width * height << " bytes past its end";
  }
}

class Mono8ThresholdTest
    : public ::testing::TestWithParam<std::pair<size_t, size_t>> {};

// Checks that the GPU decimates and thresholds MONO8 images exactly like
// aprilrobotics, for images whose threshold blocks leave the fused kernel's
// thread blocks full or partial, and for images 4 bytes past an 8 byte
// boundary, which is as aligned as a flatbuffer's image is.
TEST_P(Mono8ThresholdTest, MatchesAprilRobotics) {
  const auto [width, height] = GetParam();
  const size_t decimated_width = width / 2;
  const size_t decimated_height = height / 2;
  const size_t decimated_size = decimated_width * decimated_height;

  std::mt19937 random(aos::testing::RandomSeed());
  std::unique_ptr<Threshold> gpu_threshold =
      MakeThreshold(vision::ImageFormat::MONO8, width, height);
  apriltag_detector_t *tag_detector = apriltag_detector_create();

  // Room to start the image 4 bytes into an allocation, which cudaMalloc
  // aligns to at least 256 bytes.
  GpuMemory<uint8_t> image_device(width * height + 4);
  GpuMemory<uint8_t> decimated_device(decimated_size + kGuardBytes);
  GpuMemory<uint8_t> thresholded_device(decimated_size + kGuardBytes);
  CudaStream stream;

  for (Pattern pattern : {Pattern::kNoise, Pattern::kPatchyContrast,
                          Pattern::kConstant, Pattern::kRectangles}) {
    std::vector<uint8_t> image = MakeImage(pattern, width, height, &random);
    image_u8_t image_u8 = {
        .width = static_cast<int32_t>(width),
        .height = static_cast<int32_t>(height),
        .stride = static_cast<int32_t>(width),
        .buf = image.data(),
    };
    image_u8_t *decimated = image_u8_decimate(&image_u8, 2);

    for (apriltag_size_t min_white_black_diff : {0, 1, 5, 20, 255, 256}) {
      tag_detector->qtp.min_white_black_diff = min_white_black_diff;
      image_u8_t *thresholded = threshold(tag_detector, decimated);
      for (size_t offset : {0, 4}) {
        SCOPED_TRACE(PatternName(pattern) + ", min_white_black_diff " +
                     std::to_string(min_white_black_diff) + ", offset " +
                     std::to_string(offset));
        CHECK_CUDA(cudaMemcpy(image_device.get() + offset, image.data(),
                              image.size(), cudaMemcpyHostToDevice));
        decimated_device.MemsetAsync(kSentinel, &stream);
        thresholded_device.MemsetAsync(kSentinel, &stream);
        gpu_threshold->ThresholdAndDecimate(
            image_device.get() + offset, decimated_device.get(),
            thresholded_device.get(), min_white_black_diff, &stream);
        CHECK_CUDA(cudaStreamSynchronize(stream.get()));
        CHECK_CUDA(cudaGetLastError());

        ExpectImagesMatch(decimated_device.Copy(), *decimated, decimated_width,
                          decimated_height, "decimated");
        ExpectImagesMatch(thresholded_device.Copy(), *thresholded,
                          decimated_width, decimated_height, "thresholded");
      }
      image_u8_destroy(thresholded);
    }
    image_u8_destroy(decimated);
  }
  apriltag_detector_destroy(tag_detector);
}

// Each thread block of the fused kernel thresholds 30 x 14 of the decimated
// image's 4x4 blocks.  An image of W x H pixels has W / 8 x H / 8 of them.
INSTANTIATE_TEST_SUITE_P(
    Sizes, Mono8ThresholdTest,
    ::testing::Values(
        // The robot's cameras: 200 x 163 blocks, so the last column of thread
        // blocks is 20 wide and the last row is 9 high.
        std::make_pair(1600, 1304),
        // 182 x 136 blocks: 2 columns and 10 rows left over.
        std::make_pair(1456, 1088),
        // One block.
        std::make_pair(8, 8),
        // 11 x 9 blocks, all in one thread block.
        std::make_pair(88, 72),
        // One short of 30 x 14 blocks, exactly that, and one past.
        std::make_pair(232, 104), std::make_pair(240, 112),
        std::make_pair(248, 120),
        // One block wide or high.
        std::make_pair(8, 1304), std::make_pair(1600, 8)),
    [](const ::testing::TestParamInfo<std::pair<size_t, size_t>> &info) {
      return std::to_string(info.param.first) + "x" +
             std::to_string(info.param.second);
    });

}  // namespace
}  // namespace frc::apriltag::testing
