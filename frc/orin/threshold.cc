#include "frc/orin/threshold.h"

#include <stdint.h>

#include "absl/log/absl_check.h"

#include "frc/orin/cuda.h"

namespace frc::apriltag {
namespace {

// 1280 -> 2 * 128 * 5
// 720 -> 2 * 8 * 5 * 9
//
// 1456 -> 2 * 8 * 7 * 13
// 1088 -> 2 * 32 * 17
//
// 1600 -> 64 * 25
// 1304 -> 8 * 163
template <vision::ImageFormat IMAGE_FORMAT>
__device__ __forceinline__ uint8_t ToGray(const uint8_t *color_image) {
  if constexpr (IMAGE_FORMAT == vision::ImageFormat::MONO8) {
    return color_image[0];  // Grayscale input is already aliased to
                            // color device image
  } else if constexpr (IMAGE_FORMAT == vision::ImageFormat::MONO16) {
    return color_image[1];  // MSBits of MONO16 - does this also work
                            // on jetson or do we need a be/le split?
  } else if constexpr (IMAGE_FORMAT == vision::ImageFormat::YUYV422) {
    return color_image[0];  // YUY input
  } else if constexpr (IMAGE_FORMAT == vision::ImageFormat::BGR8) {
    return 0.114f * static_cast<float>(color_image[0]) +
           0.587f * static_cast<float>(color_image[1]) +
           0.299f * static_cast<float>(color_image[2]);  // BGR input
  } else if constexpr (IMAGE_FORMAT == vision::ImageFormat::BGRA8) {
    return 0.114f * static_cast<float>(color_image[0]) +
           0.587f * static_cast<float>(color_image[1]) +
           0.299f * static_cast<float>(
                        color_image[2]);  // BGRA input, skip alpha channel
  }
}

// Convert from input format to grayscale.
template <vision::ImageFormat IMAGE_FORMAT>
__global__ void InternalCudaToGreyscale(const uint8_t *color_image,
                                        uint8_t *gray_image,
                                        apriltag_size_t width,
                                        apriltag_size_t height) {
  constexpr apriltag_size_t kBytesPerPixel = BytesPerPixel(IMAGE_FORMAT);
  apriltag_size_t i = blockIdx.x * blockDim.x + threadIdx.x;
  while (i < width * height) {
    gray_image[i] = ToGray<IMAGE_FORMAT>(color_image + i * kBytesPerPixel);

    i += blockDim.x * gridDim.x;
  }
}

// Writes out the grayscale image and decimated image.
template <vision::ImageFormat IMAGE_FORMAT>
__global__ void InternalCudaToGreyscaleAndDecimate(
    const uint8_t *color_image, uint8_t *decimated_image,
    const apriltag_size_t in_width, const apriltag_size_t in_height) {
  constexpr apriltag_size_t kBytesPerPixel = BytesPerPixel(IMAGE_FORMAT);
  const apriltag_size_t out_height = in_height / 2;
  const apriltag_size_t out_width = in_width / 2;
  apriltag_size_t out_i = blockIdx.x * blockDim.x + threadIdx.x;

  while (out_i < out_width * out_height) {
    const apriltag_size_t out_row = out_i / out_width;
    const apriltag_size_t out_col = out_i - out_width * out_row;

    const u_int32_t in_row = out_row * 2;
    const u_int32_t in_col = out_col * 2;

    const apriltag_size_t in_i = in_row * in_width + in_col;

    decimated_image[out_row * out_width + out_col] =
        ToGray<IMAGE_FORMAT>(color_image + in_i * kBytesPerPixel);
    out_i += blockDim.x * gridDim.x;
  }
  // TODO(austin): Figure out how to load contiguous memory reasonably
  // efficiently and max/min over it.

  // TODO(austin): Can we do the threshold here too?  That would be less memory
  // bandwidth consumed...
  //
  //   could do it by merging this code with InternalBlockMinMax, altering
  //   the input indexing so it grabs from the undecimated input image.  Add
  //   the grayscale converion code in there as well?
}

// Returns the min and max for a row of 4 pixels.
__forceinline__ __device__ uchar2 minmax(uchar4 row) {
  uint8_t min_val = std::min(std::min(row.x, row.y), std::min(row.z, row.w));
  uint8_t max_val = std::max(std::max(row.x, row.y), std::max(row.z, row.w));
  return make_uchar2(min_val, max_val);
}

// Returns the min and max for a set of min and maxes.
__forceinline__ __device__ uchar2 minmax(uchar2 val0, uchar2 val1) {
  return make_uchar2(std::min(val0.x, val1.x), std::max(val0.y, val1.y));
}

// Returns the pixel index of a pixel at the provided x and y location.
__forceinline__ __device__ apriltag_size_t XYToIndex(apriltag_size_t width,
                                                     apriltag_size_t x,
                                                     apriltag_size_t y) {
  return width * y + x;
}

// Computes the min and max pixel value for each block of 4 pixels.
__global__ void InternalBlockMinMax(const uint8_t *decimated_image,
                                    uchar2 *unfiltered_minmax_image,
                                    const apriltag_size_t width,
                                    const apriltag_size_t height) {
  uchar2 vals[4];
  const apriltag_size_t x = blockIdx.x * blockDim.x + threadIdx.x;
  const apriltag_size_t y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x >= width || y >= height) {
    return;
  }

  for (int i = 0; i < 4; ++i) {
    const uchar4 decimated_block = *reinterpret_cast<const uchar4 *>(
        decimated_image + XYToIndex(width * 4, x * 4, y * 4 + i));

    vals[i] = minmax(decimated_block);
  }

  unfiltered_minmax_image[XYToIndex(width, x, y)] =
      minmax(minmax(vals[0], vals[1]), minmax(vals[2], vals[3]));
}

// Filters the min/max for the surrounding block of 9 pixels centered on our
// location using min/max and writes the result back out.
__global__ void InternalBlockFilter(const uchar2 *unfiltered_minmax_image,
                                    uchar2 *minmax_image,
                                    const apriltag_size_t width,
                                    const apriltag_size_t height) {
  uchar2 result = make_uchar2(255, 0);

  const apriltag_size_t x = blockIdx.x * blockDim.x + threadIdx.x;
  const apriltag_size_t y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x >= width || y >= height) {
    return;
  }

  // Iterate through the 3x3 set of points centered on the point this image is
  // responsible for, and compute the overall min/max.
#pragma unroll
  for (int32_t i = -1; i <= 1; ++i) {
#pragma unroll
    for (int32_t j = -1; j <= 1; ++j) {
      const int32_t read_x = x + i;
      const int32_t read_y = y + j;

      if (read_x < 0 || read_x >= static_cast<int32_t>(width)) {
        continue;
      }
      if (read_y < 0 || read_y >= static_cast<int32_t>(height)) {
        continue;
      }

      result = minmax(
          result, unfiltered_minmax_image[XYToIndex(width, read_x, read_y)]);
    }
  }

  minmax_image[XYToIndex(width, x, y)] = result;
}

// Thresholds the image based on the filtered thresholds.
__global__ void InternalThreshold(const uint8_t *decimated_image,
                                  const uchar2 *minmax_image,
                                  uint8_t *thresholded_image,
                                  const apriltag_size_t width,
                                  const apriltag_size_t height,
                                  const apriltag_size_t min_white_black_diff) {
  apriltag_size_t i = blockIdx.x * blockDim.x + threadIdx.x;
  while (i < width * height) {
    const apriltag_size_t x = i % width;
    const apriltag_size_t y = i / width;

    const uchar2 minmax_val = minmax_image[x / 4 + (y / 4) * width / 4];

    uint8_t result;
    if (minmax_val.y - minmax_val.x < min_white_black_diff) {
      result = 127;
    } else {
      uint8_t thresh = minmax_val.x + (minmax_val.y - minmax_val.x) / 2;
      if (decimated_image[i] > thresh) {
        result = 255;
      } else {
        result = 0;
      }
    }

    thresholded_image[i] = result;
    i += blockDim.x * gridDim.x;
  }
}

// The thread block shape of InternalMono8DecimateThreshold, in 4x4 decimated
// blocks.  The outer ring of blocks only feeds the 3x3 filter of the inner
// ones, so taller blocks load fewer rows twice; on the Orin 16 rows beat 8 and
// 32.
constexpr int kFusedBlockCols = 32;
constexpr int kFusedBlockRows = 16;

// Decimates and thresholds a MONO8 image in one pass, exactly as
// InternalCudaToGreyscaleAndDecimate, InternalBlockMinMax, InternalBlockFilter
// and InternalThreshold do in four.  Each thread owns one 4x4 block of the
// decimated image.  Thread blocks overlap by one 4x4 block on each side, so
// the 3x3 min/max filter of the inner blocks only reads shared memory.
__global__ void __launch_bounds__(kFusedBlockCols *kFusedBlockRows)
    InternalMono8DecimateThreshold(const uint8_t *image,
                                   const apriltag_size_t in_width,
                                   uint8_t *decimated_image,
                                   uint8_t *thresholded_image,
                                   const apriltag_size_t block_cols,
                                   const apriltag_size_t block_rows,
                                   const apriltag_size_t min_white_black_diff) {
  __shared__ uchar2 block_minmax[kFusedBlockRows][kFusedBlockCols];

  const int block_x =
      static_cast<int>(blockIdx.x * (kFusedBlockCols - 2) + threadIdx.x) - 1;
  const int block_y =
      static_cast<int>(blockIdx.y * (kFusedBlockRows - 2) + threadIdx.y) - 1;
  const bool in_image = block_x >= 0 && block_y >= 0 &&
                        block_x < static_cast<int>(block_cols) &&
                        block_y < static_cast<int>(block_rows);

  // Each word holds one row of the block's 4 decimated pixels.
  uint32_t decimated[4];
  // Blocks off the image don't change the filter, which skips them.
  uchar2 minmax = make_uchar2(255, 0);
  if (in_image) {
#pragma unroll
    for (int i = 0; i < 4; ++i) {
      // The 8 raw pixels under a decimated row, loaded 4 bytes at a time
      // because the image is only 4 byte aligned.
      const uint32_t *raw = reinterpret_cast<const uint32_t *>(
          image + (2 * (4 * block_y + i)) * in_width + 8 * block_x);
      // Keep the even pixels: bytes 0 and 2 of each word.
      decimated[i] = __byte_perm(raw[0], raw[1], 0x6420);
    }
    uint32_t min_word = __vminu4(__vminu4(decimated[0], decimated[1]),
                                 __vminu4(decimated[2], decimated[3]));
    min_word = __vminu4(min_word, min_word >> 16);
    min_word = __vminu4(min_word, min_word >> 8);
    uint32_t max_word = __vmaxu4(__vmaxu4(decimated[0], decimated[1]),
                                 __vmaxu4(decimated[2], decimated[3]));
    max_word = __vmaxu4(max_word, max_word >> 16);
    max_word = __vmaxu4(max_word, max_word >> 8);
    minmax = make_uchar2(min_word & 0xff, max_word & 0xff);
  }
  block_minmax[threadIdx.y][threadIdx.x] = minmax;
  __syncthreads();

  if (!in_image || threadIdx.x == 0 || threadIdx.x == kFusedBlockCols - 1 ||
      threadIdx.y == 0 || threadIdx.y == kFusedBlockRows - 1) {
    return;
  }

  uint32_t min_val = 255;
  uint32_t max_val = 0;
#pragma unroll
  for (int dy = -1; dy <= 1; ++dy) {
#pragma unroll
    for (int dx = -1; dx <= 1; ++dx) {
      const uchar2 neighbor = block_minmax[threadIdx.y + dy][threadIdx.x + dx];
      min_val = min(min_val, static_cast<uint32_t>(neighbor.x));
      max_val = max(max_val, static_cast<uint32_t>(neighbor.y));
    }
  }

  const bool flat = max_val - min_val < min_white_black_diff;
  const uint8_t thresh = min_val + (max_val - min_val) / 2;
  const apriltag_size_t out_width = block_cols * 4;
#pragma unroll
  for (int i = 0; i < 4; ++i) {
    const apriltag_size_t offset = (4 * block_y + i) * out_width + 4 * block_x;
    *reinterpret_cast<uint32_t *>(decimated_image + offset) = decimated[i];
    // __vcmpgtu4 sets each byte greater than thresh to 0xff and the rest to 0.
    *reinterpret_cast<uint32_t *>(thresholded_image + offset) =
        flat ? 0x7f7f7f7fu : __vcmpgtu4(decimated[i], thresh * 0x01010101u);
  }
}

template <vision::ImageFormat IMAGE_FORMAT>
class TypedThreshold : public Threshold {
 public:
  TypedThreshold(size_t width, size_t height)
      : width_(width),
        height_(height),
        unfiltered_minmax_image_device_(MinMaxImageBytes(width, height)),
        minmax_image_device_(MinMaxImageBytes(width, height)) {}
  // Create a full-size grayscale image from a color image on the provided
  // stream.
  void ToGreyscale(const uint8_t *color_image, uint8_t *gray_image,
                   CudaStream *stream) override;

  // Converts to grayscale, decimates, and thresholds an image on the provided
  // stream.
  void ThresholdAndDecimate(const uint8_t *color_image,
                            uint8_t *decimated_image,
                            uint8_t *thresholded_image,
                            apriltag_size_t min_white_black_diff,
                            CudaStream *stream) override;

  virtual ~TypedThreshold() = default;

 private:
  static size_t MinMaxImageBytes(size_t width, size_t height) {
    // MONO8 keeps the block min/max images in shared memory.
    if constexpr (IMAGE_FORMAT == vision::ImageFormat::MONO8) {
      return 0;
    }
    return (width / 2 / 4 * height / 2 / 4) * 2;
  }

  size_t width_;
  size_t height_;

  GpuMemory<uint8_t> unfiltered_minmax_image_device_;
  GpuMemory<uint8_t> minmax_image_device_;
};

template <vision::ImageFormat IMAGE_FORMAT>
void TypedThreshold<IMAGE_FORMAT>::ToGreyscale(const uint8_t *color_image,
                                               uint8_t *gray_image,
                                               CudaStream *stream) {
  constexpr size_t kThreads = 256;
  {
    // Step one, convert to gray and decimate.
    size_t kBlocks = (width_ * height_ + kThreads - 1) / kThreads / 4;
    InternalCudaToGreyscale<IMAGE_FORMAT>
        <<<kBlocks, kThreads, 0, stream->get()>>>(color_image, gray_image,
                                                  width_, height_);
    MaybeCheckAndSynchronize();
  }
}

template <vision::ImageFormat IMAGE_FORMAT>
void TypedThreshold<IMAGE_FORMAT>::ThresholdAndDecimate(
    const uint8_t *color_image, uint8_t *decimated_image,
    uint8_t *thresholded_image, apriltag_size_t min_white_black_diff,
    CudaStream *stream) {
  uint8_t *unfiltered_minmax_image = unfiltered_minmax_image_device_.get();
  uint8_t *minmax_image = minmax_image_device_.get();

  ABSL_CHECK((width_ % 8) == 0);
  ABSL_CHECK((height_ % 8) == 0);
  constexpr size_t kThreads = 256;
  const apriltag_size_t decimated_width = width_ / 2;
  const apriltag_size_t decimated_height = height_ / 2;

  {
    // Step one, convert to gray and decimate.
    const size_t kBlocks =
        (decimated_width * decimated_height + kThreads - 1) / kThreads / 4;
    InternalCudaToGreyscaleAndDecimate<IMAGE_FORMAT>
        <<<kBlocks, kThreads, 0, stream->get()>>>(color_image, decimated_image,
                                                  width_, height_);
    MaybeCheckAndSynchronize();
  }

  {
    // Step 2, compute a min/max for each block of 4x4 (16) pixels.
    const dim3 threads(16, 16, 1);
    const dim3 blocks((decimated_width / 4 + 15) / 16,
                      (decimated_height / 4 + 15) / 16, 1);

    InternalBlockMinMax<<<blocks, threads, 0, stream->get()>>>(
        decimated_image, reinterpret_cast<uchar2 *>(unfiltered_minmax_image),
        decimated_width / 4, decimated_height / 4);
    MaybeCheckAndSynchronize();

    // Step 3, Blur those min/max's a further +- 1 block in each direction using
    // min/max.
    InternalBlockFilter<<<blocks, threads, 0, stream->get()>>>(
        reinterpret_cast<uchar2 *>(unfiltered_minmax_image),
        reinterpret_cast<uchar2 *>(minmax_image), decimated_width / 4,
        decimated_height / 4);
    MaybeCheckAndSynchronize();
  }

  {
    // Now, write out 127 if the min/max are too close to each other, or 0/255
    // if the pixels are above or below the average of the min/max.
    const apriltag_size_t kBlocks =
        (width_ * height_ / 4 + kThreads - 1) / kThreads / 4;
    InternalThreshold<<<kBlocks, kThreads, 0, stream->get()>>>(
        decimated_image, reinterpret_cast<uchar2 *>(minmax_image),
        thresholded_image, decimated_width, decimated_height,
        min_white_black_diff);
    MaybeCheckAndSynchronize();
  }
}

// A MONO8 image is already gray, so it decimates and thresholds in one kernel.
template <>
void TypedThreshold<vision::ImageFormat::MONO8>::ThresholdAndDecimate(
    const uint8_t *color_image, uint8_t *decimated_image,
    uint8_t *thresholded_image, apriltag_size_t min_white_black_diff,
    CudaStream *stream) {
  ABSL_CHECK((width_ % 8) == 0);
  ABSL_CHECK((height_ % 8) == 0);
  // CameraImage.data is a flatbuffer [ubyte] vector, which flatbuffers only
  // aligns to 4 bytes.
  ABSL_CHECK_EQ(reinterpret_cast<uintptr_t>(color_image) % 4, 0u)
      << ": the MONO8 threshold loads the image 4 bytes at a time";

  // The decimated image in 4x4 blocks.
  const apriltag_size_t block_cols = width_ / 8;
  const apriltag_size_t block_rows = height_ / 8;
  const dim3 threads(kFusedBlockCols, kFusedBlockRows, 1);
  const dim3 blocks((block_cols + kFusedBlockCols - 3) / (kFusedBlockCols - 2),
                    (block_rows + kFusedBlockRows - 3) / (kFusedBlockRows - 2),
                    1);
  InternalMono8DecimateThreshold<<<blocks, threads, 0, stream->get()>>>(
      color_image, width_, decimated_image, thresholded_image, block_cols,
      block_rows, min_white_black_diff);
  MaybeCheckAndSynchronize();
}

}  // namespace

std::unique_ptr<Threshold> MakeThreshold(vision::ImageFormat image_format,
                                         size_t width, size_t height) {
  switch (image_format) {
    case vision::ImageFormat::MONO8:
      return std::make_unique<TypedThreshold<vision::ImageFormat::MONO8>>(
          width, height);
    case vision::ImageFormat::MONO16:
      return std::make_unique<TypedThreshold<vision::ImageFormat::MONO16>>(
          width, height);
    case vision::ImageFormat::YUYV422:
      return std::make_unique<TypedThreshold<vision::ImageFormat::YUYV422>>(
          width, height);
    case vision::ImageFormat::BGR8:
      return std::make_unique<TypedThreshold<vision::ImageFormat::BGR8>>(
          width, height);
    case vision::ImageFormat::BGRA8:
      return std::make_unique<TypedThreshold<vision::ImageFormat::BGRA8>>(
          width, height);
    default:
      ABSL_LOG(FATAL) << "Unknown image format: "
                      << vision::EnumNameImageFormat(image_format);
  }
}

}  // namespace frc::apriltag
