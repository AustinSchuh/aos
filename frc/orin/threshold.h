#ifndef FRC_ORIN_THRESHOLD_H_
#define FRC_ORIN_THRESHOLD_H_

#include <stdint.h>

#include "frc/orin/apriltag_types.h"
#include "frc/orin/cuda.h"
#include "frc/vision/vision_generated.h"

namespace frc::apriltag {

// Returns the number of bytes per pixel.
constexpr __host__ __device__ apriltag_size_t
BytesPerPixel(const vision::ImageFormat image_format) {
  switch (image_format) {
    case vision::ImageFormat::MONO8:
      return 1;
    case vision::ImageFormat::MONO16:
      return 2;
    case vision::ImageFormat::YUYV422:
      return 2;
    case vision::ImageFormat::BGR8:
      return 3;
    case vision::ImageFormat::BGRA8:
      return 4;
    case vision::ImageFormat::MJPEG:
      __builtin_trap();
  }
  __builtin_trap();
  return 0;
}

class Threshold {
 public:
  // Create a full-size grayscale image from a color image on the provided
  // stream.
  virtual void ToGreyscale(const uint8_t *color_image, uint8_t *gray_image,
                           CudaStream *stream) = 0;

  // Converts to grayscale, decimates, and thresholds an image on the provided
  // stream.
  virtual void ThresholdAndDecimate(const uint8_t *color_image,
                                    uint8_t *decimated_image,
                                    uint8_t *thresholded_image,
                                    apriltag_size_t min_white_black_diff,
                                    CudaStream *stream) = 0;

  // Like ThresholdAndDecimate, but the GPU reads the image's address from
  // *color_image when the work runs instead of when it is enqueued, so a CUDA
  // graph which captures this thresholds whichever image *color_image points
  // to at each launch.  color_image must be readable by the device, and the
  // image 4 byte aligned.  Only the GPU MONO8 threshold supports this.
  virtual void ThresholdAndDecimateIndirect(
      const uint8_t *const *color_image, uint8_t *decimated_image,
      uint8_t *thresholded_image, apriltag_size_t min_white_black_diff,
      CudaStream *stream);

  virtual ~Threshold() = default;
};

// Constructs an optimized threshold object for the provided image format.
std::unique_ptr<Threshold> MakeThreshold(vision::ImageFormat image_format,
                                         size_t width, size_t height);

// Constructs a NEON optimized threshold object for the provided image format.
std::unique_ptr<Threshold> MakeNeonThreshold(vision::ImageFormat image_format,
                                             size_t width, size_t height);

}  // namespace frc::apriltag

#endif  // FRC_ORIN_THRESHOLD_H_
