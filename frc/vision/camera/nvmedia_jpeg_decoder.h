#ifndef FRC_VISION_CAMERA_NVMEDIA_JPEG_DECODER_H_
#define FRC_VISION_CAMERA_NVMEDIA_JPEG_DECODER_H_

#include <cstddef>
#include <cstdint>
#include <memory>

#include "aos/util/status.h"

namespace frc::vision {

// Decodes JPEGs on one chosen NVJPG engine of a Jetson Orin through NVIDIA's
// NvMedia image JPEG decoder (IJPD, libnvmedia_ijpd_sci).
//
// The Orin has two NVJPG engines.  libnvmm_jpeg (the libjpeg API) always
// uses the first one, so four 120 Hz cameras saturate it while the second
// sits idle.  IJPD takes the engine as an argument, so each decoder process
// can be pinned to an engine and the load split between them.
//
// Each frame is decoded into an NvSciBuf YUV 4:2:0 surface, mapped cached for
// the CPU, and the Y plane is copied out (0.1 ms for 1280x800).  The CPU
// sleeps in the kernel while the engine works.  There is no CPU fallback:
// construction dies if the engine is unusable, and a frame the engine cannot
// decode entirely in hardware fails.
//
// Usage (per camera stream):
//   NvMediaJpegDecoder decoder(/*engine=*/1);
//   aos::Result<NvMediaJpegDecoder::ImageSize> size =
//       decoder.DecodeToGray(jpeg, jpeg_size, gray, gray_size);
//   if (size.has_value()) {
//     // gray holds size->width * size->height packed luma bytes.
//   }
class NvMediaJpegDecoder {
 public:
  struct ImageSize {
    uint32_t width = 0;
    uint32_t height = 0;
  };

  // |engine| is the NVJPG engine to decode on: 0 or 1.
  explicit NvMediaJpegDecoder(int engine);
  ~NvMediaJpegDecoder();

  NvMediaJpegDecoder(const NvMediaJpegDecoder &) = delete;
  NvMediaJpegDecoder &operator=(const NvMediaJpegDecoder &) = delete;

  // Decodes |jpeg_size| bytes at |jpeg_data| and copies the Y plane, packed,
  // into |gray_out|, which holds |max_out_size| bytes.  Returns the image's
  // size, or why the frame failed.
  //
  // Safe to call on a realtime thread.  It allocates, under
  // aos::ScopedNotRealtime, only where it can't avoid it: inside NVIDIA's
  // NvMediaIJPDRenderYUV on every frame, and setting the engine up for a new
  // image size (the first frame) or after a failed decode.  A failure's
  // message does not allocate.
  [[nodiscard]] aos::Result<ImageSize> DecodeToGray(const uint8_t *jpeg_data,
                                                    size_t jpeg_size,
                                                    uint8_t *gray_out,
                                                    size_t max_out_size);

  int engine() const { return engine_; }

 private:
  struct Modules;
  // The decoder, its output surface and its fence, for one image size.
  class Stream;

  const int engine_;
  std::unique_ptr<Modules> modules_;
  std::unique_ptr<Stream> stream_;
};

}  // namespace frc::vision

#endif  // FRC_VISION_CAMERA_NVMEDIA_JPEG_DECODER_H_
