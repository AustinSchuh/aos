#ifndef FRC_VISION_CAMERA_NVJPEG_DECODER_SERVICE_H_
#define FRC_VISION_CAMERA_NVJPEG_DECODER_SERVICE_H_

#include <cstdint>

#include "aos/containers/inlined_vector.h"
#include "aos/events/event_loop.h"
#include "frc/vision/camera/nvmedia_jpeg_decoder.h"
#include "frc/vision/turbojpeg_decoder_status_static.h"
#include "frc/vision/vision_generated.h"

namespace frc::vision {

// NVJPG Hardware JPEG Decoder for Jetson Orin.
//
// Drop-in alternative to turbojpeg_decoder that decodes on one of the Orin's
// two NVJPG hardware engines, chosen with --nvjpg_engine, through NvMedia's
// JPEG decoder (see nvmedia_jpeg_decoder.h).  Dies loudly at startup if the
// engine is unavailable -- no silent CPU fallback; CPU decode is
// turbojpeg_decoder's job.  Mirrors turbojpeg_decoder's interface -- same
// flags (plus --nvjpg_engine), same MONO8 output and TurboJpegDecoderStatus
// on the same /cameraX/gray channel -- so swapping decoders is an
// executable_name change in the config.
//
// One engine decodes about 450 1280x800 frames a second, so four 120 Hz
// cameras need both: split the cameras' decoders across the engines in the
// config.
//
// Each frame is decoded straight into the outgoing message's shared memory
// (the engine's luma plane copied out on the CPU, 0.1 ms at 1280x800), so a
// detector that registers the channel with CUDA reads it in place.
//
// The decoder does not check its own output, so the nvjpeg_decoder service
// adds no CPU decode load: nvmedia_jpeg_decoder_test checks it against
// turbojpeg on real camera frames, and nvjpeg_decoder_verify overrides
// CheckDecode to also decode every frame with turbojpeg and die if the
// hardware output does not match.
class NvjpegDecoder {
 public:
  explicit NvjpegDecoder(aos::EventLoop *event_loop);
  virtual ~NvjpegDecoder() = default;

 protected:
  // Called with every successfully decoded frame before it is published, on
  // the realtime path.  |gray| holds rows * cols pixels.
  virtual void CheckDecode(const CameraImage & /*image*/,
                           const uint8_t * /*gray*/, uint32_t /*rows*/,
                           uint32_t /*cols*/) {}

 private:
  void ProcessImage(const CameraImage &image);
  void SendStatus();

  aos::EventLoop *event_loop_;
  NvMediaJpegDecoder hw_decoder_;
  aos::Sender<CameraImage> camera_output_sender_;

  aos::Sender<TurboJpegDecoderStatusStatic> status_sender_;
  int successful_decodes_ = 0;
  int failed_decodes_ = 0;
  int skipped_stale_frames_ = 0;
  int dropped_sends_ = 0;
  int64_t max_publish_age_ms_ = 0;
  aos::InlinedVector<char, 128> last_error_message_;

  size_t skip_ = 0;
};

}  // namespace frc::vision

#endif  // FRC_VISION_CAMERA_NVJPEG_DECODER_SERVICE_H_
