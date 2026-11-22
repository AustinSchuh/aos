#ifndef FRC_VISION_CAMERA_CAMERA_STATUS_H_
#define FRC_VISION_CAMERA_CAMERA_STATUS_H_

#include <sys/types.h>

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

#include "aos/events/event_loop.h"
#include "frc/vision/camera/v4l2_reader.h"
#include "frc/vision/camera_status_generated.h"

namespace frc::vision {

// Which camera has these USB IDs and name (the card VIDIOC_QUERYCAP gives, or
// the USB product string), the way PhotonVision's QuirkyCamera tells.  Every
// Arducam has the same IDs, so only a name with its sensor in it says which.
CameraModel IdentifyCamera(uint16_t vendor_id, uint16_t product_id,
                           std::string_view name);

// The USB device a camera is, from sysfs.
struct UsbDevice {
  uint16_t vendor_id = 0;
  uint16_t product_id = 0;
  uint16_t device_version = 0;
  std::string manufacturer;
  std::string product;
  std::string serial;
};

// The USB device that character device `device` belongs to, or nullopt if it
// isn't one.  `sysfs` is where sysfs is mounted, "/sys" outside of tests.
std::optional<UsbDevice> ReadUsbDevice(std::string_view sysfs, dev_t device);

// Sends a CameraStatus for `reader` on `channel` when the event loop starts,
// and once a second after.
class CameraStatusSender {
 public:
  CameraStatusSender(aos::EventLoop *event_loop, V4L2ReaderBase *reader,
                     std::string_view device, std::string_view channel);

 private:
  void Send();

  aos::EventLoop *const event_loop_;
  V4L2ReaderBase *const reader_;
  const std::string device_;
  // Which device it is doesn't change while it's open.
  const std::optional<UsbDevice> usb_;
  const CameraModel model_;
  aos::Sender<CameraStatus> sender_;

  uint64_t last_images_sent_ = 0;
  aos::monotonic_clock::time_point last_send_time_;
};

}  // namespace frc::vision

#endif  // FRC_VISION_CAMERA_CAMERA_STATUS_H_
