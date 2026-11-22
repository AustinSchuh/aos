#include "frc/vision/camera/camera_status.h"

#include <linux/videodev2.h>
#include <sys/sysmacros.h>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <system_error>

#include "absl/log/absl_log.h"
#include "absl/strings/ascii.h"
#include "absl/strings/match.h"
#include "absl/strings/str_cat.h"

#include "aos/util/file.h"

namespace frc::vision {

CameraModel IdentifyCamera(uint16_t vendor_id, uint16_t product_id,
                           std::string_view name) {
  if (vendor_id == 0x1bcf && product_id == 0x28c5) {
    return CameraModel::THRIFTY_OV9281;
  }
  if (vendor_id == 0x0c45 && product_id == 0x636d &&
      absl::StrContains(name, "USB Camera")) {
    return CameraModel::INNOMAKER_OV9281;
  }
  if (vendor_id == 0x0c45 && product_id == 0x6366) {
    // PhotonVision matches the end of cscore's name; the card here carries
    // more after it ("Arducam OV9281 USB Camera"), so look anywhere in it.
    if (absl::StrContains(name, "OV9281")) {
      return CameraModel::ARDUCAM_OV9281;
    }
    if (absl::StrContains(name, "OV2311")) {
      return CameraModel::ARDUCAM_OV2311;
    }
    if (absl::StrContains(name, "OV9782")) {
      return CameraModel::ARDUCAM_OV9782;
    }
    return CameraModel::ARDUCAM_GENERIC;
  }
  return CameraModel::UNKNOWN;
}

namespace {

// A sysfs attribute, without its trailing newline, or "" if there isn't one.
std::string ReadAttribute(const std::filesystem::path &path) {
  std::optional<std::string> contents =
      aos::util::MaybeReadFileToString(path.string());
  if (!contents.has_value()) {
    return "";
  }
  return std::string(absl::StripAsciiWhitespace(*contents));
}

uint16_t ReadHexAttribute(const std::filesystem::path &path) {
  return static_cast<uint16_t>(
      std::strtoul(ReadAttribute(path).c_str(), nullptr, 16));
}

}  // namespace

std::optional<UsbDevice> ReadUsbDevice(std::string_view sysfs, dev_t device) {
  std::error_code error;
  const std::filesystem::path root =
      std::filesystem::canonical(std::filesystem::path(sysfs), error);
  if (error) {
    return std::nullopt;
  }
  // This links to the video4linux device, under the USB interface that is
  // under the USB device that has the IDs.
  std::filesystem::path path = std::filesystem::canonical(
      root / "dev" / "char" / absl::StrCat(major(device), ":", minor(device)),
      error);
  if (error) {
    return std::nullopt;
  }
  for (; path.string().size() > root.string().size();
       path = path.parent_path()) {
    if (!std::filesystem::exists(path / "idVendor", error)) {
      continue;
    }
    return UsbDevice{
        .vendor_id = ReadHexAttribute(path / "idVendor"),
        .product_id = ReadHexAttribute(path / "idProduct"),
        .device_version = ReadHexAttribute(path / "bcdDevice"),
        .manufacturer = ReadAttribute(path / "manufacturer"),
        .product = ReadAttribute(path / "product"),
        .serial = ReadAttribute(path / "serial"),
    };
  }
  return std::nullopt;
}

CameraStatusSender::CameraStatusSender(aos::EventLoop *event_loop,
                                       V4L2ReaderBase *reader,
                                       std::string_view device,
                                       std::string_view channel)
    : event_loop_(event_loop),
      reader_(reader),
      device_(device),
      usb_(ReadUsbDevice("/sys", reader->device_number())),
      model_(
          usb_.has_value()
              ? IdentifyCamera(usb_->vendor_id, usb_->product_id,
                               absl::StrCat(reader->card(), " ", usb_->product))
              : CameraModel::UNKNOWN),
      sender_(event_loop->MakeSender<CameraStatus>(channel)) {
  if (usb_.has_value()) {
    ABSL_LOG(INFO) << "  USB " << absl::Hex(usb_->vendor_id, absl::kZeroPad4)
                   << ":" << absl::Hex(usb_->product_id, absl::kZeroPad4)
                   << " \"" << usb_->manufacturer << "\" \"" << usb_->product
                   << "\" version "
                   << absl::Hex(usb_->device_version, absl::kZeroPad4)
                   << ", so " << EnumNameCameraModel(model_);
  }
  aos::TimerHandler *const timer = event_loop_->AddTimer([this]() { Send(); });
  timer->set_name("camera_status");
  event_loop_->OnRun([this, timer]() {
    last_send_time_ = event_loop_->monotonic_now();
    timer->Schedule(last_send_time_, std::chrono::seconds(1));
  });
}

void CameraStatusSender::Send() {
  aos::Sender<CameraStatus>::Builder builder = sender_.MakeBuilder();
  flatbuffers::FlatBufferBuilder *const fbb = builder.fbb();

  const auto string = [fbb](std::string_view value) {
    return value.empty() ? flatbuffers::Offset<flatbuffers::String>()
                         : fbb->CreateString(value);
  };
  const auto control = [this, fbb](uint32_t id) {
    const std::optional<V4L2ReaderBase::Control> value =
        reader_->GetControl(id);
    if (!value.has_value()) {
      return flatbuffers::Offset<CameraControl>();
    }
    CameraControl::Builder control_builder(*fbb);
    control_builder.add_value(value->value);
    control_builder.add_minimum(value->minimum);
    control_builder.add_maximum(value->maximum);
    control_builder.add_step(value->step);
    control_builder.add_default_value(value->default_value);
    control_builder.add_inactive(value->inactive);
    return control_builder.Finish();
  };

  const auto device_offset = string(device_);
  const auto driver_offset = string(reader_->driver());
  const auto card_offset = string(reader_->card());
  const auto bus_info_offset = string(reader_->bus_info());
  const auto manufacturer_offset = string(usb_ ? usb_->manufacturer : "");
  const auto product_offset = string(usb_ ? usb_->product : "");
  const auto serial_offset = string(usb_ ? usb_->serial : "");
  flatbuffers::Offset<TimePerFrame> frame_period_offset;
  if (const auto period = reader_->FramePeriod(); period.has_value()) {
    frame_period_offset =
        CreateTimePerFrame(*fbb, period->first, period->second);
  }
  const auto auto_exposure_offset = control(V4L2_CID_EXPOSURE_AUTO);
  const auto exposure_offset = control(V4L2_CID_EXPOSURE_ABSOLUTE);
  const auto gain_offset = control(V4L2_CID_GAIN);
  const auto dynamic_framerate_offset =
      control(V4L2_CID_EXPOSURE_AUTO_PRIORITY);
  const auto power_line_offset = control(V4L2_CID_POWER_LINE_FREQUENCY);

  const aos::monotonic_clock::time_point now = event_loop_->monotonic_now();
  const uint64_t images_sent = reader_->images_sent();

  CameraStatus::Builder status_builder = builder.MakeBuilder<CameraStatus>();
  status_builder.add_device(device_offset);
  status_builder.add_driver(driver_offset);
  status_builder.add_card(card_offset);
  status_builder.add_bus_info(bus_info_offset);
  if (usb_.has_value()) {
    status_builder.add_usb_vendor_id(usb_->vendor_id);
    status_builder.add_usb_product_id(usb_->product_id);
    status_builder.add_usb_device_version(usb_->device_version);
  }
  status_builder.add_usb_manufacturer(manufacturer_offset);
  status_builder.add_usb_product(product_offset);
  status_builder.add_usb_serial(serial_offset);
  status_builder.add_model(model_);
  status_builder.add_image_width(reader_->cols());
  status_builder.add_image_height(reader_->rows());
  status_builder.add_frame_period(frame_period_offset);
  status_builder.add_auto_exposure(auto_exposure_offset);
  status_builder.add_exposure_absolute(exposure_offset);
  status_builder.add_gain(gain_offset);
  status_builder.add_exposure_dynamic_framerate(dynamic_framerate_offset);
  status_builder.add_power_line_frequency(power_line_offset);
  status_builder.add_frames(images_sent - last_images_sent_);
  status_builder.add_period_ns(
      std::chrono::nanoseconds(now - last_send_time_).count());
  builder.CheckOk(builder.Send(status_builder.Finish()));

  last_images_sent_ = images_sent;
  last_send_time_ = now;
}

}  // namespace frc::vision
