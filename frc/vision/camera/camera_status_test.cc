#include "frc/vision/camera/camera_status.h"

#include <sys/sysmacros.h>

#include <filesystem>

#include "gtest/gtest.h"

#include "aos/testing/tmpdir.h"
#include "aos/util/file.h"

namespace frc::vision::testing {
namespace {

TEST(IdentifyCameraTest, ByUsbIds) {
  EXPECT_EQ(IdentifyCamera(0x1bcf, 0x28c5, "USB Camera"),
            CameraModel::THRIFTY_OV9281);
  EXPECT_EQ(IdentifyCamera(0x0c45, 0x636d, "USB Camera: USB Camera"),
            CameraModel::INNOMAKER_OV9281);
  EXPECT_EQ(IdentifyCamera(0x0c45, 0x636d, "Something else"),
            CameraModel::UNKNOWN);
  EXPECT_EQ(IdentifyCamera(0x046d, 0x0825, "C270 HD WEBCAM"),
            CameraModel::UNKNOWN);
}

// Every Arducam has the same IDs, so only its name says which sensor it has.
TEST(IdentifyCameraTest, ArducamsByName) {
  EXPECT_EQ(IdentifyCamera(0x0c45, 0x6366, "Arducam OV9281 USB Camera"),
            CameraModel::ARDUCAM_OV9281);
  EXPECT_EQ(IdentifyCamera(0x0c45, 0x6366, "Arducam OV2311 USB Camera"),
            CameraModel::ARDUCAM_OV2311);
  EXPECT_EQ(IdentifyCamera(0x0c45, 0x6366, "Arducam OV9782 USB Camera"),
            CameraModel::ARDUCAM_OV9782);
  EXPECT_EQ(IdentifyCamera(0x0c45, 0x6366, "Arducam USB Camera"),
            CameraModel::ARDUCAM_GENERIC);
}

class ReadUsbDeviceTest : public ::testing::Test {
 protected:
  ReadUsbDeviceTest() : sysfs_(aos::testing::TestTmpDir() + "/sys") {
    std::filesystem::remove_all(sysfs_);
  }

  void Write(const std::filesystem::path &path, std::string_view contents) {
    std::filesystem::create_directories(path.parent_path());
    aos::util::WriteStringToFileOrDie(path.string(), contents);
  }

  const std::filesystem::path sysfs_;
};

// Laid out the way the kernel does it: /sys/dev/char/81:0 links to the
// video4linux device, under the USB interface, under the USB device.
TEST_F(ReadUsbDeviceTest, FindsTheUsbDeviceAbove) {
  const std::filesystem::path usb =
      sysfs_ / "devices/platform/3610000.usb/usb1/1-2/1-2.1";
  Write(usb / "idVendor", "1bcf\n");
  Write(usb / "idProduct", "28c5\n");
  Write(usb / "bcdDevice", "0409\n");
  Write(usb / "manufacturer", "Thrifty\n");
  Write(usb / "product", "USB Camera\n");
  Write(usb / "serial", "SN0001\n");
  const std::filesystem::path video = usb / "1-2.1:1.0/video4linux/video0";
  std::filesystem::create_directories(video);
  std::filesystem::create_directories(sysfs_ / "dev/char");
  std::filesystem::create_directory_symlink(
      "../../devices/platform/3610000.usb/usb1/1-2/1-2.1/1-2.1:1.0/"
      "video4linux/video0",
      sysfs_ / "dev/char/81:0");

  const std::optional<UsbDevice> device =
      ReadUsbDevice(sysfs_.string(), makedev(81, 0));
  ASSERT_TRUE(device.has_value());
  EXPECT_EQ(device->vendor_id, 0x1bcf);
  EXPECT_EQ(device->product_id, 0x28c5);
  EXPECT_EQ(device->device_version, 0x0409);
  EXPECT_EQ(device->manufacturer, "Thrifty");
  EXPECT_EQ(device->product, "USB Camera");
  EXPECT_EQ(device->serial, "SN0001");
}

// A camera that isn't on USB, like a MIPI one, has nothing to find.
TEST_F(ReadUsbDeviceTest, NotUsb) {
  std::filesystem::create_directories(sysfs_ /
                                      "devices/platform/vi/video4linux/video1");
  std::filesystem::create_directories(sysfs_ / "dev/char");
  std::filesystem::create_directory_symlink(
      "../../devices/platform/vi/video4linux/video1", sysfs_ / "dev/char/81:1");
  EXPECT_FALSE(ReadUsbDevice(sysfs_.string(), makedev(81, 1)).has_value());
  // Nor does a device sysfs doesn't have.
  EXPECT_FALSE(ReadUsbDevice(sysfs_.string(), makedev(81, 2)).has_value());
}

}  // namespace
}  // namespace frc::vision::testing
