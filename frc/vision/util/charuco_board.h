#ifndef FRC_VISION_UTIL_CHARUCO_BOARD_H_
#define FRC_VISION_UTIL_CHARUCO_BOARD_H_

#include <ostream>
#include <string>
#include <string_view>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "flatbuffers/flatbuffers.h"
#include "opencv2/aruco/charuco.hpp"

#include "frc/vision/calibration_generated.h"

namespace frc::vision {

// A ChArUco board, as data rather than flags: what a calibration is made with,
// and what the page lets people pick and edit.
struct BoardSpec {
  // The squares across and down.
  int squares_x = 14;
  int squares_y = 9;
  // The side of a square, and of the marker in a white square, in meters, as
  // measured on the printed board.
  double square_length = 0.04;
  double marker_length = 0.03;
  // The markers' ArUco dictionary, by OpenCV's name.
  std::string dictionary = "DICT_5X5_100";

  bool operator==(const BoardSpec &) const = default;
};

std::ostream &operator<<(std::ostream &os, const BoardSpec &spec);

// A board with a name, for the page's list.
struct NamedBoard {
  std::string name;
  BoardSpec spec;
};

// The boards the calibration tools have used, by the flags that chose them.
// The first is the default.
const std::vector<NamedBoard> &BoardPresets();

// The ArUco dictionaries a board can use, by OpenCV's names.
std::vector<std::string_view> DictionaryNames();

// The number of markers in `dictionary`, or an error if it isn't one.
absl::StatusOr<int> DictionarySize(std::string_view dictionary);

// Whether `spec` describes a board OpenCV can detect: at least 3 squares each
// way, a marker smaller than its square, and a dictionary with a marker for
// every white square.
absl::Status ValidateBoardSpec(const BoardSpec &spec);

// The OpenCV board for `spec`, which must be valid.
cv::Ptr<cv::aruco::CharucoBoard> MakeCharucoBoard(const BoardSpec &spec);

// Conversions to and from calibration.fbs's BoardSpec.
BoardSpec BoardSpecFromFlatbuffer(const calibration::BoardSpec &spec);
flatbuffers::Offset<calibration::BoardSpec> PackBoardSpec(
    flatbuffers::FlatBufferBuilder *fbb, const BoardSpec &spec);

}  // namespace frc::vision

#endif  // FRC_VISION_UTIL_CHARUCO_BOARD_H_
