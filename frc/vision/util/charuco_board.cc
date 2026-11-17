#include "frc/vision/util/charuco_board.h"

#include <cmath>
#include <utility>

#include "absl/log/absl_check.h"
#include "absl/strings/str_cat.h"

namespace frc::vision {
namespace {

struct Dictionary {
  std::string_view name;
  // OpenCV's predefined dictionary, as getPredefinedDictionary takes it.
  int id;
  int size;
};

constexpr Dictionary kDictionaries[] = {
    {"DICT_4X4_50", cv::aruco::DICT_4X4_50, 50},
    {"DICT_4X4_100", cv::aruco::DICT_4X4_100, 100},
    {"DICT_4X4_250", cv::aruco::DICT_4X4_250, 250},
    {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000, 1000},
    {"DICT_5X5_50", cv::aruco::DICT_5X5_50, 50},
    {"DICT_5X5_100", cv::aruco::DICT_5X5_100, 100},
    {"DICT_5X5_250", cv::aruco::DICT_5X5_250, 250},
    {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000, 1000},
    {"DICT_6X6_50", cv::aruco::DICT_6X6_50, 50},
    {"DICT_6X6_100", cv::aruco::DICT_6X6_100, 100},
    {"DICT_6X6_250", cv::aruco::DICT_6X6_250, 250},
    {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000, 1000},
    {"DICT_7X7_50", cv::aruco::DICT_7X7_50, 50},
    {"DICT_7X7_100", cv::aruco::DICT_7X7_100, 100},
    {"DICT_7X7_250", cv::aruco::DICT_7X7_250, 250},
    {"DICT_7X7_1000", cv::aruco::DICT_7X7_1000, 1000},
    {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL, 1024},
};

const Dictionary *FindDictionary(std::string_view name) {
  for (const Dictionary &dictionary : kDictionaries) {
    if (dictionary.name == name) {
      return &dictionary;
    }
  }
  return nullptr;
}

}  // namespace

std::ostream &operator<<(std::ostream &os, const BoardSpec &spec) {
  return os << spec.squares_x << "x" << spec.squares_y << " ChArUco, "
            << spec.square_length * 1000.0 << " mm squares, "
            << spec.marker_length * 1000.0 << " mm markers, "
            << spec.dictionary;
}

const std::vector<NamedBoard> &BoardPresets() {
  static const std::vector<NamedBoard> kPresets = {
      {"14x9 DICT_5X5_100 (calib.io 600x400 mm)",
       {14, 9, 0.04, 0.03, "DICT_5X5_100"}},
      {"20-inch 15x15", {15, 15, 0.03, 0.022, "DICT_4X4_250"}},
      {"Large, coarse 12x9", {12, 9, 0.06, 0.04666, "DICT_5X5_250"}},
      {"Large, fine 25x18", {25, 18, 0.03, 0.0233, "DICT_5X5_250"}},
      {"Coarse 7x5", {7, 5, 0.04, 0.025, "DICT_5X5_100"}},
  };
  return kPresets;
}

std::vector<std::string_view> DictionaryNames() {
  std::vector<std::string_view> names;
  for (const Dictionary &dictionary : kDictionaries) {
    names.push_back(dictionary.name);
  }
  return names;
}

absl::StatusOr<int> DictionarySize(std::string_view name) {
  const Dictionary *dictionary = FindDictionary(name);
  if (dictionary == nullptr) {
    return absl::InvalidArgumentError(
        absl::StrCat("No ArUco dictionary is named \"", name, "\""));
  }
  return dictionary->size;
}

absl::Status ValidateBoardSpec(const BoardSpec &spec) {
  if (spec.squares_x < 3 || spec.squares_y < 3) {
    return absl::InvalidArgumentError(
        absl::StrCat("A board needs at least 3x3 squares, not ", spec.squares_x,
                     "x", spec.squares_y));
  }
  if (!std::isfinite(spec.square_length) || spec.square_length <= 0.0) {
    return absl::InvalidArgumentError(absl::StrCat(
        "A square must be bigger than 0 m, not ", spec.square_length, " m"));
  }
  if (!std::isfinite(spec.marker_length) || spec.marker_length <= 0.0 ||
      spec.marker_length >= spec.square_length) {
    return absl::InvalidArgumentError(absl::StrCat(
        "A marker must be bigger than 0 m and smaller than its square (",
        spec.square_length, " m), not ", spec.marker_length, " m"));
  }
  absl::StatusOr<int> size = DictionarySize(spec.dictionary);
  if (!size.ok()) {
    return size.status();
  }
  // Half the squares are white, and each white square holds a marker.
  const int markers = spec.squares_x * spec.squares_y / 2;
  if (markers > *size) {
    return absl::InvalidArgumentError(absl::StrCat(
        "A ", spec.squares_x, "x", spec.squares_y, " board needs ", markers,
        " markers, more than ", spec.dictionary, "'s ", *size));
  }
  return absl::OkStatus();
}

cv::Ptr<cv::aruco::CharucoBoard> MakeCharucoBoard(const BoardSpec &spec) {
  const absl::Status valid = ValidateBoardSpec(spec);
  ABSL_CHECK(valid.ok()) << valid;
  const int id = FindDictionary(spec.dictionary)->id;
#if CV_VERSION_MINOR >= 9
  return cv::makePtr<cv::aruco::CharucoBoard>(
      cv::Size(spec.squares_x, spec.squares_y), spec.square_length,
      spec.marker_length, cv::aruco::getPredefinedDictionary(id));
#else
  return cv::aruco::CharucoBoard::create(
      spec.squares_x, spec.squares_y, spec.square_length, spec.marker_length,
      cv::aruco::getPredefinedDictionary(id));
#endif
}

BoardSpec BoardSpecFromFlatbuffer(const calibration::BoardSpec &spec) {
  return BoardSpec{
      .squares_x = spec.squares_x(),
      .squares_y = spec.squares_y(),
      .square_length = spec.square_length(),
      .marker_length = spec.marker_length(),
      .dictionary = spec.has_dictionary() ? spec.dictionary()->str() : "",
  };
}

flatbuffers::Offset<calibration::BoardSpec> PackBoardSpec(
    flatbuffers::FlatBufferBuilder *fbb, const BoardSpec &spec) {
  const flatbuffers::Offset<flatbuffers::String> dictionary =
      fbb->CreateString(spec.dictionary);
  calibration::BoardSpec::Builder builder(*fbb);
  builder.add_squares_x(spec.squares_x);
  builder.add_squares_y(spec.squares_y);
  builder.add_square_length(spec.square_length);
  builder.add_marker_length(spec.marker_length);
  builder.add_dictionary(dictionary);
  return builder.Finish();
}

}  // namespace frc::vision
