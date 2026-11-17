#include "frc/vision/calibration/validation.h"

#include <algorithm>
#include <cmath>

#include "opencv2/calib3d.hpp"

namespace frc::vision {
namespace {

std::vector<cv::Point3f> ChessboardCorners(
    const cv::Ptr<cv::aruco::CharucoBoard> &board) {
#if CV_VERSION_MINOR >= 9
  return board->getChessboardCorners();
#else
  return board->chessboardCorners;
#endif
}

// The angle of the rotation from `a` to `b`.
double AngleBetween(const cv::Vec3d &a, const cv::Vec3d &b) {
  cv::Mat Ra, Rb;
  cv::Rodrigues(a, Ra);
  cv::Rodrigues(b, Rb);
  cv::Mat relative = Ra.t() * Rb;
  const double cosine =
      std::clamp((relative.at<double>(0, 0) + relative.at<double>(1, 1) +
                  relative.at<double>(2, 2) - 1.0) /
                     2.0,
                 -1.0, 1.0);
  return std::acos(cosine);
}

cv::Point3d Transform(const cv::Vec3d &rvec, const cv::Vec3d &tvec,
                      const cv::Point3d &point) {
  cv::Mat R;
  cv::Rodrigues(rvec, R);
  cv::Mat p = R * cv::Mat(cv::Vec3d(point.x, point.y, point.z)) + cv::Mat(tvec);
  return {p.at<double>(0), p.at<double>(1), p.at<double>(2)};
}

cv::Point2f Centroid(const std::vector<cv::Point2f> &points) {
  cv::Point2f sum(0, 0);
  for (const cv::Point2f &point : points) {
    sum += point;
  }
  return points.empty() ? sum : sum / static_cast<float>(points.size());
}

// The RMS distance, in pixels, between `image` and `object` projected at a
// pose.
double ReprojectionError(const std::vector<cv::Point3f> &object,
                         const std::vector<cv::Point2f> &image,
                         const Lens &lens, const cv::Vec3d &rvec,
                         const cv::Vec3d &tvec) {
  const std::vector<cv::Point2f> projected = lens.Project(object, rvec, tvec);
  double sum = 0;
  for (size_t i = 0; i < projected.size(); ++i) {
    const cv::Point2f d = image[i] - projected[i];
    sum += d.dot(d);
  }
  return std::sqrt(sum / std::max<size_t>(1, projected.size()));
}

// A tag whose best pose misses its corners by more than this, in pixels, isn't
// measured.
constexpr double kMaxTagFitError = 2.0;

}  // namespace

std::optional<std::vector<cv::Point2f>> StraightnessResiduals(
    const cv::Ptr<cv::aruco::CharucoBoard> &board, const std::vector<int> &ids,
    const std::vector<cv::Point2f> &corners, const Lens &lens) {
  if (ids.size() < 6 || ids.size() != corners.size()) {
    return std::nullopt;
  }
  const std::vector<cv::Point3f> board_corners = ChessboardCorners(board);
  std::vector<cv::Point3f> object;
  for (int id : ids) {
    object.push_back(board_corners.at(id));
  }
  cv::Vec3d rvec, tvec;
  if (!lens.SolvePnP(object, corners, rvec, tvec, cv::SOLVEPNP_ITERATIVE,
                     true)) {
    return std::nullopt;
  }
  const std::vector<cv::Point2f> projected = lens.Project(object, rvec, tvec);
  std::vector<cv::Point2f> residuals;
  for (size_t i = 0; i < projected.size(); ++i) {
    residuals.push_back(corners[i] - projected[i]);
  }
  return residuals;
}

std::optional<PoseDisagreement> BoardRigidity(
    const cv::Ptr<cv::aruco::CharucoBoard> &board, const std::vector<int> &ids,
    const std::vector<cv::Point2f> &corners, const Lens &lens, bool across) {
  const cv::Size squares = board->getChessboardSize();
  // The inner corners, numbered across then down.
  const int columns = squares.width - 1;
  const int rows = squares.height - 1;
  const std::vector<cv::Point3f> board_corners = ChessboardCorners(board);
  std::vector<cv::Point3f> object[2];
  std::vector<cv::Point2f> image[2];
  for (size_t i = 0; i < ids.size() && i < corners.size(); ++i) {
    const int column = ids[i] % columns;
    const int row = ids[i] / columns;
    const int half =
        across ? (column < columns / 2 ? 0 : 1) : (row < rows / 2 ? 0 : 1);
    object[half].push_back(board_corners.at(ids[i]));
    image[half].push_back(corners[i]);
  }
  if (object[0].size() < 8 || object[1].size() < 8) {
    return std::nullopt;
  }
  cv::Vec3d rvec[2], tvec[2];
  for (int half = 0; half < 2; ++half) {
    if (!lens.SolvePnP(object[half], image[half], rvec[half], tvec[half],
                       cv::SOLVEPNP_ITERATIVE, true)) {
      return std::nullopt;
    }
  }
  // Where each half's pose puts the board's center: comparing the poses there
  // keeps a rotation's error from being magnified by a far-off origin.
  const cv::Point3d center(squares.width * board->getSquareLength() / 2.0,
                           squares.height * board->getSquareLength() / 2.0,
                           0.0);
  const cv::Point3d a = Transform(rvec[0], tvec[0], center);
  const cv::Point3d b = Transform(rvec[1], tvec[1], center);
  return PoseDisagreement{.translation_m = cv::norm(a - b),
                          .rotation_rad = AngleBetween(rvec[0], rvec[1]),
                          .where = Centroid(corners)};
}

std::vector<Tag> DetectAprilTags(const cv::Mat &image) {
#if CV_VERSION_MINOR >= 9
  const cv::Ptr<cv::aruco::Dictionary> dictionary =
      cv::makePtr<cv::aruco::Dictionary>(
          cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11));
#else
  const cv::Ptr<cv::aruco::Dictionary> dictionary =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
#endif
  cv::Ptr<cv::aruco::DetectorParameters> parameters =
      cv::makePtr<cv::aruco::DetectorParameters>();
  parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  cv::aruco::detectMarkers(image, dictionary, corners, ids, parameters);
  std::vector<Tag> tags;
  for (size_t i = 0; i < ids.size(); ++i) {
    tags.push_back({ids[i], corners[i]});
  }
  return tags;
}

std::optional<PoseDisagreement> TagPair(const Tag &a, const Tag &b,
                                        double tag_size, const Lens &lens) {
  // OpenCV's corner order, for SOLVEPNP_IPPE_SQUARE.
  const float half = tag_size / 2.0;
  const std::vector<cv::Point3f> object = {
      {-half, half, 0}, {half, half, 0}, {half, -half, 0}, {-half, -half, 0}};
  cv::Vec3d rvec[2], tvec[2];
  const Tag *tags[2] = {&a, &b};
  for (int i = 0; i < 2; ++i) {
    // IPPE solves the square, refined against the full model if that fits
    // better.  A tag no pose fits is no measurement.
    if (tags[i]->corners.size() != 4 ||
        !lens.SolvePnP(object, tags[i]->corners, rvec[i], tvec[i],
                       cv::SOLVEPNP_IPPE_SQUARE, true)) {
      return std::nullopt;
    }
    if (tvec[i][2] <= 0 ||
        ReprojectionError(object, tags[i]->corners, lens, rvec[i], tvec[i]) >
            kMaxTagFitError) {
      return std::nullopt;
    }
  }
  return PoseDisagreement{
      .translation_m = cv::norm(tvec[0] - tvec[1]),
      .rotation_rad = AngleBetween(rvec[0], rvec[1]),
      .where = (Centroid(a.corners) + Centroid(b.corners)) / 2.0f};
}

ValidationGrid::ValidationGrid(cv::Size image_size, int columns)
    : image_size_(image_size),
      columns_(columns),
      rows_(std::max(1, static_cast<int>(std::lround(
                            columns * static_cast<double>(image_size.height) /
                            std::max(1, image_size.width))))),
      sums_(columns_ * rows_, 0.0),
      counts_(columns_ * rows_, 0) {}

void ValidationGrid::Add(double value, cv::Point2f where) {
  if (!std::isfinite(value)) {
    return;
  }
  values_.push_back(value);
  const int column =
      std::clamp(static_cast<int>(where.x / image_size_.width * columns_), 0,
                 columns_ - 1);
  const int row = std::clamp(
      static_cast<int>(where.y / image_size_.height * rows_), 0, rows_ - 1);
  sums_[row * columns_ + column] += value;
  ++counts_[row * columns_ + column];
}

double ValidationGrid::Median() const {
  if (values_.empty()) {
    return 0.0;
  }
  std::vector<double> sorted = values_;
  std::nth_element(sorted.begin(), sorted.begin() + sorted.size() / 2,
                   sorted.end());
  return sorted[sorted.size() / 2];
}

double ValidationGrid::Worst() const {
  return values_.empty() ? 0.0
                         : *std::max_element(values_.begin(), values_.end());
}

std::vector<float> ValidationGrid::Means() const {
  std::vector<float> means(sums_.size(), -1.0f);
  for (size_t i = 0; i < sums_.size(); ++i) {
    if (counts_[i] > 0) {
      means[i] = sums_[i] / counts_[i];
    }
  }
  return means;
}

TagPairTracker::TagPairTracker(cv::Size image_size, int columns)
    : image_size_(image_size), columns_(columns) {}

void TagPairTracker::Add(const std::pair<int, int> &ids,
                         const PoseDisagreement &pair) {
  pairs_[ids].push_back(pair);
}

std::optional<std::pair<int, int>> TagPairTracker::pair() const {
  std::optional<std::pair<int, int>> best;
  size_t most = 0;
  for (const auto &[ids, measurements] : pairs_) {
    if (measurements.size() > most) {
      most = measurements.size();
      best = ids;
    }
  }
  return best;
}

namespace {

double MedianOf(std::vector<double> values) {
  if (values.empty()) {
    return 0.0;
  }
  std::nth_element(values.begin(), values.begin() + values.size() / 2,
                   values.end());
  return values[values.size() / 2];
}

}  // namespace

double TagPairTracker::MedianDistance() const {
  const std::optional<std::pair<int, int>> ids = pair();
  if (!ids.has_value()) {
    return 0.0;
  }
  std::vector<double> distances;
  for (const PoseDisagreement &measurement : pairs_.at(*ids)) {
    distances.push_back(measurement.translation_m);
  }
  return MedianOf(distances);
}

ValidationGrid TagPairTracker::DistanceGrid() const {
  ValidationGrid grid(image_size_, columns_);
  const std::optional<std::pair<int, int>> ids = pair();
  if (!ids.has_value()) {
    return grid;
  }
  const double median = MedianDistance();
  for (const PoseDisagreement &measurement : pairs_.at(*ids)) {
    grid.Add(std::abs(measurement.translation_m - median), measurement.where);
  }
  return grid;
}

ValidationGrid TagPairTracker::AngleGrid() const {
  ValidationGrid grid(image_size_, columns_);
  const std::optional<std::pair<int, int>> ids = pair();
  if (!ids.has_value()) {
    return grid;
  }
  std::vector<double> angles;
  for (const PoseDisagreement &measurement : pairs_.at(*ids)) {
    angles.push_back(measurement.rotation_rad);
  }
  const double median = MedianOf(angles);
  for (const PoseDisagreement &measurement : pairs_.at(*ids)) {
    grid.Add(std::abs(measurement.rotation_rad - median), measurement.where);
  }
  return grid;
}

}  // namespace frc::vision
