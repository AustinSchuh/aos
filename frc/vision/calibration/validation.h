#ifndef FRC_VISION_CALIBRATION_VALIDATION_H_
#define FRC_VISION_CALIBRATION_VALIDATION_H_

#include <map>
#include <optional>
#include <utility>
#include <vector>

#include "opencv2/aruco/charuco.hpp"
#include "opencv2/core.hpp"

namespace frc::vision {

// Checks of how well a calibration models the lens, made on frames a solve
// wasn't fitted to.  Each says how far off a frame is, and where in the image,
// since a calibration's errors live at places in the image.

// A camera's calibration: its matrix and distortion coefficients.
struct Lens {
  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;
};

// The board is flat and its geometry known: fit its pose with `lens`, and
// return each corner's leftover error, in pixels, observed less projected.
// Distortion modelled wrong leaves curvature the fit can't take out.  A wrong
// focal length doesn't show: it only moves the board.  nullopt with fewer
// than 6 corners.
std::optional<std::vector<cv::Point2f>> StraightnessResiduals(
    const cv::Ptr<cv::aruco::CharucoBoard> &board, const std::vector<int> &ids,
    const std::vector<cv::Point2f> &corners, const Lens &lens);

// How far apart two poses of one rigid thing are.
struct PoseDisagreement {
  double translation_m;
  double rotation_rad;
  // Where in the image the thing was.
  cv::Point2f where;
};

// Splits the board's corners into two halves, left and right if `across`,
// else top and bottom, and fits each half's pose from its own corners, as the
// AprilTag detector fits each small tag.  The halves are one rigid board, so
// with a good calibration both give the same pose: how far they disagree is
// the error.  This catches what straightness misses: focal length and
// distortion errors that move each small target differently by where it is in
// the image.  nullopt unless each half has 8 corners.
std::optional<PoseDisagreement> BoardRigidity(
    const cv::Ptr<cv::aruco::CharucoBoard> &board, const std::vector<int> &ids,
    const std::vector<cv::Point2f> &corners, const Lens &lens, bool across);

// An AprilTag found in an image: its id and four corners, as OpenCV's ArUco
// detector orders them.
struct Tag {
  int id;
  std::vector<cv::Point2f> corners;
};

// The 36h11 AprilTags in `image`.
std::vector<Tag> DetectAprilTags(const cv::Mat &image);

// The distance between two tags' centers, and the angle between them, fitting
// each tag's pose alone with `lens`: what the localizer sees of a rigid pair.
// The rig's baseline needn't be known: it only has to stay the same.
std::optional<PoseDisagreement> TagPair(const Tag &a, const Tag &b,
                                        double tag_size, const Lens &lens);

// Collects a check's measurements over frames: their median and worst, and
// their mean in each cell of a grid over the image.
class ValidationGrid {
 public:
  ValidationGrid(cv::Size image_size, int columns);

  void Add(double value, cv::Point2f where);

  int count() const { return values_.size(); }
  double Median() const;
  double Worst() const;
  int columns() const { return columns_; }
  int rows() const { return rows_; }
  // Each cell's mean, row-major; negative where nothing landed.
  std::vector<float> Means() const;

 private:
  cv::Size image_size_;
  int columns_;
  int rows_;
  std::vector<double> values_;
  std::vector<double> sums_;
  std::vector<int> counts_;
};

// A rigid pair of tags' measurements, frame by frame.  The rig's distance and
// angle should never change, so each is judged against the pair's median.
class TagPairTracker {
 public:
  TagPairTracker(cv::Size image_size, int columns);

  void Add(const std::pair<int, int> &ids, const PoseDisagreement &pair);

  // The pair seen most, if any.
  std::optional<std::pair<int, int>> pair() const;
  // That pair's median distance, its distances' spread from it, worst first,
  // and a grid of how far each frame's distance is off the median.
  double MedianDistance() const;
  ValidationGrid DistanceGrid() const;
  ValidationGrid AngleGrid() const;

 private:
  cv::Size image_size_;
  int columns_;
  std::map<std::pair<int, int>, std::vector<PoseDisagreement>> pairs_;
};

}  // namespace frc::vision

#endif  // FRC_VISION_CALIBRATION_VALIDATION_H_
