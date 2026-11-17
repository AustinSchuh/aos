#include "frc/vision/calibration/validation.h"

#include <cmath>

#include "gtest/gtest.h"
#include "opencv2/calib3d.hpp"

#include "frc/vision/util/charuco_board.h"

namespace frc::vision::testing {
namespace {

constexpr int kWidth = 1600, kHeight = 1304;

Lens TrueLens() {
  return {(cv::Mat_<double>(3, 3) << 1000, 0, 800, 0, 1000, 650, 0, 0, 1),
          (cv::Mat_<double>(1, 5) << -0.25, 0.08, 0, 0, -0.01)};
}

// The true camera's matrix, with no distortion: wrong at the edges.
Lens NoDistortion() {
  return {TrueLens().camera_matrix, cv::Mat::zeros(1, 5, CV_64F)};
}

std::vector<cv::Point3f> Corners(const cv::Ptr<cv::aruco::CharucoBoard> &b) {
#if CV_VERSION_MINOR >= 9
  return b->getChessboardCorners();
#else
  return b->chessboardCorners;
#endif
}

// The board's corners as the true camera sees them, with the board's center
// at `x`, `y`, `z` meters, facing the camera, turned `yaw` about its vertical.
std::pair<std::vector<int>, std::vector<cv::Point2f>> View(
    const cv::Ptr<cv::aruco::CharucoBoard> &board, double x, double y, double z,
    double yaw = 0.0) {
  const cv::Vec3d rvec(0, yaw, 0);
  cv::Mat R;
  cv::Rodrigues(rvec, R);
  const cv::Size squares = board->getChessboardSize();
  const cv::Vec3d center(squares.width * board->getSquareLength() / 2,
                         squares.height * board->getSquareLength() / 2, 0);
  const cv::Mat t = cv::Mat(cv::Vec3d(x, y, z)) - R * cv::Mat(center);
  std::vector<cv::Point2f> projected;
  const Lens lens = TrueLens();
  cv::projectPoints(Corners(board), rvec, cv::Vec3d(t), lens.camera_matrix,
                    lens.dist_coeffs, projected);
  std::vector<int> ids;
  std::vector<cv::Point2f> corners;
  for (size_t i = 0; i < projected.size(); ++i) {
    if (projected[i].x >= 0 && projected[i].x < kWidth && projected[i].y >= 0 &&
        projected[i].y < kHeight) {
      ids.push_back(i);
      corners.push_back(projected[i]);
    }
  }
  return {ids, corners};
}

double Rms(const std::vector<cv::Point2f> &residuals) {
  double sum = 0;
  for (const cv::Point2f &r : residuals) {
    sum += r.dot(r);
  }
  return std::sqrt(sum / residuals.size());
}

TEST(ValidationTest, StraightnessSeesDistortion) {
  cv::Ptr<cv::aruco::CharucoBoard> board = MakeCharucoBoard(BoardSpec{});
  // In a corner of the image, where distortion bends the board most.
  const auto [ids, corners] = View(board, -0.45, -0.35, 1.2, 0.3);
  ASSERT_GT(ids.size(), 20u);
  const auto right = StraightnessResiduals(board, ids, corners, TrueLens());
  ASSERT_TRUE(right.has_value());
  EXPECT_LT(Rms(*right), 1e-3);
  const auto wrong = StraightnessResiduals(board, ids, corners, NoDistortion());
  ASSERT_TRUE(wrong.has_value());
  EXPECT_GT(Rms(*wrong), 1.0);
}

TEST(ValidationTest, RigiditySeesWhatMovesSmallTargets) {
  cv::Ptr<cv::aruco::CharucoBoard> board = MakeCharucoBoard(BoardSpec{});
  const auto [ids, corners] = View(board, 0.35, 0.25, 1.4, -0.4);
  for (bool across : {true, false}) {
    const auto right = BoardRigidity(board, ids, corners, TrueLens(), across);
    ASSERT_TRUE(right.has_value());
    EXPECT_LT(right->translation_m, 1e-4);
    EXPECT_LT(right->rotation_rad, 1e-4);
    const auto wrong =
        BoardRigidity(board, ids, corners, NoDistortion(), across);
    ASSERT_TRUE(wrong.has_value());
    EXPECT_GT(wrong->translation_m + wrong->rotation_rad, 1e-3);
  }
}

// A tag's corners as the true camera sees it, at `x`, `y`, `z`, in OpenCV's
// order.
Tag ProjectTag(int id, double x, double y, double z, double size) {
  const float half = size / 2;
  const std::vector<cv::Point3f> object = {
      {-half, half, 0}, {half, half, 0}, {half, -half, 0}, {-half, -half, 0}};
  // Facing the camera (the tag's y up is the image's y down), turned a little,
  // as tags are: exactly face on, a square's pose is ambiguous.
  cv::Mat R, flip, turn;
  cv::Rodrigues(cv::Vec3d(M_PI, 0, 0), flip);
  cv::Rodrigues(cv::Vec3d(0.15, 0.25, 0), turn);
  R = turn * flip;
  cv::Vec3d rvec;
  cv::Rodrigues(R, rvec);
  Tag tag{id, {}};
  const Lens lens = TrueLens();
  cv::projectPoints(object, rvec, cv::Vec3d(x, y, z), lens.camera_matrix,
                    lens.dist_coeffs, tag.corners);
  return tag;
}

TEST(ValidationTest, TagPairsKeepTheirDistance) {
  constexpr double kSize = 0.1651;
  TagPairTracker right(cv::Size(kWidth, kHeight), 8);
  TagPairTracker wrong(cv::Size(kWidth, kHeight), 8);
  // A rig 0.5 m wide, moved about the image, out to its edges.
  for (double x : {-0.7, 0.0, 0.7}) {
    for (double y : {-0.55, 0.0, 0.55}) {
      const Tag a = ProjectTag(3, x - 0.25, y, 2.0, kSize);
      const Tag b = ProjectTag(7, x + 0.25, y, 2.0, kSize);
      const auto measured = TagPair(a, b, kSize, TrueLens());
      ASSERT_TRUE(measured.has_value());
      right.Add({3, 7}, *measured);
      // A wrong lens may fit a tag too badly to measure.
      if (const auto off = TagPair(a, b, kSize, NoDistortion())) {
        wrong.Add({3, 7}, *off);
      }
    }
  }
  ASSERT_EQ(right.pair(), std::make_pair(3, 7));
  EXPECT_NEAR(right.MedianDistance(), 0.5, 1e-3);
  EXPECT_LT(right.DistanceGrid().Worst(), 1e-4);
  EXPECT_GT(wrong.DistanceGrid().Worst(), 2e-3);
}

TEST(ValidationTest, GridAveragesWhereThingsLand) {
  ValidationGrid grid(cv::Size(100, 50), 4);
  EXPECT_EQ(grid.rows(), 2);
  grid.Add(1.0, {10, 10});
  grid.Add(3.0, {20, 10});
  grid.Add(10.0, {90, 40});
  EXPECT_EQ(grid.count(), 3);
  EXPECT_EQ(grid.Median(), 3.0);
  EXPECT_EQ(grid.Worst(), 10.0);
  const std::vector<float> means = grid.Means();
  ASSERT_EQ(means.size(), 8u);
  EXPECT_EQ(means[0], 2.0f);
  EXPECT_EQ(means[7], 10.0f);
  EXPECT_LT(means[1], 0.0f);
}

}  // namespace
}  // namespace frc::vision::testing
