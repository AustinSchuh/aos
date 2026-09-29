#ifndef FRC_VISION_CALIBRATION_CALIBRATION_VALIDATOR_H_
#define FRC_VISION_CALIBRATION_CALIBRATION_VALIDATOR_H_

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <ostream>
#include <vector>

#include "absl/status/status.h"

#include "frc/orin/distortion.h"
#include "frc/vision/calibration_generated.h"

namespace frc::vision {

// ValidateCalibration's thresholds.  Each leaves room between what it accepts
// and where the detector's undistortion breaks, so a calibration which passes
// doesn't depend on the last bits of the arithmetic.  Those differ between
// x86, where the build validates, and the Orin, which fuses multiply-adds.

// The radial model must stay valid this fraction past the image's farthest
// point: its principal branch must end (at a fold, pole or zero) this far
// past it in distorted radius, and N / D must stay finite and positive this
// far past it in undistorted radius.  UnDistort itself needs none: 25-10,
// grown until its fold is 0.2% past the corners, still converges everywhere
// within 27 steps, but fails abruptly past the fold.  3%, 20-30 px at the
// robots' corners, keeps the verdict off that edge, and keeps ReDistort
// monotonic for the corners RefineEdges fits just outside the image.  More
// would reject working calibrations for where their fits extrapolate to,
// which varies: three fits of 25-01 end its branch 15% (a fold), 84% (a
// pole) and over 100% past the corners.
inline constexpr double kRadialMargin = 0.03;
// UnDistort may take at most half the steps it takes before giving up.
inline constexpr int kMaxUnDistortIterations =
    apriltag::kUndistortIterationThreshold / 2;
// ReDistort of UnDistort's result must land within this many of UnDistort's
// convergence tolerances, kUndistortConvergenceEpsilon scaled to pixels by the
// larger focal length, of the point.  That residual is R times the step
// UnDistort would take next, which is under sqrt(2) R times the tolerance
// wherever the iteration contracts, so for any plausible R a larger one means
// UnDistort stopped where it doesn't.
inline constexpr double kMaxResidualTolerances = 3;
inline double MaxResidualPixels(const apriltag::CameraMatrix &camera_matrix) {
  return kMaxResidualTolerances * apriltag::kUndistortConvergenceEpsilon *
         std::max(camera_matrix.fx, camera_matrix.fy);
}

// The first of ValidateCalibration's checks which failed.
enum class CalibrationCheck {
  kNone,
  // The camera matrix or distortion coefficients are malformed, or hold
  // values the detector ignores.
  kSanity,
  // The radial model folds, has a pole, or turns negative within
  // kRadialMargin of the image's farthest point.
  kRadialModel,
  // UnDistort fails, or passes without margin, at some point of the image.
  kPixels,
};

// A point where UnDistort failed the pixel check.
struct PixelFailure {
  double u;
  double v;
  bool converged;
  int iterations;
  // The normalized radius of UnDistort's result.
  double undistorted_radius;
  // How far ReDistort of UnDistort's result lands from (u, v), in pixels.
  double residual_pixels;
};

// What ValidateCalibration found.  Radii are normalized: relative to the
// principal point and divided by the focal lengths.  The radial model maps
// undistorted radius r to distorted radius f(r) = r N(r^2) / D(r^2).
struct CalibrationReport {
  int width = 0;
  int height = 0;
  CalibrationCheck failed_check = CalibrationCheck::kNone;

  // The largest distorted radius of the points the pixel check covers, at one
  // of the corners of [-1, width] x [-1, height], and the undistorted radius
  // which maps to it, or NaN if none does.
  double max_distorted_radius = 0;
  double max_distorted_radius_u = 0;
  double max_distorted_radius_v = 0;
  double max_undistorted_radius = std::numeric_limits<double>::quiet_NaN();

  // The first r at which the model stops being positive and increasing (N <=
  // 0, D <= 0 or f' <= 0), which ends the principal branch UnDistort must
  // stay on, and the largest f before it.
  double limit_radius = std::numeric_limits<double>::quiet_NaN();
  double limit_distorted_radius = std::numeric_limits<double>::quiet_NaN();
  const char *limit_reason = "";
  // The first r at which N / D stops being finite and positive (N <= 0 or D
  // <= 0).  It is limit_radius unless the branch ends at a fold.
  double singular_radius = std::numeric_limits<double>::quiet_NaN();
  const char *singular_reason = "";
  // The scan stops looking once the model is valid twice as far as the
  // margins need, and then these radii are where it stopped.

  // limit_distorted_radius / max_distorted_radius - 1, and singular_radius /
  // max(max_distorted_radius, max_undistorted_radius) - 1.  The second is
  // there because UnDistort starts at the distorted point, and evaluates N / D
  // everywhere from there to the undistorted one.  Past a fold N / D is still
  // well behaved, but past a pole or zero the first step jumps to another
  // branch.
  double distorted_margin = std::numeric_limits<double>::infinity();
  double undistorted_margin = std::numeric_limits<double>::infinity();

  // The pixel check, which runs unless the sanity check fails.
  int64_t pixels_checked = 0;
  int max_iterations = 0;
  double max_iterations_u = 0;
  double max_iterations_v = 0;
  double max_residual_pixels = 0;
  double max_residual_u = 0;
  double max_residual_v = 0;
  // MaxResidualPixels for the calibration.
  double residual_limit_pixels = 0;
  // Failing points by the first thing wrong: UnDistort gave up, took more
  // than kMaxUnDistortIterations steps, converged at or past limit_radius, or
  // converged more than residual_limit_pixels off.
  int64_t unconverged_pixels = 0;
  int64_t slow_pixels = 0;
  int64_t off_branch_pixels = 0;
  int64_t inexact_pixels = 0;
  // Failing points above left, above right, below left and below right of the
  // principal point, the nearest one's distance from it in pixels, and how
  // far the deepest one lies inside the image's edges, min(u, v, width - u,
  // height - v).  The detector drops tags within --pixel_border of the edges.
  std::array<int64_t, 4> failures_by_quadrant{};
  double nearest_failure_pixels = std::numeric_limits<double>::infinity();
  double deepest_failure_pixels = -std::numeric_limits<double>::infinity();
  // The first few failing points, in row order.
  std::vector<PixelFailure> failures;

  int64_t failing_pixels() const {
    return unconverged_pixels + slow_pixels + off_branch_pixels +
           inexact_pixels;
  }
};

std::ostream &operator<<(std::ostream &os, const CalibrationReport &report);

// Returns OK if the AprilTag detector can undistort every point of a width x
// height image with calibration, with room to spare, or an error saying which
// checks failed and by how much.  frc/orin/distortion.h's UnDistort, which the
// detector runs, must at every integer point of [-1, width] x [-1, height]
// converge within kMaxUnDistortIterations steps, inside
// report->limit_radius, to a point which ReDistort maps back within
// MaxResidualPixels.  Those points cover the detection corners in either
// pixel center convention, and RefineEdges' edge samples, which are within a
// pixel of the image.  The radial model must also stay valid kRadialMargin
// past the image, and the calibration must be well formed, with any
// distortion coefficients past the 8th zero, since the detector ignores them.
//
// Fills in *report, if given, with the numbers behind the result.  The pixel
// check dominates, and splits the rows between up to 8 threads.  At 1600x1304
// it takes 15 ms on a desktop, and 55 ms on the Orin's 6 cores.
absl::Status ValidateCalibration(
    const calibration::CameraCalibration &calibration, int width, int height,
    CalibrationReport *report = nullptr);

}  // namespace frc::vision

#endif  // FRC_VISION_CALIBRATION_CALIBRATION_VALIDATOR_H_
