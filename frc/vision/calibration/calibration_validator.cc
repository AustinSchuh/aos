#include "frc/vision/calibration/calibration_validator.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"

namespace frc::vision {
namespace {

constexpr size_t kMaxReportedFailures = 3;
constexpr int kMaxThreads = 8;

// The radial scan steps this fraction of the image's distorted radius at a
// time.  It stops at the first pole or zero, once it is this many times past
// everything the margins need, or at kMaxScanRadius.
constexpr double kScanStep = 1e-5;
constexpr double kScanExtent = 2.0;
constexpr double kMaxScanRadius = 1000.0;

// The rational model's radial factor is N(s) / D(s), with s = r^2.
struct RadialModel {
  double k1, k2, k3, k4, k5, k6;

  double N(double s) const { return 1 + s * (k1 + s * (k2 + s * k3)); }
  double D(double s) const { return 1 + s * (k4 + s * (k5 + s * k6)); }
  double dN(double s) const { return k1 + s * (2 * k2 + s * 3 * k3); }
  double dD(double s) const { return k4 + s * (2 * k5 + s * 3 * k6); }
};

absl::Status CheckSanity(const calibration::CameraCalibration &calibration,
                         int width, int height) {
  if (width <= 0 || height <= 0) {
    return absl::InvalidArgumentError(
        absl::StrFormat("image size %dx%d is empty", width, height));
  }
  const flatbuffers::Vector<float> *intrinsics = calibration.intrinsics();
  if (intrinsics == nullptr || intrinsics->size() != 9) {
    return absl::InvalidArgumentError(absl::StrCat(
        "intrinsics must hold a 3x3 camera matrix, but has ",
        intrinsics == nullptr ? 0 : intrinsics->size(), " values"));
  }
  if (!std::all_of(intrinsics->begin(), intrinsics->end(),
                   [](float x) { return std::isfinite(x); })) {
    return absl::InvalidArgumentError(
        absl::StrCat("intrinsics [", absl::StrJoin(*intrinsics, ", "),
                     "] are not all finite"));
  }
  if (intrinsics->Get(1) != 0 || intrinsics->Get(3) != 0 ||
      intrinsics->Get(6) != 0 || intrinsics->Get(7) != 0 ||
      intrinsics->Get(8) != 1) {
    return absl::InvalidArgumentError(absl::StrCat(
        "intrinsics [", absl::StrJoin(*intrinsics, ", "),
        "] are not of the form [fx, 0, cx, 0, fy, cy, 0, 0, 1], and the "
        "detector reads only fx, cx, fy and cy"));
  }
  const apriltag::CameraMatrix camera_matrix =
      apriltag::GetCameraMatrix(&calibration);
  if (!(camera_matrix.fx > 0) || !(camera_matrix.fy > 0)) {
    return absl::InvalidArgumentError(
        absl::StrFormat("focal lengths fx %g and fy %g must be positive",
                        camera_matrix.fx, camera_matrix.fy));
  }
  if (!(camera_matrix.cx >= 0 && camera_matrix.cx <= width &&
        camera_matrix.cy >= 0 && camera_matrix.cy <= height)) {
    return absl::InvalidArgumentError(
        absl::StrFormat("principal point (%g, %g) is outside the %dx%d image",
                        camera_matrix.cx, camera_matrix.cy, width, height));
  }

  const flatbuffers::Vector<float> *dist_coeffs = calibration.dist_coeffs();
  const size_t num_coeffs = dist_coeffs == nullptr ? 0 : dist_coeffs->size();
  if (calibration.distortion_model() ==
      calibration::DistortionModel::KANNALA_BRANDT) {
    if (num_coeffs != 4) {
      return absl::InvalidArgumentError(
          absl::StrCat("dist_coeffs has ", num_coeffs,
                       " coefficients, but Kannala-Brandt has 4"));
    }
    if (!std::all_of(dist_coeffs->begin(), dist_coeffs->end(),
                     [](float x) { return std::isfinite(x); })) {
      return absl::InvalidArgumentError(
          absl::StrCat("dist_coeffs [", absl::StrJoin(*dist_coeffs, ", "),
                       "] are not all finite"));
    }
    return absl::OkStatus();
  }
  if (num_coeffs != 5 && num_coeffs != 8 && num_coeffs != 14) {
    return absl::InvalidArgumentError(
        absl::StrCat("dist_coeffs has ", num_coeffs,
                     " coefficients, but the OpenCV models the detector reads "
                     "have 5, 8 or 14"));
  }
  if (!std::all_of(dist_coeffs->begin(), dist_coeffs->end(),
                   [](float x) { return std::isfinite(x); })) {
    return absl::InvalidArgumentError(
        absl::StrCat("dist_coeffs [", absl::StrJoin(*dist_coeffs, ", "),
                     "] are not all finite"));
  }
  for (size_t i = 8; i < num_coeffs; ++i) {
    if (dist_coeffs->Get(i) != 0) {
      return absl::InvalidArgumentError(absl::StrFormat(
          "dist_coeffs[%d] is %g, but the detector ignores the thin prism and "
          "tilt coefficients past the 8th, so it would run a different model "
          "than was calibrated",
          i, dist_coeffs->Get(i)));
    }
  }
  return absl::OkStatus();
}

// Finds where the principal branch ends and where N / D first stops being
// finite and positive, and fills in the report's radii and margins.
// report->max_distorted_radius must be set.
void ScanRadialModel(const apriltag::DistCoeffs &dist_coeffs,
                     CalibrationReport *report) {
  const RadialModel model{dist_coeffs.k1, dist_coeffs.k2, dist_coeffs.k3,
                          dist_coeffs.k4, dist_coeffs.k5, dist_coeffs.k6};
  const double max_distorted = report->max_distorted_radius;
  const double step = std::max(max_distorted, 1e-3) * kScanStep;
  // The largest radius UnDistort evaluates N / D at for the farthest point,
  // as far as the scan has found it.
  const auto start_radius = [&]() {
    return std::max(max_distorted, std::isnan(report->max_undistorted_radius)
                                       ? 0.0
                                       : report->max_undistorted_radius);
  };
  bool branch_ended = false;
  double distorted = 0;
  for (int64_t i = 1;; ++i) {
    const double r = i * step;
    const double s = r * r;
    const double n = model.N(s);
    const double d = model.D(s);
    const char *const singular_reason = !(d > 0)   ? "pole, D <= 0"
                                        : !(n > 0) ? "zero, N <= 0"
                                                   : nullptr;
    if (!branch_ended) {
      // f' has the sign of this where D > 0.
      const double df = n * d + 2 * s * (model.dN(s) * d - n * model.dD(s));
      if (singular_reason != nullptr || !(df > 0)) {
        report->limit_radius = r;
        report->limit_distorted_radius = distorted;
        report->limit_reason =
            singular_reason != nullptr ? singular_reason : "fold, f' <= 0";
        branch_ended = true;
      } else {
        distorted = r * n / d;
        if (std::isnan(report->max_undistorted_radius) &&
            distorted >= max_distorted) {
          report->max_undistorted_radius = r;
        }
      }
    }
    if (singular_reason != nullptr) {
      report->singular_radius = r;
      report->singular_reason = singular_reason;
      break;
    }
    // Past here, N / D and any unended branch are valid farther than the
    // margins need, and the pixel check only needs to know they are valid
    // this far.
    const bool far_enough =
        r >= kScanExtent * start_radius() &&
        (branch_ended || (!std::isnan(report->max_undistorted_radius) &&
                          distorted >= kScanExtent * max_distorted));
    if (far_enough || r >= kMaxScanRadius) {
      if (!branch_ended) {
        report->limit_radius = r;
        report->limit_distorted_radius = distorted;
        report->limit_reason = "where the scan stopped";
      }
      report->singular_radius = r;
      report->singular_reason = "where the scan stopped";
      break;
    }
  }

  if (max_distorted > 0) {
    report->distorted_margin =
        report->limit_distorted_radius / max_distorted - 1;
    // With no undistorted radius, the distorted margin fails anyway.
    report->undistorted_margin = report->singular_radius / start_radius() - 1;
  }
}

// ScanRadialModel for Kannala-Brandt, whose distorted radius is the angle
// theta_d(theta) for the angle theta off the axis, at undistorted radius
// tan(theta).  Its branch ends where theta_d stops increasing (a fold) or at
// 90 degrees, past which nothing is in front of the camera; it has no pole or
// zero, so N / D's limit is the branch's.
void ScanKannalaBrandt(const apriltag::DistCoeffs &dist_coeffs,
                       CalibrationReport *report) {
  const double max_distorted = report->max_distorted_radius;
  constexpr int kSteps = 100000;
  // Just short of 90 degrees, where tan(theta) runs away.
  const double max_theta = M_PI / 2 * (1 - 1e-6);
  double distorted = 0;
  report->limit_radius = std::tan(max_theta);
  report->limit_reason = "90 degrees off the axis";
  for (int i = 1; i <= kSteps; ++i) {
    const double theta = max_theta * i / kSteps;
    if (!(apriltag::KannalaBrandtSlope(dist_coeffs, theta) > 0)) {
      report->limit_radius = std::tan(theta);
      report->limit_reason = "fold, theta_d' <= 0";
      break;
    }
    distorted = apriltag::KannalaBrandtThetaD(dist_coeffs, theta);
    if (std::isnan(report->max_undistorted_radius) &&
        distorted >= max_distorted) {
      report->max_undistorted_radius = std::tan(theta);
    }
  }
  report->limit_distorted_radius = distorted;
  report->singular_radius = report->limit_radius;
  report->singular_reason = report->limit_reason;
  if (max_distorted > 0) {
    report->distorted_margin = distorted / max_distorted - 1;
    const double start =
        std::max(max_distorted, std::isnan(report->max_undistorted_radius)
                                    ? 0.0
                                    : report->max_undistorted_radius);
    report->undistorted_margin = report->singular_radius / start - 1;
  }
}

// Whether value is a new maximum over max, counting NaN as the largest.
bool IsNewMax(double value, double max) {
  return !std::isnan(max) && !(value <= max);
}

// The pixel check's results for a block of rows.
struct PixelStats {
  int64_t checked = 0;
  int max_iterations = 0;
  double max_iterations_u = 0;
  double max_iterations_v = 0;
  double max_residual_squared = 0;
  double max_residual_u = 0;
  double max_residual_v = 0;
  int64_t unconverged = 0;
  int64_t slow = 0;
  int64_t off_branch = 0;
  int64_t inexact = 0;
  std::array<int64_t, 4> failures_by_quadrant{};
  double nearest_failure_pixels = std::numeric_limits<double>::infinity();
  double deepest_failure_pixels = -std::numeric_limits<double>::infinity();
  std::vector<PixelFailure> failures;

  // Adds the stats of the rows after these, keeping the first point of each
  // maximum.
  void Append(const PixelStats &later) {
    checked += later.checked;
    if (later.max_iterations > max_iterations) {
      max_iterations = later.max_iterations;
      max_iterations_u = later.max_iterations_u;
      max_iterations_v = later.max_iterations_v;
    }
    if (IsNewMax(later.max_residual_squared, max_residual_squared)) {
      max_residual_squared = later.max_residual_squared;
      max_residual_u = later.max_residual_u;
      max_residual_v = later.max_residual_v;
    }
    unconverged += later.unconverged;
    slow += later.slow;
    off_branch += later.off_branch;
    inexact += later.inexact;
    for (size_t i = 0; i < failures_by_quadrant.size(); ++i) {
      failures_by_quadrant[i] += later.failures_by_quadrant[i];
    }
    nearest_failure_pixels =
        std::min(nearest_failure_pixels, later.nearest_failure_pixels);
    deepest_failure_pixels =
        std::max(deepest_failure_pixels, later.deepest_failure_pixels);
    for (const PixelFailure &failure : later.failures) {
      if (failures.size() < kMaxReportedFailures) {
        failures.push_back(failure);
      }
    }
  }
};

// Runs UnDistort on the points of rows [v_begin, v_end) from u = -1 to width.
PixelStats CheckRows(const apriltag::CameraMatrix &camera_matrix,
                     const apriltag::DistCoeffs &dist_coeffs,
                     double limit_radius, int width, int height, int v_begin,
                     int v_end) {
  const double limit_squared = limit_radius * limit_radius;
  const double max_residual_squared =
      std::pow(MaxResidualPixels(camera_matrix), 2);
  PixelStats stats;
  for (int v = v_begin; v < v_end; ++v) {
    for (int u = -1; u <= width; ++u) {
      double undistorted_u = u;
      double undistorted_v = v;
      int iterations;
      const bool converged =
          apriltag::UnDistort(&undistorted_u, &undistorted_v, camera_matrix,
                              dist_coeffs, &iterations);
      const double x = (undistorted_u - camera_matrix.cx) / camera_matrix.fx;
      const double y = (undistorted_v - camera_matrix.cy) / camera_matrix.fy;
      const double radius_squared = x * x + y * y;

      double redistorted_u = undistorted_u;
      double redistorted_v = undistorted_v;
      apriltag::ReDistort(&redistorted_u, &redistorted_v, camera_matrix,
                          dist_coeffs);
      const double du = redistorted_u - u;
      const double dv = redistorted_v - v;
      const double residual_squared = du * du + dv * dv;

      ++stats.checked;
      if (iterations > stats.max_iterations) {
        stats.max_iterations = iterations;
        stats.max_iterations_u = u;
        stats.max_iterations_v = v;
      }
      if (IsNewMax(residual_squared, stats.max_residual_squared)) {
        stats.max_residual_squared = residual_squared;
        stats.max_residual_u = u;
        stats.max_residual_v = v;
      }
      // The comparisons are written so NaNs fail.
      if (!converged) {
        ++stats.unconverged;
      } else if (iterations > kMaxUnDistortIterations) {
        ++stats.slow;
      } else if (!(radius_squared < limit_squared)) {
        ++stats.off_branch;
      } else if (!(residual_squared <= max_residual_squared)) {
        ++stats.inexact;
      } else {
        continue;
      }
      ++stats.failures_by_quadrant[(u > camera_matrix.cx ? 1 : 0) +
                                   (v > camera_matrix.cy ? 2 : 0)];
      stats.nearest_failure_pixels =
          std::min(stats.nearest_failure_pixels,
                   std::hypot(u - camera_matrix.cx, v - camera_matrix.cy));
      stats.deepest_failure_pixels =
          std::max<double>(stats.deepest_failure_pixels,
                           std::min({u, v, width - u, height - v}));
      if (stats.failures.size() < kMaxReportedFailures) {
        stats.failures.push_back(PixelFailure{
            .u = static_cast<double>(u),
            .v = static_cast<double>(v),
            .converged = converged,
            .iterations = iterations,
            .undistorted_radius = std::sqrt(radius_squared),
            .residual_pixels = std::sqrt(residual_squared),
        });
      }
    }
  }
  return stats;
}

// Runs UnDistort on every point of [-1, width] x [-1, height], splitting the
// rows between threads.  The result doesn't depend on how many there are.
PixelStats CheckPixels(const apriltag::CameraMatrix &camera_matrix,
                       const apriltag::DistCoeffs &dist_coeffs,
                       double limit_radius, int width, int height) {
  const int rows = height + 2;
  const int num_threads = std::clamp<int>(std::thread::hardware_concurrency(),
                                          1, std::min(kMaxThreads, rows));
  std::vector<PixelStats> blocks(num_threads);
  std::vector<std::thread> threads;
  for (int i = 0; i < num_threads; ++i) {
    threads.emplace_back([&, i]() {
      blocks[i] = CheckRows(camera_matrix, dist_coeffs, limit_radius, width,
                            height, -1 + rows * i / num_threads,
                            -1 + rows * (i + 1) / num_threads);
    });
  }
  for (std::thread &thread : threads) {
    thread.join();
  }
  PixelStats stats;
  for (const PixelStats &block : blocks) {
    stats.Append(block);
  }
  return stats;
}

std::string DescribeFailure(const PixelFailure &failure,
                            const CalibrationReport &report) {
  std::string problem;
  if (!failure.converged) {
    problem = "gave up";
  } else if (failure.iterations > kMaxUnDistortIterations) {
    problem = absl::StrCat("took ", failure.iterations, " steps");
  } else if (!(failure.undistorted_radius < report.limit_radius)) {
    problem = absl::StrFormat("converged off the principal branch, at r %.4f",
                              failure.undistorted_radius);
  } else {
    problem = "converged";
  }
  return absl::StrFormat("(%g, %g) %s, residual %.3g px", failure.u, failure.v,
                         problem, failure.residual_pixels);
}

}  // namespace

std::ostream &operator<<(std::ostream &os, const CalibrationReport &report) {
  os << absl::StrFormat(
      "%dx%d: farthest point (%g, %g) at distorted radius %.4f, undistorted "
      "%.4f; principal branch runs to r %.4f (%s), f %.4f; N / D positive to "
      "r %.4f (%s); margins %.1f%% distorted, %.1f%% undistorted",
      report.width, report.height, report.max_distorted_radius_u,
      report.max_distorted_radius_v, report.max_distorted_radius,
      report.max_undistorted_radius, report.limit_radius, report.limit_reason,
      report.limit_distorted_radius, report.singular_radius,
      report.singular_reason, 100 * report.distorted_margin,
      100 * report.undistorted_margin);
  if (report.pixels_checked > 0) {
    os << absl::StrFormat(
        "; %d of %d points fail (%d gave up, %d slow, %d off the branch, %d "
        "inexact), at most %d steps at (%g, %g), largest residual %.3g px "
        "(limit %.3g) at (%g, %g)",
        report.failing_pixels(), report.pixels_checked,
        report.unconverged_pixels, report.slow_pixels, report.off_branch_pixels,
        report.inexact_pixels, report.max_iterations, report.max_iterations_u,
        report.max_iterations_v, report.max_residual_pixels,
        report.residual_limit_pixels, report.max_residual_u,
        report.max_residual_v);
    if (report.failing_pixels() > 0) {
      os << absl::StrFormat(
          ", failures from %.0f px of the principal point to %.0f px inside "
          "the edges",
          report.nearest_failure_pixels, report.deepest_failure_pixels);
    }
  }
  return os;
}

absl::Status ValidateCalibration(
    const calibration::CameraCalibration &calibration, int width, int height,
    CalibrationReport *report) {
  CalibrationReport local_report;
  if (report == nullptr) {
    report = &local_report;
  }
  *report = CalibrationReport();
  report->width = width;
  report->height = height;

  if (absl::Status status = CheckSanity(calibration, width, height);
      !status.ok()) {
    report->failed_check = CalibrationCheck::kSanity;
    return status;
  }
  const apriltag::CameraMatrix camera_matrix =
      apriltag::GetCameraMatrix(&calibration);
  const apriltag::DistCoeffs dist_coeffs =
      apriltag::GetDistCoeffs(&calibration);

  for (const int u : {-1, width}) {
    for (const int v : {-1, height}) {
      const double radius =
          std::hypot((u - camera_matrix.cx) / camera_matrix.fx,
                     (v - camera_matrix.cy) / camera_matrix.fy);
      if (radius > report->max_distorted_radius) {
        report->max_distorted_radius = radius;
        report->max_distorted_radius_u = u;
        report->max_distorted_radius_v = v;
      }
    }
  }

  std::vector<std::string> problems;
  if (dist_coeffs.model == apriltag::DistortionModel::kKannalaBrandt) {
    ScanKannalaBrandt(dist_coeffs, report);
  } else {
    ScanRadialModel(dist_coeffs, report);
  }
  if (!(report->distorted_margin >= kRadialMargin &&
        report->undistorted_margin >= kRadialMargin)) {
    report->failed_check = CalibrationCheck::kRadialModel;
    problems.push_back(absl::StrFormat(
        "radial model: the image's farthest point (%g, %g) is at distorted "
        "radius %.4f (%.0f px) and undistorted %.4f.  The principal branch "
        "UnDistort must stay on ends at undistorted radius %.4f (%s), where "
        "the distorted radius reaches %.4f, %.1f%% past it.  N / D stays "
        "finite and positive to undistorted radius %.4f (%s), %.1f%% past the "
        "larger of the two, since UnDistort starts from the distorted point.  "
        "Both must be at least %.0f%%",
        report->max_distorted_radius_u, report->max_distorted_radius_v,
        report->max_distorted_radius,
        report->max_distorted_radius * camera_matrix.fx,
        report->max_undistorted_radius, report->limit_radius,
        report->limit_reason, report->limit_distorted_radius,
        100 * report->distorted_margin, report->singular_radius,
        report->singular_reason, 100 * report->undistorted_margin,
        100 * kRadialMargin));
  }

  PixelStats stats = CheckPixels(camera_matrix, dist_coeffs,
                                 report->limit_radius, width, height);
  report->pixels_checked = stats.checked;
  report->max_iterations = stats.max_iterations;
  report->max_iterations_u = stats.max_iterations_u;
  report->max_iterations_v = stats.max_iterations_v;
  report->max_residual_pixels = std::sqrt(stats.max_residual_squared);
  report->max_residual_u = stats.max_residual_u;
  report->max_residual_v = stats.max_residual_v;
  report->residual_limit_pixels = MaxResidualPixels(camera_matrix);
  report->unconverged_pixels = stats.unconverged;
  report->slow_pixels = stats.slow;
  report->off_branch_pixels = stats.off_branch;
  report->inexact_pixels = stats.inexact;
  report->failures_by_quadrant = stats.failures_by_quadrant;
  report->nearest_failure_pixels = stats.nearest_failure_pixels;
  report->deepest_failure_pixels = stats.deepest_failure_pixels;
  report->failures = std::move(stats.failures);
  if (report->failing_pixels() > 0) {
    if (report->failed_check == CalibrationCheck::kNone) {
      report->failed_check = CalibrationCheck::kPixels;
    }
    std::vector<std::string> examples;
    for (const PixelFailure &failure : report->failures) {
      examples.push_back(DescribeFailure(failure, *report));
    }
    problems.push_back(absl::StrFormat(
        "pixels: UnDistort fails at %d of %d points, %d above left, %d above "
        "right, %d below left and %d below right of the principal point, the "
        "nearest %.0f px from it and the deepest %.0f px inside the image's "
        "edges.  %d gave up, %d took over %d steps, %d converged off the "
        "principal branch, and %d converged more than %.3g px off.  The "
        "first: %s",
        report->failing_pixels(), report->pixels_checked,
        report->failures_by_quadrant[0], report->failures_by_quadrant[1],
        report->failures_by_quadrant[2], report->failures_by_quadrant[3],
        report->nearest_failure_pixels, report->deepest_failure_pixels,
        report->unconverged_pixels, report->slow_pixels,
        kMaxUnDistortIterations, report->off_branch_pixels,
        report->inexact_pixels, report->residual_limit_pixels,
        absl::StrJoin(examples, ", ")));
  }

  if (problems.empty()) {
    return absl::OkStatus();
  }
  return absl::InvalidArgumentError(absl::StrJoin(problems, "\n"));
}

}  // namespace frc::vision
