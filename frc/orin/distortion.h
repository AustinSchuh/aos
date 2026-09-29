#ifndef FRC_ORIN_DISTORTION_H_
#define FRC_ORIN_DISTORTION_H_

#include <cmath>

#include "frc/vision/calibration_generated.h"

// OpenCV's pinhole camera model with its 8 parameter rational distortion, as
// the AprilTag detector applies and inverts it.  Everything here is inline so
// the detector's CUDA translation units compile it in place, and
// frc/vision/calibration/calibration_validator.h checks a calibration with the
// same code.
namespace frc::apriltag {

struct CameraMatrix {
  double fx;
  double cx;
  double fy;
  double cy;
};

struct DistCoeffs {
  double k1;
  double k2;
  double p1;
  double p2;
  double k3;
  double k4;
  double k5;
  double k6;
  // Are we using 5 or 8 parameter model?
  int num_params;
};

// Returns calibration's intrinsics in the forms the detector takes.
// gpu_apriltag and apriltag_benchmark both use these, so the benchmark
// undistorts the way the robot does.
inline CameraMatrix GetCameraMatrix(
    const frc::vision::calibration::CameraCalibration *calibration) {
  auto intrinsics = calibration->intrinsics();
  return CameraMatrix{
      .fx = intrinsics->Get(0),
      .cx = intrinsics->Get(2),
      .fy = intrinsics->Get(4),
      .cy = intrinsics->Get(5),
  };
}

// Only the first 8 coefficients: the thin prism and tilt terms of OpenCV's 14
// parameter model are ignored.
inline DistCoeffs GetDistCoeffs(
    const frc::vision::calibration::CameraCalibration *calibration) {
  auto dist_coeffs = calibration->dist_coeffs();
  int num_params = dist_coeffs->size();
  return DistCoeffs{.k1 = dist_coeffs->Get(0),
                    .k2 = dist_coeffs->Get(1),
                    .p1 = dist_coeffs->Get(2),
                    .p2 = dist_coeffs->Get(3),
                    .k3 = dist_coeffs->Get(4),
                    .k4 = (num_params >= 8 ? dist_coeffs->Get(5) : 0.0),
                    .k5 = (num_params >= 8 ? dist_coeffs->Get(6) : 0.0),
                    .k6 = (num_params >= 8 ? dist_coeffs->Get(7) : 0.0),
                    .num_params = num_params};
}

// UnDistort gives up once it has taken more than this many steps, and stops
// after a step which moves both normalized coordinates by at most
// kUndistortConvergenceEpsilon.
inline constexpr int kUndistortIterationThreshold = 100;
inline constexpr double kUndistortConvergenceEpsilon = 1e-6;

// One step of UnDistort's fixed point iteration: moves (*x, *y) closer to the
// normalized point which distorts to (x0, y0).
inline void UnDistortStep(const DistCoeffs &distortion_coefficients, double x0,
                          double y0, double *x, double *y) {
  const double k1 = distortion_coefficients.k1;
  const double k2 = distortion_coefficients.k2;
  const double p1 = distortion_coefficients.p1;
  const double p2 = distortion_coefficients.p2;
  const double k3 = distortion_coefficients.k3;
  const double k4 = distortion_coefficients.k4;
  const double k5 = distortion_coefficients.k5;
  const double k6 = distortion_coefficients.k6;
  const double xP = *x;
  const double yP = *y;

  double rSq = xP * xP + yP * yP;

  double radial_distortion =
      (1 + (k1 * rSq) + (k2 * rSq * rSq) + (k3 * rSq * rSq * rSq)) /
      (1 + (k4 * rSq) + (k5 * rSq * rSq) + (k6 * rSq * rSq * rSq));

  double radial_distortion_inv = 1 / radial_distortion;

  double tangential_dx = 2 * p1 * xP * yP + p2 * (rSq + 2 * xP * xP);
  double tangential_dy = p1 * (rSq + 2 * yP * yP) + 2 * p2 * xP * yP;

  *x = (x0 - tangential_dx) * radial_distortion_inv;
  *y = (y0 - tangential_dy) * radial_distortion_inv;
}

// Moves the pixel (*u, *v) to where it would be without distortion, starting
// OpenCV's fixed point iteration from the distorted point itself.  Returns
// false if it gives up, and sets *iterations, if given, to how many steps it
// took before stopping.
//
// We're undistorting using math found from this github page
// https://yangyushi.github.io/code/2020/03/04/opencv-undistort.html
// Also, see for reference the source code:
// https://github.com/opencv/opencv/blob/master/modules/calib3d/src/undistort.dispatch.cpp
inline bool UnDistort(double *u, double *v, const CameraMatrix &camera_matrix,
                      const DistCoeffs &distortion_coefficients,
                      int *iterations_out = nullptr) {
  bool converged = true;

  const double fx = camera_matrix.fx;
  const double cx = camera_matrix.cx;
  const double fy = camera_matrix.fy;
  const double cy = camera_matrix.cy;

  const double xPP = (*u - cx) / fx;
  const double yPP = (*v - cy) / fy;

  double xP = xPP;
  double yP = yPP;

  double x0 = xP;
  double y0 = yP;

  double prev_x = 0, prev_y = 0;

  int iterations = 0;
  do {
    prev_x = xP;
    prev_y = yP;

    UnDistortStep(distortion_coefficients, x0, y0, &xP, &yP);

    if (iterations > kUndistortIterationThreshold) {
      converged = false;
      break;
    }

    iterations++;
  } while (std::abs(xP - prev_x) > kUndistortConvergenceEpsilon ||
           std::abs(yP - prev_y) > kUndistortConvergenceEpsilon);

  *u = xP * fx + cx;
  *v = yP * fy + cy;
  if (iterations_out != nullptr) {
    *iterations_out = iterations;
  }

  return converged;
}

// (Re-)distorts the pixel (*x, *y) with the pinhole camera model.
// Algorithm mainly taken from
// https://docs.opencv.org/4.0.0/d9/d0c/group__calib3d.html#ga7dfb72c9cf9780a347fbe3d1c47e5d5a
inline void ReDistort(double *x, double *y, const CameraMatrix &camera_matrix,
                      const DistCoeffs &distortion_coefficients) {
  double k1 = distortion_coefficients.k1;
  double k2 = distortion_coefficients.k2;
  double p1 = distortion_coefficients.p1;
  double p2 = distortion_coefficients.p2;
  double k3 = distortion_coefficients.k3;
  double k4 = distortion_coefficients.k4;
  double k5 = distortion_coefficients.k5;
  double k6 = distortion_coefficients.k6;

  double fx = camera_matrix.fx;
  double cx = camera_matrix.cx;
  double fy = camera_matrix.fy;
  double cy = camera_matrix.cy;

  double xP = (*x - cx) / fx;
  double yP = (*y - cy) / fy;

  double rSq = xP * xP + yP * yP;

  double linCoef = (1 + k1 * rSq + k2 * rSq * rSq + k3 * rSq * rSq * rSq) /
                   (1 + k4 * rSq + k5 * rSq * rSq + k6 * rSq * rSq * rSq);
  double xPP = xP * linCoef + 2 * p1 * xP * yP + p2 * (rSq + 2 * xP * xP);
  double yPP = yP * linCoef + p1 * (rSq + 2 * yP * yP) + 2 * p2 * xP * yP;

  *x = xPP * fx + cx;
  *y = yPP * fy + cy;
}

}  // namespace frc::apriltag

#endif  // FRC_ORIN_DISTORTION_H_
