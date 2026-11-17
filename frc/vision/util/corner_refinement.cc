#include "frc/vision/util/corner_refinement.h"

#include <algorithm>
#include <cmath>
#include <optional>

#include "opencv2/imgproc.hpp"

namespace frc::vision {
namespace {

cv::Mat Gray(const cv::Mat &image) {
  if (image.channels() == 1) {
    return image;
  }
  cv::Mat gray;
  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  return gray;
}

// The median distance from each corner to its nearest: a square's size.
double SquareSize(const std::vector<cv::Point2f> &corners) {
  std::vector<double> nearest;
  for (size_t i = 0; i < corners.size(); ++i) {
    double best = INFINITY;
    for (size_t j = 0; j < corners.size(); ++j) {
      if (i != j) {
        best = std::min<double>(best, cv::norm(corners[i] - corners[j]));
      }
    }
    nearest.push_back(best);
  }
  std::nth_element(nearest.begin(), nearest.begin() + nearest.size() / 2,
                   nearest.end());
  return nearest[nearest.size() / 2];
}

// Fits I(x, y) = a x² + b x y + c y² + d x + e y + f, weighted, to `patch`,
// centred on (0, 0), and returns the stationary point, or nullopt if it isn't
// a saddle.
std::optional<cv::Point2d> SaddleOf(const cv::Mat &patch,
                                    const cv::Mat &weights) {
  const int half = patch.cols / 2;
  // The weighted normal equations, accumulated directly: 6x6.
  cv::Matx<double, 6, 6> A = cv::Matx<double, 6, 6>::zeros();
  cv::Vec<double, 6> b = cv::Vec<double, 6>::all(0);
  for (int row = 0; row < patch.rows; ++row) {
    const double y = row - half;
    for (int column = 0; column < patch.cols; ++column) {
      const double x = column - half;
      const double w = weights.at<double>(row, column);
      const double basis[6] = {x * x, x * y, y * y, x, y, 1.0};
      const double value = patch.at<float>(row, column);
      for (int i = 0; i < 6; ++i) {
        b[i] += w * basis[i] * value;
        for (int j = i; j < 6; ++j) {
          A(i, j) += w * basis[i] * basis[j];
        }
      }
    }
  }
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < i; ++j) {
      A(i, j) = A(j, i);
    }
  }
  cv::Vec<double, 6> p;
  if (!cv::solve(A, b, p, cv::DECOMP_CHOLESKY)) {
    return std::nullopt;
  }
  // Gradient zero: [2a b; b 2c] [x; y] = -[d; e].
  const double h00 = 2 * p[0], h01 = p[1], h11 = 2 * p[2];
  const double det = h00 * h11 - h01 * h01;
  if (!(det < 0)) {
    // Not a saddle: a blob, an edge, or flat.
    return std::nullopt;
  }
  return cv::Point2d((-p[3] * h11 + p[4] * h01) / det,
                     (-p[4] * h00 + p[3] * h01) / det);
}

}  // namespace

void RefineSaddlePoints(const cv::Mat &image, std::vector<cv::Point2f> &corners,
                        const SaddlePointOptions &options) {
  if (corners.empty()) {
    return;
  }
  const cv::Mat gray = Gray(image);
  const int half = std::max(2, options.half_window);

  // Close first, so the fit starts within its window.
  cv::cornerSubPix(
      gray, corners, cv::Size(half, half), cv::Size(-1, -1),
      cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 100,
                       1e-4));

  cv::Mat blurred;
  gray.convertTo(blurred, CV_32F);
  cv::GaussianBlur(blurred, blurred, cv::Size(0, 0), options.blur_sigma);

  cv::Mat weights(2 * half + 1, 2 * half + 1, CV_64F);
  const double sigma = half / 2.0;
  for (int row = 0; row < weights.rows; ++row) {
    for (int column = 0; column < weights.cols; ++column) {
      const double x = column - half, y = row - half;
      weights.at<double>(row, column) =
          std::exp(-(x * x + y * y) / (2 * sigma * sigma));
    }
  }

  for (cv::Point2f &corner : corners) {
    const cv::Point2f start = corner;
    cv::Point2d at = start;
    bool ok = true;
    for (int i = 0; i < options.max_iterations; ++i) {
      cv::Mat patch;
      cv::getRectSubPix(blurred, weights.size(), cv::Point2f(at), patch,
                        CV_32F);
      const std::optional<cv::Point2d> step = SaddleOf(patch, weights);
      if (!step.has_value() || cv::norm(*step) > half) {
        ok = false;
        break;
      }
      at += *step;
      if (cv::norm(*step) < options.tolerance) {
        break;
      }
    }
    if (ok && cv::norm(at - cv::Point2d(start)) < half / 2.0) {
      corner = cv::Point2f(at);
    }
  }
}

void RefineCharucoCorners(const cv::Ptr<cv::aruco::CharucoBoard> &board,
                          const cv::Mat &image,
                          std::vector<cv::Point2f> &corners) {
  if (corners.size() < 2) {
    return;
  }
  // The window stays inside the white margin around the markers, which is
  // (square - marker) / 2 from the corner, so their edges don't pull on it.
  const double margin =
      (1.0 - board->getMarkerLength() / board->getSquareLength()) / 2.0;
  SaddlePointOptions options;
  options.half_window =
      std::clamp(static_cast<int>(SquareSize(corners) * margin * 0.9), 2, 8);
  // Blur in step with the window: enough to round the corner into a saddle
  // across it, not so much the markers' edges bleed in.
  options.blur_sigma = std::clamp(0.45 * options.half_window, 0.8, 2.0);
  RefineSaddlePoints(image, corners, options);
}

}  // namespace frc::vision
