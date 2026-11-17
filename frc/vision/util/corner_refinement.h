#ifndef FRC_VISION_UTIL_CORNER_REFINEMENT_H_
#define FRC_VISION_UTIL_CORNER_REFINEMENT_H_

#include <vector>

#include "opencv2/aruco/charuco.hpp"
#include "opencv2/core.hpp"

namespace frc::vision {

// How to refine chessboard corners.
struct SaddlePointOptions {
  // The blur that makes a corner's sharp edges a smooth saddle a quadratic
  // fits, as a standard deviation in pixels.
  double blur_sigma = 1.0;
  // The fit's window: points within this many pixels of the corner, weighted
  // by a Gaussian of half that.  At least 2.
  int half_window = 3;
  int max_iterations = 20;
  // Stop once a step is shorter than this, in pixels.
  double tolerance = 1e-4;
};

// Refines chessboard corners in `image` (grayscale or BGR) to the saddle
// point of a quadratic fitted to the blurred intensity around each: where a
// checkerboard's dark and light squares meet, the blurred image is a saddle,
// and its stationary point is the corner.  Each corner starts from where it
// is, refined first with cornerSubPix to get close.  A corner whose fit isn't
// a saddle, or walks off, keeps its cornerSubPix place.
void RefineSaddlePoints(const cv::Mat &image, std::vector<cv::Point2f> &corners,
                        const SaddlePointOptions &options);

// Refines the inner corners of `board` found in `image`, as
// interpolateCornersCharuco leaves them: about half a pixel off in x and y in
// OpenCV 4.6.  A shift every corner shares leaves no reprojection error, but
// moves a calibration's principal point by as much.  Picks the window from
// the board, to stay clear of its markers, and blurs in step with it.  Finds
// corners to 0.02-0.03 px in rendered images, where cornerSubPix alone finds
// them to 0.05-0.15 px.
void RefineCharucoCorners(const cv::Ptr<cv::aruco::CharucoBoard> &board,
                          const cv::Mat &image,
                          std::vector<cv::Point2f> &corners);

}  // namespace frc::vision

#endif  // FRC_VISION_UTIL_CORNER_REFINEMENT_H_
