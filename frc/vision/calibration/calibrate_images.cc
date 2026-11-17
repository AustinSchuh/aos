// calibrate_images: calibrates a camera's intrinsics from a directory of
// images of a ChArUco board, in each lens model asked for, and writes each
// calibration and a report comparing them.
//
//   bazel run -c opt //frc/vision/calibration:calibrate_images -- \
//       --images=/path/to/frames --squares_x=12 --squares_y=9 \
//       --dictionary=DICT_5X5_1000 --output=/tmp/calibration
//
// The marker's size relative to its square is measured from the images unless
// --marker_length gives it.  The board's absolute size (--square_length) only
// scales the report's millimeters: intrinsics don't depend on it.
//
// Each model is solved from every frame, for the calibration, and from half
// the frames (every other one), measured against the rest: a model fitting
// its frames' noise shows there, not in its own error.  The report
// (report.html, with its data in report.json) shows each model's fit,
// validation, error by radius, rigidity, and distortion, a frame undistorted
// by each, and where in the image the board's corners were.

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>

#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/strings/str_replace.h"
#include "absl/strings/str_split.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "frc/vision/calibration/calibrate_images_report.h"
#include "frc/vision/calibration/calibration_validator.h"
#include "frc/vision/calibration/charuco_detector.h"
#include "frc/vision/calibration/intrinsics_calibration_lib.h"
#include "frc/vision/calibration/intrinsics_solver.h"
#include "frc/vision/calibration/validation.h"
#include "frc/vision/util/charuco_board.h"

ABSL_FLAG(std::string, images, "",
          "A directory of images (.png, .jpg, .jpeg, .bmp) of the board.");
ABSL_FLAG(int32_t, squares_x, 14, "The board's squares across.");
ABSL_FLAG(int32_t, squares_y, 9, "The board's squares down.");
ABSL_FLAG(double, square_length, 0.04,
          "A square's side, in meters, as measured on the printed board.  Only "
          "scales the report's millimeters.");
ABSL_FLAG(double, marker_length, 0.0,
          "A marker's side, in meters; 0 to measure it from the images.");
ABSL_FLAG(std::string, dictionary, "DICT_5X5_100",
          "The markers' ArUco dictionary, by OpenCV's name.");
ABSL_FLAG(
    std::string, models, "kannala_brandt,opencv_rational",
    "The lens models to solve: kannala_brandt, opencv_rational, opencv5.");
ABSL_FLAG(std::string, output, "",
          "Where to write the calibrations and the report.");
ABSL_FLAG(std::string, camera_id, "00-00",
          "The camera's id, YY-NN, for the calibrations.");
ABSL_FLAG(std::string, title, "",
          "The report's title; the images' directory's name if empty.");

namespace frc::vision {
namespace {

namespace fs = std::filesystem;

// A tiny JSON writer: enough for the report's numbers, strings and arrays.
class Json {
 public:
  Json &Raw(std::string_view text) {
    out_ << text;
    return *this;
  }
  static std::string Quote(std::string_view s) {
    std::string out = "\"";
    for (char c : s) {
      switch (c) {
        case '"':
          out += "\\\"";
          break;
        case '\\':
          out += "\\\\";
          break;
        case '\n':
          out += "\\n";
          break;
        case '<':
          // Keeps "</script>" out of the page the JSON is embedded in.
          out += "\\u003c";
          break;
        default:
          out += c;
      }
    }
    return out + "\"";
  }
  static std::string Number(double x) {
    if (!std::isfinite(x)) {
      return "null";
    }
    std::ostringstream s;
    s.precision(7);
    s << x;
    return s.str();
  }
  template <typename T, typename F>
  static std::string Array(const std::vector<T> &values, F &&format) {
    std::vector<std::string> items;
    for (const T &value : values) {
      items.push_back(format(value));
    }
    return "[" + absl::StrJoin(items, ",") + "]";
  }
  static std::string Numbers(const std::vector<double> &values) {
    return Array(values, [](double x) { return Number(x); });
  }

 private:
  std::ostringstream out_;
};

std::string Base64(const std::vector<uchar> &bytes) {
  static constexpr char kAlphabet[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  size_t i = 0;
  for (; i + 2 < bytes.size(); i += 3) {
    const uint32_t n = (bytes[i] << 16) | (bytes[i + 1] << 8) | bytes[i + 2];
    out += kAlphabet[n >> 18];
    out += kAlphabet[(n >> 12) & 63];
    out += kAlphabet[(n >> 6) & 63];
    out += kAlphabet[n & 63];
  }
  if (i + 1 == bytes.size()) {
    const uint32_t n = bytes[i] << 16;
    out += kAlphabet[n >> 18];
    out += kAlphabet[(n >> 12) & 63];
    out += "==";
  } else if (i + 2 == bytes.size()) {
    const uint32_t n = (bytes[i] << 16) | (bytes[i + 1] << 8);
    out += kAlphabet[n >> 18];
    out += kAlphabet[(n >> 12) & 63];
    out += kAlphabet[(n >> 6) & 63];
    out += "=";
  }
  return out;
}

// `image`, `width` px wide, as a JPEG data URI.
std::string DataUri(const cv::Mat &image, int width) {
  cv::Mat small;
  cv::resize(image, small,
             cv::Size(width, image.rows * width / std::max(1, image.cols)), 0,
             0, cv::INTER_AREA);
  std::vector<uchar> jpeg;
  cv::imencode(".jpg", small, jpeg, {cv::IMWRITE_JPEG_QUALITY, 82});
  return "data:image/jpeg;base64," + Base64(jpeg);
}

std::vector<fs::path> ImageFiles(const fs::path &directory) {
  std::vector<fs::path> files;
  for (const fs::directory_entry &entry : fs::directory_iterator(directory)) {
    std::string extension = entry.path().extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   ::tolower);
    const std::string name = entry.path().filename().string();
    if (entry.is_regular_file() && !name.starts_with("._") &&
        (extension == ".png" || extension == ".jpg" || extension == ".jpeg" ||
         extension == ".bmp")) {
      files.push_back(entry.path());
    }
  }
  std::sort(files.begin(), files.end());
  return files;
}

// The markers' side as a fraction of the squares', measured from the images:
// for each image, a homography from the board's squares (where each marker's
// center is, whatever its size) to the image, through which the markers'
// corners map back to the board, where their sides are measured.
std::optional<double> MeasureMarkerRatio(const std::vector<cv::Mat> &images,
                                         const BoardSpec &spec) {
  BoardSpec provisional = spec;
  provisional.square_length = 1.0;
  provisional.marker_length = 0.7;
  const cv::Ptr<cv::aruco::CharucoBoard> board = MakeCharucoBoard(provisional);
#if CV_VERSION_MINOR >= 9
  const std::vector<std::vector<cv::Point3f>> object = board->getObjPoints();
  const std::vector<int> board_ids = board->getIds();
  const cv::Ptr<cv::aruco::Dictionary> dictionary =
      cv::makePtr<cv::aruco::Dictionary>(board->getDictionary());
#else
  const std::vector<std::vector<cv::Point3f>> object = board->objPoints;
  const std::vector<int> board_ids = board->ids;
  const cv::Ptr<cv::aruco::Dictionary> dictionary = board->dictionary;
#endif
  std::map<int, cv::Point2f> centers;
  for (size_t i = 0; i < board_ids.size(); ++i) {
    cv::Point2f center(0, 0);
    for (const cv::Point3f &p : object[i]) {
      center += cv::Point2f(p.x, p.y) / 4.0f;
    }
    centers[board_ids[i]] = center;
  }

  std::vector<double> ratios;
  for (const cv::Mat &image : images) {
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(image, dictionary, corners, ids);
    std::vector<cv::Point2f> board_points, image_points;
    for (size_t i = 0; i < ids.size(); ++i) {
      auto it = centers.find(ids[i]);
      if (it == centers.end()) {
        continue;
      }
      board_points.push_back(it->second);
      cv::Point2f center(0, 0);
      for (const cv::Point2f &p : corners[i]) {
        center += p / 4.0f;
      }
      image_points.push_back(center);
    }
    if (board_points.size() < 8) {
      continue;
    }
    const cv::Mat H =
        cv::findHomography(image_points, board_points, cv::RANSAC);
    if (H.empty()) {
      continue;
    }
    for (size_t i = 0; i < ids.size(); ++i) {
      if (!centers.contains(ids[i])) {
        continue;
      }
      std::vector<cv::Point2f> on_board;
      cv::perspectiveTransform(corners[i], on_board, H);
      double side = 0;
      for (int k = 0; k < 4; ++k) {
        side += cv::norm(on_board[k] - on_board[(k + 1) % 4]) / 4.0;
      }
      ratios.push_back(side);
    }
  }
  if (ratios.size() < 20) {
    return std::nullopt;
  }
  std::nth_element(ratios.begin(), ratios.begin() + ratios.size() / 2,
                   ratios.end());
  return ratios[ratios.size() / 2];
}

struct Frame {
  std::string name;
  CharucoCapture capture;
};

std::optional<LensModel> ParseModel(std::string_view name) {
  if (name == "kannala_brandt") return LensModel::kKannalaBrandt;
  if (name == "opencv_rational") return LensModel::kOpenCvRational;
  if (name == "opencv5") return LensModel::kOpenCv5;
  return std::nullopt;
}

std::string ModelLabel(LensModel model) {
  switch (model) {
    case LensModel::kKannalaBrandt:
      return "Kannala-Brandt";
    case LensModel::kOpenCvRational:
      return "OpenCV rational";
    case LensModel::kOpenCv5:
      return "OpenCV 5-term";
  }
  return "";
}

// Where each pixel of a grid over the image lands undistorted, and whether
// distorting it again comes back: where it doesn't, the model doesn't invert.
struct Field {
  int columns, rows;
  std::vector<double> dx, dy;
};

Field DistortionField(const CameraLens &lens, cv::Size size, int columns) {
  Field field{columns, std::max(1, columns * size.height / size.width), {}, {}};
  std::vector<cv::Point2f> pixels;
  for (int row = 0; row < field.rows; ++row) {
    for (int column = 0; column < field.columns; ++column) {
      pixels.emplace_back((column + 0.5) * size.width / field.columns,
                          (row + 0.5) * size.height / field.rows);
    }
  }
  const std::vector<cv::Point2f> undistorted = lens.Undistort(pixels);
  // Back through the lens: the undistorted pixels as points at depth 1.
  const double fx = lens.camera_matrix.at<double>(0, 0);
  const double fy = lens.camera_matrix.at<double>(1, 1);
  const double cx = lens.camera_matrix.at<double>(0, 2);
  const double cy = lens.camera_matrix.at<double>(1, 2);
  std::vector<cv::Point3f> rays;
  for (const cv::Point2f &p : undistorted) {
    rays.emplace_back((p.x - cx) / fx, (p.y - cy) / fy, 1.0f);
  }
  const std::vector<cv::Point2f> back =
      lens.Project(rays, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0));
  for (size_t i = 0; i < pixels.size(); ++i) {
    const bool inverts =
        std::isfinite(undistorted[i].x) && cv::norm(back[i] - pixels[i]) < 0.5;
    field.dx.push_back(inverts ? undistorted[i].x - pixels[i].x : NAN);
    field.dy.push_back(inverts ? undistorted[i].y - pixels[i].y : NAN);
  }
  return field;
}

std::string FieldJson(const Field &field) {
  return absl::StrCat("{\"columns\":", field.columns, ",\"rows\":", field.rows,
                      ",\"dx\":", Json::Numbers(field.dx),
                      ",\"dy\":", Json::Numbers(field.dy), "}");
}

int Main() {
  const fs::path directory = absl::GetFlag(FLAGS_images);
  const fs::path output = absl::GetFlag(FLAGS_output);
  ABSL_CHECK(!directory.empty()) << "--images is needed";
  ABSL_CHECK(!output.empty()) << "--output is needed";
  fs::create_directories(output);

  const std::vector<fs::path> files = ImageFiles(directory);
  ABSL_CHECK(!files.empty()) << "No images in " << directory;
  std::vector<cv::Mat> images;
  for (const fs::path &file : files) {
    images.push_back(cv::imread(file.string(), cv::IMREAD_GRAYSCALE));
    ABSL_CHECK(!images.back().empty()) << "Couldn't read " << file;
  }
  const cv::Size size = images[0].size();
  ABSL_LOG(INFO) << files.size() << " images, " << size.width << "x"
                 << size.height;

  BoardSpec spec{.squares_x = absl::GetFlag(FLAGS_squares_x),
                 .squares_y = absl::GetFlag(FLAGS_squares_y),
                 .square_length = absl::GetFlag(FLAGS_square_length),
                 .marker_length = absl::GetFlag(FLAGS_marker_length),
                 .dictionary = absl::GetFlag(FLAGS_dictionary)};
  bool measured = false;
  if (spec.marker_length <= 0) {
    const std::optional<double> ratio = MeasureMarkerRatio(images, spec);
    ABSL_CHECK(ratio.has_value())
        << "Couldn't find enough markers to measure them; give "
           "--marker_length";
    spec.marker_length = spec.square_length * *ratio;
    measured = true;
    ABSL_LOG(INFO) << "Markers measure " << *ratio << " of a square";
  }
  if (const absl::Status valid = ValidateBoardSpec(spec); !valid.ok()) {
    ABSL_LOG(FATAL) << valid;
  }
  const cv::Ptr<cv::aruco::CharucoBoard> board = MakeCharucoBoard(spec);

  // Detect, from a 60 degree pinhole guess: the corners don't depend on it.
  const double f = size.width * 0.87;
  const CameraLens start{
      .camera_matrix = (cv::Mat_<double>(3, 3) << f, 0, size.width / 2.0, 0, f,
                        size.height / 2.0, 0, 0, 1),
      .dist_coeffs = cv::Mat::zeros(5, 1, CV_64F),
      .kannala_brandt = false};
  std::vector<Frame> frames;
  std::vector<std::string> rejected;
  std::vector<int> frame_image;
  for (size_t i = 0; i < images.size(); ++i) {
    const CharucoDetection detection = DetectCharuco(board, images[i], start);
    if (static_cast<int>(detection.ids.size()) < kMinCharucoCorners) {
      rejected.push_back(files[i].filename().string());
      continue;
    }
    frames.push_back(
        {files[i].filename().string(), {detection.ids, detection.corners}});
    frame_image.push_back(i);
  }
  ABSL_LOG(INFO) << frames.size() << " frames with the board, "
                 << rejected.size() << " without";
  ABSL_CHECK_GE(frames.size(), 8u) << "Too few frames with the board";
  std::vector<CharucoCapture> all, train, test;
  for (size_t i = 0; i < frames.size(); ++i) {
    all.push_back(frames[i].capture);
    (i % 2 == 0 ? train : test).push_back(frames[i].capture);
  }

  // Where the corners were, and how far out they reach.
  const cv::Point2f center(size.width / 2.0f, size.height / 2.0f);
  const double half_diagonal = std::hypot(size.width, size.height) / 2.0;
  constexpr int kCoverageColumns = 32;
  const int coverage_rows = kCoverageColumns * size.height / size.width;
  std::vector<double> coverage(kCoverageColumns * coverage_rows, 0);
  double farthest = 0;
  std::vector<double> corner_x, corner_y;
  for (const Frame &frame : frames) {
    for (const cv::Point2f &p : frame.capture.corners) {
      const int column = std::clamp<int>(p.x / size.width * kCoverageColumns, 0,
                                         kCoverageColumns - 1);
      const int row = std::clamp<int>(p.y / size.height * coverage_rows, 0,
                                      coverage_rows - 1);
      coverage[row * kCoverageColumns + column] += 1;
      farthest = std::max(farthest, cv::norm(p - center) / half_diagonal);
      corner_x.push_back(p.x);
      corner_y.push_back(p.y);
    }
  }

  // The frame whose corners reach farthest out, to show undistorted.
  size_t showcase = 0;
  double showcase_reach = -1;
  for (size_t i = 0; i < frames.size(); ++i) {
    double reach = 0;
    for (const cv::Point2f &p : frames[i].capture.corners) {
      reach = std::max(reach, cv::norm(p - center));
    }
    if (frames[i].capture.corners.size() > 40 && reach > showcase_reach) {
      showcase_reach = reach;
      showcase = i;
    }
  }
  const cv::Mat &showcase_image = images[frame_image[showcase]];
  cv::Mat annotated;
  cv::cvtColor(showcase_image, annotated, cv::COLOR_GRAY2BGR);
  for (const cv::Point2f &p : frames[showcase].capture.corners) {
    cv::circle(annotated, p, 4, cv::Scalar(52, 180, 235), -1, cv::LINE_AA);
  }

  constexpr int kBins = 5;
  std::vector<std::string> model_json;
  std::optional<CameraLens> first_lens;
  for (std::string_view name :
       absl::StrSplit(absl::GetFlag(FLAGS_models), ',', absl::SkipEmpty())) {
    const std::optional<LensModel> model = ParseModel(name);
    ABSL_CHECK(model.has_value()) << "No model is named " << name;
    const SolveOptions options{.model = *model, .denominator_sigma = 0.0};
    absl::StatusOr<IntrinsicsSolution> solution =
        SolveIntrinsics(board, all, size, start, options);
    absl::StatusOr<IntrinsicsSolution> half =
        SolveIntrinsics(board, train, size, start, options);
    if (!solution.ok() || !half.ok()) {
      ABSL_LOG(ERROR) << name << ": "
                      << (solution.ok() ? half.status() : solution.status());
      continue;
    }
    const CameraLens lens = SolutionLens(*solution);
    if (!first_lens.has_value()) {
      first_lens = lens;
    }

    // The calibration, and whether it validates.
    const calibration::DistortionModel distortion_model =
        *model == LensModel::kKannalaBrandt
            ? calibration::DistortionModel::KANNALA_BRANDT
            : calibration::DistortionModel::OPENCV;
    const aos::FlatbufferDetachedBuffer<calibration::CameraCalibration> built =
        IntrinsicsCalibration::BuildCalibration(
            solution->camera_matrix, solution->dist_coeffs,
            aos::realtime_clock::now(), "orin", "/camera0",
            absl::GetFlag(FLAGS_camera_id), 0, solution->reprojection_error,
            spec, distortion_model);
    CalibrationReport report;
    const absl::Status valid =
        ValidateCalibration(built.message(), size.width, size.height, &report);
    const fs::path calibration_file =
        output / absl::StrCat("calibration_", name, ".json");
    std::ofstream(calibration_file)
        << aos::FlatbufferToJson(built, {.multi_line = true});

    // On frames it wasn't solved from: error by radius, and rigidity.
    absl::StatusOr<std::vector<CornerResidual>> residuals =
        HeldOutResiduals(board, test, *half);
    ABSL_CHECK(residuals.ok()) << residuals.status();
    std::vector<double> sums(kBins, 0), counts(kBins, 0);
    double total = 0;
    for (const CornerResidual &r : *residuals) {
      const int bin = std::min<int>(
          kBins - 1, cv::norm(r.corner - center) / half_diagonal * kBins);
      sums[bin] += r.residual.dot(r.residual);
      counts[bin] += 1;
      total += r.residual.dot(r.residual);
    }
    std::vector<double> by_radius;
    for (int b = 0; b < kBins; ++b) {
      by_radius.push_back(counts[b] > 0 ? std::sqrt(sums[b] / counts[b]) : NAN);
    }
    std::vector<double> rigidity_mm;
    for (const CharucoCapture &capture : test) {
      for (bool across : {true, false}) {
        if (const auto d = BoardRigidity(board, capture.ids, capture.corners,
                                         SolutionLens(*half), across)) {
          rigidity_mm.push_back(d->translation_m * 1000.0);
        }
      }
    }
    std::sort(rigidity_mm.begin(), rigidity_mm.end());
    const auto percentile = [&](double p) {
      return rigidity_mm.empty()
                 ? NAN
                 : rigidity_mm[std::min(
                       rigidity_mm.size() - 1,
                       static_cast<size_t>(p * rigidity_mm.size()))];
    };

    // The showcase frame, undistorted by it.
    const cv::Mat undistorted = lens.UndistortImage(showcase_image);

    std::vector<double> dist;
    for (int i = 0; i < static_cast<int>(solution->dist_coeffs.total()); ++i) {
      dist.push_back(solution->dist_coeffs.at<double>(i));
    }
    const std::string validation_message(valid.message());
    model_json.push_back(absl::StrCat(
        "{\"key\":", Json::Quote(name),
        ",\"label\":", Json::Quote(ModelLabel(*model)),
        ",\"fx\":", Json::Number(solution->camera_matrix.at<double>(0, 0)),
        ",\"fy\":", Json::Number(solution->camera_matrix.at<double>(1, 1)),
        ",\"cx\":", Json::Number(solution->camera_matrix.at<double>(0, 2)),
        ",\"cy\":", Json::Number(solution->camera_matrix.at<double>(1, 2)),
        ",\"dist\":", Json::Numbers(dist),
        ",\"fit_rms\":", Json::Number(solution->reprojection_error),
        ",\"half_fit_rms\":", Json::Number(half->reprojection_error),
        ",\"held_out_rms\":",
        Json::Number(std::sqrt(total / std::max<size_t>(1, residuals->size()))),
        ",\"held_out_by_radius\":", Json::Numbers(by_radius),
        ",\"held_out_counts\":", Json::Numbers(counts),
        ",\"rigidity_median_mm\":", Json::Number(percentile(0.5)),
        ",\"rigidity_p90_mm\":", Json::Number(percentile(0.9)),
        ",\"valid\":", valid.ok() ? "true" : "false", ",\"validation\":",
        Json::Quote(
            validation_message.substr(0, validation_message.find('\n'))),
        ",\"capture_errors\":", Json::Numbers(solution->per_view_errors),
        ",\"field\":", FieldJson(DistortionField(lens, size, 40)),
        ",\"undistorted\":", Json::Quote(DataUri(undistorted, 640)),
        ",\"file\":", Json::Quote(calibration_file.filename().string()), "}"));
    ABSL_LOG(INFO) << ModelLabel(*model) << ": fit "
                   << solution->reprojection_error << " px, held out "
                   << std::sqrt(total / std::max<size_t>(1, residuals->size()))
                   << " px, " << (valid.ok() ? "valid" : "INVALID");
  }

  std::vector<std::string> names;
  for (const Frame &frame : frames) {
    names.push_back(frame.name);
  }
  std::string title = absl::GetFlag(FLAGS_title);
  if (title.empty()) {
    title = fs::absolute(directory).lexically_normal().filename().string();
    if (title.empty()) {
      title = fs::absolute(directory).parent_path().filename().string();
    }
  }
  // Every corner would be large; every fourth is enough to see coverage.
  std::vector<double> sampled_x, sampled_y;
  for (size_t i = 0; i < corner_x.size(); i += 4) {
    sampled_x.push_back(std::round(corner_x[i] * 10) / 10);
    sampled_y.push_back(std::round(corner_y[i] * 10) / 10);
  }
  const std::string json = absl::StrCat(
      "{\"title\":", Json::Quote(title), ",\"width\":", size.width,
      ",\"height\":", size.height, ",\"images\":", files.size(),
      ",\"frames\":", Json::Array(names, Json::Quote),
      ",\"rejected\":", Json::Array(rejected, Json::Quote),
      ",\"board\":{\"squares_x\":", spec.squares_x,
      ",\"squares_y\":", spec.squares_y,
      ",\"square_length\":", Json::Number(spec.square_length),
      ",\"marker_length\":", Json::Number(spec.marker_length),
      ",\"marker_measured\":", measured ? "true" : "false",
      ",\"dictionary\":", Json::Quote(spec.dictionary), "}",
      ",\"farthest\":", Json::Number(farthest),
      ",\"coverage\":{\"columns\":", kCoverageColumns,
      ",\"rows\":", coverage_rows, ",\"counts\":", Json::Numbers(coverage),
      "},\"corners_x\":", Json::Numbers(sampled_x),
      ",\"corners_y\":", Json::Numbers(sampled_y),
      ",\"showcase\":{\"name\":", Json::Quote(frames[showcase].name),
      ",\"image\":", Json::Quote(DataUri(annotated, 640)), "}", ",\"models\":[",
      absl::StrJoin(model_json, ","), "]}");
  std::ofstream(output / "report.json") << json;
  const std::string html_title = absl::StrReplaceAll(
      title, {{"&", "&amp;"}, {"<", "&lt;"}, {">", "&gt;"}});
  std::ofstream(output / "report.html") << absl::StrReplaceAll(
      kCalibrationReportHtml,
      {{"/*REPORT_DATA*/ null", json}, {"<!--REPORT_TITLE-->", html_title}});
  ABSL_LOG(INFO) << "Wrote " << (output / "report.html");
  return 0;
}

}  // namespace
}  // namespace frc::vision

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  return frc::vision::Main();
}
