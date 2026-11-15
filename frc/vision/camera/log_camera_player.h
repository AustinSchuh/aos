#ifndef FRC_VISION_CAMERA_LOG_CAMERA_PLAYER_H_
#define FRC_VISION_CAMERA_LOG_CAMERA_PLAYER_H_

#include <chrono>
#include <deque>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "aos/events/event_loop.h"
#include "aos/events/logging/log_reader.h"
#include "aos/events/logging/logfile_sorting.h"
#include "aos/events/logging/replay_channels.h"
#include "frc/vision/vision_generated.h"

namespace frc::vision {

// Plays the camera images in a log as live cameras: stands in for
// camera_reader on a robot without (all of) its cameras.
//
// Reads /cameraN CameraImages from the log in a simulation of its own, and
// sends each on |event_loop|'s /cameraN at the log's pace, as camera_reader
// would: a new capture time (when it is sent) and a new capture_uuid, and
// the logged image bytes, size and format.  At the end of the log it starts
// again from the beginning.
//
// Frames that come due more than |kMaxLateness| late (the event loop was
// busy) are dropped rather than sent in a burst, as a camera would drop them.
class LogCameraPlayer {
 public:
  // How far ahead of time the log is read.
  static constexpr std::chrono::milliseconds kLookahead{250};
  // How late a frame may be sent before it is dropped instead.
  static constexpr std::chrono::milliseconds kMaxLateness{50};

  struct Statistics {
    // Frames sent, for each camera in |cameras|.
    std::vector<int> sent;
    // Frames dropped for coming due too late, or refused as too fast.
    int late = 0;
    int refused = 0;
    // How many times the log has been played to its end.
    int loops = 0;
  };

  // Plays the cameras numbered |cameras| from |log_files|, the logged node
  // |log_node| (ignored for a single-node log).  With |loop| false, it calls
  // |on_end| (if set) at the end of the log and sends nothing more.
  LogCameraPlayer(aos::EventLoop *event_loop,
                  std::vector<aos::logger::LogFile> log_files,
                  std::string log_node, std::vector<int> cameras, bool loop,
                  std::function<void()> on_end = nullptr);
  ~LogCameraPlayer();

  LogCameraPlayer(const LogCameraPlayer &) = delete;
  LogCameraPlayer &operator=(const LogCameraPlayer &) = delete;

  const Statistics &statistics() const { return statistics_; }

 private:
  // One logged frame, waiting to be sent.
  struct Frame {
    // Index into cameras_.
    size_t camera;
    // When it was sent in the log, from the start of this pass.
    aos::monotonic_clock::duration offset;
    int32_t rows;
    int32_t cols;
    ImageFormat format;
    std::vector<uint8_t> data;
  };

  // Starts a pass over the log, playing from now.
  void StartPass();
  // Reads the log until kLookahead past the newest queued frame, or its end.
  void Fill();
  // Sends what's due, and waits for the next frame.
  void Play();
  void Send(const Frame &frame);

  aos::monotonic_clock::time_point Due(const Frame &frame) const {
    return pass_start_ + frame.offset;
  }

  aos::EventLoop *const event_loop_;
  const std::vector<aos::logger::LogFile> log_files_;
  const std::string log_node_;
  const std::vector<int> cameras_;
  const bool loop_;
  const std::function<void()> on_end_;
  const aos::logger::ReplayChannels replay_channels_;

  std::vector<aos::Sender<CameraImage>> senders_;
  aos::TimerHandler *const timer_;

  // The log being read in this pass, and the event loop in its simulation.
  std::unique_ptr<aos::logger::LogReader> reader_;
  std::unique_ptr<aos::EventLoop> log_event_loop_;
  bool log_done_ = false;
  // The log time of this pass's first frame, which plays at pass_start_.
  std::optional<aos::monotonic_clock::time_point> log_first_;
  aos::monotonic_clock::time_point pass_start_;

  std::deque<Frame> frames_;
  Statistics statistics_;
};

}  // namespace frc::vision

#endif  // FRC_VISION_CAMERA_LOG_CAMERA_PLAYER_H_
