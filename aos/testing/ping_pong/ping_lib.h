#ifndef AOS_EVENTS_PING_LIB_H_
#define AOS_EVENTS_PING_LIB_H_

#include <chrono>
#include <string_view>
#include <vector>

#include "aos/events/event_loop.h"
#include "aos/testing/ping_pong/ping_static.h"
#include "aos/testing/ping_pong/pong_static.h"

namespace aos {

// Class which sends out a Ping message every X ms, and times the response.
class Ping {
 public:
  Ping(EventLoop *event_loop, std::string_view channel_name = "/test");

  void set_quiet(bool quiet) { quiet_ = quiet; }

  // Number of pongs received so far.
  int pongs_received() const { return pongs_received_; }

  // Logs the distribution (count, median, p90, p99, max) of the round-trip
  // times seen so far.  Not realtime-safe; call it after the loop has
  // stopped.  The record is capped at kMaxRecordedRoundTrips, so a long run
  // reports its beginning.
  void LogRoundTripStatistics() const;

  static constexpr size_t kMaxRecordedRoundTrips = 200000;

 private:
  // Sends out the ping message with an incrementing count.
  void SendPing();

  // Receives the reply and measures the latency.
  void HandlePong(const examples::Pong &pong);

  aos::EventLoop *event_loop_;
  aos::Sender<examples::PingStatic> sender_;
  // Timer handle which sends the Ping message.
  aos::TimerHandler *timer_handle_;
  // Number of pings sent.
  int count_ = 0;
  // Last pong value received so we can detect missed pongs.
  int last_pong_value_ = 0;
  int pongs_received_ = 0;

  // Every round trip, up to the cap.  Reserved in the constructor so that
  // recording one never allocates on the realtime loop.
  std::vector<std::chrono::nanoseconds> round_trips_;

  bool quiet_ = true;
};

}  // namespace aos

#endif  // AOS_EVENTS_PING_LIB_H_
