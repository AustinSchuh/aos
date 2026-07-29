#ifndef FRC_WPILIB_NEWROBOTBASE_H_
#define FRC_WPILIB_NEWROBOTBASE_H_

#include <functional>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/logging/logging.h"
#include "frc/wpilib/ahal/RobotBase.h"

namespace frc::wpilib {

class WPILibRobotBase {
 public:
  virtual void Run() = 0;

  // Runs all the loops, each on its own thread (the first on the calling
  // thread, to save a thread's worth of memory).  Returns once every loop's
  // Run() has returned.
  void RunLoops() {
    // TODO(austin): SIGINT handler calling Exit on all the loops.
    // TODO(austin): RegisterSignalHandler in ShmEventLoop for others.
    if (loop_factories_.empty()) {
      LOG(FATAL) << "RunLoops() called with no loops added.";
    }
    ::std::vector<::std::thread> threads;
    for (size_t i = 1; i < loop_factories_.size(); ++i) {
      threads.emplace_back([this, i]() { RunOneLoop(loop_factories_[i]); });
    }
    RunOneLoop(loop_factories_[0]);

    for (::std::thread &thread : threads) {
      thread.join();
    }

    LOG(ERROR) << "Exiting WPILibRobot";
  }

 protected:
  // Adds a loop to run, expressed as a factory rather than an already-built
  // loop: the factory is invoked on the same thread that will call Run(),
  // and the loop it returns is destroyed on that thread too, once Run()
  // returns.  Construction, Run(), and destruction sharing one thread is
  // what the io_uring backend needs to keep IORING_SETUP_SINGLE_ISSUER's
  // deterministic completion delivery (a loop constructed on one thread but
  // run on another silently downgrades to a less deterministic mode -- see
  // documentation/adr/0001-aio-io-uring-single-issuer.md), and what AOS's
  // shared-memory queues require of the senders and watchers registered on
  // the loop (each must be destroyed on its construction thread, a
  // kernel-enforced PI-futex property).
  //
  // Application objects that must live while the loop runs should be
  // constructed inside the factory and owned by the loop's own callbacks
  // (captured in the std::functions registered on it), so they share the
  // loop's thread-correct lifetime.
  void AddLoop(
      ::std::function<::std::unique_ptr<::aos::ShmEventLoop>()> factory) {
    loop_factories_.push_back(::std::move(factory));
  }

 private:
  static void RunOneLoop(
      const ::std::function<::std::unique_ptr<::aos::ShmEventLoop>()>
          &factory) {
    ::std::unique_ptr<::aos::ShmEventLoop> loop = factory();
    LOG(INFO) << "Starting " << loop->name() << " with priority "
              << loop->runtime_realtime_priority();
    loop->Run();
    // `loop` is destroyed here, on the thread that constructed and ran it.
  }

  // Factories for the event loops to run in RunLoops.
  ::std::vector<::std::function<::std::unique_ptr<::aos::ShmEventLoop>()>>
      loop_factories_;
};

#define AOS_ROBOT_CLASS(_ClassName_) \
  START_ROBOT_CLASS(::frc::wpilib::WPILibAdapterRobot<_ClassName_>)

template <typename T>
class WPILibAdapterRobot : public frc::RobotBase {
 public:
  void StartCompetition() override {
    PCHECK(setuid(0) == 0) << ": Failed to change user to root";
    // Just allow overcommit memory like usual. Various processes map memory
    // they will never use, and the roboRIO doesn't have enough RAM to handle
    // it. This is in here instead of starter.sh because starter.sh doesn't run
    // with permissions on a roboRIO.
    PCHECK(system("echo 0 > /proc/sys/vm/overcommit_memory") == 0);
    PCHECK(system("busybox ps -ef | grep '\\[ktimersoftd/0\\]' | awk '{print "
                  "$1}' | xargs chrt -f -p 70") == 0);
    PCHECK(system("busybox ps -ef | grep '\\[ktimersoftd/1\\]' | awk '{print "
                  "$1}' | xargs chrt -f -p 70") == 0);
    PCHECK(system("busybox ps -ef | grep '\\[irq/54-eth0\\]' | awk '{print "
                  "$1}' | xargs chrt -f -p 17") == 0);

    // Configure throttling so we reserve 5% of the CPU for non-rt work.
    // This makes things significantly more stable when work explodes.
    // This is in here instead of starter.sh for the same reasons, starter is
    // suid and runs as admin, so this actually works.
    PCHECK(system("/sbin/sysctl -w kernel.sched_rt_period_us=1000000") == 0);
    PCHECK(system("/sbin/sysctl -w kernel.sched_rt_runtime_us=950000") == 0);

    robot_.Run();
  }

 private:
  T robot_;
};

}  // namespace frc::wpilib

#endif  // FRC_WPILIB_NEWROBOTBASE_H_
