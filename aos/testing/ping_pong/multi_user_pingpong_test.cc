// Ping and pong as two different users on one shared-memory queue: the
// requirement documentation/adr/0006-multi-user-futex-wakeup.md exists for.
//
// Becoming two users needs root, so this skips itself otherwise; run it by
// hand (`sudo bazel-bin/...`) on a machine whose kernel has io_uring's futex
// operations.  Everything else it needs -- a world-writable --shm_base, a
// --permissions that keeps it so, and the futex wakeup kind -- it sets up
// itself.

#include <grp.h>
#include <pwd.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <filesystem>
#include <string>

#include "absl/flags/declare.h"
#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "gtest/gtest.h"

#include "aos/configuration.h"
#include "aos/events/shm_event_loop.h"
#include "aos/testing/path.h"
#include "aos/testing/ping_pong/ping_lib.h"
#include "aos/testing/ping_pong/pong_lib.h"
#include "aos/testing/tmpdir.h"

ABSL_DECLARE_FLAG(std::string, shm_base);
ABSL_DECLARE_FLAG(uint32_t, permissions);
ABSL_DECLARE_FLAG(std::string, queue_wakeup);
ABSL_DECLARE_FLAG(bool, skip_realtime_scheduler);

namespace aos::testing {
namespace {

namespace chrono = std::chrono;

// Drops every privilege and becomes `name`, in a forked child.
void BecomeUser(const char *name) {
  struct passwd *pw = getpwnam(name);
  ABSL_CHECK(pw != nullptr) << ": no user " << name;
  ABSL_PCHECK(setgroups(0, nullptr) == 0);
  ABSL_PCHECK(setgid(pw->pw_gid) == 0);
  ABSL_PCHECK(setuid(pw->pw_uid) == 0);
  ABSL_CHECK_EQ(getuid(), pw->pw_uid);
  ABSL_CHECK_EQ(geteuid(), pw->pw_uid);
}

bool UserExists(const char *name) { return getpwnam(name) != nullptr; }

}  // namespace

TEST(MultiUserPingPongTest, PingAndPongAsDifferentUsers) {
  if (geteuid() != 0) {
    GTEST_SKIP() << "needs root to become two different users";
  }
  // Two unprivileged users every Linux has.
  const char *const kPingUser = "daemon";
  const char *const kPongUser = "nobody";
  if (!UserExists(kPingUser) || !UserExists(kPongUser)) {
    GTEST_SKIP() << "needs the users " << kPingUser << " and " << kPongUser;
  }

  // A shm_base both users can write, with a mode that keeps it that way for
  // everything created underneath (no group to share here, so world).
  const std::string shm_base = TestTmpDir() + "/multi_user_shm";
  std::filesystem::remove_all(shm_base);
  ABSL_CHECK(std::filesystem::create_directory(shm_base));
  ABSL_PCHECK(chmod(shm_base.c_str(), 0777) == 0);
  absl::SetFlag(&FLAGS_shm_base, shm_base);
  absl::SetFlag(&FLAGS_permissions, 0777);
  absl::SetFlag(&FLAGS_queue_wakeup, "futex");
  // Neither user can go realtime, and Ping asks to.
  absl::SetFlag(&FLAGS_skip_realtime_scheduler, true);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(
          ArtifactPath("aos/testing/ping_pong/pingpong_config.json"));

  const pid_t pong_pid = fork();
  ABSL_PCHECK(pong_pid >= 0);
  if (pong_pid == 0) {
    BecomeUser(kPongUser);
    ShmEventLoop event_loop(&config.message());
    Pong pong(&event_loop);
    TimerHandler *stop =
        event_loop.AddTimer([&event_loop]() { event_loop.Exit(); });
    event_loop.OnRun([&event_loop, stop]() {
      stop->Schedule(event_loop.monotonic_now() + chrono::seconds(4));
    });
    event_loop.Run();
    _exit(0);
  }

  const pid_t ping_pid = fork();
  ABSL_PCHECK(ping_pid >= 0);
  if (ping_pid == 0) {
    BecomeUser(kPingUser);
    // Give pong a moment to be watching before the first ping goes out.
    std::this_thread::sleep_for(chrono::milliseconds(500));
    ShmEventLoop event_loop(&config.message());
    Ping ping(&event_loop);
    bool enough = false;
    TimerHandler *stop = event_loop.AddTimer([&]() {
      enough = ping.pongs_received() >= 50;
      event_loop.Exit();
    });
    event_loop.OnRun([&event_loop, stop]() {
      stop->Schedule(event_loop.monotonic_now() + chrono::seconds(2));
    });
    event_loop.Run();
    ABSL_LOG(INFO) << "ping received " << ping.pongs_received() << " pongs";
    _exit(enough ? 0 : 1);
  }

  int status = 0;
  ABSL_PCHECK(waitpid(ping_pid, &status, 0) == ping_pid);
  EXPECT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0)
      << "ping (as " << kPingUser << ") did not get its pongs from pong (as "
      << kPongUser << "); status " << status;
  ABSL_PCHECK(waitpid(pong_pid, &status, 0) == pong_pid);
  EXPECT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0)
      << "pong (as " << kPongUser << ") failed; status " << status;

  std::filesystem::remove_all(shm_base);
}

}  // namespace aos::testing
