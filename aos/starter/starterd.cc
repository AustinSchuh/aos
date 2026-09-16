#include <grp.h>
#include <pwd.h>
#include <sys/stat.h>
#include <unistd.h>

#include <filesystem>
#include <ostream>
#include <string>

#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#include "aos/configuration.h"
#include "aos/events/event_loop.h"
#include "aos/flatbuffers.h"
#include "aos/init.h"
#include "aos/starter/starterd_lib.h"
#include "aos/util/file.h"

ABSL_FLAG(std::string, config, "aos_config.json",
          "File path of aos configuration");
ABSL_FLAG(std::string, user, "",
          "Starter runs as though this user ran a SUID binary if set.");
ABSL_FLAG(std::string, version_string, "",
          "Version to report for starterd and subprocesses.");

ABSL_DECLARE_FLAG(std::string, shm_base);
ABSL_DECLARE_FLAG(uint32_t, permissions);
ABSL_FLAG(bool, purge_shm_base, false,
          "If true, delete everything in --shm_base before starting.");
ABSL_FLAG(std::string, shm_group, "",
          "If set, create --shm_base owned by this group, setgid, with "
          "--permissions, before starting anything or dropping privileges.  "
          "Every queue file created under it then inherits the group, which "
          "is what lets applications running as different users share "
          "queues (see documentation/adr/0006-multi-user-futex-wakeup.md).");

// Creates --shm_base as a group-shared, setgid directory.  Done while still
// root, before --user drops privileges: chown needs it, and so does chmod on a
// directory another run may have left owned by someone else.  A tmpfiles.d
// line does the same job for a deployment without starterd:
//   d /dev/shm/aos 2770 root aos -
void SetUpSharedShmBase(const std::string &group_name) {
  struct group *group_data = getgrnam(group_name.c_str());
  ABSL_CHECK(group_data != nullptr)
      << ": --shm_group " << group_name << " is not a group";
  const std::string shm_base = absl::GetFlag(FLAGS_shm_base);
  // MkdirP creates the parents of the path it is given, so the trailing
  // slash makes shm_base itself the last one it creates.
  aos::util::MkdirP(shm_base + "/", static_cast<std::filesystem::perms>(
                                        absl::GetFlag(FLAGS_permissions)));
  ABSL_PCHECK(
      chown(shm_base.c_str(), static_cast<uid_t>(-1), group_data->gr_gid) == 0)
      << ": chown " << shm_base << " to group " << group_name;
  ABSL_PCHECK(
      chmod(shm_base.c_str(), absl::GetFlag(FLAGS_permissions) | S_ISGID) == 0)
      << ": chmod " << shm_base;
}

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  if (absl::GetFlag(FLAGS_purge_shm_base)) {
    aos::util::UnlinkRecursive(absl::GetFlag(FLAGS_shm_base));
  }

  if (!absl::GetFlag(FLAGS_shm_group).empty()) {
    SetUpSharedShmBase(absl::GetFlag(FLAGS_shm_group));
  }

  if (!absl::GetFlag(FLAGS_user).empty()) {
    uid_t uid;
    uid_t gid;
    std::vector<__gid_t> groups;
    {
      struct passwd *user_data = getpwnam(absl::GetFlag(FLAGS_user).c_str());
      if (user_data != nullptr) {
        ABSL_LOG(INFO) << "Switching to user " << user_data->pw_name;
        uid = user_data->pw_uid;
        gid = user_data->pw_gid;

        // Get the group size.  This is an error since the user should always be
        // a member of more groups than 0.  Something went wrong if that isn't
        // true.
        int ngroups = 0;
        ABSL_CHECK(getgrouplist(user_data->pw_name, gid, NULL, &ngroups) == -1);
        groups.resize(ngroups);

        ABSL_PCHECK(getgrouplist(user_data->pw_name, gid, groups.data(),
                                 &ngroups) == static_cast<int>(groups.size()));
        for (int i = 0; i < ngroups; i++) {
          struct group *gr = getgrgid(groups[i]);
          ABSL_PCHECK(gr != nullptr);

          ABSL_LOG(INFO) << "  Adding supplemental group of " << gr->gr_name;
        }
      } else {
        ABSL_LOG(FATAL) << "Could not find user " << absl::GetFlag(FLAGS_user);
        return 1;
      }
    }
    // Change the supplemental groups of the user we're running as.
    ABSL_PCHECK(setgroups(groups.size(), groups.data()) == 0)
        << ": Failed to set groups";

    // Change the real and effective IDs to the user we're running as. The
    // effective IDs mean files we access (like shared memory) will happen as
    // that user. The real IDs allow child processes with an different effective
    // ID to still participate in signal sending/receiving.
    constexpr int kUnchanged = -1;
    if (setresgid(/* ruid */ gid, /* euid */ gid,
                  /* suid */ kUnchanged) != 0) {
      ABSL_PLOG(FATAL) << "Failed to change GID to "
                       << absl::GetFlag(FLAGS_user) << ", group " << gid;
    }

    if (setresuid(/* ruid */ uid, /* euid */ uid,
                  /* suid */ kUnchanged) != 0) {
      ABSL_PLOG(FATAL) << "Failed to change UID to "
                       << absl::GetFlag(FLAGS_user);
    }
  }

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  const aos::Configuration *config_msg = &config.message();

  aos::starter::Starter starter(config_msg, aos::starter::MakeCGroupManager());
  if (!absl::GetFlag(FLAGS_version_string).empty()) {
    starter.event_loop()->SetVersionString(absl::GetFlag(FLAGS_version_string));
  }

  starter.Run();

  return 0;
}
