#include <grp.h>
#include <pwd.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
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
#include "aos/ipc_lib/memory_mapped_queue.h"
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
          "If true, delete everything in --shm_base, and the queue of every "
          "channel in --config, before starting.");
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

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  const aos::Configuration *config_msg = &config.message();

  if (absl::GetFlag(FLAGS_purge_shm_base)) {
    aos::util::UnlinkRecursive(absl::GetFlag(FLAGS_shm_base));
    // On macOS, queues under /dev/shm/ aren't files, so deleting the
    // directory doesn't find them.  Delete them by name instead.  That misses
    // channels which aren't in this config, which stay until a reboot.
    aos::ipc_lib::UnlinkQueues(absl::GetFlag(FLAGS_shm_base), config_msg);
  }

  if (!absl::GetFlag(FLAGS_shm_group).empty()) {
    SetUpSharedShmBase(absl::GetFlag(FLAGS_shm_group));
  }

  if (!absl::GetFlag(FLAGS_user).empty()) {
    uid_t uid;
    uid_t gid;
    std::vector<gid_t> groups;
    {
      struct passwd *user_data = getpwnam(absl::GetFlag(FLAGS_user).c_str());
      if (user_data != nullptr) {
        ABSL_LOG(INFO) << "Switching to user " << user_data->pw_name;
        uid = user_data->pw_uid;
        gid = user_data->pw_gid;

        // Grow the list until all the groups fit.  getgrouplist() returns -1
        // if they didn't.  Linux then says how many there are, and macOS
        // doesn't.
        int ngroups = 16;
        while (true) {
          groups.resize(ngroups);
          int found = ngroups;
#if defined(__APPLE__)
          // macOS takes the groups as ints.
          static_assert(sizeof(gid_t) == sizeof(int));
          int *const group_list = reinterpret_cast<int *>(groups.data());
#else
          gid_t *const group_list = groups.data();
#endif
          if (getgrouplist(user_data->pw_name, gid, group_list, &found) != -1) {
            groups.resize(found);
            ngroups = found;
            break;
          }
          ngroups = std::max(found, ngroups * 2);
        }
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
    // ID to still participate in signal sending/receiving.  The saved IDs stay
    // root, so that children can switch to the user they are configured to run
    // as.
#if defined(__APPLE__)
    // macOS has no setresuid().  Changing only the real ID sets the saved ID to
    // the effective one, which is still root, and seteuid() changes only the
    // effective ID.
    constexpr gid_t kUnchangedGid = static_cast<gid_t>(-1);
    constexpr uid_t kUnchangedUid = static_cast<uid_t>(-1);
    if (setregid(gid, kUnchangedGid) != 0 || setegid(gid) != 0) {
      ABSL_PLOG(FATAL) << "Failed to change GID to "
                       << absl::GetFlag(FLAGS_user) << ", group " << gid;
    }

    if (setreuid(uid, kUnchangedUid) != 0 || seteuid(uid) != 0) {
      ABSL_PLOG(FATAL) << "Failed to change UID to "
                       << absl::GetFlag(FLAGS_user);
    }
#else
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
#endif
  }

  aos::starter::Starter starter(config_msg, aos::starter::MakeCGroupManager());
  if (!absl::GetFlag(FLAGS_version_string).empty()) {
    starter.event_loop()->SetVersionString(absl::GetFlag(FLAGS_version_string));
  }

  starter.Run();

  return 0;
}
