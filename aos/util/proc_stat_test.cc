#include "aos/util/proc_stat.h"

#include <string_view>

#include "gtest/gtest.h"

namespace aos::util::testing {

// Lines in the shape /proc/self/mountinfo produces; only the /proc one matters,
// and its super options are what carry hidepid.
constexpr std::string_view kRootLine =
    "22 1 8:1 / / rw,relatime shared:1 - ext4 /dev/sda1 rw,errors=remount-ro\n";
constexpr std::string_view kSysLine =
    "23 22 0:20 / /sys rw,nosuid,nodev,noexec,relatime shared:2 - sysfs sysfs "
    "rw\n";

std::string Mountinfo(std::string_view proc_super_options) {
  return std::string(kRootLine) + std::string(kSysLine) +
         "24 22 0:21 / /proc rw,nosuid,nodev,noexec,relatime shared:3 - proc "
         "proc " +
         std::string(proc_super_options) + "\n";
}

TEST(ProcStatTest, MountinfoWithoutHidepidHidesNothing) {
  EXPECT_FALSE(MountinfoHidesPids(Mountinfo("rw")));
  EXPECT_FALSE(MountinfoHidesPids(Mountinfo("rw,hidepid=0")));
  EXPECT_FALSE(MountinfoHidesPids(Mountinfo("rw,hidepid=off,subset=pid")));
}

TEST(ProcStatTest, MountinfoWithHidepidHides) {
  EXPECT_TRUE(MountinfoHidesPids(Mountinfo("rw,hidepid=1")));
  EXPECT_TRUE(MountinfoHidesPids(Mountinfo("rw,hidepid=2,gid=1001")));
  EXPECT_TRUE(MountinfoHidesPids(Mountinfo("rw,hidepid=invisible")));
  EXPECT_TRUE(MountinfoHidesPids(Mountinfo("rw,hidepid=ptraceable")));
}

// A hidepid on some other filesystem's options, or a /proc mounted somewhere
// else, is not this process's /proc.
TEST(ProcStatTest, MountinfoIgnoresOtherMounts) {
  const std::string other =
      std::string(kRootLine) +
      "25 22 0:22 / /mnt/proc rw,relatime shared:4 - proc proc rw,hidepid=2\n" +
      "26 22 0:23 / /proc rw,relatime shared:5 - proc proc rw\n";
  EXPECT_FALSE(MountinfoHidesPids(other));
  EXPECT_FALSE(MountinfoHidesPids(""));
  EXPECT_FALSE(MountinfoHidesPids("garbage without a separator\n"));
}

// Reading the real /proc/self/mountinfo works and answers something, whichever
// way this host happens to be mounted.  The answer itself is the host's, so
// there is nothing here to compare it against -- the parser is what the cases
// above pin.
TEST(ProcStatTest, ProcHidesPidsRuns) { (void)ProcHidesPids(); }

}  // namespace aos::util::testing
