#include "aos/ipc_lib/shm_mapping.h"

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <linux/magic.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/vfs.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <thread>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#include "aos/util/file.h"

namespace aos::ipc_lib {

long SystemPageSize() { return sysconf(_SC_PAGESIZE); }

namespace {

constexpr size_t RoundUp(size_t value, size_t multiple) {
  return (value + multiple - 1) / multiple * multiple;
}

// Waits until the queue file at fd has a nonzero size (the creating process
// may still be between open() and ftruncate()), then CHECKs that it matches
// the expected size.
void WaitForQueueFileSize(int fd, const std::string &path,
                          size_t expected_size) {
  while (true) {
    struct stat st;
    ABSL_PCHECK(fstat(fd, &st) == 0);
    if (st.st_size != 0) {
      ABSL_CHECK_EQ(static_cast<size_t>(st.st_size), expected_size)
          << ": Size of " << path
          << " doesn't match expected size of backing queue file.  Did "
             "the queue definition change?";
      break;
    } else {
      // The creating process didn't get around to it yet.  Give it a bit.
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      ABSL_VLOG(1) << path << " is zero size, waiting";
    }
  }
}

// Opens the queue file at hugepage_path, creating it (O_EXCL, sized to size
// rounded up to the huge page size) if it doesn't exist yet, and symlinks it
// at path.  Returns an fd whose file already has its final size, or -1 if
// path is occupied by something else the caller should open instead.
int OpenHugePageQueueFile(const std::string &path,
                          const std::string &hugepage_path,
                          std::filesystem::perms permissions, size_t size) {
  util::MkdirP(hugepage_path, permissions);
  int fd = open(hugepage_path.c_str(), O_RDWR | O_CREAT | O_EXCL | O_CLOEXEC,
                static_cast<mode_t>(permissions));
  const bool created_hugepage_file = (fd != -1);
  if (!created_hugepage_file) {
    ABSL_PCHECK(errno == EEXIST) << ": Failed to create " << hugepage_path;
    ABSL_VLOG(1) << hugepage_path << " already created.";
    fd = open(hugepage_path.c_str(), O_RDWR | O_CLOEXEC,
              static_cast<mode_t>(permissions));
    ABSL_PCHECK(fd != -1) << ": Failed to open " << hugepage_path;
  }
  struct statfs fs;
  ABSL_PCHECK(fstatfs(fd, &fs) == 0);
  size_t create_size = size;
  if (fs.f_type == static_cast<decltype(fs.f_type)>(HUGETLBFS_MAGIC)) {
    create_size = RoundUp(size, static_cast<size_t>(fs.f_bsize));
  } else {
    ABSL_LOG(WARNING) << hugepage_path
                      << " is not on hugetlbfs; creating queue as a normal "
                         "file";
  }
  if (created_hugepage_file) {
    // ftruncate before the symlink below so no reader can ever see a
    // zero-size hugetlbfs file.
    ABSL_PCHECK(ftruncate(fd, create_size) == 0);
  } else {
    WaitForQueueFileSize(fd, hugepage_path, create_size);
  }
  if (symlink(hugepage_path.c_str(), path.c_str()) == 0) {
    ABSL_VLOG(1) << "Created " << hugepage_path << " linked at " << path;
    return fd;
  }
  if (errno != EEXIST) {
    ABSL_PLOG(FATAL) << "Failed to symlink " << path << " -> " << hugepage_path;
  }
  // Something already occupies path.  If it's a symlink to the hugetlbfs
  // file (the normal stack-restart case), keep it and return the fd: it
  // already has its final size, so the caller must not wait on it again.  Any
  // other occupant wins: if we created the hugetlbfs file just now nothing
  // references it, so remove it rather than leave an orphan on the hugetlbfs
  // mount, then close fd and let the caller open path itself.
  struct stat lst;
  if (lstat(path.c_str(), &lst) == 0 && S_ISLNK(lst.st_mode)) {
    char target[PATH_MAX];
    const ssize_t len = readlink(path.c_str(), target, sizeof(target) - 1);
    if (len > 0 &&
        hugepage_path == std::string_view(target, static_cast<size_t>(len))) {
      return fd;
    }
  }
  if (created_hugepage_file) {
    ABSL_PCHECK(unlink(hugepage_path.c_str()) == 0)
        << ": Failed to remove orphaned " << hugepage_path;
  }
  ABSL_PCHECK(close(fd) == 0);
  return -1;
}

struct MappedRegion {
  void *data;
  // Bytes actually mmap()ed: the requested size rounded up to the huge page
  // size when the file lives on hugetlbfs, the requested size otherwise.
  size_t mapped_size;
};

MappedRegion MapShm(std::string_view path, size_t size,
                    std::filesystem::perms permissions, bool writable,
                    std::string_view hugepage_path) {
  std::string path_str(path);
  util::MkdirP(path_str, permissions);

  int fd = -1;
  bool created = false;
  if (writable && !hugepage_path.empty()) {
    // Large queues can be backed by huge pages: the file is created on a
    // hugetlbfs mount at hugepage_path with a symlink left at path; everyone
    // else follows the symlink transparently.  The returned fd (if any)
    // already has its final size.
    fd = OpenHugePageQueueFile(path_str, std::string(hugepage_path),
                               permissions, size);
  }
  if (fd == -1) {
    if (writable) {
      // Two cases: either the file doesn't exist yet and we create it, or it
      // already exists.  Try to create it exclusively first; if that races
      // with another creator (EEXIST) fall back to opening the existing file.
      // Once created, the file is never deleted.  Only the process whose
      // exclusive create succeeded is responsible for sizing the file;
      // everyone else waits for it (see below).
      fd = open(path_str.c_str(), O_RDWR | O_CREAT | O_EXCL | O_CLOEXEC,
                static_cast<mode_t>(permissions));
      if (fd != -1) {
        created = true;
      } else if (errno == EEXIST) {
        fd = open(path_str.c_str(), O_RDWR | O_CLOEXEC,
                  static_cast<mode_t>(permissions));
      }
    } else {
      fd = open(path_str.c_str(), O_RDONLY | O_CLOEXEC,
                static_cast<mode_t>(permissions));
    }
  }
  ABSL_PCHECK(fd != -1) << ": opening " << path_str << " failed";

  // The file may live on hugetlbfs (reached through the symlink); its size
  // and the size we must mmap are then rounded up to the huge page size, and
  // the page-fault loops below must step by that page size.
  struct statfs fs;
  ABSL_PCHECK(fstatfs(fd, &fs) == 0);
  const bool on_hugetlbfs =
      (fs.f_type == static_cast<decltype(fs.f_type)>(HUGETLBFS_MAGIC));
  const size_t mapped_size =
      on_hugetlbfs ? RoundUp(size, static_cast<size_t>(fs.f_bsize)) : size;
  const long page_size =
      on_hugetlbfs ? static_cast<long>(fs.f_bsize) : SystemPageSize();
  ABSL_VLOG(1) << path_str << " backed by "
               << (on_hugetlbfs ? "hugetlbfs" : "tmpfs") << ", mapped size "
               << mapped_size;

  if (created) {
    // We created the (empty) file, so we're the one who gives it its real size.
    ABSL_PCHECK(ftruncate(fd, mapped_size) == 0);
    struct stat st;
    ABSL_PCHECK(fstat(fd, &st) == 0);
    ABSL_CHECK_EQ(static_cast<size_t>(st.st_size), mapped_size)
        << ": Size of " << path_str
        << " doesn't match expected size of backing queue file.";
  } else {
    // We opened a file someone else created.  Creating the file (open) and
    // giving it its size (ftruncate) aren't atomic, so we can open it in the
    // window in between, while it's still zero length.  Wait for the creating
    // process to finish sizing it before we check the size and map it.  This
    // applies to readers as well as to writers that lost the create race.
    WaitForQueueFileSize(fd, path_str, mapped_size);
  }

  void *data =
      mmap(NULL, mapped_size, writable ? (PROT_READ | PROT_WRITE) : PROT_READ,
           MAP_SHARED, fd, 0);
  ABSL_PCHECK(data != MAP_FAILED);
  ABSL_PCHECK(close(fd) == 0);

  // Pre-fault the pages so a later (possibly realtime) access never takes a
  // fault.  A write-fault installs a pagetable entry that is both writable and
  // readable, so faulting the writable mapping for writing also covers reads
  // through it -- there's no need to additionally read-fault it.  The read-only
  // mapping is PROT_READ (writing to it would fault), so it can only be
  // read-faulted.
  if (writable) {
    PageFaultDataWrite(static_cast<char *>(data), mapped_size, page_size);
  } else {
    PageFaultDataRead(static_cast<const char *>(data), mapped_size, page_size);
  }

  return {data, mapped_size};
}

}  // namespace

WritableShmMapping::WritableShmMapping(std::string_view path, size_t size,
                                       std::filesystem::perms permissions,
                                       std::string_view hugepage_path)
    : size_(size) {
  const MappedRegion region =
      MapShm(path, size, permissions, /*writable=*/true, hugepage_path);
  data_ = region.data;
  mapped_size_ = region.mapped_size;
}

WritableShmMapping::~WritableShmMapping() {
  ABSL_PCHECK(munmap(data_, mapped_size_) == 0);
}

ReadOnlyShmMapping::ReadOnlyShmMapping(std::string_view path, size_t size,
                                       std::filesystem::perms permissions)
    : size_(size) {
  const MappedRegion region =
      MapShm(path, size, permissions, /*writable=*/false, {});
  data_ = region.data;
  mapped_size_ = region.mapped_size;
}

ReadOnlyShmMapping::~ReadOnlyShmMapping() {
  ABSL_PCHECK(munmap(const_cast<void *>(data_), mapped_size_) == 0);
}

bool UnlinkShm(std::string_view path) {
  return unlink(std::string(path).c_str()) == 0;
}

}  // namespace aos::ipc_lib
