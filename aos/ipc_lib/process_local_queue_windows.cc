#include <windows.h>

#include "absl/log/absl_check.h"

#include "aos/ipc_lib/process_local_queue_internal.h"

namespace aos::ipc_lib::internal {

// An unnamed page-file-backed section: documented to start out zeroed, sized
// at creation, and gone when its last handle and view go away.  This is the
// section shm_mapping_windows.cc turns down for shared memory because its
// contents die with the process -- which is exactly what is wanted here.
// The handle is closed once both views exist; the views keep the section
// alive.
ProcessLocalMapping MapProcessLocal(size_t size) {
  const ULARGE_INTEGER section_size{.QuadPart = size};
  HANDLE section =
      CreateFileMappingW(INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE,
                         section_size.HighPart, section_size.LowPart, nullptr);
  ABSL_PCHECK(section != nullptr) << ": CreateFileMappingW failed";

  void *writable =
      MapViewOfFile(section, FILE_MAP_READ | FILE_MAP_WRITE, 0, 0, 0);
  ABSL_PCHECK(writable != nullptr) << ": MapViewOfFile(writable) failed";
  const void *readonly = MapViewOfFile(section, FILE_MAP_READ, 0, 0, 0);
  ABSL_PCHECK(readonly != nullptr) << ": MapViewOfFile(readonly) failed";
  ABSL_PCHECK(CloseHandle(section));

  return ProcessLocalMapping{writable, readonly, size};
}

void UnmapProcessLocal(const ProcessLocalMapping &mapping) {
  ABSL_PCHECK(UnmapViewOfFile(mapping.writable));
  ABSL_PCHECK(UnmapViewOfFile(mapping.readonly));
}

}  // namespace aos::ipc_lib::internal
