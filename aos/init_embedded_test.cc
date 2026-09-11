// Tests InitEmbedded() in a process that has not initialized AOS.

#include <cstdlib>

#ifndef _WIN32
#include <csignal>
#endif

#include "absl/log/absl_check.h"

#include "aos/init.h"

int main() {
  ABSL_CHECK(!aos::IsInitialized());
#ifndef _WIN32
  struct sigaction before{};
  ABSL_CHECK_EQ(sigaction(SIGSEGV, nullptr, &before), 0);
#endif

  aos::InitEmbedded();
  ABSL_CHECK(aos::IsInitialized());
  // A second call does nothing.
  aos::InitEmbedded();

#ifndef _WIN32
  struct sigaction after{};
  ABSL_CHECK_EQ(sigaction(SIGSEGV, nullptr, &after), 0);
  ABSL_CHECK(after.sa_handler == before.sa_handler)
      << ": InitEmbedded() installed a crash signal handler";
#endif
  return EXIT_SUCCESS;
}
