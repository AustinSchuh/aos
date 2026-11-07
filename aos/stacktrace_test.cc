#include <signal.h>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "gtest/gtest.h"

#include "aos/realtime.h"
#include "aos/sanitizers.h"

namespace aos::testing {

namespace {

// A global function pointer purely for testing purposes when creating stack
// traces.
void (*g_function)() = nullptr;

class StacktraceTest : public ::testing::Test {
 public:
  StacktraceTest() {
    // Make sure that tests cannot accidentally use another test's helper
    // function.
    g_function = nullptr;
  }
};

using StacktraceDeathTest = StacktraceTest;

}  // namespace

// Create some (non-inlined) functions to create a predictable stack trace for
// the tests below.
#if defined(__clang__)
#define NOOPT __attribute__((optnone))
#elif defined(__GNUC__) || defined(__GNUG__)
#define NOOPT __attribute__((optimize("O0")))
#else
#error "Unknown compiler"
#endif

NOOPT void function1() {
  if (g_function != nullptr) {
    g_function();
  }
}

NOOPT void function2() { function1(); }
NOOPT void function3() { function2(); }
NOOPT void function4() { function3(); }

// Tests that we get a useful stacktrace on a `ABSL_LOG(FATAL)`. Also makes sure
// that we don't get a duplicate stack trace in the SIGABRT handler.
TEST(StacktraceDeathTest, StackTraceOnCrash) {
  g_function = [] { ABSL_LOG(FATAL) << "Triggering death!"; };

  // NOTE(james): Depending on the platform this runs on, we sometimes get
  // libunwind warnings in a line here. We may consider suppressing that in the
  // future.
  EXPECT_DEATH(
      { function4(); },
      ::testing::MatchesRegex(
          R"(F.... ..:..:......... .* stacktrace_test.cc:.*] Triggering death!
.*
?\*\*\* Check failure stack trace: \*\*\*
    @ .*  absl::.*::log_internal::LogMessage::SendToLog\(\)
    @ .*  absl::.*::log_internal::LogMessage::Flush\(\)
    @ .*  aos::testing::StacktraceDeathTest_StackTraceOnCrash_Test::TestBody\(\)::.*invoke\(\)
    @ .*  aos::testing::function1\(\)
    @ .*  aos::testing::function2\(\)
    @ .*  aos::testing::function3\(\)
    @ .*  aos::testing::function4\(\)
    @ .*  aos::testing::StacktraceDeathTest_StackTraceOnCrash_Test::TestBody\(\)
.*
\*\*\* SIGABRT received at time=[[:digit:]]+ on cpu [[:digit:]]+ \*\*\*
)"));
}

// Tests that we get a useful stacktrace on a segfault.
TEST(StacktraceDeathTest, StackTraceOnSegfault) {
#ifndef AOS_LIBUNWIND_STACK_TRACES
  GTEST_SKIP() << "absl's default unwinder can't get out of the signal "
                  "handler; build with --//aos:libunwind_stack_traces";
#endif
  g_function = [] { ABSL_CHECK_EQ(raise(SIGSEGV), 0); };

  // NOTE(james): Depending on the platform this runs on, we sometimes get
  // libunwind warnings in a line here. We may consider suppressing that in the
  // future.
  EXPECT_DEATH(
      { function4(); },
      ::testing::MatchesRegex(
          R"(\*\*\* SIGSEGV received at time=[[:digit:]]+ on cpu [[:digit:]]+ \*\*\*
.*
?PC: @ .*  .*
    @ .*  absl::.*::AbslFailureSignalHandler\(\)
    @ .*  .*
    @ .*  aos::testing::StacktraceDeathTest_StackTraceOnSegfault_Test::TestBody\(\)::.*invoke\(\)
    @ .*  aos::testing::function1\(\)
    @ .*  aos::testing::function2\(\)
    @ .*  aos::testing::function3\(\)
    @ .*  aos::testing::function4\(\)
    @ .*  aos::testing::StacktraceDeathTest_StackTraceOnSegfault_Test::TestBody\(\)
.*
)"));
}

// Malloc hooks don't work with asan/msan. So we cannot trigger a crash in those
// configurations.
#if !defined(AOS_SANITIZE_ADDRESS) && !defined(AOS_SANITIZE_MEMORY)

// Tests that we get a useful stacktrace on a malloc.
TEST(StacktraceDeathTest, StackTraceOnMalloc) {
#ifndef AOS_LIBUNWIND_STACK_TRACES
  GTEST_SKIP() << "absl's default unwinder can't get out of the signal "
                  "handler; build with --//aos:libunwind_stack_traces";
#endif
  g_function = [] {
    ScopedRealtime rt;
    volatile int *a = new int[3];
    *a = 5;
    EXPECT_EQ(*a, 5);
  };

  EXPECT_DEATH(
      { function4(); },
      ::testing::MatchesRegex(
          R"(\[realtime_linux\.cc : [[:digit:]]+\] RAW: Malloced [[:digit:]]+ bytes: [a-zA-Z._ ]*
\*\*\* SIGABRT received at time=[[:digit:]]+ on cpu [[:digit:]]+ \*\*\*
.*
?PC: @ .*  .*
    @ .*  absl::.*::AbslFailureSignalHandler\(\)
    @ .*  .*
    @ .*  aos::testing::StacktraceDeathTest_StackTraceOnMalloc_Test::TestBody\(\)::.*invoke\(\)
    @ .*  aos::testing::function1\(\)
    @ .*  aos::testing::function2\(\)
    @ .*  aos::testing::function3\(\)
    @ .*  aos::testing::function4\(\)
    @ .*  aos::testing::StacktraceDeathTest_StackTraceOnMalloc_Test::TestBody\(\)
.*
)"));
}

#endif

}  // namespace aos::testing
