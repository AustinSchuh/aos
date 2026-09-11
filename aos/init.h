#ifndef AOS_INIT_H_
#define AOS_INIT_H_

namespace aos {

// Options for InitGoogle.
struct InitOptions {
  // Parses argc and argv with absl::ParseCommandLine.  If false, argc and argv
  // are not used and may be null.
  bool parse_command_line = true;
  // Installs abseil's failure signal handler when --backtrace is set.
  bool install_failure_signal_handler = true;
};

// Initializes AOS.
void InitGoogle(int *argc, char ***argv);
void InitGoogle(int *argc, char ***argv, const InitOptions &options);

// Initializes AOS without parsing the command line or installing signal
// handlers.  Does nothing if AOS is already initialized.  Thread-safe.
void InitEmbedded();

// Returns true if we have been initialized.  This is mostly here so
// ShmEventLoop can confirm the world was initialized before running.
bool IsInitialized();

// Marks the system as initialized. This is only meant to be used from
// init_for_rust. DO NOT call this from anywhere else, use InitGoogle
// instead.
void MarkInitialized();

}  // namespace aos

#endif  // AOS_INIT_H_
