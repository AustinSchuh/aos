load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//aos:config.bzl", "aos_config")
load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")
load("//frc/downloader:downloader.bzl", "aos_downloader_dir")

package(default_visibility = ["//visibility:public"])

# What reads inside a robot's logs for robot_server's /logs: lists them, finds
# the matches in them, summarizes what went wrong, and converts them to MCAP.
# See README.md.
cc_library(
    name = "log_manager_lib",
    srcs = ["log_manager.cc"],
    hdrs = ["log_manager.h"],
    deps = [
        "//aos:configuration",
        "//aos:configuration_fbs",
        "//aos/events:event_loop",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_reader",
        "//aos/events/logging:logfile_utils",
        "//aos/logging:log_message_fbs",
        "//aos/time",
        "//aos/util:log_to_mcap_lib",
        "//frc/input:joystick_state_fbs",
        "//frc/input:robot_state_fbs",
        "@com_github_nlohmann_json//:json",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/time",
    ],
)

# The command line tool robot_server and logs_server run in a child process for
# anything that reads a log, so a corrupt log can't take the server down.
cc_binary(
    name = "log_manager",
    srcs = ["log_manager_main.cc"],
    deps = [
        ":log_manager_lib",
        "//aos:init",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/flags:usage",
        "@com_google_absl//absl/log:absl_log",
        "@com_google_absl//absl/strings",
    ],
)

static_flatbuffer(
    name = "test_position_fbs",
    srcs = ["test_position.fbs"],
    deps = ["//frc/control_loops:can_talonfx_fbs"],
)

static_flatbuffer(
    name = "test_status_fbs",
    srcs = ["test_status.fbs"],
)

aos_config(
    name = "test_config",
    src = "test_config_source.json",
    flatbuffers = [
        ":test_position_fbs",
        ":test_status_fbs",
    ],
    deps = ["//frc/input:aos_config"],
)

cc_test(
    name = "log_manager_test",
    srcs = ["log_manager_test.cc"],
    data = [":test_config"],
    deps = [
        ":log_manager_lib",
        ":test_position_fbs",
        ":test_status_fbs",
        "//aos:configuration",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_writer",
        "//aos/logging:log_message_fbs",
        "//aos/testing:googletest",
        "//aos/testing:tmpdir",
        "//frc/control_loops:can_device_error_fbs",
        "//frc/control_loops:can_talonfx_fbs",
        "//frc/input:joystick_state_fbs",
        "//frc/input:robot_state_fbs",
    ],
)

# The log page.  robot_server serves it from its --data_dir as /logs.html, and
# the vision page shows it on its Logs tab; logs_server serves it on a laptop.
filegroup(
    name = "www",
    srcs = glob(["www/*"]),
)

# The log page, for a robot's www folder, beside the other pages.
aos_downloader_dir(
    name = "www_files",
    srcs = [":www"],
    dir = "www",
    target_compatible_with = ["@platforms//os:linux"],
)

# Foxglove layouts for the MCAPs: copy into Foxglove's standalone folder.
filegroup(
    name = "layouts",
    srcs = glob(["layouts/*.json"]),
)
