load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")
load("//tools/rust:defs.bzl", "flatbuffer_rust_library", "rust_binary", "rust_library")

package(default_visibility = ["//visibility:public"])

# The robot's HTTPS front door; see lib.rs and
# documentation/adr/0007-robot-web-server-tls-mcp-logs.md.
rust_library(
    name = "robot_server_lib",
    srcs = [
        "api.rs",
        "condition.rs",
        "lib.rs",
        "mcp.rs",
        "session.rs",
        "triggers.rs",
    ],
    crate_name = "robot_server",
    test_data = glob(["testdata/*"]) + [":condition_test_fbs_reflection_out"],
    test_deps = [
        ":condition_test_rust_fbs",
        "//aos/testing:path_rs",
        "@aos_flatbuffers//rust:flatbuffers",
        "@crate_index//:tower",
    ],
    deps = [
        "//aos:configuration_rs",
        "//aos:configuration_rust_fbs",
        "//aos:flatbuffers_rs",
        "//aos/events:event_loop_c_rs",
        "//aos/events:tokio_event_loop_rs",
        "//aos/flatbuffers/reflection:reflection_rust_fbs",
        "@crate_index//:anyhow",
        "@crate_index//:axum",
        "@crate_index//:clap",
        "@crate_index//:futures",
        "@crate_index//:hyper",
        "@crate_index//:hyper-util",
        "@crate_index//:libc",
        "@crate_index//:log",
        "@crate_index//:rustls",
        "@crate_index//:rustls-pki-types",
        "@crate_index//:serde_json",
        "@crate_index//:tokio",
        "@crate_index//:tokio-rustls",
        "@crate_index//:tokio-tungstenite",
        "@crate_index//:tower-http",
    ],
)

# A message with one of everything, for condition.rs's tests.
static_flatbuffer(
    name = "condition_test_fbs",
    srcs = ["condition_test.fbs"],
)

flatbuffer_rust_library(
    name = "condition_test_rust_fbs",
    srcs = ["condition_test.fbs"],
)

_BINARY_DEPS = [
    ":robot_server_lib",
    "//aos:configuration_rs",
    "//aos:flatbuffers_rs",
    "//aos:init_rs",
    "//aos/events:tokio_event_loop_rs",
    "@crate_index//:clap",
    "@crate_index//:libc",
    "@crate_index//:log",
    "@crate_index//:tokio",
]

rust_binary(
    name = "robot_server",
    srcs = ["main.rs"],
    deps = _BINARY_DEPS,
)

# What robot_downloader deploys: every binary it ships has a .stripped twin,
# which cc_binary makes implicitly and rust_binary doesn't.
rust_binary(
    name = "robot_server.stripped",
    srcs = ["main.rs"],
    crate_name = "robot_server_main",
    rustc_flags = ["-Cstrip=symbols"],
    deps = _BINARY_DEPS,
)

# A log replay with robot_server and web_proxy in it, to use the robot's pages
# and /api against a log; see robot_server_replay.cc.
cc_binary(
    name = "robot_server_replay",
    srcs = ["robot_server_replay.cc"],
    target_compatible_with = ["//tools/platforms/rust:has_support"],
    deps = [
        ":robot_server_lib",
        "//aos:configuration",
        "//aos:flatbuffer_merge",
        "//aos:init",
        "//aos/events:simulated_event_loop",
        "//aos/events:tokio_simulation",
        "//aos/events/logging:log_reader",
        "//aos/network:web_proxy",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
    ],
)
