load("//tools/rust:defs.bzl", "rust_binary", "rust_library")

package(default_visibility = ["//visibility:public"])

# The robot's HTTPS front door; see lib.rs and
# documentation/adr/0007-robot-web-server-tls-mcp-logs.md.
rust_library(
    name = "robot_server_lib",
    srcs = [
        "api.rs",
        "lib.rs",
    ],
    crate_name = "robot_server",
    test_data = glob(["testdata/*"]),
    test_deps = [
        "//aos/testing:path_rs",
        "@crate_index//:tower",
    ],
    deps = [
        "@crate_index//:anyhow",
        "@crate_index//:axum",
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

_BINARY_DEPS = [
    ":robot_server_lib",
    "//aos:configuration_rs",
    "//aos:configuration_rust_fbs",
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
