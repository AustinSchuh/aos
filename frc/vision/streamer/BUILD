load("//tools/rust:defs.bzl", "rust_binary", "rust_library")

package(default_visibility = ["//visibility:public"])

# What needs no GStreamer: frames for the encoder, the boxes on the AprilTags
# and drawing them, joining frames with their boxes, and the signalling
# messages.  Builds and tests anywhere.
rust_library(
    name = "camera_streamer_core",
    srcs = [
        "annotations.rs",
        "core.rs",
        "draw.rs",
        "frame.rs",
        "join.rs",
        "overlay.rs",
        "signalling.rs",
        "tag_box.rs",
    ],
    crate_root = "core.rs",
    deps = [
        "//frc/vision:camera_constants_rust_fbs",
        "//frc/vision:target_map_rust_fbs",
        "@aos_flatbuffers//rust:flatbuffers",
        "@com_github_foxglove_schemas//:ImageAnnotations_rust_fbs",
        "@crate_index//:serde_json",
    ],
)

# Streams the cameras to the browser over WebRTC; see lib.rs.  A real pipeline
# needs the x264, webrtc and nice plugins, from the sysroot.  x86's links once
# the amd64 sysroot has libbz2-dev (and runs once it has the plugins):
# frc/amd64/build_rootfs.py.
rust_library(
    name = "camera_streamer_lib",
    srcs = [
        "lib.rs",
        "pipeline.rs",
        "server.rs",
        "streamer.rs",
    ],
    crate_name = "camera_streamer",
    target_compatible_with = [
        "@platforms//cpu:arm64",
        "@platforms//os:linux",
    ],
    deps = [
        ":camera_streamer_core",
        "//aos:configuration_rust_fbs",
        "//aos:flatbuffers_rs",
        "//aos/events:event_loop_c_rs",
        "//aos/events:tokio_event_loop_rs",
        "//frc/vision:vision_rust_fbs",
        "@aos_flatbuffers//rust:flatbuffers",
        "@crate_index//:anyhow",
        "@crate_index//:clap",
        "@crate_index//:env_logger",
        "@crate_index//:futures",
        "@crate_index//:gstreamer",
        "@crate_index//:gstreamer-app",
        "@crate_index//:gstreamer-sdp",
        "@crate_index//:gstreamer-video",
        "@crate_index//:gstreamer-webrtc",
        "@crate_index//:log",
        "@crate_index//:tokio",
        "@crate_index//:tokio-tungstenite",
    ],
)

_BINARY_DEPS = [
    ":camera_streamer_lib",
    "//aos:configuration_rs",
    "//aos:configuration_rust_fbs",
    "//aos:flatbuffers_rs",
    "//aos:init_rs",
    "//aos/events:tokio_event_loop_rs",
    "//frc/vision:vision_rust_fbs",
    "@aos_flatbuffers//rust:flatbuffers",
    "@crate_index//:anyhow",
    "@crate_index//:clap",
    "@crate_index//:futures",
    "@crate_index//:gstreamer",
    "@crate_index//:log",
    "@crate_index//:tokio",
    "@crate_index//:tokio-tungstenite",
]

rust_binary(
    name = "camera_streamer",
    srcs = ["main.rs"],
    deps = _BINARY_DEPS,
)

# What robot_downloader deploys: every binary it ships has a .stripped twin,
# which cc_binary makes implicitly and rust_binary doesn't.
rust_binary(
    name = "camera_streamer.stripped",
    srcs = ["main.rs"],
    crate_name = "camera_streamer_main",
    rustc_flags = ["-Cstrip=symbols"],
    deps = _BINARY_DEPS,
)
