load("@rules_cc//cc:cc_library.bzl", "cc_library")

# GStreamer and the GLib libraries it is built on, from whichever sysroot the
# target uses.  gstreamer-rs's -sys crates find these with pkg-config in their
# build scripts, which can't work in a hermetic cross build, so MODULE.bazel and
# WORKSPACE turn those build scripts off and give each crate its library here.
#
# The amd64 sysroot has GStreamer 1.22 with its core plugins only (no x264,
# webrtc or nice), and the Orin's has 1.28 with all of them, so the crates are
# built for 1.22's API and anything that runs a real pipeline runs on the Orin.

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "unavailable",
    target_compatible_with = ["@platforms//:incompatible"],
)

cc_library(
    name = "glib-2.0",
    deps = select({
        "//tools:arm64_macos": [":unavailable"],
        "//tools:cpu_arm64": ["@arm64_debian_sysroot//:glib-2.0"],
        "//tools:cpu_k8": ["@amd64_debian_sysroot//:glib-2.0"],
        "//conditions:default": [":unavailable"],
    }),
)

cc_library(
    name = "gobject-2.0",
    deps = select({
        "//tools:arm64_macos": [":unavailable"],
        "//tools:cpu_arm64": ["@arm64_debian_sysroot//:gobject-2.0"],
        "//tools:cpu_k8": ["@amd64_debian_sysroot//:gobject-2.0"],
        "//conditions:default": [":unavailable"],
    }),
)

cc_library(
    name = "gstreamer-1.0",
    deps = select({
        "//tools:arm64_macos": [":unavailable"],
        "//tools:cpu_arm64": ["@arm64_debian_sysroot//:gstreamer-1.0"],
        "//tools:cpu_k8": ["@amd64_debian_sysroot//:gstreamer-1.0"],
        "//conditions:default": [":unavailable"],
    }),
)

cc_library(
    name = "gstreamer-base-1.0",
    deps = select({
        "//tools:arm64_macos": [":unavailable"],
        "//tools:cpu_arm64": ["@arm64_debian_sysroot//:gstreamer-base-1.0"],
        "//tools:cpu_k8": ["@amd64_debian_sysroot//:gstreamer-base-1.0"],
        "//conditions:default": [":unavailable"],
    }),
)

cc_library(
    name = "gstreamer-app-1.0",
    deps = select({
        "//tools:arm64_macos": [":unavailable"],
        "//tools:cpu_arm64": ["@arm64_debian_sysroot//:gstreamer-app-1.0"],
        "//tools:cpu_k8": ["@amd64_debian_sysroot//:gstreamer-app-1.0"],
        "//conditions:default": [":unavailable"],
    }),
)

cc_library(
    name = "gstreamer-video-1.0",
    deps = select({
        "//tools:arm64_macos": [":unavailable"],
        "//tools:cpu_arm64": ["@arm64_debian_sysroot//:gstreamer-video-1.0"],
        "//tools:cpu_k8": ["@amd64_debian_sysroot//:gstreamer-video-1.0"],
        "//conditions:default": [":unavailable"],
    }),
)

cc_library(
    name = "gstreamer-webrtc-1.0",
    deps = select({
        "//tools:arm64_macos": [":unavailable"],
        "//tools:cpu_arm64": ["@arm64_debian_sysroot//:gstreamer-webrtc-1.0"],
        "//tools:cpu_k8": ["@amd64_debian_sysroot//:gstreamer-webrtc-1.0"],
        "//conditions:default": [":unavailable"],
    }),
)

cc_library(
    name = "gstreamer-sdp-1.0",
    deps = select({
        "//tools:arm64_macos": [":unavailable"],
        "//tools:cpu_arm64": ["@arm64_debian_sysroot//:gstreamer-sdp-1.0"],
        "//tools:cpu_k8": ["@amd64_debian_sysroot//:gstreamer-sdp-1.0"],
        "//conditions:default": [":unavailable"],
    }),
)

cc_library(
    name = "gio-2.0",
    deps = select({
        "//tools:arm64_macos": [":unavailable"],
        "//tools:cpu_arm64": ["@arm64_debian_sysroot//:gio-2.0"],
        "//tools:cpu_k8": ["@amd64_debian_sysroot//:gio-2.0"],
        "//conditions:default": [":unavailable"],
    }),
)
