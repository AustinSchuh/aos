load("@aspect_rules_ts//ts:defs.bzl", "ts_config")
load("@bazel_lib//lib:copy_to_directory.bzl", "copy_to_directory")
load("//frc/downloader:downloader.bzl", "aos_downloader_dir")
load("//tools/build_rules:js.bzl", "ng_application")

package(default_visibility = ["//visibility:public"])

# The vision page: the cameras' feeds from camera_streamer, with what the robot
# says about them, and their calibrations.  Served by robot_server at
# https://<robot>/vision/.  Its logic is in lib/, tested on Node; app/ is thin
# Angular components over it.

# The repo's tsconfig, made strict.
ts_config(
    name = "tsconfig",
    src = "tsconfig.json",
    deps = ["//:tsconfig"],
)

ng_application(
    name = "app",
    assets = [],
    html_assets = ["styles.css"],
    tsconfig = ":tsconfig",
    deps = [
        "//aos/network/www:proxy",
        "//frc/vision:turbojpeg_decoder_status_ts_fbs",
        "//frc/vision/streamer/page/lib",
        "@aos_flatbuffers//ts:flatbuffers_ts",
    ],
)

# The page, as a directory named vision.
copy_to_directory(
    name = "vision",
    srcs = [":app"],
    root_paths = ["frc/vision/streamer/page/prod"],
)

aos_downloader_dir(
    name = "www_files",
    srcs = [":vision"],
    dir = "www",
    target_compatible_with = ["@platforms//os:linux"],
)
