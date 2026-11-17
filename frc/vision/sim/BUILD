load("@rules_cc//cc:cc_binary.bzl", "cc_binary")

package(default_visibility = ["//visibility:public"])

# The vision robot, simulated: robot_server, web_proxy, calibration_server and
# the exposure adjuster as on the robot, with fake cameras and a fake starter.
# For using the pages on a laptop, and for their end-to-end tests.  See
# vision_robot_sim.cc.
cc_binary(
    name = "vision_robot_sim",
    testonly = True,
    srcs = ["vision_robot_sim.cc"],
    data = [
        "//frc/vision:aos_config.json",
        "//frc/vision/testdata:constants.json",
    ],
    target_compatible_with = [
        "//tools/platforms/rust:has_support",
        "@platforms//os:linux",
    ],
    deps = [
        "//aos:configuration",
        "//aos:flatbuffer_merge",
        "//aos:init",
        "//aos/events:simulated_event_loop",
        "//aos/events:tokio_simulation",
        "//aos/network:web_proxy",
        "//aos/network/robot_server:robot_server_lib",
        "//aos/starter:starter_fbs",
        "//aos/starter:starter_rpc_fbs",
        "//frc/constants:constants_sender_lib",
        "//frc/vision:camera_constants_fbs",
        "//frc/vision:camera_constants_list_fbs",
        "//frc/vision:field_side_exposure_adjuster_lib",
        "//frc/vision:vision_fbs",
        "//frc/vision/calibration:calibration_server_lib",
        "//frc/vision/calibration:charuco_rendering",
        "//frc/vision/util:charuco_board",
        "//third_party:opencv",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
        "@com_google_absl//absl/strings",
    ],
)
