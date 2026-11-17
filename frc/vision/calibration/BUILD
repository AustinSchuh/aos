load("@aos//aos/flatbuffers:ts.bzl", "flatbuffer_ts_library")
load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//aos:config.bzl", "aos_config")
load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")

cc_binary(
    name = "modify_extrinsics",
    srcs = [
        "modify_extrinsics.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:configuration",
        "//aos:init",
        "//aos/events:event_loop",
        "//frc/vision:calibration_fbs",
        "//frc/vision/util:vision_util_lib",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
        "@com_google_absl//absl/strings:str_format",
        "@eigen",
    ],
)

cc_library(
    name = "extrinsics_calibration",
    srcs = [
        "calibration_accumulator.cc",
        "calibration_accumulator.h",
        "extrinsics_calibration.cc",
        "extrinsics_calibration.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:init",
        "//aos/analysis:in_process_plotter",
        "//aos/events/logging:log_reader",
        "//frc/control_loops/drivetrain:improved_down_estimator",
        "//frc/vision/util:charuco_lib",
        "//frc/vision/util:foxglove_image_converter_lib",
        "//frc/vision/util:visualize_robot",
        "//frc/wpilib:imu_batch_fbs",
        "//frc/wpilib:imu_fbs",
        "//third_party:opencv",
        "@ceres-solver//:ceres",
        "@com_github_foxglove_schemas//:CompressedImage_schema",
        "@com_github_foxglove_schemas//:ImageAnnotations_schema",
        "@com_google_absl//absl/strings:str_format",
        "@eigen",
    ],
)

cc_library(
    name = "capture_gate",
    srcs = ["capture_gate.cc"],
    hdrs = ["capture_gate.h"],
    visibility = ["//visibility:public"],
    deps = ["@eigen"],
)

cc_test(
    name = "capture_gate_test",
    srcs = ["capture_gate_test.cc"],
    deps = [
        ":capture_gate",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "intrinsics_solver",
    srcs = ["intrinsics_solver.cc"],
    hdrs = ["intrinsics_solver.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//third_party:opencv",
        "@ceres-solver//:ceres",
        "@com_google_absl//absl/log:absl_log",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
    ],
)

cc_test(
    name = "intrinsics_solver_test",
    srcs = ["intrinsics_solver_test.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":intrinsics_solver",
        "//aos/testing:googletest",
        "//frc/vision/util:charuco_board",
        "//third_party:opencv",
        "@com_google_absl//absl/log:absl_log",
    ],
)

cc_library(
    name = "intrinsics_calibration_lib",
    srcs = [
        "intrinsics_calibration_lib.cc",
    ],
    hdrs = [
        "intrinsics_calibration_lib.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":calibration_validator",
        ":capture_gate",
        ":intrinsics_solver",
        "//aos/events:event_loop",
        "//frc/control_loops/drivetrain:improved_down_estimator",
        "//frc/vision:vision_fbs",
        "//frc/vision/util:charuco_lib",
        "//frc/vision/util:vision_util_lib",
        "//frc/wpilib:imu_batch_fbs",
        "//frc/wpilib:imu_fbs",
        "//third_party:opencv",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
        "@com_google_absl//absl/log:die_if_null",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@eigen",
    ],
)

cc_binary(
    name = "intrinsics_calibration",
    srcs = [
        "intrinsics_calibration.cc",
    ],
    data = ["//frc/vision:aos_config"],
    visibility = ["//visibility:public"],
    deps = [
        ":intrinsics_calibration_lib",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc/control_loops/drivetrain:improved_down_estimator",
        "//frc/vision:vision_fbs",
        "//frc/vision/util:charuco_lib",
        "//frc/wpilib:imu_batch_fbs",
        "//frc/wpilib:imu_fbs",
        "//third_party:opencv",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
        "@com_google_absl//absl/strings:str_format",
        "@eigen",
    ],
)

aos_config(
    name = "intrinsics_test_config",
    src = "intrinsics_test_config_source.json",
    flatbuffers = [
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:timestamp_fbs",
        "//aos/network:remote_message_fbs",
        "//frc/control_loops/drivetrain/localization:localizer_output_fbs",
        "//frc/orin:hardware_stats_fbs",
        "//frc/vision:calibration_fbs",
        "//frc/vision:target_map_fbs",
        "//frc/vision:vision_fbs",
        "//aos/util:filesystem_fbs",
        "@com_github_foxglove_schemas//:schemas",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/events:aos_config",
        "//frc/control_loops/drivetrain:aos_config",
        "//frc/input:aos_config",
    ],
)

cc_test(
    name = "intrinsics_calibration_test",
    timeout = "long",
    srcs = [
        "intrinsics_calibration_test.cc",
    ],
    data = [
        ":intrinsics_test_config",
        "//frc/vision/test_calib_files",
        "@intrinsic_calibration_test_images",
    ],
    shard_count = 4,
    deps = [
        ":intrinsics_calibration",
        ":intrinsics_calibration_lib",
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "//aos/events:shm_event_loop",
        "//aos/events:simulated_event_loop",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//aos/testing:random_seed",
        "//aos/testing:tmpdir",
        "//aos/util:math",
        "//frc/vision:calibration_fbs",
        "//third_party:opencv",
        "@bazel_tools//tools/cpp/runfiles",
        "@com_google_absl//absl/status",
    ],
)

cc_library(
    name = "calibrate_multi_cameras_lib",
    srcs = ["calibrate_multi_cameras_lib.cc"],
    hdrs = ["calibrate_multi_cameras_lib.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":extrinsics_calibration",
        "//aos:init",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_reader",
        "//aos/util:mcap_logger",
        "//frc/constants:constants_sender_lib",
        "//frc/control_loops:pose",
        "//frc/vision:calibration_fbs",
        "//frc/vision/detection:target_mapper",
        "//frc/vision/util:charuco_lib",
        "//frc/vision/util:vision_util_lib",
        "//third_party:opencv",
        "@eigen",
    ],
)

cc_binary(
    name = "camera_constants_formatter",
    srcs = ["camera_constants_formatter.cc"],
    visibility = ["//visibility:public"],
    deps = [
        ":calibration_validator",
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "//aos/util:file",
        "//frc/vision:camera_constants_list_fbs",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
        "@com_google_absl//absl/strings",
    ],
)

cc_binary(
    name = "robot_transformation_calculator",
    srcs = [
        "robot_transformation_calculator.cc",
    ],
    deps = [
        "//aos:init",
        "@com_google_absl//absl/log:absl_log",
        "@eigen",
    ],
)

cc_binary(
    name = "calibrate_multi_cameras",
    srcs = [
        "calibrate_multi_cameras.cc",
        "vision_util.cc",
        "vision_util.h",
    ],
    data = [
        "//frc/vision:aos_config",
        "//frc/vision:constants.json",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":calibrate_multi_cameras_lib",
        ":extrinsics_calibration",
        "//aos:init",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_reader",
        "//aos/util:mcap_logger",
        "//frc/constants:constants_sender_lib",
        "//frc/control_loops:pose",
        "//frc/vision:calibration_fbs",
        "//frc/vision/detection:target_mapper",
        "//frc/vision/field_map:simulated_constants_sender_lib",
        "//frc/vision/util:charuco_lib",
        "//frc/vision/util:vision_util_lib",
        "//third_party:opencv",
        "@eigen",
    ],
)

cc_library(
    name = "calibration_validator",
    srcs = ["calibration_validator.cc"],
    hdrs = ["calibration_validator.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc/orin:distortion",
        "//frc/vision:calibration_fbs",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_test(
    name = "calibration_validator_test",
    srcs = ["calibration_validator_test.cc"],
    data = ["//frc/vision/test_calib_files"],
    deps = [
        ":calibration_validator",
        "//aos:flatbuffers",
        "//aos:json_to_flatbuffer",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//frc/orin:distortion",
        "//frc/vision:calibration_fbs",
        "@com_google_absl//absl/log:absl_log",
        "@com_google_absl//absl/status",
    ],
)

static_flatbuffer(
    name = "calibration_command_fbs",
    srcs = ["calibration_command.fbs"],
    visibility = ["//visibility:public"],
    deps = [
        ":calibration_status_fbs",
        "//frc/vision:calibration_fbs",
    ],
)

static_flatbuffer(
    name = "calibration_status_fbs",
    srcs = ["calibration_status.fbs"],
    visibility = ["//visibility:public"],
)

static_flatbuffer(
    name = "calibration_session_fbs",
    srcs = ["calibration_session.fbs"],
    visibility = ["//visibility:public"],
    deps = [
        ":calibration_status_fbs",
        "//frc/vision:calibration_fbs",
    ],
)

flatbuffer_ts_library(
    name = "calibration_session_ts_fbs",
    srcs = [
        "calibration_command.fbs",
        "calibration_session.fbs",
        "calibration_status.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    ts_files = [
        "frc/vision/calibration.ts",
        "frc/vision/calibration/command-action.ts",
        "frc/vision/calibration/command.ts",
        "frc/vision/calibration/session-state.ts",
        "frc/vision/calibration/frame-verdict.ts",
        "frc/vision/calibration/status.ts",
        "frc/vision/calibration/corner.ts",
        "frc/vision/calibration/residual.ts",
        "frc/vision/calibration/capture.ts",
        "frc/vision/calibration/capture-residuals.ts",
        "frc/vision/calibration/solve.ts",
        "frc/vision/calibration/saved-board.ts",
        "frc/vision/calibration/session.ts",
        "frc/vision/calibration/camera-board.ts",
        "frc/vision/calibration/board-store.ts",
        "frc/vision/calibration/validation-check.ts",
        "frc/vision/calibration/check-result.ts",
        "frc/vision/calibration/lens-validation.ts",
        "frc/vision/calibration/board-spec.ts",
        "frc/vision/calibration/camera-calibration.ts",
        "frc/vision/calibration/transformation-matrix.ts",
    ],
    visibility = ["//visibility:public"],
    deps = ["//frc/vision:calibration_ts_fbs"],
)

cc_library(
    name = "charuco_detector",
    srcs = ["charuco_detector.cc"],
    hdrs = ["charuco_detector.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = ["//third_party:opencv"],
)

cc_library(
    name = "calibration_server_lib",
    srcs = ["calibration_server_lib.cc"],
    hdrs = ["calibration_server_lib.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":calibration_command_fbs",
        ":calibration_session_fbs",
        ":calibration_status_fbs",
        ":calibration_validator",
        ":capture_gate",
        ":charuco_detector",
        ":intrinsics_calibration_lib",
        ":intrinsics_solver",
        ":validation",
        "//aos:flatbuffer_merge",
        "//aos:flatbuffers",
        "//aos:json_to_flatbuffer",
        "//aos/events:event_loop",
        "//frc/vision:calibration_fbs",
        "//frc/vision:camera_constants_fbs",
        "//frc/vision:vision_fbs",
        "//frc/vision/util:charuco_board",
        "//frc/vision/util:charuco_lib",
        "//frc/vision/util:vision_util_lib",
        "//third_party:opencv",
        "@com_github_foxglove_schemas//:schemas",
        "@com_google_absl//absl/log:absl_log",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@eigen",
    ],
)

cc_binary(
    name = "calibration_server",
    srcs = ["calibration_server_main.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":calibration_server_lib",
        "//aos:configuration",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//aos/network:team_number",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/strings",
    ],
)

aos_config(
    name = "calibration_server_test_config",
    src = "calibration_server_test_config_source.json",
    flatbuffers = [
        ":calibration_command_fbs",
        ":calibration_session_fbs",
        ":calibration_status_fbs",
        "//frc/vision:camera_constants_fbs",
        "//frc/vision:vision_fbs",
        "@com_github_foxglove_schemas//:schemas",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = ["//aos/events:aos_config"],
)

cc_test(
    name = "calibration_server_test",
    srcs = ["calibration_server_test.cc"],
    data = [":calibration_server_test_config"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":calibration_server_lib",
        ":charuco_detector",
        "//aos/events:simulated_event_loop",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//aos/testing:tmpdir",
        "//frc/vision/util:charuco_board",
        "//third_party:opencv",
    ],
)

cc_test(
    name = "charuco_detector_test",
    srcs = ["charuco_detector_test.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":charuco_detector",
        "//aos/testing:googletest",
        "//frc/vision/util:charuco_board",
        "//third_party:opencv",
        "@com_google_absl//absl/log:absl_log",
    ],
)

cc_library(
    name = "validation",
    srcs = ["validation.cc"],
    hdrs = ["validation.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = ["//third_party:opencv"],
)

cc_test(
    name = "validation_test",
    srcs = ["validation_test.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":validation",
        "//aos/testing:googletest",
        "//frc/vision/util:charuco_board",
        "//third_party:opencv",
    ],
)
