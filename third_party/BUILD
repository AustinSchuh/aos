# Incompatible library in case one isn't available for a specific architecture.

load("@rules_cc//cc:cc_library.bzl", "cc_library")

cc_library(
    name = "unavailable",
    target_compatible_with = ["@platforms//:incompatible"],
)

cc_library(
    name = "wpilib",
    linkstatic = True,
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    visibility = ["//visibility:public"],
    deps = ["//frc/wpilib/ahal"],
)

cc_library(
    name = "wpilib_hal",
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    visibility = ["//visibility:public"],
    deps = ["@com_github_wpilibsuite_allwpilib//hal:wpiHal.static"],
)

cc_library(
    name = "phoenix",
    linkopts = [
        "-Wl,-rpath",
        "-Wl,.",
    ],
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    visibility = ["//visibility:public"],
    deps = [
        "@ctre_phoenix6_tools_athena//:tools",
        "@ctre_phoenix6_tools_headers//:tools",
        "@ctre_phoenix_api_cpp_athena//:api-cpp",
        "@ctre_phoenix_api_cpp_headers//:api-cpp",
        "@ctre_phoenix_cci_athena//:cci",
        "@ctre_phoenix_cci_headers//:cci",
    ],
)

cc_library(
    name = "phoenix6",
    linkopts = [
        "-Wl,-rpath",
        "-Wl,.",
    ],
    visibility = ["//visibility:public"],
    deps = select({
        "//tools/platforms/hardware:roborio": [
            "//third_party:wpilib",
            "@ctre_phoenix6_api_cpp_athena//:api-cpp",
            "@ctre_phoenix6_api_cpp_headers//:api-cpp",
            "@ctre_phoenix6_tools_athena//:tools",
            "@ctre_phoenix6_tools_headers//:tools",
        ],
        "@platforms//cpu:arm64": [
            "@ctre_phoenix6_arm64//:headers",
            "@ctre_phoenix6_arm64//:shared_libraries_lib",
        ],
        "//conditions:default": [":unavailable"],
    }),
)

# OpenCV.  The ORIN keeps using the prebuilt opencv4 out of its sysroot; x86
# and OSX build it from source out of the BCR module, which is the only way to
# get it on OSX at all.  Every one of those is >= 4.7, so the aruco/charuco
# detectors live in objdetect everywhere and we do not need the contrib module.
#
# We list the individual BCR modules rather than @opencv//:opencv so that the
# windowing and video-capture code is not dragged into everything that just
# wants to manipulate an image; that lives in :opencv_highgui instead.
_OPENCV_BCR_MODULES = [
    "@opencv//:core",
    "@opencv//:imgproc",
    "@opencv//:imgcodecs",
    "@opencv//:calib3d",
    "@opencv//:objdetect",
]

cc_library(
    name = "opencv",
    target_compatible_with = select({
        "//tools:has_msan": ["@platforms//:incompatible"],
        "//conditions:default": [],
    }),
    visibility = ["//visibility:public"],
    deps = select({
        "//tools:arm64_macos": _OPENCV_BCR_MODULES,
        "//tools:cpu_arm64": ["@arm64_debian_sysroot//:opencv4"],
        "//tools:cpu_k8": _OPENCV_BCR_MODULES,
        "//conditions:default": [":unavailable"],
    }),
)

# The windowing half of OpenCV (cv::imshow and friends).  This pulls in videoio
# too, and with it the AVFoundation and Cocoa objc_library targets on OSX; our
# toolchain grows Objective-C support via
# //third_party/rules_cc:0001-Declare-the-Objective-C-compile-actions.patch.
# On x86 this comes from the BCR module with its GTK3 backend turned on (see
# --@opencv//:with_gtk3 and --@opencv//:gtk3 in .bazelrc); the ORIN's prebuilt
# opencv4 already contains it.
cc_library(
    name = "opencv_highgui",
    target_compatible_with = select({
        "//tools:has_msan": ["@platforms//:incompatible"],
        "//conditions:default": [],
    }),
    visibility = ["//visibility:public"],
    deps = select({
        "//tools:arm64_macos": ["@opencv//:highgui"],
        "//tools:cpu_arm64": ["@arm64_debian_sysroot//:opencv4"],
        "//tools:cpu_k8": ["@opencv//:highgui"],
        "//conditions:default": [":unavailable"],
    }),
)

# GTK3 for the BCR OpenCV's highgui backend, taken from the sysroot rather than
# the host.  Referenced by --@opencv//:gtk3.  Only x86 needs it; the ORIN uses
# its prebuilt opencv4 instead.
alias(
    name = "gtk3",
    actual = select({
        "//tools:cpu_k8": "@amd64_debian_sysroot//:gtk+-3.0",
        "//conditions:default": ":unavailable",
    }),
    visibility = ["//visibility:public"],
)

cc_library(
    name = "gstreamer",
    target_compatible_with = select({
        "//tools:has_msan": ["@platforms//:incompatible"],
        "//conditions:default": [],
    }),
    visibility = ["//visibility:public"],
    deps = select({
        "//tools:arm64_macos": [":unavailable"],
        "//tools:cpu_arm64": [
            "@arm64_debian_sysroot//:gstreamer-1.0",
            "@arm64_debian_sysroot//:gstreamer-app-1.0",
            "@arm64_debian_sysroot//:gstreamer-webrtc-1.0",
        ],
        "//tools:cpu_k8": [
            "@amd64_debian_sysroot//:gstreamer-1.0",
            "@amd64_debian_sysroot//:gstreamer-app-1.0",
            "@amd64_debian_sysroot//:gstreamer-webrtc-1.0",
        ],
        "//conditions:default": [":unavailable"],
    }),
)

cc_library(
    name = "cudart",
    target_compatible_with = select({
        "//tools:has_msan": ["@platforms//:incompatible"],
        "//conditions:default": [],
    }),
    visibility = ["//visibility:public"],
    deps = select({
        "//tools:arm64_macos": [":unavailable"],
        "//tools:cpu_arm64": ["@arm64_debian_sysroot//:cudart-12.6"],
        "//tools:cpu_k8": ["@amd64_debian_sysroot//:cudart"],
        "//conditions:default": [":unavailable"],
    }),
)

cc_library(
    name = "tensorrt",
    target_compatible_with = select({
        "//tools:has_msan": ["@platforms//:incompatible"],
        "//conditions:default": [],
    }),
    visibility = ["//visibility:public"],
    deps = select({
        "//tools:arm64_macos": [":unavailable"],
        "//tools:cpu_arm64": ["@arm64_debian_sysroot//:tensorrt"],
        "//tools:cpu_k8": ["@amd64_tensorrt//:tensorrt"],
        "//conditions:default": [":unavailable"],
    }),
)

cc_library(
    name = "libturbojpeg",
    target_compatible_with = select({
        "//tools:has_msan": ["@platforms//:incompatible"],
        "//conditions:default": [],
    }),
    visibility = ["//visibility:public"],
    deps = select({
        "//tools:arm64_macos": [":unavailable"],
        "//tools:cpu_arm64": ["@arm64_debian_sysroot//:libturbojpeg"],
        "//tools:cpu_k8": ["@amd64_debian_sysroot//:libturbojpeg"],
        "//conditions:default": [":unavailable"],
    }),
)
