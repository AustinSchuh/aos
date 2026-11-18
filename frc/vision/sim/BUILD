load("@rules_cc//cc:cc_binary.bzl", "cc_binary")

package(default_visibility = ["//visibility:public"])

# The names the sysroot's OpenSSL and curl define, which the simulation's own
# BoringSSL and curl (web_proxy's, through rawrtc) keep to themselves.
# Exported, they would stand in for the sysroot's in the GStreamer plugins
# camera_streamer loads (dtls), which crashes: glibc lets an executable's
# unversioned definition answer a library's versioned reference.  On the robot
# they're separate processes.
genrule(
    name = "private_symbols",
    srcs = [
        "@amd64_debian_sysroot//:libcurl4-filegroup",
        "@amd64_debian_sysroot//:libssl3-filegroup",
    ],
    outs = ["private_symbols.lds"],
    cmd = """
      libraries=$$(for f in $(SRCS); do
        case "$$f" in
          */libcrypto.so.3 | */libssl.so.3 | */libcurl.so.4.*) echo "$$f" ;;
        esac
      done)
      (
        echo '{ local:'
        $(NM) -D --defined-only $$libraries |
          awk 'NF == 3 { sub(/@.*/, "", $$3); print "  " $$3 ";" }' | sort -u
        echo '};'
      ) > $@
    """,
    target_compatible_with = ["@platforms//cpu:x86_64"],
    toolchains = ["@rules_cc//cc:current_cc_toolchain"],
    tools = ["@rules_cc//cc:current_cc_toolchain"],
)

# The vision robot, simulated: robot_server, web_proxy, calibration_server,
# camera_streamer and the exposure adjuster as on the robot, with fake cameras
# and a fake starter.  For using the pages on a laptop, and for their
# end-to-end tests.  See vision_robot_sim.cc.
cc_binary(
    name = "vision_robot_sim",
    testonly = True,
    srcs = ["vision_robot_sim.cc"],
    additional_linker_inputs = select({
        "@platforms//cpu:x86_64": [":private_symbols.lds"],
        "//conditions:default": [],
    }),
    data = [
        "//frc/vision:aos_config.json",
        "//frc/vision/testdata:constants.json",
    ],
    linkopts = select({
        "@platforms//cpu:x86_64": [
            "-Wl,--version-script=$(location :private_symbols.lds)",
            # Most of OpenSSL's names, BoringSSL doesn't define.
            "-Wl,--undefined-version",
        ],
        "//conditions:default": [],
    }),
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
        "//frc/vision/streamer:camera_streamer_lib",
        "//frc/vision/util:charuco_board",
        "//third_party:opencv",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
        "@com_google_absl//absl/strings",
    ],
)
