load("@rules_cc//cc:cc_library.bzl", "cc_library")

# Copy of ceres example code from their repo

# Just the Pose3d/Constraint3d structs.  These are plain Eigen types with g2o
# serialization on them and pull in nothing else, so they are split out from
# the cost functions below: callers that only want to pass poses around should
# not have to depend on ceres, or be restricted to the platforms it builds for.
cc_library(
    name = "pose_graph_3d_types",
    hdrs = ["types.h"],
    visibility = ["//visibility:public"],
    deps = ["@eigen"],
)

cc_library(
    name = "pose_graph_3d_lib",
    hdrs = [
        "pose_graph_3d_error_term.h",
        "read_g2o.h",
    ],
    copts = [
        # Needed to silence GFlags complaints.
        "-Wno-sign-compare",
        "-Wno-unused-parameter",
        "-Wno-format-nonliteral",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":pose_graph_3d_types",
        "@ceres-solver//:ceres",
        "@com_google_absl//absl/flags:flag",
        "@eigen",
    ],
)
