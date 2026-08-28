load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//aos:flatbuffers.bzl", "cc_static_flatbuffer")
load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")

package(default_applicable_licenses = ["//:license"])

cc_library(
    name = "flatbuffer_to_proto",
    srcs = ["flatbuffer_to_proto.cc"],
    hdrs = ["flatbuffer_to_proto.h"],
    visibility = ["//visibility:public"],
    deps = [
        "@aos_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
        "@com_google_absl//absl/log:die_if_null",
        "@com_google_absl//absl/strings",
        "@com_google_protobuf//:protobuf",
    ],
)

static_flatbuffer(
    name = "flatbuffer_to_proto_test_fbs",
    srcs = ["flatbuffer_to_proto_test.fbs"],
)

cc_static_flatbuffer(
    name = "flatbuffer_to_proto_test_schema",
    function = "aos::testing::ControlDataSchema",
    target = ":flatbuffer_to_proto_test_fbs_fbs_reflection_out",
)

static_flatbuffer(
    name = "flatbuffer_to_proto_scalars_test_fbs",
    srcs = ["flatbuffer_to_proto_scalars_test.fbs"],
)

cc_static_flatbuffer(
    name = "flatbuffer_to_proto_scalars_test_schema",
    function = "aos::testing::ScalarsSchema",
    target = ":flatbuffer_to_proto_scalars_test_fbs_fbs_reflection_out",
)

static_flatbuffer(
    name = "flatbuffer_to_proto_bad_attribute_test_fbs",
    srcs = ["flatbuffer_to_proto_bad_attribute_test.fbs"],
)

cc_static_flatbuffer(
    name = "flatbuffer_to_proto_bad_attribute_test_schema",
    function = "aos::testing::BadAttributeSchema",
    target = ":flatbuffer_to_proto_bad_attribute_test_fbs_fbs_reflection_out",
)

static_flatbuffer(
    name = "flatbuffer_to_proto_spanning_test_fbs",
    srcs = ["flatbuffer_to_proto_spanning_test.fbs"],
    deps = [":flatbuffer_to_proto_test_fbs"],
)

cc_static_flatbuffer(
    name = "flatbuffer_to_proto_spanning_test_schema",
    function = "aos::testing::spanning::SpanningSchema",
    target = ":flatbuffer_to_proto_spanning_test_fbs_fbs_reflection_out",
)

cc_test(
    name = "flatbuffer_to_proto_test",
    srcs = ["flatbuffer_to_proto_test.cc"],
    deps = [
        ":flatbuffer_to_proto",
        ":flatbuffer_to_proto_bad_attribute_test_fbs",
        ":flatbuffer_to_proto_bad_attribute_test_schema",
        ":flatbuffer_to_proto_scalars_test_fbs",
        ":flatbuffer_to_proto_scalars_test_schema",
        ":flatbuffer_to_proto_spanning_test_fbs",
        ":flatbuffer_to_proto_spanning_test_schema",
        ":flatbuffer_to_proto_test_fbs",
        ":flatbuffer_to_proto_test_schema",
        "//aos:flatbuffer_merge",
        "//aos:realtime",
        "//aos:sanitizers",
        "//aos/flatbuffers:builder",
        "//aos/testing:googletest",
        "@com_google_protobuf//:protobuf",
    ],
)
