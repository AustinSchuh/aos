load("@bazel_skylib//rules:copy_file.bzl", "copy_file")
load("@rules_cc//cc:cc_import.bzl", "cc_import")
load("@rules_cc//cc:cc_library.bzl", "cc_library")

# LLVM's libunwind, taken from the LLVM distribution the toolchain comes from.
# The headers are copied into this package so they can be put on the include
# path without pulling in the rest of the distribution's include/ directory.

_TARGET_COMPATIBLE_WITH = select({
    "//tools:cpu_arm64": ["@platforms//os:linux"],
    "//tools:cpu_k8": ["@platforms//os:linux"],
    "//conditions:default": ["@platforms//:incompatible"],
})

_HEADERS = [
    "__libunwind_config.h",
    "libunwind.h",
]

[copy_file(
    name = "copy_" + header,
    src = select({
        "//tools:cpu_arm64": "@llvm_aarch64//:include/" + header,
        "//tools:cpu_k8": "@llvm_k8//:include/" + header,
    }),
    out = "include/" + header,
    target_compatible_with = _TARGET_COMPATIBLE_WITH,
) for header in _HEADERS]

# Copied too, rather than handed straight to cc_import: rules_rust's
# rustdoc_test strips each static library's root from its command line, and a
# source file from another repository has an empty root, which strips every /.
copy_file(
    name = "copy_libunwind.a",
    src = select({
        "//tools:cpu_arm64": "@llvm_aarch64//:lib/aarch64-unknown-linux-gnu/libunwind.a",
        "//tools:cpu_k8": "@llvm_k8//:lib/x86_64-unknown-linux-gnu/libunwind.a",
    }),
    out = "lib/libunwind.a",
    target_compatible_with = _TARGET_COMPATIBLE_WITH,
)

cc_import(
    name = "libunwind_archive",
    static_library = "lib/libunwind.a",
    target_compatible_with = _TARGET_COMPATIBLE_WITH,
)

cc_library(
    name = "libunwind",
    hdrs = ["include/" + header for header in _HEADERS],
    includes = ["include"],
    target_compatible_with = _TARGET_COMPATIBLE_WITH,
    visibility = ["//visibility:public"],
    deps = [":libunwind_archive"],
)
