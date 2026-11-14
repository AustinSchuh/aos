load("@rules_cc//cc:cc_library.bzl", "cc_library")

# NvMedia's image JPEG decoder; see nvmedia_ijpd.h.
cc_library(
    name = "nvmedia_ijpd",
    hdrs = ["nvmedia_ijpd.h"],
    includes = ["."],
    target_compatible_with = [
        "@platforms//cpu:arm64",
        "@platforms//os:linux",
    ],
    visibility = ["//visibility:public"],
    deps = ["@arm64_debian_sysroot//:nvmedia_ijpd"],
)
