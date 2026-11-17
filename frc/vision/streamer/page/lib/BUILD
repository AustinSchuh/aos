load("@aspect_rules_js//js:defs.bzl", "js_library", "js_test")
load("//tools/build_rules:js.bzl", "ts_project")

package(default_visibility = ["//frc/vision/streamer/page:__subpackages__"])

# The camera page's logic, in strict TypeScript, kept out of its Angular
# components so it can be tested on Node.

LIB_SRCS = [
    "calibration.ts",
    "format.ts",
    "distortion.ts",
    "intrinsics.ts",
    "rates.ts",
    "signalling.ts",
]

ts_project(
    name = "lib",
    srcs = LIB_SRCS,
    tsconfig = "//frc/vision/streamer/page:tsconfig",
    deps = [":package_json"],
)

ts_project(
    name = "test_lib",
    testonly = True,
    srcs = [src[:-len(".ts")] + ".test.ts" for src in LIB_SRCS],
    tsconfig = "//frc/vision/streamer/page:tsconfig",
    deps = [
        ":lib",
        ":package_json",
        "//:node_modules/@types/node",
    ],
)

[js_test(
    name = src[:-len(".ts")] + "_test",
    data = [
        ":package_json",
        ":test_lib",
    ],
    entry_point = src[:-len(".ts")] + ".test.js",
) for src in LIB_SRCS]

# "type": "module", so Node runs the compiled ES modules.
js_library(
    name = "package_json",
    srcs = ["package.json"],
)
