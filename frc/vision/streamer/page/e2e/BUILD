load("@aspect_rules_js//js:defs.bzl", "js_test")

# The vision page end to end: built as deployed, in headless Chrome, against
# the robot simulated.  See page.test.mjs.
js_test(
    name = "page_test",
    size = "large",
    # The robot's pages are used from runfiles as they are, not copied.
    copy_data_to_bin = False,
    data = [
        "browser.mjs",
        "testdata/localhost.crt",
        "testdata/localhost.key",
        "//frc/vision:aos_config.json",
        "//frc/vision/testdata:constants.json",
        "//frc/vision/field_map/www:field.html",
        "//frc/vision/field_map/www:www_files_filegroup",
        "//frc/vision/sim:vision_robot_sim",
        "//frc/vision/streamer/page:vision",
        "//frc/www:starter.html",
        "@chrome_linux//:all",
        "@chrome_linux//:chrome",
        # Chrome's libraries and fonts: the test machines have neither.
        "@amd64_debian_sysroot//:sysroot_files",
        "@amd64_debian_sysroot//:usr/lib/os-release",
    ],
    entry_point = "page.test.mjs",
    env = {
        "CHROME": "$(rlocationpath @chrome_linux//:chrome)",
        "FIELD_HTML": "$(rlocationpath //frc/vision/field_map/www:field.html)",
        "ROBOT_CONFIG": "$(rlocationpath //frc/vision:aos_config.json)",
        "ROBOT_CONSTANTS": "$(rlocationpath //frc/vision/testdata:constants.json)",
        "ROBOT_SIM": "$(rlocationpath //frc/vision/sim:vision_robot_sim)",
        "STARTER_HTML": "$(rlocationpath //frc/www:starter.html)",
        # A file the sysroot exports, to find it by.
        "SYSROOT_FILE": "$(rlocationpath @amd64_debian_sysroot//:usr/lib/os-release)",
        "TEST_CERT": "$(rlocationpath testdata/localhost.crt)",
        "VISION_DIR": "$(rlocationpath //frc/vision/streamer/page:vision)",
    },
    target_compatible_with = [
        "@platforms//cpu:x86_64",
        "@platforms//os:linux",
    ],
)
