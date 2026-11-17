load("@aspect_rules_js//js:defs.bzl", "js_test")

# The vision page end to end: built as deployed, in headless Chrome, against a
# fake robot_server.  See page.test.mjs.
js_test(
    name = "page_test",
    size = "medium",
    # The robot's pages are used from runfiles as they are, not copied.
    copy_data_to_bin = False,
    data = [
        "browser.mjs",
        "fake_robot.mjs",
        "testdata/localhost.crt",
        "testdata/localhost.key",
        "//frc/vision/streamer/page:vision",
        "//frc/www:starter.html",
        "//frc/www:starter_files",
        "@chrome_linux//:all",
        "@chrome_linux//:chrome",
        # Chrome's libraries and fonts: the test machines have neither.
        "@amd64_debian_sysroot//:sysroot_files",
        "@amd64_debian_sysroot//:usr/bin/ptxas",
    ],
    entry_point = "page.test.mjs",
    env = {
        "CHROME": "$(rlocationpath @chrome_linux//:chrome)",
        # A file the sysroot exports, to find it by.
        "SYSROOT_FILE": "$(rlocationpath @amd64_debian_sysroot//:usr/bin/ptxas)",
        "STARTER_HTML": "$(rlocationpath //frc/www:starter.html)",
        "TEST_CERT": "$(rlocationpath testdata/localhost.crt)",
        "VISION_DIR": "$(rlocationpath //frc/vision/streamer/page:vision)",
    },
    target_compatible_with = [
        "@platforms//cpu:x86_64",
        "@platforms//os:linux",
    ],
)
