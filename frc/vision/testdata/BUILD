load("//frc/vision:validator.bzl", "camera_constants_json")
load("//tools/build_rules:template.bzl", "jinja2_template")

# The vision constants for tests: only team 0's "test" robot, so that
# recalibrating a real robot, which changes frc/vision/constants.json, can't
# break a test.
jinja2_template(
    name = "constants_unvalidated.json",
    src = "constants.jinja2.json",
    includes = glob(["*.json"]),
)

camera_constants_json(
    name = "constants_json",
    src = ":constants_unvalidated.json",
    out = "constants.json",
)
