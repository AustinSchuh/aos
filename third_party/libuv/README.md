# libuv Bazel build support

`libuv.BUILD.bazel` is a verbatim copy of the libuv `1.48.0.bcr.2` overlay in
the Bazel Central Registry:

    https://bcr.bazel.build/modules/libuv/1.48.0.bcr.2/overlay/BUILD.bazel

Under bzlmod the registry applies that overlay itself and this copy is unused.
WORKSPACE mode has no way to apply a registry overlay, so `repositories.bzl`
passes this file as `build_file` instead.  Both modes therefore build identical
sources.  Unlike liburing, the overlay is a single BUILD file with no `.bzl`
beside it, so a `build_file` does the whole job and there is no patch here.

To update after a libuv bump, re-download the overlay for the new version --
its sha256 is in that version's `source.json`, so a mismatch is detectable --
and update the version in `MODULE.bazel` and `repositories.bzl` to match.

This whole directory goes away with WORKSPACE support.
