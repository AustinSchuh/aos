"""Thin alias repo bridging the LLVM toolchain's host tools into AOS."""

# `//tools/lint` needs a hermetic clang-format under both Bzlmod and WORKSPACE,
# but the repo providing it cannot be named the same way in each mode.  Under
# Bzlmod the `llvm` extension from `toolchains_llvm` only runs in the root
# module, so a module consuming AOS as a dependency cannot name `@llvm_toolchain`
# directly and the extension in //tools/cpp:extensions.bzl injects it instead.
# Under WORKSPACE the repo is declared directly in WORKSPACE.
#
# Both modes instantiate this rule, so the single `@aos_llvm_toolchain//:clang-format`
# label resolves either way and the BUILD files need no `select()`.
#
# `clang-format` is a host tool, so the exec-configured convenience target
# (already pinned to the host distribution) is correct for every target
# platform -- no per-arch repos needed.

def _llvm_tool_alias_repo_impl(rctx):
    rctx.file("WORKSPACE", "")
    rctx.file("BUILD.bazel", """\
alias(
    name = "clang-format",
    actual = "@{src}//:clang-format",
    visibility = ["//visibility:public"],
)
""".format(src = rctx.attr.src_repo))

llvm_tool_alias_repo = repository_rule(
    implementation = _llvm_tool_alias_repo_impl,
    attrs = {
        "src_repo": attr.string(
            doc = "Name of the LLVM toolchain repo to alias.  Under Bzlmod this " +
                  "is the apparent name injected into the extension; under " +
                  "WORKSPACE it is the global repo name.",
        ),
    },
)
