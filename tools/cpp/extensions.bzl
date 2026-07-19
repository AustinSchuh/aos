load("//:repositories_internal.bzl", "arm_frc_linux_gnueabi_repo_repo", "gcc_arm_none_eabi_repo")
load("//tools/cpp:llvm_tools.bzl", "llvm_tool_alias_repo")

def _dev_toolchains_extension_impl(_ctx):
    arm_frc_linux_gnueabi_repo_repo()

    gcc_arm_none_eabi_repo()

dev_toolchains_extension = module_extension(
    implementation = _dev_toolchains_extension_impl,
)

# The `llvm` extension from `toolchains_llvm` is hard-coded to only run in the
# root module, so a module that consumes AOS as a dependency cannot import the
# `@llvm_toolchain` repo whose `clang-format` convenience target AOS's own
# `//tools/lint` uses for a hermetic formatter.
#
# This extension bridges that gap: whichever module is root injects its
# `@llvm_toolchain` repo into the extension (via `inject_repo`), and the
# extension re-publishes a thin alias repo (`@aos_llvm_toolchain`) that AOS's
# BUILD files depend on. The same `@aos_llvm_toolchain//:clang-format` label then
# resolves whether AOS is built standalone or pulled in as a dependency.
#
# `clang-format` is a host tool, so we only need the exec-configured convenience
# target `@llvm_toolchain//:clang-format` (already resolved to the host
# distribution) -- no per-target-arch repos or `select()` required.
#
# The alias repo itself lives in //tools/cpp:llvm_tools.bzl so that WORKSPACE can
# declare the identical repo without going through a module extension.

def _llvm_tools_extension_impl(_ctx):
    llvm_tool_alias_repo(name = "aos_llvm_toolchain", src_repo = "llvm_toolchain")

llvm_tools_extension = module_extension(
    implementation = _llvm_tools_extension_impl,
)
