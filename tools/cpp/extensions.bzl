load("@com_grail_bazel_toolchain//toolchain:rules.bzl", "llvm", "toolchain")
load("//:repositories_internal.bzl", "arm_frc_linux_gnueabi_repo_repo", "gcc_arm_none_eabi_repo")

def _llvm_toolchain_extension_impl(_ctx):
    llvm_version = "21.1.1"

    llvm(
        name = "llvm_k8",
        distribution = "clang+llvm-%s-x86_64-linux-gnu-ubuntu-22.04.tar.zst" % llvm_version,
        llvm_version = llvm_version,
    )

    llvm(
        name = "llvm_aarch64",
        distribution = "clang+llvm-%s-aarch64-linux-gnu.tar.zst" % llvm_version,
        llvm_version = llvm_version,
    )

    llvm_conlyopts = ["-std=gnu99"]
    llvm_copts = [
        "-D__STDC_FORMAT_MACROS",
        "-D__STDC_CONSTANT_MACROS",
        "-D__STDC_LIMIT_MACROS",
        "-D_FILE_OFFSET_BITS=64",
        "-fmessage-length=100",
        "-fmacro-backtrace-limit=0",
        "-ggdb3",
        "-Wno-deprecated-declarations",
    ]
    llvm_cxxopts = ["-std=gnu++20"]

    toolchain(
        name = "llvm_toolchain",
        additional_target_compatible_with = {},
        conlyopts = {
            "linux-aarch64": llvm_conlyopts,
            "linux-x86_64": llvm_conlyopts,
        },
        copts = {
            "linux-aarch64": llvm_copts,
            "linux-x86_64": llvm_copts,
        },
        cxxopts = {
            "linux-aarch64": llvm_cxxopts,
            "linux-x86_64": llvm_cxxopts,
        },
        llvm_version = llvm_version,
        standard_libraries = {
            "linux-aarch64": "libstdc++-14.3.0",
            "linux-x86_64": "libstdc++-12",
        },
        static_libstdcxx = False,
        sysroot = {
            "linux-aarch64": "@@arm64_debian_sysroot+//:sysroot_files",
            "linux-x86_64": "@@amd64_debian_sysroot+//:sysroot_files",
        },
        target_toolchain_roots = {
            "linux-aarch64": "@@+llvm_toolchain_extension+llvm_aarch64//",
            "linux-x86_64": "@@+llvm_toolchain_extension+llvm_k8//",
        },
        toolchain_roots = {
            "linux-aarch64": "@@+llvm_toolchain_extension+llvm_aarch64//",
            "linux-x86_64": "@@+llvm_toolchain_extension+llvm_k8//",
        },
    )

llvm_toolchain_extension = module_extension(
    implementation = _llvm_toolchain_extension_impl,
)

def _dev_toolchains_extension_impl(_ctx):
    arm_frc_linux_gnueabi_repo_repo()

    gcc_arm_none_eabi_repo()

dev_toolchains_extension = module_extension(
    implementation = _dev_toolchains_extension_impl,
)
