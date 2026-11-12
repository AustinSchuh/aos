#!/usr/bin/python3

from __future__ import annotations
import sys
import unicodedata
import datetime
import os
import re
import subprocess
import tempfile
from absl import app
from absl import flags

from rootfs_utils import scoped_tmpdir_tegraflash_rootfs, Filesystem, check_required_deps, check_buildifier, scoped_mount, generate_build_file, write_registry_overlay

FLAGS = flags.FLAGS
flags.DEFINE_string(
    'yocto_downloads', None,
    'Yocto DL_DIR, if it is not build/downloads under the yocto checkout.')

REQUIRED_DEPS = ["xfsprogs", "zstd"]

CUDA = "cuda-13.2"


def installed_version(partition, package):
    """Returns the upstream version of a package installed in the image.

    Yocto appends its own revision (-r0) to the version it packaged.
    """
    with open(os.path.join(partition, "var/lib/dpkg/status")) as f:
        for stanza in f.read().split("\n\n"):
            fields = dict(
                line.split(": ", 1) for line in stanza.splitlines()
                if ": " in line and not line.startswith(" "))
            if fields.get("Package") == package:
                return re.sub(r"-r[0-9]+$", "", fields["Version"])
    raise ValueError(f"{package} is not installed in the image")


def do_package(yocto_root, partition):
    tarball = datetime.date.today().strftime(
        f"{os.getcwd()}/%Y-%m-%d-wrynose-arm64-nvidia-rootfs.tar")
    print(tarball, file=sys.stderr)

    subprocess.run([
        "sudo",
        "tar",
        "--sort=name",
        "--mtime=0",
        "--owner=0",
        "--group=0",
        "--numeric-owner",
        "--exclude=./usr/share/ca-certificates",
        "--exclude=./home",
        "--exclude=./root",
        "--exclude=./usr/src",
        "--exclude=./usr/lib/mesa-diverted",
        "--exclude=./usr/bin/X11",
        "--exclude=./usr/lib/systemd/system/system-systemd*.slice",
        "--exclude=./dev",
        f"--exclude=./usr/local/{CUDA}/bin/fatbinary",
        f"--exclude=./usr/local/{CUDA}/bin/ptxas",
        f"--exclude=./usr/local/{CUDA}/bin/nvlink",
        f"--exclude=./usr/local/{CUDA}/bin/bin2c",
        f"--exclude=./usr/local/{CUDA}/include/thrust",
        f"--exclude=./usr/local/{CUDA}/include/nv",
        f"--exclude=./usr/local/{CUDA}/include/cuda",
        f"--exclude=./usr/local/{CUDA}/include/cub",
        "--exclude=./usr/include/cub",
        "--exclude=./usr/include/nv",
        "--exclude=./usr/include/thrust",
        "--exclude=./usr/include/cuda",
        "--exclude=./usr/share",
        "-cf",
        tarball,
        ".",
    ],
                   cwd=partition,
                   check=True)

    # Pull the host (x86_64) CUDA tools from yocto.  The ones in the image are
    # aarch64, and the ones yocto installs into its native sysroot have paths
    # baked into them, so extract them from the .deb yocto downloaded, at the
    # version the image has.
    nvcc = "cuda-nvcc-" + CUDA.removeprefix("cuda-").replace(".", "-")
    cuda_deb = os.path.join(
        FLAGS.yocto_downloads or os.path.join(yocto_root, "build/downloads"),
        f"{nvcc}_{installed_version(partition, 'cuda-nvcc')}_amd64.deb")
    with tempfile.TemporaryDirectory() as cuda_dir:
        subprocess.run(["dpkg-deb", "-x", cuda_deb, cuda_dir], check=True)
        cuda_bin = os.path.join(cuda_dir, f"usr/local/{CUDA}/bin")

        subprocess.run([
            "sudo",
            "tar",
            "--sort=name",
            "--mtime=0",
            "--owner=0",
            "--group=0",
            "--numeric-owner",
            f'--transform=s|{cuda_bin[1:]}/fatbinary|usr/local/{CUDA}/bin/aarch64-unknown-linux-gnu-fatbinary|',
            f'--transform=s|{cuda_bin[1:]}/|usr/local/{CUDA}/bin/|',
            "--append",
            "-f",
            tarball,
            os.path.join(cuda_bin, "ptxas"),
            os.path.join(cuda_bin, "fatbinary"),
            os.path.join(cuda_bin, "nvlink"),
            os.path.join(cuda_bin, "bin2c"),
        ],
                       check=True)

    # Most of the sysroot is CUDA and TensorRT libraries which only compress
    # well with a long window.  128 MiB is the largest window zstd decodes
    # without being told to, which is all Bazel 8 will do in WORKSPACE mode.
    subprocess.run(
        ["zstd", "--rm", "-f", "-T0", "--ultra", "-21", "--long=27", tarball],
        check=True)

    subprocess.run(["sha256sum", tarball + ".zst"], check=True)


def main(argv):
    check_required_deps(REQUIRED_DEPS)

    if len(argv) != 2:
        print("Usage:", file=sys.stderr)
        print(" buildify_yocto_image /path/to/yocto", file=sys.stderr)
        return 1

    if argv[1][0] != '/':
        print("Path must be absolute", file=sys.stderr)
        return 1

    full_image = os.path.join(
        argv[1], "build/tmp/deploy/images/p3768-0000-p3767-0003",
        "demo-image-base-p3768-0000-p3767-0003.rootfs.tegraflash-tar.zst")

    if not check_buildifier():
        print(
            "ERROR: Need to have buildifier in the path.  Please resolve this.",
            file=sys.stderr)
        exit()

    if not os.path.exists(full_image):
        print("ERROR: Point to a valid image", file=sys.stderr)
        exit()

    with scoped_tmpdir_tegraflash_rootfs(full_image) as rootfs:
        with scoped_mount(rootfs) as partition:
            subprocess.run([
                "sudo", "cp", "/usr/bin/qemu-aarch64-static",
                f"{partition}/usr/bin/"
            ],
                           check=True)

            subprocess.run(["sudo", "chmod", "a+X", "-R", f"{partition}/"],
                           check=True)

            filesystem = Filesystem(
                partition,
                [
                    "sudo", "chroot", "--userspec=0:0", partition,
                    "qemu-aarch64-static", "/bin/bash"
                ],
            )

            packages_to_eval = [
                filesystem.packages['libglib-2.0-dev'],
                filesystem.packages['opencv-dev'],
                filesystem.packages['opencv'],
                filesystem.packages['tensorrt-core'],
                filesystem.packages['tensorrt-core-dev'],
                filesystem.packages['libz-dev'],
                filesystem.packages['libc6-dev'],
                filesystem.packages['xz-dev'],
                filesystem.packages['libstdc++-dev'],
                filesystem.packages['libffi-dev'],
                filesystem.packages['util-linux-dev'],
                filesystem.packages['libnpp-dev'],
                filesystem.packages['cuda-cudart-dev'],
                filesystem.packages['gstreamer1.0-dev'],
                filesystem.packages['orc-dev'],
                filesystem.packages['libgstrtp-1.0-0'],
                filesystem.packages['libpcre2-dev'],
                filesystem.packages['gstreamer1.0-plugins-bad-dev'],
                filesystem.packages['libjpeg-dev'],
            ]

            # TODO(austin): We will need to make new module versions each time we make a new sysroot.
            write_registry_overlay(
                "../registry/modules/arm64_debian_sysroot/2026.11.12",
                generate_build_file(filesystem, packages_to_eval,
                                    "orin_debian_rootfs.BUILD.template"))

            do_package(argv[1], partition)


if __name__ == '__main__':
    app.run(main)
