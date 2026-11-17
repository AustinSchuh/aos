#!/usr/bin/python3
"""Builds the amd64 sysroot: a Debian bookworm root filesystem, as a tarball.

  ./build_rootfs.py

writes YYYY-MM-DD-bookworm-amd64-nvidia-rootfs.tar.zst in the current
directory, and prints its sha256.  No root needed: mmdebstrap builds it in a
user namespace (sudo apt install mmdebstrap uidmap zstd, once).  Then
compilers/buildify_debian_x86_image.py makes its registry module version.
"""

import datetime
import os
import re
import shutil
import subprocess
import sys
import tempfile
import urllib.request

# NVIDIA's repository, for TensorRT matching the Orin's JetPack 7.2: 10.16
# (the repository's 10.16.1 to the Orin's 10.16.2; it also has 11, which the
# Orin doesn't).
NVIDIA_REPOSITORY = "https://developer.download.nvidia.com/compute/cuda/repos/debian12/x86_64"
TENSORRT_VERSION = "10.16.1.11-1+cuda13.2"

COMPONENTS = "main contrib non-free non-free-firmware"

PACKAGES = [
    "libopencv-calib3d406",
    "libopencv-contrib406",
    "libopencv-core406",
    "libopencv-features2d406",
    "libopencv-flann406",
    "libopencv-highgui406",
    "libopencv-imgcodecs406",
    "libopencv-imgproc406",
    "libopencv-ml406",
    "libopencv-objdetect406",
    "libopencv-photo406",
    "libopencv-shape406",
    "libopencv-stitching406",
    "libopencv-superres406",
    "libopencv-video406",
    "libopencv-videoio406",
    "libopencv-videostab406",
    "libopencv-viz406",
    "libopencv-dev",
    "gstreamer1.0-dev",
    "libgl1-mesa-dev",
    "libv4l-dev",
    "libc6-dev",
    "libusb-dev",
    "libusb-1.0-0-dev",
    "libstdc++-12-dev",
    # The linker, whose search path buildify_debian_x86_image.py reads.
    "binutils",
    # TensorRT's runtime and headers; not libnvinfer-dev, 3 GB of static
    # libraries.  CUDA itself comes from NVIDIA's redistributable archives
    # (MODULE.bazel), not the sysroot.
    f"libnvinfer10={TENSORRT_VERSION}",
    f"libnvinfer-headers-dev={TENSORRT_VERSION}",
    "libgstreamer-plugins-base1.0-dev",
    "libgstreamer-plugins-bad1.0-dev",
    # camera_streamer's pipeline: x264enc, rtph264pay and webrtcbin with its
    # nice, dtls and srtp, so it runs on x86 (in simulations, for the pages'
    # end-to-end tests) as on the Orin.
    "gstreamer1.0-plugins-good",
    "gstreamer1.0-plugins-bad",
    "gstreamer1.0-plugins-ugly",
    "gstreamer1.0-nice",
    # libdw, which libgstreamer links, needs it; Rust links it by its
    # unversioned name.
    "libbz2-dev",
    "libturbojpeg0-dev",
    "postgresql",
    "apache2",
    "ssh",
    "rsync",
    "xvfb",
    "libglib2.0-0",
    "gir1.2-gtk-3.0",
    "libgtk-3-dev",
    "librsvg2-common",
    "gvfs-libs",
    "gir1.2-pango-1.0",
]

# Left out of the tarball, relative to its root; shell globs.
REMOVED = [
    "usr/share/ca-certificates",
    "usr/src",
    # A recursive symlink.  Thanks NVIDIA.  Bazel can't take it.
    "usr/lib/mesa-diverted/libGL.so-master",
    "usr/bin/X11",
    # Nonprinting characters, which Bazel can't take.  Not useful anyway.
    "usr/lib/systemd/system/system-systemd*cryptsetup.slice",
    "usr/include/cub",
    "usr/include/nv",
    "usr/include/thrust",
    "usr/include/cuda",
    # TensorRT's builder, 2.5 GB, and its links to it: yolo only
    # deserializes engines.
    "usr/lib/x86_64-linux-gnu/libnvinfer_builder_resource*",
    "usr/lib/x86_64-linux-gnu/do_not_link_against_nvinfer_builder_resource*",
    # Perl's man pages are named like Dpkg::Arch.3perl.gz, which Bazel can't
    # take, and no man page is any use in a sysroot.
    "usr/share/man",
]


def nvidia_keyring(directory):
    """NVIDIA's repository key, from its cuda-keyring package."""
    deb = os.path.join(directory, "cuda-keyring.deb")
    urllib.request.urlretrieve(
        f"{NVIDIA_REPOSITORY}/cuda-keyring_1.1-1_all.deb", deb)
    subprocess.run(["dpkg-deb", "-x", deb, directory], check=True)
    keyring = os.path.join(directory, "cuda-archive-keyring.gpg")
    shutil.copy(
        os.path.join(directory, "usr/share/keyrings/cuda-archive-keyring.gpg"),
        keyring)
    os.chmod(keyring, 0o644)
    return keyring


def apt_policy(directory):
    """apt's signature policy, as Debian 13's, but taking SHA-1 bindings.

    NVIDIA's key binds its signing key with SHA-1, which Debian 13's apt
    refuses since 2026-02-01.
    """
    policy = os.path.join(directory, "sequoia.config")
    with open("/usr/share/apt/default-sequoia.config") as f:
        contents = f.read()
    with open(policy, "w") as f:
        f.write(
            re.sub(r"^sha1\.second_preimage_resistance = .*$",
                   "sha1.second_preimage_resistance = 2030-02-01",
                   contents,
                   flags=re.MULTILINE))
    os.chmod(policy, 0o644)
    return policy


def main():
    missing = [
        tool for tool in ["mmdebstrap", "newuidmap", "zstd"]
        if shutil.which(tool) is None
    ]
    if missing:
        print("Missing",
              " ".join(missing) + "; install them with:",
              file=sys.stderr)
        print("  sudo apt install mmdebstrap uidmap zstd", file=sys.stderr)
        return 1

    tarball = datetime.date.today().strftime(
        f"{os.getcwd()}/%Y-%m-%d-bookworm-amd64-nvidia-rootfs.tar")

    # apt reads the key and the policy as its own unprivileged user, so in a
    # directory anyone can read.
    with tempfile.TemporaryDirectory(dir="/tmp") as tmp:
        os.chmod(tmp, 0o755)
        keyring = nvidia_keyring(tmp)
        env = dict(os.environ, APT_SEQUOIA_CRYPTO_POLICY=apt_policy(tmp))
        subprocess.run(
            [
                "mmdebstrap",
                # Root in a user namespace, not on the host.
                "--mode=unshare",
                # Required and important, as debootstrap installs.
                "--variant=important",
                "--format=tar",
                "--include=" + ",".join(PACKAGES),
                f'--customize-hook=cd "$1" && rm -rf {" ".join(REMOVED)}',
                "bookworm",
                tarball,
                f"deb http://deb.debian.org/debian bookworm {COMPONENTS}",
                f"deb http://deb.debian.org/debian bookworm-updates {COMPONENTS}",
                f"deb http://deb.debian.org/debian-security bookworm-security {COMPONENTS}",
                f"deb [signed-by={keyring}] {NVIDIA_REPOSITORY} /",
            ],
            env=env,
            check=True)

    subprocess.run(["zstd", "-T0", "-19", "--rm", "-f", tarball], check=True)
    subprocess.run(["sha256sum", tarball + ".zst"], check=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
