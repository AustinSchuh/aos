#!/usr/bin/env python3

from __future__ import annotations
import sys
import unicodedata
import datetime
import base64
import hashlib
import json
import os
import pathlib
import re
from absl import app
from absl import flags

from rootfs_utils import scoped_tmpdir_tarball, Filesystem, check_buildifier, generate_build_file, write_registry_overlay

REGISTRY = pathlib.Path("../registry/modules/amd64_debian_sysroot")
MIRROR = "https://realtimeroboticsgroup.org/build-dependencies"


def start_registry_version(version, image):
    """Makes the registry's module version for image, if it isn't there.

    Its MODULE.bazel is the newest version's with this version, and its
    source.json points at image as it will be on the mirror, with its hash;
    write_registry_overlay fills in the overlay's.
    """
    module_dir = REGISTRY / version
    if (module_dir / "source.json").exists():
        return
    metadata_path = REGISTRY / "metadata.json"
    metadata = json.loads(metadata_path.read_text())
    newest = REGISTRY / sorted(metadata["versions"])[-1]
    (module_dir / "overlay").mkdir(parents=True, exist_ok=True)
    (module_dir / "MODULE.bazel").write_text(
        re.sub(r'version = "[^"]*"',
               f'version = "{version}"', (newest / "MODULE.bazel").read_text(),
               count=1))
    digest = hashlib.sha256()
    with open(image, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            digest.update(block)
    (module_dir / "source.json").write_text(
        json.dumps(
            {
                "url": f"{MIRROR}/{os.path.basename(image)}",
                "integrity": "sha256-" +
                base64.b64encode(digest.digest()).decode(),
                "overlay": {
                    "BUILD.bazel": ""
                },
            },
            indent=4) + "\n")
    if version not in metadata["versions"]:
        metadata["versions"].append(version)
        metadata_path.write_text(json.dumps(metadata, indent=4) + "\n")


def main(argv):
    if len(argv) != 2:
        print("Usage:", file=sys.stderr)
        print(
            " |buildify_debian_x86_image /path/to/YYYY-MM-DD-bookworm-amd64-nvidia-rootfs.tar.zst",
            file=sys.stderr)
        return 1

    if not check_buildifier():
        print(
            "ERROR: Need to have buildifier in the path.  Please resolve this.",
            file=sys.stderr)
        exit()

    full_image = argv[1]

    if not os.path.exists(full_image):
        print("ERROR: Point to a valid image", file=sys.stderr)
        exit()

    date = re.match(r"(\d{4})-(\d{2})-(\d{2})-", os.path.basename(full_image))
    if date is None:
        print("ERROR: Name the image YYYY-MM-DD-..., as build_rootfs.py does",
              file=sys.stderr)
        exit()
    version = ".".join(date.groups())

    with scoped_tmpdir_tarball(full_image) as rootfs:
        filesystem = Filesystem(
            rootfs,
            # Root in a user namespace, not on the host: no sudo.
            ["unshare", "-r", "chroot", rootfs, "/bin/bash"],
        )

        # TODO(austin):
        #  I need tests for all this...
        #  This is getting too complicated to trust that it keeps working otherwise.
        # I can't wrap a cc_library around a linker script.
        #  I need to either rewrite the linker script to use sandboxed paths, or I need to inline it into the BUILD file.  Either way, I need to explode if someone tries to do another one.
        # libusb-0.1-4

        packages_to_eval = [
            filesystem.packages['libopencv-dev'],
            filesystem.packages['libc6-dev'],
            filesystem.packages['libusb-dev'],
            filesystem.packages['libusb-1.0-0-dev'],
            filesystem.packages['libstdc++-dev'],
            filesystem.packages['libturbojpeg0-dev'],
            filesystem.packages['libnvinfer10'],
            filesystem.packages['libnvinfer-headers-dev'],
            filesystem.packages['ssh'],
            filesystem.packages['rsync'],
            filesystem.packages['libgstreamer-plugins-bad1.0-dev'],
            filesystem.packages['libgstreamer-plugins-base1.0-dev'],
            filesystem.packages['libgstreamer1.0-dev'],
        ]

        # A new module version for each sysroot, named for the date
        # build_rootfs.py made it: 2026-11-16-bookworm-...tar.zst is 2026.11.16.
        start_registry_version(version, full_image)
        write_registry_overlay(
            f"../registry/modules/amd64_debian_sysroot/{version}",
            generate_build_file(filesystem, packages_to_eval,
                                "amd64_debian_rootfs.BUILD.template"))


if __name__ == '__main__':
    app.run(main)
