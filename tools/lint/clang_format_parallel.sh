#!/bin/bash

# Wrapper around clang-format which fans the files out across all of the cores.
#
# rules_lint invokes a formatter as `$tool <flags...> <file>...`, and its driver
# pipes the file list through a plain `xargs` with no `-P`.  That formats the
# whole repository in a single clang-format process, which is the slowest part
# of `//tools/lint:format.check` by an order of magnitude.  Splitting the files
# across `nproc` processes takes the C++ check from ~6.4s to ~1.0s.

# --- begin runfiles.bash initialization v2 ---
# Copy-pasted from the Bazel Bash runfiles library v2.
set -uo pipefail; f=bazel_tools/tools/bash/runfiles/runfiles.bash
source "${RUNFILES_DIR:-/dev/null}/$f" 2>/dev/null || \
  source "$(grep -sm1 "^$f " "${RUNFILES_MANIFEST_FILE:-/dev/null}" | cut -f2- -d' ')" 2>/dev/null || \
  source "$0.runfiles/$f" 2>/dev/null || \
  source "$(grep -sm1 "^$f " "$0.runfiles_manifest" | cut -f2- -d' ')" 2>/dev/null || \
  source "$(grep -sm1 "^$f " "$0.exe.runfiles_manifest" | cut -f2- -d' ')" 2>/dev/null || \
  { echo>&2 "ERROR: cannot find $f"; exit 1; }; f=; set -e
# --- end runfiles.bash initialization v2 ---

readonly CLANG_FORMAT="$(rlocation llvm_toolchain/clang-format)"

# How many files to hand to each clang-format process.  Small batches keep every
# core busy through the end of the run, while larger ones leave cores idle while
# the last few batches drain.
readonly BATCH_SIZE=4

# rules_lint passes the flags first and the files afterwards.  Every
# clang-format flag we pass starts with a '-', so split the arguments on that.
flags=()
files=()
for arg in "$@"; do
    if [[ "${arg}" == -* ]]; then
        flags+=("${arg}")
    else
        files+=("${arg}")
    fi
done

# There is nothing to format.  Return without running clang-format, since with
# no files it would block reading from stdin.
if ((${#files[@]} == 0)); then
    exit 0
fi

# `getconf` is used instead of `nproc` so that this works on macOS too.
readonly JOBS="$(getconf _NPROCESSORS_ONLN)"

status=0
printf '%s\0' "${files[@]}" \
    | xargs -0 -P "${JOBS}" -n "${BATCH_SIZE}" "${CLANG_FORMAT}" "${flags[@]}" \
    || status=$?

# xargs exits with 123 when any of the commands it ran exited between 1 and 125,
# which is how clang-format reports formatting violations under `-Werror`.  Turn
# that back into a plain failure so callers see an ordinary exit code.
if ((status == 123)); then
    status=1
fi

exit "${status}"
