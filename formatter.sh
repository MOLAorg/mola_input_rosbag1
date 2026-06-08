#!/usr/bin/env bash
# Usage: formatter.sh [--check]
#   (default) Reformat all C/C++ sources in-place with clang-format-14.
#   --check   Dry-run: exit non-zero if any file would be reformatted.
#
# Note: only this package's own sources (src/, include/) are checked. The
# vendored ROS 1 sources (ros1/, mrpt_ros_bridge/) are intentionally excluded.

set -euo pipefail

if [ "${1:-}" = "--check" ]; then
  MODE=(--dry-run --Werror)
else
  MODE=(-i)
fi

find \
    src \
    apps \
    include \
    \( -iname "*.h" -o -iname "*.hpp" -o -iname "*.cpp" -o -iname "*.c" \) \
  -print0 | xargs -0 -r -t clang-format-14 "${MODE[@]}"
