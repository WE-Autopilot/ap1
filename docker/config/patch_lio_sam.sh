#!/usr/bin/env bash
# Patch LIO-SAM (ros2 branch) for ROS 2 Jazzy. Same edits as ap1_setup.sh, but
# with perl instead of GNU sed so it also runs on macOS. Safe to rerun.
#
#   docker/config/patch_lio_sam.sh src/external/LIO-SAM
set -euo pipefail

LIO="${1:?usage: patch_lio_sam.sh <path to LIO-SAM>}"
CMAKE="$LIO/CMakeLists.txt"
XML="$LIO/package.xml"
[[ -f "$CMAKE" && -f "$XML" ]] || { echo "patch_lio_sam.sh: no LIO-SAM at $LIO" >&2; exit 1; }

# Ubuntu 24.04 ships Eigen's CMake package as Eigen3, not Eigen.
# \b keeps the last edit idempotent: a rerun won't turn Eigen3 into Eigen33.
perl -pi -e '
    s/find_package\(Eigen REQUIRED\)/find_package(Eigen3 REQUIRED)/g;
    s/Eigen_INCLUDE_DIRS/Eigen3_INCLUDE_DIRS/g;
    s/EIGEN3_INCLUDE_DIR/Eigen3_INCLUDE_DIRS/g;
    s/\bEigen\b/Eigen3/g if /ament_target_dependencies/;
' "$CMAKE"

# CMakeLists needs cv_bridge but package.xml never declares it, so rosdep
# wouldn't install it. Add it after the gtsam build_depend.
if ! grep -q "<depend>cv_bridge</depend>" "$XML"; then
    perl -pi -e 's|^( *)<build_depend>gtsam</build_depend>\n|$&$1<depend>cv_bridge</depend>\n|' "$XML"
fi
echo "LIO-SAM patched for Jazzy: $LIO"
