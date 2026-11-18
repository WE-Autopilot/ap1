#!/usr/bin/env bash
set -euo pipefail

PACKAGES=("ap1_control" "ap1_planning")

# Source ROS 2 Jazzy if not already sourced
if [ -f /opt/ros/jazzy/setup.bash ]; then
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
fi

echo "=== Building packages (and dependencies like ap1_msgs) ==="
colcon build --packages-up-to "${PACKAGES[@]}"

set +u
source install/setup.bash
set -u

echo "=== Running unit tests ==="
colcon test --packages-select "${PACKAGES[@]}"

echo "=== Test results ==="
if colcon test-result --verbose; then
  echo "RESULT: PASSED"
else
  echo "RESULT: FAILED"
  exit 1
fi
