#!/usr/bin/env bash
set -euo pipefail

# Software-only AP1 smoke test.
#
# This validates the stack up to the hardware boundary by using synthetic
# perception, synthetic odometry, and synthetic actuation feedback. It expects
# the workspace to include the related smoke-test PRs for mapping/control/AP1.

WORKSPACE="${AP1_WORKSPACE:-$HOME/weap/ap1}"
LOG_DIR="${AP1_SMOKE_LOG_DIR:-/tmp/ap1-smoke}"
STARTUP_SEC="${AP1_SMOKE_STARTUP_SEC:-8}"
TOPIC_TIMEOUT_SEC="${AP1_SMOKE_TOPIC_TIMEOUT_SEC:-8}"

mkdir -p "$LOG_DIR"

cleanup() {
    if [[ -f "$LOG_DIR/full_system.pid" ]]; then
        local pid
        pid="$(cat "$LOG_DIR/full_system.pid")"
        kill "$pid" 2>/dev/null || true
    fi
}
trap cleanup EXIT

cd "$WORKSPACE"

set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

ros2 launch ap1_bringup full_system.launch.py \
    use_synthetic_perception:=true \
    use_synthetic_odometry:=true \
    use_synthetic_actuation_feedback:=true \
    > "$LOG_DIR/full_system.log" 2>&1 &
echo "$!" > "$LOG_DIR/full_system.pid"

sleep "$STARTUP_SEC"

check_topic_once() {
    local topic="$1"
    local output_name="$2"
    local output_path="$LOG_DIR/$output_name"

    if timeout "${TOPIC_TIMEOUT_SEC}s" ros2 topic echo --once "$topic" \
        > "$output_path" 2>&1; then
        echo "PASS $topic"
    else
        echo "FAIL $topic"
        cat "$output_path"
        return 1
    fi
}

check_topic_once /ap1/mapping/odometer odometer.txt
check_topic_once /ap1/mapping/entities entities.txt
check_topic_once /ap1/mapping/lanes lanes.txt
check_topic_once /ap1/planning/target_path target_path.txt
check_topic_once /ap1/control/motor_power motor_power.txt
check_topic_once /ap1/control/turn_angle turn_angle.txt

ros2 node list > "$LOG_DIR/nodes.txt"
ros2 topic list | sort > "$LOG_DIR/topics.txt"

echo
echo "Smoke test passed. Logs: $LOG_DIR"
