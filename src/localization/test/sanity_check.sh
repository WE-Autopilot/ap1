#!/usr/bin/env bash
# Sanity tests for ap1_localization's external dependencies:
#   GTSAM        ros-jazzy-gtsam (apt)                    - used by LIO-SAM
#   LIO-SAM      src/external/LIO-SAM (source, ap1.repos) - builds the maps NDT runs on
#   Autoware NDT ros-jazzy-autoware-ndt-scan-matcher (apt) - used by localization_node
#
# Run after ./ap1_setup.sh (or colcon build), from anywhere:
#   ./src/localization/test/sanity_check.sh
# Exits non-zero if any check fails. Works the same on amd64 (Windows/WSL) and
# arm64 (Apple Silicon Ubuntu VM).
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$HERE/../../.." && pwd)"
PY=/usr/bin/python3  # system Python: rclpy is built for it, not for the perception venv

GREEN='\033[0;32m'; RED='\033[0;31m'; BOLD='\033[1m'; NC='\033[0m'
PASSED=0; FAILED=0
pass() { echo -e "  ${GREEN}PASS${NC}  $1"; PASSED=$((PASSED+1)); }
fail() { echo -e "  ${RED}FAIL${NC}  $1"; FAILED=$((FAILED+1)); }
section() { echo -e "\n${BOLD}$1${NC}"; }

TMP="$(mktemp -d)"
PIDS=()
cleanup() {
    for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null; done
    wait 2>/dev/null
    rm -rf "$TMP"
}
trap cleanup EXIT

# Start a ROS node in the background, logging to $TMP/<name>.log
start_node() {
    local name="$1"; shift
    "$@" > "$TMP/$name.log" 2>&1 &
    PIDS+=($!)
}
stop_nodes() {
    for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null; done
    wait 2>/dev/null
    PIDS=()
}
# Print a node's log, indented, when a check fails
show_log() { sed 's/^/        | /' "$TMP/$1.log" | tail -n "${2:-15}"; }

# ---------------------------------------------------------------------------
set +u
source /opt/ros/jazzy/setup.bash
if [[ ! -f "$WS_ROOT/install/setup.bash" ]]; then
    echo "No $WS_ROOT/install/setup.bash - run ./ap1_setup.sh or colcon build first." >&2
    exit 1
fi
source "$WS_ROOT/install/setup.bash"
set -u
# Keep test traffic private: own DDS domain, this machine only. Nothing else
# running (e.g. the CARLA bridge) sees these topics or interferes with them.
export ROS_DOMAIN_ID=$((150 + RANDOM % 50))
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

echo -e "${BOLD}ap1_localization dependency sanity check${NC}  ($(uname -m), workspace $WS_ROOT)"

# ---------------------------------------------------------------------------
section "GTSAM (apt: ros-jazzy-gtsam)"
if dpkg -s ros-jazzy-gtsam &>/dev/null; then
    pass "ros-jazzy-gtsam $(dpkg-query -W -f='${Version}' ros-jazzy-gtsam | cut -d- -f1) installed"
else
    fail "ros-jazzy-gtsam not installed (run ./ap1_setup.sh, or: sudo apt install ros-jazzy-gtsam)"
fi
if (cd "$WS_ROOT" && colcon list --names-only 2>/dev/null | grep -qx gtsam) || [[ -d "$WS_ROOT/install/gtsam" ]]; then
    fail "workspace still builds its own GTSAM, which shadows the apt one (rerun ./ap1_setup.sh)"
else
    pass "no source-built GTSAM in the workspace"
fi

# ---------------------------------------------------------------------------
section "LIO-SAM (source: src/external/LIO-SAM)"
LIO_PREFIX="$(ros2 pkg prefix lio_sam 2>/dev/null || true)"
LIO_NODES=(lio_sam_imuPreintegration lio_sam_imageProjection lio_sam_featureExtraction lio_sam_mapOptimization)
if [[ -z "$LIO_PREFIX" ]]; then
    fail "lio_sam package not found (did colcon build succeed?)"
else
    pass "lio_sam package found at $LIO_PREFIX"
    missing_libs=""; wrong_gtsam=""
    for exe in "${LIO_NODES[@]}"; do
        bin="$LIO_PREFIX/lib/lio_sam/$exe"
        if [[ ! -x "$bin" ]]; then missing_libs+=" $exe(missing)"; continue; fi
        ldd "$bin" | grep -q "not found" && missing_libs+=" $exe"
        gtsam_lib="$(ldd "$bin" | awk '/libgtsam\.so/{print $3; exit}')"
        [[ -n "$gtsam_lib" && "$gtsam_lib" != /opt/ros/jazzy/* ]] && wrong_gtsam+=" $exe->$gtsam_lib"
    done
    if [[ -z "$missing_libs" ]]; then
        pass "all 4 LIO-SAM executables present with every shared library resolved"
    else
        fail "LIO-SAM executables missing or with unresolved libraries:$missing_libs"
    fi
    if [[ -z "$wrong_gtsam" ]]; then
        pass "LIO-SAM links the apt GTSAM (/opt/ros/jazzy)"
    else
        fail "LIO-SAM links a non-apt GTSAM:$wrong_gtsam"
    fi

    # Override LIO-SAM's Ouster defaults to match the synthetic VLP-16 data.
    cat > "$TMP/lio_sam_test.yaml" <<'EOF'
/**:
  ros__parameters:
    sensor: velodyne
    N_SCAN: 16
    Horizon_SCAN: 1800
    extrinsicTrans: [0.0, 0.0, 0.0]
    extrinsicRot: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    extrinsicRPY: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
EOF
    for exe in "${LIO_NODES[@]}"; do
        start_node "$exe" "$LIO_PREFIX/lib/lio_sam/$exe" --ros-args \
            --params-file "$LIO_PREFIX/share/lio_sam/config/params.yaml" \
            --params-file "$TMP/lio_sam_test.yaml"
    done
    sleep 3
    if out="$($PY "$HERE/lio_sam_pipeline_test.py" 2>&1)"; then
        pass "LIO-SAM + GTSAM pipeline: $(echo "$out" | tail -n 2 | tr '\n' ' ')"
    else
        fail "LIO-SAM + GTSAM pipeline did not produce odometry"
        echo "$out" | sed 's/^/        | /'
        for exe in "${LIO_NODES[@]}"; do echo "        $exe:"; show_log "$exe" 8; done
    fi
    stop_nodes
fi

# ---------------------------------------------------------------------------
section "Autoware NDT (apt: ros-jazzy-autoware-ndt-scan-matcher)"
if dpkg -s ros-jazzy-autoware-ndt-scan-matcher &>/dev/null; then
    pass "ros-jazzy-autoware-ndt-scan-matcher $(dpkg-query -W -f='${Version}' ros-jazzy-autoware-ndt-scan-matcher | cut -d- -f1) installed"
else
    fail "ros-jazzy-autoware-ndt-scan-matcher not installed (run ./ap1_setup.sh)"
fi
LOC_PREFIX="$(ros2 pkg prefix ap1_localization 2>/dev/null || true)"
LOC_BIN="$LOC_PREFIX/lib/ap1_localization/localization_node"
if [[ -z "$LOC_PREFIX" || ! -x "$LOC_BIN" ]]; then
    fail "ap1_localization localization_node not built"
else
    ndt_lib="$(ldd "$LOC_BIN" | awk '/libmultigrid_ndt_omp\.so/{print $3; exit}')"
    if ldd "$LOC_BIN" | grep -q "not found"; then
        fail "localization_node has unresolved shared libraries"
    elif [[ "$ndt_lib" == /opt/ros/jazzy/* ]]; then
        pass "localization_node links Autoware's multigrid NDT ($ndt_lib)"
    else
        fail "localization_node does not link Autoware's libmultigrid_ndt_omp"
    fi

    initial_pose="$($PY "$HERE/ndt_synthetic_test.py" write-map "$TMP/map.pcd")"
    start_node localization_node "$LOC_BIN" --ros-args \
        --params-file "$WS_ROOT/src/localization/config/ndt_localization.yaml" \
        -p map_path:="$TMP/map.pcd" -p "initial_pose:=$initial_pose"
    sleep 3
    if out="$($PY "$HERE/ndt_synthetic_test.py" run 2>&1)"; then
        pass "NDT localization accuracy: $out"
    else
        fail "NDT localization accuracy"
        echo "$out" | sed 's/^/        | /'
        show_log localization_node
    fi
    stop_nodes
fi

# ---------------------------------------------------------------------------
echo
if [[ $FAILED -eq 0 ]]; then
    echo -e "${GREEN}${BOLD}All $PASSED checks passed.${NC}"
else
    echo -e "${RED}${BOLD}$FAILED of $((PASSED+FAILED)) checks failed.${NC}"
fi
exit $(( FAILED > 0 ))
