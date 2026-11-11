#!/usr/bin/env bash
# One command for a CARLA mapping run: record a drive, filter it to the kart's
# sensors, run it through LIO-SAM, save the map and trajectories, print a summary.
#
#   scripts/map_carla.sh --map Town02 --short          # record a new loop and map it
#   scripts/map_carla.sh --bag ~/WEAP/bags/<x>_filtered # map a bag you already have
#
# See src/mapping/README.md. Run scripts/map_carla.sh --help for every option.
set -euo pipefail

AP1="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PARAMS="$AP1/src/mapping/config/lio_sam_carla.yaml"

MAP=""; SHORT=""; SPAWN=""; BAG=""; RATE="0.5"; HOST="${CARLA_HOST:-localhost}"
OUT_ROOT="$HOME/WEAP/maps"; PAPER_IMU=0; DRY_RUN=0; MIN_DISK_GB=20

usage() {
    cat <<EOF
Usage:
  scripts/map_carla.sh --map <Town> [--short] [--spawn-point N] [options]
  scripts/map_carla.sh --bag <filtered_bag> [options]

Record + map (CARLA must be running):
  --map <Town>        CARLA map to drive, e.g. Town02
  --short             smallest closed loop through the spawn point (much faster)
  --spawn-point N     where the loop starts and ends (try another if the route fails)
  --host <host>       CARLA host (default: \$CARLA_HOST or localhost)

Map an existing bag (no CARLA needed):
  --bag <path>        a *_filtered bag from simulation/filter_bag.py

Options:
  --rate R            LIO-SAM playback rate (default: 0.5)
  --paper-imu         use the LIO-SAM paper IMU noise values instead of the CV7 ones
  --params <yaml>     LIO-SAM params file (default: src/mapping/config/lio_sam_carla.yaml)
  --out <dir>         parent folder for runs (default: ~/WEAP/maps)
  --dry-run           print what would run and stop
  -h, --help          this help
EOF
}

die() { echo "error: $*" >&2; exit 1; }

while [[ $# -gt 0 ]]; do
    case "$1" in
        --map) MAP="${2:?--map needs a value}"; shift 2 ;;
        --short) SHORT="--short"; shift ;;
        --spawn-point) SPAWN="${2:?--spawn-point needs a value}"; shift 2 ;;
        --host) HOST="${2:?--host needs a value}"; shift 2 ;;
        --bag) BAG="${2:?--bag needs a value}"; shift 2 ;;
        --rate) RATE="${2:?--rate needs a value}"; shift 2 ;;
        --paper-imu) PAPER_IMU=1; shift ;;
        --params) PARAMS="${2:?--params needs a value}"; shift 2 ;;
        --out) OUT_ROOT="${2:?--out needs a value}"; shift 2 ;;
        --dry-run) DRY_RUN=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) usage >&2; die "unknown option: $1" ;;
    esac
done

[[ -n "$MAP" && -n "$BAG" ]] && die "give --map (record a new drive) or --bag (use an existing one), not both"
[[ -z "$MAP" && -z "$BAG" ]] && { usage >&2; die "give --map or --bag"; }
[[ -n "$BAG" && ( -n "$SHORT" || -n "$SPAWN" ) ]] && die "--short and --spawn-point only apply with --map"
[[ -f "$PARAMS" ]] || die "params file not found: $PARAMS"

# One folder per run: <date>_<time>_<map or bag name>
if [[ -n "$MAP" ]]; then
    NAME="$(echo "$MAP" | tr '[:upper:]' '[:lower:]')${SHORT:+_short}"
else
    BAG="$(cd "$(dirname "$BAG")" && pwd)/$(basename "$BAG")"
    [[ -d "$BAG" ]] || die "bag folder not found: $BAG"
    NAME="$(basename "$BAG")"
fi
RUN_DIR="$OUT_ROOT/$(date +%Y-%m-%d_%H%M)_$NAME"
RESULTS="$RUN_DIR/results"
[[ -e "$RUN_DIR" ]] && die "$RUN_DIR already exists; wait a minute or use --out"

if [[ -n "$MAP" ]]; then
    RAW="$RUN_DIR/bag_raw"
    BAG="$RUN_DIR/bag_filtered"   # record_bag.py --filter writes <raw minus _raw>_filtered
    RECORD=(python3 "$AP1/simulation/record_bag.py" --map "$MAP" --host "$HOST" --out "$RAW" --filter)
    [[ -n "$SHORT" ]] && RECORD+=(--short)
    [[ -n "$SPAWN" ]] && RECORD+=(--spawn-point "$SPAWN")
fi

LIO=(python3 "$AP1/scripts/run_lio_sam.py" --bag "$BAG" --params "$RUN_DIR/lio_sam_params.yaml"
     --launch-pkg ap1_mapping --launch-file lio_sam_carla.launch.py
     --rate "$RATE" --output "$RESULTS" --ground-truth)

if [[ $DRY_RUN -eq 1 ]]; then
    echo "Run folder: $RUN_DIR"
    [[ -n "$MAP" ]] && echo "1. ${RECORD[*]}"
    echo "2. ${LIO[*]}"
    echo "   params: $PARAMS$([[ $PAPER_IMU -eq 1 ]] && echo ' (paper IMU noise)')"
    exit 0
fi

# ---------- checks, before anything slow ----------
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash || die "ROS 2 Jazzy not found at /opt/ros/jazzy"
# shellcheck disable=SC1091
source "$AP1/install/setup.bash" 2>/dev/null || die "workspace not built: run colcon build in $AP1"
set -u
ros2 pkg prefix lio_sam >/dev/null 2>&1 || die "lio_sam isn't built (see SETUP.md, LIO-SAM)"
ros2 pkg prefix ap1_mapping >/dev/null 2>&1 || die "ap1_mapping isn't built: colcon build --packages-select ap1_mapping"
if [[ -n "$MAP" ]]; then
    python3 -c "import carla" 2>/dev/null \
        || die "CARLA client missing: python3 -m pip install -r simulation/requirements.txt --break-system-packages"
fi

mkdir -p "$RUN_DIR"
FREE_GB=$(df -BG --output=avail "$RUN_DIR" | tail -1 | tr -dc '0-9')
if [[ -n "$MAP" && "$FREE_GB" -lt "$MIN_DISK_GB" ]]; then
    echo "warning: only ${FREE_GB} GB free in $OUT_ROOT; long drives make big bags" >&2
fi

# Leftover LIO-SAM or bag playback from an earlier run would publish into this one.
if pgrep -f 'lio_sam_(imuPreintegration|imageProjection|featureExtraction|mapOptimization)|ros2 bag play' >/dev/null; then
    echo "Stopping leftover LIO-SAM / bag playback from an earlier run..."
    pkill -INT -f 'lio_sam_(imuPreintegration|imageProjection|featureExtraction|mapOptimization)|ros2 bag play' || true
    sleep 3
    pkill -KILL -f 'lio_sam_(imuPreintegration|imageProjection|featureExtraction|mapOptimization)|ros2 bag play' || true
fi

# Params copy kept with the results, so every map records what made it.
cp "$PARAMS" "$RUN_DIR/lio_sam_params.yaml"
if [[ $PAPER_IMU -eq 1 ]]; then
    sed -i -E \
        -e 's/^([[:space:]]*imuAccNoise:).*/\1 3.99e-3   # paper value (--paper-imu)/' \
        -e 's/^([[:space:]]*imuGyrNoise:).*/\1 1.56e-3   # paper value (--paper-imu)/' \
        -e 's/^([[:space:]]*imuAccBiasN:).*/\1 6.44e-5   # paper value (--paper-imu)/' \
        -e 's/^([[:space:]]*imuGyrBiasN:).*/\1 3.56e-5   # paper value (--paper-imu)/' \
        "$RUN_DIR/lio_sam_params.yaml"
fi

LOG="$RUN_DIR/run.log"
echo "Run folder: $RUN_DIR (log: $LOG)"
exec > >(tee -a "$LOG") 2>&1

# ---------- 1. record + filter ----------
if [[ -n "$MAP" ]]; then
    echo; echo "== 1/2 Recording $MAP in CARLA ($HOST), then filtering =="
    "${RECORD[@]}"
    [[ -d "$BAG" ]] || die "filtered bag not found at $BAG"
fi

# ---------- 2. LIO-SAM ----------
echo; echo "== 2/2 LIO-SAM at ${RATE}x on $(basename "$BAG") =="
LIO_OK=0
"${LIO[@]}" && LIO_OK=1

# run_lio_sam.py doesn't wait for the launch to exit; let LIO-SAM finish its shutdown save.
for _ in $(seq 30); do
    pgrep -f 'lio_sam_mapOptimization' >/dev/null || break
    sleep 1
done

if [[ $LIO_OK -ne 1 ]]; then
    echo
    echo "LIO-SAM run FAILED. Look in $RESULTS/lio_sam.log:"
    grep -n -i -E 'died|error|large velocity|segment|exit' "$RESULTS/lio_sam.log" 2>/dev/null | tail -15 || true
    echo "If imuPreintegration died or the path flew off, retry with --paper-imu --bag $BAG"
    exit 1
fi

# ---------- summary ----------
echo; echo "== Summary =="
python3 - "$RESULTS" <<'EOF'
import math, pathlib, sys
res = pathlib.Path(sys.argv[1])

def load(name):
    p = res / name
    if not p.exists():
        return []
    return [list(map(float, l.split()[1:4])) for l in p.read_text().splitlines() if l.strip()]

def path_len(t):
    return sum(math.dist(a, b) for a, b in zip(t, t[1:]))

opt, gt = load('lio_sam_optimized_trajectory.tum'), load('ground_truth_trajectory.tum')
if opt:
    print(f'LIO-SAM path: {len(opt)} keyframes, {path_len(opt):.1f} m')
    print(f'End-to-end error (start vs end of the loop): {math.dist(opt[0], opt[-1]):.3f} m')
if gt:
    print(f'Ground truth path: {path_len(gt):.1f} m, '
          f'GT start vs end {math.dist(gt[0], gt[-1]):.3f} m (how closed the drive itself was)')
for pcd in sorted((res / 'map').glob('*.pcd')):
    print(f'{pcd.name:24s} {pcd.stat().st_size / 1e6:8.1f} MB')
EOF

if command -v evo_ape >/dev/null; then
    evo_ape tum "$RESULTS/ground_truth_trajectory.tum" "$RESULTS/lio_sam_optimized_trajectory.tum" \
        --align --t_max_diff 0.05 | tee "$RESULTS/ape.txt" | grep -E 'rmse|max|mean' || true
else
    echo "(pip install evo for RMSE against ground truth)"
fi

echo
echo "Map:     $RESULTS/map/GlobalMap.pcd"
echo "Results: $RESULTS"
