#!/usr/bin/env bash
# =============================================================================
# AP1 Autopilot — Automated Environment Setup Script
# =============================================================================

set -euo pipefail

# ── Colours ──────────────────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

ok()   { echo -e "${GREEN}  ✔  $1${NC}"; }
warn() { echo -e "${YELLOW}  ⚠  $1${NC}"; }
err()  { echo -e "${RED}  ✘  $1${NC}"; }
info() { echo -e "${CYAN}  ➜  $1${NC}"; }
hdr()  { echo -e "\n${BOLD}${BLUE}══ $1 ══${NC}"; }

# ── Step counter ─────────────────────────────────────────────────────────────
STEP=0
ERRORS=0
step() { STEP=$((STEP+1)); echo -e "\n${BOLD}[Step $STEP] $1${NC}"; }

# =============================================================================
# 1. DETECT WORKSPACE ROOT
# =============================================================================
hdr "AP1 Environment Setup"
echo -e "  Detecting workspace...\n"

# Try to find the workspace root by looking for the src/ directory
# Search from the current directory upward, then common locations
find_workspace() {
    local dir="$1"
    while [[ "$dir" != "/" ]]; do
        if [[ -d "$dir/src" && -d "$dir/src/perception" ]]; then
            echo "$dir"
            return 0
        fi
        dir="$(dirname "$dir")"
    done
    return 1
}

WS_ROOT=""

# 1. Check if we're already inside the workspace
if WS_ROOT=$(find_workspace "$(pwd)"); then
    ok "Found workspace at: $WS_ROOT"
# 2. Check common locations
elif [[ -d "$HOME/Documents/ap1/src/perception" ]]; then
    WS_ROOT="$HOME/Documents/ap1"
    ok "Found workspace at: $WS_ROOT"
elif [[ -d "$HOME/ap1/src/perception" ]]; then
    WS_ROOT="$HOME/ap1"
    ok "Found workspace at: $WS_ROOT"
else
    warn "Could not auto-detect workspace. Please enter the full path to your workspace root"
    warn "(This is the folder that contains 'src/', e.g. /home/yourname/Documents/ap1)"
    read -rp "  Workspace root: " WS_ROOT
    if [[ ! -d "$WS_ROOT/src" ]]; then
        err "Directory '$WS_ROOT/src' does not exist. Aborting."
        exit 1
    fi
    ok "Using workspace: $WS_ROOT"
fi

SRC_DIR="$WS_ROOT/src"
PERCEPTION_DIR="$SRC_DIR/perception"

# =============================================================================
# 2. DETECT SHELL & RC FILE
# =============================================================================
step "Detecting shell environment"

SHELL_NAME="$(basename "$SHELL")"
case "$SHELL_NAME" in
    zsh)  RC_FILE="$HOME/.zshrc" ;;
    bash) RC_FILE="$HOME/.bashrc" ;;
    *)    RC_FILE="$HOME/.profile"; warn "Unknown shell '$SHELL_NAME', using ~/.profile" ;;
esac
ok "Shell: $SHELL_NAME  →  RC file: $RC_FILE"

# =============================================================================
# 3. CHECK ROS2
# =============================================================================
step "Checking ROS2 installation"

ROS_SETUP=""
for distro in jazzy humble iron rolling; do
    if [[ -f "/opt/ros/$distro/setup.bash" ]]; then
        ROS_SETUP="/opt/ros/$distro/setup.bash"
        ok "Found ROS2 $distro at $ROS_SETUP"
        break
    fi
done

if [[ -z "$ROS_SETUP" ]]; then
    err "No ROS2 installation found in /opt/ros/. Please install ROS2 first."
    ERRORS=$((ERRORS+1))
else
    # Check if already sourced
    if command -v ros2 &>/dev/null; then
        ok "ROS2 is already sourced in this terminal"
    else
        info "Sourcing ROS2 for this session..."
        set +u  # ROS2 setup.bash uses unbound vars
        # shellcheck disable=SC1090
        source "$ROS_SETUP"
        set -u
        ok "ROS2 sourced"
    fi
fi

# =============================================================================
# 4. CHECK FOR MISPLACED BUILD ARTIFACTS
# =============================================================================
step "Checking for misplaced build artifacts"

MISPLACED=false
for dir in build install log; do
    if [[ -d "$SRC_DIR/$dir" ]]; then
        warn "Found '$dir/' inside src/ — this means colcon was run from the wrong directory"
        MISPLACED=true
    fi
done

if $MISPLACED; then
    echo ""
    read -rp "  Remove misplaced build artifacts from src/? [y/N] " confirm
    if [[ "$confirm" =~ ^[Yy]$ ]]; then
        rm -rf "$SRC_DIR/build" "$SRC_DIR/install" "$SRC_DIR/log"
        ok "Cleaned up misplaced artifacts"
    else
        warn "Skipping cleanup — build may fail or use wrong install paths"
    fi
else
    ok "No misplaced build artifacts found"
fi

# =============================================================================
# 5. CHECK / INSTALL UV
# =============================================================================
step "Checking uv package manager"

if command -v uv &>/dev/null; then
    UV_VER=$(uv --version 2>&1 | head -1)
    ok "uv found: $UV_VER"
else
    warn "uv not found. Installing via official installer..."
    curl -LsSf https://astral.sh/uv/install.sh | sh
    export PATH="$HOME/.cargo/bin:$HOME/.local/bin:$PATH"
    if command -v uv &>/dev/null; then
        ok "uv installed successfully"
    else
        err "uv installation failed. Please install manually: https://docs.astral.sh/uv/"
        ERRORS=$((ERRORS+1))
    fi
fi

# =============================================================================
# 6. SYNC PERCEPTION UV ENVIRONMENT
# =============================================================================
step "Setting up perception Python environment (uv sync)"

if [[ ! -d "$PERCEPTION_DIR" ]]; then
    err "Perception directory not found at $PERCEPTION_DIR"
    err "Make sure you have cloned https://github.com/WE-Autopilot/perception into src/"
    ERRORS=$((ERRORS+1))
else
    info "Running uv sync in $PERCEPTION_DIR ..."
    (cd "$PERCEPTION_DIR" && uv sync)
    ok "uv sync complete"

    # Detect the actual Python version used by the venv
    VENV_PYTHON=$(ls "$PERCEPTION_DIR/.venv/lib/" 2>/dev/null | grep "python" | head -1)
    if [[ -z "$VENV_PYTHON" ]]; then
        warn "Could not detect Python version in venv, defaulting to python3.12"
        VENV_PYTHON="python3.12"
    fi
    # Build the exact absolute path — e.g. /home/maharshii/Documents/ap1/src/perception/.venv/lib/python3.12/site-packages
    VENV_SITE_PACKAGES="$PERCEPTION_DIR/.venv/lib/$VENV_PYTHON/site-packages"

    if [[ -d "$VENV_SITE_PACKAGES" ]]; then
        ok "Venv site-packages: $VENV_SITE_PACKAGES"
    else
        err "Could not find site-packages at $VENV_SITE_PACKAGES"
        ERRORS=$((ERRORS+1))
    fi

    # Verify key packages
    info "Verifying perception dependencies..."
    VENV_PY="$PERCEPTION_DIR/.venv/bin/python3"
    for pkg in ultralytics onnxruntime cv2; do
        if "$VENV_PY" -c "import $pkg" 2>/dev/null; then
            ok "$pkg importable"
        else
            warn "$pkg not importable in venv — uv sync may have failed"
            ERRORS=$((ERRORS+1))
        fi
    done
fi

# =============================================================================
# 7. CHECK / INSTALL PYQT6 FOR CONSOLE
# =============================================================================
step "Checking console PyQt6 dependency"

# Console uses system python, so check system-level first
if /usr/bin/python3 -c "from PyQt6.QtWidgets import QApplication" 2>/dev/null; then
    ok "PyQt6 available on system Python"
else
    warn "PyQt6 not found on system Python — installing..."
    if pip install PyQt6 --break-system-packages 2>/dev/null || pip install PyQt6 2>/dev/null; then
        ok "PyQt6 installed"
    else
        err "Could not install PyQt6. Try manually: pip install PyQt6"
        ERRORS=$((ERRORS+1))
    fi
fi

# =============================================================================
# 8. ROSDEP
# =============================================================================
step "Installing ROS dependencies via rosdep"

if ! command -v rosdep &>/dev/null; then
    warn "rosdep not found, skipping"
else
    # Initialize rosdep if needed
    if [[ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]]; then
        info "Initializing rosdep..."
        sudo rosdep init 2>/dev/null || warn "rosdep init failed (may already be initialized)"
        rosdep update
    fi
    # Detect architecture — some packages have no ARM64 apt binaries for Jazzy
    ARCH=$(uname -m)
    info "Architecture detected: $ARCH"

    # Base skip keys — perception Python deps are fully managed by uv, not rosdep
    SKIP_KEYS="ap1_perception ultralytics onnxruntime opencv-python torch pyqt5 pyqt6"

    # ARM64-specific: pyrealsense2 and several perception packages
    # have no Jazzy aarch64 apt binaries available
    if [[ "$ARCH" == "aarch64" || "$ARCH" == "arm64" ]]; then
        warn "ARM64 detected — adding extra skip keys for packages with no Jazzy ARM64 apt binaries"
        SKIP_KEYS="$SKIP_KEYS pyrealsense2 librealsense2 ros-jazzy-realsense2-camera ros-jazzy-perception"
    fi

    # Refresh apt cache first — stale cache causes 404s on ARM64 Ubuntu ports mirror
    info "Refreshing apt cache before rosdep..."
    sudo apt-get update -qq || warn "apt-get update had warnings — continuing anyway"

    info "Running rosdep install..."
    info "Skipping keys (managed by uv or unavailable on this arch): $SKIP_KEYS"
    (
        cd "$WS_ROOT" && rosdep install \
            --from-paths src \
            --ignore-src \
            -r -y \
            --skip-keys "$SKIP_KEYS"
    ) || {
        warn "rosdep had failures — retrying with --fix-missing..."
        sudo apt-get install -f -y 2>/dev/null || true
        (
            cd "$WS_ROOT" && rosdep install \
                --from-paths src \
                --ignore-src \
                -r -y \
                --skip-keys "$SKIP_KEYS"
        )
    }
    ok "rosdep install complete"
fi

# =============================================================================
# 9. BUILD THE WORKSPACE
# =============================================================================
step "Building the workspace with colcon"

info "Building from $WS_ROOT ..."
(
    cd "$WS_ROOT"
    set +u  # ROS2 setup.bash uses unbound vars (AMENT_TRACE_SETUP_FILES)
    source "$ROS_SETUP"
    set -u
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | \
        grep -E "(Starting|Finished|Failed|Error|error:|warning:)" || true
)

if [[ -f "$WS_ROOT/install/setup.bash" ]]; then
    ok "Build succeeded — install/setup.bash exists"
    set +u
    source "$WS_ROOT/install/setup.bash"
    set -u
else
    err "Build may have failed — install/setup.bash not found"
    ERRORS=$((ERRORS+1))
fi

# Activate perception venv AFTER the workspace is built
# (colcon must not run inside the venv to avoid polluting the build)
if [[ -f "$PERCEPTION_DIR/.venv/bin/activate" ]]; then
    set +u
    source "$PERCEPTION_DIR/.venv/bin/activate"
    set -u
    ok "Perception venv activated: $VIRTUAL_ENV"
else
    warn "Perception venv not found — was uv sync successful?"
fi

# =============================================================================
# 10. CONFIGURE SHELL RC FILE
# =============================================================================
step "Configuring $RC_FILE"

WS_INSTALL_SETUP="$WS_ROOT/install/setup.bash"
# Resolve to absolute path so the RC line works in any future shell (no variables)
VENV_SITE_PACKAGES_ABS="$(realpath "$VENV_SITE_PACKAGES" 2>/dev/null || echo "$VENV_SITE_PACKAGES")"
PYTHONPATH_LINE="export PYTHONPATH=$VENV_SITE_PACKAGES_ABS:\$PYTHONPATH"
ROS_LINE="source $ROS_SETUP"
WS_LINE="source $WS_INSTALL_SETUP"

add_to_rc() {
    local line="$1"
    local label="$2"
    if grep -qF "$line" "$RC_FILE" 2>/dev/null; then
        ok "$label already in $RC_FILE"
    else
        echo "" >> "$RC_FILE"
        echo "# AP1 Autopilot — added by ap1_setup.sh" >> "$RC_FILE"
        echo "$line" >> "$RC_FILE"
        ok "Added $label to $RC_FILE"
    fi
}

add_to_rc "$ROS_LINE"           "ROS2 source"
add_to_rc "$WS_LINE"            "workspace overlay source"
add_to_rc "$PYTHONPATH_LINE"    "perception venv PYTHONPATH"

# =============================================================================
# 11. FINAL VERIFICATION
# =============================================================================
hdr "Verification"

info "Checking ROS2 can find ap1 packages..."
set +u
source "$ROS_SETUP"
source "$WS_INSTALL_SETUP"
set -u
export PYTHONPATH="${VENV_SITE_PACKAGES_ABS:-$VENV_SITE_PACKAGES}:${PYTHONPATH:-}"

PACKAGES=(ap1_msgs ap1_bringup ap1_control ap1_planning ap1_perception ap1_console mapping_localization_python)
for pkg in "${PACKAGES[@]}"; do
    if ros2 pkg prefix "$pkg" &>/dev/null; then
        ok "$pkg found"
    else
        warn "$pkg not found — may not have been cloned or built"
    fi
done

echo ""
info "Checking system Python can see perception dependencies..."
for pkg in ultralytics onnxruntime; do
    if /usr/bin/python3 -c "import $pkg" 2>/dev/null; then
        ok "/usr/bin/python3 sees $pkg"
    else
        warn "/usr/bin/python3 cannot see $pkg — PYTHONPATH may need a shell reload"
    fi
done

# =============================================================================
# SUMMARY
# =============================================================================
hdr "Setup Complete"

if [[ $ERRORS -eq 0 ]]; then
    echo -e "${GREEN}${BOLD}  All steps completed successfully!${NC}\n"
else
    echo -e "${YELLOW}${BOLD}  Setup finished with $ERRORS warning(s) — review output above${NC}\n"
fi

echo -e "${BOLD}  To launch the full system, open a new terminal and run:${NC}"
echo -e "  ${CYAN}ros2 launch ap1bringup fullsystem.launch.py${NC}\n"
echo -e "${BOLD}  Or for PnC + sim only:${NC}"
echo -e "  ${CYAN}ros2 launch ap1bringup pncbackend.launch.py${NC}\n"
echo -e "  ${YELLOW}Note: Open a NEW terminal so the RC file changes take effect.${NC}\n"
