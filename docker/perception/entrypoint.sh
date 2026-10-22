#!/usr/bin/env bash
# Source ROS + workspace overlay, expose the perception uv venv to system Python,
# then exec the container command (default: perception launch).
set -eo pipefail

AP1_WS="${AP1_WS:-/opt/ap1}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

# ROS setup scripts reference unbound vars (e.g. AMENT_TRACE_SETUP_FILES).
set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
# shellcheck disable=SC1090
source "${AP1_WS}/install/setup.bash"
set -u

# Prefer an explicit override, then the known Jazzy path, then a short scan.
VENV_ROOT="${AP1_PERCEPTION_VENV:-${AP1_WS}/src/perception/.venv}"
SITE_PACKAGES=""
if [[ -d "${VENV_ROOT}/lib/python3.12/site-packages" ]]; then
  SITE_PACKAGES="${VENV_ROOT}/lib/python3.12/site-packages"
elif [[ -d "${VENV_ROOT}" ]]; then
  SITE_PACKAGES="$(find "${VENV_ROOT}/lib" -maxdepth 2 -type d -name 'site-packages' 2>/dev/null | head -n 1 || true)"
fi

if [[ -n "${SITE_PACKAGES}" ]]; then
  export PYTHONPATH="${SITE_PACKAGES}${PYTHONPATH:+:${PYTHONPATH}}"
  export AP1_PERCEPTION_VENV="${VENV_ROOT}"
fi

exec "$@"
