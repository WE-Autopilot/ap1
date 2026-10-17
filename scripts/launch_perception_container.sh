#!/usr/bin/env bash
# Build the ap1-perception image and launch perception nodes in one command.
# Usage (from anywhere):
#   ./scripts/launch_perception_container.sh [extra docker compose args...]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
COMPOSE_FILE="${WS_ROOT}/docker/perception/compose.yaml"

if [[ ! -f "${COMPOSE_FILE}" ]]; then
  echo "error: compose file not found at ${COMPOSE_FILE}" >&2
  exit 1
fi

if ! command -v docker >/dev/null 2>&1; then
  echo "error: docker is required but was not found on PATH" >&2
  exit 1
fi

if ! docker info >/dev/null 2>&1; then
  echo "error: docker daemon is not running" >&2
  exit 1
fi

# Prefer host networking on native Linux for ROS 2 DDS discovery.
if [[ -z "${AP1_DOCKER_NETWORK_MODE:-}" ]]; then
  if [[ "$(uname -s)" == "Linux" ]]; then
    export AP1_DOCKER_NETWORK_MODE=host
  else
    export AP1_DOCKER_NETWORK_MODE=bridge
  fi
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

cd "${WS_ROOT}"
echo "Building and launching AP1 perception (network_mode=${AP1_DOCKER_NETWORK_MODE}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID})"
exec docker compose -f "${COMPOSE_FILE}" up --build "$@"
