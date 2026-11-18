# Sourced by the entrypoint and by interactive shells in every AP1 container.
# Layers, lowest first: ROS, the image's built workspace, then a dev overlay
# built inside the container (only exists in dev containers).

# shellcheck disable=SC1090,SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f "${AP1_INSTALL}/setup.bash" ]; then
    source "${AP1_INSTALL}/setup.bash"
fi
if [ -f "${AP1_WS}/install/setup.bash" ]; then
    source "${AP1_WS}/install/setup.bash"
fi
