#!/bin/bash
set -e

# ROS/colcon setup scripts parse "$@", so hide the command's arguments from them
command=("$@")
set --
# shellcheck disable=SC1091
source /usr/local/share/ap1/ap1_env.sh

# exec so the command gets signals directly (compose runs with init: true)
exec "${command[@]}"
