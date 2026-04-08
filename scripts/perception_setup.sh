#!/bin/bash
set -e # fail on error

apt-get update -y
rosdep update
rosdep install --from-paths src -y --ignore-src

pip install onnxruntime ultralytics --break-system-packages --ignore-installed numpy
