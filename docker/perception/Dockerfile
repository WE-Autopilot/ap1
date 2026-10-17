# AP1 Perception runtime image.
# Base: ROS 2 Jazzy desktop (Python 3.12) + uv + colcon, per #35/#48.
# Python ML deps come from uv.lock; ROS deps from package.xml via rosdep.

FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    ROS_DISTRO=jazzy \
    AP1_WS=/opt/ap1 \
    UV_LINK_MODE=copy \
    UV_COMPILE_BYTECODE=1 \
    PYTHONUNBUFFERED=1

# System tooling: git/curl for fetch, python3-pip/vcstool/colcon for workspace build.
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl \
        git \
        python3-pip \
        python3-venv \
        ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# uv (pinned installer channel; binary lands in ~/.local/bin)
RUN curl -LsSf https://astral.sh/uv/install.sh | sh
ENV PATH="/root/.local/bin:${PATH}"

# colcon + vcstool on system Python (outside the perception venv)
RUN pip3 install --break-system-packages --no-cache-dir \
        "colcon-common-extensions" \
        "vcstool" \
        "empy==3.3.4"

WORKDIR ${AP1_WS}

# Shared messages (declared dependency of ap1_perception).
ARG AP1_MSGS_REF=main
RUN mkdir -p src \
    && git clone --depth 1 --branch "${AP1_MSGS_REF}" \
        https://github.com/WE-Autopilot/ap1_msgs.git src/msgs

# Perception package (includes pyproject.toml, uv.lock, package.xml, models).
COPY src/perception src/perception

# Resolve Python deps with uv (frozen lockfile) into a local .venv.
WORKDIR ${AP1_WS}/src/perception
RUN uv sync --frozen --no-dev

# ROS system deps for perception + msgs only. Skip keys managed by uv.
WORKDIR ${AP1_WS}
RUN apt-get update \
    && rosdep update --rosdistro="${ROS_DISTRO}" \
    && rosdep install \
        --from-paths src/perception src/msgs \
        --ignore-src \
        -r -y \
        --rosdistro="${ROS_DISTRO}" \
        --skip-keys="ap1_perception ultralytics onnxruntime opencv-python torch pyqt5 pyqt6 python3-pyqt6" \
    && rm -rf /var/lib/apt/lists/*

# Build only what perception needs. Use a normal (non-symlink) install so the
# image remains valid after build/ is removed for size.
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build \
        --packages-up-to ap1_perception \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log /root/.cache

# Dynamic venv path for launch files (no hard-coded host home paths).
ENV AP1_PERCEPTION_VENV=${AP1_WS}/src/perception/.venv \
    ROS_DOMAIN_ID=0

COPY docker/perception/entrypoint.sh /usr/local/bin/ap1-perception-entrypoint
RUN chmod +x /usr/local/bin/ap1-perception-entrypoint

WORKDIR ${AP1_WS}
ENTRYPOINT ["/usr/local/bin/ap1-perception-entrypoint"]
CMD ["ros2", "launch", "ap1_perception", "perception.launch.py"]
