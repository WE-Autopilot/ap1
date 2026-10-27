# AP1 base image: ROS 2 and AP1 tooling on top of any Ubuntu 24.04 image.
#
# Every hardware variant is built from this one file, so the CPU and GPU images
# differ only in GENERIC_IMAGE (plain Ubuntu vs NVIDIA's CUDA image) and AP1_GPU.
# Variants are listed in docker-bake.hcl.

ARG GENERIC_IMAGE=ubuntu:24.04
FROM ${GENERIC_IMAGE}

ARG ROS_DISTRO=jazzy
ARG ROS_APT_SOURCE_VERSION=1.3.0
# 1 on images with CUDA. package.xml files read it in rosdep conditions to pick
# GPU builds of dependencies (e.g. onnxruntime-gpu instead of onnxruntime).
ARG AP1_GPU=0

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    TZ=Etc/UTC

# ROS 2 apt repository (ros2-apt-source manages the signing key) and ROS base + build tools
RUN apt-get update && \
    apt-get install -y --no-install-recommends ca-certificates curl tzdata && \
    curl -fsSL -o /tmp/ros2-apt-source.deb \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo "${VERSION_CODENAME}")_all.deb" && \
    apt-get install -y --no-install-recommends /tmp/ros2-apt-source.deb && \
    rm /tmp/ros2-apt-source.deb && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
      "ros-${ROS_DISTRO}-ros-base" \
      build-essential \
      git \
      python3-pip \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-vcstool && \
    rm -rf /var/lib/apt/lists/*

RUN rosdep init && rosdep update --rosdistro "${ROS_DISTRO}"

ENV ROS_DISTRO=${ROS_DISTRO} \
    AP1_GPU=${AP1_GPU} \
    AP1_WS=/ws \
    AP1_INSTALL=/opt/ap1 \
    # Containers own their Python environment, so pip may install next to apt
    # (rosdep uses pip for keys like python3-onnxruntime-pip)
    PIP_BREAK_SYSTEM_PACKAGES=1 \
    PIP_ROOT_USER_ACTION=ignore

# ROS + AP1 environment for the entrypoint and for every interactive shell (docker exec)
COPY docker/config/ap1_env.sh /usr/local/share/ap1/ap1_env.sh
COPY docker/config/ap1_entrypoint.sh /usr/local/bin/ap1_entrypoint.sh
RUN echo 'source /usr/local/share/ap1/ap1_env.sh' >> /etc/bash.bashrc

WORKDIR ${AP1_WS}
ENTRYPOINT ["/usr/local/bin/ap1_entrypoint.sh"]
CMD ["bash"]
