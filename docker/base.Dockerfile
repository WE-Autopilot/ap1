# AP1 base image - Defines ap1's base image, installing ros2 via ros2-apt-source and configuring for ap1 usage.
#                  Dependencies from the <module>.Dockerfile file(s) that base is paired with are handled by rosdep 
#                  and pip respectively.
#
# We define 6 build stages:
#   base          ROS 2 and AP1 tooling on top of any Ubuntu 24.04 image
#   rosdep_scan   resolve a <module>.Dockerfile's package.xml into apt and pip package lists
#   dependencies  install those lists (cached until the lists change)
#   build         colcon build into ${AP1_INSTALL}
#   deploy        dependencies + install tree only; what runs on the car
#   develop       dependencies + dev tools

ARG GENERIC_IMAGE=ubuntu:24.04

########################### base ############################
FROM ${GENERIC_IMAGE} AS base

ARG ROS_DISTRO=jazzy
# pinning the ros2-apt-source package (noble build) at this version.
# when updating version, make sure to update the hash as well
ARG ROS_APT_SOURCE_VERSION=1.3.0
ARG ROS_APT_SOURCE_SHA256=f31d84adf5054c7d60ded0e82c0f776a77ea33af53d08f40b9ad7c94cca55296
ARG AP1_GPU=0

# all RUN commands use bash + pipefail
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# user environment setup
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    TZ=Etc/UTC

# setup ROS 2 apt repository sources and install ros-base
RUN apt-get update && \
    apt-get install -y --no-install-recommends ca-certificates curl tzdata && \
    curl -fsSL -o /tmp/ros2-apt-source.deb \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb" && \
    echo "${ROS_APT_SOURCE_SHA256} */tmp/ros2-apt-source.deb" | sha256sum --strict --check && \
    apt-get install -y --no-install-recommends /tmp/ros2-apt-source.deb && \
    rm -f /tmp/ros2-apt-source.deb && \
    apt-get update && \
    apt-get install -y --no-install-recommends "ros-${ROS_DISTRO}-ros-base" && \
    rm -rf /var/lib/apt/lists/*

# install bootstrap tools
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      build-essential \
      git \
      python3-pip \
      python3-colcon-common-extensions \
      python3-rosdep && \
    rm -rf /var/lib/apt/lists/*

# initialise rosdep
RUN rosdep init && rosdep update --rosdistro "${ROS_DISTRO}"

# setup env variables
ENV ROS_DISTRO=${ROS_DISTRO} \
    AP1_GPU=${AP1_GPU} \
    AP1_WS=/ws \
    AP1_INSTALL=/opt/ap1 \
    PIP_BREAK_SYSTEM_PACKAGES=1 \
    PIP_ROOT_USER_ACTION=ignore

# No COPY from the build context in this stage: bake passes it to other targets
# as a context, and a local COPY here breaks the cache of every stage built on
# it, including the slow dependencies install (see docker/README.md).
WORKDIR ${AP1_WS}

######################## rosdep_scan ########################
FROM module-source AS rosdep_scan
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
WORKDIR ${AP1_WS}
RUN rosdep update --rosdistro "${ROS_DISTRO}" > /dev/null && \
    rosdep keys --from-paths src --ignore-src | sort -u > /tmp/rosdep_keys && \
    xargs -r -a /tmp/rosdep_keys rosdep resolve --rosdistro "${ROS_DISTRO}" > /tmp/rosdep_resolved && \
    : > /tmp/apt_packages && : > /tmp/pip_packages && \
    awk '/^#ROSDEP\[/ { next } \
         /^#/ { installer = substr($0, 2); \
                if (installer != "apt" && installer != "pip") { \
                  print "unsupported rosdep installer: " installer > "/dev/stderr"; exit 1 } \
                next } \
         { for (i = 1; i <= NF; i++) print $i >> ("/tmp/" installer "_packages") }' \
        /tmp/rosdep_resolved && \
    sort -u -o /tmp/apt_packages /tmp/apt_packages && \
    sort -u -o /tmp/pip_packages /tmp/pip_packages && \
    echo "apt:" $(cat /tmp/apt_packages) && echo "pip:" $(cat /tmp/pip_packages)

# TODO: add cmake depedency handling
####################### dependencies ########################
FROM module-deps AS dependencies
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
# we copy the results of the rosdep scan from the rosdep_scan build stage 
COPY --from=rosdep_scan /tmp/apt_packages /tmp/pip_packages /tmp/
COPY docker/config/pip-constraints.txt /tmp/pip-constraints.txt
RUN apt-get update && \
    xargs -r -a /tmp/apt_packages apt-get install -y --no-install-recommends && \
    rm -rf /var/lib/apt/lists/* && \
    xargs -r -a /tmp/pip_packages pip install --no-cache-dir -c /tmp/pip-constraints.txt && \
    rm /tmp/apt_packages /tmp/pip_packages /tmp/pip-constraints.txt

# After the install, so editing these files doesn't reinstall packages.
# ROS + AP1 environment for the entrypoint and for every interactive shell (docker exec)
COPY docker/config/ap1_env.sh /usr/local/share/ap1/ap1_env.sh
COPY docker/config/ap1_entrypoint.sh /usr/local/bin/ap1_entrypoint.sh
RUN echo 'source /usr/local/share/ap1/ap1_env.sh' >> /etc/bash.bashrc
# source ROS + AP1 workspaces on entry
ENTRYPOINT ["/usr/local/bin/ap1_entrypoint.sh"]
CMD ["bash"]

########################### build ###########################
FROM dependencies AS build
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
COPY --from=rosdep_scan ${AP1_WS}/src ${AP1_WS}/src
WORKDIR ${AP1_WS}
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" && \
    colcon build \
      --merge-install \
      --install-base "${AP1_INSTALL}" \
      --event-handlers console_direct- \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTING=OFF \
        -DPython3_EXECUTABLE=/usr/bin/python3

########################## deploy ###########################
FROM dependencies AS deploy
COPY --from=build ${AP1_INSTALL} ${AP1_INSTALL}
USER ubuntu

########################## develop ##########################
FROM dependencies AS develop
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# install dev tools and enforce usage of ubuntu user 1000:1000
RUN apt-get update && \
    apt-get install -y --no-install-recommends python3-vcstool gdb less nano sudo tmux vim && \
    rm -rf /var/lib/apt/lists/* && \
    echo "ubuntu ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/ubuntu && \
    chmod 0440 /etc/sudoers.d/ubuntu && \
    mkdir -p "${AP1_WS}/src" "${AP1_WS}/build" "${AP1_WS}/install" "${AP1_WS}/log" && \
    chown -R ubuntu:ubuntu "${AP1_WS}"

USER ubuntu
WORKDIR ${AP1_WS}
