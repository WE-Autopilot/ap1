# Shared build pipeline for every AP1 module
#
# A module Dockerfile (e.g. perception.Dockerfile) provides two stages, which
# docker-bake.hcl passes in as named build contexts:
#   module-source  colcon workspace sources in ${AP1_WS}/src
#   module-deps    the base image plus anything rosdep can't express
#
# Stages:
#   rosdep_scan   resolve every package.xml into apt and pip package lists
#   dependencies  install those lists (cached until the lists change, not the code)
#   build         colcon build into ${AP1_INSTALL}
#   deploy        dependencies + install tree only; what runs on the car
#   develop       dependencies + dev tools, no prebuilt workspace; source is
#                 mounted and built in ${AP1_WS}. Its user "ubuntu" takes the
#                 host's UID/GID so files in bind mounts keep their owner

######################## rosdep_scan ########################
FROM module-source AS rosdep_scan
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
WORKDIR ${AP1_WS}
# Fail on any installer other than apt or pip, rather than silently skipping it
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

####################### dependencies ########################
FROM module-deps AS dependencies
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
COPY --from=rosdep_scan /tmp/apt_packages /tmp/pip_packages /tmp/
COPY docker/config/pip-constraints.txt /tmp/pip-constraints.txt
RUN apt-get update && \
    xargs -r -a /tmp/apt_packages apt-get install -y --no-install-recommends && \
    rm -rf /var/lib/apt/lists/* && \
    xargs -r -a /tmp/pip_packages pip install --no-cache-dir -c /tmp/pip-constraints.txt && \
    rm /tmp/apt_packages /tmp/pip_packages /tmp/pip-constraints.txt

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

########################## develop ##########################
# Built from dependencies, not deploy, so there is one copy of the workspace:
# the mounted source, built with colcon in ${AP1_WS}.
FROM dependencies AS develop
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG USER_UID=1000
ARG USER_GID=1000

RUN apt-get update && \
    apt-get install -y --no-install-recommends gdb less nano sudo tmux vim && \
    rm -rf /var/lib/apt/lists/*

# Give Ubuntu's built-in "ubuntu" user (UID/GID 1000) the host's IDs, like
# devcontainers do, so files created in bind mounts belong to the host user.
# If the host GID is already another group's (e.g. 20 on macOS), make that
# group ubuntu's primary one. If the host UID belongs to another user (e.g. 0),
# keep 1000. Pre-create the workspace dirs so named volumes mounted there
# inherit ubuntu's ownership.
RUN if ! id ubuntu > /dev/null 2>&1; then useradd --create-home --shell /bin/bash ubuntu; fi && \
    if [ "$(id -u ubuntu)" != "${USER_UID}" ]; then \
      if getent passwd "${USER_UID}" > /dev/null; then \
        echo "warning: UID ${USER_UID} is taken by $(getent passwd "${USER_UID}" | cut -d: -f1), keeping ubuntu at $(id -u ubuntu)"; \
      else \
        usermod --uid "${USER_UID}" ubuntu; \
      fi; \
    fi && \
    if [ "$(id -g ubuntu)" != "${USER_GID}" ]; then \
      if getent group "${USER_GID}" > /dev/null; then \
        usermod --gid "${USER_GID}" ubuntu; \
      else \
        groupmod --gid "${USER_GID}" ubuntu; \
      fi; \
    fi && \
    echo "ubuntu ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/ubuntu && \
    chmod 0440 /etc/sudoers.d/ubuntu && \
    mkdir -p "${AP1_WS}/src" "${AP1_WS}/build" "${AP1_WS}/install" "${AP1_WS}/log" && \
    chown -R ubuntu: "${AP1_WS}" /home/ubuntu

USER ubuntu
WORKDIR ${AP1_WS}
