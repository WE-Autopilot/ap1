# Perception module. Copies over source code built by colcon and relevant to perception.
# Also present as an intermediary for dependency management in build stages.

########################## source ##########################
FROM base AS source
WORKDIR ${AP1_WS}/src
# ap1_msgs: the message package every module shares
COPY src/msgs msgs
COPY src/perception perception
# The uv files serve the native (non-Docker) setup only. pyproject.toml's
# [project] table breaks colcon's setup.py introspection, and package.xml is
# the dependency list here.
RUN rm -f perception/pyproject.toml perception/uv.lock perception/.python-version

####################### dependencies #######################
# Dependencies rosdep can't express go here. Use it as a last resort: anything
# declared in src/perception/package.xml is installed by base.Dockerfile.
FROM base AS dependencies
ENV AP1_PERCEPTION_VENV=none
