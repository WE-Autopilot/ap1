# Control module. Copies over source code built by colcon and relevant to control.
# Also present as an intermediary for dependency management in build stages.

########################## source ##########################
FROM base AS source
WORKDIR ${AP1_WS}/src
COPY src/msgs msgs
COPY src/control control

####################### dependencies #######################
FROM base AS dependencies