# Planning module. Copies over source code built by colcon and relevant to planning.
# Also present as an intermediary for dependency management in build stages.

########################## source ##########################
FROM base AS source
WORKDIR ${AP1_WS}/src
COPY src/msgs msgs
COPY src/planning planning

####################### dependencies #######################
FROM base AS dependencies
