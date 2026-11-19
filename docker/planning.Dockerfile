# Provide module sources and a stage for extra dependencies.


########################## source ##########################
FROM base AS source
WORKDIR ${AP1_WS}/src
COPY src/msgs msgs
COPY src/planning planning

####################### dependencies #######################
FROM base AS dependencies
