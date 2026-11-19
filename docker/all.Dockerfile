# All modules in one image. Merges every module's source stage, which
# docker-bake.hcl passes in as named contexts (<module>-source).
# Dependencies aren't merged here: a dependencies stage can install packages,
# which COPY can't merge. docker-bake.hcl stacks them instead (all-deps).

########################## source ##########################
FROM base AS source
COPY --from=perception-source ${AP1_WS}/src ${AP1_WS}/src
COPY --from=control-source ${AP1_WS}/src ${AP1_WS}/src
