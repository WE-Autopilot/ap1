# Mapping & Localization module: ap1_mapping, ap1_localization and LIO-SAM.
# Also present as an intermediary for dependency management in build stages.
#
# LIO-SAM isn't in the build context (.dockerignore drops src/external), so the
# source stage clones it at the commit pinned in ap1.repos and patches it for
# Jazzy with the same script you run on your checkout for the dev container.

########################## source ##########################
FROM base AS source
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
WORKDIR ${AP1_WS}/src
# ap1_msgs: the message package every module shares
COPY src/msgs msgs
COPY src/mapping mapping
COPY src/localization localization
COPY ap1.repos /tmp/ap1.repos
COPY docker/config/patch_lio_sam.sh /usr/local/bin/patch_lio_sam.sh
# url and version of the LIO-SAM entry in ap1.repos, so the pin lives in one place
RUN LIO_URL="$(awk '/LIO-SAM:/{f=1} f && /url:/{print $2; exit}' /tmp/ap1.repos)" && \
    LIO_REF="$(awk '/LIO-SAM:/{f=1} f && /version:/{print $2; exit}' /tmp/ap1.repos)" && \
    test -n "$LIO_URL" && test -n "$LIO_REF" && \
    git clone --quiet "$LIO_URL" external/LIO-SAM && \
    git -C external/LIO-SAM checkout --quiet "$LIO_REF" && \
    rm -rf external/LIO-SAM/.git /tmp/ap1.repos && \
    bash /usr/local/bin/patch_lio_sam.sh external/LIO-SAM

####################### dependencies #######################
# Dependencies rosdep can't express go here. Use it as a last resort: anything
# declared in a package.xml above is installed by base.Dockerfile.
FROM base AS dependencies
# procps: pgrep/pkill, used by scripts/map_carla.sh to stop leftover LIO-SAM runs
RUN apt-get update && \
    apt-get install -y --no-install-recommends procps && \
    rm -rf /var/lib/apt/lists/*
