ARG REPOSITORY=ghcr.io/intrinsic-ai
ARG TAG=latest
FROM ${REPOSITORY}/intrinsic_sdk_cmake:${TAG} as build_image

FROM ${REPOSITORY}/intrinsic_sdk_cmake_base:${TAG} as result

# Get the installed artifacts from the build stage.
COPY --from=build_image \
    /opt/intrinsic/intrinsic_sdk_cmake/install \
    /opt/intrinsic/intrinsic_sdk_cmake/install

# Get the list of exec dependencies from a previous build stage.
COPY --from=build_image \
    /exec_apt_packages.txt \
    /exec_apt_packages.txt

# Re-install the packages needed for exec.
# This avoids having the source code and unnecessary dependencies in the final image.
RUN \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    set -x \
    && rm -f /etc/apt/apt.conf.d/docker-clean \
    && echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' >/etc/apt/apt.conf.d/keep-cache \
    && apt-get update \
    && apt-cache dumpavail | dpkg --merge-avail \
    && dpkg --set-selections < /exec_apt_packages.txt \
    && apt-get dselect-upgrade -y
