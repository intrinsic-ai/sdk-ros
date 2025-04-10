ARG TAG=latest
FROM ghcr.io/intrinsic-ai/intrinsic_sdk_cmake:${TAG} as build_image

FROM ghcr.io/intrinsic-ai/intrinsic_sdk_cmake_base:${TAG} as result

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
RUN set -x \
    && apt-get update \
    && apt-cache dumpavail | dpkg --merge-avail \
    && dpkg --set-selections < /exec_apt_packages.txt \
    && apt-get dselect-upgrade -y \
    && rm -rf /var/lib/apt/lists/*
