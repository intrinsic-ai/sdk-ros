# This Dockerfile provides an image with intrinsic_sdk_cmake with its build dependencies.
# Use the intrinsic_sdk_cmake_run image if you only want the runtime dependencies.

ARG REPOSITORY=ghcr.io/intrinsic-ai
ARG TAG=latest
FROM ${REPOSITORY}/intrinsic_sdk_cmake_base:${TAG} AS base

# source stage: base + source added
FROM base AS source

ADD ./ /opt/intrinsic/intrinsic_sdk_cmake/src/intrinsic_sdk_ros

# Ensure we're running from the root of the repository so the ADD worked.
RUN cd /opt/intrinsic/intrinsic_sdk_cmake/src/intrinsic_sdk_ros \
    && ls ./intrinsic_sdk_cmake/docker_images/intrinsic_sdk_cmake.Dockerfile \
    || (echo "you must build this dockerfile from the root of the repository" \
    && false)

# build_export stage: source + rosdep install build_export depends
FROM source AS build_export

# Setup rosdep
RUN \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    set -x \
    && rm -f /etc/apt/apt.conf.d/docker-clean \
    && echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' >/etc/apt/apt.conf.d/keep-cache \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-rosdep

# Bootstrap rosdep
RUN set -x \
    && rosdep init \
    && rosdep update --rosdistro jazzy

# Install exec-like dependencies for the packages in intrinsic_sdk_cmake and
# save them for re-install in a later stage.
# Exclude intrinsic_sdk_ros and intrinsic_sdk for now.
# TODO(wjwwood): it would be nice to get the list of packages from rosdep
#   without installing them, but I couldn't figure out how to do that easily.
RUN \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    . /opt/ros/jazzy/setup.sh \
    && set -x \
    && rm -f /etc/apt/apt.conf.d/docker-clean \
    && echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' >/etc/apt/apt.conf.d/keep-cache \
    && apt-get update \
    && rosdep update --rosdistro jazzy \
    && cd /opt/intrinsic/intrinsic_sdk_cmake \
    && touch src/intrinsic_sdk_ros/intrinsic_sdk/COLCON_IGNORE \
    && touch src/intrinsic_sdk_ros/intrinsic_sdk_ros/COLCON_IGNORE \
    && rosdep install \
        --from-paths src \
        --ignore-src \
        --default-yes \
        --dependency-types exec \
    && dpkg --get-selections > /exec_apt_packages.txt \
    && rosdep install \
        --from-paths src \
        --ignore-src \
        --default-yes \
        --dependency-types buildtool_export \
        --dependency-types build_export \
        --dependency-types exec \
    && dpkg --get-selections > /build_export_apt_packages.txt

# build stage: build_export + rosdep install all depends + packages up to intrinsic_sdk_cmake built
FROM build_export AS build

# Install other development tools.
RUN \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    set -x \
    && rm -f /etc/apt/apt.conf.d/docker-clean \
    && echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' >/etc/apt/apt.conf.d/keep-cache \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        ccache \
        git \
        python3-colcon-common-extensions \
        python3-colcon-mixin \
        python3-vcstool

# Setup colcon mixin and metadata.
RUN set -x \
    && colcon mixin add default \
        https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml \
    && colcon mixin update \
    && colcon metadata add default \
        https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml \
    && colcon metadata update

# Install the rest of the standard dependencies for the packages in intrinsic_sdk_ros.
# Exclude intrinsic_sdk_ros and intrinsic_sdk for now.
RUN \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    . /opt/ros/jazzy/setup.sh \
    && set -x \
    && rm -f /etc/apt/apt.conf.d/docker-clean \
    && echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' >/etc/apt/apt.conf.d/keep-cache \
    && apt-get update \
    && rosdep update --rosdistro jazzy \
    && cd /opt/intrinsic/intrinsic_sdk_cmake \
    && touch src/intrinsic_sdk_ros/intrinsic_sdk/COLCON_IGNORE \
    && touch src/intrinsic_sdk_ros/intrinsic_sdk_ros/COLCON_IGNORE \
    && rosdep install \
        --from-paths src \
        --ignore-src \
        --default-yes

# Build intrinsic_sdk_cmake and all of its dependencies.
RUN \
    --mount=type=cache,target=/ccache/ \
    . /opt/ros/jazzy/setup.sh \
    && set -x \
    && export CCACHE_DIR=/ccache \
    && export PATH="/usr/lib/ccache:/usr/local/opt/ccache/libexec:$PATH" \
    && ccache -z \
    && cd /opt/intrinsic/intrinsic_sdk_cmake \
    && touch src/intrinsic_sdk_ros/intrinsic_sdk/COLCON_IGNORE \
    && touch src/intrinsic_sdk_ros/intrinsic_sdk_ros/COLCON_IGNORE \
    && colcon build \
        --cmake-args -DBUILD_TESTING=OFF \
        --merge-install \
    && ccache -s

# result stage: base + copy install artifacts + re-installed build_export depends + dev tools
FROM base AS result

# Get the installed artifacts from the build stage.
COPY --from=build \
    /opt/intrinsic/intrinsic_sdk_cmake/install \
    /opt/intrinsic/intrinsic_sdk_cmake/install

# Get the list of build_export depends from the build_export stage.
COPY --from=build_export \
    /build_export_apt_packages.txt \
    /build_export_apt_packages.txt
# Also get the list of exec depends from the build_export stage for use in run image later.
COPY --from=build_export \
    /exec_apt_packages.txt \
    /exec_apt_packages.txt

# Re-install the packages from the run_deps stage.
# This avoids having the source code in the final image.
RUN \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    set -x \
    && rm -f /etc/apt/apt.conf.d/docker-clean \
    && echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' >/etc/apt/apt.conf.d/keep-cache \
    && apt-get update \
    && apt-cache dumpavail | dpkg --merge-avail \
    && dpkg --set-selections < /build_export_apt_packages.txt \
    && apt-get dselect-upgrade -y

# Install other development tools.
RUN \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    set -x \
    && rm -f /etc/apt/apt.conf.d/docker-clean \
    && echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' >/etc/apt/apt.conf.d/keep-cache \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        git \
        python3-colcon-common-extensions \
        python3-colcon-mixin \
        python3-rosdep \
        python3-vcstool

# Bootstrap rosdep
RUN set -x \
    && rosdep init \
    && rosdep update --rosdistro jazzy

# Setup colcon mixin and metadata.
RUN set -x \
    && colcon mixin add default \
        https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml \
    && colcon mixin update \
    && colcon metadata add default \
        https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml \
    && colcon metadata update

CMD ["bash"]
