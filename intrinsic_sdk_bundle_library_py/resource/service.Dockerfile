# Copyright 2026 Intrinsic Innovation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# intrinsic_sdk_cmake build base + user service code built and setup to run
ARG REPOSITORY=ghcr.io/intrinsic-ai
ARG TAG=latest
ARG ROS_DISTRO=lyrical
ARG BASE_IMAGE_TAG=${TAG}-${ROS_DISTRO}

# source stage: copy user source code and ignore pre-built underlay packages
FROM ${REPOSITORY}/intrinsic_sdk_cmake:${BASE_IMAGE_TAG} AS source

ARG SERVICE_PACKAGE
# Kept for backwards compatibility with callers passing --build-arg SOURCE_DIR=...
ARG SOURCE_DIR=src/sdk-ros
ARG OVERLAY_SOURCE=src

WORKDIR /opt/ros/overlay
ADD ${OVERLAY_SOURCE} /opt/ros/overlay/src

# Clean up unused test packages and ignore pre-built intrinsic_sdk_cmake underlay
# packages so rosdep and colcon only process the overlay service package(s).
RUN find /opt/ros/overlay/src -type d -path "*/test/functional_tests" 2>/dev/null | while read -r ft_dir; do \
        echo "Cleaning up unused test packages in ${ft_dir}..." \
        && find "${ft_dir}" -maxdepth 1 -type d ! -name "${SERVICE_PACKAGE}" ! -name "functional_tests" -exec rm -rf {} +; \
    done \
    && for pkg_path in $(colcon list --base-paths src --paths-only --packages-up-to intrinsic_sdk_cmake 2>/dev/null || true) \
                       $(colcon list --base-paths src --paths-only --packages-select intrinsic_sdk intrinsic_sdk_ros 2>/dev/null || true); do \
        if [ "$(basename "$pkg_path")" != "$SERVICE_PACKAGE" ]; then \
            touch "$pkg_path/COLCON_IGNORE"; \
        fi; \
    done


# build stage: install build dependencies + build the service package(s)
FROM source AS build

ARG SERVICE_PACKAGE
ARG DEPENDENCIES
ARG ROS_DISTRO=lyrical

RUN if [ "$ROS_DISTRO" != "jazzy" ] && [ "$ROS_DISTRO" != "lyrical" ]; then \
        echo "Error: Only ROS_DISTRO=jazzy or ROS_DISTRO=lyrical is supported for services currently." >&2; \
        exit 1; \
    fi

# Install build and run dependencies for the service package(s).
RUN \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    . /opt/intrinsic/intrinsic_sdk_cmake/install/setup.sh \
    && set -x \
    && rm -f /etc/apt/apt.conf.d/docker-clean \
    && echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' >/etc/apt/apt.conf.d/keep-cache \
    && apt-get update \
    && (rosdep init || true) \
    && rosdep update \
    && cd /opt/ros/overlay \
    && rosdep install \
        --from-paths src \
        --ignore-src \
        --default-yes \
    && apt-get install -y ccache python3-protobuf ${DEPENDENCIES} \
    && /usr/sbin/update-ccache-symlinks

# Build the service package(s).
RUN \
    --mount=type=cache,target=/ccache/ \
    . /opt/intrinsic/intrinsic_sdk_cmake/install/setup.sh \
    && set -x \
    && export CCACHE_DIR=/ccache \
    && export PATH="/usr/lib/ccache:$PATH" \
    && ccache -z \
    && cd /opt/ros/overlay \
    && colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
        --event-handlers=console_direct+ \
        --merge-install \
        --packages-up-to ${SERVICE_PACKAGE} \
    && ccache -s


# exec_depends stage: capture just the exec depends on top of intrinsic_sdk_cmake_run
FROM ${REPOSITORY}/intrinsic_sdk_cmake_run:${BASE_IMAGE_TAG} AS exec_depends

COPY --from=source \
    /opt/ros/overlay/src \
    /opt/ros/overlay/src

RUN \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    . /opt/intrinsic/intrinsic_sdk_cmake/install/setup.sh \
    && set -x \
    && rm -f /etc/apt/apt.conf.d/docker-clean \
    && echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' >/etc/apt/apt.conf.d/keep-cache \
    && apt-get update \
    && (rosdep init || true) \
    && rosdep update \
    && cd /opt/ros/overlay \
    && rosdep install \
        --from-paths src \
        --ignore-src \
        --default-yes \
        --dependency-types exec \
    && dpkg --get-selections > /service_exec_apt_packages.txt


# run stage: install exec dependencies + copy install artifacts from build stage
FROM ${REPOSITORY}/intrinsic_sdk_cmake_run:${BASE_IMAGE_TAG} AS run

ARG SERVICE_PACKAGE
ARG SERVICE_NAME
ARG DEPENDENCIES

# Install run dependencies for the service package(s).
COPY --from=exec_depends \
    /service_exec_apt_packages.txt \
    /service_exec_apt_packages.txt
RUN \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    set -x \
    && rm -f /etc/apt/apt.conf.d/docker-clean \
    && echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' >/etc/apt/apt.conf.d/keep-cache \
    && apt-get update \
    && apt-cache dumpavail | dpkg --merge-avail \
    && dpkg --set-selections < /service_exec_apt_packages.txt \
    && apt-get dselect-upgrade -y \
    && if [ -n "${DEPENDENCIES}" ]; then apt-get install -y ${DEPENDENCIES}; fi

COPY --from=build /opt/ros/overlay/install /opt/ros/overlay/install

RUN sed --in-place \
        --expression '$isource "/opt/intrinsic/intrinsic_sdk_cmake/install/setup.bash"' \
        /ros_entrypoint.sh \
    && sed --in-place \
        --expression '$isource "/opt/ros/overlay/install/setup.bash"' \
        /ros_entrypoint.sh

# Build arguments are not substituted in the CMD command and hence we set environment variables
# which can be substituted instead.
ENV SERVICE_PACKAGE=${SERVICE_PACKAGE}
ENV SERVICE_NAME=${SERVICE_NAME}

CMD exec /opt/ros/overlay/install/lib/${SERVICE_PACKAGE}/${SERVICE_NAME}_main
