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

# intrinsic_sdk_cmake build base + user code built and setup to run
ARG REPOSITORY=ghcr.io/intrinsic-ai
ARG TAG=latest
ARG ROS_DISTRO=jazzy
ARG SKILL_TYPE=cpp
FROM ${REPOSITORY}/intrinsic_sdk_cmake:${TAG} AS source

# The name of the skill.
ARG SKILL_NAME
# The name of the cmake project/ament package that contains the skill executable.
ARG SKILL_PACKAGE
# The relative path for the skill executable.
ARG SKILL_EXECUTABLE
# The relative path for the skill config.
ARG SKILL_CONFIG
# The reverse domain name for the organization in the asset-id label.
ARG SKILL_ASSET_ID_ORG=com.example

# Colcon workspace for building the user's packages.
# Note use the _workspace suffix to prevent accidental collision with other
# things in opt like ros or intrinsic_sdk_cmake.
ENV SKILL_WORKSPACE=/opt/${SKILL_NAME}_workspace

# Add the user's code to the container.
ADD ./ $SKILL_WORKSPACE/src

# build stage: build dependencies + build the packages
FROM source AS build

ARG SKILL_PACKAGE
ARG SKILL_TYPE
ARG ROS_DISTRO

RUN if [ "$ROS_DISTRO" != "jazzy" ]; then \
        echo "Error: Only ROS_DISTRO=jazzy is supported for skills currently." >&2; \
        exit 1; \
    fi

# Install build and run dependencies for the user's packages.
RUN \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    . /opt/intrinsic/intrinsic_sdk_cmake/install/setup.sh \
    && set -x \
    && rm -f /etc/apt/apt.conf.d/docker-clean \
    && echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' >/etc/apt/apt.conf.d/keep-cache \
    && apt-get update \
    && (rosdep init || true) \
    && echo "python3-absl-py:" > /etc/ros/rosdep/custom.yaml \
    && echo "  ubuntu: [python3-absl]" >> /etc/ros/rosdep/custom.yaml \
    && echo "python3-retrying:" >> /etc/ros/rosdep/custom.yaml \
    && echo "  ubuntu: [python3-retrying]" >> /etc/ros/rosdep/custom.yaml \
    && echo "yaml file:///etc/ros/rosdep/custom.yaml" > /etc/ros/rosdep/sources.list.d/50-custom.list \
    && rosdep update \
    && cd $SKILL_WORKSPACE \
    && rosdep install \
        --from-paths src \
        --ignore-src \
        --default-yes \
        --dependency-types exec \
    && dpkg --get-selections > /user_exec_apt_packages.txt \
    && rosdep install \
        --from-paths src \
        --ignore-src \
        --default-yes \
    && apt-get install -y ccache \
    && /usr/sbin/update-ccache-symlinks

# Build the user's packages.
RUN \
    --mount=type=cache,target=/ccache/ \
    . /opt/intrinsic/intrinsic_sdk_cmake/install/setup.sh \
    && set -x \
    && export CCACHE_DIR=/ccache \
    && export PATH="/usr/lib/ccache:$PATH" \
    && ccache -z \
    && cd $SKILL_WORKSPACE \
    && colcon build \
        --cmake-args -DBUILD_TESTING=ON \
        --merge-install \
        --packages-up-to $SKILL_PACKAGE \
    && ccache -s

# Copy pybind11_abseil if python skill
RUN mkdir -p $SKILL_WORKSPACE/bindings/pybind11_abseil \
    && if [ -d $SKILL_WORKSPACE/build/pybind11_abseil_vendor/pybind11_abseil_vendor-prefix/src/pybind11_abseil_vendor-build ]; then \
        cp -r $SKILL_WORKSPACE/build/pybind11_abseil_vendor/pybind11_abseil_vendor-prefix/src/pybind11_abseil_vendor-build/* $SKILL_WORKSPACE/bindings/pybind11_abseil/; \
    fi

# exec_depends stage: capture just the exec depends using the source
FROM ${REPOSITORY}/intrinsic_sdk_cmake_run:${TAG} AS exec_depends

ARG SKILL_NAME
ARG SKILL_TYPE
ENV SKILL_WORKSPACE=/opt/${SKILL_NAME}_workspace

COPY --from=source \
    $SKILL_WORKSPACE/src \
    $SKILL_WORKSPACE/src

RUN \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    . /opt/intrinsic/intrinsic_sdk_cmake/install/setup.sh \
    && set -x \
    && rm -f /etc/apt/apt.conf.d/docker-clean \
    && echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' >/etc/apt/apt.conf.d/keep-cache \
    && apt-get update \
    && (rosdep init || true) \
    && echo "python3-absl-py:" > /etc/ros/rosdep/custom.yaml \
    && echo "  ubuntu: [python3-absl]" >> /etc/ros/rosdep/custom.yaml \
    && echo "python3-retrying:" >> /etc/ros/rosdep/custom.yaml \
    && echo "  ubuntu: [python3-retrying]" >> /etc/ros/rosdep/custom.yaml \
    && echo "yaml file:///etc/ros/rosdep/custom.yaml" > /etc/ros/rosdep/sources.list.d/50-custom.list \
    && rosdep update \
    && cd $SKILL_WORKSPACE \
    && rosdep install \
        --from-paths src \
        --ignore-src \
        --default-yes \
        --dependency-types exec \
    && dpkg --get-selections > /user_exec_apt_packages.txt

# run stage: install exec dependencies + copy install artifacts from build stage
FROM ${REPOSITORY}/intrinsic_sdk_cmake_run:${TAG} AS run

ARG SKILL_EXECUTABLE
ARG SKILL_CONFIG
ARG SKILL_ASSET_ID_ORG

ARG SKILL_NAME
ENV SKILL_WORKSPACE=/opt/${SKILL_NAME}_workspace
ARG SKILL_TYPE
ENV SKILL_TYPE=$SKILL_TYPE

# Install run dependencies for user's packages.
COPY --from=exec_depends \
    /user_exec_apt_packages.txt \
    /user_exec_apt_packages.txt
RUN \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    set -x \
    && rm -f /etc/apt/apt.conf.d/docker-clean \
    && echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' >/etc/apt/apt.conf.d/keep-cache \
    && apt-get update \
    && apt-cache dumpavail | dpkg --merge-avail \
    && dpkg --set-selections < /user_exec_apt_packages.txt \
    && apt-get dselect-upgrade -y \
    && apt-get install -y python3-pip \
    && pip3 install --upgrade protobuf grpcio grpcio-status --break-system-packages

# Copy build artifacts from user's packages.
COPY --from=build $SKILL_WORKSPACE/install $SKILL_WORKSPACE/install

# Copy pybind11_abseil bindings
COPY --from=build $SKILL_WORKSPACE/bindings/pybind11_abseil /opt/bindings/pybind11_abseil
ENV PYTHONPATH="/opt/bindings/pybind11_abseil"

# Ensure skill executable and config file exist.
ENV SKILL_EXECUTABLE_ABS=$SKILL_WORKSPACE/install/$SKILL_EXECUTABLE
RUN ls $SKILL_EXECUTABLE_ABS \
    || (echo "Skill executable does not exist '$SKILL_EXECUTABLE_ABS'" \
        && false)
ENV SKILL_CONFIG_ABS=$SKILL_WORKSPACE/install/$SKILL_CONFIG
RUN ls $SKILL_CONFIG_ABS \
    || (echo "Skill executable does not exist '$SKILL_CONFIG_ABS'" \
        && false)

# Link skill executable and skill config file into well known locations needed by Flowstate.
# Also ensure the user's workspace is sourced so the skill main can be run correctly.
RUN if [ "$SKILL_TYPE" = "python" ]; then \
        sed -i '1i #!/usr/bin/env python3' "$SKILL_EXECUTABLE_ABS" && chmod +x "$SKILL_EXECUTABLE_ABS"; \
    fi

RUN set -x \

    && mkdir -p /skills \
    && ln -sf $SKILL_EXECUTABLE_ABS /skills/skill_service \
    && ln -sf $SKILL_CONFIG_ABS /skills/skill_service_config.proto.bin \
    && sed --in-place \
        --expression '$isource "$SKILL_WORKSPACE/install/setup.bash"' \
        /ros_entrypoint.sh

# Set some labels used by Flowstate.
LABEL "ai.intrinsic.asset-id"="${SKILL_ASSET_ID_ORG}.${SKILL_NAME}"
LABEL "ai.intrinsic.skill-image-name"="${SKILL_NAME}"

# Execute the skill main by default, but note that Flowstate will likely override this statement.
CMD ["sh", "-c", "if [ \"$SKILL_TYPE\" = \"python\" ]; then python3 /skills/skill_service --skill_service_config_filename=/skills/skill_service_config.proto.bin; else /skills/skill_service --skill_service_config_filename=/skills/skill_service_config.proto.bin; fi"]
