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

# Build up the service with these stages:
#   - base (ros:lyrical + settings) ->
#   - underlay (dependencies) ->
#   - overlay (user code) ->
#   - result (base + copied install folder of underlay and overlay)

ARG ROS_DISTRO=lyrical

# base stage: ros:lyrical + configs
FROM ros:${ROS_DISTRO} AS base

WORKDIR /opt/ros/underlay

ENV ROS_HOME=/tmp
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp

# Compatibility symlink for prebuilt LLVM/Clang toolchain binaries that expect libxml2.so.2
RUN if [ -f /usr/lib/x86_64-linux-gnu/libxml2.so.16 ] && [ ! -f /usr/lib/x86_64-linux-gnu/libxml2.so.2 ]; then \
        ln -s /usr/lib/x86_64-linux-gnu/libxml2.so.16 /usr/lib/x86_64-linux-gnu/libxml2.so.2; \
    fi \
    && if [ -f /usr/lib/aarch64-linux-gnu/libxml2.so.16 ] && [ ! -f /usr/lib/aarch64-linux-gnu/libxml2.so.2 ]; then \
        ln -s /usr/lib/aarch64-linux-gnu/libxml2.so.16 /usr/lib/aarch64-linux-gnu/libxml2.so.2; \
    fi

# underlay stage: base + dependencies built
FROM base AS underlay

ARG SOURCE_DIR=src/sdk-ros
ARG SERVICE_PACKAGE
ADD ${SOURCE_DIR} /opt/ros/underlay/src/sdk-ros

# Clean up unused test packages to prevent rosdep from installing their dependencies.
RUN if [ -d /opt/ros/underlay/src/sdk-ros/intrinsic_sdk_bundle_library_py/test/functional_tests ]; then \
        echo "Cleaning up unused test packages..." \
        && cd /opt/ros/underlay/src/sdk-ros/intrinsic_sdk_bundle_library_py/test/functional_tests \
        && find . -maxdepth 1 -type d ! -name "$SERVICE_PACKAGE" ! -name "." -exec rm -rf {} +; \
    fi


RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && apt-get update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y \
    && apt install -y ros-${ROS_DISTRO}-ament-cmake-vendor-package \
    && colcon build \
        --continue-on-error \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --event-handlers=console_direct+ \
        --merge-install \
        --packages-up-to intrinsic_sdk

# overlay stage: underlay + service code built
FROM underlay AS overlay

ARG SERVICE_PACKAGE
ARG DEPENDENCIES

ARG ROS_DISTRO=lyrical
RUN apt-get update \
    && apt install -y ros-${ROS_DISTRO}-rmw-zenoh-cpp python3-protobuf ${DEPENDENCIES} \
    && rm -rf /var/lib/apt/lists/*

ARG OVERLAY_SOURCE=src
ADD ${OVERLAY_SOURCE} /opt/ros/overlay/src

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && . /opt/ros/underlay/install/setup.sh \
    && apt-get update \
    && rosdep update \
    && cd /opt/ros/overlay \
    && rosdep install --from-paths src --ignore-src -r -y --dependency-types exec \
    && dpkg --get-selections > /service_exec_apt_packages.txt \
    && rosdep install --from-paths src --ignore-src -r -y \
    && colcon build \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
      --event-handlers=console_direct+ \
      --merge-install \
      --packages-up-to=${SERVICE_PACKAGE} \
      --packages-skip intrinsic_sdk grpc_vendor


# result stage: base + copied install folders from the overlay + service setup
FROM base

ARG SERVICE_PACKAGE
ARG SERVICE_NAME
ARG DEPENDENCIES

ARG ROS_DISTRO=lyrical
COPY --from=overlay /service_exec_apt_packages.txt /service_exec_apt_packages.txt
RUN apt-get update \
    && apt-cache dumpavail | dpkg --merge-avail \
    && dpkg --set-selections < /service_exec_apt_packages.txt \
    && apt-get dselect-upgrade -y \
    && apt-get install -y ros-${ROS_DISTRO}-rmw-zenoh-cpp ${DEPENDENCIES} \
    && rm -rf /var/lib/apt/lists/*

COPY --from=overlay /opt/ros/underlay/install /opt/ros/underlay/install
COPY --from=overlay /opt/ros/overlay/install /opt/ros/overlay/install

RUN sed --in-place \
        --expression '$isource "/opt/ros/overlay/install/setup.bash"' \
        /ros_entrypoint.sh \
    && sed --in-place \
        --expression '$iexport RMW_IMPLEMENTATION=rmw_zenoh_cpp' \
        /ros_entrypoint.sh \
    && sed --in-place \
        --expression '$iexport ROS_HOME=/tmp' \
        /ros_entrypoint.sh \
    && sed --in-place \
        --expression '$iexport ZENOH_CONFIG_OVERRIDE='\'connect/endpoints=[\"tcp/zenoh-router.app-intrinsic-base.svc.cluster.local:7447\"]\''' \
        /ros_entrypoint.sh

# Build arguments are not substituted in the CMD command and hence we set environemnt variables
# which can be substituted instead.
ENV SERVICE_PACKAGE=${SERVICE_PACKAGE}
ENV SERVICE_NAME=${SERVICE_NAME}

CMD exec /opt/ros/overlay/install/lib/${SERVICE_PACKAGE}/${SERVICE_NAME}_main
