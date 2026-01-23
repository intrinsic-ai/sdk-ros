# This Dockerfile provides only the source code for intrinsic_sdk_cmake.
# It needs to be built from the root of the intrinsic_sdk_ros repository.

# base stage: ghcr.io/sloretz/ros:jazzy-ros-core + configs + rmw_zenoh
FROM ghcr.io/sloretz/ros:jazzy-ros-core-2025-06-08 AS base
# TODO(wjwwood): this is based on the ROS image because we still use
#   ament_cmake, we should move away from that and either vendor ament_cmake or
#   avoid it entirely.
# Note: ghcr.io/sloretz/ros:jazzy-ros-core is appropriate here as it does not
#   have any build tools and though it has some ros packages installed, it is
#   unlikely that any of them would not be require for even the most trivial
#   ROS applications.
# Note: We are using the ghcr.io/sloretz/ros OCI ROS images since they are more
#   granular and stay up-to-date more often than the "official" ROS docker
#   images like ros:jazzy, for example.

WORKDIR /opt/intrinsic

ENV ROS_HOME=/tmp

# Install rmw_zenoh_cpp and set it as the default RMW implementation.
RUN \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    set -x \
    && rm -f /etc/apt/apt.conf.d/docker-clean \
    && echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' >/etc/apt/apt.conf.d/keep-cache \
    && apt-get update \
    && apt-get dist-upgrade -y \
    && apt-get install -y --no-install-recommends ros-jazzy-rmw-zenoh-cpp
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
RUN set -x \
    && sed --in-place \
        --expression '$iexport RMW_IMPLEMENTATION=rmw_zenoh_cpp' \
        /ros_entrypoint.sh \
    && sed --in-place \
        --expression '$iexport ENV ROS_HOME=/tmp' \
        /ros_entrypoint.sh \
    && sed --in-place \
        --expression '$iexport ZENOH_CONFIG_OVERRIDE='\'connect/endpoints=[\"tcp/zenoh-router.app-intrinsic-base.svc.cluster.local:7447\"]\''' \
        /ros_entrypoint.sh

CMD ["bash"]
