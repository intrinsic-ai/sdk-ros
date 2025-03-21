# This Dockerfile provides only the source code for intrinsic_sdk_cmake.
# It needs to be built from the root of the intrinsic_sdk_ros repository.

# base stage: ros:jazzy + configs
FROM ros:jazzy AS base
# TODO(wjwwood): this is based on the ROS image because we still use
#   ament_cmake, we should move away from that and either vendor ament_cmake or
#   avoid it entirely.

WORKDIR /opt/intrinsic

ENV ROS_HOME=/tmp
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN apt-get update \
    && apt-get install -y --no-install-recommends ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# source stage: base + source added
FROM base AS source

ADD ./ /opt/intrinsic/intrinsic_sdk_cmake/src/intrinsic_sdk_ros

# Ensure we're running from the root of the repository so the ADD worked.
RUN cd /opt/intrinsic/intrinsic_sdk_cmake/src/intrinsic_sdk_ros \
    && ls ./intrinsic_sdk_cmake/docker_images/intrinsic_sdk_cmake.Dockerfile \
    || (echo "you must build this dockerfile from the root of the repository" \
    && false)

CMD ["bash"]
