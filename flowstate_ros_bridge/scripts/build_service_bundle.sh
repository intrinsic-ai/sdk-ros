#!/bin/bash
if [ ! -d "src/sdk-ros" ]; then
  echo "This script must be run at the top of a Colcon workspace with sdk-ros."
  exit
fi

# Reads first argument as ROS_DISTRO, default to jazzy
ROS_DISTRO=${1:-"jazzy"}

set -o errexit
set -o verbose
src/sdk-ros/scripts/build_container.sh \
  --service_name flowstate_ros_bridge \
  --service_package flowstate_ros_bridge \
  --dependencies nlohmann-json3-dev \
  --ros_distro "$ROS_DISTRO"
src/sdk-ros/scripts/build_bundle.sh --service_name flowstate_ros_bridge --service_package flowstate_ros_bridge
