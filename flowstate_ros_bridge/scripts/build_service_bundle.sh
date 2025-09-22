#!/bin/bash
if [ ! -d "src/sdk-ros" ]; then
  echo "This script must be run at the top of a Colcon workspace with sdk-ros."
  exit
fi

set -o errexit
set -o verbose
src/sdk-ros/scripts/build_container.sh --service_name flowstate_ros_bridge --service_package flowstate_ros_bridge --dockerfile src/sdk-ros/flowstate_ros_bridge/Dockerfile.service
src/sdk-ros/scripts/build_bundle.sh --service_name flowstate_ros_bridge --service_package flowstate_ros_bridge
