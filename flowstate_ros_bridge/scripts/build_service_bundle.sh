#!/bin/bash
if [ ! -d "src/sdk-ros" ]; then
  echo "This script must be run at the top of a Colcon workspace with sdk-ros."
  exit
fi

set -o errexit
set -o verbose
src/sdk-ros/scripts/build_container.sh \
  --service_name flowstate_ros_bridge \
  --service_package flowstate_ros_bridge \
  --dependencies nlohmann-json3-dev
src/sdk-ros/scripts/build_bundle.sh \
  --service_name flowstate_ros_bridge \
  --service_package flowstate_ros_bridge \
  --manifest_path src/sdk-ros/flowstate_ros_bridge/flowstate_ros_bridge.manifest.textproto \
  --default_config src/sdk-ros/flowstate_ros_bridge/flowstate_ros_bridge_default_config.pbtxt
