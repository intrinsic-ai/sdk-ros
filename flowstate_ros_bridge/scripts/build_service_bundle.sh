#!/bin/bash
if [ -d "src/sdk-ros" ]; then
  SDK_ROS_DIR="src/sdk-ros"
  SOURCE_ARGS=()
elif [ -f "intrinsic_sdk_cmake/package.xml" ] && [ -d "flowstate_ros_bridge" ]; then
  SDK_ROS_DIR="."
  SOURCE_ARGS=(--source-dir . --overlay-source .)
else
  echo "This script must be run at the top of a Colcon workspace with src/sdk-ros or from the sdk-ros repository root."
  exit 1
fi

ROS_DISTRO="lyrical"

show_help() {
  echo "Usage: $(basename "$0") [OPTIONS]"
  echo ""
  echo "Build and bundle the AIC model container image for Flowstate."
  echo ""
  echo "Options:"
  echo "  -h, --help           Show this help message and exit"
  echo "  --ros_distro ROS_DISTRO  Name of the ROS distro (default: lyrical)"
  echo ""
}

while [[ $# -gt 0 ]]; do
  case $1 in
    -h|--help)
      show_help
      exit 0
      ;;
    --ros_distro)
      ROS_DISTRO="$2"
      shift
      shift
      ;;
    -*|--*)
      echo "Unknown option $1"
      exit 1
      ;;
  esac
done

set -o errexit
set -o verbose
"$SDK_ROS_DIR/scripts/build_container.sh" \
  --ros_distro "$ROS_DISTRO" \
  --service_name flowstate_ros_bridge \
  --service_package flowstate_ros_bridge \
  "${SOURCE_ARGS[@]}"
"$SDK_ROS_DIR/scripts/build_bundle.sh" \
  --service_name flowstate_ros_bridge \
  --service_package flowstate_ros_bridge \
  --manifest_path "$SDK_ROS_DIR/flowstate_ros_bridge/flowstate_ros_bridge.manifest.textproto" \
  --default_config "$SDK_ROS_DIR/flowstate_ros_bridge/flowstate_ros_bridge_default_config.pbtxt"
