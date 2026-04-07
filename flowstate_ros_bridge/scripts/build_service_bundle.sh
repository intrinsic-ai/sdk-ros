#!/bin/bash
if [ ! -d "src/sdk-ros" ]; then
  echo "This script must be run at the top of a Colcon workspace with sdk-ros."
  exit
fi

ROS_DISTRO="jazzy"

show_help() {
  echo "Usage: $(basename "$0") [OPTIONS]"
  echo ""
  echo "Build and bundle the AIC model container image for Flowstate."
  echo ""
  echo "Options:"
  echo "  -h, --help           Show this help message and exit"
  echo "  --ros_distro ROS_DISTRO  Name of the ROS distro (default: jazzy)"
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
src/sdk-ros/scripts/build_container.sh \
  --service_name flowstate_ros_bridge \
  --service_package flowstate_ros_bridge \
  --dependencies nlohmann-json3-dev \
  --ros_distro "$ROS_DISTRO"
src/sdk-ros/scripts/build_bundle.sh --service_name flowstate_ros_bridge --service_package flowstate_ros_bridge
