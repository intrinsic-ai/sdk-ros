#!/bin/bash
if [ ! -d "src/sdk-ros" ]; then
  echo "This script must be run at the top of a Colcon workspace with sdk-ros."
  exit
fi

IMAGES_DIR=./images
BUILDER_NAME=container-builder

usage() {
  echo "Usage: build_service_bundle.sh [options]"
  echo ""
  echo "Options:"
  echo "  --images_dir <dir>    Path to the images directory (default: ./images)"
  echo "  --builder_name <name> Name of the builder (default: container-builder)"
  echo "  --help                Show this help message"
  exit 0
}

# Parse arguments
if ! OPTS=$(getopt -o "" --longoptions images_dir:,builder_name:,help -n 'build_service_bundle.sh' -- "$@"); then
  echo "Failed to parse options... exiting." >&2
  exit 1
fi
eval set -- "$OPTS"

while true; do
  case "$1" in
    --images_dir) IMAGES_DIR="$2"; shift 2 ;;
    --builder_name) BUILDER_NAME="$2"; shift 2 ;;
    --help) usage ;;
    --) shift; break ;;
    *) break ;;
  esac
done

set -o errexit
set -o verbose

src/sdk-ros/scripts/build_container.sh \
  --service_name flowstate_ros_bridge \
  --service_package flowstate_ros_bridge \
  --dependencies nlohmann-json3-dev \
  --builder_name "$BUILDER_NAME" \
  --images_dir "$IMAGES_DIR"
src/sdk-ros/scripts/build_bundle.sh --service_name flowstate_ros_bridge --service_package flowstate_ros_bridge \
  --builder_name "$BUILDER_NAME" \
  --images_dir "$IMAGES_DIR"
