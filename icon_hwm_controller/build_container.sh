#!/bin/bash
set -e

# Ensure we are at the root of the workspace
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WORKSPACE_DIR="$( cd "$SCRIPT_DIR/../../.." &> /dev/null && pwd )"
cd "$WORKSPACE_DIR"

STAGING_DIR="container_staging"

echo "Cleaning up old staging directory..."
rm -rf "$STAGING_DIR"
mkdir -p "$STAGING_DIR"

echo "Copying files to staging..."
PACKAGES=(
    "icon_hwm_controller"
    "ros2_control_demo_example_7"
    "ros2_control_demo_description"
    "realtime_tools"
    "flatbuffers_vendor"
    "control_msgs"
    "controller_manager_msgs"
)

for pkg in "${PACKAGES[@]}"; do
    if [ -d "install/$pkg" ]; then
        echo "Copying $pkg..."
        distrobox-enter -n ubuntu24 -- cp -rL "install/$pkg" "$STAGING_DIR/"
    else
        echo "Warning: install/$pkg not found!"
    fi
done

echo "Copying setup files..."
distrobox-enter -n ubuntu24 -- cp -L install/local_setup.* "$STAGING_DIR/"
distrobox-enter -n ubuntu24 -- cp -L install/setup.* "$STAGING_DIR/"
distrobox-enter -n ubuntu24 -- cp -L install/_local_setup_util_*.py "$STAGING_DIR/"

echo "Building docker container..."
docker build -t icon_hwm -f src/sdk-ros/icon_hwm_controller/Dockerfile "$STAGING_DIR"

echo "Cleaning up staging dir"
rm -rf "$STAGING_DIR"
echo "Done!"
