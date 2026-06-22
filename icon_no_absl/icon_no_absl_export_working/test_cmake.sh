#!/bin/bash
# Script to test CMake configuration for incode/icon/no_absl

set -euo pipefail

# 1. Create a temp directory
TMP_DIR=$(mktemp -d -t cmake_test_XXXXXX)
echo "Created temp directory: ${TMP_DIR}"

# 2. Copy the no_absl subtree keeping layout relative to repo root
mkdir -p "${TMP_DIR}/incode/icon"
cp -r "/usr/local/google/home/nilsb/insrc/incode/icon/no_absl" "${TMP_DIR}/incode/icon/"
echo "Copied source files to ${TMP_DIR}/incode/icon/no_absl"

# 3. Create CMake build folder
BUILD_DIR="${TMP_DIR}/build"
mkdir -p "${BUILD_DIR}"
echo "Created build directory: ${BUILD_DIR}"

# 4. Enter ubuntu24 distrobox and run cmake configuration
echo "Configuring CMake inside ubuntu24 distrobox..."
distrobox-enter -n ubuntu24 -- bash -c "
  source /opt/ros/jazzy/setup.bash
  cd \"${BUILD_DIR}\"
  cmake \"${TMP_DIR}/incode/icon/no_absl\"
"

echo "CMake configuration finished successfully inside distrobox."
echo "Temp build artifacts are located in: ${TMP_DIR}"
