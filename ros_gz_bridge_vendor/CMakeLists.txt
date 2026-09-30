cmake_minimum_required(VERSION 3.8)
project(ros_gz_bridge_vendor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

find_package(grpc_vendor QUIET)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz_transport_vendor REQUIRED)
set(deps_vendored FALSE)
if(grpc_vendor_FOUND AND EXISTS "${grpc_vendor_DIR}/../../../opt/grpc_vendor")
  list(PREPEND CMAKE_PREFIX_PATH "${grpc_vendor_DIR}/../../../opt/grpc_vendor")
  set(deps_vendored TRUE)
endif()
if(EXISTS "${gz_msgs_vendor_DIR}/../../../opt/gz_msgs_vendor")
  list(PREPEND CMAKE_PREFIX_PATH "${gz_msgs_vendor_DIR}/../../../opt/gz_msgs_vendor")
  set(deps_vendored TRUE)
endif()
if(EXISTS "${gz_transport_vendor_DIR}/../../../opt/gz_transport_vendor")
  list(PREPEND CMAKE_PREFIX_PATH "${gz_transport_vendor_DIR}/../../../opt/gz_transport_vendor")
  set(deps_vendored TRUE)
endif()

set(ros_gz_bridge_satisfied_by_system FALSE)
if(NOT deps_vendored)
  find_package(ros_gz_bridge QUIET)
endif()
if(ros_gz_bridge_FOUND)
  message(STATUS "Found system ros_gz_bridge, skipping build.")
  set(ros_gz_bridge_satisfied_by_system TRUE)
endif()

ament_vendor(ros_gz_bridge_vendor
  VCS_URL https://github.com/gazebosim/ros_gz/
  VCS_VERSION 6247cb07c34d6e68e849fc97b82fca8a3a096d21 # Main at 10/02/2026, later requires a message that is not released as a binary yet
  SOURCE_SUBDIR ros_gz_bridge
  SATISFIED ${ros_gz_bridge_satisfied_by_system}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
