cmake_minimum_required(VERSION 3.8)
project(ros_gz_bridge_vendor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

set(ros_gz_bridge_satisfied_by_system FALSE)
find_package(ros_gz_bridge QUIET)
if(ros_gz_bridge_FOUND)
  message(STATUS "Found system ros_gz_bridge, skipping build.")
  set(ros_gz_bridge_satisfied_by_system TRUE)
endif()

ament_vendor(ros_gz_bridge_vendor
  VCS_URL https://github.com/gazebosim/ros_gz/
  VCS_VERSION 6247cb07c34d6e68e849fc97b82fca8a3a096d21 # Main at 10/02/2026, later requires a message that is not released as a binary yet
  SOURCE_SUBDIR ros_gz_bridge
  GLOBAL_HOOK
  SATISFIED ${ros_gz_bridge_satisfied_by_system}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
