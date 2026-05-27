cmake_minimum_required(VERSION 3.10)
project(gz_msgs_vendor CXX)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

set(gz_msgs_satisfied_by_system FALSE)
find_package(gz-msgs QUIET)
if(gz-msgs_FOUND AND gz-msgs_VERSION VERSION_GREATER_EQUAL 12.0.0)
  message(STATUS "Found gz-msgs 12+, skipping build.")
  set(gz_msgs_satisfied_by_system TRUE)
else()
  message(STATUS "gz-msgs 12+ not found, building.")
  set(gz_msgs_satisfied_by_system FALSE)
endif()

if(NOT gz_msgs_satisfied_by_system)
  find_package(grpc_vendor REQUIRED)
  find_package(gz_cmake_vendor REQUIRED)
  find_package(gz_math_vendor REQUIRED)
  find_package(gz_utils_vendor REQUIRED)
endif()

ament_vendor(gz_msgs_vendor
  VCS_URL https://github.com/gazebosim/gz-msgs
  # Target Intrinsic SDK version requires Gazebo Jetty packages.
  VCS_VERSION gz-msgs12_12.0.1
  SATISFIED ${gz_msgs_satisfied_by_system}
  GLOBAL_HOOK
  PATCHES patches
)


ament_package()
