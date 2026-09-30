cmake_minimum_required(VERSION 3.10)
project(gz_utils_vendor CXX)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

set(gz_utils_satisfied_by_system FALSE)
find_package(gz-utils QUIET)
if(gz-utils_FOUND AND gz-utils_VERSION VERSION_GREATER_EQUAL 4.0.0)
  message(STATUS "Found gz-utils 4+, skipping build.")
  set(gz_utils_satisfied_by_system TRUE)
else()
  message(STATUS "gz-utils 4+ not found, building.")
  set(gz_utils_satisfied_by_system FALSE)
endif()

if(NOT gz_utils_satisfied_by_system)
  find_package(gz_cmake_vendor REQUIRED)
endif()

ament_vendor(gz_utils_vendor
  VCS_URL https://github.com/gazebosim/gz-utils
  # Target Intrinsic SDK version requires Gazebo Jetty packages.
  VCS_VERSION gz-utils4_4.0.0
  CMAKE_ARGS
    -DGZ_UTILS_VENDOR_CLI11=ON
  SATISFIED ${gz_utils_satisfied_by_system}
  GLOBAL_HOOK
)

ament_package()
