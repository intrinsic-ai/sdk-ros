cmake_minimum_required(VERSION 3.10)
project(gz_cmake_vendor)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

set(gz_cmake_satisfied_by_system FALSE)
find_package(gz-cmake5 QUIET)
if(gz-cmake5_FOUND)
  message(STATUS "Found gz-cmake5, skipping build.")
  set(gz_cmake_satisfied_by_system TRUE)
else()
  message(STATUS "gz-cmake5 not found, building.")
  set(gz_cmake_satisfied_by_system FALSE)
endif()

ament_vendor(gz_cmake_vendor
  VCS_URL https://github.com/gazebosim/gz-cmake
  # Target Intrinsic SDK version requires Gazebo Jetty packages.
  VCS_VERSION gz-cmake5_5.0.1
  SATISFIED ${gz_cmake_satisfied_by_system}
  GLOBAL_HOOK
)


ament_package()
