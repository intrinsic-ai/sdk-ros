cmake_minimum_required(VERSION 3.10)
project(gz_cmake_vendor)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

set(gz_cmake_satisfied_by_system FALSE)
find_package(gz-cmake4 QUIET)
if(gz-cmake4_FOUND)
  message(STATUS "Found gz-cmake4, skipping build.")
  set(gz_cmake_satisfied_by_system TRUE)
else()
  message(STATUS "gz-cmake4 not found, building.")
  set(gz_cmake_satisfied_by_system FALSE)
endif()

ament_vendor(gz_cmake_vendor
  VCS_URL https://github.com/gazebosim/gz-cmake
  # Target Intrinsic SDK version requires Gazebo Ionic packages.
  VCS_VERSION gz-cmake4_4.2.0
  SATISFIED ${gz_cmake_satisfied_by_system}
  GLOBAL_HOOK
)


ament_package()
