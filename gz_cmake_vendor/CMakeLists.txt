cmake_minimum_required(VERSION 3.10)
project(gz_cmake_vendor)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

find_package(gz-cmake4 COMPONENTS all QUIET)

ament_vendor(gz_cmake_vendor
  SATISFIED ${gz-cmake4_FOUND}
  VCS_URL https://github.com/gazebosim/gz-cmake
  # Target Intrinsic SDK version requires Gazebo Ionic packages.
  VCS_VERSION gz-cmake4_4.2.0
)

ament_package()
