cmake_minimum_required(VERSION 3.10)
project(gz_utils_vendor CXX)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)
find_package(gz_cmake_vendor REQUIRED)

ament_vendor(gz_utils_vendor
  VCS_URL https://github.com/gazebosim/gz-utils
  # Target Intrinsic SDK version requires Gazebo Jetty packages.
  VCS_VERSION gz-utils4_4.0.0
  CMAKE_ARGS
    -DGZ_UTILS_VENDOR_CLI11=ON
  GLOBAL_HOOK
)

ament_package()
