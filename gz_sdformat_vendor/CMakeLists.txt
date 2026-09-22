cmake_minimum_required(VERSION 3.10)
project(gz_sdformat_vendor CXX)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

set(gz_sdformat_satisfied_by_system FALSE)
find_package(sdformat QUIET)
if(sdformat_FOUND AND sdformat_VERSION VERSION_GREATER_EQUAL 16.0.0)
  message(STATUS "Found sdformat 16+, skipping build.")
  set(gz_sdformat_satisfied_by_system TRUE)
else()
  message(STATUS "sdformat 16+ not found, building.")
  set(gz_sdformat_satisfied_by_system FALSE)
endif()

if(NOT gz_sdformat_satisfied_by_system)
  find_package(gz_cmake_vendor REQUIRED)
  find_package(gz_math_vendor REQUIRED)
  find_package(gz_utils_vendor REQUIRED)
endif()

ament_vendor(gz_sdformat_vendor
  VCS_URL https://github.com/gazebosim/sdformat
  # Target Intrinsic SDK version requires Gazebo Jetty packages.
  VCS_VERSION sdformat16_16.1.0
  CMAKE_ARGS
    -DSKIP_PYBIND11=ON
    -DBUILD_TESTING=OFF
  SATISFIED ${gz_sdformat_satisfied_by_system}
  GLOBAL_HOOK
)

ament_package()
