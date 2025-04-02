cmake_minimum_required(VERSION 3.20)
project(intrinsic_sdk_ros LANGUAGES CXX)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)

# Find the cmake wrapper around the Intrinsic SDK.
find_package(intrinsic_sdk_cmake REQUIRED)

ament_export_dependencies(intrinsic_sdk_cmake)

ament_package()
