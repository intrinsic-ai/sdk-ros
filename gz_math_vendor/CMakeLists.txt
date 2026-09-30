cmake_minimum_required(VERSION 3.10)
project(gz_math_vendor CXX)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

set(gz_math_satisfied_by_system FALSE)
find_package(gz-math QUIET)
if(gz-math_FOUND AND gz-math_VERSION VERSION_GREATER_EQUAL 9.0.0)
  message(STATUS "Found gz-math 9+, skipping build.")
  set(gz_math_satisfied_by_system TRUE)
else()
  message(STATUS "gz-math 9+ not found, building.")
  set(gz_math_satisfied_by_system FALSE)
endif()

if(NOT gz_math_satisfied_by_system)
  find_package(eigen_vendor REQUIRED)
  find_package(gz_cmake_vendor REQUIRED)
  find_package(gz_utils_vendor REQUIRED)
  find_package(Eigen3 REQUIRED)
endif()

ament_vendor(gz_math_vendor
  VCS_URL https://github.com/gazebosim/gz-math
  # Target Intrinsic SDK version requires Gazebo Jetty packages.
  VCS_VERSION gz-math9_9.0.0
  SATISFIED ${gz_math_satisfied_by_system}
  GLOBAL_HOOK
)


ament_package()
