cmake_minimum_required(VERSION 3.10)
project(gz_math_vendor CXX)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)
find_package(eigen_vendor REQUIRED)
find_package(gz_cmake_vendor REQUIRED)
find_package(gz_utils_vendor REQUIRED)

find_package(Eigen3 REQUIRED)

ament_vendor(gz_math_vendor
  VCS_URL https://github.com/gazebosim/gz-math
  # Target Intrinsic SDK version requires Gazebo Ionic packages.
  VCS_VERSION gz-math8_8.2.0
)

ament_package()
