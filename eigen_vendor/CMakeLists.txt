cmake_minimum_required(VERSION 3.10)
project(eigen_vendor CXX)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

ament_vendor(eigen_vendor
  VCS_URL https://gitlab.com/libeigen/eigen
  # VCS_VERSION 4.0.0-20241125
  VCS_VERSION 3.4.0  # TODO(wjwwood) check where the version 4.0.0-20241125 comes from
  # CMAKE_ARGS
  #   -DCMAKE_CXX_STANDARD=20
)

ament_package()
