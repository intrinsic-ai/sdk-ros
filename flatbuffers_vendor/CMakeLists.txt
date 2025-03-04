cmake_minimum_required(VERSION 3.10)
project(flatbuffers_vendor CXX)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

ament_vendor(flatbuffers_vendor
  VCS_URL https://github.com/google/flatbuffers
  VCS_VERSION v25.2.10
  # CMAKE_ARGS
  #   -DCMAKE_CXX_STANDARD=20
)

ament_package()
