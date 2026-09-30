cmake_minimum_required(VERSION 3.10)
project(flatbuffers_vendor CXX)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

set(flatbuffers_satisfied_by_system FALSE)
find_package(flatbuffers QUIET)
if(flatbuffers_FOUND)
  message(STATUS "Found flatbuffers, skipping build.")
  set(flatbuffers_satisfied_by_system TRUE)
else()
  message(STATUS "flatbuffers not found, building.")
  set(flatbuffers_satisfied_by_system FALSE)
endif()

ament_vendor(flatbuffers_vendor
  VCS_URL https://github.com/google/flatbuffers
  VCS_VERSION v25.12.19
  SATISFIED ${flatbuffers_satisfied_by_system}
)


ament_package()
