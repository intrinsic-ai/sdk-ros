cmake_minimum_required(VERSION 3.10)
project(icon_shared_memory_vendor CXX)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

ament_vendor(icon_shared_memory_vendor
  VCS_TYPE path
  VCS_URL ./icon_no_absl_export
)

ament_package()
