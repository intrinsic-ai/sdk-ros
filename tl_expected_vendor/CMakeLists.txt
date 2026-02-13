cmake_minimum_required(VERSION 3.10)
project(tl_expected_vendor CXX)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

ament_vendor(tl_expected_vendor
  VCS_URL https://github.com/TartanLlama/expected
  VCS_VERSION v1.3.1
)

ament_package()
