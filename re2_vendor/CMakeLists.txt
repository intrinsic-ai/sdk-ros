cmake_minimum_required(VERSION 3.10)
project(re2_vendor CXX)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)
find_package(abseil_cpp_vendor REQUIRED)

ament_vendor(re2_vendor
  VCS_URL https://github.com/google/re2
  VCS_VERSION 2024-07-02
  PATCHES patches
  CMAKE_ARGS
    -DCMAKE_CXX_STANDARD=20
    -DRE2_BUILD_TESTING:BOOL=OFF
)

ament_package()
