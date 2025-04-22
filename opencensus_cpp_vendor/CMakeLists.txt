cmake_minimum_required(VERSION 3.10)
project(opencensus_cpp_vendor CXX)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)
find_package(abseil_cpp_vendor REQUIRED)
find_package(prometheus_cpp_vendor REQUIRED)

ament_vendor(opencensus_cpp_vendor
  VCS_URL https://github.com/census-instrumentation/opencensus-cpp
  # Last commit on main branch before it was archived
  VCS_VERSION 50eb5de762e5f87e206c011a4f930adb1a1775b1
  PATCHES patches
  CMAKE_ARGS
    -DCMAKE_CXX_STANDARD=20
    -DOpenCensus_BUILD_TESTING:BOOL=OFF
)

ament_package()