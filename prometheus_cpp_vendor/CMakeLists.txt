cmake_minimum_required(VERSION 3.10)
project(prometheus_cpp_vendor CXX)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

ament_vendor(prometheus_cpp_vendor
  VCS_URL https://github.com/jupp0r/prometheus-cpp
  # Based on:
  #   https://github.com/census-instrumentation/opencensus-cpp/blob/\
  #   50eb5de762e5f87e206c011a4f930adb1a1775b1/cmake/OpenCensusDeps.cmake#L28C11-L28C17
  VCS_VERSION v1.1.0
  CMAKE_ARGS
    -DCMAKE_CXX_STANDARD=20
    # Standard arguments from the readme:
    #   https://github.com/jupp0r/prometheus-cpp/blob/v1.1.0/README.md#via-cmake
    -DBUILD_SHARED_LIBS=ON
    -DENABLE_PUSH=OFF
    -DENABLE_COMPRESSION=OFF
)

ament_package()