cmake_minimum_required(VERSION 3.10)
project(abseil_cpp_vendor CXX)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

ament_vendor(abseil_cpp_vendor
  VCS_URL https://github.com/abseil/abseil-cpp
  VCS_VERSION 20240722.0
  CMAKE_ARGS
    -DCMAKE_CXX_STANDARD=${CMAKE_CXX_STANDARD}
    -DABSL_BUILD_TESTING:BOOL=OFF
    -DABSL_ENABLE_INSTALL:BOOL=ON
)

ament_package()
