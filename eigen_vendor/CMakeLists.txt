cmake_minimum_required(VERSION 3.10)
project(eigen_vendor CXX)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

set(eigen_satisfied_by_system FALSE)
find_package(Eigen3 QUIET)
if(Eigen3_FOUND)
  message(STATUS "Found Eigen3, skipping build.")
  set(eigen_satisfied_by_system TRUE)
else()
  message(STATUS "Eigen3 not found, building.")
  set(eigen_satisfied_by_system FALSE)
endif()

ament_vendor(eigen_vendor
  VCS_URL https://gitlab.com/libeigen/eigen
  VCS_VERSION 5.0.1
  SATISFIED ${eigen_satisfied_by_system}
)


ament_package()
