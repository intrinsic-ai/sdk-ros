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
  # TODO(wjwwood): in our bazel builds we use a custom version which is
  #   unreleased by upstream, however 3.4.0 seems to work as well, so we're
  #   using that for now...
  # VCS_VERSION 4.0.0-20241125
  VCS_VERSION 3.4.0
)

ament_package()
