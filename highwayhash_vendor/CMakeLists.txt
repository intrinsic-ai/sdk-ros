cmake_minimum_required(VERSION 3.10)
project(highwayhash_vendor CXX)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# TODO(wjwwood): highwayhash is required by riegeli (riegeli_vendor). Upstream
# google/highwayhash does not provide a CMakeLists.txt, and there is no rosdep key
# for libhighwayhash-dev. Once a rosdep key exists or SDK C++ targets are extracted
# from Bazel, this vendor package can be retired.

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

set(highwayhash_satisfied_by_system FALSE)
find_package(highwayhash QUIET)
if(highwayhash_FOUND)
  message(STATUS "Found highwayhash, skipping build.")
  set(highwayhash_satisfied_by_system TRUE)
else()
  message(STATUS "highwayhash not found, building from source.")
  set(highwayhash_satisfied_by_system FALSE)
endif()

ament_vendor(highwayhash_vendor
  VCS_URL https://github.com/google/highwayhash.git
  # Matches riegeli MODULE.bazel version: 5ad3bf8444cfc663b11bf367baaa31f36e7ff7c8
  VCS_VERSION 5ad3bf8444cfc663b11bf367baaa31f36e7ff7c8
  PATCHES patches
  CMAKE_ARGS
    -DCMAKE_CXX_STANDARD=${CMAKE_CXX_STANDARD}
    -DCMAKE_INSTALL_LIBDIR=lib
    -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=ON
  SATISFIED ${highwayhash_satisfied_by_system}
)

ament_package()
