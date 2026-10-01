cmake_minimum_required(VERSION 3.10)
project(riegeli_vendor CXX)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# TODO(wjwwood): riegeli (and its leaf vendor packages brotli_vendor, snappy_vendor,
# and highwayhash_vendor) is needed by the Intrinsic SDK C++ sources. Upstream
# google/riegeli only ships a Bazel build and is not available via rosdep. Once
# riegeli is available as a system/rosdep package or SDK C++ targets are extracted
# from Bazel, this vendor package can be retired.

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

set(riegeli_satisfied_by_system FALSE)
find_package(riegeli QUIET)
if(riegeli_FOUND)
  message(STATUS "Found riegeli, skipping build.")
  set(riegeli_satisfied_by_system TRUE)
else()
  message(STATUS "riegeli not found, building from source.")
  set(riegeli_satisfied_by_system FALSE)
endif()

if(NOT riegeli_satisfied_by_system)
  find_package(abseil_cpp_vendor REQUIRED)
  find_package(brotli_vendor REQUIRED)
  find_package(grpc_vendor REQUIRED)
  find_package(highwayhash_vendor REQUIRED)
  find_package(snappy_vendor REQUIRED)
endif()

ament_vendor(riegeli_vendor
  VCS_URL https://github.com/google/riegeli.git
  # Matches Intrinsic SDK MODULE.bazel version: 0.0.0-20250822-9f2744d
  VCS_VERSION 9f2744dc23e81d84c02f6f51244e9e9bb9802d57
  PATCHES patches
  CMAKE_ARGS
    -DCMAKE_CXX_STANDARD=${CMAKE_CXX_STANDARD}
    -DCMAKE_INSTALL_LIBDIR=lib
    -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=ON
  SATISFIED ${riegeli_satisfied_by_system}
)

ament_export_dependencies(
  abseil_cpp_vendor
  brotli_vendor
  grpc_vendor
  highwayhash_vendor
  snappy_vendor
)

ament_package()
