cmake_minimum_required(VERSION 3.10)
project(snappy_vendor C CXX)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# TODO(wjwwood): riegeli requires snappy >= 1.2.0 (for snappy::CompressionOptions).
# Ubuntu 26.04 (Lyrical) ships libsnappy-dev 1.2.2, which satisfies this check and
# skips building from source. Ubuntu 24.04 (Jazzy) only ships 1.1.10, so snappy 1.2.0
# is built from source there. Once Ubuntu 24.04 (Jazzy) support is dropped, this
# vendor package can be retired and replaced directly with the 'snappy' rosdep key.

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

set(snappy_satisfied_by_system FALSE)
find_package(Snappy 1.2.0 QUIET)
if(Snappy_FOUND)
  message(STATUS "Found Snappy ${Snappy_VERSION} (>= 1.2.0), skipping build.")
  set(snappy_satisfied_by_system TRUE)
else()
  message(STATUS "Snappy >= 1.2.0 not found, building 1.2.0 from source.")
  set(snappy_satisfied_by_system FALSE)
endif()

ament_vendor(snappy_vendor
  VCS_URL https://github.com/google/snappy.git
  # Matches riegeli MODULE.bazel version: 1.2.0
  VCS_VERSION 1.2.0
  PATCHES patches
  CMAKE_ARGS
    -DCMAKE_CXX_STANDARD=${CMAKE_CXX_STANDARD}
    -DCMAKE_INSTALL_LIBDIR=lib
    -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=ON
    -DSNAPPY_BUILD_TESTS:BOOL=OFF
    -DSNAPPY_BUILD_BENCHMARKS:BOOL=OFF
  SATISFIED ${snappy_satisfied_by_system}
)

ament_package()
