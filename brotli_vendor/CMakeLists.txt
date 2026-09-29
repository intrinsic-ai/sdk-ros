cmake_minimum_required(VERSION 3.10)
project(brotli_vendor C CXX)

# TODO(wjwwood): brotli is required by riegeli (riegeli_vendor), which in turn is
# required by the Intrinsic SDK. Ubuntu provides libbrotli-dev in apt, but there is
# currently no upstream rosdep key for libbrotli-dev. Once a rosdep key exists (or
# once SDK C++ targets are extracted from Bazel), this vendor package can be retired.

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)
find_package(PkgConfig REQUIRED)

set(brotli_satisfied_by_system FALSE)
pkg_check_modules(BROTLI QUIET libbrotlicommon libbrotlidec libbrotlienc)
if(BROTLI_FOUND)
  message(STATUS "Found system brotli, skipping build.")
  set(brotli_satisfied_by_system TRUE)
else()
  message(STATUS "brotli not found, building from source.")
  set(brotli_satisfied_by_system FALSE)
endif()

ament_vendor(brotli_vendor
  VCS_URL https://github.com/google/brotli.git
  # Matches riegeli MODULE.bazel version: 1.1.0
  VCS_VERSION v1.1.0
  CMAKE_ARGS
    -DBROTLI_DISABLE_TESTS:BOOL=ON
    -DCMAKE_INSTALL_LIBDIR=lib
    -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=ON
  SATISFIED ${brotli_satisfied_by_system}
)

ament_package(
  CONFIG_EXTRAS brotli_vendor-extras.cmake.in
)
