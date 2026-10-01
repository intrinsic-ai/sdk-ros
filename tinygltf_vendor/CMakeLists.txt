cmake_minimum_required(VERSION 3.10)
project(tinygltf_vendor CXX)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# TODO(wjwwood): tinygltf is required by the Intrinsic SDK (intrinsic/math/Create_gltf,
# gltf_io.cc, etc.). Ubuntu provides libtinygltf-dev in apt, but there is currently no
# upstream rosdep key for it (and Ubuntu 24.04 ships 2.8.x while the SDK pins v2.9.6
# and expects tinygltf's bundled stb_image.h / json.hpp headers). Once a rosdep key
# is available and compatible across supported distros (or SDK C++ targets are extracted
# from Bazel), this vendor package can be retired.

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

set(tinygltf_satisfied_by_system FALSE)
find_package(TinyGLTF 2.9.6 QUIET)
if(TinyGLTF_FOUND)
  message(STATUS "Found TinyGLTF >= 2.9.6, skipping build.")
  set(tinygltf_satisfied_by_system TRUE)
else()
  message(STATUS "TinyGLTF >= 2.9.6 not found, building v2.9.6 from source.")
  set(tinygltf_satisfied_by_system FALSE)
endif()

ament_vendor(tinygltf_vendor
  VCS_URL https://github.com/syoyo/tinygltf.git
  # Matches Intrinsic SDK MODULE.bazel version: v2.9.6
  VCS_VERSION v2.9.6
  CMAKE_ARGS
    -DCMAKE_CXX_STANDARD=${CMAKE_CXX_STANDARD}
    -DCMAKE_INSTALL_LIBDIR=lib
    -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=ON
    -DTINYGLTF_BUILD_LOADER_EXAMPLE:BOOL=OFF
    -DTINYGLTF_HEADER_ONLY:BOOL=OFF
    -DTINYGLTF_INSTALL:BOOL=ON
    -DTINYGLTF_INSTALL_VENDOR:BOOL=ON
  SATISFIED ${tinygltf_satisfied_by_system}
)

ament_package()
