cmake_minimum_required(VERSION 3.10)
project(ortools_vendor CXX)

if(NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  message(FATAL_ERROR
  "Clang is required. Detected: ${CMAKE_CXX_COMPILER_ID}. "
  "'sudo apt install clang-19' and invoke colcon with --cmake-args as shown in "
  "https://github.com/intrinsic-ai/sdk-ros?tab=readme-ov-file#getting-started")
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)
find_package(abseil_cpp_vendor REQUIRED)
find_package(re2_vendor REQUIRED)
find_package(grpc_vendor REQUIRED)
ament_add_default_options()

ament_vendor(or-tools
  VCS_URL https://github.com/google/or-tools
  VCS_VERSION 0d60e8afe450ec817f510aae965ab8898310cb41  # 9.14
  CMAKE_ARGS
    -DCMAKE_CXX_STANDARD=20
    -DBUILD_DEPS:BOOL=OFF
    -DUSE_COINOR:BOOL=OFF
    -DUSE_HIGHS:BOOL=OFF
    -DUSE_SCIP:BOOL=OFF
    -DBUILD_SAMPLES:BOOL=OFF
    -DBUILD_EXAMPLES:BOOL=OFF
  PATCHES patches
)

ament_package()
