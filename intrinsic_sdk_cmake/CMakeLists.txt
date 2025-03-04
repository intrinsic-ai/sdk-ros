cmake_minimum_required(VERSION 3.10)
project(intrinsic_sdk_cmake CXX)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Fetch the sdk, intrinsic_sdk_SOURCE_DIR is set after this.
include(cmake/fetch_sdk.cmake)

# Find vendor packages to ensure dependencies are available.
find_package(bazelisk_vendor REQUIRED)
find_package(eigen_vendor REQUIRED)
find_package(flatbuffers_vendor REQUIRED)
find_package(grpc_vendor REQUIRED)  # Provides protobuf and abseilcpp
find_package(pybind11_vendor REQUIRED)
find_package(pybind11_abseil_vendor REQUIRED)
find_package(ortools_vendor REQUIRED)

# Find project dependencies.
find_package(Eigen3 REQUIRED)
find_package(flatbuffers REQUIRED)
find_package(Protobuf CONFIG REQUIRED)
find_package(gRPC CONFIG REQUIRED)
set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 CONFIG REQUIRED)
find_package(pybind11_abseil CONFIG REQUIRED)
find_package(ortools CONFIG REQUIRED)

# Set important bazel related variables like intrinsic_sdk_BAZEL_BIN_DIR.
include(cmake/bazel_setup.cmake)

# Bring in additional googleapis necessary to build Intrinsic proto files.
# Note that googleapis_SOURCE_DIR and grpc_gateway_SOURCE_DIR come from here.
include(cmake/fetch_googleapis.cmake)

# Build protos into a library.
include(cmake/sdk_protos.cmake)

# Build flatbuffers.
include(cmake/sdk_flatbuffers.cmake)

# Build the sdk C++ sources into a library.
include(cmake/sdk.cmake)

# Extract tool binaries from bazel build of sdk.
include(cmake/sdk_tools.cmake)

# Install cmake helper functions.
install(
  DIRECTORY cmake/api
  DESTINATION share/${PROJECT_NAME}/cmake
)

# Generate and install the export file.
install(
  EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

# Generate and install the package configuration files.
include(CMakePackageConfigHelpers)
configure_package_config_file(
  ${PROJECT_NAME}Config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION share/${PROJECT_NAME}/cmake
)
install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  DESTINATION
    share/${PROJECT_NAME}/cmake
)

# Tests
if(BUILD_TESTING)
  # Imitate find_package(intrinsic_sdk_cmake) for testing
  set(intrinsic_sdk_cmake_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
  set(intrinsic_sdk_cmake_DESCRIPTOR_SET_FILE "${CMAKE_CURRENT_BINARY_DIR}/intrinsic_proto.desc")
  include(cmake/api/all.cmake)

  add_subdirectory(test)
endif()
