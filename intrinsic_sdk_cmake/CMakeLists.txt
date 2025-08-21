cmake_minimum_required(VERSION 3.19)
project(intrinsic_sdk_cmake CXX)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Fetch the sdk, intrinsic_sdk_SOURCE_DIR is set after this.
include(cmake/fetch_sdk.cmake)

# Find dependencies.
include(cmake/dependencies.cmake)

# Bring in additional googleapis necessary to build Intrinsic proto files.
# Note that googleapis_SOURCE_DIR and grpc_gateway_SOURCE_DIR come from here.
include(cmake/fetch_googleapis.cmake)

# Build protos into a library.
include(cmake/sdk_protos.cmake)

# Build flatbuffers.
include(cmake/sdk_flatbuffers.cmake)

# Find and install the imw_zenoh shared-library and config file.
# Note that the cmake variables used in compiler definitions in sdk.cmake, like
# intrinsic_sdk_IMW_ZENOH_LIBRARY_RELPATH and others, are set in this file.
include(cmake/sdk_imw_zenoh.cmake)

# Build the sdk C++ sources into a library.
include(cmake/sdk.cmake)

# Install cmake helper functions.
install(
  DIRECTORY cmake/api
  DESTINATION share/${PROJECT_NAME}/cmake
)
install(
  FILES cmake/dependencies.cmake
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

# Install the essential ament_index package resources and the package.xml.
install(CODE "
  file(MAKE_DIRECTORY ${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages)
  file(TOUCH ${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages/${PROJECT_NAME})
")
install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Tests
if(BUILD_TESTING)
  # Imitate find_package(intrinsic_sdk_cmake) for testing
  set(intrinsic_sdk_cmake_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
  set(intrinsic_sdk_cmake_API_DIR "${intrinsic_sdk_cmake_DIR}/cmake/api")
  set(intrinsic_sdk_cmake_DESCRIPTOR_SET_FILE "${CMAKE_CURRENT_BINARY_DIR}/intrinsic_proto.desc")
  include(cmake/api/all.cmake)

  add_subdirectory(test)
endif()
