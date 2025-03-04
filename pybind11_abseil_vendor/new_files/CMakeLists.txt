cmake_minimum_required(VERSION 3.11)
project(pybind11_abseil LANGUAGES CXX)

include(CMakePackageConfigHelpers)
include(CTest)
include(FetchContent)
include(GNUInstallDirs)

if(NOT DEFINED CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif(NOT DEFINED CMAKE_CXX_STANDARD)
set(ABSL_PROPAGATE_CXX_STD ON)
set(BUILD_TESTING OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE TRUE)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION FALSE)

if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.24)
  cmake_policy(SET CMP0135 NEW)
endif()

# FetchContent_Declare(
#   abseil-cpp
#   URL https://github.com/abseil/abseil-cpp/archive/refs/tags/20230802.0.tar.gz
#   URL_HASH
#     SHA256=59d2976af9d6ecf001a81a35749a6e551a335b949d34918cfade07737b9d93c5)

# FetchContent_Declare(
#   pybind11
#   URL https://github.com/pybind/pybind11/archive/refs/heads/master.tar.gz)

# FetchContent_MakeAvailable(pybind11)
# FetchContent_MakeAvailable(abseil-cpp pybind11)

find_package(absl REQUIRED)
find_package(pybind11 REQUIRED)

set(TOP_LEVEL_DIR ${CMAKE_CURRENT_LIST_DIR})
include_directories(${TOP_LEVEL_DIR} ${pybind11_INCLUDE_DIRS})

add_subdirectory(pybind11_abseil)

target_include_directories(absl_casters INTERFACE
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

# message(FATAL_ERROR "pybind11_INCLUDE_DIRS: ${pybind11_INCLUDE_DIRS}")
# target_include_directories(absl_casters INTERFACE $<BUILD_INTERFACE:${pybind11_INCLUDE_DIRS}>)

configure_package_config_file("${pybind11_abseil_vendor_NEW_FILES_DIR}/Config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/pybind11_abseilConfig.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/pybind11_abseil
)

install(
  TARGETS absl_casters
  EXPORT pybind11_abseil-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  EXPORT pybind11_abseil-targets
  FILE pybind11_abseilTargets.cmake
  NAMESPACE pybind11_abseil::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/pybind11_abseil
)

install(
  DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/"
  DESTINATION "include/${PROJECT_NAME}"
  FILES_MATCHING
  PATTERN "*.h"
)

install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/pybind11_abseilConfig.cmake"
    # "${CMAKE_CURRENT_BINARY_DIR}/pybind11_abseilConfigVersion.cmake"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/pybind11_abseil
)
