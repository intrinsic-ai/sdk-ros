cmake_minimum_required(VERSION 3.10)
project(pybind11_abseil_vendor CXX)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

find_package(grpc_vendor REQUIRED)  # needed for abseilcpp
find_package(pybind11_vendor REQUIRED)  # needed for abseilcpp

execute_process(
  COMMAND python3 generate_patches.py
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/new_files"
)
set_property(
  DIRECTORY
  APPEND
  PROPERTY CMAKE_CONFIGURE_DEPENDS
  new_files/CMakeLists.txt
  new_files/generate_patches.py
  new_files/pybind11_abseilConfig.cmake
  new_files/Config.cmake.in
)
set(pybind11_abseil_vendor_NEW_FILES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/new_files")

ament_vendor(pybind11_abseil_vendor
  VCS_URL https://github.com/pybind/pybind11_abseil.git
  VCS_VERSION v202402.0
  CMAKE_ARGS
    -DCMAKE_CXX_STANDARD=20
    -Dpybind11_abseil_vendor_NEW_FILES_DIR=${pybind11_abseil_vendor_NEW_FILES_DIR}
  PATCHES patches
)

ament_package()
