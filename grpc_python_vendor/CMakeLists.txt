cmake_minimum_required(VERSION 3.10)
project(grpc_python_vendor CXX)

find_package(ament_cmake REQUIRED)
find_package(Python REQUIRED)

# Fetch grpcio and protobuf packages from pypi
set(grpcio_version "1.74.0")
set(protobuf_version "6.31.1")

message(STATUS "Downloading grpcio and protobuf wheels to ${CMAKE_CURRENT_BINARY_DIR}/wheels...")
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/wheels)
execute_process(
  COMMAND ${Python_EXECUTABLE} -m pip download
    grpcio==${grpcio_version}
    protobuf==${protobuf_version}
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/wheels
  RESULT_VARIABLE pip_download_result
)

if(NOT pip_download_result EQUAL 0)
  message(FATAL_ERROR "Failed to download grpcio and protobuf wheels")
endif()

configure_file(
  cmake/install_wheels.cmake.in 
  ${CMAKE_CURRENT_BINARY_DIR}/install_wheels.cmake 
  @ONLY
)

install(SCRIPT ${CMAKE_CURRENT_BINARY_DIR}/install_wheels.cmake)

ament_environment_hooks(
  hooks/pythonpath.dsv.in
  hooks/pythonpath.sh.in
)

ament_package()
