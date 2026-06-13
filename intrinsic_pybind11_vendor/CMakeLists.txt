cmake_minimum_required(VERSION 3.10)
project(intrinsic_pybind11_vendor CXX)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

set(_pybind11_ver "3.0.0")
set(pybind11_satisfied_by_system FALSE)
find_package(pybind11 CONFIG QUIET)
if(pybind11_FOUND)
  if(pybind11_VERSION VERSION_GREATER_EQUAL _pybind11_ver)
    message(STATUS "Found pybind11 '${pybind11_VERSION}', needed '${_pybind11_ver}', skipping.")
    set(pybind11_satisfied_by_system TRUE)
  else()
    message(STATUS "Found pybind11 '${pybind11_VERSION}', needed '${_pybind11_ver}', building.")
    set(pybind11_satisfied_by_system FALSE)
  endif()
else()
  message(STATUS "Finding pybind11: not found, building.")
  set(pybind11_satisfied_by_system FALSE)
endif()

ament_vendor(intrinsic_pybind11_vendor
  VCS_URL https://github.com/pybind/pybind11.git
  VCS_VERSION "v${_pybind11_ver}"
  SATISFIED ${pybind11_satisfied_by_system}
  CMAKE_ARGS
    -DCMAKE_CXX_STANDARD=20
)

ament_package()
