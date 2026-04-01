cmake_minimum_required(VERSION 3.10)
project(pybind11_protobuf_vendor CXX)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

find_package(abseil_cpp_vendor REQUIRED)
find_package(grpc_vendor REQUIRED) 
find_package(intrinsic_pybind11_vendor REQUIRED) 
find_package(pybind11_abseil_vendor REQUIRED)

ament_vendor(pybind11_protobuf_vendor
  VCS_URL https://github.com/pybind/pybind11_protobuf.git
  VCS_VERSION f02a2b7653bc50eb5119d125842a3870db95d251
  CMAKE_ARGS
    -DCMAKE_CXX_STANDARD=20
  PATCHES patches
)

ament_package()
