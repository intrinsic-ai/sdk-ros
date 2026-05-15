cmake_minimum_required(VERSION 3.10)
project(gz_transport_vendor CXX)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)
set(gz_transport_satisfied_by_system FALSE)
find_package(gz-transport QUIET)
if(gz-transport_FOUND AND gz-transport_VERSION VERSION_GREATER_EQUAL 15.0.0)
  message(STATUS "Found gz-transport 15+, skipping build.")
  set(gz_transport_satisfied_by_system TRUE)
else()
  message(STATUS "gz-transport 15+ not found, building.")
  set(gz_transport_satisfied_by_system FALSE)
endif()

if(NOT gz_transport_satisfied_by_system)
  find_package(grpc_vendor REQUIRED)  # Provides protobuf
  find_package(gz_cmake_vendor REQUIRED)
  find_package(gz_math_vendor REQUIRED)
  find_package(gz_msgs_vendor REQUIRED)
  find_package(gz_utils_vendor REQUIRED)
endif()

ament_vendor(gz_transport_vendor
  VCS_URL https://github.com/gazebosim/gz-transport
  # Target Intrinsic SDK version requires Gazebo Jetty packages.
  VCS_VERSION gz-transport15_15.0.2
  CMAKE_ARGS -DGZ_TRANSPORT_ENABLE_ZENOH=OFF
  SATISFIED ${gz_transport_satisfied_by_system}
  GLOBAL_HOOK
)

ament_package()
