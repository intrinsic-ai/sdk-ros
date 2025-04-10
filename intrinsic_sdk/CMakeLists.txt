cmake_minimum_required(VERSION 3.20)
project(intrinsic_sdk LANGUAGES CXX)

find_package(ament_cmake REQUIRED)

find_package(intrinsic_sdk_ros REQUIRED)

ament_export_dependencies(intrinsic_sdk_ros)

ament_package(CONFIG_EXTRAS cmake/intrinsic_sdk-extras.cmake.in)
