cmake_minimum_required(VERSION 3.10)
project(ros_gz_interfaces_vendor)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

ament_vendor(ros_gz_interfaces_vendor
  VCS_URL https://github.com/gazebosim/ros_gz/
  VCS_VERSION e2b1dbd6385bd92e6e83ad9f96042cc9979a4655 # Latest main as of 26/03/2026
  SOURCE_SUBDIR ros_gz_interfaces
  GLOBAL_HOOK
)

ament_package()
