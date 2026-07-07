cmake_minimum_required(VERSION 3.10)
project(zenoh_cpp_vendor)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

set(zenoh_cpp_vendor_satisfied_by_system FALSE)
find_package(zenoh_cpp_vendor QUIET)
if(zenoh_cpp_vendor_FOUND)
  message(STATUS "Found zenoh_cpp_vendor, skipping build.")
  set(zenoh_cpp_vendor_satisfied_by_system TRUE)
else()
  message(STATUS "zenoh_cpp_vendor not found, building.")
  set(zenoh_cpp_vendor_satisfied_by_system FALSE)
endif()

ament_vendor(zenoh_cpp_vendor
  VCS_URL https://github.com/ros2/zenoh_cpp_vendor.git
  VCS_VERSION 0.2.9
  SATISFIED ${zenoh_cpp_vendor_satisfied_by_system}
  GLOBAL_HOOK
)

ament_export_dependencies(zenohc zenohcxx)
if(zenoh_cpp_vendor_satisfied_by_system)
  get_filename_component(SYSTEM_ZENOH_PREFIX "${zenoh_cpp_vendor_DIR}/../../.." ABSOLUTE)
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/zenoh_system_prefix.cmake"
    "list(PREPEND CMAKE_PREFIX_PATH \"${SYSTEM_ZENOH_PREFIX}/opt/zenoh_cpp_vendor\")\n"
  )
  ament_package(CONFIG_EXTRAS "${CMAKE_CURRENT_BINARY_DIR}/zenoh_system_prefix.cmake")
else()
  ament_package()
endif()
