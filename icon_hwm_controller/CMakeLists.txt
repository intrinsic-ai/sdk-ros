cmake_minimum_required(VERSION 3.19)
project(icon_hwm_controller CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(cmake/dependencies.cmake)
# Build flatbuffers.
include(cmake/hwm_flatbuffers.cmake)


# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(testing testing.cpp)
add_dependencies(testing icon_hwm_fbs)
target_link_libraries(testing
PUBLIC
flatbuffers::flatbuffers)
target_include_directories(testing PUBLIC
  "$<BUILD_INTERFACE:${icon_hwm_gen_dir}>")

add_library(shared_memory_manager
  src/shared_memory_manager.cpp
  src/domain_socket_server.cpp
  src/domain_socket_utils.cpp
  src/segment_header.cpp
  src/memory_segment.cpp
  src/unique_segment_name.cpp
  src/segment_info_utils.cpp
  src/icon_state_utils.cpp
  src/thread.cpp
  src/joint_limits.cpp
  src/joint_limits_utils.cpp
  src/hardware_interface_registry.cpp
)

add_dependencies(shared_memory_manager icon_hwm_fbs)
target_link_libraries(shared_memory_manager
  PUBLIC
  tl::expected
  flatbuffers::flatbuffers)
target_include_directories(shared_memory_manager PUBLIC
    "$<BUILD_INTERFACE:${icon_hwm_gen_dir}>")


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  ament_add_gmock(shared_memory_manager_test test/shared_memory_manager_test.cpp)
  target_include_directories(shared_memory_manager_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(shared_memory_manager_test shared_memory_manager)

  ament_add_gmock(get_hardware_interface_test test/get_hardware_interface_test.cpp)
  target_include_directories(get_hardware_interface_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(get_hardware_interface_test shared_memory_manager)

  ament_add_gmock(segment_header_test test/segment_header_test.cpp)
  target_include_directories(segment_header_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(segment_header_test shared_memory_manager)

  ament_add_gmock(segment_info_utils_test test/segment_info_utils_test.cpp)
  target_include_directories(segment_info_utils_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(segment_info_utils_test shared_memory_manager)

  ament_add_gmock(domain_socket_utils_test test/domain_socket_utils_test.cpp)
  target_include_directories(domain_socket_utils_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(domain_socket_utils_test shared_memory_manager)

  ament_add_gmock(thread_test test/thread_test.cpp)
  target_include_directories(thread_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(thread_test shared_memory_manager)

  ament_add_gmock(joint_limits_utils_test test/joint_limits_utils_test.cpp)
  target_include_directories(joint_limits_utils_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(joint_limits_utils_test shared_memory_manager)

  ament_add_gmock(hardware_interface_registry_test test/hardware_interface_registry_test.cpp)
  target_include_directories(hardware_interface_registry_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(hardware_interface_registry_test shared_memory_manager)

endif()

ament_package()
