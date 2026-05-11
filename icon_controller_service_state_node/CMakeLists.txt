cmake_minimum_required(VERSION 3.19)
project(icon_controller_service_state_node CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(intrinsic_sdk_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(grpc_vendor REQUIRED)
find_package(gRPC REQUIRED)
find_package(icon_hwm_controller_msgs REQUIRED)

include_directories(
  include
)

add_library(
  icon_controller_service_state src/icon_controller_service_state.cpp)
target_link_libraries(
  icon_controller_service_state
  PUBLIC
  intrinsic_sdk_cmake::intrinsic_sdk_services
  intrinsic_sdk_cmake::intrinsic_sdk_protos
  ${controller_manager_msgs_TARGETS}
  rclcpp::rclcpp
  icon_hwm_controller_msgs::icon_hwm_controller_msgs__rosidl_typesupport_cpp
)
add_executable(
  icon_controller_service_state_node src/service_state_node_main.cpp)
target_link_libraries(
  icon_controller_service_state_node
  PUBLIC
  icon_controller_service_state
  rclcpp::rclcpp
)

install(TARGETS
  icon_controller_service_state_node
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

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
  ament_add_gmock(icon_controller_service_state_test test/service_state_test.cpp)
  target_include_directories(icon_controller_service_state_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(icon_controller_service_state_test icon_controller_service_state)
endif()

ament_export_dependencies(
  intrinsic_sdk_cmake
  rclcpp
  rclcpp_lifecycle
  controller_manager_msgs
  lifecycle_msgs
  std_msgs
)
ament_package()
