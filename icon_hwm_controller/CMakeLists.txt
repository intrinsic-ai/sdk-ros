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

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(cmake/dependencies.cmake)



# find dependencies
find_package(ament_cmake REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

generate_parameter_library(icon_hwm_controller_parameters
  src/icon_hwm_controller_parameters.yaml
)

add_library( icon_hwm_controller SHARED
  src/icon_hwm_controller.cpp
)

set_property(
  TARGET icon_hwm_controller
  PROPERTY POSITION_INDEPENDENT_CODE ON
)
target_include_directories(icon_hwm_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/icon_hwm_controller>
)
target_link_libraries(icon_hwm_controller PUBLIC
  tl::expected
  icon_hwm_controller_parameters
  icon_shared_memory_icon_interprocess_shared_memory_manager
  icon_shared_memory_util_thread
  icon_shared_memory_icon_utils_log
  icon_shared_memory_icon_utils_time
  icon_shared_memory_icon_hal_interfaces_joint_command_fbs_utils
  icon_shared_memory_icon_hal_interfaces_joint_state_fbs_utils
  icon_shared_memory_icon_hal_interfaces_joint_limits_fbs_utils
  icon_shared_memory_icon_hal_interfaces_hardware_module_state_fbs_utils
  icon_shared_memory_icon_hal_hardware_interface_traits
  icon_shared_memory_icon_hal_hardware_interface_registry
  icon_shared_memory_icon_hal_icon_state_register
  icon_shared_memory_icon_interprocess_shared_memory_manager_domain_socket_server
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
  ${controller_manager_msgs_TARGETS}
  ${lifecycle_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${icon_hwm_controller_msgs_TARGETS}
)

pluginlib_export_plugin_description_file(controller_interface icon_hwm_controller.xml)

install(TARGETS
  icon_hwm_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_dependencies(
  flatbuffers_vendor
  tl_expected_vendor
  icon_shared_memory_vendor
  eigen_vendor
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  controller_manager_msgs
  lifecycle_msgs
  std_msgs
  generate_parameter_library
  icon_hwm_controller_msgs
)
ament_package()
