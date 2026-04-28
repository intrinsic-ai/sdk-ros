cmake_minimum_required(VERSION 3.8)
project(test_cpp_service)

find_package(rclcpp REQUIRED)

set(SERVICE_NAME "test_cpp_service")

intrinsic_sdk_generate_service_manifest(
  SERVICE_NAME ${SERVICE_NAME}
  MANIFEST ${SERVICE_NAME}.manifest.textproto
  PARAMETER_DESCRIPTOR ${SERVICE_NAME}.proto
  DEFAULT_CONFIGURATION ${SERVICE_NAME}_default_config.pbtxt
  PROTOS_TARGET ${SERVICE_NAME}_protos
)

add_executable(${SERVICE_NAME}_main
  test_cpp_service_main.cpp
)

target_link_libraries(${SERVICE_NAME}_main
  intrinsic_sdk_cmake::intrinsic_sdk_cmake
  ${SERVICE_NAME}_protos
  rclcpp::rclcpp
)

install(TARGETS ${SERVICE_NAME}_main DESTINATION lib/intrinsic_sdk_cmake)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${SERVICE_NAME}_protos.desc
  DESTINATION share/intrinsic_sdk_cmake
)

