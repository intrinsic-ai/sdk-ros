cmake_minimum_required(VERSION 3.8)
project(flowstate_ros_gz_bridge)

set(SERVICE_NAME "flowstate_ros_gz_bridge")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(intrinsic_sdk_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz_bridge REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Create the service manifest
intrinsic_sdk_generate_service_manifest(
  SERVICE_NAME ${SERVICE_NAME}_service
  MANIFEST ${SERVICE_NAME}_service.manifest.textproto
  DEFAULT_CONFIGURATION ${SERVICE_NAME}_service_default_config.pbtxt
  PARAMETER_DESCRIPTOR ${SERVICE_NAME}_config.proto
  PROTOS_TARGET ${SERVICE_NAME}_service_protos
)

add_executable(${SERVICE_NAME}_service_main
  src/${SERVICE_NAME}.cc
  src/sim_connection.cc
)

target_link_libraries(
  ${SERVICE_NAME}_service_main
  rclcpp::rclcpp
  ros_gz_bridge::ros_gz_bridge
  ${SERVICE_NAME}_service_protos
  intrinsic_sdk_cmake::intrinsic_sdk_cmake
)

add_dependencies(${SERVICE_NAME}_service_main
  ${SERVICE_NAME}_service_manifest
  ${SERVICE_NAME}_service_default_config
)

# Install the flowstate_ros_gz_bridge service
install(
  TARGETS ${SERVICE_NAME}_service_main
  DESTINATION lib/${PROJECT_NAME}
)

# Install the proto descriptor database, manifest and default config
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${SERVICE_NAME}_service_protos.desc
  ${CMAKE_CURRENT_BINARY_DIR}/service_manifest.binarypb
  ${CMAKE_CURRENT_BINARY_DIR}/default_config.binarypb
  DESTINATION "share/${PROJECT_NAME}/${SERVICE_NAME}_service"
)

ament_package()
