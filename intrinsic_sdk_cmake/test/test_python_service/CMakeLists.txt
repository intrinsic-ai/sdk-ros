cmake_minimum_required(VERSION 3.8)
project(test_python_service)

find_package(ament_cmake REQUIRED)

set(SERVICE_NAME "test_python_service")

intrinsic_sdk_generate_service_manifest(
  SERVICE_NAME ${SERVICE_NAME}
  MANIFEST ${SERVICE_NAME}.manifest.textproto
  PARAMETER_DESCRIPTOR ${SERVICE_NAME}.proto
  DEFAULT_CONFIGURATION ${SERVICE_NAME}_default_config.pbtxt
  PROTOS_TARGET ${SERVICE_NAME}_protos
)

# Install Python script as executable
install(PROGRAMS
  test_python_service.py
  DESTINATION lib/intrinsic_sdk_cmake
  RENAME test_python_service_main
)

# Install proto descriptor file
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${SERVICE_NAME}_protos.desc
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

