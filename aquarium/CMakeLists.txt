# Copyright 2026 Intrinsic Innovation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.20)
project(aquarium)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(backward_ros REQUIRED)
find_package(intrinsic_sdk_cmake REQUIRED)
find_package(rcl_action REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(type_description_interfaces REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(zenoh_cpp_vendor REQUIRED)
find_package(zenohc REQUIRED)
find_package(zenohcxx REQUIRED)

# Serialization library
add_library(aquarium_gateway_serialization STATIC
  src/gateway/serialization.cpp
)
target_include_directories(aquarium_gateway_serialization PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(aquarium_gateway_serialization
  PUBLIC
  ${type_description_interfaces_TARGETS}
  intrinsic_sdk_cmake::intrinsic_sdk_cmake
)

# Gateway core library
add_library(aquarium_gateway_core STATIC
  src/gateway/aquarium_gateway.cpp
)
target_include_directories(aquarium_gateway_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(aquarium_gateway_core
  PUBLIC
  aquarium_gateway_serialization
  rclcpp::rclcpp
  rcl_action::rcl_action
  rclcpp_action::rclcpp_action
  yaml-cpp
  zenohcxx::zenohc
  intrinsic_sdk_cmake::intrinsic_sdk_cmake
  ${CMAKE_DL_LIBS}
)

# Gateway executable
add_executable(aquarium_gateway
  src/gateway/main.cpp
)
target_include_directories(aquarium_gateway PRIVATE
  ${backward_ros_INCLUDE_DIRS}
)
target_link_libraries(aquarium_gateway
  aquarium_gateway_core
  ${backward_ros_LIBRARIES}
)

# Setup assistant TUI executable
add_executable(aquarium_setup_assistant
  src/setup_assistant/aquarium_setup_assistant_node.cpp
  src/setup_assistant/terminal_ui.cpp
  src/setup_assistant/config_writer.cpp
)
target_include_directories(aquarium_setup_assistant PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(aquarium_setup_assistant
  aquarium_gateway_serialization
  rclcpp::rclcpp
  rcl_action::rcl_action
  rclcpp_action::rclcpp_action
  ${type_description_interfaces_TARGETS}
  ament_index_cpp::ament_index_cpp
  zenohcxx::zenohc
  yaml-cpp
)

# Python CLI package, packaged resources/templates, and executable wrappers
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY
  aquarium/resources
  aquarium/templates
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

install(PROGRAMS
  scripts/aquarium
  DESTINATION bin
)
install(PROGRAMS
  scripts/aquarium
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(builtin_interfaces REQUIRED)
  find_package(control_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(rmw REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(tf2_msgs REQUIRED)

  # Locate sdk_version.json from intrinsic_sdk_bundle_library_py via ament_index
  set(SDK_VERSION_JSON_FILE "")
  ament_index_get_prefix_path(ament_prefixes)
  foreach(prefix IN LISTS ament_prefixes)
    set(sdk_pkg_share "${prefix}/share/intrinsic_sdk_bundle_library_py")
    set(candidate "${sdk_pkg_share}/resource/sdk_version.json")
    if(EXISTS "${candidate}")
      set(SDK_VERSION_JSON_FILE "${candidate}")
      break()
    endif()
  endforeach()
  if(NOT SDK_VERSION_JSON_FILE)
    message(FATAL_ERROR "Could not find sdk_version.json via ament_index")
  endif()

  file(READ "${SDK_VERSION_JSON_FILE}" sdk_version_json)
  string(JSON sdk_version GET ${sdk_version_json} "sdk_version")
  string(JSON sdk_checksum GET ${sdk_version_json} "sdk_checksum")

  include(FetchContent)
  FetchContent_Declare(
    intrinsic_sdk_protos_fetch
    URL https://github.com/intrinsic-ai/sdk/archive/refs/tags/${sdk_version}.tar.gz
    URL_HASH ${sdk_checksum}
    DOWNLOAD_EXTRACT_TIMESTAMP FALSE
  )
  FetchContent_GetProperties(intrinsic_sdk_protos_fetch)
  if(NOT intrinsic_sdk_protos_fetch_POPULATED)
    FetchContent_MakeAvailable(intrinsic_sdk_protos_fetch)
  endif()

  set(CAMERA_PROTO_REL "intrinsic/perception/proto/v1/camera_service.proto")
  set(CAMERA_SERVICE_PROTO_PATH
    "${intrinsic_sdk_protos_fetch_SOURCE_DIR}/${CAMERA_PROTO_REL}"
  )
  set(PERCEPTION_GEN_OUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated_perception")
  file(REMOVE_RECURSE "${PERCEPTION_GEN_OUT_DIR}")

  execute_process(
    COMMAND ${CMAKE_COMMAND} -E env
      "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:$ENV{PYTHONPATH}"
      ${Python3_EXECUTABLE}
      "${CMAKE_CURRENT_SOURCE_DIR}/scripts/aquarium"
      generate ros
      --service CameraService
      --proto_path "${CAMERA_SERVICE_PROTO_PATH}"
      --output_path "${PERCEPTION_GEN_OUT_DIR}"
      --single_package "${PROJECT_NAME}"
      --target_package demo_camera_driver
      --target_node camera_driver_node
    RESULT_VARIABLE GEN_RESULT
    OUTPUT_VARIABLE GEN_OUT
    ERROR_VARIABLE GEN_ERR
  )
  if(NOT GEN_RESULT EQUAL 0)
    message(STATUS "Generator stdout: ${GEN_OUT}")
    message(STATUS "Generator stderr: ${GEN_ERR}")
    message(
      FATAL_ERROR "Failed to generate perception ROS interfaces for testing"
    )
  endif()

  set(PERCEPTION_BASE_DIR "${PERCEPTION_GEN_OUT_DIR}/${PROJECT_NAME}")
  file(GLOB PERCEPTION_MSG_FILES "${PERCEPTION_BASE_DIR}/msg/*.msg")
  file(GLOB PERCEPTION_SRV_FILES "${PERCEPTION_BASE_DIR}/srv/*.srv")
  set(PERCEPTION_ROSIDL_FILES)
  foreach(idl_file ${PERCEPTION_MSG_FILES} ${PERCEPTION_SRV_FILES})
    file(RELATIVE_PATH rel_path "${PERCEPTION_BASE_DIR}" "${idl_file}")
    list(APPEND PERCEPTION_ROSIDL_FILES "${PERCEPTION_BASE_DIR}:${rel_path}")
  endforeach()

  rosidl_generate_interfaces(${PROJECT_NAME}
    "${CMAKE_CURRENT_SOURCE_DIR}/test:msg/TestCameraConfig.msg"
    "${CMAKE_CURRENT_SOURCE_DIR}/test:msg/TestStorageLocation.msg"
    "${CMAKE_CURRENT_SOURCE_DIR}/test:msg/TestSensorImage.msg"
    "${CMAKE_CURRENT_SOURCE_DIR}/test:msg/TestCaptureResult.msg"
    "${CMAKE_CURRENT_SOURCE_DIR}/test:srv/TestCapture.srv"
    ${PERCEPTION_ROSIDL_FILES}
    DEPENDENCIES
      builtin_interfaces
      geometry_msgs
      std_msgs
    SKIP_GROUP_MEMBERSHIP_CHECK
  )

  rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME} rosidl_typesupport_cpp
  )
  rosidl_get_typesupport_target(c_typesupport_target
    ${PROJECT_NAME} rosidl_typesupport_c
  )

  ament_add_gtest(test_serialization
    test/test_serialization.cpp
  )
  target_include_directories(test_serialization PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}/test
  )
  protobuf_generate(
    TARGET test_serialization
    PROTOS test/test_messages.proto
  )
  target_link_libraries(test_serialization
    aquarium_gateway_serialization
    rclcpp::rclcpp
    rmw::rmw
    intrinsic_sdk_cmake::intrinsic_sdk_cmake
    sensor_msgs::sensor_msgs__rosidl_generator_c
    sensor_msgs::sensor_msgs__rosidl_typesupport_c
    ${sensor_msgs_TARGETS}
    geometry_msgs::geometry_msgs__rosidl_generator_c
    geometry_msgs::geometry_msgs__rosidl_typesupport_c
    ${geometry_msgs_TARGETS}
    tf2_msgs::tf2_msgs__rosidl_generator_c
    tf2_msgs::tf2_msgs__rosidl_typesupport_c
    ${tf2_msgs_TARGETS}
    nav_msgs::nav_msgs__rosidl_generator_c
    nav_msgs::nav_msgs__rosidl_typesupport_c
    ${nav_msgs_TARGETS}
  )

  ament_add_gtest(test_perception_serialization
    test/test_perception_serialization.cpp
  )
  target_include_directories(test_perception_serialization PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}/test
  )
  protobuf_generate(
    TARGET test_perception_serialization
    PROTOS test/test_perception_messages.proto
  )
  target_link_libraries(test_perception_serialization
    aquarium_gateway_serialization
    rclcpp::rclcpp
    rmw::rmw
    intrinsic_sdk_cmake::intrinsic_sdk_cmake
    ${cpp_typesupport_target}
    ${c_typesupport_target}
    ${builtin_interfaces_TARGETS}
    ${geometry_msgs_TARGETS}
    ${std_msgs_TARGETS}
  )

  ament_add_gtest(test_action_gateway
    test/test_action_gateway.cpp
  )
  target_include_directories(test_action_gateway PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}/test
  )
  target_link_libraries(test_action_gateway
    aquarium_gateway_core
    aquarium_gateway_serialization
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    rcl_action::rcl_action
    rmw::rmw
    intrinsic_sdk_cmake::intrinsic_sdk_cmake
    control_msgs::control_msgs__rosidl_generator_c
    control_msgs::control_msgs__rosidl_typesupport_c
    ${control_msgs_TARGETS}
    sensor_msgs::sensor_msgs__rosidl_generator_c
    sensor_msgs::sensor_msgs__rosidl_typesupport_c
    ${sensor_msgs_TARGETS}
  )

  ament_add_gtest(test_terminal_ui
    test/test_terminal_ui.cpp
    src/setup_assistant/terminal_ui.cpp
    src/setup_assistant/config_writer.cpp
  )
  target_include_directories(test_terminal_ui PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_terminal_ui
    rclcpp::rclcpp
    ${type_description_interfaces_TARGETS}
    ament_index_cpp::ament_index_cpp
    zenohcxx::zenohc
    yaml-cpp
  )

  ament_add_pytest_test(test_targets test/test_targets.py)

  ament_add_gtest_executable(test_live_introspection
    test/test_live_introspection.cpp
    src/setup_assistant/terminal_ui.cpp
    SKIP_LINKING_MAIN_LIBRARIES
  )
  if(TARGET test_live_introspection)
    target_include_directories(test_live_introspection PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    target_link_libraries(test_live_introspection
      aquarium_gateway_serialization
      rclcpp::rclcpp
      ${type_description_interfaces_TARGETS}
      ament_index_cpp::ament_index_cpp
      zenohcxx::zenohc
    )
    install(TARGETS test_live_introspection
      DESTINATION lib/${PROJECT_NAME}
    )
  endif()

  install(FILES
    test/dummy_camera_server.py
    test/dummy_grpc_server.py
    test/test_aquarium_add_two_ints.yaml
    test/test_aquarium_chatter.yaml
    test/test_aquarium_parallel_gripper_command.yaml
    test/test_complex_types.yaml
    test/test_selection.yaml
    DESTINATION share/${PROJECT_NAME}/test
  )

  add_launch_test(test/test_add_two_ints_skill.py
    TARGET test_add_two_ints_skill
    TIMEOUT 120
  )
  add_launch_test(test/test_topic_subscription_skill.py
    TARGET test_topic_subscription_skill
    TIMEOUT 120
  )
  add_launch_test(test/test_live_introspection.launch.py
    TARGET test_live_introspection_launch
    TIMEOUT 60
  )
  add_launch_test(test/test_assistant_run.py
    TARGET test_assistant_run
    TIMEOUT 60
  )
  add_launch_test(test/test_add_two_ints_node.py
    TARGET test_add_two_ints_node
    TIMEOUT 60
  )
  add_launch_test(test/test_complex_types.py
    TARGET test_complex_types
    TIMEOUT 120
  )
  set_tests_properties(
    test_add_two_ints_skill
    test_topic_subscription_skill
    test_live_introspection_launch
    test_assistant_run
    test_add_two_ints_node
    test_complex_types
    PROPERTIES RUN_SERIAL TRUE
  )
endif()

install(TARGETS
  aquarium_gateway
  aquarium_setup_assistant
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  examples
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  LICENSE
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
