cmake_minimum_required(VERSION 3.8)
project(generic_action_skill)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(intrinsic_sdk_cmake REQUIRED)

intrinsic_sdk_protobuf_generate(
  NAME generic_action_skill
  SOURCES src/generic_action_skill.proto
  TARGET generic_action_skill_protos
  DESCRIPTOR_SET_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generic_action_skill_protos.desc"
)

add_custom_command(
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generic_action_skill_pb2.py"
  COMMAND protoc -I${CMAKE_CURRENT_SOURCE_DIR}/src --python_out=${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/src/generic_action_skill.proto
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/generic_action_skill.proto
)
add_custom_target(generate_python_proto ALL DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/generic_action_skill_pb2.py")

intrinsic_sdk_generate_skill_config(
  TARGET generic_action_skill_skill_config
  SKILL_NAME generic_action_skill
  MANIFEST "${CMAKE_CURRENT_SOURCE_DIR}/src/generic_action_skill.manifest.textproto"
  PROTO_DESCRIPTOR_FILE "${CMAKE_CURRENT_BINARY_DIR}/generic_action_skill_protos.desc"
  SKILL_CONFIG_FILE_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generic_action_skill_config.pbbin"
)

add_custom_target(force_manifest_generation ALL)
add_dependencies(force_manifest_generation 
  generic_action_skill_protos
  generate_python_proto
  generic_action_skill_skill_config
)

install(
  FILES 
    "${CMAKE_CURRENT_BINARY_DIR}/generic_action_skill_config.pbbin"
    "${CMAKE_CURRENT_BINARY_DIR}/generic_action_skill_protos.desc"
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  generic_action_skill/generic_action_skill.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME generic_action_skill_main
)

ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(
  FILES 
    "${CMAKE_CURRENT_BINARY_DIR}/generic_action_skill_pb2.py"
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

ament_package()
