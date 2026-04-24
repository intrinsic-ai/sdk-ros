cmake_minimum_required(VERSION 3.8)
project(test_python_skill)

# Generate descriptor set from proto
intrinsic_sdk_protobuf_generate(
  NAME test_python_skill
  SOURCES test_python_skill.proto
  TARGET test_python_skill_protos
  DESCRIPTOR_SET_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test_python_skill_protos.desc"
)

# Generate Python proto
add_custom_command(
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test_python_skill_pb2.py"
  COMMAND protoc -I${CMAKE_CURRENT_SOURCE_DIR} --python_out=${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/test_python_skill.proto
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/test_python_skill.proto
)
add_custom_target(generate_python_proto ALL DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/test_python_skill_pb2.py")

# Generate skill config
intrinsic_sdk_generate_skill_config(
  TARGET test_python_skill_skill_config
  SKILL_NAME test_python_skill
  MANIFEST "${CMAKE_CURRENT_SOURCE_DIR}/test_python_skill.manifest.textproto"
  PROTO_DESCRIPTOR_FILE "${CMAKE_CURRENT_BINARY_DIR}/test_python_skill_protos.desc"
  SKILL_CONFIG_FILE_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test_python_skill_config.pbbin"
)

add_custom_target(force_manifest_generation ALL)
add_dependencies(force_manifest_generation 
  test_python_skill_protos
  generate_python_proto
  test_python_skill_skill_config
)

# Install files
install(
  FILES 
    "${CMAKE_CURRENT_BINARY_DIR}/test_python_skill_config.pbbin"
    "${CMAKE_CURRENT_BINARY_DIR}/test_python_skill_protos.desc"
  DESTINATION share/intrinsic_sdk_cmake
)

# Install Python script as executable
install(PROGRAMS
  test_python_skill.py
  DESTINATION lib/intrinsic_sdk_cmake
  RENAME test_python_skill_main
)

# Install Python files as a package
# Create an empty __init__.py file
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/__init__.py" "")

install(
  FILES 
    "${CMAKE_CURRENT_BINARY_DIR}/test_python_skill_pb2.py"
    "${CMAKE_CURRENT_BINARY_DIR}/__init__.py"
    "test_python_skill.py"
  DESTINATION "lib/python3.12/site-packages/${PROJECT_NAME}"
)
