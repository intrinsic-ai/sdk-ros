cmake_minimum_required(VERSION 3.8)
project(test_python_skill)



# Generate descriptor set from proto
intrinsic_sdk_protobuf_generate(
  NAME test_python_skill
  SOURCES test_python_skill.proto
  TARGET test_python_skill_protos
  DESCRIPTOR_SET_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test_python_skill_protos.desc"
)

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
  test_python_skill_skill_config
)

# Install files
install(
  FILES 
    "${CMAKE_CURRENT_BINARY_DIR}/test_python_skill_config.pbbin"
    "${CMAKE_CURRENT_BINARY_DIR}/test_python_skill_protos.desc"
  DESTINATION share/intrinsic_sdk_cmake
)

# Install Python script
install(PROGRAMS
  test_python_skill.py
  DESTINATION lib/intrinsic_sdk_cmake
  RENAME test_python_skill_main
)


