# Generate the skill's protobuf files, protobuf library, and descriptor set file.
intrinsic_sdk_protobuf_generate(
  NAME test_cpp_skill
  SOURCES test_cpp_skill.proto
  TARGET test_cpp_skill_protos
  DESCRIPTOR_SET_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test_cpp_skill.desc"
)

# Generate the skill config file.
intrinsic_sdk_generate_skill_config(
  TARGET test_cpp_skill_skill_config
  MANIFEST "${CMAKE_CURRENT_SOURCE_DIR}/test_cpp_skill.manifest.textproto"
  PROTO_DESCRIPTOR_FILE "${CMAKE_CURRENT_BINARY_DIR}/test_cpp_skill.desc"
  SKILL_CONFIG_FILE_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test_cpp_skill_config.pbbin"
)
install(
  FILES "${CMAKE_CURRENT_BINARY_DIR}/test_cpp_skill_config.pbbin"
  DESTINATION share/${PROJECT_NAME}
)

# Generate the c++ main file for the skill.
intrinsic_sdk_generate_skill_main_cc(
  MANIFEST "${CMAKE_CURRENT_SOURCE_DIR}/test_cpp_skill.manifest.textproto"
  HEADER_FILE test_cpp_skill.h
  MAIN_FILE_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test_cpp_skill_main.cc"
)

# Build the skill main.
add_executable(test_cpp_skill_main
  # Generated main file
  "${CMAKE_CURRENT_BINARY_DIR}/test_cpp_skill_main.cc"
  # User created skill class
  test_cpp_skill.cc
)
target_include_directories(test_cpp_skill_main
  PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(test_cpp_skill_main
  # SDK library
  intrinsic_sdk_cmake::intrinsic_sdk_cmake
  # Target that generates skill's protobuf files and library
  test_cpp_skill_protos
)
install(TARGETS test_cpp_skill_main DESTINATION lib/${PROJECT_NAME} OPTIONAL)
add_dependencies(test_cpp_skill_main test_cpp_skill_skill_config)

# Create the skill container image.
intrinsic_sdk_generate_skill_container_image(
  TARGET test_cpp_skill_container_image
  # TODO(wjwwood): extract the skill name from the manifest
  SKILL_NAME test_cpp_skill
  SKILL_PACKAGE ${PROJECT_NAME}
  SKILL_EXECUTABLE lib/${PROJECT_NAME}/test_cpp_skill_main
  SKILL_CONFIG share/${PROJECT_NAME}/test_cpp_skill_config.pbbin
  SKILL_ASSET_ID_ORG ai.intrinsic
  CONTAINER_TAG_NAME "test_cpp_skill:latest"
  CONTAINER_CONTEXT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  CONTAINER_IMAGE_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test_cpp_skill_container_image.tar"
  # TODO(wjwwood): implement custom dockerfiles
  # You could provide a custom dockerfile if the standard one is insufficient.
  # DOCKERFILE my_custom_skill.Dockerfile
)

# Create the skill bundle.
intrinsic_sdk_generate_skill_bundle(
  TARGET test_cpp_skill_bundle
  MANIFEST "${CMAKE_CURRENT_SOURCE_DIR}/test_cpp_skill.manifest.textproto"
  PROTO_DESCRIPTOR_FILE "${CMAKE_CURRENT_BINARY_DIR}/test_cpp_skill.desc"
  OCI_IMAGE "${CMAKE_CURRENT_BINARY_DIR}/test_cpp_skill_container_image.tar"
  SKILL_BUNDLE_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test_cpp_skill.bundle.tar"
)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/test_cpp_skill.bundle.tar" DESTINATION share/${PROJECT_NAME})