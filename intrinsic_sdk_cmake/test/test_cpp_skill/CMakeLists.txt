# Generate the skill's protobuf files, protobuf library, and descriptor set file.
intrinsic_sdk_protobuf_generate(
  NAME test_cpp_skill
  SOURCES test_cpp_skill.proto
  TARGET test_cpp_skill_protos
  DESCRIPTOR_SET_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test_cpp_skill.desc"
)

# Generate the protobufbin (pbbin) file for the skill manifest.
intrinsic_sdk_generate_skill_manifest_pbbin(
  TARGET test_cpp_skill_manifest
  MANIFEST_TEXTPROTO test_cpp_skill.manifest.textproto
  MANIFEST_PBBIN_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test_cpp_skill.manifest.pbbin"
)

# Generate the c++ main file for the skill.
intrinsic_sdk_generate_skill_main_cc(
  MANIFEST_PBBIN "${CMAKE_CURRENT_BINARY_DIR}/test_cpp_skill.manifest.pbbin"
  HEADER_FILES test_cpp_skill.h
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

# Create the skill container image.
# intrinsic_sdk_generate_skill_container_image(
#   # DOCKERFILE Dockerfile.my_custom_skill
#
# )

# # Create the skill bundle.
# intrisic_sdk_generate_skill_bundle(

# )
