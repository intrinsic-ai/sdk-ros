set(intrinsic_fbs_gen_dir "${CMAKE_CURRENT_BINARY_DIR}/fbs_gen")
file(MAKE_DIRECTORY "${intrinsic_fbs_gen_dir}")

file(GLOB_RECURSE intrinsic_fbs_SRCS "${intrinsic_sdk_SOURCE_DIR}/**/*.fbs")
# _print_list(intrinsic_fbs_SRCS)

# Generate code from flatbuffers schemas.
add_custom_target(intrinsic_sdk_fbs)
foreach(schema_path ${intrinsic_fbs_SRCS})
  get_filename_component(schema_name ${schema_path} NAME_WE)
  get_filename_component(schema_dir ${schema_path} DIRECTORY)
  file(RELATIVE_PATH schema_relpath "${intrinsic_sdk_SOURCE_DIR}" "${schema_dir}")
  set(schema_header "${intrinsic_fbs_gen_dir}/${schema_relpath}/${schema_name}.fbs.h")
  add_custom_command(
    OUTPUT "${schema_header}"
    COMMAND
      flatbuffers::flatc
        --cpp
        --scoped-enums
        --keep-prefix
        --gen-mutable
        --filename-suffix .fbs
        -I "${intrinsic_sdk_SOURCE_DIR}"
        -o "${intrinsic_fbs_gen_dir}/${schema_relpath}"
        ${intrinsic_fbs_SRCS}
    WORKING_DIRECTORY
      "${intrinsic_sdk_SOURCE_DIR}"
    COMMENT
      "Generating code for flatbuffer '${schema_relpath}/${schema_name}.fbs'"
  )
  add_custom_target(intrinsic_sdk_${schema_name}_flatc DEPENDS "${schema_header}")
  add_dependencies(intrinsic_sdk_fbs intrinsic_sdk_${schema_name}_flatc)
endforeach()

# Install generated flatbuffers code.
install(DIRECTORY "${intrinsic_fbs_gen_dir}/"
        DESTINATION "include/${PROJECT_NAME}"
        FILES_MATCHING # install only matched files
        PATTERN "*.h" # select header files
)

# Install the proto files.
install(
  DIRECTORY "${intrinsic_sdk_SOURCE_DIR}" # source directory
  DESTINATION "share/${PROJECT_NAME}/fbs" # target directory
  FILES_MATCHING # install only matched files
  PATTERN "*.fbs" # select header files
)
