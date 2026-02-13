set(icon_hwm_gen_dir "${CMAKE_CURRENT_BINARY_DIR}/fbs_gen")
file(MAKE_DIRECTORY "${icon_hwm_gen_dir}")

set(hwm_fbs_root_dir "${CMAKE_CURRENT_SOURCE_DIR}")
file(GLOB_RECURSE icon_hwm_SRCS "${hwm_fbs_root_dir}/hwm_fbs/*.fbs")
# _print_list(icon_hwm_SRCS)

# Generate code from flatbuffers schemas.
add_custom_target(icon_hwm_fbs)
message("Added icon_hwm_fbs target, walking through fbs sources from root dir ${hwm_fbs_root_dir}: ${icon_hwm_SRCS}")
foreach(schema_path ${icon_hwm_SRCS})
  get_filename_component(schema_name ${schema_path} NAME_WE)
  get_filename_component(schema_dir ${schema_path} DIRECTORY)
  file(RELATIVE_PATH schema_relpath "${hwm_fbs_root_dir}" "${schema_dir}")
  set(schema_header "${icon_hwm_gen_dir}/${schema_relpath}/${schema_name}.fbs.h")
  message("Adding custom command for ${schema_path}")
  add_custom_command(
    OUTPUT "${schema_header}"
    COMMAND
      flatbuffers::flatc
        --cpp
        --scoped-enums
        --keep-prefix
        --gen-mutable
        --filename-suffix .fbs
        -I "${hwm_fbs_root_dir}"
        -o "${icon_hwm_gen_dir}/${schema_relpath}"
        ${icon_hwm_SRCS}
    WORKING_DIRECTORY
      "${hwm_fbs_root_dir}"
    COMMENT
      "Generating code for flatbuffer '${schema_relpath}/${schema_name}.fbs'"
  )
  add_custom_target(icon_hwm_${schema_name}_flatc DEPENDS "${schema_header}")
  add_dependencies(icon_hwm_fbs icon_hwm_${schema_name}_flatc)
endforeach()

# Install generated flatbuffers code.
install(DIRECTORY "${icon_hwm_gen_dir}/"
        DESTINATION "include/${PROJECT_NAME}"
        FILES_MATCHING # install only matched files
        PATTERN "*.h" # select header files
)

# Install the proto files.
install(
  DIRECTORY "${hwm_fbs_root_dir}" # source directory
  DESTINATION "share/${PROJECT_NAME}/fbs" # target directory
  FILES_MATCHING # install only matched files
  PATTERN "*.fbs" # select header files
)
