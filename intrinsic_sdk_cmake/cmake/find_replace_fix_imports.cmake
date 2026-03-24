set(python_package_directory ${PYTHON_PACKAGE_DIRECTORY})
set(python_package_to_install_package_dir
  "${CMAKE_CURRENT_BINARY_DIR}/protos_gen_py/${python_package_directory}")
set(python_package_to_install_package_dir_fixed
  "${CMAKE_CURRENT_BINARY_DIR}/protos_gen_py/fixed_imports/${python_package_directory}")

file(GLOB_RECURSE python_package_sources
  RELATIVE "${python_package_to_install_package_dir}"
  "${python_package_to_install_package_dir}/**/*.py"
)

set(FIND_LIST
  "from google.api"
  "from google.longrunning"
  "from google.rpc"
  "from google.type"
)
set(REPLACE_LIST
  "from googleapis.google.api"
  "from googleapis.google.longrunning"
  "from googleapis.google.rpc"
  "from googleapis.google.type"
)

foreach(python_package_source ${python_package_sources})
  set(source_file "${python_package_to_install_package_dir}/${python_package_source}")
  set(target_file "${python_package_to_install_package_dir_fixed}/${python_package_source}")

  file(READ "${source_file}" contents)

  list(LENGTH FIND_LIST len1)
  math(EXPR len2 "${len1} - 1")

  foreach(val RANGE ${len2})
    list(GET FIND_LIST ${val} find)
    list(GET REPLACE_LIST ${val} replace)

    message(STATUS "'${find}' -> '${replace}' in '${source_file}'")
    string(REPLACE "${find}" "${replace}" contents "${contents}")
  endforeach()

  get_filename_component(target_file_dir "${target_file}" DIRECTORY)
  file(MAKE_DIRECTORY "${target_file_dir}")

  file(WRITE "${target_file}" "${contents}")
endforeach()
