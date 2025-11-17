# set(python_proto_output_dir "${CMAKE_CURRENT_BINARY_DIR}/protos_gen_grpc_tools_py")

# set(python_proto_include_flags "")
# foreach(sdk_proto_import_dir ${sdk_proto_import_dirs})
#   list(APPEND python_proto_include_flags "-I${sdk_proto_import_dir}")
# endforeach()
# list(REMOVE_DUPLICATES python_proto_include_flags)

# set(python_proto_outputs "")
# foreach(sdk_proto ${sdk_protos})

# endforeach()

# add_custom_command(
#   COMMAND python3
#     -m grpc_tools.protoc
#     ${python_proto_include_flags}
#     --python_out="${python_proto_output_dir}"
#     --pyi_out="${python_proto_output_dir}"
#     --grpc_python_out="${python_proto_output_dir}"
#     ${sdk_protos}
# )

# protobuf_generate(
#   LANGUAGE python
#   PROTOS ${sdk_protos}
#   IMPORT_DIRS ${sdk_proto_import_dirs}
#   PROTOC_OUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/protos_gen_py2
#   OUT_VAR sdk_protos_python_sources2
#   PROTOC_EXE python3 -m grpc_tools.protoc
# )

# Namespace the googleapis in Python to avoid collision with google.protobuf.
# This does require some rewriting of python files in the sdk.
set(python_proto_gen_dir "${CMAKE_CURRENT_BINARY_DIR}/protos_gen_py")
set(renamed_googleapis_dir "${python_proto_gen_dir}/googleapis")
add_custom_target(intrinsic_sdk_protos_python_rename_googleapis
  DEPENDS intrinsic_sdk_protos_python
  COMMAND
    ${CMAKE_COMMAND}
      -E copy_directory_if_different
      "${python_proto_gen_dir}/google"
      "${renamed_googleapis_dir}/google"
  COMMAND
    ${CMAKE_COMMAND}
      -E touch
      "${renamed_googleapis_dir}/__init__.py"
  COMMENT "Copying 'google' protobufs into 'googleapis' namespace..."
)

# Glob the source files and then exclude files that don't make sense to be in the glob.
# Note(wjwwood): we know this isn't the "right" way to do this, but it helps us catch
# additions to the sdk when it is upgraded until we have a more structured approach
# to extract build targets out of bazel.
file(GLOB_RECURSE intrinsic_python_SRCS
  RELATIVE "${intrinsic_sdk_SOURCE_DIR}"
  "${intrinsic_sdk_SOURCE_DIR}/**/*.py"
)
list(FILTER intrinsic_python_SRCS EXCLUDE REGEX "/\\.github/")
list(FILTER intrinsic_python_SRCS EXCLUDE REGEX "/examples/")
list(FILTER intrinsic_python_SRCS EXCLUDE REGEX "_test\\.py$")

# Overlay sdk Python files on top of generated protobuf Python files.
set(copied_python_sdk_files "")
foreach(sdk_python_file ${intrinsic_python_SRCS})
  # Ensure the target directory exists.
  set(python_sdk_destination_dir "${CMAKE_CURRENT_BINARY_DIR}/protos_gen_py")
  get_filename_component(sdk_python_file_dir "${sdk_python_file}" DIRECTORY)
  set(python_sdk_file_dest_dir "${python_sdk_destination_dir}/${sdk_python_file_dir}")
  file(MAKE_DIRECTORY "${python_sdk_file_dest_dir}")
  # Ensure every python package (directory in the output) has at least an empty __init__.py.
  set(python_init_py_file "${python_sdk_file_dest_dir}/__init__.py")
  if(NOT EXISTS "${python_init_py_file}")
    file(TOUCH "${python_init_py_file}")
  endif()
  # Copy the file over.
  set(python_sdk_file_destination "${python_sdk_destination_dir}/${sdk_python_file}")
  file(COPY_FILE "${intrinsic_sdk_SOURCE_DIR}/${sdk_python_file}" "${python_sdk_file_destination}" ONLY_IF_DIFFERENT)
  list(APPEND copied_python_sdk_files "${python_sdk_file_destination}")
endforeach()

# Check to see if there are any collisions between the protobuf generation and copied files.
foreach(copied_python_sdk_file ${copied_python_sdk_files})
  if("${copied_python_sdk_file}" IN_LIST sdk_protos_python_sources)
    message(FATAL_ERROR "Python sdk file '${copied_python_sdk_file}' would be overwritten by protoc.")
  endif()
endforeach()

# Install select (for now) python packages.
# Note(wjwwood): More work needs to be done to wrangle all of the generated protobuf code into
#   a single python project and/or distrbute them separately by namespace.
set(python_packages_to_install
  "googleapis"  # This one is a re-namespacing of the generated googleapis proto python packages
  "intrinsic"
  "protoc_gen_openapiv2"  # This one should be fixed to have a better python package name
  # "src"  # This one is disabled because it isn't being used atm and is a weird layout
  # "third_party"  # This one contains protobuf versions of ROS messages and is unused atm
)

macro(_get_python_install_dir)
  if(NOT DEFINED PYTHON_INSTALL_DIR)
    # avoid storing backslash in cached variable since CMake will interpret it as escape character
    # This auto detection code uses the same logic as get_python_install_path() in colcon-core
    set(_python_code
      "\
import os
import sysconfig
schemes = sysconfig.get_scheme_names()
kwargs = {'vars': {'base': '${CMAKE_INSTALL_PREFIX}'}}
if 'deb_system' in schemes or 'osx_framework_library' in schemes:
    kwargs['scheme'] = 'posix_prefix'
elif 'rpm_prefix' in schemes:
    kwargs['scheme'] = 'rpm_prefix'
print(os.path.relpath(sysconfig.get_path('purelib', **kwargs), start='${CMAKE_INSTALL_PREFIX}').replace(os.sep, '/'))"
    )
    get_executable_path(_python_interpreter Python3::Interpreter CONFIGURE)
    execute_process(
      COMMAND
      "${_python_interpreter}"
      "-c"
      "${_python_code}"
      OUTPUT_VARIABLE _output
      RESULT_VARIABLE _result
      OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if(NOT _result EQUAL 0)
      message(FATAL_ERROR
        "execute_process(${_python_interpreter} -c '${_python_code}') returned "
        "error code ${_result}")
    endif()

    set(PYTHON_INSTALL_DIR
      "${_output}"
      CACHE INTERNAL
      "The directory for Python library installation. This needs to be in PYTHONPATH when 'setup.py install' is called.")
  endif()
endmacro()

_get_python_install_dir()

function(_install_python_package package_name)
  cmake_parse_arguments(
    ARG "SKIP_COMPILE" "PACKAGE_DIR;DESTINATION" "DEPENDS" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "_install_python_package() called with unused "
      "arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()

  if(NOT ARG_PACKAGE_DIR)
    message(FATAL_ERROR "ARG PACKAGE_DIR required")
  endif()

  if(NOT ARG_DESTINATION)
    if(NOT PYTHON_INSTALL_DIR)
      message(FATAL_ERROR "_install_python_package() variable 'PYTHON_INSTALL_DIR' must not be empty")
    endif()
    set(ARG_DESTINATION ${PYTHON_INSTALL_DIR})
  endif()

  set(build_dir "${CMAKE_CURRENT_BINARY_DIR}/_install_python_package/${package_name}")

  string(CONFIGURE "\
from setuptools import find_packages
from setuptools import setup

setup(
    name='${package_name}',
    version='${sdk_version}',
    packages=find_packages(
        include=('${package_name}', '${package_name}.*')),
)
" setup_py_content)

  file(GENERATE
    OUTPUT "${build_dir}/setup.py"
    CONTENT "${setup_py_content}"
  )

  add_custom_target(
    _install_python_package_copy_${package_name}
    COMMAND ${CMAKE_COMMAND} -E copy_directory
      "${ARG_PACKAGE_DIR}" "${build_dir}/${package_name}"
    DEPENDS ${ARG_DEPENDS}
  )
  set(egg_dependencies _install_python_package_copy_${package_name})

  get_executable_path(python_interpreter Python3::Interpreter BUILD)

  add_custom_target(
    _install_python_package_build_${package_name}_egg ALL
    COMMAND ${python_interpreter} setup.py egg_info
    WORKING_DIRECTORY "${build_dir}"
    DEPENDS ${egg_dependencies}
  )

  set(python_version "py${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}")

  set(egg_name "${package_name}")
  set(egg_install_name "${egg_name}-${ARG_VERSION}")
  set(egg_install_name "${egg_install_name}-${python_version}")

  install(
    DIRECTORY "${build_dir}/${egg_name}.egg-info/"
    DESTINATION "${ARG_DESTINATION}/${egg_install_name}.egg-info"
  )

  install(
    DIRECTORY "${ARG_PACKAGE_DIR}/"
    DESTINATION "${ARG_DESTINATION}/${package_name}"
    PATTERN "*.pyc" EXCLUDE
    PATTERN "__pycache__" EXCLUDE
  )

  if(NOT ARG_SKIP_COMPILE)
    get_executable_path(python_interpreter_config Python3::Interpreter CONFIGURE)
    # compile Python files
    install(CODE
      "execute_process(
        COMMAND
        \"${python_interpreter_config}\" \"-m\" \"compileall\"
        \"${CMAKE_INSTALL_PREFIX}/${ARG_DESTINATION}/${package_name}\"
      )"
    )
  endif()
endfunction()

foreach(python_package_to_install ${python_packages_to_install})
  set(python_package_to_install_package_dir
    "${CMAKE_CURRENT_BINARY_DIR}/protos_gen_py/${python_package_to_install}")
  set(python_package_to_install_package_dir_fixed
    "${CMAKE_CURRENT_BINARY_DIR}/protos_gen_py/fixed_imports/${python_package_to_install}")

  # before installing the python files, rewrite them to use the namespaced googleapi package
  set(script "${CMAKE_CURRENT_SOURCE_DIR}/cmake/find_replace_fix_imports.cmake")
  add_custom_target(fix_imports_before_install_${python_package_to_install}
    DEPENDS
      ${copied_python_sdk_files}
      ${sdk_protos_python_sources}
      "${script}"
      intrinsic_sdk_protos_python
      # intrinsic_sdk_protos_python_grpc
      intrinsic_sdk_protos_python_rename_googleapis
    COMMAND
      ${CMAKE_COMMAND}
      -DPYTHON_PACKAGE_DIRECTORY="${python_package_to_install}"
      -P "${script}"
    COMMENT "Fixing python import statements for '${python_package_to_install}'..."
  )

  _install_python_package(
    ${python_package_to_install}
    PACKAGE_DIR "${python_package_to_install_package_dir_fixed}"
    DEPENDS fix_imports_before_install_${python_package_to_install}
  )
endforeach()
