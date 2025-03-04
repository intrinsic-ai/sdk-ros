# Copyright 2025 Intrinsic Innovation LLC
#
# You are hereby granted a non-exclusive, worldwide, royalty-free license to use,
# copy, modify, and distribute this Intrinsic SDK in source code or binary form for use
# in connection with the services and APIs provided by Intrinsic Innovation LLC (“Intrinsic”).
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
# FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
# COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
# IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#
# If you use this Intrinsic SDK with any Intrinsic services, your use is subject to the Intrinsic
# Platform Terms of Service [https://intrinsic.ai/platform-terms].  If you create works that call
# Intrinsic APIs, you must agree to the terms of service for those APIs separately. This license
# does not grant you any special rights to use the services.
#
# This copyright notice shall be included in all copies or substantial portions of the software.

include_guard(GLOBAL)

#
# Generate protobuf files into source code for cpp and into a descriptor file.
#
# :param NAME: the name of the skill
# :type NAME: string
# :param TARGET: the name of the target to use for generating the protos library
# :type TARGET: string
# :param IMPORT_DIRS: list of directories to include in the protoc invocation
# :type IMPORT_DIRS: list of strings
# :param SOURCES: the list of source files for the proto generation
# :type SOURCES: list of strings
#
# @public
#
function(intrinsic_sdk_protobuf_generate)
  set(options)
  set(one_value_args NAME TARGET)
  set(multi_value_args IMPORT_DIRS SOURCES)

  cmake_parse_arguments(
    GENERATE_ARGS
    "${options}"
    "${one_value_args}"
    "${multi_value_args}"
    ${ARGN}
  )

  set(OUT_DIR ${CMAKE_CURRENT_BINARY_DIR})

  if(GENERATE_ARGS_IMPORT_DIRS)
    set(IMPORT_DIRS ${GENERATE_ARGS_IMPORT_DIRS})
  else()
    set(IMPORT_DIRS ${CMAKE_CURRENT_SOURCE_DIR})
  endif()

  list(APPEND IMPORT_DIRS ${intrinsic_sdk_PROTO_DIR} ${googleapis_SOURCE_DIR})

  set(DESCRIPTOR_SET ${OUT_DIR}/${GENERATE_ARGS_NAME}_protos.desc)

  add_library(${GENERATE_ARGS_TARGET} STATIC ${GENERATE_ARGS_SOURCES})

  protobuf_generate(
    TARGET ${GENERATE_ARGS_TARGET}
    LANGUAGE cpp
    IMPORT_DIRS ${IMPORT_DIRS}
    PROTOC_OUT_DIR ${OUT_DIR}
  )

  target_include_directories(${GENERATE_ARGS_TARGET}
    PUBLIC
      ${OUT_DIR}
  )
  target_link_libraries(${GENERATE_ARGS_TARGET}
    PUBLIC
      absl::cordz_functions
      absl::log_internal_check_op
      protobuf::libprotobuf
      intrinsic_sdk_cmake::intrinsic_sdk_cmake
  )

  set(PROTOC_ARGS "")

  foreach(DIR ${IMPORT_DIRS})
    list(APPEND PROTOC_ARGS "-I${DIR}")
  endforeach()

  list(APPEND PROTOC_ARGS
    "--include_imports"
    "--include_source_info"
    "--descriptor_set_out=${DESCRIPTOR_SET}"
  )
  list(APPEND PROTOC_ARGS "${GENERATE_ARGS_SOURCES}")

  add_custom_command(
    OUTPUT ${DESCRIPTOR_SET}
    COMMAND protobuf::protoc
    ARGS ${PROTOC_ARGS}
    DEPENDS ${protobuf_PROTOC_EXE}
    COMMENT "Generating skill descriptor set for: ${GENERATE_ARGS_NAME}"
    VERBATIM
  )

  add_custom_target(${GENERATE_ARGS_TARGET}_desc DEPENDS ${DESCRIPTOR_SET})

  add_dependencies(${GENERATE_ARGS_TARGET} ${GENERATE_ARGS_TARGET}_desc)
endfunction()
