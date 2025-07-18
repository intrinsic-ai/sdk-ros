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

if(NOT DEFINED intrinsic_sdk_cmake_API_DIR)
  message(FATAL_ERROR "intrinsic_sdk_cmake_API_DIR not defined, include via cmake/api/all.cmake")
endif()
include("${intrinsic_sdk_cmake_API_DIR}/intrinsic_sdk_protobuf_generate.cmake")

#
# Generate the manifest file for an Intrinsic Service bundle.
#
# :param SERVICE_NAME: the name of the service
# :type SERVICE_NAME: string
# :param MANIFEST: the path to the manifest file
# :type MANIFEST: string
# :param PARAMETER_DESCRIPTOR: the path to the service parameter descritor file
# :type PARAMETER_DESCRIPTOR: string
# :param DEFAULT_CONFIGURATION: the path to the file containing the default configuration
# :type DEFAULT_CONFIGURATION: string
# :param PROTOS_TARGET: the cmake target for generating the services's proto files
# :type PROTOS_TARGET: cmake target
#
# @public
#
function(intrinsic_sdk_generate_service_manifest)
  set(options)
  set(multi_value_args)
  set(one_value_args
    SERVICE_NAME
    MANIFEST
    PARAMETER_DESCRIPTOR
    DEFAULT_CONFIGURATION
    PROTOS_TARGET
  )

  cmake_parse_arguments(
    arg
    "${options}"
    "${one_value_args}"
    "${multi_value_args}"
    ${ARGN}
  )

  set(OUT_DIR ${CMAKE_CURRENT_BINARY_DIR})

  if (NOT arg_DEFAULT_CONFIGURATION STREQUAL "")
    if (arg_PARAMETER_DESCRIPTOR STREQUAL "")
      message(ERROR "PARAMETER_DESCRIPTOR must be passed with DEFAULT_CONFIGURATION")
    endif()

    intrinsic_sdk_protobuf_generate(
      NAME ${arg_SERVICE_NAME}
      SOURCES ${arg_PARAMETER_DESCRIPTOR}
      TARGET ${arg_PROTOS_TARGET}
      DESCRIPTOR_SET_OUTPUT "${OUT_DIR}/${arg_SERVICE_NAME}_protos.desc")

    add_custom_command(
      OUTPUT ${OUT_DIR}/default_config.binarypb
      COMMAND protobuf::protoc
        --encode=google.protobuf.Any
        --descriptor_set_in="${OUT_DIR}/${arg_SERVICE_NAME}_protos.desc:${intrinsic_sdk_cmake_DESCRIPTOR_SET_FILE}"
        < ${CMAKE_CURRENT_SOURCE_DIR}/${arg_DEFAULT_CONFIGURATION}
        > ${OUT_DIR}/default_config.binarypb
      DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/${arg_DEFAULT_CONFIGURATION}
        ${OUT_DIR}/${arg_SERVICE_NAME}_protos.desc
      COMMENT "Generating default config for ${arg_SERVICE_NAME}"
    )

    add_custom_target(
      ${arg_SERVICE_NAME}_default_config DEPENDS
        ${OUT_DIR}/default_config.binarypb
    )

  endif()

  add_custom_command(
    OUTPUT ${OUT_DIR}/service_manifest.binarypb
    COMMAND protobuf::protoc
      --encode=intrinsic_proto.services.ServiceManifest
      --descriptor_set_in=${intrinsic_sdk_cmake_DESCRIPTOR_SET_FILE}
      < ${CMAKE_CURRENT_SOURCE_DIR}/${arg_MANIFEST}
      > ${OUT_DIR}/service_manifest.binarypb
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${arg_MANIFEST} ${intrinsic_sdk_cmake_DESCRIPTOR_SET_FILE}
    COMMENT "Generating service manifest for ${arg_SERVICE_NAME}"
  )

  add_custom_target(
    ${arg_SERVICE_NAME}_manifest DEPENDS
      ${OUT_DIR}/service_manifest.binarypb
  )
endfunction()
