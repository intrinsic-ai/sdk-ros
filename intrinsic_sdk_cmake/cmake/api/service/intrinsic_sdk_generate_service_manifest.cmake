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

include("${intrinsic_sdk_cmake_DIR}/cmake/api/intrinsic_sdk_protobuf_generate.cmake")

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
    GENERATE_ARGS
    "${options}"
    "${one_value_args}"
    "${multi_value_args}"
    ${ARGN}
  )

  set(OUT_DIR ${CMAKE_CURRENT_BINARY_DIR})

  if (NOT GENERATE_ARGS_DEFAULT_CONFIGURATION STREQUAL "")
    if (GENERATE_ARGS_PARAMETER_DESCRIPTOR STREQUAL "")
      message(ERROR "PARAMETER_DESCRIPTOR must be passed with DEFAULT_CONFIGURATION")
    endif()

    intrinsic_sdk_protobuf_generate(
      NAME ${GENERATE_ARGS_SERVICE_NAME}
      SOURCES ${GENERATE_ARGS_PARAMETER_DESCRIPTOR}
      TARGET ${GENERATE_ARGS_PROTOS_TARGET})

    add_custom_command(
      OUTPUT ${OUT_DIR}/default_config.binarypb
      COMMAND ${intrinsic_sdk_DIR}/../../../lib/intrinsic_sdk/textproto_to_binproto.py
      ARGS
        --descriptor_database
          ${intrinsic_sdk_DESCRIPTOR_DATABASE}
          ${OUT_DIR}/${GENERATE_ARGS_SERVICE_NAME}_protos.desc
        --message_type=google.protobuf.Any
        --textproto_in=${CMAKE_CURRENT_SOURCE_DIR}/${GENERATE_ARGS_DEFAULT_CONFIGURATION}
        --binproto_out=${OUT_DIR}/default_config.binarypb
      DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/${GENERATE_ARGS_DEFAULT_CONFIGURATION}
        ${OUT_DIR}/${GENERATE_ARGS_SERVICE_NAME}_protos.desc
      COMMENT "Generating default config for ${GENERATE_ARGS_SERVICE_NAME}"
    )

    add_custom_target(
      ${GENERATE_ARGS_SERVICE_NAME}_default_config DEPENDS
        ${OUT_DIR}/default_config.binarypb
    )

  endif()

  add_custom_command(
    OUTPUT ${OUT_DIR}/service_manifest.binarypb
    COMMAND ${intrinsic_sdk_DIR}/../../../lib/intrinsic_sdk/textproto_to_binproto.py
    ARGS
      --descriptor_database ${intrinsic_sdk_DESCRIPTOR_DATABASE}
      --message_type=intrinsic_proto.services.ServiceManifest
      --textproto_in=${CMAKE_CURRENT_SOURCE_DIR}/${GENERATE_ARGS_MANIFEST}
      --binproto_out=${OUT_DIR}/service_manifest.binarypb
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${GENERATE_ARGS_MANIFEST}
    COMMENT "Generating service manifest for ${GENERATE_ARGS_SERVICE_NAME}"
  )

  add_custom_target(
    ${GENERATE_ARGS_SERVICE_NAME}_manifest DEPENDS
      ${OUT_DIR}/service_manifest.binarypb
  )
endfunction()
